/**************************************************************************/
/* test_body_frame.cpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_BodyFrame: where a body's prime meridian is, and the rotation built on it
// (docs/slop/solar_system_overhaul.md §9, §14).
//
// A body-fixed frame is easy to get plausibly wrong: pick the pole correctly, add the IAU
// angle W to the wrong reference direction, and every check that only inspects the pole or
// only round-trips through your own conversion still passes — while the body's whole
// surface sits at the wrong longitude. That is exactly the shape of the bug this file
// exists to keep out, so the decisive test here is external to the conversion: the Moon is
// tidally locked, so an observer standing at selenographic (0, 0) must see the Earth
// within the libration cone of their zenith. No amount of self-consistency can fake that.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/body_orientation.hpp>
#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/ephemeris.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/scene_frame.hpp>
#include <SushiEngine/astro/topocentric.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    /** The observer's zenith, expressed in the body-fixed frame the axes define. */
    Vector3 up_in_body_frame(const Render::Environment& environment)
    {
        const Vector3* axes = environment.planet_body_axes;
        const Vector3 up{0.0, 1.0, 0.0};
        return Vector3{dot(up, axes[0]), dot(up, axes[1]), dot(up, axes[2])};
    }

    /** Fills a sky for an observer standing at a place on a body. */
    Render::Environment sky_at(int body, double latitude, double longitude, double julian_date)
    {
        Render::Environment environment;
        environment.observer.observer_body = body;
        environment.observer.latitude_radians = latitude;
        environment.observer.longitude_radians = longitude;
        environment.observer.julian_date = julian_date;
        fill_environment_sky(environment, WorldVector3{0.0, 0.0, 0.0});
        return environment;
    }

    constexpr double DEGREE = 0.017453292519943295;
} // namespace

TEST(Unit_BodyFrame, EarthsPrimeMeridianIsSiderealTimeAndNotW)
{
    // Earth's body-equatorial frame is the J2000 equatorial one, so its prime meridian's
    // angle in it is Greenwich sidereal time by definition. The IAU W for Earth is a third
    // of a degree away from that, which is 35 km at the equator — small enough to have
    // gone unnoticed while nothing but the sky spoke this frame, and large enough to
    // matter the moment real geography does.
    for (double day : {0.0, 1.0, 1234.5, 9000.25})
    {
        const double julian_date = J2000_JULIAN_DATE + day;
        EXPECT_DOUBLE_EQ(prime_meridian_angle(BodyId::Earth, julian_date),
                         local_mean_sidereal_time(julian_date, 0.0));
    }

    const double gap = std::fabs(prime_meridian_angle(BodyId::Earth, J2000_JULIAN_DATE) -
                                 body_rotation_angle(BodyId::Earth, J2000_JULIAN_DATE));
    EXPECT_GT(gap, 0.3 * DEGREE);
}

TEST(Unit_BodyFrame, TheHomeSkyDoesNotMove)
{
    // The Earth path did change shape -- the meridian is wrapped before the longitude is
    // added rather than after -- so "unchanged" is a claim that has to be measured. Same
    // angle modulo a turn, but not the same double, and the sine of the two differs in the
    // last bits. The bound here is a thousand times looser than what it actually costs
    // (6.3e-16 rad, four nanometres at Earth's radius) and still tight enough that any
    // real change to the home sky trips it.
    const Vector3 star = normalize(Vector3{0.37, -0.51, 0.77});
    double worst = 0.0;
    for (int index = 0; index < 400; ++index)
    {
        const double julian_date = J2000_JULIAN_DATE + index * 37.0 + 0.31;
        const double longitude = (index % 37 - 18) * 10.0 * DEGREE;
        const double latitude = (index % 19 - 9) * 9.0 * DEGREE;

        const LocalSkyBasis before =
            local_sky_basis(local_mean_sidereal_time(julian_date, longitude), latitude);
        const LocalSkyBasis after = local_sky_basis(
            prime_meridian_angle(BodyId::Earth, julian_date) + longitude, latitude);
        worst = std::fmax(worst, length(to_local(before, star) - to_local(after, star)));
    }
    EXPECT_LT(worst, 1e-12) << "the home sky moved by " << worst << " radians";
}

TEST(Unit_BodyFrame, WIsNotAnAngleInThisEnginesBodyEquatorialFrame)
{
    // The whole reason prime_meridian_angle exists. W is measured from the node of the
    // body equator on the *J2000 equator*; ecliptic_to_body_equatorial puts +X on the node
    // with the *ecliptic*. The offset between them is a fixed property of the body's pole,
    // and it is not small.
    struct Expectation
    {
        BodyId body;
        double degrees;
    };
    const Expectation expected[] = {{BodyId::Moon, 52.6763},
                                    {BodyId::Mars, -40.8586},
                                    {BodyId::Venus, -117.6487}};

    for (const Expectation& item : expected)
    {
        const double julian_date = J2000_JULIAN_DATE + 400.0;
        double offset = prime_meridian_angle(item.body, julian_date) -
                        body_rotation_angle(item.body, julian_date);
        // Both are wrapped, so the difference can be out by a whole turn.
        while (offset > 3.141592653589793)
            offset -= 6.283185307179586;
        while (offset < -3.141592653589793)
            offset += 6.283185307179586;
        EXPECT_NEAR(offset, item.degrees * DEGREE, 1e-3 * DEGREE);
    }
}

TEST(Unit_BodyFrame, TidalLockPutsEarthOverTheMoonsNearSide)
{
    // The check that no self-consistent-but-wrong frame can pass. The Moon keeps one face
    // to the Earth, so from selenographic (0, 0) the Earth stands within about ten degrees
    // of the zenith — the libration cone, 7.9 degrees in longitude and 6.7 in latitude.
    // Under the pre-fix convention this landed at 29 to 40 degrees of elevation instead:
    // a plausible-looking sky over a Moon rotated by fifty degrees.
    for (double day : {0.0, 7.0, 14.0, 21.0, 28.0, 200.0, 4000.0})
    {
        const double julian_date = J2000_JULIAN_DATE + day;
        const Vector3 earth_from_moon = moon_geocentric_ecliptic_au(julian_date) * -1.0;

        const SceneFrame frame = scene_frame_for(julian_date, 0.0, 0.0, BodyId::Moon);
        const Vector3 local = normalize(frame.direction_to_scene(earth_from_moon));

        // The scene's +Y is the observer's geodetic up.
        EXPECT_GT(local.y, std::cos(11.0 * DEGREE))
            << "Earth " << (std::asin(local.y) / DEGREE) << " degrees up on day " << day;
    }
}

TEST(Unit_BodyFrame, AnObserversZenithReadsBackTheirOwnCoordinates)
{
    // The scene is anchored at the observer's surface point, so rotating the scene's up
    // back through planet_body_axes has to return exactly the latitude and longitude the
    // observer was placed at. This is what ties the terrain — which reads elevations
    // indexed by those coordinates — to the ground the ephemeris built.
    struct Place
    {
        int body;
        double latitude_degrees;
        double longitude_degrees;
        double day;
    };
    const Place places[] = {{4, 0.0, 0.0, 0.0},         // Moon, the origin of selenography
                            {4, 26.0, 18.0, 0.0},       // Mare Serenitatis
                            {4, 26.0, 18.0, 90.0},      // and again, a quarter year on
                            {4, -80.0, -170.0, 33.0},   // deep south, west longitude
                            {5, -14.0, 175.0, 33.0},    // Mars
                            {3, 41.0, 29.0, 9000.0}};   // Earth

    for (const Place& place : places)
    {
        const Render::Environment environment =
            sky_at(place.body, place.latitude_degrees * DEGREE,
                   place.longitude_degrees * DEGREE, J2000_JULIAN_DATE + place.day);
        const Vector3 up = up_in_body_frame(environment);

        EXPECT_NEAR(std::asin(up.z / length(up)) / DEGREE, place.latitude_degrees, 1e-9);
        double longitude = std::atan2(up.y, up.x) / DEGREE;
        if (longitude - place.longitude_degrees > 180.0)
            longitude -= 360.0;
        if (place.longitude_degrees - longitude > 180.0)
            longitude += 360.0;
        EXPECT_NEAR(longitude, place.longitude_degrees, 1e-9);
    }
}

TEST(Unit_BodyFrame, TheAxesAreARotationAndTheirThirdColumnIsThePole)
{
    const Render::Environment environment =
        sky_at(4, 12.0 * DEGREE, -47.0 * DEGREE, J2000_JULIAN_DATE + 611.0);
    const Vector3* axes = environment.planet_body_axes;

    for (int axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(length(axes[axis]), 1.0, 1e-12);
    EXPECT_NEAR(dot(axes[0], axes[1]), 0.0, 1e-12);
    EXPECT_NEAR(dot(axes[0], axes[2]), 0.0, 1e-12);
    EXPECT_NEAR(dot(axes[1], axes[2]), 0.0, 1e-12);
    // Right-handed, not merely orthogonal: a reflection here mirrors the whole planet.
    EXPECT_NEAR(dot(cross(axes[0], axes[1]), axes[2]), 1.0, 1e-12);

    // Derived independently of the fill, from the pole the sky itself uses.
    const Vector3 pole = normalize(to_local(
        scene_frame_for(J2000_JULIAN_DATE + 611.0, 12.0 * DEGREE, -47.0 * DEGREE,
                        BodyId::Moon)
            .basis,
        equatorial_to_body_equatorial(BodyId::Moon, body_north_pole_equatorial(BodyId::Moon))));
    EXPECT_NEAR(length(axes[2] - pole), 0.0, 1e-12);
    EXPECT_NEAR(length(axes[2] - environment.planet_pole), 0.0, 1e-15);
}

TEST(Unit_BodyFrame, TheDominantBodyIsNamedAndDeepSpaceNamesNothing)
{
    // A consumer holding per-body data — a baked terrain pack above all — cannot pick the
    // right one from a description of the body; it needs the body.
    const Render::Environment moon = sky_at(4, 0.0, 0.0, J2000_JULIAN_DATE);
    EXPECT_EQ(moon.planet_body, 4);
    EXPECT_TRUE(moon.planet_surface_visible);

    // Far above the Moon's handoff altitude: no near-field body at all.
    Render::Environment space;
    space.observer.observer_body = 4;
    space.observer.julian_date = J2000_JULIAN_DATE;
    const double far_away =
        surface_preset(BodyId::Moon).semi_major_metres * (SURFACE_HANDOFF_ALTITUDE_RADII + 40.0);
    fill_environment_sky(space, WorldVector3{0.0, far_away, 0.0});
    EXPECT_EQ(space.planet_body, -1);
    EXPECT_FALSE(space.planet_surface_visible);
}

TEST(Unit_BodyFrame, TheSceneFrameAgreesWithTheAxesItWasBuiltFrom)
{
    // The scene-frame bijection and the rendered axes are two derivations of one rotation,
    // used by two different layers — the simulation's entity anchoring and the renderer's
    // terrain. They have to be the same rotation, or a building placed at a coordinate
    // stands somewhere other than the ground baked for it.
    const double julian_date = J2000_JULIAN_DATE + 777.5;
    const double latitude = -33.0 * DEGREE;
    const double longitude = 151.0 * DEGREE;

    const Render::Environment environment = sky_at(4, latitude, longitude, julian_date);
    const SceneFrame frame = scene_frame_for(julian_date, latitude, longitude, BodyId::Moon);
    const double spin = prime_meridian_angle(BodyId::Moon, julian_date);

    // A body-fixed direction, carried to the scene the simulation's way.
    const Vector3 body_fixed = normalize(Vector3{0.31, -0.62, 0.72});
    const double cos_spin = std::cos(spin);
    const double sin_spin = std::sin(spin);
    const Vector3 equatorial{body_fixed.x * cos_spin - body_fixed.y * sin_spin,
                             body_fixed.x * sin_spin + body_fixed.y * cos_spin, body_fixed.z};
    const Vector3 simulation =
        frame.direction_to_scene(body_equatorial_to_ecliptic(BodyId::Moon, equatorial));

    // And the renderer's way: the axes are the columns of the same rotation.
    const Vector3* axes = environment.planet_body_axes;
    const Vector3 renderer{
        axes[0].x * body_fixed.x + axes[1].x * body_fixed.y + axes[2].x * body_fixed.z,
        axes[0].y * body_fixed.x + axes[1].y * body_fixed.y + axes[2].y * body_fixed.z,
        axes[0].z * body_fixed.x + axes[1].z * body_fixed.y + axes[2].z * body_fixed.z};

    EXPECT_NEAR(length(simulation - renderer), 0.0, 1e-12);
}
