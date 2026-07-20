/**************************************************************************/
/* test_ephemeris.cpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Integration_Ephemeris: the assembler that turns a date and an observer into a full sky
// Environment. It must place a bounded set of unit-direction bodies, exclude the body the
// observer stands on (it is the analytic ground), track the Sun with the directional
// light, fill the star field, hand off to the near-field surface regime near a planet and
// switch it off in deep space, and classify each body's LOD and surface style correctly.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/ephemeris.hpp>
#include <SushiEngine/render/environment.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

TEST(Integration_Ephemeris, LodLadderIsMonotonicInAngularSize)
{
    using Astro::lod_for_angular_radius;
    // Bigger apparent size never demotes the regime.
    EXPECT_EQ(lod_for_angular_radius(0.3), Render::BodyLod::Surface);
    EXPECT_EQ(lod_for_angular_radius(0.05), Render::BodyLod::Mesh);
    EXPECT_EQ(lod_for_angular_radius(0.005), Render::BodyLod::Impostor);
    EXPECT_EQ(lod_for_angular_radius(1e-4), Render::BodyLod::Disk);
    EXPECT_EQ(lod_for_angular_radius(1e-7), Render::BodyLod::Point);

    Render::BodyLod previous = lod_for_angular_radius(1e-8);
    for (double a = 1e-7; a < 1.0; a *= 1.7)
    {
        const Render::BodyLod lod = lod_for_angular_radius(a);
        EXPECT_GE(static_cast<int>(lod), static_cast<int>(previous));
        previous = lod;
    }
}

TEST(Integration_Ephemeris, SurfaceStyleClassification)
{
    using Astro::surface_style_for;
    EXPECT_EQ(surface_style_for(Astro::BodyId::Sun), Render::SurfaceStyle::Star);
    EXPECT_EQ(surface_style_for(Astro::BodyId::Earth), Render::SurfaceStyle::EarthLike);
    EXPECT_EQ(surface_style_for(Astro::BodyId::Jupiter), Render::SurfaceStyle::Banded);
    EXPECT_EQ(surface_style_for(Astro::BodyId::Mars), Render::SurfaceStyle::Rocky);
}

TEST(Integration_Ephemeris, FillsABoundedSkyOfUnitDirectionBodies)
{
    Render::Environment environment; // defaults: J2000, Earth observer, astronomical sun
    Astro::fill_environment_sky(environment);

    EXPECT_GT(environment.body_count, 0);
    EXPECT_LE(environment.body_count, Render::MAX_CELESTIAL_BODIES);

    bool saw_star = false;
    for (int i = 0; i < environment.body_count; ++i)
    {
        const Render::CelestialBody& body = environment.bodies[i];
        EXPECT_TRUE(Harness::is_unit(body.direction, 1e-6)) << "body " << i;
        EXPECT_TRUE(Harness::is_unit(body.sun_direction, 1e-6)) << "body " << i;
        EXPECT_GE(body.angular_radius, 0.0f);
        // The observer stands on Earth (body index 3): it is the ground, never a sky body.
        EXPECT_NE(body.body_id, 3u) << "the observer's own body must not appear in the sky";
        saw_star = saw_star || body.is_star != 0;
    }
    EXPECT_TRUE(saw_star) << "the Sun should be placed in the sky";
}

TEST(Integration_Ephemeris, TracksTheSunWithTheDirectionalLightAndFillsTheStarField)
{
    Render::Environment environment;
    Astro::fill_environment_sky(environment);

    // astronomical_sun is on by default: the directional light points at the ephemeris Sun.
    EXPECT_TRUE(Harness::is_unit(environment.sun.direction, 1e-6));

    EXPECT_GT(environment.sky_star_count, 0);
    EXPECT_LE(environment.sky_star_count, Render::MAX_SKY_STARS);
    for (int i = 0; i < environment.sky_star_count; ++i)
        EXPECT_TRUE(Harness::is_unit(environment.sky_stars[i].direction, 1e-6)) << "star " << i;
}

TEST(Integration_Ephemeris, HandsOffToTheGroundNearEarthAndOffInDeepSpace)
{
    // Camera at the scene origin (the observer's surface point): Earth is the dominant
    // body and drives the analytic ground/atmosphere.
    Render::Environment surface;
    Astro::fill_environment_sky(surface, WorldVector3{0.0, 0.0, 0.0});
    EXPECT_EQ(surface.dominant_body_id, 3); // Earth
    EXPECT_TRUE(surface.planet_surface_visible);

    // Camera pulled far out of the solar system: no body within the hand-off range, so the
    // analytic ground and atmosphere switch off.
    Render::Environment space;
    Astro::fill_environment_sky(space, WorldVector3{0.0, 1.0e13, 0.0});
    EXPECT_EQ(space.dominant_body_id, -1);
    EXPECT_FALSE(space.planet_surface_visible);
    EXPECT_FALSE(space.atmosphere.enabled);
}

TEST(Integration_Ephemeris, IsDeterministicForAFixedObserver)
{
    Render::Environment a;
    Render::Environment b;
    Astro::fill_environment_sky(a);
    Astro::fill_environment_sky(b);

    ASSERT_EQ(a.body_count, b.body_count);
    for (int i = 0; i < a.body_count; ++i)
    {
        EXPECT_EQ(a.bodies[i].body_id, b.bodies[i].body_id);
        EXPECT_EQ(a.bodies[i].direction.x, b.bodies[i].direction.x);
        EXPECT_EQ(a.bodies[i].direction.y, b.bodies[i].direction.y);
        EXPECT_EQ(a.bodies[i].direction.z, b.bodies[i].direction.z);
    }
}
