/**************************************************************************/
/* ephemeris.hpp                                                          */
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

#pragma once

/**
 * @file ephemeris.hpp
 * @brief The assembler that turns a date and an observer into a sky full of bodies.
 *
 * This is where the solar-system model meets the renderer's `Environment`. Given the
 * observer's epoch and geodetic position it:
 *   * builds the local East-Up-South basis in the equatorial frame from sidereal time
 *     and latitude (the topocentric transform);
 *   * places every planet, the Moon, and the Sun as a local-frame direction, angular
 *     size, phase-lighting direction, colour, and true metric distance;
 *   * points the scene's directional light at the astronomical Sun so the ground,
 *     atmosphere, and the visible Sun disk are all lit consistently;
 *   * rotates the bright-star catalogue into the same local frame.
 *
 * The engine owns no device code here — this is plain double-precision C++ that fills a
 * trivially-copyable struct crossing the render seam once per frame.
 */

#include <cmath>

#include <SushiEngine/astro/body_orientation.hpp>
#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>
#include <SushiEngine/astro/star_catalog.hpp>
#include <SushiEngine/astro/topocentric.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/environment.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief Assigns a body's LOD regime from how large it appears.
         *
         * The ladder the camera climbs approaching a body: a huge apparent size means the
         * body fills the view (surface/mesh regimes), a small one is a shaded disk, an
         * imperceptible one is a point. Thresholds are apparent angular radius in radians.
         *
         * @param angular_radius_radians Apparent angular radius of the body.
         * @return The representation regime to draw it in.
         */
        inline Render::BodyLod lod_for_angular_radius(double angular_radius_radians) noexcept
        {
            if (angular_radius_radians > 0.15)
                return Render::BodyLod::Surface;
            if (angular_radius_radians > 0.02)
                return Render::BodyLod::Mesh;
            if (angular_radius_radians > 0.0005)
                return Render::BodyLod::Impostor;
            if (angular_radius_radians > 2.0e-6)
                return Render::BodyLod::Disk;
            return Render::BodyLod::Point;
        }

        /**
         * @brief The procedural surface pattern the shader paints a body with.
         * @param body Which body to classify.
         * @return Star for the Sun, Banded for the four giants, EarthLike for Earth,
         *         Rocky for everything else.
         */
        inline Render::SurfaceStyle surface_style_for(BodyId body) noexcept
        {
            switch (body)
            {
                case BodyId::Sun:
                    return Render::SurfaceStyle::Star;
                case BodyId::Earth:
                    return Render::SurfaceStyle::EarthLike;
                case BodyId::Jupiter:
                case BodyId::Saturn:
                case BodyId::Uranus:
                case BodyId::Neptune:
                    return Render::SurfaceStyle::Banded;
                default:
                    return Render::SurfaceStyle::Rocky;
            }
        }

        /**
         * @brief Fraction of one disk hidden behind an overlapping second disk.
         *
         * The circle-circle lens area over the first disk's area, treating both bodies as
         * flat circles on the sky (exact for the small angles solar-system disks subtend).
         * The host mirror of `disk_overlap_fraction` in `sky.frag`, so the CPU-side eclipse
         * scalar and the shader's disk drawing agree by construction.
         *
         * @param covered_radius  Angular radius of the disk being hidden, radians.
         * @param occluder_radius Angular radius of the disk in front, radians.
         * @param separation      Angle between the two disk centres, radians.
         * @return Hidden fraction of the covered disk, [0, 1].
         */
        inline double disk_overlap_fraction(double covered_radius, double occluder_radius,
                                            double separation) noexcept
        {
            if (covered_radius <= 0.0)
                return 0.0;
            if (separation >= covered_radius + occluder_radius)
                return 0.0;
            if (separation <= std::fabs(covered_radius - occluder_radius))
            {
                const double smaller = std::fmin(covered_radius, occluder_radius);
                return std::fmin(1.0, (smaller * smaller) / (covered_radius * covered_radius));
            }
            const double a2 = covered_radius * covered_radius;
            const double b2 = occluder_radius * occluder_radius;
            const double d2 = separation * separation;
            const double alpha = std::acos(std::fmin(
                1.0, std::fmax(-1.0, (d2 + a2 - b2) / (2.0 * separation * covered_radius))));
            const double beta = std::acos(std::fmin(
                1.0, std::fmax(-1.0, (d2 + b2 - a2) / (2.0 * separation * occluder_radius))));
            const double lens = a2 * (alpha - 0.5 * std::sin(2.0 * alpha)) +
                                b2 * (beta - 0.5 * std::sin(2.0 * beta));
            return std::fmin(1.0, lens / (3.14159265358979323846 * a2));
        }

        /**
         * @brief Disk-integrated brightness of a sunlit sphere, relative to full phase.
         *
         * The factor that turns a body's geometric albedo — defined at full phase — into
         * what it actually reflects at the phase angle it is seen under, so the same
         * derivation covers a full moon and a thin crescent.
         *
         * A smooth sphere follows the Lambert law, but a regolith one does not: shadow
         * hiding between grains makes it surge near opposition, so the full Moon is about
         * eleven times a quarter Moon rather than the two Lambert predicts. That surge is
         * the whole character of moonlit night, so rocky bodies use Allen's empirical
         * lunar magnitude fit instead, faded back to the Lambert limit past the ~150° its
         * calibration covers.
         *
         * @param phase_angle_radians Sun-body-observer angle; 0 is full, pi is new.
         * @param style               Surface the body reflects with; only @c Rocky surges.
         * @return Brightness relative to full phase, [0, 1].
         */
        inline double phase_brightness(double phase_angle_radians,
                                       Render::SurfaceStyle style) noexcept
        {
            constexpr double PI = 3.14159265358979323846;
            const double alpha = std::fmin(PI, std::fmax(0.0, phase_angle_radians));
            const double lambert = (std::sin(alpha) + (PI - alpha) * std::cos(alpha)) / PI;
            if (style != Render::SurfaceStyle::Rocky)
                return lambert;

            const double degrees = alpha * (180.0 / PI);
            const double magnitudes =
                0.026 * degrees + 4.0e-9 * degrees * degrees * degrees * degrees;
            const double regolith = std::pow(10.0, -0.4 * magnitudes);
            const double beyond_fit =
                std::fmin(1.0, std::fmax(0.0, (degrees - 140.0) / 40.0));
            return regolith * (1.0 - beyond_fit) + lambert * beyond_fit;
        }

        /**
         * @brief Camera altitude (in the body's own radii) past which a body's analytic
         *        ground and atmosphere hand off to drawing it as a sky body.
         *
         * Below it the nearest landable body is the near-field regime — its ellipsoid,
         * atmosphere shell, and clouds — and it is deliberately absent from the body list
         * (the observer is at it). Above it those layers switch off and the body joins
         * the list as a phase-lit sphere — one continuous scene, no mode.
         */
        constexpr double SURFACE_HANDOFF_ALTITUDE_RADII = 10.0;

        /**
         * @brief Fills an environment's sky from its observer settings and the camera.
         *
         * One continuous regime from the ground to the edge of the solar system. Every
         * body is placed at its true position in the scene's local ENU frame (origin at
         * the observer's surface point, metres, Earth's centre straight down), then made
         * camera-relative in double precision — so flying toward the Moon or Mars grows
         * it on screen naturally, with no mode switch. The landable body whose surface
         * the camera is nearest (within @ref SURFACE_HANDOFF_ALTITUDE_RADII of its own
         * radii) becomes the analytic ground/atmosphere, configured from its
         * @ref SurfacePreset; every other body is a phase-lit sky body. When
         * @c observer.astronomical_sun is set the directional light tracks the
         * camera-relative Sun.
         *
         * @param environment            The environment to populate; its observer is the input.
         * @param camera_position_metres Camera position in the scene's local frame, metres.
         */
        inline void fill_environment_sky(Render::Environment& environment,
                                         const WorldVector3& camera_position_metres =
                                             WorldVector3{}) noexcept
        {
            const Render::SkyObserver& observer = environment.observer;
            const double julian_date = observer.julian_date;
            const BodyId observer_body = static_cast<BodyId>(observer.observer_body);

            // The observer's meridian angle, in its own body's equatorial frame. Earth
            // keeps sidereal time exactly (the home sky is unchanged); every other body
            // uses its own spin (@ref body_rotation_angle), so its day turns its own sky.
            const double meridian =
                observer_body == BodyId::Earth
                    ? local_mean_sidereal_time(julian_date, observer.longitude_radians)
                    : body_rotation_angle(observer_body, julian_date) +
                          observer.longitude_radians;
            const LocalSkyBasis basis = local_sky_basis(meridian, observer.latitude_radians);

            const Vector3 observer_helio =
                planet_heliocentric_au(observer_body, julian_date);

            const Vector3 camera{camera_position_metres.x, camera_position_metres.y,
                                 camera_position_metres.z};

            // The scene is anchored so the observer's geodetic surface point on its own
            // body is the origin, with +Y along the geodetic normal: the centre sits at
            // minus the observer's position on that body's ellipsoid (prime-vertical
            // radius N), so the true, pole-oriented ellipsoid passes through the ground
            // grid. Fully body-parametric — the same construction on Earth, Mars, or the
            // Moon, driven by @c observer_body rather than a hard-coded Earth.
            const SurfacePreset observer_surface = surface_preset(observer_body);
            const double body_a = observer_surface.semi_major_metres;
            const double body_f = observer_surface.inverse_flattening > 0.0
                                      ? 1.0 / observer_surface.inverse_flattening
                                      : 0.0;
            const double body_e2 = body_f * (2.0 - body_f);
            const double sin_lat = std::sin(observer.latitude_radians);
            const double cos_lat = std::cos(observer.latitude_radians);
            const double prime_vertical =
                body_a / std::sqrt(1.0 - body_e2 * sin_lat * sin_lat);
            const Vector3 observer_equatorial{prime_vertical * cos_lat * std::cos(meridian),
                                              prime_vertical * cos_lat * std::sin(meridian),
                                              prime_vertical * (1.0 - body_e2) * sin_lat};
            const Vector3 observer_center = to_local(basis, observer_equatorial) * -1.0;

            // First pass: every body's absolute position in the scene frame — the
            // geocentric direction rotated into the local basis, scaled by the true
            // distance, anchored at Earth's centre.
            Vector3 world_position[BODY_COUNT];
            Vector3 helio_ecliptic[BODY_COUNT];
            bool valid[BODY_COUNT];
            for (int index = 0; index < BODY_COUNT; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                if (body == BodyId::Sun)
                    helio_ecliptic[index] = Vector3{0.0, 0.0, 0.0};
                else if (body == BodyId::Moon)
                    helio_ecliptic[index] =
                        planet_heliocentric_au(BodyId::Earth, julian_date) +
                        moon_geocentric_ecliptic_au(julian_date);
                else
                    helio_ecliptic[index] = planet_heliocentric_au(body, julian_date);

                // The body the observer stands on is the ground at the scene origin;
                // every other body is placed by its position relative to the observer's
                // body, rotated into the observer body's own equatorial frame — the same
                // maths whichever body that is.
                if (body == observer_body)
                {
                    world_position[index] = observer_center;
                    valid[index] = true;
                    continue;
                }
                const Vector3 observer_centric = helio_ecliptic[index] - observer_helio;
                const double observer_centric_au = length(observer_centric);
                valid[index] = observer_centric_au > 0.0;
                if (!valid[index])
                    continue;
                const Vector3 direction = normalize(to_local(
                    basis, ecliptic_to_body_equatorial(observer_body, observer_centric)));
                world_position[index] =
                    observer_center +
                    direction * (observer_centric_au * METRES_PER_ASTRONOMICAL_UNIT);
            }

            // The heliocentric frame, expressed in the scene frame. The Sun's centre is
            // where the interplanetary grid is pinned, and the ecliptic pole and the
            // vernal equinox give that grid its plane and its rotation — so the planets
            // ride on the grid rather than cutting through it at an arbitrary angle.
            environment.sun_center_metres =
                WorldVector3{world_position[static_cast<int>(BodyId::Sun)].x,
                             world_position[static_cast<int>(BodyId::Sun)].y,
                             world_position[static_cast<int>(BodyId::Sun)].z};
            environment.ecliptic_normal = normalize(to_local(
                basis, ecliptic_to_body_equatorial(observer_body, Vector3{0.0, 0.0, 1.0})));
            environment.ecliptic_reference = normalize(to_local(
                basis, ecliptic_to_body_equatorial(observer_body, Vector3{1.0, 0.0, 0.0})));

            // Dominant body: the landable body whose surface the camera is closest to,
            // if within the hand-off range. It becomes the analytic ground; everything
            // else stays a sky body.
            int dominant = -1;
            double dominant_altitude = 0.0;
            for (int index = 0; index < BODY_COUNT; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                if (!valid[index] || !surface_preset(body).landable)
                    continue;
                const double body_altitude = length(world_position[index] - camera) -
                                             body_properties(body).mean_radius_metres;
                if (dominant < 0 || body_altitude < dominant_altitude)
                {
                    dominant = index;
                    dominant_altitude = body_altitude;
                }
            }
            if (dominant >= 0 &&
                dominant_altitude >=
                    SURFACE_HANDOFF_ALTITUDE_RADII *
                        body_properties(static_cast<BodyId>(dominant)).mean_radius_metres)
                dominant = -1;

            environment.dominant_body_id = dominant;
            if (dominant >= 0)
                environment.dominant_center_metres =
                    WorldVector3{world_position[dominant].x, world_position[dominant].y,
                                 world_position[dominant].z};

            int count = 0;
            for (int index = 0; index < BODY_COUNT && count < Render::MAX_CELESTIAL_BODIES; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                if (!valid[index])
                    continue;
                if (index == dominant)
                    continue; // the observer is at it; it is the ground, not a sky body

                const Vector3 relative = world_position[index] - camera;
                const double camera_distance = length(relative);
                if (camera_distance <= 0.0)
                    continue;
                const Vector3 local_direction = relative * (1.0 / camera_distance);

                // Direction the body's lit hemisphere faces: from the body toward the Sun
                // (for the Sun itself it is unused — the disk is emissive).
                const Vector3 to_sun_ecliptic = helio_ecliptic[index] * -1.0;
                const Vector3 sun_local =
                    body == BodyId::Sun
                        ? local_direction
                        : normalize(to_local(
                              basis, ecliptic_to_body_equatorial(observer_body, to_sun_ecliptic)));

                const BodyProperties properties = body_properties(body);
                const double angular_radius =
                    std::asin(std::fmin(1.0, properties.mean_radius_metres / camera_distance));

                Render::CelestialBody& out = environment.bodies[count];
                out.direction = local_direction;
                out.sun_direction = sun_local;
                out.pole = normalize(to_local(
                    basis, equatorial_to_body_equatorial(observer_body,
                                                         body_north_pole_equatorial(body))));
                out.surface_style = surface_style_for(body);
                out.color = properties.color;
                out.heliocentric_position =
                    WorldVector3{helio_ecliptic[index].x * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_ecliptic[index].y * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_ecliptic[index].z * METRES_PER_ASTRONOMICAL_UNIT};
                out.angular_radius = static_cast<float>(angular_radius);
                out.distance_metres = static_cast<float>(camera_distance);
                out.mean_radius_metres = static_cast<float>(properties.mean_radius_metres);
                const RingExtent ring = ring_extent(body);
                out.ring_inner_metres = static_cast<float>(ring.inner_metres);
                out.ring_outer_metres = static_cast<float>(ring.outer_metres);
                out.body_id = static_cast<std::uint32_t>(index);
                out.is_star = properties.is_star ? 1u : 0u;
                out.lod = lod_for_angular_radius(angular_radius);
                // Reflected bodies scale with albedo; the Sun emits, so it keeps unit scale.
                out.brightness =
                    properties.is_star ? 1.0f : static_cast<float>(properties.geometric_albedo);

                if (body == BodyId::Sun && observer.astronomical_sun)
                    environment.sun.direction = local_direction;

                ++count;
            }
            environment.body_count = count;

            // Solar eclipse: the fraction of the Sun's disk hidden by any nearer body this
            // frame (the Moon from Earth, but the same test catches a Mercury/Venus transit
            // or a moon crossing the Sun elsewhere). Computed once here — a global sky event
            // with negligible parallax across the far Sun — so the sky pass, the analytic
            // ground, and the shaded meshes all dim by one scalar instead of the sky pass
            // dimming alone while lit geometry blazes on. Mirrors sky.frag's disk test.
            environment.solar_eclipse = 0.0f;
            int sun_body = -1;
            for (int i = 0; i < count; ++i)
            {
                if (environment.bodies[i].is_star)
                {
                    sun_body = i;
                    break;
                }
            }
            if (sun_body >= 0)
            {
                const Render::CelestialBody& sun = environment.bodies[sun_body];
                double coverage = 0.0;
                for (int i = 0; i < count; ++i)
                {
                    if (i == sun_body)
                        continue;
                    const Render::CelestialBody& occluder = environment.bodies[i];
                    if (occluder.distance_metres >= sun.distance_metres)
                        continue; // only a body in front of the Sun occludes it
                    const double separation = std::acos(std::fmin(
                        1.0, std::fmax(-1.0, dot(sun.direction, occluder.direction))));
                    coverage = std::fmax(coverage,
                                         disk_overlap_fraction(sun.angular_radius,
                                                               occluder.angular_radius,
                                                               separation));
                }
                environment.solar_eclipse = static_cast<float>(coverage);
            }

            // Lunar eclipse: from Earth, the Moon slides into Earth's own shadow. Its umbra
            // is a disk of angular radius (Moon parallax + Sun parallax - Sun angular radius)
            // centred on the anti-solar point; how much of it covers the Moon's disk is the
            // umbral fraction. Deep in the umbra the Moon is lit only by sunlight refracted
            // red through Earth's atmosphere, so it darkens to a coppery disk — folded here
            // into the Moon body's colour and brightness, no shader change. Earth-specific:
            // it is Earth's shadow, so it applies only to an Earth-based observer.
            if (sun_body >= 0 && observer_body == BodyId::Earth)
            {
                int moon_body = -1;
                for (int i = 0; i < count; ++i)
                {
                    if (environment.bodies[i].body_id ==
                        static_cast<std::uint32_t>(BodyId::Moon))
                    {
                        moon_body = i;
                        break;
                    }
                }
                if (moon_body >= 0)
                {
                    const Render::CelestialBody& sun = environment.bodies[sun_body];
                    Render::CelestialBody& moon = environment.bodies[moon_body];
                    const double earth_radius_metres = body_properties(BodyId::Earth).mean_radius_metres;
                    const double moon_parallax = earth_radius_metres / moon.distance_metres;
                    const double sun_parallax = earth_radius_metres / sun.distance_metres;
                    const double umbra_radius = moon_parallax + sun_parallax - sun.angular_radius;
                    const Vector3 anti_solar = sun.direction * -1.0;
                    const double separation = std::acos(std::fmin(
                        1.0, std::fmax(-1.0, dot(moon.direction, anti_solar))));
                    const double umbral = disk_overlap_fraction(moon.angular_radius,
                                                                umbra_radius, separation);
                    if (umbral > 0.0)
                    {
                        const Vector3 copper{0.42, 0.12, 0.05}; // totally-eclipsed blood moon
                        moon.color = moon.color * (1.0 - umbral) + copper * umbral;
                        moon.brightness *= static_cast<float>(1.0 - 0.85 * umbral);
                    }
                }
            }

            // Every body in the sky is a directional light. The emitter's irradiance is
            // authored; a reflector's follows from the definition of geometric albedo —
            // the flux a sunlit sphere of radius R sends to an observer at distance d is
            // its incident irradiance times albedo * (R/d)^2, scaled by how much of its
            // lit hemisphere faces us. Nothing below names a body: the same three lines
            // give the Moon over Earth, Jupiter over Europa, and earthshine on the Moon,
            // so the night sky lights the scene correctly wherever the camera stands.
            //
            // The numbers are real, not tuned. A full Moon lands near 2.8e-6 of sunlight,
            // which is the measured 0.25 lux against noon's 120000 — so the exposure, not
            // an authored night mode, is what makes moonlight visible.
            {
                const double observer_solar_distance_au =
                    std::fmax(1e-6, length(helio_ecliptic[static_cast<int>(observer_body)]));

                // The list is ordered by what each light actually delivers here, not by its
                // raw output, because a body below the horizon delivers nothing: the Sun
                // outshines the Moon by six orders of magnitude and would still sort first
                // at midnight, and the renderer spends its one shadow map on whichever
                // light sorts first. Weighting by elevation is what hands that map to the
                // Moon after sunset. Only the ordering uses it — the shaders still run the
                // exact horizon test against the body's own ellipsoid. In deep space, with
                // no ground to set behind, nothing is occluded and the weight is one.
                const bool has_horizon = dominant >= 0;
                const auto delivered = [has_horizon](const Render::CelestialLight& light) {
                    if (!has_horizon)
                        return static_cast<double>(light.irradiance);
                    const double above =
                        std::fmin(1.0, std::fmax(0.0, light.direction.y / 0.05 + 0.5));
                    return static_cast<double>(light.irradiance) * above;
                };

                Render::CelestialLight derived[Render::MAX_CELESTIAL_LIGHTS];
                double key[Render::MAX_CELESTIAL_LIGHTS];
                int light_count = 0;

                derived[light_count].direction = normalize(environment.sun.direction);
                derived[light_count].color = environment.sun.color;
                derived[light_count].irradiance = environment.sun.intensity;
                derived[light_count].body_id = static_cast<std::uint32_t>(BodyId::Sun);
                derived[light_count].is_star = 1u;
                key[light_count] = delivered(derived[light_count]);
                ++light_count;

                if (environment.night.enabled)
                {
                    for (int i = 0; i < count; ++i)
                    {
                        const Render::CelestialBody& body = environment.bodies[i];
                        if (body.is_star || body.distance_metres <= 0.0f)
                            continue;

                        // Inverse-square on the body's own solar distance, expressed
                        // against the observer's, because the Sun's authored intensity is
                        // the irradiance here — so the ratio is 1 for a moon of the planet
                        // we stand on and correctly dims a reflector further out.
                        const WorldVector3& helio = body.heliocentric_position;
                        const double body_solar_distance_au =
                            std::fmax(1e-6, std::sqrt(helio.x * helio.x + helio.y * helio.y +
                                                      helio.z * helio.z) /
                                                METRES_PER_ASTRONOMICAL_UNIT);
                        const double solar_falloff =
                            (observer_solar_distance_au * observer_solar_distance_au) /
                            (body_solar_distance_au * body_solar_distance_au);

                        const double solid_angle_ratio =
                            static_cast<double>(body.mean_radius_metres) /
                            static_cast<double>(body.distance_metres);
                        const double phase_angle = std::acos(std::fmin(
                            1.0, std::fmax(-1.0, dot(body.sun_direction,
                                                     body.direction * -1.0))));

                        const double irradiance =
                            environment.sun.intensity * solar_falloff * body.brightness *
                            solid_angle_ratio * solid_angle_ratio *
                            phase_brightness(phase_angle, body.surface_style) *
                            environment.night.reflected_intensity;
                        if (irradiance <= 0.0)
                            continue;

                        // The body's colour is a tint, while its albedo is already carried
                        // by `brightness`; normalising to unit mean keeps the two from
                        // multiplying into a second, silent darkening.
                        const double tint_mean =
                            std::fmax(1e-6, (body.color.x + body.color.y + body.color.z) / 3.0);

                        Render::CelestialLight light;
                        light.direction = body.direction;
                        light.color =
                            Vector3{environment.sun.color.x * body.color.x / tint_mean,
                                    environment.sun.color.y * body.color.y / tint_mean,
                                    environment.sun.color.z * body.color.z / tint_mean};
                        light.irradiance = static_cast<float>(irradiance);
                        light.body_id = body.body_id;
                        light.is_star = 0u;

                        // Insertion sort into a brightest-first list: the array is five
                        // long, and keeping it ordered is what lets the renderer spend its
                        // one shadow map on whichever body actually dominates the frame.
                        const double light_key = delivered(light);
                        int slot = light_count;
                        if (slot >= Render::MAX_CELESTIAL_LIGHTS)
                        {
                            if (key[Render::MAX_CELESTIAL_LIGHTS - 1] >= light_key)
                                continue;
                            slot = Render::MAX_CELESTIAL_LIGHTS - 1;
                        }
                        else
                        {
                            ++light_count;
                        }
                        while (slot > 0 && key[slot - 1] < light_key)
                        {
                            derived[slot] = derived[slot - 1];
                            key[slot] = key[slot - 1];
                            --slot;
                        }
                        derived[slot] = light;
                        key[slot] = light_key;
                    }
                }

                for (int i = 0; i < light_count; ++i)
                    environment.lights[i] = derived[i];
                environment.light_count = light_count;
            }

            // The star field has no direction, so unlike the reflecting bodies above it
            // stays an ambient floor — the residual glow that keeps a moonless night from
            // being pure black. Its ratio to sunlight is as measured (~1e-3 lux against
            // 120000), so it sits far below even a crescent and never washes one out.
            // Only touches `ambient` when astronomical_sun drives the sky, matching how the
            // Sun's direction itself is only astronomically driven under that same flag —
            // otherwise the environment panel's authored ambient is left alone.
            if (observer.astronomical_sun && environment.night.enabled)
            {
                constexpr double STARLIGHT_SOLAR_RATIO = 8.3e-9;
                const double sun_elevation = environment.sun.direction.y;
                const double day_factor =
                    std::fmin(1.0, std::fmax(0.0, sun_elevation / 0.15 + 0.5));

                const Vector3 day_ambient{0.03, 0.04, 0.06};
                const Vector3 night_floor{0.10, 0.11, 0.14}; // starlight tint
                const double star_light = (1.0 - day_factor) * environment.sun.intensity *
                                          STARLIGHT_SOLAR_RATIO *
                                          environment.night.star_intensity;

                environment.ambient = day_ambient * day_factor + night_floor * star_light;
            }

            // Apply the near-field regime. The dominant body's ellipsoid, atmosphere,
            // and ground colours drive the analytic surface pass; with no dominant body
            // (deep space) the ground and air are off and every body is a sky body.
            // Earth keeps the authored environment values so the panel stays in charge
            // of the home planet's look.
            if (dominant >= 0)
            {
                const BodyId dominant_body = static_cast<BodyId>(dominant);
                environment.planet_surface_visible = true;
                environment.planet_center =
                    WorldVector3{world_position[dominant].x, world_position[dominant].y,
                                 world_position[dominant].z};
                environment.planet_pole = normalize(to_local(
                    basis, equatorial_to_body_equatorial(
                               observer_body, body_north_pole_equatorial(dominant_body))));
                environment.planet_surface_style = surface_style_for(dominant_body);
                const RingExtent dominant_ring = ring_extent(dominant_body);
                environment.planet_ring_inner_metres =
                    static_cast<float>(dominant_ring.inner_metres);
                environment.planet_ring_outer_metres =
                    static_cast<float>(dominant_ring.outer_metres);
                // The spherical atmosphere/cloud shells reference this radius, so pick it
                // to put altitude zero at the local ground: at home that is the geodetic
                // distance from Earth's centre to the scene origin (which sits on the
                // ellipsoid), not the equatorial radius — the difference is kilometres of
                // air density at mid latitudes.
                environment.planet_surface_reference_metres =
                    dominant_body == observer_body
                        ? length(observer_center)
                        : surface_preset(dominant_body).semi_major_metres;
                if (dominant_body != BodyId::Earth)
                {
                    const SurfacePreset preset =
                        surface_preset(static_cast<BodyId>(dominant));
                    environment.planet.semi_major = preset.semi_major_metres;
                    environment.planet.inverse_flattening = preset.inverse_flattening;
                    environment.atmosphere.enabled =
                        environment.atmosphere.enabled && preset.has_atmosphere;
                    environment.atmosphere.height = preset.atmosphere_height_metres;
                    environment.atmosphere.rayleigh_coefficient = preset.rayleigh_coefficient;
                    environment.atmosphere.mie_coefficient = preset.mie_coefficient;
                    environment.atmosphere.mie_anisotropy = preset.mie_anisotropy;
                    environment.atmosphere.rayleigh_scale_height =
                        preset.rayleigh_scale_height_metres;
                    environment.atmosphere.mie_scale_height = preset.mie_scale_height_metres;
                    environment.surface.ground_albedo = preset.ground_albedo;
                    environment.surface.ocean_color = preset.ocean_color;
                    environment.surface.roughness = preset.ground_roughness;
                    environment.clouds.enabled =
                        environment.clouds.enabled && preset.has_clouds;
                }
            }
            else
            {
                environment.planet_surface_visible = false;
                environment.planet_ring_inner_metres = 0.0f;
                environment.planet_ring_outer_metres = 0.0f;
                environment.atmosphere.enabled = false;
                environment.clouds.enabled = false;
            }

            // The fixed stars: rotate each catalogue direction into the local frame.
            int star_count = 0;
            for (std::size_t i = 0;
                 i < BRIGHT_STAR_COUNT && star_count < Render::MAX_SKY_STARS; ++i)
            {
                const BrightStar& star = BRIGHT_STARS[i];
                Render::SkyStar& out = environment.sky_stars[star_count];
                out.direction = normalize(to_local(
                    basis, equatorial_to_body_equatorial(observer_body,
                                                         star_equatorial_direction(star))));
                out.color = bv_to_rgb(star.color_index);
                out.brightness = magnitude_to_brightness(star.magnitude);
                ++star_count;
            }
            environment.sky_star_count = star_count;
        }

    } // namespace Astro
} // namespace SushiEngine
