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

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>
#include <SushiEngine/astro/star_catalog.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/environment.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief The observer's local East-Up-South basis, expressed in the equatorial frame.
         *
         * Right-handed with @c up along the geodetic vertical and @c east along the
         * horizon; @c south completes it (X=east, Y=up, Z=south) to match the scene's
         * local axes. Any equatorial direction dotted against these three lands in the
         * scene's local ENU frame.
         */
        struct LocalSkyBasis
        {
            Vector3 east;  /**< Unit east, equatorial coordinates. */
            Vector3 up;    /**< Unit geodetic up, equatorial coordinates. */
            Vector3 south; /**< Unit south, equatorial coordinates. */
        };

        /**
         * @brief Builds the topocentric basis from sidereal time and latitude.
         * @param local_sidereal_time_radians Local mean sidereal time, radians.
         * @param latitude_radians Observer geodetic latitude, radians.
         * @return The East-Up-South basis in equatorial coordinates.
         */
        inline LocalSkyBasis local_sky_basis(double local_sidereal_time_radians,
                                             double latitude_radians) noexcept
        {
            const double sin_lst = std::sin(local_sidereal_time_radians);
            const double cos_lst = std::cos(local_sidereal_time_radians);
            const double sin_lat = std::sin(latitude_radians);
            const double cos_lat = std::cos(latitude_radians);

            LocalSkyBasis basis;
            basis.up = Vector3{cos_lat * cos_lst, cos_lat * sin_lst, sin_lat};
            basis.east = Vector3{-sin_lst, cos_lst, 0.0};
            const Vector3 north{-sin_lat * cos_lst, -sin_lat * sin_lst, cos_lat};
            basis.south = north * -1.0;
            return basis;
        }

        /**
         * @brief Projects an equatorial direction into the scene's local ENU frame.
         * @param basis The observer's topocentric basis.
         * @param equatorial A direction in the equatorial frame (need not be unit).
         * @return The same direction in local coordinates (x=east, y=up, z=south).
         */
        inline Vector3 to_local(const LocalSkyBasis& basis, const Vector3& equatorial) noexcept
        {
            return Vector3{dot(equatorial, basis.east), dot(equatorial, basis.up),
                           dot(equatorial, basis.south)};
        }

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
         * @brief Fills an environment's far-field sky from its observer settings.
         *
         * Reads @c environment.observer (epoch, latitude, longitude) and writes the
         * @c bodies / @c body_count and @c sky_stars / @c sky_star_count arrays, and —
         * when @c observer.astronomical_sun is set — the directional-light direction. The
         * near-field planet, atmosphere, cloud, and material fields are left untouched.
         *
         * @param environment The environment to populate; its observer is the input.
         */
        inline void fill_environment_sky(Render::Environment& environment) noexcept
        {
            const Render::SkyObserver& observer = environment.observer;
            const double julian_date = observer.julian_date;
            const double lst =
                local_mean_sidereal_time(julian_date, observer.longitude_radians);
            const LocalSkyBasis basis = local_sky_basis(lst, observer.latitude_radians);

            const Vector3 earth_helio = planet_heliocentric_au(BodyId::Earth, julian_date);

            int count = 0;
            for (int index = 0; index < BODY_COUNT && count < Render::MAX_CELESTIAL_BODIES; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);
                if (body == BodyId::Earth)
                    continue; // the observer stands on it; it is the ground, not a sky body

                // Heliocentric ecliptic position (Sun at origin, Moon geocentric special case).
                Vector3 helio_ecliptic;
                Vector3 geocentric_ecliptic;
                if (body == BodyId::Sun)
                {
                    helio_ecliptic = Vector3{0.0, 0.0, 0.0};
                    geocentric_ecliptic = earth_helio * -1.0;
                }
                else if (body == BodyId::Moon)
                {
                    geocentric_ecliptic = moon_geocentric_ecliptic_au(julian_date);
                    helio_ecliptic = earth_helio + geocentric_ecliptic;
                }
                else
                {
                    helio_ecliptic = planet_heliocentric_au(body, julian_date);
                    geocentric_ecliptic = helio_ecliptic - earth_helio;
                }

                const double distance_au = length(geocentric_ecliptic);
                if (distance_au <= 0.0)
                    continue;
                const double distance_metres = distance_au * METRES_PER_ASTRONOMICAL_UNIT;

                const Vector3 geocentric_equatorial = ecliptic_to_equatorial(geocentric_ecliptic);
                const Vector3 local_direction = normalize(to_local(basis, geocentric_equatorial));

                // Direction the body's lit hemisphere faces: from the body toward the Sun.
                const Vector3 to_sun_ecliptic =
                    body == BodyId::Sun ? geocentric_ecliptic : (helio_ecliptic * -1.0);
                const Vector3 sun_local =
                    normalize(to_local(basis, ecliptic_to_equatorial(to_sun_ecliptic)));

                const BodyProperties properties = body_properties(body);
                const double angular_radius =
                    std::asin(std::fmin(1.0, properties.mean_radius_metres / distance_metres));

                Render::CelestialBody& out = environment.bodies[count];
                out.direction = local_direction;
                out.sun_direction = sun_local;
                out.color = properties.color;
                out.heliocentric_position =
                    WorldVector3{helio_ecliptic.x * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_ecliptic.y * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_ecliptic.z * METRES_PER_ASTRONOMICAL_UNIT};
                out.angular_radius = static_cast<float>(angular_radius);
                out.distance_metres = static_cast<float>(distance_metres);
                out.mean_radius_metres = static_cast<float>(properties.mean_radius_metres);
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

            // The fixed stars: rotate each catalogue direction into the local frame.
            int star_count = 0;
            for (std::size_t i = 0;
                 i < BRIGHT_STAR_COUNT && star_count < Render::MAX_SKY_STARS; ++i)
            {
                const BrightStar& star = BRIGHT_STARS[i];
                Render::SkyStar& out = environment.sky_stars[star_count];
                out.direction = normalize(to_local(basis, star_equatorial_direction(star)));
                out.color = bv_to_rgb(star.color_index);
                out.brightness = magnitude_to_brightness(star.magnitude);
                ++star_count;
            }
            environment.sky_star_count = star_count;
        }

        /** @brief Metres in one gigametre, the unit length of the interplanetary world frame. */
        constexpr double METRES_PER_GIGAMETRE = 1.0e9;

        /**
         * @brief Remaps a J2000 ecliptic vector into the space regime's world frame.
         *
         * The space regime works in an ecliptic frame rotated so world up (+Y) is the
         * ecliptic north pole and the ecliptic plane is the world XZ plane — the natural
         * orientation for free-look flight through a solar system that lies roughly in one
         * plane. A single fixed rotation: (x, y, z) -> (x, z, -y).
         *
         * @param ecliptic A vector in the J2000 ecliptic frame (any units).
         * @return The vector in the world frame, Y up.
         */
        inline Vector3 ecliptic_to_world(const Vector3& ecliptic) noexcept
        {
            return Vector3{ecliptic.x, ecliptic.z, -ecliptic.y};
        }

        /**
         * @brief Fills an environment's bodies for the interplanetary (space) regime.
         *
         * Places every body — the Sun at the origin, the planets, and the Moon — plus the
         * fixed stars relative to a free-flying camera, all in a single heliocentric
         * ecliptic world frame measured in gigametres. Working camera-relative in double on
         * the CPU and in gigametres on the GPU keeps a body a kilometre away and one five
         * billion kilometres away both representable in single precision: the near body
         * becomes a real lit sphere (the shader's near regime), the far ones stay
         * direction-space disks. The directional light points at the Sun (the origin), and
         * the atmosphere/ground of the near-field surface regime is switched off — that
         * hand-off is a separate stage.
         *
         * @param environment      The environment to populate; its observer supplies the epoch.
         * @param camera_world_gigametres The camera position in the world frame, gigametres.
         */
        inline void fill_environment_space(Render::Environment& environment,
                                           const WorldVector3& camera_world_gigametres) noexcept
        {
            const double julian_date = environment.observer.julian_date;
            const Vector3 camera{camera_world_gigametres.x, camera_world_gigametres.y,
                                 camera_world_gigametres.z};
            const double au_to_gigametre =
                METRES_PER_ASTRONOMICAL_UNIT / METRES_PER_GIGAMETRE;

            const Vector3 earth_helio_au = planet_heliocentric_au(BodyId::Earth, julian_date);

            int count = 0;
            for (int index = 0; index < BODY_COUNT && count < Render::MAX_CELESTIAL_BODIES; ++index)
            {
                const BodyId body = static_cast<BodyId>(index);

                Vector3 helio_au;
                if (body == BodyId::Sun)
                    helio_au = Vector3{0.0, 0.0, 0.0};
                else if (body == BodyId::Moon)
                    helio_au = earth_helio_au + moon_geocentric_ecliptic_au(julian_date);
                else if (body == BodyId::Earth)
                    helio_au = earth_helio_au;
                else
                    helio_au = planet_heliocentric_au(body, julian_date);

                const Vector3 helio_world = ecliptic_to_world(helio_au * au_to_gigametre);
                const Vector3 relative = helio_world - camera;
                const double distance = length(relative);
                if (distance <= 0.0)
                    continue;
                const Vector3 direction = relative * (1.0 / distance);

                const BodyProperties properties = body_properties(body);
                const double radius_gigametres =
                    properties.mean_radius_metres / METRES_PER_GIGAMETRE;
                const double angular_radius =
                    std::asin(std::fmin(1.0, radius_gigametres / distance));

                // The body's lit hemisphere faces the Sun at the world origin.
                const Vector3 sun_direction =
                    body == BodyId::Sun ? direction : normalize(helio_world * -1.0);

                Render::CelestialBody& out = environment.bodies[count];
                out.direction = direction;
                out.sun_direction = sun_direction;
                out.color = properties.color;
                out.heliocentric_position =
                    WorldVector3{helio_au.x * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_au.y * METRES_PER_ASTRONOMICAL_UNIT,
                                 helio_au.z * METRES_PER_ASTRONOMICAL_UNIT};
                out.angular_radius = static_cast<float>(angular_radius);
                // In this regime distance and radius are the world frame's gigametres, not
                // metres — the shader only needs them self-consistent for depth and sizing.
                out.distance_metres = static_cast<float>(distance);
                out.mean_radius_metres = static_cast<float>(radius_gigametres);
                out.is_star = properties.is_star ? 1u : 0u;
                out.lod = lod_for_angular_radius(angular_radius);
                out.brightness =
                    properties.is_star ? 1.0f : static_cast<float>(properties.geometric_albedo);
                ++count;
            }
            environment.body_count = count;

            int star_count = 0;
            for (std::size_t i = 0;
                 i < BRIGHT_STAR_COUNT && star_count < Render::MAX_SKY_STARS; ++i)
            {
                const BrightStar& star = BRIGHT_STARS[i];
                Render::SkyStar& out = environment.sky_stars[star_count];
                out.direction =
                    ecliptic_to_world(equatorial_to_ecliptic(star_equatorial_direction(star)));
                out.color = bv_to_rgb(star.color_index);
                out.brightness = magnitude_to_brightness(star.magnitude);
                ++star_count;
            }
            environment.sky_star_count = star_count;

            // Sunlight comes from the world origin; the near-field ground/air is off here.
            if (environment.observer.astronomical_sun)
            {
                const double camera_distance = length(camera);
                environment.sun.direction =
                    camera_distance > 0.0 ? camera * (-1.0 / camera_distance)
                                          : Vector3{0.0, 1.0, 0.0};
            }
            environment.observer.space_mode = true;
            environment.atmosphere.enabled = false;
            environment.clouds.enabled = false;
        }
    } // namespace Astro
} // namespace SushiEngine
