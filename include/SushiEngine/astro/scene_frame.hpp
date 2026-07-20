/**************************************************************************/
/* scene_frame.hpp                                                       */
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
 * @file scene_frame.hpp
 * @brief The bijection between a heliocentric-ecliptic position and the scene's local
 *        frame: the one place that knows where the scene origin sits in the solar system.
 *
 * The scene is drawn in a body-anchored topocentric frame — origin at the observer's
 * surface point, +Y along the local vertical, axes turning with the observer body's day
 * (see @ref topocentric.hpp). The ephemeris uses this construction to place every sky
 * body; a free entity whose physical truth is a heliocentric @ref StateVector needs the
 * inverse of it to be drawn, and its authored scene pose needs the forward direction to
 * become truth. @ref SceneFrame owns that map both ways so neither the ephemeris nor the
 * simulation re-derives it.
 *
 * The map is a rigid isometry: @c scene = observer_center + R * (heliocentric -
 * observer_heliocentric), with R = @ref to_local composed with
 * @ref ecliptic_to_body_equatorial, so it is exact and its inverse is R's transpose. It
 * is purely positional; the scene frame is non-inertial (it orbits and spins with the
 * body), so converting a *velocity* between the frames additionally needs the frame's own
 * motion and is handled by the dynamics layer, not here.
 */

#include <cmath>

#include <SushiEngine/astro/body_orientation.hpp>
#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>
#include <SushiEngine/astro/topocentric.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief The rigid map between heliocentric-ecliptic metres and scene metres.
         *
         * Built for one observer instant by @ref scene_frame_for. Holds the observer
         * body's topocentric basis, its heliocentric position, and the scene-frame offset
         * of the body centre, which together define the isometry and its inverse. All
         * double precision, metres.
         */
        struct SceneFrame
        {
            BodyId observer_body;                 /**< The body the scene is anchored to. */
            LocalSkyBasis basis;                  /**< Observer topocentric basis, equatorial coords. */
            WorldVector3 observer_heliocentric;   /**< Observer body centre, heliocentric-ecliptic metres. */
            WorldVector3 observer_center;         /**< Observer body centre expressed in scene coords, metres. */

            /**
             * @brief Rotates a heliocentric-ecliptic direction into scene axes.
             *
             * The rotation half of the map (no translation): ecliptic → observer-body
             * equatorial → local scene axes. Use for directions — a pole, a velocity's
             * orientation — not for positions, which also carry the origin offset.
             *
             * @param ecliptic A direction in heliocentric-ecliptic coordinates.
             * @return The same direction in scene axes.
             */
            Vector3 direction_to_scene(const Vector3& ecliptic) const noexcept
            {
                return to_local(basis, ecliptic_to_body_equatorial(observer_body, ecliptic));
            }

            /**
             * @brief Rotates a scene-axis direction back into heliocentric-ecliptic.
             * @param scene A direction in scene axes.
             * @return The same direction in heliocentric-ecliptic coordinates.
             */
            Vector3 direction_to_heliocentric(const Vector3& scene) const noexcept
            {
                return body_equatorial_to_ecliptic(observer_body, from_local(basis, scene));
            }

            /**
             * @brief Places a heliocentric-ecliptic position in the scene frame.
             * @param heliocentric A position in heliocentric-ecliptic metres.
             * @return The same point in scene metres.
             */
            WorldVector3 scene_from_heliocentric(const WorldVector3& heliocentric) const noexcept
            {
                const Vector3 offset{heliocentric.x - observer_heliocentric.x,
                                     heliocentric.y - observer_heliocentric.y,
                                     heliocentric.z - observer_heliocentric.z};
                const Vector3 scene = direction_to_scene(offset);
                return WorldVector3{scene.x + observer_center.x,
                                    scene.y + observer_center.y,
                                    scene.z + observer_center.z};
            }

            /**
             * @brief Recovers the heliocentric-ecliptic position of a scene point.
             * @param scene A position in scene metres.
             * @return The same point in heliocentric-ecliptic metres.
             */
            WorldVector3 heliocentric_from_scene(const WorldVector3& scene) const noexcept
            {
                const Vector3 local{scene.x - observer_center.x,
                                    scene.y - observer_center.y,
                                    scene.z - observer_center.z};
                const Vector3 ecliptic = direction_to_heliocentric(local);
                return WorldVector3{ecliptic.x + observer_heliocentric.x,
                                    ecliptic.y + observer_heliocentric.y,
                                    ecliptic.z + observer_heliocentric.z};
            }
        };

        /**
         * @brief Builds the scene frame for an observer instant.
         *
         * Reproduces the ephemeris's scene construction exactly (see @ref fill_environment_sky):
         * the meridian is sidereal time on Earth and the body-rotation angle elsewhere; the
         * origin is the observer's geodetic surface point on the body's reference ellipsoid,
         * so a body-anchored entity and the rendered ground share one frame.
         *
         * @param julian_date   The epoch.
         * @param latitude_radians  Observer geodetic latitude.
         * @param longitude_radians Observer east longitude.
         * @param observer_body The body the scene is anchored to.
         * @return The scene frame for that instant.
         */
        inline SceneFrame scene_frame_for(double julian_date, double latitude_radians,
                                          double longitude_radians, BodyId observer_body) noexcept
        {
            const double meridian =
                observer_body == BodyId::Earth
                    ? local_mean_sidereal_time(julian_date, longitude_radians)
                    : body_rotation_angle(observer_body, julian_date) + longitude_radians;

            SceneFrame frame;
            frame.observer_body = observer_body;
            frame.basis = local_sky_basis(meridian, latitude_radians);

            const Vector3 helio_au = planet_heliocentric_au(observer_body, julian_date);
            frame.observer_heliocentric = WorldVector3{helio_au.x * METRES_PER_ASTRONOMICAL_UNIT,
                                                       helio_au.y * METRES_PER_ASTRONOMICAL_UNIT,
                                                       helio_au.z * METRES_PER_ASTRONOMICAL_UNIT};

            const SurfacePreset surface = surface_preset(observer_body);
            const double a = surface.semi_major_metres;
            const double f = surface.inverse_flattening > 0.0 ? 1.0 / surface.inverse_flattening : 0.0;
            const double e2 = f * (2.0 - f);
            const double sin_lat = std::sin(latitude_radians);
            const double cos_lat = std::cos(latitude_radians);
            const double prime_vertical = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            const Vector3 observer_equatorial{prime_vertical * cos_lat * std::cos(meridian),
                                              prime_vertical * cos_lat * std::sin(meridian),
                                              prime_vertical * (1.0 - e2) * sin_lat};
            const Vector3 center = to_local(frame.basis, observer_equatorial) * -1.0;
            frame.observer_center = WorldVector3{center.x, center.y, center.z};
            return frame;
        }
    } // namespace Astro
} // namespace SushiEngine
