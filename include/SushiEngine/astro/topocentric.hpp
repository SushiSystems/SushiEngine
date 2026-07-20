/**************************************************************************/
/* topocentric.hpp                                                       */
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
 * @file topocentric.hpp
 * @brief The observer's local East-Up-South basis: how an equatorial direction lands in
 *        the scene's local frame, and the projection that does it.
 *
 * The pure, render-free core of the scene's orientation. An observer standing at a
 * latitude with a given local sidereal (or body-rotation) meridian sees a fixed
 * rotation between the equatorial frame the ephemeris works in and the local ENU frame
 * the scene is drawn in. This header owns that basis and its projection so both the
 * ephemeris (which places every sky body) and the scene-frame bijection (which converts
 * an entity between heliocentric and scene coordinates) share one construction rather
 * than each inlining it.
 *
 * Axes are the scene convention: x = east, y = up (geodetic vertical), z = south.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief The observer's topocentric basis in equatorial coordinates.
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
         * @brief Expands a scene-local direction back into equatorial coordinates.
         *
         * The exact inverse of @ref to_local: because the basis is orthonormal, the
         * inverse is its transpose — the local components scale the basis vectors and sum.
         * A round trip through @ref to_local and this is the identity.
         *
         * @param basis The observer's topocentric basis.
         * @param local A direction in local coordinates (x=east, y=up, z=south).
         * @return The same direction in equatorial coordinates.
         */
        inline Vector3 from_local(const LocalSkyBasis& basis, const Vector3& local) noexcept
        {
            return Vector3{basis.east.x * local.x + basis.up.x * local.y + basis.south.x * local.z,
                           basis.east.y * local.x + basis.up.y * local.y + basis.south.y * local.z,
                           basis.east.z * local.x + basis.up.z * local.y + basis.south.z * local.z};
        }
    } // namespace Astro
} // namespace SushiEngine
