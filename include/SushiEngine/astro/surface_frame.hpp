/**************************************************************************/
/* surface_frame.hpp                                                     */
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
 * @file surface_frame.hpp
 * @brief The body-fixed surface frame: where "down" is and which way is "up" at any
 *        point on a planet, so an entity's pose can be stored relative to the ground.
 *
 * This is the piece that makes a planet-relative transform work. An entity near a body
 * stores its position as body-fixed Cartesian metres (rotating with the planet, origin
 * at its centre) and its orientation relative to the local East-North-Up tangent basis
 * at that position. The tangent basis is *derived from position*, never authored — which
 * is the whole point: a figure standing upright is identity orientation at the north
 * pole, at the equator, and in the southern hemisphere alike, because the "where on the
 * planet" rotation lives entirely in @ref local_tangent_basis and not in the stored pose.
 *
 * Geodetic latitude/longitude are provided only as a boundary conversion, for authoring
 * a spawn point and for the map/UI — the storage and the physics stay Cartesian, free of
 * the pole singularity and the non-uniform metric a lat/lon representation would drag in.
 *
 * Axes are the standard body-fixed convention: +Z along the north rotation pole, +X
 * through the prime meridian, +Y completing the right-handed set. All double precision.
 */

#include <cmath>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief A geodetic surface position: where on a body, and how high above it.
         *
         * Latitude and longitude are radians in the body-fixed frame; altitude is metres
         * along the local ellipsoid normal. The human-facing coordinate — used to author
         * a spawn and to label the map — not the internal transform, which is Cartesian.
         */
        struct GeodeticCoordinate
        {
            double latitude_radians;  /**< Geodetic latitude, [-pi/2, pi/2]. */
            double longitude_radians; /**< Longitude east of the prime meridian, [-pi, pi]. */
            double altitude_metres;   /**< Height above the reference ellipsoid, metres. */
        };

        /**
         * @brief The local East-North-Up tangent basis at a surface point.
         *
         * Body-fixed unit axes: @c up is the outward ellipsoid normal (the direction
         * "down" is the negation of), @c north points along the surface toward the pole,
         * @c east completes the right-handed frame. Composing an entity's authored
         * orientation onto this basis is what removes the hemisphere-dependent rotation
         * from the stored pose.
         */
        struct TangentBasis
        {
            Vector3 east;  /**< Unit east, body-fixed. */
            Vector3 north; /**< Unit north (toward the +Z pole), body-fixed. */
            Vector3 up;    /**< Unit outward ellipsoid normal, body-fixed. */
        };

        /**
         * @brief The reference ellipsoid's squared eccentricity for a body.
         * @param body Which body's ellipsoid to describe.
         * @return e^2 = f(2 - f); zero for a body modelled as a sphere.
         */
        inline double ellipsoid_eccentricity_squared(BodyId body) noexcept
        {
            const SurfacePreset preset = surface_preset(body);
            if (preset.inverse_flattening <= 0.0)
                return 0.0;
            const double f = 1.0 / preset.inverse_flattening;
            return f * (2.0 - f);
        }

        /**
         * @brief Converts a geodetic coordinate to body-fixed Cartesian metres.
         *
         * The standard geodetic-to-ECEF mapping through the prime-vertical radius of
         * curvature N; exact for the reference ellipsoid, and reducing to the sphere when
         * the body has no flattening.
         *
         * @param body       Which body's ellipsoid to place the point on.
         * @param coordinate The geodetic latitude, longitude, and altitude.
         * @return Body-fixed Cartesian position, metres (origin at the body centre).
         */
        inline Vector3 geodetic_to_body_fixed(BodyId body,
                                              const GeodeticCoordinate& coordinate) noexcept
        {
            const SurfacePreset preset = surface_preset(body);
            const double a = preset.semi_major_metres;
            const double e2 = ellipsoid_eccentricity_squared(body);
            const double sin_lat = std::sin(coordinate.latitude_radians);
            const double cos_lat = std::cos(coordinate.latitude_radians);
            const double sin_lon = std::sin(coordinate.longitude_radians);
            const double cos_lon = std::cos(coordinate.longitude_radians);
            const double prime_vertical = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            const double horizontal = (prime_vertical + coordinate.altitude_metres) * cos_lat;
            return Vector3{horizontal * cos_lon, horizontal * sin_lon,
                           (prime_vertical * (1.0 - e2) + coordinate.altitude_metres) * sin_lat};
        }

        /**
         * @brief Converts a body-fixed Cartesian position to a geodetic coordinate.
         *
         * Bowring's closed-form inversion of @ref geodetic_to_body_fixed: one auxiliary
         * angle gives geodetic latitude to sub-millimetre accuracy without iterating.
         * Longitude is exact; altitude falls out of the prime-vertical radius. Provided
         * for the map/UI and for reading back an authored spawn — not called in the
         * per-frame path.
         *
         * @param body     Which body's ellipsoid the point sits on.
         * @param position Body-fixed Cartesian position, metres.
         * @return The geodetic latitude, longitude, and altitude.
         */
        inline GeodeticCoordinate body_fixed_to_geodetic(BodyId body,
                                                         const Vector3& position) noexcept
        {
            const SurfacePreset preset = surface_preset(body);
            const double a = preset.semi_major_metres;
            const double e2 = ellipsoid_eccentricity_squared(body);
            const double b = a * std::sqrt(1.0 - e2);
            const double longitude = std::atan2(position.y, position.x);
            const double horizontal =
                std::sqrt(position.x * position.x + position.y * position.y);

            if (e2 <= 0.0)
            {
                const double radius =
                    std::sqrt(horizontal * horizontal + position.z * position.z);
                const double latitude = std::atan2(position.z, horizontal);
                return GeodeticCoordinate{latitude, longitude, radius - a};
            }

            const double ep2 = e2 / (1.0 - e2);
            const double beta = std::atan2(a * position.z, b * horizontal);
            const double sin_beta = std::sin(beta);
            const double cos_beta = std::cos(beta);
            const double latitude =
                std::atan2(position.z + ep2 * b * sin_beta * sin_beta * sin_beta,
                           horizontal - e2 * a * cos_beta * cos_beta * cos_beta);
            const double sin_lat = std::sin(latitude);
            const double prime_vertical = a / std::sqrt(1.0 - e2 * sin_lat * sin_lat);
            const double altitude = horizontal / std::cos(latitude) - prime_vertical;
            return GeodeticCoordinate{latitude, longitude, altitude};
        }

        /**
         * @brief The outward ellipsoid normal at a body-fixed position — local "up".
         *
         * The gradient of the ellipsoid, normalised; the direction gravity's "down" is the
         * negation of. Differs from the radial direction on a flattened body, which is why
         * it is computed from the ellipsoid rather than as @c normalize(position).
         *
         * @param body     Which body's ellipsoid defines the normal.
         * @param position Body-fixed Cartesian position, metres.
         * @return Unit outward normal, body-fixed.
         */
        inline Vector3 geodetic_normal(BodyId body, const Vector3& position) noexcept
        {
            const double e2 = ellipsoid_eccentricity_squared(body);
            const double one_minus_e2 = 1.0 - e2;
            const Vector3 gradient{position.x, position.y,
                                   one_minus_e2 > 0.0 ? position.z / one_minus_e2 : position.z};
            return normalize(gradient);
        }

        /**
         * @brief The local East-North-Up basis at a body-fixed position.
         *
         * Up is the ellipsoid normal; north is the pole direction projected into the
         * tangent plane; east completes the right-handed frame. This is the rotation an
         * entity's authored orientation is composed onto, so that identity means "upright,
         * facing north" everywhere on the body.
         *
         * @param body     Which body's ellipsoid defines the tangent plane.
         * @param position Body-fixed Cartesian position, metres.
         * @return The East-North-Up unit basis, body-fixed.
         */
        inline TangentBasis local_tangent_basis(BodyId body, const Vector3& position) noexcept
        {
            const Vector3 up = geodetic_normal(body, position);
            const Vector3 pole{0.0, 0.0, 1.0};
            Vector3 east = cross(pole, up);
            const double east_length = length(east);
            // At the poles the pole and up are parallel; fall back to the +X meridian.
            east = east_length > 1e-9 ? east * (1.0 / east_length) : Vector3{1.0, 0.0, 0.0};
            const Vector3 north = cross(up, east);
            return TangentBasis{east, north, up};
        }

        /**
         * @brief Surface gravitational acceleration magnitude at a body's equator.
         *
         * The reference value g0 = mu / a^2 the near-field regime scales its uniform
         * "down" by; direction comes from @ref geodetic_normal, so the field points to the
         * planet on a round world without a full field evaluation.
         *
         * @param body Which body's surface gravity to report.
         * @return Surface gravity magnitude, metres/second^2.
         */
        inline double surface_gravity(BodyId body) noexcept
        {
            const double mu = standard_gravitational_parameter(body);
            const double a = surface_preset(body).semi_major_metres;
            if (a <= 0.0)
                return 0.0;
            return mu / (a * a);
        }

        /**
         * @brief The near-field gravity vector at a body-fixed position — "down" times g0.
         *
         * The surface regime's cheap stand-in for @ref gravity_field: a constant-magnitude
         * pull along the inward ellipsoid normal. Correctly oriented everywhere on the
         * body (curving around it), which is what a uniform world-frame gravity could
         * never be — and the reason a southern-hemisphere entity stands upright rather
         * than tilted.
         *
         * @param body     Which body the entity stands on.
         * @param position Body-fixed Cartesian position, metres.
         * @return Gravity acceleration, body-fixed metres/second^2, pointing inward.
         */
        inline Vector3 surface_gravity_vector(BodyId body, const Vector3& position) noexcept
        {
            return geodetic_normal(body, position) * (-surface_gravity(body));
        }
    } // namespace Astro
} // namespace SushiEngine
