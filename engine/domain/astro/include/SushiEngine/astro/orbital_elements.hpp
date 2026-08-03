/**************************************************************************/
/* orbital_elements.hpp                                                   */
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
 * @file orbital_elements.hpp
 * @brief Keplerian orbital elements and the two-body position they generate.
 *
 * Each planet's heliocentric position is reconstructed from six osculating
 * elements that vary linearly with time (Standish's "Keplerian Elements for
 * Approximate Positions of the Major Planets", valid 1800-2050 AD). Given a date
 * the element set is linearly propagated, Kepler's equation is solved for the
 * eccentric anomaly, and the orbit-plane position is rotated into the J2000 ecliptic
 * frame. Positions come out in astronomical units — the unit the whole solar-system
 * scale is built on.
 *
 * The element convention (all angles degrees, distances AU, rates per century):
 *   a  semi-major axis, e eccentricity, I inclination, L mean longitude,
 *   longitude_of_perihelion, longitude_of_ascending_node.
 */

#include <cmath>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /** @brief Metres in one astronomical unit (IAU 2012 definition). */
        constexpr double METRES_PER_ASTRONOMICAL_UNIT = 1.495978707e11;

        /**
         * @brief A set of six Keplerian elements plus their per-century linear rates.
         *
         * Propagated to a date by @c element_at; the rate fields carry the secular
         * drift that makes a single row valid across the 1800-2050 span rather than
         * only at the J2000 epoch.
         */
        struct KeplerianElements
        {
            double semi_major_axis_au;            /**< a at J2000, astronomical units. */
            double semi_major_axis_rate;          /**< da/dt, AU per century. */
            double eccentricity;                  /**< e at J2000, dimensionless. */
            double eccentricity_rate;             /**< de/dt per century. */
            double inclination_degrees;           /**< I at J2000, degrees. */
            double inclination_rate;              /**< dI/dt, degrees per century. */
            double mean_longitude_degrees;        /**< L at J2000, degrees. */
            double mean_longitude_rate;           /**< dL/dt, degrees per century. */
            double longitude_perihelion_degrees;  /**< longitude of perihelion, degrees. */
            double longitude_perihelion_rate;     /**< its rate, degrees per century. */
            double longitude_node_degrees;        /**< longitude of ascending node, degrees. */
            double longitude_node_rate;           /**< its rate, degrees per century. */
        };

        /**
         * @brief The six elements evaluated linearly at a number of Julian centuries.
         * @param base The J2000 element set with its rates.
         * @param centuries Julian centuries since J2000.0.
         * @return A copy of @p base with each element advanced by rate * centuries.
         */
        inline KeplerianElements element_at(const KeplerianElements& base, double centuries) noexcept
        {
            KeplerianElements e = base;
            e.semi_major_axis_au += base.semi_major_axis_rate * centuries;
            e.eccentricity += base.eccentricity_rate * centuries;
            e.inclination_degrees += base.inclination_rate * centuries;
            e.mean_longitude_degrees += base.mean_longitude_rate * centuries;
            e.longitude_perihelion_degrees += base.longitude_perihelion_rate * centuries;
            e.longitude_node_degrees += base.longitude_node_rate * centuries;
            return e;
        }

        /**
         * @brief Solves Kepler's equation M = E - e sin(E) for the eccentric anomaly.
         *
         * Newton-Raphson from an M seed; converges in a handful of iterations for the
         * planets' modest eccentricities (Pluto's 0.25 is the worst case here).
         *
         * @param mean_anomaly_radians Mean anomaly M, radians.
         * @param eccentricity Orbital eccentricity e in [0, 1).
         * @return The eccentric anomaly E, radians.
         */
        inline double solve_kepler(double mean_anomaly_radians, double eccentricity) noexcept
        {
            double e_anomaly = mean_anomaly_radians;
            for (int i = 0; i < 8; ++i)
            {
                const double delta =
                    (e_anomaly - eccentricity * std::sin(e_anomaly) - mean_anomaly_radians) /
                    (1.0 - eccentricity * std::cos(e_anomaly));
                e_anomaly -= delta;
                if (std::fabs(delta) < 1e-12)
                    break;
            }
            return e_anomaly;
        }

        /**
         * @brief Heliocentric J2000 ecliptic position from a propagated element set.
         *
         * Reduces the elements to argument of perihelion and mean anomaly, solves for
         * the eccentric anomaly, forms the position in the orbital plane, then rotates
         * by node, inclination, and perihelion into the ecliptic frame.
         *
         * @param elements The elements already evaluated at the target date.
         * @return Heliocentric position in the J2000 ecliptic frame, astronomical units.
         */
        inline Vector3 heliocentric_ecliptic_position(const KeplerianElements& elements) noexcept
        {
            const double a = elements.semi_major_axis_au;
            const double e = elements.eccentricity;
            const double inclination = elements.inclination_degrees * DEGREES_TO_RADIANS;
            const double node = elements.longitude_node_degrees * DEGREES_TO_RADIANS;
            const double perihelion_longitude =
                elements.longitude_perihelion_degrees * DEGREES_TO_RADIANS;
            const double argument_perihelion = perihelion_longitude - node;

            double mean_anomaly =
                (elements.mean_longitude_degrees - elements.longitude_perihelion_degrees);
            mean_anomaly = wrap_degrees(mean_anomaly + 180.0) - 180.0;
            const double mean_anomaly_radians = mean_anomaly * DEGREES_TO_RADIANS;

            const double eccentric_anomaly = solve_kepler(mean_anomaly_radians, e);

            // Position in the orbital plane: x toward perihelion, y along motion.
            const double x_plane = a * (std::cos(eccentric_anomaly) - e);
            const double y_plane = a * std::sqrt(1.0 - e * e) * std::sin(eccentric_anomaly);

            const double cos_w = std::cos(argument_perihelion);
            const double sin_w = std::sin(argument_perihelion);
            const double cos_o = std::cos(node);
            const double sin_o = std::sin(node);
            const double cos_i = std::cos(inclination);
            const double sin_i = std::sin(inclination);

            const double x =
                (cos_w * cos_o - sin_w * sin_o * cos_i) * x_plane +
                (-sin_w * cos_o - cos_w * sin_o * cos_i) * y_plane;
            const double y =
                (cos_w * sin_o + sin_w * cos_o * cos_i) * x_plane +
                (-sin_w * sin_o + cos_w * cos_o * cos_i) * y_plane;
            const double z = (sin_w * sin_i) * x_plane + (cos_w * sin_i) * y_plane;

            return Vector3{x, y, z};
        }

        /**
         * @brief Rotates a J2000 ecliptic vector into the J2000 equatorial frame.
         *
         * A single rotation about the x-axis (the equinox line) by the mean obliquity —
         * the tilt between the Earth's equator and its orbit plane.
         *
         * @param ecliptic A vector in the ecliptic frame (any units).
         * @return The same vector expressed in the equatorial frame.
         */
        inline Vector3 ecliptic_to_equatorial(const Vector3& ecliptic) noexcept
        {
            const double cos_e = std::cos(OBLIQUITY_J2000_RADIANS);
            const double sin_e = std::sin(OBLIQUITY_J2000_RADIANS);
            return Vector3{ecliptic.x, ecliptic.y * cos_e - ecliptic.z * sin_e,
                           ecliptic.y * sin_e + ecliptic.z * cos_e};
        }

        /**
         * @brief Rotates a J2000 equatorial vector into the J2000 ecliptic frame.
         *
         * The inverse of @ref ecliptic_to_equatorial: a rotation about the x-axis by the
         * negative obliquity. Used to bring the equatorial star catalogue into the ecliptic
         * frame the interplanetary (space) regime works in.
         *
         * @param equatorial A vector in the equatorial frame (any units).
         * @return The same vector expressed in the ecliptic frame.
         */
        inline Vector3 equatorial_to_ecliptic(const Vector3& equatorial) noexcept
        {
            const double cos_e = std::cos(OBLIQUITY_J2000_RADIANS);
            const double sin_e = std::sin(OBLIQUITY_J2000_RADIANS);
            return Vector3{equatorial.x, equatorial.y * cos_e + equatorial.z * sin_e,
                           -equatorial.y * sin_e + equatorial.z * cos_e};
        }
    } // namespace Astro
} // namespace SushiEngine
