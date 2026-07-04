/**************************************************************************/
/* celestial_bodies.hpp                                                   */
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
 * @file celestial_bodies.hpp
 * @brief The catalogue of solar-system bodies: which they are, their physical size
 *        and colour, and (for the planets) the Keplerian element rows that place them.
 *
 * The Sun anchors the heliocentric frame at the origin; the eight planets and Pluto
 * are propagated from Standish's element table; the Moon is a geocentric special case
 * with its own short-period series (@ref moon_geocentric_ecliptic_au). Physical radii
 * and display colours let the renderer size and tint each body's disk without a
 * texture. All distances are metres, all colours linear RGB.
 */

#include <cmath>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief Every body the engine models, in heliocentric order plus the Moon.
         *
         * The ordinal doubles as the catalogue index; @c Count bounds the arrays. The
         * Sun and Moon are handled specially (origin / geocentric); the rest are driven
         * by @ref keplerian_elements_for.
         */
        enum class BodyId : int
        {
            Sun = 0,
            Mercury,
            Venus,
            Earth,
            Moon,
            Mars,
            Jupiter,
            Saturn,
            Uranus,
            Neptune,
            Pluto,
            Count
        };

        /** @brief Number of catalogued bodies. */
        constexpr int BODY_COUNT = static_cast<int>(BodyId::Count);

        /**
         * @brief Static physical description of one body: how big it is and how it looks.
         *
         * @c mean_radius_metres sizes the rendered disk and the mesh LOD; @c color tints
         * it; @c geometric_albedo scales reflected sunlight for relative brightness. The
         * Sun's albedo is unused (it is the light source).
         */
        struct BodyProperties
        {
            const char* name;          /**< Display name. */
            double mean_radius_metres; /**< Volumetric mean radius, metres. */
            double geometric_albedo;   /**< Fraction of incident light reflected, [0, 1]. */
            Vector3 color;             /**< Linear RGB tint of the body's surface. */
            bool is_star;              /**< True for the Sun: it emits rather than reflects. */
        };

        /**
         * @brief The physical properties of a body.
         * @param body Which body to describe.
         * @return Its radius, albedo, colour, and star flag.
         */
        inline BodyProperties body_properties(BodyId body) noexcept
        {
            switch (body)
            {
                case BodyId::Sun:
                    return {"Sun", 6.957e8, 0.0, Vector3{1.0, 0.96, 0.9}, true};
                case BodyId::Mercury:
                    return {"Mercury", 2.4397e6, 0.142, Vector3{0.62, 0.58, 0.53}, false};
                case BodyId::Venus:
                    return {"Venus", 6.0518e6, 0.689, Vector3{0.96, 0.92, 0.78}, false};
                case BodyId::Earth:
                    return {"Earth", 6.371e6, 0.434, Vector3{0.28, 0.42, 0.62}, false};
                case BodyId::Moon:
                    return {"Moon", 1.7374e6, 0.136, Vector3{0.66, 0.65, 0.62}, false};
                case BodyId::Mars:
                    return {"Mars", 3.3895e6, 0.170, Vector3{0.82, 0.44, 0.28}, false};
                case BodyId::Jupiter:
                    return {"Jupiter", 6.9911e7, 0.538, Vector3{0.86, 0.76, 0.62}, false};
                case BodyId::Saturn:
                    return {"Saturn", 5.8232e7, 0.499, Vector3{0.90, 0.83, 0.66}, false};
                case BodyId::Uranus:
                    return {"Uranus", 2.5362e7, 0.488, Vector3{0.62, 0.85, 0.90}, false};
                case BodyId::Neptune:
                    return {"Neptune", 2.4622e7, 0.442, Vector3{0.33, 0.47, 0.86}, false};
                case BodyId::Pluto:
                    return {"Pluto", 1.1883e6, 0.52, Vector3{0.74, 0.68, 0.62}, false};
                default:
                    return {"", 0.0, 0.0, Vector3{1.0, 1.0, 1.0}, false};
            }
        }

        /**
         * @brief The J2000 Keplerian element row for a planet (Standish 1800-2050 table).
         *
         * Defined for Mercury through Pluto and for Earth (the Earth-Moon barycentre row,
         * which the geocentric transform uses to find the observer's position). The Sun
         * and Moon return a zeroed set — they are not propagated this way.
         *
         * @param body Which body's elements to fetch.
         * @return The element row with its per-century rates.
         */
        inline KeplerianElements keplerian_elements_for(BodyId body) noexcept
        {
            switch (body)
            {
                case BodyId::Mercury:
                    return {0.38709927, 0.00000037, 0.20563593, 0.00001906, 7.00497902, -0.00594749,
                            252.25032350, 149472.67411175, 77.45779628, 0.16047689, 48.33076593,
                            -0.12534081};
                case BodyId::Venus:
                    return {0.72333566, 0.00000390, 0.00677672, -0.00004107, 3.39467605, -0.00078890,
                            181.97909950, 58517.81538729, 131.60246718, 0.00268329, 76.67984255,
                            -0.27769418};
                case BodyId::Earth:
                    return {1.00000261, 0.00000562, 0.01671123, -0.00004392, -0.00001531, -0.01294668,
                            100.46457166, 35999.37244981, 102.93768193, 0.32327364, 0.0, 0.0};
                case BodyId::Mars:
                    return {1.52371034, 0.00001847, 0.09339410, 0.00007882, 1.84969142, -0.00813131,
                            -4.55343205, 19140.30268499, -23.94362959, 0.44441088, 49.55953891,
                            -0.29257343};
                case BodyId::Jupiter:
                    return {5.20288700, -0.00011607, 0.04838624, -0.00013253, 1.30439695, -0.00183714,
                            34.39644051, 3034.74612775, 14.72847983, 0.21252668, 100.47390909,
                            0.20469106};
                case BodyId::Saturn:
                    return {9.53667594, -0.00125060, 0.05386179, -0.00050991, 2.48599187, 0.00193609,
                            49.95424423, 1222.49362201, 92.59887831, -0.41897216, 113.66242448,
                            -0.28867794};
                case BodyId::Uranus:
                    return {19.18916464, -0.00196176, 0.04725744, -0.00004397, 0.77263783, -0.00242939,
                            313.23810451, 428.48202785, 170.95427630, 0.40805281, 74.01692503,
                            0.04240589};
                case BodyId::Neptune:
                    return {30.06992276, 0.00026291, 0.00859048, 0.00005105, 1.77004347, 0.00035372,
                            -55.12002969, 218.45945325, 44.96476227, -0.32241464, 131.78422574,
                            -0.00508664};
                case BodyId::Pluto:
                    return {39.48211675, -0.00031596, 0.24882730, 0.00005170, 17.14001206, 0.00004818,
                            238.92903833, 145.20780515, 224.06891629, -0.04062942, 110.30393684,
                            -0.01183482};
                default:
                    return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            }
        }

        /**
         * @brief Heliocentric J2000 ecliptic position of a planet at a Julian Date.
         *
         * A convenience wrapping @ref element_at and @ref heliocentric_ecliptic_position;
         * meaningful only for the planets and Earth (the Sun sits at the origin).
         *
         * @param body Which planet to place.
         * @param julian_date The Julian Date.
         * @return Heliocentric ecliptic position, astronomical units.
         */
        inline Vector3 planet_heliocentric_au(BodyId body, double julian_date) noexcept
        {
            const double centuries = julian_centuries_since_j2000(julian_date);
            return heliocentric_ecliptic_position(element_at(keplerian_elements_for(body), centuries));
        }

        namespace detail
        {
            /** @brief Sine of an angle given in degrees. */
            inline double sin_degrees(double degrees) noexcept
            {
                return std::sin(degrees * DEGREES_TO_RADIANS);
            }

            /** @brief Cosine of an angle given in degrees. */
            inline double cos_degrees(double degrees) noexcept
            {
                return std::cos(degrees * DEGREES_TO_RADIANS);
            }
        } // namespace detail

        /**
         * @brief Geocentric J2000 ecliptic position of the Moon, in astronomical units.
         *
         * Evaluates a short lunar series (Schlyter's abridged theory: the two-body orbit
         * plus the dominant evection, variation, and yearly-equation perturbations),
         * accurate to a few arcminutes — ample for placing and phasing the lunar disk.
         * The result is geocentric (the Moon is not heliocentric like the planets), so
         * the ephemeris uses it directly rather than differencing against the Earth.
         *
         * @param julian_date The Julian Date.
         * @return Geocentric ecliptic position of the Moon, astronomical units.
         */
        inline Vector3 moon_geocentric_ecliptic_au(double julian_date) noexcept
        {
            using detail::cos_degrees;
            using detail::sin_degrees;

            // Schlyter's day number: days since 1999-12-31 0:00 UT (JD 2451543.5).
            const double d = julian_date - 2451543.5;

            // Moon orbital elements (degrees / Earth radii).
            const double node = 125.1228 - 0.0529538083 * d;
            const double inclination = 5.1454;
            const double argument = 318.0634 + 0.1643573223 * d;
            const double semi_major = 60.2666; // Earth radii
            const double eccentricity = 0.054900;
            double mean_anomaly = 115.3654 + 13.0649929509 * d;

            // Sun's elements, needed for the perturbation arguments.
            const double sun_perihelion = 282.9404 + 4.70935e-5 * d;
            const double sun_mean_anomaly = 356.0470 + 0.9856002585 * d;

            // Eccentric anomaly by iteration (degrees form).
            double eccentric = mean_anomaly +
                               eccentricity * RADIANS_TO_DEGREES * sin_degrees(mean_anomaly) *
                                   (1.0 + eccentricity * cos_degrees(mean_anomaly));
            for (int i = 0; i < 6; ++i)
            {
                const double delta =
                    (eccentric - eccentricity * RADIANS_TO_DEGREES * sin_degrees(eccentric) -
                     mean_anomaly) /
                    (1.0 - eccentricity * cos_degrees(eccentric));
                eccentric -= delta;
            }

            // Position in the orbital plane, Earth radii.
            const double x_plane = semi_major * (cos_degrees(eccentric) - eccentricity);
            const double y_plane =
                semi_major * std::sqrt(1.0 - eccentricity * eccentricity) * sin_degrees(eccentric);
            const double true_anomaly = std::atan2(y_plane, x_plane) * RADIANS_TO_DEGREES;
            const double radius = std::sqrt(x_plane * x_plane + y_plane * y_plane);

            // Rotate into geocentric ecliptic coordinates (Earth radii).
            const double lon_orbit = true_anomaly + argument;
            double xh = radius * (cos_degrees(node) * cos_degrees(lon_orbit) -
                                  sin_degrees(node) * sin_degrees(lon_orbit) *
                                      cos_degrees(inclination));
            double yh = radius * (sin_degrees(node) * cos_degrees(lon_orbit) +
                                  cos_degrees(node) * sin_degrees(lon_orbit) *
                                      cos_degrees(inclination));
            double zh = radius * (sin_degrees(lon_orbit) * sin_degrees(inclination));

            double longitude = std::atan2(yh, xh) * RADIANS_TO_DEGREES;
            double latitude =
                std::atan2(zh, std::sqrt(xh * xh + yh * yh)) * RADIANS_TO_DEGREES;

            // Perturbation arguments.
            const double moon_mean_longitude = node + argument + mean_anomaly;
            const double sun_mean_longitude = sun_perihelion + sun_mean_anomaly;
            const double elongation = moon_mean_longitude - sun_mean_longitude;
            const double argument_latitude = moon_mean_longitude - node;

            longitude += -1.274 * sin_degrees(mean_anomaly - 2.0 * elongation);
            longitude += 0.658 * sin_degrees(2.0 * elongation);
            longitude += -0.186 * sin_degrees(sun_mean_anomaly);
            longitude += -0.059 * sin_degrees(2.0 * mean_anomaly - 2.0 * elongation);
            longitude += -0.057 * sin_degrees(mean_anomaly - 2.0 * elongation + sun_mean_anomaly);
            longitude += 0.053 * sin_degrees(mean_anomaly + 2.0 * elongation);
            longitude += 0.046 * sin_degrees(2.0 * elongation - sun_mean_anomaly);
            longitude += 0.041 * sin_degrees(mean_anomaly - sun_mean_anomaly);
            longitude += -0.035 * sin_degrees(elongation);
            longitude += -0.031 * sin_degrees(mean_anomaly + sun_mean_anomaly);
            longitude += -0.015 * sin_degrees(2.0 * argument_latitude - 2.0 * elongation);
            longitude += 0.011 * sin_degrees(mean_anomaly - 4.0 * elongation);

            latitude += -0.173 * sin_degrees(argument_latitude - 2.0 * elongation);
            latitude += -0.055 * sin_degrees(mean_anomaly - argument_latitude - 2.0 * elongation);
            latitude += -0.046 * sin_degrees(mean_anomaly + argument_latitude - 2.0 * elongation);
            latitude += 0.033 * sin_degrees(argument_latitude + 2.0 * elongation);
            latitude += 0.017 * sin_degrees(2.0 * mean_anomaly + argument_latitude);

            double distance = radius; // Earth radii
            distance += -0.58 * cos_degrees(mean_anomaly - 2.0 * elongation);
            distance += -0.46 * cos_degrees(2.0 * elongation);

            // Rebuild the geocentric ecliptic vector from the perturbed spherical
            // coordinates, then convert Earth radii to astronomical units.
            const double earth_radius_metres = 6378137.0;
            const double earth_radii_to_au = earth_radius_metres / METRES_PER_ASTRONOMICAL_UNIT;
            const double cos_lat = cos_degrees(latitude);
            return Vector3{distance * cos_lat * cos_degrees(longitude) * earth_radii_to_au,
                           distance * cos_lat * sin_degrees(longitude) * earth_radii_to_au,
                           distance * sin_degrees(latitude) * earth_radii_to_au};
        }
    } // namespace Astro
} // namespace SushiEngine
