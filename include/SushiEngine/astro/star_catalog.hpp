/**************************************************************************/
/* star_catalog.hpp                                                       */
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
 * @file star_catalog.hpp
 * @brief The brightest fixed stars, as real J2000 positions, magnitudes, and colours.
 *
 * A compact embedded catalogue of the ~60 brightest stars — enough to render the
 * recognisable constellations (Orion, the Big Dipper, Crux, Cassiopeia, Leo, Scorpius,
 * ...) as their true patterns rather than a random hash. Each entry is a J2000
 * equatorial position (right ascension / declination), an apparent magnitude that sets
 * its brightness, and a B-V colour index turned into an RGB tint. The topocentric
 * transform in the ephemeris rotates these fixed directions into the observer's sky,
 * so the constellations rise and set with sidereal time.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Astro
    {
        /**
         * @brief One catalogued star: where it is, how bright, and what colour.
         *
         * @c right_ascension_degrees / @c declination_degrees are J2000 equatorial;
         * @c magnitude is apparent visual magnitude (smaller is brighter);
         * @c color_index is Johnson B-V, mapped to a display colour by @ref bv_to_rgb.
         */
        struct BrightStar
        {
            double right_ascension_degrees; /**< J2000 right ascension, degrees [0, 360). */
            double declination_degrees;     /**< J2000 declination, degrees [-90, 90]. */
            float magnitude;                /**< Apparent visual magnitude. */
            float color_index;              /**< Johnson B-V colour index. */
        };

        /**
         * @brief The embedded bright-star catalogue (J2000), brightest first.
         *
         * Roughly the stars down to magnitude 2.3 — the set that draws the classical
         * constellation figures.
         */
        inline constexpr BrightStar BRIGHT_STARS[] = {
            {101.2750, -16.7167, -1.46f, 0.00f},  // Sirius
            {96.0000, -52.7000, -0.74f, 0.15f},   // Canopus
            {219.9000, -60.8333, -0.27f, 0.71f},  // Rigil Kentaurus
            {213.9250, 19.1833, -0.05f, 1.23f},   // Arcturus
            {279.2250, 38.7833, 0.03f, 0.00f},    // Vega
            {79.1750, 46.0000, 0.08f, 0.80f},     // Capella
            {78.6250, -8.2000, 0.13f, -0.03f},    // Rigel
            {114.8250, 5.2167, 0.34f, 0.42f},     // Procyon
            {88.8000, 7.4000, 0.50f, 1.85f},      // Betelgeuse
            {24.4250, -57.2333, 0.46f, -0.16f},   // Achernar
            {210.9500, -60.3667, 0.61f, -0.23f},  // Hadar
            {297.7000, 8.8667, 0.77f, 0.22f},     // Altair
            {186.6500, -63.1000, 0.76f, -0.24f},  // Acrux
            {68.9750, 16.5167, 0.85f, 1.54f},     // Aldebaran
            {247.3500, -26.4333, 0.96f, 1.83f},   // Antares
            {201.3000, -11.1667, 1.04f, -0.23f},  // Spica
            {116.3250, 28.0333, 1.14f, 1.00f},    // Pollux
            {344.4000, -29.6167, 1.16f, 0.09f},   // Fomalhaut
            {310.3500, 45.2833, 1.25f, 0.09f},    // Deneb
            {191.9250, -59.6833, 1.25f, -0.23f},  // Mimosa
            {152.1000, 11.9667, 1.35f, -0.11f},   // Regulus
            {104.6500, -28.9667, 1.50f, -0.21f},  // Adhara
            {113.6500, 31.8833, 1.57f, 0.03f},    // Castor
            {187.8000, -57.1167, 1.63f, 1.60f},   // Gacrux
            {263.4000, -37.1000, 1.62f, -0.23f},  // Shaula
            {81.2750, 6.3500, 1.64f, -0.22f},     // Bellatrix
            {81.5750, 28.6000, 1.65f, -0.13f},    // Elnath
            {138.3000, -69.7167, 1.68f, 0.00f},   // Miaplacidus
            {84.0500, -1.2000, 1.69f, -0.18f},    // Alnilam
            {85.2000, -1.9500, 1.77f, -0.21f},    // Alnitak
            {193.5000, 55.9667, 1.77f, -0.02f},   // Alioth
            {165.9250, 61.7500, 1.79f, 1.07f},    // Dubhe
            {51.0750, 49.8667, 1.79f, 0.48f},     // Mirfak
            {107.1000, -26.4000, 1.83f, 0.68f},   // Wezen
            {276.0500, -34.3833, 1.85f, -0.03f},  // Kaus Australis
            {206.8750, 49.3167, 1.86f, -0.10f},   // Alkaid
            {264.3250, -43.0000, 1.86f, 0.40f},   // Sargas
            {125.6250, -59.5167, 1.86f, 1.28f},   // Avior
            {89.8750, 44.9500, 1.90f, 0.03f},     // Menkalinan
            {99.4250, 16.4000, 1.93f, 0.00f},     // Alhena
            {306.4000, -56.7333, 1.94f, -0.12f},  // Peacock
            {95.6750, -17.9500, 1.98f, -0.24f},   // Mirzam
            {141.9000, -8.6667, 1.98f, 1.44f},    // Alphard
            {37.9500, 89.2667, 1.98f, 0.60f},     // Polaris
            {155.0000, 19.8500, 2.01f, 1.12f},    // Algieba
            {31.8000, 23.4667, 2.00f, 1.15f},     // Hamal
            {10.9000, -17.9833, 2.04f, 1.02f},    // Diphda
            {283.8250, -26.3000, 2.05f, -0.13f},  // Nunki
            {211.6750, -36.3667, 2.06f, 1.01f},   // Menkent
            {17.4250, 35.6167, 2.07f, 1.58f},     // Mirach
            {2.1000, 29.0833, 2.06f, -0.11f},     // Alpheratz
            {263.7250, 12.5667, 2.08f, 0.16f},    // Rasalhague
            {222.6750, 74.1500, 2.08f, 1.47f},    // Kochab
            {86.9500, -9.6667, 2.06f, -0.17f},    // Saiph
            {177.2750, 14.5667, 2.11f, 0.09f},    // Denebola
            {47.0500, 40.9500, 2.12f, -0.05f},    // Algol
            {190.3750, -48.9667, 2.20f, -0.01f},  // Muhlifain
            {233.6750, 26.7167, 2.22f, -0.02f},   // Alphecca
            {83.0000, -0.3000, 2.23f, -0.18f},    // Mintaka
            {305.5500, 40.2500, 2.23f, 0.68f},    // Sadr
            {10.1250, 56.5333, 2.24f, 1.17f},     // Schedar
            {2.3000, 59.1500, 2.28f, 0.34f},      // Caph
        };

        /** @brief Number of stars in @ref BRIGHT_STARS. */
        inline constexpr std::size_t BRIGHT_STAR_COUNT =
            sizeof(BRIGHT_STARS) / sizeof(BRIGHT_STARS[0]);

        /**
         * @brief The J2000 equatorial unit direction to a star.
         * @param star The catalogue entry.
         * @return Unit vector (cosDec cosRA, cosDec sinRA, sinDec) in the equatorial frame.
         */
        inline Vector3 star_equatorial_direction(const BrightStar& star) noexcept
        {
            const double ra = star.right_ascension_degrees * 0.017453292519943295;
            const double dec = star.declination_degrees * 0.017453292519943295;
            const double cos_dec = std::cos(dec);
            return Vector3{cos_dec * std::cos(ra), cos_dec * std::sin(ra), std::sin(dec)};
        }

        /**
         * @brief Maps a B-V colour index to a linear RGB tint.
         *
         * Converts B-V to an effective temperature (Ballesteros' formula) and that to an
         * RGB colour with a compact blackbody approximation, so hot blue-white stars and
         * cool orange-red stars read distinctly. The result is normalised so its largest
         * channel is 1; the renderer applies brightness separately from magnitude.
         *
         * @param color_index The Johnson B-V index (roughly [-0.4, 2.0]).
         * @return A linear RGB tint with peak channel 1.
         */
        inline Vector3 bv_to_rgb(float color_index) noexcept
        {
            const double bv = color_index < -0.4f ? -0.4 : (color_index > 2.0f ? 2.0 : color_index);
            const double temperature =
                4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62));
            const double t = temperature / 100.0;

            double red;
            double green;
            double blue;
            if (t <= 66.0)
            {
                red = 255.0;
                green = 99.4708025861 * std::log(t) - 161.1195681661;
            }
            else
            {
                red = 329.698727446 * std::pow(t - 60.0, -0.1332047592);
                green = 288.1221695283 * std::pow(t - 60.0, -0.0755148492);
            }
            if (t >= 66.0)
                blue = 255.0;
            else if (t <= 19.0)
                blue = 0.0;
            else
                blue = 138.5177312231 * std::log(t - 10.0) - 305.0447927307;

            const auto clamp255 = [](double v) -> double
            {
                return v < 0.0 ? 0.0 : (v > 255.0 ? 255.0 : v);
            };
            Vector3 rgb{clamp255(red) / 255.0, clamp255(green) / 255.0, clamp255(blue) / 255.0};
            const double peak = std::fmax(rgb.x, std::fmax(rgb.y, rgb.z));
            if (peak > 0.0)
                rgb = rgb * (1.0 / peak);
            return rgb;
        }

        /**
         * @brief Relative linear brightness of a star from its apparent magnitude.
         *
         * Pogson's ratio: each magnitude is 2.512x in flux, referenced so magnitude 0
         * returns 1. Fainter (larger-magnitude) stars fall off geometrically.
         *
         * @param magnitude Apparent visual magnitude.
         * @return Linear brightness relative to a magnitude-0 star.
         */
        inline float magnitude_to_brightness(float magnitude) noexcept
        {
            return static_cast<float>(std::pow(10.0, -0.4 * static_cast<double>(magnitude)));
        }
    } // namespace Astro
} // namespace SushiEngine
