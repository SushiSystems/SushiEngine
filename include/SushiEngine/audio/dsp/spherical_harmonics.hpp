/**************************************************************************/
/* spherical_harmonics.hpp                                               */
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

#ifndef SUSHIENGINE_AUDIO_DSP_SPHERICAL_HARMONICS_HPP
#define SUSHIENGINE_AUDIO_DSP_SPHERICAL_HARMONICS_HPP

/**
 * @file spherical_harmonics.hpp
 * @brief Real spherical harmonics — the encode gains of the ambisonic scene bus.
 *
 * A source is placed into the ambisonic field by scaling it with the spherical
 * harmonic values evaluated at its direction: `(order + 1)²` gains, one per channel
 * (see `docs/design/audio_system.md` §3.8, §4). This is the single most important
 * scaling decision in the spatializer — the per-source cost is only these gains, so N
 * sources collapse into one fixed bus and the expensive binaural decode runs once.
 *
 * Convention is **AmbiX**: **ACN** channel ordering (`index = ℓ² + ℓ + m`) and **SN3D**
 * (Schmidt semi-normalised) scaling, so the 0th channel (W) is a constant 1 and the
 * three first-order channels are exactly the direction's `y, z, x` components. The
 * coordinate frame is right-handed with **x = front, y = left, z = up**. The gains are
 * computed from the general associated-Legendre recurrence, valid for any order.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /** @brief The number of ambisonic channels for an order: `(order + 1)²`. */
            constexpr int ambisonic_channel_count(int order) noexcept
            {
                return (order + 1) * (order + 1);
            }

            /** @brief The maximum ambisonic order this header evaluates. */
            constexpr int MAX_AMBISONIC_ORDER = 7;

            /** @brief The channel count at @ref MAX_AMBISONIC_ORDER, for fixed-size scratch. */
            constexpr int MAX_AMBISONIC_CHANNELS = ambisonic_channel_count(MAX_AMBISONIC_ORDER);

            /**
             * @brief Evaluates the real (ACN/SN3D) spherical harmonics at a direction.
             *
             * Writes `(order + 1)²` gains, one per ACN channel, for a source in the
             * given direction. The direction need not be normalised. With SN3D the W
             * channel is 1 and the first-order channels are the unit direction's
             * `y, z, x` — the property the binaural decode relies on.
             *
             * @param order The ambisonic order (0..@ref MAX_AMBISONIC_ORDER).
             * @param x     Direction front component.
             * @param y     Direction left component.
             * @param z     Direction up component.
             * @param gains Output array of at least `(order + 1)²` gains (ACN order).
             */
            inline void ambisonic_encode_gains(int order, float x, float y, float z,
                                               float* gains) noexcept
            {
                if (order < 0)
                    order = 0;
                if (order > MAX_AMBISONIC_ORDER)
                    order = MAX_AMBISONIC_ORDER;

                // Normalise the direction; a zero vector encodes as straight ahead.
                double length = std::sqrt(static_cast<double>(x) * x +
                                          static_cast<double>(y) * y +
                                          static_cast<double>(z) * z);
                double dx = 1.0, dy = 0.0, dz = 0.0;
                if (length > 1e-9)
                {
                    dx = x / length;
                    dy = y / length;
                    dz = z / length;
                }

                const double sin_elevation = dz;                        // sin θ
                double cos_elevation = std::sqrt(1.0 - dz * dz);        // cos θ ≥ 0
                if (cos_elevation < 0.0)
                    cos_elevation = 0.0;
                const double azimuth = std::atan2(dy, dx);              // φ

                // Associated Legendre P_l^m(sin θ) without the Condon-Shortley phase,
                // built by the standard three-term recurrence. legendre[l][m].
                double legendre[MAX_AMBISONIC_ORDER + 1][MAX_AMBISONIC_ORDER + 1] = {{0.0}};
                legendre[0][0] = 1.0;
                for (int m = 1; m <= order; ++m)
                {
                    // P_m^m = (2m-1)!! · (cos θ)^m.
                    legendre[m][m] = legendre[m - 1][m - 1] * (2.0 * m - 1.0) * cos_elevation;
                }
                for (int m = 0; m <= order; ++m)
                {
                    if (m + 1 <= order)
                        legendre[m + 1][m] = sin_elevation * (2.0 * m + 1.0) * legendre[m][m];
                    for (int l = m + 2; l <= order; ++l)
                    {
                        legendre[l][m] = ((2.0 * l - 1.0) * sin_elevation * legendre[l - 1][m] -
                                          (l + m - 1.0) * legendre[l - 2][m]) /
                                         (l - m);
                    }
                }

                for (int l = 0; l <= order; ++l)
                {
                    for (int m = -l; m <= l; ++m)
                    {
                        const int abs_m = m < 0 ? -m : m;
                        // SN3D normalisation N_l^m = sqrt((2 - δ_{m0}) (l-|m|)! / (l+|m|)!).
                        double numerator = 1.0;   // (l - |m|)!
                        for (int k = 2; k <= l - abs_m; ++k)
                            numerator *= k;
                        double denominator = 1.0; // (l + |m|)!
                        for (int k = 2; k <= l + abs_m; ++k)
                            denominator *= k;
                        const double delta = (m == 0) ? 1.0 : 2.0;
                        const double norm = std::sqrt(delta * numerator / denominator);

                        const double trig = (m < 0) ? std::sin(abs_m * azimuth)
                                                    : std::cos(abs_m * azimuth);
                        gains[l * l + l + m] =
                            static_cast<float>(norm * legendre[l][abs_m] * trig);
                    }
                }
            }
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
