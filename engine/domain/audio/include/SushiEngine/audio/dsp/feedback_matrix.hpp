/**************************************************************************/
/* feedback_matrix.hpp                                                    */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FEEDBACK_MATRIX_HPP
#define SUSHIENGINE_AUDIO_DSP_FEEDBACK_MATRIX_HPP

/**
 * @file feedback_matrix.hpp
 * @brief The lossless mixing matrices at the heart of a feedback delay network.
 *
 * An FDN's late field is built by feeding N delay lines back into each other through a
 * single **energy-preserving (orthogonal) matrix**: every echo is scattered into every
 * line, so the echo density grows geometrically toward a natural-sounding diffuse tail,
 * and — because the matrix is lossless — the *only* thing that sets the decay is the
 * per-line damping (§3.7 of `docs/design/audio_system.md`). Decouple mixing from decay
 * and the reverb time becomes a clean, per-band knob.
 *
 * Two matrices, both applied **in place** to the N-vector of (already damped) delay
 * outputs, both `noexcept` and allocation-free:
 *
 *   - **Householder** `H = I − (2/N)·1·1ᵀ`. Symmetric and orthogonal, so `‖Hx‖ = ‖x‖`.
 *     Every entry off the diagonal is non-zero → maximum scattering / echo density.
 *     Costs one sum and one subtract per element: `O(N)`, no per-pair multiplies.
 *   - **Hadamard** (normalized Walsh–Hadamard). Orthonormal for `N = 2^k`; the
 *     fast in-place butterfly is `O(N·log N)` adds with a single `1/√N` scale — no
 *     multiplies in the butterfly, the reason it is the classic choice for power-of-two
 *     line counts.
 *
 * Losslessness is the property the whole FDN's stability rests on: with an orthogonal
 * mixing matrix and strictly-contractive per-line damping filters, the network's poles
 * stay inside the unit circle for any delay lengths.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace DSP
        {
            /** @brief Which lossless mixing matrix an FDN scatters its lines through. */
            enum class FeedbackMatrix
            {
                Householder, ///< `I − (2/N)·1·1ᵀ`: max echo density, `O(N)`, any N.
                Hadamard     ///< Normalized Walsh–Hadamard: `O(N·log N)` adds, needs N = 2^k.
            };

            /**
             * @brief Applies the Householder reflection `H = I − (2/N)·1·1ᵀ` in place.
             *
             * `y_i = x_i − (2/N)·Σx`. Symmetric and orthogonal for any N ≥ 1, so it
             * preserves the vector's energy exactly (a reflection, `det = −1`).
             *
             * @param v The N-vector, overwritten with `Hv`.
             * @param n The vector length N.
             */
            inline void apply_householder(float* v, int n) noexcept
            {
                if (n <= 0)
                    return;
                float sum = 0.0f;
                for (int i = 0; i < n; ++i)
                    sum += v[i];
                const float k = 2.0f * sum / static_cast<float>(n);
                for (int i = 0; i < n; ++i)
                    v[i] -= k;
            }

            /**
             * @brief Applies the normalized Walsh–Hadamard transform in place.
             *
             * The in-place butterfly computes the (unnormalized) transform, whose action
             * scales the norm by `√N`; the final `1/√N` pass makes it orthonormal, so
             * `‖Hv‖ = ‖v‖`. @p n **must** be a power of two (the FDN uses N = 16); a
             * non-power-of-two produces a partial, non-lossless transform.
             *
             * @param v The N-vector, overwritten with `Hv`.
             * @param n The vector length N (a power of two).
             */
            inline void apply_hadamard(float* v, int n) noexcept
            {
                if (n <= 0)
                    return;
                for (int length = 1; length < n; length <<= 1)
                {
                    for (int i = 0; i < n; i += (length << 1))
                    {
                        for (int j = i; j < i + length; ++j)
                        {
                            const float a = v[j];
                            const float b = v[j + length];
                            v[j] = a + b;
                            v[j + length] = a - b;
                        }
                    }
                }
                const float inv_norm = 1.0f / std::sqrt(static_cast<float>(n));
                for (int i = 0; i < n; ++i)
                    v[i] *= inv_norm;
            }

            /**
             * @brief Applies the selected lossless mixing matrix in place.
             * @param type The matrix to use.
             * @param v    The N-vector, overwritten in place.
             * @param n    The vector length (a power of two if @p type is Hadamard).
             */
            inline void apply_feedback_matrix(FeedbackMatrix type, float* v, int n) noexcept
            {
                if (type == FeedbackMatrix::Hadamard)
                    apply_hadamard(v, n);
                else
                    apply_householder(v, n);
            }
        } // namespace DSP
    } // namespace Audio
} // namespace SushiEngine

#endif
