/**************************************************************************/
/* fourier_transform.hpp                                                  */
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
 * @file fourier_transform.hpp
 * @brief A power-of-two discrete Fourier transform, and the pairing that halves it.
 *
 * The global core inverts its elliptic operator by transforming in longitude and solving a
 * tridiagonal system in latitude (`docs/design/atmosphere_system.md` §5), so it needs a
 * transform on every row of the grid, several times a step. This is that transform:
 * iterative radix-2 Cooley-Tukey with a precomputed bit-reversal permutation and a
 * precomputed twiddle table, so a step allocates nothing and computes no trigonometry.
 *
 * **Everything transformed here is real, and they come in twos.** The core's two fields are
 * the barotropic and baroclinic combinations of its layers, and a complex transform of a
 * real row wastes half its work. Riding one field on the real part and the other on the
 * imaginary part of a single transform recovers that half exactly — the two spectra separate
 * afterwards by Hermitian symmetry, which @ref FourierTransform::forward_real_pair does. It
 * is not an approximation and it costs ten lines.
 *
 * It lives in the atmosphere module because the atmosphere is what needs it. A second
 * consumer is what would earn it a neutral home of its own, and there is not one yet.
 */

#include <complex>
#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Atmosphere
    {
        /**
         * @brief A fixed-length radix-2 transform, reusable across rows and across steps.
         *
         * Const throughout after construction, so one instance serves every row of a grid
         * and can be shared between threads. Every entry point takes the buffers it works
         * on; the object itself holds only the tables.
         */
        class FourierTransform
        {
            public:
                /**
                 * @brief Builds the permutation and twiddle tables for transforms of @p length.
                 *
                 * @param length Points per transform. Must be a power of two and at least 2;
                 *               anything else leaves the object @ref valid() == false, which is
                 *               how a caller that sized a grid badly finds out.
                 */
                explicit FourierTransform(int length);

                /** @brief Whether construction was given a usable length. */
                bool valid() const noexcept { return length_ >= 2; }

                /** @brief Points per transform. */
                int length() const noexcept { return length_; }

                /**
                 * @brief Transforms @p data in place, unnormalized.
                 *
                 * Sign convention `X_m = sum_i x_i exp(-2*pi*i*m*i/N)` — the one that makes
                 * `d/dlambda` a multiplication by `+i*m`.
                 *
                 * @param data Exactly @ref length() elements, overwritten with the spectrum.
                 */
                void forward(std::complex<double>* data) const noexcept;

                /**
                 * @brief Transforms @p data in place and divides by @ref length().
                 *
                 * Normalized here rather than at the call site so that `inverse(forward(x))`
                 * is the identity and no caller has to remember which side carries the scale.
                 *
                 * @param data Exactly @ref length() elements, overwritten with the samples.
                 */
                void inverse(std::complex<double>* data) const noexcept;

                /**
                 * @brief Transforms two real rows for the price of one complex transform.
                 *
                 * @p first rides the real part and @p second the imaginary part of a single
                 * transform; the two spectra are then separated by the Hermitian symmetry each
                 * real signal's spectrum has. Both spectra are written in full length, not
                 * folded to a half-spectrum, because the caller indexes them by wavenumber and
                 * a fold would put a branch in that inner loop.
                 *
                 * @param first          @ref length() real samples.
                 * @param second         @ref length() real samples.
                 * @param first_spectrum Receives @p first's spectrum; @ref length() elements.
                 * @param second_spectrum Receives @p second's spectrum; @ref length() elements.
                 *                        May not alias @p first_spectrum.
                 */
                void forward_real_pair(const double* first, const double* second,
                                       std::complex<double>* first_spectrum,
                                       std::complex<double>* second_spectrum) const noexcept;

                /**
                 * @brief The inverse of @ref forward_real_pair, back to two real rows.
                 *
                 * Only the real part of each recovered signal is written. A spectrum that is
                 * not Hermitian-symmetric therefore loses its imaginary residue silently, which
                 * is correct for this caller: the operators applied between the two transforms
                 * are real, so the symmetry is preserved by construction and the residue is
                 * rounding.
                 *
                 * @param first_spectrum  @ref length() elements.
                 * @param second_spectrum @ref length() elements.
                 * @param first           Receives @ref length() real samples.
                 * @param second          Receives @ref length() real samples.
                 * @param work            Scratch of @ref length() elements, supplied by the
                 *                        caller so a step allocates nothing.
                 */
                void inverse_real_pair(const std::complex<double>* first_spectrum,
                                       const std::complex<double>* second_spectrum, double* first,
                                       double* second, std::complex<double>* work) const noexcept;

            private:
                /** @brief Shared body of @ref forward and @ref inverse; @p sign is -1 or +1. */
                void transform(std::complex<double>* data, double sign) const noexcept;

                int length_ = 0;
                std::vector<int> reversed_;                  /**< Bit-reversal permutation. */
                std::vector<std::complex<double>> twiddles_; /**< exp(-2*pi*i*k/N), k < N/2. */
        };
    } // namespace Atmosphere
} // namespace SushiEngine
