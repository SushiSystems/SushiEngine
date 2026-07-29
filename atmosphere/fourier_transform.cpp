/**************************************************************************/
/* fourier_transform.cpp                                                  */
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

#include <SushiEngine/atmosphere/fourier_transform.hpp>

#include <cmath>

namespace SushiEngine
{
    namespace Atmosphere
    {
        namespace
        {
            constexpr double PI = 3.14159265358979323846;

            bool power_of_two(int value) noexcept
            {
                return value >= 2 && (value & (value - 1)) == 0;
            }
        } // namespace

        FourierTransform::FourierTransform(int length)
        {
            if (!power_of_two(length))
                return;

            length_ = length;

            // The permutation is precomputed rather than derived per element, because the
            // transform runs once per grid row and the reversal is otherwise the only part of
            // it that costs a loop per element.
            reversed_.resize(static_cast<std::size_t>(length_));
            int bits = 0;
            while ((1 << bits) < length_)
                ++bits;
            for (int i = 0; i < length_; ++i)
            {
                int value = 0;
                for (int bit = 0; bit < bits; ++bit)
                    if ((i >> bit) & 1)
                        value |= 1 << (bits - 1 - bit);
                reversed_[static_cast<std::size_t>(i)] = value;
            }

            twiddles_.resize(static_cast<std::size_t>(length_ / 2));
            for (int k = 0; k < length_ / 2; ++k)
            {
                const double angle = -2.0 * PI * double(k) / double(length_);
                twiddles_[static_cast<std::size_t>(k)] =
                    std::complex<double>(std::cos(angle), std::sin(angle));
            }
        }

        void FourierTransform::transform(std::complex<double>* data, double sign) const noexcept
        {
            for (int i = 0; i < length_; ++i)
            {
                const int j = reversed_[static_cast<std::size_t>(i)];
                if (j > i)
                {
                    const std::complex<double> temporary = data[i];
                    data[i] = data[j];
                    data[j] = temporary;
                }
            }

            for (int span = 2; span <= length_; span <<= 1)
            {
                const int half = span / 2;
                const int stride = length_ / span;
                for (int base = 0; base < length_; base += span)
                    for (int k = 0; k < half; ++k)
                    {
                        std::complex<double> twiddle = twiddles_[static_cast<std::size_t>(k * stride)];
                        // One table serves both directions: the inverse kernel is the forward
                        // kernel's conjugate, so a sign flip on the imaginary part is the whole
                        // difference and there is no second table to keep in step.
                        twiddle.imag(sign * twiddle.imag());
                        const std::complex<double> upper = data[base + k];
                        const std::complex<double> lower = data[base + k + half] * twiddle;
                        data[base + k] = upper + lower;
                        data[base + k + half] = upper - lower;
                    }
            }
        }

        void FourierTransform::forward(std::complex<double>* data) const noexcept
        {
            if (!valid())
                return;
            transform(data, 1.0);
        }

        void FourierTransform::inverse(std::complex<double>* data) const noexcept
        {
            if (!valid())
                return;
            transform(data, -1.0);
            const double scale = 1.0 / double(length_);
            for (int i = 0; i < length_; ++i)
                data[i] *= scale;
        }

        void FourierTransform::forward_real_pair(const double* first, const double* second,
                                                 std::complex<double>* first_spectrum,
                                                 std::complex<double>* second_spectrum) const noexcept
        {
            if (!valid())
                return;

            for (int i = 0; i < length_; ++i)
                first_spectrum[i] = std::complex<double>(first[i], second[i]);
            transform(first_spectrum, 1.0);

            // Separate the two spectra by Hermitian symmetry. Both members of a (m, N-m) pair
            // are read before either is written, which is what lets the combined transform stay
            // in `first_spectrum` instead of needing a buffer of its own.
            for (int m = 0; m <= length_ / 2; ++m)
            {
                const int mirror = (length_ - m) % length_;
                const std::complex<double> forward_mode = first_spectrum[m];
                const std::complex<double> mirror_mode = std::conj(first_spectrum[mirror]);

                const std::complex<double> a = 0.5 * (forward_mode + mirror_mode);
                const std::complex<double> b =
                    std::complex<double>(0.0, -0.5) * (forward_mode - mirror_mode);

                first_spectrum[m] = a;
                first_spectrum[mirror] = std::conj(a);
                second_spectrum[m] = b;
                second_spectrum[mirror] = std::conj(b);
            }
        }

        void FourierTransform::inverse_real_pair(const std::complex<double>* first_spectrum,
                                                 const std::complex<double>* second_spectrum,
                                                 double* first, double* second,
                                                 std::complex<double>* work) const noexcept
        {
            if (!valid())
                return;

            for (int m = 0; m < length_; ++m)
                work[m] = first_spectrum[m] + std::complex<double>(0.0, 1.0) * second_spectrum[m];
            inverse(work);

            for (int i = 0; i < length_; ++i)
            {
                first[i] = work[i].real();
                second[i] = work[i].imag();
            }
        }
    } // namespace Atmosphere
} // namespace SushiEngine
