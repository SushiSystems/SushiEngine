/**************************************************************************/
/* fft.hpp                                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_DSP_FFT_HPP
#define SUSHIENGINE_AUDIO_DSP_FFT_HPP

/**
 * @file fft.hpp
 * @brief The FFT seam (`IFourierTransform`) and a from-scratch radix-2 implementation.
 *
 * Fast convolution — the partitioned convolution reverb of §S10 / §3.5 — needs a forward
 * and inverse complex FFT. The `IFourierTransform` seam (§13 of `docs/slop/audio_system.md`)
 * isolates it so a tuned vendor FFT can drop in later without touching the convolver; @ref RadixFFT
 * is the portable in-house implementation used today: an in-place iterative radix-2
 * Cooley–Tukey with a precomputed bit-reversal table and twiddle factors, so a transform
 * is allocation-free after @ref prepare. Sizes must be a power of two.
 *
 * Portable `float` DSP (twiddles computed in `double`), no SDL and no SushiRuntime.
 */

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        namespace DSP
        {
            /** @brief A power-of-two complex FFT: forward and (normalised) inverse, in place. */
            class IFourierTransform
            {
                public:
                    virtual ~IFourierTransform() = default;

                    /** @brief Sizes the transform (a power of two); allocates its tables. */
                    virtual void prepare(int size) = 0;

                    /** @brief The transform size. */
                    virtual int size() const noexcept = 0;

                    /** @brief In-place forward FFT of @p data (length @ref size). */
                    virtual void forward(std::complex<float>* data) noexcept = 0;

                    /** @brief In-place inverse FFT of @p data, scaled by 1/size. */
                    virtual void inverse(std::complex<float>* data) noexcept = 0;
            };

            /** @brief An in-place iterative radix-2 Cooley–Tukey FFT. */
            class RadixFFT final : public IFourierTransform
            {
                public:
                    void prepare(int size) override
                    {
                        size_ = size;
                        reversal_.resize(static_cast<std::size_t>(size));
                        int bits = 0;
                        while ((1 << bits) < size)
                            ++bits;
                        for (int i = 0; i < size; ++i)
                        {
                            int r = 0;
                            for (int b = 0; b < bits; ++b)
                                if (i & (1 << b))
                                    r |= 1 << (bits - 1 - b);
                            reversal_[static_cast<std::size_t>(i)] = r;
                        }
                        // Twiddles for the forward transform: W_N^k = e^{-2πi k / N}, for the
                        // largest stage; smaller stages index into it by stride.
                        twiddles_.resize(static_cast<std::size_t>(size / 2));
                        for (int k = 0; k < size / 2; ++k)
                        {
                            const double angle = -2.0 * 3.14159265358979 * k / size;
                            twiddles_[static_cast<std::size_t>(k)] =
                                std::complex<float>(static_cast<float>(std::cos(angle)),
                                                    static_cast<float>(std::sin(angle)));
                        }
                    }

                    int size() const noexcept override { return size_; }

                    void forward(std::complex<float>* data) noexcept override { transform(data, false); }

                    void inverse(std::complex<float>* data) noexcept override
                    {
                        transform(data, true);
                        const float inv = 1.0f / static_cast<float>(size_);
                        for (int i = 0; i < size_; ++i)
                            data[i] *= inv;
                    }

                private:
                    void transform(std::complex<float>* data, bool invert) noexcept
                    {
                        // Bit-reversal permutation.
                        for (int i = 0; i < size_; ++i)
                        {
                            const int r = reversal_[static_cast<std::size_t>(i)];
                            if (r > i)
                            {
                                const std::complex<float> t = data[i];
                                data[i] = data[r];
                                data[r] = t;
                            }
                        }
                        // Butterfly stages.
                        for (int length = 2; length <= size_; length <<= 1)
                        {
                            const int half = length >> 1;
                            const int stride = size_ / length;
                            for (int i = 0; i < size_; i += length)
                            {
                                int k = 0;
                                for (int j = 0; j < half; ++j)
                                {
                                    std::complex<float> w = twiddles_[static_cast<std::size_t>(k)];
                                    if (invert)
                                        w = std::conj(w);
                                    const std::complex<float> u = data[i + j];
                                    const std::complex<float> v = data[i + j + half] * w;
                                    data[i + j] = u + v;
                                    data[i + j + half] = u - v;
                                    k += stride;
                                }
                            }
                        }
                    }

                    std::vector<int> reversal_;
                    std::vector<std::complex<float>> twiddles_;
                    int size_ = 0;
            };
        } // namespace DSP
    } // namespace Audio
} // namespace SushiEngine

#endif
