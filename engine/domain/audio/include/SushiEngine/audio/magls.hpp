/**************************************************************************/
/* magls.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_MAGLS_HPP
#define SUSHIENGINE_AUDIO_MAGLS_HPP

/**
 * @file magls.hpp
 * @brief Magnitude-Least-Squares ambisonic→binaural decode + anthropometric HRTF scaling.
 *
 * The fidelity tier above the virtual-loudspeaker HRIR decode of `spatializer.hpp`. Rather
 * than routing the ambisonic bus through a fixed speaker layout and convolving each speaker
 * with its measured HRIR, @ref MaglsBinauralDecoder solves — once, at configure time — for a
 * set of decode filters (one FIR per ambisonic channel per ear) that best reproduce the full
 * measured HRTF set directly from the bus. Below a cutoff it is a plain complex least-squares
 * fit (matching magnitude *and* phase, so the interaural time difference is exact); above it,
 * where a finite ambisonic order cannot resolve the fast phase variation, it switches to
 * **magnitude** least squares — matching |HRTF| while letting the phase run smoothly from the
 * previous bin — which removes the coloration and localisation error complex LS produces at
 * high frequency (Schörkhuber/Zaunschirm/Zotter). The per-channel FIRs are then applied to the
 * bus each block, so cost is `channels × 2` convolutions regardless of the measurement count.
 *
 * @ref AnthropometricHRTFDatabase personalizes any @ref IHRTFDatabase by a listener's head
 * size: warping the impulse-response time axis scales the interaural delay and the spectral
 * pinna notches together, the classic head-size HRTF individualization.
 *
 * Dependency-free (FFT + the `IHRTFDatabase` seam + real SH), so it rides the `audio.hpp`
 * umbrella; the measured data still arrives through the seam (e.g. the SOFA loader).
 */

#include <cmath>
#include <complex>
#include <cstddef>
#include <vector>

#include <SushiEngine/audio/dsp/fft.hpp>
#include <SushiEngine/audio/dsp/spherical_harmonics.hpp>
#include <SushiEngine/audio/hrtf.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief An @ref IHRTFDatabase decorator that scales HRIRs to a listener's head size.
         *
         * Wraps a base database and resamples each impulse response's time axis by the ratio of
         * a reference head radius to the listener's — a larger head stretches the response
         * (longer ITD, lower pinna notches), a smaller one compresses it. A cheap, well-known
         * personalization that needs no per-user measurement, only a head radius.
         */
        class AnthropometricHRTFDatabase final : public IHRTFDatabase
        {
            public:
                /**
                 * @brief Wraps a database with a head-size warp.
                 * @param base                 The measured database (borrowed; must outlive this).
                 * @param listener_head_radius The listener's head radius in metres.
                 * @param reference_head_radius The radius the base set was measured at (default KEMAR).
                 */
                AnthropometricHRTFDatabase(const IHRTFDatabase& base, float listener_head_radius,
                                           float reference_head_radius = 0.0875f) noexcept
                    : base_(base),
                      warp_(reference_head_radius > 1e-6f ? listener_head_radius / reference_head_radius
                                                          : 1.0f)
                {
                }

                int ir_length() const noexcept override { return base_.ir_length(); }
                double sample_rate() const noexcept override { return base_.sample_rate(); }

                void get_hrir(float front, float left, float up, float* left_ir,
                              float* right_ir) const noexcept override
                {
                    const int n = base_.ir_length();
                    scratch_left_.resize(static_cast<std::size_t>(n));
                    scratch_right_.resize(static_cast<std::size_t>(n));
                    base_.get_hrir(front, left, up, scratch_left_.data(), scratch_right_.data());
                    warp(scratch_left_.data(), left_ir, n);
                    warp(scratch_right_.data(), right_ir, n);
                }

            private:
                // Linearly resample source[0..n) at rate 1/warp_ into destination[0..n)
                // (stretch if warp_>1).
                void warp(const float* source, float* destination, int n) const noexcept
                {
                    for (int t = 0; t < n; ++t)
                    {
                        const float sp = t / warp_;
                        const int i0 = static_cast<int>(sp);
                        if (i0 + 1 >= n)
                        {
                            destination[t] = 0.0f;
                            continue;
                        }
                        const float frac = sp - i0;
                        destination[t] = source[i0] * (1.0f - frac) + source[i0 + 1] * frac;
                    }
                }

                const IHRTFDatabase& base_;
                float warp_;
                mutable std::vector<float> scratch_left_;
                mutable std::vector<float> scratch_right_;
        };

        /** @brief A Magnitude-Least-Squares ambisonic→binaural decoder. */
        class MaglsBinauralDecoder
        {
            public:
                /**
                 * @brief Solves the decode filters from a measured HRTF set.
                 *
                 * Samples a Fibonacci-sphere direction grid, builds the real-SH encoding matrix and
                 * its pseudo-inverse, transforms every HRIR to the frequency domain, and per bin per
                 * ear solves for the per-channel decode spectrum (complex LS below @p magls_cutoff_hz,
                 * magnitude LS with phase continuation above), then inverse-transforms to FIR taps.
                 *
                 * @param order          Ambisonic order (matches the encoder).
                 * @param database       The measured HRTF source.
                 * @param sample_rate    Stream sample rate.
                 * @param fft_size       Transform length (power of two ≥ HRIR length; default 1024).
                 * @param grid_points    Direction-grid size for the fit (default 400).
                 * @param magls_cutoff_hz Frequency above which magnitude LS is used (default 1500).
                 * @return True on success.
                 */
                bool configure(int order, const IHRTFDatabase& database, double sample_rate,
                               int fft_size = 1024, int grid_points = 400,
                               double magls_cutoff_hz = 1500.0)
                {
                    order_ = order;
                    channels_ = DSP::ambisonic_channel_count(order_);
                    sample_rate_ = sample_rate;
                    fft_size_ = fft_size;
                    const int ir_length = database.ir_length();
                    if (ir_length <= 0 || ir_length > fft_size_)
                        return false;

                    fft_.prepare(fft_size_);
                    const int bins = fft_size_ / 2;

                    // 1. Direction grid + real-SH matrix Y (M×C).
                    const int m = grid_points;
                    std::vector<float> dirs(static_cast<std::size_t>(m * 3), 0.0f);
                    std::vector<double> y(static_cast<std::size_t>(m * channels_), 0.0);
                    fibonacci_sphere(m, dirs.data());
                    for (int i = 0; i < m; ++i)
                    {
                        float gains[DSP::MAX_AMBISONIC_CHANNELS];
                        DSP::ambisonic_encode_gains(order_, dirs[i * 3 + 0], dirs[i * 3 + 1],
                                                    dirs[i * 3 + 2], gains);
                        for (int c = 0; c < channels_; ++c)
                            y[static_cast<std::size_t>(i * channels_ + c)] = gains[c];
                    }

                    // 2. Pseudo-inverse Ypinv (C×M) = (YᵀY)⁻¹ Yᵀ.
                    std::vector<double> ypinv;
                    if (!pseudo_inverse(y, m, channels_, ypinv))
                        return false;

                    // 3. HRTF spectra per direction per ear.
                    std::vector<std::complex<float>> hl(static_cast<std::size_t>(m * bins));
                    std::vector<std::complex<float>> hr(static_cast<std::size_t>(m * bins));
                    std::vector<float> lir(static_cast<std::size_t>(ir_length));
                    std::vector<float> rir(static_cast<std::size_t>(ir_length));
                    std::vector<std::complex<float>> spectrum(static_cast<std::size_t>(fft_size_));
                    for (int i = 0; i < m; ++i)
                    {
                        database.get_hrir(dirs[i * 3 + 0], dirs[i * 3 + 1], dirs[i * 3 + 2],
                                          lir.data(), rir.data());
                        transform_ir(lir.data(), ir_length, spectrum.data());
                        for (int k = 0; k < bins; ++k)
                            hl[static_cast<std::size_t>(i * bins + k)] =
                                spectrum[static_cast<std::size_t>(k)];
                        transform_ir(rir.data(), ir_length, spectrum.data());
                        for (int k = 0; k < bins; ++k)
                            hr[static_cast<std::size_t>(i * bins + k)] =
                                spectrum[static_cast<std::size_t>(k)];
                    }

                    // 4. Solve decode filters per ear.
                    filters_left_.assign(static_cast<std::size_t>(channels_), {});
                    filters_right_.assign(static_cast<std::size_t>(channels_), {});
                    solve_ear(hl, y, ypinv, m, bins, magls_cutoff_hz, filters_left_);
                    solve_ear(hr, y, ypinv, m, bins, magls_cutoff_hz, filters_right_);

                    // 5. Install as per-channel FIRs.
                    conv_left_.assign(static_cast<std::size_t>(channels_), {});
                    conv_right_.assign(static_cast<std::size_t>(channels_), {});
                    for (int c = 0; c < channels_; ++c)
                    {
                        conv_left_[static_cast<std::size_t>(c)].prepare(
                            filters_left_[static_cast<std::size_t>(c)].data(), fft_size_);
                        conv_right_[static_cast<std::size_t>(c)].prepare(
                            filters_right_[static_cast<std::size_t>(c)].data(), fft_size_);
                    }
                    valid_ = true;
                    return true;
                }

                /** @brief The ambisonic channel count the decoder expects. */
                int channel_count() const noexcept { return channels_; }
                /** @brief Whether the decoder is configured and usable. */
                bool valid() const noexcept { return valid_; }
                /** @brief The decode FIR length. */
                int filter_length() const noexcept { return fft_size_; }

                /** @brief Zeroes the per-channel convolver histories. */
                void reset() noexcept
                {
                    for (HrirConvolver& c : conv_left_)
                        c.reset();
                    for (HrirConvolver& c : conv_right_)
                        c.reset();
                }

                /**
                 * @brief Decodes the ambisonic bus to the two ears and accumulates.
                 * @param bus         The ambisonic channels (at least @ref channel_count of them).
                 * @param channels    Channels available in @p bus.
                 * @param frame_count Number of samples.
                 * @param left        Left-ear output (accumulated into).
                 * @param right       Right-ear output (accumulated into).
                 */
                void process(const float* const* bus, int channels, int frame_count, float* left,
                             float* right) noexcept
                {
                    const int n = channels < channels_ ? channels : channels_;
                    for (int c = 0; c < n; ++c)
                    {
                        conv_left_[static_cast<std::size_t>(c)].process_block(bus[c], left, frame_count);
                        conv_right_[static_cast<std::size_t>(c)].process_block(bus[c], right, frame_count);
                    }
                }

            private:
                static void fibonacci_sphere(int count, float* dirs) noexcept
                {
                    const double golden = 3.14159265358979323846 * (3.0 - std::sqrt(5.0));
                    for (int i = 0; i < count; ++i)
                    {
                        const double z = 1.0 - 2.0 * (i + 0.5) / count; // up axis
                        const double r = std::sqrt(1.0 - z * z);
                        const double theta = golden * i;
                        dirs[i * 3 + 0] = static_cast<float>(r * std::cos(theta)); // front
                        dirs[i * 3 + 1] = static_cast<float>(r * std::sin(theta)); // left
                        dirs[i * 3 + 2] = static_cast<float>(z);                   // up
                    }
                }

                // Zero-pad an impulse response to fft_size and forward transform into spectrum.
                void transform_ir(const float* ir, int ir_length,
                                  std::complex<float>* spectrum) noexcept
                {
                    for (int i = 0; i < fft_size_; ++i)
                        spectrum[i] = (i < ir_length) ? std::complex<float>(ir[i], 0.0f)
                                               : std::complex<float>(0.0f, 0.0f);
                    fft_.forward(spectrum);
                }

                // (YᵀY)⁻¹Yᵀ with Y row-major m×c; result ypinv row-major c×m. Regularized.
                static bool pseudo_inverse(const std::vector<double>& y, int m, int c,
                                           std::vector<double>& ypinv)
                {
                    std::vector<double> gram(static_cast<std::size_t>(c * c), 0.0);
                    for (int a = 0; a < c; ++a)
                        for (int b = 0; b < c; ++b)
                        {
                            double s = 0.0;
                            for (int i = 0; i < m; ++i)
                                s += y[static_cast<std::size_t>(i * c + a)] *
                                     y[static_cast<std::size_t>(i * c + b)];
                            gram[static_cast<std::size_t>(a * c + b)] = s;
                        }
                    for (int a = 0; a < c; ++a)
                        gram[static_cast<std::size_t>(a * c + a)] += 1e-6; // Tikhonov

                    std::vector<double> inv;
                    if (!invert(gram, c, inv))
                        return false;

                    ypinv.assign(static_cast<std::size_t>(c * m), 0.0);
                    for (int a = 0; a < c; ++a)
                        for (int i = 0; i < m; ++i)
                        {
                            double s = 0.0;
                            for (int b = 0; b < c; ++b)
                                s += inv[static_cast<std::size_t>(a * c + b)] *
                                     y[static_cast<std::size_t>(i * c + b)];
                            ypinv[static_cast<std::size_t>(a * m + i)] = s;
                        }
                    return true;
                }

                // Gauss-Jordan inverse of an n×n row-major matrix.
                static bool invert(std::vector<double> a, int n, std::vector<double>& out)
                {
                    out.assign(static_cast<std::size_t>(n * n), 0.0);
                    for (int i = 0; i < n; ++i)
                        out[static_cast<std::size_t>(i * n + i)] = 1.0;
                    for (int col = 0; col < n; ++col)
                    {
                        int pivot = col;
                        double best = std::fabs(a[static_cast<std::size_t>(col * n + col)]);
                        for (int r = col + 1; r < n; ++r)
                        {
                            const double v = std::fabs(a[static_cast<std::size_t>(r * n + col)]);
                            if (v > best)
                            {
                                best = v;
                                pivot = r;
                            }
                        }
                        if (best < 1e-12)
                            return false;
                        if (pivot != col)
                            for (int k = 0; k < n; ++k)
                            {
                                std::swap(a[static_cast<std::size_t>(col * n + k)],
                                          a[static_cast<std::size_t>(pivot * n + k)]);
                                std::swap(out[static_cast<std::size_t>(col * n + k)],
                                          out[static_cast<std::size_t>(pivot * n + k)]);
                            }
                        const double diag = a[static_cast<std::size_t>(col * n + col)];
                        for (int k = 0; k < n; ++k)
                        {
                            a[static_cast<std::size_t>(col * n + k)] /= diag;
                            out[static_cast<std::size_t>(col * n + k)] /= diag;
                        }
                        for (int r = 0; r < n; ++r)
                        {
                            if (r == col)
                                continue;
                            const double factor = a[static_cast<std::size_t>(r * n + col)];
                            for (int k = 0; k < n; ++k)
                            {
                                a[static_cast<std::size_t>(r * n + k)] -=
                                    factor * a[static_cast<std::size_t>(col * n + k)];
                                out[static_cast<std::size_t>(r * n + k)] -=
                                    factor * out[static_cast<std::size_t>(col * n + k)];
                            }
                        }
                    }
                    return true;
                }

                // Solve one ear's per-channel decode filters into `filters` (C vectors of fft_size).
                void solve_ear(const std::vector<std::complex<float>>& h, const std::vector<double>& y,
                               const std::vector<double>& ypinv, int m, int bins,
                               double magls_cutoff_hz, std::vector<std::vector<float>>& filters)
                {
                    const int c = channels_;
                    std::vector<std::vector<std::complex<float>>> d(
                        static_cast<std::size_t>(c),
                        std::vector<std::complex<float>>(static_cast<std::size_t>(fft_size_),
                                                         std::complex<float>(0.0f, 0.0f)));
                    std::vector<double> phase(static_cast<std::size_t>(m), 0.0);
                    std::vector<std::complex<double>> target(static_cast<std::size_t>(m));

                    for (int k = 0; k < bins; ++k)
                    {
                        const double f = k * sample_rate_ / fft_size_;
                        const bool magnitude = f > magls_cutoff_hz;
                        for (int i = 0; i < m; ++i)
                        {
                            const std::complex<double> hv = h[static_cast<std::size_t>(i * bins + k)];
                            target[static_cast<std::size_t>(i)] =
                                magnitude ? std::polar(std::abs(hv), phase[static_cast<std::size_t>(i)])
                                          : hv;
                        }
                        for (int a = 0; a < c; ++a)
                        {
                            std::complex<double> acc(0.0, 0.0);
                            for (int i = 0; i < m; ++i)
                                acc += ypinv[static_cast<std::size_t>(a * m + i)] *
                                       target[static_cast<std::size_t>(i)];
                            d[static_cast<std::size_t>(a)][static_cast<std::size_t>(k)] =
                                std::complex<float>(static_cast<float>(acc.real()),
                                                    static_cast<float>(acc.imag()));
                        }
                        // Reconstruct the field to continue the phase into the next bin.
                        for (int i = 0; i < m; ++i)
                        {
                            std::complex<double> recon(0.0, 0.0);
                            for (int a = 0; a < c; ++a)
                                recon += y[static_cast<std::size_t>(i * c + a)] *
                                         std::complex<double>(
                                             d[static_cast<std::size_t>(a)][static_cast<std::size_t>(k)]);
                            phase[static_cast<std::size_t>(i)] = std::arg(recon);
                        }
                    }

                    // Hermitian-complete and inverse-transform each channel to real taps.
                    filters.assign(static_cast<std::size_t>(c),
                                   std::vector<float>(static_cast<std::size_t>(fft_size_), 0.0f));
                    std::vector<std::complex<float>> spectrum(static_cast<std::size_t>(fft_size_));
                    for (int a = 0; a < c; ++a)
                    {
                        for (int k = 0; k < fft_size_; ++k)
                            spectrum[static_cast<std::size_t>(k)] =
                                d[static_cast<std::size_t>(a)][static_cast<std::size_t>(k)];
                        for (int k = 1; k < bins; ++k)
                            spectrum[static_cast<std::size_t>(fft_size_ - k)] =
                                std::conj(spectrum[static_cast<std::size_t>(k)]);
                        spectrum[static_cast<std::size_t>(bins)] = std::complex<float>(
                            spectrum[static_cast<std::size_t>(bins)].real(), 0.0f);
                        fft_.inverse(spectrum.data());
                        for (int t = 0; t < fft_size_; ++t)
                            filters[static_cast<std::size_t>(a)][static_cast<std::size_t>(t)] =
                                spectrum[static_cast<std::size_t>(t)].real();
                    }
                }

                DSP::RadixFFT fft_;
                std::vector<std::vector<float>> filters_left_;
                std::vector<std::vector<float>> filters_right_;
                std::vector<HrirConvolver> conv_left_;
                std::vector<HrirConvolver> conv_right_;
                int order_ = 3;
                int channels_ = 16;
                int fft_size_ = 1024;
                double sample_rate_ = 48000.0;
                bool valid_ = false;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
