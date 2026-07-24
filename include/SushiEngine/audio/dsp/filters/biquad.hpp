/**************************************************************************/
/* biquad.hpp                                                            */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FILTERS_BIQUAD_HPP
#define SUSHIENGINE_AUDIO_DSP_FILTERS_BIQUAD_HPP

/**
 * @file biquad.hpp
 * @brief The second-order (biquad) filter, RBJ cookbook coefficients, TDF-II.
 *
 * The general-purpose EQ/crossover building block: low/high-pass, band-pass, notch,
 * peaking, and shelving from the Robert Bristow-Johnson "Audio EQ Cookbook" analog
 * prototypes mapped to digital by the bilinear transform. The runtime form is the
 * **Transposed Direct Form II** — two state samples, the numerically better-behaved
 * biquad topology for time-varying coefficients. Coefficients are designed in
 * `double` and stored as `float` (the audio-path rule); design happens off the audio
 * thread and the state recursion runs on it.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A single biquad section with RBJ-cookbook designers and TDF-II state.
             *
             * Call one `set_*` designer (off-thread) to load coefficients, then
             * @ref process per sample. @ref reset clears the two state samples. The
             * default (freshly constructed) filter is a unity pass-through.
             */
            class Biquad
            {
                public:
                    /**
                     * @brief Designs a low-pass section.
                     * @param cutoff_hz   The −3 dB corner in Hz.
                     * @param q           The resonance / quality factor (0.707 = Butterworth).
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_low_pass(double cutoff_hz, double q, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(cutoff_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double alpha = std::sin(w0) / (2.0 * safe_q(q));
                        b0 = (1.0 - cw) * 0.5;
                        b1 = 1.0 - cw;
                        b2 = (1.0 - cw) * 0.5;
                        a0 = 1.0 + alpha;
                        a1 = -2.0 * cw;
                        a2 = 1.0 - alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a high-pass section.
                     * @param cutoff_hz   The −3 dB corner in Hz.
                     * @param q           The resonance / quality factor.
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_high_pass(double cutoff_hz, double q, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(cutoff_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double alpha = std::sin(w0) / (2.0 * safe_q(q));
                        b0 = (1.0 + cw) * 0.5;
                        b1 = -(1.0 + cw);
                        b2 = (1.0 + cw) * 0.5;
                        a0 = 1.0 + alpha;
                        a1 = -2.0 * cw;
                        a2 = 1.0 - alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a constant-skirt-gain band-pass section (0 dB peak).
                     * @param centre_hz   The centre frequency in Hz.
                     * @param q           The quality factor (centre / bandwidth).
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_band_pass(double centre_hz, double q, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(centre_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double alpha = std::sin(w0) / (2.0 * safe_q(q));
                        b0 = alpha;
                        b1 = 0.0;
                        b2 = -alpha;
                        a0 = 1.0 + alpha;
                        a1 = -2.0 * cw;
                        a2 = 1.0 - alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a notch (band-reject) section.
                     * @param centre_hz   The rejected centre frequency in Hz.
                     * @param q           The quality factor.
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_notch(double centre_hz, double q, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(centre_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double alpha = std::sin(w0) / (2.0 * safe_q(q));
                        b0 = 1.0;
                        b1 = -2.0 * cw;
                        b2 = 1.0;
                        a0 = 1.0 + alpha;
                        a1 = -2.0 * cw;
                        a2 = 1.0 - alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a peaking (bell) EQ section.
                     * @param centre_hz   The centre frequency in Hz.
                     * @param q           The quality factor.
                     * @param gain_db      The peak gain in decibels (positive boosts, negative cuts).
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_peaking(double centre_hz, double q, double gain_db,
                                     double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(centre_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double alpha = std::sin(w0) / (2.0 * safe_q(q));
                        const double amp = std::pow(10.0, gain_db / 40.0);
                        b0 = 1.0 + alpha * amp;
                        b1 = -2.0 * cw;
                        b2 = 1.0 - alpha * amp;
                        a0 = 1.0 + alpha / amp;
                        a1 = -2.0 * cw;
                        a2 = 1.0 - alpha / amp;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a low-shelf section.
                     * @param corner_hz   The shelf corner frequency in Hz.
                     * @param gain_db      The shelf gain in decibels.
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_low_shelf(double corner_hz, double gain_db, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(corner_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double amp = std::pow(10.0, gain_db / 40.0);
                        // Shelf slope S = 1 (max without gain overshoot).
                        const double alpha = std::sin(w0) * 0.5 * std::sqrt((amp + 1.0 / amp) + 2.0);
                        const double two_sqrt_a_alpha = 2.0 * std::sqrt(amp) * alpha;
                        b0 = amp * ((amp + 1.0) - (amp - 1.0) * cw + two_sqrt_a_alpha);
                        b1 = 2.0 * amp * ((amp - 1.0) - (amp + 1.0) * cw);
                        b2 = amp * ((amp + 1.0) - (amp - 1.0) * cw - two_sqrt_a_alpha);
                        a0 = (amp + 1.0) + (amp - 1.0) * cw + two_sqrt_a_alpha;
                        a1 = -2.0 * ((amp - 1.0) + (amp + 1.0) * cw);
                        a2 = (amp + 1.0) + (amp - 1.0) * cw - two_sqrt_a_alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /**
                     * @brief Designs a high-shelf section.
                     * @param corner_hz   The shelf corner frequency in Hz.
                     * @param gain_db      The shelf gain in decibels.
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void set_high_shelf(double corner_hz, double gain_db, double sample_rate) noexcept
                    {
                        double b0, b1, b2, a0, a1, a2;
                        const double w0 = omega(corner_hz, sample_rate);
                        const double cw = std::cos(w0);
                        const double amp = std::pow(10.0, gain_db / 40.0);
                        const double alpha = std::sin(w0) * 0.5 * std::sqrt((amp + 1.0 / amp) + 2.0);
                        const double two_sqrt_a_alpha = 2.0 * std::sqrt(amp) * alpha;
                        b0 = amp * ((amp + 1.0) + (amp - 1.0) * cw + two_sqrt_a_alpha);
                        b1 = -2.0 * amp * ((amp - 1.0) + (amp + 1.0) * cw);
                        b2 = amp * ((amp + 1.0) + (amp - 1.0) * cw - two_sqrt_a_alpha);
                        a0 = (amp + 1.0) - (amp - 1.0) * cw + two_sqrt_a_alpha;
                        a1 = 2.0 * ((amp - 1.0) - (amp + 1.0) * cw);
                        a2 = (amp + 1.0) - (amp - 1.0) * cw - two_sqrt_a_alpha;
                        normalize(b0, b1, b2, a0, a1, a2);
                    }

                    /** @brief Clears the two state samples. */
                    void reset() noexcept
                    {
                        z1_ = 0.0f;
                        z2_ = 0.0f;
                    }

                    /**
                     * @brief Processes one sample through the Transposed Direct Form II recursion.
                     * @param input The input sample.
                     * @return The filtered sample.
                     */
                    float process(float input) noexcept
                    {
                        const float output = b0_ * input + z1_;
                        z1_ = b1_ * input - a1_ * output + z2_;
                        z2_ = b2_ * input - a2_ * output;
                        return output;
                    }

                private:
                    static double omega(double frequency_hz, double sample_rate) noexcept
                    {
                        const double two_pi = 6.28318530717958647692;
                        double f = frequency_hz;
                        const double nyquist = sample_rate * 0.5;
                        if (f < 1.0)
                            f = 1.0;
                        if (f > nyquist - 1.0)
                            f = nyquist - 1.0;
                        return two_pi * f / sample_rate;
                    }

                    static double safe_q(double q) noexcept { return q > 1e-4 ? q : 1e-4; }

                    void normalize(double b0, double b1, double b2,
                                   double a0, double a1, double a2) noexcept
                    {
                        const double inv_a0 = 1.0 / a0;
                        b0_ = static_cast<float>(b0 * inv_a0);
                        b1_ = static_cast<float>(b1 * inv_a0);
                        b2_ = static_cast<float>(b2 * inv_a0);
                        a1_ = static_cast<float>(a1 * inv_a0);
                        a2_ = static_cast<float>(a2 * inv_a0);
                    }

                    float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f;
                    float a1_ = 0.0f, a2_ = 0.0f;
                    float z1_ = 0.0f, z2_ = 0.0f;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
