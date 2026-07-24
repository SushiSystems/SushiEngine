/**************************************************************************/
/* state_variable.hpp                                                    */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FILTERS_STATE_VARIABLE_HPP
#define SUSHIENGINE_AUDIO_DSP_FILTERS_STATE_VARIABLE_HPP

/**
 * @file state_variable.hpp
 * @brief The Cytomic TPT state-variable filter — the modulatable, robust default.
 *
 * Andrew Simper's (Cytomic) topology-preserving-transform SVF: a bilinear-accurate
 * 2-pole filter whose two integrator states stay stable when the cutoff is swept in
 * real time — where a naïve biquad's coefficient jumps click or blow up — and which
 * produces low-pass, high-pass, band-pass, notch, peak, and all-pass outputs from the
 * *same* two states at once. This makes it the go-to for anything modulated
 * (filter sweeps, per-voice cutoff on an RTPC) and for multimode filters; the
 * @ref Biquad cookbook stays the choice for fixed EQ curves.
 *
 * The coefficient recurrence is the "SvfLinearTrapOptimised2" form; coefficients are
 * computed in `double` and stored `float`, as everywhere on the audio path.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A trapezoidal-integrator (TPT) state-variable filter.
             *
             * Set cutoff and resonance with @ref set (off-thread), then call
             * @ref process per sample and read whichever mode output the caller wants
             * (@ref low_pass, @ref band_pass, @ref high_pass, @ref notch, @ref peak,
             * @ref all_pass) — all valid after the same @ref process call.
             */
            class StateVariableFilter
            {
                public:
                    /**
                     * @brief Sets cutoff and resonance.
                     * @param cutoff_hz    The cutoff frequency in Hz.
                     * @param q            The resonance / quality factor (0.707 = Butterworth).
                     * @param sample_rate  The stream sample rate in Hz.
                     */
                    void set(double cutoff_hz, double q, double sample_rate) noexcept
                    {
                        const double nyquist = sample_rate * 0.5;
                        double fc = cutoff_hz;
                        if (fc < 1.0)
                            fc = 1.0;
                        if (fc > nyquist - 1.0)
                            fc = nyquist - 1.0;
                        const double pi = 3.14159265358979323846;
                        const double g = std::tan(pi * fc / sample_rate);
                        const double k = 1.0 / (q > 1e-4 ? q : 1e-4);
                        const double a1 = 1.0 / (1.0 + g * (g + k));
                        const double a2 = g * a1;
                        const double a3 = g * a2;
                        g_ = static_cast<float>(g);
                        k_ = static_cast<float>(k);
                        a1_ = static_cast<float>(a1);
                        a2_ = static_cast<float>(a2);
                        a3_ = static_cast<float>(a3);
                    }

                    /** @brief Clears both integrator states. */
                    void reset() noexcept
                    {
                        ic1eq_ = 0.0f;
                        ic2eq_ = 0.0f;
                    }

                    /**
                     * @brief Advances the filter one sample.
                     * @param input The input sample.
                     * @return The low-pass output (the other modes are read via the accessors).
                     */
                    float process(float input) noexcept
                    {
                        const float v3 = input - ic2eq_;
                        const float v1 = a1_ * ic1eq_ + a2_ * v3;
                        const float v2 = ic2eq_ + a2_ * ic1eq_ + a3_ * v3;
                        ic1eq_ = 2.0f * v1 - ic1eq_;
                        ic2eq_ = 2.0f * v2 - ic2eq_;
                        low_pass_ = v2;
                        band_pass_ = v1;
                        high_pass_ = input - k_ * v1 - v2;
                        return low_pass_;
                    }

                    /** @brief The low-pass output from the last @ref process. */
                    float low_pass() const noexcept { return low_pass_; }

                    /** @brief The band-pass output from the last @ref process. */
                    float band_pass() const noexcept { return band_pass_; }

                    /** @brief The high-pass output from the last @ref process. */
                    float high_pass() const noexcept { return high_pass_; }

                    /** @brief The notch output (high-pass + low-pass) from the last @ref process. */
                    float notch() const noexcept { return high_pass_ + low_pass_; }

                    /** @brief The peaking output (high-pass − low-pass) from the last @ref process. */
                    float peak() const noexcept { return high_pass_ - low_pass_; }

                    /** @brief The all-pass output from the last @ref process. */
                    float all_pass() const noexcept { return high_pass_ + low_pass_ - k_ * band_pass_; }

                private:
                    float g_ = 0.0f, k_ = 1.0f;
                    float a1_ = 0.0f, a2_ = 0.0f, a3_ = 0.0f;
                    float ic1eq_ = 0.0f, ic2eq_ = 0.0f;
                    float low_pass_ = 0.0f, band_pass_ = 0.0f, high_pass_ = 0.0f;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
