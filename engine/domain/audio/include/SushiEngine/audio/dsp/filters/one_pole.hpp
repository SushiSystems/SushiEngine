/**************************************************************************/
/* one_pole.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FILTERS_ONE_POLE_HPP
#define SUSHIENGINE_AUDIO_DSP_FILTERS_ONE_POLE_HPP

/**
 * @file one_pole.hpp
 * @brief The one-pole filter — the cheapest smoother, DC block, and gentle tone control.
 *
 * A single leaky integrator, `y += a·(x − y)`. It is the parameter smoother that
 * turns an atomically-published target into a click-free ramp on the audio thread,
 * the envelope follower behind level meters and duckers, the air-absorption cutoff
 * (§5), and a 6 dB/oct tilt when a full biquad is overkill. Coefficients are computed
 * in double and stored as `float`, the house rule for the audio path.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A first-order low-pass / high-pass / smoothing filter.
             *
             * Holds one state sample. The coefficient sets the −3 dB corner; the
             * low-pass output is the state, the high-pass output is the input minus the
             * state (their sum reconstructs the input). @ref process_smoother is the
             * same recursion framed as "approach a target", the form parameter ramps
             * use.
             */
            class OnePole
            {
                public:
                    /**
                     * @brief Sets the low-pass corner frequency.
                     * @param cutoff_hz    The −3 dB corner in Hz.
                     * @param sample_rate  The stream sample rate in Hz.
                     */
                    void set_low_pass(double cutoff_hz, double sample_rate) noexcept
                    {
                        if (cutoff_hz < 0.0)
                            cutoff_hz = 0.0;
                        const double nyquist = sample_rate * 0.5;
                        if (cutoff_hz > nyquist)
                            cutoff_hz = nyquist;
                        // a = 1 - e^(-2*pi*fc/fs): the impulse-invariant one-pole coefficient.
                        const double two_pi = 6.28318530717958647692;
                        coefficient_ = static_cast<float>(1.0 - std::exp(-two_pi * cutoff_hz / sample_rate));
                    }

                    /**
                     * @brief Sets the coefficient directly (for smoothers keyed to a time constant).
                     * @param coefficient The blend factor in [0, 1]; larger tracks faster.
                     */
                    void set_coefficient(float coefficient) noexcept { coefficient_ = coefficient; }

                    /**
                     * @brief Sets the smoothing coefficient from a time constant.
                     * @param time_constant_seconds The 1/e settling time in seconds.
                     * @param sample_rate           The stream sample rate in Hz.
                     */
                    void set_time_constant(double time_constant_seconds, double sample_rate) noexcept
                    {
                        if (time_constant_seconds <= 0.0)
                        {
                            coefficient_ = 1.0f;
                            return;
                        }
                        coefficient_ = static_cast<float>(
                            1.0 - std::exp(-1.0 / (time_constant_seconds * sample_rate)));
                    }

                    /** @brief Clears the filter state to zero. */
                    void reset() noexcept { state_ = 0.0f; }

                    /**
                     * @brief Processes one sample, returning the low-pass output.
                     * @param input The input sample.
                     * @return The low-passed sample.
                     */
                    float process_low_pass(float input) noexcept
                    {
                        state_ += coefficient_ * (input - state_);
                        return state_;
                    }

                    /**
                     * @brief Processes one sample, returning the high-pass output.
                     * @param input The input sample.
                     * @return The input minus its low-passed part.
                     */
                    float process_high_pass(float input) noexcept
                    {
                        return input - process_low_pass(input);
                    }

                    /**
                     * @brief Advances a smoother one sample toward @p target.
                     * @param target The value the smoother is ramping toward.
                     * @return The smoothed value this sample.
                     */
                    float process_smoother(float target) noexcept { return process_low_pass(target); }

                    /** @brief The current internal state (the last low-pass output). */
                    float state() const noexcept { return state_; }

                private:
                    float coefficient_ = 1.0f;
                    float state_ = 0.0f;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
