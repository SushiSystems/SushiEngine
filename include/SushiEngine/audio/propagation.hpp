/**************************************************************************/
/* propagation.hpp                                                       */
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

#ifndef SUSHIENGINE_AUDIO_PROPAGATION_HPP
#define SUSHIENGINE_AUDIO_PROPAGATION_HPP

/**
 * @file propagation.hpp
 * @brief Per-source propagation: Doppler + delay, air absorption, distance gain.
 *
 * The whole of a source's travel through the air is modelled as one variable
 * fractional delay line of length distance/c (see `docs/design/audio_system.md` §5).
 * The read pointer sits that far behind the write pointer; when the distance changes
 * between blocks the read rate stops being one, and *that* is the Doppler shift —
 * physically exact, free propagation delay, no radial-velocity ratio and no separate
 * resampler. After the delay the block is dulled by a distance-driven air-absorption
 * low-pass and scaled by the distance-attenuation law.
 *
 * The delay is slew-limited (a source cannot appear to move faster than sound —
 * `|Δdelay| < 0.9` sample per sample, i.e. `|v_radial| < 0.9·c`), and a large jump
 * (a teleport/respawn) **snaps** the delay rather than sweeping it, which would
 * otherwise fire a synthetic Doppler screech.
 */

#include <cmath>

#include <SushiEngine/audio/dsp/air_absorption.hpp>
#include <SushiEngine/audio/dsp/filters/one_pole.hpp>
#include <SushiEngine/audio/dsp/fractional_delay.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/parameter.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief The per-source propagation processor: delay/Doppler, air, distance gain.
         *
         * One instance per spatial voice. @ref prepare sizes the delay line for the
         * farthest modelled distance; @ref process renders one block given the current
         * source-to-listener distance, deriving the Doppler from how that distance
         * changed since the previous block.
         */
        class SourcePropagation
        {
            public:
                /**
                 * @brief Allocates state for a run.
                 * @param sample_rate       The stream sample rate in Hz.
                 * @param max_block_frames  The largest block processed.
                 * @param max_delay_samples The delay-line capacity (farthest distance / c · fs).
                 */
                void prepare(double sample_rate, int max_block_frames, int max_delay_samples)
                {
                    sample_rate_ = sample_rate;
                    max_block_ = max_block_frames;
                    delay_.prepare(max_delay_samples);
                    cutoff_.configure(0.02, sample_rate); // 20 ms cutoff slew
                    cutoff_.snap(20000.0f);
                    air_.reset();
                    initialized_ = false;
                }

                /** @brief Clears delay, filter, and motion state (e.g. when a voice restarts). */
                void reset() noexcept
                {
                    delay_.reset();
                    air_.reset();
                    initialized_ = false;
                }

                /**
                 * @brief Renders one block of propagation for a source at @p distance_meters.
                 * @param input           The dry mono source block.
                 * @param output          The processed block (may alias @p input).
                 * @param frame_count     Number of samples.
                 * @param distance_meters Current source-to-listener distance in metres.
                 * @param atmosphere      The atmospheric state (speed of sound, absorption).
                 * @param descriptor      The voice's distance model, doppler scale, and delay toggle.
                 * @return The distance-attenuation gain applied this block (for diagnostics).
                 */
                float process(const float* input, float* output, int frame_count,
                              float distance_meters, const Dsp::Atmosphere& atmosphere,
                              const VoiceDescriptor& descriptor) noexcept
                {
                    const float c = Dsp::speed_of_sound(atmosphere);
                    const float physical_now =
                        (distance_meters / c) * static_cast<float>(sample_rate_);

                    if (!initialized_)
                    {
                        read_delay_ = physical_now;
                        physical_prev_ = physical_now;
                        gain_prev_ = distance_attenuation(descriptor.model, distance_meters,
                                                          descriptor.min_distance,
                                                          descriptor.max_distance, descriptor.rolloff);
                        initialized_ = true;
                    }

                    if (descriptor.propagation_delay)
                    {
                        float delta = physical_now - physical_prev_;

                        // Teleport: a jump larger than the whole block's worth of samples
                        // is a discontinuity, not motion — snap instead of sweeping.
                        const float teleport = teleport_threshold(frame_count);
                        if (std::fabs(delta) > teleport)
                        {
                            read_delay_ = physical_now;
                            delta = 0.0f;
                        }

                        float end_delay = read_delay_ + descriptor.doppler_scale * delta;

                        // |v_radial| < 0.9·c: the delay may not change by more than 0.9
                        // sample per sample, or the source would break the sound barrier.
                        const float max_change = 0.9f * static_cast<float>(frame_count);
                        if (end_delay - read_delay_ > max_change)
                            end_delay = read_delay_ + max_change;
                        else if (read_delay_ - end_delay > max_change)
                            end_delay = read_delay_ - max_change;

                        const float max_delay = delay_.max_delay();
                        read_delay_ = clamp(read_delay_, 1.0f, max_delay);
                        end_delay = clamp(end_delay, 1.0f, max_delay);

                        delay_.process_block(input, output, frame_count, read_delay_, end_delay);
                        read_delay_ = end_delay;
                        physical_prev_ = physical_now;
                    }
                    else
                    {
                        // Delay (and Doppler) disabled: keep the delay line primed so a
                        // later enable does not click, but pass the dry signal through.
                        for (int i = 0; i < frame_count; ++i)
                        {
                            delay_.push(input[i]);
                            output[i] = input[i];
                        }
                        physical_prev_ = physical_now;
                    }

                    // Air absorption: a distance-driven low-pass, its cutoff slewed.
                    const float cutoff_target =
                        Dsp::air_absorption_cutoff(distance_meters, atmosphere);
                    cutoff_.set_target(cutoff_target);
                    const float cutoff = cutoff_.advance_block(frame_count);
                    air_.set_low_pass(cutoff, sample_rate_);
                    for (int i = 0; i < frame_count; ++i)
                        output[i] = air_.process_low_pass(output[i]);

                    // Distance attenuation, ramped from the previous block's gain.
                    const float gain_now =
                        distance_attenuation(descriptor.model, distance_meters,
                                             descriptor.min_distance, descriptor.max_distance,
                                             descriptor.rolloff);
                    Dsp::Simd::apply_gain_ramp(output, frame_count, gain_prev_, gain_now);
                    gain_prev_ = gain_now;
                    return gain_now;
                }

            private:
                static float clamp(float value, float low, float high) noexcept
                {
                    if (value < low)
                        return low;
                    if (value > high)
                        return high;
                    return value;
                }

                static float teleport_threshold(int frame_count) noexcept
                {
                    const float block_worth = 4.0f * static_cast<float>(frame_count);
                    return block_worth > 480.0f ? block_worth : 480.0f;
                }

                Dsp::FractionalDelayLine delay_;
                Dsp::OnePole air_;
                SmoothedValue cutoff_;
                double sample_rate_ = 48000.0;
                int max_block_ = 0;
                float read_delay_ = 1.0f;
                float physical_prev_ = 0.0f;
                float gain_prev_ = 1.0f;
                bool initialized_ = false;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
