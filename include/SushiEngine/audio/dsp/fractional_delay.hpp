/**************************************************************************/
/* fractional_delay.hpp                                                  */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FRACTIONAL_DELAY_HPP
#define SUSHIENGINE_AUDIO_DSP_FRACTIONAL_DELAY_HPP

/**
 * @file fractional_delay.hpp
 * @brief The fractional delay line — the primitive Doppler and propagation are built on.
 *
 * A circular buffer read at a **fractional** number of samples behind the write head,
 * interpolated with a 4-point cubic Lagrange kernel (the Farrow form: the fractional
 * part is the polynomial variable). Non-recursive, so there is no ringing and the
 * delay can be modulated cleanly — the property that matters, because a *time-varying*
 * read delay is exactly a pitch shift: as the read delay grows the output plays back
 * slower (a falling Doppler), as it shrinks it plays faster (a rising one). Modelling
 * a source's propagation as one delay line of length distance/c makes the Doppler fall
 * out for free and keeps the dry signal in sync with its reverb sends (see
 * `docs/design/audio_system.md` §3.6, §5).
 *
 * Allpass (Thiran) interpolation is reserved for magnitude-preserving feedback loops
 * and windowed-sinc for hero/offline resampling; cubic Lagrange is the moving-source
 * default here.
 */

#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A circular delay line with cubic-Lagrange fractional read.
             *
             * @ref prepare sizes the buffer for the largest delay ever requested;
             * @ref read returns the signal a fractional number of samples in the past;
             * @ref process_block writes an input block while reading with a per-sample
             * linearly-ramped delay (the Doppler path).
             */
            class FractionalDelayLine
            {
                public:
                    /**
                     * @brief Allocates the delay buffer.
                     *
                     * The capacity is rounded up to a power of two at least
                     * @p max_delay_samples + 4 (headroom for the 4-tap kernel), so index
                     * wrap is a mask.
                     *
                     * @param max_delay_samples The largest delay, in samples, that will be read.
                     */
                    void prepare(int max_delay_samples)
                    {
                        std::size_t needed = static_cast<std::size_t>(max_delay_samples) + 4;
                        std::size_t capacity = 4;
                        while (capacity < needed)
                            capacity <<= 1;
                        buffer_.assign(capacity, 0.0f);
                        mask_ = capacity - 1;
                        write_ = 0;
                    }

                    /** @brief Clears the buffer and resets the write head. */
                    void reset() noexcept
                    {
                        for (float& sample : buffer_)
                            sample = 0.0f;
                        write_ = 0;
                    }

                    /** @brief The largest delay the buffer can serve (capacity − 4 samples). */
                    float max_delay() const noexcept
                    {
                        return buffer_.empty() ? 0.0f : static_cast<float>(buffer_.size() - 4);
                    }

                    /**
                     * @brief Writes one sample and advances the write head.
                     * @param input The sample to store.
                     */
                    void push(float input) noexcept
                    {
                        buffer_[write_ & mask_] = input;
                        ++write_;
                    }

                    /**
                     * @brief Reads the signal @p delay_samples in the past (cubic Lagrange).
                     *
                     * @p delay_samples is clamped to `[1, max_delay()]`; the 1-sample floor
                     * keeps the read causal (never sampling ahead of the write head) and
                     * leaves room for the newer of the four interpolation taps.
                     *
                     * @param delay_samples The fractional delay in samples.
                     * @return The interpolated delayed sample.
                     */
                    float read(float delay_samples) const noexcept
                    {
                        const float max_d = max_delay();
                        if (delay_samples < 1.0f)
                            delay_samples = 1.0f;
                        if (delay_samples > max_d)
                            delay_samples = max_d;

                        const int id = static_cast<int>(delay_samples);
                        const float f = delay_samples - static_cast<float>(id);

                        // The most recently written sample sits at (write_ - 1). Delay d
                        // means (write_ - 1 - d). Gather the four taps bracketing the
                        // fractional position: delays id-1 (newer), id, id+1, id+2 (older).
                        const std::size_t base = write_ - 1;
                        const float ym1 = buffer_[(base - static_cast<std::size_t>(id - 1)) & mask_];
                        const float y0 = buffer_[(base - static_cast<std::size_t>(id)) & mask_];
                        const float y1 = buffer_[(base - static_cast<std::size_t>(id + 1)) & mask_];
                        const float y2 = buffer_[(base - static_cast<std::size_t>(id + 2)) & mask_];

                        // 3rd-order Lagrange coefficients for the point at fraction f
                        // between y0 (delay id) and y1 (delay id+1).
                        const float c_m1 = -f * (f - 1.0f) * (f - 2.0f) / 6.0f;
                        const float c_0 = (f + 1.0f) * (f - 1.0f) * (f - 2.0f) * 0.5f;
                        const float c_1 = -(f + 1.0f) * f * (f - 2.0f) * 0.5f;
                        const float c_2 = (f + 1.0f) * f * (f - 1.0f) / 6.0f;

                        return c_m1 * ym1 + c_0 * y0 + c_1 * y1 + c_2 * y2;
                    }

                    /**
                     * @brief Processes a block, reading with a delay that ramps across it.
                     *
                     * For each sample the input is written and the output read at a delay
                     * linearly interpolated from @p delay_start (first sample) to
                     * @p delay_end (last). A read rate other than one — i.e. a changing
                     * delay — is the Doppler shift. Safe to call in place (`out == in`).
                     *
                     * @param input       The input block.
                     * @param output      The output block (may alias @p input).
                     * @param frame_count Number of samples.
                     * @param delay_start The read delay at the first sample.
                     * @param delay_end   The read delay at the last sample.
                     */
                    void process_block(const float* input, float* output, int frame_count,
                                       float delay_start, float delay_end) noexcept
                    {
                        const float increment = (frame_count > 1)
                                                    ? (delay_end - delay_start) /
                                                          static_cast<float>(frame_count - 1)
                                                    : 0.0f;
                        float delay = delay_start;
                        for (int i = 0; i < frame_count; ++i)
                        {
                            push(input[i]);
                            output[i] = read(delay);
                            delay += increment;
                        }
                    }

                private:
                    std::vector<float> buffer_;
                    std::size_t mask_ = 0;
                    std::size_t write_ = 0;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
