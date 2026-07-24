/**************************************************************************/
/* nodes.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_DSP_NODES_HPP
#define SUSHIENGINE_AUDIO_DSP_NODES_HPP

/**
 * @file nodes.hpp
 * @brief The S1 built-in graph nodes: sine source, gain, mix, biquad.
 *
 * Enough to stand the block graph up end to end — generate, filter, and mix a sine —
 * and the pattern every later node follows: control-thread setters publish to
 * `std::atomic` targets, which @ref Node::process reads once per block and (for
 * anything audible, like gain) *ramps* toward, so a parameter change never clicks.
 */

#include <atomic>
#include <cmath>

#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/dsp/graph.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /**
             * @brief A sine oscillator source (0 inputs, 1 output).
             *
             * Phase is accumulated in `double` so it stays accurate over long runs.
             * Frequency and amplitude are atomically settable from the control thread
             * and read once per block.
             */
            class SineNode final : public Node
            {
                public:
                    /**
                     * @brief Constructs a sine source.
                     * @param frequency_hz Initial frequency in Hz.
                     * @param amplitude    Initial peak amplitude.
                     */
                    explicit SineNode(float frequency_hz = 440.0f, float amplitude = 1.0f) noexcept
                        : Node(0, 1), frequency_(frequency_hz), amplitude_(amplitude)
                    {
                    }

                    /** @brief Sets the oscillator frequency (Hz), taking effect next block. */
                    void set_frequency(float frequency_hz) noexcept
                    {
                        frequency_.store(frequency_hz, std::memory_order_relaxed);
                    }

                    /** @brief Sets the peak amplitude, taking effect next block. */
                    void set_amplitude(float amplitude) noexcept
                    {
                        amplitude_.store(amplitude, std::memory_order_relaxed);
                    }

                    void prepare(double sample_rate, int max_block_frames) override
                    {
                        (void)max_block_frames;
                        sample_rate_ = sample_rate;
                    }

                    void reset() noexcept override { phase_ = 0.0; }

                    void process(const float* const* inputs, float* const* outputs,
                                 int frame_count) noexcept override
                    {
                        (void)inputs;
                        float* out = outputs[0];
                        const double amplitude = static_cast<double>(amplitude_.load(std::memory_order_relaxed));
                        const double frequency = static_cast<double>(frequency_.load(std::memory_order_relaxed));
                        const double two_pi = 6.28318530717958647692;
                        const double increment = two_pi * frequency / sample_rate_;
                        double phase = phase_;
                        for (int i = 0; i < frame_count; ++i)
                        {
                            out[i] = static_cast<float>(amplitude * std::sin(phase));
                            phase += increment;
                            if (phase >= two_pi)
                                phase -= two_pi;
                        }
                        phase_ = phase;
                    }

                private:
                    std::atomic<float> frequency_;
                    std::atomic<float> amplitude_;
                    double sample_rate_ = 48000.0;
                    double phase_ = 0.0;
            };

            /**
             * @brief A gain stage (1 input, 1 output) with click-free per-block ramping.
             *
             * The applied gain sweeps from its previous value to the atomically-set
             * target across each block, so a level change never steps.
             */
            class GainNode final : public Node
            {
                public:
                    /**
                     * @brief Constructs a gain stage.
                     * @param gain Initial linear gain.
                     */
                    explicit GainNode(float gain = 1.0f) noexcept
                        : Node(1, 1), target_gain_(gain), applied_gain_(gain)
                    {
                    }

                    /** @brief Sets the linear gain target, ramped to over the next block. */
                    void set_gain(float gain) noexcept
                    {
                        target_gain_.store(gain, std::memory_order_relaxed);
                    }

                    void process(const float* const* inputs, float* const* outputs,
                                 int frame_count) noexcept override
                    {
                        const float* in = inputs[0];
                        float* out = outputs[0];
                        const float target = target_gain_.load(std::memory_order_relaxed);
                        const float start = applied_gain_;
                        Simd::copy_scaled(out, in, frame_count, 1.0f);
                        Simd::apply_gain_ramp(out, frame_count, start, target);
                        applied_gain_ = target;
                    }

                private:
                    std::atomic<float> target_gain_;
                    float applied_gain_;
            };

            /**
             * @brief A summing mixer (N inputs, 1 output).
             *
             * Sums all inputs into the output at unity — the graph's fan-in point,
             * since each input port takes a single source.
             */
            class MixNode final : public Node
            {
                public:
                    /**
                     * @brief Constructs a mixer.
                     * @param input_count Number of inputs to sum.
                     */
                    explicit MixNode(int input_count) noexcept : Node(input_count, 1) {}

                    void process(const float* const* inputs, float* const* outputs,
                                 int frame_count) noexcept override
                    {
                        float* out = outputs[0];
                        Simd::fill(out, frame_count, 0.0f);
                        for (int p = 0; p < input_port_count(); ++p)
                            Simd::mix_accumulate(out, inputs[p], frame_count, 1.0f);
                    }
            };

            /**
             * @brief A biquad filter stage (1 input, 1 output) wrapping @ref Biquad.
             *
             * Configure the filter (off the audio thread) via @ref filter, e.g.
             * `node.filter().set_low_pass(800.0, 0.707, sample_rate)`, before the run.
             */
            class BiquadNode final : public Node
            {
                public:
                    BiquadNode() noexcept : Node(1, 1) {}

                    /** @brief The underlying biquad, for configuration. */
                    Biquad& filter() noexcept { return filter_; }

                    void reset() noexcept override { filter_.reset(); }

                    void process(const float* const* inputs, float* const* outputs,
                                 int frame_count) noexcept override
                    {
                        const float* in = inputs[0];
                        float* out = outputs[0];
                        for (int i = 0; i < frame_count; ++i)
                            out[i] = filter_.process(in[i]);
                    }

                private:
                    Biquad filter_;
            };
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
