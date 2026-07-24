/**************************************************************************/
/* mixer.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_MIXER_HPP
#define SUSHIENGINE_AUDIO_MIXER_HPP

/**
 * @file mixer.hpp
 * @brief The mixer: a bus DAG with insert chains and aux sends, evaluated per block.
 *
 * Voices sum into **buses**, not into each other, so an effect runs once on a summed
 * bus buffer (O(bus)) instead of once per voice (see `docs/design/audio_system.md`
 * §8). A @ref Bus has an **insert chain** (effects in series on its own signal), a
 * post-fader **gain**, an **output** route into a parent bus, and any number of **aux
 * sends** that copy its signal at a level into a parallel bus (the reverb-send
 * pattern). @ref MixerGraph orders the buses so every contributor is processed before
 * its consumer — a topological sort over the routing and send edges — and the master
 * bus, the sink, is rendered last.
 *
 * Buses are stereo (planar L/R): a mono voice is placed by a pan into the field, which
 * is what makes a multi-source mix audibly spread. The ambisonic scene bus of §4
 * replaces this stereo path at S4; the routing/insert/send structure here is unchanged
 * by that.
 */

#include <cstddef>
#include <memory>
#include <vector>

#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/parameter.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief Sentinel @ref MixerGraph output meaning "no parent" (the master/root bus). */
        constexpr int NO_BUS = -1;

        /**
         * @brief An in-place stereo effect that runs on a bus's summed signal.
         *
         * Inserts form a series chain on a bus; aux-bus effects (a reverb) are the same
         * interface on a parallel bus. @ref process must be allocation-free.
         */
        class IBusEffect
        {
            public:
                virtual ~IBusEffect() = default;

                /** @brief Allocates/sizes effect state (off the audio thread). */
                virtual void prepare(double sample_rate, int max_block_frames)
                {
                    (void)sample_rate;
                    (void)max_block_frames;
                }

                /** @brief Clears internal state. */
                virtual void reset() noexcept {}

                /**
                 * @brief Processes one stereo block in place.
                 * @param left        Left channel, @p frame_count samples.
                 * @param right       Right channel, @p frame_count samples.
                 * @param frame_count Number of samples.
                 */
                virtual void process(float* left, float* right, int frame_count) noexcept = 0;
        };

        /** @brief A constant-gain insert (a simple trim). */
        class GainBusEffect final : public IBusEffect
        {
            public:
                explicit GainBusEffect(float gain) noexcept : gain_(gain) {}

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    Dsp::Simd::apply_gain(left, frame_count, gain_);
                    Dsp::Simd::apply_gain(right, frame_count, gain_);
                }

            private:
                float gain_;
            };

        /** @brief A biquad EQ insert: one @ref Dsp::Biquad per channel. */
        class BiquadBusEffect final : public IBusEffect
        {
            public:
                /** @brief The left-channel biquad, for configuration. */
                Dsp::Biquad& left() noexcept { return left_; }

                /** @brief The right-channel biquad, for configuration. */
                Dsp::Biquad& right() noexcept { return right_; }

                void reset() noexcept override
                {
                    left_.reset();
                    right_.reset();
                }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    for (int i = 0; i < frame_count; ++i)
                        left[i] = left_.process(left[i]);
                    for (int i = 0; i < frame_count; ++i)
                        right[i] = right_.process(right[i]);
                }

            private:
                Dsp::Biquad left_;
                Dsp::Biquad right_;
            };

        /**
         * @brief A stereo mixer bus DAG.
         *
         * Build buses with @ref add_bus (passing each one's output parent), add
         * @ref add_insert effects and @ref add_aux_send sends, then @ref prepare once.
         * Per block: @ref begin_block to clear accumulators, @ref accumulate each voice
         * contribution, then @ref process to run the graph; @ref master_left /
         * @ref master_right hold the result.
         */
        class MixerGraph
        {
            public:
                /**
                 * @brief Adds a bus.
                 * @param output_bus The parent bus this routes into, or @ref NO_BUS for the
                 *                    master/root (the last @ref NO_BUS bus added becomes master).
                 * @return The new bus id.
                 */
                int add_bus(int output_bus = NO_BUS)
                {
                    std::unique_ptr<Bus> bus(new Bus());
                    bus->output = output_bus;
                    bus->gain.snap(1.0f);
                    buses_.push_back(std::move(bus));
                    const int id = static_cast<int>(buses_.size()) - 1;
                    if (output_bus == NO_BUS)
                        master_ = id;
                    return id;
                }

                /** @brief Sets which bus is the master (final) output. */
                void set_master(int bus_id) noexcept { master_ = bus_id; }

                /** @brief Adds a series insert effect to a bus. */
                void add_insert(int bus_id, std::unique_ptr<IBusEffect> effect)
                {
                    buses_[static_cast<std::size_t>(bus_id)]->inserts.push_back(std::move(effect));
                }

                /**
                 * @brief Adds an aux send: a copy of @p bus_id's signal into @p target_bus.
                 * @param bus_id     The sending bus.
                 * @param target_bus The parallel bus that receives the copy.
                 * @param level      The linear send level.
                 */
                void add_aux_send(int bus_id, int target_bus, float level)
                {
                    buses_[static_cast<std::size_t>(bus_id)]->sends.push_back(Send{target_bus, level});
                }

                /** @brief A bus's post-insert fader (a @ref SmoothedValue), for control. */
                SmoothedValue& bus_gain(int bus_id) noexcept
                {
                    return buses_[static_cast<std::size_t>(bus_id)]->gain;
                }

                /** @brief The number of buses. */
                int bus_count() const noexcept { return static_cast<int>(buses_.size()); }

                /** @brief The master bus id. */
                int master() const noexcept { return master_; }

                /**
                 * @brief Allocates buffers, prepares effects, and computes the bus order.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block that will be processed.
                 */
                void prepare(double sample_rate, int max_block_frames)
                {
                    max_block_ = max_block_frames;
                    for (std::unique_ptr<Bus>& bus : buses_)
                    {
                        bus->left.assign(static_cast<std::size_t>(max_block_), 0.0f);
                        bus->right.assign(static_cast<std::size_t>(max_block_), 0.0f);
                        for (std::unique_ptr<IBusEffect>& effect : bus->inserts)
                            effect->prepare(sample_rate, max_block_frames);
                    }
                    compute_order();
                }

                /** @brief Zeroes every bus's accumulation buffer for a fresh block. */
                void begin_block(int frame_count) noexcept
                {
                    if (frame_count > max_block_)
                        frame_count = max_block_;
                    for (std::unique_ptr<Bus>& bus : buses_)
                    {
                        Dsp::Simd::fill(bus->left.data(), frame_count, 0.0f);
                        Dsp::Simd::fill(bus->right.data(), frame_count, 0.0f);
                    }
                }

                /**
                 * @brief Sums a mono voice into a bus at per-channel gains.
                 * @param bus_id      The target bus.
                 * @param mono        The mono voice buffer.
                 * @param frame_count Number of samples.
                 * @param gain_left   Gain into the left channel.
                 * @param gain_right  Gain into the right channel.
                 */
                void accumulate(int bus_id, const float* mono, int frame_count,
                                float gain_left, float gain_right) noexcept
                {
                    Bus& bus = *buses_[static_cast<std::size_t>(bus_id)];
                    Dsp::Simd::mix_accumulate(bus.left.data(), mono, frame_count, gain_left);
                    Dsp::Simd::mix_accumulate(bus.right.data(), mono, frame_count, gain_right);
                }

                /**
                 * @brief Runs the bus graph: inserts, fader, routing, and aux sends, in order.
                 * @param frame_count Number of samples (clamped to the prepared maximum).
                 */
                void process(int frame_count) noexcept
                {
                    if (frame_count > max_block_)
                        frame_count = max_block_;

                    for (int id : order_)
                    {
                        Bus& bus = *buses_[static_cast<std::size_t>(id)];
                        float* left = bus.left.data();
                        float* right = bus.right.data();

                        for (std::unique_ptr<IBusEffect>& effect : bus.inserts)
                            effect->process(left, right, frame_count);

                        float g0 = 0.0f, g1 = 0.0f;
                        bus.gain.advance_block(frame_count, g0, g1);
                        Dsp::Simd::apply_gain_ramp(left, frame_count, g0, g1);
                        Dsp::Simd::apply_gain_ramp(right, frame_count, g0, g1);

                        if (bus.output != NO_BUS && bus.output != id)
                        {
                            Bus& parent = *buses_[static_cast<std::size_t>(bus.output)];
                            Dsp::Simd::mix_accumulate(parent.left.data(), left, frame_count, 1.0f);
                            Dsp::Simd::mix_accumulate(parent.right.data(), right, frame_count, 1.0f);
                        }

                        for (const Send& send : bus.sends)
                        {
                            if (send.target == id)
                                continue;
                            Bus& target = *buses_[static_cast<std::size_t>(send.target)];
                            Dsp::Simd::mix_accumulate(target.left.data(), left, frame_count, send.level);
                            Dsp::Simd::mix_accumulate(target.right.data(), right, frame_count, send.level);
                        }
                    }
                }

                /** @brief The master bus's left channel from the last @ref process. */
                const float* master_left() const noexcept
                {
                    return buses_[static_cast<std::size_t>(master_)]->left.data();
                }

                /** @brief The master bus's right channel from the last @ref process. */
                const float* master_right() const noexcept
                {
                    return buses_[static_cast<std::size_t>(master_)]->right.data();
                }

                /** @brief The evaluation order computed by @ref prepare (for diagnostics/tests). */
                const std::vector<int>& order() const noexcept { return order_; }

            private:
                struct Send
                {
                    int target;
                    float level;
                };

                struct Bus
                {
                    int output = NO_BUS;
                    SmoothedValue gain;
                    std::vector<std::unique_ptr<IBusEffect>> inserts;
                    std::vector<Send> sends;
                    std::vector<float> left;
                    std::vector<float> right;
                };

                void compute_order()
                {
                    const std::size_t count = buses_.size();
                    std::vector<int> in_degree(count, 0);
                    std::vector<std::vector<int>> adjacency(count);

                    auto add_edge = [&](int from, int to) {
                        if (from == to || to < 0)
                            return;
                        adjacency[static_cast<std::size_t>(from)].push_back(to);
                        ++in_degree[static_cast<std::size_t>(to)];
                    };

                    for (std::size_t id = 0; id < count; ++id)
                    {
                        add_edge(static_cast<int>(id), buses_[id]->output);
                        for (const Send& send : buses_[id]->sends)
                            add_edge(static_cast<int>(id), send.target);
                    }

                    order_.clear();
                    order_.reserve(count);
                    std::vector<int> ready;
                    for (std::size_t id = 0; id < count; ++id)
                        if (in_degree[id] == 0)
                            ready.push_back(static_cast<int>(id));

                    while (!ready.empty())
                    {
                        const int id = ready.back();
                        ready.pop_back();
                        order_.push_back(id);
                        for (int m : adjacency[static_cast<std::size_t>(id)])
                            if (--in_degree[static_cast<std::size_t>(m)] == 0)
                                ready.push_back(m);
                    }

                    if (order_.size() != count)
                        for (std::size_t id = 0; id < count; ++id)
                            if (in_degree[id] > 0)
                                order_.push_back(static_cast<int>(id));
                }

                std::vector<std::unique_ptr<Bus>> buses_;
                std::vector<int> order_;
                int master_ = 0;
                int max_block_ = 0;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
