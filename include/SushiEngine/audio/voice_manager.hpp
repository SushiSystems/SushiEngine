/**************************************************************************/
/* voice_manager.hpp                                                     */
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

#ifndef SUSHIENGINE_AUDIO_VOICE_MANAGER_HPP
#define SUSHIENGINE_AUDIO_VOICE_MANAGER_HPP

/**
 * @file voice_manager.hpp
 * @brief The voice manager: the virtual/real split and prioritized multi-source mix.
 *
 * A game may want hundreds of sounds at once; only a bounded set can afford the full
 * render pipeline. The voice manager holds a fixed pool and, each block, computes an
 * **effective audibility** per active voice (base gain × distance attenuation),
 * ranks the active set by `(priority, audibility)`, and promotes the top **real**
 * voices (capped) to full rendering while the rest go **virtual** — position
 * bookkeeping only, at ~no cost, ready to resume seamlessly if they climb back into
 * the audible set (see `docs/design/audio_system.md` §8).
 *
 * Real voices render mono, ramp their gain, and pan into their target mixer bus, so N
 * sources collapse into a handful of bus buffers before any effect runs. Occlusion,
 * the HDR window, and a separate decode cap layer onto this ranking in later phases;
 * voice stealing when the pool itself is exhausted is a later refinement (today a
 * @ref play beyond capacity returns @ref INVALID_VOICE).
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <SushiEngine/audio/dsp/air_absorption.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/propagation.hpp>
#include <SushiEngine/audio/spatializer.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief Returned by @ref VoiceManager::play when no pool slot is free. */
        constexpr int INVALID_VOICE = -1;

        /** @brief The listener the voice manager attenuates and spatializes against. */
        struct ListenerState
        {
            AudioVec3 position;
            AudioVec3 forward{1.0f, 0.0f, 0.0f}; /**< Facing direction (head frame front). */
            AudioVec3 up{0.0f, 0.0f, 1.0f};      /**< Up direction (head frame up). */
        };

        /**
         * @brief A fixed-pool manager that renders the audible subset of active voices.
         *
         * Construct with the pool size and the real-voice cap, @ref prepare it, then
         * @ref play voices (control side) and call @ref render once per block (audio
         * side) to fold the real voices into a @ref MixerGraph.
         */
        class VoiceManager
        {
            public:
                /**
                 * @brief Builds the manager and its voice pool.
                 * @param pool_capacity   Maximum simultaneously-active voices (real + virtual).
                 * @param max_real_voices Maximum voices rendered (the rest are virtualized).
                 */
                VoiceManager(int pool_capacity, int max_real_voices)
                    : max_real_(max_real_voices)
                {
                    slots_.reserve(static_cast<std::size_t>(pool_capacity));
                    for (int i = 0; i < pool_capacity; ++i)
                        slots_.push_back(std::unique_ptr<Slot>(new Slot()));
                    ranking_.reserve(static_cast<std::size_t>(pool_capacity));
                }

                /**
                 * @brief Sets the atmosphere used for the speed of sound and air absorption.
                 * @param atmosphere The temperature, humidity, and pressure.
                 */
                void set_atmosphere(const Dsp::Atmosphere& atmosphere) noexcept
                {
                    atmosphere_ = atmosphere;
                }

                /**
                 * @brief Sets the farthest distance the propagation delay lines are sized for.
                 *
                 * Call before @ref prepare; it determines the per-voice delay-buffer size
                 * (distance / c · sample-rate). A source beyond it still plays, but its
                 * propagation delay saturates (it is silent at that range regardless).
                 *
                 * @param meters The maximum modelled propagation distance.
                 */
                void set_max_propagation_distance(float meters) noexcept
                {
                    max_propagation_distance_ = meters;
                }

                /**
                 * @brief Allocates per-voice scratch and propagation state and stores the run format.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block that will be rendered.
                 */
                void prepare(double sample_rate, int max_block_frames)
                {
                    sample_rate_ = sample_rate;
                    max_block_ = max_block_frames;
                    // Size delay lines against the coldest plausible air (slowest sound =
                    // longest delay) so the buffer always spans the modelled distance.
                    const float slowest_c = 331.3f;
                    const int max_delay_samples =
                        static_cast<int>((max_propagation_distance_ / slowest_c) *
                                         static_cast<float>(sample_rate_)) + 1;
                    for (std::unique_ptr<Slot>& slot : slots_)
                    {
                        slot->scratch.assign(static_cast<std::size_t>(max_block_), 0.0f);
                        slot->propagation.prepare(sample_rate_, max_block_, max_delay_samples);
                    }
                }

                /**
                 * @brief Starts a voice (control side).
                 * @param descriptor The voice's gain, priority, bus, pan, and spatial params.
                 * @param source     The mono source to render (ownership transferred).
                 * @return A voice handle, or @ref INVALID_VOICE if the pool is full.
                 */
                int play(const VoiceDescriptor& descriptor, std::unique_ptr<VoiceSource> source)
                {
                    for (std::size_t i = 0; i < slots_.size(); ++i)
                    {
                        Slot& slot = *slots_[i];
                        if (slot.used)
                            continue;
                        slot.used = true;
                        slot.state = VoiceState::Virtual;
                        slot.descriptor = descriptor;
                        slot.source = std::move(source);
                        slot.audibility = 0.0f;
                        slot.gain.configure(0.005, sample_rate_); // ~5 ms de-zipper
                        slot.gain.snap(descriptor.base_gain);
                        slot.propagation.reset();
                        if (slot.source)
                            slot.source->prepare(sample_rate_, max_block_);
                        return static_cast<int>(i);
                    }
                    return INVALID_VOICE;
                }

                /**
                 * @brief Stops and frees a voice immediately.
                 * @param handle The voice handle from @ref play.
                 */
                void stop(int handle) noexcept
                {
                    if (!valid(handle))
                        return;
                    free_slot(*slots_[static_cast<std::size_t>(handle)]);
                }

                /** @brief Sets the listener used for distance attenuation. */
                void set_listener(const ListenerState& listener) noexcept { listener_ = listener; }

                /**
                 * @brief Updates a spatial voice's world position (per wall-clock snapshot).
                 *
                 * The way a moving emitter drives Doppler: the game updates the position
                 * each block, and the change in source-to-listener distance between blocks
                 * is what the propagation delay turns into a pitch shift. A no-op for an
                 * invalid handle.
                 *
                 * @param handle   The voice handle from @ref play.
                 * @param position The new world position.
                 */
                void set_voice_position(int handle, const AudioVec3& position) noexcept
                {
                    if (valid(handle))
                        slots_[static_cast<std::size_t>(handle)]->descriptor.position = position;
                }

                /**
                 * @brief Sets a voice's per-voice gain target (ramped, click-free).
                 * @param handle The voice handle.
                 * @param gain   The new linear gain target.
                 */
                void set_voice_gain(int handle, float gain) noexcept
                {
                    if (valid(handle))
                        slots_[static_cast<std::size_t>(handle)]->gain.set_target(gain);
                }

                /** @brief Sets the maximum number of voices rendered per block. */
                void set_max_real_voices(int count) noexcept { max_real_ = count; }

                /**
                 * @brief Installs the binaural spatializer spatial voices encode into.
                 *
                 * When set, a spatial real voice is encoded into the ambisonic scene bus
                 * (head-relative direction) instead of being stereo-panned; non-spatial
                 * voices always pan into their mixer bus. Pass nullptr to fall back to the
                 * stereo-pan placement for spatial voices too.
                 *
                 * @param spatializer The spatializer, or nullptr for stereo panning.
                 */
                void set_spatializer(BinauralSpatializer* spatializer) noexcept
                {
                    spatializer_ = spatializer;
                }

                /**
                 * @brief Ranks the active voices and folds the real ones into @p mixer.
                 * @param mixer       The bus graph real voices accumulate into.
                 * @param frame_count Number of samples this block.
                 */
                void render(MixerGraph& mixer, int frame_count) noexcept
                {
                    ranking_.clear();
                    for (std::size_t i = 0; i < slots_.size(); ++i)
                    {
                        Slot& slot = *slots_[i];
                        if (!slot.used)
                            continue;
                        slot.audibility =
                            slot.descriptor.base_gain * slot.descriptor.attenuation(listener_.position);
                        ranking_.push_back(static_cast<int>(i));
                    }

                    // Primary key: priority; secondary: audibility. Highest first.
                    std::sort(ranking_.begin(), ranking_.end(), [this](int a, int b) {
                        const Slot& sa = *slots_[static_cast<std::size_t>(a)];
                        const Slot& sb = *slots_[static_cast<std::size_t>(b)];
                        if (sa.descriptor.priority != sb.descriptor.priority)
                            return sa.descriptor.priority > sb.descriptor.priority;
                        return sa.audibility > sb.audibility;
                    });

                    real_count_ = 0;
                    for (int rank = 0; rank < static_cast<int>(ranking_.size()); ++rank)
                    {
                        Slot& slot = *slots_[static_cast<std::size_t>(ranking_[static_cast<std::size_t>(rank)])];
                        const bool audible = slot.audibility > 1.0e-6f;
                        if (real_count_ < max_real_ && audible)
                        {
                            slot.state = VoiceState::Real;
                            ++real_count_;
                        }
                        else
                        {
                            slot.state = VoiceState::Virtual;
                        }
                    }

                    for (std::unique_ptr<Slot>& slot_ptr : slots_)
                    {
                        Slot& slot = *slot_ptr;
                        if (!slot.used)
                            continue;

                        if (slot.state == VoiceState::Real)
                        {
                            float* scratch = slot.scratch.data();
                            const bool alive = slot.source && slot.source->render(scratch, frame_count);
                            if (!alive)
                            {
                                free_slot(slot);
                                continue;
                            }

                            // Per-voice base gain first (the fader), then, for a spatial
                            // voice, propagation (delay/Doppler, air absorption, and the
                            // distance-attenuation gain) before it is placed.
                            float g0 = 0.0f, g1 = 0.0f;
                            slot.gain.advance_block(frame_count, g0, g1);
                            Dsp::Simd::apply_gain_ramp(scratch, frame_count, g0, g1);

                            bool placed = false;
                            if (slot.descriptor.spatial)
                            {
                                const float dist =
                                    distance(slot.descriptor.position, listener_.position);
                                slot.propagation.process(scratch, scratch, frame_count, dist,
                                                         atmosphere_, slot.descriptor);

                                if (spatializer_ != nullptr)
                                {
                                    // Encode into the ambisonic scene bus in head-relative
                                    // coordinates, so a head turn re-aims the source.
                                    const float rel_x = slot.descriptor.position.x - listener_.position.x;
                                    const float rel_y = slot.descriptor.position.y - listener_.position.y;
                                    const float rel_z = slot.descriptor.position.z - listener_.position.z;
                                    float hx = 0.0f, hy = 0.0f, hz = 0.0f;
                                    head_relative_direction(rel_x, rel_y, rel_z,
                                                            listener_.forward.x, listener_.forward.y,
                                                            listener_.forward.z, listener_.up.x,
                                                            listener_.up.y, listener_.up.z, hx, hy, hz);
                                    spatializer_->encode(scratch, frame_count, hx, hy, hz, 1.0f);
                                    placed = true;
                                }
                            }

                            if (!placed)
                            {
                                float gain_left = 0.0f, gain_right = 0.0f;
                                Dsp::Simd::equal_power_pan(slot.descriptor.pan, gain_left, gain_right);
                                mixer.accumulate(slot.descriptor.bus, scratch, frame_count,
                                                 gain_left, gain_right);
                            }
                        }
                        else
                        {
                            const bool alive = !slot.source || slot.source->advance(frame_count);
                            if (!alive)
                                free_slot(slot);
                        }
                    }
                }

                /** @brief The number of voices rendered in the last @ref render (real count). */
                int real_count() const noexcept { return real_count_; }

                /** @brief The number of active voices (real + virtual). */
                int active_count() const noexcept
                {
                    int count = 0;
                    for (const std::unique_ptr<Slot>& slot : slots_)
                        if (slot->used)
                            ++count;
                    return count;
                }

                /** @brief The pool capacity. */
                int capacity() const noexcept { return static_cast<int>(slots_.size()); }

                /**
                 * @brief The current state of a voice.
                 * @param handle The voice handle.
                 * @return Its @ref VoiceState, or @ref VoiceState::Free if the handle is not active.
                 */
                VoiceState state_of(int handle) const noexcept
                {
                    if (!valid(handle))
                        return VoiceState::Free;
                    return slots_[static_cast<std::size_t>(handle)]->state;
                }

            private:
                struct Slot
                {
                    bool used = false;
                    VoiceState state = VoiceState::Free;
                    VoiceDescriptor descriptor;
                    std::unique_ptr<VoiceSource> source;
                    SmoothedValue gain;
                    SourcePropagation propagation;
                    float audibility = 0.0f;
                    std::vector<float> scratch;
                };

                bool valid(int handle) const noexcept
                {
                    return handle >= 0 && static_cast<std::size_t>(handle) < slots_.size() &&
                           slots_[static_cast<std::size_t>(handle)]->used;
                }

                static void free_slot(Slot& slot) noexcept
                {
                    slot.used = false;
                    slot.state = VoiceState::Free;
                    slot.source.reset();
                }

                std::vector<std::unique_ptr<Slot>> slots_;
                std::vector<int> ranking_;
                ListenerState listener_;
                BinauralSpatializer* spatializer_ = nullptr;
                Dsp::Atmosphere atmosphere_;
                float max_propagation_distance_ = 200.0f;
                int max_real_ = 32;
                int real_count_ = 0;
                double sample_rate_ = 48000.0;
                int max_block_ = 0;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
