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
 * the audible set (see `docs/slop/audio_system.md` §8).
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
#include <cstdint>
#include <memory>
#include <vector>

#include <SushiEngine/audio/dsp/air_absorption.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/dsp/spsc_ring.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/occlusion.hpp>
#include <SushiEngine/audio/propagation.hpp>
#include <SushiEngine/audio/spatializer.hpp>
#include <SushiEngine/audio/voice.hpp>
#include <SushiEngine/audio/voice_render_pool.hpp>

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

        /** @brief What a @ref VoiceCommand carried across the control→audio ring does. */
        enum class VoiceCommandKind : std::uint32_t
        {
            Play,
            Stop,
            SetPosition,
            SetGain,
            SetPitch,
            SetOcclusion,
            SetListener
        };

        /**
         * @brief One control-thread intent, drained and applied by the audio thread.
         *
         * A trivially-copyable POD so it crosses the lock-free @ref DSP::SpscRing by value.
         * `source` is a raw owning pointer for @ref VoiceCommandKind::Play — ownership is
         * transferred to the audio thread, which wraps it back in a `unique_ptr`.
         */
        struct VoiceCommand
        {
            VoiceCommandKind kind = VoiceCommandKind::Stop;
            int handle = INVALID_VOICE;      /**< Target voice (Play: the pre-allocated handle). */
            VoiceDescriptor descriptor;      /**< Play parameters. */
            VoiceSource* source = nullptr;   /**< Play source (ownership → audio thread). */
            AudioVec3 v0;                    /**< Position, or listener position. */
            AudioVec3 v1;                    /**< Listener forward. */
            AudioVec3 v2;                    /**< Listener up. */
            float f0 = 0.0f;                 /**< Gain / pitch / obstruction. */
            float f1 = 0.0f;                 /**< Occlusion. */
            float transmission[3] = {1.0f, 1.0f, 1.0f};
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
                    : max_real_(max_real_voices),
                      commands_(static_cast<std::size_t>(pool_capacity) * 4),
                      free_indices_(static_cast<std::size_t>(pool_capacity) * 2),
                      finished_(static_cast<std::size_t>(pool_capacity) * 2),
                      control_gen_(static_cast<std::size_t>(pool_capacity), 1u)
                {
                    slots_.reserve(static_cast<std::size_t>(pool_capacity));
                    for (int i = 0; i < pool_capacity; ++i)
                        slots_.push_back(std::unique_ptr<Slot>(new Slot()));
                    ranking_.reserve(static_cast<std::size_t>(pool_capacity));
                    // Pre-fill the free-slot list for the command-ring path (single-threaded
                    // here, before any audio thread exists).
                    for (int i = 0; i < pool_capacity; ++i)
                        free_indices_.push(i);
                }

                /**
                 * @brief Sets the atmosphere used for the speed of sound and air absorption.
                 * @param atmosphere The temperature, humidity, and pressure.
                 */
                void set_atmosphere(const DSP::Atmosphere& atmosphere) noexcept
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
                        slot->occlusion.prepare(sample_rate_, max_block_);
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
                        if (!slots_[i]->used)
                            return install(i, descriptor, std::move(source));
                    }

                    // Pool full: steal the least-important voice if the newcomer outranks it
                    // (higher priority, or equal priority and louder). Otherwise drop it.
                    const int victim = least_important_voice();
                    if (victim >= 0)
                    {
                        const Slot& v = *slots_[static_cast<std::size_t>(victim)];
                        const bool outranks =
                            descriptor.priority > v.descriptor.priority ||
                            (descriptor.priority == v.descriptor.priority &&
                             descriptor.base_gain > v.audibility);
                        if (outranks)
                            return install(static_cast<std::size_t>(victim), descriptor,
                                           std::move(source));
                    }
                    return INVALID_VOICE;
                }

                /**
                 * @brief Stops and frees a voice immediately.
                 * @param handle The voice handle from @ref play.
                 */
                void stop(int handle) noexcept
                {
                    if (Slot* slot = resolve(handle))
                        free_slot(*slot);
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
                    if (Slot* slot = resolve(handle))
                        slot->descriptor.position = position;
                }

                /**
                 * @brief Sets a voice's per-voice gain target (ramped, click-free).
                 * @param handle The voice handle.
                 * @param gain   The new linear gain target.
                 */
                void set_voice_gain(int handle, float gain) noexcept
                {
                    if (Slot* slot = resolve(handle))
                        slot->gain.set_target(gain);
                }

                /**
                 * @brief Sets a voice's pitch/rate multiplier (1 = natural).
                 * @param handle The voice handle.
                 * @param pitch  The pitch multiplier.
                 */
                void set_voice_pitch(int handle, float pitch) noexcept
                {
                    if (Slot* slot = resolve(handle))
                        if (slot->source)
                            slot->source->set_pitch(pitch);
                }

                /**
                 * @brief Publishes a spatial voice's occlusion state (per wall-clock frame).
                 *
                 * The geometry layer (`acoustic_geometry.hpp`) measures what is blocked; this
                 * hands the result to the voice's @ref OcclusionFilter, which muffles and
                 * attenuates the dry signal and (for occlusion, not obstruction) pulls the
                 * reverb send down. A no-op for an invalid handle.
                 *
                 * @param handle       The voice handle from @ref play.
                 * @param obstruction  Direct-path blockage in [0, 1] (dry only).
                 * @param occlusion    Direct+reverb blockage in [0, 1] (dry and wet).
                 * @param transmission Three-band leak of the blocked path (1 = fully open).
                 */
                void set_voice_occlusion(int handle, float obstruction, float occlusion,
                                         const float transmission[3]) noexcept
                {
                    if (Slot* slot = resolve(handle))
                        slot->occlusion.set_targets(obstruction, occlusion, transmission);
                }

                // --- Concurrency-safe control→audio command API (§0) ----------------------
                //
                // These are the thread-safe counterparts of play/stop/set_voice_*: the
                // control (game/ECS) thread calls them at any time while the audio thread
                // renders, and the audio thread applies them at the top of the next block.
                // Use *either* this API or the direct one on a given manager, not both.

                /**
                 * @brief Enqueues a voice to start (control thread; wait-free).
                 *
                 * Allocates a slot, prepares the source off the audio thread, and posts a Play
                 * command; the audio thread installs it at the next block. Returns the handle
                 * immediately (generational, so it stays valid until the voice ends).
                 *
                 * @param descriptor The voice parameters.
                 * @param source     The mono source (ownership transferred).
                 * @return A handle, or @ref INVALID_VOICE if the pool or ring is full.
                 */
                int enqueue_play(const VoiceDescriptor& descriptor, std::unique_ptr<VoiceSource> source)
                {
                    int index = 0;
                    if (!free_indices_.pop(index))
                        return INVALID_VOICE; // pool exhausted / all voices in flight
                    const std::uint32_t generation =
                        next_generation(control_gen_[static_cast<std::size_t>(index)]);
                    control_gen_[static_cast<std::size_t>(index)] = generation;
                    const int handle = pack_handle(generation, static_cast<std::size_t>(index));
                    if (source)
                        source->prepare(sample_rate_, max_block_); // allocate off the audio thread
                    VoiceCommand command;
                    command.kind = VoiceCommandKind::Play;
                    command.handle = handle;
                    command.descriptor = descriptor;
                    command.source = source.release();
                    if (!commands_.push(command))
                    {
                        delete command.source; // ring full (pathological with 4× sizing)
                        return INVALID_VOICE;
                    }
                    return handle;
                }

                /** @brief Enqueues a voice stop (control thread; wait-free). */
                void enqueue_stop(int handle)
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::Stop;
                    c.handle = handle;
                    commands_.push(c);
                }

                /** @brief Enqueues a spatial voice's new position (control thread). */
                void enqueue_set_position(int handle, const AudioVec3& position)
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::SetPosition;
                    c.handle = handle;
                    c.v0 = position;
                    commands_.push(c);
                }

                /** @brief Enqueues a voice gain target (control thread). */
                void enqueue_set_gain(int handle, float gain)
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::SetGain;
                    c.handle = handle;
                    c.f0 = gain;
                    commands_.push(c);
                }

                /** @brief Enqueues a voice pitch multiplier (control thread). */
                void enqueue_set_pitch(int handle, float pitch)
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::SetPitch;
                    c.handle = handle;
                    c.f0 = pitch;
                    commands_.push(c);
                }

                /** @brief Enqueues a voice's occlusion state (control thread). */
                void enqueue_set_occlusion(int handle, float obstruction, float occlusion,
                                           const float transmission[3])
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::SetOcclusion;
                    c.handle = handle;
                    c.f0 = obstruction;
                    c.f1 = occlusion;
                    c.transmission[0] = transmission[0];
                    c.transmission[1] = transmission[1];
                    c.transmission[2] = transmission[2];
                    commands_.push(c);
                }

                /** @brief Enqueues the listener pose (control thread). */
                void enqueue_set_listener(const AudioVec3& position, const AudioVec3& forward,
                                          const AudioVec3& up)
                {
                    VoiceCommand c;
                    c.kind = VoiceCommandKind::SetListener;
                    c.v0 = position;
                    c.v1 = forward;
                    c.v2 = up;
                    commands_.push(c);
                }

                /**
                 * @brief Drains one ended-voice report (control thread).
                 *
                 * The audio thread reports every voice that finished (a one-shot ran out, was
                 * stolen, or was stopped); the control thread polls these to drop its own
                 * handle bookkeeping. Call in a loop until it returns false.
                 *
                 * @param handle Set to the ended voice's handle on success.
                 * @return True if a report was dequeued.
                 */
                bool poll_finished(int& handle) { return finished_.pop(handle); }

                /**
                 * @brief Installs a worker pool to render voices across CPU cores.
                 *
                 * When set (and enough real voices are active), the per-voice DSP phase of
                 * @ref render runs in parallel; the mixdown stays serial, so the output is
                 * identical to the single-threaded path. Pass nullptr to render on one core.
                 * The pool must outlive the manager's use of it and belong to no other
                 * manager concurrently.
                 *
                 * @param pool The pool, or nullptr for single-threaded rendering.
                 */
                void set_render_pool(VoiceRenderPool* pool) noexcept { pool_ = pool; }

                /** @brief Sets the maximum number of voices rendered per block. */
                void set_max_real_voices(int count) noexcept { max_real_ = count; }

                /**
                 * @brief Sets the HDR loudness window in dB (voices this far below the
                 *        loudest are culled). A large value effectively disables HDR.
                 * @param db The window depth in decibels.
                 */
                void set_hdr_window(double db) noexcept { hdr_window_db_ = db; }

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
                    drain_commands(); // apply queued control intents before ranking/rendering
                    ranking_.clear();
                    float max_audibility = 0.0f;
                    for (std::size_t i = 0; i < slots_.size(); ++i)
                    {
                        Slot& slot = *slots_[i];
                        if (!slot.used)
                            continue;
                        slot.audibility =
                            slot.descriptor.base_gain * slot.descriptor.attenuation(listener_.position);
                        if (slot.audibility > max_audibility)
                            max_audibility = slot.audibility;
                        ranking_.push_back(static_cast<int>(i));
                    }

                    // HDR window: a voice more than `hdr_window_db_` below the loudest one is
                    // masked — culled to virtual even under the real cap — so a loud transient
                    // frees slots from sounds it would drown out (Wwise-HDR style).
                    const float hdr_floor =
                        max_audibility * static_cast<float>(std::pow(10.0, -hdr_window_db_ / 20.0));

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
                        const bool audible = slot.audibility > 1.0e-6f && slot.audibility >= hdr_floor;
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

                    // The list of real voices to render this block.
                    real_indices_.clear();
                    for (std::size_t si = 0; si < slots_.size(); ++si)
                        if (slots_[si]->used && slots_[si]->state == VoiceState::Real)
                            real_indices_.push_back(static_cast<int>(si));
                    const int real_voices = static_cast<int>(real_indices_.size());

                    // Phase 1 — per-voice DSP into each voice's own scratch (no shared state):
                    // source render, fader, and for a spatial voice the propagation and
                    // occlusion. This is the heavy, embarrassingly-parallel work; a render pool
                    // spreads it across cores, otherwise it runs inline. The result is
                    // identical either way — only the serial mixdown in phase 2 touches shared
                    // buffers.
                    auto render_voice = [this, frame_count](int k) noexcept {
                        Slot& slot = *slots_[static_cast<std::size_t>(real_indices_[static_cast<std::size_t>(k)])];
                        float* scratch = slot.scratch.data();
                        slot.render_alive = slot.source && slot.source->render(scratch, frame_count);
                        if (!slot.render_alive)
                            return;
                        float g0 = 0.0f, g1 = 0.0f;
                        slot.gain.advance_block(frame_count, g0, g1);
                        DSP::SIMD::apply_gain_ramp(scratch, frame_count, g0, g1);
                        slot.wet_scale = 1.0f;
                        if (slot.descriptor.spatial)
                        {
                            const float dist = distance(slot.descriptor.position, listener_.position);
                            slot.propagation.process(scratch, scratch, frame_count, dist, atmosphere_,
                                                     slot.descriptor);
                            slot.wet_scale = slot.occlusion.process(scratch, frame_count);
                        }
                    };
                    if (pool_ != nullptr && real_voices >= kParallelVoiceThreshold)
                        pool_->dispatch(real_voices, render_voice);
                    else
                        for (int k = 0; k < real_voices; ++k)
                            render_voice(k);

                    // Phase 2 — serial mixdown: place each rendered voice (ambisonic encode or
                    // stereo pan) and post its reverb send; retire any that finished.
                    for (int k = 0; k < real_voices; ++k)
                    {
                        const std::size_t si = static_cast<std::size_t>(real_indices_[static_cast<std::size_t>(k)]);
                        Slot& slot = *slots_[si];
                        if (!slot.render_alive)
                        {
                            retire_slot(si);
                            continue;
                        }
                        float* scratch = slot.scratch.data();
                        bool placed = false;
                        if (slot.descriptor.spatial && spatializer_ != nullptr)
                        {
                            const float rel_x = slot.descriptor.position.x - listener_.position.x;
                            const float rel_y = slot.descriptor.position.y - listener_.position.y;
                            const float rel_z = slot.descriptor.position.z - listener_.position.z;
                            float hx = 0.0f, hy = 0.0f, hz = 0.0f;
                            head_relative_direction(rel_x, rel_y, rel_z, listener_.forward.x,
                                                    listener_.forward.y, listener_.forward.z,
                                                    listener_.up.x, listener_.up.y, listener_.up.z, hx,
                                                    hy, hz);
                            spatializer_->encode(scratch, frame_count, hx, hy, hz, 1.0f);
                            placed = true;
                        }
                        if (!placed)
                        {
                            float gain_left = 0.0f, gain_right = 0.0f;
                            DSP::SIMD::equal_power_pan(slot.descriptor.pan, gain_left, gain_right);
                            mixer.accumulate(slot.descriptor.bus, scratch, frame_count, gain_left,
                                             gain_right);
                        }
                        if (slot.descriptor.reverb_bus >= 0 && slot.descriptor.reverb_send > 0.0f)
                        {
                            const float s = 0.70710678f * slot.descriptor.reverb_send * slot.wet_scale;
                            mixer.accumulate(slot.descriptor.reverb_bus, scratch, frame_count, s, s);
                        }
                    }

                    // Virtual voices just advance their play position (serial, cheap).
                    for (std::size_t si = 0; si < slots_.size(); ++si)
                    {
                        Slot& slot = *slots_[si];
                        if (!slot.used || slot.state != VoiceState::Virtual)
                            continue;
                        const bool alive = !slot.source || slot.source->advance(frame_count);
                        if (!alive)
                            retire_slot(si);
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
                    const Slot* slot = resolve(handle);
                    return slot != nullptr ? slot->state : VoiceState::Free;
                }

            private:
                struct Slot
                {
                    bool used = false;
                    VoiceState state = VoiceState::Free;
                    std::uint32_t generation = 1; /**< Bumped on each install; makes handles stale-safe. */
                    VoiceDescriptor descriptor;
                    std::unique_ptr<VoiceSource> source;
                    SmoothedValue gain;
                    SourcePropagation propagation;
                    OcclusionFilter occlusion;
                    float audibility = 0.0f;
                    float wet_scale = 1.0f;    /**< Reverb-send scale from occlusion (phase 1 → phase 2). */
                    bool render_alive = true;  /**< Whether the source still had output this block. */
                    std::vector<float> scratch;
                };

                static constexpr int SLOT_BITS = 16;
                static constexpr int SLOT_MASK = (1 << SLOT_BITS) - 1;

                /** @brief Packs a generation + slot index into an opaque voice handle. */
                static int pack_handle(std::uint32_t generation, std::size_t index) noexcept
                {
                    return static_cast<int>((generation << SLOT_BITS) |
                                            (static_cast<std::uint32_t>(index) & SLOT_MASK));
                }

                /** @brief The next non-zero generation (wraps within 15 bits). */
                static std::uint32_t next_generation(std::uint32_t g) noexcept
                {
                    g = (g + 1) & 0x7fffu;
                    return g == 0 ? 1u : g;
                }

                /** @brief Resolves a handle to its live slot, or nullptr if stale/invalid. */
                Slot* resolve(int handle) const noexcept
                {
                    if (handle < 0)
                        return nullptr;
                    const std::size_t index = static_cast<std::size_t>(handle & SLOT_MASK);
                    const std::uint32_t generation = static_cast<std::uint32_t>(handle) >> SLOT_BITS;
                    if (index >= slots_.size())
                        return nullptr;
                    Slot& slot = *slots_[index];
                    if (!slot.used || slot.generation != generation)
                        return nullptr;
                    return &slot;
                }

                bool valid(int handle) const noexcept { return resolve(handle) != nullptr; }

                static void free_slot(Slot& slot) noexcept
                {
                    slot.used = false;
                    slot.state = VoiceState::Free;
                    slot.source.reset();
                }

                /** @brief Frees a slot and reports it back to the control thread (audio side). */
                void retire_slot(std::size_t index) noexcept
                {
                    const int handle = pack_handle(slots_[index]->generation, index);
                    free_slot(*slots_[index]);
                    free_indices_.push(static_cast<int>(index)); // hand the slot back for reuse
                    finished_.push(handle);                       // report the voice ended
                }

                /** @brief Applies every queued control command (audio thread, top of render). */
                void drain_commands()
                {
                    VoiceCommand c;
                    while (commands_.pop(c))
                    {
                        switch (c.kind)
                        {
                            case VoiceCommandKind::Play:
                            {
                                const std::size_t idx = static_cast<std::size_t>(c.handle & SLOT_MASK);
                                const std::uint32_t gen =
                                    static_cast<std::uint32_t>(c.handle) >> SLOT_BITS;
                                if (idx < slots_.size())
                                    install_at(idx, gen, c.descriptor,
                                               std::unique_ptr<VoiceSource>(c.source));
                                else
                                    delete c.source;
                                break;
                            }
                            case VoiceCommandKind::Stop:
                                if (resolve(c.handle) != nullptr)
                                    retire_slot(static_cast<std::size_t>(c.handle & SLOT_MASK));
                                break;
                            case VoiceCommandKind::SetPosition:
                                if (Slot* s = resolve(c.handle))
                                    s->descriptor.position = c.v0;
                                break;
                            case VoiceCommandKind::SetGain:
                                if (Slot* s = resolve(c.handle))
                                    s->gain.set_target(c.f0);
                                break;
                            case VoiceCommandKind::SetPitch:
                                if (Slot* s = resolve(c.handle))
                                    if (s->source)
                                        s->source->set_pitch(c.f0);
                                break;
                            case VoiceCommandKind::SetOcclusion:
                                if (Slot* s = resolve(c.handle))
                                    s->occlusion.set_targets(c.f0, c.f1, c.transmission);
                                break;
                            case VoiceCommandKind::SetListener:
                                listener_.position = c.v0;
                                listener_.forward = c.v1;
                                listener_.up = c.v2;
                                break;
                        }
                    }
                }

                /**
                 * @brief Installs a fresh voice into a slot (also used when stealing).
                 * @return The generational handle for the installed voice.
                 */
                /** @brief Fills a slot's play state (shared by both install paths). */
                void setup_slot(Slot& slot, const VoiceDescriptor& descriptor,
                                std::unique_ptr<VoiceSource> source, bool prepared)
                {
                    slot.used = true;
                    slot.state = VoiceState::Virtual;
                    slot.descriptor = descriptor;
                    slot.source = std::move(source);
                    slot.audibility = descriptor.base_gain;
                    slot.gain.configure(0.005, sample_rate_); // ~5 ms de-zipper
                    slot.gain.snap(descriptor.base_gain);
                    slot.propagation.reset();
                    slot.occlusion.reset();
                    if (slot.source)
                    {
                        if (!prepared) // the enqueue path prepares off the audio thread
                            slot.source->prepare(sample_rate_, max_block_);
                        slot.source->set_pitch(descriptor.pitch);
                    }
                }

                /** @brief Installs a voice (direct API): bumps the generation, returns the handle. */
                int install(std::size_t index, const VoiceDescriptor& descriptor,
                            std::unique_ptr<VoiceSource> source)
                {
                    Slot& slot = *slots_[index];
                    slot.generation = next_generation(slot.generation);
                    setup_slot(slot, descriptor, std::move(source), /*prepared=*/false);
                    return pack_handle(slot.generation, index);
                }

                /** @brief Installs a voice at a control-chosen generation (command-ring path). */
                void install_at(std::size_t index, std::uint32_t generation,
                                const VoiceDescriptor& descriptor, std::unique_ptr<VoiceSource> source)
                {
                    Slot& slot = *slots_[index];
                    slot.generation = generation;
                    setup_slot(slot, descriptor, std::move(source), /*prepared=*/true);
                }

                /**
                 * @brief The used slot with the lowest (priority, audibility), or −1 if none.
                 *
                 * The voice-stealing victim: the least-important active voice. Uses the
                 * audibility from the last @ref render (base gain for a just-started voice).
                 */
                int least_important_voice() const noexcept
                {
                    int victim = -1;
                    for (std::size_t i = 0; i < slots_.size(); ++i)
                    {
                        const Slot& s = *slots_[i];
                        if (!s.used)
                            continue;
                        if (victim < 0)
                        {
                            victim = static_cast<int>(i);
                            continue;
                        }
                        const Slot& v = *slots_[static_cast<std::size_t>(victim)];
                        if (s.descriptor.priority < v.descriptor.priority ||
                            (s.descriptor.priority == v.descriptor.priority &&
                             s.audibility < v.audibility))
                            victim = static_cast<int>(i);
                    }
                    return victim;
                }

                /** @brief Below this many real voices, parallel dispatch is not worth its overhead. */
                static constexpr int kParallelVoiceThreshold = 8;

                std::vector<std::unique_ptr<Slot>> slots_;
                std::vector<int> ranking_;
                std::vector<int> real_indices_; /**< Real-voice slot indices for the two-phase render. */
                VoiceRenderPool* pool_ = nullptr;
                ListenerState listener_;
                BinauralSpatializer* spatializer_ = nullptr;
                DSP::Atmosphere atmosphere_;
                float max_propagation_distance_ = 200.0f;
                int max_real_ = 32;
                double hdr_window_db_ = 60.0;
                int real_count_ = 0;
                double sample_rate_ = 48000.0;
                int max_block_ = 0;

                // The concurrency-safe control→audio path (§0): the control thread pushes
                // intents into `commands_`, the audio thread drains them at the top of
                // `render()`. `free_indices_` hands freed slots back to the control thread
                // (which allocates them for Play), and `finished_` reports voices that ended
                // so the control side can reconcile. `control_gen_` is the control thread's
                // per-slot generation, so the handle it returns from `enqueue_play` matches
                // the one the audio thread installs.
                DSP::SpscRing<VoiceCommand> commands_;
                DSP::SpscRing<int> free_indices_;
                DSP::SpscRing<int> finished_;
                std::vector<std::uint32_t> control_gen_;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
