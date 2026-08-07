/**************************************************************************/
/* audio_scene.hpp                                                        */
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

#ifndef SUSHIENGINE_AUDIO_AUDIO_SCENE_HPP
#define SUSHIENGINE_AUDIO_AUDIO_SCENE_HPP

/**
 * @file audio_scene.hpp
 * @brief The control-plane bridge: a world snapshot of emitters/listener → live voices.
 *
 * This is "*the control-plane structure the voice manager reads*" of the design (§9 of
 * `docs/design/audio_system.md`). The ECS world produces a @ref SceneSnapshot each
 * wall-clock frame — the listener's facing and every audible emitter's **listener-local**
 * position and play parameters — and @ref AudioScene reconciles that against the live
 * voice pool: it starts a voice when an emitter first appears, updates the position and
 * gain of one that persists (the frame-to-frame position change is what the propagation
 * model turns into Doppler), and stops one whose emitter went silent or was destroyed.
 *
 * The deliberate seam here is Dependency-Inversion: **this file knows nothing about the
 * ECS**. It works in plain float, listener-local coordinates, so it is unit-testable
 * with a hand-built snapshot and never drags the SushiRuntime into the audio module.
 * The ECS half — walking the `World`, converting double `WorldVector3` positions to
 * listener-local float, and reading the Orientation quaternion into the listener frame —
 * lives one layer up in `sim/audio_extract.hpp`, which *can* see the runtime.
 *
 * Which concrete sound an emitter's `sound` id maps to is resolved by an injected
 * @ref IEmitterSourceFactory — a tone or sample today, a bank **event** at S8 — so the
 * asset/event system slots in behind this seam without touching the reconciliation.
 */

#include <cstdint>
#include <iterator>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <SushiEngine/audio/reverb.hpp>
#include <SushiEngine/audio/voice.hpp>
#include <SushiEngine/audio/voice_manager.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief Resolves an emitter's `sound` id to a fresh mono source.
         *
         * The injection point for the asset/event system. @ref AudioScene calls this the
         * first frame an emitter becomes audible; returning `nullptr` means "no source
         * this frame" (the scene simply retries next frame). At S8 a bank/event resolver
         * implements this; a demo or test maps an id to a `ToneSource`/`BufferSource`.
         */
        class IEmitterSourceFactory
        {
            public:
                virtual ~IEmitterSourceFactory() = default;

                /**
                 * @brief Creates the source for a sound id.
                 * @param sound_id The emitter's `sound` field.
                 * @return An owned mono source, or nullptr if none is available.
                 */
                virtual std::unique_ptr<VoiceSource> create(std::uint32_t sound_id) = 0;
        };

        /** @brief One emitter's state in a @ref SceneSnapshot (listener-local float). */
        struct EmitterSnapshot
        {
            std::uint64_t key = 0;       /**< Stable per-entity identity (index + generation). */
            std::uint32_t sound = 0;     /**< Sound/event id the factory resolves. */
            AudioVec3 position;          /**< Position **relative to the listener** (metres). */
            float gain = 1.0f;           /**< Linear base gain. */
            float priority = 0.0f;       /**< Voice-manager real-slot priority. */
            int bus = 0;                 /**< Target mixer bus id. */
            bool spatial = true;         /**< Whether distance/Doppler apply. */
            float min_distance = 1.0f;   /**< Full gain within this radius. */
            float max_distance = 100.0f; /**< Silent/cullable beyond this radius. */
            DistanceModel model = DistanceModel::Linear; /**< Rolloff law. */
            float rolloff = 1.0f;        /**< Rolloff factor. */
            float doppler_scale = 1.0f;  /**< Doppler exaggeration. */
            bool playing = true;         /**< False stops the voice. */

            /**
             * @brief A count whose change restarts the sound from its beginning.
             *
             * The one-shot pulse, expressed as a number because a snapshot carries one
             * value per frame and an edge written and cleared between two frames is an
             * edge nothing here observes. Unchanged means "whatever this voice is
             * doing, let it finish" — including having already finished, which is what
             * keeps a non-looping sound from restarting forever.
             */
            std::uint32_t trigger = 0;

            int reverb_bus = -1;         /**< Reverb aux-send target bus (−1 = no send). */
            float reverb_send = 0.0f;    /**< Reverb aux-send level in [0, 1]. */
            float obstruction = 0.0f;    /**< Direct-path blockage in [0, 1] (dry only). */
            float occlusion = 0.0f;      /**< Direct+reverb blockage in [0, 1] (dry and wet). */
            float transmission[3] = {1.0f, 1.0f, 1.0f}; /**< Three-band leak of the blocked path. */
        };

        /**
         * @brief One wall-clock frame of the audible world, in listener-local float space.
         *
         * The listener sits at the origin (positions are already listener-relative, the
         * double subtraction done during extraction), so only its facing needs carrying.
         * @ref reverb, when @ref has_reverb is set, is the active zone's I3DL2 parameters.
         */
        struct SceneSnapshot
        {
            AudioVec3 listener_forward{1.0f, 0.0f, 0.0f}; /**< Listener facing (same frame as positions). */
            AudioVec3 listener_up{0.0f, 0.0f, 1.0f};      /**< Listener up. */
            std::vector<EmitterSnapshot> emitters;        /**< Every audible emitter this frame. */
            bool has_reverb = false;                      /**< Whether a reverb zone is active. */
            I3DL2Reverb reverb;                           /**< The active zone's reverb (if any). */
        };

        /**
         * @brief Reconciles a per-frame @ref SceneSnapshot against the live voice pool.
         *
         * Construct with the voice manager it drives and a source factory; optionally
         * give it the reverb effect a @ref ReverbZone should steer. Call @ref apply once
         * per wall-clock frame with the freshly-extracted snapshot.
         */
        class AudioScene
        {
            public:
                /**
                 * @brief Builds the scene bridge.
                 * @param voices  The voice manager voices are started/stopped/moved in.
                 * @param factory The resolver from a `sound` id to a source (borrowed).
                 */
                AudioScene(VoiceManager& voices, IEmitterSourceFactory& factory) noexcept
                    : voices_(voices), factory_(factory)
                {
                }

                /**
                 * @brief Installs the reverb effect a @ref ReverbZone drives (optional).
                 * @param reverb The aux-bus reverb to push zone I3DL2 parameters into, or nullptr.
                 */
                void set_reverb(IReverb* reverb) noexcept { reverb_ = reverb; }

                /**
                 * @brief Reconciles one frame: start/update/stop voices, set the listener.
                 * @param snapshot The wall-clock world snapshot (listener-local float).
                 */
                void apply(const SceneSnapshot& snapshot)
                {
                    // The listener is the origin of the snapshot frame; only its facing
                    // varies. Positions are already listener-relative.
                    voices_.set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f},
                                                       snapshot.listener_forward,
                                                       snapshot.listener_up});

                    seen_.clear();
                    for (const EmitterSnapshot& e : snapshot.emitters)
                    {
                        seen_.insert(e.key);
                        auto it = voice_of_.find(e.key);
                        const bool live = it != voice_of_.end();

                        if (!e.playing)
                        {
                            if (live)
                            {
                                voices_.stop(it->second);
                                voice_of_.erase(it);
                            }
                            trigger_of_.erase(e.key);
                            continue;
                        }

                        // A bumped count restarts the sound from its beginning, whatever
                        // the voice was doing — which is the only way to hear the same
                        // impact sound twice on one emitter, since a still-live voice
                        // below merely has its gain updated.
                        const auto seen_trigger = trigger_of_.find(e.key);
                        const bool retriggered =
                            seen_trigger != trigger_of_.end() && seen_trigger->second != e.trigger;
                        trigger_of_[e.key] = e.trigger;
                        if (retriggered)
                        {
                            if (live)
                            {
                                voices_.stop(it->second);
                                voice_of_.erase(it);
                            }
                            finished_.erase(e.key);
                            start_voice(e);
                            continue;
                        }

                        // Finished and not retriggered: leave it alone. This is the whole
                        // of the fix — an emitter still marked playing after its sound
                        // ended used to fall through to the start below on the very next
                        // frame, which put every non-looping voice on a permanent loop.
                        if (finished_.find(e.key) != finished_.end())
                            continue;

                        if (!live)
                        {
                            start_voice(e);
                            continue;
                        }

                        // A one-shot has freed itself. Remember that, so the branch above
                        // catches it from now until something bumps the count.
                        if (voices_.state_of(it->second) == VoiceState::Free)
                        {
                            voice_of_.erase(it);
                            finished_.insert(e.key);
                            continue;
                        }

                        voices_.set_voice_position(it->second, e.position);
                        voices_.set_voice_gain(it->second, e.gain);
                        voices_.set_voice_occlusion(it->second, e.obstruction, e.occlusion,
                                                    e.transmission);
                    }

                    // Any mapped emitter absent from this snapshot was destroyed — stop it.
                    for (auto it = voice_of_.begin(); it != voice_of_.end();)
                    {
                        if (seen_.find(it->first) == seen_.end())
                        {
                            voices_.stop(it->second);
                            it = voice_of_.erase(it);
                        }
                        else
                        {
                            ++it;
                        }
                    }

                    // The same sweep for the two side tables. A destroyed emitter that
                    // left a `finished_` mark behind would silence the next entity to
                    // reuse its key, and one that left a trigger count behind would make
                    // that entity's first snapshot look like a retrigger.
                    for (auto it = finished_.begin(); it != finished_.end();)
                        it = seen_.find(*it) == seen_.end() ? finished_.erase(it) : std::next(it);
                    for (auto it = trigger_of_.begin(); it != trigger_of_.end();)
                        it = seen_.find(it->first) == seen_.end() ? trigger_of_.erase(it)
                                                                  : std::next(it);

                    // Steer the reverb aux bus from the active zone, on change only.
                    if (reverb_ != nullptr && snapshot.has_reverb)
                    {
                        if (!reverb_set_ || !same_reverb(active_reverb_, snapshot.reverb))
                        {
                            reverb_->set_parameters(snapshot.reverb);
                            active_reverb_ = snapshot.reverb;
                            reverb_set_ = true;
                        }
                    }
                }

                /** @brief Stops every voice this scene owns and clears the mapping. */
                void clear() noexcept
                {
                    for (auto& entry : voice_of_)
                        voices_.stop(entry.second);
                    voice_of_.clear();
                    // With these left behind, a scene reloaded into the same keys would
                    // come back with half its one-shots already marked spent.
                    finished_.clear();
                    trigger_of_.clear();
                }

                /** @brief The number of emitters currently mapped to a live voice. */
                std::size_t voice_count() const noexcept { return voice_of_.size(); }

                /**
                 * @brief The voice handle an emitter key maps to, or @ref INVALID_VOICE.
                 * @param key The emitter's stable key.
                 */
                int voice_for(std::uint64_t key) const noexcept
                {
                    auto it = voice_of_.find(key);
                    return it == voice_of_.end() ? INVALID_VOICE : it->second;
                }

            private:
                void start_voice(const EmitterSnapshot& e)
                {
                    std::unique_ptr<VoiceSource> source = factory_.create(e.sound);
                    if (!source)
                        return; // no source available this frame; retried next frame

                    VoiceDescriptor d;
                    d.base_gain = e.gain;
                    d.priority = e.priority;
                    d.bus = e.bus;
                    d.pan = 0.0f;
                    d.spatial = e.spatial;
                    d.position = e.position;
                    d.min_distance = e.min_distance;
                    d.max_distance = e.max_distance;
                    d.model = e.model;
                    d.rolloff = e.rolloff;
                    d.doppler_scale = e.doppler_scale;
                    d.reverb_bus = e.reverb_bus;
                    d.reverb_send = e.reverb_send;

                    const int handle = voices_.play(d, std::move(source));
                    if (handle != INVALID_VOICE)
                    {
                        voice_of_.emplace(e.key, handle);
                        voices_.set_voice_occlusion(handle, e.obstruction, e.occlusion, e.transmission);
                    }
                    // else the pool was full; retried next frame.
                }

                static bool same_reverb(const I3DL2Reverb& a, const I3DL2Reverb& b) noexcept
                {
                    return a.room == b.room && a.room_hf == b.room_hf &&
                           a.decay_time == b.decay_time && a.decay_hf_ratio == b.decay_hf_ratio &&
                           a.reverb == b.reverb && a.reverb_delay == b.reverb_delay &&
                           a.diffusion == b.diffusion && a.density == b.density &&
                           a.hf_reference == b.hf_reference && a.wet_dry_mix == b.wet_dry_mix;
                }

                VoiceManager& voices_;
                IEmitterSourceFactory& factory_;
                IReverb* reverb_ = nullptr;
                std::unordered_map<std::uint64_t, int> voice_of_;
                std::unordered_set<std::uint64_t> seen_;

                /** @brief The trigger count last seen per emitter; a change restarts it. */
                std::unordered_map<std::uint64_t, std::uint32_t> trigger_of_;

                /**
                 * @brief Emitters whose one-shot has played out.
                 *
                 * Held separately from @ref voice_of_ rather than as a sentinel inside
                 * it, because "has no voice" and "had a voice that finished" are
                 * different states with opposite meanings: the first should start a
                 * sound and the second must not.
                 */
                std::unordered_set<std::uint64_t> finished_;
                I3DL2Reverb active_reverb_;
                bool reverb_set_ = false;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
