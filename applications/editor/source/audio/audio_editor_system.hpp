/**************************************************************************/
/* audio_editor_system.hpp                                               */
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

#pragma once

/**
 * @file audio_editor_system.hpp
 * @brief The editor's live audio system: makes the authored world audible in-editor.
 *
 * The editor half of the S9 authoring surface (§11 of `docs/slop/audio_system.md`). It
 * owns a live `Audio::AudioEngine` and an SDL output device, and each frame projects the
 * world's audio components — every `AudioEmitter`, the `ReverbZone` the listener stands
 * in, read through the `IWorldEditor` seam — into the engine's voice pool, with the Scene
 * camera as the listener. So placing an Audio Emitter and moving the camera is *heard*,
 * exactly as a designer expects, without a game running.
 *
 * A default `IEmitterSourceFactory` turns each emitter's `sound` id into a distinct looping
 * tone (there is no bank in the editor yet, S8's bank plugs in behind this same seam), so
 * the authoring loop is complete today. The engine also publishes its live-profiler
 * telemetry, which the Audio Mixer and Audio Profiler panels read.
 *
 * This is not a SYCL translation unit and holds no runtime dependency — it links
 * `sushiengine_audio_backend` (the SDL device) exactly as the editor already links
 * `sushiengine_input_backend`.
 */

#include <cstdint>
#include <memory>

#include <SushiEngine/audio/audio.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/simulation/simulation.hpp>
#include <sdl/sdl_audio_device.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Owns a live audio engine and drives it from the edited world each frame.
         *
         * Construct once (it builds the mixer topology and a per-zone reverb aux bus and
         * prepares the engine), call @ref set_enabled to open/close the output device, and
         * call @ref update every frame with the listener (Scene camera) pose. The panels
         * read @ref profile, the mixer accessors, and drive @ref preview.
         */
        class AudioEditorSystem
        {
            public:
                AudioEditorSystem();
                ~AudioEditorSystem();

                AudioEditorSystem(const AudioEditorSystem&) = delete;
                AudioEditorSystem& operator=(const AudioEditorSystem&) = delete;

                /**
                 * @brief Opens or closes the output device (turns editor audio on/off).
                 * @param enabled True to open the device and start hearing the world.
                 * @return True if the resulting state matches the request (a device may fail
                 *         to open on a headless machine, leaving audio off).
                 */
                bool set_enabled(bool enabled);

                /** @brief Whether editor audio is requested on. */
                bool enabled() const noexcept { return enabled_; }

                /** @brief Whether the output device is actually open and rendering. */
                bool device_open() const noexcept { return device_open_; }

                /**
                 * @brief Projects the world's audio components into the live voice pool.
                 *
                 * Builds a listener-local `Audio::SceneSnapshot` from every audio emitter in
                 * @p world (positions relative to @p listener_position, the Scene camera),
                 * picks the highest-priority reverb zone containing the listener, and applies
                 * it to the voice manager via the `AudioScene` bridge. A no-op when audio is
                 * disabled. Call once per editor frame after the camera has moved.
                 *
                 * @param world             The edited world (read through the seam).
                 * @param listener_position The listener (Scene camera) world position.
                 * @param listener_forward  The listener facing direction.
                 * @param listener_up       The listener up direction.
                 */
                void update(Simulation::IWorldEditor& world, const Vector3& listener_position,
                            const Vector3& listener_forward, const Vector3& listener_up);

                /**
                 * @brief Auditions a one-shot of a sound id at the listener (Inspector "Play").
                 * @param sound_id The emitter's sound id.
                 * @param gain     Linear gain for the audition.
                 */
                void preview(std::uint32_t sound_id, float gain);

                /** @brief The latest profiler snapshot (voice counts, meters, scope). */
                const Audio::AudioProfileSnapshot& profile() const noexcept { return profile_; }

                /** @brief Refreshes @ref profile from the engine (call once per frame). */
                void poll_profile();

                /** @brief The mixer bus ids the panels display and drive. */
                int master_bus() const noexcept { return master_bus_; }
                int sfx_bus() const noexcept { return sfx_bus_; }
                int music_bus() const noexcept { return music_bus_; }
                int reverb_bus() const noexcept { return reverb_bus_; }

                /** @brief Sets a bus's linear gain (0..~2), click-free. */
                void set_bus_gain(int bus_id, float gain);
                /** @brief A bus's current linear gain target. */
                float bus_gain(int bus_id);
                /** @brief A human name for a bus id (Master/SFX/Music/Reverb/Bus N). */
                const char* bus_name(int bus_id) const noexcept;

                /** @brief The live engine (for the panels to read voice counts, etc.). */
                Audio::AudioEngine& engine() noexcept { return engine_; }

                /** @brief The number of active voices this frame (real + virtual). */
                int active_voices() const noexcept { return profile_.active_voices; }

            private:
                /** @brief Resolves a `sound` id to a distinct looping tone (no bank in-editor yet). */
                class ToneFactory final : public Audio::IEmitterSourceFactory
                {
                    public:
                        std::unique_ptr<Audio::VoiceSource> create(std::uint32_t sound_id) override;
                        bool looping = true;
                };

                static Audio::I3DL2Reverb to_i3dl2(const Simulation::ReverbZoneParams& p) noexcept;

                /** @brief Maps an emitter's bus code (0 Master,1 SFX,2 Music,3 Reverb) to a bus id. */
                int bus_for(std::uint32_t code) const noexcept;

                Audio::AudioEngine engine_;
                ToneFactory factory_;
                Audio::AudioScene scene_;
                Audio::AudioProfileSnapshot profile_{};
                std::unique_ptr<SushiEngine::Audio::SDLAudioDevice> device_;
                Audio::IReverb* reverb_ = nullptr; /**< The FDN on the reverb aux bus (borrowed). */

                int master_bus_ = 0;
                int sfx_bus_ = 0;
                int music_bus_ = 0;
                int reverb_bus_ = 0;

                bool enabled_ = false;
                bool device_open_ = false;
                double sample_rate_ = 48000.0;
                int block_ = 512;
        };
    } // namespace Editor
} // namespace SushiEngine
