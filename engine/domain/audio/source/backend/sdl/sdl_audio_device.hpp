/**************************************************************************/
/* sdl_audio_device.hpp                                                  */
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

#ifndef SUSHIENGINE_AUDIO_SDL_SDL_AUDIO_DEVICE_HPP
#define SUSHIENGINE_AUDIO_SDL_SDL_AUDIO_DEVICE_HPP

/**
 * @file sdl_audio_device.hpp
 * @brief The only SDL-aware audio component: an @ref IAudioDevice over SDL2.
 *
 * SDL2 is already in the stack (input + window), so the device layer adds zero new
 * dependency and gives one Windows/Linux/macOS output path. This is *only* the
 * device I/O and buffer plumbing — the entire mix is our from-scratch DSP core; the
 * device just opens an `SDL_AudioDevice`, and on each callback deinterleaves nothing
 * (it renders planar), calls the @ref IAudioRenderer to fill one block, then
 * interleaves the result into SDL's output buffer.
 *
 * The header exposes no SDL type: the OS handle is held as an opaque `std::uint32_t`
 * (an `SDL_AudioDeviceID`), so a consumer includes this and links `sushi_audio`
 * without pulling SDL into its own translation unit — the same discipline that keeps
 * the input action layer SDL-free.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/audio/device.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief Opens an SDL2 audio output stream and drives an @ref IAudioRenderer.
         *
         * One instance owns at most one open device. It initializes (and later quits)
         * only SDL's audio subsystem, so it composes with a host that already runs
         * SDL for video/input. The planar scratch buffers the renderer writes into
         * are allocated once in @ref open — never in the callback — keeping the audio
         * thread allocation-free.
         */
        class SdlAudioDevice final : public IAudioDevice
        {
            public:
                SdlAudioDevice() = default;

                /** @brief Closes the device if still open. */
                ~SdlAudioDevice() override;

                SdlAudioDevice(const SdlAudioDevice&) = delete;
                SdlAudioDevice& operator=(const SdlAudioDevice&) = delete;

                /** @copydoc IAudioDevice::open */
                bool open(const AudioStreamFormat& desired, IAudioRenderer& renderer) override;

                /** @copydoc IAudioDevice::close */
                void close() noexcept override;

                /** @copydoc IAudioDevice::is_running */
                bool is_running() const noexcept override { return running_; }

                /** @copydoc IAudioDevice::format */
                AudioStreamFormat format() const noexcept override { return format_; }

            private:
                /**
                 * @brief SDL's C callback trampoline; forwards to @ref render_block.
                 * @param user   The `this` pointer registered as the callback userdata.
                 * @param stream The interleaved output byte buffer SDL wants filled.
                 * @param length The buffer length in bytes.
                 */
                static void audio_callback(void* user, unsigned char* stream, int length);

                /**
                 * @brief Renders one block: fill planar scratch, interleave into @p interleaved.
                 * @param interleaved The SDL output buffer, as interleaved floats.
                 * @param frame_count Number of sample frames this callback expects.
                 */
                void render_block(float* interleaved, int frame_count) noexcept;

                std::uint32_t device_id_ = 0;      /**< Opaque SDL_AudioDeviceID; 0 = none open. */
                bool audio_subsystem_ = false;     /**< Whether we initialized SDL's audio subsystem. */
                bool running_ = false;             /**< Whether a stream is open and rendering. */
                AudioStreamFormat format_{};       /**< The obtained (running) format. */
                IAudioRenderer* renderer_ = nullptr;
                std::vector<float> planar_;        /**< channel_count * block_frames, planar scratch. */
                std::vector<float*> channel_ptrs_; /**< One pointer per channel into planar_. */
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
