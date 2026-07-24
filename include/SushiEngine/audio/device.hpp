/**************************************************************************/
/* device.hpp                                                            */
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

#ifndef SUSHIENGINE_AUDIO_DEVICE_HPP
#define SUSHIENGINE_AUDIO_DEVICE_HPP

/**
 * @file device.hpp
 * @brief The audio device I/O seam: the boundary between the from-scratch DSP mix
 *        and whatever OS backend actually moves samples to the speakers.
 *
 * This is the first of the audio subsystem's SOLID seams (see
 * `docs/design/audio_system.md` §12–§13). It carries **no** backend type: the SDL2
 * implementation (`sushi_audio`) hides `SDL_AudioDeviceID` entirely, so a consumer
 * includes this header and links the backend without pulling SDL into its own
 * translation unit — exactly as the input module keeps SDL behind
 * `SdlInputTranslator`.
 *
 * The seam encodes the one architectural invariant of every shipping AAA audio
 * runtime (§0): a strict split between the control plane (the game/ECS thread) and
 * the audio-render plane. The backend owns the render plane — a high-priority
 * callback thread — and pulls one fixed block per callback through an
 * @ref IAudioRenderer. That callback runs under hard real-time discipline: no heap
 * allocation, no locks, no syscalls, no file I/O, no exceptions.
 */

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief The immutable stream format a device runs at, negotiated at open.
         *
         * The requested value is a preference; a backend may return a different
         * sample rate or block size (e.g. the OS mixer's native rate), so a consumer
         * always reads the *obtained* format back from @ref IAudioDevice::format
         * before sizing its own DSP state. @ref block_frames is the number of sample
         * frames handed to the renderer per callback and is kept a power of two so
         * the DSP core's SIMD kernels never see a ragged tail on the common path.
         */
        struct AudioStreamFormat
        {
            int sample_rate = 48000;  /**< Output sample rate in Hz. */
            int channel_count = 2;    /**< Interleaved output channel count (2 = stereo). */
            int block_frames = 512;   /**< Sample frames rendered per callback; power of two. */
        };

        /**
         * @brief The real-time render sink the device drives once per block.
         *
         * The device backend calls @ref render on its own high-priority callback
         * thread, handing it @p channel_count **planar** (deinterleaved) output
         * buffers of @p frame_count samples each; the implementation fills them with
         * the block's mix. The device interleaves the result into the OS buffer, so
         * the renderer never touches interleaving or the backend's sample layout.
         *
         * @warning @ref render executes on the audio thread. It must be
         * allocation-, lock-, and syscall-free, and must not throw — hence
         * `noexcept`. This is the render plane of the two-plane model (§0); anything
         * that can block belongs on the control plane behind the command ring.
         */
        class IAudioRenderer
        {
            public:
                virtual ~IAudioRenderer() = default;

                /**
                 * @brief Renders one block of audio into planar output buffers.
                 *
                 * @param channels      Array of @p channel_count pointers, each to a
                 *                       buffer of at least @p frame_count floats, to be
                 *                       overwritten with this block's samples.
                 * @param channel_count Number of output channels (matches the device format).
                 * @param frame_count   Number of sample frames to produce this call.
                 */
                virtual void render(float* const* channels, int channel_count,
                                    int frame_count) noexcept = 0;
        };

        /**
         * @brief The device I/O seam: open a stream, drive a renderer, close it.
         *
         * Isolates the one unstable, platform-specific dependency (the OS audio API)
         * so the entire from-scratch DSP mix is testable against a trivial renderer
         * and the backend (SDL2 today; miniaudio / raw WASAPI-PipeWire later) is
         * swappable without touching a line of the mix. The device owns the callback
         * thread; the App/voice-manager owns the @ref IAudioRenderer it is handed.
         */
        class IAudioDevice
        {
            public:
                virtual ~IAudioDevice() = default;

                /**
                 * @brief Opens the output stream and begins driving @p renderer.
                 *
                 * On success the backend's callback thread starts immediately and
                 * calls `renderer.render(...)` once per block until @ref close. The
                 * obtained format (which may differ from @p desired) is readable from
                 * @ref format once this returns true.
                 *
                 * @param desired  The preferred sample rate, channel count, and block size.
                 * @param renderer The sink driven on the audio thread; must outlive the
                 *                  open stream (until @ref close returns).
                 * @return True if the device opened and started; false if no device is
                 *         available (e.g. a headless host) or the backend refused.
                 */
                virtual bool open(const AudioStreamFormat& desired, IAudioRenderer& renderer) = 0;

                /**
                 * @brief Stops the callback thread and releases the device.
                 *
                 * Safe to call when not open (a no-op) and idempotent. After it
                 * returns, the renderer passed to @ref open is no longer referenced.
                 */
                virtual void close() noexcept = 0;

                /** @brief Whether a stream is currently open and rendering. */
                virtual bool is_running() const noexcept = 0;

                /**
                 * @brief The format the stream is actually running at.
                 * @return The obtained format while open; the last-requested or default
                 *         format otherwise.
                 */
                virtual AudioStreamFormat format() const noexcept = 0;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
