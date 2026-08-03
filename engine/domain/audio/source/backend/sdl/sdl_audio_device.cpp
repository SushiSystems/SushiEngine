/**************************************************************************/
/* sdl_audio_device.cpp                                                  */
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

#include "sdl/sdl_audio_device.hpp"

#include <cstddef>
#include <cstring>

#include <SDL.h>

namespace SushiEngine
{
    namespace Audio
    {
        SdlAudioDevice::~SdlAudioDevice()
        {
            close();
        }

        bool SdlAudioDevice::open(const AudioStreamFormat& desired, IAudioRenderer& renderer)
        {
            if (running_)
                return false;

            // Bring up only SDL's audio subsystem; a host already running SDL for
            // video/input keeps its own init — SDL ref-counts subsystems.
            if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0)
                return false;
            audio_subsystem_ = true;

            SDL_AudioSpec want;
            SDL_zero(want);
            want.freq = desired.sample_rate;
            want.format = AUDIO_F32SYS; // our mix is planar float; SDL takes interleaved float
            want.channels = static_cast<Uint8>(desired.channel_count);
            want.samples = static_cast<Uint16>(desired.block_frames);
            want.callback = &SdlAudioDevice::audio_callback;
            want.userdata = this;

            // Let the OS mixer pick its native rate and buffer size (common on shared
            // devices); the channel count stays as requested so the mix's bus layout
            // is fixed. We read the obtained format back and size scratch to it.
            SDL_AudioSpec have;
            SDL_zero(have);
            device_id_ = SDL_OpenAudioDevice(
                nullptr, 0, &want, &have,
                SDL_AUDIO_ALLOW_FREQUENCY_CHANGE | SDL_AUDIO_ALLOW_SAMPLES_CHANGE);
            if (device_id_ == 0)
            {
                SDL_QuitSubSystem(SDL_INIT_AUDIO);
                audio_subsystem_ = false;
                return false;
            }

            format_.sample_rate = have.freq;
            format_.channel_count = have.channels;
            format_.block_frames = have.samples;
            renderer_ = &renderer;

            // Pre-allocate the planar scratch and channel-pointer table before the
            // stream starts, so nothing on the audio thread ever allocates.
            const std::size_t channels = static_cast<std::size_t>(format_.channel_count);
            const std::size_t frames = static_cast<std::size_t>(format_.block_frames);
            planar_.assign(channels * frames, 0.0f);
            channel_ptrs_.resize(channels);
            for (std::size_t c = 0; c < channels; ++c)
                channel_ptrs_[c] = planar_.data() + c * frames;

            running_ = true;
            SDL_PauseAudioDevice(device_id_, 0); // unpause: the callback thread starts now
            return true;
        }

        void SdlAudioDevice::close() noexcept
        {
            if (device_id_ != 0)
            {
                // Closing the device stops and joins the callback thread, so after
                // this returns the renderer is no longer referenced.
                SDL_CloseAudioDevice(device_id_);
                device_id_ = 0;
            }
            if (audio_subsystem_)
            {
                SDL_QuitSubSystem(SDL_INIT_AUDIO);
                audio_subsystem_ = false;
            }
            running_ = false;
            renderer_ = nullptr;
        }

        void SdlAudioDevice::render_block(float* interleaved, int frame_count) noexcept
        {
            const int channels = format_.channel_count;

            // Guard against a callback larger than the negotiated block (SDL should
            // never do this once ALLOW_SAMPLES_CHANGE settled the size, but the mix
            // must never write past its scratch).
            if (frame_count > format_.block_frames)
                frame_count = format_.block_frames;

            if (renderer_ != nullptr)
            {
                renderer_->render(channel_ptrs_.data(), channels, frame_count);
            }
            else
            {
                for (int c = 0; c < channels; ++c)
                    std::memset(channel_ptrs_[static_cast<std::size_t>(c)], 0,
                                sizeof(float) * static_cast<std::size_t>(frame_count));
            }

            // Interleave planar scratch into SDL's output buffer.
            for (int f = 0; f < frame_count; ++f)
                for (int c = 0; c < channels; ++c)
                    interleaved[f * channels + c] =
                        channel_ptrs_[static_cast<std::size_t>(c)][f];
        }

        void SdlAudioDevice::audio_callback(void* user, unsigned char* stream, int length)
        {
            SdlAudioDevice* self = static_cast<SdlAudioDevice*>(user);
            float* out = reinterpret_cast<float*>(stream);
            const int bytes_per_frame =
                static_cast<int>(sizeof(float)) * self->format_.channel_count;
            const int frame_count = bytes_per_frame > 0 ? length / bytes_per_frame : 0;
            self->render_block(out, frame_count);
        }
    } // namespace Audio
} // namespace SushiEngine
