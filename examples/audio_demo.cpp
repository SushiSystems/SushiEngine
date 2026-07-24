/**************************************************************************/
/* audio_demo.cpp                                                        */
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

/**
 * @file audio_demo.cpp
 * @brief Phase S0 vertical slice: the silent block-producing device loop.
 *
 * This is the audio subsystem's first executable — it stands up the two-plane model's
 * render plane end to end with a trivial renderer, proving the seams compose and the
 * block loop runs before any DSP exists. It does two things:
 *
 *   1. A headless self-check that needs no audio hardware: it pumps a fixed number of
 *      blocks through a @ref SilenceRenderer and asserts every sample came out silent
 *      (and that the renderer was actually driven). This is the deterministic part and
 *      is what makes the demo a valid CI check on a machine with no sound device.
 *   2. A best-effort real open through the SDL2 backend. On a host with an audio
 *      device this starts the callback thread and renders silence for a moment; on a
 *      headless host `open` fails cleanly and that is reported, not treated as an error.
 *
 * Exits 0 on success. Later phases replace the silence with the real mix (S1+).
 */

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>
#include <vector>

#include <SushiEngine/audio/device.hpp>
#include <sdl/sdl_audio_device.hpp>

using SushiEngine::Audio::AudioStreamFormat;
using SushiEngine::Audio::IAudioRenderer;
using SushiEngine::Audio::SdlAudioDevice;

namespace
{
    /**
     * @brief A renderer that outputs pure silence and counts how often it ran.
     *
     * The S0 stand-in for the real mix: it clears every channel and tallies blocks and
     * frames so the demo can prove both the software pump and (when a device opens) the
     * audio callback thread actually drove it. The counters are atomic because the audio
     * callback thread writes them concurrently with the main thread's reads.
     */
    class SilenceRenderer final : public IAudioRenderer
    {
        public:
            void render(float* const* channels, int channel_count, int frame_count) noexcept override
            {
                for (int c = 0; c < channel_count; ++c)
                    for (int f = 0; f < frame_count; ++f)
                        channels[c][f] = 0.0f;
                blocks_.fetch_add(1, std::memory_order_relaxed);
                frames_.fetch_add(static_cast<std::uint64_t>(frame_count), std::memory_order_relaxed);
            }

            std::uint64_t blocks() const noexcept { return blocks_.load(std::memory_order_relaxed); }
            std::uint64_t frames() const noexcept { return frames_.load(std::memory_order_relaxed); }

        private:
            std::atomic<std::uint64_t> blocks_{0};
            std::atomic<std::uint64_t> frames_{0};
    };
} // namespace

int main()
{
    SilenceRenderer renderer;

    // (1) Headless block-producing loop. Seed the scratch non-zero so a silent result
    // proves the renderer actually overwrote every sample, then pump N blocks.
    const int channels = 2;
    const int block_frames = 512;
    const int blocks_to_pump = 128;

    std::vector<float> planar(static_cast<std::size_t>(channels) * block_frames, 0.25f);
    std::vector<float*> channel_ptrs(static_cast<std::size_t>(channels));
    for (int c = 0; c < channels; ++c)
        channel_ptrs[static_cast<std::size_t>(c)] =
            planar.data() + static_cast<std::size_t>(c) * block_frames;

    for (int i = 0; i < blocks_to_pump; ++i)
        renderer.render(channel_ptrs.data(), channels, block_frames);

    for (float sample : planar)
    {
        if (sample != 0.0f)
        {
            std::fprintf(stderr, "audio_demo FAILED: renderer left a non-silent sample\n");
            return 1;
        }
    }
    if (renderer.blocks() != static_cast<std::uint64_t>(blocks_to_pump))
    {
        std::fprintf(stderr, "audio_demo FAILED: expected %d blocks, saw %llu\n",
                     blocks_to_pump, static_cast<unsigned long long>(renderer.blocks()));
        return 1;
    }
    std::printf("software block loop: %llu blocks, %llu frames, all silent\n",
                static_cast<unsigned long long>(renderer.blocks()),
                static_cast<unsigned long long>(renderer.frames()));

    // (2) Best-effort device open. Not an error to fail on a headless host.
    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = 512;

    if (device.open(desired, renderer))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch, %d frames/block\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);

        const std::uint64_t before = renderer.blocks();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        device.close();

        std::printf("device callback rendered %llu blocks of silence\n",
                    static_cast<unsigned long long>(renderer.blocks() - before));
    }
    else
    {
        std::printf("no audio device available (headless) — software loop already verified\n");
    }

    std::printf("audio_demo OK\n");
    return 0;
}
