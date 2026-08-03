/**************************************************************************/
/* audio_miniaudio_demo.cpp                                               */
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
 * @file audio_miniaudio_demo.cpp
 * @brief The miniaudio (native low-latency) IAudioDevice backend, end to end.
 *
 * Opens the same AudioEngine through @ref MaAudioDevice (WASAPI/CoreAudio/ALSA/…) instead
 * of SDL and plays a tone. Self-checks the render path headless first (so it passes with no
 * device), then best-effort opens the native device. Exits 0 on success.
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <miniaudio/ma_audio_device.hpp>

using namespace SushiEngine::Audio;

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    AudioEngine engine(8, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});
    {
        VoiceDescriptor d;
        d.base_gain = 0.5f;
        d.priority = 10.0f;
        d.bus = master;
        d.spatial = false;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(440.0f, 0.5f)));
    }

    // Headless render check (no device needed).
    {
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        float* ch[2] = {l.data(), r.data()};
        double peak = 0.0;
        for (int b = 0; b < 50; ++b)
        {
            engine.render(ch, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::fmax(peak, std::fabs(static_cast<double>(l[i])));
        }
        std::printf("headless render peak=%.3f\n", peak);
        if (!(peak > 0.1))
        {
            std::fprintf(stderr, "audio_miniaudio_demo FAILED: render produced no signal\n");
            return 1;
        }
    }

    MaAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("miniaudio device open: %d Hz, %d ch, %d frames — playing 440 Hz for 1.5 s\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        device.close();
    }
    else
    {
        std::printf("no miniaudio device available (headless) — render path already verified\n");
    }

    std::printf("audio_miniaudio_demo OK\n");
    return 0;
}
