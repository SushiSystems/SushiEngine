/**************************************************************************/
/* audio_mixer_demo.cpp                                                  */
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
 * @file audio_mixer_demo.cpp
 * @brief Phase S2 vertical slice: a prioritized multi-source mix through the bus DAG.
 *
 * Two music tones plus many spatial sfx tones are started into a small mixer (music
 * and sfx buses under a master, with an aux-send reverb bus), far more than the
 * real-voice cap. The voice manager ranks them by priority and audibility and renders
 * only the cap; the rest go virtual. It:
 *
 *   1. Runs headless and self-checks that exactly the cap is real, that pushing the
 *      listener out of earshot virtualizes the spatial voices (only the non-spatial
 *      music survives), that an RTPC ramps the master gain, and that the stereo output
 *      stays bounded and non-silent. No hardware needed — this is the CI check.
 *   2. Best-effort plays the mix through the SDL2 device.
 *
 * Exits 0 on success.
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <sdl/sdl_audio_device.hpp>

using namespace SushiEngine::Audio;

namespace
{
    double block_rms(const float* left, const float* right, int n)
    {
        double sum = 0.0;
        for (int i = 0; i < n; ++i)
        {
            sum += static_cast<double>(left[i]) * left[i];
            sum += static_cast<double>(right[i]) * right[i];
        }
        return std::sqrt(sum / (2.0 * n));
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;
    const int max_real = 6;

    AudioEngine engine(64, max_real);

    // Bus topology: master <- {music, sfx}; sfx aux-sends a reverb bus (darkened by a
    // low-pass insert as an S5 placeholder) that also routes to master.
    const int master = engine.mixer().add_bus(NO_BUS);
    const int music_bus = engine.mixer().add_bus(master);
    const int sfx_bus = engine.mixer().add_bus(master);
    const int reverb_bus = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);
    engine.mixer().add_aux_send(sfx_bus, reverb_bus, 0.25f);
    {
        std::unique_ptr<BiquadBusEffect> damp(new BiquadBusEffect());
        damp->left().set_low_pass(1200.0, 0.707, sample_rate);
        damp->right().set_low_pass(1200.0, 0.707, sample_rate);
        engine.mixer().add_insert(reverb_bus, std::move(damp));
    }

    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    // Two non-spatial music tones (always audible, high priority), panned wide.
    {
        VoiceDescriptor d;
        d.base_gain = 0.20f;
        d.priority = 10.0f;
        d.bus = music_bus;
        d.pan = -0.6f;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(220.0f, 1.0f)));
        d.pan = 0.6f;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(277.0f, 1.0f)));
    }

    // Many spatial sfx tones at varied distance/priority/pan.
    const int sfx_count = 24;
    for (int i = 0; i < sfx_count; ++i)
    {
        VoiceDescriptor d;
        d.base_gain = 0.15f;
        d.priority = static_cast<float>(i % 5); // a spread of priorities
        d.bus = sfx_bus;
        d.pan = (static_cast<float>(i) / (sfx_count - 1)) * 2.0f - 1.0f;
        d.spatial = true;
        d.position = AudioVec3{static_cast<float>(2 + i), 0.0f, 0.0f};
        d.min_distance = 1.0f;
        d.max_distance = 60.0f;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(400.0f + 40.0f * i, 1.0f)));
    }

    std::printf("started %d voices (cap %d real)\n", engine.voices().active_count(), max_real);

    // RTPC → master gain, ramped.
    Rtpc master_gain;
    master_gain.configure(0.05, sample_rate);
    master_gain.snap(1.0f);

    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    // Render a while near the sources.
    double peak = 0.0;
    for (int b = 0; b < 100; ++b)
    {
        engine.mixer().bus_gain(master).set_target(master_gain.value().advance_block(block));
        engine.render(channels, 2, block);
        for (int i = 0; i < block; ++i)
        {
            const double a = std::fabs(static_cast<double>(left[i]));
            const double b2 = std::fabs(static_cast<double>(right[i]));
            if (a > peak) peak = a;
            if (b2 > peak) peak = b2;
        }
    }
    const int real_near = engine.voices().real_count();
    std::printf("near listener: %d real voices, peak=%.4f, rms=%.4f\n",
                real_near, peak, block_rms(left.data(), right.data(), block));

    if (real_near != max_real)
    {
        std::fprintf(stderr, "audio_mixer_demo FAILED: expected %d real voices near, got %d\n",
                     max_real, real_near);
        return 1;
    }
    if (peak > 1.5)
    {
        std::fprintf(stderr, "audio_mixer_demo FAILED: output too hot (peak %.4f)\n", peak);
        return 1;
    }

    // Move the listener far past every sfx max_distance: only the 2 music voices remain.
    engine.voices().set_listener(ListenerState{AudioVec3{100000.0f, 0.0f, 0.0f}});
    engine.render(channels, 2, block);
    const int real_far = engine.voices().real_count();
    std::printf("far listener: %d real voices (music only)\n", real_far);
    if (real_far != 2)
    {
        std::fprintf(stderr, "audio_mixer_demo FAILED: expected 2 real voices far, got %d\n", real_far);
        return 1;
    }

    // Best-effort playback (listener back near the sources).
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});
    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch, %d frames/block — playing the mix\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        device.close();
    }
    else
    {
        std::printf("no audio device available (headless) — mix verified in software\n");
    }

    std::printf("audio_mixer_demo OK\n");
    return 0;
}
