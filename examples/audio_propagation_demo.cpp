/**************************************************************************/
/* audio_propagation_demo.cpp                                            */
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
 * @file audio_propagation_demo.cpp
 * @brief Phase S3 vertical slice: a flyby Doppler through the propagation model.
 *
 * A steady tone flies past the listener at a fraction of the speed of sound. Because
 * the source's propagation is one variable-length delay line, the shrinking distance
 * on approach speeds the read rate (a rising pitch) and the growing distance on
 * departure slows it (a falling pitch) — Doppler with no explicit velocity term. It:
 *
 *   1. Runs the flyby headless and self-checks that the tone's measured pitch is above
 *      the source frequency while approaching and below it while receding. No hardware
 *      needed — this is the CI check.
 *   2. Best-effort plays the flyby through the SDL2 device, animating the source
 *      position on the main thread while the device renders.
 *
 * Exits 0 on success.
 */

#include <atomic>
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
    // Dominant frequency of a block, estimated from upward zero crossings.
    double zero_crossing_frequency(const float* x, int n, double sample_rate)
    {
        int crossings = 0;
        for (int i = 1; i < n; ++i)
            if (x[i - 1] <= 0.0f && x[i] > 0.0f)
                ++crossings;
        return static_cast<double>(crossings) / (static_cast<double>(n) / sample_rate);
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 256;
    const float source_hz = 1000.0f;

    AudioEngine engine(4, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.voices().set_max_propagation_distance(400.0f);
    // Prepare with headroom above the block used here so an OS mixer that hands a
    // larger callback block than requested still renders fully (the engine clamps
    // rather than overruns, but headroom avoids clamped-to-silence gaps).
    const int max_block = 2048;
    engine.prepare(sample_rate, max_block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    VoiceDescriptor d;
    d.base_gain = 1.0f;
    d.priority = 1.0f;
    d.bus = master;
    d.spatial = true;
    d.model = DistanceModel::Inverse;
    d.min_distance = 1.0f;
    d.max_distance = 400.0f;
    d.position = AudioVec3{-150.0f, 20.0f, 0.0f};
    const int voice = engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(source_hz, 1.0f)));

    // (1) Headless flyby: source travels -150 → +150 m along x, 20 m to the side.
    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};
    const float speed = 150.0f;
    const float dt = static_cast<float>(block) / static_cast<float>(sample_rate);

    double approach_sum = 0.0, recede_sum = 0.0;
    int approach_n = 0, recede_n = 0;
    float x = -150.0f;
    while (x < 150.0f)
    {
        engine.voices().set_voice_position(voice, AudioVec3{x, 20.0f, 0.0f});
        engine.render(channels, 2, block);
        const double f = zero_crossing_frequency(left.data(), block, sample_rate);
        if (x > -80.0f && x < -15.0f)
        {
            approach_sum += f;
            ++approach_n;
        }
        else if (x > 15.0f && x < 80.0f)
        {
            recede_sum += f;
            ++recede_n;
        }
        x += speed * dt;
    }
    const double approach_hz = approach_sum / approach_n;
    const double recede_hz = recede_sum / recede_n;
    std::printf("flyby: approaching ~%.0f Hz, receding ~%.0f Hz (source %.0f Hz)\n",
                approach_hz, recede_hz, static_cast<double>(source_hz));

    if (!(approach_hz > source_hz))
    {
        std::fprintf(stderr, "audio_propagation_demo FAILED: approach %.0f not above %.0f\n",
                     approach_hz, static_cast<double>(source_hz));
        return 1;
    }
    if (!(recede_hz < source_hz))
    {
        std::fprintf(stderr, "audio_propagation_demo FAILED: recede %.0f not below %.0f\n",
                     recede_hz, static_cast<double>(source_hz));
        return 1;
    }

    // (2) Best-effort audible flyby: the main thread animates the source while the
    // device renders on its callback thread.
    engine.voices().set_voice_position(voice, AudioVec3{-150.0f, 20.0f, 0.0f});
    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        std::printf("audio device open — flying the source past the listener\n");
        float px = -150.0f;
        for (int step = 0; step < 60 && px < 150.0f; ++step)
        {
            engine.voices().set_voice_position(voice, AudioVec3{px, 20.0f, 0.0f});
            px += 5.0f;
            std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
        device.close();
    }
    else
    {
        std::printf("no audio device available (headless) — flyby verified in software\n");
    }

    std::printf("audio_propagation_demo OK\n");
    return 0;
}
