/**************************************************************************/
/* audio_spatial_demo.cpp                                                */
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
 * @file audio_spatial_demo.cpp
 * @brief Phase S4 vertical slice: a source orbiting the head in binaural 3D.
 *
 * A tone circles the listener in the horizontal plane. It is encoded into the
 * ambisonic scene bus at its head-relative direction and decoded to the two ears
 * through the analytic head model, so on headphones it audibly travels left → behind →
 * right → front. It:
 *
 *   1. Runs the orbit headless and self-checks the binaural cues: the left ear is
 *      louder when the source is to the left, the right ear when it is to the right,
 *      and the two ears match when it is dead ahead. It also checks head-tracking — a
 *      fixed front source moves to the right ear when the listener turns to face left.
 *      No hardware needed — this is the CI check.
 *   2. Best-effort plays the full orbit through the SDL2 device (best on headphones).
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
    const double kTwoPi = 6.28318530717958647692;

    double block_rms(const float* x, int n)
    {
        double sum = 0.0;
        for (int i = 0; i < n; ++i)
            sum += static_cast<double>(x[i]) * x[i];
        return std::sqrt(sum / n);
    }

    // Places the source at azimuth `angle` on a radius-`r` horizontal orbit, renders a
    // few blocks, and returns the settled per-ear RMS.
    void sample_orbit(AudioEngine& engine, int voice, double angle, float radius, int block,
                      double& left_rms, double& right_rms)
    {
        engine.voices().set_voice_position(
            voice, AudioVec3{radius * static_cast<float>(std::cos(angle)),
                             radius * static_cast<float>(std::sin(angle)), 0.0f});
        std::vector<float> left, right;
        std::vector<float> l(static_cast<std::size_t>(block)), r(static_cast<std::size_t>(block));
        float* channels[2] = {l.data(), r.data()};
        for (int b = 0; b < 24; ++b)
        {
            engine.render(channels, 2, block);
            if (b >= 12)
            {
                left.insert(left.end(), l.begin(), l.end());
                right.insert(right.end(), r.begin(), r.end());
            }
        }
        left_rms = block_rms(left.data(), static_cast<int>(left.size()));
        right_rms = block_rms(right.data(), static_cast<int>(right.size()));
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 256;
    const float radius = 3.0f;

    AudioEngine engine(4, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.set_ambisonic_order(3);
    engine.voices().set_max_propagation_distance(50.0f);
    engine.prepare(sample_rate, 2048); // headroom for the device block
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f},
                                               AudioVec3{1.0f, 0.0f, 0.0f}, AudioVec3{0.0f, 0.0f, 1.0f}});

    VoiceDescriptor d;
    d.base_gain = 1.0f;
    d.priority = 1.0f;
    d.bus = master;
    d.spatial = true;
    d.model = DistanceModel::Inverse;
    d.min_distance = 1.0f;
    d.max_distance = 50.0f;
    d.propagation_delay = false; // constant-radius orbit: isolate the spatial cue
    d.position = AudioVec3{radius, 0.0f, 0.0f};
    const int voice = engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(500.0f, 1.0f)));

    // (1) Headless cue check at front, left, and right.
    double front_l = 0.0, front_r = 0.0, left_l = 0.0, left_r = 0.0, right_l = 0.0, right_r = 0.0;
    sample_orbit(engine, voice, 0.0, radius, block, front_l, front_r);          // front (+x)
    sample_orbit(engine, voice, kTwoPi * 0.25, radius, block, left_l, left_r);  // left (+y)
    sample_orbit(engine, voice, kTwoPi * 0.75, radius, block, right_l, right_r); // right (-y)

    std::printf("front L/R = %.3f/%.3f  left L/R = %.3f/%.3f  right L/R = %.3f/%.3f\n",
                front_l, front_r, left_l, left_r, right_l, right_r);

    if (std::fabs(front_l - front_r) > 0.10 * front_l)
    {
        std::fprintf(stderr, "audio_spatial_demo FAILED: front not symmetric\n");
        return 1;
    }
    if (!(left_l > left_r * 1.1))
    {
        std::fprintf(stderr, "audio_spatial_demo FAILED: left source not louder in left ear\n");
        return 1;
    }
    if (!(right_r > right_l * 1.1))
    {
        std::fprintf(stderr, "audio_spatial_demo FAILED: right source not louder in right ear\n");
        return 1;
    }

    // Head-tracking: a fixed front source with the listener turned to face left should
    // move to the right ear.
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f},
                                               AudioVec3{0.0f, 1.0f, 0.0f}, AudioVec3{0.0f, 0.0f, 1.0f}});
    double turned_l = 0.0, turned_r = 0.0;
    sample_orbit(engine, voice, 0.0, radius, block, turned_l, turned_r); // source at +x, head faces +y
    std::printf("front source, head turned left: L/R = %.3f/%.3f (right ear should lead)\n",
                turned_l, turned_r);
    if (!(turned_r > turned_l * 1.1))
    {
        std::fprintf(stderr, "audio_spatial_demo FAILED: head-tracking did not re-aim the source\n");
        return 1;
    }

    // (2) Best-effort audible orbit (headphones recommended).
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f},
                                               AudioVec3{1.0f, 0.0f, 0.0f}, AudioVec3{0.0f, 0.0f, 1.0f}});
    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        std::printf("audio device open — orbiting the source around your head (use headphones)\n");
        for (int step = 0; step < 120; ++step)
        {
            const double angle = kTwoPi * step / 120.0;
            engine.voices().set_voice_position(
                voice, AudioVec3{radius * static_cast<float>(std::cos(angle)),
                                 radius * static_cast<float>(std::sin(angle)), 0.0f});
            std::this_thread::sleep_for(std::chrono::milliseconds(25));
        }
        device.close();
    }
    else
    {
        std::printf("no audio device available (headless) — spatialization verified in software\n");
    }

    std::printf("audio_spatial_demo OK\n");
    return 0;
}
