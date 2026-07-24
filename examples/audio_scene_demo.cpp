/**************************************************************************/
/* audio_scene_demo.cpp                                                  */
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
 * @file audio_scene_demo.cpp
 * @brief Phase S6 vertical slice: the ECS→audio control-plane bridge (AudioScene).
 *
 * The S6 bridge turns a per-frame world snapshot (listener facing + emitter positions)
 * into live voices: a voice starts when an emitter appears, moves and re-gains as the
 * emitter persists (the frame-to-frame position change is the Doppler), and stops when
 * the emitter goes silent or is destroyed. This drives that reconciliation directly with
 * hand-built @ref SceneSnapshot frames (the exact structure `sim/audio_extract.hpp`
 * fills from the ECS world), so it needs no runtime — the real ECS read is covered by
 * `Integration_AudioEcs`. It:
 *
 *   1. Runs headless and self-checks the reconciliation: an emitter flying past the
 *      listener holds exactly one voice; adding a second emitter makes two; dropping one
 *      from the snapshot stops its voice; the rendered stereo stays bounded and non-silent.
 *   2. Best-effort plays the flyby (audible Doppler + left/right motion on headphones).
 *
 * Exits 0 on success.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <sdl/sdl_audio_device.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // The S8 event/bank resolver stands in as a fixed tone per id: even ids a low tone,
    // odd ids a higher one, so two emitters are distinguishable.
    class ToneFactory final : public IEmitterSourceFactory
    {
        public:
            std::unique_ptr<VoiceSource> create(std::uint32_t sound_id) override
            {
                const float hz = (sound_id % 2 == 0) ? 330.0f : 495.0f;
                return std::unique_ptr<VoiceSource>(new ToneSource(hz, 1.0f));
            }
    };

    EmitterSnapshot flyby(std::uint64_t key, std::uint32_t sound, float z)
    {
        EmitterSnapshot e;
        e.key = key;
        e.sound = sound;
        // 4 m to the listener's right, sweeping in depth (−Z is in front): approaches
        // from far in front, passes the side, recedes behind.
        e.position = AudioVec3{4.0f, 0.0f, z};
        e.gain = 0.7f;
        e.priority = 5.0f;
        e.spatial = true;
        e.min_distance = 1.0f;
        e.max_distance = 200.0f;
        e.doppler_scale = 1.0f;
        return e;
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    AudioEngine engine(16, 8);
    const int master = engine.mixer().add_bus(NO_BUS);
    const int sfx = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);
    (void)sfx;
    engine.prepare(sample_rate, block);

    ToneFactory factory;
    AudioScene scene(engine.voices(), factory);

    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    auto render_frame = [&](double& peak, double& energy) {
        engine.render(channels, 2, block);
        for (int i = 0; i < block; ++i)
        {
            peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
            energy += std::fabs(static_cast<double>(left[i])) + std::fabs(static_cast<double>(right[i]));
        }
    };

    // --- 1. Headless reconciliation self-check ---------------------------------------

    const int frames = 120;          // "world" frames
    const int blocks_per_frame = 4;  // audio blocks rendered per world frame
    double peak = 0.0, energy = 0.0;

    for (int f = 0; f < frames; ++f)
    {
        const float t = static_cast<float>(f) / (frames - 1); // 0..1
        SceneSnapshot snap;
        // Emitter 1 flies from z=−60 (far front) to z=+60 (behind) across the run.
        const float z = -60.0f + 120.0f * t;
        snap.emitters.push_back(flyby(1, 2 /*even → low tone*/, z));

        // Halfway through, a second, closer emitter appears on the left.
        if (f >= frames / 2)
        {
            EmitterSnapshot e2 = flyby(2, 3 /*odd → high tone*/, 6.0f);
            e2.position = AudioVec3{-3.0f, 0.0f, 6.0f};
            snap.emitters.push_back(e2);
        }

        scene.apply(snap);

        if (f == frames / 4)
        {
            if (scene.voice_count() != 1)
            {
                std::fprintf(stderr, "audio_scene_demo FAILED: expected 1 voice, got %zu\n",
                             scene.voice_count());
                return 1;
            }
        }
        if (f == frames / 2 + 2)
        {
            if (scene.voice_count() != 2)
            {
                std::fprintf(stderr, "audio_scene_demo FAILED: expected 2 voices, got %zu\n",
                             scene.voice_count());
                return 1;
            }
        }

        for (int b = 0; b < blocks_per_frame; ++b)
            render_frame(peak, energy);
    }

    // Drop the first emitter from the snapshot: its voice must stop.
    {
        SceneSnapshot snap;
        EmitterSnapshot e2 = flyby(2, 3, 6.0f);
        e2.position = AudioVec3{-3.0f, 0.0f, 6.0f};
        snap.emitters.push_back(e2); // only emitter 2 remains
        scene.apply(snap);
        if (scene.voice_for(1) != INVALID_VOICE)
        {
            std::fprintf(stderr, "audio_scene_demo FAILED: dropped emitter's voice not stopped\n");
            return 1;
        }
    }

    std::printf("reconciliation OK: peak=%.4f, energy=%.1f, voices now=%zu\n",
                peak, energy, scene.voice_count());
    if (!(energy > 0.0))
    {
        std::fprintf(stderr, "audio_scene_demo FAILED: silent output\n");
        return 1;
    }
    if (peak > 2.0)
    {
        std::fprintf(stderr, "audio_scene_demo FAILED: output too hot (peak %.4f)\n", peak);
        return 1;
    }

    std::printf("headless scene checks passed\n");

    // --- 2. Best-effort audible flyby ------------------------------------------------

    // Start from a clean pool, then create the flyby voice *before* opening the device:
    // like the other spatial demos, only position updates cross to the audio thread
    // during playback (mid-stream voice start/stop is the deferred command-ring phase).
    scene.clear();
    {
        SceneSnapshot snap;
        snap.emitters.push_back(flyby(1, 2, -60.0f));
        scene.apply(snap); // creates emitter 1's voice on this (control) thread
    }

    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch — flyby (headphones for the L/R + Doppler)\n",
                    obtained.sample_rate, obtained.channel_count);
        for (int f = 0; f < 200; ++f)
        {
            const float t = static_cast<float>(f) / 199.0f;
            // Same emitter key, still playing → apply only moves the existing voice.
            SceneSnapshot snap;
            snap.emitters.push_back(flyby(1, 2, -60.0f + 120.0f * t));
            scene.apply(snap);
            std::this_thread::sleep_for(std::chrono::milliseconds(12));
        }
        device.close();
    }
    else
    {
        std::printf("no audio device available (headless) — scene verified in software\n");
    }

    std::printf("audio_scene_demo OK\n");
    return 0;
}
