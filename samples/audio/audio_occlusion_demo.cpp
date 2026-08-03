/**************************************************************************/
/* audio_occlusion_demo.cpp                                             */
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
 * @file audio_occlusion_demo.cpp
 * @brief Phase S7 vertical slice: occlusion/obstruction, rooms/portals, early reflections.
 *
 * The geometry layer decides what is blocked; the DSP turns that into sound. This:
 *
 *   1. Runs headless and self-checks the pieces against the real code: the acoustic BVH
 *      (a wall occludes, a clear line does not, moving the wall via a TLAS refit flips
 *      the answer), three-band material transmission (through concrete the lows leak more
 *      than the highs), the room/portal graph (a cross-room source resolves to a doorway
 *      secondary source), image-source early reflections, and the *integrated* path — a
 *      source rendered through the voice manager is quieter when occluded than when clear.
 *      No hardware needed — this is the CI check.
 *   2. Best-effort plays a tone that slides left-to-right and passes behind a concrete
 *      wall, with the occlusion driven live from the acoustic scene each block, so you
 *      hear it muffle and drop as it goes behind cover and open back up past it.
 *
 * Exits 0 on success.
 */

#include <algorithm>
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
    double rms(const float* v, int n)
    {
        double s = 0.0;
        for (int i = 0; i < n; ++i)
            s += static_cast<double>(v[i]) * v[i];
        return std::sqrt(s / (n > 0 ? n : 1));
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    // --- 1a. Acoustic BVH: a wall occludes; a clear line does not; refit flips it -----
    AcousticMesh mesh;
    AcousticBlas blas;
    const std::uint32_t concrete = mesh.add_material(AcousticMaterial::concrete());
    mesh.add_box(AudioVec3{0, 0, 0}, AudioVec3{0.3f, 4.0f, 4.0f}, concrete);
    blas.build(mesh);

    AcousticScene scene;
    AcousticInstance wall;
    wall.blas = &blas;
    wall.set_position(AudioVec3{0, 0, 0});
    const std::size_t wall_id = scene.add_instance(wall);
    scene.commit();

    if (!scene.occluded(AudioVec3{-6, 0, 0}, AudioVec3{6, 0, 0}))
    {
        std::fprintf(stderr, "audio_occlusion_demo FAILED: wall did not occlude\n");
        return 1;
    }
    if (scene.occluded(AudioVec3{-6, 10, 0}, AudioVec3{6, 10, 0}))
    {
        std::fprintf(stderr, "audio_occlusion_demo FAILED: clear line reported occluded\n");
        return 1;
    }

    float trans[3];
    scene.line_of_sight(AudioVec3{-6, 0, 0}, AudioVec3{6, 0, 0}, 4, trans);
    std::printf("wall transmission: low=%.4f mid=%.4f high=%.4f (bassy → low > high)\n",
                trans[0], trans[1], trans[2]);
    if (!(trans[0] > trans[2]))
    {
        std::fprintf(stderr, "audio_occlusion_demo FAILED: transmission not bassy\n");
        return 1;
    }

    scene.instance(wall_id).set_position(AudioVec3{0, 100, 0});
    scene.refit();
    if (scene.occluded(AudioVec3{-6, 0, 0}, AudioVec3{6, 0, 0}))
    {
        std::fprintf(stderr, "audio_occlusion_demo FAILED: refit did not move the wall\n");
        return 1;
    }
    scene.instance(wall_id).set_position(AudioVec3{0, 0, 0});
    scene.refit();

    // --- 1b. Room/portal graph: a cross-room source becomes a doorway source ----------
    {
        PortalGraph graph;
        AcousticAABB ra, rb;
        ra.min = AudioVec3{-10, -4, -4}; ra.max = AudioVec3{-0.1f, 4, 4};
        rb.min = AudioVec3{0.1f, -4, -4}; rb.max = AudioVec3{10, 4, 4};
        graph.add_room(1, ra);
        graph.add_room(2, rb);
        graph.add_portal(1, 2, AudioVec3{0, 0, 0}, AudioVec3{0.1f, 1.2f, 1.2f});
        graph.build();

        const PortalResolution pr =
            graph.resolve(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}, 3.0f, 2);
        std::printf("portal: same_room=%d reachable=%d doorways=%zu gain=%.3f\n",
                    pr.same_room ? 1 : 0, pr.source_reachable ? 1 : 0, pr.doorways.size(),
                    pr.doorways.empty() ? 0.0f : pr.doorways[0].gain);
        if (pr.same_room || !pr.source_reachable || pr.doorways.empty())
        {
            std::fprintf(stderr, "audio_occlusion_demo FAILED: cross-room did not resolve to a doorway\n");
            return 1;
        }
    }

    // --- 1c. Image-source early reflections -------------------------------------------
    {
        std::vector<ReflectionTap> taps;
        ImageSourceModel::compute(AudioVec3{0, 0, 0}, AudioVec3{2, 0, 0}, AudioVec3{0, 0, 0},
                                  AudioVec3{6, 5, 4}, 0.7f, 343.0f, taps);
        std::printf("early reflections: %zu taps (first delay %.2f ms)\n", taps.size(),
                    taps.empty() ? 0.0f : taps[0].delay_seconds * 1000.0f);
        if (taps.size() != 6u)
        {
            std::fprintf(stderr, "audio_occlusion_demo FAILED: shoebox did not yield six taps\n");
            return 1;
        }
    }

    // --- 1d. Integrated: a source is quieter through the voice manager when occluded ---
    auto integrated_level = [&](bool occluded) {
        AudioEngine engine(8, 4);
        const int master = engine.mixer().add_bus(NO_BUS);
        engine.mixer().set_master(master);
        engine.prepare(sample_rate, block);
        engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

        VoiceDescriptor d;
        d.base_gain = 0.8f;
        d.priority = 10.0f;
        d.bus = master;
        d.spatial = true;
        d.position = AudioVec3{-6.0f, 0.0f, 0.0f};
        d.min_distance = 1.0f;
        d.max_distance = 50.0f;
        d.propagation_delay = false; // static source: isolate the occlusion effect
        const int handle = engine.voices().play(
            d, std::unique_ptr<VoiceSource>(new ToneSource(1000.0f, 0.8f)));

        const float open_t[3] = {1.0f, 1.0f, 1.0f};
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        float* channels[2] = {l.data(), r.data()};
        double level = 0.0;
        for (int b = 0; b < 60; ++b)
        {
            if (occluded)
            {
                const OcclusionResult o =
                    scene.soft_occlusion(d.position, AudioVec3{0, 0, 0}, 0.5f, 12, 4);
                engine.voices().set_voice_occlusion(handle, 0.0f, o.fraction, o.transmission);
            }
            else
            {
                engine.voices().set_voice_occlusion(handle, 0.0f, 0.0f, open_t);
            }
            engine.render(channels, 2, block);
            if (b >= 50) // measure once settled
                level += 0.5 * (rms(l.data(), block) + rms(r.data(), block));
        }
        return level;
    };

    const double clear_level = integrated_level(false);
    const double occluded_level = integrated_level(true);
    std::printf("integrated: clear level=%.5f  occluded level=%.5f\n", clear_level, occluded_level);
    if (!(occluded_level < clear_level * 0.7))
    {
        std::fprintf(stderr, "audio_occlusion_demo FAILED: occlusion did not attenuate the voice\n");
        return 1;
    }

    std::printf("headless occlusion checks passed\n");

    // --- 2. Best-effort audible playback: a tone slides behind the wall ---------------
    AudioEngine engine(8, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    const int reverb_bus = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);
    {
        std::unique_ptr<FDNReverbEffect> fx(new FDNReverbEffect());
        I3DL2Reverb room = I3DL2Reverb::room_small();
        room.wet_dry_mix = 100.0f;
        fx->set_parameters(room);
        engine.mixer().add_insert(
            reverb_bus, std::unique_ptr<IBusEffect>(new ReverbBusEffect(std::move(fx))));
    }
    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    VoiceDescriptor d;
    d.base_gain = 0.7f;
    d.priority = 10.0f;
    d.bus = master;
    d.spatial = true;
    d.position = AudioVec3{-8.0f, 0.0f, -3.0f};
    d.min_distance = 1.0f;
    d.max_distance = 60.0f;
    d.reverb_bus = reverb_bus;
    d.reverb_send = 0.5f;
    const int handle = engine.voices().play(
        d, std::unique_ptr<VoiceSource>(new ToneSource(440.0f, 0.7f)));

    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    SDLAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch, %d frames/block — tone sliding behind a wall\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);
        const double seconds = 5.0;
        const int total_blocks = static_cast<int>(sample_rate * seconds / block);
        for (int b = 0; b < total_blocks; ++b)
        {
            // Slide the source from x=−8 to x=+8 (crossing the wall at x=0).
            const float x = -8.0f + 16.0f * static_cast<float>(b) / static_cast<float>(total_blocks);
            const AudioVec3 pos{x, 0.0f, -3.0f};
            engine.voices().set_voice_position(handle, pos);
            const OcclusionResult o = scene.soft_occlusion(pos, AudioVec3{0, 0, 0}, 0.5f, 12, 4);
            engine.voices().set_voice_occlusion(handle, 0.0f, o.fraction, o.transmission);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<int>(1000.0 * block / sample_rate)));
        }
        device.close();
    }
    else
    {
        double peak = 0.0;
        for (int b = 0; b < 200; ++b)
        {
            const float x = -8.0f + 16.0f * static_cast<float>(b) / 200.0f;
            const AudioVec3 pos{x, 0.0f, -3.0f};
            engine.voices().set_voice_position(handle, pos);
            const OcclusionResult o = scene.soft_occlusion(pos, AudioVec3{0, 0, 0}, 0.5f, 12, 4);
            engine.voices().set_voice_occlusion(handle, 0.0f, o.fraction, o.transmission);
            engine.render(channels, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
        }
        std::printf("no audio device available (headless) — rendered flyby, peak=%.4f\n", peak);
        if (peak > 4.0)
        {
            std::fprintf(stderr, "audio_occlusion_demo FAILED: mix unbounded (peak %.4f)\n", peak);
            return 1;
        }
    }

    std::printf("audio_occlusion_demo OK\n");
    return 0;
}
