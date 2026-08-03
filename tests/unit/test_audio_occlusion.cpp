/**************************************************************************/
/* test_audio_occlusion.cpp                                               */
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

// Unit_Audio: the phase-S7 occlusion/obstruction layer — the acoustic BVH (BLAS ray
// hit/miss, TLAS instance placement + refit), three-band material transmission (through-
// wall sound is bassy), soft (multi-ray) occlusion fraction, the occlusion DSP filter
// (obstruction muffles the dry, occlusion also pulls the reverb send down), the room/
// portal graph (a cross-room source becomes a doorway secondary source), and the image-
// source early reflections. Pure header-only maths against the real code.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // A YZ-plane "wall" box centred at the origin, thin in x, tall/wide in y/z.
    AcousticScene make_wall_scene(AcousticMesh& mesh, AcousticBlas& blas,
                                  const AcousticMaterial& material,
                                  const AudioVec3& center, const AudioVec3& half)
    {
        const std::uint32_t m = mesh.add_material(material);
        mesh.add_box(center, half, m);
        blas.build(mesh);
        AcousticScene scene;
        AcousticInstance inst;
        inst.blas = &blas;
        inst.set_position(AudioVec3{0, 0, 0});
        scene.add_instance(inst);
        scene.commit();
        return scene;
    }

    double block_rms(const std::vector<float>& v)
    {
        double s = 0.0;
        for (float x : v)
            s += static_cast<double>(x) * x;
        return std::sqrt(s / static_cast<double>(v.empty() ? 1 : v.size()));
    }

    // Fill a block with a sine at frequency f, phase-continuous across calls.
    void fill_sine(std::vector<float>& block, double freq, double sr, double& phase)
    {
        const double two_pi = 6.283185307179586;
        const double inc = two_pi * freq / sr;
        for (float& s : block)
        {
            s = static_cast<float>(std::sin(phase));
            phase += inc;
            if (phase >= two_pi)
                phase -= two_pi;
        }
    }
}

// The BLAS ray test: a segment through the wall is blocked; one that clears it is not.
TEST(Unit_Audio, OcclusionBVHLineOfSight)
{
    AcousticMesh mesh;
    AcousticBlas blas;
    AcousticScene scene = make_wall_scene(mesh, blas, AcousticMaterial::concrete(),
                                          AudioVec3{0, 0, 0}, AudioVec3{0.2f, 5.0f, 5.0f});

    EXPECT_TRUE(scene.occluded(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}));
    // Above the wall (y = 10 > half-height 5): clear.
    EXPECT_FALSE(scene.occluded(AudioVec3{-5, 10, 0}, AudioVec3{5, 10, 0}));
    // A path that never crosses x = 0 (both endpoints on the same side): clear.
    EXPECT_FALSE(scene.occluded(AudioVec3{-5, 0, 0}, AudioVec3{-3, 0, 0}));
}

// Three-band transmission: through concrete, the low band leaks far more than the high.
TEST(Unit_Audio, OcclusionTransmissionIsBassy)
{
    AcousticMesh mesh;
    AcousticBlas blas;
    AcousticScene scene = make_wall_scene(mesh, blas, AcousticMaterial::concrete(),
                                          AudioVec3{0, 0, 0}, AudioVec3{0.2f, 5.0f, 5.0f});

    float t[3];
    ASSERT_TRUE(scene.line_of_sight(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}, 4, t));
    EXPECT_GT(t[0], t[2]);       // low leaks more than high
    EXPECT_LT(t[2], 0.05f);      // high is heavily blocked
    EXPECT_LE(t[0], 1.0f);
}

// Soft occlusion: fully behind a wall → fraction 1; well clear → 0; straddling → between.
TEST(Unit_Audio, OcclusionSoftFraction)
{
    AcousticMesh mesh;
    AcousticBlas blas;
    AcousticScene scene = make_wall_scene(mesh, blas, AcousticMaterial::concrete(),
                                          AudioVec3{0, 0, 0}, AudioVec3{0.2f, 5.0f, 5.0f});

    const OcclusionResult behind =
        scene.soft_occlusion(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}, 0.4f, 16, 4);
    EXPECT_FLOAT_EQ(behind.fraction, 1.0f);

    const OcclusionResult clear =
        scene.soft_occlusion(AudioVec3{-5, 20, 0}, AudioVec3{5, 20, 0}, 0.4f, 16, 4);
    EXPECT_FLOAT_EQ(clear.fraction, 0.0f);

    // Source straddling the top edge (y ≈ 5) with a large sampling radius.
    const OcclusionResult edge =
        scene.soft_occlusion(AudioVec3{-3, 5, 0}, AudioVec3{5, 5, 0}, 2.5f, 32, 4);
    EXPECT_GT(edge.fraction, 0.0f);
    EXPECT_LT(edge.fraction, 1.0f);
}

// Moving an instance and refitting the TLAS changes the occlusion answer.
TEST(Unit_Audio, OcclusionTlasRefit)
{
    AcousticMesh mesh;
    AcousticBlas blas;
    const std::uint32_t m = mesh.add_material(AcousticMaterial::concrete());
    mesh.add_box(AudioVec3{0, 0, 0}, AudioVec3{0.2f, 3.0f, 3.0f}, m);
    blas.build(mesh);

    AcousticScene scene;
    AcousticInstance inst;
    inst.blas = &blas;
    inst.set_position(AudioVec3{0, 0, 0});
    const std::size_t id = scene.add_instance(inst);
    scene.commit();
    EXPECT_TRUE(scene.occluded(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}));

    // Slide the wall far out of the way and refit.
    scene.instance(id).set_position(AudioVec3{0, 100, 0});
    scene.refit();
    EXPECT_FALSE(scene.occluded(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}));
}

// The occlusion DSP: obstruction muffles the dry (high loses more than low).
TEST(Unit_Audio, OcclusionFilterMufflesHighs)
{
    const double sr = 48000.0;
    const int block = 480;
    const float open_t[3] = {1.0f, 1.0f, 1.0f};

    auto settled_gain = [&](double freq, float obstruction) {
        OcclusionFilter f;
        f.prepare(sr, block);
        f.set_targets(obstruction, 0.0f, open_t);
        std::vector<float> buffer(static_cast<std::size_t>(block));
        double phase = 0.0, in_rms = 0.0, out_rms = 0.0;
        for (int b = 0; b < 80; ++b)
        {
            fill_sine(buffer, freq, sr, phase);
            in_rms = block_rms(buffer);
            f.process(buffer.data(), block);
            out_rms = block_rms(buffer);
        }
        return static_cast<float>(out_rms / (in_rms + 1e-12));
    };

    const float low_gain = settled_gain(200.0, 1.0f);
    const float high_gain = settled_gain(8000.0, 1.0f);
    EXPECT_LT(high_gain, low_gain);   // the edge-diffraction low-pass darkens the tone
    EXPECT_LT(high_gain, 0.5f);       // 8 kHz is strongly attenuated when fully obstructed

    // With no blockage the filter is essentially transparent.
    const float open_gain = settled_gain(1000.0, 0.0f);
    EXPECT_GT(open_gain, 0.9f);
}

// The occlusion DSP: occlusion pulls the reverb send down; obstruction does not.
TEST(Unit_Audio, OcclusionFilterWetSend)
{
    const double sr = 48000.0;
    const int block = 480;
    const float open_t[3] = {1.0f, 1.0f, 1.0f};
    std::vector<float> buffer(static_cast<std::size_t>(block), 0.0f);

    OcclusionFilter obstruct;
    obstruct.prepare(sr, block);
    obstruct.set_targets(1.0f, 0.0f, open_t); // pillar: dry blocked, reverb open
    float wet_obstruct = 1.0f;
    for (int b = 0; b < 80; ++b)
    {
        std::fill(buffer.begin(), buffer.end(), 0.0f);
        wet_obstruct = obstruct.process(buffer.data(), block);
    }
    EXPECT_GT(wet_obstruct, 0.98f);

    OcclusionFilter occlude;
    occlude.prepare(sr, block);
    occlude.set_targets(0.0f, 1.0f, open_t); // wall: dry and reverb blocked
    float wet_occlude = 1.0f;
    for (int b = 0; b < 80; ++b)
    {
        std::fill(buffer.begin(), buffer.end(), 0.0f);
        wet_occlude = occlude.process(buffer.data(), block);
    }
    EXPECT_LT(wet_occlude, 0.02f);
}

// The room/portal graph: a cross-room source is heard through the doorway.
TEST(Unit_Audio, PortalGraphDoorwaySource)
{
    PortalGraph graph;
    AcousticAABB room_a, room_b;
    room_a.min = AudioVec3{-10, -5, -5}; room_a.max = AudioVec3{-0.1f, 5, 5};
    room_b.min = AudioVec3{0.1f, -5, -5}; room_b.max = AudioVec3{10, 5, 5};
    graph.add_room(1, room_a);
    graph.add_room(2, room_b);
    graph.add_portal(1, 2, AudioVec3{0, 0, 0}, AudioVec3{0.1f, 1.5f, 1.5f});
    graph.build();

    EXPECT_EQ(graph.room_of(AudioVec3{-5, 0, 0}), 1u);
    EXPECT_EQ(graph.room_of(AudioVec3{5, 0, 0}), 2u);

    const PortalResolution cross =
        graph.resolve(AudioVec3{-5, 0, 0}, AudioVec3{5, 0, 0}, 3.0f, 2);
    EXPECT_FALSE(cross.same_room);
    EXPECT_TRUE(cross.source_reachable);
    ASSERT_EQ(cross.doorways.size(), 1u);
    EXPECT_NEAR(cross.doorways[0].position.x, 0.0f, 1e-4f);
    EXPECT_NEAR(cross.doorways[0].path_length, 10.0f, 1e-3f); // 5 + 5
    EXPECT_NEAR(cross.doorways[0].gain, 0.3f, 1e-3f);         // ref 3 / path 10

    const PortalResolution same =
        graph.resolve(AudioVec3{-5, 0, 0}, AudioVec3{-3, 0, 0}, 3.0f, 2);
    EXPECT_TRUE(same.same_room);
    EXPECT_TRUE(same.doorways.empty());
}

// Image-source early reflections: a shoebox yields six taps, nearer wall arrives first.
TEST(Unit_Audio, ImageSourceShoeboxTaps)
{
    std::vector<ReflectionTap> taps;
    ImageSourceModel::compute(AudioVec3{0, 0, 0}, AudioVec3{3, 0, 0}, AudioVec3{0, 0, 0},
                              AudioVec3{5, 4, 3}, 0.7f, 343.0f, taps);
    ASSERT_EQ(taps.size(), 6u);
    for (const ReflectionTap& t : taps)
        EXPECT_GT(t.delay_seconds, 0.0f);

    // +x image at (10,0,0): listener (3,0,0) → 7 m; −x image at (−10,0,0) → 13 m.
    EXPECT_LT(taps[1].delay_seconds, taps[0].delay_seconds);

    EXPECT_NEAR(ImageSourceModel::reflectivity_from_absorption(0.0f), 1.0f, 1e-6f);
    EXPECT_NEAR(ImageSourceModel::reflectivity_from_absorption(1.0f), 0.0f, 1e-6f);
}

// The early-reflection renderer places a delayed, attenuated copy at the tap delay.
TEST(Unit_Audio, EarlyReflectionsRendersTap)
{
    const double sr = 48000.0;
    const int block = 256;
    EarlyReflections er;
    er.prepare(sr, block, 0.05f);

    std::vector<ReflectionTap> taps(1);
    taps[0].delay_seconds = 0.01f; // 480 samples
    taps[0].gain = 0.5f;
    er.set_taps(taps);

    std::vector<float> in(static_cast<std::size_t>(block), 0.0f);
    std::vector<float> out(static_cast<std::size_t>(block), 0.0f);
    std::vector<float> tail;
    in[0] = 1.0f; // an impulse in the first block
    for (int b = 0; b < 4; ++b)
    {
        er.process(in.data(), out.data(), block);
        for (float s : out)
            tail.push_back(s);
        std::fill(in.begin(), in.end(), 0.0f);
    }

    float peak = 0.0f;
    int peak_index = -1;
    for (int i = 0; i < static_cast<int>(tail.size()); ++i)
    {
        if (std::fabs(tail[static_cast<std::size_t>(i)]) > peak)
        {
            peak = std::fabs(tail[static_cast<std::size_t>(i)]);
            peak_index = i;
        }
    }
    EXPECT_NEAR(peak, 0.5f, 0.05f);
    EXPECT_NEAR(peak_index, 480, 2);
}
