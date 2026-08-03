/**************************************************************************/
/* test_audio_ecs.cpp                                                     */
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

// Integration_AudioECS: the phase-S6 audio snapshot extract against the real ECS world
// and runtime. It verifies that the wall-clock read of AudioListener/AudioEmitter/
// ReverbZone columns produces a correct listener-local snapshot (facing from the
// Orientation quaternion, emitter positions eye-subtracted in double, the containing
// reverb zone), that the extract never mutates the world (audio on/off is byte-
// identical), and that driving an AudioScene from it starts and moves the right voices.

#include <memory>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/simulation/audio_extract.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    class ToneFactory final : public Audio::IEmitterSourceFactory
    {
        public:
            std::unique_ptr<Audio::VoiceSource> create(std::uint32_t sound_id) override
            {
                (void)sound_id;
                return std::unique_ptr<Audio::VoiceSource>(new Audio::ToneSource(440.0f, 1.0f));
            }
    };
} // namespace

TEST(Integration_AudioECS, ExtractReadsListenerFacingAndListenerLocalEmitters)
{
    World world(Harness::shared_context(), 256);

    // Listener at (10,0,0), identity orientation → world glTF facing (−Z forward, +Y up).
    world.spawn(Transform{Vector3{10, 0, 0}}, Orientation{}, AudioListener{});
    // An emitter 5 m in front of the listener (−Z) and 3 m to its right (+X world).
    world.spawn(Transform{Vector3{13, 0, -5}}, AudioEmitter{});

    Audio::SceneSnapshot snap;
    build_audio_snapshot(world, snap);

    EXPECT_NEAR(snap.listener_forward.x, 0.0f, 1e-5f);
    EXPECT_NEAR(snap.listener_forward.y, 0.0f, 1e-5f);
    EXPECT_NEAR(snap.listener_forward.z, -1.0f, 1e-5f);
    EXPECT_NEAR(snap.listener_up.y, 1.0f, 1e-5f);

    ASSERT_EQ(snap.emitters.size(), 1u);
    // Position is relative to the listener: (13,0,-5) − (10,0,0) = (3,0,-5).
    EXPECT_NEAR(snap.emitters[0].position.x, 3.0f, 1e-5f);
    EXPECT_NEAR(snap.emitters[0].position.z, -5.0f, 1e-5f);
    EXPECT_TRUE(snap.emitters[0].spatial);
    EXPECT_TRUE(snap.emitters[0].playing);
}

TEST(Integration_AudioECS, ExtractPicksTheContainingReverbZoneAndLeavesTheWorldUnchanged)
{
    World world(Harness::shared_context(), 256);
    world.spawn(Transform{Vector3{0, 0, 0}}, Orientation{}, AudioListener{});
    const Entity emitter = world.spawn(Transform{Vector3{0, 0, -4}}, AudioEmitter{});

    // A cave zone whose 20 m box contains the listener at the origin.
    ReverbZone zone;
    zone.half_extents = Vector3{20, 20, 20};
    zone.reverb = Audio::I3DL2Reverb::cave();
    world.spawn(Transform{Vector3{0, 0, 0}}, zone);

    Audio::SceneSnapshot snap;
    build_audio_snapshot(world, snap);

    EXPECT_TRUE(snap.has_reverb);
    EXPECT_FLOAT_EQ(snap.reverb.decay_time, Audio::I3DL2Reverb::cave().decay_time);

    // The extract only reads: the emitter's transform is untouched (on/off byte-identical).
    EXPECT_DOUBLE_EQ(world.get<Transform>(emitter).position.z, -4.0);
}

TEST(Integration_AudioECS, ExtractOutsideAnyZoneReportsNoReverb)
{
    World world(Harness::shared_context(), 256);
    world.spawn(Transform{Vector3{0, 0, 0}}, Orientation{}, AudioListener{});

    ReverbZone zone;
    zone.half_extents = Vector3{5, 5, 5};
    zone.reverb = Audio::I3DL2Reverb::cave();
    world.spawn(Transform{Vector3{1000, 0, 0}}, zone); // far from the listener

    Audio::SceneSnapshot snap;
    build_audio_snapshot(world, snap);
    EXPECT_FALSE(snap.has_reverb);
}

TEST(Integration_AudioECS, ExtractDrivesAudioSceneVoicesAsEmittersMove)
{
    World world(Harness::shared_context(), 256);
    world.spawn(Transform{Vector3{0, 0, 0}}, Orientation{}, AudioListener{});
    const Entity emitter = world.spawn(Transform{Vector3{0, 0, -3}}, AudioEmitter{});

    Audio::VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    Audio::AudioScene scene(voices, factory);
    Audio::SceneSnapshot scratch;

    extract_audio_scene(world, scene, scratch);
    EXPECT_EQ(scene.voice_count(), 1u);
    // The scene keys emitters as (generation << 32) | index, matching the extract.
    const std::uint64_t key =
        (static_cast<std::uint64_t>(emitter.generation) << 32) | emitter.index;
    const int handle = scene.voice_for(key);
    EXPECT_NE(handle, Audio::INVALID_VOICE);

    // Move the emitter in the world; the extract updates the *same* voice, not a new one.
    world.get<Transform>(emitter).position = Vector3{0, 0, -30};
    extract_audio_scene(world, scene, scratch);
    EXPECT_EQ(scene.voice_count(), 1u);
    EXPECT_EQ(scene.voice_for(key), handle);

    // Destroy the emitter; the next extract stops its voice.
    world.destroy(emitter);
    extract_audio_scene(world, scene, scratch);
    EXPECT_EQ(scene.voice_count(), 0u);
}
