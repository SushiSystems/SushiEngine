/**************************************************************************/
/* test_audio_scene.cpp                                                  */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_Audio: the phase-S6 control-plane bridge (AudioScene) in isolation — the
// snapshot → live-voice reconciliation: an emitter appearing starts a voice, one that
// persists moves and re-gains it, one that goes silent or vanishes from the snapshot is
// stopped, and an active reverb zone steers the reverb effect. Pure header-only, no ECS
// and no runtime — the snapshot is hand-built.

#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // Resolves every sound id to a tone, except id 0 which yields no source (to exercise
    // the "no source available this frame" path).
    class ToneFactory final : public IEmitterSourceFactory
    {
        public:
            int created = 0;

            std::unique_ptr<VoiceSource> create(std::uint32_t sound_id) override
            {
                if (sound_id == 0)
                    return nullptr;
                ++created;
                return std::unique_ptr<VoiceSource>(new ToneSource(440.0f, 1.0f));
            }
    };

    EmitterSnapshot make_emitter(std::uint64_t key, std::uint32_t sound, AudioVec3 pos)
    {
        EmitterSnapshot e;
        e.key = key;
        e.sound = sound;
        e.position = pos;
        e.gain = 1.0f;
        e.priority = 1.0f;
        e.spatial = true;
        e.min_distance = 1.0f;
        e.max_distance = 100.0f;
        return e;
    }
} // namespace

TEST(Unit_Audio, AudioSceneStartsAVoiceWhenAnEmitterAppears)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    SceneSnapshot snap;
    snap.emitters.push_back(make_emitter(1001, 7, AudioVec3{2.0f, 0.0f, 0.0f}));
    scene.apply(snap);

    EXPECT_EQ(factory.created, 1);
    EXPECT_EQ(scene.voice_count(), 1u);
    EXPECT_NE(scene.voice_for(1001), INVALID_VOICE);

    // A second apply with the same emitter must NOT create a second voice.
    scene.apply(snap);
    EXPECT_EQ(factory.created, 1);
    EXPECT_EQ(scene.voice_count(), 1u);
}

TEST(Unit_Audio, AudioSceneMovesAndStopsVoicesFromSnapshotChanges)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    SceneSnapshot snap;
    snap.emitters.push_back(make_emitter(1, 5, AudioVec3{1.0f, 0.0f, 0.0f}));
    snap.emitters.push_back(make_emitter(2, 5, AudioVec3{-1.0f, 0.0f, 0.0f}));
    scene.apply(snap);
    ASSERT_EQ(scene.voice_count(), 2u);
    const int handle1 = scene.voice_for(1);

    // Emitter 1 moves; emitter 2 goes silent; the same handles are reused/stopped.
    snap.emitters[0].position = AudioVec3{50.0f, 0.0f, 0.0f};
    snap.emitters[1].playing = false;
    scene.apply(snap);

    EXPECT_EQ(scene.voice_for(1), handle1); // moved, not recreated
    EXPECT_EQ(scene.voice_for(2), INVALID_VOICE); // stopped
    EXPECT_EQ(scene.voice_count(), 1u);
}

TEST(Unit_Audio, AudioSceneStopsVoicesForVanishedEmitters)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    SceneSnapshot snap;
    snap.emitters.push_back(make_emitter(42, 3, AudioVec3{0.0f, 0.0f, 3.0f}));
    scene.apply(snap);
    ASSERT_EQ(scene.voice_count(), 1u);

    // The emitter entity was destroyed → it is simply absent from the next snapshot.
    SceneSnapshot empty;
    scene.apply(empty);
    EXPECT_EQ(scene.voice_count(), 0u);
    EXPECT_EQ(scene.voice_for(42), INVALID_VOICE);
}

TEST(Unit_Audio, AudioSceneRetriesWhenNoSourceIsAvailable)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    // sound id 0 → factory returns nullptr, so no voice is created yet.
    SceneSnapshot snap;
    snap.emitters.push_back(make_emitter(9, 0, AudioVec3{1.0f, 0.0f, 0.0f}));
    scene.apply(snap);
    EXPECT_EQ(scene.voice_count(), 0u);

    // The sound becomes resolvable; the retry now starts the voice.
    snap.emitters[0].sound = 12;
    scene.apply(snap);
    EXPECT_EQ(scene.voice_count(), 1u);
}

TEST(Unit_Audio, AudioSceneSteersReverbFromActiveZone)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    FDNReverbEffect reverb;
    reverb.prepare(48000.0, 512);
    reverb.set_parameters(I3DL2Reverb::generic());
    scene.set_reverb(&reverb);

    SceneSnapshot snap;
    snap.has_reverb = true;
    snap.reverb = I3DL2Reverb::cave();
    scene.apply(snap);

    // The cave's long decay was pushed into the effect.
    EXPECT_FLOAT_EQ(reverb.parameters().decay_time, I3DL2Reverb::cave().decay_time);
    EXPECT_FLOAT_EQ(reverb.parameters().decay_hf_ratio, I3DL2Reverb::cave().decay_hf_ratio);
}

TEST(Unit_Audio, AudioSceneClearStopsEverything)
{
    VoiceManager voices(16, 8);
    voices.prepare(48000.0, 512);
    ToneFactory factory;
    AudioScene scene(voices, factory);

    SceneSnapshot snap;
    snap.emitters.push_back(make_emitter(1, 1, AudioVec3{1.0f, 0.0f, 0.0f}));
    snap.emitters.push_back(make_emitter(2, 1, AudioVec3{2.0f, 0.0f, 0.0f}));
    scene.apply(snap);
    ASSERT_EQ(scene.voice_count(), 2u);

    scene.clear();
    EXPECT_EQ(scene.voice_count(), 0u);
    EXPECT_EQ(voices.active_count(), 0);
}
