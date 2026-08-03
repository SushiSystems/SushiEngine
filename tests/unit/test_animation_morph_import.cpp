/**************************************************************************/
/* test_animation_morph_import.cpp                                       */
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

// Unit_AnimationMorphImport: the glTF `weights` channel lane, end to end on a real file.
// A morph-weight track can only reach the SkinningPass if three things agree — the
// importer reads the channel, the cooked clip carries the track under the target's name,
// and sample_morph_state resolves a mesh's target order onto it by name. This pins all
// three against assets/models/morph_face.gltf: a skinned triangle with two named
// targets, one animation driving them and one driving only a joint rotation.

#include <cstddef>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/morph.hpp>
#include <SushiEngine/gltf/skeleton_import.hpp>

using namespace SushiEngine;

namespace
{
    constexpr float SAMPLE_RATE = 30.0f;

    const char* asset_path()
    {
        return SE_TEST_ASSET_DIR "/morph_face.gltf";
    }

    // The whole file, imported once: every test here reads the same two animations.
    const Animation::GLTFAnimationImport& imported()
    {
        static const Animation::GLTFAnimationImport import = []
        {
            Animation::GLTFAnimationImport result;
            Animation::import_gltf_animated(asset_path(), result, SAMPLE_RATE);
            return result;
        }();
        return import;
    }

    const Animation::GLTFClip* clip_named(const char* name)
    {
        for (const Animation::GLTFClip& clip : imported().clips)
            if (clip.name == name)
                return &clip;
        return nullptr;
    }
}

TEST(Unit_AnimationMorphImport, ImportsBothAnimationsAndTheTargetNames)
{
    ASSERT_FALSE(imported().skeleton_blob.empty());
    ASSERT_EQ(imported().clips.size(), 2u);
    ASSERT_NE(clip_named("talk"), nullptr);
    ASSERT_NE(clip_named("turn"), nullptr);

    // The mesh's target order, which the render mesh's delta buffer is uploaded in.
    ASSERT_EQ(imported().morph_target_names.size(), 2u);
    EXPECT_EQ(imported().morph_target_names[0], "jawOpen");
    EXPECT_EQ(imported().morph_target_names[1], "eyeBlinkLeft");
}

TEST(Unit_AnimationMorphImport, WeightsChannelBecomesOneTrackPerTarget)
{
    const Animation::GLTFClip* talk = clip_named("talk");
    ASSERT_NE(talk, nullptr);
    const Animation::ClipView clip = Animation::load_clip_blob(talk->blob.data(), talk->blob.size());
    ASSERT_TRUE(clip.valid());

    // One channel carries both targets, so it contributes two tracks, addressed by name.
    EXPECT_EQ(clip.morph_track_count, 2u);
    EXPECT_EQ(clip.find_morph(Animation::hash_name("jawOpen")), 0);
    EXPECT_EQ(clip.find_morph(Animation::hash_name("eyeBlinkLeft")), 1);
    EXPECT_EQ(clip.find_morph(Animation::hash_name("noSuchTarget")), -1);

    // The channel's last key is at 1.0 s, so 30 fps resampling spans 31 frames.
    EXPECT_EQ(clip.frame_count, 31u);
}

TEST(Unit_AnimationMorphImport, ResampledWeightsReproduceTheAuthoredKeys)
{
    const Animation::GLTFClip* talk = clip_named("talk");
    ASSERT_NE(talk, nullptr);
    const Animation::ClipView clip = Animation::load_clip_blob(talk->blob.data(), talk->blob.size());
    ASSERT_TRUE(clip.valid());

    const int jaw = clip.find_morph(Animation::hash_name("jawOpen"));
    const int blink = clip.find_morph(Animation::hash_name("eyeBlinkLeft"));
    ASSERT_GE(jaw, 0);
    ASSERT_GE(blink, 0);
    const auto weight = [&](float seconds, int track)
    {
        return clip.sample_morph_track(seconds, false, static_cast<std::uint32_t>(track));
    };

    // The three authored keys land on frames 0, 15 and 30 exactly at this sample rate.
    EXPECT_NEAR(weight(0.0f, jaw), 0.0f, 1e-5f);
    EXPECT_NEAR(weight(0.0f, blink), 1.0f, 1e-5f);
    EXPECT_NEAR(weight(0.5f, jaw), 1.0f, 1e-5f);
    EXPECT_NEAR(weight(0.5f, blink), 0.0f, 1e-5f);
    EXPECT_NEAR(weight(1.0f, jaw), 0.0f, 1e-5f);
    EXPECT_NEAR(weight(1.0f, blink), 0.25f, 1e-5f);

    // Between keys the channel is sampled linearly, not stepped: 7/30 s is 46.67% of the
    // way from the first key to the second.
    EXPECT_NEAR(weight(7.0f / SAMPLE_RATE, jaw), 7.0f / 15.0f, 1e-4f);
    EXPECT_NEAR(weight(7.0f / SAMPLE_RATE, blink), 1.0f - 7.0f / 15.0f, 1e-4f);
}

TEST(Unit_AnimationMorphImport, AnimationWithoutAWeightsChannelHasNoMorphTracks)
{
    const Animation::GLTFClip* turn = clip_named("turn");
    ASSERT_NE(turn, nullptr);
    const Animation::ClipView clip = Animation::load_clip_blob(turn->blob.data(), turn->blob.size());
    ASSERT_TRUE(clip.valid());
    EXPECT_EQ(clip.morph_track_count, 0u);
    EXPECT_GT(clip.joint_count, 0u);
}

TEST(Unit_AnimationMorphImport, MorphStateResolvesTargetsByNameNotPosition)
{
    const Animation::GLTFClip* talk = clip_named("talk");
    ASSERT_NE(talk, nullptr);
    const Animation::ClipView clip = Animation::load_clip_blob(talk->blob.data(), talk->blob.size());
    ASSERT_TRUE(clip.valid());

    // A mesh whose target order is the reverse of the clip's track order, plus a third
    // target no track drives — the case a positional mapping silently gets wrong.
    const Animation::NameHash targets[3] = {Animation::hash_name("eyeBlinkLeft"),
                                            Animation::hash_name("jawOpen"),
                                            Animation::hash_name("browInnerUp")};
    Animation::MorphState state;
    Animation::sample_morph_state(clip, 0.5f, false, targets, 3, state);

    ASSERT_EQ(state.count, 3u);
    EXPECT_NEAR(state.weights[0], 0.0f, 1e-5f); // eyeBlinkLeft at 0.5 s
    EXPECT_NEAR(state.weights[1], 1.0f, 1e-5f); // jawOpen at 0.5 s
    EXPECT_NEAR(state.weights[2], 0.0f, 1e-5f); // undriven, left at rest
}

TEST(Unit_AnimationMorphImport, MorphStateZeroesEveryTargetForAClipWithoutTracks)
{
    const Animation::GLTFClip* turn = clip_named("turn");
    ASSERT_NE(turn, nullptr);
    const Animation::ClipView clip = Animation::load_clip_blob(turn->blob.data(), turn->blob.size());
    ASSERT_TRUE(clip.valid());

    const Animation::NameHash targets[2] = {Animation::hash_name("jawOpen"),
                                            Animation::hash_name("eyeBlinkLeft")};
    Animation::MorphState state;
    state.weights[0] = 0.75f; // stale weight from a previous frame
    Animation::sample_morph_state(clip, 0.5f, false, targets, 2, state);

    ASSERT_EQ(state.count, 2u);
    EXPECT_NEAR(state.weights[0], 0.0f, 1e-5f);
    EXPECT_NEAR(state.weights[1], 0.0f, 1e-5f);
}

TEST(Unit_AnimationMorphImport, JointTracksStillImportAlongsideTheWeightsChannel)
{
    // The channel loop routes weights and joint paths in the same pass; a rotation channel
    // must still resample, and a weights-only animation must still be one frame per joint
    // holding the bind pose rather than an empty clip.
    const Animation::GLTFClip* turn = clip_named("turn");
    ASSERT_NE(turn, nullptr);
    const Animation::ClipView rotated =
        Animation::load_clip_blob(turn->blob.data(), turn->blob.size());
    ASSERT_TRUE(rotated.valid());
    ASSERT_EQ(rotated.joint_count, 1u);
    ASSERT_EQ(rotated.frame_count, 31u);

    // The authored keys are identity at 0 s and a 90 degree turn at 1 s.
    Animation::Vector3f translation{};
    Animation::Quaternionf rotation{};
    Animation::Vector3f scale{};
    rotated.sample(0.0f, false, &translation, &rotation, &scale);
    EXPECT_NEAR(rotation.w, 1.0f, 1e-5f);
    rotated.sample(1.0f, false, &translation, &rotation, &scale);
    EXPECT_NEAR(rotation.z, 0.7071068f, 1e-4f);
    EXPECT_NEAR(rotation.w, 0.7071068f, 1e-4f);

    const Animation::GLTFClip* talk = clip_named("talk");
    ASSERT_NE(talk, nullptr);
    const Animation::ClipView weighted =
        Animation::load_clip_blob(talk->blob.data(), talk->blob.size());
    ASSERT_TRUE(weighted.valid());
    EXPECT_EQ(weighted.joint_count, 1u);
}
