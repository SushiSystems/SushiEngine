/**************************************************************************/
/* test_animation_clip.cpp                                                */
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

// Unit_AnimationClip: the asset layer the whole animation stack sits on — the skeleton
// and clip blob cooks, their refusals, the sampling contract (clamp vs. wrap), the
// evaluator's model-space compose, and the compressed format's error bound. Phases A1/A2
// shipped these with `clip_demo`/`animation_benchmark` as their only check; this is the
// guarded version.
//
// The skeleton cook's topological reorder is the single most load-bearing invariant here:
// `parent[i] < i` is what lets every consumer compose a pose in one forward scan, and the
// reorder it performs to get there is what makes joint indices *not* the authored ones —
// a gotcha that has produced real bugs in this subsystem more than once.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/clip_compress.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    const Quaternionf IDENTITY{0.0f, 0.0f, 0.0f, 1.0f};

    Quaternionf turn_about_z(float radians)
    {
        return quaternion_axis_angle(Vector3T<float>{0.0f, 0.0f, 1.0f}, radians);
    }

    JointDescription joint(const char* name, int parent,
                           Vector3f bind_translation = Vector3f{0, 0, 0})
    {
        JointDescription description;
        description.name = name;
        description.parent = parent;
        description.bind_translation = bind_translation;
        return description;
    }

    // A two-joint arm: root at the origin, child one unit down +X.
    SkeletonDescription arm_skeleton()
    {
        SkeletonDescription description;
        description.joints = {joint("root", -1), joint("child", 0, Vector3f{1.0f, 0.0f, 0.0f})};
        return description;
    }

    // A clip over that arm whose only motion is the root turning 90 degrees about Z, so the
    // child's model-space position traces a quarter circle and every sample has a closed form.
    ClipDescription quarter_turn_clip()
    {
        ClipDescription clip;
        clip.joint_count = 2;
        clip.frame_count = 2;
        clip.sample_rate = 1.0f;
        clip.translations = {Vector3f{0, 0, 0}, Vector3f{1, 0, 0},
                             Vector3f{0, 0, 0}, Vector3f{1, 0, 0}};
        clip.rotations = {IDENTITY, IDENTITY, turn_about_z(PI * 0.5f), IDENTITY};
        clip.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
        return clip;
    }

    Vector3 model_position(const ClipEvaluator& evaluator, std::uint32_t index)
    {
        const Matrix4& matrix = evaluator.model()[index];
        return Vector3{matrix.m[12], matrix.m[13], matrix.m[14]};
    }

    constexpr float COMPRESSION_THRESHOLD = 0.002f;

    // A clip long and wide enough for the compressed format's segmenting and per-segment
    // range reduction to matter. `swing` is the rotation amplitude in radians and
    // `moving_translation` decides whether translation is rig-fixed (the usual case for a
    // character) or animated every frame (hostile to range reduction).
    ClipDescription synthetic_clip(std::uint32_t joints, std::uint32_t frames, float swing,
                           bool moving_translation)
    {
        ClipDescription description;
        description.joint_count = joints;
        description.frame_count = frames;
        description.sample_rate = 30.0f;
        const std::size_t count = static_cast<std::size_t>(joints) * frames;
        description.translations.resize(count);
        description.rotations.resize(count);
        description.scales.assign(count, Vector3f{1, 1, 1});
        for (std::uint32_t f = 0; f < frames; ++f)
            for (std::uint32_t j = 0; j < joints; ++j)
            {
                const std::size_t index = static_cast<std::size_t>(f) * joints + j;
                const float phase = static_cast<float>(f) / description.sample_rate * PI +
                                    static_cast<float>(j) * 0.15f;
                description.translations[index] =
                    moving_translation
                        ? Vector3f{std::sin(phase) * 0.5f, static_cast<float>(j) * 0.1f,
                                   std::cos(phase) * 0.25f}
                        : Vector3f{0.0f, 0.1f, 0.0f};
                description.rotations[index] = turn_about_z(std::sin(phase) * swing);
            }
        return description;
    }

    // The claim "transparent quality" rests on: the decoded clip must stay inside the
    // threshold it was solved for at every frame and every joint, measured against the raw
    // clip rather than against the compressor's own idea of its error.
    void expect_reconstruction_within_threshold(const ClipDescription& description,
                                               const std::vector<std::byte>& raw_blob,
                                               const std::vector<std::byte>& compressed_blob)
    {
        const ClipView raw = load_clip_blob(raw_blob.data(), raw_blob.size());
        const ClipView compressed =
            load_any_clip_blob(compressed_blob.data(), compressed_blob.size());
        ASSERT_TRUE(raw.valid());
        ASSERT_TRUE(compressed.valid());

        const std::uint32_t joints = description.joint_count;
        std::vector<Vector3f> raw_translations(joints), raw_scales(joints);
        std::vector<Quaternionf> raw_rotations(joints);
        std::vector<Vector3f> got_translations(joints), got_scales(joints);
        std::vector<Quaternionf> got_rotations(joints);
        float worst_translation = 0.0f;
        float worst_rotation = 0.0f;
        for (std::uint32_t f = 0; f < description.frame_count; ++f)
        {
            const float seconds = static_cast<float>(f) / description.sample_rate;
            raw.sample(seconds, false, raw_translations.data(), raw_rotations.data(),
                       raw_scales.data());
            compressed.sample(seconds, false, got_translations.data(), got_rotations.data(),
                              got_scales.data());
            for (std::uint32_t j = 0; j < joints; ++j)
            {
                const Vector3f& a = raw_translations[j];
                const Vector3f& b = got_translations[j];
                worst_translation =
                    std::max(worst_translation,
                             std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                                       (a.z - b.z) * (a.z - b.z)));
                // Rotation error as the displacement of a virtual vertex at unit distance —
                // the measure the bit-rate solver itself minimizes.
                const Quaternionf& p = raw_rotations[j];
                const Quaternionf& q = got_rotations[j];
                const float dot = p.x * q.x + p.y * q.y + p.z * q.z + p.w * q.w;
                worst_rotation = std::max(
                    worst_rotation, 2.0f * std::sqrt(std::max(0.0f, 1.0f - dot * dot)));
            }
        }
        EXPECT_LE(worst_translation, COMPRESSION_THRESHOLD) << "worst translation error";
        EXPECT_LE(worst_rotation, COMPRESSION_THRESHOLD * 8.0f) << "worst rotation error";
    }
}

TEST(Unit_AnimationClip, SkeletonCookSortsEveryParentBeforeItsChild)
{
    // Authored deepest-first, the worst case for the cook: nothing is already in order.
    SkeletonDescription description;
    description.joints = {joint("hand", 1), joint("forearm", 2), joint("shoulder", -1)};

    std::vector<std::byte> blob;
    std::vector<int> order;
    ASSERT_TRUE(build_skeleton_blob(description, blob, &order));
    const SkeletonView skeleton = load_skeleton_blob(blob.data(), blob.size());
    ASSERT_TRUE(skeleton.valid());
    ASSERT_EQ(skeleton.joint_count, 3u);

    // The invariant every forward-scan compose depends on.
    EXPECT_EQ(skeleton.parents[0], NO_PARENT);
    for (std::uint32_t i = 1; i < skeleton.joint_count; ++i)
        EXPECT_LT(skeleton.parents[i], i) << "joint " << i << " precedes its parent";

    // And the reason a caller must resolve by name: the cooked indices are not the
    // authored ones, so an index carried over from the source data addresses another joint.
    const int shoulder = skeleton.find_joint(hash_name("shoulder"));
    const int forearm = skeleton.find_joint(hash_name("forearm"));
    const int hand = skeleton.find_joint(hash_name("hand"));
    ASSERT_GE(shoulder, 0);
    ASSERT_GE(forearm, 0);
    ASSERT_GE(hand, 0);
    EXPECT_EQ(shoulder, 0);
    EXPECT_LT(forearm, hand);
    EXPECT_EQ(skeleton.parents[forearm], static_cast<std::uint16_t>(shoulder));
    EXPECT_EQ(skeleton.parents[hand], static_cast<std::uint16_t>(forearm));
    EXPECT_EQ(skeleton.find_joint(hash_name("nonexistent")), -1);

    // out_order maps a cooked index back to the authored one, which is how the clip cook
    // resamples into the blob's order.
    ASSERT_EQ(order.size(), 3u);
    EXPECT_EQ(order[static_cast<std::size_t>(shoulder)], 2);
    EXPECT_EQ(order[static_cast<std::size_t>(hand)], 0);
}

TEST(Unit_AnimationClip, SkeletonCookRefusesWhatItCannotRepresent)
{
    std::vector<std::byte> blob;
    EXPECT_FALSE(build_skeleton_blob(SkeletonDescription{}, blob)) << "an empty skeleton";
    EXPECT_TRUE(blob.empty());

    SkeletonDescription too_many;
    too_many.joints.resize(MAX_JOINTS + 1, joint("j", -1));
    EXPECT_FALSE(build_skeleton_blob(too_many, blob)) << "more joints than MAX_JOINTS";
}

TEST(Unit_AnimationClip, SkeletonBlobValidationRejectsDamagedBytes)
{
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_skeleton_blob(arm_skeleton(), blob));
    ASSERT_TRUE(validate_skeleton_blob(blob.data(), blob.size()));

    EXPECT_FALSE(validate_skeleton_blob(nullptr, blob.size()));
    EXPECT_FALSE(validate_skeleton_blob(blob.data(), sizeof(SkeletonBlobHeader) - 1))
        << "a buffer shorter than the header";
    EXPECT_FALSE(validate_skeleton_blob(blob.data(), blob.size() - 1))
        << "total_size reaching past the buffer";

    std::vector<std::byte> wrong_magic = blob;
    wrong_magic[0] = std::byte{'X'};
    EXPECT_FALSE(validate_skeleton_blob(wrong_magic.data(), wrong_magic.size()));

    // A future format version must be refused, not misread as this one.
    std::vector<std::byte> wrong_version = blob;
    const std::uint32_t bumped = SKELETON_BLOB_VERSION + 1;
    std::memcpy(wrong_version.data() + offsetof(SkeletonBlobHeader, version), &bumped,
                sizeof(bumped));
    EXPECT_FALSE(validate_skeleton_blob(wrong_version.data(), wrong_version.size()));

    // And a view over refused bytes must be invalid rather than dangling.
    EXPECT_FALSE(load_skeleton_blob(wrong_magic.data(), wrong_magic.size()).valid());
}

TEST(Unit_AnimationClip, ClipCookRoundTripsEveryTrackKind)
{
    ClipDescription clip = quarter_turn_clip();
    clip.morph_names = {"jawOpen", "eyeBlinkLeft"};
    clip.morph_weights = {0.0f, 1.0f, 1.0f, 0.5f}; // frame-major over two tracks
    clip.generic_names = {"emission"};
    clip.generic_values = {0.25f, 0.75f};

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(clip, blob));
    const ClipView view = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(view.valid());

    EXPECT_EQ(view.joint_count, 2u);
    EXPECT_EQ(view.frame_count, 2u);
    EXPECT_FLOAT_EQ(view.sample_rate, 1.0f);
    EXPECT_FLOAT_EQ(view.duration, 1.0f) << "(frame_count - 1) / sample_rate";
    EXPECT_EQ(view.format, ClipFormat::Raw);

    for (std::size_t i = 0; i < clip.rotations.size(); ++i)
    {
        EXPECT_FLOAT_EQ(view.translations[i].x, clip.translations[i].x);
        EXPECT_FLOAT_EQ(view.rotations[i].w, clip.rotations[i].w);
        EXPECT_FLOAT_EQ(view.scales[i].y, clip.scales[i].y);
    }

    ASSERT_EQ(view.morph_track_count, 2u);
    ASSERT_EQ(view.generic_track_count, 1u);
    EXPECT_EQ(view.find_morph(hash_name("eyeBlinkLeft")), 1);
    EXPECT_EQ(view.find_generic(hash_name("emission")), 0);
    // Names are stored as hashes, so a name the clip does not carry must miss, not alias.
    EXPECT_EQ(view.find_morph(hash_name("emission")), -1);
}

TEST(Unit_AnimationClip, ClipCookRefusesMissizedOrDegenerateDescriptions)
{
    std::vector<std::byte> blob;

    ClipDescription short_track = quarter_turn_clip();
    short_track.rotations.pop_back();
    EXPECT_FALSE(build_clip_blob(short_track, blob)) << "a track shorter than frames * joints";

    ClipDescription no_frames = quarter_turn_clip();
    no_frames.frame_count = 0;
    EXPECT_FALSE(build_clip_blob(no_frames, blob));

    ClipDescription bad_rate = quarter_turn_clip();
    bad_rate.sample_rate = 0.0f;
    EXPECT_FALSE(build_clip_blob(bad_rate, blob));

    // A morph name with no weights behind it is a mis-sized clip, not an empty track.
    ClipDescription dangling_morph = quarter_turn_clip();
    dangling_morph.morph_names = {"jawOpen"};
    EXPECT_FALSE(build_clip_blob(dangling_morph, blob));
}

TEST(Unit_AnimationClip, SamplingClampsWhenItDoesNotLoopAndWrapsWhenItDoes)
{
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(quarter_turn_clip(), blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());

    Vector3f translations[2];
    Quaternionf rotations[2];
    Vector3f scales[2];
    const auto root_turn = [&](float seconds, bool loop)
    {
        clip.sample(seconds, loop, translations, rotations, scales);
        return rotations[0];
    };

    const Quaternionf turned = turn_about_z(PI * 0.5f);

    // Non-looping clamps at both ends rather than extrapolating or wrapping.
    EXPECT_NEAR(root_turn(-5.0f, false).w, 1.0f, 1e-5f);
    EXPECT_NEAR(root_turn(0.0f, false).w, 1.0f, 1e-5f);
    EXPECT_NEAR(root_turn(1.0f, false).z, turned.z, 1e-5f);
    EXPECT_NEAR(root_turn(50.0f, false).z, turned.z, 1e-5f) << "clamped to the last frame";

    // Looping wraps cyclically: one full period later is frame 0 again. The period is
    // frame_count / sample_rate (the last frame blends back to the first), not duration.
    EXPECT_NEAR(root_turn(2.0f, true).w, 1.0f, 1e-5f);
    EXPECT_NEAR(root_turn(-2.0f, true).w, 1.0f, 1e-5f) << "negative time wraps forward";
}

TEST(Unit_AnimationClip, SamplingInterpolatesBetweenAuthoredFrames)
{
    ClipDescription description = quarter_turn_clip();
    // Three frames so there is an interior frame to land on exactly as well as between.
    description.frame_count = 3;
    description.translations = {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}, Vector3f{0, 0, 0},
                               Vector3f{1, 0, 0}, Vector3f{0, 0, 0}, Vector3f{1, 0, 0}};
    description.rotations = {IDENTITY, IDENTITY, turn_about_z(PI * 0.5f), IDENTITY,
                             turn_about_z(PI), IDENTITY};
    description.scales.assign(6, Vector3f{1, 1, 1});

    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(description, blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());
    ASSERT_FLOAT_EQ(clip.duration, 2.0f);

    std::uint32_t frame0 = 0;
    std::uint32_t frame1 = 0;
    float alpha = 0.0f;
    clip.bracket(1.5f, false, frame0, frame1, alpha);
    EXPECT_EQ(frame0, 1u);
    EXPECT_EQ(frame1, 2u);
    EXPECT_NEAR(alpha, 0.5f, 1e-5f);

    // Halfway between a 90 and a 180 degree turn is 135, and the result stays a unit
    // quaternion — nlerp, not a component lerp that would shorten it.
    Vector3f translations[2];
    Quaternionf rotations[2];
    Vector3f scales[2];
    clip.sample(1.5f, false, translations, rotations, scales);
    const Quaternionf expected = turn_about_z(PI * 0.75f);
    EXPECT_NEAR(rotations[0].z, expected.z, 2e-3f);
    EXPECT_NEAR(rotations[0].w, expected.w, 2e-3f);
    const float length = std::sqrt(rotations[0].x * rotations[0].x + rotations[0].y * rotations[0].y +
                                  rotations[0].z * rotations[0].z + rotations[0].w * rotations[0].w);
    EXPECT_NEAR(length, 1.0f, 1e-5f);
}

TEST(Unit_AnimationClip, EvaluatorComposesModelSpaceThroughTheHierarchy)
{
    AnimationDatabase database;
    std::vector<std::byte> skeleton_blob;
    std::vector<std::byte> clip_blob;
    ASSERT_TRUE(build_skeleton_blob(arm_skeleton(), skeleton_blob));
    ASSERT_TRUE(build_clip_blob(quarter_turn_clip(), clip_blob));
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const AssetId clip_id = database.add_clip(std::move(clip_blob));
    ASSERT_NE(skeleton_id, INVALID_ASSET);
    ASSERT_NE(clip_id, INVALID_ASSET);

    // The database shares one id space across asset kinds, so an id must not resolve as
    // the wrong kind — the mix-up would otherwise reinterpret one blob as another.
    EXPECT_NE(skeleton_id, clip_id);
    EXPECT_TRUE(database.has_skeleton(skeleton_id));
    EXPECT_TRUE(database.has_clip(clip_id));
    EXPECT_FALSE(database.has_clip(skeleton_id));
    EXPECT_FALSE(database.has_skeleton(clip_id));

    const SkeletonView skeleton = database.skeleton(skeleton_id);
    const ClipView clip = database.clip(clip_id);
    ClipEvaluator evaluator;

    // Frame 0 is the bind pose, so every skin matrix (model x inverse-bind) is identity.
    evaluator.evaluate(skeleton, clip, 0.0f, true);
    for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
        for (int k = 0; k < 16; ++k)
            EXPECT_NEAR(evaluator.palette()[j].m[k], (k % 5 == 0) ? 1.0f : 0.0f, 1e-5f)
                << "palette joint " << j << " element " << k;
    Vector3 rest = model_position(evaluator, 1);
    EXPECT_NEAR(rest.x, 1.0, 1e-4);
    EXPECT_NEAR(rest.y, 0.0, 1e-4);

    // The child holds its own bind transform throughout; everything below comes from the
    // parent's rotation being composed into it, which is what the forward scan is for.
    evaluator.evaluate(skeleton, clip, 1.0f, false);
    Vector3 turned = model_position(evaluator, 1);
    EXPECT_NEAR(turned.x, 0.0, 1e-4);
    EXPECT_NEAR(turned.y, 1.0, 1e-4);
    EXPECT_NEAR(turned.z, 0.0, 1e-4);

    evaluator.evaluate(skeleton, clip, 0.5f, false);
    Vector3 halfway = model_position(evaluator, 1);
    const double diagonal = std::sqrt(0.5);
    EXPECT_NEAR(halfway.x, diagonal, 2e-3);
    EXPECT_NEAR(halfway.y, diagonal, 2e-3);
    // Rotation preserves length: the child never leaves the unit circle mid-turn.
    EXPECT_NEAR(std::sqrt(halfway.x * halfway.x + halfway.y * halfway.y), 1.0, 2e-3);

    evaluator.evaluate(skeleton, clip, 2.0f, true);
    Vector3 wrapped = model_position(evaluator, 1);
    EXPECT_NEAR(wrapped.x, 1.0, 1e-4);
    EXPECT_NEAR(wrapped.y, 0.0, 1e-4);
}

TEST(Unit_AnimationClip, CompressionReachesAclClassRatiosOnLocomotionContent)
{
    // The shape design §0.7's "6.6x-17.6x at transparent quality" was measured on, and the
    // shape real character animation has: a long chain of joints whose translation is fixed
    // by the rig and whose rotation swings gently. That is what per-segment range reduction
    // and the three-smallest-component quaternion encoding are built to exploit.
    const ClipDescription description =
        synthetic_clip(80, 181, 0.35f, /*moving_translation=*/false);

    std::vector<std::byte> raw_blob;
    std::vector<std::byte> compressed_blob;
    ASSERT_TRUE(build_clip_blob(description, raw_blob));
    ASSERT_TRUE(compress_clip(description, COMPRESSION_THRESHOLD, compressed_blob));

    const double ratio = static_cast<double>(raw_blob.size()) /
                         static_cast<double>(compressed_blob.size());
    EXPECT_GT(ratio, 6.0) << "compressed to " << compressed_blob.size() << " of "
                          << raw_blob.size() << " bytes";
    expect_reconstruction_within_threshold(description, raw_blob, compressed_blob);
}

TEST(Unit_AnimationClip, CompressionHoldsItsErrorBoundOnContentThatCompressesBadly)
{
    // The same threshold against content deliberately hostile to the format: every joint's
    // translation moves every frame and the rotations swing through most of a radian, so
    // the per-segment ranges stay wide and the bit-rate solver cannot economize. The ratio
    // is expected to be far worse than above — what must *not* degrade is the error bound,
    // since a compressor that met its ratio by exceeding its threshold would be wrong.
    const ClipDescription description = synthetic_clip(40, 121, 0.9f, /*moving_translation=*/true);

    std::vector<std::byte> raw_blob;
    std::vector<std::byte> compressed_blob;
    ASSERT_TRUE(build_clip_blob(description, raw_blob));
    ASSERT_TRUE(compress_clip(description, COMPRESSION_THRESHOLD, compressed_blob));

    const double ratio = static_cast<double>(raw_blob.size()) /
                         static_cast<double>(compressed_blob.size());
    EXPECT_GT(ratio, 2.0) << "compressed to " << compressed_blob.size() << " of "
                          << raw_blob.size() << " bytes";
    expect_reconstruction_within_threshold(description, raw_blob, compressed_blob);
}

TEST(Unit_AnimationClip, ClipLoadingRecognizesTheFormatFromTheBytes)
{
    // A consumer holds bytes, not a flag saying which cook produced them, so the loader has
    // to tell raw from compressed itself — and refuse anything that is neither.
    const ClipDescription description = synthetic_clip(4, 16, 0.4f, false);
    std::vector<std::byte> raw_blob;
    std::vector<std::byte> compressed_blob;
    ASSERT_TRUE(build_clip_blob(description, raw_blob));
    ASSERT_TRUE(compress_clip(description, COMPRESSION_THRESHOLD, compressed_blob));

    EXPECT_EQ(load_any_clip_blob(raw_blob.data(), raw_blob.size()).format, ClipFormat::Raw);
    const ClipView compressed = load_any_clip_blob(compressed_blob.data(), compressed_blob.size());
    ASSERT_TRUE(compressed.valid());
    EXPECT_EQ(compressed.format, ClipFormat::Compressed);
    EXPECT_EQ(compressed.joint_count, description.joint_count);
    EXPECT_EQ(compressed.frame_count, description.frame_count);

    // Each loader must refuse the other's format rather than reinterpreting its header.
    EXPECT_FALSE(validate_clip_blob(compressed_blob.data(), compressed_blob.size()));
    EXPECT_FALSE(validate_compressed_clip_blob(raw_blob.data(), raw_blob.size()));
    EXPECT_FALSE(load_any_clip_blob(nullptr, 0).valid());

    std::vector<std::byte> garbage(compressed_blob.size(), std::byte{0x5A});
    EXPECT_FALSE(load_any_clip_blob(garbage.data(), garbage.size()).valid());
}
