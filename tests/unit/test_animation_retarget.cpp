/**************************************************************************/
/* test_animation_retarget.cpp                                            */
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

// Humanoid retargeting: the avatar that names which joint plays which canonical bone, and the
// three transfers built on it — a clip retargeted onto a differently-proportioned rig, a clip
// mirrored left-to-right on one rig, and the per-frame version that does the same for a target
// chosen at runtime.
//
// The property every case here is really asserting is the one design §4.4 is built on: what
// transfers is the *delta from the source's bind pose*, never the source's absolute rotation.
// The two rigs are therefore deliberately given different bind poses, because a suite whose
// source and target bind identically cannot tell the two apart and would pass against a
// retargeter that just copied rotations across.

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/humanoid.hpp>
#include <SushiEngine/animation/retarget.hpp>
#include <SushiEngine/animation/runtime_retarget.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    const Quaternionf IDENTITY{0.0f, 0.0f, 0.0f, 1.0f};

    /** @brief A rotation of @p degrees about a normalized axis. */
    Quaternionf axis_angle(float x, float y, float z, float degrees)
    {
        const float radians = degrees * 3.14159265358979323846f / 180.0f;
        const float half = radians * 0.5f;
        const float s = std::sin(half);
        return normalize(Quaternionf{x * s, y * s, z * s, std::cos(half)});
    }

    /** @brief The angle between two rotations, in degrees; zero when they agree. */
    float angle_between(const Quaternionf& a, const Quaternionf& b)
    {
        const Quaternionf delta = normalize(mul(conjugate(a), b));
        const float w = std::fabs(delta.w) > 1.0f ? 1.0f : std::fabs(delta.w);
        return 2.0f * std::acos(w) * 180.0f / 3.14159265358979323846f;
    }

    /**
     * @brief One authored joint: a name, a parent, and a bind pose.
     *
     * The bind rotation is what makes these tests meaningful, so it is authored per joint
     * rather than defaulted.
     */
    struct Joint
    {
        std::string name;
        int parent;
        Vector3f translation;
        Quaternionf rotation;
    };

    /** @brief A cooked skeleton and its blob, kept together so the view stays valid. */
    class Rig
    {
        public:
            explicit Rig(const std::vector<Joint>& joints)
            {
                SkeletonDescription description;
                description.joints.resize(joints.size());
                for (std::size_t i = 0; i < joints.size(); ++i)
                {
                    description.joints[i].name = joints[i].name;
                    description.joints[i].parent = joints[i].parent;
                    description.joints[i].bind_translation = joints[i].translation;
                    description.joints[i].bind_rotation = joints[i].rotation;
                }
                built_ = build_skeleton_blob(description, blob_);
                if (built_)
                    view_ = load_skeleton_blob(blob_.data(), blob_.size());
            }

            bool built() const noexcept { return built_ && view_.valid(); }
            const SkeletonView& view() const noexcept { return view_; }

            /** @brief The joint index playing @p name; the cook reorders, so never assume one. */
            std::uint32_t joint(const char* name) const
            {
                const int index = view_.find_joint(hash_name(name));
                EXPECT_GE(index, 0) << "rig has no joint named " << name;
                return static_cast<std::uint32_t>(index >= 0 ? index : 0);
            }

        private:
            std::vector<std::byte> blob_;
            SkeletonView view_;
            bool built_ = false;
    };

    /**
     * @brief A minimal humanoid: hips, spine, head, and both arms.
     *
     * @param hip_height     The hips' bind translation in +Y, which is what the root-motion
     *                       scale is derived from.
     * @param arm_length     Length of each arm segment, so two rigs can differ in proportion.
     * @param shoulder_droop A bind rotation applied to both upper arms, so the two rigs bind
     *                       *differently* and a delta transfer is distinguishable from a copy.
     */
    std::vector<Joint> humanoid_joints(float hip_height, float arm_length, float shoulder_droop)
    {
        const Quaternionf droop = axis_angle(0.0f, 0.0f, 1.0f, shoulder_droop);
        return {
            {"Hips", -1, Vector3f{0.0f, hip_height, 0.0f}, IDENTITY},
            {"Spine", 0, Vector3f{0.0f, 0.2f, 0.0f}, IDENTITY},
            {"Head", 1, Vector3f{0.0f, 0.3f, 0.0f}, IDENTITY},
            {"LeftUpperArm", 1, Vector3f{0.2f, 0.1f, 0.0f}, droop},
            {"LeftLowerArm", 3, Vector3f{arm_length, 0.0f, 0.0f}, IDENTITY},
            {"RightUpperArm", 1, Vector3f{-0.2f, 0.1f, 0.0f}, droop},
            {"RightLowerArm", 5, Vector3f{-arm_length, 0.0f, 0.0f}, IDENTITY},
        };
    }

    /** @brief A clip seeded to @p rig's bind pose, ready for a caller to animate one joint. */
    ClipDescription bind_clip(const Rig& rig, std::uint32_t frame_count)
    {
        ClipDescription clip;
        clip.joint_count = rig.view().joint_count;
        clip.frame_count = frame_count;
        clip.sample_rate = 30.0f;
        detail::seed_bind_tracks(rig.view(), clip);
        return clip;
    }
} // namespace

TEST(Unit_AnimationRetarget,TheHeuristicMapsCommonNamingConventions)
{
    // Three conventions in one rig, which is what the alias table exists for: a glTF-style
    // "LeftUpperArm", a Mixamo-style "LeftForeArm", and an Unreal-style "thigh_l".
    const Rig rig({{"Pelvis", -1, Vector3f{0.0f, 1.0f, 0.0f}, IDENTITY},
                   {"spine_01", 0, Vector3f{0.0f, 0.2f, 0.0f}, IDENTITY},
                   {"LeftUpperArm", 1, Vector3f{0.2f, 0.0f, 0.0f}, IDENTITY},
                   {"LeftForeArm", 2, Vector3f{0.3f, 0.0f, 0.0f}, IDENTITY},
                   {"thigh_l", 0, Vector3f{0.1f, -0.1f, 0.0f}, IDENTITY}});
    ASSERT_TRUE(rig.built());

    const Avatar avatar = build_avatar_heuristic(rig.view());
    EXPECT_EQ(avatar.joint(HumanBone::Hips), std::int32_t(rig.joint("Pelvis")));
    EXPECT_EQ(avatar.joint(HumanBone::Spine), std::int32_t(rig.joint("spine_01")));
    EXPECT_EQ(avatar.joint(HumanBone::LeftUpperArm), std::int32_t(rig.joint("LeftUpperArm")));
    EXPECT_EQ(avatar.joint(HumanBone::LeftLowerArm), std::int32_t(rig.joint("LeftForeArm")));
    EXPECT_EQ(avatar.joint(HumanBone::LeftUpperLeg), std::int32_t(rig.joint("thigh_l")));

    // A bone the rig has no name for stays unmapped rather than defaulting to a joint, because
    // a wrong mapping poses the wrong limb and an absent one poses nothing.
    EXPECT_FALSE(avatar.has(HumanBone::RightUpperArm));
    EXPECT_FALSE(avatar.has(HumanBone::Head));
    EXPECT_EQ(avatar.mapped_count(), 5u);
}

TEST(Unit_AnimationRetarget,AnAuthoredMappingReachesRigsTheHeuristicCannot)
{
    // The escape hatch for a rig whose naming no alias list will ever cover.
    const Rig rig({{"root_00", -1, Vector3f{0.0f, 1.0f, 0.0f}, IDENTITY},
                   {"seg_a", 0, Vector3f{0.0f, 0.2f, 0.0f}, IDENTITY},
                   {"seg_b", 1, Vector3f{0.0f, 0.3f, 0.0f}, IDENTITY}});
    ASSERT_TRUE(rig.built());

    EXPECT_EQ(build_avatar_heuristic(rig.view()).mapped_count(), 0u);

    AvatarDescription description;
    description.entries.push_back({HumanBone::Hips, "root_00"});
    description.entries.push_back({HumanBone::Spine, "seg_a"});
    description.entries.push_back({HumanBone::Head, "seg_b"});
    description.entries.push_back({HumanBone::Neck, "does_not_exist"});

    const Avatar avatar = build_avatar(description, rig.view());
    EXPECT_EQ(avatar.joint(HumanBone::Hips), std::int32_t(rig.joint("root_00")));
    EXPECT_EQ(avatar.joint(HumanBone::Head), std::int32_t(rig.joint("seg_b")));
    // A named joint the rig lacks leaves its bone unmapped rather than failing the whole
    // avatar: an author fixing one typo should not lose the twenty entries that were right.
    EXPECT_FALSE(avatar.has(HumanBone::Neck));
    EXPECT_EQ(avatar.mapped_count(), 3u);
}

TEST(Unit_AnimationRetarget,EveryLateralBoneMirrorsAndEveryCentralOneDoesNot)
{
    // `opposite` is a switch, and a switch is where a copy-paste error lives. Asserting the
    // whole enum is what catches a lateral bone that maps to itself, which would silently make
    // `mirror_clip` a no-op for that limb.
    EXPECT_EQ(opposite(HumanBone::LeftHand), HumanBone::RightHand);
    EXPECT_EQ(opposite(HumanBone::RightToes), HumanBone::LeftToes);
    for (std::uint32_t b = 0; b < HUMAN_BONE_COUNT; ++b)
    {
        const HumanBone bone = static_cast<HumanBone>(b);
        // Mirroring is an involution: the opposite of the opposite is the bone itself.
        EXPECT_EQ(opposite(opposite(bone)), bone);
    }

    const HumanBone central[] = {HumanBone::Hips, HumanBone::Spine, HumanBone::Chest,
                                 HumanBone::UpperChest, HumanBone::Neck, HumanBone::Head};
    for (const HumanBone bone : central)
        EXPECT_EQ(opposite(bone), bone);

    // And no lateral bone is its own opposite, which is the other half of the same mistake.
    for (std::uint32_t b = 0; b < HUMAN_BONE_COUNT; ++b)
    {
        const HumanBone bone = static_cast<HumanBone>(b);
        bool is_central = false;
        for (const HumanBone c : central)
            is_central = is_central || c == bone;
        if (!is_central)
            EXPECT_NE(opposite(bone), bone) << "bone " << b << " has no mirror";
    }
}

TEST(Unit_AnimationRetarget,ARetargetedJointBendsByTheSameDeltaAndNotToTheSameRotation)
{
    // The whole of §4.4 in one assertion. The two rigs bind their upper arms 20 degrees apart,
    // so "the target ends up where the source was" and "the target bends as far as the source
    // bent" are different answers, and only the second one is right.
    const Rig source(humanoid_joints(1.0f, 0.30f, 0.0f));
    const Rig target(humanoid_joints(1.5f, 0.45f, 20.0f));
    ASSERT_TRUE(source.built());
    ASSERT_TRUE(target.built());

    const Avatar source_avatar = build_avatar_heuristic(source.view());
    const Avatar target_avatar = build_avatar_heuristic(target.view());
    ASSERT_TRUE(source_avatar.has(HumanBone::LeftUpperArm));
    ASSERT_TRUE(target_avatar.has(HumanBone::LeftUpperArm));

    const std::uint32_t source_arm = source.joint("LeftUpperArm");
    const std::uint32_t target_arm = target.joint("LeftUpperArm");

    ClipDescription clip = bind_clip(source, 1);
    const Quaternionf lift = axis_angle(0.0f, 0.0f, 1.0f, 35.0f);
    clip.rotations[source_arm] = normalize(mul(source.view().bind_rotations[source_arm], lift));

    ClipDescription retargeted;
    ASSERT_TRUE(retarget_clip(clip, source_avatar, source.view(), target_avatar, target.view(),
                              retargeted));
    ASSERT_EQ(retargeted.joint_count, target.view().joint_count);
    ASSERT_EQ(retargeted.frame_count, 1u);

    // The delta the target carries is the delta the source carried.
    const Quaternionf produced = retargeted.rotations[target_arm];
    const Quaternionf target_delta =
        normalize(mul(conjugate(target.view().bind_rotations[target_arm]), produced));
    EXPECT_NEAR(angle_between(target_delta, lift), 0.0f, 1e-3f);

    // And the absolute rotation is *not* the source's, because the binds differ by 20 degrees.
    // Without this the test would pass against a retargeter that copied rotations verbatim.
    EXPECT_GT(angle_between(produced, clip.rotations[source_arm]), 15.0f);

    // A target joint playing no canonical bone the source has holds its own bind pose, rather
    // than being left at whatever the vector was constructed with.
    const std::uint32_t head = target.joint("Head");
    EXPECT_NEAR(angle_between(retargeted.rotations[head], target.view().bind_rotations[head]),
                0.0f, 1e-4f);
}

TEST(Unit_AnimationRetarget,RootTranslationScalesWithHipHeightSoStrideFollowsProportion)
{
    // A taller rig must take a longer stride from the same clip, or a walk cycle slides. The
    // scale is the hip-height ratio, and the *offset from bind* is what it multiplies — not the
    // absolute position, which would move the target's hips to the source's height.
    const Rig source(humanoid_joints(1.0f, 0.30f, 0.0f));
    const Rig target(humanoid_joints(2.0f, 0.45f, 0.0f));
    ASSERT_TRUE(source.built());
    ASSERT_TRUE(target.built());

    const Avatar source_avatar = build_avatar_heuristic(source.view());
    const Avatar target_avatar = build_avatar_heuristic(target.view());
    const std::uint32_t source_hips = source.joint("Hips");
    const std::uint32_t target_hips = target.joint("Hips");

    ClipDescription clip = bind_clip(source, 1);
    clip.translations[source_hips] =
        source.view().bind_translations[source_hips] + Vector3f{0.4f, 0.0f, 0.0f};

    ClipDescription retargeted;
    ASSERT_TRUE(retarget_clip(clip, source_avatar, source.view(), target_avatar, target.view(),
                              retargeted));

    const Vector3f produced = retargeted.translations[target_hips];
    const Vector3f offset = produced - target.view().bind_translations[target_hips];
    EXPECT_NEAR(offset.x, 0.8f, 1e-4f); // 0.4 stride at a 2.0/1.0 hip ratio
    EXPECT_NEAR(offset.y, 0.0f, 1e-4f);
    // The hips sit at the *target's* bind height plus the stride, not the source's.
    EXPECT_NEAR(produced.y, 2.0f, 1e-4f);
}

TEST(Unit_AnimationRetarget,RetargetingBetweenIdenticalRigsIsTheIdentity)
{
    // The degenerate case worth pinning: when both rigs are the same, the transfer must give
    // the clip back. A sign error in the delta transfer survives every proportion test above
    // (both sides move) and dies here.
    const Rig rig(humanoid_joints(1.0f, 0.30f, 12.0f));
    ASSERT_TRUE(rig.built());
    const Avatar avatar = build_avatar_heuristic(rig.view());

    const std::uint32_t arm = rig.joint("LeftUpperArm");
    ClipDescription clip = bind_clip(rig, 3);
    for (std::uint32_t f = 0; f < 3; ++f)
    {
        const std::uint32_t index = f * clip.joint_count + arm;
        clip.rotations[index] = normalize(
            mul(rig.view().bind_rotations[arm], axis_angle(0.0f, 0.0f, 1.0f, 10.0f * float(f))));
    }

    ClipDescription retargeted;
    ASSERT_TRUE(retarget_clip(clip, avatar, rig.view(), avatar, rig.view(), retargeted));
    for (std::uint32_t f = 0; f < 3; ++f)
    {
        const std::uint32_t index = f * clip.joint_count + arm;
        EXPECT_NEAR(angle_between(retargeted.rotations[index], clip.rotations[index]), 0.0f, 1e-3f)
            << "frame " << f;
    }
}

TEST(Unit_AnimationRetarget,MirroringMovesTheGestureToTheOtherSideAndBackAgain)
{
    // A lift of the left arm becomes a lift of the right. Applying the mirror twice must return
    // the original, which is the assertion that catches a reflection applied to the wrong
    // component: a wrong axis still produces *a* mirrored-looking pose but does not undo itself.
    const Rig rig(humanoid_joints(1.0f, 0.30f, 0.0f));
    ASSERT_TRUE(rig.built());
    const Avatar avatar = build_avatar_heuristic(rig.view());

    const std::uint32_t left = rig.joint("LeftUpperArm");
    const std::uint32_t right = rig.joint("RightUpperArm");

    ClipDescription clip = bind_clip(rig, 1);
    const Quaternionf lift = axis_angle(0.0f, 0.0f, 1.0f, 40.0f);
    clip.rotations[left] = normalize(mul(rig.view().bind_rotations[left], lift));

    ClipDescription mirrored;
    ASSERT_TRUE(mirror_clip(clip, avatar, rig.view(), mirrored));

    // The right arm now carries the reflected delta; the left is back at its bind pose because
    // the right arm it took its delta from was not animated.
    const Quaternionf right_delta =
        normalize(mul(conjugate(rig.view().bind_rotations[right]), mirrored.rotations[right]));
    EXPECT_NEAR(right_delta.x, lift.x, 1e-4f);
    EXPECT_NEAR(right_delta.y, -lift.y, 1e-4f);
    EXPECT_NEAR(right_delta.z, -lift.z, 1e-4f);
    EXPECT_NEAR(std::fabs(right_delta.w), std::fabs(lift.w), 1e-4f);
    EXPECT_NEAR(angle_between(mirrored.rotations[left], rig.view().bind_rotations[left]), 0.0f,
                1e-4f);

    ClipDescription twice;
    ASSERT_TRUE(mirror_clip(mirrored, avatar, rig.view(), twice));
    EXPECT_NEAR(angle_between(twice.rotations[left], clip.rotations[left]), 0.0f, 1e-3f);
}

TEST(Unit_AnimationRetarget,MirroringNegatesLateralTranslationAndLeavesTheOtherTwoAlone)
{
    // The sagittal plane is x = 0, so mirroring negates x and must not touch y or z. Getting
    // this wrong makes a mirrored walk drift vertically, which reads as a physics bug.
    const Rig rig(humanoid_joints(1.0f, 0.30f, 0.0f));
    ASSERT_TRUE(rig.built());
    const Avatar avatar = build_avatar_heuristic(rig.view());
    const std::uint32_t hips = rig.joint("Hips");

    ClipDescription clip = bind_clip(rig, 1);
    clip.translations[hips] =
        rig.view().bind_translations[hips] + Vector3f{0.3f, 0.1f, -0.2f};

    ClipDescription mirrored;
    ASSERT_TRUE(mirror_clip(clip, avatar, rig.view(), mirrored));

    const Vector3f offset = mirrored.translations[hips] - rig.view().bind_translations[hips];
    EXPECT_NEAR(offset.x, -0.3f, 1e-4f);
    EXPECT_NEAR(offset.y, 0.1f, 1e-4f);
    EXPECT_NEAR(offset.z, -0.2f, 1e-4f);
}

TEST(Unit_AnimationRetarget,BothTransfersRefuseAClipThatDoesNotMatchItsSkeleton)
{
    // A clip whose joint count disagrees with the rig cannot be indexed safely, and the failure
    // mode of not checking is reading past the end of the track arrays.
    const Rig rig(humanoid_joints(1.0f, 0.30f, 0.0f));
    ASSERT_TRUE(rig.built());
    const Avatar avatar = build_avatar_heuristic(rig.view());

    ClipDescription wrong_joint_count = bind_clip(rig, 1);
    wrong_joint_count.joint_count += 1;
    ClipDescription out;
    EXPECT_FALSE(retarget_clip(wrong_joint_count, avatar, rig.view(), avatar, rig.view(), out));
    EXPECT_FALSE(mirror_clip(wrong_joint_count, avatar, rig.view(), out));

    ClipDescription truncated = bind_clip(rig, 2);
    truncated.rotations.pop_back();
    EXPECT_FALSE(retarget_clip(truncated, avatar, rig.view(), avatar, rig.view(), out));
    EXPECT_FALSE(mirror_clip(truncated, avatar, rig.view(), out));

    ClipDescription no_frames = bind_clip(rig, 0);
    EXPECT_FALSE(retarget_clip(no_frames, avatar, rig.view(), avatar, rig.view(), out));
    EXPECT_FALSE(mirror_clip(no_frames, avatar, rig.view(), out));
}

TEST(Unit_AnimationRetarget,SkeletonIndependentTracksSurviveBothTransfers)
{
    // Morph and generic tracks are named, not indexed by joint, so retargeting has nothing to
    // do to them — and dropping them is the quiet failure, because a face stops animating while
    // the body still works.
    const Rig source(humanoid_joints(1.0f, 0.30f, 0.0f));
    const Rig target(humanoid_joints(1.5f, 0.45f, 20.0f));
    ASSERT_TRUE(source.built());
    ASSERT_TRUE(target.built());

    ClipDescription clip = bind_clip(source, 2);
    clip.morph_names = {"jawOpen"};
    clip.morph_weights = {0.25f, 0.75f};
    clip.generic_names = {"handIkWeight"};
    clip.generic_values = {1.0f, 0.5f};

    const Avatar source_avatar = build_avatar_heuristic(source.view());
    const Avatar target_avatar = build_avatar_heuristic(target.view());

    ClipDescription retargeted;
    ASSERT_TRUE(retarget_clip(clip, source_avatar, source.view(), target_avatar, target.view(),
                              retargeted));
    EXPECT_EQ(retargeted.morph_names, clip.morph_names);
    EXPECT_EQ(retargeted.morph_weights, clip.morph_weights);
    EXPECT_EQ(retargeted.generic_names, clip.generic_names);
    EXPECT_EQ(retargeted.generic_values, clip.generic_values);

    ClipDescription mirrored;
    ASSERT_TRUE(mirror_clip(clip, source_avatar, source.view(), mirrored));
    EXPECT_EQ(mirrored.morph_weights, clip.morph_weights);
    EXPECT_EQ(mirrored.generic_values, clip.generic_values);
}

TEST(Unit_AnimationRetarget,TheRuntimePathAgreesWithTheBakedOne)
{
    // `RuntimeRetargeter` exists for a target rig not known at cook time, and its whole claim is
    // that it shares `retarget_pose_frame` with `retarget_clip` rather than reimplementing the
    // transfer. That claim is only worth anything if the two agree, so this compares them
    // directly: bake a retargeted clip, play it against the target rig, and play the *source*
    // clip through the runtime retargeter. Same pose, both ways.
    const Rig source(humanoid_joints(1.0f, 0.30f, 0.0f));
    const Rig target(humanoid_joints(1.6f, 0.45f, 25.0f));
    ASSERT_TRUE(source.built());
    ASSERT_TRUE(target.built());

    const Avatar source_avatar = build_avatar_heuristic(source.view());
    const Avatar target_avatar = build_avatar_heuristic(target.view());
    const std::uint32_t source_arm = source.joint("LeftUpperArm");
    const std::uint32_t source_hips = source.joint("Hips");

    ClipDescription description = bind_clip(source, 2);
    for (std::uint32_t f = 0; f < 2; ++f)
    {
        const std::uint32_t arm = f * description.joint_count + source_arm;
        const std::uint32_t hips = f * description.joint_count + source_hips;
        description.rotations[arm] = normalize(
            mul(source.view().bind_rotations[source_arm], axis_angle(0, 0, 1, 30.0f * float(f))));
        description.translations[hips] =
            source.view().bind_translations[source_hips] + Vector3f{0.2f * float(f), 0.0f, 0.0f};
    }

    ClipDescription baked;
    ASSERT_TRUE(retarget_clip(description, source_avatar, source.view(), target_avatar,
                              target.view(), baked));

    std::vector<std::byte> source_blob;
    std::vector<std::byte> baked_blob;
    ASSERT_TRUE(build_clip_blob(description, source_blob));
    ASSERT_TRUE(build_clip_blob(baked, baked_blob));
    const ClipView source_clip = load_clip_blob(source_blob.data(), source_blob.size());
    const ClipView baked_clip = load_clip_blob(baked_blob.data(), baked_blob.size());
    ASSERT_TRUE(source_clip.valid());
    ASSERT_TRUE(baked_clip.valid());

    RuntimeRetargeter runtime;
    runtime.bind(source.view(), source_avatar, target.view(), target_avatar);
    ClipEvaluator baked_player;

    // The last frame's time, so the comparison is made where the animation is furthest from
    // bind: at t = 0 a bug in the transfer is hidden by both answers being the bind pose.
    const float time = 1.0f / description.sample_rate;
    runtime.evaluate(source_clip, time, false);
    baked_player.evaluate(target.view(), baked_clip, time, false);

    ASSERT_EQ(runtime.local_rotations().size(), std::size_t(target.view().joint_count));
    for (std::uint32_t j = 0; j < target.view().joint_count; ++j)
    {
        EXPECT_NEAR(angle_between(runtime.local_rotations()[j], baked_player.local_rotations()[j]),
                    0.0f, 1e-2f)
            << "joint " << j;
        const Vector3f difference =
            runtime.local_translations()[j] - baked_player.local_translations()[j];
        EXPECT_NEAR(length(difference), 0.0f, 1e-4f) << "joint " << j;
    }

    // And the palette it produces is shaped for the target rig, which is what a caller uploads.
    EXPECT_EQ(runtime.palette().size(), std::size_t(target.view().joint_count));
}

TEST(Unit_AnimationRetarget,AnUnmappedTargetBoneIsPosedByItsParentAndNotLeftUninitialized)
{
    // A target rig missing a bone the source has (no toes, say) is the normal case, not an
    // error. Every target joint must still come out of the runtime path holding a real pose,
    // because a skinning palette built from an uninitialized rotation is a mesh that vanishes.
    const Rig source(humanoid_joints(1.0f, 0.30f, 0.0f));
    const Rig target({{"Hips", -1, Vector3f{0.0f, 1.2f, 0.0f}, IDENTITY},
                      {"Spine", 0, Vector3f{0.0f, 0.25f, 0.0f}, IDENTITY},
                      {"prop_socket", 1, Vector3f{0.1f, 0.0f, 0.0f}, axis_angle(1, 0, 0, 15.0f)}});
    ASSERT_TRUE(source.built());
    ASSERT_TRUE(target.built());

    const Avatar source_avatar = build_avatar_heuristic(source.view());
    const Avatar target_avatar = build_avatar_heuristic(target.view());
    ASSERT_FALSE(target_avatar.has(HumanBone::LeftUpperArm));

    ClipDescription description = bind_clip(source, 1);
    description.rotations[source.joint("LeftUpperArm")] = axis_angle(0, 0, 1, 45.0f);
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(description, blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());

    RuntimeRetargeter runtime;
    runtime.bind(source.view(), source_avatar, target.view(), target_avatar);
    runtime.evaluate(clip, 0.0f, false);

    const std::uint32_t socket = target.joint("prop_socket");
    EXPECT_NEAR(angle_between(runtime.local_rotations()[socket],
                             target.view().bind_rotations[socket]),
                0.0f, 1e-4f);
    for (std::uint32_t j = 0; j < target.view().joint_count; ++j)
    {
        // A unit quaternion is the cheapest proof the slot was written rather than defaulted:
        // a value-initialized Quaternionf has length zero, which composes to a zero matrix.
        const Quaternionf& rotation = runtime.local_rotations()[j];
        const float magnitude = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
                                         rotation.z * rotation.z + rotation.w * rotation.w);
        EXPECT_NEAR(magnitude, 1.0f, 1e-4f) << "joint " << j;
    }
}
