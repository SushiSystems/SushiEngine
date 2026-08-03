/**************************************************************************/
/* test_animation_authoring_tail.cpp                                      */
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

// Design §12.4's four remaining pieces, each deliberately scoped smaller than the AAA feature it
// is named after, and each therefore worth pinning at exactly the scope it claims: the
// motion-matching database and the reference crossfade driver over it, dual-quaternion skinning's
// blend math, the ARKit-52 facial blendshape mapping, and the sequencer timeline's evaluation core.
//
// Dual-quaternion skinning gets the most attention here for a specific reason: it is the one item
// in this file whose real verification is visual, on a GPU, on a bent joint — and that has never
// been done. What *can* be verified without a display is the algebra, so the algebra is verified
// against closed forms and against the linear-blend baseline it exists to beat. That is not a
// substitute for looking at it; it is the half that does not need to be guessed at.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/dual_quaternion_skinning.hpp>
#include <SushiEngine/animation/facial_blendshapes.hpp>
#include <SushiEngine/animation/motion_match_sampler.hpp>
#include <SushiEngine/animation/motion_matching.hpp>
#include <SushiEngine/animation/sequence_timeline.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    const Quaternionf IDENTITY{0.0f, 0.0f, 0.0f, 1.0f};

    Quaternionf axis_angle(float x, float y, float z, float degrees)
    {
        const float radians = degrees * 3.14159265358979323846f / 180.0f;
        const float half = radians * 0.5f;
        const float s = std::sin(half);
        return normalize(Quaternionf{x * s, y * s, z * s, std::cos(half)});
    }

    float magnitude(const Vector3f& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    float distance(const Vector3f& a, const Vector3f& b)
    {
        return magnitude(Vector3f{a.x - b.x, a.y - b.y, a.z - b.z});
    }

    /**
     * @brief A database holding one skeleton and clips built on demand.
     *
     * The skeleton is three joints in a chain along +Y: a root whose motion the feature vector is
     * measured from, and two "feet" whose heights the proxy reads.
     */
    struct Fixture
    {
        AnimationDatabase database;
        AssetId skeleton = INVALID_ASSET;
        SkeletonView view{};

        Fixture()
        {
            SkeletonDesc description;
            description.joints.resize(3);
            const char* names[3] = {"root", "footLeft", "footRight"};
            const int parents[3] = {-1, 0, 0};
            const Vector3f offsets[3] = {
                Vector3f{0.0f, 1.0f, 0.0f}, Vector3f{0.2f, -1.0f, 0.0f}, Vector3f{-0.2f, -1.0f, 0.0f}};
            for (std::size_t i = 0; i < 3; ++i)
            {
                description.joints[i].name = names[i];
                description.joints[i].parent = parents[i];
                description.joints[i].bind_translation = offsets[i];
            }
            std::vector<std::byte> blob;
            build_skeleton_blob(description, blob);
            skeleton = database.add_skeleton(std::move(blob));
            view = database.skeleton(skeleton);
        }

        std::uint32_t joint(const char* name) const
        {
            const int index = view.find_joint(hash_name(name));
            return index >= 0 ? static_cast<std::uint32_t>(index) : 0u;
        }

        /**
         * @brief A clip translating the root at a constant speed along +Z, feet at a fixed height.
         *
         * A constant velocity is what makes the finite-difference feature exactly predictable, so
         * a wrong `dt` or a wrong sample pair is visible as a number rather than as "about right".
         */
        AssetId constant_velocity_clip(float speed_z, float foot_drop, float duration_seconds)
        {
            const float rate = 30.0f;
            const std::uint32_t frames =
                static_cast<std::uint32_t>(duration_seconds * rate) + 1u;
            ClipDesc clip;
            clip.joint_count = view.joint_count;
            clip.frame_count = frames;
            clip.sample_rate = rate;
            clip.translations.resize(static_cast<std::size_t>(frames) * clip.joint_count);
            clip.rotations.assign(static_cast<std::size_t>(frames) * clip.joint_count, IDENTITY);
            clip.scales.assign(static_cast<std::size_t>(frames) * clip.joint_count,
                               Vector3f{1.0f, 1.0f, 1.0f});
            const std::uint32_t root = joint("root");
            for (std::uint32_t f = 0; f < frames; ++f)
            {
                const float time = float(f) / rate;
                for (std::uint32_t j = 0; j < clip.joint_count; ++j)
                    clip.translations[f * clip.joint_count + j] = view.bind_translations[j];
                clip.translations[f * clip.joint_count + root] =
                    Vector3f{0.0f, 1.0f + foot_drop, speed_z * time};
            }
            std::vector<std::byte> blob;
            build_clip_blob(clip, blob);
            return database.add_clip(std::move(blob));
        }
    };
} // namespace

// ---- motion matching: the searchable database ----

TEST(Unit_AnimationAuthoringTail,TheDatabaseSamplesEvenlyAndMeasuresVelocityFromTheClip)
{
    // The database's own claim is that every candidate is measured the same way, by its own
    // finite-difference step rather than a caller's. A constant-velocity clip is what makes that
    // checkable: every candidate must report the same velocity, and it must be the authored one.
    Fixture fixture;
    const AssetId clip = fixture.constant_velocity_clip(2.0f, 0.0f, 1.0f);

    MotionDatabase motion;
    motion.add_clip(clip, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 8);
    ASSERT_EQ(motion.size(), 8u);

    for (std::size_t i = 0; i < motion.size(); ++i)
    {
        EXPECT_EQ(motion[i].clip, clip);
        EXPECT_NEAR(motion[i].feature.root_velocity.z, 2.0f, 1e-2f) << "candidate " << i;
        EXPECT_NEAR(motion[i].feature.root_velocity.x, 0.0f, 1e-3f);
        // Not named, so left at zero rather than filled with whatever the joint happened to be.
        EXPECT_FLOAT_EQ(motion[i].feature.left_foot_height, 0.0f);
        EXPECT_FLOAT_EQ(motion[i].feature.right_foot_height, 0.0f);
        if (i > 0)
            EXPECT_GT(motion[i].time_seconds, motion[i - 1].time_seconds);
    }

    // Evenly spaced across the duration, starting at zero.
    EXPECT_FLOAT_EQ(motion[0].time_seconds, 0.0f);
    EXPECT_NEAR(motion[4].time_seconds, motion[0].time_seconds + 4.0f * (1.0f / 8.0f), 1e-3f);
}

TEST(Unit_AnimationAuthoringTail,NamingAFootJointFillsItsHeightProxyAndNotNamingItDoesNot)
{
    // The proxy is model-space Y of a named joint, which is exactly why the header calls it a
    // proxy rather than contact phase. Asserting it against the composed height is what confirms
    // it reads the model transform rather than the local one — the two differ here because the
    // feet hang off a root that is a metre up.
    Fixture fixture;
    const AssetId clip = fixture.constant_velocity_clip(0.0f, 0.5f, 0.5f);

    MotionDatabase motion;
    motion.add_clip(clip, fixture.database, fixture.view, fixture.joint("root"),
                    static_cast<int>(fixture.joint("footLeft")), -1, 3);
    ASSERT_EQ(motion.size(), 3u);
    for (std::size_t i = 0; i < motion.size(); ++i)
    {
        // Root at y = 1.5, foot one unit below it in local space, so 0.5 in model space.
        EXPECT_NEAR(motion[i].feature.left_foot_height, 0.5f, 1e-3f);
        EXPECT_FLOAT_EQ(motion[i].feature.right_foot_height, 0.0f) << "unnamed, so untouched";
    }
}

TEST(Unit_AnimationAuthoringTail,TheSearchPicksTheNearestCandidateAndTheWeightsDecideWhatNearMeans)
{
    // Two clips whose velocities and foot heights disagree, so the same query resolves to
    // different candidates depending on the weights. Without this the weights could be ignored
    // entirely and every test above would still pass.
    Fixture fixture;
    const AssetId slow = fixture.constant_velocity_clip(1.0f, 0.0f, 0.5f);  // foot at 0.0
    const AssetId fast = fixture.constant_velocity_clip(5.0f, 0.6f, 0.5f);  // foot at 0.6

    MotionDatabase motion;
    const int left = static_cast<int>(fixture.joint("footLeft"));
    motion.add_clip(slow, fixture.database, fixture.view, fixture.joint("root"), left, -1, 2);
    motion.add_clip(fast, fixture.database, fixture.view, fixture.joint("root"), left, -1, 2);
    ASSERT_EQ(motion.size(), 4u);

    // Asking for the fast velocity picks a fast candidate when velocity dominates.
    MotionFeature query;
    query.root_velocity = Vector3f{0.0f, 0.0f, 5.0f};
    query.left_foot_height = 0.0f;
    std::size_t best = motion.find_best(query, 1.0f, 0.0f);
    ASSERT_NE(best, MotionDatabase::NOT_FOUND);
    EXPECT_EQ(motion[best].clip, fast);

    // Weighting the foot proxy hard enough flips the answer to the clip whose foot matches, even
    // though its velocity is four units away.
    best = motion.find_best(query, 0.01f, 1000.0f);
    ASSERT_NE(best, MotionDatabase::NOT_FOUND);
    EXPECT_EQ(motion[best].clip, slow);

    // An exact match wins outright.
    query.root_velocity = Vector3f{0.0f, 0.0f, 1.0f};
    best = motion.find_best(query, 1.0f, 0.25f);
    EXPECT_EQ(motion[best].clip, slow);
}

TEST(Unit_AnimationAuthoringTail,AnEmptyOrUnbuildableDatabaseAnswersNotFoundRatherThanIndexZero)
{
    // `NOT_FOUND` exists so a caller can tell "no candidate" from "candidate 0", and every path
    // that produces no candidates has to reach it — including the ones that look like successes.
    Fixture fixture;
    MotionDatabase motion;
    EXPECT_EQ(motion.size(), 0u);
    EXPECT_EQ(motion.find_best(MotionFeature{}), MotionDatabase::NOT_FOUND);

    // Zero samples requested: a no-op, not one candidate.
    const AssetId clip = fixture.constant_velocity_clip(1.0f, 0.0f, 0.5f);
    motion.add_clip(clip, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 0);
    EXPECT_EQ(motion.size(), 0u);

    // A clip that does not resolve: the database must not fabricate candidates for it.
    motion.add_clip(INVALID_ASSET, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 4);
    EXPECT_EQ(motion.size(), 0u);
    EXPECT_EQ(motion.find_best(MotionFeature{}), MotionDatabase::NOT_FOUND);
}

// ---- motion matching: the reference crossfade driver ----

TEST(Unit_AnimationAuthoringTail,TheSamplerSnapsOnResetAndDoesNotCrossfade)
{
    Fixture fixture;
    const AssetId clip = fixture.constant_velocity_clip(1.0f, 0.0f, 1.0f);
    MotionDatabase motion;
    motion.add_clip(clip, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 4);

    MotionMatchSampler sampler;
    sampler.bind(fixture.view);
    sampler.reset(motion, fixture.database, 2);
    EXPECT_EQ(sampler.current_candidate(), 2u);
    EXPECT_FALSE(sampler.crossfading()) << "a snap is not a blend";
    ASSERT_EQ(sampler.local_rotations().size(), std::size_t(fixture.view.joint_count));

    // Out-of-range is ignored rather than clamped: clamping would silently play the wrong pose.
    sampler.reset(motion, fixture.database, 99);
    EXPECT_EQ(sampler.current_candidate(), 2u);
}

TEST(Unit_AnimationAuthoringTail,TheSamplerResamplesOnItsIntervalRatherThanEveryFrame)
{
    // The hysteresis the header exists for. Re-searching every frame is what makes two candidates
    // that nearly tie flip back and forth, and the visible symptom is a character that jitters.
    Fixture fixture;
    const AssetId slow = fixture.constant_velocity_clip(1.0f, 0.0f, 0.5f);
    const AssetId fast = fixture.constant_velocity_clip(6.0f, 0.0f, 0.5f);
    MotionDatabase motion;
    motion.add_clip(slow, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 2);
    motion.add_clip(fast, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 2);

    MotionMatchSamplerConfig config;
    config.resample_interval_seconds = 0.1f;
    config.crossfade_seconds = 0.2f;

    MotionMatchSampler sampler;
    sampler.bind(fixture.view);

    // The first update with no clip bound searches immediately: a driver that waited for its
    // interval before ever picking anything would output the bind pose for a tenth of a second.
    MotionFeature slow_query;
    slow_query.root_velocity = Vector3f{0.0f, 0.0f, 1.0f};
    sampler.update(motion, fixture.database, slow_query, 1.0f / 60.0f, config);
    ASSERT_NE(sampler.current_candidate(), MotionDatabase::NOT_FOUND);
    const std::size_t first = sampler.current_candidate();
    EXPECT_EQ(motion[first].clip, slow);

    // A contradicting query inside the interval changes nothing.
    MotionFeature fast_query;
    fast_query.root_velocity = Vector3f{0.0f, 0.0f, 6.0f};
    sampler.update(motion, fixture.database, fast_query, 0.01f, config);
    EXPECT_EQ(sampler.current_candidate(), first) << "a search inside the interval must not run";
    EXPECT_FALSE(sampler.crossfading());

    // Past the interval, it switches and begins a crossfade.
    sampler.update(motion, fixture.database, fast_query, 0.12f, config);
    EXPECT_NE(sampler.current_candidate(), first);
    EXPECT_EQ(motion[sampler.current_candidate()].clip, fast);
    EXPECT_TRUE(sampler.crossfading());
}

TEST(Unit_AnimationAuthoringTail,ACrossfadeCompletesAfterItsDurationAndEndsOnTheIncomingPose)
{
    // A blend that never finishes leaves two clips advancing forever, which costs double and
    // reads as a pose that never quite settles.
    Fixture fixture;
    const AssetId slow = fixture.constant_velocity_clip(1.0f, 0.0f, 0.5f);
    const AssetId fast = fixture.constant_velocity_clip(6.0f, 0.0f, 0.5f);
    MotionDatabase motion;
    motion.add_clip(slow, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 2);
    motion.add_clip(fast, fixture.database, fixture.view, fixture.joint("root"), -1, -1, 2);

    MotionMatchSamplerConfig config;
    config.resample_interval_seconds = 0.05f;
    config.crossfade_seconds = 0.2f;

    MotionFeature slow_query;
    slow_query.root_velocity = Vector3f{0.0f, 0.0f, 1.0f};
    MotionFeature fast_query;
    fast_query.root_velocity = Vector3f{0.0f, 0.0f, 6.0f};

    MotionMatchSampler sampler;
    sampler.bind(fixture.view);
    sampler.update(motion, fixture.database, slow_query, 1.0f / 60.0f, config);
    sampler.update(motion, fixture.database, fast_query, 0.06f, config);
    ASSERT_TRUE(sampler.crossfading());

    // Enough steps to cover the fade, and no more: it must end because the duration elapsed, not
    // because the loop ran out.
    for (int step = 0; step < 5; ++step)
        sampler.update(motion, fixture.database, fast_query, 0.05f, config);
    EXPECT_FALSE(sampler.crossfading());

    // And the output is a real pose for every joint, not a half-written one.
    ASSERT_EQ(sampler.local_rotations().size(), std::size_t(fixture.view.joint_count));
    for (std::uint32_t j = 0; j < fixture.view.joint_count; ++j)
    {
        const Quaternionf& r = sampler.local_rotations()[j];
        EXPECT_NEAR(std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z + r.w * r.w), 1.0f, 1e-4f);
        EXPECT_NEAR(sampler.local_scales()[j].x, 1.0f, 1e-4f);
    }
}

// ---- dual-quaternion skinning ----

TEST(Unit_AnimationAuthoringTail,ADualQuaternionReproducesTheRigidTransformItWasBuiltFrom)
{
    // The construction and the application have to be inverses of the same thing, and this is the
    // only case where the right answer is available in closed form: one influence, so the blend
    // does nothing and what remains is the encode/decode pair.
    const Quaternionf rotation = axis_angle(0.0f, 1.0f, 0.0f, 37.0f);
    const Vector3f translation{1.5f, -2.0f, 0.25f};
    const DualQuaternion dq = dual_quaternion_from_rigid(rotation, translation);

    const Vector3f points[4] = {{0.0f, 0.0f, 0.0f},
                                {1.0f, 0.0f, 0.0f},
                                {0.0f, 2.0f, -1.0f},
                                {-3.0f, 0.5f, 4.0f}};
    for (const Vector3f& point : points)
    {
        const Vector3f expected = rotate(rotation, point) + translation;
        EXPECT_NEAR(distance(skin_position_dqs(dq, point), expected), 0.0f, 1e-4f);
    }

    // A single influence at any positive weight is that influence, so the blend must be the
    // identity operation here — including when the weight is not one.
    const float weights[1] = {0.37f};
    const Vector3f blended = skin_position_dqs(blend_dual_quaternions(&dq, weights, 1), points[3]);
    EXPECT_NEAR(distance(blended, rotate(rotation, points[3]) + translation), 0.0f, 1e-4f);
}

TEST(Unit_AnimationAuthoringTail,DualQuaternionSkinningKeepsVolumeWhereLinearBlendingCollapsesIt)
{
    // The candy-wrapper collapse, which is the entire reason this header exists. Two influences
    // whose rotations are 180 degrees apart, blended half and half: linear blending averages the
    // two rotations and the result shrinks toward the twist axis, while a dual-quaternion blend
    // stays on the rigid-motion manifold and preserves the distance from that axis.
    const Quaternionf rotations[2] = {IDENTITY, axis_angle(0.0f, 1.0f, 0.0f, 180.0f)};
    const Vector3f translations[2] = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};
    const float weights[2] = {0.5f, 0.5f};

    const DualQuaternion dual_quaternions[2] = {
        dual_quaternion_from_rigid(rotations[0], translations[0]),
        dual_quaternion_from_rigid(rotations[1], translations[1])};

    const Vector3f rest{1.0f, 0.0f, 0.0f}; // one unit off the Y twist axis
    const float rest_radius = magnitude(rest);

    const Vector3f dqs =
        skin_position_dqs(blend_dual_quaternions(dual_quaternions, weights, 2), rest);
    const Vector3f lbs = skin_position_lbs(rotations, translations, weights, 2, rest);

    const float dqs_radius = std::sqrt(dqs.x * dqs.x + dqs.z * dqs.z);
    const float lbs_radius = std::sqrt(lbs.x * lbs.x + lbs.z * lbs.z);

    EXPECT_NEAR(dqs_radius, rest_radius, 1e-3f) << "a rigid blend cannot change the radius";
    EXPECT_LT(lbs_radius, rest_radius * 0.5f) << "the linear baseline must visibly collapse";
}

TEST(Unit_AnimationAuthoringTail,BlendingIsInvariantToAnInfluencesQuaternionSign)
{
    // `q` and `-q` are the same rotation, so a skin that renders differently depending on which
    // one the cook happened to store is broken — and the way it breaks is that the two influences
    // partially cancel, which looks like a joint that collapses at certain angles only. The
    // dual-quaternion blend needs an explicit hemisphere correction to get this right, since it
    // sums both parts; the linear baseline gets it for free, because it sums transformed
    // positions and never a quaternion. Both are asserted, because "for free" is a property of
    // the current implementation and this is the test that would notice it changing.
    const Quaternionf a = axis_angle(0.0f, 0.0f, 1.0f, 20.0f);
    const Quaternionf b = axis_angle(0.0f, 0.0f, 1.0f, 150.0f);
    const Quaternionf b_negated{-b.x, -b.y, -b.z, -b.w};
    const Vector3f translations[2] = {{0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f}};
    const float weights[2] = {0.6f, 0.4f};
    const Vector3f rest{0.0f, 1.0f, 0.0f};

    const Quaternionf plain[2] = {a, b};
    const Quaternionf flipped[2] = {a, b_negated};

    const DualQuaternion dq_plain[2] = {dual_quaternion_from_rigid(plain[0], translations[0]),
                                        dual_quaternion_from_rigid(plain[1], translations[1])};
    const DualQuaternion dq_flipped[2] = {dual_quaternion_from_rigid(flipped[0], translations[0]),
                                          dual_quaternion_from_rigid(flipped[1], translations[1])};

    const Vector3f dqs_plain = skin_position_dqs(blend_dual_quaternions(dq_plain, weights, 2), rest);
    const Vector3f dqs_flipped =
        skin_position_dqs(blend_dual_quaternions(dq_flipped, weights, 2), rest);
    EXPECT_NEAR(distance(dqs_plain, dqs_flipped), 0.0f, 1e-4f);

    const Vector3f lbs_plain = skin_position_lbs(plain, translations, weights, 2, rest);
    const Vector3f lbs_flipped = skin_position_lbs(flipped, translations, weights, 2, rest);
    EXPECT_NEAR(distance(lbs_plain, lbs_flipped), 0.0f, 1e-4f);
}

TEST(Unit_AnimationAuthoringTail,TheLinearBaselineAveragesTranslationsRegardlessOfQuaternionSign)
{
    // A regression test for a defect the baseline used to have and now cannot. It blended the
    // rotation as a weighted quaternion sum, which needs a hemisphere sign — and it carried that
    // sign into the *translation* sum too, so an influence whose stored quaternion sat in the
    // opposite hemisphere had its translation subtracted while the divisor stayed positive, and
    // the vertex landed somewhere no influence had asked for. A 200-degree rotation reaches it:
    // its quaternion has a negative dot with the identity.
    //
    // The baseline now transforms by each influence separately and averages the results, so no
    // quaternion is ever summed and the sign cannot reach anything. This test pins the outcome
    // rather than the mechanism, so it keeps guarding the property whichever way the function is
    // written.
    const Quaternionf rotations[2] = {IDENTITY, axis_angle(0.0f, 1.0f, 0.0f, 200.0f)};
    ASSERT_LT(rotations[1].w, 0.0f) << "the setup must actually cross the hemisphere";

    const Vector3f translations[2] = {{0.0f, 0.0f, 0.0f}, {4.0f, 0.0f, 0.0f}};
    const float weights[2] = {0.5f, 0.5f};
    const Vector3f origin{0.0f, 0.0f, 0.0f};

    // With the origin as the rest position the rotation contributes nothing, so what comes out
    // *is* the blended translation: 0.5 * (0,0,0) + 0.5 * (4,0,0) = (2,0,0).
    const Vector3f skinned = skin_position_lbs(rotations, translations, weights, 2, origin);
    EXPECT_NEAR(skinned.x, 2.0f, 1e-4f) << "a negated translation would land at -2";
    EXPECT_NEAR(skinned.y, 0.0f, 1e-4f);
    EXPECT_NEAR(skinned.z, 0.0f, 1e-4f);

    // And the same blend agrees with the dual-quaternion path on the translation, which it cannot
    // if one of them has the sign wrong.
    const DualQuaternion dual_quaternions[2] = {
        dual_quaternion_from_rigid(rotations[0], translations[0]),
        dual_quaternion_from_rigid(rotations[1], translations[1])};
    const Vector3f dqs =
        skin_position_dqs(blend_dual_quaternions(dual_quaternions, weights, 2), origin);
    EXPECT_GT(dqs.x, 0.0f) << "both paths must move the vertex the same way along x";
}

TEST(Unit_AnimationAuthoringTail,ZeroAndNegativeWeightsDegradeToIdentityRatherThanNotANumber)
{
    // An unweighted vertex is a real thing in a real mesh — a stray vertex the exporter left with
    // four zero weights — and the answer has to be a position, not a not-a-number that poisons
    // the whole draw.
    const DualQuaternion dual_quaternions[2] = {
        dual_quaternion_from_rigid(axis_angle(1.0f, 0.0f, 0.0f, 40.0f), Vector3f{1.0f, 0.0f, 0.0f}),
        dual_quaternion_from_rigid(IDENTITY, Vector3f{0.0f, 1.0f, 0.0f})};
    const float zero[2] = {0.0f, 0.0f};
    const Vector3f rest{0.5f, -0.25f, 2.0f};

    const DualQuaternion blended = blend_dual_quaternions(dual_quaternions, zero, 2);
    EXPECT_FLOAT_EQ(blended.real.w, 1.0f);
    EXPECT_FLOAT_EQ(blended.dual.w, 0.0f);
    const Vector3f identity_skinned = skin_position_dqs(blended, rest);
    EXPECT_NEAR(distance(identity_skinned, rest), 0.0f, 1e-5f);

    const Quaternionf rotations[2] = {axis_angle(1.0f, 0.0f, 0.0f, 40.0f), IDENTITY};
    const Vector3f translations[2] = {{1.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f}};
    EXPECT_NEAR(distance(skin_position_lbs(rotations, translations, zero, 2, rest), rest), 0.0f,
                1e-5f);

    // A negative weight is skipped rather than subtracted, so a single negative influence is the
    // same as no influence at all.
    const float negative[2] = {-1.0f, 0.0f};
    EXPECT_NEAR(distance(skin_position_lbs(rotations, translations, negative, 2, rest), rest), 0.0f,
                1e-5f);
    EXPECT_FLOAT_EQ(blend_dual_quaternions(dual_quaternions, negative, 2).real.w, 1.0f);
}

// ---- facial blendshapes ----

TEST(Unit_AnimationAuthoringTail,EveryARKitShapeHasADistinctNonEmptyNameAndCountHasNone)
{
    // A hand-written table of 52 strings is where a duplicated or omitted line hides, and the
    // consequence is two canonical shapes resolving to one mesh target — a face where blinking
    // also opens the jaw.
    std::vector<std::string> names;
    for (std::uint32_t i = 0; i < ARKIT_BLENDSHAPE_COUNT; ++i)
    {
        const char* name = arkit_blendshape_name(static_cast<ARKitBlendshape>(i));
        ASSERT_NE(name, nullptr);
        EXPECT_NE(std::string(name), std::string()) << "shape " << i << " has no name";
        names.push_back(name);
    }
    EXPECT_EQ(ARKIT_BLENDSHAPE_COUNT, 52u);

    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            EXPECT_NE(names[i], names[j]) << "shapes " << i << " and " << j << " share a name";

    // Apple's own spelling, which is the whole point of the table: a rig exported for ARKit uses
    // these exact strings, so a "correct-looking" rename breaks every such rig silently.
    EXPECT_EQ(names[static_cast<std::uint32_t>(ARKitBlendshape::JawOpen)], "jawOpen");
    EXPECT_EQ(names[static_cast<std::uint32_t>(ARKitBlendshape::EyeBlinkLeft)], "eyeBlinkLeft");
    EXPECT_EQ(names[static_cast<std::uint32_t>(ARKitBlendshape::TongueOut)], "tongueOut");

    EXPECT_EQ(std::string(arkit_blendshape_name(ARKitBlendshape::Count)), std::string());
}

TEST(Unit_AnimationAuthoringTail,TheMapResolvesAMeshsOwnTargetOrderAndReportsWhatIsMissing)
{
    // A mesh's target order is its own business — the map's job is to bridge it to the canonical
    // names — so the fixture deliberately lists targets in an order unrelated to the enum, and
    // includes one target ARKit has no name for.
    const NameHash targets[4] = {hash_name("tongueOut"), hash_name("jawOpen"),
                                 hash_name("customCheekScar"), hash_name("eyeBlinkLeft")};
    const FacialBlendshapeMap map = build_facial_blendshape_map(targets, 4);

    EXPECT_EQ(map.target_index(ARKitBlendshape::TongueOut), 0);
    EXPECT_EQ(map.target_index(ARKitBlendshape::JawOpen), 1);
    EXPECT_EQ(map.target_index(ARKitBlendshape::EyeBlinkLeft), 3);
    EXPECT_TRUE(map.has(ARKitBlendshape::JawOpen));
    EXPECT_FALSE(map.has(ARKitBlendshape::EyeBlinkRight));
    EXPECT_EQ(map.target_index(ARKitBlendshape::EyeBlinkRight), -1);
    EXPECT_EQ(map.mapped_count(), 3u);

    // The missing list is what a caller logs at load, and its size is the complement of what was
    // mapped — so a shape can be neither mapped nor missing only if the map is inconsistent.
    std::vector<ARKitBlendshape> missing;
    map.list_missing(missing);
    EXPECT_EQ(missing.size(), std::size_t(ARKIT_BLENDSHAPE_COUNT - 3));
    bool jaw_listed = false;
    bool right_blink_listed = false;
    for (const ARKitBlendshape shape : missing)
    {
        jaw_listed = jaw_listed || shape == ARKitBlendshape::JawOpen;
        right_blink_listed = right_blink_listed || shape == ARKitBlendshape::EyeBlinkRight;
    }
    EXPECT_FALSE(jaw_listed);
    EXPECT_TRUE(right_blink_listed);

    // Appended, not cleared: a caller collecting across several meshes must not lose the earlier
    // ones, and the doc says so explicitly.
    map.list_missing(missing);
    EXPECT_EQ(missing.size(), std::size_t(2 * (ARKIT_BLENDSHAPE_COUNT - 3)));

    // An empty mesh maps nothing rather than mapping everything to target zero.
    const FacialBlendshapeMap none = build_facial_blendshape_map(nullptr, 0);
    EXPECT_EQ(none.mapped_count(), 0u);
}

TEST(Unit_AnimationAuthoringTail,SettingAShapeWritesItsOwnSlotAndSkipsWhatTheMeshLacks)
{
    const NameHash targets[3] = {hash_name("eyeBlinkLeft"), hash_name("jawOpen"),
                                 hash_name("mouthSmileLeft")};
    const FacialBlendshapeMap map = build_facial_blendshape_map(targets, 3);

    MorphState state;
    state.count = 3;
    set_facial_blendshape(map, ARKitBlendshape::JawOpen, 0.75f, state);
    EXPECT_FLOAT_EQ(state.weights[1], 0.75f);
    EXPECT_FLOAT_EQ(state.weights[0], 0.0f) << "only the addressed slot is written";
    EXPECT_FLOAT_EQ(state.weights[2], 0.0f);

    // A shape the mesh has no target for is a no-op, which is what lets a facial driver push all
    // 52 shapes at a rig that only implements a few.
    set_facial_blendshape(map, ARKitBlendshape::TongueOut, 1.0f, state);
    for (std::uint32_t i = 0; i < 3; ++i)
        EXPECT_LE(state.weights[i], 0.75f);

    // ARKit allows slight overshoot, so the weight is not clamped here — the shader's business,
    // not the mapping's.
    set_facial_blendshape(map, ARKitBlendshape::EyeBlinkLeft, 1.2f, state);
    EXPECT_FLOAT_EQ(state.weights[0], 1.2f);

    // A state smaller than the map's targets must not be written past its own count: a mesh
    // swapped for a lower-detail one mid-session is exactly how the two disagree.
    MorphState small;
    small.count = 1;
    set_facial_blendshape(map, ARKitBlendshape::MouthSmileLeft, 1.0f, small);
    EXPECT_FLOAT_EQ(small.weights[2], 0.0f) << "index 2 is past a count of 1";
    EXPECT_FLOAT_EQ(small.weights[0], 0.0f);
}

// ---- sequencer timeline ----

TEST(Unit_AnimationAuthoringTail,AFloatTrackInterpolatesAndClampsAndSortsWhenAsked)
{
    SequenceFloatTrack track;
    track.parameter_index = 3;
    track.keys = {{0.0f, 10.0f}, {2.0f, 30.0f}};

    EXPECT_FLOAT_EQ(track.sample(-1.0f), 10.0f);
    EXPECT_FLOAT_EQ(track.sample(0.0f), 10.0f);
    EXPECT_FLOAT_EQ(track.sample(1.0f), 20.0f);
    EXPECT_FLOAT_EQ(track.sample(2.0f), 30.0f);
    EXPECT_FLOAT_EQ(track.sample(99.0f), 30.0f);

    const SequenceFloatTrack empty;
    EXPECT_FLOAT_EQ(empty.sample(1.0f), 0.0f);

    // The header used to claim `evaluate` sorted a local copy of the keys; it does not, and doing
    // it there would put a copy and a sort on the per-frame path. `sort_keys` is where that cost
    // belongs, and this is the case that proves it is needed: bulk-loaded keys out of order read
    // the wrong segment until it is called.
    SequenceFloatTrack unsorted;
    unsorted.keys = {{2.0f, 30.0f}, {0.0f, 10.0f}, {1.0f, 100.0f}};
    unsorted.sort_keys();
    EXPECT_FLOAT_EQ(unsorted.keys[0].time, 0.0f);
    EXPECT_FLOAT_EQ(unsorted.keys[2].time, 2.0f);
    EXPECT_FLOAT_EQ(unsorted.sample(0.5f), 55.0f);
    EXPECT_FLOAT_EQ(unsorted.sample(1.5f), 65.0f);
}

TEST(Unit_AnimationAuthoringTail,EvaluateWritesEachTrackToItsOwnParameterSlot)
{
    SequenceTimeline timeline;
    SequenceFloatTrack fov;
    fov.parameter_index = 0;
    fov.keys = {{0.0f, 60.0f}, {1.0f, 30.0f}};
    SequenceFloatTrack blend;
    blend.parameter_index = 5;
    blend.keys = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    timeline.float_tracks = {fov, blend};
    timeline.sort_tracks();

    AnimatorParameterBlock parameters{};
    timeline.evaluate(0.5f, parameters);
    EXPECT_FLOAT_EQ(parameters.values[0].as_float, 45.0f);
    EXPECT_FLOAT_EQ(parameters.values[5].as_float, 0.5f);
    // A slot no track names is left alone, so a timeline can drive part of a parameter block
    // while something else drives the rest.
    EXPECT_FLOAT_EQ(parameters.values[1].as_float, 0.0f);

    // Pure in time: scrubbing backward gives the same answer as arriving forward, which is what
    // makes an editor scrub safe.
    AnimatorParameterBlock forward{};
    AnimatorParameterBlock backward{};
    timeline.evaluate(0.25f, forward);
    timeline.evaluate(0.9f, backward);
    timeline.evaluate(0.25f, backward);
    EXPECT_FLOAT_EQ(forward.values[0].as_float, backward.values[0].as_float);
}

TEST(Unit_AnimationAuthoringTail,AdvanceFiresTheHalfOpenIntervalInTimeOrderAndAppends)
{
    // The interval is `(previous, current]`, which is the only choice that fires each event
    // exactly once across contiguous steps: closing both ends double-fires on a boundary and
    // opening both ends drops it.
    SequenceTimeline timeline;
    timeline.events = {{1.0f, 101u}, {0.5f, 100u}, {2.0f, 102u}};

    std::vector<std::uint32_t> fired;
    timeline.advance(0.0f, 0.5f, fired);
    ASSERT_EQ(fired.size(), 1u);
    EXPECT_EQ(fired[0], 100u) << "an event exactly at `current` fires";

    // The same event does not fire again from the next step, whose interval opens where this one
    // closed.
    fired.clear();
    timeline.advance(0.5f, 0.9f, fired);
    EXPECT_TRUE(fired.empty()) << "an event exactly at `previous` must not fire twice";

    // A large step crosses several, and they arrive in time order rather than authoring order —
    // the events above are deliberately authored out of order to make that observable.
    fired.clear();
    timeline.advance(0.9f, 5.0f, fired);
    ASSERT_EQ(fired.size(), 2u);
    EXPECT_EQ(fired[0], 101u);
    EXPECT_EQ(fired[1], 102u);

    // Appended, not cleared: a caller accumulating from several timelines in one frame keeps them
    // all, and the doc commits to this.
    timeline.advance(0.0f, 0.6f, fired);
    ASSERT_EQ(fired.size(), 3u);
    EXPECT_EQ(fired[2], 100u);
}

TEST(Unit_AnimationAuthoringTail,ScrubbingBackwardOrStandingStillFiresNothing)
{
    // Forward-only is the documented contract, and it is the right one: an editor dragging the
    // playhead left would otherwise re-fire every cue it passed, starting sounds in reverse.
    SequenceTimeline timeline;
    timeline.events = {{0.5f, 100u}, {1.0f, 101u}};

    std::vector<std::uint32_t> fired;
    timeline.advance(2.0f, 0.0f, fired);
    EXPECT_TRUE(fired.empty());
    timeline.advance(1.0f, 1.0f, fired);
    EXPECT_TRUE(fired.empty()) << "no time passing is not a crossing";

    // A timeline with no events at all is not a special case.
    const SequenceTimeline bare;
    bare.advance(0.0f, 10.0f, fired);
    EXPECT_TRUE(fired.empty());
}
