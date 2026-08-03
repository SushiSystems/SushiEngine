/**************************************************************************/
/* test_animation_keyframe.cpp                                            */
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

// The authoring side of a clip: the sparse curves a dope sheet edits, the recorder that turns a
// posed rig into keys, the bake that resamples all of it into the dense `ClipDescription` the cook
// consumes, and the generic float tracks a baked clip dispatches to arbitrary consumers.
//
// The bake is the load-bearing part, because it is the only place the editable representation and
// the runtime one have to agree. Every bake case here therefore checks the baked frame against
// the *curve* evaluated at that frame's time, rather than against a hand-written constant: a
// constant would pass for a bake that resampled at the wrong times, as long as it was wrong
// consistently.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/generic_track.hpp>
#include <SushiEngine/animation/keyframe.hpp>

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

    /** @brief A curve of the given keys, in the given mode. */
    ScalarCurve curve_of(InterpolationMode mode,
                         const std::vector<std::pair<float, float>>& keys)
    {
        ScalarCurve curve;
        curve.mode = mode;
        for (const auto& key : keys)
            curve.insert(key.first, key.second);
        return curve;
    }

    /** @brief Records every dispatched property/value pair, in the order it arrived. */
    class RecordingSink : public IFloatSink
    {
        public:
            void set_value(NameHash property, float value) override
            {
                properties.push_back(property);
                values.push_back(value);
            }

            std::vector<NameHash> properties;
            std::vector<float> values;
    };
} // namespace

TEST(Unit_AnimationKeyframe,InsertKeepsKeysSortedAndReplacesRatherThanDuplicating)
{
    // The curve editor inserts in whatever order the user clicks, and every evaluation below
    // walks the keys assuming they ascend. An out-of-order insert makes `evaluate` pick the
    // wrong segment, which reads as a curve that jumps.
    ScalarCurve curve;
    curve.insert(1.0f, 10.0f);
    curve.insert(0.0f, 0.0f);
    curve.insert(0.5f, 5.0f);
    curve.insert(2.0f, 20.0f);

    ASSERT_EQ(curve.keys.size(), 4u);
    for (std::size_t i = 1; i < curve.keys.size(); ++i)
        EXPECT_LT(curve.keys[i - 1].time, curve.keys[i].time);
    EXPECT_FLOAT_EQ(curve.duration(), 2.0f);

    // Re-keying a time that already has a key overwrites it: keying the same frame twice is what
    // a user does every time they adjust a pose, and two keys at one time would make the segment
    // between them zero-length.
    const std::size_t index = curve.insert(0.5f, 55.0f);
    EXPECT_EQ(curve.keys.size(), 4u);
    EXPECT_FLOAT_EQ(curve.keys[index].value, 55.0f);

    curve.remove_at(0.5f);
    EXPECT_EQ(curve.keys.size(), 3u);
    // Removing a time with no key is a no-op rather than an error: a delete on empty space.
    curve.remove_at(0.75f);
    EXPECT_EQ(curve.keys.size(), 3u);
}

TEST(Unit_AnimationKeyframe,EveryInterpolationModeDoesWhatItsNameSays)
{
    const std::vector<std::pair<float, float>> keys = {{0.0f, 0.0f}, {1.0f, 10.0f}};

    const ScalarCurve constant = curve_of(InterpolationMode::Constant, keys);
    EXPECT_FLOAT_EQ(constant.evaluate(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(constant.evaluate(0.5f), 0.0f);
    // A step holds the earlier key right up to the next one, then takes the next key's value.
    EXPECT_FLOAT_EQ(constant.evaluate(0.999f), 0.0f);
    EXPECT_FLOAT_EQ(constant.evaluate(1.0f), 10.0f);

    const ScalarCurve linear = curve_of(InterpolationMode::Linear, keys);
    EXPECT_FLOAT_EQ(linear.evaluate(0.25f), 2.5f);
    EXPECT_FLOAT_EQ(linear.evaluate(0.5f), 5.0f);

    // Cubic with zero tangents is ease-in-ease-out, so it lags the line early and leads it late
    // while still passing through both keys. Asserting the midpoint alone would not distinguish
    // it from linear, because Hermite's midpoint with equal end tangents is the mean.
    ScalarCurve cubic = curve_of(InterpolationMode::Cubic, keys);
    EXPECT_FLOAT_EQ(cubic.evaluate(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(cubic.evaluate(1.0f), 10.0f);
    EXPECT_LT(cubic.evaluate(0.25f), 2.5f);
    EXPECT_GT(cubic.evaluate(0.75f), 7.5f);
    EXPECT_NEAR(cubic.evaluate(0.5f), 5.0f, 1e-5f);
}

TEST(Unit_AnimationKeyframe,ACubicCurveInterpolatesItsKeysRatherThanApproachingThem)
{
    // The Hermite basis is only correct if the tangents are scaled by the segment span, because
    // a tangent is authored in value-per-second and the basis wants value-per-unit-parameter.
    // Getting that wrong still produces a smooth curve — it just misses its own keys, by more the
    // shorter the segment. Two segments of very different lengths is what exposes it.
    ScalarCurve curve;
    curve.mode = InterpolationMode::Cubic;
    curve.insert(0.0f, 0.0f);
    curve.insert(0.1f, 4.0f);
    curve.insert(3.0f, -2.0f);
    curve.auto_tangents();

    for (const ScalarKey& key : curve.keys)
        EXPECT_NEAR(curve.evaluate(key.time), key.value, 1e-4f) << "at t = " << key.time;
}

TEST(Unit_AnimationKeyframe,AutoTangentsOnAStraightLineReproduceTheLineExactly)
{
    // The strongest available property for Catmull-Rom: if the keys are collinear in time and
    // value, the smooth curve must *be* the line. A tangent computed over the wrong neighbour
    // interval passes every "is it smooth" check and fails this one.
    ScalarCurve cubic;
    cubic.mode = InterpolationMode::Cubic;
    for (int i = 0; i <= 4; ++i)
        cubic.insert(float(i) * 0.5f, float(i) * 0.5f * 3.0f + 1.0f);
    cubic.auto_tangents();

    for (int step = 0; step <= 20; ++step)
    {
        const float time = float(step) * 0.1f;
        EXPECT_NEAR(cubic.evaluate(time), time * 3.0f + 1.0f, 1e-4f) << "at t = " << time;
    }
}

TEST(Unit_AnimationKeyframe,ACurveClampsOutsideItsRangeAndFallsBackWhenEmpty)
{
    const ScalarCurve curve = curve_of(InterpolationMode::Linear, {{1.0f, 4.0f}, {2.0f, 8.0f}});
    // Holding the end values is what makes a channel keyed over part of a clip well-defined for
    // the whole clip; extrapolating instead would send it somewhere no one authored.
    EXPECT_FLOAT_EQ(curve.evaluate(-5.0f), 4.0f);
    EXPECT_FLOAT_EQ(curve.evaluate(0.5f), 4.0f);
    EXPECT_FLOAT_EQ(curve.evaluate(100.0f), 8.0f);

    // An empty curve returns the caller's fallback, which is what lets a joint with no keys pose
    // at its rest value instead of at zero — a zero scale collapses the mesh.
    const ScalarCurve empty;
    EXPECT_TRUE(empty.empty());
    EXPECT_FLOAT_EQ(empty.duration(), 0.0f);
    EXPECT_FLOAT_EQ(empty.evaluate(0.5f, 7.5f), 7.5f);

    const ScalarCurve single = curve_of(InterpolationMode::Cubic, {{1.0f, 3.0f}});
    EXPECT_FLOAT_EQ(single.evaluate(0.0f), 3.0f);
    EXPECT_FLOAT_EQ(single.evaluate(1.0f), 3.0f);
    EXPECT_FLOAT_EQ(single.evaluate(9.0f), 3.0f);
}

TEST(Unit_AnimationKeyframe,ARotationCurveSlerpsAndStaysUnitLength)
{
    QuaternionCurve curve;
    curve.insert(0.0f, IDENTITY);
    curve.insert(1.0f, axis_angle(0.0f, 1.0f, 0.0f, 90.0f));

    // Halfway along a 90-degree turn is 45 degrees, and the result must be unit length or the
    // composed matrix scales the joint.
    const Quaternionf middle = curve.evaluate(0.5f);
    const float magnitude = std::sqrt(middle.x * middle.x + middle.y * middle.y +
                                     middle.z * middle.z + middle.w * middle.w);
    EXPECT_NEAR(magnitude, 1.0f, 1e-5f);
    const float half_angle = std::acos(std::fabs(middle.w)) * 2.0f * 180.0f / 3.14159265358979f;
    EXPECT_NEAR(half_angle, 45.0f, 1e-2f);

    EXPECT_NEAR(curve.evaluate(-1.0f).w, IDENTITY.w, 1e-6f);
    EXPECT_NEAR(curve.evaluate(5.0f).w, curve.keys.back().value.w, 1e-6f);

    const QuaternionCurve empty;
    const Quaternionf fallback = axis_angle(1.0f, 0.0f, 0.0f, 30.0f);
    EXPECT_FLOAT_EQ(empty.evaluate(0.5f, fallback).x, fallback.x);
}

TEST(Unit_AnimationKeyframe,AChannelWithNoKeysPosesAtItsRestDefault)
{
    // The claim the header makes for `JointChannels`, and the reason the defaults are there at
    // all: an author who keys only rotation must not have the joint's translation and scale
    // collapse to zero.
    JointChannels channels;
    channels.default_translation = Vector3f{1.0f, 2.0f, 3.0f};
    channels.default_rotation = axis_angle(0.0f, 0.0f, 1.0f, 20.0f);
    channels.default_scale = Vector3f{2.0f, 2.0f, 2.0f};
    channels.rotation.insert(0.0f, IDENTITY);
    channels.rotation.insert(1.0f, axis_angle(0.0f, 0.0f, 1.0f, 90.0f));

    const Vector3f translation = channels.translation_at(0.5f);
    EXPECT_FLOAT_EQ(translation.x, 1.0f);
    EXPECT_FLOAT_EQ(translation.z, 3.0f);
    EXPECT_FLOAT_EQ(channels.scale_at(0.5f).y, 2.0f);
    // The keyed channel is driven, not defaulted — otherwise the assertion above would pass for
    // a `*_at` that ignored its curves entirely.
    EXPECT_NEAR(channels.rotation_at(0.0f).w, 1.0f, 1e-6f);
    EXPECT_LT(channels.rotation_at(1.0f).w, 0.95f);

    // A single axis keyed while the others are not: mixing driven and defaulted within one
    // vector is the normal authoring case and the one an all-or-nothing fallback breaks.
    channels.translation_y.insert(0.0f, 0.0f);
    channels.translation_y.insert(1.0f, 10.0f);
    const Vector3f mixed = channels.translation_at(0.5f);
    EXPECT_FLOAT_EQ(mixed.x, 1.0f);
    EXPECT_FLOAT_EQ(mixed.y, 5.0f);
    EXPECT_FLOAT_EQ(mixed.z, 3.0f);
}

TEST(Unit_AnimationKeyframe,TheBakeReproducesTheCurveAtEveryFrameItWrites)
{
    // The bake's whole contract. Checking each frame against the curve evaluated at that frame's
    // own time is what catches an off-by-one in the time-per-frame arithmetic: a bake at
    // `f / (rate - 1)` or at `(f + 1) / rate` still produces a plausible animation.
    ClipAuthoring authoring;
    authoring.joints.resize(2);
    authoring.joints[0].joint_name = hash_name("root");
    authoring.joints[0].translation_y = curve_of(InterpolationMode::Linear,
                                                {{0.0f, 0.0f}, {0.5f, 2.0f}, {1.0f, -1.0f}});
    authoring.joints[0].rotation.insert(0.0f, IDENTITY);
    authoring.joints[0].rotation.insert(1.0f, axis_angle(0.0f, 1.0f, 0.0f, 120.0f));
    authoring.joints[1].joint_name = hash_name("child");
    authoring.joints[1].default_scale = Vector3f{3.0f, 3.0f, 3.0f};
    authoring.joints[1].scale_x = curve_of(InterpolationMode::Cubic, {{0.0f, 1.0f}, {1.0f, 4.0f}});
    authoring.joints[1].scale_x.auto_tangents();

    EXPECT_FLOAT_EQ(authoring.duration(), 1.0f);

    ClipDescription baked;
    const float rate = 20.0f;
    ASSERT_TRUE(authoring.bake(rate, baked));

    // One frame per sample plus the endpoint: a one-second clip at 20 Hz holds 21 frames, so the
    // last frame lands exactly on the last key rather than a twentieth of a second before it.
    EXPECT_EQ(baked.frame_count, 21u);
    EXPECT_EQ(baked.joint_count, 2u);
    EXPECT_FLOAT_EQ(baked.sample_rate, rate);
    ASSERT_EQ(baked.translations.size(), std::size_t(21 * 2));

    for (std::uint32_t f = 0; f < baked.frame_count; ++f)
    {
        const float time = float(f) / rate;
        const std::uint32_t root = f * baked.joint_count + 0;
        const std::uint32_t child = f * baked.joint_count + 1;
        EXPECT_NEAR(baked.translations[root].y, authoring.joints[0].translation_y.evaluate(time),
                    1e-5f)
            << "frame " << f;
        EXPECT_NEAR(baked.scales[child].x, authoring.joints[1].scale_x.evaluate(time), 1e-5f)
            << "frame " << f;
        // An unkeyed axis carries the joint's default into every baked frame.
        EXPECT_FLOAT_EQ(baked.scales[child].y, 3.0f);

        // Every baked rotation is unit length: the bake normalizes, and a slerp of near-parallel
        // keys is where the drift that makes that necessary comes from.
        const Quaternionf& rotation = baked.rotations[root];
        const float magnitude = std::sqrt(rotation.x * rotation.x + rotation.y * rotation.y +
                                         rotation.z * rotation.z + rotation.w * rotation.w);
        EXPECT_NEAR(magnitude, 1.0f, 1e-5f) << "frame " << f;
    }
}

TEST(Unit_AnimationKeyframe,TheBakeCarriesNamedCurvesFrameMajorWithTheRightStride)
{
    // Morph and generic tracks are stored frame-major with the track count as the stride, which
    // is the same layout `ClipView::sample_morph_track` indexes. A transposed bake reads track
    // zero's curve for every track at frame one, and looks like a clip whose face is stuck.
    ClipAuthoring authoring;
    authoring.joints.resize(1);
    authoring.morphs.push_back({"jawOpen", curve_of(InterpolationMode::Linear,
                                                    {{0.0f, 0.0f}, {1.0f, 1.0f}})});
    authoring.morphs.push_back({"eyeBlink", curve_of(InterpolationMode::Linear,
                                                     {{0.0f, 1.0f}, {1.0f, 0.0f}})});
    authoring.generics.push_back({"emissive", curve_of(InterpolationMode::Constant,
                                                       {{0.0f, 5.0f}, {1.0f, 9.0f}})});

    ClipDescription baked;
    ASSERT_TRUE(authoring.bake(10.0f, baked));
    ASSERT_EQ(baked.frame_count, 11u);
    ASSERT_EQ(baked.morph_names.size(), 2u);
    EXPECT_EQ(baked.morph_names[0], "jawOpen");
    EXPECT_EQ(baked.morph_names[1], "eyeBlink");
    ASSERT_EQ(baked.morph_weights.size(), std::size_t(11 * 2));
    ASSERT_EQ(baked.generic_values.size(), 11u);

    for (std::uint32_t f = 0; f < baked.frame_count; ++f)
    {
        const float time = float(f) / 10.0f;
        EXPECT_NEAR(baked.morph_weights[f * 2 + 0], authoring.morphs[0].curve.evaluate(time), 1e-5f);
        EXPECT_NEAR(baked.morph_weights[f * 2 + 1], authoring.morphs[1].curve.evaluate(time), 1e-5f);
        EXPECT_NEAR(baked.generic_values[f], authoring.generics[0].curve.evaluate(time), 1e-5f);
    }

    // The two morph curves run opposite ways, so a transposed or duplicated bake would make them
    // agree somewhere they must not.
    EXPECT_NEAR(baked.morph_weights[0], 0.0f, 1e-5f);
    EXPECT_NEAR(baked.morph_weights[1], 1.0f, 1e-5f);
}

TEST(Unit_AnimationKeyframe,TheBakeRefusesWhatItCannotResampleAndHandlesAStaticPose)
{
    ClipAuthoring empty;
    ClipDescription out;
    // No joints means no clip: a `ClipDescription` with zero joints would validate and pose
    // nothing, which is harder to diagnose than a refusal.
    EXPECT_FALSE(empty.bake(30.0f, out));

    ClipAuthoring one_joint;
    one_joint.joints.resize(1);
    EXPECT_FALSE(one_joint.bake(0.0f, out));
    EXPECT_FALSE(one_joint.bake(-30.0f, out));

    // A pose with no keys at all is a legitimate one-frame clip — a static prop, or a rest pose
    // recorded once — and must not divide by a zero-length duration.
    one_joint.joints[0].default_translation = Vector3f{0.0f, 1.0f, 0.0f};
    ASSERT_TRUE(one_joint.bake(30.0f, out));
    EXPECT_EQ(out.frame_count, 1u);
    EXPECT_FLOAT_EQ(out.translations[0].y, 1.0f);
    EXPECT_FLOAT_EQ(out.scales[0].x, 1.0f);
}

TEST(Unit_AnimationKeyframe,RecordingAPoseAndBakingItReproducesThePosesRecorded)
{
    // The record workflow end to end: a rig posed by hand, IK or physics is sampled over time and
    // must come back out of the bake as the poses that went in. This is the round trip the
    // "turn a ragdoll into a clip" feature rests on.
    const NameHash names[2] = {hash_name("hips"), hash_name("spine")};
    ClipAuthoring authoring;
    PoseRecorder recorder;
    recorder.begin(authoring, names, 2);
    ASSERT_EQ(recorder.clip(), &authoring);
    ASSERT_EQ(authoring.joints.size(), 2u);
    EXPECT_EQ(authoring.joints[0].joint_name, names[0]);
    EXPECT_EQ(authoring.joints[1].joint_name, names[1]);

    const float times[3] = {0.0f, 0.5f, 1.0f};
    Vector3f translations[3][2] = {{{0, 0, 0}, {0, 1, 0}},
                                  {{0, 0.25f, 0}, {0, 1, 0.5f}},
                                  {{0, 0.5f, 0}, {0, 1, 1.0f}}};
    Quaternionf rotations[3][2] = {{IDENTITY, IDENTITY},
                                   {IDENTITY, axis_angle(1, 0, 0, 30.0f)},
                                   {IDENTITY, axis_angle(1, 0, 0, 60.0f)}};
    Vector3f scales[2] = {{1, 1, 1}, {1, 1, 1}};

    for (std::size_t i = 0; i < 3; ++i)
        recorder.record_pose(times[i], translations[i], rotations[i], scales, 2);

    // One key per recorded time per channel, and no duplicates.
    EXPECT_EQ(authoring.joints[1].translation_z.keys.size(), 3u);
    EXPECT_EQ(authoring.joints[1].rotation.keys.size(), 3u);

    ClipDescription baked;
    ASSERT_TRUE(authoring.bake(2.0f, baked)); // exactly the recorded times: 0, 0.5, 1.0
    ASSERT_EQ(baked.frame_count, 3u);

    for (std::uint32_t f = 0; f < 3; ++f)
    {
        const std::uint32_t spine = f * baked.joint_count + 1;
        EXPECT_NEAR(baked.translations[spine].z, translations[f][1].z, 1e-5f) << "frame " << f;
        EXPECT_NEAR(baked.rotations[spine].x, rotations[f][1].x, 1e-4f) << "frame " << f;
    }

    // Re-recording a time that was already keyed overwrites it, which is what makes scrubbing
    // back and re-posing a frame an edit rather than a second key at the same instant.
    Vector3f corrected[2] = {{0, 0, 0}, {0, 1, 9.0f}};
    recorder.record_pose(0.5f, corrected, rotations[1], scales, 2);
    EXPECT_EQ(authoring.joints[1].translation_z.keys.size(), 3u);
    EXPECT_FLOAT_EQ(authoring.joints[1].translation_z.evaluate(0.5f), 9.0f);

    // And a recorder never pointed at a clip drops the pose instead of dereferencing null.
    PoseRecorder unbound;
    EXPECT_EQ(unbound.clip(), nullptr);
    unbound.record_pose(0.0f, corrected, rotations[0], scales, 2);
}

TEST(Unit_AnimationKeyframe,GenericTracksDispatchByNameToTheirBoundTargets)
{
    // The `IFloatSink` seam: the animation layer knows a property hash and a float, and nothing
    // about what either means. The registry is the concrete half, and the case worth pinning is
    // the unbound property — dropped, not written somewhere arbitrary.
    ClipAuthoring authoring;
    authoring.joints.resize(1);
    authoring.generics.push_back({"emissive", curve_of(InterpolationMode::Linear,
                                                       {{0.0f, 0.0f}, {1.0f, 4.0f}})});
    authoring.generics.push_back({"widgetAlpha", curve_of(InterpolationMode::Linear,
                                                          {{0.0f, 1.0f}, {1.0f, 0.0f}})});

    ClipDescription description;
    ASSERT_TRUE(authoring.bake(10.0f, description));
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(description, blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());
    ASSERT_EQ(clip.generic_track_count, 2u);

    // Looked up by name, because a track's index is a property of the clip and a consumer knows
    // only what it asked for.
    const int emissive = clip.find_generic(hash_name("emissive"));
    ASSERT_GE(emissive, 0);
    EXPECT_LT(clip.find_generic(hash_name("nothingNamedThis")), 0);
    EXPECT_NEAR(clip.sample_generic_track(0.5f, false, std::uint32_t(emissive)), 2.0f, 1e-4f);

    float emissive_target = -1.0f;
    float alpha_target = -1.0f;
    GenericBindingRegistry registry;
    registry.bind(hash_name("emissive"), &emissive_target);
    registry.bind(hash_name("widgetAlpha"), &alpha_target);
    apply_generic_tracks(clip, 0.5f, false, registry);
    EXPECT_NEAR(emissive_target, 2.0f, 1e-4f);
    EXPECT_NEAR(alpha_target, 0.5f, 1e-4f);

    // A clip driving a property nothing bound must leave every bound target alone rather than
    // writing through a stale or null pointer.
    float only_target = -1.0f;
    GenericBindingRegistry partial;
    partial.bind(hash_name("emissive"), &only_target);
    partial.bind(hash_name("unbound"), nullptr);
    apply_generic_tracks(clip, 1.0f, false, partial);
    EXPECT_NEAR(only_target, 4.0f, 1e-4f);

    // Every track is dispatched, in track order.
    RecordingSink sink;
    apply_generic_tracks(clip, 0.0f, false, sink);
    ASSERT_EQ(sink.properties.size(), 2u);
    EXPECT_EQ(sink.properties[0], hash_name("emissive"));
    EXPECT_EQ(sink.properties[1], hash_name("widgetAlpha"));
}

TEST(Unit_AnimationKeyframe,AClipWithMoreTracksThanTheDispatchBoundStaysWithinIt)
{
    // A regression test for a real buffer overrun. `apply_generic_tracks` clamped its *dispatch
    // loop* to MAX_GENERIC_TRACKS but sampled with `sample_generic`, which writes one value per
    // track in the clip — so a clip with more tracks than the bound wrote past the end of a
    // fixed-size stack array. Nothing caps a cooked clip's generic track count, so authoring 70
    // named properties was all it took. The fix samples a track at a time; this asserts both that
    // it stays in bounds and that the first MAX_GENERIC_TRACKS are still delivered correctly.
    const std::uint32_t track_count = MAX_GENERIC_TRACKS + 6;
    ClipAuthoring authoring;
    authoring.joints.resize(1);
    for (std::uint32_t t = 0; t < track_count; ++t)
    {
        ScalarCurve curve = curve_of(InterpolationMode::Linear,
                                     {{0.0f, float(t)}, {1.0f, float(t) * 2.0f}});
        authoring.generics.push_back({"property_" + std::to_string(t), curve});
    }

    ClipDescription description;
    ASSERT_TRUE(authoring.bake(4.0f, description));
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(description, blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());
    ASSERT_EQ(clip.generic_track_count, track_count);

    RecordingSink sink;
    apply_generic_tracks(clip, 0.0f, false, sink);
    EXPECT_EQ(sink.properties.size(), std::size_t(MAX_GENERIC_TRACKS));
    for (std::uint32_t t = 0; t < MAX_GENERIC_TRACKS; ++t)
    {
        EXPECT_EQ(sink.properties[t], hash_name(("property_" + std::to_string(t)).c_str()));
        EXPECT_NEAR(sink.values[t], float(t), 1e-4f) << "track " << t;
    }

    // The tracks past the bound are still readable one at a time — the bound is the dispatch
    // helper's buffer, not a limit on what a clip may carry.
    const int last = clip.find_generic(hash_name(("property_" + std::to_string(track_count - 1)).c_str()));
    ASSERT_GE(last, 0);
    EXPECT_NEAR(clip.sample_generic_track(1.0f, false, std::uint32_t(last)),
                float(track_count - 1) * 2.0f, 1e-3f);
    // And an out-of-range track answers zero rather than reading past the values array.
    EXPECT_FLOAT_EQ(clip.sample_generic_track(0.0f, false, track_count), 0.0f);
    EXPECT_FLOAT_EQ(clip.sample_generic_track(0.0f, false, track_count + 100), 0.0f);
}

TEST(Unit_AnimationKeyframe,DispatchingFromAClipWithNoGenericTracksDoesNothing)
{
    ClipAuthoring authoring;
    authoring.joints.resize(1);
    authoring.joints[0].translation_y = curve_of(InterpolationMode::Linear,
                                                {{0.0f, 0.0f}, {1.0f, 1.0f}});
    ClipDescription description;
    ASSERT_TRUE(authoring.bake(10.0f, description));
    std::vector<std::byte> blob;
    ASSERT_TRUE(build_clip_blob(description, blob));
    const ClipView clip = load_clip_blob(blob.data(), blob.size());
    ASSERT_TRUE(clip.valid());
    EXPECT_EQ(clip.generic_track_count, 0u);

    RecordingSink sink;
    apply_generic_tracks(clip, 0.5f, true, sink);
    EXPECT_TRUE(sink.properties.empty());

    // An invalid clip is the other early-out, and it is the one a caller hits by playing an
    // asset that failed to load.
    const ClipView invalid{};
    apply_generic_tracks(invalid, 0.5f, true, sink);
    EXPECT_TRUE(sink.properties.empty());
}
