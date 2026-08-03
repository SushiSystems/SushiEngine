/**************************************************************************/
/* test_animator_step.cpp                                                 */
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

// Unit_AnimatorStep: the deterministic half of the animator (design §5.1/§5.5). The
// contract this guards is §0.2's: everything that can affect gameplay advances only in the
// fixed tick, lives in trivially-copyable columns, and survives a rollback capture/restore
// byte-exactly. A regression here does not look like a wrong pose — it looks like a replay
// diverging from the session it is replaying, which is far more expensive to find later.
//
// Also covers the state-machine semantics the authoring surface promises: exit time,
// typed conditions, a trigger consumed exactly once, crossfades, events fired on the tick
// they are crossed, and root motion in both translation and rotation.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_step.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    constexpr float PI = 3.14159265358979323846f;
    constexpr float TICK = 1.0f / 60.0f;
    const Quaternionf IDENTITY{0.0f, 0.0f, 0.0f, 1.0f};

    Quaternionf turn_about_y(float radians)
    {
        return quaternion_axis_angle(Vector3T<float>{0.0f, 1.0f, 0.0f}, radians);
    }

    struct CountingSink : IAnimationEventSink
    {
        int footsteps = 0;
        std::uint32_t last_layer = 0xFFFFFFFFu;
        std::int32_t last_payload = 0;

        void on_animation_event(std::uint32_t, const AnimatorEvent& event) override
        {
            if (event.name == hash_name("footstep"))
            {
                ++footsteps;
                last_layer = event.layer;
                last_payload = event.payload;
            }
        }
    };

    // A world holding the assets a controller needs, so each test builds only its controller.
    struct Fixture
    {
        AnimationDatabase database;
        AssetId skeleton = INVALID_ASSET;

        Fixture()
        {
            SkeletonDescription description;
            JointDescription root;
            root.name = "root";
            root.parent = -1;
            JointDescription child;
            child.name = "child";
            child.parent = 0;
            child.bind_translation = Vector3f{0.0f, 1.0f, 0.0f};
            description.joints = {root, child};
            std::vector<std::byte> blob;
            build_skeleton_blob(description, blob);
            skeleton = database.add_skeleton(std::move(blob));
        }

        // A one-second clip at 30 Hz whose root advances `forward` units in +Z and turns
        // `turn` radians about Y over its length. The child never moves.
        AssetId add_clip(float forward, float turn)
        {
            ClipDescription clip;
            clip.joint_count = 2;
            clip.frame_count = 31;
            clip.sample_rate = 30.0f;
            clip.translations.resize(31 * 2);
            clip.rotations.resize(31 * 2, IDENTITY);
            clip.scales.assign(31 * 2, Vector3f{1, 1, 1});
            for (std::uint32_t f = 0; f < 31; ++f)
            {
                const float phase = static_cast<float>(f) / 30.0f;
                clip.translations[f * 2 + 0] = Vector3f{0.0f, 0.0f, phase * forward};
                clip.rotations[f * 2 + 0] = turn_about_y(phase * turn);
                clip.translations[f * 2 + 1] = Vector3f{0.0f, 1.0f, 0.0f};
            }
            std::vector<std::byte> blob;
            build_clip_blob(clip, blob);
            return database.add_clip(std::move(blob));
        }

        ControllerView compile(const ControllerDescription& description)
        {
            std::vector<std::byte> blob;
            if (!compile_controller_blob(description, blob))
                return ControllerView{};
            const AssetId id = database.add_controller(std::move(blob));
            controller_id = id;
            return database.controller(id);
        }

        AnimatorInstance instance() const
        {
            AnimatorInstance animator{};
            animator.controller = controller_id;
            animator.skeleton = skeleton;
            return animator;
        }

        AssetId controller_id = INVALID_ASSET;
    };

    StateDescription state_with(const char* name, AssetId clip)
    {
        StateDescription state;
        state.name = name;
        state.clip = clip;
        return state;
    }

    // Idle <-> Walk over a "moving" bool: a crossfade in, an instant transition back, and a
    // footstep event halfway through Walk. The shape almost every locomotion graph starts as.
    ControllerDescription locomotion(AssetId idle_clip, AssetId walk_clip, float crossfade = 0.1f)
    {
        ControllerDescription description;
        description.parameters.push_back(ParameterDescription{"moving", ParameterType::Bool, 0.0f});

        LayerDescription layer;
        layer.name = "base";
        layer.default_state = "Idle";

        StateDescription idle = state_with("Idle", idle_clip);
        TransitionDescription to_walk;
        to_walk.destination = "Walk";
        to_walk.duration = crossfade;
        to_walk.conditions.push_back(ConditionDescription{"moving", Comparator::If, 0.0f});
        idle.transitions.push_back(to_walk);

        StateDescription walk = state_with("Walk", walk_clip);
        walk.events.push_back(StateEventDescription{0.5f, "footstep", 7});
        TransitionDescription to_idle;
        to_idle.destination = "Idle";
        to_idle.duration = 0.0f;
        to_idle.conditions.push_back(ConditionDescription{"moving", Comparator::IfNot, 0.0f});
        walk.transitions.push_back(to_idle);

        layer.states = {idle, walk};
        description.layers.push_back(layer);
        return description;
    }

    float quaternion_angle(const Quaternion& q)
    {
        const double w = std::min(1.0, std::max(-1.0, static_cast<double>(q.w)));
        return static_cast<float>(2.0 * std::acos(w));
    }
}

TEST(Unit_AnimatorStep, TheDeterministicStateIsTriviallyCopyable)
{
    // §0.2 and §5.5: rollback captures these columns with a memcpy, so a member that needs a
    // constructor or owns memory would silently corrupt a snapshot rather than fail to build.
    static_assert(std::is_trivially_copyable<AnimatorInstance>::value,
                  "AnimatorInstance must be memcpy-snapshottable");
    static_assert(std::is_trivially_copyable<AnimatorLayerState>::value, "");
    static_assert(std::is_trivially_copyable<AnimatorParameterBlock>::value, "");
    static_assert(std::is_trivially_copyable<AnimatorEventQueue>::value, "");
    static_assert(std::is_trivially_copyable<RootMotionDelta>::value, "");
    SUCCEED();
}

TEST(Unit_AnimatorStep, ReachesItsDestinationStateAndReturns)
{
    Fixture world;
    const AssetId idle = world.add_clip(0.0f, 0.0f);
    const AssetId walk = world.add_clip(2.0f, 0.0f);
    const ControllerView controller = world.compile(locomotion(idle, walk));
    ASSERT_TRUE(controller.valid());
    const int moving = controller.find_parameter(hash_name("moving"));
    ASSERT_EQ(moving, 0);

    AnimatorInstance animator = world.instance();
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 0) << "starts in the layer's default state";
    EXPECT_EQ(animator.layers[0].transition_state, -1);

    // Raising the condition starts a crossfade rather than snapping, because the transition
    // carries a duration.
    animator.parameters.set_bool(static_cast<std::uint32_t>(moving), true);
    animator_step(controller, world.database, animator, TICK);
    EXPECT_GE(animator.layers[0].transition_state, 0) << "a crossfade is in progress";
    EXPECT_EQ(animator.layers[0].next_state, 1);
    EXPECT_EQ(animator.layers[0].current_state, 0) << "still leaving Idle";

    for (int tick = 0; tick < 30; ++tick)
        animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 1) << "the crossfade completed into Walk";
    EXPECT_EQ(animator.layers[0].transition_state, -1);

    // Dropping it takes the zero-duration transition, which switches on the same tick.
    animator.parameters.set_bool(static_cast<std::uint32_t>(moving), false);
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 0);
    EXPECT_EQ(animator.layers[0].transition_state, -1) << "instant, so never a crossfade";
}

TEST(Unit_AnimatorStep, ExitTimeHoldsATransitionUntilTheClipReachesIt)
{
    // A transition with an exit time may only fire as the clip crosses that normalized time,
    // however long its condition has been true — the difference between a graph that cuts
    // mid-stride and one that waits for the foot to land.
    Fixture world;
    const AssetId clip = world.add_clip(0.0f, 0.0f);
    ControllerDescription description;
    LayerDescription layer;
    layer.name = "base";
    layer.default_state = "A";
    StateDescription a = state_with("A", clip);
    TransitionDescription timed;
    timed.destination = "B";
    timed.has_exit_time = true;
    timed.exit_time = 0.75f;
    timed.duration = 0.0f;
    a.transitions.push_back(timed);
    layer.states = {a, state_with("B", clip)};
    description.layers.push_back(layer);

    const ControllerView controller = world.compile(description);
    ASSERT_TRUE(controller.valid());
    AnimatorInstance animator = world.instance();

    // The clip is one second long; 0.75 normalized is 0.75 s, i.e. tick 45 at 60 Hz.
    int switched_at = -1;
    for (int tick = 0; tick < 120 && switched_at < 0; ++tick)
    {
        animator_step(controller, world.database, animator, TICK);
        if (animator.layers[0].current_state == 1)
            switched_at = tick;
    }
    ASSERT_GE(switched_at, 0) << "the transition never fired";
    EXPECT_GE(switched_at, 44) << "fired before the exit time";
    EXPECT_LE(switched_at, 46) << "fired later than the tick that crosses the exit time";
}

TEST(Unit_AnimatorStep, ATriggerIsConsumedByExactlyOneTransition)
{
    // A trigger is a one-shot: the transition that takes it must clear it, or the next state
    // fires straight back out on the following tick. Two states each waiting on the same
    // trigger is the arrangement that exposes it.
    Fixture world;
    const AssetId clip = world.add_clip(0.0f, 0.0f);
    ControllerDescription description;
    description.parameters.push_back(ParameterDescription{"fire", ParameterType::Trigger, 0.0f});
    LayerDescription layer;
    layer.name = "base";
    layer.default_state = "A";

    StateDescription a = state_with("A", clip);
    TransitionDescription a_to_b;
    a_to_b.destination = "B";
    a_to_b.conditions.push_back(ConditionDescription{"fire", Comparator::If, 0.0f});
    a.transitions.push_back(a_to_b);

    StateDescription b = state_with("B", clip);
    TransitionDescription b_to_a;
    b_to_a.destination = "A";
    b_to_a.conditions.push_back(ConditionDescription{"fire", Comparator::If, 0.0f});
    b.transitions.push_back(b_to_a);

    layer.states = {a, b};
    description.layers.push_back(layer);
    const ControllerView controller = world.compile(description);
    ASSERT_TRUE(controller.valid());
    const int fire = controller.find_parameter(hash_name("fire"));
    ASSERT_GE(fire, 0);

    AnimatorInstance animator = world.instance();
    animator_step(controller, world.database, animator, TICK);
    ASSERT_EQ(animator.layers[0].current_state, 0);

    animator.parameters.set_trigger(static_cast<std::uint32_t>(fire));
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 1) << "the trigger moved A -> B";

    // Without re-setting it, nothing more may happen — the one set must not also pay for
    // B -> A on any later tick.
    for (int tick = 0; tick < 10; ++tick)
        animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 1) << "the trigger fired a second transition";

    animator.parameters.set_trigger(static_cast<std::uint32_t>(fire));
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 0) << "a fresh set moves B -> A";
}

TEST(Unit_AnimatorStep, TypedConditionsCompareAgainstTheirThreshold)
{
    Fixture world;
    const AssetId clip = world.add_clip(0.0f, 0.0f);
    ControllerDescription description;
    description.parameters.push_back(ParameterDescription{"speed", ParameterType::Float, 0.0f});
    description.parameters.push_back(ParameterDescription{"stance", ParameterType::Int, 0.0f});
    LayerDescription layer;
    layer.name = "base";
    layer.default_state = "A";

    // Both conditions on one transition, so it fires only when they agree — the AND
    // semantics a Mecanim transition has.
    StateDescription a = state_with("A", clip);
    TransitionDescription guarded;
    guarded.destination = "B";
    guarded.conditions.push_back(ConditionDescription{"speed", Comparator::Greater, 1.5f});
    guarded.conditions.push_back(ConditionDescription{"stance", Comparator::Equals, 2.0f});
    a.transitions.push_back(guarded);
    layer.states = {a, state_with("B", clip)};
    description.layers.push_back(layer);

    const ControllerView controller = world.compile(description);
    ASSERT_TRUE(controller.valid());
    const auto speed = static_cast<std::uint32_t>(controller.find_parameter(hash_name("speed")));
    const auto stance = static_cast<std::uint32_t>(controller.find_parameter(hash_name("stance")));

    AnimatorInstance animator = world.instance();
    animator_step(controller, world.database, animator, TICK);

    // Above the float threshold but the wrong int: no transition.
    animator.parameters.set_float(speed, 3.0f);
    animator.parameters.set_int(stance, 1);
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 0);

    // Right int, but exactly at the threshold — Greater is strict.
    animator.parameters.set_float(speed, 1.5f);
    animator.parameters.set_int(stance, 2);
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 0) << "Greater must be strict, not >=";

    animator.parameters.set_float(speed, 1.6f);
    animator_step(controller, world.database, animator, TICK);
    EXPECT_EQ(animator.layers[0].current_state, 1);
}

TEST(Unit_AnimatorStep, EventsFireOnceEachTimeTheClipCrossesThem)
{
    Fixture world;
    const AssetId idle = world.add_clip(0.0f, 0.0f);
    const AssetId walk = world.add_clip(2.0f, 0.0f);
    const ControllerView controller = world.compile(locomotion(idle, walk, 0.0f));
    ASSERT_TRUE(controller.valid());
    const auto moving = static_cast<std::uint32_t>(controller.find_parameter(hash_name("moving")));

    AnimatorInstance animator = world.instance();
    CountingSink sink;
    animator.parameters.set_bool(moving, true);
    // Three full seconds inside Walk, whose clip is one second long with the event at 0.5.
    for (int tick = 0; tick < 3 * 60 + 2; ++tick)
    {
        animator_step(controller, world.database, animator, TICK);
        drain_events(animator, 1, sink);
    }
    EXPECT_EQ(animator.layers[0].current_state, 1);
    EXPECT_EQ(sink.footsteps, 3) << "one crossing per loop, not per tick and not only once";
    EXPECT_EQ(sink.last_layer, 0u);
    EXPECT_EQ(sink.last_payload, 7) << "the authored payload rides along";

    // The queue is per tick: draining twice must not deliver the same event again.
    CountingSink second_drain;
    drain_events(animator, 1, second_drain);
    animator_step(controller, world.database, animator, TICK);
    drain_events(animator, 1, second_drain);
    EXPECT_LE(second_drain.footsteps, 1);
}

TEST(Unit_AnimatorStep, RootMotionAdvancesTranslationAlongTheEntityFacing)
{
    Fixture world;
    const AssetId idle = world.add_clip(0.0f, 0.0f);
    const AssetId walk = world.add_clip(2.0f, 0.0f);
    const ControllerView controller = world.compile(locomotion(idle, walk, 0.0f));
    ASSERT_TRUE(controller.valid());
    const auto moving = static_cast<std::uint32_t>(controller.find_parameter(hash_name("moving")));

    AnimatorInstance animator = world.instance();
    Vector3 position{0, 0, 0};
    Quaternion orientation{0, 0, 0, 1};
    animator.parameters.set_bool(moving, true);
    for (int tick = 0; tick < 120; ++tick)
    {
        animator_step(controller, world.database, animator, TICK);
        apply_root_motion(animator.root_motion, position, orientation);
    }
    // Two seconds of a clip advancing 2 units per loop, and the loop seam must not lose or
    // double a stride — that is what root_at's per-cycle term exists for.
    EXPECT_NEAR(position.z, 4.0, 0.15);
    EXPECT_NEAR(position.x, 0.0, 1e-6);

    // The delta is in the entity's local frame: facing 180 degrees about Y sends the same
    // clip backwards in world space.
    AnimatorInstance turned_animator = world.instance();
    turned_animator.parameters.set_bool(moving, true);
    Vector3 turned_position{0, 0, 0};
    Quaternion facing_back =
        quaternion_axis_angle(Vector3{0.0, 1.0, 0.0}, static_cast<Scalar>(PI));
    for (int tick = 0; tick < 120; ++tick)
    {
        animator_step(controller, world.database, turned_animator, TICK);
        apply_root_motion(turned_animator.root_motion, turned_position, facing_back);
    }
    EXPECT_NEAR(turned_position.z, -4.0, 0.15) << "root motion ignored the entity orientation";

    // And an animator with root motion switched off must not move at all, while its state
    // machine keeps running.
    AnimatorInstance still = world.instance();
    still.apply_root_motion = 0;
    still.parameters.set_bool(moving, true);
    Vector3 still_position{0, 0, 0};
    Quaternion still_orientation{0, 0, 0, 1};
    for (int tick = 0; tick < 120; ++tick)
    {
        animator_step(controller, world.database, still, TICK);
        apply_root_motion(still.root_motion, still_position, still_orientation);
    }
    EXPECT_NEAR(still_position.z, 0.0, 1e-9);
    EXPECT_EQ(still.layers[0].current_state, 1) << "the state machine still advanced";
}

TEST(Unit_AnimatorStep, RootMotionTurnsTheEntityAndKeepsTurningAcrossTheLoopSeam)
{
    // §0.1 requires root motion to move the entity in translation *and* rotation. A clip
    // whose root turns a quarter circle per loop must turn the entity a quarter circle per
    // loop, including at the wrap — the seam a per-frame difference silently drops.
    Fixture world;
    const AssetId turning = world.add_clip(0.0f, PI * 0.5f);
    ControllerDescription description;
    LayerDescription layer;
    layer.name = "base";
    layer.default_state = "Turn";
    layer.states = {state_with("Turn", turning)};
    description.layers.push_back(layer);
    const ControllerView controller = world.compile(description);
    ASSERT_TRUE(controller.valid());

    AnimatorInstance animator = world.instance();
    Vector3 position{0, 0, 0};
    Quaternion orientation{0, 0, 0, 1};

    // One loop: a quarter turn.
    for (int tick = 0; tick < 60; ++tick)
    {
        animator_step(controller, world.database, animator, TICK);
        apply_root_motion(animator.root_motion, position, orientation);
    }
    EXPECT_NEAR(quaternion_angle(orientation), PI * 0.5f, 0.05f);

    // Two more loops: three quarters total, so the accumulation survives two wraps.
    for (int tick = 0; tick < 120; ++tick)
    {
        animator_step(controller, world.database, animator, TICK);
        apply_root_motion(animator.root_motion, position, orientation);
    }
    EXPECT_NEAR(quaternion_angle(orientation), PI * 1.5f, 0.05f);
    // The turn is about the clip's axis and nothing else.
    EXPECT_NEAR(orientation.x, 0.0, 1e-3);
    EXPECT_NEAR(orientation.z, 0.0, 1e-3);
    EXPECT_GT(orientation.y, 0.0);

    // A clip whose root does not turn must leave the orientation exactly alone, rather than
    // drifting by an accumulated near-identity delta.
    const AssetId straight = world.add_clip(1.0f, 0.0f);
    ControllerDescription straight_description;
    LayerDescription straight_layer;
    straight_layer.name = "base";
    straight_layer.default_state = "Walk";
    straight_layer.states = {state_with("Walk", straight)};
    straight_description.layers.push_back(straight_layer);
    const ControllerView straight_controller = world.compile(straight_description);
    ASSERT_TRUE(straight_controller.valid());

    AnimatorInstance walker = world.instance();
    Vector3 walk_position{0, 0, 0};
    Quaternion walk_orientation{0, 0, 0, 1};
    for (int tick = 0; tick < 300; ++tick)
    {
        animator_step(straight_controller, world.database, walker, TICK);
        apply_root_motion(walker.root_motion, walk_position, walk_orientation);
    }
    EXPECT_NEAR(quaternion_angle(walk_orientation), 0.0f, 1e-3f);
}

TEST(Unit_AnimatorStep, SameInputsReproduceTheAnimatorStateByteForByte)
{
    // The determinism contract, checked the way a lockstep replay checks it: the same
    // scripted input timeline run twice must leave identical bytes, not merely a similar pose.
    Fixture world;
    const AssetId idle = world.add_clip(0.0f, 0.0f);
    const AssetId walk = world.add_clip(2.0f, PI * 0.25f);
    const ControllerView controller = world.compile(locomotion(idle, walk));
    ASSERT_TRUE(controller.valid());
    const auto moving = static_cast<std::uint32_t>(controller.find_parameter(hash_name("moving")));

    const auto moving_at = [](int tick) { return tick >= 30 && tick < 150; };
    const int TICKS = 200;

    const auto run = [&](AnimatorInstance& animator, CountingSink& sink, int from, int to)
    {
        for (int tick = from; tick < to; ++tick)
        {
            animator.parameters.set_bool(moving, moving_at(tick));
            animator_step(controller, world.database, animator, TICK);
            drain_events(animator, 1, sink);
        }
    };

    AnimatorInstance first = world.instance();
    CountingSink first_sink;
    run(first, first_sink, 0, TICKS);

    AnimatorInstance second = world.instance();
    CountingSink second_sink;
    run(second, second_sink, 0, TICKS);

    EXPECT_EQ(std::memcmp(&first, &second, sizeof(AnimatorInstance)), 0)
        << "the same inputs produced different animator bytes";
    EXPECT_EQ(first_sink.footsteps, second_sink.footsteps);
    EXPECT_GT(first_sink.footsteps, 0) << "the run must actually exercise events";
    EXPECT_EQ(first.layers[0].current_state, 0) << "and end back in Idle";
}

TEST(Unit_AnimatorStep, RollbackRestoreAndReplayReproducesTheSameFinalState)
{
    // §5.5: a snapshot is a memcpy of the instance and nothing else. Restoring one and
    // replaying the same inputs must land on the same bytes as the run that was rolled back,
    // or a mispredicted tick in a networked session desynchronizes permanently.
    Fixture world;
    const AssetId idle = world.add_clip(0.0f, 0.0f);
    const AssetId walk = world.add_clip(2.0f, PI * 0.25f);
    const ControllerView controller = world.compile(locomotion(idle, walk));
    ASSERT_TRUE(controller.valid());
    const auto moving = static_cast<std::uint32_t>(controller.find_parameter(hash_name("moving")));

    const auto moving_at = [](int tick) { return tick >= 30 && tick < 150; };
    constexpr int TICKS = 200;
    constexpr int CAPTURE_AT = 90; // mid-crossfade-and-walk, the interesting place to resume

    AnimatorInstance animator = world.instance();
    const auto advance = [&](int from, int to)
    {
        for (int tick = from; tick < to; ++tick)
        {
            animator.parameters.set_bool(moving, moving_at(tick));
            animator_step(controller, world.database, animator, TICK);
        }
    };

    advance(0, CAPTURE_AT);
    AnimatorInstance snapshot{};
    std::memcpy(&snapshot, &animator, sizeof(AnimatorInstance));

    advance(CAPTURE_AT, TICKS);
    AnimatorInstance without_rollback{};
    std::memcpy(&without_rollback, &animator, sizeof(AnimatorInstance));

    std::memcpy(&animator, &snapshot, sizeof(AnimatorInstance));
    advance(CAPTURE_AT, TICKS);

    EXPECT_EQ(std::memcmp(&without_rollback, &animator, sizeof(AnimatorInstance)), 0)
        << "restore + replay diverged from the run it replayed";

    // The snapshot itself must be mid-flight, or the test proves nothing about resuming.
    EXPECT_EQ(snapshot.layers[0].current_state, 1);
    EXPECT_GT(snapshot.layers[0].normalized_time, 0.0f);
}

TEST(Unit_AnimatorStep, AnInvalidControllerLeavesTheAnimatorUntouched)
{
    // The animator is stepped for every animated entity every tick, including ones whose
    // asset failed to load; that must be a no-op rather than a read through a null table.
    Fixture world;
    AnimatorInstance animator = world.instance();
    AnimatorInstance before{};
    std::memcpy(&before, &animator, sizeof(AnimatorInstance));

    animator_step(ControllerView{}, world.database, animator, TICK);
    EXPECT_EQ(std::memcmp(&before, &animator, sizeof(AnimatorInstance)), 0);
    EXPECT_EQ(animator.initialized, 0u);
}
