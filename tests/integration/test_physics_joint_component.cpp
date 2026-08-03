/**************************************************************************/
/* test_physics_joint_component.cpp                                       */
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
 * @file test_physics_joint_component.cpp
 * @brief §5.5's `PhysicsJoint` from the authoring boundary down to the solver.
 *
 * P3 built the joint library, its force recovery and its break thresholds, and P3's
 * one outstanding item was that none of it could be reached from a scene: `ISimulation`
 * deliberately does not expose the physics boundary, so a joint could only be created
 * by C++ that named `IJointService` directly. These tests are the proof that the gap is
 * closed — an author attaches a component, and a body is held.
 *
 * The load-bearing assertion is the *control*: the hang test builds a third body
 * identically and joints it to nothing, so a pass means the joint held the first one
 * rather than the scene happening not to move. Every other test that asserts a fall
 * measures it against the same body's own starting height for the same reason.
 */

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

#include <SushiEngine/authoring/command_history.hpp>
#include "scene_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief How many fixed steps a hang test runs before it reads a height. */
    constexpr int SETTLE_TICKS = 60;

    /** @brief Empties the world, demo seeds included, so a test builds from zero. */
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    /** @brief The first entity carrying @p name, or `NULL_ENTITY`. */
    EntityId find_by_name(IWorldEditor& world, const std::string& name)
    {
        for (const EntityId id : world.entities())
            if (world.name(id) == name)
                return id;
        return NULL_ENTITY;
    }

    /**
     * @brief A box with a rigid body at @p position.
     * @param world    The world to build in.
     * @param name     The entity's display name.
     * @param position Where to put it.
     * @param pinned   Whether it has zero inverse mass, making it an immovable endpoint.
     */
    EntityId make_body(IWorldEditor& world, const std::string& name, const Vector3& position,
                       bool pinned)
    {
        const EntityId id = world.create_box(name);
        EntityTransform transform = world.transform(id);
        transform.position = position;
        world.set_transform(id, transform);
        world.set_has_physics_body(id, true);
        PhysicsBodyParameters body;
        body.inv_mass = pinned ? Scalar(0) : Scalar(1);
        body.inv_inertia = pinned ? Vector3{0, 0, 0} : Vector3{6, 6, 6};
        world.set_physics_body_parameters(id, body);
        return id;
    }

    /**
     * @brief A joint whose every field differs from its default.
     *
     * Every field, deliberately: a round-trip test that leaves one field at its default
     * passes whether that field is written or dropped, which is the one outcome it was
     * supposed to distinguish.
     */
    PhysicsJointParameters reference_joint(EntityId partner)
    {
        PhysicsJointParameters parameters;
        parameters.connected_body = partner;
        parameters.joint.type = JointType::Hinge;
        parameters.joint.anchor_a = Vector3{0.25, -0.5, 0.75};
        parameters.joint.anchor_b = Vector3{-1.5, 2.0, 0.125};
        parameters.joint.axis_a = Vector3{0, 1, 0};
        parameters.joint.axis_b = Vector3{0, 0, 1};
        parameters.joint.compliance = Scalar(1e-6);
        parameters.joint.linear_limit =
            JointLimitDescription{Scalar(-0.5), Scalar(0.5), Scalar(1e-5), true};
        parameters.joint.twist_limit =
            JointLimitDescription{Scalar(-1.25), Scalar(0.75), Scalar(2e-5), true};
        parameters.joint.swing_limit =
            JointLimitDescription{Scalar(0), Scalar(0.4), Scalar(3e-5), true};
        parameters.joint.motor.type = JointMotorType::Velocity;
        parameters.joint.motor.target = Scalar(2.5);
        parameters.joint.motor.max_force = Scalar(125);
        parameters.joint.motor.compliance = Scalar(4e-5);
        parameters.joint.motor.damping = Scalar(8);
        parameters.joint.break_force = Scalar(4200);
        parameters.joint.break_torque = Scalar(310);
        return parameters;
    }

    void expect_limit_equal(const JointLimitDescription& actual,
                            const JointLimitDescription& expected)
    {
        EXPECT_DOUBLE_EQ(double(actual.lower), double(expected.lower));
        EXPECT_DOUBLE_EQ(double(actual.upper), double(expected.upper));
        EXPECT_DOUBLE_EQ(double(actual.compliance), double(expected.compliance));
        EXPECT_EQ(actual.enabled, expected.enabled);
    }

    void expect_joint_equal(const PhysicsJointParameters& actual,
                            const PhysicsJointParameters& expected)
    {
        EXPECT_EQ(actual.connected_body, expected.connected_body);
        EXPECT_EQ(actual.joint.type, expected.joint.type);
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_a.x), double(expected.joint.anchor_a.x));
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_a.y), double(expected.joint.anchor_a.y));
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_a.z), double(expected.joint.anchor_a.z));
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_b.x), double(expected.joint.anchor_b.x));
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_b.y), double(expected.joint.anchor_b.y));
        EXPECT_DOUBLE_EQ(double(actual.joint.anchor_b.z), double(expected.joint.anchor_b.z));
        EXPECT_DOUBLE_EQ(double(actual.joint.axis_a.y), double(expected.joint.axis_a.y));
        EXPECT_DOUBLE_EQ(double(actual.joint.axis_b.z), double(expected.joint.axis_b.z));
        EXPECT_DOUBLE_EQ(double(actual.joint.compliance), double(expected.joint.compliance));
        expect_limit_equal(actual.joint.linear_limit, expected.joint.linear_limit);
        expect_limit_equal(actual.joint.twist_limit, expected.joint.twist_limit);
        expect_limit_equal(actual.joint.swing_limit, expected.joint.swing_limit);
        EXPECT_EQ(actual.joint.motor.type, expected.joint.motor.type);
        EXPECT_DOUBLE_EQ(double(actual.joint.motor.target), double(expected.joint.motor.target));
        EXPECT_DOUBLE_EQ(double(actual.joint.motor.max_force),
                         double(expected.joint.motor.max_force));
        EXPECT_DOUBLE_EQ(double(actual.joint.motor.compliance),
                         double(expected.joint.motor.compliance));
        EXPECT_DOUBLE_EQ(double(actual.joint.motor.damping), double(expected.joint.motor.damping));
        EXPECT_DOUBLE_EQ(double(actual.joint.break_force), double(expected.joint.break_force));
        EXPECT_DOUBLE_EQ(double(actual.joint.break_torque), double(expected.joint.break_torque));
    }

    /** @brief Runs @p count fixed steps. */
    void step(ISimulation& simulation, int count)
    {
        for (int i = 0; i < count; ++i)
            simulation.tick(simulation.fixed_dt_seconds());
    }
}

TEST(Integration_PhysicsJoint, AJointHoldsUpABodyThatWouldOtherwiseFall)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    const EntityId control = make_body(world, "Control", Vector3{6, 10, 0}, false);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Fixed;
    parameters.joint.anchor_a = Vector3{0, 0, 0};
    parameters.joint.anchor_b = Vector3{2, 0, 0};
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);

    const Scalar start = world.transform(held).position.y;
    step(*simulation, SETTLE_TICKS);

    const Scalar held_drop = start - world.transform(held).position.y;
    const Scalar control_drop = start - world.transform(control).position.y;

    // The control is what makes this an assertion about the joint rather than about
    // whether the scene has gravity at all.
    EXPECT_GT(double(control_drop), 0.5);
    EXPECT_LT(double(held_drop), 0.05);
}

TEST(Integration_PhysicsJoint, ItIsLiveOnlyWhenBothEndsAreBodies)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    const EntityId bare = world.create("NoBody");

    PhysicsJointParameters parameters;
    parameters.joint.type = JointType::Fixed;
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);
    step(*simulation, 2);

    JointState load;
    EXPECT_FALSE(world.joint_load(held, load)) << "a joint with no partner cannot be live";

    parameters.connected_body = bare;
    world.set_joint_parameters(held, parameters);
    step(*simulation, 2);
    EXPECT_FALSE(world.joint_load(held, load)) << "a partner with no rigid body is no endpoint";

    parameters.connected_body = held;
    world.set_joint_parameters(held, parameters);
    step(*simulation, 2);
    EXPECT_FALSE(world.joint_load(held, load)) << "a joint to itself is degenerate, not stiff";

    parameters.connected_body = anchor;
    world.set_joint_parameters(held, parameters);
    step(*simulation, 2);
    EXPECT_TRUE(world.joint_load(held, load));
}

TEST(Integration_PhysicsJoint, ALoadedJointReportsWhatItIsCarrying)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Fixed;
    parameters.joint.anchor_b = Vector3{2, 0, 0};
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);
    step(*simulation, SETTLE_TICKS);

    JointState load;
    ASSERT_TRUE(world.joint_load(held, load));

    // §10.4's recovery is exact, so a settled mount carries the weight it is holding
    // and nothing else: one unit mass under this scene's gravity. Asserted as a band
    // rather than a number because the field is sampled per body per tick from the
    // celestial sum, not from a scene-wide constant.
    const double magnitude = double(length(load.force));
    EXPECT_GT(magnitude, 1.0);
    EXPECT_LT(magnitude, 100.0);
    EXPECT_GE(double(load.peak_force), magnitude * 0.5);
}

TEST(Integration_PhysicsJoint, ABreakableJointTearsOutAndStaysOut)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Fixed;
    parameters.joint.anchor_b = Vector3{2, 0, 0};
    // Below the weight it is being asked to hold, so the first loaded substep exceeds it.
    parameters.joint.break_force = Scalar(0.001);
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);

    const Scalar start = world.transform(held).position.y;
    step(*simulation, SETTLE_TICKS);

    EXPECT_TRUE(world.joint_broken(held));
    JointState load;
    EXPECT_FALSE(world.joint_load(held, load)) << "a broken joint is gone, not merely slack";
    EXPECT_GT(double(start - world.transform(held).position.y), 0.5) << "so the body falls";

    // The authoring survives the break — an author who set a threshold has not thereby
    // deleted their hinge — and it is not silently rebuilt on the next tick either.
    EXPECT_TRUE(world.has_joint(held));
    step(*simulation, 10);
    EXPECT_TRUE(world.joint_broken(held));
}

TEST(Integration_PhysicsJoint, EditingABrokenJointPutsItBack)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Fixed;
    parameters.joint.anchor_b = Vector3{2, 0, 0};
    parameters.joint.break_force = Scalar(0.001);
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);
    step(*simulation, 10);
    ASSERT_TRUE(world.joint_broken(held));

    parameters.joint.break_force = Scalar(0);
    world.set_joint_parameters(held, parameters);
    step(*simulation, 2);

    EXPECT_FALSE(world.joint_broken(held));
    JointState load;
    EXPECT_TRUE(world.joint_load(held, load));
}

TEST(Integration_PhysicsJoint, DestroyingThePartnerReleasesTheJoint)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Fixed;
    parameters.joint.anchor_b = Vector3{2, 0, 0};
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);
    step(*simulation, 4);

    JointState load;
    ASSERT_TRUE(world.joint_load(held, load));

    const Scalar start = world.transform(held).position.y;
    world.destroy(anchor);
    step(*simulation, SETTLE_TICKS);

    EXPECT_FALSE(world.joint_load(held, load));
    EXPECT_FALSE(world.joint_broken(held)) << "released is not broken; nothing exceeded a limit";
    EXPECT_GT(double(start - world.transform(held).position.y), 0.5);
}

TEST(Integration_PhysicsJoint, EveryFieldSurvivesCaptureAndApply)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, reference_joint(anchor));

    const nlohmann::json snapshot = Scene::capture_scene(world);
    world.set_joint_parameters(held, PhysicsJointParameters{});
    world.set_has_joint(held, false);

    Scene::apply_scene(world, snapshot);

    const EntityId restored = find_by_name(world, "Held");
    const EntityId restored_anchor = find_by_name(world, "Anchor");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_joint(restored));
    expect_joint_equal(world.joint_parameters(restored), reference_joint(restored_anchor));
}

TEST(Integration_PhysicsJoint, ThePartnerSurvivesBeingWrittenAfterTheJoint)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    // The owner is created *first*, so it is written to the file before the entity it
    // names. A partner resolved during the first load pass would find nothing here,
    // which is exactly why the second pass exists.
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);

    PhysicsJointParameters parameters;
    parameters.connected_body = anchor;
    parameters.joint.type = JointType::Hinge;
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, parameters);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    clear_world(world);
    Scene::apply_scene(world, snapshot);

    const EntityId restored = find_by_name(world, "Held");
    const EntityId restored_anchor = find_by_name(world, "Anchor");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_NE(restored_anchor, NULL_ENTITY);
    EXPECT_EQ(world.joint_parameters(restored).connected_body, restored_anchor);
}

TEST(Integration_PhysicsJoint, AnUnconnectedJointRoundTripsAsUnconnected)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    world.set_has_joint(held, true);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    clear_world(world);
    Scene::apply_scene(world, snapshot);

    const EntityId restored = find_by_name(world, "Held");
    ASSERT_NE(restored, NULL_ENTITY);
    EXPECT_TRUE(world.has_joint(restored)) << "authoring in progress is still authoring";
    EXPECT_EQ(world.joint_parameters(restored).connected_body, NULL_ENTITY);
}

TEST(Integration_PhysicsJoint, DetachingTheComponentLeavesNothingBehind)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId anchor = make_body(world, "Anchor", Vector3{0, 10, 0}, true);
    const EntityId held = make_body(world, "Held", Vector3{2, 10, 0}, false);
    world.set_has_joint(held, true);
    world.set_joint_parameters(held, reference_joint(anchor));
    step(*simulation, 4);

    world.set_has_joint(held, false);
    step(*simulation, 2);

    JointState load;
    EXPECT_FALSE(world.has_joint(held));
    EXPECT_FALSE(world.joint_load(held, load));

    const nlohmann::json snapshot = Scene::capture_scene(world);
    clear_world(world);
    Scene::apply_scene(world, snapshot);
    EXPECT_FALSE(world.has_joint(find_by_name(world, "Held")));
}
