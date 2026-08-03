/**************************************************************************/
/* test_joint_parking.cpp                                                 */
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

// §16.44's park_sleeping_joints: a joint whose island is asleep is removed from the
// solve graph rather than dispatched for its projection's own early return to catch.
// The claim under test is the one the feature exists to make true — §13.2 item 1, "a
// settled island costs nothing" — measured the one way a test without a device timer
// can measure it: PhysicsStatistics::joints, the live count in the joint colour band,
// must actually drop to zero rather than merely stop mattering to the answer.

#include <gtest/gtest.h>

#include <SushiEngine/simulation/physics_simulation.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr std::size_t ITERATIONS = 8;
    constexpr std::size_t SUBSTEPS = 8;
    constexpr Scalar SUBSTEP_DT = Scalar(1.0 / 480.0);

    constexpr EntityId ANCHOR = 1;
    constexpr EntityId HELD = 2;

    GravitySampler earth_gravity()
    {
        return [](const Vector3&) { return Vector3{0, Scalar(-9.81), 0}; };
    }

    WindSampler still_air() { return WindSampler{}; }

    /** @brief A pinned body: zero inverse mass and inertia, so nothing can move it. */
    RigidBodyDesc anchor_desc()
    {
        RigidBodyDesc desc;
        desc.id = ANCHOR;
        desc.position = Vector3{0, Scalar(2), 0};
        desc.inv_mass = 0;
        desc.inv_inertia = Vector3{0, 0, 0};
        return desc;
    }

    /** @brief The body the fixed joint holds in place against gravity. */
    RigidBodyDesc held_desc()
    {
        RigidBodyDesc desc;
        desc.id = HELD;
        desc.position = Vector3{Scalar(1), Scalar(2), 0};
        desc.inv_mass = Scalar(1);
        desc.inv_inertia = Vector3{Scalar(1), Scalar(1), Scalar(1)};
        return desc;
    }

    /** @brief A rigid attachment between the anchor and the held body. */
    JointDesc anchor_joint()
    {
        JointDesc joint;
        joint.body_a = ANCHOR;
        joint.body_b = HELD;
        joint.params.type = JointType::Fixed;
        joint.params.anchor_b = Vector3{Scalar(-1), 0, 0};
        return joint;
    }

    /**
     * @brief Steps until the held body has settled and its island has had time to sleep.
     *
     * `sleep_motion_threshold`/`sleep_delay` default to 0.01 and 0.5s; at 8 substeps of
     * 1/480s each, one tick is 1/60s, so 0.5s is 30 ticks. 360 ticks (6s) gives the
     * fixed joint's positional solve room to settle first and the sleep timer room to
     * run out afterward, with a wide margin either side.
     */
    void settle_and_sleep(IPhysicsScene& physics)
    {
        for (int tick = 0; tick < 360; ++tick)
            physics.step(earth_gravity(), still_air(), SUBSTEPS);
    }
}

TEST(Integration_JointParking, OffByDefaultNeverParksAJoint)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({anchor_desc(), held_desc()}, ITERATIONS, SUBSTEP_DT);
    const JointId joint = physics->create_joint(anchor_joint());
    ASSERT_NE(joint, NULL_JOINT);

    // Parking is never requested, so a joint whose island sleeps stays exactly where
    // it always has: dispatched, its projection early-returning on the sleeping flag.
    settle_and_sleep(*physics);

    EXPECT_GT(physics->statistics().sleeping_bodies, 0u) << "the island never settled";
    EXPECT_EQ(physics->statistics().joints, 1u)
        << "a joint must not be parked unless parking was requested";
}

TEST(Integration_JointParking, ASettledIslandParksItsJointAndTheLiveCountDropsToZero)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_park_sleeping_joints_requested(true);
    physics->set_rigid_bodies({anchor_desc(), held_desc()}, ITERATIONS, SUBSTEP_DT);
    const JointId joint = physics->create_joint(anchor_joint());
    ASSERT_NE(joint, NULL_JOINT);

    JointState before_sleep;
    settle_and_sleep(*physics);

    EXPECT_GT(physics->statistics().sleeping_bodies, 0u) << "the island never settled";
    EXPECT_EQ(physics->statistics().joints, 0u)
        << "a settled island's joint should have been dropped from the solve graph";

    // Parked, not gone: the last solved state is exactly what a settled joint was
    // carrying, and it must still answer as if nothing had changed about it.
    JointState parked;
    ASSERT_TRUE(physics->joint_state(joint, parked));
    EXPECT_NEAR(double(length(parked.force)), 9.81, 1.0);
    (void)before_sleep;
}

TEST(Integration_JointParking, TeleportingTheHeldBodyWakesAndUnparksItsJoint)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_park_sleeping_joints_requested(true);
    physics->set_rigid_bodies({anchor_desc(), held_desc()}, ITERATIONS, SUBSTEP_DT);
    const JointId joint = physics->create_joint(anchor_joint());
    ASSERT_NE(joint, NULL_JOINT);

    settle_and_sleep(*physics);
    ASSERT_EQ(physics->statistics().joints, 0u) << "setup failed to park the joint";

    // A disturbance from outside the tick: the same call `AHardEnoughImpactTearsThe
    // DoorOff` uses to simulate an impact, here used only for the wake it causes.
    physics->set_rigid_pose(HELD, Vector3{Scalar(1), Scalar(1.9), 0}, Quaternion{});
    // Two ticks, not one: the first is the tick `update_joint_parking` observes the
    // wake and calls `add_joint` on, which is real and immediate for the constraint
    // store itself, but `PhysicsStatistics::joints` is a snapshot of the *solver's*
    // own count, refreshed once per `solver_->step()` — this tick's snapshot was
    // already taken before the add happened. The second tick's solve starts with the
    // joint already live and its own refresh reports it.
    physics->step(earth_gravity(), still_air(), SUBSTEPS);
    physics->step(earth_gravity(), still_air(), SUBSTEPS);

    EXPECT_EQ(physics->statistics().joints, 1u)
        << "waking the held body must restore its joint to the solve graph";

    // And the restored joint behaves exactly as an unparked one should: it settles
    // back to holding the body up rather than dropping it or leaving it displaced.
    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose held;
    ASSERT_TRUE(physics->rigid_pose(HELD, held));
    EXPECT_NEAR(double(held.position.y), 2.0, 5e-2);

    JointState state;
    ASSERT_TRUE(physics->joint_state(joint, state));
    EXPECT_NEAR(double(length(state.force)), 9.81, 1.0);
}

TEST(Integration_JointParking, EditingAParkedJointsMotorWakesItImmediately)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_park_sleeping_joints_requested(true);
    physics->set_rigid_bodies({anchor_desc(), held_desc()}, ITERATIONS, SUBSTEP_DT);
    const JointId joint = physics->create_joint(anchor_joint());
    ASSERT_NE(joint, NULL_JOINT);

    settle_and_sleep(*physics);
    ASSERT_EQ(physics->statistics().joints, 0u) << "setup failed to park the joint";

    JointMotorDesc motor;
    motor.type = JointMotorType::Velocity;
    motor.target = Scalar(0);
    motor.max_force = Scalar(10);
    ASSERT_TRUE(physics->set_joint_motor(joint, motor))
        << "editing a parked joint must succeed, not fail as if it had no live state";

    // The unpark itself is immediate — `set_joint_motor` called `add_joint` on the
    // constraint store synchronously, not on the next tick, exactly the way creating
    // a joint wakes its bodies rather than waiting for the tick to notice. What still
    // needs a tick is `PhysicsStatistics::joints`, the *solver's* own snapshot of its
    // count, refreshed once per `solver_->step()` rather than read live.
    physics->step(earth_gravity(), still_air(), SUBSTEPS);
    EXPECT_EQ(physics->statistics().joints, 1u)
        << "editing a parked joint's motor must wake it immediately";
}
