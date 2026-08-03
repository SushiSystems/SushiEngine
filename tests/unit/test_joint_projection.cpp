/**************************************************************************/
/* test_joint_projection.cpp                                              */
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

// The joint library, in isolation: the frame algebra it is built on, the seven kinds
// it registers, the limits and drives they read, and the force recovery every one of
// them folds. Host-only -- nothing here names a runtime type, so a joint can be held
// to its behaviour without standing up a device.
//
// Two things these tests are deliberately arranged to catch.
//
// The **sign conventions**. Every row in `joint_primitives.hpp` takes a world-space
// violation vector and treats it as a constraint function whose gradient acts along
// +v on body b and -v on body a. Get that backwards anywhere and the row amplifies
// what it was meant to remove, which is not subtle for long -- so each kind is given
// a violated configuration and required to reduce it.
//
// The **registration list**. `JointKinds` is the one place a kind is declared to
// exist, so a static assertion holds its length against `JOINT_KIND_COUNT`: adding an
// enumerator without registering its traits fails the build rather than silently
// projecting nothing.

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraints/joint_projection.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief How many kinds the registration list holds. */
    template <typename... Kinds>
    constexpr std::size_t registered_kind_count(JointKindList<Kinds...>) noexcept
    {
        return sizeof...(Kinds);
    }

    // The Open/Closed guard: a kind added to `JointKind` and forgotten in
    // `JointKinds` would be a joint the solver silently declines to project.
    static_assert(registered_kind_count(JointKinds{}) == JOINT_KIND_COUNT,
                  "every JointKind must have registered traits in JointKinds");

    /** @brief The budget every scene here runs under. */
    PhysicsConfiguration joint_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 32;
        configuration.capacities.constraints = 128;
        configuration.capacities.joints = 128;
        configuration.capacities.contacts = 128;
        configuration.capacities.colors = 8;
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 16;
        return configuration;
    }

    /** @brief An immovable anchor: a body with no inverse mass and no inverse inertia. */
    RigidBody anchor_body(const Vector3& position)
    {
        RigidBody body;
        body.position = position;
        return body;
    }

    /** @brief A body of @p mass with isotropic inertia @p inertia. */
    RigidBody moving_body(const Vector3& position, Scalar mass, Scalar inertia)
    {
        RigidBody body;
        body.position = position;
        body.inv_mass = Scalar(1) / mass;
        body.inv_inertia =
            Vector3{Scalar(1) / inertia, Scalar(1) / inertia, Scalar(1) / inertia};
        return body;
    }

    /** @brief A joint of @p kind between two slots, with both frames built from @p axis. */
    JointConstraint make_joint(JointKind kind, std::size_t a, std::size_t b,
                               const Vector3& axis = Vector3{1, 0, 0})
    {
        JointConstraint joint;
        joint.a = std::uint32_t(a);
        joint.b = std::uint32_t(b);
        joint.kind = kind;
        joint.local_basis_a = joint_frame_from_axis(axis);
        joint.local_basis_b = joint.local_basis_a;
        return joint;
    }

    /** @brief An enabled limit over `[lower, upper]`. */
    JointLimit limit_of(Scalar lower, Scalar upper)
    {
        JointLimit limit;
        limit.enabled = true;
        limit.lower = lower;
        limit.upper = upper;
        return limit;
    }

    /** @brief The hinge angle a joint's two bodies currently hold, in radians. */
    Scalar twist_of(const IConstraintSolver<Scalar>& solver, JointHandle handle)
    {
        JointConstraint joint;
        if (!solver.read_joint(handle, joint))
            return Scalar(0);
        RigidBody a;
        RigidBody b;
        solver.read_bodies(joint.a, 1, &a);
        solver.read_bodies(joint.b, 1, &b);
        return joint_twist_angle(joint_relative_rotation(resolve_joint_frames(joint, a, b)));
    }

    /** @brief The swing angle a joint's two bodies currently hold, in radians. */
    Scalar swing_of(const IConstraintSolver<Scalar>& solver, JointHandle handle)
    {
        JointConstraint joint;
        if (!solver.read_joint(handle, joint))
            return Scalar(0);
        RigidBody a;
        RigidBody b;
        solver.read_bodies(joint.a, 1, &a);
        solver.read_bodies(joint.b, 1, &b);
        const JointAngularState<Scalar> angular =
            resolve_joint_angular_state(resolve_joint_frames(joint, a, b));
        return length(joint_rotation_vector(angular.swing));
    }

    /** @brief The world gap between a joint's two attachment points. */
    Scalar anchor_gap(const IConstraintSolver<Scalar>& solver, JointHandle handle)
    {
        JointConstraint joint;
        if (!solver.read_joint(handle, joint))
            return Scalar(-1);
        RigidBody a;
        RigidBody b;
        solver.read_bodies(joint.a, 1, &a);
        solver.read_bodies(joint.b, 1, &b);
        const JointWorldFrames<Scalar> frames = resolve_joint_frames(joint, a, b);
        return length(frames.point_b - frames.point_a);
    }

    /** @brief Advances @p solver @p ticks times under @p gravity. */
    void run(IConstraintSolver<Scalar>& solver, int ticks, const Vector3& gravity)
    {
        StepParameters<Scalar> parameters;
        parameters.gravity = gravity;
        for (int tick = 0; tick < ticks; ++tick)
            solver.step(parameters);
    }

    const Vector3 EARTH{0, Scalar(-9.81), 0};
    const Vector3 WEIGHTLESS{0, 0, 0};
}

// -- The frame algebra the whole library rests on ---------------------------

TEST(Unit_JointProjection, FrameFromAxisPutsTheFrameXOnTheAxis)
{
    const Vector3 axes[] = {Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1},
                            normalize(Vector3{1, Scalar(2), Scalar(-3)})};
    for (const Vector3& axis : axes)
    {
        const Quaternion frame = joint_frame_from_axis(axis);
        const Vector3 mapped = rotate(frame, Vector3{1, 0, 0});
        EXPECT_NEAR(double(mapped.x), double(axis.x), 1e-12);
        EXPECT_NEAR(double(mapped.y), double(axis.y), 1e-12);
        EXPECT_NEAR(double(mapped.z), double(axis.z), 1e-12);
    }
}

TEST(Unit_JointProjection, FrameFromAxisHandlesTheAntiparallelCase)
{
    // The cross product vanishes when the axis is exactly -x, so the shortest
    // rotation's axis is undetermined and any perpendicular one is as short as any
    // other. What must not happen is normalizing a zero and producing a direction out
    // of nothing.
    const Quaternion frame = joint_frame_from_axis(Vector3{-1, 0, 0});
    const Vector3 mapped = rotate(frame, Vector3{1, 0, 0});
    EXPECT_NEAR(double(mapped.x), -1.0, 1e-12);
    EXPECT_NEAR(double(mapped.y), 0.0, 1e-12);
    EXPECT_NEAR(double(mapped.z), 0.0, 1e-12);
}

TEST(Unit_JointProjection, RotationVectorIsExactAtLargeAngles)
{
    // The small-angle form `2 vec(q)` under-reports a large misalignment by exactly
    // the factor that leaves a joint never converging, so the exact logarithm is used.
    const double angles[] = {0.01, PI / 4.0, PI / 2.0, 2.0, 3.0};
    for (const double angle : angles)
    {
        const Quaternion q = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(angle));
        const Vector3 v = joint_rotation_vector(q);
        EXPECT_NEAR(double(length(v)), angle, 1e-12);
        EXPECT_NEAR(double(v.z), angle, 1e-12);
    }
    EXPECT_NEAR(double(length(joint_rotation_vector(Quaternion{}))), 0.0, 1e-15);
}

TEST(Unit_JointProjection, RotationVectorTakesTheShorterPath)
{
    // A quaternion and its negation are the same rotation. Taking the long way round
    // a full turn is not a correction any joint means.
    const Quaternion q = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(0.5));
    const Quaternion negated{-q.x, -q.y, -q.z, -q.w};
    const Vector3 direct = joint_rotation_vector(q);
    const Vector3 flipped = joint_rotation_vector(negated);
    EXPECT_NEAR(double(flipped.z), double(direct.z), 1e-12);
}

TEST(Unit_JointProjection, SwingTwistSeparatesTheTwoPureRotations)
{
    Quaternion swing;
    Quaternion twist;

    // Pure rotation about the primary axis is all twist.
    joint_swing_twist(quaternion_axis_angle(Vector3{1, 0, 0}, Scalar(0.7)), swing, twist);
    EXPECT_NEAR(double(length(joint_rotation_vector(swing))), 0.0, 1e-12);
    EXPECT_NEAR(double(length(joint_rotation_vector(twist))), 0.7, 1e-12);

    // Pure rotation off it is all swing.
    joint_swing_twist(quaternion_axis_angle(Vector3{0, 1, 0}, Scalar(0.4)), swing, twist);
    EXPECT_NEAR(double(length(joint_rotation_vector(twist))), 0.0, 1e-12);
    EXPECT_NEAR(double(length(joint_rotation_vector(swing))), 0.4, 1e-12);
}

TEST(Unit_JointProjection, TwistAngleIsSignedAndAgreesWithItsRotation)
{
    for (const double angle : {-1.2, -0.3, 0.0, 0.3, 1.2})
    {
        const Quaternion q = quaternion_axis_angle(Vector3{1, 0, 0}, Scalar(angle));
        EXPECT_NEAR(double(joint_twist_angle(q)), angle, 1e-12);
    }
}

TEST(Unit_JointProjection, LimitViolationIsZeroInsideAndSignedOutside)
{
    const JointLimit limit = limit_of(Scalar(-1), Scalar(2));
    EXPECT_EQ(double(joint_limit_violation(limit, Scalar(0))), 0.0);
    EXPECT_EQ(double(joint_limit_violation(limit, Scalar(-1))), 0.0);
    EXPECT_EQ(double(joint_limit_violation(limit, Scalar(2))), 0.0);
    EXPECT_NEAR(double(joint_limit_violation(limit, Scalar(3))), 1.0, 1e-15);
    EXPECT_NEAR(double(joint_limit_violation(limit, Scalar(-2))), -1.0, 1e-15);

    // A disabled limit is free, and an inverted range is rejected rather than read as
    // a lock: an empty range is an authoring mistake, and locking is what
    // `lower == upper` already says.
    JointLimit disabled = limit;
    disabled.enabled = false;
    EXPECT_EQ(double(joint_limit_violation(disabled, Scalar(100))), 0.0);
    EXPECT_EQ(double(joint_limit_violation(limit_of(Scalar(2), Scalar(-1)), Scalar(0))), 0.0);
}

// -- The kinds -------------------------------------------------------------

TEST(Unit_JointProjection, BallJointHoldsTheAnchorAndLeavesRotationFree)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    RigidBody spinning = moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(10), Scalar(1));
    spinning.angular_velocity = Vector3{Scalar(2), 0, 0};
    const BodyHandle b = solver.add_body(spinning);

    JointConstraint joint = make_joint(JointKind::Ball, solver.body_slot(a), solver.body_slot(b));
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    const JointHandle handle = solver.add_joint(joint);
    ASSERT_TRUE(handle.valid());

    run(solver, 120, EARTH);

    // Attached: the anchors stay together while the body swings like a pendulum.
    EXPECT_LT(double(anchor_gap(solver, handle)), 1e-3);

    // Free: the spin about the anchor axis survives, so the joint removed three
    // translational degrees and no rotational ones.
    RigidBody solved;
    ASSERT_TRUE(solver.read_body(b, solved));
    EXPECT_GT(double(std::abs(solved.angular_velocity.x)), 1.0);
}

TEST(Unit_JointProjection, FixedJointAlignsBothTheFramesAndTheAnchors)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    RigidBody tilted = moving_body(Vector3{Scalar(1), 0, 0}, Scalar(10), Scalar(1));
    tilted.orientation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(0.9));
    const BodyHandle b = solver.add_body(tilted);

    JointConstraint joint = make_joint(JointKind::Fixed, solver.body_slot(a), solver.body_slot(b));
    joint.local_anchor_a = Vector3{Scalar(1), 0, 0};
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 200, WEIGHTLESS);

    EXPECT_LT(double(anchor_gap(solver, handle)), 1e-6);
    EXPECT_LT(double(swing_of(solver, handle)), 1e-6);
    EXPECT_NEAR(double(twist_of(solver, handle)), 0.0, 1e-6);
}

TEST(Unit_JointProjection, HingeRemovesTheSwingAndKeepsTheTwist)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    RigidBody door = moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(35), Scalar(3));
    // Spun about the hinge axis and shoved off it: only the first should survive.
    door.angular_velocity = Vector3{0, Scalar(2), Scalar(2)};
    const BodyHandle b = solver.add_body(door);

    JointConstraint joint =
        make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 120, WEIGHTLESS);

    EXPECT_LT(double(swing_of(solver, handle)), 1e-3);
    EXPECT_LT(double(anchor_gap(solver, handle)), 1e-4);

    RigidBody solved;
    ASSERT_TRUE(solver.read_body(b, solved));
    // The off-axis spin is gone; the on-axis one persists. It is not the value it
    // started at, and it should not be: a body pinned off its centre of mass must
    // orbit the pin, and angular momentum about the pin is shared between the spin and
    // the orbit. What matters is that the axis survived at all.
    EXPECT_GT(double(std::abs(solved.angular_velocity.y)), 0.3);
    EXPECT_LT(double(std::abs(solved.angular_velocity.z)), 0.05);
}

TEST(Unit_JointProjection, HingeTwistLimitBoundsTheAngleFromBothSides)
{
    const Scalar upper = Scalar(68.0 * PI / 180.0);

    for (const Scalar spin : {Scalar(3), Scalar(-3)})
    {
        HostXPBDSolver<Scalar> solver(joint_scene());
        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        RigidBody door = moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(35), Scalar(3));
        door.angular_velocity = Vector3{0, spin, 0};
        const BodyHandle b = solver.add_body(door);

        JointConstraint joint = make_joint(JointKind::Hinge, solver.body_slot(a),
                                          solver.body_slot(b), Vector3{0, 1, 0});
        joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
        joint.twist_limit = limit_of(Scalar(0), upper);
        const JointHandle handle = solver.add_joint(joint);

        Scalar highest = Scalar(0);
        Scalar lowest = Scalar(0);
        StepParameters<Scalar> parameters;
        parameters.gravity = WEIGHTLESS;
        for (int tick = 0; tick < 240; ++tick)
        {
            solver.step(parameters);
            const Scalar angle = twist_of(solver, handle);
            highest = angle > highest ? angle : highest;
            lowest = angle < lowest ? angle : lowest;
        }

        EXPECT_LT(double(highest), double(upper) + 1e-2);
        EXPECT_GT(double(lowest), -1e-2);
    }
}

TEST(Unit_JointProjection, HingePositionMotorDrivesTheDoorToItsTarget)
{
    const Scalar target = Scalar(45.0 * PI / 180.0);

    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(35), Scalar(3)));

    JointConstraint joint =
        make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    joint.motor.mode = JointMotorMode::Position;
    joint.motor.target = target;
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 240, WEIGHTLESS);

    EXPECT_NEAR(double(twist_of(solver, handle)), double(target), 1e-2);
}

TEST(Unit_JointProjection, HingeVelocityMotorWithZeroTargetIsFriction)
{
    // The same scene twice, differing only in whether the hinge has friction. The free
    // one keeps spinning; the seized one stops. This is how sec. 10.2's "it does not
    // swing free" is expressed: a rate drive toward standstill with a small limit.
    Scalar free_spin = 0;
    Scalar damped_spin = 0;

    for (int with_friction = 0; with_friction < 2; ++with_friction)
    {
        HostXPBDSolver<Scalar> solver(joint_scene());
        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        RigidBody door = moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(35), Scalar(3));
        door.angular_velocity = Vector3{0, Scalar(2), 0};
        const BodyHandle b = solver.add_body(door);

        JointConstraint joint = make_joint(JointKind::Hinge, solver.body_slot(a),
                                          solver.body_slot(b), Vector3{0, 1, 0});
        joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
        if (with_friction != 0)
        {
            joint.motor.mode = JointMotorMode::Velocity;
            joint.motor.target = 0;
            joint.motor.max_force = Scalar(4);
        }
        solver.add_joint(joint);

        run(solver, 120, WEIGHTLESS);

        RigidBody solved;
        ASSERT_TRUE(solver.read_body(b, solved));
        if (with_friction != 0)
            damped_spin = solved.angular_velocity.y;
        else
            free_spin = solved.angular_velocity.y;
    }

    EXPECT_GT(double(std::abs(free_spin)), 0.3);
    EXPECT_LT(double(std::abs(damped_spin)), 1e-3);
}

TEST(Unit_JointProjection, SliderTravelsAlongItsAxisAndNowhereElse)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b = solver.add_body(moving_body(Vector3{0, 0, 0}, Scalar(20), Scalar(2)));

    JointConstraint joint =
        make_joint(JointKind::Slider, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.linear_limit = limit_of(Scalar(-0.3), Scalar(0));
    solver.add_joint(joint);

    run(solver, 300, EARTH);

    RigidBody solved;
    ASSERT_TRUE(solver.read_body(b, solved));
    // Gravity drives it down the rail until the travel limit stops it, and the two
    // perpendicular axes are held at zero throughout.
    EXPECT_NEAR(double(solved.position.y), -0.3, 1e-3);
    EXPECT_NEAR(double(solved.position.x), 0.0, 1e-6);
    EXPECT_NEAR(double(solved.position.z), 0.0, 1e-6);
}

TEST(Unit_JointProjection, DistanceJointIsSlackInsideItsRangeAndTautAtTheEnd)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{0, Scalar(-0.5), 0}, Scalar(5), Scalar(1)));

    JointConstraint joint =
        make_joint(JointKind::Distance, solver.body_slot(a), solver.body_slot(b));
    joint.linear_limit = limit_of(Scalar(0), Scalar(2));
    const JointHandle handle = solver.add_joint(joint);

    // One tick: still slack, so the rope has done nothing and the body is in free fall.
    run(solver, 1, EARTH);
    RigidBody early;
    ASSERT_TRUE(solver.read_body(b, early));
    EXPECT_LT(double(early.position.y), -0.5);
    EXPECT_GT(double(early.position.y), -0.6);
    JointConstraint slack;
    ASSERT_TRUE(solver.read_joint(handle, slack));
    EXPECT_NEAR(double(length(joint_force(slack))), 0.0, 1e-9);

    run(solver, 400, EARTH);
    RigidBody settled;
    ASSERT_TRUE(solver.read_body(b, settled));
    EXPECT_NEAR(double(settled.position.y), -2.0, 1e-3);
}

TEST(Unit_JointProjection, ConeTwistBoundsTheSwingCone)
{
    const Scalar cone = Scalar(20.0 * PI / 180.0);

    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    RigidBody limb = moving_body(Vector3{0, Scalar(-0.5), 0}, Scalar(8), Scalar(1));
    const BodyHandle b = solver.add_body(limb);

    JointConstraint joint = make_joint(JointKind::ConeTwist, solver.body_slot(a),
                                       solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_b = Vector3{0, Scalar(0.5), 0};
    JointLimit swing;
    swing.enabled = true;
    swing.upper = cone;
    joint.swing_limit = swing;
    const JointHandle handle = solver.add_joint(joint);

    // A sideways field, so the limb is pushed hard against the cone rather than
    // hanging inside it.
    run(solver, 400, Vector3{Scalar(9.81), Scalar(-9.81), 0});

    EXPECT_LT(double(swing_of(solver, handle)), double(cone) + 2e-2);
    EXPECT_LT(double(anchor_gap(solver, handle)), 1e-3);
}

TEST(Unit_JointProjection, SixDegreeOfFreedomHonoursEachAxisSeparately)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b = solver.add_body(moving_body(Vector3{0, 0, 0}, Scalar(10), Scalar(1)));

    // The frame's x is world x. Lock x, leave y free, bound z.
    JointConstraint joint = make_joint(JointKind::SixDegreeOfFreedom, solver.body_slot(a),
                                       solver.body_slot(b), Vector3{1, 0, 0});
    joint.linear_limit = limit_of(Scalar(0), Scalar(0));
    joint.linear_limit_z = limit_of(Scalar(-0.25), Scalar(0.25));
    const JointHandle handle = solver.add_joint(joint);
    ASSERT_TRUE(handle.valid());

    // Pushed diagonally so every axis is asked the question at once.
    run(solver, 400, Vector3{Scalar(9.81), Scalar(-9.81), Scalar(9.81)});

    RigidBody solved;
    ASSERT_TRUE(solver.read_body(b, solved));
    EXPECT_NEAR(double(solved.position.x), 0.0, 1e-4);   // locked
    EXPECT_LT(double(solved.position.y), -1.0);          // free: it fell away
    EXPECT_NEAR(double(solved.position.z), 0.25, 1e-3);  // bounded
}

// -- Force recovery, drives, and lifetime ----------------------------------

TEST(Unit_JointProjection, ForceRecoveryReportsTheLoadTheJointActuallyCarries)
{
    // Sec. 10.4's claim, tested against statics rather than against itself. A door of
    // mass m hung from a hinge and at rest: the only external force is gravity, the
    // angular row carries pure torque, so the hinge's reaction must be exactly mg and
    // its torque exactly mg times the lever arm.
    const Scalar mass = Scalar(35);
    const Scalar lever = Scalar(0.5);

    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b = solver.add_body(moving_body(Vector3{lever, 0, 0}, mass, Scalar(3)));

    JointConstraint joint =
        make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_b = Vector3{-lever, 0, 0};
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 120, EARTH);

    JointConstraint solved;
    ASSERT_TRUE(solver.read_joint(handle, solved));
    EXPECT_GT(solved.force_samples, 0u);
    EXPECT_NEAR(double(length(joint_force(solved))), 35.0 * 9.81, 1.0);
    EXPECT_NEAR(double(length(joint_torque(solved))), 35.0 * 9.81 * 0.5, 1.0);
}

TEST(Unit_JointProjection, ABreakThresholdIsMeasuredAgainstThePeakAndNotTheMean)
{
    JointConstraint joint;
    joint.force_samples = 4;
    joint.force_sum = Vector3{Scalar(400), 0, 0};   // mean 100 N
    joint.torque_sum = Vector3{0, Scalar(40), 0};   // mean 10 N.m
    joint.peak_force = Scalar(100);
    joint.peak_torque = Scalar(10);

    joint.break_force = Scalar(120);
    joint.break_torque = 0;
    EXPECT_FALSE(joint_should_break(joint));

    joint.break_force = Scalar(80);
    EXPECT_TRUE(joint_should_break(joint));

    joint.break_force = 0;
    joint.break_torque = Scalar(8);
    EXPECT_TRUE(joint_should_break(joint));

    // Zero is unbreakable, not "breaks immediately", and a joint that has never been
    // stepped reports no load rather than dividing by no samples.
    joint.break_force = 0;
    joint.break_torque = 0;
    EXPECT_FALSE(joint_should_break(joint));
    joint.force_samples = 0;
    joint.break_force = Scalar(1);
    EXPECT_FALSE(joint_should_break(joint));
    EXPECT_EQ(double(length(joint_force(joint))), 0.0);

    // The case that decided which of the two the threshold reads. An impact is a large
    // correction followed by an almost equally large one the other way, so the vector
    // sums cancel and the *mean* reports a joint at rest while the substeps either side
    // of the hit carried meganewtons. A threshold on the mean never fires.
    JointConstraint slammed;
    slammed.force_samples = 8;
    slammed.force_sum = Vector3{Scalar(340), 0, 0};       // mean 42.5 N: nothing happened
    slammed.peak_force = Scalar(1.6e7);                   // and yet
    slammed.break_force = Scalar(12000);
    EXPECT_TRUE(joint_should_break(slammed))
        << "an impact whose net load cancels must still break the mount";
}

TEST(Unit_JointProjection, AnImpactIsReportedAsAPeakTheMeanCannotSee)
{
    // The same claim through the solver rather than by construction: a joint yanked far
    // out of place and snapped back reports a mean near its resting load and a peak
    // orders of magnitude above it.
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, Scalar(2), 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{Scalar(1.5), Scalar(2), 0}, Scalar(35), Scalar(3)));

    JointConstraint joint =
        make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_a = Vector3{Scalar(1), 0, 0};
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 60, EARTH);
    JointConstraint resting;
    ASSERT_TRUE(solver.read_joint(handle, resting));
    // At rest the two agree to about a newton in three hundred, which is why a
    // peak-based threshold does not spuriously break a settled joint.
    EXPECT_NEAR(double(resting.peak_force), double(length(joint_force(resting))), 5.0);

    RigidBody door;
    ASSERT_TRUE(solver.read_body(b, door));
    door.position = Vector3{Scalar(3.5), Scalar(2), 0};
    door.previous_position = door.position;
    door.velocity = Vector3{0, 0, 0};
    ASSERT_TRUE(solver.write_body(b, door));

    run(solver, 1, EARTH);
    JointConstraint slammed;
    ASSERT_TRUE(solver.read_joint(handle, slammed));
    EXPECT_GT(double(slammed.peak_force), 1e5) << "the impact left no peak";
    EXPECT_LT(double(length(joint_force(slammed))), 1e4)
        << "the mean was expected to cancel; if it did not, the peak is not load-bearing";
}

TEST(Unit_JointProjection, AMotorForceLimitCapsWhatTheDriveCanSpend)
{
    // Two identical scenes; the drive in one is saturated far below the load it would
    // need. It must fail to reach its target, and the unsaturated one must reach it.
    const Scalar target = Scalar(1.0);
    Scalar reached_limited = 0;
    Scalar reached_ideal = 0;

    for (int limited = 0; limited < 2; ++limited)
    {
        HostXPBDSolver<Scalar> solver(joint_scene());
        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        const BodyHandle b =
            solver.add_body(moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(200), Scalar(60)));

        JointConstraint joint = make_joint(JointKind::Hinge, solver.body_slot(a),
                                          solver.body_slot(b), Vector3{0, 1, 0});
        joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
        joint.motor.mode = JointMotorMode::Position;
        joint.motor.target = target;
        joint.motor.max_force = limited != 0 ? Scalar(0.05) : Scalar(0);
        const JointHandle handle = solver.add_joint(joint);

        run(solver, 60, WEIGHTLESS);

        if (limited != 0)
            reached_limited = twist_of(solver, handle);
        else
            reached_ideal = twist_of(solver, handle);
    }

    EXPECT_NEAR(double(reached_ideal), double(target), 1e-2);
    EXPECT_LT(double(reached_limited), 0.5 * double(target));
}

TEST(Unit_JointProjection, ADisabledJointIsNotProjectedAtAll)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(10), Scalar(1)));

    JointConstraint joint = make_joint(JointKind::Ball, solver.body_slot(a), solver.body_slot(b));
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    joint.flags = JointFlags::none;  // authored, kept, and silent
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 60, EARTH);

    RigidBody solved;
    ASSERT_TRUE(solver.read_body(b, solved));
    // Free fall: 60 ticks at 1/60 s each under 9.81 m/s^2 lands near -4.9 m. The
    // constraint never ran, so the body is not held anywhere.
    EXPECT_LT(double(solved.position.y), -4.0);
    // And it accumulated no load, because a joint that projected nothing carried
    // nothing -- the sample count is what says so.
    JointConstraint read;
    ASSERT_TRUE(solver.read_joint(handle, read));
    EXPECT_EQ(read.force_samples, 0u);
}

// -- Lifetime through the solver seam --------------------------------------

TEST(Unit_JointProjection, RemovingABodyRemovesTheJointsThatNamedIt)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{Scalar(1), 0, 0}, Scalar(10), Scalar(1)));
    const BodyHandle c =
        solver.add_body(moving_body(Vector3{Scalar(2), 0, 0}, Scalar(10), Scalar(1)));

    const JointHandle ab =
        solver.add_joint(make_joint(JointKind::Ball, solver.body_slot(a), solver.body_slot(b)));
    const JointHandle bc =
        solver.add_joint(make_joint(JointKind::Ball, solver.body_slot(b), solver.body_slot(c)));
    ASSERT_TRUE(ab.valid());
    ASSERT_TRUE(bc.valid());

    solver.step(StepParameters<Scalar>{});
    EXPECT_EQ(solver.statistics().joints, std::size_t(2));

    // Body b is named by both joints, so both go with it. A joint left naming a freed
    // slot would act on whichever body claimed the slot next.
    ASSERT_TRUE(solver.remove_body(b));
    solver.step(StepParameters<Scalar>{});
    EXPECT_EQ(solver.statistics().joints, std::size_t(0));
    JointConstraint read;
    EXPECT_FALSE(solver.read_joint(ab, read));
    EXPECT_FALSE(solver.read_joint(bc, read));
}

TEST(Unit_JointProjection, RemovingAJointKeepsTheRemainingOnesReadable)
{
    // The bands are kept dense by moving the last joint into the vacated slot, so the
    // handle of whatever moved has to keep resolving. A stale slot map is invisible
    // until a caller reads the wrong joint.
    HostXPBDSolver<Scalar> solver(joint_scene());
    std::vector<BodyHandle> bodies;
    for (int i = 0; i < 6; ++i)
        bodies.push_back(
            solver.add_body(moving_body(Vector3{Scalar(i), 0, 0}, Scalar(10), Scalar(1))));

    std::vector<JointHandle> handles;
    for (std::size_t i = 0; i + 1 < bodies.size(); ++i)
    {
        JointConstraint joint = make_joint(JointKind::Distance, solver.body_slot(bodies[i]),
                                           solver.body_slot(bodies[i + 1]));
        joint.linear_limit = limit_of(Scalar(1), Scalar(1));
        handles.push_back(solver.add_joint(joint));
        ASSERT_TRUE(handles.back().valid());
    }

    ASSERT_TRUE(solver.remove_joint(handles[1]));
    EXPECT_FALSE(solver.remove_joint(handles[1]));

    JointConstraint read;
    EXPECT_FALSE(solver.read_joint(handles[1], read));
    for (std::size_t i = 0; i < handles.size(); ++i)
    {
        if (i == 1)
            continue;
        ASSERT_TRUE(solver.read_joint(handles[i], read)) << "joint " << i << " became unreadable";
        EXPECT_EQ(read.a, std::uint32_t(solver.body_slot(bodies[i])));
        EXPECT_EQ(read.b, std::uint32_t(solver.body_slot(bodies[i + 1])));
    }
}

TEST(Unit_JointProjection, WritingAJointChangesWhatTheNextStepProjects)
{
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
    const BodyHandle b =
        solver.add_body(moving_body(Vector3{Scalar(0.5), 0, 0}, Scalar(35), Scalar(3)));

    JointConstraint joint =
        make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b), Vector3{0, 1, 0});
    joint.local_anchor_b = Vector3{Scalar(-0.5), 0, 0};
    const JointHandle handle = solver.add_joint(joint);

    run(solver, 30, WEIGHTLESS);
    EXPECT_NEAR(double(twist_of(solver, handle)), 0.0, 1e-6);

    // The whole descriptor rather than a setter per parameter: a seam with one virtual
    // per authored field grows every time a joint kind gains one.
    JointConstraint edited;
    ASSERT_TRUE(solver.read_joint(handle, edited));
    edited.motor.mode = JointMotorMode::Position;
    edited.motor.target = Scalar(0.6);
    ASSERT_TRUE(solver.write_joint(handle, edited));

    run(solver, 240, WEIGHTLESS);
    EXPECT_NEAR(double(twist_of(solver, handle)), 0.6, 1e-2);
}

TEST(Unit_JointProjection, ExhaustingTheJointBudgetIsCountedRatherThanFatal)
{
    // Every pair here is disjoint, so every joint sees every colour as free. The whole
    // budget must therefore be reachable across the bands -- placing only the first
    // band's worth and reporting the rest as overflow, with the buffer mostly empty,
    // is the failure `ConstraintStore::place` records.
    PhysicsConfiguration configuration = joint_scene();
    configuration.capacities.joints = 4;
    configuration.capacities.colors = 2;  // two bands of two
    HostXPBDSolver<Scalar> solver(configuration);

    std::vector<BodyHandle> bodies;
    for (int i = 0; i < 12; ++i)
        bodies.push_back(
            solver.add_body(moving_body(Vector3{Scalar(i), 0, 0}, Scalar(10), Scalar(1))));

    std::size_t placed = 0;
    for (std::size_t i = 0; i + 1 < bodies.size(); i += 2)
    {
        JointConstraint joint = make_joint(JointKind::Ball, solver.body_slot(bodies[i]),
                                           solver.body_slot(bodies[i + 1]));
        if (solver.add_joint(joint).valid())
            ++placed;
    }

    EXPECT_EQ(placed, solver.joint_capacity());
    EXPECT_GT(solver.statistics().capacity_overflows, std::size_t(0));
}

// -- The damper: the other half of §11.2's "spring-damper drive" ------------

TEST(Unit_JointProjection, MotorDampingBleedsOffAHingeSpin)
{
    // A drive that is not driving at all, and a joint that still slows down: a damping
    // rate with a disabled mode is a pure damper -- a steering damper, a door closer --
    // and the mode gate must not swallow it.
    Scalar remaining[2] = {0, 0};
    for (int damped = 0; damped < 2; ++damped)
    {
        HostXPBDSolver<Scalar> solver(joint_scene());
        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        const BodyHandle b =
            solver.add_body(moving_body(Vector3{0, 0, 0}, Scalar(20), Scalar(1.2)));

        JointConstraint joint =
            make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b));
        joint.motor.mode = JointMotorMode::Disabled;
        joint.motor.damping = damped != 0 ? Scalar(4) : Scalar(0);
        const JointHandle handle = solver.add_joint(joint);

        RigidBody wheel;
        ASSERT_TRUE(solver.read_body(b, wheel));
        wheel.angular_velocity = Vector3{Scalar(20), 0, 0};
        ASSERT_TRUE(solver.write_body(b, wheel));

        run(solver, 60, WEIGHTLESS);
        ASSERT_TRUE(solver.read_body(b, wheel));
        remaining[damped] = wheel.angular_velocity.x;
        EXPECT_TRUE(solver.read_joint(handle, joint));
    }

    EXPECT_NEAR(double(remaining[0]), 20.0, 1e-6) << "an undamped hinge must not slow at all";
    EXPECT_LT(double(remaining[1]), 5.0) << "a damped one must";
}

TEST(Unit_JointProjection, MotorDampingIsARateAndNotAPerSubstepFraction)
{
    // The claim `JointMotorT::damping` makes: the same damping over the same wall time
    // must remove the same motion whatever the substep schedule. A per-substep fraction
    // would damp four times as hard at four times the substeps -- and the substep count
    // is derived from scene motion (§6.2), so that error would make a suspension's
    // firmness depend on what else was moving nearby.
    Scalar remaining[2] = {0, 0};
    const std::size_t schedules[2] = {4, 16};

    for (int schedule = 0; schedule < 2; ++schedule)
    {
        PhysicsConfiguration configuration = joint_scene();
        configuration.substeps.minimum = schedules[schedule];
        configuration.substeps.maximum = schedules[schedule];
        HostXPBDSolver<Scalar> solver(configuration);

        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        const BodyHandle b =
            solver.add_body(moving_body(Vector3{0, 0, 0}, Scalar(20), Scalar(1.2)));
        JointConstraint joint =
            make_joint(JointKind::Hinge, solver.body_slot(a), solver.body_slot(b));
        joint.motor.damping = Scalar(3);
        solver.add_joint(joint);

        RigidBody wheel;
        ASSERT_TRUE(solver.read_body(b, wheel));
        wheel.angular_velocity = Vector3{Scalar(20), 0, 0};
        ASSERT_TRUE(solver.write_body(b, wheel));

        run(solver, 30, WEIGHTLESS);
        ASSERT_TRUE(solver.read_body(b, wheel));
        remaining[schedule] = wheel.angular_velocity.x;
    }

    // Not equal -- a rate integrated in four steps and in sixteen is the same
    // exponential sampled differently -- but within a tenth of what is left, which a
    // per-substep fraction would miss by a factor of four.
    EXPECT_NEAR(double(remaining[0]), double(remaining[1]), 0.1 * double(remaining[0]));
}

TEST(Unit_JointProjection, ASpringDamperStrutSettlesWhereASpringAloneRings)
{
    // §11.2's suspension row, as one joint: a slider whose position drive is the spring
    // and whose damping is the damper. The undamped strut must still be swinging when
    // the damped one has stopped.
    Scalar swing[2] = {0, 0};
    for (int damped = 0; damped < 2; ++damped)
    {
        HostXPBDSolver<Scalar> solver(joint_scene());
        const BodyHandle a = solver.add_body(anchor_body(Vector3{0, 0, 0}));
        const BodyHandle b =
            solver.add_body(moving_body(Vector3{0, Scalar(-0.4), 0}, Scalar(300), Scalar(50)));

        JointConstraint joint = make_joint(JointKind::Slider, solver.body_slot(a),
                                           solver.body_slot(b), Vector3{0, 1, 0});
        joint.motor.mode = JointMotorMode::Position;
        joint.motor.target = Scalar(-0.4);
        joint.motor.compliance = Scalar(1) / Scalar(30000);
        joint.motor.damping = damped != 0 ? Scalar(8) : Scalar(0);
        const JointHandle handle = solver.add_joint(joint);

        run(solver, 90, EARTH);
        Scalar low = Scalar(1e9);
        Scalar high = Scalar(-1e9);
        for (int i = 0; i < 90; ++i)
        {
            run(solver, 1, EARTH);
            RigidBody body;
            ASSERT_TRUE(solver.read_body(b, body));
            low = body.position.y < low ? body.position.y : low;
            high = body.position.y > high ? body.position.y : high;
        }
        swing[damped] = high - low;
        EXPECT_TRUE(solver.read_joint(handle, joint));
    }

    EXPECT_LT(double(swing[1]), 0.1 * double(swing[0]));
}

TEST(Unit_JointProjection, AFreeBodyKeepsItsSpinExactly)
{
    // Not a joint test, and here because this is where it was found. `predict` and
    // `update_velocity` are an exponential map and its logarithm, and a first-order
    // pair instead of an exact one leaks angular velocity in proportion to the spin --
    // measured at a third of a wheel's speed per second before it was fixed, on a body
    // with no constraint on it at all.
    HostXPBDSolver<Scalar> solver(joint_scene());
    const BodyHandle b = solver.add_body(moving_body(Vector3{0, 0, 0}, Scalar(20), Scalar(1.2)));

    RigidBody body;
    ASSERT_TRUE(solver.read_body(b, body));
    body.angular_velocity = Vector3{Scalar(50), 0, 0};
    ASSERT_TRUE(solver.write_body(b, body));

    run(solver, 120, WEIGHTLESS);
    ASSERT_TRUE(solver.read_body(b, body));
    EXPECT_NEAR(double(body.angular_velocity.x), 50.0, 1e-9);
    EXPECT_NEAR(double(body.angular_velocity.y), 0.0, 1e-9);
}
