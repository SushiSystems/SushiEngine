/**************************************************************************/
/* test_vehicle_tyre.cpp                                                  */
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
 * @file test_vehicle_tyre.cpp
 * @brief §11.5's patch, held to what a tyre may not do.
 *
 * The model half needs no solver: slip and load in, force out, and every assertion is a
 * closed form. The world half needs a wheel standing on something, and there the
 * questions are different — is the load the one the solver actually recovered, does the
 * force land at the patch, and does the ground take the other end of it.
 *
 * **The ground here is a hand-built contact rather than a narrowphase.** These tests are
 * about the tyre, and a manifold built in six lines is a ground whose normal, patch and
 * load are known to the assertion rather than inferred from it. The contact carries a
 * rest offset for a real reason: a manifold anchors a contact to a *material point* on
 * the wheel, so a spinning wheel carries that point round the rim and the bottom of the
 * wheel stops being where the contact is. That is a property of §7.3's anchor-based
 * refresh and not of the tyre, and the offset is the tolerance that absorbs it.
 *
 * The wheels are given no Coulomb friction, which is what a real vehicle must also do:
 * the solver's own friction runs on the same contact, and a wheel with both has grip
 * nobody authored.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/vehicle/tyre_projection.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar RADIUS = Scalar(0.34);
    constexpr Scalar WHEEL_MASS = Scalar(20);
    constexpr Scalar GRAVITY = Scalar(9.81);
    constexpr Scalar TICK = Scalar(1) / Scalar(60);
    constexpr Scalar RATED_LOAD = Scalar(4000);

    /** @brief The tolerance that absorbs a spinning wheel's anchor drift; see the file note. */
    constexpr Scalar SKIN = Scalar(0.02);

    /** @brief Slow enough that the anchor stays well inside @ref SKIN over one tick. */
    constexpr Scalar DRIVE_SPIN = Scalar(6);

    TyreSettings road_tyre()
    {
        TyreSettings tyre;
        tyre.friction = Scalar(1.2);
        tyre.longitudinal_stiffness = 15;
        tyre.lateral_stiffness = 12;
        tyre.rated_load = RATED_LOAD;
        tyre.load_sensitivity = Scalar(0.15);
        tyre.low_speed_reference = 2;
        return tyre;
    }

    PhysicsConfiguration wheel_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 16;
        configuration.capacities.constraints = 32;
        configuration.capacities.contacts = 32;
        configuration.capacities.joints = 16;
        configuration.capacities.beams = 16;
        configuration.capacities.colors = 4;
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 8;
        // A lone wheel on the spot is exactly what the sleeping heuristic is for, and a
        // sleeping body takes no impulse at all. A whole car keeps its own wheels awake.
        configuration.sleep_motion_threshold = 0;
        return configuration;
    }

    BodyHandle spawn_wheel(HostXPBDSolver<Scalar>& solver)
    {
        RigidBody wheel;
        wheel.position = Vector3{0, RADIUS + SKIN, 0};
        wheel.prev_position = wheel.position;
        wheel.orientation = Quaternion{0, 0, 0, 1};
        wheel.prev_orientation = wheel.orientation;
        wheel.inv_mass = 1 / WHEEL_MASS;
        const Scalar axial = Scalar(0.5) * WHEEL_MASS * RADIUS * RADIUS;
        const Scalar transverse =
            WHEEL_MASS * (3 * RADIUS * RADIUS + Scalar(0.22) * Scalar(0.22)) / 12;
        wheel.inv_inertia = Vector3{1 / axial, 1 / transverse, 1 / transverse};
        wheel.flags = BodyFlags::dynamic_body;
        return solver.add_body(wheel);
    }

    /** @brief One tick: a hand-built ground contact, the tyre, then the solve. */
    TyreReportT<Scalar> roll(HostXPBDSolver<Scalar>& solver, BodyHandle wheel,
                             const TyreSettings& tyre, ContactManifold<Scalar>& carried,
                             BodyHandle ground = BodyHandle{})
    {
        RigidBody body;
        solver.read_body(wheel, body);

        ContactManifold<Scalar> manifold;
        // The normal points away from body a, and the wheel is body a — so for a wheel
        // standing on the road it aims down into the road.
        manifold.normal = Vector3{0, -1, 0};
        manifold.point_count = 1;
        manifold.points[0].anchor_a_local =
            rotate(conjugate(body.orientation), Vector3{0, -RADIUS, 0});
        const Vector3 patch{body.position.x, 0, body.position.z};
        manifold.points[0].separation = body.position.y - RADIUS;
        if (ground.valid())
        {
            RigidBody floor;
            solver.read_body(ground, floor);
            manifold.points[0].anchor_b_local =
                to_local_anchor(floor.position, floor.orientation, patch);
        }
        else
        {
            manifold.points[0].anchor_b_local = patch;
        }
        warm_start_manifold(manifold, carried);

        ContactConstraint contact;
        contact.a = std::uint32_t(solver.body_slot(wheel));
        contact.b =
            ground.valid() ? std::uint32_t(solver.body_slot(ground)) : null_contact_body;
        contact.key = 1;
        contact.manifold = manifold;
        contact.params.static_friction = 0;   // the tyre is the only tangential source
        contact.params.dynamic_friction = 0;
        contact.params.restitution = 0;
        contact.params.rest_offset = SKIN;
        solver.begin_contacts();
        solver.add_contact(contact);

        const TyreReportT<Scalar> report =
            apply_tyre_force(solver, wheel, tyre, Vector3{1, 0, 0}, TICK, TICK / 8);

        StepParameters<Scalar> parameters;
        parameters.delta_time = TICK;
        parameters.gravity = Vector3{0, -GRAVITY, 0};
        solver.step(parameters);

        ContactConstraint solved;
        if (solver.read_contact(0, solved))
            carried = solved.manifold;
        return report;
    }

    /** @brief Holds a wheel's spin against whatever the tyre takes out of it. */
    void hold_spin(HostXPBDSolver<Scalar>& solver, BodyHandle wheel, Scalar rate)
    {
        RigidBody body;
        solver.read_body(wheel, body);
        body.angular_velocity = Vector3{rate, 0, 0};
        solver.write_body(wheel, body);
    }
} // namespace

/** @brief Below the limit the force is the stiffness times the slip, and nothing else. */
TEST(Unit_VehicleTyre, SmallSlipReadsTheStiffnessStraightOff)
{
    const TyreSettings tyre = road_tyre();
    TyreSlip slip;
    slip.longitudinal = Scalar(0.001);

    const TyreForce force = tyre_force(tyre, slip, RATED_LOAD);
    const double expected =
        double(tyre.longitudinal_stiffness * RATED_LOAD * slip.longitudinal);
    EXPECT_NEAR(double(force.longitudinal), expected, expected * 0.01);
    EXPECT_EQ(double(force.lateral), 0.0) << "pure longitudinal slip is pure longitudinal force";
}

/** @brief No slip, however large, takes a tyre past the friction the surface has. */
TEST(Unit_VehicleTyre, NoSlipTakesATyrePastItsFrictionLimit)
{
    const TyreSettings tyre = road_tyre();
    const Scalar limit = tyre_friction(tyre, RATED_LOAD) * RATED_LOAD;

    for (Scalar s = Scalar(0.01); s < 5; s *= Scalar(1.5))
    {
        TyreSlip slip;
        slip.longitudinal = s;
        ASSERT_LE(double(tyre_force(tyre, slip, RATED_LOAD).longitudinal), double(limit) + 1e-9)
            << "at slip " << double(s);
    }

    TyreSlip sliding;
    sliding.longitudinal = 100;
    const TyreForce force = tyre_force(tyre, sliding, RATED_LOAD);
    EXPECT_NEAR(double(force.longitudinal), double(limit), 1e-9);
    EXPECT_EQ(double(force.saturation), 1.0);
}

/**
 * @brief The brush curve rises monotonically and arrives at its peak without a kink.
 *
 * The derivative of the cubic is `(1 - θ)²`, which is zero exactly at the peak. A kink
 * there is a discontinuity a car crosses several times a second under hard driving, and
 * it is felt as a tyre that grips and lets go rather than one that gives way.
 */
TEST(Unit_VehicleTyre, ThePeakIsReachedSmoothly)
{
    const TyreSettings tyre = road_tyre();
    Scalar previous = 0;
    Scalar worst_step = 0;
    for (int i = 1; i <= 2000; ++i)
    {
        TyreSlip slip;
        slip.longitudinal = Scalar(i) * Scalar(0.0005);
        const Scalar force = tyre_force(tyre, slip, RATED_LOAD).longitudinal;
        ASSERT_GE(double(force), double(previous) - 1e-9) << "fell back at step " << i;
        if (i > 1 && force - previous > worst_step)
            worst_step = force - previous;
        previous = force;
    }
    EXPECT_LT(double(worst_step), 40.0) << "a kink would show as one large step";
}

/**
 * @brief Two saturated axes together still carry only `μN`.
 *
 * The friction *circle*. Saturating the two axes separately would give 1.41 times the
 * friction the surface has, which is the classic combined-slip mistake.
 */
TEST(Unit_VehicleTyre, CombinedSlipIsACircleAndNotASquare)
{
    const TyreSettings tyre = road_tyre();
    const Scalar limit = tyre_friction(tyre, RATED_LOAD) * RATED_LOAD;

    TyreSlip both;
    both.longitudinal = 50;
    both.lateral = 50;
    const TyreForce force = tyre_force(tyre, both, RATED_LOAD);
    const double magnitude =
        std::sqrt(double(force.longitudinal * force.longitudinal +
                         force.lateral * force.lateral));
    EXPECT_NEAR(magnitude, double(limit), 1e-9);
}

/** @brief A tyre already sliding under braking has almost nothing left to steer with. */
TEST(Unit_VehicleTyre, BrakingHardLeavesNothingToSteerWith)
{
    const TyreSettings tyre = road_tyre();

    TyreSlip steering;
    steering.lateral = Scalar(0.1);
    const Scalar free_grip = tyre_force(tyre, steering, RATED_LOAD).lateral;

    TyreSlip braking = steering;
    braking.longitudinal = -1;
    const Scalar used_grip = tyre_force(tyre, braking, RATED_LOAD).lateral;

    EXPECT_LT(std::fabs(double(used_grip)), std::fabs(double(free_grip)) * 0.5)
        << "understeer under braking must fall out of the model, not be detected";
}

/** @brief Friction falls with load, and never to nothing. */
TEST(Unit_VehicleTyre, GripFallsAsLoadRises)
{
    const TyreSettings tyre = road_tyre();
    const Scalar light = tyre_friction(tyre, RATED_LOAD / 2);
    const Scalar rated = tyre_friction(tyre, RATED_LOAD);
    const Scalar heavy = tyre_friction(tyre, RATED_LOAD * 2);

    EXPECT_GT(double(light), double(rated));
    EXPECT_GT(double(rated), double(heavy));
    EXPECT_NEAR(double(rated), double(tyre.friction), 1e-12) << "at rated load, as authored";
    EXPECT_GT(double(heavy), 0.0) << "a tyre that lost all grip to load would let go silently";
}

/** @brief Standstill is a damper, not a division by zero. */
TEST(Unit_VehicleTyre, StandstillIsADamperAndNotASingularity)
{
    const TyreSettings tyre = road_tyre();
    const TyreSlip creeping = tyre_slip(tyre, Scalar(0.001), Scalar(0), Scalar(0.001));
    EXPECT_LT(std::fabs(double(creeping.longitudinal)), 0.001)
        << "divided by the hub speed this would be enormous";

    const TyreSlip rolling = tyre_slip(tyre, Scalar(0), Scalar(0), Scalar(30));
    EXPECT_EQ(double(rolling.longitudinal), 0.0);
}

/** @brief A driven wheel turns its spin into travel and settles at rolling. */
TEST(Unit_VehicleTyre, ASpinningWheelDrivesItselfForward)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);
    ContactManifold<Scalar> carried;
    const TyreSettings tyre = road_tyre();

    hold_spin(solver, wheel, DRIVE_SPIN);
    TyreReportT<Scalar> report;
    for (int tick = 0; tick < 120; ++tick)
    {
        report = roll(solver, wheel, tyre, carried);
        // The tyre alone would spin the wheel down; hold the throttle open so that what
        // is under test is whether the patch converts spin into travel.
        RigidBody body;
        solver.read_body(wheel, body);
        if (body.angular_velocity.x < DRIVE_SPIN)
            hold_spin(solver, wheel, DRIVE_SPIN);
    }

    RigidBody body;
    ASSERT_TRUE(solver.read_body(wheel, body));
    EXPECT_TRUE(report.grounded);
    EXPECT_GT(double(report.load), 0.0) << "and the load is the one the solver recovered";
    EXPECT_NEAR(double(body.velocity.z), double(DRIVE_SPIN * RADIUS), 0.05)
        << "a driven wheel accelerates until it is rolling";
}

/** @brief A freely rolling wheel is not slowed by its own tyre. */
TEST(Unit_VehicleTyre, ARollingWheelIsLeftAlone)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);
    ContactManifold<Scalar> carried;
    const TyreSettings tyre = road_tyre();

    RigidBody body;
    solver.read_body(wheel, body);
    body.velocity = Vector3{0, 0, 6};
    body.angular_velocity = Vector3{6 / RADIUS, 0, 0};
    solver.write_body(wheel, body);

    TyreReportT<Scalar> report;
    for (int tick = 0; tick < 20; ++tick)
        report = roll(solver, wheel, tyre, carried);

    ASSERT_TRUE(solver.read_body(wheel, body));
    EXPECT_NEAR(double(body.velocity.z), 6.0, 0.5);
    EXPECT_LT(double(report.force.saturation), 0.1);
}

/** @brief A locked wheel slides at exactly the friction limit, and slows. */
TEST(Unit_VehicleTyre, ALockedWheelSlidesAtTheFrictionLimit)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);
    ContactManifold<Scalar> carried;
    const TyreSettings tyre = road_tyre();

    RigidBody body;
    solver.read_body(wheel, body);
    body.velocity = Vector3{0, 0, 25};
    solver.write_body(wheel, body);

    TyreReportT<Scalar> report;
    for (int tick = 0; tick < 20; ++tick)
    {
        report = roll(solver, wheel, tyre, carried);
        hold_spin(solver, wheel, 0);   // the brake, held on
    }

    const Scalar limit = tyre_friction(tyre, report.load) * report.load;
    EXPECT_LT(double(report.force.longitudinal), 0.0) << "dragged the way it is sliding";
    EXPECT_NEAR(double(report.force.longitudinal), -double(limit), 1e-9);
    EXPECT_EQ(double(report.force.saturation), 1.0);

    ASSERT_TRUE(solver.read_body(wheel, body));
    EXPECT_LT(double(body.velocity.z), 25.0);
}

/**
 * @brief A parked wheel sits still and reports exactly its own weight.
 *
 * The assertion that earns its keep: a load recovered in the wrong units is a load off by
 * a few hundred, and a tyre asked for grip proportional to half a newton simply produces
 * almost nothing — a car that rolls but will not drive, with every formula correct.
 */
TEST(Unit_VehicleTyre, AParkedWheelSitsStillAndCarriesItsOwnWeight)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);
    ContactManifold<Scalar> carried;
    const TyreSettings tyre = road_tyre();

    TyreReportT<Scalar> report;
    for (int tick = 0; tick < 120; ++tick)
        report = roll(solver, wheel, tyre, carried);

    RigidBody body;
    ASSERT_TRUE(solver.read_body(wheel, body));
    const double drift = std::sqrt(double(body.velocity.x * body.velocity.x +
                                          body.velocity.z * body.velocity.z));
    EXPECT_LT(drift, 0.01) << "a parked wheel must not buzz";
    EXPECT_NEAR(double(report.load), double(WHEEL_MASS * GRAVITY), 1.0);
}

/** @brief Whatever the wheel pushes against takes the other end of it. */
TEST(Unit_VehicleTyre, TheGroundTakesTheReaction)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());

    RigidBody floor;
    floor.position = Vector3{0, 0, 0};
    floor.prev_position = floor.position;
    floor.orientation = Quaternion{0, 0, 0, 1};
    floor.prev_orientation = floor.orientation;
    floor.inv_mass = Scalar(1) / 500;
    floor.inv_inertia = Vector3{0, 0, 0};
    // Gravity cancelled: a dynamic floor would otherwise fall alongside the wheel and the
    // pair would carry no load at all.
    floor.external_acceleration = Vector3{0, GRAVITY, 0};
    floor.flags = BodyFlags::dynamic_body;
    const BodyHandle ground = solver.add_body(floor);
    const BodyHandle wheel = spawn_wheel(solver);

    ContactManifold<Scalar> carried;
    const TyreSettings tyre = road_tyre();

    RigidBody body;
    solver.read_body(wheel, body);
    body.velocity = Vector3{0, 0, 15};
    solver.write_body(wheel, body);

    for (int tick = 0; tick < 6; ++tick)
    {
        roll(solver, wheel, tyre, carried, ground);
        hold_spin(solver, wheel, 0);
    }

    RigidBody before_wheel;
    RigidBody before_floor;
    ASSERT_TRUE(solver.read_body(wheel, before_wheel));
    ASSERT_TRUE(solver.read_body(ground, before_floor));
    const Scalar before = before_wheel.velocity.z / before_wheel.inv_mass +
                          before_floor.velocity.z / before_floor.inv_mass;

    const TyreReportT<Scalar> report = roll(solver, wheel, tyre, carried, ground);

    RigidBody after_wheel;
    RigidBody after_floor;
    ASSERT_TRUE(solver.read_body(wheel, after_wheel));
    ASSERT_TRUE(solver.read_body(ground, after_floor));
    const Scalar after = after_wheel.velocity.z / after_wheel.inv_mass +
                         after_floor.velocity.z / after_floor.inv_mass;

    EXPECT_GT(double(report.load), 0.0);
    EXPECT_GT(double(after_floor.velocity.z), 0.0) << "dragged the way the wheel is sliding";
    EXPECT_NEAR(double(after), double(before), 1e-9) << "the pair's momentum is unchanged";
}

/** @brief A tyre with no friction is a castor: grounded, and doing nothing. */
TEST(Unit_VehicleTyre, ATyreWithNoFrictionDoesNothing)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);
    ContactManifold<Scalar> carried;
    TyreSettings castor = road_tyre();
    castor.friction = 0;

    hold_spin(solver, wheel, DRIVE_SPIN);
    TyreReportT<Scalar> report;
    for (int tick = 0; tick < 30; ++tick)
        report = roll(solver, wheel, castor, carried);

    RigidBody body;
    ASSERT_TRUE(solver.read_body(wheel, body));
    EXPECT_EQ(double(report.force.longitudinal), 0.0);
    EXPECT_NEAR(double(body.velocity.z), 0.0, 1e-9) << "it spins on the spot";
    EXPECT_TRUE(report.grounded) << "while still reporting the ground it is on";
}

/** @brief A wheel with no contact reports nothing rather than guessing. */
TEST(Unit_VehicleTyre, AnAirborneWheelReportsNothing)
{
    HostXPBDSolver<Scalar> solver(wheel_scene());
    const BodyHandle wheel = spawn_wheel(solver);

    solver.begin_contacts();
    const TyreReportT<Scalar> report =
        apply_tyre_force(solver, wheel, road_tyre(), Vector3{1, 0, 0}, TICK, TICK / 8);
    EXPECT_FALSE(report.grounded);
    EXPECT_EQ(double(report.load), 0.0);
}
