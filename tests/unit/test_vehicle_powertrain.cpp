/**************************************************************************/
/* test_vehicle_powertrain.cpp                                            */
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
 * @file test_vehicle_powertrain.cpp
 * @brief §11.4's chain, held to the three things a drivetrain must not get wrong.
 *
 * A powertrain is easy to write so that it *looks* right: throttle makes the wheels turn
 * and the engine note rises. Three properties separate that from a drivetrain, and each
 * one has a closed form to check it against.
 *
 * 1. **A clutch below its capacity locks exactly.** Not "converges", not "nearly" — the
 *    torque is solved for, so after the step the crank and the geared wheel speed agree
 *    to round-off. A stiff spring in place of that solve would ring, and the ringing
 *    would be blamed on the tyres.
 * 2. **A differential never invents torque.** Whatever the lock setting, the wheel
 *    torques sum to what came down the shaft. A differential that failed this would
 *    accelerate a car with its wheels off the ground.
 * 3. **The engine's reaction lands on the chassis.** The impulse the wheels gain and the
 *    impulse the core loses are the same number, so a vehicle cannot turn itself by
 *    revving.
 *
 * The one-dimensional half needs no solver at all, which is the point of the seam: those
 * cases integrate the wheels themselves in two lines and are exact.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/vehicle/vehicle_instance.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar CORE_MASS = Scalar(800);
    constexpr Scalar SHELL_MASS = Scalar(80);
    constexpr Scalar TICK = Scalar(1) / Scalar(60);

    /** @brief The core's principal inertia, which the reaction test reads back through. */
    constexpr Scalar CORE_INERTIA_X = Scalar(600);
    constexpr Scalar CORE_INERTIA_Y = Scalar(900);
    constexpr Scalar CORE_INERTIA_Z = Scalar(400);

    /** @brief An ordinary road car: five forward gears, a reverse, and a neutral. */
    PowertrainSettings road_car(Scalar lock_torque = 0)
    {
        PowertrainSettings settings;
        settings.engine.curve = {{80, 120}, {200, 260}, {400, 300}, {600, 250}, {700, 180}};
        settings.engine.inertia = Scalar(0.22);
        settings.engine.idle_rate = 90;
        settings.engine.idle_band = 20;
        settings.engine.limit_rate = 680;
        settings.engine.friction_torque = 12;
        settings.engine.viscous_damping = Scalar(0.02);
        settings.gearbox.ratios = {Scalar(-3.2), 0, Scalar(3.4), Scalar(2.1), Scalar(1.4), 1,
                                   Scalar(0.8)};
        settings.gearbox.final_drive = Scalar(3.9);
        settings.gearbox.inertia = Scalar(0.05);
        settings.differential.lock_torque = lock_torque;
        settings.clutch_capacity = 900;
        return settings;
    }

    /** @brief Gear indices into @ref road_car's ratio list, named so the tests read. */
    constexpr std::size_t REVERSE = 0;
    constexpr std::size_t NEUTRAL = 1;
    constexpr std::size_t FIRST = 2;
    constexpr std::size_t FOURTH = 4;

    /** @brief A pair of identical driven wheels at the given speeds. */
    std::vector<DrivenWheel> wheel_pair(Scalar left, Scalar right,
                                        Scalar inertia = Scalar(1.156))
    {
        std::vector<DrivenWheel> wheels(2);
        wheels[0].spin_rate = left;
        wheels[0].inertia = inertia;
        wheels[1].spin_rate = right;
        wheels[1].inertia = inertia;
        return wheels;
    }

    /** @brief Integrates the wheels by the torque they were just handed. */
    void carry_wheels(std::vector<DrivenWheel>& wheels)
    {
        for (DrivenWheel& wheel : wheels)
            wheel.spin_rate += wheel.drive_torque / wheel.inertia * TICK;
    }

    PhysicsConfiguration vehicle_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 64;
        configuration.capacities.constraints = 256;
        configuration.capacities.contacts = 256;
        configuration.capacities.joints = 64;
        configuration.capacities.beams = 256;
        configuration.capacities.colors = 12;
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 8;
        return configuration;
    }

    /** @brief A four-node shell on a rigid core, the same chassis P7-E and P7-F use. */
    Cooking::NodeBeamAsset chassis_asset()
    {
        Cooking::NodeBeamAsset asset;
        const Vector3 corners[4] = {{-0.8, 0.3, -1.4},
                                    {0.8, 0.3, -1.4},
                                    {0.8, 0.3, 1.4},
                                    {-0.8, 0.3, 1.4}};
        for (const Vector3& corner : corners)
        {
            Cooking::NodeBeamNodeRecord node{};
            node.position = corner;
            node.mass = SHELL_MASS / 4;
            node.radius = Scalar(0.05);
            asset.nodes.push_back(node);
        }
        for (std::uint32_t i = 0; i < 4; ++i)
        {
            Cooking::NodeBeamBeamRecord beam{};
            beam.a = i;
            beam.b = (i + 1) % 4;
            beam.rest_length = length(corners[(i + 1) % 4] - corners[i]);
            beam.compliance = Scalar(1e-8);
            asset.beams.push_back(beam);

            Cooking::NodeBeamAttachmentRecord attachment{};
            attachment.node = i;
            attachment.core_anchor = corners[i];
            asset.attachments.push_back(attachment);
        }
        asset.core.mass = CORE_MASS;
        asset.core.principal_inertia =
            Vector3{CORE_INERTIA_X, CORE_INERTIA_Y, CORE_INERTIA_Z};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
        asset.summary.node_mass = SHELL_MASS;
        asset.summary.total_mass = SHELL_MASS + CORE_MASS;
        asset.summary.part_count = 1;
        return asset;
    }

    /** @brief Front corners steer, rear corners drive; every axle points the same way. */
    VehicleAsset rear_wheel_drive()
    {
        VehicleAsset asset;
        const Scalar x[4] = {Scalar(-0.75), Scalar(0.75), Scalar(0.75), Scalar(-0.75)};
        const Scalar z[4] = {Scalar(-1.3), Scalar(-1.3), Scalar(1.3), Scalar(1.3)};
        for (int i = 0; i < 4; ++i)
        {
            SuspensionSetup setup;
            setup.mount = Vector3{x[i], Scalar(0.55), z[i]};
            setup.axis = Vector3{0, 1, 0};
            setup.axle = Vector3{1, 0, 0};
            setup.steered = z[i] < 0;
            setup.driven = z[i] > 0;
            asset.corners.push_back(setup);
        }
        asset.powertrain = road_car();
        return asset;
    }

    /** @brief Holds a cooked blob alive alongside the view into it. */
    struct Blob
    {
        std::vector<std::byte> bytes;
        Cooking::NodeBeamAssetView view;

        explicit Blob(const Cooking::NodeBeamAsset& asset)
        {
            if (Cooking::build_node_beam_blob(asset, bytes))
                view = Cooking::load_node_beam_blob(bytes.data(), bytes.size());
        }
    };
} // namespace

/** @brief The curve interpolates inside itself and is held flat outside. */
TEST(Unit_VehiclePowertrain, TorqueCurveInterpolatesAndIsHeldFlatOutside)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    EXPECT_NEAR(double(train.curve_torque(0)), 120.0, 1e-9) << "below the first sample";
    EXPECT_NEAR(double(train.curve_torque(900)), 180.0, 1e-9)
        << "an extrapolated curve would go negative on a long straight";
    EXPECT_NEAR(double(train.curve_torque(140)), 190.0, 1e-9) << "halfway between two samples";
    EXPECT_NEAR(double(train.curve_torque(400)), 300.0, 1e-9) << "a sample reads itself";
}

/** @brief An unusable curve is refused and the last good one survives the refusal. */
TEST(Unit_VehiclePowertrain, RefusesACurveItCannotWalk)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));

    PowertrainSettings empty = road_car();
    empty.engine.curve.clear();
    EXPECT_FALSE(train.configure(empty));

    PowertrainSettings backwards = road_car();
    backwards.engine.curve = {{200, 260}, {80, 120}};
    EXPECT_FALSE(train.configure(backwards));

    EXPECT_EQ(train.settings().engine.curve.size(), std::size_t(5))
        << "a refused drivetrain must not half-replace the one in force";
}

/** @brief Neutral is a ratio of zero, and a ratio of zero drives nothing. */
TEST(Unit_VehiclePowertrain, NeutralFreeRevsAndDrivesNothing)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(NEUTRAL));
    train.set_throttle(1);

    std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
    PowertrainReport report;
    for (int tick = 0; tick < 600; ++tick)
        report = train.step(wheels.data(), wheels.size(), TICK);

    EXPECT_EQ(double(report.drive_torque), 0.0);
    EXPECT_EQ(double(wheels[0].drive_torque), 0.0);
    EXPECT_GT(double(report.engine_rate), 600.0) << "and the engine runs up to its limiter";
}

/**
 * @brief The limiter holds the engine, within the one tick it takes to notice.
 *
 * An ignition cut is a discrete event and the crank is light: a whole tick of peak
 * torque on 0.22 kg·m² is 22.7 rad/s, so the overshoot is real rather than a defect,
 * and the assertion is that bound rather than a round number.
 */
TEST(Unit_VehiclePowertrain, TheLimiterCapsTheEngineWithinOneTick)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(NEUTRAL));
    train.set_throttle(1);

    std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
    Scalar peak = 0;
    for (int tick = 0; tick < 1200; ++tick)
    {
        const PowertrainReport report = train.step(wheels.data(), wheels.size(), TICK);
        if (report.engine_rate > peak)
            peak = report.engine_rate;
    }

    const double bound = 680.0 + 300.0 / 0.22 * double(TICK);
    EXPECT_LT(double(peak), bound) << "the cut must bite within a tick";
    EXPECT_GT(double(peak), 670.0) << "and must not hold the engine far below its cut";
}

/** @brief The governor lifts the engine to idle and leaves it inside its droop. */
TEST(Unit_VehiclePowertrain, TheIdleGovernorHoldsIdleWithNoThrottle)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(NEUTRAL));
    train.set_throttle(0);
    train.set_engine_rate(0);

    std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
    PowertrainReport report;
    for (int tick = 0; tick < 900; ++tick)
        report = train.step(wheels.data(), wheels.size(), TICK);

    EXPECT_GT(double(report.engine_rate), 85.0) << "inside the 20 rad/s proportional band";
    EXPECT_LT(double(report.engine_rate), 90.0)
        << "a proportional governor has no authority at its own target";
}

/**
 * @brief An engaged clutch below its capacity locks the two sides exactly.
 *
 * The closed form the whole design rests on. A spring-damper clutch would leave a
 * residual here that grew with the gear ratio.
 */
TEST(Unit_VehiclePowertrain, AnEngagedClutchLocksTheCrankToTheWheels)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(5));
    train.set_throttle(1);
    train.set_clutch(1);
    train.set_engine_rate(300);

    std::vector<DrivenWheel> wheels = wheel_pair(20, 20);
    const Scalar ratio = train.total_ratio();
    PowertrainReport report;
    for (int tick = 0; tick < 20; ++tick)
    {
        report = train.step(wheels.data(), wheels.size(), TICK);
        carry_wheels(wheels);
    }

    const Scalar output = ratio * (wheels[0].spin_rate + wheels[1].spin_rate) / 2;
    EXPECT_NEAR(double(report.engine_rate), double(output), 1e-6);
    EXPECT_FALSE(report.clutch_slipping);
}

/** @brief A clutch that cannot hold carries exactly its capacity, and none when released. */
TEST(Unit_VehiclePowertrain, AnOverwhelmedClutchCarriesExactlyItsCapacity)
{
    PowertrainSettings worn = road_car();
    worn.clutch_capacity = 40;

    Powertrain train;
    ASSERT_TRUE(train.configure(worn));
    ASSERT_TRUE(train.select_gear(FIRST));
    train.set_throttle(1);
    train.set_clutch(1);
    train.set_engine_rate(400);

    std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
    const PowertrainReport slipping = train.step(wheels.data(), wheels.size(), TICK);
    EXPECT_TRUE(slipping.clutch_slipping);
    EXPECT_NEAR(double(slipping.clutch_torque), 40.0, 1e-9);

    train.set_clutch(0);
    const PowertrainReport released = train.step(wheels.data(), wheels.size(), TICK);
    EXPECT_EQ(double(released.clutch_torque), 0.0);
    EXPECT_EQ(double(wheels[0].drive_torque), 0.0);
}

/** @brief An open differential splits evenly whatever its outputs are doing. */
TEST(Unit_VehiclePowertrain, AnOpenDifferentialSplitsEvenly)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car(0)));
    ASSERT_TRUE(train.select_gear(FIRST));
    train.set_throttle(1);
    train.set_engine_rate(300);

    std::vector<DrivenWheel> wheels = wheel_pair(5, 12);
    const PowertrainReport report = train.step(wheels.data(), wheels.size(), TICK);

    EXPECT_NEAR(double(wheels[0].drive_torque), double(wheels[1].drive_torque), 1e-9)
        << "an open differential cannot see the speed difference";
    EXPECT_NEAR(double(wheels[0].drive_torque + wheels[1].drive_torque),
                double(report.drive_torque), 1e-9);
}

/** @brief A locked differential drives the slower wheel harder and still conserves. */
TEST(Unit_VehiclePowertrain, ALockedDifferentialFavoursTheSlowerWheel)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car(4000)));
    ASSERT_TRUE(train.select_gear(FIRST));
    train.set_throttle(1);
    train.set_engine_rate(300);

    std::vector<DrivenWheel> wheels = wheel_pair(5, 12);
    const PowertrainReport report = train.step(wheels.data(), wheels.size(), TICK);

    EXPECT_GT(double(wheels[0].drive_torque), double(wheels[1].drive_torque));
    EXPECT_NEAR(double(wheels[0].drive_torque + wheels[1].drive_torque),
                double(report.drive_torque), 1e-9)
        << "the lock torques must cancel; a differential is not a source of torque";
}

/** @brief Unequal wheels and a biting clamp still conserve. */
TEST(Unit_VehiclePowertrain, AnUnequalPairStillConserves)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car(30)));
    ASSERT_TRUE(train.select_gear(FIRST));
    train.set_throttle(1);
    train.set_engine_rate(300);

    std::vector<DrivenWheel> wheels(2);
    wheels[0].spin_rate = 3;
    wheels[0].inertia = Scalar(0.9);
    wheels[1].spin_rate = 14;
    wheels[1].inertia = Scalar(2.4);

    const PowertrainReport report = train.step(wheels.data(), wheels.size(), TICK);
    EXPECT_NEAR(double(wheels[0].drive_torque + wheels[1].drive_torque),
                double(report.drive_torque), 1e-9);
}

/** @brief Lifting off in gear turns the chain into a brake. */
TEST(Unit_VehiclePowertrain, ClosingTheThrottleInGearBrakesTheWheels)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(FOURTH));
    train.set_throttle(0);
    train.set_clutch(1);
    train.set_engine_rate(train.total_ratio() * 60);

    std::vector<DrivenWheel> wheels = wheel_pair(60, 60);
    const PowertrainReport report = train.step(wheels.data(), wheels.size(), TICK);

    EXPECT_LT(double(report.drive_torque), 0.0) << "engine braking is a negative drive torque";
    EXPECT_LT(double(wheels[0].drive_torque), 0.0) << "and it reaches the wheels";
}

/** @brief Reverse is a negative ratio and needs nothing else to be one. */
TEST(Unit_VehiclePowertrain, ReverseIsANegativeRatioAndNothingElse)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(REVERSE));
    train.set_throttle(1);
    train.set_engine_rate(200);

    std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
    for (int tick = 0; tick < 40; ++tick)
    {
        train.step(wheels.data(), wheels.size(), TICK);
        carry_wheels(wheels);
    }

    EXPECT_LT(double(wheels[0].spin_rate), 0.0) << "the wheel ends up turning backwards";
    EXPECT_GT(double(train.engine_rate()), 0.0) << "while the crank still turns forwards";
}

/** @brief A step of no time changes nothing and leaves no stale torque behind. */
TEST(Unit_VehiclePowertrain, AZeroStepChangesNothing)
{
    Powertrain train;
    ASSERT_TRUE(train.configure(road_car()));
    ASSERT_TRUE(train.select_gear(FIRST));
    train.set_throttle(1);
    train.set_engine_rate(300);

    std::vector<DrivenWheel> wheels = wheel_pair(10, 10);
    wheels[0].drive_torque = 999;
    const PowertrainReport report = train.step(wheels.data(), wheels.size(), 0);

    EXPECT_EQ(double(train.engine_rate()), 300.0);
    EXPECT_EQ(double(report.drive_torque), 0.0);
    EXPECT_EQ(double(wheels[0].drive_torque), 0.0) << "a torque from last tick must not linger";
}

/** @brief Two identical runs agree bit for bit (§0.5). */
TEST(Unit_VehiclePowertrain, ReplayIsIdentical)
{
    Scalar results[2] = {0, 0};
    for (int pass = 0; pass < 2; ++pass)
    {
        Powertrain train;
        ASSERT_TRUE(train.configure(road_car(120)));
        ASSERT_TRUE(train.select_gear(FIRST));
        train.set_throttle(Scalar(0.7));
        train.set_engine_rate(150);

        std::vector<DrivenWheel> wheels = wheel_pair(0, 0);
        for (int tick = 0; tick < 300; ++tick)
        {
            train.step(wheels.data(), wheels.size(), TICK);
            carry_wheels(wheels);
        }
        results[pass] = train.engine_rate() + wheels[0].spin_rate;
    }
    EXPECT_EQ(double(results[0]), double(results[1]));
}

/** @brief The torque reaches the driven wheels and only those. */
TEST(Unit_VehiclePowertrain, TheDrivetrainTurnsOnlyTheDrivenWheels)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, rear_wheel_drive(), settings));
    ASSERT_EQ(vehicle.driven_corner_count(), std::size_t(2));

    ASSERT_TRUE(vehicle.select_gear(FIRST));
    vehicle.set_throttle(1);
    vehicle.set_clutch(1);

    StepParameters<Scalar> parameters;
    parameters.delta_time = TICK;
    parameters.gravity = Vector3{0, 0, 0};
    for (int tick = 0; tick < 30; ++tick)
    {
        vehicle.begin_tick(solver, parameters.delta_time);
        solver.step(parameters);
        vehicle.end_tick(solver);
    }

    EXPECT_GT(double(vehicle.corner(2).spin_rate(solver)), 5.0) << "rear right";
    EXPECT_GT(double(vehicle.corner(3).spin_rate(solver)), 5.0) << "rear left";
    EXPECT_NEAR(double(vehicle.corner(0).spin_rate(solver)), 0.0, 1e-6) << "front left is idle";
    EXPECT_GT(double(vehicle.powertrain().engine_rate()), 90.0);
}

/**
 * @brief The chassis takes exactly the reaction of the drive, and about the axle only.
 *
 * The statement that stops a car from driving itself around in mid-air. Read as angular
 * momentum rather than as a velocity, because the core's inertia is not isotropic and
 * only the momentum is comparable with an impulse.
 */
TEST(Unit_VehiclePowertrain, TheChassisTakesTheReactionOfTheDrive)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, rear_wheel_drive(), settings));
    ASSERT_TRUE(vehicle.select_gear(FIRST));
    vehicle.set_throttle(1);
    vehicle.set_clutch(1);

    RigidBody before;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), before));

    const PowertrainReport report = vehicle.begin_tick(solver, TICK);
    ASSERT_GT(double(report.drive_torque), 0.0);

    RigidBody after;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), after));
    const Vector3 gained = after.angular_velocity - before.angular_velocity;

    EXPECT_NEAR(double(gained.x * CORE_INERTIA_X), -double(report.drive_torque * TICK), 1e-9);
    EXPECT_EQ(double(gained.y * CORE_INERTIA_Y), 0.0) << "and about the axle and nothing else";
    EXPECT_EQ(double(gained.z * CORE_INERTIA_Z), 0.0);
}

/** @brief A vehicle with no drivetrain instances, and stays still however hard it is asked. */
TEST(Unit_VehiclePowertrain, AnUndrivenVehicleIsUntouched)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());

    VehicleAsset trailer = rear_wheel_drive();
    for (SuspensionSetup& corner : trailer.corners)
        corner.driven = false;
    trailer.powertrain = PowertrainSettings{};

    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, trailer, settings))
        << "a drivetrain that configure() would refuse is a trailer, not a failure";
    EXPECT_EQ(vehicle.driven_corner_count(), std::size_t(0));

    vehicle.set_throttle(1);
    const PowertrainReport report = vehicle.begin_tick(solver, TICK);
    EXPECT_EQ(double(report.drive_torque), 0.0);
    EXPECT_EQ(double(vehicle.corner(0).spin_rate(solver)), 0.0);
}
