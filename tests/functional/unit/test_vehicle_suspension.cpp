/**************************************************************************/
/* test_vehicle_suspension.cpp                                            */
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
 * @file test_vehicle_suspension.cpp
 * @brief §11.2's third row, held to numbers: a strut that carries the weight it should,
 *        a damper that settles it, steering that turns the right wheels, and a brake.
 *
 * The assertions here are deliberately *quantitative* where the physics gives a closed
 * form. A suspension that merely "looks sprung" is the failure mode this kind of code
 * has: a strut with the spring rate misread as a compliance, or the travel coordinate
 * signed the wrong way, still holds a car up and still moves when it is pushed. What it
 * does not do is settle at `m·g/k`, which is the one number an author can predict.
 *
 * The wheels are bolted to the world to stand in for the ground, because nothing here
 * generates a contact yet (§11.5 and P7-H). Gravity alone cannot load a vehicle: it is
 * uniform, so a car with nothing under it falls as one piece and every spring in it
 * carries exactly nothing.
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
    constexpr Scalar SPRING_RATE = Scalar(35000);
    constexpr Scalar GRAVITY = Scalar(9.81);

    /** @brief The budget every scene here runs under. */
    PhysicsConfiguration vehicle_scene(std::size_t bodies = 64)
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = bodies;
        configuration.capacities.constraints = 256;
        configuration.capacities.contacts = 256;
        configuration.capacities.joints = 64;
        configuration.capacities.beams = 256;
        configuration.capacities.colors = 12;
        // Fixed rather than derived, because two of these tests compare runs against
        // each other and a schedule that varied with the motion would vary between them.
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 8;
        return configuration;
    }

    /** @brief A four-node shell on an 800 kg core: the smallest thing a corner can hang from. */
    Cooking::NodeBeamAsset chassis_asset()
    {
        Cooking::NodeBeamAsset asset;
        const Vector3 corners[4] = {{Scalar(-0.8), Scalar(0.3), Scalar(-1.4)},
                                    {Scalar(0.8), Scalar(0.3), Scalar(-1.4)},
                                    {Scalar(0.8), Scalar(0.3), Scalar(1.4)},
                                    {Scalar(-0.8), Scalar(0.3), Scalar(1.4)}};
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
        asset.core.principal_inertia = Vector3{Scalar(600), Scalar(900), Scalar(400)};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
        asset.summary.node_mass = SHELL_MASS;
        asset.summary.total_mass = SHELL_MASS + CORE_MASS;
        asset.summary.part_count = 1;
        return asset;
    }

    /** @brief Four MacPherson corners, the front two steered. */
    VehicleAsset four_corners(Scalar damping, Scalar spring_rate = SPRING_RATE)
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
            setup.rest_length = Scalar(0.35);
            setup.travel_bump = Scalar(0.12);
            setup.travel_droop = Scalar(0.12);
            setup.spring_rate = spring_rate;
            setup.damping = damping;
            setup.carrier_mass = Scalar(15);
            setup.wheel_mass = Scalar(20);
            setup.wheel_radius = Scalar(0.34);
            setup.wheel_width = Scalar(0.22);
            setup.steered = z[i] < 0;
            asset.corners.push_back(setup);
        }
        return asset;
    }

    /** @brief An authored asset, serialized and loaded, which is how a vehicle arrives. */
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

    /** @brief Advances the solver and runs the vehicle's tick boundary, @p ticks times. */
    void run(HostXpbdSolver<Scalar>& solver, VehicleInstance& vehicle, int ticks)
    {
        StepParameters<Scalar> parameters;
        parameters.delta_time = Scalar(1) / Scalar(60);
        parameters.gravity = Vector3{0, -GRAVITY, 0};
        for (int tick = 0; tick < ticks; ++tick)
        {
            solver.step(parameters);
            vehicle.end_tick(solver);
        }
    }

    /**
     * @brief Bolts a corner's hub and wheel to the world.
     *
     * The ground, until P7-H gives a tyre something to push against. Crude, and
     * deliberately so: the point of these tests is the strut above the wheel, and a
     * contact model underneath it would put a second unverified thing in every
     * measurement.
     */
    void plant(HostXpbdSolver<Scalar>& solver, const SuspensionUnit& corner)
    {
        const BodyHandle handles[2] = {corner.carrier(), corner.wheel()};
        for (BodyHandle handle : handles)
        {
            RigidBody body;
            if (!solver.read_body(handle, body))
                continue;
            body.inv_mass = 0;
            body.inv_inertia = Vector3{0, 0, 0};
            EXPECT_TRUE(solver.write_body(handle, body));
        }
    }

    /** @brief The world direction a corner's wheel spins about. */
    Vector3 axle_direction(const HostXpbdSolver<Scalar>& solver, const SuspensionUnit& corner)
    {
        RigidBody wheel;
        if (!solver.read_body(corner.wheel(), wheel))
            return Vector3{1, 0, 0};
        return rotate(wheel.orientation, Vector3{1, 0, 0});
    }

    /** @brief The angle between two unit directions, in radians. */
    Scalar angle_between(const Vector3& a, const Vector3& b)
    {
        Scalar cosine = dot(a, b);
        cosine = cosine > Scalar(1) ? Scalar(1) : cosine;
        cosine = cosine < Scalar(-1) ? Scalar(-1) : cosine;
        return Scalar(std::acos(double(cosine)));
    }
} // namespace

/** @brief Every corner in the asset becomes two bodies and two joints. */
TEST(Unit_VehicleSuspension, InstancesEveryCorner)
{
    const Blob blob(chassis_asset());
    ASSERT_TRUE(blob.view.valid);

    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(6)),
                               NodeBeamStructureSettings<Scalar>{}));

    EXPECT_EQ(vehicle.corner_count(), 4u);
    for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
        EXPECT_TRUE(vehicle.corner(i).valid()) << "corner " << i;
    EXPECT_TRUE(vehicle.structure().has_core());
}

/** @brief The wheel is a cylinder about its axle, and its body frame says so. */
TEST(Unit_VehicleSuspension, WheelInertiaIsACylinderAboutTheAxle)
{
    const Blob blob(chassis_asset());
    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(6)),
                               NodeBeamStructureSettings<Scalar>{}));

    RigidBody wheel;
    ASSERT_TRUE(solver.read_body(vehicle.corner(0).wheel(), wheel));

    const Scalar mass = Scalar(20);
    const Scalar radius = Scalar(0.34);
    const Scalar width = Scalar(0.22);
    const Scalar axial = Scalar(0.5) * mass * radius * radius;
    const Scalar transverse = mass * (Scalar(3) * radius * radius + width * width) / Scalar(12);
    EXPECT_NEAR(double(Scalar(1) / wheel.inv_inertia.x), double(axial), 1e-9);
    EXPECT_NEAR(double(Scalar(1) / wheel.inv_inertia.y), double(transverse), 1e-9);
    EXPECT_NEAR(double(Scalar(1) / wheel.inv_inertia.z), double(transverse), 1e-9);
}

/**
 * @brief The strut settles at the compression the corner weight and the spring rate imply.
 *
 * The one number an author can predict without running anything: `m·g / k`. A spring
 * rate read as a compliance, or a travel coordinate signed the wrong way, still holds
 * the car up — and lands nowhere near this.
 */
TEST(Unit_VehicleSuspension, TheSpringCarriesTheCornerWeight)
{
    const Blob blob(chassis_asset());
    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(8)),
                               NodeBeamStructureSettings<Scalar>{}));
    for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
        plant(solver, vehicle.corner(i));

    run(solver, vehicle, 240);

    const Scalar weight = (CORE_MASS + SHELL_MASS) * GRAVITY / Scalar(4);
    const Scalar expected = weight / SPRING_RATE;
    for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
    {
        EXPECT_NEAR(double(vehicle.corner(i).compression(solver)), double(expected),
                    0.05 * double(expected))
            << "corner " << i;
    }

    // And the load readout agrees with the same weight, which is §10.4's recovery
    // answering a question a suspension inspector actually asks.
    EXPECT_NEAR(double(length(vehicle.corner(0).load(solver))), double(weight),
                0.1 * double(weight));
}

/** @brief The damper settles a strut a spring alone would leave ringing. */
TEST(Unit_VehicleSuspension, TheDamperSettlesTheStrut)
{
    const Blob blob(chassis_asset());
    Scalar swing[2] = {0, 0};

    for (int damped = 0; damped < 2; ++damped)
    {
        HostXpbdSolver<Scalar> solver(vehicle_scene());
        VehicleInstance vehicle;
        ASSERT_TRUE(vehicle.create(solver, blob.view,
                                   four_corners(damped != 0 ? Scalar(8) : Scalar(0)),
                                   NodeBeamStructureSettings<Scalar>{}));
        for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
            plant(solver, vehicle.corner(i));

        run(solver, vehicle, 120);
        Scalar low = Scalar(1e9);
        Scalar high = Scalar(-1e9);
        for (int i = 0; i < 120; ++i)
        {
            run(solver, vehicle, 1);
            const Scalar compression = vehicle.corner(0).compression(solver);
            low = compression < low ? compression : low;
            high = compression > high ? compression : high;
        }
        swing[damped] = high - low;
    }

    EXPECT_LT(double(swing[1]), 0.1 * double(swing[0]));
}

/** @brief Steering turns the corners that steer, by the angle asked, and no others. */
TEST(Unit_VehicleSuspension, SteeringTurnsTheFrontAxlesAndLeavesTheRear)
{
    const Blob blob(chassis_asset());
    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(8)),
                               NodeBeamStructureSettings<Scalar>{}));
    run(solver, vehicle, 10);

    Vector3 before[4];
    for (int i = 0; i < 4; ++i)
        before[i] = axle_direction(solver, vehicle.corner(std::size_t(i)));

    const Scalar angle = Scalar(0.4);
    EXPECT_EQ(vehicle.set_steer_angle(solver, angle), 2u);
    run(solver, vehicle, 30);

    for (int i = 0; i < 4; ++i)
    {
        const Scalar turned =
            angle_between(before[i], axle_direction(solver, vehicle.corner(std::size_t(i))));
        if (i < 2)
            EXPECT_NEAR(double(turned), double(angle), 0.02) << "front corner " << i;
        else
            EXPECT_LT(double(turned), 0.01) << "rear corner " << i;
    }
}

/** @brief The axle is free until the brake is applied, and then it is not. */
TEST(Unit_VehicleSuspension, TheBrakeStopsAWheelThatOtherwiseKeepsSpinning)
{
    const Blob blob(chassis_asset());
    Scalar remaining[2] = {0, 0};

    for (int braked = 0; braked < 2; ++braked)
    {
        HostXpbdSolver<Scalar> solver(vehicle_scene());
        VehicleInstance vehicle;
        ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(8)),
                                   NodeBeamStructureSettings<Scalar>{}));
        for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
            plant(solver, vehicle.corner(i));

        // The planted wheel is given its mass back, because a wheel bolted to the world
        // cannot be slowed by anything and would report a brake that works whether or
        // not it does.
        RigidBody wheel;
        ASSERT_TRUE(solver.read_body(vehicle.corner(0).wheel(), wheel));
        wheel.inv_mass = Scalar(1) / Scalar(20);
        wheel.inv_inertia =
            Vector3{Scalar(1) / (Scalar(0.5) * Scalar(20) * Scalar(0.34) * Scalar(0.34)), 0, 0};
        wheel.angular_velocity = rotate(wheel.orientation, Vector3{1, 0, 0}) * Scalar(50);
        ASSERT_TRUE(solver.write_body(vehicle.corner(0).wheel(), wheel));

        if (braked != 0)
            EXPECT_EQ(vehicle.set_brake_torque(solver, Scalar(400)), 4u);

        run(solver, vehicle, 60);
        remaining[braked] = vehicle.corner(0).spin_rate(solver);
    }

    EXPECT_NEAR(double(remaining[0]), 50.0, 1e-6) << "a free axle must not slow at all";
    EXPECT_LT(std::fabs(double(remaining[1])), 25.0);
}

/** @brief A soft enough spring rides down onto its bump stop and no further. */
TEST(Unit_VehicleSuspension, TheBumpStopCatchesAStrutDrivenPastItsTravel)
{
    const Blob blob(chassis_asset());
    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    // Three kilonewtons per metre under two hundred kilograms a corner: the spring alone
    // would sink far past the twelve centimetres of travel it has.
    ASSERT_TRUE(vehicle.create(solver, blob.view, four_corners(Scalar(8), Scalar(3000)),
                               NodeBeamStructureSettings<Scalar>{}));
    for (std::size_t i = 0; i < vehicle.corner_count(); ++i)
        plant(solver, vehicle.corner(i));

    run(solver, vehicle, 240);

    const Scalar compression = vehicle.corner(0).compression(solver);
    EXPECT_GT(double(compression), 0.11) << "the strut never reached its stop";
    EXPECT_LT(double(compression), 0.121) << "the stop did not hold";
}

/** @brief §11.2's pure node-beam path has no core, and therefore cannot carry corners. */
TEST(Unit_VehicleSuspension, CornersRequireARigidCore)
{
    Cooking::NodeBeamAsset coreless = chassis_asset();
    coreless.core.mass = 0;
    coreless.attachments.clear();
    coreless.summary.total_mass = SHELL_MASS;
    const Blob blob(coreless);
    ASSERT_TRUE(blob.view.valid);

    HostXpbdSolver<Scalar> solver(vehicle_scene());
    VehicleInstance vehicle;
    EXPECT_FALSE(vehicle.create(solver, blob.view, four_corners(Scalar(8)),
                                NodeBeamStructureSettings<Scalar>{}));
    EXPECT_EQ(vehicle.corner_count(), 0u);
    EXPECT_EQ(vehicle.structure().node_count(), 0u);

    // The same asset with no corners is a perfectly good vehicle: it is the pure
    // node-beam one §11.2 keeps the door open for.
    EXPECT_TRUE(
        vehicle.create(solver, blob.view, VehicleAsset{}, NodeBeamStructureSettings<Scalar>{}));
}

/**
 * @brief A budget that runs out part way through a vehicle gives every slot back.
 *
 * Twelve bodies is one short of a four-corner car — a core, four shell nodes, and a hub
 * and a wheel per corner — so the last corner cannot be placed. Measured by what fits
 * afterwards rather than by a flag.
 */
TEST(Unit_VehicleSuspension, RefusalLeavesTheSolverEmpty)
{
    const Blob blob(chassis_asset());
    HostXpbdSolver<Scalar> solver(vehicle_scene(12));

    VehicleInstance refused;
    ASSERT_FALSE(refused.create(solver, blob.view, four_corners(Scalar(8)),
                                NodeBeamStructureSettings<Scalar>{}));
    EXPECT_EQ(refused.corner_count(), 0u);

    VehicleAsset one = four_corners(Scalar(8));
    one.corners.resize(1);
    VehicleInstance accepted;
    EXPECT_TRUE(accepted.create(solver, blob.view, one, NodeBeamStructureSettings<Scalar>{}));
}
