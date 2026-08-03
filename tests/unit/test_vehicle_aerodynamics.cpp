/**************************************************************************/
/* test_vehicle_aerodynamics.cpp                                          */
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
 * @file test_vehicle_aerodynamics.cpp
 * @brief §11.6 held to two properties, one of which is about doing nothing.
 *
 * 1. **Still air must cost exactly nothing.** The wind term is a *difference* against the
 *    still-air drag `predict` applies anyway, so a scene with no weather installed has to
 *    be bit-for-bit the scene that had no wind seam at all. A term that returned a small
 *    number instead of zero would perturb every determinism test in the suite, and it
 *    would do it by an amount small enough to be blamed on anything.
 * 2. **A body moving with the air is a body in still air.** That is the whole physical
 *    content of the difference form, and it is a closed form: the correction must be the
 *    exact negation of what `predict` is about to apply.
 *
 * Downforce is checked against `½ρClAv²` and against its lever — a wing off the centre of
 * mass pitches the car, and one on it does not, which is the difference between modelling
 * aerodynamic balance and modelling extra grip.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/aero/wind.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/vehicle/vehicle_instance.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar CORE_MASS = Scalar(800);
    constexpr Scalar SHELL_MASS = Scalar(80);
    constexpr Scalar NODE_DRAG_AREA = Scalar(0.35);
    constexpr Scalar FRONTAL_AREA = Scalar(2.2);
    constexpr Scalar BODY_DRAG = Scalar(0.32);
    constexpr Scalar AIR_DENSITY = Scalar(1.225);
    constexpr Scalar TICK = Scalar(1) / Scalar(60);

    RigidBody moving_body(Scalar speed, Scalar drag)
    {
        RigidBody body;
        body.velocity = Vector3{0, 0, speed};
        body.drag_coefficient = drag;
        body.flags = BodyFlags::dynamic_body;
        return body;
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
            node.drag_area = NODE_DRAG_AREA;
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
        asset.core.principal_inertia = Vector3{600, 900, 400};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
        asset.summary.node_mass = SHELL_MASS;
        asset.summary.total_mass = SHELL_MASS + CORE_MASS;
        asset.summary.part_count = 1;
        return asset;
    }

    VehicleAsset a_car(Scalar downforce_coefficient = 0, Scalar pressure_z = 0)
    {
        VehicleAsset asset;
        const Scalar x[4] = {Scalar(-0.75), Scalar(0.75), Scalar(0.75), Scalar(-0.75)};
        const Scalar z[4] = {Scalar(-1.3), Scalar(-1.3), Scalar(1.3), Scalar(1.3)};
        for (int i = 0; i < 4; ++i)
        {
            SuspensionSetup setup;
            setup.mount = Vector3{x[i], Scalar(0.55), z[i]};
            setup.axle = Vector3{1, 0, 0};
            setup.steered = z[i] < 0;
            asset.corners.push_back(setup);
        }
        asset.aerodynamics.frontal_area = FRONTAL_AREA;
        asset.aerodynamics.drag_coefficient = BODY_DRAG;
        asset.aerodynamics.downforce_coefficient = downforce_coefficient;
        asset.aerodynamics.center_of_pressure = Vector3{0, Scalar(0.6), pressure_z};
        return asset;
    }

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

/** @brief The drag constant is derived from the numbers an author actually has. */
TEST(Unit_VehicleAerodynamics, TheDragConstantIsDerivedAndNotAuthored)
{
    const Scalar k = quadratic_drag_constant<Scalar>(BODY_DRAG, FRONTAL_AREA, 900, AIR_DENSITY);
    const double expected = 0.5 * double(AIR_DENSITY * BODY_DRAG * FRONTAL_AREA) / 900.0;
    EXPECT_NEAR(double(k), expected, 1e-15);

    EXPECT_EQ(double(quadratic_drag_constant<Scalar>(BODY_DRAG, 0, 900, AIR_DENSITY)), 0.0);
    EXPECT_EQ(double(quadratic_drag_constant<Scalar>(BODY_DRAG, FRONTAL_AREA, 0, AIR_DENSITY)),
              0.0);
}

/**
 * @brief Still air costs exactly nothing.
 *
 * Not "nearly nothing". A wind term that perturbed a scene with no weather in it would
 * move every determinism test in the suite by an amount small enough to be blamed on
 * anything at all.
 */
TEST(Unit_VehicleAerodynamics, StillAirCostsExactlyNothing)
{
    const RigidBody body = moving_body(40, Scalar(0.0005));
    const Vector3 none = wind_drag_acceleration(body, Vector3{0, 0, 0});
    EXPECT_EQ(double(none.x), 0.0);
    EXPECT_EQ(double(none.y), 0.0);
    EXPECT_EQ(double(none.z), 0.0);

    const RigidBody dragless = moving_body(40, 0);
    const Vector3 unaffected = wind_drag_acceleration(dragless, Vector3{0, 0, 30});
    EXPECT_EQ(double(unaffected.z), 0.0) << "a body with no drag cannot feel wind";
}

/** @brief A body moving with the air is a body in still air, exactly. */
TEST(Unit_VehicleAerodynamics, MovingWithTheAirCancelsTheDragPredictWillApply)
{
    const Scalar drag = Scalar(0.0005);
    const RigidBody body = moving_body(40, drag);
    const Vector3 correction = wind_drag_acceleration(body, Vector3{0, 0, 40});

    // What predict is about to spend, and what must therefore be given back.
    const double assumed = -double(drag) * 40.0 * 40.0;
    EXPECT_NEAR(double(correction.z), -assumed, 1e-12);
}

/** @brief A headwind adds retardation and a tailwind takes it away, by the exact difference. */
TEST(Unit_VehicleAerodynamics, AHeadwindPushesHarderThanStillAir)
{
    const Scalar drag = Scalar(0.0005);
    const RigidBody body = moving_body(40, drag);

    const Vector3 head = wind_drag_acceleration(body, Vector3{0, 0, -20});
    const Vector3 tail = wind_drag_acceleration(body, Vector3{0, 0, 20});
    EXPECT_LT(double(head.z), 0.0);
    EXPECT_GT(double(tail.z), 0.0);

    const double expected = -double(drag) * 60.0 * 60.0 + double(drag) * 40.0 * 40.0;
    EXPECT_NEAR(double(head.z), expected, 1e-12);
}

/** @brief A crosswind blows a moving body downwind and costs it forward speed. */
TEST(Unit_VehicleAerodynamics, ACrosswindPushesSideways)
{
    const RigidBody body = moving_body(40, Scalar(0.0005));
    const Vector3 correction = wind_drag_acceleration(body, Vector3{15, 0, 0});
    EXPECT_GT(double(correction.x), 0.0) << "downwind";
    EXPECT_LT(double(correction.z), 0.0)
        << "the longer airspeed vector costs forward speed still air was not charging for";
}

/** @brief Aerodynamic force is quadratic in airspeed. */
TEST(Unit_VehicleAerodynamics, AerodynamicForceIsQuadraticInSpeed)
{
    const Scalar at_20 = aerodynamic_force<Scalar>(20, FRONTAL_AREA, Scalar(1.5), AIR_DENSITY);
    const Scalar at_40 = aerodynamic_force<Scalar>(40, FRONTAL_AREA, Scalar(1.5), AIR_DENSITY);
    EXPECT_NEAR(double(at_40 / at_20), 4.0, 1e-12);
    EXPECT_EQ(double(aerodynamic_force<Scalar>(40, FRONTAL_AREA, 0, AIR_DENSITY)), 0.0);
}

/** @brief The cooker's per-node area finally reaches a body (§11.6). */
TEST(Unit_VehicleAerodynamics, ShellNodesCarryTheCookersDragArea)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, a_car(), settings));

    RigidBody node;
    ASSERT_TRUE(solver.read_body(vehicle.structure().node(0), node));
    const Scalar expected =
        quadratic_drag_constant<Scalar>(Scalar(1.2), NODE_DRAG_AREA, SHELL_MASS / 4, AIR_DENSITY);
    EXPECT_NEAR(double(node.drag_coefficient), double(expected), 1e-15);
    EXPECT_GT(double(node.drag_coefficient), 0.0)
        << "the area travelled in the asset from P7-C and was read by nothing until now";
}

/** @brief The body's Cd and area become the core's own drag constant. */
TEST(Unit_VehicleAerodynamics, TheCoreCarriesTheBodysDrag)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, a_car(), settings));

    RigidBody core;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), core));
    const Scalar expected =
        quadratic_drag_constant<Scalar>(BODY_DRAG, FRONTAL_AREA, CORE_MASS, AIR_DENSITY);
    EXPECT_NEAR(double(core.drag_coefficient), double(expected), 1e-15)
        << "so a car meets a gust through the same path a flag on a pole does";
}

/** @brief Downforce presses the car onto the road by `½ρClAv²` and no more. */
TEST(Unit_VehicleAerodynamics, DownforcePressesTheCarOntoTheRoad)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    settings.velocity = Vector3{0, 0, 60};
    ASSERT_TRUE(vehicle.create(solver, blob.view, a_car(Scalar(1.8)), settings));

    RigidBody before;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), before));
    vehicle.begin_tick(solver, TICK);
    RigidBody after;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), after));

    const double applied = double((after.velocity.y - before.velocity.y) * CORE_MASS / TICK);
    const double expected =
        -double(aerodynamic_force<Scalar>(60, FRONTAL_AREA, Scalar(1.8), AIR_DENSITY));
    EXPECT_LT(applied, 0.0) << "down, not up";
    EXPECT_NEAR(applied, expected, std::fabs(expected) * 1e-9);
}

/**
 * @brief A wing off the centre of mass pitches the car, and one on it does not.
 *
 * The difference between modelling aerodynamic *balance* and modelling extra grip.
 */
TEST(Unit_VehicleAerodynamics, DownforceOffTheCentreOfMassPitchesTheCar)
{
    NodeBeamStructureSettings<Scalar> settings;
    settings.velocity = Vector3{0, 0, 60};

    HostXPBDSolver<Scalar> winged_solver(vehicle_scene());
    Blob winged_blob(chassis_asset());
    VehicleInstance winged;
    ASSERT_TRUE(winged.create(winged_solver, winged_blob.view,
                              a_car(Scalar(1.8), Scalar(-1.2)), settings));

    RigidBody before;
    ASSERT_TRUE(winged_solver.read_body(winged.structure().core(), before));
    winged.begin_tick(winged_solver, TICK);
    RigidBody after;
    ASSERT_TRUE(winged_solver.read_body(winged.structure().core(), after));
    EXPECT_GT(std::fabs(double(after.angular_velocity.x - before.angular_velocity.x)), 1e-6);

    HostXPBDSolver<Scalar> centred_solver(vehicle_scene());
    Blob centred_blob(chassis_asset());
    VehicleInstance centred;
    ASSERT_TRUE(centred.create(centred_solver, centred_blob.view, a_car(Scalar(1.8), 0),
                               settings));

    RigidBody centred_before;
    ASSERT_TRUE(centred_solver.read_body(centred.structure().core(), centred_before));
    centred.begin_tick(centred_solver, TICK);
    RigidBody centred_after;
    ASSERT_TRUE(centred_solver.read_body(centred.structure().core(), centred_after));
    EXPECT_NEAR(double(centred_after.angular_velocity.x),
                double(centred_before.angular_velocity.x), 1e-12);
}

/** @brief A road car generates no downforce, which is the honest default. */
TEST(Unit_VehicleAerodynamics, ARoadCarGeneratesNoDownforce)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene());
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    settings.velocity = Vector3{0, 0, 60};
    ASSERT_TRUE(vehicle.create(solver, blob.view, a_car(), settings));

    RigidBody before;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), before));
    vehicle.begin_tick(solver, TICK);
    RigidBody after;
    ASSERT_TRUE(solver.read_body(vehicle.structure().core(), after));

    EXPECT_EQ(double(after.velocity.y), double(before.velocity.y));
    EXPECT_EQ(double(after.angular_velocity.x), double(before.angular_velocity.x));
}
