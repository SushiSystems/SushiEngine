/**************************************************************************/
/* test_vehicle_acceptance.cpp                                            */
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
 * @file test_vehicle_acceptance.cpp
 * @brief P7's acceptance row, as one scene: *"a drivable vehicle that deforms permanently
 *        on impact and loses parts, at the §13.1 target, deterministic under replay."*
 *
 * Every earlier P7 file tests one piece against its own closed form. This one tests that
 * the pieces are a car. It builds a hybrid vehicle at the §13.1 shape — a rigid core, a
 * node-beam shell with a door as its own part, four corners on struts, a drivetrain — and
 * asks the four questions the roadmap row asks, in the roadmap's own words.
 *
 * **Drivable.** The throttle reaches the driven wheels through the clutch, the gearbox and
 * the differential, and they turn. Steering moves the front corners and not the rear.
 *
 * **Deforms permanently.** A beam loaded past its yield threshold creeps: after the impact
 * its rest length is *different*, which is what separates a dent from a spring.
 *
 * **Loses parts.** A part whose every mount and cross-part beam has broken is reported
 * detached, once.
 *
 * **Deterministic under replay.** Two runs of the same crash agree bit for bit on every
 * node position, every wheel speed, and the crankshaft.
 *
 * The §13.1 timing row is *measured and printed rather than asserted*, and that is a
 * deliberate limit rather than an omission: the target is quoted for "one desktop-class
 * GPU through SushiRuntime" and this suite runs the host reference solver on the CPU.
 * A number from one solver that the other cannot produce is a number nobody can compare
 * (§16's own words), so what is asserted here is the *shape* of the scene — that it really
 * is 400 nodes and 2 000 beams — and the cost is reported for a human to read.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/vehicle/vehicle_instance.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar CORE_MASS = Scalar(900);
    constexpr Scalar SHELL_MASS = Scalar(120);
    constexpr Scalar TICK = Scalar(1) / Scalar(60);

    /** @brief The door: its own part, so losing it is losing a part and not a node. */
    constexpr std::uint32_t DOOR_PART = 1;

    PhysicsConfiguration vehicle_scene(std::size_t bodies, std::size_t beams,
                                       std::size_t colors)
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = bodies;
        configuration.capacities.constraints = 256;
        configuration.capacities.contacts = 512;
        configuration.capacities.joints = bodies;
        configuration.capacities.beams = beams;
        configuration.capacities.colors = colors;
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 8;
        return configuration;
    }

    /**
     * @brief A chassis shell: a ring of hull nodes plus a door hung off two of them.
     *
     * Small on purpose — the behavioural questions do not need 400 nodes to answer, and a
     * scene small enough to reason about is a scene whose failures are diagnosable. The
     * §13.1 shape is built separately, below, where the question is cost.
     */
    Cooking::NodeBeamAsset chassis_asset()
    {
        Cooking::NodeBeamAsset asset;
        const Vector3 hull[6] = {{-0.8, 0.4, -1.6}, {0.8, 0.4, -1.6}, {0.8, 0.4, 1.6},
                                 {-0.8, 0.4, 1.6},  {0.0, 0.9, -0.8}, {0.0, 0.9, 0.8}};
        for (const Vector3& position : hull)
        {
            Cooking::NodeBeamNodeRecord node{};
            node.position = position;
            node.mass = SHELL_MASS / 8;
            node.radius = Scalar(0.06);
            node.drag_area = Scalar(0.3);
            asset.nodes.push_back(node);
        }

        // The door, on the near side, as part one.
        const Vector3 door[2] = {{-0.95, 0.65, -0.4}, {-0.95, 0.65, 0.4}};
        for (const Vector3& position : door)
        {
            Cooking::NodeBeamNodeRecord node{};
            node.position = position;
            node.mass = SHELL_MASS / 8;
            node.radius = Scalar(0.06);
            node.drag_area = Scalar(0.3);
            node.part = DOOR_PART;
            asset.nodes.push_back(node);
        }

        // A soft-ish hull ring so an impact has somewhere to go, and a stiff door hinge.
        const std::uint32_t ring[8][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                          {4, 0}, {4, 1}, {5, 2}, {5, 3}};
        for (const std::uint32_t(&pair)[2] : ring)
        {
            Cooking::NodeBeamBeamRecord beam{};
            beam.a = pair[0];
            beam.b = pair[1];
            beam.rest_length = length(hull[pair[1]] - hull[pair[0]]);
            beam.compliance = Scalar(3e-6);
            beam.damping = 4;
            beam.deform_force = 100;
            beam.break_force = 60000;
            beam.plastic_creep = Scalar(0.5);
            beam.maximum_plastic_strain = Scalar(0.35);
            asset.beams.push_back(beam);
        }

        // Two beams tie the door to the hull, and they are the only thing that does.
        const std::uint32_t hinge[2][2] = {{6, 0}, {7, 3}};
        for (const std::uint32_t(&pair)[2] : hinge)
        {
            const Vector3 from = asset.nodes[pair[0]].position;
            const Vector3 to = asset.nodes[pair[1]].position;
            Cooking::NodeBeamBeamRecord beam{};
            beam.a = pair[0];
            beam.b = pair[1];
            beam.rest_length = length(to - from);
            beam.compliance = Scalar(1e-8);
            beam.damping = 4;
            beam.deform_force = 700;
            beam.break_force = Scalar(2500);
            beam.plastic_creep = Scalar(0.5);
            beam.maximum_plastic_strain = Scalar(0.35);
            asset.beams.push_back(beam);
        }

        asset.core.mass = CORE_MASS;
        asset.core.principal_inertia = Vector3{700, 1100, 500};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};

        // Only the hull is bolted to the core. The door hangs off the hull, which is what
        // makes losing its two beams enough to lose it.
        for (std::uint32_t i = 0; i < 6; ++i)
        {
            Cooking::NodeBeamAttachmentRecord attachment{};
            attachment.node = i;
            attachment.core_anchor = hull[i];
            attachment.compliance = Scalar(1e-9);
            attachment.break_force = 400000;
            asset.attachments.push_back(attachment);
        }

        asset.summary.node_mass = SHELL_MASS;
        asset.summary.total_mass = SHELL_MASS + CORE_MASS;
        asset.summary.part_count = 2;
        return asset;
    }

    /** @brief A rear-wheel-drive car with front steering and a five-speed gearbox. */
    VehicleAsset road_car()
    {
        VehicleAsset asset;
        const Scalar x[4] = {Scalar(-0.75), Scalar(0.75), Scalar(0.75), Scalar(-0.75)};
        const Scalar z[4] = {Scalar(-1.4), Scalar(-1.4), Scalar(1.4), Scalar(1.4)};
        for (int i = 0; i < 4; ++i)
        {
            SuspensionSetup setup;
            setup.mount = Vector3{x[i], Scalar(0.6), z[i]};
            setup.axis = Vector3{0, 1, 0};
            setup.axle = Vector3{1, 0, 0};
            setup.spring_rate = 38000;
            setup.damping = 6;
            setup.steered = z[i] < 0;
            setup.driven = z[i] > 0;
            asset.corners.push_back(setup);
        }

        asset.powertrain.engine.curve = {{80, 130}, {220, 280}, {420, 310}, {620, 240}};
        asset.powertrain.engine.inertia = Scalar(0.22);
        asset.powertrain.gearbox.ratios = {Scalar(-3.2), 0, Scalar(3.4), Scalar(2.1),
                                           Scalar(1.4), 1, Scalar(0.8)};
        asset.powertrain.gearbox.final_drive = Scalar(3.9);
        asset.powertrain.differential.lock_torque = 200;
        asset.aerodynamics.downforce_coefficient = 0;
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

    /** @brief One whole tick, in the order `VehicleInstanceT` documents. */
    NodeBeamTickReport drive(HostXPBDSolver<Scalar>& solver, VehicleInstance& vehicle, int ticks)
    {
        StepParameters<Scalar> parameters;
        parameters.delta_time = TICK;
        parameters.gravity = Vector3{0, Scalar(-9.81), 0};

        NodeBeamTickReport total;
        for (int tick = 0; tick < ticks; ++tick)
        {
            vehicle.begin_tick(solver, TICK);
            solver.step(parameters);
            const NodeBeamTickReport report = vehicle.end_tick(solver);
            total.beams_deformed += report.beams_deformed;
            total.beams_broken += report.beams_broken;
            total.attachments_broken += report.attachments_broken;
            total.parts_detached += report.parts_detached;
        }
        return total;
    }

    /**
     * @brief Throws the door outward hard enough to tear it off.
     *
     * Gravity cannot load a vehicle — it is uniform, so a car with nothing under it falls
     * as one piece and every beam in it carries exactly nothing. A crash has to be a
     * *relative* velocity, and this is the smallest one that is: the door keeps going and
     * the car does not.
     */
    void tear_off_the_door(HostXPBDSolver<Scalar>& solver, VehicleInstance& vehicle,
                           Scalar speed)
    {
        for (std::size_t i = 6; i < 8; ++i)
        {
            RigidBody node;
            if (!solver.read_body(vehicle.structure().node(i), node))
                continue;
            node.velocity = Vector3{-speed, 0, 0};
            solver.write_body(vehicle.structure().node(i), node);
        }
    }
} // namespace

/** @brief Drivable: the throttle reaches the driven wheels and the steering the front ones. */
TEST(Integration_VehicleAcceptance, ItIsDrivable)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene(64, 64, 16));
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, road_car(), settings));
    ASSERT_EQ(vehicle.corner_count(), std::size_t(4));
    ASSERT_EQ(vehicle.driven_corner_count(), std::size_t(2));

    ASSERT_TRUE(vehicle.select_gear(2));
    vehicle.set_throttle(1);
    vehicle.set_clutch(1);
    EXPECT_EQ(vehicle.set_steer_angle(solver, Scalar(0.35)), std::size_t(2));

    drive(solver, vehicle, 40);

    EXPECT_GT(double(vehicle.corner(2).spin_rate(solver)), 3.0) << "rear right turns";
    EXPECT_GT(double(vehicle.corner(3).spin_rate(solver)), 3.0) << "rear left turns";
    // Not zero: the drive reaction turns the chassis, the carriers turn with it, and
    // `spin_rate` reads an *absolute* rate. What matters is that nothing is driving them.
    EXPECT_LT(std::fabs(double(vehicle.corner(0).spin_rate(solver))), 0.01)
        << "and nothing drives the front";
    EXPECT_GT(double(vehicle.powertrain().engine_rate()), 90.0) << "the engine is running";

    EXPECT_NEAR(double(vehicle.corner(0).steer_angle()), 0.35, 1e-12);
    EXPECT_EQ(double(vehicle.corner(2).steer_angle()), 0.0) << "and the rear is not steered";

    // Out of gear as well as off the throttle: braking with the clutch engaged is braking
    // against a flywheel at 680 rad/s, and the wheel settles where the two balance rather
    // than stopping. That is right, and it is not what is under test here.
    vehicle.set_throttle(0);
    vehicle.set_clutch(0);
    vehicle.set_brake_torque(solver, 4000);
    drive(solver, vehicle, 90);
    EXPECT_LT(double(vehicle.corner(2).spin_rate(solver)), 0.05) << "and the brake stops it";
}

/**
 * @brief Deforms permanently: after the impact the beam's rest length is *different*.
 *
 * The assertion that separates a dent from a spring. A structure that merely bent and
 * sprang back would pass every test about forces and fail this one.
 */
TEST(Integration_VehicleAcceptance, ItDeformsPermanently)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene(64, 64, 16));
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, road_car(), settings));

    BeamConstraint before;
    ASSERT_TRUE(solver.read_beam(vehicle.structure().beam(0), before));
    const Scalar rest_before = before.rest_length;

    // A wall: the core is stopped dead and the shell keeps going. Gravity cannot load a
    // vehicle — it is uniform, so a car with nothing under it falls as one piece and every
    // beam in it carries exactly nothing. A crash has to be a *relative* velocity.
    for (std::size_t i = 0; i < 6; ++i)
    {
        RigidBody node;
        ASSERT_TRUE(solver.read_body(vehicle.structure().node(i), node));
        node.velocity = Vector3{0, 0, Scalar(-25)};
        solver.write_body(vehicle.structure().node(i), node);
    }

    const NodeBeamTickReport report = drive(solver, vehicle, 60);
    EXPECT_GT(report.beams_deformed, 0u) << "something in the hull had to yield";

    BeamConstraint after;
    ASSERT_TRUE(solver.read_beam(vehicle.structure().beam(0), after));
    EXPECT_NE(double(after.rest_length), double(rest_before))
        << "a dent is a rest length that changed, not a spring that returned";
    // The magnitude is small — micrometres — because this shell is bolted to a rigid core
    // and cannot move far. The claim is that it is *permanent*, and permanence is not a
    // matter of degree.
    EXPECT_GT(double(after.accumulated_plastic_strain), 0.0);
    EXPECT_LE(double(after.accumulated_plastic_strain), double(after.maximum_plastic_strain))
        << "and it work-hardens at the strain the material was authored with";
}

/** @brief Loses parts: a door that has lost its last tie is reported detached, once. */
TEST(Integration_VehicleAcceptance, ItLosesParts)
{
    HostXPBDSolver<Scalar> solver(vehicle_scene(64, 64, 16));
    Blob blob(chassis_asset());
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, road_car(), settings));
    ASSERT_EQ(vehicle.structure().part_count(), std::size_t(2));
    ASSERT_FALSE(vehicle.structure().part_detached(DOOR_PART));

    tear_off_the_door(solver, vehicle, 60);
    const NodeBeamTickReport report = drive(solver, vehicle, 90);

    EXPECT_GE(report.beams_broken, 2u) << "both hinges have to go";
    EXPECT_EQ(report.parts_detached, 1u) << "reported exactly once, not once per tick";
    EXPECT_TRUE(vehicle.structure().part_detached(DOOR_PART));
    EXPECT_FALSE(vehicle.structure().part_detached(0)) << "the hull is still a car";

    // And the door's nodes are still bodies, still beamed to each other, now flying away.
    RigidBody door;
    ASSERT_TRUE(solver.read_body(vehicle.structure().node(6), door));
    EXPECT_LT(double(door.position.x), -1.0) << "it left";
}

/** @brief Deterministic under replay: two runs of the same crash agree bit for bit (§0.5). */
TEST(Integration_VehicleAcceptance, ReplayIsIdentical)
{
    std::vector<Vector3> traces[2];
    Scalar engine_rate[2] = {0, 0};
    Scalar wheel_rate[2] = {0, 0};

    for (int pass = 0; pass < 2; ++pass)
    {
        HostXPBDSolver<Scalar> solver(vehicle_scene(64, 64, 16));
        Blob blob(chassis_asset());
        VehicleInstance vehicle;
        NodeBeamStructureSettings<Scalar> settings;
        ASSERT_TRUE(vehicle.create(solver, blob.view, road_car(), settings));

        ASSERT_TRUE(vehicle.select_gear(2));
        vehicle.set_throttle(Scalar(0.8));
        vehicle.set_clutch(1);
        vehicle.set_steer_angle(solver, Scalar(0.2));
        tear_off_the_door(solver, vehicle, 55);
        drive(solver, vehicle, 120);

        vehicle.structure().refresh_node_positions(solver);
        traces[pass] = vehicle.structure().node_positions();
        engine_rate[pass] = vehicle.powertrain().engine_rate();
        wheel_rate[pass] = vehicle.corner(2).spin_rate(solver);
    }

    ASSERT_EQ(traces[0].size(), traces[1].size());
    for (std::size_t i = 0; i < traces[0].size(); ++i)
    {
        EXPECT_EQ(double(traces[0][i].x), double(traces[1][i].x)) << "node " << i;
        EXPECT_EQ(double(traces[0][i].y), double(traces[1][i].y)) << "node " << i;
        EXPECT_EQ(double(traces[0][i].z), double(traces[1][i].z)) << "node " << i;
    }
    EXPECT_EQ(double(engine_rate[0]), double(engine_rate[1]));
    EXPECT_EQ(double(wheel_rate[0]), double(wheel_rate[1]));
}

/**
 * @brief §13.1's hybrid vehicle, at its stated shape, with the cost printed.
 *
 * The target — 2 ms/tick — is quoted for one desktop-class GPU through SushiRuntime, and
 * this runs the host reference solver on a CPU. Asserting the number here would be
 * asserting a number the target was never about. What *is* asserted is that the scene is
 * really the one §13.1 names: 400 nodes, 2 000 beams, four corners and a powertrain, all
 * of it instanced and stepping. The cost is printed for a human to read, in the same form
 * `Unit_CollisionScale` prints its own.
 */
TEST(Integration_VehicleAcceptance, TheHybridVehicleSceneIsTheOne13Point1Names)
{
    constexpr std::uint32_t NODES = 400;
    constexpr std::uint32_t BEAMS = 2000;

    Cooking::NodeBeamAsset asset;
    // A 10 x 10 x 4 lattice at 0.3 m spacing: a car-shaped cloud of the right size, whose
    // exact geometry does not matter because the question here is cost and not behaviour.
    for (std::uint32_t i = 0; i < NODES; ++i)
    {
        Cooking::NodeBeamNodeRecord node{};
        node.position = Vector3{Scalar(i % 10) * Scalar(0.3) - Scalar(1.35),
                                Scalar((i / 100) % 4) * Scalar(0.3) + Scalar(0.3),
                                Scalar((i / 10) % 10) * Scalar(0.3) - Scalar(1.35)};
        node.mass = SHELL_MASS / Scalar(NODES);
        node.radius = Scalar(0.05);
        node.drag_area = Scalar(0.05);
        asset.nodes.push_back(node);
    }
    // Five beams per node to the nodes above it in index order: a connected network of
    // exactly the stated size, without a topology argument this test does not need.
    for (std::uint32_t i = 0; asset.beams.size() < BEAMS; ++i)
    {
        const std::uint32_t a = i % NODES;
        const std::uint32_t b = (a + 1 + (i / NODES)) % NODES;
        if (a == b)
            continue;
        Cooking::NodeBeamBeamRecord beam{};
        beam.a = a;
        beam.b = b;
        beam.rest_length =
            length(asset.nodes[b].position - asset.nodes[a].position);
        beam.compliance = Scalar(1e-7);
        beam.damping = 4;
        beam.deform_force = 1200;
        beam.break_force = 90000;
        asset.beams.push_back(beam);
    }
    asset.core.mass = CORE_MASS;
    asset.core.principal_inertia = Vector3{700, 1100, 500};
    asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
    for (std::uint32_t i = 0; i < NODES; i += 8)
    {
        Cooking::NodeBeamAttachmentRecord attachment{};
        attachment.node = i;
        attachment.core_anchor = asset.nodes[i].position;
        attachment.compliance = Scalar(1e-9);
        attachment.break_force = 400000;
        asset.attachments.push_back(attachment);
    }
    asset.summary.node_mass = SHELL_MASS;
    asset.summary.total_mass = SHELL_MASS + CORE_MASS;
    asset.summary.part_count = 1;

    // Nodes, the core, and two bodies per corner, with room to spare for the joints.
    HostXPBDSolver<Scalar> solver(vehicle_scene(NODES + 16, BEAMS * 3, 128));
    Blob blob(asset);
    VehicleInstance vehicle;
    NodeBeamStructureSettings<Scalar> settings;
    ASSERT_TRUE(vehicle.create(solver, blob.view, road_car(), settings));

    ASSERT_EQ(vehicle.structure().live_beam_count(), std::size_t(BEAMS));
    ASSERT_EQ(vehicle.corner_count(), std::size_t(4));
    ASSERT_TRUE(vehicle.select_gear(2));
    vehicle.set_throttle(1);
    vehicle.set_clutch(1);

    drive(solver, vehicle, 10);   // warm the caches and the colouring

    constexpr int MEASURED = 60;
    const std::chrono::steady_clock::time_point began = std::chrono::steady_clock::now();
    drive(solver, vehicle, MEASURED);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
            .count();

    std::printf("[ vehicle  ] hybrid vehicle (%u nodes, %u beams, 4 wheels, powertrain): "
                "%.3f ms/tick on the host reference solver (§13.1 target is 2 ms/tick on "
                "one desktop-class GPU through SushiRuntime)\n",
                NODES, BEAMS, elapsed_ms / MEASURED);

    EXPECT_EQ(vehicle.structure().live_attachment_count(), std::size_t(50));
    EXPECT_GT(double(vehicle.corner(2).spin_rate(solver)), 0.0) << "and it is still driving";
}
