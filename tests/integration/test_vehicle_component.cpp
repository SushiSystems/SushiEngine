/**************************************************************************/
/* test_vehicle_component.cpp                                             */
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
 * @file test_vehicle_component.cpp
 * @brief §5.5's `VehicleInstance`: a cooked vehicle placed in a scene and driven from it.
 *
 * P7 built a drivable vehicle and proved it against a solver the tests constructed
 * themselves. What it could not do was reach a *scene*: there was no component naming a
 * `.sushinodebeam`, no service on `IPhysicsScene` to instance one, and nothing that put a
 * vehicle's pose back on an entity. These are the tests that the path from an author's
 * component to a car that moves is closed.
 *
 * The asset is cooked into a real file rather than handed over as bytes, because that is
 * the difference this phase is about: a scene file stores a *path*, and a path is the one
 * thing that survives being reopened on another machine.
 */

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include <SushiEngine/authoring/command_history.hpp>
#include "scene_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief The rigid core's mass, in kilograms; the shell's is spread over its nodes. */
    constexpr Scalar CORE_MASS = 900;

    /** @brief What the whole shell weighs, split evenly across its nodes. */
    constexpr Scalar SHELL_MASS = 120;

    /**
     * @brief The six shell nodes, in asset space.
     *
     * At namespace scope because two things need the same numbers: the cook that turns
     * them into node records, and the test that checks where those nodes ended up. A
     * second copy would let the two drift and the test would still pass.
     */
    const Vector3 HULL[6] = {{-0.8, 0.4, -1.6}, {0.8, 0.4, -1.6}, {0.8, 0.4, 1.6},
                             {-0.8, 0.4, 1.6},  {0.0, 0.9, -0.8}, {0.0, 0.9, 0.8}};

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
     * @brief A six-node hull with a rigid core: the smallest thing that is a vehicle.
     *
     * Small on purpose. Nothing here asks a question that needs four hundred nodes to
     * answer, and a structure small enough to reason about is one whose failures are
     * diagnosable — the §13.1 shape is measured in the acceptance suite, where the
     * question is cost rather than correctness.
     */
    Physics::Cooking::NodeBeamAsset hull_asset()
    {
        Physics::Cooking::NodeBeamAsset asset;
        for (const Vector3& position : HULL)
        {
            Physics::Cooking::NodeBeamNodeRecord node{};
            node.position = position;
            node.mass = SHELL_MASS / 6;
            node.radius = Scalar(0.06);
            node.drag_area = Scalar(0.3);
            asset.nodes.push_back(node);
        }

        const std::uint32_t ring[8][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0},
                                          {4, 0}, {4, 1}, {5, 2}, {5, 3}};
        for (const std::uint32_t(&pair)[2] : ring)
        {
            Physics::Cooking::NodeBeamBeamRecord beam{};
            beam.a = pair[0];
            beam.b = pair[1];
            beam.rest_length = length(HULL[pair[1]] - HULL[pair[0]]);
            beam.compliance = Scalar(3e-6);
            beam.damping = 4;
            beam.deform_force = 100;
            beam.break_force = 60000;
            beam.plastic_creep = Scalar(0.5);
            beam.maximum_plastic_strain = Scalar(0.35);
            asset.beams.push_back(beam);
        }

        asset.core.mass = CORE_MASS;
        asset.core.principal_inertia = Vector3{700, 1100, 500};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
        for (std::uint32_t i = 0; i < 6; ++i)
        {
            Physics::Cooking::NodeBeamAttachmentRecord attachment{};
            attachment.node = i;
            attachment.core_anchor = HULL[i];
            attachment.break_force = 40000;
            asset.attachments.push_back(attachment);
        }
        // Every record defaults to part 0 (a single connected structure), so the
        // summary's count has to say one part -- build_node_beam_blob() derives the
        // count from the records and refuses a summary that disagrees with them.
        asset.summary.part_count = 1;
        return asset;
    }

    /** @brief Four corners, rear-driven, front-steered, and a gearbox with a first gear. */
    Physics::VehicleAsset road_car()
    {
        Physics::VehicleAsset asset;
        const Scalar x[4] = {Scalar(-0.75), Scalar(0.75), Scalar(0.75), Scalar(-0.75)};
        const Scalar z[4] = {Scalar(-1.4), Scalar(-1.4), Scalar(1.4), Scalar(1.4)};
        for (int i = 0; i < 4; ++i)
        {
            Physics::SuspensionSetup setup;
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
        asset.powertrain.gearbox.ratios = {Scalar(-3.2), 0, Scalar(3.4), Scalar(2.1)};
        asset.powertrain.gearbox.final_drive = Scalar(3.9);
        asset.powertrain.differential.lock_torque = 200;
        asset.aerodynamics.downforce_coefficient = 0;
        return asset;
    }

    /**
     * @brief Cooks @ref hull_asset into a real file and hands back its path.
     *
     * A file rather than bytes, because a path is what this phase is about: the component
     * stores one, the scene file stores one, and the loading that turns it back into a
     * vehicle is the thing under test. Written once per process and reused — a temporary
     * that every test rewrote would be testing the filesystem.
     */
    const std::string& cooked_vehicle_path()
    {
        static const std::string path = []
        {
            std::vector<std::byte> bytes;
            if (!Physics::Cooking::build_node_beam_blob(hull_asset(), bytes))
                return std::string();
            const std::filesystem::path file =
                std::filesystem::temp_directory_path() / "se_test_vehicle.sushinodebeam";
            std::ofstream out(file, std::ios::binary);
            if (!out)
                return std::string();
            out.write(reinterpret_cast<const char*>(bytes.data()),
                      std::streamsize(bytes.size()));
            return out ? file.string() : std::string();
        }();
        return path;
    }

    /** @brief An entity carrying a Vehicle built from the cooked structure. */
    EntityId make_vehicle(IWorldEditor& world, const std::string& name, const Vector3& position,
                          const std::string& asset_path)
    {
        const EntityId id = world.create(name);
        EntityTransform transform = world.transform(id);
        transform.position = position;
        world.set_transform(id, transform);

        world.set_has_vehicle(id, true);
        VehicleInstanceParameters parameters;
        parameters.asset_path = asset_path;
        parameters.setup = road_car();
        world.set_vehicle_parameters(id, parameters);
        return id;
    }

    /**
     * @brief A plain dynamic box: the one thing in the scene already known to fall.
     *
     * The reference a vehicle is measured against. A constant would measure the tick
     * length and the substep schedule as much as the field, and both are the runtime's
     * to change; a body the scene samples through the path that already works measures
     * only whether the vehicle is sampled the same way.
     */
    EntityId make_falling_box(IWorldEditor& world, const std::string& name,
                              const Vector3& position)
    {
        const EntityId id = world.create_box(name);
        EntityTransform transform = world.transform(id);
        transform.position = position;
        transform.scale = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
        world.set_transform(id, transform);

        PhysicsBodyParameters body;
        body.density = Scalar(1000);
        world.set_has_physics_body(id, true);
        world.set_physics_body_parameters(id, body);
        return id;
    }

    /** @brief Runs @p count fixed steps. */
    void step(ISimulation& simulation, int count)
    {
        for (int i = 0; i < count; ++i)
            simulation.tick(simulation.fixed_dt_seconds());
    }
}

TEST(Integration_VehicleComponent, ACookedStructureBecomesALiveVehicle)
{
    ASSERT_FALSE(cooked_vehicle_path().empty()) << "the test asset failed to cook";

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId car = make_vehicle(world, "Car", Vector3{0, 20, 0}, cooked_vehicle_path());
    step(*simulation, 2);

    VehicleReport report;
    ASSERT_TRUE(world.vehicle_report(car, report))
        << "a named, loadable structure must produce a vehicle the boundary can read";

    // The shell reached the solver, which is the half a report alone would not prove: a
    // drivetrain with no structure under it would still report an engine speed.
    std::vector<Vector3> nodes;
    ASSERT_TRUE(world.vehicle_node_positions(car, nodes));
    EXPECT_EQ(nodes.size(), std::size_t(6));
}

TEST(Integration_VehicleComponent, TheEntityFollowsTheVehiclesCore)
{
    ASSERT_FALSE(cooked_vehicle_path().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId car = make_vehicle(world, "Car", Vector3{0, 40, 0}, cooked_vehicle_path());
    const EntityId control = world.create("Control");
    EntityTransform control_transform = world.transform(control);
    control_transform.position = Vector3{5, 40, 0};
    world.set_transform(control, control_transform);

    const Scalar start = world.transform(car).position.y;
    step(*simulation, 90);

    // The car falls because its core does, and the entity's Transform is written from it.
    // The control has no vehicle and no body, so it stays exactly where it was put — which
    // is what makes the first assertion about the write-back rather than about gravity
    // reaching every entity in the scene.
    EXPECT_GT(double(start - world.transform(car).position.y), 0.5)
        << "the entity follows its vehicle's rigid core";
    EXPECT_DOUBLE_EQ(double(world.transform(control).position.y), 40.0);
}

TEST(Integration_VehicleComponent, EveryBodyAVehicleOwnsTakesTheScenesGravityField)
{
    ASSERT_FALSE(cooked_vehicle_path().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    // Nothing is under either of them. No entity in this scene carries a Plane collider,
    // so what is measured is ten ticks of free fall and never a landing.
    const Vector3 spawn{0, 40, 0};
    const EntityId car = make_vehicle(world, "Car", spawn, cooked_vehicle_path());
    const EntityId weight = make_falling_box(world, "Weight", Vector3{10, 40, 0});

    step(*simulation, 10);

    const double reference = double(spawn.y) - double(world.transform(weight).position.y);
    ASSERT_GT(reference, 0.05) << "the reference body did not fall; the scene has no gravity";

    // The mechanism, not an accumulated distance: a vehicle's core is a rigid body in the
    // same solver, so over the same ten ticks it must fall exactly as far as one. Falling
    // a little rather than not at all is the failure this bounds -- the field reaching one
    // body of the vehicle and not the rest would still move the core some distance.
    const double core = double(spawn.y) - double(world.transform(car).position.y);
    EXPECT_NEAR(core, reference, reference * 0.02)
        << "the vehicle's core does not take the field the scene applies to a rigid body";

    // And the shell falls with it. The attachments would drag the nodes after a core that
    // fell alone, so this is the assertion that the whole body inventory is enumerated
    // rather than only its first entry.
    std::vector<Vector3> nodes;
    ASSERT_TRUE(world.vehicle_node_positions(car, nodes));
    ASSERT_EQ(nodes.size(), std::size_t(6));
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        const double node = double(spawn.y) + double(HULL[i].y) - double(nodes[i].y);
        EXPECT_NEAR(node, core, 5e-3) << "shell node " << i << " did not fall with the core";
    }
}

TEST(Integration_VehicleComponent, ThrottleReachesTheEngineThroughTheBoundary)
{
    ASSERT_FALSE(cooked_vehicle_path().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId car = make_vehicle(world, "Car", Vector3{0, 5, 0}, cooked_vehicle_path());
    step(*simulation, 2);

    VehicleReport idling;
    ASSERT_TRUE(world.vehicle_report(car, idling));

    VehicleInput input;
    input.throttle = 1;
    input.clutch = 0;   // Free the engine, so what is measured is the engine and not the car.
    input.gear = 2;
    ASSERT_TRUE(world.set_vehicle_input(car, input));
    step(*simulation, 60);

    VehicleReport revving;
    ASSERT_TRUE(world.vehicle_report(car, revving));
    EXPECT_GT(double(revving.engine_rate), double(idling.engine_rate) + 50.0)
        << "full throttle against a disconnected clutch must spin the crankshaft up";

    // And the input is *held*, not consumed: a caller that stops asking keeps the pedal
    // where it left it, which is what a pedal does and what makes this callable from a UI
    // slider and an input action without the two fighting.
    EXPECT_DOUBLE_EQ(double(world.vehicle_input(car).throttle), 1.0);

    input.throttle = 0;
    ASSERT_TRUE(world.set_vehicle_input(car, input));
    step(*simulation, 90);
    VehicleReport coasting;
    ASSERT_TRUE(world.vehicle_report(car, coasting));
    EXPECT_LT(double(coasting.engine_rate), double(revving.engine_rate))
        << "and lifting off must let it fall back";
}

TEST(Integration_VehicleComponent, AnAssetThatDoesNotLoadIsReportedRatherThanGuessedAt)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    // Three ways to have no vehicle, all of which look like a stationary car from outside
    // and all of which the panel has to be able to tell apart.
    const EntityId unnamed = world.create("Unnamed");
    world.set_has_vehicle(unnamed, true);

    const EntityId missing = make_vehicle(world, "Missing", Vector3{0, 5, 0},
                                          "does/not/exist.sushinodebeam");

    const EntityId bare = world.create("NoComponent");

    step(*simulation, 4);

    VehicleReport report;
    EXPECT_FALSE(world.vehicle_report(unnamed, report)) << "authoring in progress is not live";
    EXPECT_FALSE(world.vehicle_report(missing, report)) << "a path that did not load is not live";
    EXPECT_FALSE(world.vehicle_report(bare, report));

    // The authoring survives all three, because none of them is an author's mistake being
    // corrected for them.
    EXPECT_TRUE(world.has_vehicle(unnamed));
    EXPECT_TRUE(world.has_vehicle(missing));
    EXPECT_EQ(world.vehicle_parameters(missing).asset_path, "does/not/exist.sushinodebeam");
}

TEST(Integration_VehicleComponent, TheStructurePathSurvivesTheSceneFile)
{
    ASSERT_FALSE(cooked_vehicle_path().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    make_vehicle(world, "Car", Vector3{1, 2, 3}, cooked_vehicle_path());

    const nlohmann::json snapshot = Scene::capture_scene(world);
    clear_world(world);
    Scene::apply_scene(world, snapshot);

    const EntityId restored = find_by_name(world, "Car");
    ASSERT_NE(restored, NULL_ENTITY);
    ASSERT_TRUE(world.has_vehicle(restored));
    EXPECT_EQ(world.vehicle_parameters(restored).asset_path, cooked_vehicle_path());

    // And it is live again after the reload, which is the clause a stored path exists for:
    // a scene reopened somewhere else finds its car.
    step(*simulation, 2);
    VehicleReport report;
    EXPECT_TRUE(world.vehicle_report(restored, report));
}

TEST(Integration_VehicleComponent, DetachingTheComponentTakesTheVehicleWithIt)
{
    ASSERT_FALSE(cooked_vehicle_path().empty());

    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId car = make_vehicle(world, "Car", Vector3{0, 5, 0}, cooked_vehicle_path());
    step(*simulation, 2);
    VehicleReport report;
    ASSERT_TRUE(world.vehicle_report(car, report));

    world.set_has_vehicle(car, false);
    step(*simulation, 2);
    EXPECT_FALSE(world.vehicle_report(car, report));

    std::vector<Vector3> nodes;
    EXPECT_FALSE(world.vehicle_node_positions(car, nodes))
        << "its six bodies are gone from the solver, not merely unreachable";
}
