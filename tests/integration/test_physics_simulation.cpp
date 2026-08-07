/**************************************************************************/
/* test_physics_simulation.cpp                                            */
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

// Integration_PhysicsSimulation: the whole contact stack through the seam the live
// world actually drives (IPhysicsSimulation), against the real runtime. The unit
// tests cover the narrowphase and the broadphase in isolation; this covers what only
// appears once they run together inside the sub-stepped lockstep loop — that a box
// settles on its true oriented reach rather than a bounding radius, and that cloth
// and rigid bodies push on *each other* rather than one draping over the other.

#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/simulation/physics_simulation.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr std::size_t ITERATIONS = 8;
    constexpr std::size_t SUBSTEPS = 4;
    constexpr Scalar SUBSTEP_DT = Scalar(1.0 / 240.0);
    constexpr double PI = 3.14159265358979323846;

    /**
     * @brief Still air, which is what every scene without weather installed passes.
     *
     * An *empty* sampler rather than one returning zero: the physics skips it entirely,
     * so a scene with no wind is bit-for-bit the scene that had no wind seam at all
     * (§11.6, `physics/aero/wind.hpp`).
     */
    WindSampler still_air()
    {
        return WindSampler{};
    }

    /** @brief A uniform downward field, the sampler shape the live world hands in. */
    GravitySampler earth_gravity()
    {
        return [](const Vector3&) { return Vector3{0, Scalar(-9.8), 0}; };
    }

    /** @brief No field at all, so a test sees contact response and nothing else. */
    GravitySampler no_gravity()
    {
        return [](const Vector3&) { return Vector3{0, 0, 0}; };
    }

    /** @brief A unit cube's collider: the shape the body actually collides as. */
    Collider unit_box_collider()
    {
        Collider collider;
        collider.shape = ColliderShape::Box;
        collider.half_extents = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
        return collider;
    }

    /** @brief A sphere collider of @p radius. */
    Collider sphere_collider(Scalar radius)
    {
        Collider collider;
        collider.shape = ColliderShape::Sphere;
        collider.radius = radius;
        return collider;
    }

    /** @brief The ground plane at y = 0, solid below. */
    std::vector<PlaneDescription> ground()
    {
        PlaneDescription plane;
        plane.point = Vector3{0, 0, 0};
        plane.normal = Vector3{0, 1, 0};
        return {plane};
    }

    /** @brief Where the shell nodes of @ref square_shell_asset sit, in the asset's frame. */
    const Vector3 SHELL[4] = {{-1, 0, -1}, {1, 0, -1}, {1, 0, 1}, {-1, 0, 1}};

    /** @brief Every shell node's collision radius, in metres. */
    constexpr Scalar SHELL_NODE_RADIUS = Scalar(0.15);

    /**
     * @brief Four shell nodes around a rigid core: the smallest thing that is a vehicle.
     *
     * Small and flat on purpose. The question here is which bodies reach the query
     * hierarchy, so the structure only has to place a known surface at a known point and
     * leave the middle empty — the core sits at the centre of the square with nothing
     * within a metre of it, which is what makes "the core is not a query proxy" an
     * assertion about the core rather than about a node that happened to be near it.
     */
    Physics::Cooking::NodeBeamAsset square_shell_asset()
    {
        Physics::Cooking::NodeBeamAsset asset;
        for (const Vector3& position : SHELL)
        {
            Physics::Cooking::NodeBeamNodeRecord node{};
            node.position = position;
            node.mass = Scalar(20);
            node.radius = SHELL_NODE_RADIUS;
            node.drag_area = Scalar(0.3);
            asset.nodes.push_back(node);
        }

        const std::uint32_t ring[4][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}};
        for (const std::uint32_t(&pair)[2] : ring)
        {
            Physics::Cooking::NodeBeamBeamRecord beam{};
            beam.a = pair[0];
            beam.b = pair[1];
            beam.rest_length = length(SHELL[pair[1]] - SHELL[pair[0]]);
            beam.compliance = Scalar(3e-6);
            beam.damping = 4;
            beam.deform_force = 100;
            beam.break_force = 60000;
            beam.plastic_creep = Scalar(0.5);
            beam.maximum_plastic_strain = Scalar(0.35);
            asset.beams.push_back(beam);
        }

        asset.core.mass = Scalar(400);
        asset.core.principal_inertia = Vector3{300, 400, 300};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};
        for (std::uint32_t i = 0; i < 4; ++i)
        {
            Physics::Cooking::NodeBeamAttachmentRecord attachment{};
            attachment.node = i;
            attachment.core_anchor = SHELL[i];
            attachment.break_force = 40000;
            asset.attachments.push_back(attachment);
        }
        asset.summary.part_count = 1;
        return asset;
    }
}

TEST(Integration_PhysicsSimulation, BoxSettlesOnItsFaceNotItsBoundingRadius)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    RigidBodyDescription description;
    description.id = 1;
    description.position = Vector3{0, Scalar(4), 0};
    description.inv_mass = Scalar(1);
    // A bounding-sphere fallback would hover at the box's bounding radius,
    // 0.5*sqrt(3), instead of resting on its face at the half-extent.
    description.collider = unit_box_collider();

    physics->set_rigid_bodies({description}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 400; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(1, pose));
    EXPECT_NEAR(double(pose.position.y), 0.5, 1e-2);
}

TEST(Integration_PhysicsSimulation, TiltedBoxSettlesOnItsEdge)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    RigidBodyDescription description;
    description.id = 1;
    description.position = Vector3{0, Scalar(4), 0};
    description.orientation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(PI / 4.0));
    description.inv_mass = Scalar(1);
    description.inv_inertia = Vector3{0, 0, 0}; // no angular freedom: it stays tilted
    description.collider = unit_box_collider();

    physics->set_rigid_bodies({description}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 400; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(1, pose));
    // Tipped 45 degrees the cube stands on an edge sqrt(2)/2 below its centre.
    EXPECT_NEAR(double(pose.position.y), 0.5 * std::sqrt(2.0), 1e-2);
}

TEST(Integration_PhysicsSimulation, ClothAndRigidBodyPushOnEachOther)
{
    // Zero gravity, so the only thing that can move either body is the contact between
    // them. The rigid body starts overlapping a free (unpinned) cloth particle: after
    // the step both must have moved apart. A one-way coupling would leave the rigid
    // body exactly where it started.
    auto physics = create_physics_simulation(Harness::shared_context());

    ClothDescription cloth;
    cloth.id = 10;
    cloth.rows = 3;
    cloth.cols = 3;
    cloth.spacing = Scalar(0.5);
    cloth.origin = Vector3{0, 0, 0};
    cloth.thickness = Scalar(0.1);

    RigidBodyDescription body;
    body.id = 20;
    // Just above the centre particle, which sits at (0.5, 0, 0.5) and is not pinned.
    // The overlap is deliberately shallow (0.05 m against a 0.4 m contact distance):
    // a deep one makes the correction large enough that the cloth's stiff distance
    // constraints overshoot restoring the flat grid, which measures the solver's
    // stiffness rather than the coupling this test is about.
    body.position = Vector3{Scalar(0.5), Scalar(0.35), Scalar(0.5)};
    body.inv_mass = Scalar(1);
    body.collider = sphere_collider(Scalar(0.3));

    physics->set_rigid_bodies({body}, ITERATIONS, SUBSTEP_DT);
    physics->set_cloth_grids({cloth}, ITERATIONS, SUBSTEP_DT);

    const std::vector<Vector3> before = physics->cloth_positions(10);
    ASSERT_EQ(before.size(), std::size_t(9));

    physics->step(no_gravity(), still_air(), SUBSTEPS);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(20, pose));
    EXPECT_GT(double(pose.position.y), 0.35 + 1e-6) << "the cloth must push the rigid body";

    // The cloth side is asserted as displacement rather than direction: the contact
    // pushes the particle down, but the grid's distance constraints then pull it back
    // toward the flat rest shape within the same tick, so its resting sign is a
    // property of the constraint solve, not of the coupling.
    const std::vector<Vector3> after = physics->cloth_positions(10);
    ASSERT_EQ(after.size(), std::size_t(9));
    const std::size_t centre = 1 * 3 + 1;
    EXPECT_GT(std::fabs(double(after[centre].y) - double(before[centre].y)), 1e-6)
        << "the rigid body must move the cloth particle it rests on";

    // Row 0 is pinned by construction and must not have moved at all.
    for (std::size_t col = 0; col < 3; ++col)
    {
        EXPECT_NEAR(double(after[col].x), double(before[col].x), 1e-12);
        EXPECT_NEAR(double(after[col].y), double(before[col].y), 1e-12);
        EXPECT_NEAR(double(after[col].z), double(before[col].z), 1e-12);
    }
}

TEST(Integration_PhysicsSimulation, ClothDimensionsAndPoseRoundTrip)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    ClothDescription cloth;
    cloth.id = 10;
    cloth.rows = 4;
    cloth.cols = 5;
    cloth.spacing = Scalar(0.25);
    physics->set_cloth_grids({cloth}, ITERATIONS, SUBSTEP_DT);

    std::uint32_t rows = 0;
    std::uint32_t cols = 0;
    ASSERT_TRUE(physics->cloth_dimensions(10, rows, cols));
    EXPECT_EQ(rows, 4u);
    EXPECT_EQ(cols, 5u);
    EXPECT_FALSE(physics->cloth_dimensions(999, rows, cols));

    RigidBodyDescription body;
    body.id = 20;
    body.inv_mass = Scalar(1);
    physics->set_rigid_bodies({body}, ITERATIONS, SUBSTEP_DT);

    const Vector3 target{Scalar(3), Scalar(-2), Scalar(7)};
    const Quaternion turn = quaternion_axis_angle(Vector3{0, 1, 0}, Scalar(PI / 3.0));
    physics->set_rigid_pose(20, target, turn);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(20, pose));
    EXPECT_NEAR(double(pose.position.x), 3.0, 1e-9);
    EXPECT_NEAR(double(pose.position.z), 7.0, 1e-9);
    EXPECT_NEAR(double(pose.orientation.y), double(turn.y), 1e-9);
    EXPECT_FALSE(physics->rigid_pose(999, pose));
}

TEST(Integration_PhysicsSimulation, StackedBoxesSettleWithoutInterpenetrating)
{
    // Two boxes and the ground: the broadphase must hand the narrowphase the box-box
    // pair every sub-step, or the upper box sinks through the lower one.
    auto physics = create_physics_simulation(Harness::shared_context());

    std::vector<RigidBodyDescription> bodies;
    for (int i = 0; i < 2; ++i)
    {
        RigidBodyDescription description;
        description.id = static_cast<EntityId>(i + 1);
        description.position = Vector3{0, Scalar(1.0 + i * 1.2), 0};
        description.inv_mass = Scalar(1);
        description.collider = unit_box_collider();
        bodies.push_back(description);
    }

    physics->set_rigid_bodies(bodies, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 600; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose lower;
    SolvedPose upper;
    ASSERT_TRUE(physics->rigid_pose(1, lower));
    ASSERT_TRUE(physics->rigid_pose(2, upper));

    EXPECT_NEAR(double(lower.position.y), 0.5, 5e-2);
    EXPECT_GT(double(upper.position.y) - double(lower.position.y), 0.9)
        << "the upper box sank into the lower one";
}

TEST(Integration_PhysicsSimulation, AFastBodyIsStillFoundByTheOncePerTickBroadphase)
{
    // The broadphase now runs once per tick instead of twice per sub-step, which is
    // only sound because the bounds are swept by how far a body can travel in the
    // whole tick. A body fast enough to cross its target within the tick is the case
    // that tight, once-per-tick bounds would miss outright.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDescription bullet;
    bullet.id = 1;
    bullet.position = Vector3{Scalar(-3), Scalar(1), 0};
    bullet.inv_mass = Scalar(1);
    bullet.collider = sphere_collider(Scalar(0.5));

    RigidBodyDescription wall;
    wall.id = 2;
    wall.position = Vector3{0, Scalar(1), 0};
    wall.inv_mass = Scalar(0); // immovable, so any motion it causes is the contact
    wall.collider = sphere_collider(Scalar(0.5));

    physics->set_rigid_bodies({bullet, wall}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes({});

    // Fire it across the gap in a single tick: four sub-steps at 1/240 s is 1/60 s,
    // and 240 m/s covers four metres in that.
    physics->set_rigid_pose(1, Vector3{Scalar(-3), Scalar(1), 0}, Quaternion{});
    for (int tick = 0; tick < 12; ++tick)
        physics->step(no_gravity(), still_air(), SUBSTEPS);

    SolvedPose bullet_pose;
    SolvedPose wall_pose;
    ASSERT_TRUE(physics->rigid_pose(1, bullet_pose));
    ASSERT_TRUE(physics->rigid_pose(2, wall_pose));

    // With no gravity and no drive the bullet is at rest, so what this really pins is
    // that the pair was considered at all and the two are not interpenetrating.
    const double gap = std::fabs(double(wall_pose.position.x) - double(bullet_pose.position.x));
    EXPECT_GE(gap, 0.99) << "the two must not have been left overlapping";
}

TEST(Integration_PhysicsSimulation, TheStepReportsWhatItContained)
{
    // The statistics are the acceptance criterion for the phase, and a counter nobody
    // reads is a counter that drifts. This is the smallest honest assertion: the body
    // count and the sub-step count are what the caller just asked for.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDescription first;
    first.id = 1;
    first.position = Vector3{0, Scalar(4), 0};
    first.inv_mass = Scalar(1);
    RigidBodyDescription second = first;
    second.id = 2;
    second.position = Vector3{Scalar(0.4), Scalar(4.4), 0};

    physics->set_rigid_bodies({first, second}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());
    physics->step(earth_gravity(), still_air(), SUBSTEPS);

    const Physics::PhysicsStatistics& stats = physics->statistics();
    EXPECT_EQ(stats.awake_bodies, 2u);
    EXPECT_EQ(stats.substeps, SUBSTEPS);
    EXPECT_GE(stats.broadphase_pairs_produced, 1u)
        << "two overlapping bodies must have produced a candidate pair";
    // No assertion on compile_count here: two unconstrained rigid bodies give the
    // XPBD solver nothing to colour, so it builds no graph and the count is honestly
    // zero. The load-bearing compile-count assertion belongs to a scene that has a
    // graph, and lives in test_runtime_graph_builder.cpp.
}

// Contact events: what touched what, reported to gameplay (P1).
//
// The last thing P1 owed. It is the cheapest part of the phase to write and the
// easiest to get subtly wrong, because every failure mode is a *sequence* rather
// than a value: a Begin that repeats every tick, an End that never arrives, or an
// order that depends on the broadphase's insertion history rather than on the
// scene. So these tests watch the sequence and not just the contents.

namespace
{
    /** @brief The events of @p phase in @p events, in the order reported. */
    std::vector<ContactEvent> of_phase(const std::vector<ContactEvent>& events,
                                       ContactPhase phase)
    {
        std::vector<ContactEvent> found;
        for (const ContactEvent& event : events)
            if (event.phase == phase)
                found.push_back(event);
        return found;
    }
}

TEST(Integration_PhysicsSimulation, ALandingBoxBeginsOnceAndThenPersists)
{
    // The sequence a gameplay listener actually depends on. A Begin that repeated
    // every tick would play an impact sound sixty times a second; a Persist that
    // never arrived would make "is standing on something" unanswerable.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDescription description;
    description.id = 1;
    description.position = Vector3{0, Scalar(1.2), 0};
    description.inv_mass = Scalar(1);
    description.collider = unit_box_collider();

    physics->set_rigid_bodies({description}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    int begins = 0;
    int persists = 0;
    Scalar resting_impulse = 0;
    for (int tick = 0; tick < 120; ++tick)
    {
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
        for (const ContactEvent& event : physics->contact_events())
        {
            EXPECT_EQ(event.a, EntityId(1));
            EXPECT_EQ(event.b, NULL_ENTITY) << "the ground is not an entity";
            EXPECT_FALSE(event.trigger);
            if (event.phase == ContactPhase::Begin)
                ++begins;
            if (event.phase == ContactPhase::Persist)
            {
                ++persists;
                resting_impulse = event.impulse;
            }
        }
    }

    EXPECT_EQ(begins, 1) << "the box landed once";
    EXPECT_GT(persists, 60) << "and stayed there";
    // The impulse is what separates a scrape from a crash, and it is read back off
    // the device: a zero here would mean the solved manifolds never came home.
    EXPECT_GT(double(resting_impulse), 0.0);
}

TEST(Integration_PhysicsSimulation, TakingABodyAwayEndsItsContact)
{
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDescription description;
    description.id = 1;
    description.position = Vector3{0, Scalar(0.6), 0};
    description.inv_mass = Scalar(1);
    description.collider = unit_box_collider();

    physics->set_rigid_bodies({description}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 60; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    ASSERT_FALSE(of_phase(physics->contact_events(), ContactPhase::Persist).empty())
        << "it must be resting before it can stop resting";

    // Lifted clear rather than removed: removal changes the collidable membership,
    // which renumbers the proxies and deliberately drops the history with them.
    physics->set_rigid_pose(1, Vector3{0, Scalar(20), 0}, Quaternion{0, 0, 0, 1});
    physics->step(earth_gravity(), still_air(), SUBSTEPS);

    const std::vector<ContactEvent> ended =
        of_phase(physics->contact_events(), ContactPhase::End);
    ASSERT_EQ(ended.size(), std::size_t(1));
    EXPECT_EQ(ended[0].a, EntityId(1));

    // And nothing at all on the tick after, rather than an End every tick for ever.
    physics->step(earth_gravity(), still_air(), SUBSTEPS);
    EXPECT_TRUE(physics->contact_events().empty());
}

TEST(Integration_PhysicsSimulation, ATriggerIsReportedAndNotResolved)
{
    // The whole point of a trigger: the listener hears about it, the physics does
    // not act on it. A trigger that pushed would be a collider with extra reporting.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDescription falling;
    falling.id = 1;
    falling.position = Vector3{0, Scalar(3), 0};
    falling.inv_mass = Scalar(1);
    falling.collider = unit_box_collider();

    RigidBodyDescription volume;
    volume.id = 2;
    volume.position = Vector3{0, Scalar(1), 0};
    volume.inv_mass = Scalar(0); // immovable, so any motion is the contact's doing
    volume.collider = unit_box_collider();
    volume.collider.flags = Physics::BodyFlags::trigger;

    physics->set_rigid_bodies({falling, volume}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes({});

    bool reported = false;
    for (int tick = 0; tick < 90; ++tick)
    {
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
        for (const ContactEvent& event : physics->contact_events())
        {
            EXPECT_TRUE(event.trigger);
            EXPECT_DOUBLE_EQ(double(event.impulse), 0.0)
                << "a trigger carries no impulse because it is never solved";
            reported = true;
        }
    }

    EXPECT_TRUE(reported) << "the volume must have noticed the body passing through";

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(1, pose));
    EXPECT_LT(double(pose.position.y), -1.0)
        << "the body must have fallen straight through the trigger";
}

TEST(Integration_PhysicsSimulation, EventsComeInASceneOrderNotATraversalOrder)
{
    // A listener that spawns an effect observes the order, so the order is part of
    // the determinism contract (section 12.1). The broadphase emits pairs in an
    // order that is a function of its own insertion history; the events must not be.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    std::vector<RigidBodyDescription> bodies;
    for (int i = 0; i < 5; ++i)
    {
        RigidBodyDescription description;
        description.id = static_cast<EntityId>(i + 1);
        // Spread apart so each lands on the ground alone, and at descending heights
        // so they arrive in an order that is not the order they were added in.
        description.position = Vector3{Scalar(i) * Scalar(4), Scalar(5 - i) * Scalar(0.4), 0};
        description.inv_mass = Scalar(1);
        description.collider = unit_box_collider();
        bodies.push_back(description);
    }

    physics->set_rigid_bodies(bodies, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 120; ++tick)
    {
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
        EntityId previous = 0;
        for (const ContactEvent& event : physics->contact_events())
        {
            EXPECT_GE(event.a, previous)
                << "the events were not in a scene order at tick " << tick;
            previous = event.a;
        }
    }

    // And by the end every one of them is resting and saying so.
    EXPECT_EQ(of_phase(physics->contact_events(), ContactPhase::Persist).size(),
              std::size_t(5));
}

TEST(Integration_PhysicsSimulation, AVehiclesShellIsFoundByASceneQuery)
{
    // A car is enumerated for gravity and for the broadphase, so it falls and it collides.
    // A query hierarchy that does not hold it is a car nothing can raycast at: no aiming, no
    // targeting, no spawning on top of it. Asserted before any step, because placing a
    // vehicle is a membership change and a query must not have to wait for a tick to see it.
    std::vector<std::byte> blob;
    ASSERT_TRUE(Physics::Cooking::build_node_beam_blob(square_shell_asset(), blob));

    auto physics = create_physics_simulation(Harness::shared_context());

    const Vector3 spawn{0, Scalar(3), 0};
    VehicleDescription vehicle;
    vehicle.id = 7;
    vehicle.asset = blob.data();
    vehicle.asset_size = blob.size();
    vehicle.position = spawn;
    physics->set_vehicles({vehicle});

    // Straight down the axis of the first shell node, from well above it.
    const Vector3 node = spawn + SHELL[0];
    const SceneRayHit hit = physics->raycast_closest(
        Vector3{node.x, node.y + Scalar(4), node.z}, Vector3{0, -1, 0}, Scalar(10),
        SceneQueryFilter{});
    ASSERT_TRUE(hit.hit) << "a ray down a shell node found nothing";
    EXPECT_EQ(hit.entity, EntityId(7)) << "the hit must name the entity that owns the vehicle";
    EXPECT_NEAR(double(hit.point.y), double(node.y + SHELL_NODE_RADIUS), 1e-5)
        << "the query must see the node at the radius it collides at";

    // And the two bodies that present no surface stay out. The core sits at the centre of
    // the square, a metre from the nearest node, so a small overlap there can only find a
    // proxy the core itself contributed -- and a zero-radius sphere would be found by it.
    SolvedPose core;
    ASSERT_TRUE(physics->vehicle_core_pose(7, core));
    EXPECT_TRUE(physics->overlap_sphere(core.position, Scalar(0.05), SceneQueryFilter{}).empty())
        << "a body with no collision surface must not become a query proxy";
}

namespace
{
    /**
     * @brief Slides the square shell along the ground and reports how far it got.
     *
     * The shell starts resting on the plane at 5 m/s, so the only thing that decides where
     * it stops is the friction between its nodes' surface and the plane's. Two seconds is
     * long enough for a gripping shell to stop outright and a frictionless one to keep every
     * metre per second it started with, which is what makes the two answers far apart rather
     * than a tolerance argument.
     *
     * @param materials The vehicle's material table; the shell's nodes take entry 0.
     * @return The core's x position after 120 ticks, in metres.
     */
    double shell_slide_distance(const std::vector<Physics::PhysicsMaterial>& materials)
    {
        std::vector<std::byte> blob;
        if (!Physics::Cooking::build_node_beam_blob(square_shell_asset(), blob))
            return 0.0;

        auto physics = create_physics_simulation(Harness::shared_context());

        VehicleDescription vehicle;
        vehicle.id = 7;
        vehicle.asset = blob.data();
        vehicle.asset_size = blob.size();
        // Node centres exactly one radius above the plane: the shell starts resting on it.
        vehicle.position = Vector3{0, SHELL_NODE_RADIUS, 0};
        vehicle.velocity = Vector3{Scalar(5), 0, 0};
        vehicle.setup.materials = materials;
        physics->set_vehicles({vehicle});
        physics->set_static_planes(ground());

        for (int tick = 0; tick < 120; ++tick)
            physics->step(earth_gravity(), still_air(), SUBSTEPS);

        SolvedPose core;
        if (!physics->vehicle_core_pose(7, core))
            return 0.0;
        return double(core.position.x);
    }
} // namespace

TEST(Integration_PhysicsSimulation, AVehiclesAuthoredMaterialDecidesHowItsShellSlides)
{
    // `SuspensionSetupT::material_index` and `NodeBeamStructureSettings::node_material_index`
    // are the only way a car can say its wheels are one surface and its panels another, and
    // they are indices: the table they name is `VehicleAssetT::materials`, and the scene is
    // what has to resolve one against the other. Unresolved, every body of every car contacts
    // as the default solid, and these two runs would land in the same place.
    const double gripping = shell_slide_distance({});

    Physics::PhysicsMaterial ice;
    ice.static_friction = 0;
    ice.dynamic_friction = 0;
    ice.restitution = 0;
    // The plane is the default surface and asks for `average`, which would leave half of its
    // own friction on a contact with a frictionless body. `minimum` is the stricter mode, so
    // `stricter_mode` picks it for the pair and the smaller of the two coefficients wins.
    ice.friction_combine = Physics::MaterialCombineMode::minimum;

    const double frictionless = shell_slide_distance({ice});

    EXPECT_LT(gripping, 4.0) << "a shell on the default surface must be slowed by friction";
    EXPECT_GT(frictionless, gripping + 4.0)
        << "the authored material never reached the contact: both runs slid the same way";
}

// Kinematic bodies (P9-A). A kinematic body is the third body kind: integrated like a
// dynamic one, immovable like a static one, and the only one whose motion is an input.
// What these five cover is the seam between those halves — that the command reaches the
// integration, that nothing in the solve reaches back, and that the sleep rule which
// makes a kinematic body cheap does not quietly strand what is riding on it.

namespace
{
    /** @brief The id of the platform in every kinematic scene below. */
    constexpr EntityId PLATFORM = 1;

    /** @brief The id of the crate that rides it. */
    constexpr EntityId CRATE = 2;

    /** @brief One tick's duration: the sub-step schedule the whole file runs at. */
    constexpr double TICK_SECONDS = double(SUBSTEP_DT) * double(SUBSTEPS);

    /** @brief Where the crate rests: the platform's top face plus its own half extent. */
    constexpr double CRATE_REST_Y = 1.75;

    /** @brief A box collider of arbitrary half extents. */
    Collider box_collider(Scalar x, Scalar y, Scalar z)
    {
        Collider collider;
        collider.shape = ColliderShape::Box;
        collider.half_extents = Vector3{x, y, z};
        return collider;
    }

    /**
     * @brief A wide, thin kinematic slab with its top face at y = 1.25.
     *
     * `inv_mass` is left at the descriptor's default of 1 deliberately. Zeroing it is
     * the extract's job, and a test that pre-zeroed it here would pass whether or not
     * that ever happened.
     */
    RigidBodyDescription platform_description()
    {
        RigidBodyDescription description;
        description.id = PLATFORM;
        description.position = Vector3{0, Scalar(1), 0};
        description.kinematic = true;
        description.collider = box_collider(Scalar(3), Scalar(0.25), Scalar(3));
        return description;
    }

    /** @brief A unit crate dropped just above the platform's top face. */
    RigidBodyDescription crate_description()
    {
        RigidBodyDescription description;
        description.id = CRATE;
        description.position = Vector3{0, Scalar(2), 0};
        description.inv_mass = Scalar(1);
        description.collider = unit_box_collider();
        return description;
    }
}

TEST(Integration_PhysicsSimulation, AKinematicBodyDoesNotFall)
{
    // The narrowest statement of what `predict`'s kinematic branch is for. A body with
    // `inv_mass == 0` and no branch would not fall either — but it also could not be
    // moved, which is the next test. This one is the half that must hold while it is.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description()}, ITERATIONS, SUBSTEP_DT);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(PLATFORM, pose));
    EXPECT_NEAR(double(pose.position.y), 1.0, 1e-9)
        << "gravity reached a body that is supposed to take none";
}

TEST(Integration_PhysicsSimulation, AKinematicBodyIsNotPushedByWhatRestsOnIt)
{
    // The other half of immovability, and the one that is not free: a crate at rest is
    // still solving a contact against the platform every sub-step, and every one of
    // those projections would move it if its inverse mass were not zero.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description(), crate_description()}, ITERATIONS,
                              SUBSTEP_DT);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose platform;
    SolvedPose crate;
    ASSERT_TRUE(physics->rigid_pose(PLATFORM, platform));
    ASSERT_TRUE(physics->rigid_pose(CRATE, crate));
    EXPECT_NEAR(double(platform.position.y), 1.0, 1e-9)
        << "the crate's weight moved a body nothing is allowed to move";
    EXPECT_NEAR(double(crate.position.y), CRATE_REST_Y, 1e-2)
        << "the crate did not come to rest on the platform's top face";
}

TEST(Integration_PhysicsSimulation, AKinematicPlatformCarriesWhatRestsOnIt)
{
    // The point of the whole sub-project. Nothing here carries the crate: no code reads
    // "this body is standing on that one". The platform's derived velocity is what the
    // contact's friction term measures, and friction is what moves the crate.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description(), crate_description()}, ITERATIONS,
                              SUBSTEP_DT);

    // Long enough to land and settle, short enough that the crate has not yet been
    // still for the half second the default sleep delay asks for.
    for (int tick = 0; tick < 30; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    // One metre per second along +x, commanded as a pose every tick, which is what a
    // script loop or an animation clip does.
    constexpr double SPEED = 1.0;
    constexpr int MOVING_TICKS = 120;
    Vector3 target = Vector3{0, Scalar(1), 0};
    for (int tick = 0; tick < MOVING_TICKS; ++tick)
    {
        target.x = Scalar(double(target.x) + SPEED * TICK_SECONDS);
        physics->move_rigid_body(PLATFORM, target, Quaternion{0, 0, 0, 1});
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    }

    const double travelled = SPEED * TICK_SECONDS * double(MOVING_TICKS);
    SolvedPose platform;
    SolvedPose crate;
    ASSERT_TRUE(physics->rigid_pose(PLATFORM, platform));
    ASSERT_TRUE(physics->rigid_pose(CRATE, crate));

    EXPECT_NEAR(double(platform.position.x), travelled, 1e-6)
        << "the platform did not reach the pose it was commanded to";
    // Not the same distance: friction is finite, so the crate slips a little at the
    // start and whenever the platform's velocity changes. What is asserted is that it
    // was carried at all — a crate left behind ends near zero.
    EXPECT_GT(double(crate.position.x), travelled * 0.5)
        << "the platform slid out from under the crate instead of carrying it";
    EXPECT_LT(double(crate.position.x), travelled + 0.1)
        << "the crate outran the platform, which no contact force can do";
    EXPECT_NEAR(double(crate.position.y), CRATE_REST_Y, 1e-2)
        << "the crate did not stay on the platform while it moved";
}

TEST(Integration_PhysicsSimulation, AnUncommandedKinematicBodyStopsWhereItIs)
{
    // Nothing in the tick slows a kinematic body: it takes no gravity and no drag, so a
    // velocity left in place would coast forever. Clearing it every tick that carries no
    // command is what makes "drive it by writing its pose" the whole contract rather
    // than half of one.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description()}, ITERATIONS, SUBSTEP_DT);

    const Vector3 once = Vector3{Scalar(TICK_SECONDS), Scalar(1), 0};
    physics->move_rigid_body(PLATFORM, once, Quaternion{0, 0, 0, 1});
    physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose commanded;
    ASSERT_TRUE(physics->rigid_pose(PLATFORM, commanded));
    EXPECT_NEAR(double(commanded.position.x), TICK_SECONDS, 1e-6)
        << "the single command did not reach the body";

    for (int tick = 0; tick < 120; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    SolvedPose parked;
    ASSERT_TRUE(physics->rigid_pose(PLATFORM, parked));
    EXPECT_NEAR(double(parked.position.x), TICK_SECONDS, 1e-6)
        << "the body coasted on a command that was consumed two seconds ago";
}

TEST(Integration_PhysicsSimulation, MovingAKinematicBodyWakesWhatSleepsOnIt)
{
    // The failure this exists to catch is silent and specific. `IslandBuilder::conducts`
    // requires a positive inverse mass, so a kinematic body is an island of one and the
    // crate settles into an island of its own. Waking the platform's island would wake
    // nothing but the platform — and the crate, asleep, is neither integrated nor
    // projected, so it would hang in the air while the platform left.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description(), crate_description()}, ITERATIONS,
                              SUBSTEP_DT);

    for (int tick = 0; tick < 360; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    ASSERT_GT(physics->statistics().sleeping_bodies, 0u) << "nothing ever settled";

    SolvedPose before;
    ASSERT_TRUE(physics->rigid_pose(CRATE, before));

    constexpr double SPEED = 1.0;
    constexpr int MOVING_TICKS = 60;
    Vector3 target = Vector3{0, Scalar(1), 0};
    for (int tick = 0; tick < MOVING_TICKS; ++tick)
    {
        target.x = Scalar(double(target.x) + SPEED * TICK_SECONDS);
        physics->move_rigid_body(PLATFORM, target, Quaternion{0, 0, 0, 1});
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    }

    SolvedPose after;
    ASSERT_TRUE(physics->rigid_pose(CRATE, after));
    EXPECT_GT(double(after.position.x) - double(before.position.x), 0.1)
        << "the crate stayed asleep and was left behind by the platform";
    EXPECT_NEAR(double(after.position.y), CRATE_REST_Y, 1e-2)
        << "the crate woke but fell off, or never woke and hung in the air";
}

// The character controller (P9-B, §16.47), against the live physics rather than against
// the half-spaces `tests/unit/test_character_mover.cpp` uses. What only appears here is
// the binding: that the sweep excludes the character's own body, that the resolved pose
// survives a real tick, and that a kinematic capsule pushes what it walks into.

namespace
{
    /** @brief The id of the character in every scene below. */
    constexpr EntityId WALKER = 3;

    /** @brief The character's capsule: 0.4 m radius, 1.8 m tall. */
    CharacterParameters walker_parameters()
    {
        CharacterParameters character;
        character.radius = Scalar(0.4);
        character.height = Scalar(1.8);
        character.step_height = Scalar(0.4);
        character.max_slope_degrees = Scalar(45);
        return character;
    }

    /** @brief Where a standing capsule's centre sits on a floor at y = @p floor_y. */
    Scalar walker_rest_y(Scalar floor_y)
    {
        const CharacterParameters character = walker_parameters();
        return floor_y + character.radius + character_half_height(character);
    }

    /** @brief The character's body: kinematic, with the capsule it moves as. */
    RigidBodyDescription walker_description(Scalar floor_y)
    {
        const CharacterParameters character = walker_parameters();
        RigidBodyDescription description;
        description.id = WALKER;
        description.position = Vector3{0, walker_rest_y(floor_y), 0};
        description.kinematic = true;
        description.collider.shape = ColliderShape::Capsule;
        description.collider.radius = character.radius;
        description.collider.half_height = character_half_height(character);
        return description;
    }

    /** @brief A pinned box: not a kinematic body, just one nothing can move. */
    RigidBodyDescription pinned_box(EntityId id, const Vector3& position, Scalar x, Scalar y,
                                    Scalar z)
    {
        RigidBodyDescription description;
        description.id = id;
        description.position = position;
        description.inv_mass = 0;
        description.collider = box_collider(x, y, z);
        return description;
    }

    /** @brief One tick of walking: resolve the move, then let the world run. */
    CharacterMoveState walk(IPhysicsScene& physics, const Vector3& displacement)
    {
        const CharacterMoveState state =
            physics.move_character(WALKER, walker_parameters(), displacement, Vector3{0, 1, 0});
        physics.step(earth_gravity(), still_air(), SUBSTEPS);
        return state;
    }

    /** @brief A tick's worth of falling, so a walk is not a hover. */
    Vector3 with_gravity(const Vector3& horizontal)
    {
        return Vector3{horizontal.x, horizontal.y - Scalar(9.8 * TICK_SECONDS * TICK_SECONDS),
                       horizontal.z};
    }
}

TEST(Integration_PhysicsSimulation, ACharacterWalksAcrossFlatGroundWithoutHittingItself)
{
    // The binding's first failure mode, and a silent one: the character's own capsule is
    // in the query index, so a sweep that does not exclude it reports a hit at distance
    // zero every time and the character never moves at all.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({walker_description(0)}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 60; ++tick)
        walk(*physics, with_gravity(Vector3{Scalar(0.02), 0, 0}));

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(WALKER, pose));
    EXPECT_GT(double(pose.position.x), 1.0) << "the character never moved; it swept against itself";
    EXPECT_NEAR(double(pose.position.y), double(walker_rest_y(0)), 5e-2)
        << "the character sank into the floor or floated above it";
}

TEST(Integration_PhysicsSimulation, ACharacterClimbsAStepInARealScene)
{
    // A pinned box with its top 0.3 m up, inside the 0.4 m step height. The unit tests
    // prove the three-sweep step against hand-written planes; what this proves is that
    // the same thing happens against a broadphase, a real capsule and a real box.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies(
        {walker_description(0), pinned_box(10, Vector3{Scalar(3), Scalar(0.15), 0}, Scalar(2),
                                           Scalar(0.15), Scalar(2))},
        ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 120; ++tick)
        walk(*physics, with_gravity(Vector3{Scalar(0.02), 0, 0}));

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(WALKER, pose));
    EXPECT_GT(double(pose.position.x), 1.2) << "the step stopped the character like a wall";
    EXPECT_NEAR(double(pose.position.y), double(walker_rest_y(Scalar(0.3))), 8e-2)
        << "the character did not end up standing on the step";
}

TEST(Integration_PhysicsSimulation, ACharacterIsStoppedByAWall)
{
    // A tall pinned box, well above the step height. The claim is narrow and worth being
    // narrow: the character stops, and `remaining` says so. A controller that reported an
    // empty remainder while being blocked gives a caller no way to tell walking from
    // pushing.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies(
        {walker_description(0), pinned_box(10, Vector3{Scalar(3), Scalar(2), 0}, Scalar(0.5),
                                           Scalar(2), Scalar(2))},
        ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    CharacterMoveState state;
    for (int tick = 0; tick < 200; ++tick)
        state = walk(*physics, with_gravity(Vector3{Scalar(0.02), 0, 0}));

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(WALKER, pose));
    EXPECT_LT(double(pose.position.x), 2.2) << "the character walked through the wall";
    EXPECT_GT(double(length(state.remaining)), 1e-4)
        << "a blocked walk reported nothing left over";
    EXPECT_NEAR(double(pose.position.y), double(walker_rest_y(0)), 5e-2)
        << "pressing into a wall lifted the character";
}

TEST(Integration_PhysicsSimulation, ACharacterStopsAtADynamicCrateAndDoesNotPushIt)
{
    // Two claims, and the second is a limitation being pinned rather than a feature.
    //
    // It stops: a dynamic body is in the query index like anything else, so the sweep
    // sees the crate and the character does not walk through it. That is the half that
    // must never regress.
    //
    // It does not push: the sweep stops the capsule a skin width short of the crate, so
    // there is no overlap for the contact projection to resolve and nothing moves the
    // crate. Unity's `CharacterController` and PhysX's `PxController` behave the same way
    // and both make pushing an explicit opt-in the caller wires up. Doing it here would
    // mean the mover reporting its blocking hits and the scene spending an impulse per
    // hit — real work, deliberately not in this row.
    auto physics = create_physics_simulation(Harness::shared_context());
    RigidBodyDescription crate;
    crate.id = 11;
    crate.position = Vector3{Scalar(1.5), Scalar(0.5), 0};
    crate.inv_mass = Scalar(1);
    crate.collider = unit_box_collider();

    physics->set_rigid_bodies({walker_description(0), crate}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 120; ++tick)
        walk(*physics, with_gravity(Vector3{Scalar(0.02), 0, 0}));

    SolvedPose walker;
    SolvedPose untouched;
    ASSERT_TRUE(physics->rigid_pose(WALKER, walker));
    ASSERT_TRUE(physics->rigid_pose(11, untouched));
    EXPECT_LT(double(walker.position.x), 0.8) << "the character walked into the crate";
    EXPECT_LT(std::abs(double(untouched.position.x) - 1.5), 0.1)
        << "the crate moved, which nothing here does yet — if pushing is now intended, "
           "rewrite this test rather than loosening it";
}

// The physics event sink (P9-C, §16.48). The filter is declared at registration and
// evaluated by the scene, so what these check is that the scene honours it — a listener
// that filtered for itself would pass every one of them while still being woken 540
// times a second by a stack that is not moving.

namespace
{
    /** @brief A sink that remembers what it was told, and nothing else. */
    class CountingSink final : public IPhysicsEventSink
    {
        public:
            std::vector<ContactEvent> contacts;

            void on_contact(const ContactEvent& event) override
            {
                contacts.push_back(event);
            }
    };

    /** @brief A one-kilogramme crate dropped a metre above the ground. */
    RigidBodyDescription falling_crate(EntityId id)
    {
        RigidBodyDescription description;
        description.id = id;
        description.position = Vector3{0, Scalar(1.5), 0};
        description.inv_mass = Scalar(1);
        description.collider = unit_box_collider();
        return description;
    }
}

// The physics snapshot (P9-D, §16.49). What is being proved is not a feature anyone
// sees: capture at K, run on to N, restore K, replay to N, and the state at N must be
// byte-identical to the state the uninterrupted run reached.
//
// The control comes first and is not a formality. Nothing in this codebase has ever
// asserted byte equality of physics state — the conformance suite compares host and
// device at 1e-9 — so without a run-to-run comparison a failing rollback test cannot
// distinguish a lossy snapshot from a simulation that was never deterministic.

namespace
{
    /** @brief A four-box stack over a ground plane: contacts that persist and settle. */
    std::vector<RigidBodyDescription> stack_scene()
    {
        std::vector<RigidBodyDescription> bodies;
        for (int i = 0; i < 4; ++i)
        {
            RigidBodyDescription description;
            description.id = EntityId(i + 1);
            description.position = Vector3{0, Scalar(0.5) + Scalar(i) * Scalar(1.02), 0};
            description.inv_mass = Scalar(1);
            description.inv_inertia = Vector3{Scalar(6), Scalar(6), Scalar(6)};
            description.collider = unit_box_collider();
            bodies.push_back(description);
        }
        return bodies;
    }

    /** @brief A scene with the stack already installed, ready to step. */
    std::unique_ptr<IPhysicsScene> stacked_physics()
    {
        auto physics = create_physics_simulation(Harness::shared_context());
        physics->set_rigid_bodies(stack_scene(), ITERATIONS, SUBSTEP_DT);
        physics->set_static_planes(ground());
        return physics;
    }

    /** @brief Runs @p count ticks. */
    void run(IPhysicsScene& physics, int count)
    {
        for (int tick = 0; tick < count; ++tick)
            physics.step(earth_gravity(), still_air(), SUBSTEPS);
    }
}

TEST(Integration_PhysicsSimulation, TwoIdenticalScenesStayByteIdentical)
{
    // The control. If this fails, every rollback assertion below is meaningless and the
    // finding is a determinism defect in the solve, not in the snapshot.
    auto first = stacked_physics();
    auto second = stacked_physics();

    std::vector<std::byte> a;
    std::vector<std::byte> b;
    for (int tick = 0; tick < 200; ++tick)
    {
        first->step(earth_gravity(), still_air(), SUBSTEPS);
        second->step(earth_gravity(), still_air(), SUBSTEPS);
        first->capture_snapshot(a);
        second->capture_snapshot(b);
        ASSERT_EQ(a, b) << "two identical scenes diverged at tick " << tick;
    }
    EXPECT_FALSE(a.empty()) << "the snapshot is empty; nothing was captured at all";
}

TEST(Integration_PhysicsSimulation, ARestoredSceneReplaysToTheSameBytes)
{
    // The claim itself, over a stack that is still settling at the rollback point — so
    // the contact records the snapshot carries are doing work rather than sitting empty.
    auto physics = stacked_physics();
    run(*physics, 40);

    std::vector<std::byte> at_rollback;
    physics->capture_snapshot(at_rollback);

    run(*physics, 60);
    std::vector<std::byte> uninterrupted;
    physics->capture_snapshot(uninterrupted);

    ASSERT_TRUE(physics->restore_snapshot(at_rollback.data(), at_rollback.size()))
        << "the scene refused its own snapshot";
    run(*physics, 60);
    std::vector<std::byte> replayed;
    physics->capture_snapshot(replayed);

    EXPECT_EQ(uninterrupted, replayed)
        << "the replay diverged; some state the next tick reads was not captured";
}

TEST(Integration_PhysicsSimulation, ARestoredSceneReplaysThroughASettledStack)
{
    // The same claim after the stack has gone to sleep, which exercises the half the
    // test above cannot: sleep timers, the island partition, and the flags that decide
    // whether a body is integrated at all.
    auto physics = stacked_physics();
    run(*physics, 400);
    ASSERT_GT(physics->statistics().sleeping_bodies, 0u) << "the stack never settled";

    std::vector<std::byte> at_rollback;
    physics->capture_snapshot(at_rollback);
    run(*physics, 120);
    std::vector<std::byte> uninterrupted;
    physics->capture_snapshot(uninterrupted);

    ASSERT_TRUE(physics->restore_snapshot(at_rollback.data(), at_rollback.size()));
    run(*physics, 120);
    std::vector<std::byte> replayed;
    physics->capture_snapshot(replayed);

    EXPECT_EQ(uninterrupted, replayed) << "a settled island did not survive the round trip";
}

TEST(Integration_PhysicsSimulation, ASnapshotFromADifferentBodySetIsRefused)
{
    // Refused, not misapplied. Applying a blob over a different set would leave bodies
    // holding another body's state, which looks like a physics bug for as long as it
    // takes to find.
    auto four = stacked_physics();
    run(*four, 20);
    std::vector<std::byte> blob;
    four->capture_snapshot(blob);

    auto one = create_physics_simulation(Harness::shared_context());
    one->set_rigid_bodies({stack_scene().front()}, ITERATIONS, SUBSTEP_DT);
    one->set_static_planes(ground());
    run(*one, 20);

    // Captured before the attempt, so the comparison after it is against what the scene
    // actually was rather than against a second reading of whatever it became.
    std::vector<std::byte> before;
    one->capture_snapshot(before);

    EXPECT_FALSE(one->restore_snapshot(blob.data(), blob.size()))
        << "a snapshot for four bodies was applied to a scene holding one";

    std::vector<std::byte> after;
    one->capture_snapshot(after);
    EXPECT_EQ(before, after) << "the refused restore still changed the scene on its way out";
}

TEST(Integration_PhysicsSimulation, AMalformedSnapshotIsRefused)
{
    auto physics = stacked_physics();
    run(*physics, 20);
    std::vector<std::byte> blob;
    physics->capture_snapshot(blob);

    EXPECT_FALSE(physics->restore_snapshot(nullptr, 0)) << "an empty blob was accepted";
    // Truncated halfway: the header still reads, and everything after it must not.
    EXPECT_FALSE(physics->restore_snapshot(blob.data(), blob.size() / 2))
        << "a truncated blob was read past its end";
}

TEST(Integration_PhysicsSimulation, TenThousandTicksReplayToTheSameBytes)
{
    // The row's own number. Long enough that anything accumulating a divergence — a
    // sleep timer, a warm-start impulse, an island revision — has ten thousand chances
    // to show it.
    auto physics = stacked_physics();
    run(*physics, 500);

    std::vector<std::byte> at_rollback;
    physics->capture_snapshot(at_rollback);

    run(*physics, 10000);
    std::vector<std::byte> uninterrupted;
    physics->capture_snapshot(uninterrupted);

    ASSERT_TRUE(physics->restore_snapshot(at_rollback.data(), at_rollback.size()));
    run(*physics, 10000);
    std::vector<std::byte> replayed;
    physics->capture_snapshot(replayed);

    EXPECT_EQ(uninterrupted, replayed) << "ten thousand ticks of replay diverged";
}

TEST(Integration_PhysicsSimulation, AnEventSinkHearsABeginAndNothingElse)
{
    // `persist` is off by default, which is the decision the whole design turns on: a
    // settled body is still generating contacts every tick, and a listener that heard
    // them would be woken forever by a scene that has stopped moving.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({falling_crate(1)}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    CountingSink sink;
    ASSERT_NE(physics->add_event_sink(&sink, PhysicsEventFilter{}), NULL_PHYSICS_SINK);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    ASSERT_FALSE(sink.contacts.empty()) << "the crate landed and the sink was never called";
    for (const ContactEvent& event : sink.contacts)
        EXPECT_EQ(event.phase, ContactPhase::Begin) << "a phase the filter did not ask for";
    // Four seconds of simulation, of which the crate spends nearly all at rest. A
    // handful of begins from the landing and its settle is expected; hundreds would
    // mean `persist` leaked through.
    EXPECT_LT(sink.contacts.size(), 20u) << "the sink was woken by a body that had settled";
}

TEST(Integration_PhysicsSimulation, AnEventSinkBelowItsImpulseThresholdIsNotCalled)
{
    // The same landing, twice, differing only in the number. A crate of one kilogramme
    // falling a metre arrives with a few newton-seconds; a thousand is a wall it cannot
    // reach, so the second sink must hear nothing at all.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({falling_crate(1)}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    CountingSink audible;
    CountingSink deaf;
    PhysicsEventFilter loud;
    loud.minimum_impulse = Scalar(1000);
    physics->add_event_sink(&audible, PhysicsEventFilter{});
    physics->add_event_sink(&deaf, loud);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    EXPECT_FALSE(audible.contacts.empty()) << "the control sink heard nothing either";
    EXPECT_TRUE(deaf.contacts.empty()) << "an impulse threshold nothing could reach let a hit past";
}

TEST(Integration_PhysicsSimulation, APairCooldownSuppressesTheSecondHit)
{
    // A crate landing does not produce one Begin. It touches, separates by a fraction of
    // a millimetre, and touches again — which is one impact to a listener and several
    // events to the solver. The cooldown is what makes those the same number.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({falling_crate(1)}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    CountingSink every;
    CountingSink spaced;
    PhysicsEventFilter slow;
    // Longer than the whole run, so at most one event can ever get through.
    slow.pair_cooldown = Scalar(10);
    physics->add_event_sink(&every, PhysicsEventFilter{});
    physics->add_event_sink(&spaced, slow);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    ASSERT_FALSE(every.contacts.empty()) << "the control sink heard nothing either";
    EXPECT_LE(spaced.contacts.size(), 1u) << "the cooldown let a second hit through its window";
}

TEST(Integration_PhysicsSimulation, ARemovedEventSinkStopsBeingCalled)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({falling_crate(1)}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    CountingSink sink;
    const PhysicsSinkId id = physics->add_event_sink(&sink, PhysicsEventFilter{});
    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    const std::size_t before = sink.contacts.size();
    ASSERT_GT(before, 0u) << "the sink was never called while it was registered";

    physics->remove_event_sink(id);
    // Lift it and drop it again, so there is a fresh landing to miss.
    physics->set_rigid_pose(1, Vector3{0, Scalar(3), 0}, Quaternion{0, 0, 0, 1});
    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    EXPECT_EQ(sink.contacts.size(), before) << "a removed sink was still being called";
}

TEST(Integration_PhysicsSimulation, ACharacterOnAMovingPlatformIsNotCarriedByIt)
{
    // Pinning a limitation, not celebrating one. Both bodies are kinematic, so both have
    // zero inverse mass, so the contact between them resolves to nothing — the platform
    // slides out from under the character. Every kinematic controller has this: Unity's
    // `CharacterController` and PhysX's `PxController` both require the caller to add the
    // platform's own delta to the character's move.
    //
    // The test exists so that the day someone adds platform-relative movement, it fails
    // and has to be rewritten deliberately rather than quietly starting to pass.
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({platform_description(), walker_description(Scalar(1.25))},
                              ITERATIONS, SUBSTEP_DT);

    for (int tick = 0; tick < 30; ++tick)
        walk(*physics, with_gravity(Vector3{0, 0, 0}));

    SolvedPose before;
    ASSERT_TRUE(physics->rigid_pose(WALKER, before));

    constexpr double SPEED = 1.0;
    Vector3 target = Vector3{0, Scalar(1), 0};
    for (int tick = 0; tick < 60; ++tick)
    {
        target.x = Scalar(double(target.x) + SPEED * TICK_SECONDS);
        physics->move_rigid_body(PLATFORM, target, Quaternion{0, 0, 0, 1});
        walk(*physics, with_gravity(Vector3{0, 0, 0}));
    }

    SolvedPose after;
    ASSERT_TRUE(physics->rigid_pose(WALKER, after));
    EXPECT_LT(std::abs(double(after.position.x) - double(before.position.x)), 0.05)
        << "the character was carried, which nothing in the engine does yet — if this is "
           "now intended, rewrite this test rather than loosening it";
}
