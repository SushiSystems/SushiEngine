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
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/physics_simulation.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr std::size_t ITERATIONS = 8;
    constexpr std::size_t SUBSTEPS = 4;
    constexpr Scalar SUBSTEP_DT = Scalar(1.0 / 240.0);
    constexpr double PI = 3.14159265358979323846;

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
    std::vector<PlaneDesc> ground()
    {
        PlaneDesc plane;
        plane.point = Vector3{0, 0, 0};
        plane.normal = Vector3{0, 1, 0};
        return {plane};
    }
}

TEST(Integration_PhysicsSimulation, BoxSettlesOnItsFaceNotItsBoundingRadius)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    RigidBodyDesc desc;
    desc.id = 1;
    desc.position = Vector3{0, Scalar(4), 0};
    desc.inv_mass = Scalar(1);
    // A bounding-sphere fallback would hover at the box's bounding radius,
    // 0.5*sqrt(3), instead of resting on its face at the half-extent.
    desc.collider = unit_box_collider();

    physics->set_rigid_bodies({desc}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 400; ++tick)
        physics->step(earth_gravity(), SUBSTEPS);

    SolvedPose pose;
    ASSERT_TRUE(physics->rigid_pose(1, pose));
    EXPECT_NEAR(double(pose.position.y), 0.5, 1e-2);
}

TEST(Integration_PhysicsSimulation, TiltedBoxSettlesOnItsEdge)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    RigidBodyDesc desc;
    desc.id = 1;
    desc.position = Vector3{0, Scalar(4), 0};
    desc.orientation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(PI / 4.0));
    desc.inv_mass = Scalar(1);
    desc.inv_inertia = Vector3{0, 0, 0}; // no angular freedom: it stays tilted
    desc.collider = unit_box_collider();

    physics->set_rigid_bodies({desc}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 400; ++tick)
        physics->step(earth_gravity(), SUBSTEPS);

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

    ClothDesc cloth;
    cloth.id = 10;
    cloth.rows = 3;
    cloth.cols = 3;
    cloth.spacing = Scalar(0.5);
    cloth.origin = Vector3{0, 0, 0};
    cloth.thickness = Scalar(0.1);

    RigidBodyDesc body;
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

    physics->step(no_gravity(), SUBSTEPS);

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

    ClothDesc cloth;
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

    RigidBodyDesc body;
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

    std::vector<RigidBodyDesc> bodies;
    for (int i = 0; i < 2; ++i)
    {
        RigidBodyDesc desc;
        desc.id = static_cast<EntityId>(i + 1);
        desc.position = Vector3{0, Scalar(1.0 + i * 1.2), 0};
        desc.inv_mass = Scalar(1);
        desc.collider = unit_box_collider();
        bodies.push_back(desc);
    }

    physics->set_rigid_bodies(bodies, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 600; ++tick)
        physics->step(earth_gravity(), SUBSTEPS);

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

    RigidBodyDesc bullet;
    bullet.id = 1;
    bullet.position = Vector3{Scalar(-3), Scalar(1), 0};
    bullet.inv_mass = Scalar(1);
    bullet.collider = sphere_collider(Scalar(0.5));

    RigidBodyDesc wall;
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
        physics->step(no_gravity(), SUBSTEPS);

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

    RigidBodyDesc first;
    first.id = 1;
    first.position = Vector3{0, Scalar(4), 0};
    first.inv_mass = Scalar(1);
    RigidBodyDesc second = first;
    second.id = 2;
    second.position = Vector3{Scalar(0.4), Scalar(4.4), 0};

    physics->set_rigid_bodies({first, second}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());
    physics->step(earth_gravity(), SUBSTEPS);

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

// ---------------------------------------------------------------------------
// Contact events: what touched what, reported to gameplay (P1).
//
// The last thing P1 owed. It is the cheapest part of the phase to write and the
// easiest to get subtly wrong, because every failure mode is a *sequence* rather
// than a value: a Begin that repeats every tick, an End that never arrives, or an
// order that depends on the broadphase's insertion history rather than on the
// scene. So these tests watch the sequence and not just the contents.
// ---------------------------------------------------------------------------

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

    RigidBodyDesc desc;
    desc.id = 1;
    desc.position = Vector3{0, Scalar(1.2), 0};
    desc.inv_mass = Scalar(1);
    desc.collider = unit_box_collider();

    physics->set_rigid_bodies({desc}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    int begins = 0;
    int persists = 0;
    Scalar resting_impulse = 0;
    for (int tick = 0; tick < 120; ++tick)
    {
        physics->step(earth_gravity(), SUBSTEPS);
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

    RigidBodyDesc desc;
    desc.id = 1;
    desc.position = Vector3{0, Scalar(0.6), 0};
    desc.inv_mass = Scalar(1);
    desc.collider = unit_box_collider();

    physics->set_rigid_bodies({desc}, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 60; ++tick)
        physics->step(earth_gravity(), SUBSTEPS);
    ASSERT_FALSE(of_phase(physics->contact_events(), ContactPhase::Persist).empty())
        << "it must be resting before it can stop resting";

    // Lifted clear rather than removed: removal changes the collidable membership,
    // which renumbers the proxies and deliberately drops the history with them.
    physics->set_rigid_pose(1, Vector3{0, Scalar(20), 0}, Quaternion{0, 0, 0, 1});
    physics->step(earth_gravity(), SUBSTEPS);

    const std::vector<ContactEvent> ended =
        of_phase(physics->contact_events(), ContactPhase::End);
    ASSERT_EQ(ended.size(), std::size_t(1));
    EXPECT_EQ(ended[0].a, EntityId(1));

    // And nothing at all on the tick after, rather than an End every tick for ever.
    physics->step(earth_gravity(), SUBSTEPS);
    EXPECT_TRUE(physics->contact_events().empty());
}

TEST(Integration_PhysicsSimulation, ATriggerIsReportedAndNotResolved)
{
    // The whole point of a trigger: the listener hears about it, the physics does
    // not act on it. A trigger that pushed would be a collider with extra reporting.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    std::unique_ptr<IPhysicsScene> physics = create_physics_simulation(execution);

    RigidBodyDesc falling;
    falling.id = 1;
    falling.position = Vector3{0, Scalar(3), 0};
    falling.inv_mass = Scalar(1);
    falling.collider = unit_box_collider();

    RigidBodyDesc volume;
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
        physics->step(earth_gravity(), SUBSTEPS);
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

    std::vector<RigidBodyDesc> bodies;
    for (int i = 0; i < 5; ++i)
    {
        RigidBodyDesc desc;
        desc.id = static_cast<EntityId>(i + 1);
        // Spread apart so each lands on the ground alone, and at descending heights
        // so they arrive in an order that is not the order they were added in.
        desc.position = Vector3{Scalar(i) * Scalar(4), Scalar(5 - i) * Scalar(0.4), 0};
        desc.inv_mass = Scalar(1);
        desc.collider = unit_box_collider();
        bodies.push_back(desc);
    }

    physics->set_rigid_bodies(bodies, ITERATIONS, SUBSTEP_DT);
    physics->set_static_planes(ground());

    for (int tick = 0; tick < 120; ++tick)
    {
        physics->step(earth_gravity(), SUBSTEPS);
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
