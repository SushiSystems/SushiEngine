/**************************************************************************/
/* test_islands.cpp                                                       */
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

// Unit_Islands: connected components, and the sleeping built on them (§13.2, §6.6).
//
// Three claims are worth a test here and the rest follows from them. That the
// ground does not conduct — otherwise a warehouse is one island and one kicked
// crate wakes all of it. That island numbering is a function of the scene's state
// and not of the order the merges ran, since §6.6 derives cross-region edges in
// ascending region-key order. And that the revision counter advances on genuine
// sleep/wake transitions and on nothing else, because that number is what a graph
// composition keys off and a counter that ticks every frame is a counter that has
// disabled the optimization it exists to enable.

#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/scene/islands.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;
    using Body = RigidBodyT<Real>;

    /** @brief A dynamic body at rest. */
    Body dynamic_body()
    {
        Body body;
        body.inv_mass = 1.0;
        body.inv_inertia = Vector3T<Real>{1.0, 1.0, 1.0};
        return body;
    }

    /** @brief A body that never moves and never conducts. */
    Body static_body()
    {
        Body body;
        body.inv_mass = 0.0;
        body.flags = BodyFlags::static_body;
        return body;
    }

    /** @brief Runs one tick of island building with the given edges. */
    IslandSet build(std::vector<Body>& bodies,
                    const std::vector<std::pair<std::uint32_t, std::uint32_t>>& edges,
                    IslandBuilder<Real>& builder, Real dt = 1.0 / 60.0)
    {
        builder.begin(bodies.size());
        for (const auto& edge : edges)
            builder.connect(edge.first, edge.second, bodies.data());
        IslandSet set;
        builder.finish(bodies.data(), bodies.size(), dt, 0.01, 0.5, set);
        return set;
    }
} // namespace

TEST(Unit_Islands, DisconnectedBodiesAreSeparateIslands)
{
    std::vector<Body> bodies(4, dynamic_body());
    IslandBuilder<Real> builder;
    const IslandSet set = build(bodies, {{0u, 1u}}, builder);

    ASSERT_EQ(set.islands.size(), 3u);
    EXPECT_EQ(set.islands[0].count, 2u);
    EXPECT_EQ(set.islands[1].count, 1u);
    EXPECT_EQ(set.largest, 2u);
}

TEST(Unit_Islands, TheGroundDoesNotJoinWhatRestsOnIt)
{
    // The claim §13.2 depends on: everything touches the ground, so if the ground
    // conducted, "island" would mean "the scene" and sleeping would never pay.
    std::vector<Body> bodies;
    bodies.push_back(static_body());     // 0: the ground
    bodies.push_back(dynamic_body());    // 1
    bodies.push_back(dynamic_body());    // 2
    bodies.push_back(dynamic_body());    // 3

    IslandBuilder<Real> builder;
    const IslandSet set = build(bodies, {{0u, 1u}, {0u, 2u}, {0u, 3u}}, builder);

    EXPECT_EQ(set.islands.size(), 3u) << "three crates on one floor are three islands";
    for (const Island& island : set.islands)
        EXPECT_EQ(island.count, 1u);
    // The static body is in no island at all; it has nothing to be woken from.
    EXPECT_EQ(set.bodies.size(), 3u);
}

TEST(Unit_Islands, IslandKeysAreTheLowestMemberAndAreOrderedByIt)
{
    // Built back to front on purpose: the numbering must come from the contents,
    // not from the order the merges happened to run in.
    std::vector<Body> bodies(6, dynamic_body());
    IslandBuilder<Real> builder;
    const IslandSet set = build(bodies, {{5u, 4u}, {2u, 1u}, {4u, 3u}}, builder);

    ASSERT_EQ(set.islands.size(), 3u);
    EXPECT_EQ(set.islands[0].key, 0u); // the lone body 0
    EXPECT_EQ(set.islands[1].key, 1u); // {1, 2}
    EXPECT_EQ(set.islands[2].key, 3u); // {3, 4, 5}
    EXPECT_EQ(set.islands[2].count, 3u);

    // And the bodies inside an island come out ascending too.
    EXPECT_EQ(set.bodies[set.islands[2].first], 3u);
    EXPECT_EQ(set.bodies[set.islands[2].first + 2u], 5u);
}

TEST(Unit_Islands, ASettledIslandFallsAsleepOnceItsTimerElapses)
{
    std::vector<Body> bodies(2, dynamic_body());
    IslandBuilder<Real> builder;
    const Real dt = 1.0 / 60.0;

    IslandSet set;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, {{0u, 1u}}, builder, dt);

    ASSERT_EQ(set.islands.size(), 1u);
    EXPECT_TRUE(set.islands[0].sleeping);
    EXPECT_EQ(set.awake_count, 0u);
    EXPECT_TRUE(has_any_flag(bodies[0].flags, BodyFlags::sleeping));
    EXPECT_TRUE(has_any_flag(bodies[1].flags, BodyFlags::sleeping));
}

TEST(Unit_Islands, OneMovingMemberKeepsTheWholeIslandAwake)
{
    // A crate leaning on one that is still rolling is not at rest; it is about to
    // be hit. Sleeping per body rather than per island is how a stack falls asleep
    // from the top down and then sinks into itself.
    std::vector<Body> bodies(3, dynamic_body());
    bodies[2].motion_measure = 5.0;

    IslandBuilder<Real> builder;
    IslandSet set;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, {{0u, 1u}, {1u, 2u}}, builder);

    ASSERT_EQ(set.islands.size(), 1u);
    EXPECT_FALSE(set.islands[0].sleeping);
    EXPECT_FALSE(has_any_flag(bodies[0].flags, BodyFlags::sleeping));
}

TEST(Unit_Islands, ABodyThatMayNeverSleepKeepsItsIslandAwake)
{
    std::vector<Body> bodies(2, dynamic_body());
    bodies[1].flags |= BodyFlags::never_sleep;

    IslandBuilder<Real> builder;
    IslandSet set;
    for (int tick = 0; tick < 60; ++tick)
        set = build(bodies, {{0u, 1u}}, builder);
    EXPECT_FALSE(set.islands[0].sleeping);
}

TEST(Unit_Islands, SleepingZeroesTheVelocityRatherThanLeavingItToDrift)
{
    std::vector<Body> bodies(1, dynamic_body());
    bodies[0].velocity = Vector3T<Real>{0.0001, 0.0, 0.0};

    IslandBuilder<Real> builder;
    IslandSet set;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, {}, builder);

    ASSERT_TRUE(set.islands[0].sleeping);
    EXPECT_EQ(bodies[0].velocity.x, 0.0);
}

TEST(Unit_Islands, TheRevisionAdvancesOnTransitionsAndNotOnMotion)
{
    // The number a graph composition keys off (§6.6). It must move when an island
    // sleeps or wakes and stay put for every tick in between, or "sleeping islands
    // are dropped from the DynamicGraph" costs a recomposition per frame and is
    // worse than not doing it.
    std::vector<Body> bodies(2, dynamic_body());
    bodies[0].motion_measure = 5.0;
    bodies[1].motion_measure = 5.0;

    IslandBuilder<Real> builder;
    IslandSet set = build(bodies, {{0u, 1u}}, builder);
    const std::uint64_t after_first = set.revision;

    // Awake and moving, for many ticks: nothing recomposes.
    for (int tick = 0; tick < 10; ++tick)
    {
        bodies[0].motion_measure = 5.0;
        bodies[1].motion_measure = 5.0;
        set = build(bodies, {{0u, 1u}}, builder);
    }
    EXPECT_EQ(set.revision, after_first) << "motion alone is not a composition change";

    // Now let them settle: one transition, one advance, and then quiet again.
    bodies[0].motion_measure = 0.0;
    bodies[1].motion_measure = 0.0;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, {{0u, 1u}}, builder);
    ASSERT_TRUE(set.islands[0].sleeping);
    const std::uint64_t after_sleep = set.revision;
    EXPECT_GT(after_sleep, after_first);

    for (int tick = 0; tick < 10; ++tick)
        set = build(bodies, {{0u, 1u}}, builder);
    EXPECT_EQ(set.revision, after_sleep) << "a settled island costs nothing, including this";
}

TEST(Unit_Islands, WakingOneBodyWakesEverythingRestingOnIt)
{
    std::vector<Body> bodies(3, dynamic_body());
    IslandBuilder<Real> builder;
    IslandSet set;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, {{0u, 1u}, {1u, 2u}}, builder);
    ASSERT_TRUE(set.islands[0].sleeping);

    wake_island(bodies.data(), bodies.size(), 2u, set);
    for (const Body& body : bodies)
    {
        EXPECT_FALSE(has_any_flag(body.flags, BodyFlags::sleeping));
        EXPECT_EQ(body.sleep_timer, 0.0);
    }
}

TEST(Unit_Islands, TenThousandSettledBodiesAreTenThousandSleepingIslands)
{
    // The §13.1 scene, at the level this unit is responsible for: the partition
    // itself must not become the cost. Every crate rests on the ground and on
    // nothing else, so nothing conducts and every one of them sleeps alone.
    std::vector<Body> bodies(10001, dynamic_body());
    bodies[0] = static_body();
    std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
    for (std::uint32_t i = 1; i < bodies.size(); ++i)
        edges.emplace_back(0u, i);

    IslandBuilder<Real> builder;
    IslandSet set;
    for (int tick = 0; tick < 40; ++tick)
        set = build(bodies, edges, builder);

    EXPECT_EQ(set.islands.size(), 10000u);
    EXPECT_EQ(set.awake_count, 0u);
    EXPECT_EQ(set.largest, 1u);
}

// ---------------------------------------------------------------------------
// The measure the decision reads
// ---------------------------------------------------------------------------

TEST(Unit_Islands, TheMotionMeasureSmoothsRatherThanSampling)
{
    // A body at the apex of a bounce is momentarily still. An unsmoothed test puts
    // it to sleep in mid-air; this one must still read it as moving.
    Body body = dynamic_body();
    body.velocity = Vector3T<Real>{0.0, 8.0, 0.0};
    for (int tick = 0; tick < 20; ++tick)
        update_motion_measure(body, 1.0 / 60.0);
    const Real moving = body.motion_measure;
    EXPECT_GT(moving, 1.0);

    body.velocity = Vector3T<Real>{0.0, 0.0, 0.0};
    update_motion_measure(body, 1.0 / 60.0);
    EXPECT_GT(body.motion_measure, moving * 0.5) << "one still instant is not settling";

    for (int tick = 0; tick < 120; ++tick)
        update_motion_measure(body, 1.0 / 60.0);
    EXPECT_LT(body.motion_measure, 0.01) << "but staying still is";
}

TEST(Unit_Islands, ABodySpinningOnTheSpotIsNotMistakenForAStillOne)
{
    Body body = dynamic_body();
    body.inv_inertia = Vector3T<Real>{0.1, 0.1, 0.1}; // a substantial body
    body.angular_velocity = Vector3T<Real>{0.0, 12.0, 0.0};
    for (int tick = 0; tick < 60; ++tick)
        update_motion_measure(body, 1.0 / 60.0);
    EXPECT_GT(body.motion_measure, 0.5);
}

TEST(Unit_Islands, TheSmoothingIsATimeConstantAndNotAPerCallFraction)
{
    // The same second of stillness must decay the measure by the same amount
    // whether it was sampled at 60 Hz or at 240 — otherwise the substep count,
    // which is a quality dial derived from state (§6.2), would change when bodies
    // fall asleep.
    Body coarse = dynamic_body();
    Body fine = dynamic_body();
    coarse.motion_measure = 1.0;
    fine.motion_measure = 1.0;
    for (int tick = 0; tick < 60; ++tick)
        update_motion_measure(coarse, 1.0 / 60.0);
    for (int tick = 0; tick < 240; ++tick)
        update_motion_measure(fine, 1.0 / 240.0);
    EXPECT_NEAR(coarse.motion_measure, fine.motion_measure, 1e-6);
}
