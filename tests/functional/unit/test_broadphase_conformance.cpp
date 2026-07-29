/**************************************************************************/
/* test_broadphase_conformance.cpp                                        */
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

// Unit_Broadphase*: the §4.4 substitutability suite for `IBroadphase`, plus the
// hierarchy's own invariants.
//
// The suite's shape is the point. Every scene here is run against *both*
// implementations and their sorted pair sets are required to be equal — not
// "compatible", not "a superset" — because that is the promise that lets the
// production hierarchy replace the reference sweep without anything downstream
// noticing. A test that only exercised the fast one would be testing an
// implementation; this tests the seam.
//
// Beneath that sit the tree's own invariants (parent links, enclosure, height) and
// the pair cache's three streams, which are what warm starting actually consumes.

#include <algorithm>
#include <cstdint>
#include <random>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/broadphase.hpp>
#include <SushiEngine/physics/collision/bvh_broadphase.hpp>
#include <SushiEngine/physics/collision/dynamic_bvh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;

    Aabb<Real> cube(Real x, Real y, Real z, Real half)
    {
        return Aabb<Real>{Vector3T<Real>{x - half, y - half, z - half},
                          Vector3T<Real>{x + half, y + half, z + half}};
    }

    Vector3T<Real> vec(Real x, Real y, Real z) { return Vector3T<Real>{x, y, z}; }

    /** @brief One body in a generated scene: where it is and how it moves. */
    struct Sample
    {
        Aabb<Real> bounds;
        Vector3T<Real> velocity;
        CollisionFilter filter;
        std::uint32_t flags = 0;
    };

    /** @brief A deterministic scene of overlapping and separated boxes. */
    std::vector<Sample> make_scene(std::size_t count, unsigned seed)
    {
        std::mt19937 engine(seed);
        std::uniform_real_distribution<double> position(-20.0, 20.0);
        std::uniform_real_distribution<double> size(0.3, 2.0);
        std::uniform_real_distribution<double> speed(-3.0, 3.0);

        std::vector<Sample> scene;
        scene.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            Sample sample;
            sample.bounds = cube(position(engine), position(engine), position(engine),
                                 size(engine));
            sample.velocity = vec(speed(engine), speed(engine), speed(engine));
            scene.push_back(sample);
        }
        return scene;
    }

    /** @brief Fills a broadphase with a scene; returns the proxies in scene order. */
    std::vector<ProxyId> populate(IBroadphase<Real>& broadphase,
                                  const std::vector<Sample>& scene)
    {
        std::vector<ProxyId> proxies;
        proxies.reserve(scene.size());
        for (std::uint32_t i = 0; i < scene.size(); ++i)
            proxies.push_back(broadphase.create_proxy(scene[i].bounds, scene[i].filter,
                                                      scene[i].flags, i));
        return proxies;
    }

    /** @brief Every overlapping pair, by the definition, with nothing clever. */
    std::vector<BroadphasePair> brute_force(const IBroadphase<Real>& broadphase,
                                            const std::vector<ProxyId>& proxies)
    {
        std::vector<BroadphasePair> pairs;
        for (std::size_t i = 0; i < proxies.size(); ++i)
            for (std::size_t j = i + 1; j < proxies.size(); ++j)
            {
                const BroadphaseProxy<Real>& a = broadphase.proxy(proxies[i]);
                const BroadphaseProxy<Real>& b = broadphase.proxy(proxies[j]);
                if (!a.alive || !b.alive)
                    continue;
                if (!proxies_may_collide(a, b))
                    continue;
                if (aabb_overlap(a.bounds, b.bounds))
                    pairs.push_back(BroadphasePair::make(proxies[i], proxies[j]));
            }
        std::sort(pairs.begin(), pairs.end());
        return pairs;
    }
} // namespace

// ---------------------------------------------------------------------------
// The tree
// ---------------------------------------------------------------------------

TEST(Unit_DynamicBvh, StaysWellFormedThroughInsertionAndRemoval)
{
    DynamicBvh<Real> tree;
    std::vector<std::uint32_t> nodes;
    const std::vector<Sample> scene = make_scene(256, 11u);
    for (std::uint32_t i = 0; i < scene.size(); ++i)
    {
        nodes.push_back(tree.insert(scene[i].bounds, i));
        ASSERT_TRUE(tree.validate()) << "after inserting " << i;
    }
    EXPECT_EQ(tree.leaf_count(), scene.size());

    // A balanced tree of 256 leaves is 8 deep. The rotation on the insertion path
    // is what keeps it near that; without it a sorted stream builds a comb, and
    // this bound is the assertion that catches its removal.
    EXPECT_LE(tree.height(), 16);

    for (std::size_t i = 0; i < nodes.size(); i += 2)
    {
        tree.remove(nodes[i]);
        ASSERT_TRUE(tree.validate()) << "after removing " << i;
    }
    EXPECT_EQ(tree.leaf_count(), scene.size() / 2);
}

TEST(Unit_DynamicBvh, SortedInsertionDoesNotDegenerateIntoAList)
{
    // The case rotation exists for: a terrain grid loaded row by row, every box
    // further along X than the last. An unbalanced tree would be 512 deep here.
    DynamicBvh<Real> tree;
    for (std::uint32_t i = 0; i < 512; ++i)
        tree.insert(cube(Real(i), 0, 0, 0.4), i);
    EXPECT_TRUE(tree.validate());
    EXPECT_LE(tree.height(), 24);
}

TEST(Unit_DynamicBvh, QueryFindsExactlyTheOverlappingLeaves)
{
    DynamicBvh<Real> tree;
    const std::vector<Sample> scene = make_scene(180, 23u);
    for (std::uint32_t i = 0; i < scene.size(); ++i)
        tree.insert(scene[i].bounds, i);

    const Aabb<Real> box = cube(3, -2, 5, 4);
    std::vector<std::uint32_t> found;
    tree.query(box, [&](std::uint32_t payload, std::uint32_t) { found.push_back(payload); });
    std::sort(found.begin(), found.end());

    std::vector<std::uint32_t> expected;
    for (std::uint32_t i = 0; i < scene.size(); ++i)
        if (aabb_overlap(scene[i].bounds, box))
            expected.push_back(i);

    EXPECT_EQ(found, expected);
}

TEST(Unit_DynamicBvh, RayQueryAgreesWithTheSlabTestOnEveryLeaf)
{
    DynamicBvh<Real> tree;
    const std::vector<Sample> scene = make_scene(150, 37u);
    for (std::uint32_t i = 0; i < scene.size(); ++i)
        tree.insert(scene[i].bounds, i);

    const Vector3T<Real> origin = vec(-40, 0, 0);
    const Vector3T<Real> direction = vec(1, 0.1, 0.05);
    const Real reach = 90;

    std::vector<std::uint32_t> found;
    tree.query_ray(origin, direction, reach,
                   [&](std::uint32_t payload, std::uint32_t) { found.push_back(payload); });
    std::sort(found.begin(), found.end());

    std::vector<std::uint32_t> expected;
    const Vector3T<Real> inverse = ray_inverse_direction(direction);
    for (std::uint32_t i = 0; i < scene.size(); ++i)
        if (ray_hits_aabb(scene[i].bounds, origin, inverse, reach))
            expected.push_back(i);

    EXPECT_EQ(found, expected);
    EXPECT_FALSE(expected.empty());
}

TEST(Unit_DynamicBvh, AnAxisParallelRayDoesNotBecomeANotANumber)
{
    // The `0 * infinity` case: a ray straight down X with exactly zero Y and Z.
    // Computed with a true infinity this misses everything, silently.
    DynamicBvh<Real> tree;
    tree.insert(cube(5, 0, 0, 1), 7);
    std::vector<std::uint32_t> found;
    tree.query_ray(vec(-10, 0, 0), vec(1, 0, 0), 100,
                   [&](std::uint32_t payload, std::uint32_t) { found.push_back(payload); });
    ASSERT_EQ(found.size(), 1u);
    EXPECT_EQ(found[0], 7u);
}

TEST(Unit_DynamicBvh, AProxyInsideItsStoredBoxIsNotReinserted)
{
    DynamicBvh<Real> tree;
    const std::uint32_t node = tree.insert(cube(0, 0, 0, 2), 0);
    EXPECT_FALSE(tree.move_proxy(node, cube(0.5, 0, 0, 1), cube(0.5, 0, 0, 2)));
    EXPECT_TRUE(tree.move_proxy(node, cube(9, 0, 0, 1), cube(9, 0, 0, 2)));
    EXPECT_TRUE(tree.validate());
}

// ---------------------------------------------------------------------------
// The seam: both implementations, one answer
// ---------------------------------------------------------------------------

TEST(Unit_BroadphaseConformance, BothImplementationsEmitTheSamePairSet)
{
    const std::vector<Sample> scene = make_scene(300, 5u);
    SweepAndPruneBroadphase<Real> sweep;
    BvhBroadphase<Real> hierarchy;

    const std::vector<ProxyId> a = populate(sweep, scene);
    const std::vector<ProxyId> b = populate(hierarchy, scene);
    ASSERT_EQ(a, b) << "proxy identifiers must be assigned identically for the "
                       "pair sets to be comparable at all";

    sweep.update();
    hierarchy.update();

    EXPECT_EQ(sweep.pairs(), hierarchy.pairs());
    EXPECT_EQ(sweep.pairs(), brute_force(sweep, a));
    EXPECT_FALSE(sweep.pairs().empty());
    EXPECT_TRUE(hierarchy.dynamic_tree().validate());
}

TEST(Unit_BroadphaseConformance, TheyAgreeThroughMotionRemovalAndReinsertion)
{
    std::vector<Sample> scene = make_scene(200, 91u);
    SweepAndPruneBroadphase<Real> sweep;
    BvhBroadphase<Real> hierarchy;
    const std::vector<ProxyId> proxies = populate(sweep, scene);
    populate(hierarchy, scene);

    const Real dt = 1.0 / 60.0;
    for (int tick = 0; tick < 20; ++tick)
    {
        for (std::size_t i = 0; i < scene.size(); ++i)
        {
            const Vector3T<Real> step = scene[i].velocity * dt;
            scene[i].bounds.min = scene[i].bounds.min + step;
            scene[i].bounds.max = scene[i].bounds.max + step;
            sweep.update_proxy(proxies[i], scene[i].bounds, scene[i].velocity * dt);
            hierarchy.update_proxy(proxies[i], scene[i].bounds, scene[i].velocity * dt);
        }
        if (tick == 7)
        {
            // Churn: the same proxies removed from both, then new ones added, so
            // the identifier reuse has to land the same way in both structures.
            for (std::size_t i = 0; i < scene.size(); i += 5)
            {
                sweep.destroy_proxy(proxies[i]);
                hierarchy.destroy_proxy(proxies[i]);
            }
            for (int extra = 0; extra < 12; ++extra)
            {
                const Aabb<Real> box = cube(Real(extra) - 6, 1, 2, 0.9);
                const ProxyId left = sweep.create_proxy(box, CollisionFilter{}, 0u, 900u);
                const ProxyId right = hierarchy.create_proxy(box, CollisionFilter{}, 0u, 900u);
                ASSERT_EQ(left, right);
            }
        }

        sweep.update();
        hierarchy.update();
        ASSERT_EQ(sweep.pairs(), hierarchy.pairs()) << "tick " << tick;
        ASSERT_EQ(sweep.added_pairs(), hierarchy.added_pairs()) << "tick " << tick;
        ASSERT_EQ(sweep.removed_pairs(), hierarchy.removed_pairs()) << "tick " << tick;
        ASSERT_TRUE(hierarchy.dynamic_tree().validate()) << "tick " << tick;
    }
}

// ---------------------------------------------------------------------------
// The pair cache
// ---------------------------------------------------------------------------

TEST(Unit_BroadphasePairCache, ReportsAddedThenPersistedThenRemoved)
{
    BvhBroadphase<Real> broadphase;
    broadphase.set_enlargement(0.0, 0.0); // no hysteresis, so the streams are exact
    const ProxyId a = broadphase.create_proxy(cube(0, 0, 0, 1), CollisionFilter{}, 0u, 0u);
    const ProxyId b = broadphase.create_proxy(cube(1.5, 0, 0, 1), CollisionFilter{}, 0u, 1u);

    broadphase.update();
    ASSERT_EQ(broadphase.pairs().size(), 1u);
    EXPECT_EQ(broadphase.added_pairs().size(), 1u);
    EXPECT_TRUE(broadphase.persisted_pairs().empty());
    EXPECT_TRUE(broadphase.removed_pairs().empty());

    // Unchanged: the pair persists, which is the signal that says "keep the
    // manifold and its warm-start impulses".
    broadphase.update();
    EXPECT_TRUE(broadphase.added_pairs().empty());
    ASSERT_EQ(broadphase.persisted_pairs().size(), 1u);
    EXPECT_EQ(broadphase.persisted_pairs()[0], (BroadphasePair::make(a, b)));

    // Moved apart: removed once, and then not reported again.
    broadphase.update_proxy(b, cube(9, 0, 0, 1), vec(0, 0, 0));
    broadphase.update();
    EXPECT_TRUE(broadphase.pairs().empty());
    ASSERT_EQ(broadphase.removed_pairs().size(), 1u);
    broadphase.update();
    EXPECT_TRUE(broadphase.removed_pairs().empty());
}

TEST(Unit_BroadphasePairCache, HysteresisKeepsAStationaryBodyOutOfTheTree)
{
    BvhBroadphase<Real> broadphase;
    broadphase.set_enlargement(0.25, 0.0);
    const ProxyId proxy = broadphase.create_proxy(cube(0, 0, 0, 1), CollisionFilter{}, 0u, 0u);
    const Aabb<Real> stored = broadphase.proxy(proxy).bounds;

    // A body jittering inside its margin never restructures anything.
    broadphase.update_proxy(proxy, cube(0.1, 0.05, -0.08, 1), vec(0, 0, 0));
    EXPECT_EQ(broadphase.proxy(proxy).bounds.min.x, stored.min.x);

    // One that leaves it does.
    broadphase.update_proxy(proxy, cube(0.9, 0, 0, 1), vec(0, 0, 0));
    EXPECT_GT(broadphase.proxy(proxy).bounds.min.x, stored.min.x);
}

// ---------------------------------------------------------------------------
// Filters, layers, and flags (§5.5, §4.4)
// ---------------------------------------------------------------------------

TEST(Unit_BroadphaseFilters, LayersRejectAPairBeforeAnyGeometryIsConsulted)
{
    const std::uint32_t debris = 1u << 1;
    const std::uint32_t character = 1u << 2;

    CollisionFilter debris_filter;
    debris_filter.layer = debris;
    debris_filter.collides_with = ~debris; // debris ignores other debris
    CollisionFilter character_filter;
    character_filter.layer = character;

    for (int implementation = 0; implementation < 2; ++implementation)
    {
        SweepAndPruneBroadphase<Real> sweep;
        BvhBroadphase<Real> hierarchy;
        IBroadphase<Real>& broadphase =
            implementation == 0 ? static_cast<IBroadphase<Real>&>(sweep)
                                : static_cast<IBroadphase<Real>&>(hierarchy);

        broadphase.create_proxy(cube(0, 0, 0, 1), debris_filter, 0u, 0u);
        broadphase.create_proxy(cube(0.5, 0, 0, 1), debris_filter, 0u, 1u);
        broadphase.create_proxy(cube(0.5, 0, 0, 1), character_filter, 0u, 2u);
        broadphase.update();

        // Three overlapping boxes, but the two pieces of debris ignore each other.
        ASSERT_EQ(broadphase.pairs().size(), 2u) << "implementation " << implementation;
        for (const BroadphasePair& pair : broadphase.pairs())
            EXPECT_TRUE(pair.a == 2u || pair.b == 2u);
    }
}

TEST(Unit_BroadphaseFilters, TwoQuietBodiesNeverPairButAWakingOneDoes)
{
    BvhBroadphase<Real> broadphase;
    const ProxyId ground =
        broadphase.create_proxy(cube(0, 0, 0, 5), CollisionFilter{}, BodyFlags::static_body, 0u);
    const ProxyId sleeper =
        broadphase.create_proxy(cube(0, 4, 0, 1), CollisionFilter{}, BodyFlags::sleeping, 1u);

    broadphase.update();
    EXPECT_TRUE(broadphase.pairs().empty());

    // This is what "a settled island costs nothing" is, at the smallest scale:
    // the pair does not exist until something wakes.
    broadphase.set_proxy_state(sleeper, CollisionFilter{}, 0u);
    broadphase.update();
    ASSERT_EQ(broadphase.pairs().size(), 1u);
    EXPECT_EQ(broadphase.pairs()[0], (BroadphasePair::make(ground, sleeper)));
}

TEST(Unit_BroadphaseFilters, StaticProxiesLiveInTheirOwnTreeAndMoveWhenTheyStopBeingStatic)
{
    BvhBroadphase<Real> broadphase;
    broadphase.create_proxy(cube(0, 0, 0, 5), CollisionFilter{}, BodyFlags::static_body, 0u);
    const ProxyId crate =
        broadphase.create_proxy(cube(0, 6, 0, 1), CollisionFilter{}, BodyFlags::static_body, 1u);
    EXPECT_EQ(broadphase.static_tree().leaf_count(), 2u);
    EXPECT_EQ(broadphase.dynamic_tree().leaf_count(), 0u);

    broadphase.set_proxy_state(crate, CollisionFilter{}, 0u);
    EXPECT_EQ(broadphase.static_tree().leaf_count(), 1u);
    EXPECT_EQ(broadphase.dynamic_tree().leaf_count(), 1u);
    EXPECT_TRUE(broadphase.static_tree().validate());
    EXPECT_TRUE(broadphase.dynamic_tree().validate());
}

TEST(Unit_BroadphaseFilters, ATriggerStillPairs)
{
    // A trigger refuses the impulse, not the pair — it has to hear about the
    // overlap to report it (§7.7).
    BvhBroadphase<Real> broadphase;
    broadphase.create_proxy(cube(0, 0, 0, 1), CollisionFilter{}, BodyFlags::trigger, 0u);
    broadphase.create_proxy(cube(0.5, 0, 0, 1), CollisionFilter{}, 0u, 1u);
    broadphase.update();
    EXPECT_EQ(broadphase.pairs().size(), 1u);
}

// ---------------------------------------------------------------------------
// Continuous collision: the pair exists before the impact
// ---------------------------------------------------------------------------

TEST(Unit_BroadphaseSweptBounds, AFastBodyPairsWithWhatItIsAboutToHit)
{
    const Vector3T<Real> travel = vec(6, 0, 0); // this tick's motion, six metres

    BvhBroadphase<Real> ordinary;
    ordinary.set_enlargement(0.05, 0.0); // no prediction, to isolate the sweep
    const ProxyId bullet =
        ordinary.create_proxy(cube(0, 0, 0, 0.1), CollisionFilter{}, 0u, 0u);
    ordinary.create_proxy(cube(5, 0, 0, 0.5), CollisionFilter{}, BodyFlags::static_body, 1u);
    ordinary.update_proxy(bullet, cube(0, 0, 0, 0.1), travel);
    ordinary.update();
    EXPECT_TRUE(ordinary.pairs().empty()) << "without the sweep there is nothing to solve "
                                             "against until the bullet is already past";

    BvhBroadphase<Real> continuous;
    continuous.set_enlargement(0.05, 0.0);
    const ProxyId fast = continuous.create_proxy(cube(0, 0, 0, 0.1), CollisionFilter{},
                                                 BodyFlags::continuous_collision, 0u);
    continuous.create_proxy(cube(5, 0, 0, 0.5), CollisionFilter{}, BodyFlags::static_body, 1u);
    continuous.update_proxy(fast, cube(0, 0, 0, 0.1), travel);
    continuous.update();
    EXPECT_EQ(continuous.pairs().size(), 1u);
}
