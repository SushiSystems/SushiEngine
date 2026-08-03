/**************************************************************************/
/* test_broadphase.cpp                                                    */
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

// Unit_Broadphase: the sweep-and-prune pair cull (physics/broadphase.hpp). The
// narrowphase is exact but costs one test per pair, so everything downstream trusts
// this to drop non-overlapping pairs *without* dropping overlapping ones — a false
// negative here is a body that silently falls through another. The tests pin both
// directions (no missed overlaps, no spurious pairs), the emitted index convention,
// and the axis asymmetry that comes from sweeping X only.

#include <algorithm>
#include <cstdint>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/broadphase.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Pair = std::pair<std::uint32_t, std::uint32_t>;

    /** @brief A cube AABB of half-size @p half centred on (@p x, @p y, @p z). */
    AABB<Scalar> cube(Scalar x, Scalar y, Scalar z, Scalar half)
    {
        return AABB<Scalar>{Vector3{x - half, y - half, z - half},
                            Vector3{x + half, y + half, z + half}};
    }

    bool contains(const std::vector<Pair>& pairs, std::uint32_t a, std::uint32_t b)
    {
        return std::find(pairs.begin(), pairs.end(), Pair{a, b}) != pairs.end();
    }
}

TEST(Unit_Broadphase, AABBOverlapNeedsAllThreeAxes)
{
    const AABB<Scalar> origin = cube(0, 0, 0, Scalar(1));
    EXPECT_TRUE(aabb_overlap(origin, cube(Scalar(1.5), 0, 0, Scalar(1))));
    EXPECT_FALSE(aabb_overlap(origin, cube(Scalar(2.5), 0, 0, Scalar(1))));
    // Overlapping in x and y but not z is still a miss.
    EXPECT_FALSE(aabb_overlap(origin, cube(Scalar(0.5), Scalar(0.5), Scalar(2.5), Scalar(1))));
    // Touching exactly counts as overlapping (closed intervals).
    EXPECT_TRUE(aabb_overlap(origin, cube(Scalar(2), 0, 0, Scalar(1))));
}

TEST(Unit_Broadphase, SweepEmitsNothingBelowTwoBoxes)
{
    std::vector<Pair> pairs{{7, 9}}; // must be cleared on entry
    std::vector<AABB<Scalar>> boxes;
    sweep_and_prune(boxes, pairs);
    EXPECT_TRUE(pairs.empty());

    boxes.push_back(cube(0, 0, 0, Scalar(1)));
    sweep_and_prune(boxes, pairs);
    EXPECT_TRUE(pairs.empty());
}

TEST(Unit_Broadphase, SweepDropsSpatiallySeparatedBodies)
{
    std::vector<AABB<Scalar>> boxes;
    for (int i = 0; i < 16; ++i)
        boxes.push_back(cube(Scalar(i) * Scalar(10), 0, 0, Scalar(1)));

    std::vector<Pair> pairs;
    sweep_and_prune(boxes, pairs);
    EXPECT_TRUE(pairs.empty());
}

TEST(Unit_Broadphase, SweepFindsEveryPairInAFullyOverlappingCluster)
{
    constexpr std::size_t COUNT = 8;
    std::vector<AABB<Scalar>> boxes;
    for (std::size_t i = 0; i < COUNT; ++i)
        boxes.push_back(cube(0, 0, 0, Scalar(1)));

    std::vector<Pair> pairs;
    sweep_and_prune(boxes, pairs);
    EXPECT_EQ(pairs.size(), COUNT * (COUNT - 1) / 2);
    for (std::uint32_t i = 0; i < COUNT; ++i)
        for (std::uint32_t j = i + 1; j < COUNT; ++j)
            EXPECT_TRUE(contains(pairs, i, j)) << "missing pair " << i << "," << j;
}

TEST(Unit_Broadphase, SweepRejectsPairsOverlappingOnlyInX)
{
    // The sweep axis is X, so these two survive the sweep front and must then be
    // rejected by the full three-axis test rather than emitted.
    std::vector<AABB<Scalar>> boxes{cube(0, 0, 0, Scalar(1)), cube(0, Scalar(5), 0, Scalar(1))};
    std::vector<Pair> pairs;
    sweep_and_prune(boxes, pairs);
    EXPECT_TRUE(pairs.empty());
}

TEST(Unit_Broadphase, SweepEmitsAscendingIndexPairsRegardlessOfPosition)
{
    // Index 0 sorts *after* index 1 on the sweep axis; the emitted pair must still be
    // ordered by index, because the contact pass indexes its body array with it.
    std::vector<AABB<Scalar>> boxes{cube(Scalar(1), 0, 0, Scalar(1)), cube(0, 0, 0, Scalar(1))};
    std::vector<Pair> pairs;
    sweep_and_prune(boxes, pairs);
    ASSERT_EQ(pairs.size(), 1u);
    EXPECT_EQ(pairs[0].first, 0u);
    EXPECT_EQ(pairs[0].second, 1u);
}

TEST(Unit_Broadphase, SweepMatchesBruteForceOnAMixedScene)
{
    // The property that actually matters: whatever the sweep emits must equal the
    // exhaustive answer. A lattice with deliberate partial overlaps exercises the
    // active-set eviction rather than the trivial all-in or all-out cases.
    std::vector<AABB<Scalar>> boxes;
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 3; ++y)
            boxes.push_back(cube(Scalar(x) * Scalar(1.5), Scalar(y) * Scalar(1.5), 0, Scalar(1)));

    std::vector<Pair> pairs;
    sweep_and_prune(boxes, pairs);

    std::vector<Pair> expected;
    for (std::uint32_t i = 0; i < boxes.size(); ++i)
        for (std::uint32_t j = i + 1; j < boxes.size(); ++j)
            if (aabb_overlap(boxes[i], boxes[j]))
                expected.emplace_back(i, j);

    std::sort(pairs.begin(), pairs.end());
    EXPECT_FALSE(expected.empty()) << "the fixture must actually produce overlaps";
    EXPECT_EQ(pairs, expected);
}
