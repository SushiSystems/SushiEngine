/**************************************************************************/
/* test_graph_coloring.cpp                                               */
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

// Unit_GraphColoring: the greedy edge-colouring that makes a Gauss-Seidel sweep
// parallel. The two properties that matter for correctness are that every colour
// is conflict-free (no shared body within a colour) and that every constraint is
// coloured exactly once; the colour count is the sweep's sequential depth.

#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraint.hpp>
#include <SushiEngine/physics/graph_coloring.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    // Asserts no colour reuses a body and every constraint index appears once.
    void check_partition_is_valid(const ColorBatches& colors,
                                  const std::vector<DistanceConstraint>& constraints)
    {
        std::set<std::uint32_t> seen;
        for (const std::vector<std::uint32_t>& batch : colors)
        {
            std::set<std::uint32_t> bodies;
            for (std::uint32_t index : batch)
            {
                EXPECT_TRUE(seen.insert(index).second) << "constraint " << index << " coloured twice";
                const DistanceConstraint& c = constraints[index];
                EXPECT_TRUE(bodies.insert(c.a).second) << "body " << c.a << " reused within a colour";
                EXPECT_TRUE(bodies.insert(c.b).second) << "body " << c.b << " reused within a colour";
            }
        }
        EXPECT_EQ(seen.size(), constraints.size());
    }
}

TEST(Unit_GraphColoring, ChainPartitionsIntoTwoColors)
{
    constexpr std::uint32_t N = 32;
    std::vector<DistanceConstraint> chain;
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        chain.push_back(DistanceConstraint{i, i + 1, Scalar(1)});

    const ColorBatches colors = color_constraints(chain, N);

    EXPECT_EQ(colors.size(), 2u);
    check_partition_is_valid(colors, chain);
}

TEST(Unit_GraphColoring, StarNeedsOneColorPerEdge)
{
    // Every constraint touches body 0, so no two can share a colour.
    constexpr std::uint32_t N = 6;
    std::vector<DistanceConstraint> star;
    for (std::uint32_t i = 1; i < N; ++i)
        star.push_back(DistanceConstraint{0, i, Scalar(1)});

    const ColorBatches colors = color_constraints(star, N);

    EXPECT_EQ(colors.size(), star.size());
    check_partition_is_valid(colors, star);
}

TEST(Unit_GraphColoring, EmptyConstraintsProduceNoColors)
{
    const ColorBatches colors = color_constraints(std::vector<DistanceConstraint>{}, 4);
    EXPECT_TRUE(colors.empty());
}
