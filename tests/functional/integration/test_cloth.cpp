/**************************************************************************/
/* test_cloth.cpp                                                        */
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

// Integration_Cloth: SushiLoop M5's cloth grid (physics/cloth.hpp), against the
// real runtime. build_cloth_grid must wire the expected topology (structural +
// shear constraints, only the pinned top row has zero inverse mass) and, under
// gravity, the pinned row must never move while the rest of the grid settles
// under its own constraints — the same "pinned body never moves" invariant
// Integration_PhysicsWorld already proves for a single link, generalized to a grid.

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr std::size_t ROWS       = 4;
    constexpr std::size_t COLS       = 5;
    constexpr Scalar      SPACING    = Scalar(0.25);
    constexpr Scalar      GRAVITY_Y  = Scalar(-9.8);
    constexpr Scalar      SUBSTEP_DT = Scalar(0.005);
    constexpr std::size_t SUBSTEPS_PER_FRAME = 4;
    constexpr std::size_t FRAMES     = 300;
    constexpr std::size_t ITERATIONS = 12;
}

TEST(Integration_Cloth, GridTopologyHasExpectedBodyAndConstraintCounts)
{
    PhysicsWorld<XpbdDistanceConstraint> world(Harness::shared_runtime());
    const ClothGrid grid =
        build_cloth_grid(world, ROWS, COLS, SPACING, Vector3{0, 0, 0}, Scalar(0));

    ASSERT_EQ(grid.rows, ROWS);
    ASSERT_EQ(grid.cols, COLS);
    EXPECT_EQ(world.body_count(), ROWS * COLS);

    world.finalize(ITERATIONS, SUBSTEP_DT, XpbdDistanceProjection{});

    // Every row-0 body is pinned; every other body is free.
    for (std::size_t col = 0; col < COLS; ++col)
        EXPECT_EQ(world.body(grid.at(0, col)).inv_mass, Scalar(0));
    for (std::size_t row = 1; row < ROWS; ++row)
        for (std::size_t col = 0; col < COLS; ++col)
            EXPECT_EQ(world.body(grid.at(row, col)).inv_mass, Scalar(1));
}

TEST(Integration_Cloth, PinnedTopRowNeverMovesWhileGridHangs)
{
    PhysicsWorld<XpbdDistanceConstraint> world(Harness::shared_runtime());
    const ClothGrid grid =
        build_cloth_grid(world, ROWS, COLS, SPACING, Vector3{0, 0, 0}, Scalar(0));
    world.finalize(ITERATIONS, SUBSTEP_DT, XpbdDistanceProjection{});

    std::vector<Vector3> pinned_start;
    for (std::size_t col = 0; col < COLS; ++col)
        pinned_start.push_back(world.body(grid.at(0, col)).position);

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
        world.step(Vector3{0, GRAVITY_Y, 0}, SUBSTEPS_PER_FRAME);

    EXPECT_EQ(world.compile_count(), 1u);

    for (std::size_t col = 0; col < COLS; ++col)
        EXPECT_TRUE(Harness::approx_equal(world.body(grid.at(0, col)).position,
                                          pinned_start[col], Scalar(1e-6)));

    // The bottom row must have fallen below the pinned row.
    for (std::size_t col = 0; col < COLS; ++col)
        EXPECT_LT(world.body(grid.at(ROWS - 1, col)).position.y, Scalar(0));

    // Structural constraints must be close to satisfied (loose tolerance: XPBD with
    // finite iterations does not reach zero residual, only that the grid is not
    // wildly stretched).
    for (std::size_t row = 0; row < ROWS; ++row)
        for (std::size_t col = 0; col + 1 < COLS; ++col)
        {
            const Vector3 d = world.body(grid.at(row, col)).position -
                           world.body(grid.at(row, col + 1)).position;
            const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
            EXPECT_NEAR(double(dist), double(SPACING), 0.05);
        }
}
