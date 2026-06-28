/**************************************************************************/
/* test_pgs_solver.cpp                                                   */
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

// Integration_PgsSolver: the graph-coloured Projected Gauss-Seidel solver against
// the real runtime. A hanging chain is solved on the device and compared to a
// scalar reference running the same colours in the same order; the parallel-within-
// colour / sequential-across-colour scheme must equal Gauss-Seidel, the chain must
// colour into two batches, and the graph must compile exactly once.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr std::uint32_t N          = 48;
    constexpr Scalar        SPACING    = Scalar(0.5);
    constexpr std::size_t   ITERATIONS = 20;
    constexpr std::size_t   FRAMES     = 100;
    constexpr Scalar        GY         = Scalar(-9.8);
    constexpr Scalar        DT         = Scalar(0.016);

    // Byte-for-byte host mirror of DistanceProjection.
    void project_host(const DistanceConstraint& c, std::vector<Vec3>& pos,
                      const std::vector<Scalar>& inv_mass)
    {
        const Vec3 pa = pos[c.a];
        const Vec3 pb = pos[c.b];
        const Vec3 d = pa - pb;
        const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (dist <= Scalar(1e-8))
            return;
        const Scalar wa = inv_mass[c.a];
        const Scalar wb = inv_mass[c.b];
        const Scalar w = wa + wb;
        if (w <= Scalar(0))
            return;
        const Scalar s = (dist - c.rest_length) / (dist * w);
        pos[c.a] = pa - d * (wa * s);
        pos[c.b] = pb + d * (wb * s);
    }
}

TEST(Integration_PgsSolver, HangingChainMatchesReference)
{
    auto& runtime = Harness::shared_runtime();
    auto positions = runtime.buffer<Vec3>(N);
    auto inv_mass = runtime.buffer<Scalar>(N);

    std::vector<Vec3> ref_pos(N);
    std::vector<Scalar> ref_inv(N);
    std::vector<DistanceConstraint> constraints;

    for (std::uint32_t i = 0; i < N; ++i)
    {
        const Vec3 p{Scalar(i) * SPACING, Scalar(0), Scalar(0)};
        positions[i] = p;
        ref_pos[i] = p;
        const Scalar w = (i == 0) ? Scalar(0) : Scalar(1); // pin the first body
        inv_mass[i] = w;
        ref_inv[i] = w;
    }
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        constraints.push_back(DistanceConstraint{i, i + 1, SPACING});

    ConstraintSolver<DistanceConstraint> solver(
        runtime, positions, inv_mass, constraints, N, ITERATIONS, DistanceProjection{});

    EXPECT_EQ(solver.color_count(), 2u);

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        for (std::uint32_t i = 0; i < N; ++i)
            if (inv_mass[i] > Scalar(0))
            {
                positions[i].y += GY * DT * DT;
                ref_pos[i].y += GY * DT * DT;
            }

        solver.solve();

        for (std::size_t iteration = 0; iteration < ITERATIONS; ++iteration)
            for (const std::vector<std::uint32_t>& batch : solver.colors())
                for (std::uint32_t k : batch)
                    project_host(constraints[k], ref_pos, ref_inv);
    }

    EXPECT_EQ(solver.compile_count(), 1u);

    const Scalar tol = Scalar(0.02);
    for (std::uint32_t i = 0; i < N; ++i)
    {
        const Vec3 p = positions[i];
        EXPECT_TRUE(Harness::approx_equal(p, ref_pos[i], tol))
            << "body " << i << " diverged from the reference";
    }

    // The chain must also actually satisfy its constraints (rest length held).
    Scalar max_residual = Scalar(0);
    for (const DistanceConstraint& c : constraints)
    {
        const Vec3 pa = positions[c.a];
        const Vec3 pb = positions[c.b];
        const Vec3 d = pa - pb;
        const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        max_residual = std::max(max_residual, std::fabs(dist - c.rest_length));
    }
    EXPECT_LT(max_residual, Scalar(0.1));
}
