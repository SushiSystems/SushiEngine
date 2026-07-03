/**************************************************************************/
/* pgs_demo.cpp                                                          */
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

// WP-3 physics worked example: a hanging chain solved with Projected Gauss-Seidel,
// validated against an independent scalar reference. It proves the solver claim:
//
//   * Distance constraints between consecutive particles are graph-coloured into
//     conflict-free batches; a chain yields exactly two colours.
//   * Each colour becomes one parallel task; because every colour reads and writes
//     the shared position array, the runtime orders the colours into a sequential
//     Gauss-Seidel sweep, repeated for the iteration count. The whole solve is
//     compiled once and replayed every frame (asserted: compile_count == 1).
//   * The device result matches a scalar loop running the same colours in the same
//     order, so the parallel-within-colour / sequential-across-colour equivalence
//     to Gauss-Seidel is a verified claim.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr std::size_t N          = 64;          // particles in the chain
    constexpr Scalar      SPACING    = Scalar(0.5); // rest length between neighbours
    constexpr std::size_t ITERATIONS = 20;          // Gauss-Seidel sweeps per frame
    constexpr std::size_t FRAMES     = 200;
    constexpr Scalar      GY         = Scalar(-9.8); // gravity
    constexpr Scalar      DT         = Scalar(0.016);

    // Host mirror of DistanceProjection, byte-for-byte the same arithmetic.
    void project_host(const DistanceConstraint& c, std::vector<Vector3>& pos,
                      const std::vector<Scalar>& inv_mass)
    {
        const Vector3 pa = pos[c.a];
        const Vector3 pb = pos[c.b];
        const Vector3 d = pa - pb;
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

    Scalar abs_max(Scalar acc, Scalar value) { return std::fabs(value) > acc ? std::fabs(value) : acc; }
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();
    auto positions = runtime.buffer<Vector3>(N);
    auto inv_mass = runtime.buffer<Scalar>(N);

    std::vector<Vector3> ref_pos(N);
    std::vector<Scalar> ref_inv(N);
    std::vector<DistanceConstraint> constraints;

    // A horizontal chain; the first particle is pinned (inverse mass zero).
    for (std::uint32_t i = 0; i < N; ++i)
    {
        const Vector3 p{Scalar(i) * SPACING, Scalar(0), Scalar(0)};
        positions[i] = p;
        ref_pos[i] = p;
        const Scalar w = (i == 0) ? Scalar(0) : Scalar(1);
        inv_mass[i] = w;
        ref_inv[i] = w;
    }
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        constraints.push_back(DistanceConstraint{i, i + 1, SPACING});

    ConstraintSolver<DistanceConstraint> solver(
        runtime, positions, inv_mass, constraints, N, ITERATIONS, DistanceProjection{});

    double solve_ms = 0.0;
    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        // Position-based gravity on the free particles, applied to both worlds.
        for (std::uint32_t i = 0; i < N; ++i)
        {
            if (inv_mass[i] > Scalar(0))
            {
                positions[i].y += GY * DT * DT;
                ref_pos[i].y += GY * DT * DT;
            }
        }

        // Device solve.
        const SushiRuntime::RunReport report = solver.solve();
        solve_ms += report.total_duration_ms;

        // Reference solve: the same colours, in the same order, sequentially.
        for (std::size_t iteration = 0; iteration < ITERATIONS; ++iteration)
            for (const std::vector<std::uint32_t>& batch : solver.colors())
                for (std::uint32_t k : batch)
                    project_host(constraints[k], ref_pos, ref_inv);
    }

    // Compare the device world to the reference, and report constraint satisfaction.
    Scalar max_error = Scalar(0);
    for (std::uint32_t i = 0; i < N; ++i)
    {
        const Vector3 p = positions[i];
        max_error = abs_max(max_error, p.x - ref_pos[i].x);
        max_error = abs_max(max_error, p.y - ref_pos[i].y);
        max_error = abs_max(max_error, p.z - ref_pos[i].z);
    }

    Scalar max_residual = Scalar(0);
    for (const DistanceConstraint& c : constraints)
    {
        const Vector3 d = positions[c.a] - positions[c.b];
        const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        max_residual = abs_max(max_residual, dist - c.rest_length);
    }

    const Scalar tol = Scalar(0.02);
    const std::size_t compiles = solver.compile_count();
    const std::size_t colors = solver.color_count();

    std::printf("bodies=%zu  constraints=%zu  colors=%zu (expected 2)  iterations=%zu\n",
                N, constraints.size(), colors, ITERATIONS);
    std::printf("compile_count=%zu (expected 1)  max_error_vs_reference=%.6f (tol %.3f)\n",
                compiles, double(max_error), double(tol));
    std::printf("max_constraint_residual=%.6f  solve: %.3f ms over %zu frames, %.4f ms/frame\n",
                double(max_residual), solve_ms, FRAMES,
                FRAMES ? solve_ms / double(FRAMES) : 0.0);

    const bool ok = (max_error < tol) && (compiles == 1) && (colors == 2);
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
