/**************************************************************************/
/* xpbd_demo.cpp                                                         */
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

// SushiLoop M2 worked example: pgs_demo.cpp's hanging chain, ported from plain
// Projected Gauss-Seidel to the unified rigid-body XPBD solver
// (docs/slop/SUSHILOOP.md). Bodies carry zero inverse inertia and anchors at their
// own centre, so no angular coupling is possible and the chain's linear behaviour
// is the rigid-body solver's `compliance == 0` case of the original constraint —
// the device result is checked against a byte-for-byte host mirror of
// XpbdDistanceProjection, exactly as pgs_demo checks DistanceProjection.

#include <algorithm>
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
    constexpr std::size_t N          = 64;          // bodies in the chain
    constexpr Scalar      SPACING    = Scalar(0.5); // rest length between neighbours
    constexpr std::size_t ITERATIONS = 20;          // Gauss-Seidel sweeps per frame
    constexpr std::size_t FRAMES     = 200;
    constexpr Scalar      GY         = Scalar(-9.8); // gravity
    constexpr Scalar      DT         = Scalar(0.016);

    // Host mirror of XpbdDistanceProjection, byte-for-byte the same arithmetic.
    void project_host(const XpbdDistanceConstraint& c, std::vector<RigidBody>& bodies,
                      Scalar& lambda, Scalar h)
    {
        RigidBody& body_a = bodies[c.a];
        RigidBody& body_b = bodies[c.b];

        const Vector3 anchor_a = rotate(body_a.orientation, c.local_anchor_a);
        const Vector3 anchor_b = rotate(body_b.orientation, c.local_anchor_b);
        const Vector3 p1 = body_a.position + anchor_a;
        const Vector3 p2 = body_b.position + anchor_b;
        const Vector3 d = p2 - p1;
        const Scalar len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        if (len <= Scalar(1e-8))
            return;
        const Vector3 n = d * (Scalar(1) / len);
        const Scalar error = len - c.rest_length;

        const Vector3 rxn_a = rotate(conjugate(body_a.orientation), cross(anchor_a, n));
        const Vector3 rxn_b = rotate(conjugate(body_b.orientation), cross(anchor_b, n));

        const Vector3 iixn_a{body_a.inv_inertia.x * rxn_a.x, body_a.inv_inertia.y * rxn_a.y,
                          body_a.inv_inertia.z * rxn_a.z};
        const Vector3 iixn_b{body_b.inv_inertia.x * rxn_b.x, body_b.inv_inertia.y * rxn_b.y,
                          body_b.inv_inertia.z * rxn_b.z};

        const Scalar w = body_a.inv_mass + body_b.inv_mass + dot(rxn_a, iixn_a) +
                         dot(rxn_b, iixn_b);
        if (w <= Scalar(0))
            return;

        const Scalar alpha_tilde = h > Scalar(0) ? c.compliance / (h * h) : Scalar(0);
        const Scalar delta_lambda = (-error - alpha_tilde * lambda) / (w + alpha_tilde);
        lambda += delta_lambda;

        const Vector3 impulse = n * delta_lambda;
        body_a.position = body_a.position - impulse * body_a.inv_mass;
        body_b.position = body_b.position + impulse * body_b.inv_mass;

        body_a.orientation =
            apply_angular_correction(body_a.orientation, iixn_a * (-delta_lambda));
        body_b.orientation =
            apply_angular_correction(body_b.orientation, iixn_b * delta_lambda);
    }

    Scalar abs_max(Scalar acc, Scalar value) { return std::fabs(value) > acc ? std::fabs(value) : acc; }
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();
    auto bodies = runtime.buffer<RigidBody>(N);

    std::vector<RigidBody> ref_bodies(N);
    std::vector<XpbdDistanceConstraint> constraints;

    // A horizontal chain; the first body is pinned (inverse mass zero).
    for (std::uint32_t i = 0; i < N; ++i)
    {
        RigidBody body;
        body.position = Vector3{Scalar(i) * SPACING, Scalar(0), Scalar(0)};
        body.inv_mass = (i == 0) ? Scalar(0) : Scalar(1);
        body.inv_inertia = Vector3{0, 0, 0};
        bodies[i] = body;
        ref_bodies[i] = body;
    }
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        constraints.push_back(
            XpbdDistanceConstraint{i, i + 1, Vector3{0, 0, 0}, Vector3{0, 0, 0}, SPACING, Scalar(0)});

    std::vector<Scalar> ref_lambda(constraints.size(), Scalar(0));

    XpbdSolver<XpbdDistanceConstraint> solver(
        runtime, bodies, constraints, N, ITERATIONS, DT, XpbdDistanceProjection{});

    double solve_ms = 0.0;
    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        // Position-based gravity on the free bodies, applied to both worlds.
        for (std::uint32_t i = 0; i < N; ++i)
        {
            if (bodies[i].inv_mass > Scalar(0))
            {
                bodies[i].position.y += GY * DT * DT;
                ref_bodies[i].position.y += GY * DT * DT;
            }
        }

        // Device solve.
        const SushiRuntime::RunReport report = solver.solve();
        solve_ms += report.total_duration_ms;

        // Reference solve: the same colours, in the same order, sequentially.
        std::fill(ref_lambda.begin(), ref_lambda.end(), Scalar(0));
        for (std::size_t iteration = 0; iteration < ITERATIONS; ++iteration)
            for (const std::vector<std::uint32_t>& batch : solver.colors())
                for (std::uint32_t k : batch)
                    project_host(constraints[k], ref_bodies, ref_lambda[k], DT);
    }

    // Compare the device world to the reference, and report constraint satisfaction.
    Scalar max_error = Scalar(0);
    for (std::uint32_t i = 0; i < N; ++i)
    {
        const Vector3 p = bodies[i].position;
        max_error = abs_max(max_error, p.x - ref_bodies[i].position.x);
        max_error = abs_max(max_error, p.y - ref_bodies[i].position.y);
        max_error = abs_max(max_error, p.z - ref_bodies[i].position.z);
    }

    Scalar max_residual = Scalar(0);
    for (const XpbdDistanceConstraint& c : constraints)
    {
        const Vector3 d = bodies[c.a].position - bodies[c.b].position;
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
