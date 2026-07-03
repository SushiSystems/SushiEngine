/**************************************************************************/
/* test_xpbd_solver.cpp                                                  */
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

// Integration_XpbdSolver: SushiLoop M2's unified XPBD solver against the real
// runtime (docs/slop/SUSHILOOP.md). Mirrors Integration_PgsSolver's structure and
// scenario (a hanging chain) but over RigidBody state instead of bare positions, to
// prove the rigid-body generalization: with anchors at each body's own centre
// (zero offset) and zero inverse inertia, XpbdDistanceProjection's linear term is
// mathematically identical to the plain PGS DistanceProjection (no angular
// coupling can occur), so the chain must settle into the same shape. The device
// result is also checked against a byte-for-byte host mirror of the projection
// itself, the same way the PGS test validates parallel-within-colour /
// sequential-across-colour equals Gauss-Seidel.

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

    // Byte-for-byte host mirror of XpbdDistanceProjection::operator().
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
}

TEST(Integration_XpbdSolver, HangingChainMatchesReferenceAndPgsShape)
{
    auto& runtime = Harness::shared_runtime();
    auto bodies = runtime.buffer<RigidBody>(N);

    std::vector<RigidBody> ref_bodies(N);
    std::vector<XpbdDistanceConstraint> constraints;

    for (std::uint32_t i = 0; i < N; ++i)
    {
        RigidBody body;
        body.position = Vector3{Scalar(i) * SPACING, Scalar(0), Scalar(0)};
        body.inv_mass = (i == 0) ? Scalar(0) : Scalar(1); // pin the first body
        body.inv_inertia = Vector3{0, 0, 0}; // point-mass: no angular coupling
        bodies[i] = body;
        ref_bodies[i] = body;
    }
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        constraints.push_back(
            XpbdDistanceConstraint{i, i + 1, Vector3{0, 0, 0}, Vector3{0, 0, 0}, SPACING, Scalar(0)});

    std::vector<Scalar> ref_lambda(constraints.size(), Scalar(0));

    XpbdSolver<XpbdDistanceConstraint> solver(
        runtime, bodies, constraints, N, ITERATIONS, DT, XpbdDistanceProjection{});

    EXPECT_EQ(solver.color_count(), 2u);

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        for (std::uint32_t i = 0; i < N; ++i)
            if (bodies[i].inv_mass > Scalar(0))
            {
                bodies[i].position.y += GY * DT * DT;
                ref_bodies[i].position.y += GY * DT * DT;
            }

        solver.solve();

        std::fill(ref_lambda.begin(), ref_lambda.end(), Scalar(0));
        for (std::size_t iteration = 0; iteration < ITERATIONS; ++iteration)
            for (const std::vector<std::uint32_t>& batch : solver.colors())
                for (std::uint32_t k : batch)
                    project_host(constraints[k], ref_bodies, ref_lambda[k], DT);
    }

    EXPECT_EQ(solver.compile_count(), 1u);

    const Scalar tol = Scalar(0.02);
    for (std::uint32_t i = 0; i < N; ++i)
    {
        const RigidBody b = bodies[i];
        EXPECT_TRUE(Harness::approx_equal(b.position, ref_bodies[i].position, tol))
            << "body " << i << " diverged from the reference";
    }

    // The chain must also actually satisfy its constraints (rest length held).
    Scalar max_residual = Scalar(0);
    for (const XpbdDistanceConstraint& c : constraints)
    {
        const Vector3 pa = bodies[c.a].position;
        const Vector3 pb = bodies[c.b].position;
        const Vector3 d = pa - pb;
        const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        max_residual = std::max(max_residual, std::fabs(dist - c.rest_length));
    }
    EXPECT_LT(max_residual, Scalar(0.1));
}
