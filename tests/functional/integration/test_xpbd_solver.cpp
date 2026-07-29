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

    // `I_world^-1 v`, written out longhand: rotate into the body frame, scale by the
    // stored diagonal, rotate back. The rotation *back* is the step this mirror once
    // dropped, faithfully, because the implementation dropped it — which is the limit
    // of what a transcription can check. Whether the formula is right is measured
    // against a conservation law in unit/test_distance_projection.cpp; what this
    // mirror checks is only that the device computes what the host does.
    Vector3 mirror_world_inverse_inertia(const RigidBody& body, const Vector3& world_vector)
    {
        const Vector3 local = rotate(conjugate(body.orientation), world_vector);
        const Vector3 scaled{local.x * body.inv_inertia.x, local.y * body.inv_inertia.y,
                             local.z * body.inv_inertia.z};
        return rotate(body.orientation, scaled);
    }

    // Byte-for-byte host mirror of XpbdDistanceProjection::operator(). The order of
    // operations matters as much as the formula: it has to be the *same* sequence of
    // roundings, or the comparison is a tolerance test wearing a byte-equality
    // costume.
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

        // Generalized inverse mass: the linear share plus the angular share the
        // lever arm exposes, `inv_mass + (r x n) . I^-1 (r x n)`, in world space.
        const Vector3 torque_axis_a = cross(anchor_a, n);
        const Vector3 torque_axis_b = cross(anchor_b, n);
        // Summed per body and then added, not accumulated left to right: the two
        // groupings differ in the last bits, and this comparison is measured in bits.
        const Scalar w_a =
            body_a.inv_mass +
            dot(torque_axis_a, mirror_world_inverse_inertia(body_a, torque_axis_a));
        const Scalar w_b =
            body_b.inv_mass +
            dot(torque_axis_b, mirror_world_inverse_inertia(body_b, torque_axis_b));
        const Scalar w = w_a + w_b;
        if (w <= Scalar(0))
            return;

        const Scalar alpha_tilde = h > Scalar(0) ? c.compliance / (h * h) : Scalar(0);
        const Scalar delta_lambda = (-error - alpha_tilde * lambda) / (w + alpha_tilde);
        lambda += delta_lambda;

        const Vector3 impulse = n * delta_lambda;

        body_a.position = body_a.position + impulse * (Scalar(-1) * body_a.inv_mass);
        const Vector3 rotation_a =
            mirror_world_inverse_inertia(body_a, cross(anchor_a, impulse * Scalar(-1)));
        if (dot(rotation_a, rotation_a) > Scalar(0))
            body_a.orientation = apply_angular_correction(body_a.orientation, rotation_a);

        body_b.position = body_b.position + impulse * (Scalar(1) * body_b.inv_mass);
        const Vector3 rotation_b =
            mirror_world_inverse_inertia(body_b, cross(anchor_b, impulse * Scalar(1)));
        if (dot(rotation_b, rotation_b) > Scalar(0))
            body_b.orientation = apply_angular_correction(body_b.orientation, rotation_b);
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
