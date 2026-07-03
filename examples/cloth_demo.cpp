/**************************************************************************/
/* cloth_demo.cpp                                                        */
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

// SushiLoop M5 worked example: xpbd_demo.cpp's hanging chain, generalized to a
// pinned-top cloth grid (physics/cloth.hpp) — structural and shear
// XpbdDistanceConstraints over a PhysicsWorld<XpbdDistanceConstraint>, no new
// solver or constraint type. The device result is checked against a
// byte-for-byte host mirror of XpbdDistanceProjection, exactly as xpbd_demo does.

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
    constexpr std::size_t ROWS       = 8;
    constexpr std::size_t COLS       = 8;
    constexpr Scalar      SPACING    = Scalar(0.3);
    constexpr std::size_t ITERATIONS = 20;
    constexpr std::size_t FRAMES     = 200;
    constexpr Scalar      GY         = Scalar(-9.8);
    constexpr Scalar      DT         = Scalar(0.016);

    // Host mirror of XpbdDistanceProjection, byte-for-byte the same arithmetic as
    // xpbd_demo.cpp's, since PhysicsWorld does not expose the raw solve internals.
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
    PhysicsWorld<XpbdDistanceConstraint> world(runtime);

    const ClothGrid grid =
        build_cloth_grid(world, ROWS, COLS, SPACING, Vector3{0, 0, 0}, Scalar(0));

    world.finalize(ITERATIONS, DT, XpbdDistanceProjection{});

    // A separate host reference mirroring the grid's own bodies/constraints,
    // solved with the identical colours in the identical order.
    std::vector<RigidBody> ref_bodies(world.body_count());
    for (std::size_t i = 0; i < world.body_count(); ++i)
        ref_bodies[i] = world.body(BodyId(i));

    // PhysicsWorld does not expose its constraint list, so the reference solve
    // rebuilds the identical topology by walking the same grid helper produced.
    std::vector<XpbdDistanceConstraint> constraints;
    const Scalar diagonal = SPACING * Scalar(std::sqrt(2.0));
    for (std::size_t row = 0; row < ROWS; ++row)
        for (std::size_t col = 0; col < COLS; ++col)
        {
            if (col + 1 < COLS)
                constraints.push_back(XpbdDistanceConstraint{
                    grid.at(row, col), grid.at(row, col + 1), Vector3{0, 0, 0}, Vector3{0, 0, 0},
                    SPACING, Scalar(0)});
            if (row + 1 < ROWS)
                constraints.push_back(XpbdDistanceConstraint{
                    grid.at(row, col), grid.at(row + 1, col), Vector3{0, 0, 0}, Vector3{0, 0, 0},
                    SPACING, Scalar(0)});
            if (row + 1 < ROWS && col + 1 < COLS)
            {
                constraints.push_back(XpbdDistanceConstraint{
                    grid.at(row, col), grid.at(row + 1, col + 1), Vector3{0, 0, 0}, Vector3{0, 0, 0},
                    diagonal, Scalar(0)});
                constraints.push_back(XpbdDistanceConstraint{
                    grid.at(row, col + 1), grid.at(row + 1, col), Vector3{0, 0, 0}, Vector3{0, 0, 0},
                    diagonal, Scalar(0)});
            }
        }

    std::vector<Scalar> ref_lambda(constraints.size(), Scalar(0));

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        world.step(Vector3{0, GY, 0}, 1);

        for (RigidBody& body : ref_bodies)
            predict(body, Vector3{0, GY, 0}, DT);

        std::fill(ref_lambda.begin(), ref_lambda.end(), Scalar(0));
        for (std::size_t iteration = 0; iteration < ITERATIONS; ++iteration)
            for (std::size_t k = 0; k < constraints.size(); ++k)
                project_host(constraints[k], ref_bodies, ref_lambda[k], DT);

        for (RigidBody& body : ref_bodies)
            update_velocity(body, DT);
    }

    Scalar max_error = Scalar(0);
    for (std::size_t i = 0; i < world.body_count(); ++i)
    {
        const Vector3 p = world.body(BodyId(i)).position;
        max_error = abs_max(max_error, p.x - ref_bodies[i].position.x);
        max_error = abs_max(max_error, p.y - ref_bodies[i].position.y);
        max_error = abs_max(max_error, p.z - ref_bodies[i].position.z);
    }

    Scalar max_residual = Scalar(0);
    for (const XpbdDistanceConstraint& c : constraints)
    {
        const Vector3 d = world.body(c.a).position - world.body(c.b).position;
        const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
        max_residual = abs_max(max_residual, dist - c.rest_length);
    }

    const Scalar tol = Scalar(0.05);
    const std::size_t compiles = world.compile_count();

    std::printf("cloth: rows=%zu cols=%zu bodies=%zu constraints=%zu iterations=%zu\n",
                ROWS, COLS, world.body_count(), constraints.size(), ITERATIONS);
    std::printf("compile_count=%zu (expected 1)  max_error_vs_reference=%.6f (tol %.3f)\n",
                compiles, double(max_error), double(tol));
    std::printf("max_constraint_residual=%.6f over %zu frames\n", double(max_residual), FRAMES);

    // Row 0 must stay pinned exactly where it started.
    bool pinned_ok = true;
    for (std::size_t col = 0; col < COLS; ++col)
    {
        const Vector3 p = world.body(grid.at(0, col)).position;
        const Vector3 expected{Scalar(col) * SPACING, Scalar(0), Scalar(0)};
        if (std::fabs(p.x - expected.x) > Scalar(1e-5) || std::fabs(p.y - expected.y) > Scalar(1e-5) ||
            std::fabs(p.z - expected.z) > Scalar(1e-5))
            pinned_ok = false;
    }

    const bool ok = (max_error < tol) && (compiles == 1) && pinned_ok;
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
