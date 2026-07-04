/**************************************************************************/
/* test_soft_body.cpp                                                    */
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

// Integration_SoftBody: the volumetric lattice (physics/soft_body.hpp) over the real
// runtime and the real XPBD solver — it must register the expected particle count,
// compile its solve graph exactly once, keep its pinned layer fixed, and hold its
// structural rest lengths under gravity (a rigid lattice deforms only slightly).

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/physics_world.hpp>
#include <SushiEngine/physics/soft_body.hpp>
#include <SushiEngine/physics/xpbd_constraint.hpp>
#include <SushiEngine/physics/xpbd_solver.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;

TEST(Integration_SoftBody, PinnedLatticeHoldsShapeUnderGravity)
{
    constexpr std::size_t N = 3;
    constexpr std::size_t SUBSTEPS = 4;
    const Scalar spacing = Scalar(0.5);
    const Scalar outer_dt = Scalar(1.0 / 60.0);
    const Scalar h = outer_dt / Scalar(SUBSTEPS);
    const Vector3 origin{0, Scalar(2), 0};

    PhysicsWorld<XpbdDistanceConstraint> world(Harness::shared_runtime());
    const SoftBodyLattice lattice = build_soft_body_lattice(
        world, N, N, N, spacing, origin, Scalar(0), /*pin_bottom=*/true);
    world.finalize(16, h, XpbdDistanceProjection{});

    EXPECT_EQ(world.body_count(), N * N * N);

    for (int step = 0; step < 60; ++step)
        world.step(Vector3{0, Scalar(-9.8), 0}, SUBSTEPS);

    // The graph compiles once and replays.
    EXPECT_EQ(world.compile_count(), std::size_t(1));

    // The pinned bottom layer never moves.
    const RigidBody& pinned = world.body(lattice.at(0, 0, 0));
    EXPECT_NEAR(double(pinned.position.y), 2.0, 1e-3);

    // A rigid (compliance 0) lattice keeps its structural rest length: the top
    // particle stays about `spacing` above the one below it, not collapsed.
    const RigidBody& top = world.body(lattice.at(0, N - 1, 0));
    const RigidBody& below = world.body(lattice.at(0, N - 2, 0));
    const Scalar gap = length(top.position - below.position);
    EXPECT_NEAR(double(gap), double(spacing), 0.2);
}
