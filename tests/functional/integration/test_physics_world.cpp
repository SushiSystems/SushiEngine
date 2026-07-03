/**************************************************************************/
/* test_physics_world.cpp                                                */
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

// Integration_PhysicsWorld: the sub-stepped XPBD loop (predict / solve / derive
// velocity) that `PhysicsWorld` wraps around `XpbdSolver`, against the real
// runtime. Two bodies, the first pinned, joined by a rigid link: under gravity the
// free body must fall until the link is taut and then hold at rest length —
// proving `step()`'s own gravity integration and velocity derivation, not just the
// one-shot constraint projection `Integration_XpbdSolver` already covers.

#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar      REST_LENGTH = Scalar(1.0);
    constexpr Scalar      GRAVITY_Y   = Scalar(-9.8);
    constexpr Scalar      SUBSTEP_DT  = Scalar(0.005);
    constexpr std::size_t SUBSTEPS_PER_FRAME = 4;
    constexpr std::size_t FRAMES      = 400;
    constexpr std::size_t ITERATIONS  = 8;
}

TEST(Integration_PhysicsWorld, PinnedPairSettlesAtRestLength)
{
    PhysicsWorld<XpbdDistanceConstraint> world(Harness::shared_runtime());

    RigidBody anchor;
    anchor.position = Vec3{0, 0, 0};
    anchor.inv_mass = Scalar(0); // pinned
    const BodyId anchor_id = world.add_body(anchor);

    RigidBody weight;
    weight.position = Vec3{0, -REST_LENGTH, 0};
    weight.inv_mass = Scalar(1);
    const BodyId weight_id = world.add_body(weight);

    world.add_constraint(XpbdDistanceConstraint{
        anchor_id, weight_id, Vec3{0, 0, 0}, Vec3{0, 0, 0}, REST_LENGTH, Scalar(0)});

    world.finalize(ITERATIONS, SUBSTEP_DT, XpbdDistanceProjection{});
    ASSERT_EQ(world.color_count(), 1u);

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
        world.step(Vec3{0, GRAVITY_Y, 0}, SUBSTEPS_PER_FRAME);

    EXPECT_EQ(world.compile_count(), 1u);

    // The anchor never moves (pinned); the weight settles directly below it.
    EXPECT_TRUE(Harness::approx_equal(world.body(anchor_id).position, Vec3{0, 0, 0}, Scalar(1e-6)));

    const Vec3 d = world.body(weight_id).position - world.body(anchor_id).position;
    const Scalar dist = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    EXPECT_NEAR(double(dist), double(REST_LENGTH), 0.02);
    EXPECT_NEAR(double(d.x), 0.0, 0.02);
    EXPECT_LT(d.y, Scalar(0)); // hangs below the anchor, not above or beside it
}
