/**************************************************************************/
/* test_physics_bridge.cpp                                               */
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

// Integration_PhysicsBridge: the ECS-facing half of the physics seam
// (sim/physics_bridge.hpp). An entity carrying Transform/Orientation/PhysicsBody is
// registered with a PhysicsWorld from its starting pose (initial_rigid_body), the
// world is stepped under gravity, and sync_transforms_from_physics must copy the
// solved pose back into the entity's Transform/Orientation. An entity with an
// unregistered PhysicsBody (INVALID) must be left untouched.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/components.hpp>
#include <SushiEngine/sim/physics_bridge.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar      SUBSTEP_DT         = Scalar(0.005);
    constexpr std::size_t SUBSTEPS_PER_FRAME = 4;
    constexpr std::size_t FRAMES             = 100;
}

TEST(Integration_PhysicsBridge, RegisteredEntitySyncsAndUnregisteredDoesNot)
{
    World world(Harness::shared_runtime(), 8);
    world.reserve<Simulation::Transform, Simulation::Orientation, Simulation::PhysicsBody>(8);

    const Entity falling = world.spawn(
        Simulation::Transform{Vector3{0, 10, 0}}, Simulation::Orientation{}, Simulation::PhysicsBody{});
    const Entity untouched = world.spawn(
        Simulation::Transform{Vector3{3, 3, 3}}, Simulation::Orientation{}, Simulation::PhysicsBody{});

    PhysicsWorld<XpbdDistanceConstraint> physics(Harness::shared_runtime());
    const BodyId falling_id =
        physics.add_body(Simulation::initial_rigid_body(world, falling, Scalar(1)));
    // No constraints: a single free-falling body under gravity is enough to prove
    // the bridge, without pulling constraint mechanics into this test.
    physics.finalize(1, SUBSTEP_DT, XpbdDistanceProjection{});

    world.get<Simulation::PhysicsBody>(falling).body_id = falling_id;
    // `untouched` keeps PhysicsBody::INVALID: never registered with the physics world.

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        physics.step(Vector3{0, Scalar(-9.8), 0}, SUBSTEPS_PER_FRAME);
        Simulation::sync_transforms_from_physics(world, physics);
    }

    EXPECT_LT(world.get<Simulation::Transform>(falling).position.y, Scalar(10));
    EXPECT_TRUE(Harness::approx_equal(world.get<Simulation::Transform>(falling).position,
                                       physics.body(falling_id).position, Scalar(1e-6)));

    EXPECT_TRUE(Harness::approx_equal(world.get<Simulation::Transform>(untouched).position,
                                       Vector3{3, 3, 3}, Scalar(0)));
}
