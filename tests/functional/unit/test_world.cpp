/**************************************************************************/
/* test_world.cpp                                                         */
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

// Unit_World: entity directory mechanics — spawn, get, destroy, generation
// invalidation, swap-remove record repointing, reserve, structure versioning,
// and archetype queries. No device kernels run here; this is the host bookkeeping.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Position { Vector3 v; };
    struct Velocity { Vector3 v; };
    struct Health   { Scalar value; };
}

TEST(Unit_World, SpawnReturnsLiveHandleWithStoredComponents)
{
    World world(Harness::shared_runtime(), 256);

    const Entity e = world.spawn(Position{Vector3{1, 2, 3}}, Velocity{Vector3{4, 5, 6}});

    EXPECT_TRUE(world.alive(e));
    EXPECT_TRUE(Harness::approx_equal(world.get<Position>(e).v, Vector3{1, 2, 3}, Scalar(0)));
    EXPECT_TRUE(Harness::approx_equal(world.get<Velocity>(e).v, Vector3{4, 5, 6}, Scalar(0)));
}

TEST(Unit_World, GetReturnsMutableReference)
{
    World world(Harness::shared_runtime(), 256);
    const Entity e = world.spawn(Position{Vector3{0, 0, 0}});

    world.get<Position>(e).v = Vector3{7, 8, 9};

    EXPECT_TRUE(Harness::approx_equal(world.get<Position>(e).v, Vector3{7, 8, 9}, Scalar(0)));
}

TEST(Unit_World, DestroyClearsLivenessAndStaleHandleStaysDead)
{
    World world(Harness::shared_runtime(), 256);
    const Entity e = world.spawn(Position{});

    world.destroy(e);
    EXPECT_FALSE(world.alive(e));

    // A reused slot must not resurrect the old generation's handle.
    const Entity reused = world.spawn(Position{});
    EXPECT_TRUE(world.alive(reused));
    EXPECT_FALSE(world.alive(e));
}

TEST(Unit_World, SwapRemoveRepointsTheMovedRow)
{
    World world(Harness::shared_runtime(), 256);
    const Entity a = world.spawn(Health{Scalar(10)});
    const Entity b = world.spawn(Health{Scalar(20)});
    const Entity c = world.spawn(Health{Scalar(30)});

    // Removing the first row swap-moves the last row (c) into slot 0; c's handle
    // must still resolve to its own data through the repointed directory record.
    world.destroy(a);

    EXPECT_FALSE(world.alive(a));
    EXPECT_TRUE(world.alive(b));
    EXPECT_TRUE(world.alive(c));
    EXPECT_TRUE(Harness::approx_equal(world.get<Health>(b).value, Scalar(20), Scalar(0)));
    EXPECT_TRUE(Harness::approx_equal(world.get<Health>(c).value, Scalar(30), Scalar(0)));
}

TEST(Unit_World, ReserveAndNewChunkBumpStructureVersion)
{
    World world(Harness::shared_runtime(), 4);

    const std::uint64_t v0 = world.structure_version();
    world.reserve<Position>(4);
    EXPECT_GT(world.structure_version(), v0);

    // Filling the reserved chunk and overflowing it allocates another chunk, which
    // is the only spawn path that ticks the version again.
    const std::uint64_t v1 = world.structure_version();
    for (int i = 0; i < 4; ++i)
        world.spawn(Position{});
    EXPECT_EQ(world.structure_version(), v1); // still room: no new chunk

    world.spawn(Position{}); // overflow -> new chunk
    EXPECT_GT(world.structure_version(), v1);
}

TEST(Unit_World, QueryMatchesArchetypesContainingTheRequestedComponents)
{
    World world(Harness::shared_runtime(), 256);
    world.spawn(Position{}, Velocity{}); // archetype {Position, Velocity}
    world.spawn(Position{});             // archetype {Position}

    // A query for Position alone matches both archetypes; adding Velocity narrows
    // it to the one that carries it.
    EXPECT_EQ(world.query(make_signature<Position>()).size(), 2u);
    EXPECT_EQ(world.query(make_signature<Position, Velocity>()).size(), 1u);
    EXPECT_EQ(world.query(make_signature<Health>()).size(), 0u);
}
