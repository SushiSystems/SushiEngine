/**************************************************************************/
/* test_rollback.cpp                                                     */
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

// Integration_Rollback: SushiLoop M3's key invariant (docs/slop/SUSHILOOP.md) — a
// rollback-and-replay must produce exactly the same result as an uninterrupted
// simulation. One world runs straight through every tick as the baseline. A second
// world runs the same recorded input stream and the same per-entity seeded RngState,
// but partway through is rolled back to an earlier tick (via RollbackBuffer) and
// replayed forward from there. If the byte snapshot were lossy, or if any piece of
// state the step touches (Position *and* the per-entity RngState both live in the
// same chunk) were not captured, the replayed world would diverge from the baseline
// after the rollback point; this proves it does not.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Position { Vec3 v; };
    struct Random   { loop::RngState state; };

    constexpr Scalar      FIXED_DT               = Scalar(0.02);
    constexpr std::size_t ENTITIES               = 8;
    constexpr std::size_t CHUNK_CAPACITY         = 32;
    constexpr std::size_t TOTAL_TICKS            = 40;
    constexpr std::size_t ROLLBACK_TARGET_TICK   = 15;
    constexpr std::size_t TICK_ROLLBACK_HAPPENS  = 30;

    void step(World& world, const std::vector<Entity>& entities, Scalar command)
    {
        for (Entity e : entities)
        {
            Random& rnd = world.get<Random>(e);
            const double jitter = loop::next_unit(rnd.state) - 0.5;
            Position& pos = world.get<Position>(e);
            pos.v.x += command * FIXED_DT;
            pos.v.y += Scalar(jitter) * FIXED_DT;
        }
    }

    std::vector<Entity> seed_world(World& world)
    {
        world.reserve<Position, Random>(CHUNK_CAPACITY);
        std::vector<Entity> entities;
        for (std::size_t i = 0; i < ENTITIES; ++i)
            entities.push_back(
                world.spawn(Position{}, Random{loop::seed_rng(std::uint64_t(i) + 1)}));
        return entities;
    }
}

TEST(Integration_Rollback, RollbackAndReplayMatchesUninterruptedRun)
{
    auto& runtime = Harness::shared_runtime();

    loop::InputHistory<Scalar> input;
    loop::RngState input_rng = loop::seed_rng(0xABCDEFu);
    for (loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
        input.record(tick, Scalar(loop::next_unit(input_rng)) - Scalar(0.5));

    // Baseline: straight through, no rollback.
    World baseline_world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> baseline_entities = seed_world(baseline_world);
    for (loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
        step(baseline_world, baseline_entities, *input.find(tick));

    // Rolled-back run: capture every tick up to TICK_ROLLBACK_HAPPENS, then rewind to
    // ROLLBACK_TARGET_TICK and replay the same input stream forward from there.
    World rolled_world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> rolled_entities = seed_world(rolled_world);

    loop::RollbackBuffer rollback(TOTAL_TICKS);
    for (loop::TickId tick = 0; tick < TICK_ROLLBACK_HAPPENS; ++tick)
    {
        rollback.capture(rolled_world, tick);
        step(rolled_world, rolled_entities, *input.find(tick));
    }

    ASSERT_TRUE(rollback.has(ROLLBACK_TARGET_TICK));
    ASSERT_TRUE(rollback.restore(ROLLBACK_TARGET_TICK));

    for (loop::TickId tick = ROLLBACK_TARGET_TICK; tick < TOTAL_TICKS; ++tick)
        step(rolled_world, rolled_entities, *input.find(tick));

    ASSERT_EQ(baseline_entities.size(), rolled_entities.size());
    for (std::size_t i = 0; i < baseline_entities.size(); ++i)
    {
        const Vec3 expected = baseline_world.get<Position>(baseline_entities[i]).v;
        const Vec3 actual = rolled_world.get<Position>(rolled_entities[i]).v;
        EXPECT_EQ(actual.x, expected.x) << "entity " << i;
        EXPECT_EQ(actual.y, expected.y) << "entity " << i;
        EXPECT_EQ(actual.z, expected.z) << "entity " << i;
    }
}

TEST(Integration_Rollback, RestoreOfEvictedTickFails)
{
    auto& runtime = Harness::shared_runtime();
    World world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> entities = seed_world(world);

    loop::RollbackBuffer rollback(4); // small ring: tick 0 will be evicted quickly
    for (loop::TickId tick = 0; tick < 10; ++tick)
    {
        rollback.capture(world, tick);
        step(world, entities, Scalar(0));
    }

    EXPECT_FALSE(rollback.has(0));
    EXPECT_FALSE(rollback.restore(0));
    EXPECT_TRUE(rollback.has(9));
    EXPECT_EQ(rollback.size(), rollback.capacity());
}
