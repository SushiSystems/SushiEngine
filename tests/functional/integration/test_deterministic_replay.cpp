/**************************************************************************/
/* test_deterministic_replay.cpp                                         */
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

// Integration_DeterministicReplay: proves SushiLoop M0/M1's core claim end to end
// (docs/slop/SUSHILOOP.md) — replaying the same numbered input stream against a
// fresh world, driven by the same fixed-step clock and the same seeded per-entity
// RngState, produces bit-identical world state. This is the property rollback
// depends on: same input, same result, on the same binary.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Position { Vector3 v; };
    struct Random   { Loop::RngState state; };

    constexpr Scalar FIXED_DT       = Scalar(0.02);
    constexpr std::size_t ENTITIES  = 16;
    constexpr std::size_t TICKS     = 40;
    constexpr std::size_t CAPACITY  = 32;

    // One step of gameplay logic: nudges every entity by the tick's input command
    // plus a per-entity jitter drawn from its own seeded RngState. Pure host code —
    // no SYCL kernel — so this test isolates SushiLoop's determinism guarantee from
    // the runtime's device dispatch.
    void step(World& world, const std::vector<Entity>& entities, Scalar command)
    {
        for (Entity e : entities)
        {
            Random& rnd = world.get<Random>(e);
            const double jitter = Loop::next_unit(rnd.state) - 0.5;
            Position& pos = world.get<Position>(e);
            pos.v.x += command * FIXED_DT;
            pos.v.y += Scalar(jitter) * FIXED_DT;
        }
    }

    // Runs a full, independent simulation: fresh world, fresh RngState seeds, the
    // same recorded input stream, and returns the final Position of every entity.
    std::vector<Vector3> run(SushiRuntime::API::Runtime& runtime,
                           const Loop::InputHistory<Scalar>& input)
    {
        World world(runtime, CAPACITY);
        world.reserve<Position, Random>(CAPACITY);

        std::vector<Entity> entities;
        for (std::size_t i = 0; i < ENTITIES; ++i)
            entities.push_back(
                world.spawn(Position{}, Random{Loop::seed_rng(std::uint64_t(i) + 1)}));

        Loop::FixedTimestepClock clock(FIXED_DT);
        Loop::TickId tick = 0;
        for (std::size_t frame = 0; frame < TICKS; ++frame)
        {
            clock.accumulate(FIXED_DT); // one host frame == exactly one fixed step
            while (clock.consume_step())
            {
                const Scalar* command = input.find(tick);
                if (command == nullptr)
                {
                    // ASSERT_NE's implicit `return;` only compiles in a void-returning
                    // function; `run()` returns a result, so fail non-fatally and bail
                    // out ourselves.
                    ADD_FAILURE() << "no recorded input for tick " << tick;
                    return {};
                }
                step(world, entities, *command);
                ++tick;
            }
        }

        std::vector<Vector3> result;
        result.reserve(entities.size());
        for (Entity e : entities)
            result.push_back(world.get<Position>(e).v);
        return result;
    }
}

TEST(Integration_DeterministicReplay, SameInputStreamProducesSameWorldState)
{
    // The input stream itself comes from a seeded RNG, standing in for a captured
    // player command sequence — deterministic, numbered by tick, and shared between
    // both runs exactly as a real replay would receive it over the network.
    Loop::InputHistory<Scalar> input;
    Loop::RngState input_rng = Loop::seed_rng(0xC0FFEEu);
    for (Loop::TickId tick = 0; tick < TICKS; ++tick)
        input.record(tick, Scalar(Loop::next_unit(input_rng)) - Scalar(0.5));

    const std::vector<Vector3> first = run(Harness::shared_runtime(), input);
    const std::vector<Vector3> second = run(Harness::shared_runtime(), input);

    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        EXPECT_EQ(first[i].x, second[i].x) << "entity " << i;
        EXPECT_EQ(first[i].y, second[i].y) << "entity " << i;
        EXPECT_EQ(first[i].z, second[i].z) << "entity " << i;
    }
}
