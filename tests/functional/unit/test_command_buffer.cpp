/**************************************************************************/
/* test_command_buffer.cpp                                               */
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

// Unit_CommandBuffer: deferred structural changes. Spawns and destroys recorded
// during a frame must not touch the world until apply() at the barrier, and must
// then take effect in the recorded order. Double/stale destroys are guarded.

#include <cstddef>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Tag { std::uint32_t value; };

    // Live entity count of the {Tag} archetype: sum of its chunks' live rows.
    std::size_t tag_count(World& world)
    {
        std::size_t total = 0;
        for (Archetype* a : world.query(make_signature<Tag>()))
            for (const std::unique_ptr<Chunk>& c : a->chunks())
                total += c->count();
        return total;
    }
}

TEST(Unit_CommandBuffer, RecordedSpawnsAreDeferredUntilApply)
{
    World world(Harness::shared_runtime(), 256);
    CommandBuffer commands;

    commands.spawn(Tag{1});
    commands.spawn(Tag{2});
    commands.spawn(Tag{3});

    EXPECT_FALSE(commands.empty());
    EXPECT_EQ(commands.size(), 3u);
    EXPECT_EQ(tag_count(world), 0u); // nothing applied yet

    commands.apply(world);

    EXPECT_EQ(tag_count(world), 3u);
    EXPECT_TRUE(commands.empty()); // apply clears the buffer
}

TEST(Unit_CommandBuffer, RecordedDestroyTakesEffectAtApply)
{
    World world(Harness::shared_runtime(), 256);
    const Entity e = world.spawn(Tag{42});
    CommandBuffer commands;

    commands.destroy(e);
    EXPECT_TRUE(world.alive(e)); // still alive before the barrier

    commands.apply(world);
    EXPECT_FALSE(world.alive(e));
}

TEST(Unit_CommandBuffer, DoubleDestroyIsHarmless)
{
    World world(Harness::shared_runtime(), 256);
    const Entity e = world.spawn(Tag{7});
    CommandBuffer commands;

    commands.destroy(e);
    commands.destroy(e); // enqueued twice; the guard at apply makes the second a no-op

    commands.apply(world);
    EXPECT_FALSE(world.alive(e));
    EXPECT_EQ(tag_count(world), 0u);
}
