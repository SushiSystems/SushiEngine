/**************************************************************************/
/* test_regression.cpp                                                   */
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

// Regression_Ecs: behaviours that have a single correct answer and must never
// drift. Each test pins one invariant that a future change could silently break.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Value { std::uint32_t v; };
}

// Destroying many entities in arbitrary order must keep every surviving handle
// resolving to its own data — the swap-remove directory repointing must hold up
// across a full churn, not just a single removal.
TEST(Regression_Ecs, SwapRemoveSurvivesInterleavedChurn)
{
    World world(Harness::shared_runtime(), 1024);

    std::vector<Entity> entities;
    for (std::uint32_t i = 0; i < 200; ++i)
        entities.push_back(world.spawn(Value{i}));

    // Destroy every third entity; survivors must still read back their own value.
    for (std::size_t i = 0; i < entities.size(); i += 3)
        world.destroy(entities[i]);

    for (std::size_t i = 0; i < entities.size(); ++i)
    {
        if (i % 3 == 0)
        {
            EXPECT_FALSE(world.alive(entities[i]));
        }
        else
        {
            ASSERT_TRUE(world.alive(entities[i]));
            EXPECT_EQ(world.get<Value>(entities[i]).v, static_cast<std::uint32_t>(i));
        }
    }
}

// A schedule whose systems match no live entities must run cleanly without
// throwing, so an idle frame is safe even before anything is spawned.
TEST(Regression_Ecs, EmptyScheduleRunIsANoOp)
{
    World world(Harness::shared_runtime(), 64);
    Schedule schedule(Harness::shared_runtime());

    schedule.each<Write<Value>>("touch", [](std::size_t, Value*) {});

    EXPECT_NO_THROW(schedule.run(world)); // no entities of this archetype exist
    EXPECT_EQ(schedule.system_count(), 1u);
}
