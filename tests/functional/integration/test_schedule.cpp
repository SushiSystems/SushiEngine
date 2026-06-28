/**************************************************************************/
/* test_schedule.cpp                                                     */
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

// Integration_Schedule: the ECS thesis end to end against the real runtime. A
// read-after-write chain (apply_forces writes velocity, integrate reads it) plus
// a disjoint system run as device kernels; the result is checked against an
// independent scalar reference, and the graph is compiled once and replayed.

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    struct Position { Vec3 v; };
    struct Velocity { Vec3 v; };
    struct Mass     { Scalar value; };
    struct Lifetime { Scalar value; };

    constexpr Scalar DT      = Scalar(0.01);
    constexpr Scalar FORCE_Y = Scalar(-9.8);
    constexpr std::size_t COUNT  = 128;
    constexpr std::size_t FRAMES = 50;

    struct Reference
    {
        Vec3 position;
        Vec3 velocity;
        Scalar lifetime;
        Scalar mass;
    };
}

TEST(Integration_Schedule, MatchesScalarReferenceAndCompilesOnce)
{
    World world(Harness::shared_runtime(), 256);
    Schedule schedule(Harness::shared_runtime());
    world.reserve<Position, Velocity, Mass, Lifetime>(COUNT);

    schedule.each<Write<Velocity>, Read<Mass>>("apply_forces",
        [](std::size_t i, Velocity* vel, const Mass* mass)
        {
            vel[i].v.y += (FORCE_Y / mass[i].value) * DT;
        });
    schedule.each<Write<Position>, Read<Velocity>>("integrate",
        [](std::size_t i, Position* pos, const Velocity* vel)
        {
            pos[i].v = pos[i].v + vel[i].v * DT;
        });
    schedule.each<Write<Lifetime>>("decay_lifetime",
        [](std::size_t i, Lifetime* life)
        {
            life[i].value -= DT;
        });

    std::vector<Entity> entities;
    std::vector<Reference> reference;
    for (std::size_t i = 0; i < COUNT; ++i)
    {
        const Scalar m = Scalar(1) + Scalar(i % 4);
        entities.push_back(world.spawn(Position{}, Velocity{}, Mass{m}, Lifetime{Scalar(100)}));
        reference.push_back(Reference{Vec3{}, Vec3{}, Scalar(100), m});
    }

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        schedule.run(world);
        for (Reference& r : reference)
        {
            r.velocity.y += (FORCE_Y / r.mass) * DT;
            r.position = r.position + r.velocity * DT;
            r.lifetime -= DT;
        }
    }

    EXPECT_EQ(schedule.system_count(), 3u);
    EXPECT_EQ(schedule.compile_count(), 1u);

    const Scalar tol = Scalar(0.05);
    for (std::size_t i = 0; i < COUNT; ++i)
    {
        ASSERT_TRUE(world.alive(entities[i]));
        EXPECT_TRUE(Harness::approx_equal(world.get<Position>(entities[i]).v, reference[i].position, tol));
        EXPECT_TRUE(Harness::approx_equal(world.get<Velocity>(entities[i]).v, reference[i].velocity, tol));
        EXPECT_TRUE(Harness::approx_equal(world.get<Lifetime>(entities[i]).value, reference[i].lifetime, tol));
    }
}
