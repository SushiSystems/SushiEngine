/**************************************************************************/
/* test_app.cpp                                                          */
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

// Integration_App: the SushiLoop authoring surface (Loop::App) as a checked test —
// the same claims examples/first_game.cpp prints, asserted here so `se test` guards
// them. Two properties: a fixed-step run driven by pure-ECS systems matches an
// independent closed-form reference (determinism through the runtime), and the same
// game, made networked by one connect(), converges a mispredicting client to an
// authoritative server via rollback reconciliation.

#include <cstddef>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    constexpr Scalar FIXED_DT = Scalar(1.0 / 60.0);
    constexpr Scalar MOVE_SPEED = Scalar(4);
    constexpr std::size_t CAPACITY = 32;
    constexpr std::size_t TICKS = 40;
    constexpr Scalar TOLERANCE = Scalar(1e-5);

    struct MoveCommand
    {
        Scalar x = 0;
        Scalar z = 0;

        bool operator==(const MoveCommand& other) const noexcept
        {
            return x == other.x && z == other.z;
        }
    };

    struct PlayerInput { Vector3 move; };
    struct Velocity { Vector3 value; };
    struct Position { Vector3 value; };

    MoveCommand authoritative_command(Loop::TickId tick) noexcept
    {
        return MoveCommand{Scalar(tick % 3) - Scalar(1), Scalar((tick + 1) % 4) - Scalar(2)};
    }

    bool is_mispredicted(Loop::TickId tick) noexcept
    {
        return tick == 7 || tick == 8 || tick == 20;
    }

    MoveCommand predicted_command(Loop::TickId tick) noexcept
    {
        if (is_mispredicted(tick))
            return MoveCommand{Scalar(9), Scalar(-9)};
        return authoritative_command(tick);
    }

    // Wires the identical game onto any App: two disjoint-write systems and the
    // command-to-input binding. Shared so the single-player and networked cases are
    // provably the same game, differing only by the connected transport.
    void build_game(Loop::App<MoveCommand>& app, Entity& player)
    {
        app.world().reserve<Position, Velocity, PlayerInput>(CAPACITY);

        const Scalar speed = MOVE_SPEED;
        app.system<Read<PlayerInput>, Write<Velocity>>("control").each(
            [speed](std::size_t i, const PlayerInput* input, Velocity* velocity)
            {
                velocity[i].value = input[i].move * speed;
            });

        const Scalar dt = app.fixed_dt();
        app.system<Read<Velocity>, Write<Position>>("integrate").each(
            [dt](std::size_t i, const Velocity* velocity, Position* position)
            {
                position[i].value = position[i].value + velocity[i].value * dt;
            });

        Entity* player_slot = &player;
        app.on_start([player_slot](World& world)
        {
            *player_slot = world.spawn(Position{}, Velocity{}, PlayerInput{});
        });
        app.on_command([player_slot](World& world, Loop::TickId, const MoveCommand& command)
        {
            if (world.alive(*player_slot))
                world.get<PlayerInput>(*player_slot).move = Vector3{command.x, 0, command.z};
        });
    }

    Vector3 closed_form_reference()
    {
        Vector3 position{};
        for (Loop::TickId tick = 0; tick < TICKS; ++tick)
        {
            const MoveCommand command = authoritative_command(tick);
            const Vector3 velocity = Vector3{command.x, 0, command.z} * MOVE_SPEED;
            position = position + velocity * FIXED_DT;
        }
        return position;
    }
}

TEST(Integration_App, FixedStepRunMatchesClosedFormReference)
{
    Loop::AppConfig config;
    config.fixed_dt_seconds = FIXED_DT;
    config.chunk_capacity = CAPACITY;

    Loop::App<MoveCommand> game(Harness::shared_runtime(), config);
    Entity player{};
    build_game(game, player);
    game.sample_command([](Loop::TickId tick) { return authoritative_command(tick); });
    game.start();
    game.run_for(TICKS);

    const Vector3 actual = game.world().get<Position>(player).value;
    EXPECT_TRUE(Harness::approx_equal(actual, closed_form_reference(), TOLERANCE))
        << "actual=(" << double(actual.x) << "," << double(actual.y) << ","
        << double(actual.z) << ")";
    // The archetype is reserved before the run, so the schedule compiles exactly once.
    EXPECT_EQ(game.compile_count(), std::size_t(1));
}

TEST(Integration_App, ClientReconcilesToAuthoritativeServer)
{
    Loop::AppConfig config;
    config.fixed_dt_seconds = FIXED_DT;
    config.chunk_capacity = CAPACITY;

    // The server: driven straight through by its own authoritative command stream.
    Loop::App<MoveCommand> server(Harness::shared_runtime(), config);
    Entity server_player{};
    build_game(server, server_player);
    server.sample_command([](Loop::TickId tick) { return authoritative_command(tick); });
    server.start();
    server.run_for(TICKS);
    const Vector3 server_position = server.world().get<Position>(server_player).value;

    // The client: mispredicts some ticks, reconciles via rollback against the server.
    Loop::AppConfig client_config = config;
    client_config.rollback_capacity = TICKS;
    Loop::App<MoveCommand> client(Harness::shared_runtime(), client_config);
    Entity client_player{};
    build_game(client, client_player);
    client.sample_command([](Loop::TickId tick) { return predicted_command(tick); });

    Loop::Net::LoopbackTransport<MoveCommand> transport(
        [](Loop::TickId tick, MoveCommand) { return authoritative_command(tick); });
    client.connect(&transport);

    client.start();
    client.run_for(TICKS);
    const Vector3 client_position = client.world().get<Position>(client_player).value;

    EXPECT_TRUE(Harness::approx_equal(client_position, server_position, TOLERANCE))
        << "client=(" << double(client_position.x) << "," << double(client_position.y) << ","
        << double(client_position.z) << ") server=(" << double(server_position.x) << ","
        << double(server_position.y) << "," << double(server_position.z) << ")";
}
