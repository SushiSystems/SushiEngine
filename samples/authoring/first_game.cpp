/**************************************************************************/
/* first_game.cpp                                                        */
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

// The first SushiLoop game: the smallest complete program written against the
// settled Loop::App authoring API. It is deliberately a game, not a physics or
// networking unit test — it shows the shape a real game is written in — while still
// being headless and self-checking (RESULT: OK on success), the way every example
// in the tree validates itself.
//
// What it demonstrates, in order:
//   * Authoring is pure ECS. Gameplay is two systems declaring their component
//     access — "control" turns the player's input into a velocity, "integrate"
//     advances the position — never a per-object Update(). The App runs them in a
//     fixed-step deterministic loop.
//   * Determinism. A closed-form reference computes the expected final position
//     independently; the world must match it exactly.
//   * Always multiplayer-ready, one decision. The identical game is then run as a
//     client that mispredicts some inputs and is corrected by an authoritative
//     server — reached only by connect()-ing a transport, with no change to the
//     systems, the components, or the command type. The client must converge to the
//     server's authoritative state.

#include <cstddef>
#include <cstdio>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

namespace
{
    constexpr std::size_t CHUNK_CAPACITY = 64;
    constexpr std::size_t TOTAL_TICKS = 40;
    constexpr Scalar FIXED_DT = Scalar(1.0 / 60.0);
    constexpr Scalar MOVE_SPEED = Scalar(4);

    // Ticks where the client guesses an input the server never authorised, so the
    // reconcile-and-replay path is actually exercised rather than merely present.
    constexpr Loop::TickId MISPREDICTED_TICKS[] = {7, 8, 20};

    bool is_mispredicted(Loop::TickId tick) noexcept
    {
        for (const Loop::TickId mispredicted : MISPREDICTED_TICKS)
            if (mispredicted == tick)
                return true;
        return false;
    }

    /**
     * @brief The game's per-tick command: a two-axis movement nudge.
     *
     * Trivially copyable and `operator==`-comparable, the two things Loop::App asks
     * of a command so it can be numbered, transmitted, and compared during
     * reconciliation.
     */
    struct MoveCommand
    {
        Scalar x = 0;
        Scalar z = 0;

        bool operator==(const MoveCommand& other) const noexcept
        {
            return x == other.x && z == other.z;
        }
    };

    /** @brief The player's current input for the tick, written from its command. */
    struct PlayerInput
    {
        Vector3 move;
    };

    /** @brief The player's velocity, derived from input by the "control" system. */
    struct Velocity
    {
        Vector3 value;
    };

    /** @brief The player's world position, advanced by the "integrate" system. */
    struct Position
    {
        Vector3 value;
    };

    /** @brief The server's authoritative command for a tick: a small deterministic sweep. */
    MoveCommand authoritative_command(Loop::TickId tick) noexcept
    {
        return MoveCommand{Scalar(tick % 3) - Scalar(1), Scalar((tick + 1) % 4) - Scalar(2)};
    }

    /** @brief The client's local guess: right everywhere except the mispredicted ticks. */
    MoveCommand predicted_command(Loop::TickId tick) noexcept
    {
        if (is_mispredicted(tick))
            return MoveCommand{Scalar(9), Scalar(-9)};
        return authoritative_command(tick);
    }

    /**
     * @brief Wires a game's systems, components, and input handling onto an App.
     *
     * Shared by the single-player run and the networked client so both are provably
     * the same game — the multiplayer version differs only by the transport
     * connected to it, which is the whole point of the demo.
     *
     * @param app    The app to configure.
     * @param player Receives the spawned player entity.
     */
    void build_game(Loop::App<MoveCommand>& app, Entity& player)
    {
        app.world().reserve<Position, Velocity, PlayerInput>(CHUNK_CAPACITY);

        // "control": input -> velocity. "integrate": velocity -> position. Disjoint
        // writes (Velocity vs Position), so the dependency tracker can order them by
        // the Velocity they share and run whatever it may in parallel — no scheduler
        // is written here.
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

    bool nearly_equal(Scalar a, Scalar b) noexcept
    {
        constexpr Scalar EPSILON = Scalar(1e-6);
        const Scalar diff = a - b;
        return diff > -EPSILON && diff < EPSILON;
    }

    bool nearly_equal(const Vector3& a, const Vector3& b) noexcept
    {
        return nearly_equal(a.x, b.x) && nearly_equal(a.y, b.y) && nearly_equal(a.z, b.z);
    }
}

int main()
{
    // One runtime shared by every world in this process, matching net_demo — the
    // borrowing App constructor exists precisely so a client and a server can coexist
    // without each bringing up its own runtime.
    auto runtime = SushiRuntime::API::Runtime::create();

    // --- Single-player: a deterministic run matched to a closed-form reference ---
    Loop::AppConfig config;
    config.fixed_dt_seconds = FIXED_DT;
    config.chunk_capacity = CHUNK_CAPACITY;

    Loop::App<MoveCommand> game(runtime, config);
    Entity player{};
    build_game(game, player);
    game.sample_command([](Loop::TickId tick) { return authoritative_command(tick); });
    game.start();
    game.run_for(TOTAL_TICKS);

    // Reference: v = move * speed, p += v * dt, tick by tick.
    Vector3 expected{};
    for (Loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
    {
        const MoveCommand command = authoritative_command(tick);
        const Vector3 velocity = Vector3{command.x, 0, command.z} * MOVE_SPEED;
        expected = expected + velocity * FIXED_DT;
    }

    const Vector3 actual = game.world().get<Position>(player).value;
    const bool single_player_ok = nearly_equal(actual, expected);
    const bool one_compile = game.compile_count() == 1;

    // --- Multiplayer: the same game, made networked by one connect() ------------
    Loop::App<MoveCommand> server(runtime, config);
    Entity server_player{};
    build_game(server, server_player);
    server.sample_command([](Loop::TickId tick) { return authoritative_command(tick); });
    server.start();
    server.run_for(TOTAL_TICKS);
    const Vector3 server_position = server.world().get<Position>(server_player).value;

    Loop::AppConfig client_config = config;
    client_config.rollback_capacity = TOTAL_TICKS; // retain enough ticks to rewind
    Loop::App<MoveCommand> client(runtime, client_config);
    Entity client_player{};
    build_game(client, client_player);
    client.sample_command([](Loop::TickId tick) { return predicted_command(tick); });

    // The server is authoritative: for every tick it ignores the client's guess and
    // returns the true command. Connecting this transport is the only difference
    // between the single-player game above and a networked one.
    Loop::Net::LoopbackTransport<MoveCommand> transport(
        [](Loop::TickId tick, MoveCommand) { return authoritative_command(tick); });
    client.connect(&transport);

    client.start();
    client.run_for(TOTAL_TICKS);
    const Vector3 client_position = client.world().get<Position>(client_player).value;

    const bool client_converged = nearly_equal(client_position, server_position);

    std::printf("first_game: ticks=%zu mispredicted=%zu\n", TOTAL_TICKS,
                sizeof(MISPREDICTED_TICKS) / sizeof(MISPREDICTED_TICKS[0]));
    std::printf("single_player position=(%.5f,%.5f,%.5f) expected=(%.5f,%.5f,%.5f) ok=%s\n",
                double(actual.x), double(actual.y), double(actual.z), double(expected.x),
                double(expected.y), double(expected.z), single_player_ok ? "true" : "false");
    std::printf("compile_count=%zu (expected 1)\n", game.compile_count());
    std::printf("client position=(%.5f,%.5f,%.5f) server=(%.5f,%.5f,%.5f) converged=%s\n",
                double(client_position.x), double(client_position.y), double(client_position.z),
                double(server_position.x), double(server_position.y), double(server_position.z),
                client_converged ? "true" : "false");

    const bool ok = single_player_ok && one_compile && client_converged;
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
