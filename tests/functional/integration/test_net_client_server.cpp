/**************************************************************************/
/* test_net_client_server.cpp                                            */
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

// Integration_NetClientServer: the client/server harness in examples/net_demo.cpp,
// as an assertable test, driven by a real per-tick gameplay Command
// (PlayerCommand, a two-axis movement input mapped onto Position) instead of the
// toy Scalar command test_net_reconciliation.cpp uses. That older test still adds
// value narrowly proving Loop::Net::reconcile's mechanics in isolation, so it is
// kept rather than removed; this one additionally proves Loop::Net::make_network_id
// against a real client/server spawn, kept outside the reconciled tick range per
// Loop::RollbackBuffer's no-structural-change constraint (see net_demo.cpp's
// top-of-file comment for the full rationale).

#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    constexpr Scalar      FIXED_DT       = Scalar(0.02);
    constexpr std::size_t CHUNK_CAPACITY = 32;
    constexpr std::size_t TOTAL_TICKS    = 30;
    constexpr Loop::Net::ClientId CLIENT_ID = 1;

    constexpr Loop::TickId MISPREDICTED_TICKS[] = {5, 6, 12};

    bool is_mispredicted(Loop::TickId tick)
    {
        for (Loop::TickId mispredicted : MISPREDICTED_TICKS)
            if (mispredicted == tick)
                return true;
        return false;
    }

    /** @brief A player's per-tick movement input; see net_demo.cpp's Doxygen. */
    struct PlayerCommand
    {
        Scalar move_x = 0;
        Scalar move_z = 0;

        bool operator==(const PlayerCommand& other) const noexcept
        {
            return move_x == other.move_x && move_z == other.move_z;
        }
    };

    struct Position { Vector3 v; };
    struct NetworkIdTag { Loop::Net::NetworkId id; };

    void apply_command(World& world, Entity player, const PlayerCommand& command)
    {
        Position& pos = world.get<Position>(player);
        pos.v.x += command.move_x * FIXED_DT;
        pos.v.z += command.move_z * FIXED_DT;
    }

    PlayerCommand authoritative_command(Loop::TickId tick)
    {
        return PlayerCommand{Scalar(tick % 3) - Scalar(1), Scalar((tick + 1) % 4) - Scalar(2)};
    }

    PlayerCommand client_predicted_command(Loop::TickId tick)
    {
        if (is_mispredicted(tick))
            return PlayerCommand{Scalar(-99), Scalar(99)};
        return authoritative_command(tick);
    }
}

TEST(Integration_NetClientServer, ReconciledClientConvergesWithRealCommand)
{
    auto& runtime = Harness::shared_runtime();

    World baseline_world(runtime, CHUNK_CAPACITY);
    baseline_world.reserve<Position>(CHUNK_CAPACITY);
    const Entity baseline_player = baseline_world.spawn(Position{});
    for (Loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
        apply_command(baseline_world, baseline_player, authoritative_command(tick));

    World client_world(runtime, CHUNK_CAPACITY);
    client_world.reserve<Position>(CHUNK_CAPACITY);
    const Entity client_player = client_world.spawn(Position{});

    Loop::RollbackBuffer rollback(TOTAL_TICKS);
    Loop::Net::LoopbackChannel<PlayerCommand> channel;

    const auto apply = [&](World& world, Loop::TickId tick, const PlayerCommand& command)
    {
        (void)tick;
        apply_command(world, client_player, command);
    };

    bool any_rollback_happened = false;

    for (Loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
    {
        rollback.capture(client_world, tick);

        const PlayerCommand predicted = client_predicted_command(tick);
        channel.client_send(tick, predicted);
        apply_command(client_world, client_player, predicted);

        if ((tick + 1) % 5 == 0 || tick + 1 == TOTAL_TICKS)
        {
            std::vector<Loop::Net::Ack<PlayerCommand>> acks = channel.server_process(
                [](Loop::TickId t, PlayerCommand) { return authoritative_command(t); });

            const bool rolled_back = Loop::Net::reconcile(
                client_world, rollback, channel.client_history(), acks, tick, apply);
            any_rollback_happened = any_rollback_happened || rolled_back;
        }
    }

    EXPECT_TRUE(any_rollback_happened)
        << "the test setup mispredicts ticks, so at least one reconciliation must fire";

    const Vector3 expected = baseline_world.get<Position>(baseline_player).v;
    const Vector3 actual = client_world.get<Position>(client_player).v;
    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.z, expected.z);
}

TEST(Integration_NetClientServer, DeterministicSpawnIdMatchesAcrossClientAndServer)
{
    auto& runtime = Harness::shared_runtime();

    // Kept outside any rollback-captured tick range, on purpose: this proves
    // make_network_id's agreement, not rollback surviving a structural change
    // (see ARCHITECTURE.md SS8.1 — that remains later work).
    constexpr Loop::TickId SPAWN_TICK = TOTAL_TICKS;
    constexpr Loop::Net::SpawnSequence SPAWN_SEQUENCE = 0;

    const Loop::Net::NetworkId client_id =
        Loop::Net::make_network_id(CLIENT_ID, SPAWN_TICK, SPAWN_SEQUENCE);
    const Loop::Net::NetworkId server_id =
        Loop::Net::make_network_id(CLIENT_ID, SPAWN_TICK, SPAWN_SEQUENCE);
    ASSERT_EQ(client_id, server_id);

    World client_world(runtime, CHUNK_CAPACITY);
    client_world.reserve<Position, NetworkIdTag>(CHUNK_CAPACITY);
    World server_world(runtime, CHUNK_CAPACITY);
    server_world.reserve<Position, NetworkIdTag>(CHUNK_CAPACITY);

    const Vector3 spawn_position{Scalar(1), Scalar(2), Scalar(3)};
    const Entity client_projectile =
        client_world.spawn(Position{spawn_position}, NetworkIdTag{client_id});
    const Entity server_projectile =
        server_world.spawn(Position{spawn_position}, NetworkIdTag{server_id});

    EXPECT_EQ(client_world.get<NetworkIdTag>(client_projectile).id,
              server_world.get<NetworkIdTag>(server_projectile).id);
    EXPECT_EQ(client_world.get<Position>(client_projectile).v.x,
              server_world.get<Position>(server_projectile).v.x);
}
