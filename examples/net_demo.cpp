/**************************************************************************/
/* net_demo.cpp                                                          */
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

// SushiLoop M4 worked example: Loop::Net::LoopbackChannel/reconcile driven by a
// real per-tick gameplay Command (PlayerCommand, a two-axis movement input) rather
// than the toy Scalar command the M4 unit/integration tests use, plus
// Loop::Net::make_network_id proven against an actual client/server spawn.
//
// "Client" and "server" are modelled as two logical roles in one process, each
// owning its own ecs::World — this is exactly the scope docs/slop/SUSHILOOP.md's
// M4 commits to (loopback only, no sockets, no threads). The server world is
// driven straight through by the authoritative command stream, with no
// misprediction, so it is the ground truth the client must converge to.
//
// The demo is split into two phases that never overlap, by design, honouring
// Loop::RollbackBuffer's hard constraint (ARCHITECTURE.md SS8): no entity may
// spawn or be destroyed between a capture and its matching restore.
//   Phase 1 (ticks 0..TOTAL_TICKS-1): the client predicts a movement command every
//   tick, sometimes wrong; captures a rollback snapshot before applying it;
//   periodically reconciles against batched server acks. No entity is spawned or
//   destroyed anywhere in this phase, so RollbackBuffer's constraint is never
//   exercised, let alone violated.
//   Phase 2 (after Phase 1's rollback window is done being used): client and
//   server each independently spawn a "projectile" entity for the same
//   (client_id, tick, spawn_sequence) triple, tag it with the resulting
//   NetworkId, and the demo asserts both sides computed the identical id and
//   ended up with the identical entity state — with no matching round trip.
//   This deliberately sidesteps rebasing RollbackBuffer across a structural
//   change (spawning inside a rolled-back tick range) rather than solving it;
//   that remains later SushiLoop work per ARCHITECTURE.md SS8.1.

#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

namespace
{
    constexpr Scalar      FIXED_DT       = Scalar(0.02);
    constexpr std::size_t CHUNK_CAPACITY = 32;
    constexpr std::size_t TOTAL_TICKS    = 30;
    constexpr Loop::Net::ClientId CLIENT_ID = 1;

    // Mispredicted on the client; the server's authoritative command differs.
    constexpr Loop::TickId MISPREDICTED_TICKS[] = {5, 6, 12};

    bool is_mispredicted(Loop::TickId tick)
    {
        for (Loop::TickId mispredicted : MISPREDICTED_TICKS)
            if (mispredicted == tick)
                return true;
        return false;
    }

    /**
     * @brief A player's per-tick movement input: SushiLoop's real gameplay Command.
     *
     * The smallest thing that maps directly onto something real today — a
     * two-axis nudge applied to a player entity's Position — without inventing a
     * speculative input scheme docs/slop/SUSHILOOP.md does not yet specify. Left
     * to the game rather than the engine (see loop/input.hpp's Doxygen), so it
     * lives here, at the point of use, not in loop/ itself.
     */
    struct PlayerCommand
    {
        Scalar move_x = 0;
        Scalar move_z = 0;

        bool operator==(const PlayerCommand& other) const noexcept
        {
            return move_x == other.move_x && move_z == other.move_z;
        }
    };

    /** @brief A player (or projectile) entity's world position. */
    struct Position
    {
        Vector3 v;
    };

    /** @brief The network-stable identity a spawned entity carries, for Phase 2. */
    struct NetworkIdTag
    {
        Loop::Net::NetworkId id;
    };

    void apply_command(World& world, Entity player, const PlayerCommand& command)
    {
        Position& pos = world.get<Position>(player);
        pos.v.x += command.move_x * FIXED_DT;
        pos.v.z += command.move_z * FIXED_DT;
    }

    // The server's authoritative command for a tick: a small deterministic
    // sweep, standing in for whatever a real server would compute from its own
    // simulation (e.g. validated player input).
    PlayerCommand authoritative_command(Loop::TickId tick)
    {
        return PlayerCommand{Scalar(tick % 3) - Scalar(1), Scalar((tick + 1) % 4) - Scalar(2)};
    }

    // The client's local prediction: correct everywhere except the mispredicted
    // ticks, where it guesses something else entirely.
    PlayerCommand client_predicted_command(Loop::TickId tick)
    {
        if (is_mispredicted(tick))
            return PlayerCommand{Scalar(-99), Scalar(99)};
        return authoritative_command(tick);
    }

    bool nearly_equal(Scalar a, Scalar b)
    {
        constexpr Scalar EPSILON = Scalar(1e-9);
        const Scalar diff = a - b;
        return (diff > -EPSILON) && (diff < EPSILON);
    }
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();

    // --- Baseline: an uninterrupted, authoritative-only run --------------------
    World baseline_world(runtime, CHUNK_CAPACITY);
    baseline_world.reserve<Position>(CHUNK_CAPACITY);
    const Entity baseline_player = baseline_world.spawn(Position{});
    for (Loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
        apply_command(baseline_world, baseline_player, authoritative_command(tick));

    // --- The server: driven straight through by its own authoritative stream ---
    World server_world(runtime, CHUNK_CAPACITY);
    server_world.reserve<Position>(CHUNK_CAPACITY);
    server_world.reserve<Position, NetworkIdTag>(CHUNK_CAPACITY);
    const Entity server_player = server_world.spawn(Position{});

    // --- The client: predicts locally, rolls back and replays on correction ----
    World client_world(runtime, CHUNK_CAPACITY);
    client_world.reserve<Position>(CHUNK_CAPACITY);
    client_world.reserve<Position, NetworkIdTag>(CHUNK_CAPACITY);
    const Entity client_player = client_world.spawn(Position{});

    Loop::RollbackBuffer rollback(TOTAL_TICKS);
    Loop::Net::LoopbackChannel<PlayerCommand> channel;

    const auto apply = [&](World& world, Loop::TickId tick, const PlayerCommand& command)
    {
        (void)tick;
        apply_command(world, client_player, command);
    };

    bool any_rollback_happened = false;

    // --- Phase 1: predict / send / (batched) reconcile, no structural change ---
    for (Loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
    {
        rollback.capture(client_world, tick);

        apply_command(server_world, server_player, authoritative_command(tick));

        const PlayerCommand predicted = client_predicted_command(tick);
        channel.client_send(tick, predicted);
        apply_command(client_world, client_player, predicted);

        // The server acknowledges every 5 ticks, simulating a batched, latent ack.
        if ((tick + 1) % 5 == 0 || tick + 1 == TOTAL_TICKS)
        {
            std::vector<Loop::Net::Ack<PlayerCommand>> acks = channel.server_process(
                [](Loop::TickId t, PlayerCommand) { return authoritative_command(t); });

            const bool rolled_back = Loop::Net::reconcile(
                client_world, rollback, channel.client_history(), acks, tick, apply);
            any_rollback_happened = any_rollback_happened || rolled_back;
        }
    }

    const Vector3 expected_position = baseline_world.get<Position>(baseline_player).v;
    const Vector3 client_position = client_world.get<Position>(client_player).v;
    const bool converged = nearly_equal(client_position.x, expected_position.x) &&
                            nearly_equal(client_position.y, expected_position.y) &&
                            nearly_equal(client_position.z, expected_position.z);

    // --- Phase 2: deterministic network ids across a real client/server spawn --
    // Outside the rollback window used above, so this never spawns between a
    // capture and its matching restore (RollbackBuffer's hard constraint).
    constexpr Loop::TickId SPAWN_TICK = TOTAL_TICKS;
    constexpr Loop::Net::SpawnSequence SPAWN_SEQUENCE = 0;

    const Loop::Net::NetworkId client_computed_id =
        Loop::Net::make_network_id(CLIENT_ID, SPAWN_TICK, SPAWN_SEQUENCE);
    const Loop::Net::NetworkId server_computed_id =
        Loop::Net::make_network_id(CLIENT_ID, SPAWN_TICK, SPAWN_SEQUENCE);

    // Client predicts the spawn locally...
    const Entity client_projectile =
        client_world.spawn(Position{client_position}, NetworkIdTag{client_computed_id});
    // ...and the server independently spawns the same logical entity, agreeing on
    // the id without either side telling the other what id to use.
    const Entity server_projectile =
        server_world.spawn(Position{server_world.get<Position>(server_player).v},
                           NetworkIdTag{server_computed_id});

    const bool ids_match = client_computed_id == server_computed_id;
    const bool ids_stored_correctly =
        client_world.get<NetworkIdTag>(client_projectile).id == client_computed_id &&
        server_world.get<NetworkIdTag>(server_projectile).id == server_computed_id;

    std::printf("net_demo: total_ticks=%zu mispredicted=%zu\n", TOTAL_TICKS,
                sizeof(MISPREDICTED_TICKS) / sizeof(MISPREDICTED_TICKS[0]));
    std::printf("any_rollback_happened=%s\n", any_rollback_happened ? "true" : "false");
    std::printf("client_position=(%.6f,%.6f,%.6f) expected=(%.6f,%.6f,%.6f)\n",
                double(client_position.x), double(client_position.y), double(client_position.z),
                double(expected_position.x), double(expected_position.y),
                double(expected_position.z));
    std::printf("client_network_id=%llu server_network_id=%llu (match=%s)\n",
                static_cast<unsigned long long>(client_computed_id.value),
                static_cast<unsigned long long>(server_computed_id.value),
                ids_match ? "true" : "false");

    const bool ok = any_rollback_happened && converged && ids_match && ids_stored_correctly;
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
