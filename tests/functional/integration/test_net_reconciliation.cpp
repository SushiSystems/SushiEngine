/**************************************************************************/
/* test_net_reconciliation.cpp                                           */
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

// Integration_NetReconciliation: SushiLoop M4's key invariant — a client that
// mispredicts a tick and later reconciles against the server's authoritative
// command must converge to exactly what an uninterrupted server-only simulation
// would have produced, the same way Integration_Rollback proves rollback+replay
// alone is bit-identical to an uninterrupted run. Here the world diverges *before*
// the correction (the client guesses a different command than the server's
// authoritative one for a handful of ticks), which is the scenario M3's own test
// cannot exercise since it always replays the same input stream.

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

    constexpr Scalar      FIXED_DT       = Scalar(0.02);
    constexpr std::size_t ENTITIES       = 6;
    constexpr std::size_t CHUNK_CAPACITY = 32;
    constexpr std::size_t TOTAL_TICKS    = 30;

    // Mispredicted on the client; the server's true, authoritative values differ.
    constexpr loop::TickId MISPREDICTED_TICKS[] = {5, 6, 12};

    bool is_mispredicted(loop::TickId tick)
    {
        for (loop::TickId mispredicted : MISPREDICTED_TICKS)
            if (mispredicted == tick)
                return true;
        return false;
    }

    void apply_command(World& world, const std::vector<Entity>& entities, Scalar command)
    {
        for (Entity e : entities)
        {
            Position& pos = world.get<Position>(e);
            pos.v.x += command * FIXED_DT;
        }
    }

    std::vector<Entity> seed_world(World& world)
    {
        world.reserve<Position>(CHUNK_CAPACITY);
        std::vector<Entity> entities;
        for (std::size_t i = 0; i < ENTITIES; ++i)
            entities.push_back(world.spawn(Position{}));
        return entities;
    }

    // The server's authoritative command for a tick.
    Scalar authoritative_command(loop::TickId tick)
    {
        return Scalar(tick % 5) - Scalar(2);
    }

    // The client's local prediction for a tick: correct everywhere except the
    // mispredicted ticks, where it guesses something else entirely.
    Scalar client_predicted_command(loop::TickId tick)
    {
        if (is_mispredicted(tick))
            return Scalar(-999);
        return authoritative_command(tick);
    }
}

TEST(Integration_NetReconciliation, ReconciledClientMatchesServerOnlySimulation)
{
    auto& runtime = Harness::shared_runtime();

    // Baseline: a "server-only" world, driven straight through by the
    // authoritative command stream, no misprediction at all.
    World server_only_world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> server_only_entities = seed_world(server_only_world);
    for (loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
        apply_command(server_only_world, server_only_entities, authoritative_command(tick));

    // The client: predicts locally (sometimes wrong), captures a rollback
    // snapshot before applying each tick, and periodically reconciles against
    // acks arriving over a loopback channel.
    World client_world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> client_entities = seed_world(client_world);

    loop::RollbackBuffer rollback(TOTAL_TICKS);
    loop::net::LoopbackChannel<Scalar> channel;

    const auto apply = [&](World& world, loop::TickId tick, const Scalar& command)
    {
        (void)tick;
        apply_command(world, client_entities, command);
    };

    bool any_rollback_happened = false;

    for (loop::TickId tick = 0; tick < TOTAL_TICKS; ++tick)
    {
        rollback.capture(client_world, tick);

        const Scalar predicted = client_predicted_command(tick);
        channel.client_send(tick, predicted);
        apply_command(client_world, client_entities, predicted);

        // The server responds every 5 ticks with authoritative acks for
        // everything sent so far, simulating a batched, latent acknowledgment.
        if ((tick + 1) % 5 == 0 || tick + 1 == TOTAL_TICKS)
        {
            std::vector<loop::net::Ack<Scalar>> acks = channel.server_process(
                [](loop::TickId t, Scalar) { return authoritative_command(t); });

            const bool rolled_back = loop::net::reconcile(
                client_world, rollback, channel.client_history(), acks, tick, apply);
            any_rollback_happened = any_rollback_happened || rolled_back;
        }
    }

    EXPECT_TRUE(any_rollback_happened)
        << "the test setup mispredicts ticks, so at least one reconciliation must fire";

    ASSERT_EQ(server_only_entities.size(), client_entities.size());
    for (std::size_t i = 0; i < server_only_entities.size(); ++i)
    {
        const Vec3 expected = server_only_world.get<Position>(server_only_entities[i]).v;
        const Vec3 actual = client_world.get<Position>(client_entities[i]).v;
        EXPECT_EQ(actual.x, expected.x) << "entity " << i;
        EXPECT_EQ(actual.y, expected.y) << "entity " << i;
        EXPECT_EQ(actual.z, expected.z) << "entity " << i;
    }
}

TEST(Integration_NetReconciliation, NoReconciliationWhenServerAgreesWithClient)
{
    auto& runtime = Harness::shared_runtime();
    World world(runtime, CHUNK_CAPACITY);
    std::vector<Entity> entities = seed_world(world);

    loop::RollbackBuffer rollback(TOTAL_TICKS);
    loop::net::LoopbackChannel<Scalar> channel;

    for (loop::TickId tick = 0; tick < 5; ++tick)
    {
        rollback.capture(world, tick);
        const Scalar command = authoritative_command(tick);
        channel.client_send(tick, command);
        apply_command(world, entities, command);
    }

    std::vector<loop::net::Ack<Scalar>> acks =
        channel.server_process([](loop::TickId t, Scalar sent) { (void)t; return sent; });

    const auto apply = [&](World& w, loop::TickId t, const Scalar& command)
    { (void)t; apply_command(w, entities, command); };

    const bool rolled_back =
        loop::net::reconcile(world, rollback, channel.client_history(), acks, 4, apply);

    EXPECT_FALSE(rolled_back);
}
