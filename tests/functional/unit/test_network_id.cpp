/**************************************************************************/
/* test_network_id.cpp                                                   */
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

// Unit_NetworkId: SushiLoop M4's deterministic entity identity
// (Loop::Net::make_network_id). Two independent computations from the same
// (client_id, tick, spawn_sequence) triple must always agree — the whole point is
// that server and client each derive the same id without a matching round-trip —
// and distinct inputs along any one axis must not collide within the packed range
// this milestone commits to.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Loop::net;

TEST(Unit_NetworkId, SameInputsProduceTheSameId)
{
    const NetworkId a = make_network_id(7, 1234, 3);
    const NetworkId b = make_network_id(7, 1234, 3);
    EXPECT_EQ(a, b);
}

TEST(Unit_NetworkId, DifferentClientsNeverCollide)
{
    const NetworkId a = make_network_id(1, 1000, 0);
    const NetworkId b = make_network_id(2, 1000, 0);
    EXPECT_NE(a, b);
}

TEST(Unit_NetworkId, DifferentTicksNeverCollide)
{
    const NetworkId a = make_network_id(1, 1000, 0);
    const NetworkId b = make_network_id(1, 1001, 0);
    EXPECT_NE(a, b);
}

TEST(Unit_NetworkId, DifferentSpawnSequenceNeverCollide)
{
    const NetworkId a = make_network_id(1, 1000, 0);
    const NetworkId b = make_network_id(1, 1000, 1);
    EXPECT_NE(a, b);
}

TEST(Unit_NetworkId, NotAssignedByWhicheverSideSpawnsFirst)
{
    // Simulates server and client independently computing the id for the same
    // spawn (client 3's second spawn on tick 42), in arbitrary evaluation order —
    // neither "wins", both compute the same value from the same facts.
    const ClientId client = 3;
    const Loop::TickId tick = 42;
    const SpawnSequence sequence = 1;

    const NetworkId server_computed = make_network_id(client, tick, sequence);
    const NetworkId client_computed = make_network_id(client, tick, sequence);
    EXPECT_EQ(server_computed, client_computed);
}
