/**************************************************************************/
/* test_rng.cpp                                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_Rng: the determinism guard rail. SushiLoop requires that the only source of
// nondeterminism is player input and that RNG state lives in the world so rollback can
// snapshot and rewind it. These tests pin exactly that: identical seeds produce identical
// streams, a copied state replays the future exactly (rollback), seed 0 is well-mixed
// (not the degenerate all-zero xorshift state), and next_unit stays in [0, 1).

#include <cstdint>
#include <type_traits>

#include <gtest/gtest.h>

#include <SushiEngine/loop/rng.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Loop;

TEST(Unit_Rng, StateIsTriviallyCopyableSoItCanBeAComponent)
{
    // component_id<T>() static_asserts trivial copyability; rollback snapshots it.
    EXPECT_TRUE(std::is_trivially_copyable<RngState>::value);
}

TEST(Unit_Rng, IdenticalSeedsProduceIdenticalStreams)
{
    RngState a = seed_rng(0xC0FFEE);
    RngState b = seed_rng(0xC0FFEE);
    for (int i = 0; i < 1000; ++i)
        ASSERT_EQ(next_u64(a), next_u64(b)) << "diverged at draw " << i;
}

TEST(Unit_Rng, ZeroSeedIsWellMixedNotDegenerate)
{
    // xorshift128+ collapses to all zeros if seeded with zero state; SplitMix64 must
    // spread the bits so seed 0 still generates a live stream.
    RngState state = seed_rng(0);
    EXPECT_FALSE(state.s0 == 0 && state.s1 == 0);
    bool any_nonzero = false;
    for (int i = 0; i < 16; ++i)
        any_nonzero = any_nonzero || (next_u64(state) != 0);
    EXPECT_TRUE(any_nonzero);
}

TEST(Unit_Rng, CopiedStateReplaysTheFutureExactly)
{
    RngState live = seed_rng(42);
    for (int i = 0; i < 50; ++i)
        next_u64(live); // advance to some point in the stream

    // Snapshot, keep drawing, then replay from the snapshot: rollback-and-replay.
    const RngState snapshot = live;
    std::uint64_t forward[32];
    for (std::uint64_t& value : forward)
        value = next_u64(live);

    RngState replay = snapshot;
    for (std::uint64_t expected : forward)
        EXPECT_EQ(next_u64(replay), expected);
}

TEST(Unit_Rng, DifferentSeedsDiverge)
{
    RngState a = seed_rng(1);
    RngState b = seed_rng(2);
    EXPECT_NE(next_u64(a), next_u64(b));
}

TEST(Unit_Rng, NextUnitStaysInUnitInterval)
{
    RngState state = seed_rng(0xABCDEF123456);
    for (int i = 0; i < 100000; ++i)
    {
        const double u = next_unit(state);
        ASSERT_GE(u, 0.0);
        ASSERT_LT(u, 1.0);
    }
}
