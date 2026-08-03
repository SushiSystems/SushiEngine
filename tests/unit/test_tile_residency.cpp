/**************************************************************************/
/* test_tile_residency.cpp                                                */
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

// Unit_TileResidency: the tile cache's bookkeeping
// (docs/slop/solar_system_overhaul.md §7.2).
//
// The decisive case is inheritance. When a node draws from a coarser ancestor's image, the
// UV rectangle has to land on the *same geographic point* the node's own tile would have —
// so the check here computes that point both ways and compares, rather than asserting the
// four numbers the implementation happens to produce. Getting this wrong shifts terrain by
// a fraction of a texel everywhere it inherits, which looks like a shimmer at LOD
// boundaries and is essentially undiagnosable from a screenshot.

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/terrain/tile_residency.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Terrain;

namespace
{
    /** The UV a tile's own grid parameter maps to, straight from the layout constants. */
    double own_uv(double parameter)
    {
        const double apron = static_cast<double>(TILE_APRON);
        const double cells = static_cast<double>(TILE_GRID_SIZE - 1u);
        return (apron + 0.5 + parameter * cells) / static_cast<double>(TILE_STRIDE);
    }
} // namespace

TEST(Unit_TileResidency, IdentityRectIsTheApronOffsetAndTheGridsShare)
{
    const TileAddress tile{CubeFace::PositiveZ, 3, 5, 2};
    const TileUvRect rect = tile_uv_rect(tile, tile);

    // Not (0, 1): a tile's own grid starts one texel in and occupies 128 of 131 texels.
    EXPECT_NEAR(rect.offset_s, 1.5f / 131.0f, 1e-7f);
    EXPECT_NEAR(rect.offset_t, 1.5f / 131.0f, 1e-7f);
    EXPECT_NEAR(rect.scale_s, 128.0f / 131.0f, 1e-7f);
    EXPECT_NEAR(rect.scale_t, 128.0f / 131.0f, 1e-7f);

    // The first and last grid samples land exactly on their texel centres.
    EXPECT_NEAR(rect.offset_s, own_uv(0.0), 1e-6);
    EXPECT_NEAR(rect.offset_s + rect.scale_s, own_uv(1.0), 1e-6);
}

TEST(Unit_TileResidency, InheritedRectSamplesTheSameGeographicPoint)
{
    // Every ancestor depth of a deep node, against every sample position along it. The
    // expectation is computed from the grid coordinates rather than from the rectangle, so
    // the two agree only if the rectangle is right.
    const TileAddress node{CubeFace::NegativeY, 6, 41, 19};
    const TileGridRect inner = tile_grid_rect(node);

    TileAddress ancestor = node;
    while (ancestor.depth > 0)
    {
        ancestor = tile_parent(ancestor);
        const TileUvRect rect = tile_uv_rect(node, ancestor);
        const TileGridRect outer = tile_grid_rect(ancestor);

        for (int step = 0; step <= 8; ++step)
        {
            const double parameter = static_cast<double>(step) / 8.0;

            // The grid coordinate this sample sits at, and where that lands in the
            // ancestor's own grid.
            const double grid_s =
                inner.s_minimum + parameter * (inner.s_maximum - inner.s_minimum);
            const double grid_t =
                inner.t_minimum + parameter * (inner.t_maximum - inner.t_minimum);
            const double in_ancestor_s =
                (grid_s - outer.s_minimum) / (outer.s_maximum - outer.s_minimum);
            const double in_ancestor_t =
                (grid_t - outer.t_minimum) / (outer.t_maximum - outer.t_minimum);

            EXPECT_NEAR(static_cast<double>(rect.offset_s) +
                            parameter * static_cast<double>(rect.scale_s),
                        own_uv(in_ancestor_s), 1e-6)
                << "depth " << static_cast<int>(ancestor.depth) << " step " << step;
            EXPECT_NEAR(static_cast<double>(rect.offset_t) +
                            parameter * static_cast<double>(rect.scale_t),
                        own_uv(in_ancestor_t), 1e-6);
        }

        // And an inherited rectangle is strictly smaller than the identity one, halving
        // per level, which is what "the same terrain at lower detail" means.
        const double expected_scale =
            (128.0 / 131.0) / static_cast<double>(std::uint64_t(1)
                                                  << (node.depth - ancestor.depth));
        EXPECT_NEAR(static_cast<double>(rect.scale_s), expected_scale, 1e-6);
    }
}

TEST(Unit_TileResidency, BindPrefersTheNodesOwnTileAndFallsBackToAnAncestor)
{
    TileResidency residency(16);
    residency.begin_frame(1);

    const TileAddress root{CubeFace::PositiveX, 0, 0, 0};
    const TileAddress child{CubeFace::PositiveX, 2, 1, 1};
    ASSERT_NE(residency.insert(root, -100.0f, 900.0f), INVALID_TILE_SLOT);

    TileBinding binding;
    ASSERT_TRUE(residency.bind(child, binding));
    EXPECT_FALSE(binding.exact) << "the child's own tile is not resident";
    EXPECT_EQ(binding.source, root);
    EXPECT_FLOAT_EQ(binding.minimum_metres, -100.0f);
    EXPECT_FLOAT_EQ(binding.maximum_metres, 900.0f);

    // Once its own tile arrives it takes precedence, with the identity rectangle.
    const std::uint32_t slot = residency.insert(child, 10.0f, 20.0f);
    ASSERT_NE(slot, INVALID_TILE_SLOT);
    ASSERT_TRUE(residency.bind(child, binding));
    EXPECT_TRUE(binding.exact);
    EXPECT_EQ(binding.slot, slot);
    EXPECT_EQ(binding.source, child);
    EXPECT_NEAR(binding.rect.scale_s, 128.0f / 131.0f, 1e-6f);
    EXPECT_FLOAT_EQ(binding.minimum_metres, 10.0f);
}

TEST(Unit_TileResidency, BindFailsWhenNothingCoversTheNode)
{
    TileResidency residency(4);
    residency.begin_frame(1);
    residency.insert(TileAddress{CubeFace::PositiveX, 0, 0, 0}, 0.0f, 1.0f);

    TileBinding binding;
    binding.slot = 123;
    EXPECT_FALSE(residency.bind(TileAddress{CubeFace::NegativeZ, 3, 0, 0}, binding));
    EXPECT_EQ(binding.slot, INVALID_TILE_SLOT) << "a failed bind must not leave stale state";
}

TEST(Unit_TileResidency, ReinsertingATileKeepsItsSlotAndUpdatesItsRange)
{
    // What a recompile after a layer edit does: the image is rewritten in place, and the
    // decode range moves with it.
    TileResidency residency(8);
    residency.begin_frame(1);

    const TileAddress tile{CubeFace::PositiveY, 1, 0, 1};
    const std::uint32_t first = residency.insert(tile, 0.0f, 100.0f);
    const std::uint32_t second = residency.insert(tile, -500.0f, 500.0f);
    EXPECT_EQ(first, second);
    EXPECT_EQ(residency.resident_count(), 1u);

    TileBinding binding;
    ASSERT_TRUE(residency.bind(tile, binding));
    EXPECT_FLOAT_EQ(binding.minimum_metres, -500.0f);
    EXPECT_FLOAT_EQ(binding.maximum_metres, 500.0f);
}

TEST(Unit_TileResidency, EvictionTakesTheColdestSlot)
{
    TileResidency residency(3);

    residency.begin_frame(1);
    const TileAddress a{CubeFace::PositiveX, 1, 0, 0};
    const TileAddress b{CubeFace::PositiveX, 1, 1, 0};
    const TileAddress c{CubeFace::PositiveX, 1, 0, 1};
    residency.insert(a, 0.0f, 1.0f);
    residency.insert(b, 0.0f, 1.0f);
    residency.insert(c, 0.0f, 1.0f);
    EXPECT_EQ(residency.resident_count(), 3u);

    // Frame 2 touches b and c but not a.
    residency.begin_frame(2);
    TileBinding binding;
    ASSERT_TRUE(residency.bind(b, binding));
    ASSERT_TRUE(residency.bind(c, binding));

    // Frame 3 needs a fourth tile; a is the one nobody has drawn from.
    residency.begin_frame(3);
    const TileAddress d{CubeFace::PositiveX, 1, 1, 1};
    const std::uint32_t slot = residency.insert(d, 0.0f, 1.0f);
    ASSERT_NE(slot, INVALID_TILE_SLOT);
    EXPECT_EQ(residency.find(a), INVALID_TILE_SLOT) << "the coldest tile should have gone";
    EXPECT_NE(residency.find(b), INVALID_TILE_SLOT);
    EXPECT_NE(residency.find(c), INVALID_TILE_SLOT);
    EXPECT_NE(residency.find(d), INVALID_TILE_SLOT);
}

TEST(Unit_TileResidency, BindingThroughAnAncestorKeepsThatAncestorAlive)
{
    // The subtlety the whole inheritance scheme rests on. An ancestor may have no node of
    // its own being drawn and still be read by every one of its descendants; evicting it
    // because "nothing drew it" would pull the image out from under all of them.
    TileResidency residency(2);

    residency.begin_frame(1);
    const TileAddress root{CubeFace::NegativeX, 0, 0, 0};
    const TileAddress other{CubeFace::PositiveY, 0, 0, 0};
    residency.insert(root, 0.0f, 1.0f);
    residency.insert(other, 0.0f, 1.0f);

    residency.begin_frame(2);
    TileBinding binding;
    ASSERT_TRUE(residency.bind(TileAddress{CubeFace::NegativeX, 4, 3, 7}, binding));
    EXPECT_EQ(binding.source, root);

    residency.begin_frame(3);
    residency.insert(TileAddress{CubeFace::PositiveZ, 0, 0, 0}, 0.0f, 1.0f);
    EXPECT_NE(residency.find(root), INVALID_TILE_SLOT)
        << "an ancestor being inherited from was evicted";
    EXPECT_EQ(residency.find(other), INVALID_TILE_SLOT);
}

TEST(Unit_TileResidency, RefusesToEvictSlotsAlreadyBoundThisFrame)
{
    // Handing out a slot that a node already queued to draw from is reading would corrupt
    // that node's terrain, so a full cache refuses rather than steals.
    TileResidency residency(2);
    residency.begin_frame(7);
    const TileAddress a{CubeFace::PositiveX, 0, 0, 0};
    const TileAddress b{CubeFace::NegativeX, 0, 0, 0};
    ASSERT_NE(residency.insert(a, 0.0f, 1.0f), INVALID_TILE_SLOT);
    ASSERT_NE(residency.insert(b, 0.0f, 1.0f), INVALID_TILE_SLOT);

    const TileAddress c{CubeFace::PositiveY, 0, 0, 0};
    EXPECT_EQ(residency.insert(c, 0.0f, 1.0f), INVALID_TILE_SLOT);
    EXPECT_EQ(residency.resident_count(), 2u);

    // A later frame may take them.
    residency.begin_frame(8);
    EXPECT_NE(residency.insert(c, 0.0f, 1.0f), INVALID_TILE_SLOT);
}

TEST(Unit_TileResidency, EvictFreesTheSlotForReuse)
{
    TileResidency residency(1);
    residency.begin_frame(1);
    const TileAddress a{CubeFace::PositiveX, 0, 0, 0};
    const std::uint32_t slot = residency.insert(a, 0.0f, 1.0f);
    ASSERT_NE(slot, INVALID_TILE_SLOT);
    EXPECT_TRUE(residency.slot_occupied(slot));
    EXPECT_EQ(residency.slot_address(slot), a);

    EXPECT_TRUE(residency.evict(a));
    EXPECT_FALSE(residency.evict(a));
    EXPECT_EQ(residency.resident_count(), 0u);

    const TileAddress b{CubeFace::NegativeZ, 2, 1, 1};
    EXPECT_EQ(residency.insert(b, 0.0f, 1.0f), slot) << "the freed slot should be reused";
}

TEST(Unit_TileResidency, CapacityBoundsResidency)
{
    TileResidency residency(5);
    EXPECT_EQ(residency.capacity(), 5u);
    for (std::uint32_t index = 0; index < 40u; ++index)
    {
        residency.begin_frame(index + 1u);
        residency.insert(TileAddress{CubeFace::PositiveX, 3, index % 8u, index / 8u},
                         0.0f, 1.0f);
        EXPECT_LE(residency.resident_count(), residency.capacity());
    }
}

TEST(Unit_TileResidency, EvictionSparesEveryFrameStillInFlight)
{
    // The hazard the frames-in-flight window closes. A slot the previous frame bound is
    // being read by a command buffer the device has not finished, so re-pointing it and
    // copying new pixels in is a write racing that read -- and the render graph cannot
    // help, because the two are in different submissions. Sparing only "this frame" is
    // enough for a caller that submits and waits and wrong for every real frame chain.
    TileResidency residency(2, 3);

    residency.begin_frame(10);
    const TileAddress oldest{CubeFace::PositiveX, 0, 0, 0};
    ASSERT_NE(residency.insert(oldest, 0.0f, 1.0f), INVALID_TILE_SLOT);

    residency.begin_frame(11);
    const TileAddress newer{CubeFace::NegativeX, 0, 0, 0};
    ASSERT_NE(residency.insert(newer, 0.0f, 1.0f), INVALID_TILE_SLOT);

    // Frame 12: both slots were touched inside the three-frame window, so the pool has
    // nothing it may take -- the tile waits a frame rather than corrupting one.
    residency.begin_frame(12);
    EXPECT_EQ(residency.insert(TileAddress{CubeFace::PositiveY, 0, 0, 0}, 0.0f, 1.0f),
              INVALID_TILE_SLOT);

    // Frame 13: the oldest has finally aged out of the window and becomes the victim.
    residency.begin_frame(13);
    const TileAddress arrival{CubeFace::NegativeY, 0, 0, 0};
    const std::uint32_t slot = residency.insert(arrival, 0.0f, 1.0f);
    ASSERT_NE(slot, INVALID_TILE_SLOT);
    EXPECT_EQ(residency.slot_address(slot), arrival);
    EXPECT_EQ(residency.find(oldest), INVALID_TILE_SLOT);
    EXPECT_NE(residency.find(newer), INVALID_TILE_SLOT);
}

TEST(Unit_TileResidency, AWindowOfOneIsTheOldBehaviourExactly)
{
    // The default has to keep meaning what it meant, or every test above this one is
    // testing a different class than it was written against.
    TileResidency residency(1);
    residency.begin_frame(5);
    const TileAddress held{CubeFace::PositiveZ, 1, 0, 1};
    ASSERT_NE(residency.insert(held, 0.0f, 1.0f), INVALID_TILE_SLOT);
    EXPECT_EQ(residency.insert(TileAddress{CubeFace::PositiveZ, 1, 1, 1}, 0.0f, 1.0f),
              INVALID_TILE_SLOT)
        << "a slot bound this frame is never a victim";

    residency.begin_frame(6);
    const TileAddress next{CubeFace::PositiveZ, 1, 1, 1};
    EXPECT_NE(residency.insert(next, 0.0f, 1.0f), INVALID_TILE_SLOT)
        << "and one bound last frame is, at a window of one";
}

TEST(Unit_TileResidency, ClearForgetsEveryTileAndKeepsThePool)
{
    // What re-pointing the pool at another body needs: the slots are anonymous storage, so
    // the pool survives and only its index is body-specific. A tile from the old body must
    // not still resolve, or the new world would draw the old world's ground.
    TileResidency residency(4, 3);
    residency.begin_frame(1);
    const TileAddress before[] = {{CubeFace::PositiveX, 0, 0, 0},
                                  {CubeFace::NegativeX, 1, 1, 0},
                                  {CubeFace::PositiveY, 2, 3, 1}};
    for (const TileAddress& tile : before)
        ASSERT_NE(residency.insert(tile, -100.0f, 900.0f), INVALID_TILE_SLOT);
    ASSERT_EQ(residency.resident_count(), 3u);

    residency.clear();

    EXPECT_EQ(residency.resident_count(), 0u);
    EXPECT_EQ(residency.capacity(), 4u);
    for (const TileAddress& tile : before)
    {
        EXPECT_EQ(residency.find(tile), INVALID_TILE_SLOT);
        TileBinding binding;
        EXPECT_FALSE(residency.bind(tile, binding));
    }
    for (std::uint32_t slot = 0; slot < residency.capacity(); ++slot)
        EXPECT_FALSE(residency.slot_occupied(slot));

    // And all four slots are usable again immediately -- the window does not outlive the
    // tiles it was protecting, because the caller idled the device before calling this.
    residency.begin_frame(2);
    for (std::uint32_t index = 0; index < 4u; ++index)
        EXPECT_NE(residency.insert(TileAddress{CubeFace::NegativeZ, 3, index, 0}, 0.0f, 1.0f),
                  INVALID_TILE_SLOT);
}
