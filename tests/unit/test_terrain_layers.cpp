/**************************************************************************/
/* test_terrain_layers.cpp                                                */
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

// Unit_TerrainLayers: the editable half of the terrain height
// (docs/slop/solar_system_overhaul.md §6), and the height function that composes it over
// a measured source (§2 T2).
//
// The claim that carries the most weight is order stability. Composed ground has to be a
// pure function of the *set* of layers, because a server and a client that received the
// same edits in different orders must compute the same collidable surface. The operations
// used here are deliberately non-commuting -- a flatten followed by a raise is not a raise
// followed by a flatten -- so a stack that quietly broke ties by arrival order would fail
// rather than coincidentally agree.

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/cube_sphere.hpp>
#include <SushiEngine/terrain/height_function.hpp>
#include <SushiEngine/terrain/height_source.hpp>
#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Terrain;

namespace
{
    /** A source with no relief: every sample at one elevation, every address covered. */
    class FlatHeightSource final : public IHeightSource
    {
        public:
            explicit FlatHeightSource(float elevation_metres, std::uint8_t depth = 7)
                : elevation_metres_(elevation_metres), depth_(depth)
            {
            }

            std::uint8_t data_depth(const TileAddress&) const override { return depth_; }

            bool sample_tile(const TileAddress&, float* heights_metres,
                             TileStatistics& statistics) const override
            {
                for (std::uint32_t index = 0; index < TILE_SAMPLE_COUNT; ++index)
                    heights_metres[index] = elevation_metres_;
                statistics.minimum_metres = elevation_metres_;
                statistics.maximum_metres = elevation_metres_;
                return true;
            }

        private:
            float elevation_metres_;
            std::uint8_t depth_;
    };

    /** A source that covers nothing, so a composite would move past it. */
    class EmptyHeightSource final : public IHeightSource
    {
        public:
            std::uint8_t data_depth(const TileAddress&) const override { return 0; }

            bool sample_tile(const TileAddress&, float*, TileStatistics&) const override
            {
                return false;
            }
    };

    TerrainLayer make_flatten(std::uint32_t order, const Vector3& direction, double target)
    {
        TerrainLayer layer;
        layer.order = order;
        layer.operation = LayerOperation::Flatten;
        layer.footprint.direction = direction;
        layer.footprint.inner_radians = 0.010;
        layer.footprint.outer_radians = 0.020;
        layer.profile.target_metres = target;
        return layer;
    }

    TerrainLayer make_raise(std::uint32_t order, const Vector3& direction, double amount)
    {
        TerrainLayer layer;
        layer.order = order;
        layer.operation = LayerOperation::Raise;
        layer.footprint.direction = direction;
        layer.footprint.inner_radians = 0.010;
        layer.footprint.outer_radians = 0.020;
        layer.profile.amount_metres = amount;
        return layer;
    }

    const Vector3 SITE{1.0, 0.0, 0.0};
} // namespace

TEST(Unit_TerrainLayers, StrengthIsOneInsideAndZeroOutside)
{
    LayerFootprint footprint;
    footprint.direction = SITE;
    footprint.inner_radians = 0.1;
    footprint.outer_radians = 0.2;

    EXPECT_DOUBLE_EQ(layer_strength(footprint, 0.0), 1.0);
    EXPECT_DOUBLE_EQ(layer_strength(footprint, 0.1), 1.0);
    EXPECT_DOUBLE_EQ(layer_strength(footprint, 0.2), 0.0);
    EXPECT_DOUBLE_EQ(layer_strength(footprint, 5.0), 0.0);
    EXPECT_NEAR(layer_strength(footprint, 0.15), 0.5, 1e-12);

    // Monotone across the falloff, which is what stops a normal derived from the result
    // catching on a fold.
    double previous = 1.0;
    for (int step = 0; step <= 20; ++step)
    {
        const double angle = 0.1 + 0.1 * static_cast<double>(step) / 20.0;
        const double strength = layer_strength(footprint, angle);
        EXPECT_LE(strength, previous + 1e-15);
        previous = strength;
    }
}

TEST(Unit_TerrainLayers, ComposedGroundDoesNotDependOnInsertionSequence)
{
    // Flatten to 100 then raise by 50 gives 150; the reverse gives 100. The two orders
    // are distinguishable, so agreeing under shuffled insertion is a real result.
    const TerrainLayer flatten = make_flatten(10, SITE, 100.0);
    const TerrainLayer raise = make_raise(20, SITE, 50.0);

    LayerStack forward;
    ASSERT_TRUE(forward.insert(flatten));
    ASSERT_TRUE(forward.insert(raise));

    LayerStack shuffled;
    ASSERT_TRUE(shuffled.insert(raise));
    ASSERT_TRUE(shuffled.insert(flatten));

    EXPECT_DOUBLE_EQ(forward.apply(SITE, 0.0), 150.0);
    EXPECT_DOUBLE_EQ(shuffled.apply(SITE, 0.0), 150.0);

    // And the order field is what actually decides: swap the orders and the answer changes.
    LayerStack swapped;
    ASSERT_TRUE(swapped.insert(make_flatten(20, SITE, 100.0)));
    ASSERT_TRUE(swapped.insert(make_raise(10, SITE, 50.0)));
    EXPECT_DOUBLE_EQ(swapped.apply(SITE, 0.0), 100.0);
}

TEST(Unit_TerrainLayers, StackIsHeldInOrderAndRefusesADuplicate)
{
    LayerStack stack;
    ASSERT_TRUE(stack.insert(make_raise(30, SITE, 1.0)));
    ASSERT_TRUE(stack.insert(make_raise(10, SITE, 1.0)));
    ASSERT_TRUE(stack.insert(make_raise(20, SITE, 1.0)));
    ASSERT_EQ(stack.size(), 3u);
    EXPECT_EQ(stack.at(0).order, 10u);
    EXPECT_EQ(stack.at(1).order, 20u);
    EXPECT_EQ(stack.at(2).order, 30u);

    // A duplicate order would make composition depend on arrival, so it is refused
    // rather than resolved.
    EXPECT_FALSE(stack.insert(make_flatten(20, SITE, 5.0)));
    EXPECT_EQ(stack.size(), 3u);

    EXPECT_TRUE(stack.remove(20));
    EXPECT_EQ(stack.size(), 2u);
    EXPECT_FALSE(stack.remove(20));
    EXPECT_EQ(stack.at(1).order, 30u);

    stack.clear();
    EXPECT_EQ(stack.size(), 0u);
    EXPECT_DOUBLE_EQ(stack.apply(SITE, 42.0), 42.0);
}

TEST(Unit_TerrainLayers, CraterIsABowlWithARimAndMeetsTheGroundContinuously)
{
    TerrainLayer crater;
    crater.order = 1;
    crater.operation = LayerOperation::Crater;
    crater.footprint.direction = SITE;
    crater.footprint.inner_radians = 0.02; // the rim
    crater.footprint.outer_radians = 0.05; // the ejecta blanket's edge
    crater.profile.depth_metres = 300.0;
    crater.profile.rim_metres = 60.0;

    const auto height_at = [&crater](double angle)
    {
        const Vector3 direction{std::cos(angle), std::sin(angle), 0.0};
        return apply_layer(crater, direction, 0.0);
    };

    EXPECT_NEAR(height_at(0.0), -300.0, 1e-9);   // floor
    EXPECT_NEAR(height_at(0.02), 60.0, 1e-9);    // rim crest, reached from both sides
    EXPECT_NEAR(height_at(0.05), 0.0, 1e-9);     // meets the surrounding ground
    EXPECT_NEAR(height_at(0.20), 0.0, 1e-12);    // and is untouched beyond it

    // Continuity at the rim: approaching from inside and from outside must agree. The
    // step has to be small because the rim wall is genuinely steep -- its inner face
    // climbs 720 m per radian, so a 1e-6 rad step is a 4 cm height change that says
    // nothing about continuity.
    EXPECT_NEAR(height_at(0.02 - 1e-9), height_at(0.02 + 1e-9), 1e-4);

    // The floor rises monotonically out to the rim.
    double previous = height_at(0.0);
    for (int step = 1; step <= 20; ++step)
    {
        const double current = height_at(0.02 * static_cast<double>(step) / 20.0);
        EXPECT_GE(current, previous - 1e-9);
        previous = current;
    }

    // A degenerate footprint is refused rather than dividing by zero.
    crater.footprint.inner_radians = 0.0;
    EXPECT_DOUBLE_EQ(apply_layer(crater, SITE, 17.0), 17.0);
}

TEST(Unit_TerrainLayers, OverlapTestSeesNearLayersAndNotFarOnes)
{
    LayerStack stack;
    ASSERT_TRUE(stack.insert(make_raise(1, SITE, 10.0)));

    EXPECT_TRUE(stack.overlaps(SITE, 0.0));
    EXPECT_TRUE(stack.overlaps(Vector3{std::cos(0.025), std::sin(0.025), 0.0}, 0.01));
    EXPECT_FALSE(stack.overlaps(Vector3{-1.0, 0.0, 0.0}, 0.5));
}

TEST(Unit_TerrainLayers, HeightFunctionAppliesTheStackAndUpdatesStatistics)
{
    const FlatHeightSource source(0.0f);
    LayerStack stack;
    TerrainLayer raise = make_raise(1, SITE, 50.0);
    raise.footprint.inner_radians = 0.30;
    raise.footprint.outer_radians = 0.50;
    ASSERT_TRUE(stack.insert(raise));

    const Ellipsoid moon = ellipsoid_of_revolution(1737400.0, 0.0);
    const HeightFunction height(source, stack, moon);

    // The whole +X face, whose centre sample is exactly the layer's own centre.
    const TileAddress tile{CubeFace::PositiveX, 0, 0, 0};
    std::vector<float> heights(TILE_SAMPLE_COUNT, -1.0f);
    TileStatistics statistics;
    ASSERT_TRUE(height.evaluate_tile(tile, heights.data(), statistics));

    const std::uint32_t centre = TILE_STRIDE / 2u;
    EXPECT_NEAR(heights[tile_sample_index(centre, centre)], 50.0f, 1e-3f);
    EXPECT_NEAR(heights[tile_sample_index(0, 0)], 0.0f, 1e-6f); // a corner is far outside

    EXPECT_NEAR(statistics.maximum_metres, 50.0f, 1e-3f);
    EXPECT_NEAR(statistics.minimum_metres, 0.0f, 1e-6f);
}

TEST(Unit_TerrainLayers, HeightFunctionLeavesATileNoLayerReaches)
{
    const FlatHeightSource source(123.0f);
    LayerStack stack;
    ASSERT_TRUE(stack.insert(make_raise(1, Vector3{-1.0, 0.0, 0.0}, 500.0)));

    const Ellipsoid moon = ellipsoid_of_revolution(1737400.0, 0.0);
    const HeightFunction height(source, stack, moon);

    const TileAddress tile{CubeFace::PositiveX, 0, 0, 0};
    std::vector<float> heights(TILE_SAMPLE_COUNT, -1.0f);
    TileStatistics statistics;
    ASSERT_TRUE(height.evaluate_tile(tile, heights.data(), statistics));

    for (std::uint32_t index = 0; index < TILE_SAMPLE_COUNT; ++index)
        ASSERT_FLOAT_EQ(heights[index], 123.0f);
    EXPECT_FLOAT_EQ(statistics.minimum_metres, 123.0f);
    EXPECT_FLOAT_EQ(statistics.maximum_metres, 123.0f);
}

TEST(Unit_TerrainLayers, HeightFunctionReportsAnUncoveredTile)
{
    EmptyHeightSource source;
    const LayerStack stack;
    const Ellipsoid moon = ellipsoid_of_revolution(1737400.0, 0.0);
    const HeightFunction height(source, stack, moon);

    std::vector<float> heights(TILE_SAMPLE_COUNT, 7.0f);
    TileStatistics statistics;
    statistics.minimum_metres = 5.0f;
    statistics.maximum_metres = 5.0f;

    const TileAddress tile{CubeFace::PositiveZ, 2, 1, 1};
    EXPECT_FALSE(height.evaluate_tile(tile, heights.data(), statistics));
    EXPECT_FLOAT_EQ(heights[0], 7.0f);       // neither output is written
    EXPECT_FLOAT_EQ(statistics.minimum_metres, 5.0f);
    EXPECT_EQ(static_cast<int>(height.data_depth(tile)), 0);
}

TEST(Unit_TerrainLayers, StatisticsIgnoreTheApron)
{
    // The apron holds the neighbours' samples, so a bounding volume built from it would
    // extend past the tile it bounds.
    std::vector<float> heights(TILE_SAMPLE_COUNT, 10.0f);
    for (std::uint32_t index = 0; index < TILE_STRIDE; ++index)
    {
        heights[tile_sample_index(index, 0)] = 9000.0f;
        heights[tile_sample_index(0, index)] = -9000.0f;
    }
    const TileStatistics statistics = tile_statistics(heights.data());
    EXPECT_FLOAT_EQ(statistics.minimum_metres, 10.0f);
    EXPECT_FLOAT_EQ(statistics.maximum_metres, 10.0f);
}

TEST(Unit_TerrainLayers, BilinearSamplingRecoversStoredValuesAndInterpolatesBetweenThem)
{
    std::vector<float> heights(TILE_SAMPLE_COUNT, 0.0f);
    for (std::uint32_t row = 0; row < TILE_STRIDE; ++row)
        for (std::uint32_t column = 0; column < TILE_STRIDE; ++column)
            heights[tile_sample_index(column, row)] = static_cast<float>(column);

    // A grid vertex reads back exactly what is stored there.
    EXPECT_NEAR(sample_tile_bilinear(heights.data(), 0.0, 0.0),
                static_cast<float>(TILE_APRON), 1e-4f);
    EXPECT_NEAR(sample_tile_bilinear(heights.data(), 1.0, 0.0),
                static_cast<float>(TILE_APRON + TILE_GRID_SIZE - 1u), 1e-3f);

    // And halfway between two columns reads their average.
    const double half = 0.5 / static_cast<double>(TILE_GRID_SIZE - 1u);
    EXPECT_NEAR(sample_tile_bilinear(heights.data(), half, 0.0),
                static_cast<float>(TILE_APRON) + 0.5f, 1e-4f);

    // A sample just outside the tile reads the apron rather than clamping to the edge,
    // which is the whole reason the apron is stored.
    const double outside = -1.0 / static_cast<double>(TILE_GRID_SIZE - 1u);
    EXPECT_NEAR(sample_tile_bilinear(heights.data(), outside, 0.0), 0.0f, 1e-4f);
}
