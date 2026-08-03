/**************************************************************************/
/* test_terrain_quadtree.cpp                                              */
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

// Unit_TerrainQuadtree: CDLOD node selection
// (docs/slop/solar_system_overhaul.md §7.1).
//
// The load-bearing property is not "roughly the right number of nodes" -- it is that the
// emitted set is a *proper cut* of the tree: every point of the body is covered by exactly
// one node. That is checkable exactly rather than approximately, because a node at depth d
// covers 4^-d of its face and those are exact binary fractions, so the per-face areas must
// sum to exactly one. A selector that double-covers a region draws it twice and z-fights;
// one that leaves a gap draws a hole. Both are invisible in a node count and obvious here.

#include <cmath>
#include <cstdint>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/terrain/pack_format.hpp>
#include <SushiEngine/terrain/quadtree.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Terrain;

namespace
{
    constexpr double MOON_RADIUS_METRES = 1737400.0;

    /** A source with no tiles that reports one elevation band everywhere. */
    class BandedSource final : public IHeightSource
    {
        public:
            BandedSource(float minimum, float maximum)
                : minimum_(minimum), maximum_(maximum)
            {
            }

            std::uint8_t data_depth(const TileAddress&) const override { return 0; }

            bool tile_bounds(const TileAddress&, float& minimum_metres,
                             float& maximum_metres) const override
            {
                minimum_metres = minimum_;
                maximum_metres = maximum_;
                return true;
            }

            bool sample_tile(const TileAddress&, float*, TileStatistics&) const override
            {
                return false;
            }

        private:
            float minimum_;
            float maximum_;
    };

    Ellipsoid moon() { return ellipsoid_of_revolution(MOON_RADIUS_METRES, 0.0); }

    /** A camera at @p altitude above the +Z pole, in body-fixed metres. */
    Vector3 camera_above_pole(double altitude_metres)
    {
        return Vector3{0.0, 0.0, MOON_RADIUS_METRES + altitude_metres};
    }

    /** The fraction of its face a node covers. */
    double node_face_fraction(const TerrainNode& node)
    {
        return 1.0 / static_cast<double>(std::uint64_t(1) << (2u * node.address.depth));
    }

    /** Whether any strict ancestor of @p node is also in @p emitted. */
    bool ancestor_emitted(const TerrainNode& node, const std::set<std::uint64_t>& emitted)
    {
        TileAddress walk = node.address;
        while (walk.depth > 0)
        {
            walk = tile_parent(walk);
            if (emitted.count(tile_address_key(walk)) != 0)
                return true;
        }
        return false;
    }

    std::string locate_shipped_asset()
    {
        const char* prefixes[] = {"", "../", "../../", "../../../", "../../../../"};
        for (const char* prefix : prefixes)
        {
            const std::string candidate =
                std::string(prefix) + "assets/planet/moon.compact.planet";
            std::ifstream probe(candidate, std::ios::binary);
            if (probe)
                return candidate;
        }
        return std::string();
    }
} // namespace

TEST(Unit_TerrainQuadtree, SelectionIsAProperCutOfTheTree)
{
    const BandedSource source(-9000.0f, 11000.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 8;
    parameters.maximum_nodes = 100000;

    std::vector<TerrainNode> nodes;
    const QuadtreeStatistics statistics = select_terrain_nodes(
        moon(), source, camera_above_pole(50000.0), parameters, nodes);

    ASSERT_GT(nodes.size(), 0u);
    EXPECT_EQ(statistics.selected, nodes.size());
    EXPECT_FALSE(statistics.budget_exhausted);

    // Every face is covered exactly once. Exact equality, not a tolerance: the areas are
    // sums of exact binary fractions, so anything else is a gap or an overlap.
    double covered[CUBE_FACE_COUNT] = {};
    std::set<std::uint64_t> emitted;
    for (const TerrainNode& node : nodes)
    {
        covered[static_cast<std::size_t>(node.address.face)] += node_face_fraction(node);
        EXPECT_TRUE(emitted.insert(tile_address_key(node.address)).second)
            << "the same node was emitted twice";
    }
    for (std::size_t face = 0; face < CUBE_FACE_COUNT; ++face)
        EXPECT_DOUBLE_EQ(covered[face], 1.0) << "face " << face << " is not exactly covered";

    // And no node is inside another, which the area check alone would not catch if a gap
    // and an overlap happened to cancel.
    for (const TerrainNode& node : nodes)
        EXPECT_FALSE(ancestor_emitted(node, emitted))
            << "a node and its ancestor were both emitted";
}

TEST(Unit_TerrainQuadtree, RefinesTowardTheCameraAndCoarsensAwayFromIt)
{
    const BandedSource source(-9000.0f, 11000.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 12;
    parameters.maximum_nodes = 100000;

    std::vector<TerrainNode> close;
    std::vector<TerrainNode> distant;
    const QuadtreeStatistics near_statistics =
        select_terrain_nodes(moon(), source, camera_above_pole(2000.0), parameters, close);
    const QuadtreeStatistics far_statistics = select_terrain_nodes(
        moon(), source, camera_above_pole(20000000.0), parameters, distant);

    EXPECT_GT(close.size(), distant.size());
    EXPECT_GT(static_cast<int>(near_statistics.deepest),
              static_cast<int>(far_statistics.deepest));

    // The node nearest the camera is the deepest one, which is the whole point of the
    // criterion: resolution follows the observer rather than the geometry.
    const TerrainNode* nearest = &close[0];
    const TerrainNode* deepest = &close[0];
    for (const TerrainNode& node : close)
    {
        if (node.distance_metres < nearest->distance_metres)
            nearest = &node;
        if (node.address.depth > deepest->address.depth)
            deepest = &node;
    }
    EXPECT_EQ(static_cast<int>(nearest->address.depth),
              static_cast<int>(deepest->address.depth));
}

TEST(Unit_TerrainQuadtree, ScreenErrorTargetControlsDensity)
{
    const BandedSource source(0.0f, 0.0f);
    QuadtreeParameters coarse;
    coarse.maximum_depth = 12;
    coarse.maximum_nodes = 100000;
    coarse.screen_error_pixels = 8.0;

    QuadtreeParameters fine = coarse;
    fine.screen_error_pixels = 1.0;

    std::vector<TerrainNode> coarse_nodes;
    std::vector<TerrainNode> fine_nodes;
    select_terrain_nodes(moon(), source, camera_above_pole(30000.0), coarse, coarse_nodes);
    select_terrain_nodes(moon(), source, camera_above_pole(30000.0), fine, fine_nodes);

    EXPECT_GT(fine_nodes.size(), coarse_nodes.size())
        << "a tighter error target must buy more nodes, not the same ones";
}

TEST(Unit_TerrainQuadtree, TheBudgetCoarsensRatherThanTruncates)
{
    const BandedSource source(0.0f, 0.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 14;
    parameters.maximum_nodes = 200;

    std::vector<TerrainNode> nodes;
    const QuadtreeStatistics statistics = select_terrain_nodes(
        moon(), source, camera_above_pole(1000.0), parameters, nodes);

    EXPECT_LE(nodes.size(), parameters.maximum_nodes);
    EXPECT_TRUE(statistics.budget_exhausted) << "the budget must be reported when it bites";

    // The cut is still complete: running out of budget draws the whole body at lower
    // resolution, never part of it at the right one.
    double covered[CUBE_FACE_COUNT] = {};
    for (const TerrainNode& node : nodes)
        covered[static_cast<std::size_t>(node.address.face)] += node_face_fraction(node);
    for (std::size_t face = 0; face < CUBE_FACE_COUNT; ++face)
        EXPECT_DOUBLE_EQ(covered[face], 1.0) << "face " << face << " has a hole in it";
}

TEST(Unit_TerrainQuadtree, FrustumRejectsWhatIsBehindTheCamera)
{
    const BandedSource source(0.0f, 0.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 8;
    parameters.maximum_nodes = 100000;

    std::vector<TerrainNode> everything;
    select_terrain_nodes(moon(), source, camera_above_pole(100000.0), parameters, everything);

    // One plane is enough to prove the mechanism: keep the half of the body on the camera's
    // -x side. A plane that everything already satisfies would pass this test without the
    // rejection path ever running.
    FrustumPlanes frustum;
    for (int index = 0; index < 6; ++index)
    {
        frustum.plane[index][0] = -1.0;
        frustum.plane[index][1] = 0.0;
        frustum.plane[index][2] = 0.0;
        frustum.plane[index][3] = 0.0;
    }
    parameters.frustum = &frustum;

    std::vector<TerrainNode> visible;
    const QuadtreeStatistics statistics =
        select_terrain_nodes(moon(), source, camera_above_pole(100000.0), parameters, visible);

    EXPECT_GT(statistics.rejected, 0u);
    EXPECT_LT(visible.size(), everything.size());
    for (const TerrainNode& node : visible)
    {
        // Everything kept must intersect the half-space it was tested against.
        EXPECT_LE(node.origin_camera_relative.x, node.bounding_radius_metres + 1.0);
    }
}

TEST(Unit_TerrainQuadtree, MorphRangesHandOverBetweenDepths)
{
    const BandedSource source(0.0f, 0.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 10;
    parameters.maximum_nodes = 100000;

    std::vector<TerrainNode> nodes;
    select_terrain_nodes(moon(), source, camera_above_pole(20000.0), parameters, nodes);
    ASSERT_GT(nodes.size(), 0u);

    // One morph range per depth, and each is exactly twice the next one down -- which is
    // what makes a node finish morphing precisely where its parent would have taken over.
    std::vector<double> range_by_depth(MAX_TILE_DEPTH + 1u, 0.0);
    for (const TerrainNode& node : nodes)
    {
        if (node.address.depth == 0)
            continue;
        EXPECT_LT(node.morph_start_metres, node.morph_end_metres);
        EXPECT_NEAR(node.morph_start_metres,
                    node.morph_end_metres * NODE_MORPH_START_RATIO, 1e-6);

        const double previous = range_by_depth[node.address.depth];
        if (previous != 0.0)
            EXPECT_DOUBLE_EQ(previous, node.morph_end_metres)
                << "two nodes at one depth disagree about where they hand over";
        range_by_depth[node.address.depth] = node.morph_end_metres;
    }

    for (std::size_t depth = 2; depth <= MAX_TILE_DEPTH; ++depth)
    {
        if (range_by_depth[depth] == 0.0 || range_by_depth[depth - 1] == 0.0)
            continue;
        EXPECT_NEAR(range_by_depth[depth - 1], range_by_depth[depth] * 2.0,
                    range_by_depth[depth] * 1e-9)
            << "the hand-over distances of adjacent depths do not meet";
    }
}

TEST(Unit_TerrainQuadtree, CameraRelativeOriginsStaySmallEnoughForSinglePrecision)
{
    // The reason the selector holds doubles at all: what it hands downstream must be small.
    // A node's origin is bounded by its own distance plus its radius, so the far side of a
    // body never reaches the renderer as a planet-scale coordinate.
    const BandedSource source(-9000.0f, 11000.0f);
    QuadtreeParameters parameters;
    parameters.maximum_depth = 10;
    parameters.maximum_nodes = 100000;

    std::vector<TerrainNode> nodes;
    select_terrain_nodes(moon(), source, camera_above_pole(5000.0), parameters, nodes);
    ASSERT_GT(nodes.size(), 0u);

    for (const TerrainNode& node : nodes)
    {
        const double reach = length(node.origin_camera_relative);
        // Nothing can be further away than the whole body plus the camera's altitude.
        EXPECT_LT(reach, 2.0 * MOON_RADIUS_METRES + 20000.0);
        // And the deepest nodes -- the ones whose cells are smallest -- must be the near
        // ones, where float32 resolves millimetres.
        if (node.address.depth >= 9)
            EXPECT_LT(node.distance_metres, 200000.0);
    }
}

TEST(Unit_TerrainQuadtree, TighterBoundsFromASourceShrinkTheBoundingVolume)
{
    // Depth zero, so both selections emit exactly the six root faces and the only thing
    // that can differ between them is the volume each was bounded with.
    QuadtreeParameters parameters;
    parameters.maximum_depth = 0;
    parameters.maximum_nodes = 100000;

    const BandedSource wide(-12000.0f, 12000.0f);
    const BandedSource narrow(-10.0f, 10.0f);
    std::vector<TerrainNode> wide_nodes;
    std::vector<TerrainNode> narrow_nodes;
    select_terrain_nodes(moon(), wide, camera_above_pole(100000.0), parameters, wide_nodes);
    select_terrain_nodes(moon(), narrow, camera_above_pole(100000.0), parameters,
                         narrow_nodes);

    ASSERT_EQ(wide_nodes.size(), narrow_nodes.size());
    for (std::size_t index = 0; index < wide_nodes.size(); ++index)
    {
        EXPECT_EQ(wide_nodes[index].address, narrow_nodes[index].address);
        EXPECT_GT(wide_nodes[index].bounding_radius_metres,
                  narrow_nodes[index].bounding_radius_metres);
    }
}

TEST(Unit_TerrainQuadtree, SelectsAgainstTheShippedLunarPack)
{
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked lunar terrain checked out; run `se planet bake`";

    const PlanetPack pack = load_planet_pack(path);
    ASSERT_TRUE(pack.loaded());
    const PackHeightSource source(pack);

    // No frustum, so this selects the *whole* body — the far side included. That is a
    // stress case rather than a frame: a real frame rejects most of it, and the case below
    // measures that one. Measured 2026-08-01: about 14 000 nodes here against about 2 000
    // with a frustum, which is the ratio the budget in §17 is stated against.
    QuadtreeParameters parameters;
    parameters.maximum_depth = 10;
    parameters.maximum_nodes = 32768;

    std::vector<TerrainNode> nodes;
    const QuadtreeStatistics statistics = select_terrain_nodes(
        pack.ellipsoid(), source, camera_above_pole(50000.0), parameters, nodes);

    ASSERT_GT(nodes.size(), 0u);
    EXPECT_FALSE(statistics.budget_exhausted);

    // The real pack's bands are tighter than the fallback, so the bounding volumes are too.
    for (const TerrainNode& node : nodes)
    {
        EXPECT_GE(node.minimum_metres, -12000.0f);
        EXPECT_LE(node.maximum_metres, 14000.0f);
        EXPECT_GT(node.bounding_radius_metres, 0.0);
    }

    double covered[CUBE_FACE_COUNT] = {};
    for (const TerrainNode& node : nodes)
        covered[static_cast<std::size_t>(node.address.face)] += node_face_fraction(node);
    for (std::size_t face = 0; face < CUBE_FACE_COUNT; ++face)
        EXPECT_DOUBLE_EQ(covered[face], 1.0);
}

TEST(Unit_TerrainQuadtree, AFramesWorthOfNodesStaysWithinTheStatedBudget)
{
    // §17 budgets the draw against a node count, so the node count is pinned here: this is
    // the number a real frame produces, and a change to the split criterion that quietly
    // triples it should fail a test rather than a frame.
    const std::string path = locate_shipped_asset();
    if (path.empty())
        GTEST_SKIP() << "no baked lunar terrain checked out; run `se planet bake`";

    const PlanetPack pack = load_planet_pack(path);
    ASSERT_TRUE(pack.loaded());
    const PackHeightSource source(pack);

    // A 60-degree, 16:9 frustum looking straight down at the body.
    FrustumPlanes frustum;
    const double tangent = std::tan(0.5 * 1.0471975511965976);
    const double aspect = 16.0 / 9.0;
    const double horizontal = 1.0 / std::sqrt(1.0 + tangent * tangent * aspect * aspect);
    const double horizontal_z = tangent * aspect * horizontal;
    const double vertical = 1.0 / std::sqrt(1.0 + tangent * tangent);
    const double vertical_z = tangent * vertical;
    const double planes[6][4] = {{horizontal, 0.0, -horizontal_z, 0.0},
                                 {-horizontal, 0.0, -horizontal_z, 0.0},
                                 {0.0, vertical, -vertical_z, 0.0},
                                 {0.0, -vertical, -vertical_z, 0.0},
                                 {0.0, 0.0, -1.0, 0.0},
                                 {0.0, 0.0, -1.0, 0.0}};
    for (int index = 0; index < 6; ++index)
        for (int component = 0; component < 4; ++component)
            frustum.plane[index][component] = planes[index][component];

    QuadtreeParameters parameters;
    parameters.maximum_depth = 12;
    parameters.maximum_nodes = 8192;
    parameters.screen_error_pixels = 4.0;
    parameters.frustum = &frustum;

    const double altitudes[] = {5.0e3, 5.0e4, 5.0e5};
    for (double altitude : altitudes)
    {
        std::vector<TerrainNode> nodes;
        const QuadtreeStatistics statistics = select_terrain_nodes(
            pack.ellipsoid(), source, camera_above_pole(altitude), parameters, nodes);
        EXPECT_FALSE(statistics.budget_exhausted) << "at " << altitude << " m";
        EXPECT_GT(nodes.size(), 100u) << "at " << altitude << " m";
        // Measured 2026-08-01: 621 to 1272 across this range at a 4 pixel target.
        EXPECT_LT(nodes.size(), 2500u)
            << "at " << altitude << " m the selection produced " << nodes.size()
            << " nodes; §17's draw budget assumes well under this";
    }
}
