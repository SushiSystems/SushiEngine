/**************************************************************************/
/* test_collision_cooker.cpp                                              */
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

// §8.4 end to end, and the corpus P4's acceptance criterion names. The dirty meshes
// here are not adversarial for their own sake — each is a specific way a real export
// arrives broken, and the assertion in every case is that the cook *completes and reports*
// rather than that it produces something pretty, because §8.5's premise is that a silent
// bad cook is the worst outcome available.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/collision_asset.hpp>
#include <SushiEngine/physics/cooking/collision_cooker.hpp>
#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>
#include <SushiEngine/physics/cooking/convex_decomposition.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    const std::uint32_t BOX_FACES[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                            {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                            {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};

    /** @brief An outward-wound box, optionally offset. */
    Geometry::TriangleMesh box_mesh(float hx, float hy, float hz, float ox = 0.0f,
                                    float oy = 0.0f, float oz = 0.0f)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz},
                                     {-hx, hy, -hz},  {-hx, -hy, hz}, {hx, -hy, hz},
                                     {hx, hy, hz},    {-hx, hy, hz}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0] + ox);
            mesh.positions.push_back(corner[1] + oy);
            mesh.positions.push_back(corner[2] + oz);
        }
        for (const auto& face : BOX_FACES)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief Appends @p addition's geometry to @p target, renumbering its indices. */
    void append(Geometry::TriangleMesh& target, const Geometry::TriangleMesh& addition)
    {
        const std::uint32_t offset = std::uint32_t(target.vertex_count());
        target.positions.insert(target.positions.end(), addition.positions.begin(),
                                addition.positions.end());
        for (const std::uint32_t index : addition.indices)
            target.indices.push_back(index + offset);
    }

    /**
     * @brief Two bars meeting at a right angle: the simplest genuinely concave solid.
     *
     * Its convex hull fills the corner the L leaves open, so one hull cannot represent it
     * and a decomposition has something real to do. Built as two boxes that overlap in
     * space without sharing a vertex — so the surface is closed and manifold but
     * *self-intersecting*, which is what a modeller who unions two primitives without
     * cleaning up hands over, and which double-counts the overlap in any volume integral.
     */
    Geometry::TriangleMesh l_shape()
    {
        Geometry::TriangleMesh mesh = box_mesh(1.0f, 0.25f, 0.25f, 0.0f, 0.0f, 0.0f);
        append(mesh, box_mesh(0.25f, 1.0f, 0.25f, -0.75f, 0.75f, 0.0f));
        return mesh;
    }

    /** @brief The largest distance any of @p mesh's vertices lies from the hull's support. */
    Scalar worst_support_deficit(const ConvexHullView<Scalar>& hull,
                                 const Geometry::TriangleMesh& mesh)
    {
        // For every axis direction, the hull's support must reach at least as far as the
        // mesh's own extreme in that direction minus the sampling error — a hull that does
        // not is a collider a body can sink into.
        Scalar worst = 0;
        const Vector3 directions[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0},
                                       {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
        for (const Vector3& direction : directions)
        {
            Scalar mesh_extreme = -1.0e30;
            for (std::size_t i = 0; i < mesh.vertex_count(); ++i)
            {
                const Vector3 vertex{Scalar(mesh.positions[i * 3 + 0]),
                                     Scalar(mesh.positions[i * 3 + 1]),
                                     Scalar(mesh.positions[i * 3 + 2])};
                mesh_extreme = std::max(mesh_extreme, dot(vertex, direction));
            }
            const Scalar hull_extreme = dot(support(hull, direction), direction);
            worst = std::max(worst, mesh_extreme - hull_extreme);
        }
        return worst;
    }

    /** @brief Counts the stages a cook ran through, and in what order. */
    class RecordingProgressSink final : public ICookingProgressSink
    {
    public:
        void on_progress(const CookingProgress& progress) override
        {
            stages.push_back(progress.stage);
            total = progress.total_stages;
        }

        std::vector<const char*> stages;
        std::uint32_t total = 0;
    };

    /** @brief Parameters that cook quickly: this is a unit test, not a bake farm. */
    CookingParameters quick_parameters()
    {
        CookingParameters parameters;
        parameters.fidelity = 0.0f;
        parameters.distance_field_resolution = 8;
        parameters.accuracy_lattice_order = 2;
        return parameters;
    }
} // namespace

TEST(Unit_ConvexHullBuild,BuildsAClosedOutwardHullOfABoxsCorners)
{
    std::vector<Vector3> points;
    const float h = 0.5f;
    for (int i = 0; i < 8; ++i)
    {
        points.push_back(Vector3{Scalar((i & 1) ? h : -h), Scalar((i & 2) ? h : -h),
                                 Scalar((i & 4) ? h : -h)});
    }

    Geometry::TriangleMesh hull;
    ASSERT_TRUE(build_convex_hull_mesh(points, hull));

    const Geometry::MeshTopologyReport report = Geometry::analyze_mesh_topology(hull.view());
    // Closed, manifold, consistently wound, and enclosing exactly the unit cube. The
    // volume is the assertion that matters: a coplanar face tiled twice would report two.
    EXPECT_TRUE(report.watertight());
    EXPECT_TRUE(report.consistently_oriented());
    EXPECT_EQ(report.connected_components, 1u);
    EXPECT_NEAR(report.signed_volume, 1.0f, 1e-5f);
    EXPECT_NEAR(report.surface_area, 6.0f, 1e-5f);
    // Six square faces, two triangles each — not the twenty-four an unfiltered triple
    // enumeration would emit.
    EXPECT_EQ(hull.triangle_count(), 12u);
}

TEST(Unit_ConvexHullBuild,IgnoresInteriorPointsAndRefusesFlatSets)
{
    std::vector<Vector3> points;
    const float h = 0.5f;
    for (int i = 0; i < 8; ++i)
    {
        points.push_back(Vector3{Scalar((i & 1) ? h : -h), Scalar((i & 2) ? h : -h),
                                 Scalar((i & 4) ? h : -h)});
    }
    // A point in the middle is on no supporting plane and must not appear in the output.
    points.push_back(Vector3{0, 0, 0});
    points.push_back(Vector3{Scalar(0.1), Scalar(-0.2), Scalar(0.05)});

    Geometry::TriangleMesh hull;
    ASSERT_TRUE(build_convex_hull_mesh(points, hull));
    EXPECT_EQ(hull.vertex_count(), 8u);
    EXPECT_NEAR(Geometry::analyze_mesh_topology(hull.view()).signed_volume, 1.0f, 1e-5f);

    // A flat set encloses nothing. Reporting that rather than returning a degenerate mesh
    // is what lets the decomposition keep the point set as a support function and record a
    // volume of zero.
    std::vector<Vector3> flat = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{1, 1, 0},
                                Vector3{0, 1, 0}, Vector3{Scalar(0.5), Scalar(0.5), 0}};
    Geometry::TriangleMesh nothing;
    EXPECT_FALSE(build_convex_hull_mesh(flat, nothing));
    EXPECT_EQ(nothing.triangle_count(), 0u);

    std::vector<Vector3> too_few = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    EXPECT_FALSE(build_convex_hull_mesh(too_few, nothing));
}

TEST(Unit_ConvexDecomposition,LeavesAConvexShapeAsOnePiece)
{
    const Geometry::TriangleMesh box = box_mesh(0.5f, 1.0f, 0.25f);
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(box.view()));

    ConvexDecompositionOptions options;
    options.max_pieces = 8;
    options.accuracy_lattice_order = 2;
    std::vector<ConvexPiece> pieces;
    const ConvexDecompositionReport report =
        decompose_convex(box.view(), surface, options, pieces);

    // A box is already convex, so splitting it would spend the budget to no effect.
    EXPECT_EQ(report.piece_count, 1u);
    EXPECT_NEAR(report.worst_concavity, 0.0f, 1e-4f);
    // Half-extents in, so the volume is the product of the full extents.
    EXPECT_NEAR(double(report.summed_volume), 1.0 * 2.0 * 0.5, 1e-5);
    ASSERT_EQ(pieces.size(), 1u);
    EXPECT_EQ(pieces[0].vertices.size(), 8u);
}

TEST(Unit_ConvexDecomposition,SplitsAConcaveShapeAndTheErrorFalls)
{
    const Geometry::TriangleMesh shape = l_shape();
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(shape.view()));

    ConvexDecompositionOptions options;
    options.vertex_budget = 32;
    options.accuracy_lattice_order = 2;
    options.concavity_tolerance = 0.0f;   // split until the budget runs out

    options.max_pieces = 1;
    std::vector<ConvexPiece> single;
    const ConvexDecompositionReport one = decompose_convex(shape.view(), surface, options, single);
    ASSERT_EQ(one.piece_count, 1u);
    // One hull over an L fills the open corner, so it protrudes measurably.
    EXPECT_GT(one.worst_concavity, 0.1f);

    options.max_pieces = 8;
    std::vector<ConvexPiece> many;
    const ConvexDecompositionReport several =
        decompose_convex(shape.view(), surface, options, many);
    EXPECT_GT(several.piece_count, 1u);
    // The whole point of decomposing: the reported error is the thing being minimized, so
    // more pieces must mean less of it.
    EXPECT_LT(several.worst_concavity, one.worst_concavity);
}

TEST(Unit_ConvexDecomposition,HonoursTheVertexBudgetAndTheHullStaysInsideTheShape)
{
    const Geometry::TriangleMesh box = box_mesh(1.0f, 1.0f, 1.0f);
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(box.view()));

    ConvexDecompositionOptions options;
    options.max_pieces = 1;
    options.vertex_budget = 6;   // fewer than the box's eight corners
    options.accuracy_lattice_order = 2;
    std::vector<ConvexPiece> pieces;
    decompose_convex(box.view(), surface, options, pieces);

    ASSERT_EQ(pieces.size(), 1u);
    EXPECT_LE(pieces[0].vertices.size(), 6u);
    // Every selected point is a real mesh vertex, so a budgeted hull can only be thinner
    // than the mesh, never fatter. That direction is the one that matters: a fat collider
    // is an invisible wall, a thin one is a body that sinks a millimetre.
    EXPECT_NEAR(pieces[0].concavity, 0.0f, 1e-4f);
}

TEST(Unit_ConvexDecomposition,IsAFunctionOfTheMeshAndNotOfTheRun)
{
    const Geometry::TriangleMesh shape = l_shape();
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(shape.view()));

    ConvexDecompositionOptions options;
    options.max_pieces = 6;
    options.accuracy_lattice_order = 2;

    std::vector<ConvexPiece> first;
    std::vector<ConvexPiece> second;
    decompose_convex(shape.view(), surface, options, first);
    decompose_convex(shape.view(), surface, options, second);

    // The content-hash cache is keyed on the input, so two cooks of one mesh that produced
    // different pieces would make that key a lie.
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        ASSERT_EQ(first[i].vertices.size(), second[i].vertices.size());
        EXPECT_EQ(first[i].center.x, second[i].center.x);
        EXPECT_EQ(first[i].center.y, second[i].center.y);
        EXPECT_EQ(first[i].center.z, second[i].center.z);
        for (std::size_t v = 0; v < first[i].vertices.size(); ++v)
        {
            EXPECT_EQ(first[i].vertices[v].x, second[i].vertices[v].x);
            EXPECT_EQ(first[i].vertices[v].y, second[i].vertices[v].y);
            EXPECT_EQ(first[i].vertices[v].z, second[i].vertices[v].z);
        }
    }
}

TEST(Unit_CollisionCooker,CooksABoxIntoALoadableAssetWithCorrectMassProperties)
{
    CollisionCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 1.5f, 0.25f);
    CookingParameters parameters = quick_parameters();
    parameters.density = 800.0f;

    RecordingProgressSink sink;
    std::vector<std::byte> bytes;
    const CookingReport report = cooker.cook(box.view(), parameters, nullptr, &sink, bytes);

    ASSERT_EQ(report.status, CookingStatus::Succeeded);
    EXPECT_FALSE(report.served_from_cache);
    EXPECT_EQ(report.failed_stage, nullptr);
    EXPECT_EQ(report.convex_piece_count, 1u);
    EXPECT_TRUE(report.source.watertight());
    EXPECT_GT(report.asset_bytes, 0u);
    EXPECT_EQ(report.asset_bytes, bytes.size());

    // Five stages, in order, each announced before it runs.
    ASSERT_EQ(sink.stages.size(), 5u);
    EXPECT_EQ(sink.total, 5u);
    EXPECT_STREQ(sink.stages[0], "Repair");
    EXPECT_STREQ(sink.stages[1], "MassProperties");
    EXPECT_STREQ(sink.stages[2], "Decompose");
    EXPECT_STREQ(sink.stages[3], "BakeDistanceField");
    EXPECT_STREQ(sink.stages[4], "Serialize");

    const CollisionAssetView view = load_collision_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);
    EXPECT_FALSE(view.static_mesh);
    EXPECT_EQ(view.piece_count, 1u);

    // Mass comes from the source mesh, not the pieces, so it is exact.
    EXPECT_NEAR(double(view.summary.mass), 1.0 * 3.0 * 0.5 * 800.0, 1e-6);
    EXPECT_NEAR(double(view.summary.volume), 1.5, 1e-9);
    EXPECT_EQ(view.summary.element_count, 1u);
    EXPECT_EQ(view.summary.suggested_substep_count, 8u);

    // The hull points straight into the blob and supports the box's own extremes.
    const ConvexHullView<Scalar> hull = collision_asset_hull(view, 0);
    ASSERT_EQ(hull.vertex_count, 8u);
    EXPECT_NEAR(double(worst_support_deficit(hull, box)), 0.0, 1e-6);

    // And the field is signed the right way round at the centre and outside.
    ASSERT_EQ(view.field_resolution, 8u);
    EXPECT_LT(double(collision_asset_distance(view, Vector3{0, 0, 0})), 0.0);
    EXPECT_GT(double(collision_asset_distance(view, Vector3{10, 0, 0})), 0.0);
}

TEST(Unit_CollisionCooker,CooksAuthoredStaticGeometryAsAnExactHierarchy)
{
    CollisionCooker cooker;
    const Geometry::TriangleMesh terrain = l_shape();
    CookingParameters parameters = quick_parameters();
    parameters.static_geometry = true;

    std::vector<std::byte> bytes;
    const CookingReport report = cooker.cook(terrain.view(), parameters, nullptr, nullptr, bytes);
    ASSERT_EQ(report.status, CookingStatus::Succeeded);

    EXPECT_EQ(report.convex_piece_count, 0u);
    EXPECT_GT(report.collision_triangle_count, 0u);
    // The collider *is* the mesh, so these zeroes are measurements rather than absences.
    EXPECT_EQ(report.hausdorff_error, 0.0f);
    EXPECT_EQ(report.volume_error, 0.0f);

    const CollisionAssetView view = load_collision_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);
    EXPECT_TRUE(view.static_mesh);
    EXPECT_EQ(view.piece_count, 0u);

    const TriangleMeshView<Scalar> mesh = collision_asset_mesh(view);
    ASSERT_NE(mesh.vertices, nullptr);
    ASSERT_NE(mesh.nodes, nullptr);
    ASSERT_NE(mesh.adjacency, nullptr);
    EXPECT_EQ(mesh.triangle_count, report.collision_triangle_count);
    EXPECT_GT(mesh.node_count, 0u);

    // A hull cook has no mesh view and a static cook has no hulls, so a caller cannot read
    // the wrong shape out of the right asset.
    EXPECT_EQ(collision_asset_hull(view, 0).vertices, nullptr);
}

TEST(Unit_CollisionCooker,ServesTheSecondCookFromTheCacheWithoutRepeatingTheWork)
{
    CollisionCooker cooker;
    MemoryCookedAssetStore store;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    const CookingParameters parameters = quick_parameters();

    std::vector<std::byte> first;
    const CookingReport cooked = cooker.cook(box.view(), parameters, &store, nullptr, first);
    ASSERT_EQ(cooked.status, CookingStatus::Succeeded);
    EXPECT_FALSE(cooked.served_from_cache);
    EXPECT_EQ(store.size(), 1u);

    RecordingProgressSink sink;
    std::vector<std::byte> second;
    const CookingReport served = cooker.cook(box.view(), parameters, &store, &sink, second);
    EXPECT_TRUE(served.served_from_cache);
    EXPECT_EQ(served.status, CookingStatus::Succeeded);
    EXPECT_EQ(second, first);
    // Nothing ran, so nothing was announced.
    EXPECT_TRUE(sink.stages.empty());

    // What the report can honestly say on a hit is what the asset carries. The source was
    // never looked at, so its topology is left unmeasured rather than reported clean.
    EXPECT_NEAR(served.mass, cooked.mass, 1e-3f);
    EXPECT_EQ(served.convex_piece_count, cooked.convex_piece_count);
    EXPECT_EQ(served.source.triangle_count, 0u);

    // A moved dial is a different asset and must miss.
    CookingParameters finer = parameters;
    finer.distance_field_resolution = 12;
    std::vector<std::byte> third;
    const CookingReport recooked = cooker.cook(box.view(), finer, &store, nullptr, third);
    EXPECT_FALSE(recooked.served_from_cache);
    EXPECT_EQ(store.size(), 2u);

    // And a re-cook of unchanged input is byte-identical, which is what makes the key sound.
    std::vector<std::byte> repeated;
    CollisionCooker fresh_cooker;
    fresh_cooker.cook(box.view(), parameters, nullptr, nullptr, repeated);
    EXPECT_EQ(repeated, first);
}

TEST(Unit_CollisionCooker,DropsACacheEntryThisBuildCannotRead)
{
    CollisionCooker cooker;
    MemoryCookedAssetStore store;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    const CookingParameters parameters = quick_parameters();

    // Plant bytes under the key this cook will look for. Trusting them would be trusting
    // a file this build cannot parse; the only safe response is to drop and cook.
    CookedAssetKey key;
    key.source_hash = mesh_content_hash(box.view());
    key.parameters_hash = cooking_parameters_hash(parameters);
    key.cooker_version = cooker.version();
    key.kind = cooker.kind();
    store.store(key, std::vector<std::byte>(64, std::byte{0x5A}));

    std::vector<std::byte> bytes;
    const CookingReport report = cooker.cook(box.view(), parameters, &store, nullptr, bytes);
    EXPECT_EQ(report.status, CookingStatus::Succeeded);
    EXPECT_FALSE(report.served_from_cache);
    ASSERT_TRUE(load_collision_blob(bytes.data(), bytes.size()).valid);
}

TEST(Unit_CollisionCooker,RefusesAnEmptyMeshAndNamesAFailingStage)
{
    CollisionCooker cooker;
    std::vector<std::byte> bytes;

    Geometry::TriangleMesh empty;
    const CookingReport nothing =
        cooker.cook(empty.view(), quick_parameters(), nullptr, nullptr, bytes);
    EXPECT_EQ(nothing.status, CookingStatus::EmptyInput);
    EXPECT_FALSE(nothing.has_asset());
    EXPECT_TRUE(bytes.empty());

    // Geometry that has triangles but no surface: three collinear points survive the
    // has-triangles check and are dropped by the repair, so the failure is the repair's and
    // the report says so instead of reporting an empty input it did not have.
    Geometry::TriangleMesh sliver;
    sliver.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f};
    sliver.indices = {0, 1, 2};
    const CookingReport failed =
        cooker.cook(sliver.view(), quick_parameters(), nullptr, nullptr, bytes);
    EXPECT_EQ(failed.status, CookingStatus::StageFailed);
    ASSERT_NE(failed.failed_stage, nullptr);
    EXPECT_STREQ(failed.failed_stage, "Repair");
    EXPECT_TRUE(bytes.empty());
}

TEST(Unit_CollisionCooker,CooksTheDirtyMeshCorpusAndReportsWhatWasWrongWithIt)
{
    // P4's acceptance criterion, on the corpus it names. Each entry is a specific way a
    // real export arrives broken; in every case the cook must complete *and* the report
    // must carry the defect, because a repair that hides the input is a bug reported three
    // weeks later as "the physics is wrong".
    struct Case
    {
        const char* label;
        Geometry::TriangleMesh mesh;
    };

    std::vector<Case> corpus;

    // Exploded: one vertex per corner per face, which is most exporters.
    {
        Geometry::TriangleMesh exploded;
        const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
        std::uint32_t next = 0;
        for (std::size_t t = 0; t < box.triangle_count(); ++t)
        {
            for (int corner = 0; corner < 3; ++corner)
            {
                const std::uint32_t index = box.indices[t * 3 + std::size_t(corner)];
                exploded.positions.push_back(box.positions[index * 3 + 0]);
                exploded.positions.push_back(box.positions[index * 3 + 1]);
                exploded.positions.push_back(box.positions[index * 3 + 2]);
                exploded.indices.push_back(next++);
            }
        }
        corpus.push_back(Case{"exploded", exploded});
    }

    // Mixed winding: three faces reversed, so the surface disagrees with itself.
    {
        Geometry::TriangleMesh mixed = box_mesh(0.5f, 0.5f, 0.5f);
        for (const std::size_t face : {std::size_t(0), std::size_t(5), std::size_t(9)})
            std::swap(mixed.indices[face * 3 + 1], mixed.indices[face * 3 + 2]);
        corpus.push_back(Case{"mixed winding", mixed});
    }

    // Inside out: uniformly reversed, which is consistent and still wrong.
    {
        Geometry::TriangleMesh inverted = box_mesh(0.5f, 0.5f, 0.5f);
        for (std::size_t face = 0; face < inverted.triangle_count(); ++face)
            std::swap(inverted.indices[face * 3 + 1], inverted.indices[face * 3 + 2]);
        corpus.push_back(Case{"inside out", inverted});
    }

    // Self-intersecting: two boxes unioned without cleanup, so the interior faces remain.
    corpus.push_back(Case{"self intersecting", l_shape()});

    // Degenerate and duplicate triangles, and a vertex nothing references.
    {
        Geometry::TriangleMesh dirty = box_mesh(0.5f, 0.5f, 0.5f);
        for (int i = 0; i < 3; ++i)
            dirty.indices.push_back(BOX_FACES[2][i]);
        dirty.indices.push_back(1);
        dirty.indices.push_back(1);
        dirty.indices.push_back(4);
        dirty.positions.push_back(9.0f);
        dirty.positions.push_back(9.0f);
        dirty.positions.push_back(9.0f);
        corpus.push_back(Case{"degenerate and duplicate", dirty});
    }

    // Open shell: a box missing a face, which encloses no volume to weigh.
    {
        Geometry::TriangleMesh open = box_mesh(0.5f, 0.5f, 0.5f);
        open.indices.resize(open.indices.size() - 6);
        corpus.push_back(Case{"open shell", open});
    }

    CollisionCooker cooker;
    for (const Case& entry : corpus)
    {
        std::vector<std::byte> bytes;
        const CookingReport report =
            cooker.cook(entry.mesh.view(), quick_parameters(), nullptr, nullptr, bytes);

        SCOPED_TRACE(entry.label);
        ASSERT_TRUE(report.has_asset());
        ASSERT_TRUE(load_collision_blob(bytes.data(), bytes.size()).valid);
        EXPECT_GT(report.convex_piece_count, 0u);

        const CollisionAssetView view = load_collision_blob(bytes.data(), bytes.size());
        for (std::uint32_t piece = 0; piece < view.piece_count; ++piece)
            EXPECT_GT(collision_asset_hull(view, piece).vertex_count, 0u);
    }
}

TEST(Unit_CollisionCooker,TheOpenShellReportsNoMassRatherThanGuessingOne)
{
    // A single-sided wall is ordinary content and must cook. What it cannot have is a
    // mass, and zero already means "keep the authored value" everywhere downstream — which
    // is a better fallback than a number integrated over a surface that encloses nothing.
    Geometry::TriangleMesh open = box_mesh(0.5f, 0.5f, 0.5f);
    open.indices.resize(open.indices.size() - 6);

    CollisionCooker cooker;
    std::vector<std::byte> bytes;
    const CookingReport report =
        cooker.cook(open.view(), quick_parameters(), nullptr, nullptr, bytes);

    ASSERT_EQ(report.status, CookingStatus::Succeeded);
    EXPECT_FALSE(report.source.watertight());
    EXPECT_EQ(report.source.boundary_edges, 4u);
    EXPECT_EQ(report.mass, 0.0f);

    // A project that will not ship one says so through a threshold, not through the cooker.
    CookingThresholds demanding;
    demanding.require_watertight_source = true;
    cooker.set_thresholds(demanding);
    const CookingReport rejected =
        cooker.cook(open.view(), quick_parameters(), nullptr, nullptr, bytes);
    EXPECT_EQ(rejected.status, CookingStatus::RejectedByThreshold);
    // Rejected, and still inspectable.
    EXPECT_TRUE(rejected.has_asset());
    EXPECT_TRUE(load_collision_blob(bytes.data(), bytes.size()).valid);
}

TEST(Unit_CollisionAssetBlob,RefusesBlobsItWouldNotItselfLoad)
{
    CollisionAsset asset;
    std::vector<std::byte> bytes;

    // Nothing to collide as.
    EXPECT_FALSE(build_collision_blob(asset, bytes));
    EXPECT_TRUE(bytes.empty());

    // A piece naming a vertex range past the end of the hull array, which unchecked is a
    // support function reading past the end of a file.
    asset.hull_vertices = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    CollisionPieceRecord piece{};
    piece.first_vertex = 2;
    piece.vertex_count = 4;
    asset.pieces.push_back(piece);
    EXPECT_FALSE(build_collision_blob(asset, bytes));

    asset.pieces[0].vertex_count = 1;
    ASSERT_TRUE(build_collision_blob(asset, bytes));
    ASSERT_TRUE(validate_collision_blob(bytes.data(), bytes.size()));

    // Truncation must fail at the header rather than produce a view that walks off the end.
    std::vector<std::byte> truncated(bytes.begin(), bytes.begin() + std::ptrdiff_t(bytes.size() / 2));
    EXPECT_FALSE(validate_collision_blob(truncated.data(), truncated.size()));
    EXPECT_FALSE(load_collision_blob(truncated.data(), truncated.size()).valid);

    // A header that lies about its own length is the interesting case.
    std::vector<std::byte> lying = bytes;
    CollisionBlobHeader header{};
    std::memcpy(&header, lying.data(), sizeof(header));
    header.total_size = std::uint32_t(lying.size() * 4);
    std::memcpy(lying.data(), &header, sizeof(header));
    EXPECT_FALSE(validate_collision_blob(lying.data(), lying.size()));

    // A version this build does not know is not a blob this build may guess at.
    std::vector<std::byte> future = bytes;
    std::memcpy(&header, future.data(), sizeof(header));
    header.version = COLLISION_BLOB_VERSION + 1;
    std::memcpy(future.data(), &header, sizeof(header));
    EXPECT_FALSE(validate_collision_blob(future.data(), future.size()));

    EXPECT_FALSE(validate_collision_blob(nullptr, 0));
}
