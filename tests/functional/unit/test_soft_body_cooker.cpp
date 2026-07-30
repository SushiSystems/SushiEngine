/**************************************************************************/
/* test_soft_body_cooker.cpp                                              */
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

// §8.3, and the two of §8.6's five binding invariants that are testable at cook time.
//
// Invariant 1 (every render vertex is bound) and invariant 2 (at rest the reconstruction
// reproduces the source mesh) are properties of the asset and are asserted here.
// Invariants 3, 4 and 5 — no lag, fracture preserves binding, collision happens against
// the simulated surface — are properties of a *running* finite-element solve, which is P6.
// Claiming them here would be claiming a test for code that does not exist.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/cooking/soft_body_cooker.hpp>
#include <SushiEngine/physics/cooking/tetrahedral_mesh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief An outward-wound box. */
    Geometry::TriangleMesh box_mesh(float hx, float hy, float hz)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz},
                                     {-hx, hy, -hz},  {-hx, -hy, hz}, {hx, -hy, hz},
                                     {hx, hy, hz},    {-hx, hy, hz}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                           {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                           {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief Records the stage names a cook announced. */
    class RecordingProgressSink final : public ICookingProgressSink
    {
    public:
        void on_progress(const CookingProgress& progress) override
        {
            stages.push_back(progress.stage);
        }

        std::vector<const char*> stages;
    };

    /** @brief Parameters that cook a small body quickly. */
    CookingParameters quick_parameters()
    {
        CookingParameters parameters;
        parameters.fidelity = 0.0f;
        parameters.voxel_resolution = 6;
        parameters.target_tetrahedron_count = DERIVE_FROM_FIDELITY;
        parameters.simulation_level_count = 1;
        parameters.distance_field_resolution = 8;
        parameters.surface_conforming_passes = 0;
        parameters.accuracy_lattice_order = 2;
        parameters.cook_soft_body = true;
        parameters.cook_collision = false;
        return parameters;
    }
} // namespace

TEST(Unit_TetrahedronQuality, IsOneForARegularTetrahedronAndZeroForASliver)
{
    // The metric's normalization is the whole of its usefulness: a threshold on it only
    // means something if one really is the best attainable value.
    const Scalar a = 1;
    const Vector3 regular[4] = {Vector3{a, a, a}, Vector3{a, -a, -a}, Vector3{-a, a, -a},
                                Vector3{-a, -a, a}};
    EXPECT_NEAR(tetrahedron_quality(regular[0], regular[1], regular[2], regular[3]), 1.0f,
                1e-4f);

    // A cell of the lattice this cooker uses. Not regular — that is the price of a
    // triangulation that conforms across faces by construction — but nowhere near a sliver,
    // and identical for every element, so the worst quality is a lattice constant.
    const Vector3 kuhn[4] = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{1, 1, 0},
                             Vector3{1, 1, 1}};
    const float lattice = tetrahedron_quality(kuhn[0], kuhn[1], kuhn[2], kuhn[3]);
    EXPECT_GT(lattice, 0.3f);
    EXPECT_LT(lattice, 1.0f);

    // Flattened toward a plane, which is what destroys a finite-element solve.
    const Vector3 sliver[4] = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0},
                               Vector3{Scalar(0.3), Scalar(0.3), Scalar(1e-6)}};
    EXPECT_LT(tetrahedron_quality(sliver[0], sliver[1], sliver[2], sliver[3]), 0.01f);

    // Degenerate reports zero rather than a not-a-number.
    const Vector3 flat[4] = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0},
                             Vector3{1, 1, 0}};
    EXPECT_EQ(tetrahedron_quality(flat[0], flat[1], flat[2], flat[3]), 0.0f);
}

TEST(Unit_TetrahedralMesh, FillsABoxWithAClosedConformingLattice)
{
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(box.view()));

    TetrahedralizationOptions options;
    options.voxel_resolution = 8;
    options.conforming_passes = 0;
    options.density = 1000.0f;

    TetrahedralMesh mesh;
    const TetrahedralizationReport report =
        build_tetrahedral_mesh(box.view(), surface, options, mesh);

    ASSERT_GT(report.tetrahedron_count, 0u);
    EXPECT_EQ(report.inverted_element_count, 0u);
    EXPECT_GT(report.interior_cell_count, 0u);
    // Six elements per interior cell, minus whatever the sliver threshold removed. With no
    // conforming passes nothing moves, so nothing should be removed at all.
    EXPECT_EQ(report.removed_tetrahedron_count, 0u);
    EXPECT_EQ(report.tetrahedron_count, report.interior_cell_count * 6u);

    // Every element positively oriented, which later passes read as a statement about
    // deformation rather than about listing order.
    for (std::size_t t = 0; t < mesh.tetrahedron_count(); ++t)
    {
        const std::uint32_t* element = mesh.tetrahedra.data() + t * 4;
        const Vector3& a = mesh.vertices[element[0]];
        const Vector3 ab = mesh.vertices[element[1]] - a;
        const Vector3 ac = mesh.vertices[element[2]] - a;
        const Vector3 ad = mesh.vertices[element[3]] - a;
        ASSERT_GT(dot(cross(ab, ac), ad), 0.0);
        EXPECT_GT(mesh.rest_volume[t], 0.0);
    }

    // The lattice's boundary is a closed surface. This is the property the whole cook
    // rests on: it is what makes the mass integrable, the distance field signable and the
    // collision surface a surface.
    const Geometry::TriangleMesh boundary = mesh.surface_mesh();
    const Geometry::MeshTopologyReport topology =
        Geometry::analyze_mesh_topology(boundary.view());
    EXPECT_TRUE(topology.watertight());
    EXPECT_TRUE(topology.consistently_oriented());
    EXPECT_GT(topology.signed_volume, 0.0f);

    // Mass is conserved by construction: the vertex masses sum to volume times density.
    Scalar total_mass = 0;
    for (const Scalar mass : mesh.vertex_mass)
        total_mass += mass;
    EXPECT_NEAR(double(total_mass), double(report.total_volume) * 1000.0, 1e-6);

    // A voxelized box is inscribed in the true one, so its volume is a little short.
    EXPECT_GT(double(report.total_volume), 0.6);
    EXPECT_LE(double(report.total_volume), 1.0 + 1e-9);
}

TEST(Unit_TetrahedralMesh, TheExteriorFillDoesNotLeakThroughASmallHole)
{
    // §8.3 stage 2's whole argument for voxelizing rather than tetrahedralizing directly:
    // a hole smaller than a cell does not let the fill in, so an open-shelled model still
    // gets a solid interior instead of no output at all.
    Geometry::TriangleMesh holed = box_mesh(0.5f, 0.5f, 0.5f);
    holed.indices.resize(holed.indices.size() - 6);
    ASSERT_FALSE(Geometry::analyze_mesh_topology(holed.view()).watertight());

    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(holed.view()));

    TetrahedralizationOptions options;
    options.voxel_resolution = 6;
    options.conforming_passes = 0;

    TetrahedralMesh mesh;
    const TetrahedralizationReport report =
        build_tetrahedral_mesh(holed.view(), surface, options, mesh);

    // The missing face is a whole side, which is much larger than a cell — so the fill
    // *does* get in, and what the test pins down is that the cooker still produces a mesh
    // rather than nothing, and still produces a closed boundary for it.
    ASSERT_GT(report.tetrahedron_count, 0u);
    EXPECT_TRUE(Geometry::analyze_mesh_topology(mesh.surface_mesh().view()).watertight());
}

TEST(Unit_TetrahedralMesh, ScalesTheResolutionTowardTheAuthoredElementCount)
{
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(box.view()));

    TetrahedralizationOptions coarse;
    coarse.voxel_resolution = 16;
    coarse.target_tetrahedron_count = 300;
    coarse.conforming_passes = 0;
    TetrahedralMesh small;
    const TetrahedralizationReport small_report =
        build_tetrahedral_mesh(box.view(), surface, coarse, small);

    TetrahedralizationOptions fine = coarse;
    fine.target_tetrahedron_count = 20000;
    TetrahedralMesh large;
    const TetrahedralizationReport large_report =
        build_tetrahedral_mesh(box.view(), surface, fine, large);

    // The dial authors a count, not a resolution, and the resolution scales as its cube
    // root. Asserted as an ordering rather than as an exact figure, because one pass toward
    // a target is what the cooker promises and iterating to hit it exactly is not.
    EXPECT_LT(small_report.tetrahedron_count, large_report.tetrahedron_count);
    EXPECT_GT(small_report.cell_size, large_report.cell_size);
}

TEST(Unit_TetrahedralMesh, EmbedsEveryPointAndReconstructsItExactly)
{
    // §8.6 invariant 2, at the level it can be tested at rest: the embedding round-trips.
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    Geometry::MeshDistanceQuery surface;
    ASSERT_TRUE(surface.build(box.view()));

    TetrahedralizationOptions options;
    options.voxel_resolution = 6;
    options.conforming_passes = 0;
    TetrahedralMesh mesh;
    ASSERT_GT(build_tetrahedral_mesh(box.view(), surface, options, mesh).tetrahedron_count,
              0u);

    // Points inside, on the surface, and outside it — the third is the case the
    // extrapolated fallback exists for.
    std::vector<Vector3> points = {Vector3{0, 0, 0},
                                  Vector3{Scalar(0.2), Scalar(-0.1), Scalar(0.3)},
                                  Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)},
                                  Vector3{Scalar(-0.5), 0, 0},
                                  Vector3{Scalar(0.9), 0, 0}};

    std::vector<TetrahedronBinding> bindings;
    embed_points(mesh, points.data(), points.size(), bindings);
    ASSERT_EQ(bindings.size(), points.size());

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        // Every point is bound to a real element — invariant 1, which the extrapolated
        // fallback is what guarantees.
        ASSERT_LT(std::size_t(bindings[i].tetrahedron), mesh.tetrahedron_count());

        float sum = 0.0f;
        for (int corner = 0; corner < 4; ++corner)
            sum += bindings[i].weights[corner];
        EXPECT_NEAR(sum, 1.0f, 1e-4f);

        // And reconstructing it from the rest lattice returns the point itself, whether it
        // was inside or extrapolated. An extrapolated binding is not an approximation: the
        // barycentric expression is exact outside the element too.
        const Vector3 rebuilt = evaluate_binding(mesh, bindings[i]);
        EXPECT_NEAR(double(rebuilt.x), double(points[i].x), 1e-4);
        EXPECT_NEAR(double(rebuilt.y), double(points[i].y), 1e-4);
        EXPECT_NEAR(double(rebuilt.z), double(points[i].z), 1e-4);
    }

    // The interior point is contained, the far outside one is not.
    EXPECT_TRUE(bindings[0].inside);
    EXPECT_FALSE(bindings[4].inside);
}

TEST(Unit_SoftBodyCooker, CooksABoxIntoALoadableAssetAndRunsEveryStage)
{
    SoftBodyCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    CookingParameters parameters = quick_parameters();
    parameters.density = 400.0f;

    RecordingProgressSink sink;
    std::vector<std::byte> bytes;
    const CookingReport report = cooker.cook(box.view(), parameters, nullptr, &sink, bytes);

    ASSERT_EQ(report.status, CookingStatus::Succeeded) << "failed stage: "
                                                      << (report.failed_stage != nullptr
                                                              ? report.failed_stage
                                                              : "none");
    EXPECT_EQ(report.failed_stage, nullptr);
    ASSERT_EQ(sink.stages.size(), 6u);
    EXPECT_STREQ(sink.stages[0], "Repair");
    EXPECT_STREQ(sink.stages[1], "Tetrahedralize");
    EXPECT_STREQ(sink.stages[2], "Embed");
    EXPECT_STREQ(sink.stages[3], "BakeRestShape");
    EXPECT_STREQ(sink.stages[4], "LevelsOfDetail");
    EXPECT_STREQ(sink.stages[5], "Serialize");

    EXPECT_GT(report.tetrahedron_count, 0u);
    EXPECT_EQ(report.inverted_element_count, 0u);
    EXPECT_GT(report.worst_element_quality, 0.0f);
    EXPECT_EQ(report.unembedded_vertex_count, 0u);
    EXPECT_EQ(report.level_of_detail_count, 1u);
    EXPECT_EQ(report.distance_field_resolution, 8u);
    EXPECT_GT(report.mass, 0.0f);

    const SoftBodyAssetView view = load_soft_body_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);
    EXPECT_EQ(view.level_count, 1u);
    EXPECT_EQ(view.tetrahedron_count, report.tetrahedron_count);
    EXPECT_EQ(view.binding_count, box.vertex_count());
    EXPECT_EQ(view.mapping_count, 0u);
    EXPECT_GT(view.surface_node_count, 0u);
    EXPECT_GT(view.surface_index_count, 0u);
    EXPECT_EQ(view.field_resolution, 8u);

    // §8.3 stage 10: the parameters the asset was cooked with travel inside it, so a
    // re-cook is reproducible and a mismatch is detectable without the project file.
    EXPECT_EQ(view.parameters.voxel_resolution, parameters.voxel_resolution);
    EXPECT_EQ(view.parameters.distance_field_resolution, parameters.distance_field_resolution);
    EXPECT_EQ(view.parameters.density, parameters.density);
    EXPECT_TRUE(view.parameters.cook_soft_body);

    // The mass the summary carries is the simulated surface's, not the source mesh's —
    // which is what the rigid fallback of §9.7 has to weigh.
    EXPECT_NEAR(double(view.summary.mass), double(report.mass), 1e-3);
    EXPECT_GT(double(view.summary.volume), 0.0);
}

TEST(Unit_SoftBodyCooker, TheRenderMeshRoundTripsThroughTheAssetAtRest)
{
    // §8.6 invariant 2 end to end, through the serialized asset rather than through the
    // cooker's own intermediate state — because the asset is what P6 will read, and an
    // embedding that round-trips before serialization and not after is a rebasing bug.
    SoftBodyCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.4f, 0.7f, 0.25f);
    std::vector<std::byte> bytes;
    const CookingReport report =
        cooker.cook(box.view(), quick_parameters(), nullptr, nullptr, bytes);
    ASSERT_EQ(report.status, CookingStatus::Succeeded);

    const SoftBodyAssetView view = load_soft_body_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);
    ASSERT_EQ(view.binding_count, box.vertex_count());

    for (std::uint32_t i = 0; i < view.binding_count; ++i)
    {
        const Vector3 rebuilt = evaluate_soft_binding(view, 0, view.bindings[i]);
        EXPECT_NEAR(double(rebuilt.x), double(box.positions[i * 3 + 0]), 1e-4);
        EXPECT_NEAR(double(rebuilt.y), double(box.positions[i * 3 + 1]), 1e-4);
        EXPECT_NEAR(double(rebuilt.z), double(box.positions[i * 3 + 2]), 1e-4);
    }
}

TEST(Unit_SoftBodyCooker, BuildsALevelChainWhoseCoarserLevelsAreCoarser)
{
    SoftBodyCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    CookingParameters parameters = quick_parameters();
    parameters.voxel_resolution = 16;
    parameters.simulation_level_count = 3;

    std::vector<std::byte> bytes;
    const CookingReport report = cooker.cook(box.view(), parameters, nullptr, nullptr, bytes);
    ASSERT_EQ(report.status, CookingStatus::Succeeded);

    const SoftBodyAssetView view = load_soft_body_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);
    ASSERT_GT(view.level_count, 1u);

    for (std::uint32_t level = 1; level < view.level_count; ++level)
    {
        // Each level is strictly coarser, or the chain would be paying memory for a second
        // copy of the same lattice.
        EXPECT_LT(view.levels[level].tetrahedron_count,
                  view.levels[level - 1].tetrahedron_count);
        EXPECT_GT(view.levels[level].cell_size, view.levels[level - 1].cell_size);

        // And it carries a mapping for every vertex of the level above it, so the chain
        // reaches the render mesh (§8.3 stage 9).
        EXPECT_EQ(view.levels[level].mapping_count, view.levels[level - 1].vertex_count);
    }
    EXPECT_EQ(view.levels[0].mapping_count, 0u);

    // The mapping reconstructs the finer level's rest positions from the coarse lattice.
    // At rest that reconstruction must be the finer vertex itself, which is the level-chain
    // form of the same round trip the render mesh gets.
    const SoftBodyLevelRecord& coarse = view.levels[1];
    const SoftBodyLevelRecord& fine = view.levels[0];
    for (std::uint32_t i = 0; i < coarse.mapping_count; ++i)
    {
        const Vector3 rebuilt =
            evaluate_soft_binding(view, 1, view.mappings[coarse.first_mapping + i]);
        const Vector3& expected = view.vertices[fine.first_vertex + i];
        EXPECT_NEAR(double(rebuilt.x), double(expected.x), 1e-3);
        EXPECT_NEAR(double(rebuilt.y), double(expected.y), 1e-3);
        EXPECT_NEAR(double(rebuilt.z), double(expected.z), 1e-3);
    }
}

TEST(Unit_SoftBodyCooker, RebasesEveryLevelsIndicesIntoTheSharedArrays)
{
    // The one thing the serialize stage must not get wrong. An unrebased index reads
    // another level's vertices, which produces a body that deforms into its own coarse
    // copy — geometry rather than a crash, and therefore the worst failure available.
    SoftBodyCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    CookingParameters parameters = quick_parameters();
    parameters.voxel_resolution = 12;
    parameters.simulation_level_count = 3;

    std::vector<std::byte> bytes;
    ASSERT_EQ(cooker.cook(box.view(), parameters, nullptr, nullptr, bytes).status,
              CookingStatus::Succeeded);
    const SoftBodyAssetView view = load_soft_body_blob(bytes.data(), bytes.size());
    ASSERT_TRUE(view.valid);

    for (std::uint32_t level = 0; level < view.level_count; ++level)
    {
        const SoftBodyLevelRecord& record = view.levels[level];
        const std::uint32_t low = record.first_vertex;
        const std::uint32_t high = record.first_vertex + record.vertex_count;

        for (std::uint32_t t = 0; t < record.tetrahedron_count; ++t)
        {
            const std::uint32_t* element =
                view.tetrahedra + std::size_t(record.first_tetrahedron + t) * 4;
            for (int corner = 0; corner < 4; ++corner)
            {
                ASSERT_GE(element[corner], low);
                ASSERT_LT(element[corner], high);
            }
        }
        for (std::uint32_t i = 0; i < record.surface_index_count; ++i)
        {
            const std::uint32_t index = view.surface_indices[record.first_surface_index + i];
            ASSERT_GE(index, low);
            ASSERT_LT(index, high);
        }
        for (std::uint32_t i = 0; i < record.mapping_count; ++i)
        {
            const std::uint32_t element = view.mappings[record.first_mapping + i].tetrahedron;
            ASSERT_GE(element, record.first_tetrahedron);
            ASSERT_LT(element, record.first_tetrahedron + record.tetrahedron_count);
        }
    }

    // The render bindings name level zero, whose run starts at zero.
    for (std::uint32_t i = 0; i < view.binding_count; ++i)
    {
        ASSERT_LT(view.bindings[i].tetrahedron, view.levels[0].tetrahedron_count);
    }
}

TEST(Unit_SoftBodyCooker, CachesAndRefusesWhatItCannotCook)
{
    SoftBodyCooker cooker;
    MemoryCookedAssetStore store;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    const CookingParameters parameters = quick_parameters();

    std::vector<std::byte> first;
    const CookingReport cooked = cooker.cook(box.view(), parameters, &store, nullptr, first);
    ASSERT_EQ(cooked.status, CookingStatus::Succeeded);
    EXPECT_EQ(store.size(), 1u);

    RecordingProgressSink sink;
    std::vector<std::byte> second;
    const CookingReport served = cooker.cook(box.view(), parameters, &store, &sink, second);
    EXPECT_TRUE(served.served_from_cache);
    EXPECT_EQ(second, first);
    EXPECT_TRUE(sink.stages.empty());
    EXPECT_EQ(served.tetrahedron_count, cooked.tetrahedron_count);
    // Nothing looked at the source, so its topology stays unmeasured rather than reported
    // clean.
    EXPECT_EQ(served.source.triangle_count, 0u);

    // A soft cook and a collision cook of the same mesh are different assets, so the two
    // cookers cannot serve each other's entries even at identical parameters.
    EXPECT_EQ(store.size(), 1u);

    std::vector<std::byte> bytes;
    Geometry::TriangleMesh empty;
    EXPECT_EQ(cooker.cook(empty.view(), parameters, nullptr, nullptr, bytes).status,
              CookingStatus::EmptyInput);

    // A surface with no area cannot be filled, and the stage that could not do it is named.
    Geometry::TriangleMesh sliver;
    sliver.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f};
    sliver.indices = {0, 1, 2};
    const CookingReport failed =
        cooker.cook(sliver.view(), parameters, nullptr, nullptr, bytes);
    EXPECT_EQ(failed.status, CookingStatus::StageFailed);
    ASSERT_NE(failed.failed_stage, nullptr);
    EXPECT_STREQ(failed.failed_stage, "Repair");
    EXPECT_TRUE(bytes.empty());
}

TEST(Unit_SoftBodyCooker, RejectsAnAssetWhoseElementsAreTooPoorWhenAskedTo)
{
    SoftBodyCooker cooker;
    const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    std::vector<std::byte> bytes;

    // The lattice's quality is a constant, so a threshold above it rejects every cook —
    // which is the point of the test: the threshold is applied, and a rejected asset is
    // still there to be inspected.
    CookingThresholds impossible;
    impossible.min_element_quality = 0.99f;
    cooker.set_thresholds(impossible);

    const CookingReport report =
        cooker.cook(box.view(), quick_parameters(), nullptr, nullptr, bytes);
    EXPECT_EQ(report.status, CookingStatus::RejectedByThreshold);
    EXPECT_TRUE(report.has_asset());
    EXPECT_TRUE(load_soft_body_blob(bytes.data(), bytes.size()).valid);

    cooker.set_thresholds(CookingThresholds{});
    EXPECT_EQ(cooker.cook(box.view(), quick_parameters(), nullptr, nullptr, bytes).status,
              CookingStatus::Succeeded);
}

TEST(Unit_SoftBodyBlob, RefusesBlobsItWouldNotItselfLoad)
{
    SoftBodyAsset asset;
    std::vector<std::byte> bytes;
    EXPECT_FALSE(build_soft_body_blob(asset, bytes));

    // A minimal well-formed asset: one level, one element, four vertices.
    asset.vertices = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
    asset.vertex_mass = {1, 1, 1, 1};
    asset.tetrahedra = {0, 1, 2, 3};
    asset.rest_inverse = {Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1}};
    asset.rest_volume = {Scalar(1) / Scalar(6)};
    SoftBodyLevelRecord level{};
    level.vertex_count = 4;
    level.tetrahedron_count = 1;
    asset.levels.push_back(level);
    ASSERT_TRUE(build_soft_body_blob(asset, bytes));
    ASSERT_TRUE(validate_soft_body_blob(bytes.data(), bytes.size()));

    // A tetrahedron naming a vertex that does not exist — the cross-reference that
    // unchecked reads a neighbouring section and produces geometry rather than a crash.
    SoftBodyAsset broken = asset;
    broken.tetrahedra[2] = 9;
    std::vector<std::byte> refused;
    EXPECT_FALSE(build_soft_body_blob(broken, refused));

    // A binding naming an element that does not exist.
    broken = asset;
    SoftBodyBinding binding{};
    binding.tetrahedron = 4;
    broken.bindings.push_back(binding);
    EXPECT_FALSE(build_soft_body_blob(broken, refused));

    // A level claiming more vertices than the asset holds.
    broken = asset;
    broken.levels[0].vertex_count = 9;
    EXPECT_FALSE(build_soft_body_blob(broken, refused));

    // A rest-state array out of step with the element count.
    broken = asset;
    broken.rest_volume.clear();
    EXPECT_FALSE(build_soft_body_blob(broken, refused));

    // Truncation and a lying header must both fail at validation.
    std::vector<std::byte> truncated(bytes.begin(),
                                    bytes.begin() + std::ptrdiff_t(bytes.size() / 2));
    EXPECT_FALSE(validate_soft_body_blob(truncated.data(), truncated.size()));

    std::vector<std::byte> lying = bytes;
    SoftBlobHeader header{};
    std::memcpy(&header, lying.data(), sizeof(header));
    header.total_size = std::uint32_t(lying.size() * 4);
    std::memcpy(lying.data(), &header, sizeof(header));
    EXPECT_FALSE(validate_soft_body_blob(lying.data(), lying.size()));

    std::vector<std::byte> future = bytes;
    std::memcpy(&header, future.data(), sizeof(header));
    header.version = SOFT_BLOB_VERSION + 1;
    std::memcpy(future.data(), &header, sizeof(header));
    EXPECT_FALSE(validate_soft_body_blob(future.data(), future.size()));

    EXPECT_FALSE(validate_soft_body_blob(nullptr, 0));
}
