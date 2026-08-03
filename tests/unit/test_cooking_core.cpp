/**************************************************************************/
/* test_cooking_core.cpp                                                  */
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

// The cooking pipeline's contracts, as distinct from its geometry. Three of them are
// asserted here because each is a claim the rest of the pipeline is built on and none
// of the three is visible in an asset: that the fidelity dial resolves to §8.2's
// documented numbers, that the cache key changes exactly when the produced asset
// would, and that a store hands back the bytes it was given.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/cooked_asset_store.hpp>
#include <SushiEngine/physics/cooking/cooker_interface.hpp>
#include <SushiEngine/physics/cooking/cooking_parameters.hpp>
#include <SushiEngine/physics/cooking/cooking_report.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief A tetrahedron, when a test needs geometry rather than a specific shape. */
    Geometry::TriangleMesh tetrahedron()
    {
        Geometry::TriangleMesh mesh;
        mesh.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                          0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
        mesh.indices = {0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
        return mesh;
    }

    /** @brief Bytes that are recognisable when they come back out of a store. */
    std::vector<std::byte> sample_blob(unsigned char first)
    {
        std::vector<std::byte> bytes;
        for (int i = 0; i < 64; ++i)
            bytes.push_back(std::byte(static_cast<unsigned char>(first + i)));
        return bytes;
    }

    /** @brief A key that differs from its neighbours in exactly one component. */
    CookedAssetKey make_key(std::uint64_t source, std::uint64_t parameters,
                            std::uint32_t version, CookedAssetKind kind)
    {
        CookedAssetKey key;
        key.source_hash = source;
        key.parameters_hash = parameters;
        key.cooker_version = version;
        key.kind = kind;
        return key;
    }
} // namespace

TEST(Unit_CookingParameters,ResolvesTheDialsEndpointsToTheDocumentedTable)
{
    // §8.2 verbatim. This test exists because the document and the cooker disagreeing
    // about what fidelity means is a bug nobody can see: both halves look right alone.
    CookingParameters lowest;
    lowest.fidelity = 0.0f;
    const DerivedCookingParameters fast = resolve_cooking_parameters(lowest);
    EXPECT_EQ(fast.voxel_resolution, 16);
    EXPECT_EQ(fast.target_tetrahedron_count, 200);
    EXPECT_EQ(fast.simulation_level_count, 1);
    EXPECT_EQ(fast.convex_piece_count, 4);
    EXPECT_EQ(fast.distance_field_resolution, 16);
    EXPECT_EQ(fast.surface_conforming_passes, 0);
    EXPECT_EQ(fast.suggested_substep_count, 8);

    CookingParameters highest;
    highest.fidelity = 1.0f;
    const DerivedCookingParameters accurate = resolve_cooking_parameters(highest);
    EXPECT_EQ(accurate.voxel_resolution, 256);
    EXPECT_EQ(accurate.target_tetrahedron_count, 120000);
    EXPECT_EQ(accurate.simulation_level_count, 4);
    EXPECT_EQ(accurate.convex_piece_count, 64);
    EXPECT_EQ(accurate.distance_field_resolution, 128);
    EXPECT_EQ(accurate.surface_conforming_passes, 3);
    EXPECT_EQ(accurate.suggested_substep_count, 32);
}

TEST(Unit_CookingParameters,InterpolatesTheResolutionsGeometrically)
{
    // Half the dial is the geometric mean, not the arithmetic one. The distinction is
    // the whole reason an artist can learn what 0.5 means: cost grows with the cube of
    // a resolution, so a linear dial spends its first half buying nothing.
    CookingParameters middle;
    middle.fidelity = 0.5f;
    const DerivedCookingParameters derived = resolve_cooking_parameters(middle);

    EXPECT_EQ(derived.voxel_resolution, 64);            // 16 * sqrt(16), not 136
    EXPECT_EQ(derived.convex_piece_count, 16);          // 4 * sqrt(16), not 34
    EXPECT_EQ(derived.suggested_substep_count, 16);     // 8 * sqrt(4), not 20
    EXPECT_EQ(derived.distance_field_resolution, 45);   // 16 * sqrt(8)

    // The tetrahedron budget's endpoints span six hundred times, so its midpoint is
    // asserted as a range: the exact integer depends on the last bit of a float power.
    EXPECT_GT(derived.target_tetrahedron_count, 4800);
    EXPECT_LT(derived.target_tetrahedron_count, 5000);

    // The small counts interpolate linearly, because 1 to 4 has no factor worth keeping.
    EXPECT_EQ(derived.simulation_level_count, 3);
    EXPECT_EQ(derived.surface_conforming_passes, 2);
}

TEST(Unit_CookingParameters,IsMonotonicInTheDialAndClampsOutsideIt)
{
    std::int32_t previous_voxels = 0;
    std::int32_t previous_pieces = 0;
    for (int step = 0; step <= 20; ++step)
    {
        CookingParameters parameters;
        parameters.fidelity = float(step) / 20.0f;
        const DerivedCookingParameters derived = resolve_cooking_parameters(parameters);
        EXPECT_GE(derived.voxel_resolution, previous_voxels);
        EXPECT_GE(derived.convex_piece_count, previous_pieces);
        previous_voxels = derived.voxel_resolution;
        previous_pieces = derived.convex_piece_count;
    }

    // A slider that reads 1.0000001 must cook at maximum fidelity, not refuse.
    CookingParameters over;
    over.fidelity = 4.0f;
    CookingParameters under;
    under.fidelity = -1.0f;
    EXPECT_EQ(resolve_cooking_parameters(over).voxel_resolution, 256);
    EXPECT_EQ(resolve_cooking_parameters(under).voxel_resolution, 16);
}

TEST(Unit_CookingParameters,PinsAnOverriddenFieldAndLeavesTheRestOnTheDial)
{
    CookingParameters parameters;
    parameters.fidelity = 0.0f;
    parameters.voxel_resolution = 96;
    // Zero is a legal value for this field, which is why the sentinel is not zero.
    parameters.surface_conforming_passes = 0;

    const DerivedCookingParameters derived = resolve_cooking_parameters(parameters);
    EXPECT_EQ(derived.voxel_resolution, 96);
    EXPECT_EQ(derived.surface_conforming_passes, 0);
    EXPECT_EQ(derived.convex_piece_count, 4);
}

TEST(Unit_CookingParameters,HashesTheResolvedNumbersRatherThanTheDial)
{
    // Two cooks that resolve to the same numbers produce byte-identical assets, so they
    // must share a cache entry even when their dials read differently. Otherwise
    // dragging a slider re-cooks at every pixel it passes through.
    CookingParameters low;
    CookingParameters high;
    low.fidelity = 0.1f;
    high.fidelity = 0.9f;
    for (CookingParameters* parameters : {&low, &high})
    {
        parameters->voxel_resolution = 32;
        parameters->target_tetrahedron_count = 1000;
        parameters->simulation_level_count = 2;
        parameters->convex_piece_count = 8;
        parameters->distance_field_resolution = 32;
        parameters->surface_conforming_passes = 1;
        parameters->suggested_substep_count = 12;
    }
    EXPECT_EQ(cooking_parameters_hash(low), cooking_parameters_hash(high));

    // And it changes when anything that reaches the asset changes.
    CookingParameters moved = low;
    moved.convex_piece_count = 9;
    EXPECT_NE(cooking_parameters_hash(low), cooking_parameters_hash(moved));

    CookingParameters denser = low;
    denser.density = low.density * 2.0f;
    EXPECT_NE(cooking_parameters_hash(low), cooking_parameters_hash(denser));

    // Which cookers ran is part of the key: the same mesh asked for a collider and for
    // a soft body is two different outputs, not one served twice.
    CookingParameters soft = low;
    soft.cook_soft_body = !low.cook_soft_body;
    EXPECT_NE(cooking_parameters_hash(low), cooking_parameters_hash(soft));

    CookingParameters as_static = low;
    as_static.static_geometry = !low.static_geometry;
    EXPECT_NE(cooking_parameters_hash(low), cooking_parameters_hash(as_static));
}

TEST(Unit_CookingParameters,HashesAMeshsGeometryAndNotItsMemoryLayout)
{
    const Geometry::TriangleMesh mesh = tetrahedron();
    const std::uint64_t packed = mesh_content_hash(mesh.view());

    // The same geometry inside a wider vertex struct must hash identically, or the cache
    // misses every time the import path changes the shape of what it hands over.
    struct FatVertex
    {
        float position[3];
        float padding[12];
    };
    std::vector<FatVertex> fat(mesh.vertex_count());
    for (std::size_t i = 0; i < mesh.vertex_count(); ++i)
    {
        for (int axis = 0; axis < 3; ++axis)
            fat[i].position[axis] = mesh.positions[i * 3 + std::size_t(axis)];
        // Filled with something non-zero, so a hash that read the whole vertex struct
        // rather than the stride would notice.
        for (int slot = 0; slot < 12; ++slot)
            fat[i].padding[slot] = float(slot) * 3.5f;
    }
    Geometry::TriangleMeshView strided;
    strided.positions = fat[0].position;
    strided.position_stride = sizeof(FatVertex);
    strided.vertex_count = fat.size();
    strided.indices = mesh.indices.data();
    strided.index_count = mesh.indices.size();
    EXPECT_EQ(mesh_content_hash(strided), packed);

    // A moved vertex, a changed index, and a reordered index buffer are all different
    // geometry and must all miss.
    Geometry::TriangleMesh moved = mesh;
    moved.positions[0] += 1.0e-4f;
    EXPECT_NE(mesh_content_hash(moved.view()), packed);

    Geometry::TriangleMesh rewound = mesh;
    std::swap(rewound.indices[1], rewound.indices[2]);
    EXPECT_NE(mesh_content_hash(rewound.view()), packed);

    // Negative zero and positive zero are the same vertex; two exporters must not
    // produce two cache entries for one mesh.
    Geometry::TriangleMesh negative_zero = mesh;
    negative_zero.positions[0] = -0.0f;
    EXPECT_EQ(mesh_content_hash(negative_zero.view()), packed);

    // An empty mesh hashes, rather than reading through a null pointer.
    Geometry::TriangleMesh empty;
    EXPECT_NE(mesh_content_hash(empty.view()), packed);
}

TEST(Unit_CookedAssetKey,DistinguishesEveryComponentOfTheKey)
{
    const CookedAssetKey base = make_key(1, 2, 3, CookedAssetKind::Collision);
    EXPECT_EQ(base, make_key(1, 2, 3, CookedAssetKind::Collision));

    // Each of the four in turn, because a key that folds one of them away is a cache
    // that serves the wrong asset — and the cooker version is the one a pipeline
    // usually forgets, whose symptom is a fix that never reaches assets cooked before it.
    EXPECT_NE(base, make_key(9, 2, 3, CookedAssetKind::Collision));
    EXPECT_NE(base, make_key(1, 9, 3, CookedAssetKind::Collision));
    EXPECT_NE(base, make_key(1, 2, 9, CookedAssetKind::Collision));
    EXPECT_NE(base, make_key(1, 2, 3, CookedAssetKind::SoftBody));

    EXPECT_NE(cooked_asset_key_hash(base),
              cooked_asset_key_hash(make_key(1, 2, 9, CookedAssetKind::Collision)));
    EXPECT_NE(cooked_asset_key_hash(base),
              cooked_asset_key_hash(make_key(1, 2, 3, CookedAssetKind::SoftBody)));
    EXPECT_EQ(cooked_asset_key_hash(base),
              cooked_asset_key_hash(make_key(1, 2, 3, CookedAssetKind::Collision)));
}

TEST(Unit_MemoryCookedAssetStore,RoundTripsAndForgetsOnDemand)
{
    MemoryCookedAssetStore store;
    const CookedAssetKey key = make_key(11, 22, 1, CookedAssetKind::Collision);
    const std::vector<std::byte> blob = sample_blob(7);

    std::vector<std::byte> loaded;
    EXPECT_FALSE(store.contains(key));
    EXPECT_FALSE(store.load(key, loaded));
    EXPECT_TRUE(loaded.empty());

    ASSERT_TRUE(store.store(key, blob));
    EXPECT_TRUE(store.contains(key));
    ASSERT_TRUE(store.load(key, loaded));
    EXPECT_EQ(loaded, blob);
    EXPECT_EQ(store.size(), 1u);

    // A bumped cooker version is a miss, which is the mechanism that stops a fixed
    // cooker from serving assets produced by the broken one.
    const CookedAssetKey newer = make_key(11, 22, 2, CookedAssetKind::Collision);
    EXPECT_FALSE(store.contains(newer));

    // Re-cook: the button exists to get past an entry whose key has not changed.
    EXPECT_TRUE(store.evict(key));
    EXPECT_FALSE(store.contains(key));
    EXPECT_FALSE(store.evict(key));
}

TEST(Unit_FilesystemCookedAssetStore,RoundTripsThroughRealFilesAndSurvivesReopening)
{
    const std::filesystem::path root =
        std::filesystem::temp_directory_path() / "sushiengine_cook_cache_test";
    std::error_code error;
    std::filesystem::remove_all(root, error);

    const CookedAssetKey key = make_key(101, 202, 4, CookedAssetKind::SoftBody);
    const std::vector<std::byte> blob = sample_blob(31);

    {
        FilesystemCookedAssetStore store(root.string());
        ASSERT_TRUE(store.usable());
        EXPECT_FALSE(store.contains(key));
        ASSERT_TRUE(store.store(key, blob));
        EXPECT_TRUE(store.contains(key));

        // The family is in the extension, so a cache directory is readable by a human
        // trying to work out what is in it.
        const std::string path = store.path_for(key);
        EXPECT_NE(path.find(".sushisoft"), std::string::npos);
        EXPECT_TRUE(std::filesystem::is_regular_file(std::filesystem::path(path), error));

        // Nothing partial is left behind: a half-written blob that validates its own
        // header is the worst kind of cache entry, because it loads.
        EXPECT_FALSE(std::filesystem::is_regular_file(
            std::filesystem::path(path + ".partial"), error));
    }

    // A second store over the same directory is the point of a filesystem cache: the
    // cook survives the process that produced it.
    {
        FilesystemCookedAssetStore reopened(root.string());
        ASSERT_TRUE(reopened.usable());
        std::vector<std::byte> loaded;
        ASSERT_TRUE(reopened.load(key, loaded));
        EXPECT_EQ(loaded, blob);

        // Overwriting an existing entry must succeed rather than fail on a rename onto
        // a file that is already there.
        const std::vector<std::byte> second = sample_blob(77);
        ASSERT_TRUE(reopened.store(key, second));
        ASSERT_TRUE(reopened.load(key, loaded));
        EXPECT_EQ(loaded, second);

        EXPECT_TRUE(reopened.evict(key));
        EXPECT_FALSE(reopened.contains(key));
    }

    std::filesystem::remove_all(root, error);
}

TEST(Unit_FilesystemCookedAssetStore,DegradesToNoCacheRatherThanFailingAnImport)
{
    // An unwritable cache directory must make cooking slow, not impossible: this
    // constructor runs on an import path.
    FilesystemCookedAssetStore store{std::string()};
    EXPECT_FALSE(store.usable());

    const CookedAssetKey key = make_key(1, 1, 1, CookedAssetKind::Collision);
    std::vector<std::byte> loaded;
    EXPECT_FALSE(store.contains(key));
    EXPECT_FALSE(store.load(key, loaded));
    EXPECT_FALSE(store.store(key, sample_blob(1)));
    EXPECT_FALSE(store.evict(key));
}

TEST(Unit_CookingThresholds,RejectsWhatIsBrokenAndPassesWhatIsMerelyImperfect)
{
    CookingReport report;
    report.status = CookingStatus::Succeeded;
    report.tetrahedron_count = 500;
    report.worst_element_quality = 0.2f;

    const CookingThresholds thresholds;
    EXPECT_TRUE(apply_cooking_thresholds(thresholds, report));
    EXPECT_EQ(report.status, CookingStatus::Succeeded);

    // A vertex bound to nothing tears the render mesh, so the default tolerates none.
    CookingReport unbound = report;
    unbound.unembedded_vertex_count = 1;
    EXPECT_FALSE(apply_cooking_thresholds(thresholds, unbound));
    EXPECT_EQ(unbound.status, CookingStatus::RejectedByThreshold);
    // The asset still exists to be looked at: being told "this failed" without being
    // able to see the geometry that failed is not a diagnosis.
    EXPECT_TRUE(unbound.has_asset());

    CookingReport inverted = report;
    inverted.inverted_element_count = 1;
    EXPECT_FALSE(apply_cooking_thresholds(thresholds, inverted));

    CookingReport sliver = report;
    sliver.worst_element_quality = 0.0001f;
    EXPECT_FALSE(apply_cooking_thresholds(thresholds, sliver));

    // A rigid cook has no elements, so element quality is not a judgement it is subject
    // to — a zero there is "not measured", not "terrible".
    CookingReport rigid = report;
    rigid.tetrahedron_count = 0;
    rigid.worst_element_quality = 0.0f;
    EXPECT_TRUE(apply_cooking_thresholds(thresholds, rigid));

    // The protrusion limit is off by default, because a five-centimetre limit is
    // generous on a car and absurd on a doorknob.
    CookingReport fat = report;
    fat.hausdorff_error = 10.0f;
    EXPECT_TRUE(apply_cooking_thresholds(thresholds, fat));
    CookingThresholds strict = thresholds;
    strict.max_hausdorff_error = 0.05f;
    EXPECT_FALSE(apply_cooking_thresholds(strict, fat));

    // A hard failure is not something a threshold can un-fail.
    CookingReport empty;
    empty.status = CookingStatus::EmptyInput;
    EXPECT_FALSE(apply_cooking_thresholds(thresholds, empty));
    EXPECT_EQ(empty.status, CookingStatus::EmptyInput);
    EXPECT_FALSE(empty.has_asset());
}

TEST(Unit_CookingThresholds,JudgesTheSourceMeshWhenAskedTo)
{
    CookingReport report;
    report.status = CookingStatus::Succeeded;
    report.source.triangle_count = 12;
    report.source.boundary_edges = 4;

    CookingThresholds permissive;
    EXPECT_TRUE(apply_cooking_thresholds(permissive, report));

    CookingThresholds demanding;
    demanding.require_watertight_source = true;
    CookingReport judged = report;
    EXPECT_FALSE(apply_cooking_thresholds(demanding, judged));

    judged.source.boundary_edges = 0;
    judged.status = CookingStatus::Succeeded;
    EXPECT_TRUE(apply_cooking_thresholds(demanding, judged));
}
