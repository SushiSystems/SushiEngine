/**************************************************************************/
/* test_cook_bake_state.cpp                                               */
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

// The bake surface's decisions, which is everything about it that can be wrong: whether
// Re-cook actually gets past the cache, whether the overlay follows the selection, whether a
// re-bake updates a row or adds one. None of that is reachable through an ImGui call, which
// is why `Authoring::CookBakeState` links no UI and the panel over it is only widgets.

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/model/import_settings_io.hpp>
#include <SushiEngine/physics/cooking/collision_asset.hpp>

#include <SushiEngine/authoring/cook_bake_state.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Authoring;

namespace
{
    /** @brief An outward-wound box. */
    Geometry::TriangleMesh box_mesh(float h)
    {
        Geometry::TriangleMesh mesh;
        const float c[8][3] = {{-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
                               {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
        for (const auto& corner : c)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t f[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                        {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                        {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : f)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief A loader answering from a table, so no file is needed. */
    Physics::Cooking::MeshLoader table_loader()
    {
        return [](const std::string& path, Geometry::TriangleMesh& out) -> bool
        {
            if (path == "crate.gltf")
            {
                out = box_mesh(0.5f);
                return true;
            }
            if (path == "pillar.gltf")
            {
                out = box_mesh(0.25f);
                return true;
            }
            return false;
        };
    }

    /** @brief Makes the state's project default cook quickly. */
    void make_quick(Authoring::CookBakeState& state, bool soft_body)
    {
        Physics::Cooking::ImportProfile profile;
        profile.parameters.fidelity = 0.0f;
        profile.parameters.voxel_resolution = 5;
        profile.parameters.distance_field_resolution = 8;
        profile.parameters.simulation_level_count = 1;
        profile.parameters.surface_conforming_passes = 0;
        profile.parameters.accuracy_lattice_order = 2;
        profile.parameters.cook_soft_body = soft_body;
        state.profiles().set_project_default(profile);
    }

    /** @brief Bakes @p path and waits for it, the way a build machine would. */
    void bake_and_settle(Authoring::CookBakeState& state, const std::string& path,
                         bool rebake = false)
    {
        if (rebake)
            state.rebake(path);
        else
            state.bake(path);
        // The panel polls per frame; a test has no frames, so it polls until the work lands.
        while (state.busy())
            state.poll();
        state.poll();
    }
} // namespace

TEST(Unit_CookBakeState, FilesACookAndBuildsTheOverlayForIt)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "crate.gltf");
    ASSERT_EQ(state.entries().size(), 1u);

    const Authoring::BakedAssetEntry* entry = state.entry("crate.gltf");
    ASSERT_NE(entry, nullptr);
    EXPECT_TRUE(entry->loaded);
    EXPECT_EQ(entry->source_triangle_count, 12u);
    ASSERT_TRUE(entry->has_collision());
    EXPECT_FALSE(entry->has_soft_body());
    EXPECT_TRUE(entry->collision_report.has_asset());

    // The first cook selects itself, so an artist who bakes one thing sees it without
    // hunting for it.
    EXPECT_EQ(state.selected(), "crate.gltf");

    // And the overlay geometry is there: §14 asks for the collider to be *visible*, and an
    // overlay with no segments is a collider that reads as absent.
    const std::vector<float>& wireframe = state.collision_wireframe();
    EXPECT_FALSE(wireframe.empty());
    EXPECT_EQ(wireframe.size() % 6, 0u);
    // A box's hull has twelve edges, and every segment must lie on it.
    EXPECT_GE(wireframe.size() / 6, 12u);
    for (std::size_t i = 0; i < wireframe.size(); ++i)
        EXPECT_LE(std::fabs(double(wireframe[i])), 0.5 + 1e-4);
}

TEST(Unit_CookBakeState, ReBakesIntoTheSameRowRatherThanASecondOne)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "crate.gltf");
    bake_and_settle(state, "pillar.gltf");
    ASSERT_EQ(state.entries().size(), 2u);

    // A re-cook of a crate updates the crate's row. A second row for the same asset is a
    // list an artist has to tell apart by nothing.
    bake_and_settle(state, "crate.gltf", true);
    EXPECT_EQ(state.entries().size(), 2u);
    ASSERT_NE(state.entry("crate.gltf"), nullptr);
    ASSERT_NE(state.entry("pillar.gltf"), nullptr);
}

TEST(Unit_CookBakeState, ReCookGetsPastTheCacheAndAPlainBakeDoesNot)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "crate.gltf");
    ASSERT_NE(state.entry("crate.gltf"), nullptr);
    EXPECT_FALSE(state.entry("crate.gltf")->collision_report.served_from_cache);

    // An ordinary bake of an unchanged mesh is served, which is §8.1's whole point.
    bake_and_settle(state, "crate.gltf");
    EXPECT_TRUE(state.entry("crate.gltf")->collision_report.served_from_cache);

    // Re-cook has to get past that entry even though its key has not changed — which is the
    // case the content hash cannot detect, because what changed is the cooker.
    bake_and_settle(state, "crate.gltf", true);
    EXPECT_FALSE(state.entry("crate.gltf")->collision_report.served_from_cache);

    // And it did not simply disable the cache: the next plain bake is served again.
    bake_and_settle(state, "crate.gltf");
    EXPECT_TRUE(state.entry("crate.gltf")->collision_report.served_from_cache);
}

TEST(Unit_CookBakeState, TheOverlayFollowsTheSelection)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "crate.gltf");
    bake_and_settle(state, "pillar.gltf");

    state.select("crate.gltf");
    const std::vector<float> crate_wireframe = state.collision_wireframe();
    ASSERT_FALSE(crate_wireframe.empty());

    state.select("pillar.gltf");
    const std::vector<float> pillar_wireframe = state.collision_wireframe();
    ASSERT_FALSE(pillar_wireframe.empty());

    // Different shapes, so different geometry — a stale overlay showing the previous
    // selection's collider is worse than none, because it is believable.
    EXPECT_NE(crate_wireframe, pillar_wireframe);
    for (std::size_t i = 0; i < pillar_wireframe.size(); ++i)
        EXPECT_LE(std::fabs(double(pillar_wireframe[i])), 0.25 + 1e-4);

    // Selecting something never cooked empties the overlay rather than leaving the last one.
    state.select("nothing.gltf");
    EXPECT_TRUE(state.collision_wireframe().empty());
}

TEST(Unit_CookBakeState, ReportsAnAssetItCouldNotLoadAndDrawsNothingForIt)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "missing.gltf");
    const Authoring::BakedAssetEntry* entry = state.entry("missing.gltf");
    // Filed, not dropped: an artist has to be able to see that the import failed.
    ASSERT_NE(entry, nullptr);
    EXPECT_FALSE(entry->loaded);
    EXPECT_FALSE(entry->has_collision());
    EXPECT_TRUE(state.collision_wireframe().empty());
}

TEST(Unit_CookBakeState, CooksBothKindsWhenTheProfileAsksForThem)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, true);

    bake_and_settle(state, "crate.gltf");
    const Authoring::BakedAssetEntry* entry = state.entry("crate.gltf");
    ASSERT_NE(entry, nullptr);
    ASSERT_TRUE(entry->has_collision());
    ASSERT_TRUE(entry->has_soft_body());
    EXPECT_GT(entry->soft_body_report.tetrahedron_count, 0u);

    // A per-asset override turns one asset off without touching the project's default, which
    // is what keeps every rock from paying for a tetrahedral mesh.
    Physics::Cooking::ImportProfileOverride rigid_only;
    rigid_only.cook_soft_body = false;
    state.profiles().set_override("pillar.gltf", rigid_only);

    bake_and_settle(state, "pillar.gltf");
    const Authoring::BakedAssetEntry* pillar = state.entry("pillar.gltf");
    ASSERT_NE(pillar, nullptr);
    EXPECT_TRUE(pillar->has_collision());
    EXPECT_FALSE(pillar->has_soft_body());
}

TEST(Unit_CookBakeState, TheProjectDefaultSurvivesASaveAndLoadAndTheDocumentHoldsNothingElse)
{
    // §16.45.3: the Bake panel's settings had no storage path at all — every edit made in a
    // session was gone the moment the editor closed. This is the claim that they are not,
    // written the way an editor restart actually exercises it: a fresh `Authoring::CookBakeState`,
    // pointed at the same path, must read back exactly what the first one wrote.
    //
    // The document holds the project default and nothing else. What one asset says differs
    // belongs beside that asset, in its `.meta` sidecar (`docs/design/model_import.md` §4.2),
    // so an override recorded on the library is deliberately absent from the file.
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / "sushiengine_cook_bake_state_test.json";
    std::filesystem::remove(path);

    {
        Authoring::CookBakeState writer(table_loader(), std::string());
        writer.set_profile_storage_path(path.string());

        Physics::Cooking::ImportProfile project;
        project.parameters.fidelity = 0.75f;
        project.parameters.cook_soft_body = true;
        project.parameters.hull_vertex_budget = 48;
        project.node_beam_settings.core_mass_fraction = 0.6f;
        writer.profiles().set_project_default(project);

        Physics::Cooking::ImportProfileOverride crate;
        crate.cook_node_beam = true;
        writer.profiles().set_override("crate.gltf", crate);

        ASSERT_TRUE(writer.save_profiles());
    }

    Authoring::CookBakeState reader(table_loader(), std::string());
    reader.set_profile_storage_path(path.string());
    ASSERT_TRUE(reader.load_profiles());

    const Physics::Cooking::ImportProfile& loaded_project = reader.profiles().project_default();
    EXPECT_FLOAT_EQ(loaded_project.parameters.fidelity, 0.75f);
    EXPECT_TRUE(loaded_project.parameters.cook_soft_body);
    EXPECT_EQ(loaded_project.parameters.hull_vertex_budget, 48);
    EXPECT_FLOAT_EQ(loaded_project.node_beam_settings.core_mass_fraction, 0.6f);

    EXPECT_FALSE(reader.profiles().has_override("crate.gltf"));
    EXPECT_EQ(reader.profiles().override_count(), 0u);
    EXPECT_TRUE(reader.last_migration().empty());

    std::filesystem::remove(path);
}

TEST(Unit_CookBakeState, AnAssetsSidecarDecidesWhichCookersRunForIt)
{
    // The per-asset override lives in the asset's `.meta` now, so the file has to be what
    // reaches the cooker — a migration that moved the settings somewhere nothing reads would
    // have cost the artist every one of them.
    const std::string asset = "test_cook_bake_sidecar_asset.gltf";
    const std::string sidecar = Model::model_import_settings_path(asset);
    std::filesystem::remove(sidecar);

    Model::ModelImportSettings settings;
    settings.cooking.cook_soft_body = false;
    ASSERT_TRUE(Model::save_model_import_settings(asset, settings));

    Authoring::CookBakeState state(
        [&asset](const std::string& path, Geometry::TriangleMesh& out) -> bool
        {
            if (path != asset)
                return false;
            out = box_mesh(0.25f);
            return true;
        },
        std::string());
    // The project default cooks a soft body; the sidecar is the only thing saying otherwise.
    make_quick(state, true);

    bake_and_settle(state, asset);
    const Authoring::BakedAssetEntry* entry = state.entry(asset);
    ASSERT_NE(entry, nullptr);
    EXPECT_TRUE(entry->has_collision());
    EXPECT_FALSE(entry->has_soft_body());

    std::filesystem::remove(sidecar);
}

TEST(Unit_CookBakeState, AMalformedSidecarIsReportedAndTheAssetCooksAtTheProjectDefault)
{
    // `docs/design/model_import.md` §8: a `.meta` that fails to parse is reported and the
    // project default is used for that import. Both halves are asserted, because either one
    // alone is a failure an artist cannot diagnose — a silent fallback looks like the setting
    // is wrong, and a report without a fallback would be an asset that did not cook at all.
    const std::string asset = "test_cook_bake_malformed_sidecar_asset.gltf";
    const std::string sidecar = Model::model_import_settings_path(asset);
    {
        std::ofstream stream(sidecar, std::ios::binary | std::ios::trunc);
        ASSERT_TRUE(bool(stream));
        stream << "{ this is not json";
    }

    Authoring::CookBakeState state(
        [&asset](const std::string& path, Geometry::TriangleMesh& out) -> bool
        {
            if (path != asset)
                return false;
            out = box_mesh(0.5f);
            return true;
        },
        std::string());
    // The project default cooks a soft body. The sidecar cannot say otherwise, so a soft body
    // in the entry is what proves the default was the profile that ran.
    make_quick(state, true);

    bake_and_settle(state, asset);

    const std::vector<std::string> reported = state.take_unreadable_sidecars();
    ASSERT_EQ(reported.size(), 1u);
    EXPECT_EQ(reported[0], asset);
    // Drained, so the panel reports it once rather than on every frame that follows.
    EXPECT_TRUE(state.take_unreadable_sidecars().empty());

    const Authoring::BakedAssetEntry* entry = state.entry(asset);
    ASSERT_NE(entry, nullptr);
    EXPECT_TRUE(entry->has_collision());
    EXPECT_TRUE(entry->has_soft_body());

    std::filesystem::remove(sidecar);
}

TEST(Unit_CookBakeState, AReadableSidecarReportsNothing)
{
    // The counterpart, and the one that stops the report from being noise: an asset with no
    // sidecar at all has never been configured, which is the ordinary case and not a fault.
    Authoring::CookBakeState state(table_loader(), std::string());
    make_quick(state, false);

    bake_and_settle(state, "crate.gltf");
    EXPECT_TRUE(state.take_unreadable_sidecars().empty());
}

TEST(Unit_CookBakeState, AnUnsetStoragePathIsANoOpRatherThanAFailure)
{
    Authoring::CookBakeState state(table_loader(), std::string());
    EXPECT_TRUE(state.save_profiles());
    EXPECT_TRUE(state.load_profiles());
}

TEST(Unit_CollisionWireframe, DrawsStaticGeometryAsItsOwnTriangles)
{
    // A static asset's collider *is* the mesh, so the overlay is the cooked triangles rather
    // than a rebuilt hull — and the segment count reflects a whole box, not one hull edge set.
    Authoring::CookBakeState state(table_loader(), std::string());
    Physics::Cooking::ImportProfile profile;
    profile.parameters.fidelity = 0.0f;
    profile.parameters.distance_field_resolution = 8;
    profile.parameters.accuracy_lattice_order = 2;
    profile.parameters.static_geometry = true;
    state.profiles().set_project_default(profile);

    bake_and_settle(state, "crate.gltf");
    const Authoring::BakedAssetEntry* entry = state.entry("crate.gltf");
    ASSERT_NE(entry, nullptr);
    ASSERT_TRUE(entry->has_collision());

    const Physics::Cooking::CollisionAssetView view = Physics::Cooking::load_collision_blob(
        entry->collision_bytes.data(), entry->collision_bytes.size());
    ASSERT_TRUE(view.valid);
    ASSERT_TRUE(view.static_mesh);

    // Eighteen unique edges in a twelve-triangle box: twelve cube edges plus one diagonal
    // per face. That is the number a dedup that works produces, and thirty-six is the number
    // it produces when it does not.
    EXPECT_EQ(state.collision_wireframe().size() / 6, 18u);
}
