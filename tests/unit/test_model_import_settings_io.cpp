/**************************************************************************/
/* test_model_import_settings_io.cpp                                      */
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

// The `.meta` sidecar, which is where a model asset's import settings live. What can be wrong
// here is what a settings file always gets wrong: a field that does not survive the round trip,
// an unset override that comes back set, and a broken file that is read as "no settings" rather
// than as a fault.
//
// And the one-way move onto it, which has its own three ways to be wrong: an override that
// arrives without the rest of the sidecar's settings, one whose asset is gone and is carried
// forward invisibly, and a migration that runs again on a project that already moved.

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include <SushiEngine/model/import_settings_io.hpp>

using SushiEngine::Model::ModelImportSettings;

namespace
{
    // `gtest_discover_tests` registers one CTest test per case and CTest may run them at once,
    // so every case names its own scratch files rather than sharing one. Two cases writing
    // `<asset>.meta` in the same working directory would be a race, and a race in a settings
    // test fails somewhere else.
    std::string scratch_asset(const char* discriminator)
    {
        return std::string("test_model_import_settings_") + discriminator + ".gltf";
    }

    // Removes an asset's sidecar however the case leaves, so a failed run leaves nothing behind.
    class ScratchSidecar
    {
    public:
        explicit ScratchSidecar(std::string asset_path) : asset_path_(std::move(asset_path)) {}

        ScratchSidecar(const ScratchSidecar&) = delete;
        ScratchSidecar& operator=(const ScratchSidecar&) = delete;

        ~ScratchSidecar()
        {
            std::remove(SushiEngine::Model::model_import_settings_path(asset_path_).c_str());
        }

        const std::string& asset() const noexcept { return asset_path_; }

    private:
        std::string asset_path_;
    };

    // Removes a plain file however the case leaves: the stand-in asset the migration looks for,
    // and the project document it rewrites.
    class ScratchFile
    {
    public:
        explicit ScratchFile(std::string path) : path_(std::move(path)) {}

        ScratchFile(const ScratchFile&) = delete;
        ScratchFile& operator=(const ScratchFile&) = delete;

        ~ScratchFile() { std::remove(path_.c_str()); }

        const std::string& path() const noexcept { return path_; }

    private:
        std::string path_;
    };

    // A cooking document in the shape `CookBakeState::save_profiles` wrote before the overrides
    // moved out of it, with the override keys `import_profile_override_to_json` actually spells.
    std::string project_document(const std::string& present, const std::string& vanished)
    {
        return "{\n"
               "  \"project_default\": { \"parameters\": { \"fidelity\": 0.5 } },\n"
               "  \"overrides\": {\n"
               "    \"" +
               present +
               "\": { \"fidelity\": 0.9, \"cook_soft_body\": true },\n"
               "    \"" +
               vanished +
               "\": { \"fidelity\": 0.1 }\n"
               "  }\n"
               "}\n";
    }

    void write_file(const std::string& path, const std::string& contents)
    {
        std::ofstream stream(path, std::ios::binary | std::ios::trunc);
        stream << contents;
    }

    std::string read_file(const std::string& path)
    {
        std::ifstream stream(path, std::ios::binary);
        return std::string(std::istreambuf_iterator<char>(stream),
                           std::istreambuf_iterator<char>());
    }
}

TEST(Unit_ModelImportSettingsIO, TheSidecarPathAppendsMetaToTheWholeAssetPath)
{
    EXPECT_EQ(SushiEngine::Model::model_import_settings_path("models/Car.gltf"),
              "models/Car.gltf.meta");
    EXPECT_EQ(SushiEngine::Model::model_import_settings_path("models/Car.glb"),
              "models/Car.glb.meta");
}

TEST(Unit_ModelImportSettingsIO, AnAssetWithNoSidecarLoadsTheDefaultsAndDoesNotFail)
{
    ModelImportSettings settings;
    settings.scale_factor = 42.0f;
    EXPECT_TRUE(
        SushiEngine::Model::load_model_import_settings("no_such_asset_anywhere.gltf", settings));
    EXPECT_TRUE(settings == ModelImportSettings{});
}

TEST(Unit_ModelImportSettingsIO, DefaultedSettingsSurviveAWriteAndRead)
{
    const ScratchSidecar scratch(scratch_asset("defaults"));
    const ModelImportSettings written;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch.asset(), written));

    ModelImportSettings read;
    read.preserve_pivots = false;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch.asset(), read));
    EXPECT_TRUE(read == written);
}

TEST(Unit_ModelImportSettingsIO, EveryFieldSurvivesAWriteAndRead)
{
    const ScratchSidecar scratch(scratch_asset("every_field"));
    ModelImportSettings written;
    written.scale_factor = 0.01f;
    written.root_rotation_degrees = SushiEngine::Model::Vector3f{-90.0f, 0.0f, 180.0f};
    written.import_materials = false;
    written.import_lights = false;
    written.import_cameras = false;
    written.preserve_pivots = false;
    written.generate_colliders = true;
    written.cooking.fidelity = 0.75f;
    written.cooking.cook_collision = true;
    written.cooking.cook_soft_body = false;
    written.cooking.cook_node_beam = true;
    written.cooking.static_geometry = false;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch.asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch.asset(), read));
    EXPECT_TRUE(read == written);
}

TEST(Unit_ModelImportSettingsIO, AnUnsetCookingOverrideStaysUnsetRatherThanBecomingAValue)
{
    const ScratchSidecar scratch(scratch_asset("unset_override"));
    ModelImportSettings written;
    written.cooking.fidelity = 0.5f;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch.asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch.asset(), read));
    EXPECT_TRUE(read.cooking.fidelity.has_value());
    EXPECT_FALSE(read.cooking.cook_collision.has_value());
    EXPECT_FALSE(read.cooking.cook_soft_body.has_value());
    EXPECT_FALSE(read.cooking.cook_node_beam.has_value());
    EXPECT_FALSE(read.cooking.static_geometry.has_value());
}

TEST(Unit_ModelImportSettingsIO, AMalformedSidecarFailsAndYieldsTheDefaults)
{
    const ScratchSidecar scratch(scratch_asset("malformed"));
    write_file(SushiEngine::Model::model_import_settings_path(scratch.asset()),
               "{ this is not json");

    ModelImportSettings read;
    read.scale_factor = 7.0f;
    EXPECT_FALSE(SushiEngine::Model::load_model_import_settings(scratch.asset(), read));
    EXPECT_TRUE(read == ModelImportSettings{});
}

TEST(Unit_CookingOverrideMigration, AnOverrideBecomesASidecarAndLeavesTheDocument)
{
    const ScratchFile document("test_migration_basic_project.json");
    const ScratchFile present("test_migration_basic_present.gltf");
    const ScratchSidecar sidecar(present.path());
    const std::string vanished = "test_migration_basic_vanished.gltf";
    write_file(document.path(), project_document(present.path(), vanished));
    write_file(present.path(), "");

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document.path(),
                                                                         migrated, dropped));

    ASSERT_EQ(migrated.size(), 1u);
    EXPECT_EQ(migrated[0], present.path());
    ASSERT_EQ(dropped.size(), 1u);
    EXPECT_EQ(dropped[0], vanished);

    ModelImportSettings settings;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(present.path(), settings));
    ASSERT_TRUE(settings.cooking.fidelity.has_value());
    EXPECT_FLOAT_EQ(*settings.cooking.fidelity, 0.9f);
    ASSERT_TRUE(settings.cooking.cook_soft_body.has_value());
    EXPECT_TRUE(*settings.cooking.cook_soft_body);
    // An override that said nothing about a field must not invent a value for it.
    EXPECT_FALSE(settings.cooking.cook_node_beam.has_value());

    const std::string rewritten = read_file(document.path());
    EXPECT_EQ(rewritten.find("overrides"), std::string::npos);
    EXPECT_NE(rewritten.find("project_default"), std::string::npos);
}

TEST(Unit_CookingOverrideMigration, ASidecarKeepsTheSettingsThatAreNotCooking)
{
    const ScratchFile document("test_migration_preserve_project.json");
    const ScratchFile present("test_migration_preserve_present.gltf");
    const ScratchSidecar sidecar(present.path());
    write_file(document.path(),
               project_document(present.path(), "test_migration_preserve_vanished.gltf"));
    write_file(present.path(), "");

    // The asset was already configured, and the migration moves one field of its settings.
    // Anything else it says has to survive, or the move costs the artist the rest of it.
    ModelImportSettings existing;
    existing.scale_factor = 0.01f;
    existing.preserve_pivots = false;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(present.path(), existing));

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document.path(),
                                                                         migrated, dropped));

    ModelImportSettings settings;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(present.path(), settings));
    EXPECT_FLOAT_EQ(settings.scale_factor, 0.01f);
    EXPECT_FALSE(settings.preserve_pivots);
    ASSERT_TRUE(settings.cooking.fidelity.has_value());
    EXPECT_FLOAT_EQ(*settings.cooking.fidelity, 0.9f);
}

TEST(Unit_CookingOverrideMigration, RunningItTwiceChangesNothingTheSecondTime)
{
    const ScratchFile document("test_migration_idempotent_project.json");
    const ScratchFile present("test_migration_idempotent_present.gltf");
    const ScratchSidecar sidecar(present.path());
    write_file(document.path(),
               project_document(present.path(), "test_migration_idempotent_vanished.gltf"));
    write_file(present.path(), "");

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document.path(),
                                                                         migrated, dropped));
    const std::string sidecar_after_first =
        read_file(SushiEngine::Model::model_import_settings_path(present.path()));

    migrated.clear();
    dropped.clear();
    ASSERT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document.path(),
                                                                         migrated, dropped));
    EXPECT_TRUE(migrated.empty());
    EXPECT_TRUE(dropped.empty());
    EXPECT_EQ(read_file(SushiEngine::Model::model_import_settings_path(present.path())),
              sidecar_after_first);
}

TEST(Unit_CookingOverrideMigration, AProjectWithNoDocumentIsNotAFailure)
{
    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    EXPECT_TRUE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(
        "no_such_project_document.json", migrated, dropped));
    EXPECT_TRUE(migrated.empty());
    EXPECT_TRUE(dropped.empty());
}

TEST(Unit_CookingOverrideMigration, AMalformedDocumentFailsAndIsLeftAlone)
{
    const ScratchFile document("test_migration_malformed_project.json");
    write_file(document.path(), "{ this is not json");

    std::vector<std::string> migrated;
    std::vector<std::string> dropped;
    EXPECT_FALSE(SushiEngine::Model::migrate_cooking_overrides_to_sidecars(document.path(),
                                                                          migrated, dropped));
    EXPECT_TRUE(migrated.empty());
    EXPECT_TRUE(dropped.empty());
    EXPECT_EQ(read_file(document.path()), "{ this is not json");
}

