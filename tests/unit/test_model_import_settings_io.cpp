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

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include <SushiEngine/model/import_settings_io.hpp>

using SushiEngine::Model::ModelImportSettings;

namespace
{
    // A path in the working directory the test owns and removes, so a run leaves nothing behind.
    std::string scratch_asset()
    {
        return std::string("test_model_import_settings.gltf");
    }

    struct ScratchSidecar
    {
        ~ScratchSidecar()
        {
            std::remove(SushiEngine::Model::model_import_settings_path(scratch_asset()).c_str());
        }
    };
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
    ScratchSidecar cleanup;
    const ModelImportSettings written;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    read.preserve_pivots = false;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_TRUE(read == written);
}

TEST(Unit_ModelImportSettingsIO, EveryFieldSurvivesAWriteAndRead)
{
    ScratchSidecar cleanup;
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
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_TRUE(read == written);
}

TEST(Unit_ModelImportSettingsIO, AnUnsetCookingOverrideStaysUnsetRatherThanBecomingAValue)
{
    ScratchSidecar cleanup;
    ModelImportSettings written;
    written.cooking.fidelity = 0.5f;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(scratch_asset(), written));

    ModelImportSettings read;
    ASSERT_TRUE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_TRUE(read.cooking.fidelity.has_value());
    EXPECT_FALSE(read.cooking.cook_collision.has_value());
    EXPECT_FALSE(read.cooking.cook_soft_body.has_value());
    EXPECT_FALSE(read.cooking.cook_node_beam.has_value());
    EXPECT_FALSE(read.cooking.static_geometry.has_value());
}

TEST(Unit_ModelImportSettingsIO, AMalformedSidecarFailsAndYieldsTheDefaults)
{
    ScratchSidecar cleanup;
    {
        std::ofstream stream(SushiEngine::Model::model_import_settings_path(scratch_asset()));
        stream << "{ this is not json";
    }
    ModelImportSettings read;
    read.scale_factor = 7.0f;
    EXPECT_FALSE(SushiEngine::Model::load_model_import_settings(scratch_asset(), read));
    EXPECT_TRUE(read == ModelImportSettings{});
}
