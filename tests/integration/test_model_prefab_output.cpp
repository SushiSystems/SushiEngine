/**************************************************************************/
/* test_model_prefab_output.cpp                                           */
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

// What a model becomes on disk. The claim `prefab_system.md` §7 makes is that reimport is not a
// feature anyone writes: a `.meta` change changes the plan, which changes the prefab, which
// changes its revision, and the refresh pass rebuilds every placed instance the next time a
// scene opens. That claim is true only if the revision moves when the settings move and stays
// still when they do not, so both halves are pinned here — and the second matters more, because
// a revision that churned on every import would rebuild every imported instance in every scene,
// every time one is opened.
//
// The hierarchy case is the user-visible claim of the whole model-import effort: a model's
// children have to be children in the file. A writer that flattened the tree would pass every
// other case here.

#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <SushiEngine/model/import_settings.hpp>
#include <SushiEngine/model/import_settings_io.hpp>
#include <SushiEngine/model_import/prefab_output.hpp>

namespace
{
    // One root, one child, one grandchild, each with a translation of its own. No meshes and no
    // buffers: what this file pins is the shape of the output, and vertex data would say
    // nothing about it.
    const char* NESTED_GLTF = R"json({
      "asset": { "version": "2.0" },
      "scene": 0,
      "scenes": [ { "nodes": [ 0 ] } ],
      "nodes": [
        { "name": "Body", "children": [ 1 ], "translation": [1, 0, 0] },
        { "name": "Wheel", "children": [ 2 ], "translation": [0, 2, 0] },
        { "name": "Tire", "translation": [0, 0, 3] }
      ]
    })json";

    // Writes `contents` to `name` in the working directory and removes it, its `.meta` and its
    // prefab on destruction, so a run leaves nothing behind and two runs cannot see each
    // other's files.
    class ScratchModel
    {
        public:
            explicit ScratchModel(const char* name, const char* contents) : path_(name)
            {
                std::ofstream stream(path_, std::ios::binary);
                stream << contents;
            }

            ~ScratchModel()
            {
                std::remove(path_.c_str());
                std::remove(prefab().c_str());
                std::remove((path_ + ".meta").c_str());
            }

            const std::string& path() const noexcept { return path_; }

            /** @brief Where write_model_prefab puts this model's prefab. */
            std::string prefab() const { return path_ + ".sushiprefab"; }

        private:
            std::string path_;
    };

    /** @brief Reads a written prefab, or an empty object when it is not there or is broken. */
    nlohmann::json read_document(const std::string& path)
    {
        std::ifstream file(path);
        if (!file)
            return nlohmann::json::object();
        nlohmann::json document;
        try
        {
            file >> document;
        }
        catch (const nlohmann::json::parse_error&)
        {
            return nlohmann::json::object();
        }
        return document;
    }

    /** @brief The index of the entry named @p name, or -1. */
    int index_of(const nlohmann::json& entities, const char* name)
    {
        for (std::size_t i = 0; i < entities.size(); ++i)
            if (entities[i].value("name", std::string()) == name)
                return static_cast<int>(i);
        return -1;
    }
} // namespace

TEST(Integration_ModelPrefabOutput, ImportingAModelWritesAPrefabBesideIt)
{
    const ScratchModel model("sushiengine_prefab_output_beside.gltf", NESTED_GLTF);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));

    const nlohmann::json document = read_document(model.prefab());
    ASSERT_TRUE(document.contains("entities"));
    ASSERT_TRUE(document.contains("revision"));
    EXPECT_FALSE(document["revision"].get<std::string>().empty());
    ASSERT_EQ(document["entities"].size(), 3u);
    EXPECT_EQ(document["entities"].front().value("name", std::string()), "Body");
    // The root is written as a root: an instance placed from this file has to hang under
    // whatever the scene gives it, not under an index into a document that is not there.
    EXPECT_EQ(document["entities"].front().value("parent", 0), -1);
}

TEST(Integration_ModelPrefabOutput, TheHierarchySurvivesIntoThePrefab)
{
    const ScratchModel model("sushiengine_prefab_output_nested.gltf", NESTED_GLTF);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));

    const nlohmann::json entities = read_document(model.prefab())["entities"];
    ASSERT_EQ(entities.size(), 3u);

    const int body = index_of(entities, "Body");
    const int wheel = index_of(entities, "Wheel");
    const int tire = index_of(entities, "Tire");
    ASSERT_NE(body, -1);
    ASSERT_NE(wheel, -1);
    ASSERT_NE(tire, -1);
    EXPECT_EQ(entities[static_cast<std::size_t>(wheel)].value("parent", -1), body);
    EXPECT_EQ(entities[static_cast<std::size_t>(tire)].value("parent", -1), wheel);

    // Local, not accumulated: the grandchild keeps its own (0, 0, 3). A writer that stored
    // world transforms would put (1, 2, 3) here, and every child would then be placed twice —
    // once by the stored transform and once by its parent's.
    const nlohmann::json& position = entities[static_cast<std::size_t>(tire)]["position"];
    ASSERT_EQ(position.size(), 3u);
    EXPECT_NEAR(position[0].get<double>(), 0.0, 1e-5);
    EXPECT_NEAR(position[1].get<double>(), 0.0, 1e-5);
    EXPECT_NEAR(position[2].get<double>(), 3.0, 1e-5);
}

TEST(Integration_ModelPrefabOutput, EveryEntryNamesTheAssetItCameFrom)
{
    const ScratchModel model("sushiengine_prefab_output_source.gltf", NESTED_GLTF);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));

    // The identifier §4.4 owes override resolution has to be there in an imported prefab too,
    // or a model placed today is unmatchable the day overrides arrive.
    const nlohmann::json entities = read_document(model.prefab())["entities"];
    ASSERT_EQ(entities.size(), 3u);
    for (const auto& entry : entities)
        EXPECT_TRUE(entry.contains("prefab_entity_id"));
}

TEST(Integration_ModelPrefabOutput, ImportingTwiceWithNoChangeKeepsTheRevision)
{
    const ScratchModel model("sushiengine_prefab_output_stable.gltf", NESTED_GLTF);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));
    const std::string first = read_document(model.prefab()).value("revision", std::string("a"));
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));
    const std::string second = read_document(model.prefab()).value("revision", std::string("b"));

    // The half that matters more: an unchanged asset must not churn its revision, or opening
    // any scene rebuilds every imported instance in it, every time.
    EXPECT_FALSE(first.empty());
    EXPECT_EQ(first, second);
}

TEST(Integration_ModelPrefabOutput, ChangingTheSettingsChangesTheRevision)
{
    const ScratchModel model("sushiengine_prefab_output_settings.gltf", NESTED_GLTF);

    SushiEngine::Model::ModelImportReport report;
    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));
    const std::string before = read_document(model.prefab()).value("revision", std::string("a"));

    // §7's claim that reimport is not a feature anyone writes: a setting change changes the
    // plan, which changes the prefab, which changes the revision, which is what makes the
    // refresh rebuild every placed instance.
    SushiEngine::Model::ModelImportSettings settings;
    settings.scale_factor = 2.0f;
    ASSERT_TRUE(SushiEngine::Model::save_model_import_settings(model.path(), settings));

    ASSERT_TRUE(SushiEngine::ModelImport::write_model_prefab(model.path(), report));
    const std::string after = read_document(model.prefab()).value("revision", std::string("b"));
    EXPECT_NE(before, after);
}

TEST(Integration_ModelPrefabOutput, AMissingAssetFailsAndWritesNothing)
{
    const std::string absent = "sushiengine_prefab_output_absent.gltf";
    const std::string prefab = absent + ".sushiprefab";
    std::error_code error;
    std::filesystem::remove(prefab, error);

    SushiEngine::Model::ModelImportReport report;
    EXPECT_FALSE(SushiEngine::ModelImport::write_model_prefab(absent, report));
    // Nothing written rather than an empty prefab: a placement made from a truncated file
    // would produce nothing and report success, which is worse than a failure that says so.
    EXPECT_FALSE(std::filesystem::exists(prefab));
}
