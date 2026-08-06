/**************************************************************************/
/* import_settings_io.cpp                                                 */
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

#include <SushiEngine/model/import_settings_io.hpp>

#include <filesystem>
#include <fstream>
#include <system_error>

#include <nlohmann/json.hpp>

namespace SushiEngine
{
    namespace Model
    {
        namespace
        {
            using nlohmann::json;
            using Physics::Cooking::ImportProfileOverride;

            // The sidecar's extension is appended to the asset's whole path rather than
            // replacing its own, so two files that differ only in extension keep two sidecars.
            const char* const SIDECAR_EXTENSION = ".meta";

            // The `{"x", "y", "z"}` object the scene and effect serializers already write a
            // vector as, rather than a three-element array, so a `.meta` opened in a text
            // editor names its own axes.
            json vector3f_to_json(const Vector3f& v)
            {
                return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
            }

            Vector3f vector3f_from_json(const json& j, const Vector3f& fallback)
            {
                Vector3f v = fallback;
                v.x = j.value("x", v.x);
                v.y = j.value("y", v.y);
                v.z = j.value("z", v.z);
                return v;
            }

            // Only the overrides that hold a value are written. An absent key is what "this
            // asset has no opinion about this" means, and writing every field with the
            // project's current value in it would freeze five decisions the asset never made.
            json cooking_override_to_json(const ImportProfileOverride& o)
            {
                json j = json::object();
                if (o.fidelity.has_value())
                    j["fidelity"] = *o.fidelity;
                if (o.cook_collision.has_value())
                    j["cook_collision"] = *o.cook_collision;
                if (o.cook_soft_body.has_value())
                    j["cook_soft_body"] = *o.cook_soft_body;
                if (o.cook_node_beam.has_value())
                    j["cook_node_beam"] = *o.cook_node_beam;
                if (o.static_geometry.has_value())
                    j["static_geometry"] = *o.static_geometry;
                return j;
            }

            ImportProfileOverride cooking_override_from_json(const json& j)
            {
                ImportProfileOverride o;
                if (j.contains("fidelity"))
                    o.fidelity = j["fidelity"].get<float>();
                if (j.contains("cook_collision"))
                    o.cook_collision = j["cook_collision"].get<bool>();
                if (j.contains("cook_soft_body"))
                    o.cook_soft_body = j["cook_soft_body"].get<bool>();
                if (j.contains("cook_node_beam"))
                    o.cook_node_beam = j["cook_node_beam"].get<bool>();
                if (j.contains("static_geometry"))
                    o.static_geometry = j["static_geometry"].get<bool>();
                return o;
            }

            bool cooking_overrides_equal(const ImportProfileOverride& a,
                                         const ImportProfileOverride& b) noexcept
            {
                return a.fidelity == b.fidelity && a.cook_collision == b.cook_collision &&
                       a.cook_soft_body == b.cook_soft_body &&
                       a.cook_node_beam == b.cook_node_beam &&
                       a.static_geometry == b.static_geometry;
            }
        } // namespace

        bool operator==(const ModelImportSettings& a, const ModelImportSettings& b) noexcept
        {
            return a.scale_factor == b.scale_factor &&
                   a.root_rotation_degrees.x == b.root_rotation_degrees.x &&
                   a.root_rotation_degrees.y == b.root_rotation_degrees.y &&
                   a.root_rotation_degrees.z == b.root_rotation_degrees.z &&
                   a.import_materials == b.import_materials &&
                   a.import_lights == b.import_lights && a.import_cameras == b.import_cameras &&
                   a.preserve_pivots == b.preserve_pivots &&
                   a.generate_colliders == b.generate_colliders &&
                   cooking_overrides_equal(a.cooking, b.cooking);
        }

        bool operator!=(const ModelImportSettings& a, const ModelImportSettings& b) noexcept
        {
            return !(a == b);
        }

        std::string model_import_settings_path(const std::string& asset_path)
        {
            return asset_path + SIDECAR_EXTENSION;
        }

        bool load_model_import_settings(const std::string& asset_path, ModelImportSettings& out)
        {
            out = ModelImportSettings{};

            std::ifstream stream(model_import_settings_path(asset_path), std::ios::binary);
            if (!stream)
                return true; // Never configured, which is the ordinary case rather than a fault.

            ModelImportSettings settings;
            try
            {
                json document;
                stream >> document;
                if (!document.is_object())
                    return false;

                // Read through `value` so a sidecar written before a field existed loads at
                // that field's default rather than failing the whole asset's settings.
                settings.scale_factor = document.value("scale_factor", settings.scale_factor);
                if (document.contains("root_rotation_degrees"))
                    settings.root_rotation_degrees = vector3f_from_json(
                        document["root_rotation_degrees"], settings.root_rotation_degrees);
                settings.import_materials =
                    document.value("import_materials", settings.import_materials);
                settings.import_lights = document.value("import_lights", settings.import_lights);
                settings.import_cameras =
                    document.value("import_cameras", settings.import_cameras);
                settings.preserve_pivots =
                    document.value("preserve_pivots", settings.preserve_pivots);
                settings.generate_colliders =
                    document.value("generate_colliders", settings.generate_colliders);
                if (document.contains("cooking"))
                    settings.cooking = cooking_override_from_json(document["cooking"]);
            }
            catch (const json::exception&)
            {
                // A sidecar that is there but unreadable is reported rather than ignored: an
                // artist whose settings stopped applying needs to know it was the file.
                return false;
            }

            out = settings;
            return true;
        }

        bool save_model_import_settings(const std::string& asset_path,
                                        const ModelImportSettings& settings)
        {
            const json document{
                {"scale_factor", settings.scale_factor},
                {"root_rotation_degrees", vector3f_to_json(settings.root_rotation_degrees)},
                {"import_materials", settings.import_materials},
                {"import_lights", settings.import_lights},
                {"import_cameras", settings.import_cameras},
                {"preserve_pivots", settings.preserve_pivots},
                {"generate_colliders", settings.generate_colliders},
                {"cooking", cooking_override_to_json(settings.cooking)}};

            std::ofstream stream(model_import_settings_path(asset_path),
                                 std::ios::binary | std::ios::trunc);
            if (!stream)
                return false;
            stream << document.dump(2);
            return static_cast<bool>(stream);
        }

        bool migrate_cooking_overrides_to_sidecars(const std::string& project_document_path,
                                                   std::vector<std::string>& out_migrated,
                                                   std::vector<std::string>& out_dropped)
        {
            out_migrated.clear();
            out_dropped.clear();
            if (project_document_path.empty())
                return true;

            json document;
            {
                std::ifstream stream(project_document_path, std::ios::binary);
                if (!stream)
                    return true; // A project that never saved a cooking document has nothing.
                try
                {
                    stream >> document;
                }
                catch (const json::exception&)
                {
                    return false;
                }
            }

            // The absent key is the second run, and every run after it. Leaving the document
            // untouched here is what keeps the migration from rewriting a file per session.
            if (!document.is_object() || !document.contains("overrides") ||
                !document["overrides"].is_object())
                return true;

            for (const auto& entry : document["overrides"].items())
            {
                const std::string& asset_path = entry.key();
                // The error-code overload, because a key the operating system will not even
                // accept as a path is a dead override like any other, not an exception.
                std::error_code path_error;
                if (!std::filesystem::exists(asset_path, path_error))
                {
                    out_dropped.push_back(asset_path);
                    continue;
                }

                // Read first, so the move overwrites the asset's cooking override and nothing
                // else it already says about how it is imported.
                ModelImportSettings settings;
                if (!load_model_import_settings(asset_path, settings))
                    return false;
                try
                {
                    settings.cooking = cooking_override_from_json(entry.value());
                }
                catch (const json::exception&)
                {
                    return false;
                }
                if (!save_model_import_settings(asset_path, settings))
                    return false;
                out_migrated.push_back(asset_path);
            }

            document.erase("overrides");
            std::ofstream stream(project_document_path, std::ios::binary | std::ios::trunc);
            if (!stream)
                return false;
            stream << document.dump(2);
            return static_cast<bool>(stream);
        }
    } // namespace Model
} // namespace SushiEngine
