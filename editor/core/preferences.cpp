/**************************************************************************/
/* preferences.cpp                                                        */
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

// std::getenv is the portable way to read the per-user config dir; MSVC's CRT flags
// it as deprecated in favour of a non-standard alternative, so silence that here.
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "preferences.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            namespace fs = std::filesystem;

            // The per-user config directory the store writes into. Mirrors the platform
            // convention so the file lands where a user expects app settings, and stays
            // separate from the build-tool config under cli/.
            fs::path user_config_dir()
            {
    #if defined(_WIN32)
                if (const char* appdata = std::getenv("APPDATA"))
                    return fs::path(appdata) / "SushiEngine";
                return fs::path(".") / "SushiEngine";
    #else
                if (const char* xdg = std::getenv("XDG_CONFIG_HOME"))
                    return fs::path(xdg) / "SushiEngine";
                if (const char* home = std::getenv("HOME"))
                    return fs::path(home) / ".config" / "SushiEngine";
                return fs::path(".") / "SushiEngine";
    #endif
            }

            const char* to_string(ScalarPrecision precision) noexcept
            {
                return precision == ScalarPrecision::Double ? "double" : "single";
            }

            ScalarPrecision precision_from(const std::string& value) noexcept
            {
                return value == "double" ? ScalarPrecision::Double : ScalarPrecision::Single;
            }

            const char* to_string(EditorTheme theme) noexcept
            {
                switch (theme)
                {
                    case EditorTheme::Light:   return "light";
                    case EditorTheme::Classic: return "classic";
                    case EditorTheme::Dark:    return "dark";
                }
                return "dark";
            }

            EditorTheme theme_from(const std::string& value) noexcept
            {
                if (value == "light")
                    return EditorTheme::Light;
                if (value == "classic")
                    return EditorTheme::Classic;
                return EditorTheme::Dark;
            }

            /**
             * @brief JSON-file implementation of the preferences store.
             *
             * Serializes @ref Preferences to a single object at a fixed path. Reads are
             * tolerant: a missing file yields defaults, and missing or malformed fields
             * fall back to the default value rather than throwing, so a partial or older
             * file still loads.
             */
            class JsonPreferencesStore final : public IPreferencesStore
            {
                public:
                    explicit JsonPreferencesStore(fs::path path) : path_(std::move(path)) {}

                    Preferences load() override
                    {
                        Preferences preferences;
                        preferences.precision = current_precision();

                        std::ifstream input(path_);
                        if (!input.is_open())
                            return preferences;

                        nlohmann::json json;
                        input >> std::noskipws;
                        try
                        {
                            json = nlohmann::json::parse(input, nullptr, false);
                        }
                        catch (const nlohmann::json::exception&)
                        {
                            return preferences;
                        }
                        if (json.is_discarded() || !json.is_object())
                            return preferences;

                        preferences.precision =
                            precision_from(json.value("precision", to_string(preferences.precision)));
                        preferences.theme =
                            theme_from(json.value("theme", to_string(preferences.theme)));
                        preferences.grid_visible = json.value("grid_visible", preferences.grid_visible);
                        preferences.camera_move_speed =
                            json.value("camera_move_speed", preferences.camera_move_speed);
                        preferences.snap_enabled = json.value("snap_enabled", preferences.snap_enabled);
                        preferences.snap_translate =
                            json.value("snap_translate", preferences.snap_translate);
                        preferences.snap_rotate_degrees =
                            json.value("snap_rotate_degrees", preferences.snap_rotate_degrees);
                        preferences.snap_scale = json.value("snap_scale", preferences.snap_scale);
                        preferences.autosave = json.value("autosave", preferences.autosave);
                        preferences.last_project_root =
                            json.value("last_project_root", preferences.last_project_root);
                        if (json.contains("recent_scenes") && json["recent_scenes"].is_array())
                            preferences.recent_scenes =
                                json["recent_scenes"].get<std::vector<std::string>>();
                        return preferences;
                    }

                    bool save(const Preferences& preferences) override
                    {
                        std::error_code error;
                        fs::create_directories(path_.parent_path(), error);

                        nlohmann::json json;
                        json["precision"] = to_string(preferences.precision);
                        json["theme"] = to_string(preferences.theme);
                        json["grid_visible"] = preferences.grid_visible;
                        json["camera_move_speed"] = preferences.camera_move_speed;
                        json["snap_enabled"] = preferences.snap_enabled;
                        json["snap_translate"] = preferences.snap_translate;
                        json["snap_rotate_degrees"] = preferences.snap_rotate_degrees;
                        json["snap_scale"] = preferences.snap_scale;
                        json["autosave"] = preferences.autosave;
                        json["recent_scenes"] = preferences.recent_scenes;
                        json["last_project_root"] = preferences.last_project_root;

                        std::ofstream output(path_, std::ios::trunc);
                        if (!output.is_open())
                            return false;
                        output << json.dump(2) << '\n';
                        return output.good();
                    }

                    std::string path() const override { return path_.string(); }

                private:
                    fs::path path_;
            };
        } // namespace

        std::unique_ptr<IPreferencesStore> create_preferences_store()
        {
            return std::make_unique<JsonPreferencesStore>(user_config_dir() / "preferences.json");
        }
    } // namespace Editor
} // namespace SushiEngine
