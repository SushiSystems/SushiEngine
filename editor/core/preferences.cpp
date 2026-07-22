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

            // RenderSettings is plain, trivially-copyable data (see render_settings.hpp), so
            // this mirrors its fields one-for-one; reads are tolerant the same way the rest
            // of this store is, so a partial or older file still loads with defaults filling
            // in whatever is missing.
            nlohmann::json render_settings_to_json(const SushiEngine::Render::RenderSettings& s)
            {
                using nlohmann::json;
                return json{
                    {"quality", static_cast<std::uint32_t>(s.quality)},
                    {"anti_aliasing", static_cast<std::uint32_t>(s.anti_aliasing)},
                    {"upscale", static_cast<std::uint32_t>(s.upscale)},
                    {"render_scale", s.render_scale},
                    {"shadows",
                     json{{"enabled", s.shadows.enabled},
                          {"cascade_count", s.shadows.cascade_count},
                          {"resolution", s.shadows.resolution},
                          {"distance", s.shadows.distance},
                          {"split_blend", s.shadows.split_blend},
                          {"normal_bias", s.shadows.normal_bias},
                          {"depth_bias", s.shadows.depth_bias},
                          {"filter_radius", s.shadows.filter_radius},
                          {"max_filter_radius", s.shadows.max_filter_radius},
                          {"softness", s.shadows.softness},
                          {"cascade_blend", s.shadows.cascade_blend},
                          {"contact_shadows", s.shadows.contact_shadows},
                          {"contact_distance", s.shadows.contact_distance},
                          {"contact_steps", s.shadows.contact_steps},
                          {"ray_traced", s.shadows.ray_traced}}},
                    {"temporal",
                     json{{"feedback_still", s.temporal.feedback_still},
                          {"feedback_moving", s.temporal.feedback_moving},
                          {"sharpness", s.temporal.sharpness},
                          {"jitter_scale", s.temporal.jitter_scale},
                          {"clamp_history", s.temporal.clamp_history}}},
                    {"dynamic_resolution",
                     json{{"enabled", s.dynamic_resolution.enabled},
                          {"target_milliseconds", s.dynamic_resolution.target_milliseconds},
                          {"minimum_scale", s.dynamic_resolution.minimum_scale},
                          {"maximum_scale", s.dynamic_resolution.maximum_scale}}},
                    {"variable_rate_shading",
                     json{{"enabled", s.variable_rate_shading.enabled},
                          {"luminance_threshold", s.variable_rate_shading.luminance_threshold},
                          {"velocity_threshold", s.variable_rate_shading.velocity_threshold}}},
                    {"lights",
                     json{{"max_lights", s.lights.max_lights},
                          {"cluster_far_distance", s.lights.cluster_far_distance},
                          {"shadow_atlas_size", s.lights.shadow_atlas_size},
                          {"max_shadow_casters", s.lights.max_shadow_casters},
                          {"max_decals", s.lights.max_decals}}},
                    {"gtao",
                     json{{"enabled", s.gtao.enabled},
                          {"radius", s.gtao.radius},
                          {"intensity", s.gtao.intensity},
                          {"power", s.gtao.power},
                          {"slices", s.gtao.slices},
                          {"steps", s.gtao.steps}}},
                    {"ssr",
                     json{{"enabled", s.ssr.enabled},
                          {"max_steps", s.ssr.max_steps},
                          {"thickness", s.ssr.thickness},
                          {"roughness_cutoff", s.ssr.roughness_cutoff},
                          {"intensity", s.ssr.intensity}}}};
            }

            SushiEngine::Render::RenderSettings render_settings_from_json(const nlohmann::json& json)
            {
                SushiEngine::Render::RenderSettings s;
                if (!json.is_object())
                    return s;

                s.quality = static_cast<SushiEngine::Render::RenderQuality>(
                    json.value("quality", static_cast<std::uint32_t>(s.quality)));
                s.anti_aliasing = static_cast<SushiEngine::Render::AntiAliasingMode>(
                    json.value("anti_aliasing", static_cast<std::uint32_t>(s.anti_aliasing)));
                s.upscale = static_cast<SushiEngine::Render::UpscaleMode>(
                    json.value("upscale", static_cast<std::uint32_t>(s.upscale)));
                s.render_scale = json.value("render_scale", s.render_scale);

                if (json.contains("shadows") && json["shadows"].is_object())
                {
                    const nlohmann::json& j = json["shadows"];
                    s.shadows.enabled = j.value("enabled", s.shadows.enabled);
                    s.shadows.cascade_count = j.value("cascade_count", s.shadows.cascade_count);
                    s.shadows.resolution = j.value("resolution", s.shadows.resolution);
                    s.shadows.distance = j.value("distance", s.shadows.distance);
                    s.shadows.split_blend = j.value("split_blend", s.shadows.split_blend);
                    s.shadows.normal_bias = j.value("normal_bias", s.shadows.normal_bias);
                    s.shadows.depth_bias = j.value("depth_bias", s.shadows.depth_bias);
                    s.shadows.filter_radius = j.value("filter_radius", s.shadows.filter_radius);
                    s.shadows.max_filter_radius =
                        j.value("max_filter_radius", s.shadows.max_filter_radius);
                    s.shadows.softness = j.value("softness", s.shadows.softness);
                    s.shadows.cascade_blend = j.value("cascade_blend", s.shadows.cascade_blend);
                    s.shadows.contact_shadows =
                        j.value("contact_shadows", s.shadows.contact_shadows);
                    s.shadows.contact_distance =
                        j.value("contact_distance", s.shadows.contact_distance);
                    s.shadows.contact_steps = j.value("contact_steps", s.shadows.contact_steps);
                    s.shadows.ray_traced = j.value("ray_traced", s.shadows.ray_traced);
                }
                if (json.contains("temporal") && json["temporal"].is_object())
                {
                    const nlohmann::json& j = json["temporal"];
                    s.temporal.feedback_still = j.value("feedback_still", s.temporal.feedback_still);
                    s.temporal.feedback_moving =
                        j.value("feedback_moving", s.temporal.feedback_moving);
                    s.temporal.sharpness = j.value("sharpness", s.temporal.sharpness);
                    s.temporal.jitter_scale = j.value("jitter_scale", s.temporal.jitter_scale);
                    s.temporal.clamp_history = j.value("clamp_history", s.temporal.clamp_history);
                }
                if (json.contains("dynamic_resolution") && json["dynamic_resolution"].is_object())
                {
                    const nlohmann::json& j = json["dynamic_resolution"];
                    s.dynamic_resolution.enabled = j.value("enabled", s.dynamic_resolution.enabled);
                    s.dynamic_resolution.target_milliseconds =
                        j.value("target_milliseconds", s.dynamic_resolution.target_milliseconds);
                    s.dynamic_resolution.minimum_scale =
                        j.value("minimum_scale", s.dynamic_resolution.minimum_scale);
                    s.dynamic_resolution.maximum_scale =
                        j.value("maximum_scale", s.dynamic_resolution.maximum_scale);
                }
                if (json.contains("variable_rate_shading") &&
                    json["variable_rate_shading"].is_object())
                {
                    const nlohmann::json& j = json["variable_rate_shading"];
                    s.variable_rate_shading.enabled =
                        j.value("enabled", s.variable_rate_shading.enabled);
                    s.variable_rate_shading.luminance_threshold =
                        j.value("luminance_threshold", s.variable_rate_shading.luminance_threshold);
                    s.variable_rate_shading.velocity_threshold =
                        j.value("velocity_threshold", s.variable_rate_shading.velocity_threshold);
                }
                if (json.contains("lights") && json["lights"].is_object())
                {
                    const nlohmann::json& j = json["lights"];
                    s.lights.max_lights = j.value("max_lights", s.lights.max_lights);
                    s.lights.cluster_far_distance =
                        j.value("cluster_far_distance", s.lights.cluster_far_distance);
                    s.lights.shadow_atlas_size =
                        j.value("shadow_atlas_size", s.lights.shadow_atlas_size);
                    s.lights.max_shadow_casters =
                        j.value("max_shadow_casters", s.lights.max_shadow_casters);
                    s.lights.max_decals = j.value("max_decals", s.lights.max_decals);
                }
                if (json.contains("gtao") && json["gtao"].is_object())
                {
                    const nlohmann::json& j = json["gtao"];
                    s.gtao.enabled = j.value("enabled", s.gtao.enabled);
                    s.gtao.radius = j.value("radius", s.gtao.radius);
                    s.gtao.intensity = j.value("intensity", s.gtao.intensity);
                    s.gtao.power = j.value("power", s.gtao.power);
                    s.gtao.slices = j.value("slices", s.gtao.slices);
                    s.gtao.steps = j.value("steps", s.gtao.steps);
                }
                if (json.contains("ssr") && json["ssr"].is_object())
                {
                    const nlohmann::json& j = json["ssr"];
                    s.ssr.enabled = j.value("enabled", s.ssr.enabled);
                    s.ssr.max_steps = j.value("max_steps", s.ssr.max_steps);
                    s.ssr.thickness = j.value("thickness", s.ssr.thickness);
                    s.ssr.roughness_cutoff = j.value("roughness_cutoff", s.ssr.roughness_cutoff);
                    s.ssr.intensity = j.value("intensity", s.ssr.intensity);
                }
                return s;
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
                        if (json.contains("render_settings"))
                            preferences.render_settings =
                                render_settings_from_json(json["render_settings"]);
                        return preferences;
                    }

                    bool save(const Preferences& preferences) override
                    {
                        std::error_code error;
                        fs::create_directories(path_.parent_path(), error);

                        nlohmann::json json;
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
                        json["render_settings"] = render_settings_to_json(preferences.render_settings);

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
