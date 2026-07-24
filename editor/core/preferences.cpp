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
                          {"max_decals", s.lights.max_decals},
                          {"stochastic_shadows", s.lights.stochastic_shadows},
                          {"stochastic_distance", s.lights.stochastic_distance},
                          {"stochastic_softness", s.lights.stochastic_softness}}},
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
                          {"intensity", s.ssr.intensity}}},
                    {"post",
                     json{{"exposure_mode", static_cast<std::uint32_t>(s.post.exposure_mode)},
                          {"exposure_compensation", s.post.exposure_compensation},
                          {"tonemap", static_cast<std::uint32_t>(s.post.tonemap)},
                          {"auto_exposure",
                           json{{"min_ev", s.post.auto_exposure.min_ev},
                                {"max_ev", s.post.auto_exposure.max_ev},
                                {"compensation", s.post.auto_exposure.compensation},
                                {"speed_up", s.post.auto_exposure.speed_up},
                                {"speed_down", s.post.auto_exposure.speed_down},
                                {"key", s.post.auto_exposure.key},
                                {"low_percent", s.post.auto_exposure.low_percent},
                                {"high_percent", s.post.auto_exposure.high_percent}}},
                          {"bloom",
                           json{{"enabled", s.post.bloom.enabled},
                                {"intensity", s.post.bloom.intensity},
                                {"threshold", s.post.bloom.threshold},
                                {"threshold_knee", s.post.bloom.threshold_knee}}},
                          {"grade",
                           json{{"temperature", s.post.grade.temperature},
                                {"tint", s.post.grade.tint},
                                {"contrast", s.post.grade.contrast},
                                {"saturation", s.post.grade.saturation},
                                {"lift", {s.post.grade.lift[0], s.post.grade.lift[1],
                                          s.post.grade.lift[2]}},
                                {"gamma", {s.post.grade.gamma[0], s.post.grade.gamma[1],
                                           s.post.grade.gamma[2]}},
                                {"gain", {s.post.grade.gain[0], s.post.grade.gain[1],
                                          s.post.grade.gain[2]}}}},
                          {"depth_of_field",
                           json{{"enabled", s.post.depth_of_field.enabled},
                                {"focus_distance", s.post.depth_of_field.focus_distance},
                                {"focus_range", s.post.depth_of_field.focus_range},
                                {"aperture", s.post.depth_of_field.aperture},
                                {"max_radius", s.post.depth_of_field.max_radius}}},
                          {"motion_blur",
                           json{{"enabled", s.post.motion_blur.enabled},
                                {"intensity", s.post.motion_blur.intensity},
                                {"samples", s.post.motion_blur.samples}}},
                          {"vignette", s.post.vignette},
                          {"chromatic_aberration", s.post.chromatic_aberration},
                          {"film_grain", s.post.film_grain}}},
                    {"gpu_culling",
                     json{{"enabled", s.gpu_culling.enabled},
                          {"occlusion", s.gpu_culling.occlusion},
                          {"frustum", s.gpu_culling.frustum},
                          {"min_screen_diameter", s.gpu_culling.min_screen_diameter},
                          {"freeze", s.gpu_culling.freeze},
                          {"show_statistics", s.gpu_culling.show_statistics}}},
                    {"delivery",
                     json{{"async_compute", s.delivery.async_compute},
                          {"frames_in_flight", s.delivery.frames_in_flight},
                          {"present_mode",
                           static_cast<std::uint32_t>(s.delivery.present_mode)}}}};
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
                    s.lights.stochastic_shadows =
                        j.value("stochastic_shadows", s.lights.stochastic_shadows);
                    s.lights.stochastic_distance =
                        j.value("stochastic_distance", s.lights.stochastic_distance);
                    s.lights.stochastic_softness =
                        j.value("stochastic_softness", s.lights.stochastic_softness);
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
                if (json.contains("post") && json["post"].is_object())
                {
                    const nlohmann::json& j = json["post"];
                    s.post.exposure_mode = static_cast<SushiEngine::Render::ExposureMode>(
                        j.value("exposure_mode", static_cast<std::uint32_t>(s.post.exposure_mode)));
                    s.post.exposure_compensation =
                        j.value("exposure_compensation", s.post.exposure_compensation);
                    s.post.tonemap = static_cast<SushiEngine::Render::TonemapOperator>(
                        j.value("tonemap", static_cast<std::uint32_t>(s.post.tonemap)));
                    if (j.contains("auto_exposure") && j["auto_exposure"].is_object())
                    {
                        const nlohmann::json& a = j["auto_exposure"];
                        s.post.auto_exposure.min_ev = a.value("min_ev", s.post.auto_exposure.min_ev);
                        s.post.auto_exposure.max_ev = a.value("max_ev", s.post.auto_exposure.max_ev);
                        s.post.auto_exposure.compensation =
                            a.value("compensation", s.post.auto_exposure.compensation);
                        s.post.auto_exposure.speed_up =
                            a.value("speed_up", s.post.auto_exposure.speed_up);
                        s.post.auto_exposure.speed_down =
                            a.value("speed_down", s.post.auto_exposure.speed_down);
                        s.post.auto_exposure.key = a.value("key", s.post.auto_exposure.key);
                        s.post.auto_exposure.low_percent =
                            a.value("low_percent", s.post.auto_exposure.low_percent);
                        s.post.auto_exposure.high_percent =
                            a.value("high_percent", s.post.auto_exposure.high_percent);
                    }
                    if (j.contains("bloom") && j["bloom"].is_object())
                    {
                        const nlohmann::json& b = j["bloom"];
                        s.post.bloom.enabled = b.value("enabled", s.post.bloom.enabled);
                        s.post.bloom.intensity = b.value("intensity", s.post.bloom.intensity);
                        s.post.bloom.threshold = b.value("threshold", s.post.bloom.threshold);
                        s.post.bloom.threshold_knee =
                            b.value("threshold_knee", s.post.bloom.threshold_knee);
                    }
                    if (j.contains("grade") && j["grade"].is_object())
                    {
                        const nlohmann::json& g = j["grade"];
                        s.post.grade.temperature = g.value("temperature", s.post.grade.temperature);
                        s.post.grade.tint = g.value("tint", s.post.grade.tint);
                        s.post.grade.contrast = g.value("contrast", s.post.grade.contrast);
                        s.post.grade.saturation = g.value("saturation", s.post.grade.saturation);
                        if (g.contains("lift") && g["lift"].is_array() && g["lift"].size() == 3)
                            for (int k = 0; k < 3; ++k)
                                s.post.grade.lift[k] = g["lift"][k].get<float>();
                        if (g.contains("gamma") && g["gamma"].is_array() && g["gamma"].size() == 3)
                            for (int k = 0; k < 3; ++k)
                                s.post.grade.gamma[k] = g["gamma"][k].get<float>();
                        if (g.contains("gain") && g["gain"].is_array() && g["gain"].size() == 3)
                            for (int k = 0; k < 3; ++k)
                                s.post.grade.gain[k] = g["gain"][k].get<float>();
                    }
                    if (j.contains("depth_of_field") && j["depth_of_field"].is_object())
                    {
                        const nlohmann::json& d = j["depth_of_field"];
                        s.post.depth_of_field.enabled =
                            d.value("enabled", s.post.depth_of_field.enabled);
                        s.post.depth_of_field.focus_distance =
                            d.value("focus_distance", s.post.depth_of_field.focus_distance);
                        s.post.depth_of_field.focus_range =
                            d.value("focus_range", s.post.depth_of_field.focus_range);
                        s.post.depth_of_field.aperture =
                            d.value("aperture", s.post.depth_of_field.aperture);
                        s.post.depth_of_field.max_radius =
                            d.value("max_radius", s.post.depth_of_field.max_radius);
                    }
                    if (j.contains("motion_blur") && j["motion_blur"].is_object())
                    {
                        const nlohmann::json& m = j["motion_blur"];
                        s.post.motion_blur.enabled = m.value("enabled", s.post.motion_blur.enabled);
                        s.post.motion_blur.intensity =
                            m.value("intensity", s.post.motion_blur.intensity);
                        s.post.motion_blur.samples = m.value("samples", s.post.motion_blur.samples);
                    }
                    s.post.vignette = j.value("vignette", s.post.vignette);
                    s.post.chromatic_aberration =
                        j.value("chromatic_aberration", s.post.chromatic_aberration);
                    s.post.film_grain = j.value("film_grain", s.post.film_grain);
                }
                if (json.contains("gpu_culling") && json["gpu_culling"].is_object())
                {
                    const nlohmann::json& j = json["gpu_culling"];
                    s.gpu_culling.enabled = j.value("enabled", s.gpu_culling.enabled);
                    s.gpu_culling.occlusion = j.value("occlusion", s.gpu_culling.occlusion);
                    s.gpu_culling.frustum = j.value("frustum", s.gpu_culling.frustum);
                    s.gpu_culling.min_screen_diameter =
                        j.value("min_screen_diameter", s.gpu_culling.min_screen_diameter);
                    s.gpu_culling.freeze = j.value("freeze", s.gpu_culling.freeze);
                    s.gpu_culling.show_statistics =
                        j.value("show_statistics", s.gpu_culling.show_statistics);
                }
                if (json.contains("delivery") && json["delivery"].is_object())
                {
                    const nlohmann::json& j = json["delivery"];
                    s.delivery.async_compute = j.value("async_compute", s.delivery.async_compute);
                    s.delivery.frames_in_flight =
                        j.value("frames_in_flight", s.delivery.frames_in_flight);
                    s.delivery.present_mode = static_cast<SushiEngine::Render::PresentMode>(
                        j.value("present_mode",
                                static_cast<std::uint32_t>(s.delivery.present_mode)));
                }
                return s;
            }

            nlohmann::json vec3_to_json(const SushiEngine::Vector3& v)
            {
                return nlohmann::json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
            }

            SushiEngine::Vector3 vec3_from_json(const nlohmann::json& j)
            {
                SushiEngine::Vector3 v;
                v.x = j.value("x", SushiEngine::Scalar(0));
                v.y = j.value("y", SushiEngine::Scalar(0));
                v.z = j.value("z", SushiEngine::Scalar(0));
                return v;
            }

            // The Environment fields the Environment/Lighting panels edit. This is a host
            // (editor) setting, not scene data (see Preferences::environment), so it is
            // serialized independently of the scene file's own environment block.
            // `bodies`/`sky_stars`/`dominant_*` are excluded: the ephemeris repopulates
            // those every frame from SceneSkyState and are not author state.
            nlohmann::json environment_to_json(const SushiEngine::Render::Environment& e)
            {
                using nlohmann::json;
                json decks = json::array();
                for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS; ++i)
                {
                    const SushiEngine::Render::CloudDeck& d = e.clouds.decks[i];
                    decks.push_back(json{{"enabled", d.enabled},
                                          {"genus", static_cast<std::uint32_t>(d.genus)},
                                          {"coverage_bias", d.coverage_bias},
                                          {"density_scale", d.density_scale}});
                }

                return json{
                    {"sun",
                     json{{"direction", vec3_to_json(e.sun.direction)},
                          {"color", vec3_to_json(e.sun.color)},
                          {"intensity", e.sun.intensity}}},
                    {"planet_surface_style", static_cast<std::uint32_t>(e.planet_surface_style)},
                    {"atmosphere",
                     json{{"enabled", e.atmosphere.enabled},
                          {"height", e.atmosphere.height},
                          {"rayleigh_coefficient", vec3_to_json(e.atmosphere.rayleigh_coefficient)},
                          {"mie_coefficient", e.atmosphere.mie_coefficient},
                          {"mie_anisotropy", e.atmosphere.mie_anisotropy},
                          {"rayleigh_scale_height", e.atmosphere.rayleigh_scale_height},
                          {"mie_scale_height", e.atmosphere.mie_scale_height}}},
                    {"surface",
                     json{{"ground_albedo", vec3_to_json(e.surface.ground_albedo)},
                          {"ocean_color", vec3_to_json(e.surface.ocean_color)},
                          {"roughness", e.surface.roughness}}},
                    {"clouds",
                     json{{"enabled", e.clouds.enabled},
                          {"light_absorption", e.clouds.light_absorption},
                          {"forward_scattering", e.clouds.forward_scattering},
                          {"powder_strength", e.clouds.powder_strength},
                          {"ambient_strength", e.clouds.ambient_strength},
                          {"ground_shadow_strength", e.clouds.ground_shadow_strength},
                          {"weather_scale", e.clouds.weather_scale},
                          {"evolution_rate", e.clouds.evolution_rate},
                          {"decks", decks}}},
                    {"stars",
                     json{{"enabled", e.stars.enabled},
                          {"brightness", e.stars.brightness},
                          {"density", e.stars.density}}},
                    {"night",
                     json{{"enabled", e.night.enabled},
                          {"reflected_intensity", e.night.reflected_intensity},
                          {"star_intensity", e.night.star_intensity}}},
                    {"ambient", vec3_to_json(e.ambient)},
                    {"exposure", e.exposure},
                    {"image_based_lighting", e.image_based_lighting},
                    {"ibl_intensity", e.ibl_intensity}};
            }

            // Applies the persisted fields onto a default-constructed Environment, leaving
            // every field the JSON omits (including the ephemeris-owned fields never
            // written above) at its default.
            SushiEngine::Render::Environment environment_from_json(const nlohmann::json& j)
            {
                SushiEngine::Render::Environment e;
                if (!j.is_object())
                    return e;

                if (j.contains("sun") && j["sun"].is_object())
                {
                    const nlohmann::json& s = j["sun"];
                    if (s.contains("direction"))
                        e.sun.direction = vec3_from_json(s["direction"]);
                    if (s.contains("color"))
                        e.sun.color = vec3_from_json(s["color"]);
                    e.sun.intensity = s.value("intensity", e.sun.intensity);
                }
                e.planet_surface_style = static_cast<SushiEngine::Render::SurfaceStyle>(
                    j.value("planet_surface_style",
                            static_cast<std::uint32_t>(e.planet_surface_style)));
                if (j.contains("atmosphere") && j["atmosphere"].is_object())
                {
                    const nlohmann::json& a = j["atmosphere"];
                    e.atmosphere.enabled = a.value("enabled", e.atmosphere.enabled);
                    e.atmosphere.height = a.value("height", e.atmosphere.height);
                    if (a.contains("rayleigh_coefficient"))
                        e.atmosphere.rayleigh_coefficient = vec3_from_json(a["rayleigh_coefficient"]);
                    e.atmosphere.mie_coefficient =
                        a.value("mie_coefficient", e.atmosphere.mie_coefficient);
                    e.atmosphere.mie_anisotropy =
                        a.value("mie_anisotropy", e.atmosphere.mie_anisotropy);
                    e.atmosphere.rayleigh_scale_height =
                        a.value("rayleigh_scale_height", e.atmosphere.rayleigh_scale_height);
                    e.atmosphere.mie_scale_height =
                        a.value("mie_scale_height", e.atmosphere.mie_scale_height);
                }
                if (j.contains("surface") && j["surface"].is_object())
                {
                    const nlohmann::json& s = j["surface"];
                    if (s.contains("ground_albedo"))
                        e.surface.ground_albedo = vec3_from_json(s["ground_albedo"]);
                    if (s.contains("ocean_color"))
                        e.surface.ocean_color = vec3_from_json(s["ocean_color"]);
                    e.surface.roughness = s.value("roughness", e.surface.roughness);
                }
                if (j.contains("clouds") && j["clouds"].is_object())
                {
                    const nlohmann::json& c = j["clouds"];
                    e.clouds.enabled = c.value("enabled", e.clouds.enabled);
                    e.clouds.light_absorption = c.value("light_absorption", e.clouds.light_absorption);
                    e.clouds.forward_scattering =
                        c.value("forward_scattering", e.clouds.forward_scattering);
                    e.clouds.powder_strength = c.value("powder_strength", e.clouds.powder_strength);
                    e.clouds.ambient_strength = c.value("ambient_strength", e.clouds.ambient_strength);
                    e.clouds.ground_shadow_strength =
                        c.value("ground_shadow_strength", e.clouds.ground_shadow_strength);
                    e.clouds.weather_scale = c.value("weather_scale", e.clouds.weather_scale);
                    e.clouds.evolution_rate = c.value("evolution_rate", e.clouds.evolution_rate);
                    if (c.contains("decks") && c["decks"].is_array())
                    {
                        const nlohmann::json& decks = c["decks"];
                        for (int i = 0;
                             i < SushiEngine::Render::CLOUD_MAX_DECKS &&
                             static_cast<std::size_t>(i) < decks.size();
                             ++i)
                        {
                            const nlohmann::json& d = decks[static_cast<std::size_t>(i)];
                            SushiEngine::Render::CloudDeck& deck = e.clouds.decks[i];
                            deck.enabled = d.value("enabled", deck.enabled);
                            deck.genus = static_cast<SushiEngine::Render::CloudGenus>(
                                d.value("genus", static_cast<std::uint32_t>(deck.genus)));
                            deck.coverage_bias = d.value("coverage_bias", deck.coverage_bias);
                            deck.density_scale = d.value("density_scale", deck.density_scale);
                        }
                    }
                }
                if (j.contains("stars") && j["stars"].is_object())
                {
                    const nlohmann::json& s = j["stars"];
                    e.stars.enabled = s.value("enabled", e.stars.enabled);
                    e.stars.brightness = s.value("brightness", e.stars.brightness);
                    e.stars.density = s.value("density", e.stars.density);
                }
                if (j.contains("night") && j["night"].is_object())
                {
                    const nlohmann::json& n = j["night"];
                    e.night.enabled = n.value("enabled", e.night.enabled);
                    e.night.reflected_intensity =
                        n.value("reflected_intensity", e.night.reflected_intensity);
                    e.night.star_intensity = n.value("star_intensity", e.night.star_intensity);
                }
                if (j.contains("ambient"))
                    e.ambient = vec3_from_json(j["ambient"]);
                e.exposure = j.value("exposure", e.exposure);
                e.image_based_lighting = j.value("image_based_lighting", e.image_based_lighting);
                e.ibl_intensity = j.value("ibl_intensity", e.ibl_intensity);
                return e;
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
                        if (json.contains("environment"))
                            preferences.environment = environment_from_json(json["environment"]);
                        // Input bindings are nested as a real object; hold their dumped text so
                        // the Preferences struct stays free of the JSON dependency.
                        if (json.contains("input_bindings") && json["input_bindings"].is_object())
                            preferences.input_bindings = json["input_bindings"].dump();
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
                        json["environment"] = environment_to_json(preferences.environment);
                        // Re-nest the bindings blob as an object when it parses; a corrupt or empty
                        // blob is simply omitted so the next load falls back to defaults.
                        if (!preferences.input_bindings.empty())
                        {
                            nlohmann::json bindings =
                                nlohmann::json::parse(preferences.input_bindings, nullptr, false);
                            if (bindings.is_object())
                                json["input_bindings"] = bindings;
                        }

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
