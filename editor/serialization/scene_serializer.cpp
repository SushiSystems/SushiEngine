/**************************************************************************/
/* scene_serializer.cpp                                                   */
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

#include "scene_serializer.hpp"

#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "effect_serializer.hpp"
#include "environment_serializer.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using nlohmann::json;
            using SushiEngine::Simulation::EntityId;
            using SushiEngine::Simulation::IWorldEditor;
            using SushiEngine::Simulation::NULL_ENTITY;

            json vec3_to_json(const SushiEngine::Vector3& v)
            {
                return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
            }

            SushiEngine::Vector3 vec3_from_json(const json& j)
            {
                SushiEngine::Vector3 v;
                v.x = j.value("x", SushiEngine::Scalar(0));
                v.y = j.value("y", SushiEngine::Scalar(0));
                v.z = j.value("z", SushiEngine::Scalar(0));
                return v;
            }

            json quaternion_to_json(const SushiEngine::Quaternion& q)
            {
                return json{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}};
            }

            json sky_to_json(const SceneSkyState& sky)
            {
                return json{{"enabled", sky.enabled},
                            {"year", sky.date.year},
                            {"month", sky.date.month},
                            {"day", sky.date.day},
                            {"hour", sky.date.hour},
                            {"minute", sky.date.minute},
                            {"second", sky.date.second},
                            {"latitude_degrees", sky.latitude_degrees},
                            {"longitude_degrees", sky.longitude_degrees},
                            {"astronomical_sun", sky.astronomical_sun},
                            {"animate", sky.animate},
                            {"days_per_second", sky.days_per_second},
                            {"accumulated_days", sky.accumulated_days}};
            }

            SceneSkyState sky_from_json(const json& j, SceneSkyState sky)
            {
                if (!j.is_object())
                    return sky;
                sky.enabled = j.value("enabled", sky.enabled);
                sky.date.year = j.value("year", sky.date.year);
                sky.date.month = j.value("month", sky.date.month);
                sky.date.day = j.value("day", sky.date.day);
                sky.date.hour = j.value("hour", sky.date.hour);
                sky.date.minute = j.value("minute", sky.date.minute);
                sky.date.second = j.value("second", sky.date.second);
                sky.latitude_degrees = j.value("latitude_degrees", sky.latitude_degrees);
                sky.longitude_degrees = j.value("longitude_degrees", sky.longitude_degrees);
                sky.astronomical_sun = j.value("astronomical_sun", sky.astronomical_sun);
                sky.animate = j.value("animate", sky.animate);
                sky.days_per_second = j.value("days_per_second", sky.days_per_second);
                sky.accumulated_days = j.value("accumulated_days", sky.accumulated_days);
                return sky;
            }

            /**
             * @brief Where the atmosphere's prognostic field lives for a given scene file.
             *
             * Appended rather than extension-substituted so the mapping is total: every scene
             * path, whatever it is named and whether or not it has an extension at all, has
             * exactly one sidecar path and no two scenes share one.
             */
            std::string atmosphere_sidecar_path(const std::string& scene_path)
            {
                return scene_path + ".atmos";
            }

            /**
             * @brief W4's procedural weather state, as of the Phase C swap.
             *
             * **The system list is gone because the systems are gone.** What used to be written
             * here was a dozen ellipses with headings and radii — an object model that could be
             * spelled in JSON because it was authored data pretending to be weather. The global
             * core has no such objects: its state is two potential-vorticity fields and a
             * moisture field on a 512x256 grid, several megabytes of numbers that mean nothing
             * individually. Writing that into the scene JSON would bloat a human-editable text
             * file past the point of being human-editable, for a payload no human would ever
             * edit, so it goes to a binary sidecar beside the scene and the JSON keeps only the
             * fact that one exists.
             *
             * T2's regional grid is still deliberately not captured: it reseeds from the
             * restored parent solution and the current observer on the next tick. That was a
             * named simplification before and it remains one, but it is a much smaller claim
             * now — the parent it reseeds from is the *actual* restored flow rather than a
             * reconstruction from a system list.
             */
            json weather_to_json(IWorldEditor& world, const std::string& scene_path)
            {
                json j;
                j["procedural_enabled"] = world.procedural_weather_enabled();
                SushiEngine::Simulation::IWeatherAuthoring* weather = world.weather_authoring();
                if (weather == nullptr)
                    return j;

                const std::vector<std::uint8_t> blob = weather->capture_state();
                if (blob.empty())
                    return j;

                const std::string sidecar = atmosphere_sidecar_path(scene_path);
                std::ofstream out(sidecar, std::ios::binary);
                if (!out)
                    return j;
                out.write(reinterpret_cast<const char*>(blob.data()),
                          static_cast<std::streamsize>(blob.size()));
                if (!out)
                    return j;

                // Written only after the bytes are actually on disk, so the flag is a claim the
                // file can back up rather than an intention.
                j["atmosphere_sidecar"] = true;
                return j;
            }

            void weather_from_json(const json& j, IWorldEditor& world,
                                   const std::string& scene_path)
            {
                if (!j.is_object())
                    return;
                const bool enabled = j.value("procedural_enabled", false);
                world.set_procedural_weather_enabled(enabled);
                if (!enabled)
                    return;
                SushiEngine::Simulation::IWeatherAuthoring* weather = world.weather_authoring();
                if (weather == nullptr)
                    return;
                if (!j.value("atmosphere_sidecar", false))
                    return;

                std::ifstream in(atmosphere_sidecar_path(scene_path), std::ios::binary);
                if (!in)
                    return;
                const std::vector<std::uint8_t> blob{std::istreambuf_iterator<char>(in),
                                                     std::istreambuf_iterator<char>()};
                // A missing, truncated, or wrong-grid sidecar leaves the core exactly as the
                // provider seeded it -- a real atmosphere, just not the saved one. Failing the
                // whole scene load over it would be the worse answer: the entities, the
                // environment and the sky are all still perfectly good.
                (void)weather->restore_state(blob);
            }

            SushiEngine::Quaternion quaternion_from_json(const json& j)
            {
                SushiEngine::Quaternion q;
                q.x = j.value("x", SushiEngine::Scalar(0));
                q.y = j.value("y", SushiEngine::Scalar(0));
                q.z = j.value("z", SushiEngine::Scalar(0));
                q.w = j.value("w", SushiEngine::Scalar(1));
                return q;
            }

            json ui_to_json(const SushiEngine::Simulation::UIElementParams& p)
            {
                return json{{"kind", static_cast<std::uint32_t>(p.kind)},
                            {"anchor_min", json{{"x", p.anchor_min_x}, {"y", p.anchor_min_y}}},
                            {"anchor_max", json{{"x", p.anchor_max_x}, {"y", p.anchor_max_y}}},
                            {"pivot", json{{"x", p.pivot_x}, {"y", p.pivot_y}}},
                            {"position", json{{"x", p.position_x}, {"y", p.position_y}}},
                            {"size", json{{"x", p.size_x}, {"y", p.size_y}}},
                            {"color", vec3_to_json(p.color)},
                            {"alpha", p.alpha},
                            {"font_size", p.font_size},
                            {"text", std::string(p.text)}};
            }

            SushiEngine::Simulation::UIElementParams ui_from_json(const json& j)
            {
                SushiEngine::Simulation::UIElementParams p;
                p.kind = static_cast<SushiEngine::Simulation::UIElementKind>(
                    j.value("kind", static_cast<std::uint32_t>(p.kind)));
                if (j.contains("anchor_min"))
                {
                    p.anchor_min_x = j["anchor_min"].value("x", p.anchor_min_x);
                    p.anchor_min_y = j["anchor_min"].value("y", p.anchor_min_y);
                }
                if (j.contains("anchor_max"))
                {
                    p.anchor_max_x = j["anchor_max"].value("x", p.anchor_max_x);
                    p.anchor_max_y = j["anchor_max"].value("y", p.anchor_max_y);
                }
                if (j.contains("pivot"))
                {
                    p.pivot_x = j["pivot"].value("x", p.pivot_x);
                    p.pivot_y = j["pivot"].value("y", p.pivot_y);
                }
                if (j.contains("position"))
                {
                    p.position_x = j["position"].value("x", p.position_x);
                    p.position_y = j["position"].value("y", p.position_y);
                }
                if (j.contains("size"))
                {
                    p.size_x = j["size"].value("x", p.size_x);
                    p.size_y = j["size"].value("y", p.size_y);
                }
                if (j.contains("color"))
                    p.color = vec3_from_json(j["color"]);
                p.alpha = j.value("alpha", p.alpha);
                p.font_size = j.value("font_size", p.font_size);
                const std::string text = j.value("text", std::string{});
                std::snprintf(p.text, sizeof(p.text), "%s", text.c_str());
                return p;
            }

            json script_to_json(const SushiEngine::Simulation::ScriptComponent& script)
            {
                json fields = json::array();
                for (const SushiEngine::Simulation::ScriptField& field : script.fields)
                    fields.push_back(json{{"name", field.name},
                                          {"kind", static_cast<std::uint32_t>(field.kind)},
                                          {"number", field.number},
                                          {"flag", field.flag},
                                          {"vector", vec3_to_json(field.vector)},
                                          {"text", field.text}});
                return json{{"type_name", script.type_name}, {"fields", std::move(fields)}};
            }

            SushiEngine::Simulation::ScriptComponent script_from_json(const json& j)
            {
                SushiEngine::Simulation::ScriptComponent script;
                script.type_name = j.value("type_name", std::string{});
                if (j.contains("fields") && j["fields"].is_array())
                    for (const json& f : j["fields"])
                    {
                        SushiEngine::Simulation::ScriptField field;
                        field.name = f.value("name", std::string{});
                        field.kind = static_cast<SushiEngine::Simulation::ScriptFieldKind>(
                            f.value("kind", static_cast<std::uint32_t>(field.kind)));
                        field.number = f.value("number", field.number);
                        field.flag = f.value("flag", field.flag);
                        if (f.contains("vector"))
                            field.vector = vec3_from_json(f["vector"]);
                        field.text = f.value("text", std::string{});
                        script.fields.push_back(field);
                    }
                return script;
            }

            json texture_transform_to_json(const SushiEngine::Render::TextureTransform& t)
            {
                return json{{"tiling_x", t.tiling_x},
                            {"tiling_y", t.tiling_y},
                            {"offset_x", t.offset_x},
                            {"offset_y", t.offset_y}};
            }

            SushiEngine::Render::TextureTransform texture_transform_from_json(const json& j)
            {
                SushiEngine::Render::TextureTransform t;
                t.tiling_x = j.value("tiling_x", t.tiling_x);
                t.tiling_y = j.value("tiling_y", t.tiling_y);
                t.offset_x = j.value("offset_x", t.offset_x);
                t.offset_y = j.value("offset_y", t.offset_y);
                return t;
            }

            // Texture ids ride along with the scalar fields for the same reason the
            // particle effect's do (see capture_effect): the in-memory snapshots
            // undo/redo and play-mode take restore them directly, while a load from
            // disk re-resolves every id from its path (resolve_scene_textures), so a
            // stale handle never survives into another session.
            json material_to_json(const SushiEngine::Render::Material& m)
            {
                return json{
                    {"albedo", vec3_to_json(m.albedo)},
                    {"base_alpha", m.base_alpha},
                    {"albedo_map", m.albedo_map},
                    {"metallic", m.metallic},
                    {"roughness", m.roughness},
                    {"metallic_roughness_map", m.metallic_roughness_map},
                    {"packed_occlusion", m.packed_occlusion},
                    {"normal_map", m.normal_map},
                    {"normal_scale", m.normal_scale},
                    {"height_map", m.height_map},
                    {"height_scale", m.height_scale},
                    {"parallax_steps", m.parallax_steps},
                    {"parallax_shadows", m.parallax_shadows},
                    {"parallax_silhouette_clip", m.parallax_silhouette_clip},
                    {"occlusion_map", m.occlusion_map},
                    {"occlusion_strength", m.occlusion_strength},
                    {"emissive", vec3_to_json(m.emissive)},
                    {"emissive_intensity", m.emissive_intensity},
                    {"emissive_map", m.emissive_map},
                    {"emissive_enabled", m.emissive_enabled},
                    {"detail_albedo_map", m.detail_albedo_map},
                    {"detail_normal_map", m.detail_normal_map},
                    {"detail_mask_map", m.detail_mask_map},
                    {"detail_normal_scale", m.detail_normal_scale},
                    {"main_transform", texture_transform_to_json(m.main_transform)},
                    {"detail_transform", texture_transform_to_json(m.detail_transform)},
                    {"anisotropy", m.anisotropy},
                    {"anisotropy_rotation", m.anisotropy_rotation},
                    {"clearcoat", m.clearcoat},
                    {"clearcoat_roughness", m.clearcoat_roughness},
                    {"sheen_color", vec3_to_json(m.sheen_color)},
                    {"sheen_roughness", m.sheen_roughness},
                    {"transmission", m.transmission},
                    {"subsurface_color", vec3_to_json(m.subsurface_color)},
                    {"thickness", m.thickness},
                    {"ior", m.ior},
                    {"surface_type", static_cast<std::uint32_t>(m.surface_type)},
                    {"alpha_cutoff", m.alpha_cutoff},
                    {"cull_mode", static_cast<std::uint32_t>(m.cull_mode)},
                    {"blend_mode", static_cast<std::uint32_t>(m.blend_mode)},
                    {"render_queue", m.render_queue},
                    {"cast_shadows", m.cast_shadows},
                    {"receive_shadows", m.receive_shadows},
                    {"gpu_instancing", m.gpu_instancing},
                    {"anisotropic_filtering", m.anisotropic_filtering},
                    {"wrap_mode", static_cast<std::uint32_t>(m.wrap_mode)},
                    {"weather_wettable", m.weather_wettable}};
            }

            SushiEngine::Render::Material material_from_json(const json& j)
            {
                SushiEngine::Render::Material m;
                if (j.contains("albedo"))
                    m.albedo = vec3_from_json(j["albedo"]);
                m.base_alpha = j.value("base_alpha", m.base_alpha);
                m.albedo_map = j.value("albedo_map", m.albedo_map);
                m.metallic = j.value("metallic", m.metallic);
                m.roughness = j.value("roughness", m.roughness);
                m.metallic_roughness_map =
                    j.value("metallic_roughness_map", m.metallic_roughness_map);
                m.packed_occlusion = j.value("packed_occlusion", m.packed_occlusion);
                m.normal_map = j.value("normal_map", m.normal_map);
                m.normal_scale = j.value("normal_scale", m.normal_scale);
                m.height_map = j.value("height_map", m.height_map);
                m.height_scale = j.value("height_scale", m.height_scale);
                m.parallax_steps = j.value("parallax_steps", m.parallax_steps);
                m.parallax_shadows = j.value("parallax_shadows", m.parallax_shadows);
                m.parallax_silhouette_clip =
                    j.value("parallax_silhouette_clip", m.parallax_silhouette_clip);
                m.occlusion_map = j.value("occlusion_map", m.occlusion_map);
                m.occlusion_strength = j.value("occlusion_strength", m.occlusion_strength);
                if (j.contains("emissive"))
                    m.emissive = vec3_from_json(j["emissive"]);
                m.emissive_intensity = j.value("emissive_intensity", m.emissive_intensity);
                m.emissive_map = j.value("emissive_map", m.emissive_map);
                m.emissive_enabled = j.value("emissive_enabled", m.emissive_enabled);
                m.detail_albedo_map = j.value("detail_albedo_map", m.detail_albedo_map);
                m.detail_normal_map = j.value("detail_normal_map", m.detail_normal_map);
                m.detail_mask_map = j.value("detail_mask_map", m.detail_mask_map);
                m.detail_normal_scale = j.value("detail_normal_scale", m.detail_normal_scale);
                if (j.contains("main_transform"))
                    m.main_transform = texture_transform_from_json(j["main_transform"]);
                if (j.contains("detail_transform"))
                    m.detail_transform = texture_transform_from_json(j["detail_transform"]);
                m.anisotropy = j.value("anisotropy", m.anisotropy);
                m.anisotropy_rotation = j.value("anisotropy_rotation", m.anisotropy_rotation);
                m.clearcoat = j.value("clearcoat", m.clearcoat);
                m.clearcoat_roughness = j.value("clearcoat_roughness", m.clearcoat_roughness);
                if (j.contains("sheen_color"))
                    m.sheen_color = vec3_from_json(j["sheen_color"]);
                m.sheen_roughness = j.value("sheen_roughness", m.sheen_roughness);
                m.transmission = j.value("transmission", m.transmission);
                if (j.contains("subsurface_color"))
                    m.subsurface_color = vec3_from_json(j["subsurface_color"]);
                m.thickness = j.value("thickness", m.thickness);
                m.ior = j.value("ior", m.ior);
                m.surface_type = static_cast<SushiEngine::Render::SurfaceType>(
                    j.value("surface_type", static_cast<std::uint32_t>(m.surface_type)));
                m.alpha_cutoff = j.value("alpha_cutoff", m.alpha_cutoff);
                m.cull_mode = static_cast<SushiEngine::Render::MaterialCullMode>(
                    j.value("cull_mode", static_cast<std::uint32_t>(m.cull_mode)));
                m.blend_mode = static_cast<SushiEngine::Render::BlendMode>(
                    j.value("blend_mode", static_cast<std::uint32_t>(m.blend_mode)));
                m.render_queue = j.value("render_queue", m.render_queue);
                m.cast_shadows = j.value("cast_shadows", m.cast_shadows);
                m.receive_shadows = j.value("receive_shadows", m.receive_shadows);
                m.gpu_instancing = j.value("gpu_instancing", m.gpu_instancing);
                m.anisotropic_filtering =
                    j.value("anisotropic_filtering", m.anisotropic_filtering);
                m.wrap_mode = static_cast<SushiEngine::Render::TextureWrap>(
                    j.value("wrap_mode", static_cast<std::uint32_t>(m.wrap_mode)));
                m.weather_wettable = j.value("weather_wettable", m.weather_wettable);
                return m;
            }

            json material_paths_to_json(const SushiEngine::Simulation::MaterialTexturePaths& p)
            {
                return json{{"albedo_map", p.albedo_map},
                            {"metallic_roughness_map", p.metallic_roughness_map},
                            {"normal_map", p.normal_map},
                            {"height_map", p.height_map},
                            {"occlusion_map", p.occlusion_map},
                            {"emissive_map", p.emissive_map},
                            {"detail_albedo_map", p.detail_albedo_map},
                            {"detail_normal_map", p.detail_normal_map},
                            {"detail_mask_map", p.detail_mask_map}};
            }

            SushiEngine::Simulation::MaterialTexturePaths material_paths_from_json(const json& j)
            {
                SushiEngine::Simulation::MaterialTexturePaths p;
                p.albedo_map = j.value("albedo_map", std::string{});
                p.metallic_roughness_map = j.value("metallic_roughness_map", std::string{});
                p.normal_map = j.value("normal_map", std::string{});
                p.height_map = j.value("height_map", std::string{});
                p.occlusion_map = j.value("occlusion_map", std::string{});
                p.emissive_map = j.value("emissive_map", std::string{});
                p.detail_albedo_map = j.value("detail_albedo_map", std::string{});
                p.detail_normal_map = j.value("detail_normal_map", std::string{});
                p.detail_mask_map = j.value("detail_mask_map", std::string{});
                return p;
            }

            bool material_paths_empty(const SushiEngine::Simulation::MaterialTexturePaths& p)
            {
                return p.albedo_map.empty() && p.metallic_roughness_map.empty() &&
                       p.normal_map.empty() && p.height_map.empty() &&
                       p.occlusion_map.empty() && p.emissive_map.empty() &&
                       p.detail_albedo_map.empty() && p.detail_normal_map.empty() &&
                       p.detail_mask_map.empty();
            }
        } // namespace

        json capture_scene(IWorldEditor& world)
        {
            const std::vector<EntityId> ids = world.entities();
            std::unordered_map<EntityId, int> index_of;
            for (std::size_t i = 0; i < ids.size(); ++i)
                index_of.emplace(ids[i], static_cast<int>(i));

            json root = json::array();
            for (const EntityId id : ids)
            {
                json entry;
                entry["name"] = world.name(id);
                entry["visible"] = world.visible(id);
                const EntityId parent_id = world.parent(id);
                entry["parent"] = parent_id == NULL_ENTITY ? -1 : index_of.at(parent_id);

                const auto transform = world.transform(id);
                entry["position"] = vec3_to_json(transform.position);
                entry["rotation"] = quaternion_to_json(transform.rotation);
                entry["scale"] = vec3_to_json(transform.scale);

                const bool is_camera = world.is_camera(id);
                entry["is_camera"] = is_camera;
                if (is_camera)
                {
                    const auto params = world.camera_params(id);
                    entry["camera"] = json{{"vertical_fov_radians", params.vertical_fov_radians},
                                           {"near_plane", params.near_plane},
                                           {"far_plane", params.far_plane},
                                           {"display_index", params.display_index},
                                           {"priority", params.priority},
                                           {"active", params.active}};
                }

                // Not mutually exclusive with camera, so it is captured independently
                // rather than in the if/else above (a camera can also carry a Renderer).
                const bool has_renderer = world.has_renderer(id);
                entry["has_renderer"] = has_renderer;
                if (has_renderer)
                    entry["color"] = vec3_to_json(world.color(id));

                // The full PBR surface, but only when it differs from a default
                // material carrying the entity's tint (the tint itself is the "color"
                // key above) — plain tinted entities stay one line in the file.
                {
                    const SushiEngine::Render::Material material = world.material(id);
                    SushiEngine::Render::Material reference{};
                    reference.albedo = material.albedo;
                    const json material_json = material_to_json(material);
                    if (material_json != material_to_json(reference))
                        entry["material"] = material_json;
                    const SushiEngine::Simulation::MaterialTexturePaths paths =
                        world.material_texture_paths(id);
                    if (!material_paths_empty(paths))
                        entry["material_texture_paths"] = material_paths_to_json(paths);
                }

                // Not mutually exclusive with camera/renderer, so it is its own field
                // rather than sharing the if/else above.
                const bool has_physics_body = world.has_physics_body(id);
                entry["has_physics_body"] = has_physics_body;
                if (has_physics_body)
                {
                    const auto params = world.physics_body_params(id);
                    entry["physics_body"] = json{{"inv_mass", params.inv_mass},
                                                 {"inv_inertia", vec3_to_json(params.inv_inertia)},
                                                 {"drag_coefficient", params.drag_coefficient}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_cloth = world.has_cloth(id);
                entry["has_cloth"] = has_cloth;
                if (has_cloth)
                {
                    const auto params = world.cloth_params(id);
                    entry["cloth"] = json{{"rows", params.rows},
                                          {"cols", params.cols},
                                          {"spacing", params.spacing},
                                          {"compliance", params.compliance}};
                }

                const bool has_particle_emitter = world.has_particle_emitter(id);
                entry["has_particle_emitter"] = has_particle_emitter;
                if (has_particle_emitter)
                {
                    const auto params = world.particle_emitter_params(id);
                    // The effect goes with the entity, not as an index into a library: it is the
                    // component's own data, so a scene that reloads without it would come back
                    // with emitters that emit nothing.
                    entry["particle_emitter"] =
                        json{{"seed", params.seed},
                             {"playing", params.playing},
                             {"source", capture_effect(world.particle_effect_source(id))}};
                }

                const bool has_audio_emitter = world.has_audio_emitter(id);
                entry["has_audio_emitter"] = has_audio_emitter;
                if (has_audio_emitter)
                {
                    const auto p = world.audio_emitter_params(id);
                    entry["audio_emitter"] = json{{"sound", p.sound},
                                                  {"gain", p.gain},
                                                  {"priority", p.priority},
                                                  {"bus", p.bus},
                                                  {"min_distance", p.min_distance},
                                                  {"max_distance", p.max_distance},
                                                  {"distance_model", p.distance_model},
                                                  {"rolloff", p.rolloff},
                                                  {"doppler_scale", p.doppler_scale},
                                                  {"reverb_send", p.reverb_send},
                                                  {"spatial", p.spatial},
                                                  {"playing", p.playing},
                                                  {"looping", p.looping}};
                }

                const bool has_reverb_zone = world.has_reverb_zone(id);
                entry["has_reverb_zone"] = has_reverb_zone;
                if (has_reverb_zone)
                {
                    const auto p = world.reverb_zone_params(id);
                    entry["reverb_zone"] = json{{"half_extents", vec3_to_json(p.half_extents)},
                                                {"room", p.room},
                                                {"room_hf", p.room_hf},
                                                {"decay_time", p.decay_time},
                                                {"decay_hf_ratio", p.decay_hf_ratio},
                                                {"reflections", p.reflections},
                                                {"reverb", p.reverb},
                                                {"diffusion", p.diffusion},
                                                {"density", p.density},
                                                {"wet_dry_mix", p.wet_dry_mix},
                                                {"send", p.send},
                                                {"priority", p.priority}};
                }

                const bool has_audio_listener = world.has_audio_listener(id);
                entry["has_audio_listener"] = has_audio_listener;
                if (has_audio_listener)
                {
                    const auto p = world.audio_listener_params(id);
                    entry["audio_listener"] = json{{"gain", p.gain}, {"active", p.active}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_shape = world.has_shape(id);
                entry["has_shape"] = has_shape;
                if (has_shape)
                {
                    const auto params = world.shape_params(id);
                    entry["shape"] = json{{"kind", static_cast<std::uint32_t>(params.kind)},
                                          {"params", vec3_to_json(params.params)}};
                }

                const bool has_collider = world.has_collider(id);
                entry["has_collider"] = has_collider;
                if (has_collider)
                {
                    const auto params = world.collider_params(id);
                    entry["collider"] = json{{"kind", static_cast<std::uint32_t>(params.kind)},
                                             {"params", vec3_to_json(params.params)}};
                }

                const bool has_light = world.has_light(id);
                entry["has_light"] = has_light;
                if (has_light)
                {
                    const auto p = world.light_params(id);
                    entry["light"] = json{{"color", vec3_to_json(p.color)},
                                          {"intensity", p.intensity},
                                          {"range", p.range},
                                          {"is_spot", p.is_spot},
                                          {"inner_degrees", p.inner_degrees},
                                          {"outer_degrees", p.outer_degrees},
                                          {"casts_shadows", p.casts_shadows}};
                }

                const bool has_decal = world.has_decal(id);
                entry["has_decal"] = has_decal;
                if (has_decal)
                {
                    // Ids and paths both, on the material/effect live-handle convention.
                    const auto p = world.decal_params(id);
                    entry["decal"] = json{{"color", vec3_to_json(p.color)},
                                          {"half_extents", vec3_to_json(p.half_extents)},
                                          {"opacity", p.opacity},
                                          {"albedo_map", p.albedo_map},
                                          {"orm_map", p.orm_map},
                                          {"albedo_map_path", p.albedo_map_path},
                                          {"orm_map_path", p.orm_map_path}};
                }

                const bool surface_anchored = world.surface_anchored(id);
                entry["surface_anchored"] = surface_anchored;
                if (surface_anchored)
                    entry["surface_local_orientation"] =
                        quaternion_to_json(world.surface_local_orientation(id));

                // The reference frame the transform is authored in (body + mode). Only the
                // descriptor is persisted; the frame-local pose is derived from the scene
                // Transform (already stored above) and the descriptor on load. A body of -1
                // (the scene root, default) is omitted to keep plain scenes clean.
                const SushiEngine::Simulation::EntityFrame frame = world.entity_frame(id);
                if (frame.reference_body >= 0)
                {
                    entry["reference_body"] = frame.reference_body;
                    entry["frame_mode"] = static_cast<int>(frame.mode);
                }

                const bool has_ui = world.has_ui(id);
                entry["has_ui"] = has_ui;
                if (has_ui)
                    entry["ui"] = ui_to_json(world.ui_params(id));

                const std::vector<std::string> scripts = world.script_components(id);
                if (!scripts.empty())
                {
                    json script_array = json::array();
                    for (const std::string& type_name : scripts)
                        script_array.push_back(script_to_json(world.script_component(id, type_name)));
                    entry["scripts"] = std::move(script_array);
                }

                root.push_back(std::move(entry));
            }

            // The environment rides every capture beside the entities. This is what makes
            // undo, Save Scene, and Play→Stop agree that lighting/sky/weather physics are
            // scene content: a snapshot that carried only the entity array could not
            // restore what its callers claimed it restored (fixed 2026-07-29 — a single
            // Ctrl+Z used to leave the environment as-is while rewinding the world).
            json capture;
            capture["entities"] = std::move(root);
            capture["environment"] = environment_to_json(world.environment());
            return capture;
        }

        void apply_scene(IWorldEditor& world, const json& root)
        {
            // A capture is an object {entities, environment}; a bare array is the older
            // entity-only shape (pre-environment captures, and clipboard-era files) and
            // simply leaves the environment as it is.
            const json& entity_list =
                root.is_object() && root.contains("entities") ? root["entities"] : root;
            if (root.is_object() && root.contains("environment"))
                // The live environment is the base, so fields the capture omits — and the
                // runtime-owned channels the shape never writes (forcing pointers, weather
                // field, ephemeris output) — ride through unchanged.
                world.set_environment(
                    environment_from_json(root["environment"], world.environment()));

            // Replace the world wholesale: clear every existing entity before
            // recreating the file's, so a load is never a merge with the prior scene.
            for (const EntityId id : world.entities())
                world.destroy(id);

            std::vector<EntityId> created;
            created.reserve(entity_list.size());
            for (const auto& entry : entity_list)
            {
                const std::string name = entry.value("name", std::string("Entity"));
                const bool is_camera = entry.value("is_camera", false);
                const EntityId id = is_camera ? world.create_camera(name) : world.create(name);
                created.push_back(id);

                SushiEngine::Simulation::EntityTransform transform;
                if (entry.contains("position"))
                    transform.position = vec3_from_json(entry["position"]);
                if (entry.contains("rotation"))
                    transform.rotation = quaternion_from_json(entry["rotation"]);
                if (entry.contains("scale"))
                    transform.scale = vec3_from_json(entry["scale"]);
                world.set_transform(id, transform);
                world.set_visible(id, entry.value("visible", true));

                if (is_camera && entry.contains("camera"))
                {
                    const json& c = entry["camera"];
                    SushiEngine::Simulation::CameraParams params;
                    params.vertical_fov_radians =
                        c.value("vertical_fov_radians", params.vertical_fov_radians);
                    params.near_plane = c.value("near_plane", params.near_plane);
                    params.far_plane = c.value("far_plane", params.far_plane);
                    params.display_index = c.value("display_index", params.display_index);
                    params.priority = c.value("priority", params.priority);
                    params.active = c.value("active", params.active);
                    world.set_camera_params(id, params);
                }

                // `create`/`create_camera` both attach a Renderer by default, so an
                // explicit false must detach it; true is a no-op re-attach.
                const bool has_renderer = entry.value("has_renderer", entry.contains("color"));
                world.set_has_renderer(id, has_renderer);
                if (has_renderer && entry.contains("color"))
                    world.set_color(id, vec3_from_json(entry["color"]));

                if (entry.contains("material"))
                    world.set_material(id, material_from_json(entry["material"]));
                if (entry.contains("material_texture_paths"))
                    world.set_material_texture_paths(
                        id, material_paths_from_json(entry["material_texture_paths"]));

                if (entry.value("has_physics_body", false))
                {
                    world.set_has_physics_body(id, true);
                    if (entry.contains("physics_body"))
                    {
                        const json& p = entry["physics_body"];
                        SushiEngine::Simulation::PhysicsBodyParams params;
                        params.inv_mass = p.value("inv_mass", params.inv_mass);
                        if (p.contains("inv_inertia"))
                            params.inv_inertia = vec3_from_json(p["inv_inertia"]);
                        params.drag_coefficient =
                            p.value("drag_coefficient", params.drag_coefficient);
                        world.set_physics_body_params(id, params);
                    }
                }

                if (entry.value("has_cloth", false))
                {
                    world.set_has_cloth(id, true);
                    if (entry.contains("cloth"))
                    {
                        const json& c = entry["cloth"];
                        SushiEngine::Simulation::ClothParams params;
                        params.rows = c.value("rows", params.rows);
                        params.cols = c.value("cols", params.cols);
                        params.spacing = c.value("spacing", params.spacing);
                        params.compliance = c.value("compliance", params.compliance);
                        world.set_cloth_params(id, params);
                    }
                }

                if (entry.value("has_particle_emitter", false))
                {
                    world.set_has_particle_emitter(id, true);
                    if (entry.contains("particle_emitter"))
                    {
                        const json& p = entry["particle_emitter"];
                        SushiEngine::Simulation::ParticleEmitterParams params;
                        // An older file's "effect" index is ignored: the effect it named was a
                        // library entry that no longer exists, and "source" carries the real thing.
                        params.seed = p.value("seed", params.seed);
                        params.playing = p.value("playing", params.playing);
                        world.set_particle_emitter_params(id, params);
                        // Absent in files written before the effect moved onto the component; the
                        // emitter then keeps the default it was seeded with rather than failing.
                        if (p.contains("source"))
                        {
                            SushiEngine::Vfx::ParticleEffect source;
                            if (apply_effect(p["source"], source))
                                world.set_particle_effect_source(id, source);
                        }
                    }
                }

                if (entry.value("has_audio_emitter", false))
                {
                    world.set_has_audio_emitter(id, true);
                    if (entry.contains("audio_emitter"))
                    {
                        const json& a = entry["audio_emitter"];
                        SushiEngine::Simulation::AudioEmitterParams p;
                        p.sound = a.value("sound", p.sound);
                        p.gain = a.value("gain", p.gain);
                        p.priority = a.value("priority", p.priority);
                        p.bus = a.value("bus", p.bus);
                        p.min_distance = a.value("min_distance", p.min_distance);
                        p.max_distance = a.value("max_distance", p.max_distance);
                        p.distance_model = a.value("distance_model", p.distance_model);
                        p.rolloff = a.value("rolloff", p.rolloff);
                        p.doppler_scale = a.value("doppler_scale", p.doppler_scale);
                        p.reverb_send = a.value("reverb_send", p.reverb_send);
                        p.spatial = a.value("spatial", p.spatial);
                        p.playing = a.value("playing", p.playing);
                        p.looping = a.value("looping", p.looping);
                        world.set_audio_emitter_params(id, p);
                    }
                }

                if (entry.value("has_reverb_zone", false))
                {
                    world.set_has_reverb_zone(id, true);
                    if (entry.contains("reverb_zone"))
                    {
                        const json& z = entry["reverb_zone"];
                        SushiEngine::Simulation::ReverbZoneParams p;
                        if (z.contains("half_extents"))
                            p.half_extents = vec3_from_json(z["half_extents"]);
                        p.room = z.value("room", p.room);
                        p.room_hf = z.value("room_hf", p.room_hf);
                        p.decay_time = z.value("decay_time", p.decay_time);
                        p.decay_hf_ratio = z.value("decay_hf_ratio", p.decay_hf_ratio);
                        p.reflections = z.value("reflections", p.reflections);
                        p.reverb = z.value("reverb", p.reverb);
                        p.diffusion = z.value("diffusion", p.diffusion);
                        p.density = z.value("density", p.density);
                        p.wet_dry_mix = z.value("wet_dry_mix", p.wet_dry_mix);
                        p.send = z.value("send", p.send);
                        p.priority = z.value("priority", p.priority);
                        world.set_reverb_zone_params(id, p);
                    }
                }

                if (entry.value("has_audio_listener", false))
                {
                    world.set_has_audio_listener(id, true);
                    if (entry.contains("audio_listener"))
                    {
                        const json& l = entry["audio_listener"];
                        SushiEngine::Simulation::AudioListenerParams p;
                        p.gain = l.value("gain", p.gain);
                        p.active = l.value("active", p.active);
                        world.set_audio_listener_params(id, p);
                    }
                }

                if (entry.value("has_shape", false))
                {
                    world.set_has_shape(id, true);
                    if (entry.contains("shape"))
                    {
                        const json& s = entry["shape"];
                        SushiEngine::Simulation::ShapeParams params;
                        params.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            s.value("kind", static_cast<std::uint32_t>(params.kind)));
                        if (s.contains("params"))
                            params.params = vec3_from_json(s["params"]);
                        world.set_shape_params(id, params);
                    }
                }

                if (entry.value("has_collider", false))
                {
                    world.set_has_collider(id, true);
                    if (entry.contains("collider"))
                    {
                        const json& c = entry["collider"];
                        SushiEngine::Simulation::ColliderParams params;
                        params.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            c.value("kind", static_cast<std::uint32_t>(params.kind)));
                        if (c.contains("params"))
                            params.params = vec3_from_json(c["params"]);
                        world.set_collider_params(id, params);
                    }
                }

                if (entry.value("has_light", false))
                {
                    world.set_has_light(id, true);
                    if (entry.contains("light"))
                    {
                        const json& l = entry["light"];
                        SushiEngine::Simulation::LightParams p;
                        if (l.contains("color"))
                            p.color = vec3_from_json(l["color"]);
                        p.intensity = l.value("intensity", p.intensity);
                        p.range = l.value("range", p.range);
                        p.is_spot = l.value("is_spot", p.is_spot);
                        p.inner_degrees = l.value("inner_degrees", p.inner_degrees);
                        p.outer_degrees = l.value("outer_degrees", p.outer_degrees);
                        p.casts_shadows = l.value("casts_shadows", p.casts_shadows);
                        world.set_light_params(id, p);
                    }
                }

                if (entry.value("has_decal", false))
                {
                    world.set_has_decal(id, true);
                    if (entry.contains("decal"))
                    {
                        const json& d = entry["decal"];
                        SushiEngine::Simulation::DecalParams p;
                        if (d.contains("color"))
                            p.color = vec3_from_json(d["color"]);
                        if (d.contains("half_extents"))
                            p.half_extents = vec3_from_json(d["half_extents"]);
                        p.opacity = d.value("opacity", p.opacity);
                        p.albedo_map = d.value("albedo_map", p.albedo_map);
                        p.orm_map = d.value("orm_map", p.orm_map);
                        p.albedo_map_path = d.value("albedo_map_path", p.albedo_map_path);
                        p.orm_map_path = d.value("orm_map_path", p.orm_map_path);
                        world.set_decal_params(id, p);
                    }
                }

                if (entry.value("surface_anchored", false))
                {
                    world.set_surface_anchored(id, true);
                    if (entry.contains("surface_local_orientation"))
                        world.set_surface_local_orientation(
                            id, quaternion_from_json(entry["surface_local_orientation"]));
                }

                // The reference frame (body + mode). Migration: a scene from before the
                // unified dynamic body stored a `has_astro_body` flag instead — map it to a
                // Free reference on the scene's dominant body, the closest representation (its
                // orbital motion returns once the body is also given a velocity; the flag
                // alone only re-expresses the transform). The scene Transform, already loaded,
                // is untouched either way.
                if (entry.contains("reference_body"))
                {
                    SushiEngine::Simulation::EntityFrame frame;
                    frame.reference_body = entry.value("reference_body", -1);
                    frame.mode = static_cast<SushiEngine::Simulation::FrameMode>(
                        entry.value("frame_mode", 0));
                    world.set_entity_frame(id, frame);
                }
                else if (entry.value("has_astro_body", false))
                {
                    SushiEngine::Simulation::EntityFrame frame;
                    frame.reference_body = world.environment().dominant_body_id;
                    frame.mode = SushiEngine::Simulation::FrameMode::Free;
                    world.set_entity_frame(id, frame);
                }

                if (entry.value("has_ui", false))
                {
                    world.set_has_ui(id, true);
                    if (entry.contains("ui"))
                        world.set_ui_params(id, ui_from_json(entry["ui"]));
                }

                if (entry.contains("scripts") && entry["scripts"].is_array())
                    for (const json& s : entry["scripts"])
                        world.add_script_component(id, script_from_json(s));
            }

            // Parent links are resolved only after every entity exists, since a child
            // can be written before its parent in the array.
            for (std::size_t i = 0; i < entity_list.size(); ++i)
            {
                const int parent_index = entity_list[i].value("parent", -1);
                if (parent_index >= 0 && static_cast<std::size_t>(parent_index) < created.size())
                    world.set_parent(created[i], created[static_cast<std::size_t>(parent_index)]);
            }
        }

        bool save_scene(IWorldEditor& world, const std::string& path, const SceneSkyState* sky)
        {
            std::ofstream file(path);
            if (!file)
                return false;

            // capture_scene already carries the entities and the environment; the file
            // adds only what a disk scene has that an in-memory snapshot does not — the
            // weather sidecar reference and the sky authoring state.
            json root = capture_scene(world);
            root["weather"] = weather_to_json(world, path);
            if (sky != nullptr)
                root["sky"] = sky_to_json(*sky);

            file << root.dump(2);
            return static_cast<bool>(file);
        }

        namespace
        {
            /**
             * @brief Re-resolves every file-backed texture handle after a load from disk.
             *
             * The capture carries both a path and the handle it had when written; only the path
             * survives a session, so the handles a file was written with are re-derived here —
             * particle sprites, material maps, and decal maps alike, an empty path resolving to
             * no texture rather than keeping the stale handle. A post-pass rather than a hook
             * inside `apply_scene` because the in-memory snapshots that share that function are
             * same-session, where the captured handles are still the right ones.
             */
            void resolve_scene_textures(IWorldEditor& world,
                                        SushiEngine::Render::IAssetLibrary& assets)
            {
                using SushiEngine::Render::INVALID_TEXTURE;
                using SushiEngine::Render::TextureColorSpace;
                const auto resolve = [&assets](const std::string& path,
                                               TextureColorSpace color_space)
                {
                    return path.empty() ? INVALID_TEXTURE
                                        : assets.load_texture(path.c_str(), color_space);
                };

                for (const EntityId id : world.entities())
                {
                    if (world.has_particle_emitter(id))
                    {
                        SushiEngine::Vfx::ParticleEffect effect = world.particle_effect_source(id);
                        resolve_effect_textures(effect, assets);
                        world.set_particle_effect_source(id, effect);
                    }

                    const SushiEngine::Simulation::MaterialTexturePaths paths =
                        world.material_texture_paths(id);
                    SushiEngine::Render::Material material = world.material(id);
                    const bool any_handle =
                        material.albedo_map != INVALID_TEXTURE ||
                        material.metallic_roughness_map != INVALID_TEXTURE ||
                        material.normal_map != INVALID_TEXTURE ||
                        material.height_map != INVALID_TEXTURE ||
                        material.occlusion_map != INVALID_TEXTURE ||
                        material.emissive_map != INVALID_TEXTURE ||
                        material.detail_albedo_map != INVALID_TEXTURE ||
                        material.detail_normal_map != INVALID_TEXTURE ||
                        material.detail_mask_map != INVALID_TEXTURE;
                    if (any_handle || !material_paths_empty(paths))
                    {
                        material.albedo_map =
                            resolve(paths.albedo_map, TextureColorSpace::Srgb);
                        material.metallic_roughness_map =
                            resolve(paths.metallic_roughness_map, TextureColorSpace::Linear);
                        material.normal_map =
                            resolve(paths.normal_map, TextureColorSpace::Linear);
                        material.height_map =
                            resolve(paths.height_map, TextureColorSpace::Linear);
                        material.occlusion_map =
                            resolve(paths.occlusion_map, TextureColorSpace::Linear);
                        material.emissive_map =
                            resolve(paths.emissive_map, TextureColorSpace::Srgb);
                        material.detail_albedo_map =
                            resolve(paths.detail_albedo_map, TextureColorSpace::Srgb);
                        material.detail_normal_map =
                            resolve(paths.detail_normal_map, TextureColorSpace::Linear);
                        material.detail_mask_map =
                            resolve(paths.detail_mask_map, TextureColorSpace::Linear);
                        world.set_material(id, material);
                    }

                    if (world.has_decal(id))
                    {
                        SushiEngine::Simulation::DecalParams decal = world.decal_params(id);
                        if (decal.albedo_map != INVALID_TEXTURE ||
                            decal.orm_map != INVALID_TEXTURE ||
                            !decal.albedo_map_path.empty() || !decal.orm_map_path.empty())
                        {
                            decal.albedo_map =
                                resolve(decal.albedo_map_path, TextureColorSpace::Srgb);
                            decal.orm_map =
                                resolve(decal.orm_map_path, TextureColorSpace::Linear);
                            world.set_decal_params(id, decal);
                        }
                    }
                }
            }
        } // namespace

        bool load_scene(IWorldEditor& world, const std::string& path, SceneSkyState* sky,
                        SushiEngine::Render::IAssetLibrary* assets)
        {
            std::ifstream file(path);
            if (!file)
                return false;

            json root;
            try
            {
                file >> root;
            }
            catch (const json::parse_error&)
            {
                return false;
            }

            // Pre-environment-persistence scenes are a bare entity array; the environment
            // and sky simply keep whatever the caller already had (their existing defaults).
            if (root.is_array())
            {
                apply_scene(world, root);
                if (assets != nullptr)
                    resolve_scene_textures(world, *assets);
                return true;
            }

            if (!root.is_object() || !root.contains("entities") || !root["entities"].is_array())
                return false;

            // apply_scene restores the entities and the environment together (the same
            // object shape capture_scene produces); the file-only extras follow.
            apply_scene(world, root);
            if (assets != nullptr)
                resolve_scene_textures(world, *assets);
            if (root.contains("weather"))
                weather_from_json(root["weather"], world, path);
            if (sky != nullptr && root.contains("sky"))
                *sky = sky_from_json(root["sky"], *sky);
            return true;
        }
    } // namespace Editor
} // namespace SushiEngine
