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

#include <SushiEngine/render/asset_library_interface.hpp>
#include <SushiEngine/render/imported_mesh.hpp>

#include "byte_encoding.hpp"
#include "effect_serializer.hpp"
#include "entity_record.hpp"
#include "environment_serializer.hpp"
#include "prefab_serializer.hpp"

namespace SushiEngine
{
    namespace Scene
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

            json joint_limit_to_json(const SushiEngine::Simulation::JointLimitDescription& limit)
            {
                return json{{"lower", limit.lower},
                            {"upper", limit.upper},
                            {"compliance", limit.compliance},
                            {"enabled", limit.enabled}};
            }

            /**
             * @brief Reads a joint limit, keeping @p limit's values for absent fields.
             *
             * The defaults come from the caller's own value rather than from a fresh
             * `JointLimitDescription`, which is the convention `sky_from_json` follows: a file
             * written by an older build is missing fields, not asserting zeros for them.
             */
            SushiEngine::Simulation::JointLimitDescription joint_limit_from_json(
                const json& j, SushiEngine::Simulation::JointLimitDescription limit)
            {
                if (!j.is_object())
                    return limit;
                limit.lower = j.value("lower", limit.lower);
                limit.upper = j.value("upper", limit.upper);
                limit.compliance = j.value("compliance", limit.compliance);
                limit.enabled = j.value("enabled", limit.enabled);
                return limit;
            }

            json joint_motor_to_json(const SushiEngine::Simulation::JointMotorDescription& motor)
            {
                return json{{"type", static_cast<std::uint32_t>(motor.type)},
                            {"target", motor.target},
                            {"max_force", motor.max_force},
                            {"compliance", motor.compliance},
                            {"damping", motor.damping}};
            }

            SushiEngine::Simulation::JointMotorDescription joint_motor_from_json(
                const json& j, SushiEngine::Simulation::JointMotorDescription motor)
            {
                if (!j.is_object())
                    return motor;
                motor.type = static_cast<SushiEngine::Simulation::JointMotorType>(
                    j.value("type", static_cast<std::uint32_t>(motor.type)));
                motor.target = j.value("target", motor.target);
                motor.max_force = j.value("max_force", motor.max_force);
                motor.compliance = j.value("compliance", motor.compliance);
                motor.damping = j.value("damping", motor.damping);
                return motor;
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
             * @brief W4's procedural weather state.
             *
             * **There is no system list, because there are no systems.** A dozen ellipses with
             * headings and radii is an object model that can be spelled in JSON only because
             * it is authored data pretending to be weather. The global core has no such
             * objects: its state is two potential-vorticity fields and a moisture field on a
             * 512x256 grid, several megabytes of numbers that mean nothing individually.
             * Writing that into the scene JSON would bloat a human-editable text file past the
             * point of being human-editable, for a payload no human would ever edit, so it
             * goes to a binary sidecar beside the scene and the JSON keeps only the fact that
             * one exists.
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
                // The mode by name rather than by the old boolean: the two are no longer
                // "procedural, or the absence of it", and a name survives a third mode being
                // added in a way `procedural_enabled: false` would not.
                j["mode"] = SushiEngine::Simulation::weather_mode_name(world.weather_mode());
                // Always written, in either mode. Manual's sky *is* this number, and Procedural
                // remembers it so that coming back to Manual returns the same sky rather than a
                // new one — see ISimulation::weather_seed.
                j["seed"] = world.weather_seed();
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
                // The seed goes in before the mode, so installing a Manual provider builds it
                // from the scene's own seed rather than from the default and then rebuilding.
                world.set_weather_seed(
                    j.value("seed", static_cast<std::uint64_t>(world.weather_seed())));
                // `procedural_enabled` is the pre-WM-SEED spelling and is still read, so scenes
                // saved before the mode existed open as what they were rather than silently
                // switching. New files write `mode`.
                const bool procedural =
                    j.contains("mode")
                        ? j.value("mode", std::string("manual")) == "procedural"
                        : j.value("procedural_enabled", false);
                world.set_weather_mode(procedural
                                           ? SushiEngine::Simulation::WeatherMode::Procedural
                                           : SushiEngine::Simulation::WeatherMode::Manual);
                if (!procedural)
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

            json ui_to_json(const SushiEngine::Simulation::UIElementParameters& p)
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

            SushiEngine::Simulation::UIElementParameters ui_from_json(const json& j)
            {
                SushiEngine::Simulation::UIElementParameters p;
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
            // disk re-resolves every id from its path (resolve_scene_assets), so a
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
                    {"cast_shadows", m.cast_shadows},
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
                m.cast_shadows = j.value("cast_shadows", m.cast_shadows);
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

            json soft_body_material_to_json(
                const SushiEngine::Physics::SoftBodyMaterialT<SushiEngine::Scalar>& material)
            {
                return json{{"young_modulus", material.young_modulus},
                            {"poisson_ratio", material.poisson_ratio},
                            {"density", material.density},
                            {"damping", material.damping},
                            {"yield_stress", material.yield_stress},
                            {"plastic_creep", material.plastic_creep},
                            {"maximum_plastic_strain", material.maximum_plastic_strain},
                            {"fracture_stress", material.fracture_stress}};
            }

            /**
             * @brief Reads a soft-body material, keeping @p material's values for absent fields.
             *
             * Same convention as `joint_limit_from_json`: a file written by an older build
             * is missing fields rather than asserting zeros for them, and a zero
             * `young_modulus` is a body with no stiffness at all.
             */
            SushiEngine::Physics::SoftBodyMaterialT<SushiEngine::Scalar>
            soft_body_material_from_json(
                const json& j,
                SushiEngine::Physics::SoftBodyMaterialT<SushiEngine::Scalar> material)
            {
                if (!j.is_object())
                    return material;
                material.young_modulus = j.value("young_modulus", material.young_modulus);
                material.poisson_ratio = j.value("poisson_ratio", material.poisson_ratio);
                material.density = j.value("density", material.density);
                material.damping = j.value("damping", material.damping);
                material.yield_stress = j.value("yield_stress", material.yield_stress);
                material.plastic_creep = j.value("plastic_creep", material.plastic_creep);
                material.maximum_plastic_strain =
                    j.value("maximum_plastic_strain", material.maximum_plastic_strain);
                material.fracture_stress = j.value("fracture_stress", material.fracture_stress);
                return material;
            }

            /**
             * @brief The local transform a record states, defaulting each axis it omits.
             *
             * Read in two places — when the entity is created and again after it is
             * parented — so it is one function rather than two copies that can disagree
             * about what a missing `scale` means.
             */
            SushiEngine::Simulation::EntityTransform transform_from_record(const json& entry)
            {
                SushiEngine::Simulation::EntityTransform transform;
                if (entry.contains("position"))
                    transform.position = vec3_from_json(entry["position"]);
                if (entry.contains("rotation"))
                    transform.rotation = quaternion_from_json(entry["rotation"]);
                if (entry.contains("scale"))
                    transform.scale = vec3_from_json(entry["scale"]);
                return transform;
            }
        } // namespace

        namespace Detail
        {
            json write_entity_record(IWorldEditor& world, EntityId id,
                                     const std::unordered_map<EntityId, int>& index_of,
                                     ISceneBlobTable* blobs)
            {
                json entry;
                entry["name"] = world.name(id);
                // Only when it has one. An empty key on every entity of every scene
                // would be noise, and its absence already means what an empty string
                // would: this entity came from no prefab.
                const std::string prefab_entity = world.prefab_entity_id(id);
                if (!prefab_entity.empty())
                    entry["prefab_entity_id"] = prefab_entity;
                entry["visible"] = world.visible(id);
                const EntityId parent_id = world.parent(id);
                // A parent outside the document is written as no parent at all, which is what
                // makes a subtree's root a root: `capture_scene` puts every entity in the map,
                // so this reads as it always did there.
                const auto parent_entry = index_of.find(parent_id);
                entry["parent"] = parent_id == NULL_ENTITY || parent_entry == index_of.end()
                                      ? -1
                                      : parent_entry->second;

                const auto transform = world.transform(id);
                entry["position"] = vec3_to_json(transform.position);
                entry["rotation"] = quaternion_to_json(transform.rotation);
                entry["scale"] = vec3_to_json(transform.scale);

                const bool is_camera = world.is_camera(id);
                entry["is_camera"] = is_camera;
                if (is_camera)
                {
                    const auto parameters = world.camera_parameters(id);
                    entry["camera"] =
                        json{{"vertical_fov_radians", parameters.vertical_fov_radians},
                             {"near_plane", parameters.near_plane},
                             {"far_plane", parameters.far_plane},
                             {"display_index", parameters.display_index},
                             {"priority", parameters.priority},
                             {"active", parameters.active}};
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
                    const auto parameters = world.physics_body_parameters(id);
                    entry["physics_body"] =
                        json{{"inv_mass", parameters.inv_mass},
                             {"inv_inertia", vec3_to_json(parameters.inv_inertia)},
                             {"drag_coefficient", parameters.drag_coefficient},
                             // Both of these override the two numbers above rather
                             // than sitting beside them, so a file that carried the
                             // numbers alone would reload as a body whose mass was
                             // right and whose *authoring* was gone: the density box
                             // empty, the kinematic box unticked, and the derived
                             // figures presented as if somebody had typed them.
                             {"density", parameters.density},
                             {"kinematic", parameters.kinematic}};
                }

                const bool has_character = world.has_character(id);
                entry["has_character"] = has_character;
                if (has_character)
                {
                    const auto c = world.character_parameters(id);
                    entry["character"] = json{{"radius", c.radius},
                                              {"height", c.height},
                                              {"step_height", c.step_height},
                                              {"max_slope_degrees", c.max_slope_degrees},
                                              {"skin_width", c.skin_width},
                                              {"ground_snap", c.ground_snap}};
                }

                const bool has_impact = world.has_impact_response(id);
                entry["has_impact_response"] = has_impact;
                if (has_impact)
                {
                    const auto r = world.impact_response(id);
                    entry["impact_response"] =
                        json{{"minimum_impulse", r.minimum_impulse},
                             {"full_impulse", r.full_impulse},
                             {"cooldown_seconds", r.cooldown_seconds},
                             {"plays_audio", r.plays_audio},
                             {"emits_particles", r.emits_particles},
                             {"particle_seconds", r.particle_seconds}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_cloth = world.has_cloth(id);
                entry["has_cloth"] = has_cloth;
                if (has_cloth)
                {
                    const auto parameters = world.cloth_parameters(id);
                    entry["cloth"] = json{{"rows", parameters.rows},
                                          {"cols", parameters.cols},
                                          {"spacing", parameters.spacing},
                                          {"compliance", parameters.compliance}};
                }

                const bool has_soft_body = world.has_soft_body(id);
                entry["has_soft_body"] = has_soft_body;
                if (has_soft_body)
                {
                    const auto parameters = world.soft_body_parameters(id);
                    json soft_body = json{{"level", parameters.level},
                                          {"thickness", parameters.thickness},
                                          {"self_collision", parameters.self_collision},
                                          {"cosmetic", parameters.cosmetic},
                                          {"material",
                                           soft_body_material_to_json(parameters.material)}};
                    // The cook *is* the body — nothing re-derives a tetrahedral lattice at
                    // runtime — so the blob travels with the entity either as its own bytes
                    // or as a name for bytes the caller is holding. Which of the two, and
                    // why, is capture_scene's own documentation.
                    if (blobs != nullptr)
                    {
                        const std::uint64_t key = Detail::content_hash(parameters.asset);
                        blobs->put(key, parameters.asset);
                        soft_body["asset_hash"] = key;
                    }
                    else
                    {
                        soft_body["asset"] = Detail::encode_base64(parameters.asset);
                    }
                    entry["soft_body"] = std::move(soft_body);
                }

                const bool has_crowd = world.has_crowd(id);
                entry["has_crowd"] = has_crowd;
                if (has_crowd)
                {
                    // Ids and paths both, on the decal/material live-handle convention: an
                    // in-memory snapshot restores the ids it was written with, while a load
                    // from disk re-derives every one of them from the paths beside them —
                    // the skeleton and clip inside `set_crowd_parameters`, the mesh and the
                    // material's maps in `resolve_scene_assets`.
                    const auto p = world.crowd_parameters(id);
                    entry["crowd"] = json{{"skeleton", p.skeleton},
                                          {"clip", p.clip},
                                          {"mesh", p.mesh},
                                          {"skeleton_path", p.skeleton_path},
                                          {"clip_path", p.clip_path},
                                          {"mesh_path", p.mesh_path},
                                          {"material", material_to_json(p.material)},
                                          {"time_seconds", p.time_seconds},
                                          {"loop", p.loop},
                                          {"playing", p.playing}};
                }

                const bool has_particle_emitter = world.has_particle_emitter(id);
                entry["has_particle_emitter"] = has_particle_emitter;
                if (has_particle_emitter)
                {
                    const auto parameters = world.particle_emitter_parameters(id);
                    // The effect goes with the entity, not as an index into a library: it is the
                    // component's own data, so a scene that reloads without it would come back
                    // with emitters that emit nothing.
                    entry["particle_emitter"] =
                        json{{"seed", parameters.seed},
                             {"playing", parameters.playing},
                             {"source", capture_effect(world.particle_effect_source(id))}};
                }

                const bool has_audio_emitter = world.has_audio_emitter(id);
                entry["has_audio_emitter"] = has_audio_emitter;
                if (has_audio_emitter)
                {
                    const auto p = world.audio_emitter_parameters(id);
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
                    const auto p = world.reverb_zone_parameters(id);
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
                    const auto p = world.audio_listener_parameters(id);
                    entry["audio_listener"] = json{{"gain", p.gain}, {"active", p.active}};
                }

                // Not mutually exclusive with any of the above, so it is its own field
                // pair too.
                const bool has_shape = world.has_shape(id);
                entry["has_shape"] = has_shape;
                if (has_shape)
                {
                    const auto parameters = world.shape_parameters(id);
                    // Ids and paths both, on the decal/material live-handle convention: an
                    // in-memory snapshot (capture_scene/apply_scene — Undo/Redo, Play->Stop)
                    // restores the handle it was written with, while a load from disk
                    // re-derives it from the path beside it in resolve_scene_assets. Writing
                    // the path alone would make every undo step that touches an unrelated
                    // entity reset this Shape's mesh to none, since apply_scene never
                    // re-imports anything.
                    entry["shape"] = json{{"kind", static_cast<std::uint32_t>(parameters.kind)},
                                          {"params", vec3_to_json(parameters.parameters)},
                                          {"mesh", parameters.mesh},
                                          {"mesh_path", parameters.mesh_path},
                                          {"source_node", parameters.source_node},
                                          {"primitive", parameters.primitive}};
                }

                const bool has_collider = world.has_collider(id);
                entry["has_collider"] = has_collider;
                if (has_collider)
                {
                    const auto parameters = world.collider_parameters(id);
                    entry["collider"] =
                        json{{"kind", static_cast<std::uint32_t>(parameters.kind)},
                             {"params", vec3_to_json(parameters.parameters)},
                             {"layer", parameters.layer},
                             {"collides_with", parameters.collides_with},
                             {"static_friction", parameters.static_friction},
                             {"dynamic_friction", parameters.dynamic_friction},
                             {"restitution", parameters.restitution},
                             {"friction_combine", parameters.friction_combine},
                             {"restitution_combine", parameters.restitution_combine},
                             {"trigger", parameters.trigger},
                             {"continuous_collision", parameters.continuous_collision}};
                }

                const bool has_vehicle = world.has_vehicle(id);
                entry["has_vehicle"] = has_vehicle;
                if (has_vehicle)
                {
                    // The path only. The authored setup - corners, tyres, drivetrain,
                    // aerodynamics - is a large nested record the Vehicle window owns, and
                    // writing it field by field here would be a second definition of it to
                    // keep in step. Until that record has a serializer of its own, a saved
                    // vehicle reloads as its cooked structure at the default setup, and the
                    // Vehicle window says so rather than the file losing it silently.
                    const auto parameters = world.vehicle_parameters(id);
                    entry["vehicle"] = json{{"asset_path", parameters.asset_path}};
                }

                const bool has_joint = world.has_joint(id);
                entry["has_joint"] = has_joint;
                if (has_joint)
                {
                    const auto parameters = world.joint_parameters(id);
                    // The partner is written as an *index into this array*, exactly like
                    // the parent link above, because an EntityId is assigned at creation
                    // and is not a property of the scene — a file that stored one would
                    // reconnect to whatever entity happened to be handed that number on
                    // the next load. Resolved in the same second pass, for the same
                    // reason: a joint can be written before the body it names.
                    const auto partner = index_of.find(parameters.connected_body);
                    entry["joint"] =
                        json{{"connected", partner != index_of.end() ? partner->second : -1},
                             {"type", static_cast<std::uint32_t>(parameters.joint.type)},
                             {"anchor_a", vec3_to_json(parameters.joint.anchor_a)},
                             {"anchor_b", vec3_to_json(parameters.joint.anchor_b)},
                             {"axis_a", vec3_to_json(parameters.joint.axis_a)},
                             {"axis_b", vec3_to_json(parameters.joint.axis_b)},
                             {"compliance", parameters.joint.compliance},
                             {"linear_limit", joint_limit_to_json(parameters.joint.linear_limit)},
                             {"twist_limit", joint_limit_to_json(parameters.joint.twist_limit)},
                             {"swing_limit", joint_limit_to_json(parameters.joint.swing_limit)},
                             {"motor", joint_motor_to_json(parameters.joint.motor)},
                             {"break_force", parameters.joint.break_force},
                             {"break_torque", parameters.joint.break_torque}};
                }

                const bool has_light = world.has_light(id);
                entry["has_light"] = has_light;
                if (has_light)
                {
                    const auto p = world.light_parameters(id);
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
                    const auto p = world.decal_parameters(id);
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
                    entry["ui"] = ui_to_json(world.ui_parameters(id));

                const std::vector<std::string> scripts = world.script_components(id);
                if (!scripts.empty())
                {
                    json script_array = json::array();
                    for (const std::string& type_name : scripts)
                        script_array.push_back(script_to_json(world.script_component(id, type_name)));
                    entry["scripts"] = std::move(script_array);
                }

                // No `has_prefab_instance` flag beside it: the block's presence is the flag,
                // because this component has no present-but-empty state to distinguish. That
                // also keeps it out of every entity in every file that has no prefab.
                if (world.has_prefab_instance(id))
                {
                    const auto parameters = world.prefab_instance(id);
                    entry["prefab_instance"] =
                        json{{"path", parameters.path}, {"revision", parameters.revision}};
                }
                return entry;
            }

            EntityId read_entity_record(IWorldEditor& world, const json& entry,
                                        const ISceneBlobTable* blobs)
            {
                const std::string name = entry.value("name", std::string("Entity"));
                const bool is_camera = entry.value("is_camera", false);
                const EntityId id = is_camera ? world.create_camera(name) : world.create(name);

                world.set_transform(id, transform_from_record(entry));
                world.set_visible(id, entry.value("visible", true));
                world.set_prefab_entity_id(
                    id, entry.value("prefab_entity_id", std::string{}));

                if (is_camera && entry.contains("camera"))
                {
                    const json& c = entry["camera"];
                    SushiEngine::Simulation::CameraParameters parameters;
                    parameters.vertical_fov_radians =
                        c.value("vertical_fov_radians", parameters.vertical_fov_radians);
                    parameters.near_plane = c.value("near_plane", parameters.near_plane);
                    parameters.far_plane = c.value("far_plane", parameters.far_plane);
                    parameters.display_index = c.value("display_index", parameters.display_index);
                    parameters.priority = c.value("priority", parameters.priority);
                    parameters.active = c.value("active", parameters.active);
                    world.set_camera_parameters(id, parameters);
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
                        SushiEngine::Simulation::PhysicsBodyParameters parameters;
                        parameters.inv_mass = p.value("inv_mass", parameters.inv_mass);
                        if (p.contains("inv_inertia"))
                            parameters.inv_inertia = vec3_from_json(p["inv_inertia"]);
                        parameters.drag_coefficient =
                            p.value("drag_coefficient", parameters.drag_coefficient);
                        parameters.density = p.value("density", parameters.density);
                        parameters.kinematic = p.value("kinematic", parameters.kinematic);
                        world.set_physics_body_parameters(id, parameters);
                    }
                }

                if (entry.value("has_character", false))
                {
                    world.set_has_character(id, true);
                    if (entry.contains("character"))
                    {
                        const json& c = entry["character"];
                        SushiEngine::Simulation::CharacterParameters parameters;
                        parameters.radius = c.value("radius", parameters.radius);
                        parameters.height = c.value("height", parameters.height);
                        parameters.step_height = c.value("step_height", parameters.step_height);
                        parameters.max_slope_degrees =
                            c.value("max_slope_degrees", parameters.max_slope_degrees);
                        parameters.skin_width = c.value("skin_width", parameters.skin_width);
                        parameters.ground_snap = c.value("ground_snap", parameters.ground_snap);
                        world.set_character_parameters(id, parameters);
                    }
                }

                if (entry.value("has_impact_response", false))
                {
                    world.set_has_impact_response(id, true);
                    if (entry.contains("impact_response"))
                    {
                        const json& r = entry["impact_response"];
                        SushiEngine::Simulation::ImpactResponse response;
                        response.minimum_impulse =
                            r.value("minimum_impulse", response.minimum_impulse);
                        response.full_impulse = r.value("full_impulse", response.full_impulse);
                        response.cooldown_seconds =
                            r.value("cooldown_seconds", response.cooldown_seconds);
                        response.plays_audio = r.value("plays_audio", response.plays_audio);
                        response.emits_particles =
                            r.value("emits_particles", response.emits_particles);
                        response.particle_seconds =
                            r.value("particle_seconds", response.particle_seconds);
                        world.set_impact_response(id, response);
                    }
                }

                if (entry.value("has_cloth", false))
                {
                    world.set_has_cloth(id, true);
                    if (entry.contains("cloth"))
                    {
                        const json& c = entry["cloth"];
                        SushiEngine::Simulation::ClothParameters parameters;
                        parameters.rows = c.value("rows", parameters.rows);
                        parameters.cols = c.value("cols", parameters.cols);
                        parameters.spacing = c.value("spacing", parameters.spacing);
                        parameters.compliance = c.value("compliance", parameters.compliance);
                        world.set_cloth_parameters(id, parameters);
                    }
                }

                if (entry.value("has_soft_body", false) && entry.contains("soft_body"))
                {
                    const json& s = entry["soft_body"];
                    SushiEngine::Simulation::SoftBodyParameters parameters;
                    // Inline bytes first, then the table: a scene file carries its own
                    // blob, and only an in-memory snapshot names one. An asset that
                    // resolves neither way leaves the entity without a soft body, since
                    // an empty blob is a Soft Body that can never become one — the same
                    // refusal create_soft_body makes at the other end of the path.
                    bool resolved = false;
                    if (s.contains("asset") && s["asset"].is_string())
                        resolved =
                            Detail::decode_base64(s["asset"].get<std::string>(), parameters.asset);
                    else if (blobs != nullptr && s.contains("asset_hash") &&
                             s["asset_hash"].is_number_unsigned())
                        resolved = blobs->get(s["asset_hash"].get<std::uint64_t>(),
                                              parameters.asset);

                    if (resolved)
                    {
                        parameters.level = s.value("level", parameters.level);
                        parameters.thickness = s.value("thickness", parameters.thickness);
                        parameters.self_collision =
                            s.value("self_collision", parameters.self_collision);
                        parameters.cosmetic = s.value("cosmetic", parameters.cosmetic);
                        parameters.material = soft_body_material_from_json(
                            s.value("material", json{}), parameters.material);
                        world.set_has_soft_body(id, true);
                        world.set_soft_body_parameters(id, parameters);
                    }
                }

                if (entry.value("has_crowd", false))
                {
                    // Presence first: `set_crowd_parameters` re-registers the skeleton and
                    // clip from their paths as it writes, so the component is bound by the
                    // time this returns and a scene load needs no second pass for them.
                    world.set_has_crowd(id, true);
                    if (entry.contains("crowd"))
                    {
                        const json& c = entry["crowd"];
                        SushiEngine::Simulation::CrowdParameters p;
                        p.skeleton = c.value("skeleton", p.skeleton);
                        p.clip = c.value("clip", p.clip);
                        p.mesh = c.value("mesh", p.mesh);
                        p.skeleton_path = c.value("skeleton_path", std::string{});
                        p.clip_path = c.value("clip_path", std::string{});
                        p.mesh_path = c.value("mesh_path", std::string{});
                        if (c.contains("material"))
                            p.material = material_from_json(c["material"]);
                        p.time_seconds = c.value("time_seconds", p.time_seconds);
                        p.loop = c.value("loop", p.loop);
                        p.playing = c.value("playing", p.playing);
                        world.set_crowd_parameters(id, p);
                    }
                }

                if (entry.value("has_particle_emitter", false))
                {
                    world.set_has_particle_emitter(id, true);
                    if (entry.contains("particle_emitter"))
                    {
                        const json& p = entry["particle_emitter"];
                        SushiEngine::Simulation::ParticleEmitterParameters parameters;
                        // An older file's "effect" index is ignored: the effect it named was a
                        // library entry that no longer exists, and "source" carries the real thing.
                        parameters.seed = p.value("seed", parameters.seed);
                        parameters.playing = p.value("playing", parameters.playing);
                        world.set_particle_emitter_parameters(id, parameters);
                        // Absent in files written before the effect moved onto the component; the
                        // emitter then keeps the default it was seeded with rather than failing.
                        if (p.contains("source"))
                        {
                            SushiEngine::VFX::ParticleEffect source;
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
                        SushiEngine::Simulation::AudioEmitterParameters p;
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
                        world.set_audio_emitter_parameters(id, p);
                    }
                }

                if (entry.value("has_reverb_zone", false))
                {
                    world.set_has_reverb_zone(id, true);
                    if (entry.contains("reverb_zone"))
                    {
                        const json& z = entry["reverb_zone"];
                        SushiEngine::Simulation::ReverbZoneParameters p;
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
                        world.set_reverb_zone_parameters(id, p);
                    }
                }

                if (entry.value("has_audio_listener", false))
                {
                    world.set_has_audio_listener(id, true);
                    if (entry.contains("audio_listener"))
                    {
                        const json& l = entry["audio_listener"];
                        SushiEngine::Simulation::AudioListenerParameters p;
                        p.gain = l.value("gain", p.gain);
                        p.active = l.value("active", p.active);
                        world.set_audio_listener_parameters(id, p);
                    }
                }

                if (entry.value("has_shape", false))
                {
                    world.set_has_shape(id, true);
                    if (entry.contains("shape"))
                    {
                        const json& s = entry["shape"];
                        SushiEngine::Simulation::ShapeParameters parameters;
                        parameters.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            s.value("kind", static_cast<std::uint32_t>(parameters.kind)));
                        if (s.contains("params"))
                            parameters.parameters = vec3_from_json(s["params"]);
                        parameters.mesh = s.value("mesh", parameters.mesh);
                        parameters.mesh_path = s.value("mesh_path", std::string{});
                        // Absent in a scene written before a Shape could name which node it
                        // drew, and zero is what those files meant: the file's first node and
                        // its first primitive.
                        parameters.source_node = s.value("source_node", parameters.source_node);
                        parameters.primitive = s.value("primitive", parameters.primitive);
                        world.set_shape_parameters(id, parameters);
                    }
                }

                if (entry.value("has_vehicle", false))
                {
                    world.set_has_vehicle(id, true);
                    if (entry.contains("vehicle"))
                    {
                        SushiEngine::Simulation::VehicleInstanceParameters parameters =
                            world.vehicle_parameters(id);
                        parameters.asset_path = entry["vehicle"].value("asset_path", std::string());
                        world.set_vehicle_parameters(id, parameters);
                    }
                }

                if (entry.value("has_collider", false))
                {
                    world.set_has_collider(id, true);
                    if (entry.contains("collider"))
                    {
                        const json& c = entry["collider"];
                        SushiEngine::Simulation::ColliderParameters parameters;
                        parameters.kind = static_cast<SushiEngine::Simulation::PrimitiveKind>(
                            c.value("kind", static_cast<std::uint32_t>(parameters.kind)));
                        if (c.contains("params"))
                            parameters.parameters = vec3_from_json(c["params"]);
                        parameters.layer = c.value("layer", parameters.layer);
                        parameters.collides_with =
                            c.value("collides_with", parameters.collides_with);
                        parameters.static_friction =
                            c.value("static_friction", parameters.static_friction);
                        parameters.dynamic_friction =
                            c.value("dynamic_friction", parameters.dynamic_friction);
                        parameters.restitution = c.value("restitution", parameters.restitution);
                        parameters.friction_combine =
                            c.value("friction_combine", parameters.friction_combine);
                        parameters.restitution_combine =
                            c.value("restitution_combine", parameters.restitution_combine);
                        parameters.trigger = c.value("trigger", parameters.trigger);
                        parameters.continuous_collision =
                            c.value("continuous_collision", parameters.continuous_collision);
                        world.set_collider_parameters(id, parameters);
                    }
                }

                if (entry.value("has_light", false))
                {
                    world.set_has_light(id, true);
                    if (entry.contains("light"))
                    {
                        const json& l = entry["light"];
                        SushiEngine::Simulation::LightParameters p;
                        if (l.contains("color"))
                            p.color = vec3_from_json(l["color"]);
                        p.intensity = l.value("intensity", p.intensity);
                        p.range = l.value("range", p.range);
                        p.is_spot = l.value("is_spot", p.is_spot);
                        p.inner_degrees = l.value("inner_degrees", p.inner_degrees);
                        p.outer_degrees = l.value("outer_degrees", p.outer_degrees);
                        p.casts_shadows = l.value("casts_shadows", p.casts_shadows);
                        world.set_light_parameters(id, p);
                    }
                }

                if (entry.value("has_decal", false))
                {
                    world.set_has_decal(id, true);
                    if (entry.contains("decal"))
                    {
                        const json& d = entry["decal"];
                        SushiEngine::Simulation::DecalParameters p;
                        if (d.contains("color"))
                            p.color = vec3_from_json(d["color"]);
                        if (d.contains("half_extents"))
                            p.half_extents = vec3_from_json(d["half_extents"]);
                        p.opacity = d.value("opacity", p.opacity);
                        p.albedo_map = d.value("albedo_map", p.albedo_map);
                        p.orm_map = d.value("orm_map", p.orm_map);
                        p.albedo_map_path = d.value("albedo_map_path", p.albedo_map_path);
                        p.orm_map_path = d.value("orm_map_path", p.orm_map_path);
                        world.set_decal_parameters(id, p);
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
                        world.set_ui_parameters(id, ui_from_json(entry["ui"]));
                }

                if (entry.contains("scripts") && entry["scripts"].is_array())
                    for (const json& s : entry["scripts"])
                        world.add_script_component(id, script_from_json(s));

                if (entry.contains("prefab_instance"))
                {
                    const json& p = entry["prefab_instance"];
                    SushiEngine::Simulation::PrefabInstanceParameters parameters;
                    parameters.path = p.value("path", std::string{});
                    parameters.revision = p.value("revision", std::string{});
                    world.set_prefab_instance(id, parameters);
                }
                return id;
            }

            // Links between entities are resolved only after every entity exists, since
            // either end can be written before the other in the array. Both kinds live
            // in this one pass: a parent link and a joint's partner are the same problem
            // and a second pass that handled only one of them would be an invitation to
            // resolve the next such link in the first pass and have it half work.
            void link_entity_record(IWorldEditor& world, const json& entry, EntityId id,
                                    const std::vector<EntityId>& created)
            {
                const int parent_index = entry.value("parent", -1);
                if (parent_index >= 0 && static_cast<std::size_t>(parent_index) < created.size())
                {
                    world.set_parent(id, created[static_cast<std::size_t>(parent_index)]);
                    // `set_parent` preserves the entity's world pose by recomputing its local
                    // transform, which is what an artist dragging a node in the Hierarchy
                    // needs and the opposite of what a load needs: the record already holds
                    // the local transform, and the entity was still a root when it was
                    // written, so that value has just been reinterpreted as a world pose and
                    // divided out. Writing it back restores what the file says.
                    //
                    // Safe in any order the array happens to parent its entities in, because
                    // a local transform is absolute data — it does not depend on whether an
                    // ancestor has been parented yet.
                    world.set_transform(id, transform_from_record(entry));
                }

                if (!entry.value("has_joint", false))
                    return;
                world.set_has_joint(id, true);
                if (!entry.contains("joint"))
                    return;

                const json& j = entry["joint"];
                SushiEngine::Simulation::PhysicsJointParameters parameters;
                const int connected = j.value("connected", -1);
                if (connected >= 0 && static_cast<std::size_t>(connected) < created.size())
                    parameters.connected_body = created[static_cast<std::size_t>(connected)];
                parameters.joint.type = static_cast<SushiEngine::Simulation::JointType>(
                    j.value("type", static_cast<std::uint32_t>(parameters.joint.type)));
                if (j.contains("anchor_a"))
                    parameters.joint.anchor_a = vec3_from_json(j["anchor_a"]);
                if (j.contains("anchor_b"))
                    parameters.joint.anchor_b = vec3_from_json(j["anchor_b"]);
                if (j.contains("axis_a"))
                    parameters.joint.axis_a = vec3_from_json(j["axis_a"]);
                if (j.contains("axis_b"))
                    parameters.joint.axis_b = vec3_from_json(j["axis_b"]);
                parameters.joint.compliance = j.value("compliance", parameters.joint.compliance);
                // `value` with a json default rather than `operator[]`: indexing a *const*
                // json with an absent key is undefined, and a scene written before a limit
                // existed is exactly the file that will not have the key.
                parameters.joint.linear_limit =
                    joint_limit_from_json(j.value("linear_limit", json{}),
                                          parameters.joint.linear_limit);
                parameters.joint.twist_limit =
                    joint_limit_from_json(j.value("twist_limit", json{}),
                                          parameters.joint.twist_limit);
                parameters.joint.swing_limit =
                    joint_limit_from_json(j.value("swing_limit", json{}),
                                          parameters.joint.swing_limit);
                parameters.joint.motor =
                    joint_motor_from_json(j.value("motor", json{}), parameters.joint.motor);
                parameters.joint.break_force = j.value("break_force", parameters.joint.break_force);
                parameters.joint.break_torque =
                    j.value("break_torque", parameters.joint.break_torque);
                world.set_joint_parameters(id, parameters);
            }
        } // namespace Detail

        json capture_scene(IWorldEditor& world, ISceneBlobTable* blobs)
        {
            const std::vector<EntityId> ids = world.entities();
            std::unordered_map<EntityId, int> index_of;
            for (std::size_t i = 0; i < ids.size(); ++i)
                index_of.emplace(ids[i], static_cast<int>(i));

            json root = json::array();
            for (const EntityId id : ids)
                root.push_back(Detail::write_entity_record(world, id, index_of, blobs));

            // The environment rides every capture beside the entities. This is what makes
            // undo, Save Scene, and Play→Stop agree that lighting/sky/weather physics are
            // scene content: a snapshot of the entity array alone cannot restore what its
            // callers claim it restores, and a single Ctrl+Z would rewind the world while
            // leaving the environment exactly as it was.
            json capture;
            capture["entities"] = std::move(root);
            capture["environment"] = environment_to_json(world.environment());
            return capture;
        }

        void apply_scene(IWorldEditor& world, const json& root, const ISceneBlobTable* blobs)
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
                created.push_back(Detail::read_entity_record(world, entry, blobs));

            for (std::size_t i = 0; i < entity_list.size(); ++i)
                Detail::link_entity_record(world, entity_list[i], created[i], created);
        }

        bool save_scene(IWorldEditor& world, const std::string& path, const SceneSkyState* sky)
        {
            std::ofstream file(path);
            if (!file)
                return false;

            // capture_scene already carries the entities and the environment; the file
            // adds only what a disk scene has that an in-memory snapshot does not — the
            // weather sidecar reference and the sky authoring state.
            //
            // No blob table on purpose: a cooked soft-body asset goes into the file by
            // value, as base64. A `.sushiscene` has to be openable on its own, and a hash
            // into a table that lives in the editor session that wrote it is not that.
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
             * @brief Replaces every texture slot of @p target with @p source's.
             *
             * A crowd character's maps are named only inside its own glTF, so they come back
             * off a re-import of that file rather than from nine paths beside them. Only the
             * slots move: the authored values around them — tint, roughness, blend state —
             * stay exactly as the scene wrote them.
             *
             * @param target The material whose slots are overwritten.
             * @param source The freshly imported material the ids are taken from.
             */
            void adopt_material_maps(SushiEngine::Render::Material& target,
                                     const SushiEngine::Render::Material& source)
            {
                target.albedo_map = source.albedo_map;
                target.metallic_roughness_map = source.metallic_roughness_map;
                target.normal_map = source.normal_map;
                target.height_map = source.height_map;
                target.occlusion_map = source.occlusion_map;
                target.emissive_map = source.emissive_map;
                target.detail_albedo_map = source.detail_albedo_map;
                target.detail_normal_map = source.detail_normal_map;
                target.detail_mask_map = source.detail_mask_map;
            }

        } // namespace

        /**
         * @brief Re-resolves every file-backed render handle after a load from disk.
         *
         * The capture carries both a path and the handle it had when written; only the path
         * survives a session, so the handles a file was written with are re-derived here —
         * particle sprites, material maps, decal maps, a crowd's skinned mesh, and a Shape's
         * imported mesh alike, an empty path resolving to no asset rather than keeping the
         * stale handle. A post-pass rather than a hook inside `apply_scene` because the
         * in-memory snapshots that share that function are same-session, where the captured
         * handles are still the right ones. A crowd's skeleton and clip are absent here on
         * purpose: those need no render library, so `set_crowd_parameters` has already
         * re-registered them.
         *
         * @param world  The freshly loaded world whose components are re-pointed.
         * @param assets The library every path is resolved through.
         */
        void resolve_scene_assets(IWorldEditor& world,
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
                    SushiEngine::VFX::ParticleEffect effect = world.particle_effect_source(id);
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
                        resolve(paths.albedo_map, TextureColorSpace::SRGB);
                    material.metallic_roughness_map =
                        resolve(paths.metallic_roughness_map, TextureColorSpace::Linear);
                    material.normal_map =
                        resolve(paths.normal_map, TextureColorSpace::Linear);
                    material.height_map =
                        resolve(paths.height_map, TextureColorSpace::Linear);
                    material.occlusion_map =
                        resolve(paths.occlusion_map, TextureColorSpace::Linear);
                    material.emissive_map =
                        resolve(paths.emissive_map, TextureColorSpace::SRGB);
                    material.detail_albedo_map =
                        resolve(paths.detail_albedo_map, TextureColorSpace::SRGB);
                    material.detail_normal_map =
                        resolve(paths.detail_normal_map, TextureColorSpace::Linear);
                    material.detail_mask_map =
                        resolve(paths.detail_mask_map, TextureColorSpace::Linear);
                    world.set_material(id, material);
                }

                if (world.has_decal(id))
                {
                    SushiEngine::Simulation::DecalParameters decal = world.decal_parameters(id);
                    if (decal.albedo_map != INVALID_TEXTURE ||
                        decal.orm_map != INVALID_TEXTURE ||
                        !decal.albedo_map_path.empty() || !decal.orm_map_path.empty())
                    {
                        decal.albedo_map =
                            resolve(decal.albedo_map_path, TextureColorSpace::SRGB);
                        decal.orm_map =
                            resolve(decal.orm_map_path, TextureColorSpace::Linear);
                        world.set_decal_parameters(id, decal);
                    }
                }

                if (world.has_crowd(id))
                {
                    SushiEngine::Simulation::CrowdParameters crowd =
                        world.crowd_parameters(id);
                    // Skin 0, the one `register_crowd_skeleton` cooks its rig from: a mesh
                    // taken from a different skin would be posed by the wrong joints. An
                    // unnamed or unimportable file leaves the invalid mesh and empty maps
                    // seeded here, so the crowd draws nothing rather than drawing with
                    // whatever now holds the ids the scene was written with.
                    SushiEngine::Render::MeshId meshes[1] = {
                        SushiEngine::Render::INVALID_MESH};
                    SushiEngine::Render::Material imported[1]{};
                    if (!crowd.mesh_path.empty())
                        (void)assets.load_gltf_skinned_mesh(crowd.mesh_path.c_str(), 0,
                                                            meshes, imported, 1);
                    crowd.mesh = meshes[0];
                    adopt_material_maps(crowd.material, imported[0]);
                    world.set_crowd_parameters(id, crowd);
                }

                if (world.has_shape(id))
                {
                    SushiEngine::Simulation::ShapeParameters shape =
                        world.shape_parameters(id);
                    // Guarded like the Material and Decal blocks above: an ordinary
                    // Box/Sphere/Cylinder Shape has no imported-mesh state to resolve, and
                    // `set_shape_parameters` triggers a full extract -- paying for that on
                    // every Shape in the scene, which is nearly every entity in most
                    // scenes, on every load would be a needless full-extract pass.
                    if (shape.mesh != SushiEngine::Render::INVALID_MESH ||
                        !shape.mesh_path.empty())
                    {
                        // Unlike a crowd's mesh, the imported material is not adopted onto
                        // the Shape's own Material: a Shape always has its own authored,
                        // serialized Material already, and silently overwriting it on every
                        // re-import would make it a value the file cannot actually hold
                        // still. Mirrors the editor's own `bind_shape_mesh`.
                        // Joined on the file's own node and primitive indices, through the
                        // import that does not bake a node's world transform into its
                        // vertices: the entity's transform already carries the placement,
                        // and a baked one would apply it twice.
                        shape.mesh = SushiEngine::Render::resolve_imported_mesh(
                            assets, shape.mesh_path.c_str(), shape.source_node,
                            shape.primitive);
                        world.set_shape_parameters(id, shape);
                    }
                }
            }
        }

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
                // The refresh first: it destroys and rebuilds subtrees, and the entities it
                // creates carry a path and no handle. Resolving before it would resolve the
                // ones it is about to replace and leave the replacements with no geometry.
                (void)refresh_prefab_instances(world);
                if (assets != nullptr)
                    resolve_scene_assets(world, *assets);
                return true;
            }

            if (!root.is_object() || !root.contains("entities") || !root["entities"].is_array())
                return false;

            // apply_scene restores the entities and the environment together (the same
            // object shape capture_scene produces); the file-only extras follow.
            apply_scene(world, root);
            // Beside resolve_scene_assets and for the same reason: both are post-load passes
            // that must not run in apply_scene, which is the path undo restores through.
            // Refreshing there would reinstate the very change being undone. `load_scene` has
            // nowhere to report to, so the unreadable paths are dropped here; the editor calls
            // this directly and surfaces them.
            //
            // Before the resolve, not after: a rebuilt subtree's Shapes carry a path and no
            // handle, so resolving first would resolve the entities the refresh is about to
            // destroy and leave the ones it creates with nothing to draw.
            (void)refresh_prefab_instances(world);
            if (assets != nullptr)
                resolve_scene_assets(world, *assets);
            if (root.contains("weather"))
                weather_from_json(root["weather"], world, path);
            if (sky != nullptr && root.contains("sky"))
                *sky = sky_from_json(root["sky"], *sky);
            return true;
        }
    } // namespace Scene
} // namespace SushiEngine
