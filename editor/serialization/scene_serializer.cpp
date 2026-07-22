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

#include <cstdio>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

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

            // The Environment fields an author edits in the Lighting/Environment panels
            // (sun, atmosphere, ground, clouds, stars, night lighting, ambient, exposure,
            // IBL). `bodies`/`sky_stars`/`dominant_*` are excluded: the ephemeris repopulates
            // those every frame from SceneSkyState and are not author state.
            json environment_to_json(const SushiEngine::Render::Environment& e)
            {
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
                          {"moon_intensity", e.night.moon_intensity},
                          {"star_intensity", e.night.star_intensity}}},
                    {"ambient", vec3_to_json(e.ambient)},
                    {"exposure", e.exposure},
                    {"image_based_lighting", e.image_based_lighting},
                    {"ibl_intensity", e.ibl_intensity}};
            }

            // Applies the persisted fields onto `environment`, leaving every field the JSON
            // omits (including the ephemeris-owned fields never written above) at whatever
            // value the caller passed in.
            SushiEngine::Render::Environment environment_from_json(
                const json& j, SushiEngine::Render::Environment environment)
            {
                if (!j.is_object())
                    return environment;

                if (j.contains("sun") && j["sun"].is_object())
                {
                    const json& s = j["sun"];
                    if (s.contains("direction"))
                        environment.sun.direction = vec3_from_json(s["direction"]);
                    if (s.contains("color"))
                        environment.sun.color = vec3_from_json(s["color"]);
                    environment.sun.intensity = s.value("intensity", environment.sun.intensity);
                }
                environment.planet_surface_style = static_cast<SushiEngine::Render::SurfaceStyle>(
                    j.value("planet_surface_style",
                            static_cast<std::uint32_t>(environment.planet_surface_style)));
                if (j.contains("atmosphere") && j["atmosphere"].is_object())
                {
                    const json& a = j["atmosphere"];
                    environment.atmosphere.enabled = a.value("enabled", environment.atmosphere.enabled);
                    environment.atmosphere.height = a.value("height", environment.atmosphere.height);
                    if (a.contains("rayleigh_coefficient"))
                        environment.atmosphere.rayleigh_coefficient =
                            vec3_from_json(a["rayleigh_coefficient"]);
                    environment.atmosphere.mie_coefficient =
                        a.value("mie_coefficient", environment.atmosphere.mie_coefficient);
                    environment.atmosphere.mie_anisotropy =
                        a.value("mie_anisotropy", environment.atmosphere.mie_anisotropy);
                    environment.atmosphere.rayleigh_scale_height =
                        a.value("rayleigh_scale_height", environment.atmosphere.rayleigh_scale_height);
                    environment.atmosphere.mie_scale_height =
                        a.value("mie_scale_height", environment.atmosphere.mie_scale_height);
                }
                if (j.contains("surface") && j["surface"].is_object())
                {
                    const json& s = j["surface"];
                    if (s.contains("ground_albedo"))
                        environment.surface.ground_albedo = vec3_from_json(s["ground_albedo"]);
                    if (s.contains("ocean_color"))
                        environment.surface.ocean_color = vec3_from_json(s["ocean_color"]);
                    environment.surface.roughness = s.value("roughness", environment.surface.roughness);
                }
                if (j.contains("clouds") && j["clouds"].is_object())
                {
                    const json& c = j["clouds"];
                    environment.clouds.enabled = c.value("enabled", environment.clouds.enabled);
                    environment.clouds.light_absorption =
                        c.value("light_absorption", environment.clouds.light_absorption);
                    environment.clouds.forward_scattering =
                        c.value("forward_scattering", environment.clouds.forward_scattering);
                    environment.clouds.powder_strength =
                        c.value("powder_strength", environment.clouds.powder_strength);
                    environment.clouds.ambient_strength =
                        c.value("ambient_strength", environment.clouds.ambient_strength);
                    environment.clouds.ground_shadow_strength =
                        c.value("ground_shadow_strength", environment.clouds.ground_shadow_strength);
                    environment.clouds.weather_scale =
                        c.value("weather_scale", environment.clouds.weather_scale);
                    environment.clouds.evolution_rate =
                        c.value("evolution_rate", environment.clouds.evolution_rate);
                    if (c.contains("decks") && c["decks"].is_array())
                    {
                        const json& decks = c["decks"];
                        for (int i = 0; i < SushiEngine::Render::CLOUD_MAX_DECKS &&
                                        static_cast<std::size_t>(i) < decks.size(); ++i)
                        {
                            const json& d = decks[static_cast<std::size_t>(i)];
                            SushiEngine::Render::CloudDeck& deck = environment.clouds.decks[i];
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
                    const json& s = j["stars"];
                    environment.stars.enabled = s.value("enabled", environment.stars.enabled);
                    environment.stars.brightness = s.value("brightness", environment.stars.brightness);
                    environment.stars.density = s.value("density", environment.stars.density);
                }
                if (j.contains("night") && j["night"].is_object())
                {
                    const json& n = j["night"];
                    environment.night.enabled = n.value("enabled", environment.night.enabled);
                    environment.night.moon_intensity =
                        n.value("moon_intensity", environment.night.moon_intensity);
                    environment.night.star_intensity =
                        n.value("star_intensity", environment.night.star_intensity);
                }
                if (j.contains("ambient"))
                    environment.ambient = vec3_from_json(j["ambient"]);
                environment.exposure = j.value("exposure", environment.exposure);
                environment.image_based_lighting =
                    j.value("image_based_lighting", environment.image_based_lighting);
                environment.ibl_intensity = j.value("ibl_intensity", environment.ibl_intensity);
                return environment;
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

            return root;
        }

        void apply_scene(IWorldEditor& world, const json& root)
        {
            // Replace the world wholesale: clear every existing entity before
            // recreating the file's, so a load is never a merge with the prior scene.
            for (const EntityId id : world.entities())
                world.destroy(id);

            std::vector<EntityId> created;
            created.reserve(root.size());
            for (const auto& entry : root)
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
            for (std::size_t i = 0; i < root.size(); ++i)
            {
                const int parent_index = root[i].value("parent", -1);
                if (parent_index >= 0 && static_cast<std::size_t>(parent_index) < created.size())
                    world.set_parent(created[i], created[static_cast<std::size_t>(parent_index)]);
            }
        }

        bool save_scene(IWorldEditor& world, const std::string& path, const SceneSkyState* sky)
        {
            std::ofstream file(path);
            if (!file)
                return false;

            json root;
            root["entities"] = capture_scene(world);
            root["environment"] = environment_to_json(world.environment());
            if (sky != nullptr)
                root["sky"] = sky_to_json(*sky);

            file << root.dump(2);
            return static_cast<bool>(file);
        }

        bool load_scene(IWorldEditor& world, const std::string& path, SceneSkyState* sky)
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
                return true;
            }

            if (!root.is_object() || !root.contains("entities") || !root["entities"].is_array())
                return false;

            apply_scene(world, root["entities"]);
            if (root.contains("environment"))
                world.set_environment(environment_from_json(root["environment"], world.environment()));
            if (sky != nullptr && root.contains("sky"))
                *sky = sky_from_json(root["sky"], *sky);
            return true;
        }
    } // namespace Editor
} // namespace SushiEngine
