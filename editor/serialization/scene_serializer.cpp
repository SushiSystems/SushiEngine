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
                                                 {"inv_inertia", vec3_to_json(params.inv_inertia)}};
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

        bool save_scene(IWorldEditor& world, const std::string& path)
        {
            std::ofstream file(path);
            if (!file)
                return false;
            file << capture_scene(world).dump(2);
            return static_cast<bool>(file);
        }

        bool load_scene(IWorldEditor& world, const std::string& path)
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
            if (!root.is_array())
                return false;

            apply_scene(world, root);
            return true;
        }
    } // namespace Editor
} // namespace SushiEngine
