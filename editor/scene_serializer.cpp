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

#include <fstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace sushi::editor
{
    namespace
    {
        using nlohmann::json;
        using SushiEngine::sim::EntityId;
        using SushiEngine::sim::IWorldEditor;
        using SushiEngine::sim::NULL_ENTITY;

        json vec3_to_json(const SushiEngine::Vec3& v)
        {
            return json{{"x", v.x}, {"y", v.y}, {"z", v.z}};
        }

        SushiEngine::Vec3 vec3_from_json(const json& j)
        {
            SushiEngine::Vec3 v;
            v.x = j.value("x", SushiEngine::Scalar(0));
            v.y = j.value("y", SushiEngine::Scalar(0));
            v.z = j.value("z", SushiEngine::Scalar(0));
            return v;
        }

        json quat_to_json(const SushiEngine::Quat& q)
        {
            return json{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}};
        }

        SushiEngine::Quat quat_from_json(const json& j)
        {
            SushiEngine::Quat q;
            q.x = j.value("x", SushiEngine::Scalar(0));
            q.y = j.value("y", SushiEngine::Scalar(0));
            q.z = j.value("z", SushiEngine::Scalar(0));
            q.w = j.value("w", SushiEngine::Scalar(1));
            return q;
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
            entry["rotation"] = quat_to_json(transform.rotation);
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
            else
            {
                entry["color"] = vec3_to_json(world.color(id));
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

            SushiEngine::sim::EntityTransform transform;
            if (entry.contains("position"))
                transform.position = vec3_from_json(entry["position"]);
            if (entry.contains("rotation"))
                transform.rotation = quat_from_json(entry["rotation"]);
            if (entry.contains("scale"))
                transform.scale = vec3_from_json(entry["scale"]);
            world.set_transform(id, transform);
            world.set_visible(id, entry.value("visible", true));

            if (is_camera && entry.contains("camera"))
            {
                const json& c = entry["camera"];
                SushiEngine::sim::CameraParams params;
                params.vertical_fov_radians =
                    c.value("vertical_fov_radians", params.vertical_fov_radians);
                params.near_plane = c.value("near_plane", params.near_plane);
                params.far_plane = c.value("far_plane", params.far_plane);
                params.display_index = c.value("display_index", params.display_index);
                params.priority = c.value("priority", params.priority);
                params.active = c.value("active", params.active);
                world.set_camera_params(id, params);
            }
            else if (!is_camera && entry.contains("color"))
            {
                world.set_color(id, vec3_from_json(entry["color"]));
            }
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
} // namespace sushi::editor
