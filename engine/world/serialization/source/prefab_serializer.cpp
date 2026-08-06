/**************************************************************************/
/* prefab_serializer.cpp                                                  */
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

#include "prefab_serializer.hpp"

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "byte_encoding.hpp"
#include "entity_record.hpp"

namespace SushiEngine
{
    namespace Scene
    {
        using SushiEngine::Simulation::EntityId;
        using SushiEngine::Simulation::EntityTransform;
        using SushiEngine::Simulation::IWorldEditor;
        using SushiEngine::Simulation::NULL_ENTITY;
        using SushiEngine::Simulation::PrefabInstanceParameters;
        using json = nlohmann::json;

        namespace
        {
            /**
             * @brief @p root and its descendants, parents before children.
             *
             * `IWorldEditor` answers `parent` but not `children`, and `entities()` is in
             * creation order, so the child lists are built once up front rather than by
             * rescanning the world per node — an import of a few thousand nodes would
             * otherwise walk the world a few thousand times.
             */
            std::vector<EntityId> subtree_of(IWorldEditor& world, EntityId root)
            {
                std::vector<EntityId> ordered;
                const std::vector<EntityId> all = world.entities();
                // Not NULL_ENTITY is not the same as live: a destroyed id is neither, and a
                // capture of one would write a record of whatever the world answers for it.
                if (root == NULL_ENTITY || std::find(all.begin(), all.end(), root) == all.end())
                    return ordered;

                std::unordered_map<EntityId, std::vector<EntityId>> children;
                for (const EntityId id : all)
                    children[world.parent(id)].push_back(id);

                std::vector<EntityId> pending{root};
                while (!pending.empty())
                {
                    const EntityId id = pending.back();
                    pending.pop_back();
                    ordered.push_back(id);

                    const auto found = children.find(id);
                    if (found == children.end())
                        continue;
                    // Pushed in reverse so they pop in authored order, which keeps the
                    // document's entity order stable across captures of the same subtree —
                    // and the revision with it.
                    for (auto it = found->second.rbegin(); it != found->second.rend(); ++it)
                        pending.push_back(*it);
                }
                return ordered;
            }
        } // namespace

        json capture_prefab(IWorldEditor& world, EntityId root)
        {
            const std::vector<EntityId> ordered = subtree_of(world, root);

            std::unordered_map<EntityId, int> index_of;
            for (std::size_t i = 0; i < ordered.size(); ++i)
                index_of.emplace(ordered[i], static_cast<int>(i));

            json entities = json::array();
            for (std::size_t i = 0; i < ordered.size(); ++i)
            {
                // No blob table: a prefab has to open on another machine, so a cooked asset
                // goes in by value, the same choice `save_scene` makes for a scene file.
                json entry = Detail::write_entity_record(world, ordered[i], index_of, nullptr);
                // A sequential index, not a random or time-based value: the identifier has to
                // be unique within the document and stable across reads, and anything varying
                // between two captures of the same subtree would change the revision with it.
                entry["prefab_entity_id"] = "e" + std::to_string(i);
                entities.push_back(std::move(entry));
            }

            json document;
            document["revision"] = prefab_revision(entities);
            document["entities"] = std::move(entities);
            return document;
        }

        EntityId apply_prefab(IWorldEditor& world, const json& document, EntityId parent)
        {
            const json& entities = document.is_object() && document.contains("entities")
                                       ? document["entities"]
                                       : document;
            if (!entities.is_array() || entities.empty())
                return NULL_ENTITY;

            // The same two passes `apply_scene` runs, without its wholesale clear: that clear
            // is what makes a scene load a replacement, and a prefab placement is an addition.
            std::vector<EntityId> created;
            created.reserve(entities.size());
            for (const auto& entry : entities)
                created.push_back(Detail::read_entity_record(world, entry, nullptr));

            for (std::size_t i = 0; i < entities.size(); ++i)
                Detail::link_entity_record(world, entities[i], created[i], created);

            // After the link pass, not before: entry 0's own `parent` is -1, so that pass
            // leaves it a root and this places it.
            world.set_parent(created.front(), parent);
            return created.front();
        }

        std::string prefab_revision(const json& entities)
        {
            return std::to_string(Detail::content_hash(entities.dump()));
        }

        bool read_prefab_revision(const std::string& path, std::string& out)
        {
            // Cleared first, so a caller that ignores the return does not compare against the
            // revision it happened to be holding and conclude the instance is current.
            out.clear();

            std::ifstream file(path);
            if (!file)
                return false;

            json document;
            try
            {
                file >> document;
            }
            catch (const json::parse_error&)
            {
                return false;
            }

            if (!document.is_object() || !document.contains("revision") ||
                !document["revision"].is_string())
                return false;

            out = document["revision"].get<std::string>();
            return !out.empty();
        }

        std::vector<std::string> refresh_prefab_instances(IWorldEditor& world)
        {
            std::vector<std::string> unreadable;

            // Snapshotted first: the body destroys and creates entities, and walking a list
            // it invalidates is the bug this line exists to prevent. An entity destroyed as
            // part of an enclosing instance is simply no longer live, and `has_prefab_instance`
            // answers false for it.
            const std::vector<EntityId> candidates = world.entities();
            for (const EntityId id : candidates)
            {
                if (!world.has_prefab_instance(id))
                    continue;
                const PrefabInstanceParameters link = world.prefab_instance(id);

                // The cheap read first, for every instance; the whole document only for the
                // stale ones. Opening a scene otherwise scales with the size of its prefabs
                // rather than their number.
                std::string current;
                if (!read_prefab_revision(link.path, current))
                {
                    unreadable.push_back(link.path);
                    continue;
                }
                if (current == link.revision)
                    continue;

                json document;
                {
                    std::ifstream file(link.path);
                    try
                    {
                        file >> document;
                    }
                    catch (const json::parse_error&)
                    {
                        // A file whose revision parsed but whose body did not is still a
                        // prefab this pass cannot use, and the subtree stays as it is.
                        unreadable.push_back(link.path);
                        continue;
                    }
                }

                // The placement belongs to the scene and the content to the file, so the two
                // are pulled apart here: everything below is rebuilt, and these three survive.
                const std::string name = world.name(id);
                const EntityTransform placement = world.transform(id);
                const EntityId parent = world.parent(id);

                // Bottom-up, because `destroy` leaves a destroyed parent's children as roots
                // rather than cascading: destroying the root first would scatter the old
                // subtree into the scene instead of removing it.
                const std::vector<EntityId> previous = subtree_of(world, id);
                for (auto it = previous.rbegin(); it != previous.rend(); ++it)
                    world.destroy(*it);

                // The root is replaced rather than kept and re-dressed, so a prefab whose own
                // root carries geometry keeps it — which is every prefab authored from a
                // single object, and would otherwise refresh into an empty entity.
                const EntityId rebuilt = apply_prefab(world, document, parent);
                if (rebuilt == NULL_ENTITY)
                {
                    unreadable.push_back(link.path);
                    continue;
                }

                world.set_name(rebuilt, name);
                world.set_transform(rebuilt, placement);
                PrefabInstanceParameters refreshed;
                refreshed.path = link.path;
                refreshed.revision = current;
                world.set_prefab_instance(rebuilt, refreshed);
            }
            return unreadable;
        }
    } // namespace Scene
} // namespace SushiEngine
