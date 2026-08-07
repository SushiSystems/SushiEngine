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
#include <unordered_set>
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

            // The identifiers, decided before anything is written, in two passes.
            //
            // An entity that already carries one keeps it. That is what lets an id survive a
            // re-author: an artist who reorders a prefab's contents and saves again keeps every
            // id, so every instance's overrides stay attached to the entity they were made
            // against. Reminting on every save would silently retarget all of them.
            //
            // The revision is a content hash of this array, and preserving an id leaves it every
            // bit as stable as a positional one did — what would move the hash is *generating* a
            // fresh id per capture, which is what the previous sequential scheme avoided and this
            // one avoids the same way.
            std::vector<std::string> identity(ordered.size());
            std::unordered_set<std::string> used;
            for (std::size_t i = 0; i < ordered.size(); ++i)
            {
                const std::string existing = world.prefab_entity_id(ordered[i]);
                if (!existing.empty() && used.insert(existing).second)
                    identity[i] = existing;
            }
            std::size_t next = 0;
            for (std::size_t i = 0; i < ordered.size(); ++i)
            {
                if (!identity[i].empty())
                    continue;
                std::string candidate;
                do
                {
                    candidate = "e" + std::to_string(next++);
                } while (!used.insert(candidate).second);
                identity[i] = candidate;
            }

            // Written back, so a subtree authored into a prefab for the first time carries the
            // same identifiers the file does. Without this the source entities stay anonymous and
            // the very next save remints every one of them.
            for (std::size_t i = 0; i < ordered.size(); ++i)
                world.set_prefab_entity_id(ordered[i], identity[i]);

            json entities = json::array();
            for (std::size_t i = 0; i < ordered.size(); ++i)
            {
                // No blob table: a prefab has to open on another machine, so a cooked asset
                // goes in by value, the same choice `save_scene` makes for a scene file.
                json entry = Detail::write_entity_record(world, ordered[i], index_of, nullptr);
                entry["prefab_entity_id"] = identity[i];
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

                const std::vector<EntityId> previous = subtree_of(world, id);

                // The document's entries, indexed by the identity each carries. A prefab
                // written before identities existed has none, and every member of every
                // instance of it is then a survivor — which is the same outcome as
                // today's wholesale rebuild except that the entities live.
                json& entries = document.is_object() && document.contains("entities")
                                    ? document["entities"]
                                    : document;
                std::unordered_map<std::string, std::size_t> entry_of;
                if (entries.is_array())
                {
                    for (std::size_t i = 0; i < entries.size(); ++i)
                    {
                        const std::string key =
                            entries[i].value("prefab_entity_id", std::string{});
                        if (!key.empty())
                            entry_of.emplace(key, i);
                    }
                }

                // Overrides are computed here rather than looked up, and that is the whole
                // of P2's design: nothing recorded this member as edited, so no component
                // setter had to know prefabs exist. What the author changed is simply what
                // this member's record and the prefab's disagree about.
                std::unordered_map<EntityId, int> index_of;
                for (std::size_t i = 0; i < previous.size(); ++i)
                    index_of.emplace(previous[i], static_cast<int>(i));

                std::vector<EntityId> survivors;
                for (const EntityId member : previous)
                {
                    const std::string key = world.prefab_entity_id(member);
                    const auto found = key.empty() ? entry_of.end() : entry_of.find(key);
                    if (found == entry_of.end())
                    {
                        // Either the artist removed this entity from the prefab, or the
                        // author added it to this instance by hand. Both keep the entity
                        // and cut its link; the second case is destroyed silently today.
                        survivors.push_back(member);
                        continue;
                    }

                    const json live =
                        Detail::write_entity_record(world, member, index_of, nullptr);
                    json& target = entries[found->second];
                    for (auto field = live.begin(); field != live.end(); ++field)
                    {
                        // `parent` is excluded because it is an index, and the two records
                        // number their entities differently — the live one by subtree
                        // order, the document by document order. Copying it across would
                        // not be an override, it would be a reparent to a stranger.
                        // `prefab_entity_id` is identity rather than content.
                        if (field.key() == "parent" || field.key() == "prefab_entity_id")
                            continue;
                        if (!target.contains(field.key()) ||
                            target[field.key()] != field.value())
                            target[field.key()] = field.value();
                    }
                }

                // A survivor's parent among the survivors keeps it; one whose parent is
                // rebuilt has to be reattached, and that is decided now while the old
                // hierarchy still exists.
                std::unordered_set<EntityId> surviving;
                for (const EntityId member : survivors)
                    surviving.insert(member);
                std::vector<EntityId> detached_roots;
                for (const EntityId member : survivors)
                {
                    if (surviving.find(world.parent(member)) == surviving.end())
                        detached_roots.push_back(member);
                }

                // Bottom-up, because `destroy` leaves a destroyed parent's children as roots
                // rather than cascading: destroying the root first would scatter the old
                // subtree into the scene instead of removing it.
                for (auto it = previous.rbegin(); it != previous.rend(); ++it)
                {
                    if (surviving.find(*it) == surviving.end())
                        world.destroy(*it);
                }

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

                // Reattached under the new root and unlinked, in that order. Clearing the
                // identity is what "unlinked" means: a later refresh must not rematch an
                // entity the prefab no longer claims, and an author who wants it back
                // puts it back in the prefab and accepts the second copy.
                for (const EntityId survivor : detached_roots)
                {
                    world.set_parent(survivor, rebuilt);
                    world.set_prefab_entity_id(survivor, std::string{});
                }

                PrefabInstanceParameters refreshed;
                refreshed.path = link.path;
                refreshed.revision = current;
                world.set_prefab_instance(rebuilt, refreshed);
            }
            return unreadable;
        }
    } // namespace Scene
} // namespace SushiEngine
