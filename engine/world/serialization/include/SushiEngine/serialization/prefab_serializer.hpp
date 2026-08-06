/**************************************************************************/
/* prefab_serializer.hpp                                                  */
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

#pragma once

/**
 * @file prefab_serializer.hpp
 * @brief An entity subtree as a reusable asset.
 *
 * A prefab document is one rooted slice of the entity array `capture_scene` writes, built from
 * the same per-entity record, so a field added to an entity is carried by prefabs the day it is
 * added to scenes. The environment is deliberately absent — a prefab is a subtree, and a street
 * light carrying the sky it was authored under would apply that sky wherever it is placed.
 */

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Scene
    {
        /**
         * @brief Captures @p root and every descendant as a prefab document.
         *
         * Adds a `prefab_entity_id` per entry: a value unique within the document, which
         * override resolution will key on. Nothing reads it in this phase and it is written
         * anyway, because a prefab authored without one cannot be matched afterwards.
         *
         * @param world The world to read.
         * @param root  The subtree's root; entry 0 of the result.
         * @return An object `{ "revision": "...", "entities": [...] }`, or one whose `entities`
         *     array is empty when @p root is not a live entity.
         */
        nlohmann::json capture_prefab(SushiEngine::Simulation::IWorldEditor& world,
                                      SushiEngine::Simulation::EntityId root);

        /**
         * @brief Builds a prefab document's entities under @p parent.
         *
         * Nothing existing is destroyed: a caller replacing a subtree removes the old one first.
         *
         * @param world    The world to populate.
         * @param document An object in the shape @ref capture_prefab produces.
         * @param parent   The entity to hang the subtree under, or `NULL_ENTITY` for a root.
         * @return The created root, or `NULL_ENTITY` when the document holds no entity.
         */
        SushiEngine::Simulation::EntityId apply_prefab(
            SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& document,
            SushiEngine::Simulation::EntityId parent);

        /**
         * @brief The content hash of a prefab document's entity array.
         *
         * A hash rather than a counter, so the same content hashes the same on two machines and
         * reverting a prefab restores its previous revision instead of advancing past it — which
         * is what a staleness comparison needs and what a counter gets wrong in both cases.
         *
         * @param entities The document's `entities` array.
         * @return The revision string.
         */
        std::string prefab_revision(const nlohmann::json& entities);

        /**
         * @brief Reads just the revision out of a prefab file.
         *
         * Separate from a full read because the refresh pass asks "is this instance stale" for
         * every instance in a scene and rebuilds only the ones that are; parsing whole documents
         * to answer a string comparison would make opening a scene scale with the size of its
         * prefabs rather than their number.
         *
         * @param path Path to a `.sushiprefab` file.
         * @param out  Receives the revision; cleared on failure.
         * @return False when the file cannot be read or parsed, or carries no revision.
         */
        bool read_prefab_revision(const std::string& path, std::string& out);

        /**
         * @brief Rebuilds every prefab instance in @p world whose revision no longer matches
         *     its file.
         *
         * For each entity carrying `PrefabInstanceParameters`: read the prefab's current
         * revision, and when it differs, replace the whole subtree from the file. The
         * instance's name and transform are written back over the rebuilt root — they are its
         * placement, not the prefab's content, and a rebuild that moved every street light
         * back to the origin would be one nobody could use. Everything else is the file's,
         * including the root's own components, which is why the root is replaced rather than
         * kept and re-dressed.
         *
         * Call this from `load_scene` and not from `apply_scene`. `apply_scene` is the path
         * undo restores through, and refreshing there would reinstate the very change being
         * undone.
         *
         * @param world The world to refresh.
         * @return The paths of prefabs that could not be read, in encounter order. Their
         *     instances are left exactly as they were, so a file that is merely missing
         *     unlinks a subtree in the editor's display rather than deleting it or severing
         *     the link a restored file would resolve.
         */
        std::vector<std::string> refresh_prefab_instances(
            SushiEngine::Simulation::IWorldEditor& world);
    } // namespace Scene
} // namespace SushiEngine
