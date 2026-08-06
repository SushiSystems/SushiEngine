/**************************************************************************/
/* entity_record.hpp                                                      */
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
 * @file entity_record.hpp
 * @brief One entity's JSON record: written, read back, and linked to its neighbours.
 *
 * Private to this module. It exists so a scene and a prefab share one record shape instead of
 * two that drift: a field added to an entity is carried by both the day it is added to one.
 *
 * Reading is two functions because a record's links are array indices into the document, so
 * neither end can be resolved until every entity exists. @ref read_entity_record builds an
 * entity from its own entry alone; @ref link_entity_record runs afterwards over the same
 * entries.
 */

#include <unordered_map>
#include <vector>

#include <nlohmann/json.hpp>

#include <SushiEngine/simulation/simulation.hpp>

#include "scene_blob_table.hpp"

namespace SushiEngine
{
    namespace Scene
    {
        namespace Detail
        {
            /**
             * @brief Writes @p id as one entity record.
             *
             * @param world    The world to read.
             * @param id       The entity to write.
             * @param index_of Document index for every entity the document contains. A parent
             *     absent from the map is written as -1, which is how a subtree's root — whose
             *     parent lies outside the document — becomes a root in the document.
             * @param blobs    Where a cooked asset is named rather than inlined, or nullptr to
             *     inline it.
             * @return The record.
             */
            nlohmann::json write_entity_record(
                SushiEngine::Simulation::IWorldEditor& world,
                SushiEngine::Simulation::EntityId id,
                const std::unordered_map<SushiEngine::Simulation::EntityId, int>& index_of,
                ISceneBlobTable* blobs);

            /**
             * @brief Creates one entity from a record and applies every component it carries.
             *
             * Leaves the record's index-based links alone; @ref link_entity_record resolves
             * them.
             *
             * @param world The world to populate.
             * @param entry The record.
             * @param blobs Where a named cooked asset is resolved from, or nullptr.
             * @return The created entity.
             */
            SushiEngine::Simulation::EntityId read_entity_record(
                SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& entry,
                const ISceneBlobTable* blobs);

            /**
             * @brief Resolves a record's index-based links once every entity exists.
             *
             * @param world   The world to update.
             * @param entry   The record.
             * @param id      The entity @p entry was read into.
             * @param created Every entity of the document, in document order.
             */
            void link_entity_record(
                SushiEngine::Simulation::IWorldEditor& world, const nlohmann::json& entry,
                SushiEngine::Simulation::EntityId id,
                const std::vector<SushiEngine::Simulation::EntityId>& created);
        } // namespace Detail
    } // namespace Scene
} // namespace SushiEngine
