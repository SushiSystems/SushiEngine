/**************************************************************************/
/* prefab_output.hpp                                                      */
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
 * @file prefab_output.hpp
 * @brief A glTF file on disk becomes a `.sushiprefab` beside it.
 *
 * The import path proper: read the file, read its `.meta`, plan what entities it becomes, build
 * them, and write the result as a prefab. Split from `Model::plan_model_instantiation` because
 * that function is pure — it reads no file and creates nothing, which is why its own tests run
 * without a simulation — and this one is not. The planner also lives a tier lower, where it
 * cannot reach `IWorldEditor` or the prefab shape at all.
 *
 * Changing an asset's `.meta` therefore changes its prefab's revision, and the refresh pass
 * rebuilds every instance of it the next time a scene is opened. Reimport is not a feature
 * anyone writes here; it is what these steps already do.
 */

#include <string>

#include <SushiEngine/gltf/scene_import.hpp>
#include <SushiEngine/model/instantiation_plan.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace ModelImport
    {
        /**
         * @brief Creates @p plan's entities in @p world and returns the subtree's root.
         *
         * Entities are created in plan order, which the planner guarantees is
         * parent-before-child, so a parent always exists when its child is parented.
         *
         * @param world       The world to populate.
         * @param plan        What to create.
         * @param description The file the plan was made from; its lights and cameras are what
         *     the plan's entries index into.
         * @param source_path The asset the plan came from. Becomes each Shape's `mesh_path`,
         *     which `resolve_scene_assets` re-derives the live mesh handle from — together
         *     with the node and primitive indices each planned entity already carries.
         * @return The subtree's root, or `NULL_ENTITY` when the plan is empty.
         */
        SushiEngine::Simulation::EntityId instantiate_plan(
            SushiEngine::Simulation::IWorldEditor& world,
            const SushiEngine::Model::ModelInstantiationPlan& plan,
            const SushiEngine::Geometry::GLTFSceneDescription& description,
            const std::string& source_path);

        /**
         * @brief Imports @p asset_path and writes its hierarchy as `<asset_path>.sushiprefab`.
         *
         * The whole path with the extension appended, matching `.meta`'s convention, so
         * `models/Car.gltf` yields `models/Car.gltf.sushiprefab` and a `.glb` of the same stem
         * cannot collide with it.
         *
         * @param asset_path Path to a `.gltf` or `.glb`.
         * @param report     Receives the import's counts and warnings; overwritten.
         * @return False when the file cannot be read, holds no node, or the prefab cannot be
         *     written. Nothing is written in any failing case, rather than a truncated file
         *     that every scene referencing it would then fail to parse.
         */
        bool write_model_prefab(const std::string& asset_path,
                                SushiEngine::Model::ModelImportReport& report);
    } // namespace ModelImport
} // namespace SushiEngine
