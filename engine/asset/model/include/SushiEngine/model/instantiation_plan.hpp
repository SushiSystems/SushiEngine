/**************************************************************************/
/* instantiation_plan.hpp                                                 */
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
 * @file instantiation_plan.hpp
 * @brief What entities a glTF file becomes, decided without creating any.
 *
 * The whole of the import's decision-making: which node becomes which entity, when a node
 * splits because it carries more materials than one entity can, how a dropped pivot folds its
 * transform into its children, how a name collision resolves, and where the scale factor goes.
 * Separated from the code that creates entities so all of it is testable with no device, no
 * window and no world — the editor's executor then has no branches worth testing.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/gltf/scene_import.hpp>
#include <SushiEngine/model/import_settings.hpp>

namespace SushiEngine
{
    namespace Model
    {
        /** @brief Unit quaternion in import precision; `Vector3f` comes from import_settings.hpp. */
        using Quaternionf = QuaternionT<float>;

        /** @brief What a planned entity carries beyond its name and transform. */
        enum class PlannedComponent
        {
            None,   /**< A pure transform: a pivot, or a node whose mesh split into children. */
            Shape,  /**< One imported primitive, drawn with one material. */
            Light,
            Camera
        };

        /** @brief One entity the import creates. */
        struct PlannedEntity
        {
            std::string name;

            /** @brief Index into @ref ModelInstantiationPlan::entities, or -1 for the root. */
            std::int32_t parent = -1;

            Vector3f translation{0.0f, 0.0f, 0.0f};
            Quaternionf rotation{0.0f, 0.0f, 0.0f, 1.0f};
            Vector3f scale{1.0f, 1.0f, 1.0f};

            PlannedComponent component = PlannedComponent::None;

            /** @brief The glTF node this came from; the key a renderer's import is joined on. */
            std::uint32_t source_node = 0;

            /** @brief Which primitive of that node's mesh, when @ref component is Shape. */
            std::uint32_t primitive = 0;

            /** @brief Index into `GLTFSceneDescription::lights`, when component is Light. */
            std::int32_t light = -1;

            /** @brief Index into `GLTFSceneDescription::cameras`, when component is Camera. */
            std::int32_t camera = -1;

            /** @brief Whether this entity's mesh should be queued for cooking (§9's setting). */
            bool generate_collider = false;
        };

        /** @brief The entities to create, parents always before their children. */
        struct ModelInstantiationPlan
        {
            std::vector<PlannedEntity> entities;
        };

        /** @brief What an import found, produced and could not use. */
        struct ModelImportReport
        {
            std::uint32_t nodes = 0;
            std::uint32_t entities = 0;
            std::uint32_t primitives_imported = 0;
            std::uint32_t lights_imported = 0;
            std::uint32_t lights_skipped_directional = 0;
            std::uint32_t cameras_imported = 0;
            std::uint32_t skinned_nodes_skipped = 0;
            std::uint32_t pivots_dropped = 0;
            std::uint32_t materials = 0;

            /** @brief Everything an artist has to be told, in the words they need. */
            std::vector<std::string> warnings;
        };

        /**
         * @brief Decides what entities a described glTF file becomes.
         *
         * Pure: reads no file, touches no device, creates nothing. Applies the rules the design
         * document's §5 states, in that order, and records in @p report everything it could not
         * carry across so a partial import is visibly partial.
         *
         * @param description The file's node graph, from `Geometry::import_gltf_scene`.
         * @param settings    The asset's `.meta` settings.
         * @param file_stem   The asset's file name without extension; names the synthetic root a
         *                    multi-root file needs, and nothing else.
         * @param report      Receives counts and warnings; overwritten.
         * @return The entities to create. Empty when the description holds no node.
         */
        ModelInstantiationPlan plan_model_instantiation(
            const Geometry::GLTFSceneDescription& description, const ModelImportSettings& settings,
            const std::string& file_stem, ModelImportReport& report);
    } // namespace Model
} // namespace SushiEngine
