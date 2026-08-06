/**************************************************************************/
/* scene_import.hpp                                                       */
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
 * @file scene_import.hpp
 * @brief A glTF file's node graph, exactly as the file states it.
 *
 * The counterpart to `mesh_import.hpp`, which answers "what triangles are in this file" by
 * merging everything into one mesh. This answers "what is this file's structure" and merges
 * nothing: one entry per node, keeping the name, the parent, the local transform and what the
 * node carries. Applying settings, deciding which node becomes what, and talking to a renderer
 * are all somebody else's job — this reports the file and stops.
 *
 * Nodes come back parent-before-child so a consumer builds a tree in one pass and never holds a
 * forward reference.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief 3-vector in import precision.
         *
         * Declared here rather than assumed global: `Vector3f` is a per-namespace alias of
         * `Vector3T<float>` in this codebase, following `SushiEngine/animation/skeleton.hpp`.
         */
        using Vector3f = Vector3T<float>;

        /** @brief Unit quaternion in import precision; see @ref Vector3f. */
        using Quaternionf = QuaternionT<float>;

        /** @brief What a `KHR_lights_punctual` light is. */
        enum class GLTFLightKind
        {
            Directional,
            Point,
            Spot
        };

        /** @brief Which projection a glTF camera uses. */
        enum class GLTFCameraKind
        {
            Perspective,
            Orthographic
        };

        /** @brief One `KHR_lights_punctual` light, in the file's own units. */
        struct GLTFLightDescription
        {
            std::string name;
            GLTFLightKind kind = GLTFLightKind::Point;
            float color[3] = {1.0f, 1.0f, 1.0f};
            float intensity = 1.0f;

            /** @brief Metres the light reaches, or zero for the glTF default of unlimited. */
            float range = 0.0f;

            float spot_inner_cone_radians = 0.0f;
            float spot_outer_cone_radians = 0.7853982f;
        };

        /** @brief One glTF camera, in the file's own units. */
        struct GLTFCameraDescription
        {
            std::string name;
            GLTFCameraKind kind = GLTFCameraKind::Perspective;

            /** @brief Vertical field of view in radians; perspective only. */
            float vertical_field_of_view_radians = 0.7853982f;

            /** @brief Horizontal magnification; orthographic only. */
            float orthographic_width = 1.0f;

            /** @brief Vertical magnification; orthographic only. */
            float orthographic_height = 1.0f;

            float near_plane = 0.1f;

            /** @brief Far plane, or zero when the file declares an infinite one. */
            float far_plane = 0.0f;
        };

        /** @brief One glTF node: where it sits, and what it carries. */
        struct GLTFNodeDescription
        {
            /** @brief The node's own name, or empty when the file does not name it. */
            std::string name;

            /** @brief Index into @ref GLTFSceneDescription::nodes, or -1 when this is a root. */
            std::int32_t parent = -1;

            /**
             * @brief The file's own node index.
             *
             * The join key a consumer matches against a renderer's imported primitives. A file
             * defines it, so two independent readers of the same file derive the same value;
             * traversal order does not have that property.
             */
            std::uint32_t source_index = 0;

            Vector3f translation{0.0f, 0.0f, 0.0f};
            Quaternionf rotation{0.0f, 0.0f, 0.0f, 1.0f};
            Vector3f scale{1.0f, 1.0f, 1.0f};

            /** @brief Index into the file's meshes, or -1 when the node carries none. */
            std::int32_t mesh = -1;

            /** @brief Primitives that mesh holds; zero when @ref mesh is -1. */
            std::uint32_t primitive_count = 0;

            /** @brief Index into @ref GLTFSceneDescription::cameras, or -1. */
            std::int32_t camera = -1;

            /** @brief Index into @ref GLTFSceneDescription::lights, or -1. */
            std::int32_t light = -1;

            /** @brief Index into the file's skins, or -1. */
            std::int32_t skin = -1;
        };

        /** @brief A glTF file's structure. */
        struct GLTFSceneDescription
        {
            /** @brief Every node, parents always before their children. */
            std::vector<GLTFNodeDescription> nodes;

            std::vector<GLTFLightDescription> lights;
            std::vector<GLTFCameraDescription> cameras;
            std::uint32_t material_count = 0;
            std::uint32_t skin_count = 0;
        };

        /**
         * @brief Reads a glTF file's node graph without reading its geometry.
         *
         * Parses the file and walks its default scene, decomposing each node's transform into
         * translation, rotation and scale rather than a matrix, because that is the form an
         * entity transform takes and converting to a matrix here only to decompose it again
         * downstream would lose precision for nothing. Buffers are not loaded: nothing here
         * reads vertex data.
         *
         * @param path Path to a `.gltf` or `.glb` file.
         * @param out  Receives the description; cleared first, and left cleared on failure.
         * @return False when the file cannot be parsed, or declares no node.
         */
        bool import_gltf_scene(const char* path, GLTFSceneDescription& out);
    } // namespace Geometry
} // namespace SushiEngine
