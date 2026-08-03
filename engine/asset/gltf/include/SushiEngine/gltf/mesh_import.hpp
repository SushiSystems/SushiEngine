/**************************************************************************/
/* mesh_import.hpp                                                        */
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
 * @file mesh_import.hpp
 * @brief A glTF file's triangles, with no renderer between the file and them.
 *
 * §8.1's `MeshImporter` box, and the piece that was missing for the cooking pipeline to
 * start at a *file* rather than at a mesh somebody already had. The renderer imports glTF
 * too, but it imports into its own sixty-byte vertex and its own Vulkan buffers, so
 * reaching a mesh through it means bringing up a graphics stack — which is exactly the
 * dependency §3.4 exists to avoid and exactly the one an importer on a build machine
 * cannot satisfy.
 *
 * Positions and indices only, because that is all any cooking stage reads. Normals,
 * tangents, UV sets and materials are the renderer's business and are skipped rather than
 * carried, which keeps this importer's output the same size as its input's geometry.
 *
 * The declaration lives here, in the neutral geometry surface; the implementation lives in
 * `import/`, the one module that links cgltf — the same split
 * `Animation::import_gltf_skeleton` already uses.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /** @brief What an import found, and what it could not use. */
        struct GltfMeshImportReport
        {
            /** @brief Triangle primitives merged into the output. */
            std::uint32_t primitives_imported = 0;

            /**
             * @brief Primitives skipped for not being indexed triangles.
             *
             * Points, lines, strips, fans and un-indexed primitives. Counted rather than
             * silently ignored: a model that imports as half its geometry is a model whose
             * collider is half a shape, and the artist needs to be told which half.
             */
            std::uint32_t primitives_skipped = 0;

            /** @brief glTF meshes the file contained, whether or not they were usable. */
            std::uint32_t meshes_in_file = 0;

            /** @brief The first mesh's name, or empty; what an asset gets named after. */
            std::string first_mesh_name;
        };

        /**
         * @brief Imports every triangle in a glTF file as one mesh, in the file's own space.
         *
         * All primitives of all meshes are merged, because a cooked collider or soft body is
         * a property of the *model* rather than of the arbitrary way an exporter split it
         * into primitives at material boundaries. A caller that wants them separately has
         * @ref GltfMeshImportReport to see how many there were and can import per node once
         * something needs that.
         *
         * **Node transforms are applied**, so a model assembled from instanced nodes arrives
         * in the shape it is drawn in. Skipping them is the classic way a cooked collider
         * comes out as a pile of overlapping parts at the origin.
         *
         * @param path   Path to a `.gltf` or `.glb` file.
         * @param out    Receives the merged mesh; cleared first, empty on failure.
         * @param report Receives what was found and skipped; may be null.
         * @return False when the file cannot be read or held no usable triangle.
         */
        bool import_gltf_mesh(const char* path, TriangleMesh& out,
                              GltfMeshImportReport* report = nullptr);
    } // namespace Geometry
} // namespace SushiEngine
