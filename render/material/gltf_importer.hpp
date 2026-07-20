/**************************************************************************/
/* gltf_importer.hpp                                                      */
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
 * @file gltf_importer.hpp
 * @brief glTF 2.0 mesh and material import.
 *
 * glTF's core material *is* the engine's core material — metallic-roughness, normal,
 * occlusion, emissive — so the conversion is close to a copy, and the
 * `KHR_materials_*` extensions map onto the advanced lobes one for one. Each
 * primitive becomes one mesh, baked into its node's world transform so a multi-part
 * asset assembles correctly without a scene graph on the render side. Missing
 * tangents are generated and missing UVs default to zero.
 */

#include <cstddef>

#include <SushiEngine/render/material.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Assets
        {
            class TextureLibrary;

            /**
             * @brief Imports every primitive in a glTF file.
             *
             * @param path      Path to a .gltf or .glb file.
             * @param meshes    Registry the geometry is uploaded into.
             * @param textures  Library the referenced images are loaded into.
             * @param out_meshes    Receives one mesh id per imported primitive.
             * @param out_materials Receives the material for each entry in @p out_meshes.
             * @param capacity      Capacity of @p out_meshes and @p out_materials.
             * @return Number of primitives written, or 0 if the file could not be read.
             */
            std::size_t import_gltf(const char* path, Geometry::MeshRegistry& meshes,
                                    TextureLibrary& textures, MeshId* out_meshes,
                                    Render::Material* out_materials, std::size_t capacity);
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
