/**************************************************************************/
/* imported_mesh.hpp                                                      */
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
 * @file imported_mesh.hpp
 * @brief Finding the one primitive of a glTF file that an entity draws.
 *
 * Header-only on purpose. Both consumers need it and only one of them links this module:
 * `serialization` is handed an `IAssetLibrary&` and adds this include root privately, because
 * a configure with `SUSHIENGINE_BUILD_RENDER` off has no library to link. A free function in
 * the render library would be unreachable from there; an inline one is not.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include "asset_library_interface.hpp"

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief The mesh a glTF file's (@p source_node, @p primitive) pair imports to.
         *
         * Joined on the file's own indices rather than on position in the returned array,
         * which is what @ref IAssetLibrary::load_gltf_scene exists to make possible: two
         * parsers agreeing on a walk order today is not a property either of them promises.
         *
         * Grows the request until the library stops filling it exactly. `load_gltf_scene`
         * treats an exact fill as possibly truncated and declines to cache it, so a caller
         * that guessed too small would both miss entries and re-parse the file every time.
         *
         * @param assets      The library to import through.
         * @param path        Path to a `.gltf` or `.glb`; an empty one resolves to nothing.
         * @param source_node The file's node index.
         * @param primitive   Which primitive of that node's mesh.
         * @param material    When non-null, receives the imported material for that primitive.
         * @return The mesh id, or @ref INVALID_MESH when the file or the pair is not there.
         */
        inline MeshId resolve_imported_mesh(IAssetLibrary& assets, const char* path,
                                            std::uint32_t source_node, std::uint32_t primitive,
                                            Material* material = nullptr)
        {
            if (path == nullptr || *path == '\0')
                return INVALID_MESH;

            std::vector<ImportedPrimitive> imported;
            std::size_t capacity = 16;
            std::size_t count = 0;
            // A file large enough to need more than this many primitives is one where the
            // doubling has already paid for itself; the cap only stops a library that
            // reported an exact fill forever from looping without end.
            constexpr std::size_t MAXIMUM_CAPACITY = 1u << 20;
            for (;;)
            {
                imported.assign(capacity, ImportedPrimitive{});
                count = assets.load_gltf_scene(path, imported.data(), capacity);
                if (count < capacity || capacity >= MAXIMUM_CAPACITY)
                    break;
                capacity *= 2;
            }

            for (std::size_t i = 0; i < count; ++i)
            {
                if (imported[i].source_node != source_node || imported[i].primitive != primitive)
                    continue;
                if (material != nullptr)
                    *material = imported[i].material;
                return imported[i].mesh;
            }
            return INVALID_MESH;
        }
    } // namespace Render
} // namespace SushiEngine
