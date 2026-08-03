/**************************************************************************/
/* cooking_parameters.cpp                                                 */
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

#include <SushiEngine/physics/cooking/cooking_parameters.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            std::uint64_t mesh_content_hash(const Geometry::TriangleMeshView& mesh) noexcept
            {
                std::uint64_t hash = 1469598103934665603ull;
                if (mesh.positions == nullptr)
                    return hash;

                // Positions are read through the view's stride and folded as their bit
                // patterns, so the hash is over the geometry and not over the memory
                // layout the caller happened to hand over. The same mesh presented tightly
                // packed and inside a sixty-byte render vertex must hash identically, or
                // the cache misses every time the import path changes shape.
                const std::uint32_t vertex_count = std::uint32_t(mesh.vertex_count);
                hash = hash_bytes(hash, vertex_count);
                for (std::size_t i = 0; i < mesh.vertex_count; ++i)
                {
                    float position[3];
                    mesh.read_position(i, position);
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        // A negative zero and a positive zero are the same vertex; folding
                        // their differing bit patterns would make one mesh hash two ways
                        // depending on which exporter wrote it.
                        const float value = position[axis] == 0.0f ? 0.0f : position[axis];
                        hash = hash_bytes(hash, value);
                    }
                }

                const std::uint32_t index_count = std::uint32_t(mesh.index_count);
                hash = hash_bytes(hash, index_count);
                if (mesh.indices != nullptr)
                {
                    for (std::size_t i = 0; i < mesh.index_count; ++i)
                        hash = hash_bytes(hash, mesh.indices[i]);
                }
                return hash;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
