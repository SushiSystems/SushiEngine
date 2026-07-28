/**************************************************************************/
/* mesh_sdf_baker.cpp                                                     */
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

#include "gi/mesh_sdf_baker.hpp"

#include "geometry/mesh_registry.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            MeshSdfBrick bake_mesh_sdf(const Geometry::MeshVertex* vertices,
                                       std::size_t vertex_count,
                                       const std::uint32_t* indices,
                                       std::size_t index_count, std::int32_t resolution)
            {
                // The only renderer-specific fact in the whole bake: where a position
                // sits inside a MeshVertex, and how far apart two of them are. The
                // vertex struct is included here rather than in the header so the
                // header stays free of the registry.
                SushiEngine::Geometry::TriangleMeshView mesh;
                mesh.positions = vertices != nullptr ? vertices[0].position : nullptr;
                mesh.position_stride = sizeof(Geometry::MeshVertex);
                mesh.vertex_count = vertex_count;
                mesh.indices = indices;
                mesh.index_count = index_count;

                return SushiEngine::Geometry::bake_signed_distance_field(mesh, resolution);
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
