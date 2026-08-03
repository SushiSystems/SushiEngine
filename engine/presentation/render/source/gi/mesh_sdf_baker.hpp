/**************************************************************************/
/* mesh_sdf_baker.hpp                                                     */
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
 * @file mesh_sdf_baker.hpp
 * @brief The renderer's view of the shared signed-distance baker.
 *
 * The bake itself moved down to `SushiEngine::Geometry`, where it belongs: a mesh's
 * distance field is what the renderer cone-traces through, what a collision
 * narrowphase queries, and what a soft-body cooker uses to decide what is inside a
 * shape. Leaving it in `render/gi/` made it a global-illumination private that the
 * physics could only reach by depending upward.
 *
 * What stays here is the one thing that is genuinely the renderer's: knowing that
 * *its* vertices are 60-byte @ref Geometry::MeshVertex records whose first three
 * floats are the position. That is expressed as a stride, so nothing is copied — the
 * shared baker walks the renderer's own array in place.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/geometry/signed_distance_field.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            struct MeshVertex;
        }

        namespace Gi
        {
            /**
             * @brief The shared brick type, under the name the renderer already uses.
             *
             * An alias rather than a rename at every call site: the renderer's GI
             * code has nothing to gain from the churn, and the type it was using was
             * always this one.
             */
            using MeshSdfBrick = SushiEngine::Geometry::SignedDistanceFieldBrick;

            /**
             * @brief Bakes a signed distance field brick for a renderer mesh.
             *
             * A thin adapter over @ref SushiEngine::Geometry::bake_signed_distance_field:
             * it describes the renderer's vertex array as a strided position view and
             * hands it over. No copy, and no geometry knowledge of its own.
             *
             * @param vertices     The mesh vertices (only position is read).
             * @param vertex_count Number of vertices.
             * @param indices      Triangle indices, index_count a multiple of three.
             * @param index_count  Number of indices.
             * @param resolution   Voxels per axis of the cube brick (e.g. 32).
             * @return The baked brick; distances empty if the mesh was degenerate.
             */
            MeshSdfBrick bake_mesh_sdf(const Geometry::MeshVertex* vertices,
                                       std::size_t vertex_count,
                                       const std::uint32_t* indices,
                                       std::size_t index_count, std::int32_t resolution);
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
