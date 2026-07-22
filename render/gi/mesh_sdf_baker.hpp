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
 * @brief A standalone CPU signed-distance-field baker for triangle meshes.
 *
 * Pure geometry: given a mesh's positions and triangle indices, it produces a
 * cube brick of signed distances sampled at voxel centres in the mesh's own local
 * space. The unsigned distance at each voxel is the minimum point-to-triangle
 * distance over every triangle; the sign is taken from the nearest triangle's
 * geometric normal, so points behind the surface read negative. The brick's AABB
 * is padded by two voxels so the zero isosurface has clearance and rays entering
 * from outside always read positive distances first. No Vulkan, no engine coupling
 * beyond @ref Geometry::MeshVertex.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

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
             * @brief A cube of signed distances baked from a triangle mesh.
             *
             * @c distances holds @c resolution^3 signed distances in local units,
             * indexed as @c index = x + resolution * (y + resolution * z) with
             * @c x the fastest-varying axis. Voxel @c (x,y,z)'s centre in local space
             * is, per axis, @c aabb_min + (voxel + 0.5) * (aabb_max - aabb_min) / resolution.
             * Negative distances are inside the surface, positive outside. @c distances
             * is empty for a degenerate mesh, in which case @c resolution is zero.
             */
            struct MeshSdfBrick
            {
                float aabb_min[3];              // padded local-space AABB minimum
                float aabb_max[3];              // padded local-space AABB maximum
                std::int32_t resolution = 0;    // voxels per axis (cube)
                std::vector<float> distances;   // resolution^3 signed distances, local units
            };

            /**
             * @brief Bakes a signed distance field brick for a triangle mesh.
             *
             * <mechanism: for each voxel centre, the minimum unsigned distance to any triangle;
             * the sign comes from the geometric normal of the nearest triangle (negative when the
             * point is behind it, i.e. inside). The AABB is padded by two voxels so the zero
             * isosurface has clearance and rays approaching from outside read positive distances.>
             *
             * @param vertices     The mesh vertices (only position is read).
             * @param vertex_count Number of vertices.
             * @param indices      Triangle indices, index_count a multiple of three.
             * @param index_count  Number of indices.
             * @param resolution   Voxels per axis of the cube brick (e.g. 32).
             * @return The baked brick; distances empty if the mesh was degenerate.
             */
            MeshSdfBrick bake_mesh_sdf(const Geometry::MeshVertex* vertices, std::size_t vertex_count,
                                       const std::uint32_t* indices, std::size_t index_count,
                                       std::int32_t resolution);
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
