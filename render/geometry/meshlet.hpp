/**************************************************************************/
/* meshlet.hpp                                                            */
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
 * @file meshlet.hpp
 * @brief Splits a mesh into meshlets for the mesh-shader draw path (Phase 10 item 4).
 *
 * A meshlet is a small cluster of triangles (a bounded number of unique vertices and
 * triangles) the mesh-shader path draws as one workgroup, after a task shader culls it by
 * frustum, hierarchical-Z, and — where the geometry is single-sided — its normal cone. The
 * clustering is a greedy sweep over the index buffer; it is device-independent CPU work, so
 * every mesh is meshletised at import whether or not the device can draw with mesh shaders
 * (the classic path simply ignores the extra buffers).
 *
 * The layouts here mirror the `std430` blocks the task and mesh shaders read; keep the two
 * in lockstep. Triangle indices are meshlet-local (0..vertex_count-1) and packed three to a
 * `uint`, eight bits each, which is why a meshlet holds at most 255 vertices.
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

            /** @brief The most vertices and triangles one meshlet holds. */
            constexpr std::uint32_t MESHLET_MAX_VERTICES = 64;
            constexpr std::uint32_t MESHLET_MAX_TRIANGLES = 124;

            /**
             * @brief One meshlet: its slices of the vertex and triangle arrays, plus bounds.
             *
             * 48 bytes, `std430`. @c bounding_sphere is in the mesh's local space (xyz centre,
             * w radius); the task shader transforms it by the instance model. @c cone is the
             * normalised average normal (xyz) and a cull cutoff (w): the meshlet is
             * back-facing when `dot(normalize(centre - camera), cone.xyz) > cone.w`; a w above
             * one marks a cone too wide to cull against (kept always-visible).
             */
            struct MeshletDescriptor
            {
                std::uint32_t vertex_offset;   /**< Into the meshlet-vertex array. */
                std::uint32_t triangle_offset; /**< Into the meshlet-triangle array (triangles). */
                std::uint32_t vertex_count;
                std::uint32_t triangle_count;
                float bounding_sphere[4]; /**< xyz = local centre, w = radius. */
                float cone[4];            /**< xyz = axis, w = cutoff (>1 = never cull). */
            };

            static_assert(sizeof(MeshletDescriptor) == 48,
                          "MeshletDescriptor must match the std430 block in the mesh shaders");

            /**
             * @brief The result of meshletising a mesh: three parallel arrays.
             *
             * @c descriptors is one entry per meshlet; @c vertices holds the global vertex
             * indices each meshlet references, sliced by @c MeshletDescriptor::vertex_offset;
             * @c triangles holds the packed local triangle indices, sliced by
             * @c triangle_offset. All three upload to device storage buffers the mesh shaders
             * read.
             */
            struct MeshletData
            {
                std::vector<MeshletDescriptor> descriptors;
                std::vector<std::uint32_t> vertices;  /**< Global vertex indices. */
                std::vector<std::uint32_t> triangles; /**< Packed local indices, 3x8 bits. */

                bool empty() const noexcept { return descriptors.empty(); }
            };

            /**
             * @brief Clusters a mesh's triangles into meshlets with bounds and normal cones.
             *
             * A greedy sweep: each triangle joins the current meshlet if it still fits the
             * vertex and triangle caps, otherwise it starts a new one. Every meshlet then gets
             * a bounding sphere (for frustum and hierarchical-Z culling) and a normal cone (for
             * back-face cluster culling on single-sided geometry).
     *
             * @param vertices     The mesh's vertices (only positions and normals are read).
             * @param vertex_count Number of entries in @p vertices.
             * @param indices      Triangle indices into @p vertices.
             * @param index_count  Number of entries in @p indices.
             * @return The meshlet data, empty if the mesh is degenerate.
             */
            MeshletData build_meshlets(const MeshVertex* vertices, std::size_t vertex_count,
                                       const std::uint32_t* indices, std::size_t index_count);
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
