/**************************************************************************/
/* deformable_mesh.hpp                                                    */
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
 * @file deformable_mesh.hpp
 * @brief The renderer's view of host-simulated, per-frame-changing geometry.
 *
 * This replaces the rows-by-cols cloth grid the renderer used to accept. A cloth
 * sheet is one shape a soft body can have; a tetrahedral body's surface is a
 * closed triangle mesh with no grid structure at all, and a *fractured* body's
 * surface is not even connected. So the seam carries what all of them actually
 * share — a vertex array and a triangle list — and nothing about how the vertices
 * were arranged.
 *
 * Losing the grid costs one thing. Grid topology is implicit: the six triangles
 * touching vertex `(r, c)` are computable from `r`, `c`, and the row stride, so
 * the GPU could gather a vertex's face normals with no lookup table and no
 * atomics. An arbitrary triangle list has no such formula, so the same
 * atomic-free gather needs the inverse map — vertex to the triangles that use it
 * — built explicitly. @ref build_vertex_triangle_adjacency builds it, and because
 * it depends only on the index list it is rebuilt only when the topology changes,
 * not per frame.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief One host-simulated surface's world-space geometry for this frame.
         *
         * A non-owning view. @ref vertices and @ref indices must outlive the
         * `render()` call that receives it, and nothing here is retained past it.
         *
         * @ref topology_revision is the one field a producer has to think about.
         * The renderer caches the vertex-to-triangle map derived from @ref indices,
         * keyed on this counter together with the two counts; a producer that
         * changes the triangle list must bump it. In practice the counts alone
         * already catch every case this engine produces — fracture splits vertices
         * (so @ref vertex_count grows) and drops elements (so @ref index_count
         * shrinks) — and the counter exists for the case where some future producer
         * rewrites a triangle list in place at unchanged sizes.
         */
        struct DeformableMeshView
        {
            const Vector3* vertices = nullptr;    /**< World-space positions, @ref vertex_count long. */
            std::uint32_t vertex_count = 0;       /**< Number of entries in @ref vertices. */
            const std::uint32_t* indices = nullptr; /**< Triangle list into @ref vertices. */
            std::uint32_t index_count = 0;        /**< Number of entries in @ref indices; a multiple of three. */
            std::uint64_t topology_revision = 0;  /**< Bumped whenever @ref indices changes at unchanged counts. */
            Vector3 color{Vector3{0.85, 0.85, 0.9}}; /**< Base colour. */
            std::uint32_t id = 0;                 /**< Picking id written to the id target (0 = none). */
        };

        /**
         * @brief The triangles touching each vertex, as a flat range table.
         *
         * Stored as one `(first, count)` pair per vertex indexing @ref triangle,
         * rather than the usual `n + 1` prefix-sum array, because the consumer is a
         * compute shader: a pair read is one load of a `uvec2` at a known offset,
         * where a prefix sum would need two loads and a per-mesh "+1 slot" rule at
         * every point where several meshes are concatenated into one buffer.
         */
        struct VertexTriangleAdjacency
        {
            std::vector<std::uint32_t> range;    /**< Two entries per vertex: first, count. */
            std::vector<std::uint32_t> triangle; /**< Triangle indices, grouped by vertex. */
        };

        /**
         * @brief Inverts a triangle list into the triangles touching each vertex.
         *
         * Two counting passes and a scatter, so it is linear in @p index_count and
         * allocates exactly what it stores. Indices at or past @p vertex_count are
         * skipped rather than trusted — a malformed list must not be able to write
         * outside the table.
         *
         * A vertex used by no triangle gets a count of zero, which the consumer must
         * read as "no normal to gather" rather than as an error: a fractured body
         * keeps its interior vertices in the particle array long after they have
         * stopped appearing on the surface.
         *
         * @param indices      Triangle list; entries past a multiple of three are ignored.
         * @param index_count  Number of entries in @p indices.
         * @param vertex_count Number of vertices the list may address.
         * @param out          Filled with the adjacency; cleared first.
         */
        inline void build_vertex_triangle_adjacency(const std::uint32_t* indices,
                                                    std::uint32_t index_count,
                                                    std::uint32_t vertex_count,
                                                    VertexTriangleAdjacency& out)
        {
            out.range.assign(static_cast<std::size_t>(vertex_count) * 2, 0);
            out.triangle.clear();
            if (indices == nullptr || vertex_count == 0)
                return;

            const std::uint32_t triangle_count = index_count / 3;

            // Pass one: how many triangles each vertex is used by, counted into the
            // slot the range pair's `count` will end up in.
            for (std::uint32_t t = 0; t < triangle_count; ++t)
                for (std::uint32_t k = 0; k < 3; ++k)
                {
                    const std::uint32_t v = indices[t * 3 + k];
                    if (v < vertex_count)
                        ++out.range[static_cast<std::size_t>(v) * 2 + 1];
                }

            // Pass two: turn the counts into offsets, keeping the counts.
            std::uint32_t running = 0;
            for (std::uint32_t v = 0; v < vertex_count; ++v)
            {
                out.range[static_cast<std::size_t>(v) * 2] = running;
                running += out.range[static_cast<std::size_t>(v) * 2 + 1];
            }
            out.triangle.resize(running);

            // The scatter, using a per-vertex cursor that starts at the offset and is
            // walked forward; when it is done it has advanced by exactly the count, so
            // no separate fill pointer array is needed.
            std::vector<std::uint32_t> cursor(vertex_count);
            for (std::uint32_t v = 0; v < vertex_count; ++v)
                cursor[v] = out.range[static_cast<std::size_t>(v) * 2];
            for (std::uint32_t t = 0; t < triangle_count; ++t)
                for (std::uint32_t k = 0; k < 3; ++k)
                {
                    const std::uint32_t v = indices[t * 3 + k];
                    if (v < vertex_count)
                        out.triangle[cursor[v]++] = t;
                }
        }

        /**
         * @brief Emits the triangle list for a row-major rows-by-cols grid.
         *
         * The topology half of what `triangulate_cloth_grid` used to do, split out so
         * a grid producer keeps its familiar shape while the renderer downstream sees
         * nothing but a triangle list. The diagonal and winding are unchanged from the
         * grid path they replace — `(r, c)-(r+1, c)-(r, c+1)` and
         * `(r, c+1)-(r+1, c)-(r+1, c+1)` — so a sheet built this way shades exactly as
         * it did before.
         *
         * @param rows Grid rows; fewer than two emits nothing.
         * @param cols Grid columns; fewer than two emits nothing.
         * @param out  Filled with the triangle list; cleared first.
         */
        inline void build_grid_indices(std::uint32_t rows, std::uint32_t cols,
                                       std::vector<std::uint32_t>& out)
        {
            out.clear();
            if (rows < 2 || cols < 2)
                return;

            out.reserve(static_cast<std::size_t>(rows - 1) * (cols - 1) * 6);
            for (std::uint32_t r = 0; r + 1 < rows; ++r)
            {
                for (std::uint32_t c = 0; c + 1 < cols; ++c)
                {
                    const std::uint32_t v00 = r * cols + c;
                    const std::uint32_t v01 = v00 + 1;
                    const std::uint32_t v10 = v00 + cols;
                    const std::uint32_t v11 = v10 + 1;

                    out.push_back(v00);
                    out.push_back(v10);
                    out.push_back(v01);
                    out.push_back(v01);
                    out.push_back(v10);
                    out.push_back(v11);
                }
            }
        }

        /** @brief One shaded deformable-mesh vertex: its world position and shading normal. */
        struct DeformableVertex
        {
            Vector3 position; /**< World-space position, copied from the source view. */
            Vector3 normal;   /**< Shading normal, area-weighted from the adjacent triangles. */
        };

        /**
         * @brief Shades a deformable mesh on the host — the reference the GPU pass matches.
         *
         * Production draws go through the compute path (`deformable.comp`), which does
         * this same arithmetic per vertex on the GPU. This exists so the arithmetic can
         * be stated once in a place a test can call without a device, and so the two can
         * be compared: the GPU gathers per vertex where this scatters per triangle, and
         * an area-weighted sum is order-independent only up to floating-point rounding,
         * so "the same" here means to within that.
         *
         * A vertex no triangle uses keeps a zero normal rather than a made-up one.
         *
         * @param view         The mesh to shade.
         * @param out_vertices Filled with one vertex per input position, same order.
         */
        inline void shade_deformable_mesh(const DeformableMeshView& view,
                                          std::vector<DeformableVertex>& out_vertices)
        {
            out_vertices.clear();
            if (view.vertices == nullptr || view.vertex_count == 0)
                return;

            out_vertices.resize(view.vertex_count);
            for (std::uint32_t i = 0; i < view.vertex_count; ++i)
                out_vertices[i].position = view.vertices[i];

            if (view.indices == nullptr)
                return;

            const std::uint32_t triangle_count = view.index_count / 3;
            for (std::uint32_t t = 0; t < triangle_count; ++t)
            {
                const std::uint32_t a = view.indices[t * 3 + 0];
                const std::uint32_t b = view.indices[t * 3 + 1];
                const std::uint32_t c = view.indices[t * 3 + 2];
                if (a >= view.vertex_count || b >= view.vertex_count || c >= view.vertex_count)
                    continue;

                // Unnormalized on purpose: its length is twice the triangle's area, which
                // is the weight a shared vertex's normal should carry.
                const Vector3 face_normal = cross(view.vertices[b] - view.vertices[a],
                                                  view.vertices[c] - view.vertices[a]);
                out_vertices[a].normal = out_vertices[a].normal + face_normal;
                out_vertices[b].normal = out_vertices[b].normal + face_normal;
                out_vertices[c].normal = out_vertices[c].normal + face_normal;
            }

            for (DeformableVertex& vertex : out_vertices)
            {
                const Scalar len = length(vertex.normal);
                if (len > Scalar(1e-12))
                    vertex.normal = vertex.normal * (Scalar(1) / len);
            }
        }
    } // namespace Render
} // namespace SushiEngine
