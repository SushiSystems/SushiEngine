/**************************************************************************/
/* cloth_mesh.hpp                                                         */
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
 * @file cloth_mesh.hpp
 * @brief Triangulates a row-major cloth particle grid into a shaded mesh.
 *
 * `Physics::ClothGrid` (and its render-side mirror, `Render::ClothStrandView`)
 * exposes only particle positions on a rows-by-cols grid. Drawing that grid as
 * a shaded, pickable surface (rather than a bare wireframe) needs a triangle
 * list with per-vertex normals; this header is the one place that turns a
 * position grid into that mesh, so both the Vulkan scene view and its tests
 * share the exact same triangulation.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /** @brief One triangulated cloth mesh vertex: its world position and shading normal. */
        struct ClothVertex
        {
            Vector3 position; /**< World-space position, copied from the source grid. */
            Vector3 normal;    /**< Shading normal, averaged from the adjacent triangles. */
        };

        /**
         * @brief Triangulates a row-major grid of world-space points into a shaded mesh.
         *
         * Splits every quad `(r, c)-(r, c+1)-(r+1, c)-(r+1, c+1)` into two triangles
         * along the `(r, c+1)-(r+1, c)` diagonal, accumulating each triangle's face
         * normal (the cross product of two of its edges) into its three vertices and
         * normalizing every vertex's accumulated normal once all triangles have been
         * summed. Degenerate input (`rows < 2` or `cols < 2`, nothing to triangulate)
         * clears both outputs and returns.
         *
         * @param vertices    Row-major world-space points, `rows * cols` long.
         * @param rows        Grid rows.
         * @param cols        Grid columns.
         * @param out_vertices Filled with one `ClothVertex` per input point, same order.
         * @param out_indices  Filled with the triangle list indexing @p out_vertices.
         */
        inline void triangulate_cloth_grid(const Vector3* vertices, std::uint32_t rows,
                                            std::uint32_t cols,
                                            std::vector<ClothVertex>& out_vertices,
                                            std::vector<std::uint32_t>& out_indices)
        {
            out_vertices.clear();
            out_indices.clear();

            if (rows < 2 || cols < 2)
                return;

            out_vertices.resize(static_cast<std::size_t>(rows) * cols);
            for (std::uint32_t i = 0; i < rows * cols; ++i)
                out_vertices[i].position = vertices[i];

            out_indices.reserve(static_cast<std::size_t>(rows - 1) * (cols - 1) * 6);

            const auto accumulate_face = [&](std::uint32_t a, std::uint32_t b, std::uint32_t c)
            {
                const Vector3 face_normal =
                    cross(out_vertices[b].position - out_vertices[a].position,
                          out_vertices[c].position - out_vertices[a].position);
                out_vertices[a].normal = out_vertices[a].normal + face_normal;
                out_vertices[b].normal = out_vertices[b].normal + face_normal;
                out_vertices[c].normal = out_vertices[c].normal + face_normal;
                out_indices.push_back(a);
                out_indices.push_back(b);
                out_indices.push_back(c);
            };

            for (std::uint32_t r = 0; r + 1 < rows; ++r)
            {
                for (std::uint32_t c = 0; c + 1 < cols; ++c)
                {
                    const std::uint32_t v00 = r * cols + c;
                    const std::uint32_t v01 = r * cols + c + 1;
                    const std::uint32_t v10 = (r + 1) * cols + c;
                    const std::uint32_t v11 = (r + 1) * cols + c + 1;

                    accumulate_face(v00, v10, v01);
                    accumulate_face(v01, v10, v11);
                }
            }

            for (ClothVertex& vertex : out_vertices)
                vertex.normal = normalize(vertex.normal);
        }
    } // namespace Render
} // namespace SushiEngine
