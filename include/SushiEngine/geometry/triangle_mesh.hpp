/**************************************************************************/
/* triangle_mesh.hpp                                                      */
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
 * @file triangle_mesh.hpp
 * @brief An engine-neutral triangle mesh: positions and indices, nothing else.
 *
 * The renderer already has a triangle mesh, and the physics cannot use it. Not
 * because it is unsuitable but because of where it sits: `Render::Geometry::Mesh`
 * owns Vulkan buffers, so pointing the physics at it would invert the layering and
 * make an offline cooker require a device — an importer that needs a GPU is an
 * importer that fails on a build machine.
 *
 * So `SushiEngine::Geometry` sits *below* both. It knows about triangles and
 * nothing about who wants them: no device handles, no material, no vertex format
 * beyond position. The renderer keeps its 60-byte `MeshVertex` for drawing; anything
 * that wants to reason about the surface — bake a distance field, decompose a hull,
 * tetrahedralize — reads it through here.
 *
 * Two shapes, and the distinction matters. @ref TriangleMesh owns its data and is
 * what a cooker builds. @ref TriangleMeshView borrows someone else's, with a stride,
 * so the renderer can hand over an array of its own vertex struct without copying a
 * megabyte to have its distance field baked.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief A borrowed view of triangle geometry, with an arbitrary vertex stride.
         *
         * The stride is the whole point: a caller's vertices are almost never three
         * bare floats. The renderer's are 60 bytes with normals, tangents and two
         * UV sets; a cooked asset's may be 12. Both are read here without a copy.
         *
         * Non-owning, so it must not outlive what it points at. Nothing here
         * allocates or mutates, which is what makes it safe to pass down a call
         * chain that knows nothing about the owner.
         */
        struct TriangleMeshView
        {
            /** @brief First vertex's position; three contiguous floats at each stride. */
            const float* positions = nullptr;

            /** @brief Bytes between consecutive positions; 12 for tightly packed floats. */
            std::size_t position_stride = sizeof(float) * 3;

            /** @brief Number of vertices reachable through @ref positions. */
            std::size_t vertex_count = 0;

            /** @brief Triangle indices; @ref index_count is a multiple of three. */
            const std::uint32_t* indices = nullptr;

            /** @brief Number of indices. */
            std::size_t index_count = 0;

            /**
             * @brief The position of vertex @p vertex.
             *
             * @param vertex     The vertex to read.
             * @param position   Receives the three components.
             */
            void read_position(std::size_t vertex, float position[3]) const noexcept
            {
                const unsigned char* base = reinterpret_cast<const unsigned char*>(positions);
                const float* p =
                    reinterpret_cast<const float*>(base + vertex * position_stride);
                position[0] = p[0];
                position[1] = p[1];
                position[2] = p[2];
            }

            /** @brief Whether this view describes at least one triangle. */
            bool has_triangles() const noexcept
            {
                return positions != nullptr && indices != nullptr && vertex_count > 0 &&
                       index_count >= 3;
            }

            /** @brief Number of triangles, ignoring a trailing partial one. */
            std::size_t triangle_count() const noexcept { return index_count / 3; }
        };

        /**
         * @brief An owning triangle mesh: tightly packed positions and indices.
         *
         * What an importer or a cooking stage produces. Normals are optional because
         * most of what reads this mesh derives its own — a distance field takes the
         * sign from a triangle's geometric normal, not from an authored vertex one —
         * and carrying them when nobody wants them is a third of the memory for
         * nothing.
         */
        struct TriangleMesh
        {
            /** @brief Three floats per vertex, tightly packed. */
            std::vector<float> positions;

            /** @brief Triangle indices into @ref positions, three per triangle. */
            std::vector<std::uint32_t> indices;

            /** @brief Three floats per vertex, or empty when none were supplied. */
            std::vector<float> normals;

            /** @brief Number of vertices. */
            std::size_t vertex_count() const noexcept { return positions.size() / 3; }

            /** @brief Number of triangles. */
            std::size_t triangle_count() const noexcept { return indices.size() / 3; }

            /** @brief A borrowed view of this mesh, for anything that reads geometry. */
            TriangleMeshView view() const noexcept
            {
                TriangleMeshView v;
                v.positions = positions.empty() ? nullptr : positions.data();
                v.position_stride = sizeof(float) * 3;
                v.vertex_count = vertex_count();
                v.indices = indices.empty() ? nullptr : indices.data();
                v.index_count = indices.size();
                return v;
            }
        };

        /**
         * @brief The axis-aligned bounds of a mesh, in its own local space.
         *
         * Reported as a value rather than stored on the mesh: a cooking stage that
         * moves vertices would leave a stored bound stale, and a stale bound is worse
         * than an absent one because nothing announces it.
         *
         * @param mesh     The geometry to measure.
         * @param minimum  Receives the per-axis minimum; untouched when empty.
         * @param maximum  Receives the per-axis maximum; untouched when empty.
         * @return Whether the mesh had any vertices to measure.
         */
        inline bool compute_bounds(const TriangleMeshView& mesh, float minimum[3],
                                   float maximum[3]) noexcept
        {
            if (mesh.positions == nullptr || mesh.vertex_count == 0)
                return false;

            mesh.read_position(0, minimum);
            mesh.read_position(0, maximum);
            for (std::size_t i = 1; i < mesh.vertex_count; ++i)
            {
                float position[3];
                mesh.read_position(i, position);
                for (int axis = 0; axis < 3; ++axis)
                {
                    if (position[axis] < minimum[axis])
                        minimum[axis] = position[axis];
                    if (position[axis] > maximum[axis])
                        maximum[axis] = position[axis];
                }
            }
            return true;
        }
    } // namespace Geometry
} // namespace SushiEngine
