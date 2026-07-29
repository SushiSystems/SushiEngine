/**************************************************************************/
/* height_field_manifold.hpp                                              */
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
 * @file height_field_manifold.hpp
 * @brief Terrain: a grid of heights, collided by arithmetic instead of by search.
 *
 * A height field is a triangle mesh with a constraint — exactly one surface
 * above each point of a regular grid — and that constraint pays for itself
 * twice. It stores one scalar per vertex instead of three plus indices, and it
 * needs no hierarchy at all: the cells a body can touch are found by dividing
 * its bounds by the cell size (§7.2's "height field by direct cell lookup").
 * A hierarchy over terrain would be a search structure for a question whose
 * answer is a subtraction.
 *
 * Everything downstream is shared with the triangle mesh. Each cell yields two
 * triangles and they go through the same
 * `generate_convex_triangle_manifold`, including the internal-edge correction —
 * and a height field is the case that correction matters most for, because a
 * vehicle crossing terrain crosses a seam every metre.
 *
 * Which edges are interior is arithmetic here rather than cooked adjacency: a
 * cell's diagonal is always shared with its sibling triangle, and its outer
 * edges are shared unless the cell is on the field's rim. That is the whole
 * adjacency table, computed rather than stored.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/collision/mesh_manifold.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A borrowed view of a height grid, placed in the world.
         *
         * Heights are row-major: `heights[row * columns + column]` is the surface
         * at local `(column * cell_size_x, height, row * cell_size_z)`. The field's
         * `(0, 0)` corner sits at @ref center, so a terrain tile is placed by moving
         * its corner rather than by computing where its middle ended up.
         */
        template <typename T>
        struct HeightFieldView
        {
            const T* heights = nullptr;
            std::uint32_t columns = 0;
            std::uint32_t rows = 0;
            T cell_size_x = T(1);
            T cell_size_z = T(1);
            Vector3T<T> center{Vector3T<T>{T(0), T(0), T(0)}};
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
        };

        /** @brief The world position of grid vertex (@p column, @p row). */
        template <typename T>
        inline Vector3T<T> height_field_vertex(const HeightFieldView<T>& field,
                                               std::uint32_t column, std::uint32_t row) noexcept
        {
            const Vector3T<T> local{static_cast<T>(column) * field.cell_size_x,
                                    field.heights[row * field.columns + column],
                                    static_cast<T>(row) * field.cell_size_z};
            return field.center + rotate(field.orientation, local);
        }

        /**
         * @brief The two triangles of cell (@p column, @p row), and their shared edges.
         *
         * The cell's quad is split along the (0,0)-(1,1) diagonal, and both halves
         * are wound so their normals point up out of the surface. The diagonal is
         * always interior; an outer edge is interior unless it is on the field's rim.
         *
         * @param field    The field.
         * @param column   Cell column, in `[0, columns - 1)`.
         * @param row      Cell row, in `[0, rows - 1)`.
         * @param triangles Receives the two world-space triangles.
         * @param shared    Receives the three shared-edge flags for each triangle.
         */
        template <typename T>
        inline void height_field_cell(const HeightFieldView<T>& field, std::uint32_t column,
                                      std::uint32_t row, TriangleCollider<T> triangles[2],
                                      bool shared[2][3]) noexcept
        {
            const Vector3T<T> v00 = height_field_vertex(field, column, row);
            const Vector3T<T> v10 = height_field_vertex(field, column + 1u, row);
            const Vector3T<T> v01 = height_field_vertex(field, column, row + 1u);
            const Vector3T<T> v11 = height_field_vertex(field, column + 1u, row + 1u);

            // Wound so the face normal is up, matching the mesh convention.
            triangles[0] = TriangleCollider<T>{v00, v01, v10};
            triangles[1] = TriangleCollider<T>{v10, v01, v11};

            const bool has_left = column > 0;
            const bool has_right = column + 2u < field.columns;
            const bool has_back = row > 0;
            const bool has_front = row + 2u < field.rows;

            // Triangle 0's edges: (v00,v01) is the left rim, (v01,v10) is the shared
            // diagonal, (v10,v00) is the back rim.
            shared[0][0] = has_left;
            shared[0][1] = true;
            shared[0][2] = has_back;
            // Triangle 1's edges: (v10,v01) is the diagonal again, (v01,v11) is the
            // front rim, (v11,v10) is the right rim.
            shared[1][0] = true;
            shared[1][1] = has_front;
            shared[1][2] = has_right;
        }

        /**
         * @brief Every manifold between a convex shape and the terrain under it.
         *
         * The cell range is computed from the shape's bounds rather than searched
         * for. Bounds are transformed into field space the same way the mesh query
         * does it — conservatively, since a query that under-covers misses contacts
         * — and then the range is a division.
         *
         * @param emit Called as `emit(manifold, cell_index)`, where the cell index
         *             identifies the triangle within the field so warm starting can
         *             match it across ticks.
         */
        template <typename T, typename Shape, typename Emit>
        inline void generate_convex_height_field_manifolds(const Shape& shape,
                                                           const HeightFieldView<T>& field,
                                                           const Vector3T<T>& center,
                                                           const QuaternionT<T>& orientation,
                                                           T contact_offset, T face_tolerance,
                                                           Emit&& emit) noexcept
        {
            if (field.heights == nullptr || field.columns < 2 || field.rows < 2)
                return;
            if (field.cell_size_x <= T(0) || field.cell_size_z <= T(0))
                return;

            const Aabb<T> world = aabb_expand(world_bounds(shape), contact_offset);

            // Into field space, conservatively: the same reasoning as the mesh
            // hierarchy's query. One transform of the box, not a transform of the
            // terrain.
            const Vector3T<T> box_center = (world.min + world.max) * T(0.5);
            const Vector3T<T> box_extent = (world.max - world.min) * T(0.5);
            const Vector3T<T> local_center =
                rotate(conjugate(field.orientation), box_center - field.center);
            Vector3T<T> axes[3];
            axes[0] = rotate(conjugate(field.orientation), Vector3T<T>{T(1), T(0), T(0)});
            axes[1] = rotate(conjugate(field.orientation), Vector3T<T>{T(0), T(1), T(0)});
            axes[2] = rotate(conjugate(field.orientation), Vector3T<T>{T(0), T(0), T(1)});
            const T components[3] = {box_extent.x, box_extent.y, box_extent.z};
            Vector3T<T> local_extent{T(0), T(0), T(0)};
            for (int i = 0; i < 3; ++i)
            {
                local_extent.x += std::abs(axes[i].x) * components[i];
                local_extent.y += std::abs(axes[i].y) * components[i];
                local_extent.z += std::abs(axes[i].z) * components[i];
            }

            const T low_x = (local_center.x - local_extent.x) / field.cell_size_x;
            const T high_x = (local_center.x + local_extent.x) / field.cell_size_x;
            const T low_z = (local_center.z - local_extent.z) / field.cell_size_z;
            const T high_z = (local_center.z + local_extent.z) / field.cell_size_z;
            if (high_x < T(0) || high_z < T(0))
                return;

            const std::uint32_t last_column = field.columns - 2u;
            const std::uint32_t last_row = field.rows - 2u;
            const auto clamp_index = [](T value, std::uint32_t high) noexcept -> std::uint32_t
            {
                if (value < T(0))
                    return 0;
                const T floored = std::floor(value);
                if (floored >= static_cast<T>(high))
                    return high;
                return static_cast<std::uint32_t>(floored);
            };
            if (low_x > static_cast<T>(field.columns - 1u) ||
                low_z > static_cast<T>(field.rows - 1u))
                return;

            const std::uint32_t first_column = clamp_index(low_x, last_column);
            const std::uint32_t final_column = clamp_index(high_x, last_column);
            const std::uint32_t first_row = clamp_index(low_z, last_row);
            const std::uint32_t final_row = clamp_index(high_z, last_row);

            for (std::uint32_t row = first_row; row <= final_row; ++row)
                for (std::uint32_t column = first_column; column <= final_column; ++column)
                {
                    TriangleCollider<T> triangles[2];
                    bool shared[2][3];
                    height_field_cell(field, column, row, triangles, shared);
                    const std::uint32_t cell = row * (field.columns - 1u) + column;
                    for (std::uint32_t half = 0; half < 2; ++half)
                    {
                        const ContactManifold<T> manifold = generate_convex_triangle_manifold<T>(
                            shape, triangles[half], shared[half], center, orientation,
                            field.center, field.orientation, cell * 2u + half, contact_offset,
                            face_tolerance);
                        if (manifold.point_count > 0)
                            emit(manifold, cell * 2u + half);
                    }
                }
        }
    } // namespace Physics
} // namespace SushiEngine
