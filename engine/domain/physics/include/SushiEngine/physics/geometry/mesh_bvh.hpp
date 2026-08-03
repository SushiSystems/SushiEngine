/**************************************************************************/
/* mesh_bvh.hpp                                                           */
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
 * @file mesh_bvh.hpp
 * @brief The cooked hierarchy that makes a static triangle mesh collidable.
 *
 * A triangle is a convex set, so it already collides against every shape in the
 * engine through the one general routine (`geometry/gjk.hpp`). What a mesh adds
 * is not geometry but *scale*: a terrain has a hundred thousand triangles and a
 * crate touches four of them, so the whole problem is finding which four
 * without looking at the rest. That is a bounding-volume hierarchy, and it is
 * cooked once rather than built at run time (§0.3).
 *
 * Two halves live here, and the split matters:
 *
 * - **Building** is host-only and allocates freely. It happens offline, in the
 *   importer, and its output is a flat blob of nodes and index arrays.
 * - **Traversal** is a fixed-capacity stack walk over borrowed arrays, with no
 *   allocation and no recursion, so it is device-copyable as it stands.
 *
 * The build also cooks **edge adjacency**, and that is not an optimization. A
 * shape sliding across a tessellated floor catches on the interior edges between
 * triangles — the classic ghost-collision bug — because a contact resolved
 * against a triangle's *edge* gets a normal pointing along the edge's Voronoi
 * region rather than out of the surface. Fixing it needs to know that the edge
 * is shared and what is on the other side, which is information the triangle
 * soup does not carry and the cooker is the only place that can afford to
 * compute (`collision/mesh_manifold.hpp` is where it is used).
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <utility>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief Marks a triangle edge with no neighbour: an open boundary. */
        inline constexpr std::uint32_t no_adjacent_triangle = 0xFFFFFFFFu;

        /** @brief The deepest a hierarchy walk may go; a balanced tree of 2^40 leaves. */
        inline constexpr std::size_t max_bvh_stack_depth = 64;

        /**
         * @brief One node of the cooked hierarchy.
         *
         * A leaf names a run of triangles in the cooked order; an interior node
         * names its left child, with the right child immediately after it. Storing
         * only the left index halves the node and is what makes the sibling pair
         * share a cache line.
         */
        template <typename T>
        struct MeshBVHNode
        {
            AABB<T> bounds;
            /** @brief Interior: the left child's index. Leaf: the first triangle. */
            std::uint32_t first = 0;
            /** @brief How many triangles this leaf holds; zero marks an interior node. */
            std::uint32_t count = 0;
        };

        /**
         * @brief A borrowed view of a cooked triangle mesh, placed in the world.
         *
         * A *view* for the same reason a hull is one: the cooked arrays are shared
         * by every instance of the asset, and a shape value must stay small and
         * trivially copyable. The vertices are in the mesh's own frame; @ref center
         * and @ref orientation place it.
         */
        template <typename T>
        struct TriangleMeshView
        {
            const Vector3T<T>* vertices = nullptr;
            /** @brief Three vertex indices per triangle. */
            const std::uint32_t* indices = nullptr;
            const MeshBVHNode<T>* nodes = nullptr;
            /** @brief The cooked triangle order the leaves index into. */
            const std::uint32_t* order = nullptr;
            /**
             * @brief Three neighbour triangle indices per triangle, or @ref no_adjacent_triangle.
             *
             * Entry `3 * t + e` is the triangle sharing edge `e` of triangle `t`,
             * where edge 0 is (v0, v1), edge 1 is (v1, v2), and edge 2 is (v2, v0).
             */
            const std::uint32_t* adjacency = nullptr;
            std::uint32_t triangle_count = 0;
            std::uint32_t node_count = 0;
            Vector3T<T> center{Vector3T<T>{T(0), T(0), T(0)}};
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};
        };

        /**
         * @brief The arrays a cooked mesh owns, produced by @ref build_mesh_bvh.
         *
         * Host-side and owning; the runtime never holds one of these, only a
         * @ref TriangleMeshView over it. Separate from the view so the ownership is
         * visible in the type rather than in a comment.
         */
        template <typename T>
        struct CookedTriangleMesh
        {
            std::vector<MeshBVHNode<T>> nodes;
            std::vector<std::uint32_t> order;
            std::vector<std::uint32_t> adjacency;
        };

        /** @brief The world-space triangle @p index of a placed mesh. */
        template <typename T>
        inline TriangleCollider<T> mesh_triangle(const TriangleMeshView<T>& mesh,
                                                 std::uint32_t index) noexcept
        {
            const std::uint32_t* corner = mesh.indices + 3u * index;
            TriangleCollider<T> triangle;
            triangle.a = mesh.center + rotate(mesh.orientation, mesh.vertices[corner[0]]);
            triangle.b = mesh.center + rotate(mesh.orientation, mesh.vertices[corner[1]]);
            triangle.c = mesh.center + rotate(mesh.orientation, mesh.vertices[corner[2]]);
            return triangle;
        }

        /** @brief The mesh-local bounds of triangle @p index. */
        template <typename T>
        inline AABB<T> local_triangle_bounds(const Vector3T<T>* vertices,
                                             const std::uint32_t* indices,
                                             std::uint32_t index) noexcept
        {
            const std::uint32_t* corner = indices + 3u * index;
            const Vector3T<T>& a = vertices[corner[0]];
            const Vector3T<T>& b = vertices[corner[1]];
            const Vector3T<T>& c = vertices[corner[2]];
            AABB<T> bounds;
            bounds.min = Vector3T<T>{std::min(a.x, std::min(b.x, c.x)),
                                     std::min(a.y, std::min(b.y, c.y)),
                                     std::min(a.z, std::min(b.z, c.z))};
            bounds.max = Vector3T<T>{std::max(a.x, std::max(b.x, c.x)),
                                     std::max(a.y, std::max(b.y, c.y)),
                                     std::max(a.z, std::max(b.z, c.z))};
            return bounds;
        }

        /**
         * @brief Cooks the edge adjacency of a triangle soup.
         *
         * Two triangles neighbour each other when they share an edge, which means
         * sharing an unordered pair of vertex indices. A map from that pair to the
         * first triangle claiming it turns the whole job into one pass.
         *
         * An edge claimed by three or more triangles is non-manifold. That is a
         * real thing artists produce, and the honest response is to record only the
         * first pairing rather than to pick a "best" neighbour: the internal-edge
         * correction then treats the extra faces as boundaries, which is the
         * conservative answer, instead of silently smoothing over geometry that
         * genuinely has a crease.
         *
         * @return Three neighbour indices per triangle, @ref no_adjacent_triangle
         *         where the edge is open.
         */
        inline std::vector<std::uint32_t> build_triangle_adjacency(const std::uint32_t* indices,
                                                                   std::uint32_t triangle_count)
        {
            std::vector<std::uint32_t> adjacency(3u * static_cast<std::size_t>(triangle_count),
                                                 no_adjacent_triangle);
            // (lower vertex, higher vertex) -> (triangle, edge) that claimed it first.
            std::map<std::pair<std::uint32_t, std::uint32_t>,
                     std::pair<std::uint32_t, std::uint32_t>>
                claimed;

            for (std::uint32_t t = 0; t < triangle_count; ++t)
                for (std::uint32_t e = 0; e < 3; ++e)
                {
                    const std::uint32_t v0 = indices[3u * t + e];
                    const std::uint32_t v1 = indices[3u * t + (e + 1u) % 3u];
                    const std::pair<std::uint32_t, std::uint32_t> key =
                        v0 < v1 ? std::make_pair(v0, v1) : std::make_pair(v1, v0);
                    const auto found = claimed.find(key);
                    if (found == claimed.end())
                    {
                        claimed.emplace(key, std::make_pair(t, e));
                        continue;
                    }
                    // Already paired by a third triangle: leave both as boundaries.
                    if (adjacency[3u * found->second.first + found->second.second] !=
                        no_adjacent_triangle)
                        continue;
                    adjacency[3u * t + e] = found->second.first;
                    adjacency[3u * found->second.first + found->second.second] = t;
                }
            return adjacency;
        }

        /**
         * @brief Builds the cooked hierarchy over a triangle soup. Host-only.
         *
         * A median split on the widest axis of each node's centroid spread, down to
         * leaves of a few triangles. Not a surface-area-heuristic build: the SAH
         * pays for itself on a hierarchy that is *traversed* by rays millions of
         * times, and a physics mesh is queried by a handful of overlapping bodies
         * per tick against a tree that is rebuilt never. The cheaper build is the
         * right trade here, and swapping it later changes this function and nothing
         * that uses it.
         *
         * @param vertices       Mesh-local vertex positions.
         * @param indices        Three vertex indices per triangle.
         * @param triangle_count How many triangles.
         * @param leaf_size      Triangles per leaf before a node stops splitting.
         */
        template <typename T>
        inline CookedTriangleMesh<T> build_mesh_bvh(const Vector3T<T>* vertices,
                                                    const std::uint32_t* indices,
                                                    std::uint32_t triangle_count,
                                                    std::uint32_t leaf_size = 4)
        {
            CookedTriangleMesh<T> cooked;
            cooked.adjacency = build_triangle_adjacency(indices, triangle_count);
            if (triangle_count == 0)
                return cooked;

            cooked.order.resize(triangle_count);
            std::vector<AABB<T>> bounds(triangle_count);
            std::vector<Vector3T<T>> centroids(triangle_count);
            for (std::uint32_t t = 0; t < triangle_count; ++t)
            {
                cooked.order[t] = t;
                bounds[t] = local_triangle_bounds(vertices, indices, t);
                centroids[t] = (bounds[t].min + bounds[t].max) * T(0.5);
            }

            cooked.nodes.reserve(2u * static_cast<std::size_t>(triangle_count));
            cooked.nodes.push_back(MeshBVHNode<T>{});

            // An explicit stack rather than recursion: the same shape the traversal
            // uses, and it cannot overflow on a degenerate mesh.
            struct Pending
            {
                std::uint32_t node;
                std::uint32_t first;
                std::uint32_t count;
            };
            std::vector<Pending> pending;
            pending.push_back(Pending{0u, 0u, triangle_count});

            while (!pending.empty())
            {
                const Pending job = pending.back();
                pending.pop_back();

                AABB<T> node_bounds = bounds[cooked.order[job.first]];
                for (std::uint32_t i = 1; i < job.count; ++i)
                    node_bounds = aabb_union(node_bounds, bounds[cooked.order[job.first + i]]);
                cooked.nodes[job.node].bounds = node_bounds;

                if (job.count <= leaf_size)
                {
                    cooked.nodes[job.node].first = job.first;
                    cooked.nodes[job.node].count = job.count;
                    continue;
                }

                // Split on the axis the centroids are most spread across.
                Vector3T<T> low = centroids[cooked.order[job.first]];
                Vector3T<T> high = low;
                for (std::uint32_t i = 1; i < job.count; ++i)
                {
                    const Vector3T<T>& centroid = centroids[cooked.order[job.first + i]];
                    low = Vector3T<T>{std::min(low.x, centroid.x), std::min(low.y, centroid.y),
                                      std::min(low.z, centroid.z)};
                    high = Vector3T<T>{std::max(high.x, centroid.x), std::max(high.y, centroid.y),
                                       std::max(high.z, centroid.z)};
                }
                const Vector3T<T> spread = high - low;
                const int axis = spread.x >= spread.y && spread.x >= spread.z
                                     ? 0
                                     : (spread.y >= spread.z ? 1 : 2);

                const auto coordinate = [&](std::uint32_t triangle) noexcept
                {
                    const Vector3T<T>& centroid = centroids[triangle];
                    return axis == 0 ? centroid.x : (axis == 1 ? centroid.y : centroid.z);
                };

                const auto begin = cooked.order.begin() + job.first;
                const auto end = begin + job.count;
                const auto middle = begin + job.count / 2;
                std::nth_element(begin, middle, end,
                                 [&](std::uint32_t l, std::uint32_t r)
                                 { return coordinate(l) < coordinate(r); });

                const std::uint32_t left_count = job.count / 2;
                const std::uint32_t left = static_cast<std::uint32_t>(cooked.nodes.size());
                cooked.nodes.push_back(MeshBVHNode<T>{});
                cooked.nodes.push_back(MeshBVHNode<T>{});
                cooked.nodes[job.node].first = left;
                cooked.nodes[job.node].count = 0;

                pending.push_back(Pending{left, job.first, left_count});
                pending.push_back(Pending{left + 1u, job.first + left_count,
                                          job.count - left_count});
            }
            return cooked;
        }

        /** @brief A view over @p cooked, placed in the world. */
        template <typename T>
        inline TriangleMeshView<T> make_mesh_view(
            const CookedTriangleMesh<T>& cooked, const Vector3T<T>* vertices,
            const std::uint32_t* indices, std::uint32_t triangle_count,
            const Vector3T<T>& center = Vector3T<T>{T(0), T(0), T(0)},
            const QuaternionT<T>& orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)}) noexcept
        {
            TriangleMeshView<T> view;
            view.vertices = vertices;
            view.indices = indices;
            view.nodes = cooked.nodes.data();
            view.order = cooked.order.data();
            view.adjacency = cooked.adjacency.data();
            view.triangle_count = triangle_count;
            view.node_count = static_cast<std::uint32_t>(cooked.nodes.size());
            view.center = center;
            view.orientation = orientation;
            return view;
        }

        /**
         * @brief The mesh-local box that conservatively contains a world-space box.
         *
         * The query comes from a body in world space and the hierarchy is in mesh
         * space, so one of them has to move. Moving the query is one transform
         * instead of a hundred thousand — and a rotated box's mesh-local extent is
         * the sum of its absolute axis projections, which over-covers rather than
         * under-covers. A conservative query costs a few extra triangle tests; an
         * optimistic one misses contacts.
         */
        template <typename T>
        inline AABB<T> world_box_to_mesh_space(const AABB<T>& box,
                                               const TriangleMeshView<T>& mesh) noexcept
        {
            const Vector3T<T> center = (box.min + box.max) * T(0.5);
            const Vector3T<T> extent = (box.max - box.min) * T(0.5);
            const Vector3T<T> local_center =
                rotate(conjugate(mesh.orientation), center - mesh.center);

            Vector3T<T> axes[3];
            axes[0] = rotate(conjugate(mesh.orientation), Vector3T<T>{T(1), T(0), T(0)});
            axes[1] = rotate(conjugate(mesh.orientation), Vector3T<T>{T(0), T(1), T(0)});
            axes[2] = rotate(conjugate(mesh.orientation), Vector3T<T>{T(0), T(0), T(1)});
            const T components[3] = {extent.x, extent.y, extent.z};
            Vector3T<T> local_extent{T(0), T(0), T(0)};
            for (int i = 0; i < 3; ++i)
            {
                local_extent.x += std::abs(axes[i].x) * components[i];
                local_extent.y += std::abs(axes[i].y) * components[i];
                local_extent.z += std::abs(axes[i].z) * components[i];
            }
            return AABB<T>{local_center - local_extent, local_center + local_extent};
        }

        /**
         * @brief Calls @p visit with every triangle whose bounds meet @p world_box.
         *
         * A fixed-depth stack walk, no allocation and no recursion, so the same code
         * runs on the device. Triangles are reported in cooked order, which is a
         * function of the mesh alone — so the set *and its order* are the same on
         * every run and on every machine, which is what §12.1 needs from a
         * narrowphase input.
         */
        template <typename T, typename Visit>
        inline void query_mesh_bvh(const TriangleMeshView<T>& mesh, const AABB<T>& world_box,
                                   Visit&& visit) noexcept
        {
            if (mesh.node_count == 0 || mesh.triangle_count == 0)
                return;
            const AABB<T> box = world_box_to_mesh_space(world_box, mesh);

            std::uint32_t stack[max_bvh_stack_depth];
            std::size_t depth = 0;
            stack[depth++] = 0;

            while (depth > 0)
            {
                const MeshBVHNode<T>& node = mesh.nodes[stack[--depth]];
                if (!aabb_overlap(node.bounds, box))
                    continue;
                if (node.count > 0)
                {
                    // The triangle's own bounds, not just its leaf's. A leaf holds
                    // several triangles and its box is their union, so most of what
                    // it contains is usually nowhere near the query. Reporting the
                    // whole leaf is correct but expensive in the wrong place: every
                    // spurious triangle costs a full convex query downstream, which
                    // is orders of magnitude more than the box test that would have
                    // rejected it here.
                    for (std::uint32_t i = 0; i < node.count; ++i)
                    {
                        const std::uint32_t triangle = mesh.order[node.first + i];
                        if (aabb_overlap(
                                local_triangle_bounds(mesh.vertices, mesh.indices, triangle), box))
                            visit(triangle);
                    }
                    continue;
                }
                if (depth + 2 > max_bvh_stack_depth)
                    continue; // pathologically deep tree: skip rather than overflow
                stack[depth++] = node.first;
                stack[depth++] = node.first + 1u;
            }
        }
    } // namespace Physics
} // namespace SushiEngine
