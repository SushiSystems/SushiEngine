/**************************************************************************/
/* soft_surface_hierarchy.hpp                                             */
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
 * @file soft_surface_hierarchy.hpp
 * @brief A bounding-volume hierarchy over a surface that moves every tick.
 *
 * §9.6.2's broad phase. A rigid mesh's hierarchy is cooked once and never
 * touched again (`geometry/mesh_bvh.hpp`); a soft body's surface has different
 * vertex positions every tick and the same topology for ever. That asymmetry
 * decides the design: the tree is **built once and refitted every tick**, not
 * rebuilt. A refit walks the nodes from the leaves up and recomputes bounds in
 * one linear pass with no allocation, against `O(n log n)` and a full
 * reallocation for a rebuild — and it is correct for any deformation, because
 * nothing about a bounding volume requires it to be the *tightest* one. Only
 * its quality decays, and a body deformed far enough for that to matter has a
 * different problem than a slightly larger box.
 *
 * The build itself is `build_mesh_bvh`, not a second implementation: the
 * hierarchy gathers its particles into a plain position array, which is exactly
 * what that builder and its traversal already take. The gather is not waste —
 * a surface test reads positions repeatedly and reading them out of a
 * 200-byte body struct would touch a new cache line per vertex.
 *
 * The refit relies on one property of that builder: a node's children are
 * always allocated after it, so a single reverse sweep over the node array sees
 * every child before its parent.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/geometry/mesh_bvh.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The hierarchy over one deformable surface, refitted per tick.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SoftSurfaceHierarchy
        {
            public:
                /**
                 * @brief Builds the tree over a surface's triangles, once.
                 *
                 * @param surface_indices Three particle indices per surface triangle;
                 *                        borrowed, and must outlive the hierarchy.
                 * @param index_count     How many indices (three times the triangle count).
                 * @param particles       The body's particles, for the initial bounds.
                 * @param particle_count  How many particles the body has.
                 */
                void build(const std::uint32_t* surface_indices, std::size_t index_count,
                           const RigidBodyT<T>* particles, std::size_t particle_count)
                {
                    indices_ = surface_indices;
                    triangle_count_ = std::uint32_t(index_count / 3);
                    positions_.assign(particle_count, Vector3T<T>{T(0), T(0), T(0)});
                    if (indices_ == nullptr || triangle_count_ == 0)
                    {
                        cooked_ = CookedTriangleMesh<T>{};
                        return;
                    }

                    gather(particles);
                    cooked_ = build_mesh_bvh(positions_.data(), indices_, triangle_count_);
                }

                /**
                 * @brief Re-reads the particles and widens every node to fit them again.
                 *
                 * @param particles The body's particles at their current positions.
                 * @param margin    Half the contact thickness the tests will use, added
                 *                  to every leaf so a pair that is close but not yet
                 *                  touching still reaches the narrow phase.
                 */
                void refit(const RigidBodyT<T>* particles, T margin) noexcept
                {
                    if (cooked_.nodes.empty())
                        return;
                    gather(particles);

                    const Vector3T<T> pad{margin, margin, margin};
                    for (std::size_t i = cooked_.nodes.size(); i-- > 0;)
                    {
                        MeshBvhNode<T>& node = cooked_.nodes[i];
                        if (node.count > 0)
                        {
                            Aabb<T> bounds =
                                local_triangle_bounds(positions_.data(), indices_,
                                                      cooked_.order[node.first]);
                            for (std::uint32_t t = 1; t < node.count; ++t)
                                bounds = aabb_union(
                                    bounds, local_triangle_bounds(positions_.data(), indices_,
                                                                  cooked_.order[node.first + t]));
                            node.bounds.min = bounds.min - pad;
                            node.bounds.max = bounds.max + pad;
                        }
                        else
                        {
                            node.bounds = aabb_union(cooked_.nodes[node.first].bounds,
                                                     cooked_.nodes[node.first + 1].bounds);
                        }
                    }
                }

                /** @brief The surface's world-space positions, indexed by particle. */
                const std::vector<Vector3T<T>>& positions() const noexcept
                {
                    return positions_;
                }

                /** @brief Three particle indices per surface triangle. */
                const std::uint32_t* indices() const noexcept
                {
                    return indices_;
                }

                /** @brief How many surface triangles the hierarchy covers. */
                std::uint32_t triangle_count() const noexcept
                {
                    return triangle_count_;
                }

                /** @brief The hierarchy's nodes; empty until @ref build has run. */
                const std::vector<MeshBvhNode<T>>& nodes() const noexcept
                {
                    return cooked_.nodes;
                }

                /** @brief The triangle order the leaves index into. */
                const std::vector<std::uint32_t>& order() const noexcept
                {
                    return cooked_.order;
                }

                /** @brief The whole surface's current bounds, or an empty box when there is none. */
                Aabb<T> bounds() const noexcept
                {
                    if (cooked_.nodes.empty())
                        return Aabb<T>{Vector3T<T>{T(0), T(0), T(0)}, Vector3T<T>{T(0), T(0), T(0)}};
                    return cooked_.nodes[0].bounds;
                }

            private:
                void gather(const RigidBodyT<T>* particles) noexcept
                {
                    for (std::size_t i = 0; i < positions_.size(); ++i)
                        positions_[i] = particles[i].position;
                }

                const std::uint32_t* indices_ = nullptr;
                std::uint32_t triangle_count_ = 0;
                std::vector<Vector3T<T>> positions_;
                CookedTriangleMesh<T> cooked_;
        };

        /**
         * @brief Calls @p visit with every triangle pair whose bounds overlap.
         *
         * A simultaneous descent of the two trees: a node pair whose boxes miss is
         * dropped whole, and otherwise the *larger* of the two is split, which is
         * what keeps the two descents in step rather than driving one to its leaves
         * against the other's root.
         *
         * @tparam T     The scalar element type.
         * @tparam Visit Callable as `visit(triangle_in_a, triangle_in_b)`.
         * @param a     The first surface, refitted this tick.
         * @param b     The second surface, refitted this tick.
         * @param visit Receives each candidate pair, in a fixed traversal order.
         */
        template <typename T, typename Visit>
        inline void for_each_overlapping_triangle_pair(const SoftSurfaceHierarchy<T>& a,
                                                       const SoftSurfaceHierarchy<T>& b,
                                                       Visit visit)
        {
            if (a.nodes().empty() || b.nodes().empty())
                return;

            struct NodePair
            {
                std::uint32_t left;
                std::uint32_t right;
            };
            std::vector<NodePair> stack;
            stack.push_back(NodePair{0u, 0u});

            const auto extent = [](const Aabb<T>& box) noexcept
            {
                const Vector3T<T> span = box.max - box.min;
                return span.x + span.y + span.z;
            };

            while (!stack.empty())
            {
                const NodePair pair = stack.back();
                stack.pop_back();

                const MeshBvhNode<T>& node_a = a.nodes()[pair.left];
                const MeshBvhNode<T>& node_b = b.nodes()[pair.right];
                if (!aabb_overlap(node_a.bounds, node_b.bounds))
                    continue;

                const bool leaf_a = node_a.count > 0;
                const bool leaf_b = node_b.count > 0;
                if (leaf_a && leaf_b)
                {
                    for (std::uint32_t i = 0; i < node_a.count; ++i)
                        for (std::uint32_t j = 0; j < node_b.count; ++j)
                            visit(a.order()[node_a.first + i], b.order()[node_b.first + j]);
                    continue;
                }

                const bool split_a =
                    leaf_b || (!leaf_a && extent(node_a.bounds) >= extent(node_b.bounds));
                if (split_a)
                {
                    stack.push_back(NodePair{node_a.first, pair.right});
                    stack.push_back(NodePair{node_a.first + 1u, pair.right});
                }
                else
                {
                    stack.push_back(NodePair{pair.left, node_b.first});
                    stack.push_back(NodePair{pair.left, node_b.first + 1u});
                }
            }
        }
    } // namespace Physics
} // namespace SushiEngine
