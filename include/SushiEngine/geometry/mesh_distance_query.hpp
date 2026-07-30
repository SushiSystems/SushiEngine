/**************************************************************************/
/* mesh_distance_query.hpp                                                */
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
 * @file mesh_distance_query.hpp
 * @brief "How far is this point from that surface", asked a hundred million times.
 *
 * Four stages of the cooking pipeline ask exactly this question and only differ in
 * what they do with the answer: the distance-field bake asks it once per voxel, the
 * soft-body cooker's boundary fit asks it once per straddling cell, the accuracy
 * report asks it once per surface sample, and the render-mesh embedding asks a
 * neighbouring form of it. Answered by walking every triangle, that is
 * O(queries x triangles) and a 128-cubed field over fifty thousand triangles is a
 * hundred billion triangle tests — which is the difference between §13.1's three-second
 * cook budget and a coffee break.
 *
 * So the triangles go into a bounding-volume hierarchy once and every query descends
 * it, nearest child first, pruning any box already further away than the best point
 * found. The hierarchy is built by median split on centroids along the widest axis:
 * cheap to build, good enough to query, and — because a median split on a fixed order
 * is a deterministic function of the input — it returns the same triangle for a tie
 * every run, which a cook report that records a distance needs.
 *
 * Separate from `physics/geometry/mesh_bvh.hpp` on purpose. That hierarchy answers
 * *overlap* queries against a shape, per tick, on the device's side of the fence and
 * in the engine's scalar policy; this one answers *distance* queries on the host in
 * plain `float`, and it lives below the physics because an offline cooker must not
 * depend upward to get it (§3.4).
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /** @brief The nearest point of a surface to a query, and which triangle carries it. */
        struct MeshClosestPoint
        {
            /** @brief The nearest point, in the mesh's own space. */
            float point[3] = {0.0f, 0.0f, 0.0f};

            /** @brief Unsigned distance from the query to @ref point. */
            float distance = 0.0f;

            /** @brief Index of the triangle @ref point lies on, into the built mesh. */
            std::uint32_t triangle = 0;

            /** @brief False when the hierarchy holds no triangles; the rest is then unset. */
            bool valid = false;
        };

        /**
         * @brief A triangle mesh prepared for distance queries.
         *
         * Owns a copy of the triangles rather than a view of them, because a cook that
         * outlives the importer's buffer is the normal case and a hierarchy pointing at
         * freed geometry fails in the least debuggable way available.
         */
        class MeshDistanceQuery
        {
        public:
            /**
             * @brief Builds the hierarchy over @p mesh's triangles.
             *
             * Degenerate and out-of-range triangles are skipped: a zero-area triangle
             * contributes no closest point that a neighbouring valid triangle does not
             * already contribute, and it would give the sign lookup a normal of zero.
             *
             * @param mesh The geometry to prepare; copied.
             * @return False when @p mesh yielded no usable triangle, leaving this empty.
             */
            bool build(const TriangleMeshView& mesh);

            /** @brief Whether a build succeeded and queries will answer. */
            bool ready() const noexcept { return !triangles_.empty(); }

            /** @brief Triangles in the hierarchy — the *usable* ones, not the source's count. */
            std::size_t triangle_count() const noexcept { return triangles_.size(); }

            /** @brief Nodes in the hierarchy; reported so a cook can record its own cost. */
            std::size_t node_count() const noexcept { return nodes_.size(); }

            /**
             * @brief The nearest point of the surface to @p point.
             *
             * @param point The query position, in the mesh's space.
             * @return The nearest point and its distance, or an invalid result when empty.
             */
            MeshClosestPoint closest_point(const float point[3]) const noexcept;

            /**
             * @brief The distance to the surface, negative inside it.
             *
             * The sign comes from the nearest triangle's geometric normal, which is the
             * rule the distance-field bake has always used and is exact except within a
             * hair of a concave edge, where the nearest triangle's plane is not the
             * surface's local side. Callers that cannot tolerate that — deciding what is
             * interior to a dirty mesh, above all — use the flood fill instead, which is
             * why §8.3 stage 2 voxelizes rather than sampling this.
             *
             * @param point The query position, in the mesh's space.
             * @return The signed distance, or zero when the hierarchy is empty.
             */
            float signed_distance(const float point[3]) const noexcept;

            /** @brief The hierarchy's root bounds; empty-mesh safe (all zeroes). */
            void bounds(float minimum[3], float maximum[3]) const noexcept;

        private:
            /**
             * @brief One hierarchy node: its box, and either its children or its triangles.
             *
             * @c count is non-zero exactly for a leaf, where @c first indexes @ref order_.
             * An internal node names both children explicitly rather than implying the
             * right one sits beside the left: the build recurses depth-first, so the left
             * child's whole subtree lies between them and "the next node" is that
             * subtree's second node, not the sibling.
             */
            struct Node
            {
                float minimum[3];
                float maximum[3];
                std::uint32_t left;
                std::uint32_t right;
                std::uint32_t first;
                std::uint32_t count;
            };

            /** @brief One triangle, with its normal and centroid cached for the build. */
            struct Triangle
            {
                float vertex[3][3];
                float normal[3];
                float centroid[3];
            };

            std::uint32_t build_node(std::size_t first, std::size_t count);

            std::vector<Triangle> triangles_;
            std::vector<std::uint32_t> order_;
            std::vector<Node> nodes_;
        };

        /**
         * @brief Deterministic sample points across a mesh's surface.
         *
         * A barycentric lattice per triangle rather than random points: a cook report's
         * accuracy number has to be reproducible, and a stochastic sampler makes
         * "re-cook produced a different Hausdorff error" an expected outcome, which
         * destroys the number's only use — noticing when it changes.
         *
         * @param mesh    The surface to sample.
         * @param order   Lattice order; produces `(order+1)(order+2)/2` points per
         *                triangle, so 1 is the three corners, 2 adds the edge midpoints,
         *                and 4 gives fifteen. Clamped up to 1.
         * @param out     Receives three floats per sample; cleared first.
         * @return The number of samples written.
         */
        std::size_t sample_surface_points(const TriangleMeshView& mesh, std::uint32_t order,
                                         std::vector<float>& out);

        /**
         * @brief The sampled one-sided Hausdorff distance from @p source to @p target.
         *
         * The largest distance any point of @p source's surface lies from @p target's —
         * "how far does this mesh depart from that one", in local units, which is §7.6's
         * reported number.
         *
         * **It is a lower bound, and the report must say so.** The true supremum is over
         * a continuous surface and this is a maximum over a finite lattice, so a spike
         * between samples is under-reported. Raising @p order tightens it. A bound that
         * is honest about its direction is usable; a sampled maximum presented as the
         * exact figure is the same failure as a made-up timing.
         *
         * @param source The surface to sample.
         * @param target The surface to measure against; must be built.
         * @param order  Lattice order, as @ref sample_surface_points.
         * @return The largest sampled distance, or zero when either side is empty.
         */
        float one_sided_hausdorff_distance(const TriangleMeshView& source,
                                           const MeshDistanceQuery& target, std::uint32_t order);

        /**
         * @brief How far @p source's surface protrudes outside @p target's, at most.
         *
         * The same sampling as @ref one_sided_hausdorff_distance but reading the *signed*
         * distance and keeping the largest positive one, which answers the question §7.6
         * actually poses about a cooked collider: not how much the two surfaces differ
         * but how much fatter than the visible mesh the collision geometry is, because
         * that is the error a player feels as an invisible wall.
         *
         * @param source The surface to sample — the collider, for that question.
         * @param target The surface to measure against — the render mesh.
         * @param order  Lattice order, as @ref sample_surface_points.
         * @return The largest sampled outward distance, or zero when nothing protrudes.
         */
        float max_protrusion_distance(const TriangleMeshView& source,
                                      const MeshDistanceQuery& target, std::uint32_t order);
    } // namespace Geometry
} // namespace SushiEngine
