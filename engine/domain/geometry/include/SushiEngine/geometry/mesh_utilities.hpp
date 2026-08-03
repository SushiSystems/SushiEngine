/**************************************************************************/
/* mesh_utilities.hpp                                                     */
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
 * @file mesh_utilities.hpp
 * @brief What a cooker has to know about a triangle mesh before it can cook it.
 *
 * Every stage downstream of an import assumes something about its input — that a
 * shared corner is one vertex, that a triangle has an area, that the outside is the
 * side the normals point at. Real project meshes honour none of that: they arrive
 * welded by nobody, triangulated by four different tools, and wound both ways in the
 * same file. So the pipeline's first stage is measurement and repair, and this is
 * where both live.
 *
 * The split is deliberate. @ref analyze_mesh_topology only *reports* — it is what an
 * inspector shows and what a cook report carries, and it never touches the geometry.
 * @ref repair_mesh produces a new mesh and reports what it had to change. Nothing
 * here fixes a mesh silently, because a non-manifold input is a fact the artist needs
 * (§8.3 stage 1) and a repair that hides it is a bug reported three weeks later as
 * "the physics is wrong".
 *
 * Raw `float[3]` rather than a vector type, matching @ref TriangleMeshView: this
 * module sits below the engine's scalar policy and must not acquire an opinion on it.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Geometry
    {
        /**
         * @brief What a mesh is, measured rather than assumed.
         *
         * Counts rather than verdicts, with the verdicts derived: a caller that wants
         * to fail a cook needs a threshold, and a threshold needs a number. The three
         * predicates below are the questions the cooker actually asks.
         */
        struct MeshTopologyReport
        {
            /** @brief Vertices in the analysed mesh. */
            std::size_t vertex_count = 0;

            /** @brief Triangles in the analysed mesh, ignoring a trailing partial one. */
            std::size_t triangle_count = 0;

            /** @brief Triangles indexing a vertex that does not exist. */
            std::size_t out_of_range_triangles = 0;

            /** @brief Triangles with a repeated corner or zero area. */
            std::size_t degenerate_triangles = 0;

            /** @brief Triangles repeating an earlier triangle's three corners. */
            std::size_t duplicate_triangles = 0;

            /** @brief Vertices no triangle references. */
            std::size_t unreferenced_vertices = 0;

            /** @brief Edges with exactly one incident triangle — the surface has a hole. */
            std::size_t boundary_edges = 0;

            /** @brief Edges with three or more incident triangles. */
            std::size_t non_manifold_edges = 0;

            /**
             * @brief Edges whose two triangles traverse them the same way round.
             *
             * The signature of a winding disagreement: two triangles sharing an edge
             * agree on the outside only if one walks the edge a→b and the other b→a.
             */
            std::size_t inconsistent_edges = 0;

            /** @brief Edge-connected components of the triangle set. */
            std::size_t connected_components = 0;

            /** @brief Total triangle area, in square local units. */
            float surface_area = 0.0f;

            /**
             * @brief Volume by the divergence theorem; meaningful only when closed.
             *
             * Signed on purpose: a closed mesh wound inside-out reports a negative
             * volume, which is how @ref repair_mesh decides a whole component needs
             * reversing. Reported for an open mesh too, where it is the volume of the
             * cone fan to the origin and means nothing.
             */
            float signed_volume = 0.0f;

            /** @brief Whether the surface encloses a volume: no holes, no fins. */
            bool watertight() const noexcept
            {
                return triangle_count > 0 && boundary_edges == 0 && non_manifold_edges == 0;
            }

            /** @brief Whether every edge has at most two triangles. */
            bool manifold() const noexcept { return non_manifold_edges == 0; }

            /** @brief Whether every shared edge is traversed in opposite directions. */
            bool consistently_oriented() const noexcept { return inconsistent_edges == 0; }
        };

        /**
         * @brief Measures a mesh without changing it.
         *
         * One pass builds the edge table — every triangle's three directed edges, keyed
         * by their unordered vertex pair — and the counts fall out of how many
         * triangles each key collected and in which directions. Components come from a
         * union-find over the same edges, so a mesh made of two shells reports two
         * rather than looking like one badly connected surface.
         *
         * Degenerate and out-of-range triangles are counted but contribute no edges: a
         * triangle indexing vertex 4 000 000 000 must not be allowed to invent an edge
         * that makes the rest of the report nonsense.
         *
         * @param mesh The geometry to measure; only positions and indices are read.
         * @return The report; all-zero with @c triangle_count zero for an empty mesh.
         */
        MeshTopologyReport analyze_mesh_topology(const TriangleMeshView& mesh);

        /** @brief Which repairs @ref repair_mesh is allowed to perform. */
        struct MeshRepairOptions
        {
            /**
             * @brief Distance below which two vertices are the same vertex, in local units.
             *
             * Absolute rather than relative to the mesh's size, because the number that
             * matters is the one the modelling tool's snap grid used, and that is in the
             * asset's own units. Zero or less welds nothing.
             */
            float weld_tolerance = 1.0e-5f;

            /**
             * @brief Whether to make each component's winding agree with itself.
             *
             * Degenerate and duplicate triangles are not optional and have no flag:
             * a triangle with no area is not geometry, and a repeated triangle makes
             * every one of its three edges non-manifold, which is precisely the input
             * orientation propagation has no answer for. Both are always dropped and
             * both are always *counted* in @ref MeshRepairReport::before, which is the
             * thing the artist actually needs (§8.3 stage 1).
             */
            bool orient_consistently = true;

            /**
             * @brief Whether to reverse a closed component that encloses negative volume.
             *
             * Consistency alone leaves a shell inside-out if that is how it arrived;
             * this is what makes the normals point *out*. Skipped for an open component,
             * where the enclosed volume is not a quantity, and read only when
             * @ref orient_consistently is set, since a component whose own windings
             * disagree has no single sign to reverse.
             */
            bool orient_outward = true;

            /** @brief Whether to drop vertices no surviving triangle references. */
            bool drop_unreferenced_vertices = true;
        };

        /** @brief What a repair changed, and what the mesh looked like either side of it. */
        struct MeshRepairReport
        {
            /** @brief The input as it arrived. */
            MeshTopologyReport before;

            /** @brief The output. Compare the two to see what the repair bought. */
            MeshTopologyReport after;

            /** @brief Vertices that disappeared into another vertex. */
            std::size_t welded_vertices = 0;

            /** @brief Vertices dropped for being referenced by nothing. */
            std::size_t removed_vertices = 0;

            /** @brief Triangles dropped as degenerate or duplicate. */
            std::size_t removed_triangles = 0;

            /** @brief Triangles whose winding was reversed to agree with a neighbour. */
            std::size_t reoriented_triangles = 0;

            /** @brief Closed components reversed whole because they enclosed negative volume. */
            std::size_t reversed_components = 0;
        };

        /**
         * @brief Repairs a mesh into @p out, reporting every change it made.
         *
         * The four stages run in the only order that works. Welding first, because
         * which triangles share an edge is undecidable until coincident corners are one
         * vertex, and everything after depends on adjacency. Then the degenerate and
         * duplicate drop, because welding *creates* degenerate triangles — a sliver
         * whose two ends weld together becomes a line. Then orientation, which needs
         * the adjacency the first two stages made meaningful. Compaction last, since it
         * is the only stage that can renumber a vertex.
         *
         * Welding uses a hash grid at the tolerance and searches the twenty-seven cells
         * around each vertex, so the result is what the tolerance says rather than what
         * a cell boundary happened to cut. Vertices weld to the lowest-numbered vertex
         * they are within tolerance of, which makes the output a function of the input's
         * order and not of the container's.
         *
         * @param source  The mesh to repair; not modified.
         * @param options Which repairs to perform and at what tolerance.
         * @param out     Receives the repaired mesh; cleared first. Normals are dropped,
         *                because welding and reorientation invalidate them and a stale
         *                normal is worse than an absent one.
         * @return What changed, with the before and after topology for the cook report.
         */
        MeshRepairReport repair_mesh(const TriangleMeshView& source,
                                     const MeshRepairOptions& options, TriangleMesh& out);

        /**
         * @brief The closest point on a triangle to a query point.
         *
         * The region-based routine from Ericson, "Real-Time Collision Detection"
         * (section 5.1.5): the point is tested against the triangle's seven Voronoi
         * regions — three vertices, three edges, the interior face — and the closest
         * point on whichever region owns the query is returned, so an edge or a corner
         * is answered exactly rather than approximated by the plane.
         *
         * Public because three separate callers want it — the distance-field baker, the
         * closest-point query, and the soft-body cooker's boundary fit — and a routine
         * this easy to get subtly wrong should exist once.
         *
         * Total for a degenerate triangle: a collapsed edge or a zero-area face returns
         * a point that is on the triangle rather than a not-a-number. Welding a sliver
         * produces exactly such a triangle, so this is the ordinary case and not a
         * hypothetical one.
         *
         * @param point The query point.
         * @param a     First triangle vertex.
         * @param b     Second triangle vertex.
         * @param c     Third triangle vertex.
         * @param out   Receives the nearest point on the triangle.
         */
        void closest_point_on_triangle(const float point[3], const float a[3], const float b[3],
                                       const float c[3], float out[3]) noexcept;

        /**
         * @brief The barycentric coordinates of a point with respect to a tetrahedron.
         *
         * The four signed volume ratios, which is the whole of §8.3 stage 6: a render
         * vertex inside the tetrahedron has four coordinates in [0, 1] summing to one,
         * and a vertex outside gets the same expression *extrapolated* — one or more
         * coordinates negative — which is exactly what keeps a thin feature attached to
         * the nearest element instead of unbound.
         *
         * @param point   The point to express.
         * @param a       First tetrahedron vertex.
         * @param b       Second tetrahedron vertex.
         * @param c       Third tetrahedron vertex.
         * @param d       Fourth tetrahedron vertex.
         * @param weights Receives the four coordinates, in vertex order.
         * @return False for a degenerate (zero-volume) tetrahedron, leaving @p weights
         *         at one-quarter each so a caller that ignores the result still averages
         *         rather than reads uninitialized memory.
         */
        bool tetrahedron_barycentric(const float point[3], const float a[3], const float b[3],
                                     const float c[3], const float d[3],
                                     float weights[4]) noexcept;

        /**
         * @brief Six times the signed volume of a tetrahedron.
         *
         * Six times, and not the volume, because every caller either compares it to
         * zero or sums it against siblings, and the division is one place at the end.
         * Positive when @p d is on the positive side of triangle @p a, @p b, @p c.
         *
         * @param a First vertex.
         * @param b Second vertex.
         * @param c Third vertex.
         * @param d Fourth vertex.
         * @return The determinant `(b-a) x (c-a) . (d-a)`.
         */
        float tetrahedron_signed_volume_times_six(const float a[3], const float b[3],
                                                  const float c[3], const float d[3]) noexcept;
    } // namespace Geometry
} // namespace SushiEngine
