/**************************************************************************/
/* convex_decomposition.hpp                                               */
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
 * @file convex_decomposition.hpp
 * @brief Turning one concave mesh into a few convex pieces, with the error measured.
 *
 * §8.4 item 2. The output is what the narrowphase collides against, so the whole design
 * follows from one fact about the shape it has to produce: `ConvexHullView` carries a
 * vertex array, a placement and a convex radius, and `support()` scans the vertices. **A
 * cooked convex hull is a point set, not a face topology.** Nothing in the runtime reads
 * a hull face, so the decomposition never has to build one to be *correct* — only to be
 * compact, and only to measure itself.
 *
 * That reframing is what makes this module small. The two hard-looking parts of §8.4
 * dissolve:
 *
 * - **Mass properties are not computed from the pieces.** §8.4 item 1 integrates over the
 *   closed source mesh, which `Physics::mesh_mass_properties` already does exactly. The
 *   decomposition's approximation therefore never reaches what a body weighs or how it
 *   spins — only what it bumps into.
 * - **The vertex budget comes before the hull, not after.** Selecting at most `N`
 *   vertices from a piece and then hulling *those* keeps the hull build bounded at `N`
 *   points, where an exact brute-force construction is affordable and obviously correct.
 *   Because every selected point is a real mesh vertex, the selected hull is contained in
 *   the piece's true hull: simplification can only make the collider **thinner** than the
 *   mesh, never fatter, which is the safe direction for the one error a player feels.
 *
 * **How concavity is measured, and why it is the reported number.** A part's concavity is
 * the furthest its hull's surface protrudes outside the source mesh, via
 * `Geometry::max_protrusion_distance` against the source's distance hierarchy. That is not
 * a proxy for the error — it *is* §7.6's number, "the collider is three centimetres fatter
 * than the mesh", in local units. Splitting until every part's concavity is under a
 * tolerance therefore optimizes the quantity that is reported, rather than a volume ratio
 * that correlates with it.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/geometry/mesh_distance_query.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /**
             * @brief One convex piece of a decomposition.
             *
             * The vertices are **relative to @ref center**, which is the frame
             * `ConvexHullView` reads them in: a world point is
             * `center + rotate(orientation, vertex)`. Storing them centred also keeps the
             * numbers small, which matters for a piece of a mesh authored far from its
             * origin.
             */
            struct ConvexPiece
            {
                /** @brief The hull's vertices, relative to @ref center. */
                std::vector<Vector3> vertices;

                /** @brief The hull's centroid, in the source mesh's frame. */
                Vector3 center{Vector3{0, 0, 0}};

                /** @brief The hull's exact volume, or zero for a flat or degenerate piece. */
                Scalar volume = 0;

                /**
                 * @brief How far this piece protrudes outside the source mesh, in local units.
                 *
                 * Sampled on the piece's own hull surface, so it is a lower bound in the
                 * same way @ref Geometry::max_protrusion_distance is.
                 */
                float concavity = 0.0f;
            };

            /** @brief What the decomposition is allowed to spend. */
            struct ConvexDecompositionOptions
            {
                /** @brief Pieces the decomposition may produce; at least one. */
                std::int32_t max_pieces = 16;

                /** @brief Vertices one piece may keep; at least four. */
                std::int32_t vertex_budget = 32;

                /**
                 * @brief Protrusion below which a part is accepted as convex enough.
                 *
                 * In local units, and compared against the same measure the report carries,
                 * so "stop splitting" and "this is the error" are one number rather than two
                 * that can disagree. Zero or less splits until the piece budget runs out.
                 */
                float concavity_tolerance = 0.01f;

                /** @brief Sampling lattice order for the protrusion measure. */
                std::uint32_t accuracy_lattice_order = 3;
            };

            /** @brief What a decomposition produced, for the cook report. */
            struct ConvexDecompositionReport
            {
                /** @brief Pieces produced. */
                std::uint32_t piece_count = 0;

                /** @brief Vertices in the largest piece — the narrowphase's per-pair cost. */
                std::uint32_t largest_piece_vertex_count = 0;

                /** @brief The worst piece's protrusion, in local units; §7.6's number. */
                float worst_concavity = 0.0f;

                /**
                 * @brief The pieces' volumes, summed.
                 *
                 * Summed and not united, so a region two pieces both cover is counted
                 * twice. The estimate therefore reports **more** collider than there is,
                 * which is the safe direction for a threshold and the reason it is not
                 * presented as the union volume.
                 */
                Scalar summed_volume = 0;

                /** @brief Whether the decomposition stopped because the piece budget ran out. */
                bool exhausted_budget = false;
            };

            /**
             * @brief The exact convex hull of a small point set, as a closed triangle mesh.
             *
             * Brute force over every triple, deliberately. The vertex budget bounds the
             * input at a few dozen points, where enumerating triples costs less than an
             * incremental hull *and* is short enough to be read and believed — and a hull
             * routine that is subtly wrong produces a collider that is subtly wrong, which
             * is the least findable class of bug this pipeline can ship.
             *
             * Coplanar points are the case that makes a naive triple enumeration wrong: the
             * four corners of a square face all satisfy every one of their own triples, so
             * emitting a triangle per accepted triple covers that face twice and doubles its
             * contribution to the volume. So triples are used to find the hull's *planes*,
             * deduplicated, and each plane is then tiled once by fanning its own points in
             * angular order.
             *
             * @param points The point set, in any frame; the output is in the same one.
             * @param out    Receives the closed, outward-wound hull; cleared first.
             * @return False when the points span no volume — fewer than four distinct
             *         points, or all of them coplanar. @p out is then empty, and the caller
             *         must treat the piece as having no volume rather than no shape: a flat
             *         point set is still a perfectly good support function.
             */
            bool build_convex_hull_mesh(const std::vector<Vector3>& points,
                                        Geometry::TriangleMesh& out);

            /**
             * @brief Decomposes a mesh into at most @p options.max_pieces convex pieces.
             *
             * Recursive bisection driven by the measured protrusion: the part that departs
             * furthest from the source surface is split by whichever of three axis-aligned
             * planes through its centroid leaves the better pair of children, and the loop
             * stops when every part is inside the tolerance or the piece budget is spent.
             * Triangles are assigned to a child by which side their centroid falls on, so
             * the parts partition the triangles exactly and the pieces overlap slightly —
             * which is correct behaviour for a collider and not a defect.
             *
             * Deterministic: the split axis, the tie-breaking and the vertex selection are
             * all functions of vertex numbering rather than of any container's order, so a
             * re-cook of an unchanged mesh produces byte-identical pieces. The cache in
             * §8.1 would otherwise be keyed on a hash of an input that maps to two outputs.
             *
             * @param mesh    The repaired source geometry.
             * @param surface A distance hierarchy already built over @p mesh; the protrusion
             *                measure is taken against it.
             * @param options What the decomposition may spend.
             * @param out     Receives the pieces; cleared first.
             * @return The report; @c piece_count zero when @p mesh held no usable triangle.
             */
            ConvexDecompositionReport decompose_convex(const Geometry::TriangleMeshView& mesh,
                                                       const Geometry::MeshDistanceQuery& surface,
                                                       const ConvexDecompositionOptions& options,
                                                       std::vector<ConvexPiece>& out);
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
