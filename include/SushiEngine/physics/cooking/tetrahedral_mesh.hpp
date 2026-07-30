/**************************************************************************/
/* tetrahedral_mesh.hpp                                                   */
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
 * @file tetrahedral_mesh.hpp
 * @brief Filling a surface with tetrahedra — §8.3 stages 2 through 7.
 *
 * The soft-body cooker's geometry engine, separated from the cooker itself because it is
 * the half with the algorithms in it and the half worth testing against a shape whose
 * answer is known.
 *
 * **Voxelize before tetrahedralize, and the choice is deliberate** (§8.3 stage 2).
 * Constrained Delaunay tetrahedralization is more accurate on clean input and *fails* on
 * dirty input, and dirty input is the common case — self-intersecting, non-manifold,
 * open-shelled meshes are what real projects contain. Rasterizing the surface into a grid
 * and flood-filling the exterior from a corner has no failure mode that produces zero
 * output: whatever is not reached from outside is inside, whatever the surface did.
 *
 * **Two deviations from §8.3 that are named rather than glossed.**
 *
 * 1. The lattice is **Kuhn's** six-tetrahedra-per-cell triangulation, not the
 *    body-centred cubic one §8.3 names. Kuhn conforms across cell faces *by
 *    construction* — the diagonal a shared face is split along is the same computed from
 *    either side, so there is no parity bookkeeping to get wrong — and every element in
 *    the lattice is congruent, which means the worst element quality is a lattice
 *    constant rather than a property of the input. Body-centred cubic gives a better
 *    constant; it is a swap inside @ref build_tetrahedral_mesh and nothing above it.
 * 2. Boundary conforming is done by **snapping** lattice vertices onto the surface, not
 *    by splitting straddling cells with marching tetrahedra. Snapping cannot add a
 *    vertex, so a feature thinner than a cell is lost rather than resolved — which the
 *    voxel resolution controls and the cook report's accuracy number measures. Marching
 *    tetrahedra is what would resolve it and is not written.
 */

#include <cstddef>
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
             * @brief A tetrahedral simulation mesh and everything derived from its rest state.
             *
             * One structure rather than several, because every field is a function of the
             * same lattice and letting them travel separately is how a rest volume ends up
             * describing a tetrahedron that was optimized away.
             */
            struct TetrahedralMesh
            {
                /** @brief Vertex positions in the source mesh's frame. */
                std::vector<Vector3> vertices;

                /** @brief Four vertex indices per tetrahedron, wound to a positive volume. */
                std::vector<std::uint32_t> tetrahedra;

                /**
                 * @brief The inverse rest-shape matrix, three rows per tetrahedron.
                 *
                 * `Dm` is the matrix whose columns are the three edges from the first
                 * vertex; its inverse is what turns a deformed shape into a deformation
                 * gradient, which is the whole of §9.1's constitutive model. Cooked here
                 * because it is a per-element constant and inverting it per substep would
                 * be paying for the rest state every tick.
                 */
                std::vector<Vector3> rest_inverse;

                /** @brief Rest volume per tetrahedron, in cubic local units. */
                std::vector<Scalar> rest_volume;

                /** @brief Mass per vertex, summed from the incident tetrahedra. */
                std::vector<Scalar> vertex_mass;

                /** @brief Three indices per boundary triangle, wound outward. */
                std::vector<std::uint32_t> surface_indices;

                /** @brief Which grid cell each tetrahedron came from; the embedding's index. */
                std::vector<std::uint32_t> tetrahedron_cell;

                /** @brief Grid cells per axis. */
                std::int32_t grid[3] = {0, 0, 0};

                /** @brief Edge length of one grid cell. */
                Scalar cell_size = 0;

                /** @brief World position of grid corner (0, 0, 0). */
                Vector3 grid_origin{Vector3{0, 0, 0}};

                /** @brief Number of vertices. */
                std::size_t vertex_count() const noexcept { return vertices.size(); }

                /** @brief Number of tetrahedra. */
                std::size_t tetrahedron_count() const noexcept { return tetrahedra.size() / 4; }

                /** @brief Number of boundary triangles. */
                std::size_t surface_triangle_count() const noexcept
                {
                    return surface_indices.size() / 3;
                }

                /** @brief The boundary surface as a triangle mesh, for a hierarchy or a bake. */
                Geometry::TriangleMesh surface_mesh() const;
            };

            /** @brief What the tetrahedralization is allowed to spend and accept. */
            struct TetrahedralizationOptions
            {
                /** @brief Grid cells along the longest axis; at least two. */
                std::int32_t voxel_resolution = 32;

                /**
                 * @brief Target tetrahedron count; zero leaves the resolution alone.
                 *
                 * The dial authors a *count*, and a count is not a resolution — so the
                 * resolution is scaled toward it by the cube root of the ratio, once, and
                 * whatever comes out is reported. Iterating to hit the number exactly would
                 * mean voxelizing repeatedly to satisfy a figure the artist chose as an
                 * order of magnitude.
                 */
                std::int32_t target_tetrahedron_count = 0;

                /** @brief Surface-snapping and smoothing passes (§8.3 stages 3 and 4). */
                std::int32_t conforming_passes = 1;

                /**
                 * @brief Quality below which an element is removed, in [0, 1].
                 *
                 * One is a regular tetrahedron. Slivers destroy the conditioning of a
                 * finite-element solve, which is what this threshold exists for.
                 */
                float min_element_quality = 0.01f;

                /** @brief Mass per unit volume, for the per-vertex masses. */
                float density = 1000.0f;
            };

            /** @brief What the tetrahedralization produced, for the cook report. */
            struct TetrahedralizationReport
            {
                /** @brief Grid cells the flood fill decided were inside the surface. */
                std::uint32_t interior_cell_count = 0;

                /** @brief Grid cells the surface passed through. */
                std::uint32_t surface_cell_count = 0;

                std::uint32_t vertex_count = 0;
                std::uint32_t tetrahedron_count = 0;

                /** @brief Elements removed for being slivers or having no volume. */
                std::uint32_t removed_tetrahedron_count = 0;

                /**
                 * @brief Elements left with a negative volume after conforming; must be zero.
                 *
                 * Non-zero means a snap or a smoothing step turned an element inside out and
                 * the guard that reverts such a move failed to. One inverted element is a
                 * solve that diverges, so this is a hard threshold rather than a metric.
                 */
                std::uint32_t inverted_element_count = 0;

                /** @brief Boundary vertices moved onto the surface. */
                std::uint32_t snapped_vertex_count = 0;

                /** @brief The worst element's quality, in [0, 1]. */
                float worst_element_quality = 0.0f;

                /** @brief Summed rest volume, for comparison against the source mesh's. */
                Scalar total_volume = 0;

                /** @brief The grid cell size the resolution resolved to. */
                Scalar cell_size = 0;

                /**
                 * @brief The resolution actually used, after the element-count target moved it.
                 *
                 * Reported because §8.2 says the inspector shows what the dial *produced*, and
                 * because the level-of-detail chain has to halve this rather than the authored
                 * number — halving the authored one while the target scaled the finest level
                 * down makes the second level finer than the first.
                 */
                std::int32_t resolution = 0;
            };

            /**
             * @brief One render vertex's binding to the simulation mesh (§8.3 stage 6).
             *
             * This record *is* §0.4's guarantee: the render mesh is driven by the simulated
             * one rather than living beside it.
             */
            struct TetrahedronBinding
            {
                /** @brief The tetrahedron whose vertices drive this point. */
                std::uint32_t tetrahedron = 0;

                /**
                 * @brief Barycentric weights, in the tetrahedron's vertex order.
                 *
                 * Summing to one always; **not** clamped to [0, 1]. A point outside every
                 * element binds by extrapolated coordinates, which is what keeps a thin
                 * feature attached and moving correctly instead of unbound (§8.3 stage 6).
                 */
                float weights[4] = {0.25f, 0.25f, 0.25f, 0.25f};

                /** @brief Whether the point was inside the element rather than extrapolated. */
                bool inside = false;
            };

            /**
             * @brief The normalized quality of a tetrahedron, in [0, 1].
             *
             * `6 sqrt(2) V / L_rms^3`, which is exactly one for a regular tetrahedron and
             * falls toward zero as an element flattens. Chosen over a dihedral-angle
             * minimum because it is one expression rather than six, and because it responds
             * to *all* the ways an element degenerates rather than only to a pinched angle.
             *
             * @param a First vertex.
             * @param b Second vertex.
             * @param c Third vertex.
             * @param d Fourth vertex.
             * @return The quality; zero for a flat or collapsed element.
             */
            float tetrahedron_quality(const Vector3& a, const Vector3& b, const Vector3& c,
                                      const Vector3& d) noexcept;

            /**
             * @brief Voxelizes, tetrahedralizes, conforms, optimizes, and derives rest state.
             *
             * §8.3 stages 2 through 7, in one call because they share the grid and splitting
             * them across the `ICookingStage` seam would mean publishing that grid as an
             * interface. The stage the cooker exposes is this whole step.
             *
             * @param mesh    The repaired source geometry.
             * @param surface A distance hierarchy over @p mesh, for the snap and the fill.
             * @param options What to spend and what to accept.
             * @param out     Receives the mesh; cleared first.
             * @return The report; @c tetrahedron_count zero when nothing could be filled,
             *         which happens for a surface enclosing less than one grid cell.
             */
            TetrahedralizationReport build_tetrahedral_mesh(
                const Geometry::TriangleMeshView& mesh,
                const Geometry::MeshDistanceQuery& surface,
                const TetrahedralizationOptions& options, TetrahedralMesh& out);

            /**
             * @brief Binds arbitrary points to the tetrahedra containing them (§8.3 stage 6).
             *
             * Searched through the grid rather than over every element: a point's cell is
             * arithmetic, and the elements that could contain it are that cell's and its
             * twenty-six neighbours'. A point no element contains widens the search and then
             * binds to the nearest element by *extrapolated* coordinates, so the returned
             * count of unbound points is zero for any mesh with a lattice anywhere near it —
             * which is what lets §8.5's threshold for it be zero.
             *
             * Used twice: for the render mesh, and for each level-of-detail lattice's
             * vertices against the next finer level, which is the same question asked of
             * different points (§8.3 stage 9).
             *
             * @param mesh     The simulation mesh to bind into.
             * @param points   The points to bind.
             * @param count    How many.
             * @param out      Receives one binding per point; cleared first.
             * @return The number of points that fell outside every element and were bound by
             *         extrapolation. Zero means every point sits inside one.
             */
            std::uint32_t embed_points(const TetrahedralMesh& mesh, const Vector3* points,
                                       std::size_t count,
                                       std::vector<TetrahedronBinding>& out);

            /**
             * @brief Reconstructs a bound point from the mesh's current vertex positions.
             *
             * The per-tick kernel, in scalar form: `sum(weight[i] * vertex[i])`. Here so the
             * cook can assert §8.6 invariant 2 — that at rest the reconstruction reproduces
             * the source vertex — against the same expression the runtime will use, rather
             * than against a second copy of it that can drift.
             *
             * @param mesh    The simulation mesh.
             * @param binding The point's binding.
             * @return The reconstructed position; the mesh's origin for an invalid binding.
             */
            Vector3 evaluate_binding(const TetrahedralMesh& mesh,
                                     const TetrahedronBinding& binding) noexcept;
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
