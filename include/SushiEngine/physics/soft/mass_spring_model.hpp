/**************************************************************************/
/* mass_spring_model.hpp                                                  */
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
 * @file mass_spring_model.hpp
 * @brief A soft body held together by distance constraints: §3.3's second
 *        `ISoftBodyModel`, and §9.7's middle tier.
 *
 * `soft/soft_body.hpp` already builds a mass-spring lattice, but it builds it
 * *into a `PhysicsWorld`* — the particles become world bodies and the springs
 * become world constraints, which is the right shape for a demo and the wrong
 * one for a level-of-detail tier: a body that changes tier would have to add and
 * remove rows from the world's constraint store mid-frame, and the coarse tier
 * would be reached through a different type from the fine one. This class is the
 * same physics owning its own arrays, so a tier swap is a pointer assignment.
 *
 * The arithmetic is **not** re-derived here. A spring is an
 * `XpbdDistanceConstraintT` and its projection is `XpbdDistanceProjectionT`, the
 * same descriptor and the same functor the rigid-body solver, the host mirror and
 * the device graph all run — which is what stops this tier from developing its
 * own private idea of what a compliance means. All this class adds is the array
 * of accumulated multipliers and the sweep order.
 *
 * **Why a coarse tier is springs and not tetrahedra.** A spring lattice has no
 * constitutive law, no stress and no volume preservation, so it is wrong for a
 * body under load — and costs a fraction of the FEM body for a shape that only
 * has to look right. That is precisely the trade §9.7 makes as a body recedes,
 * and stating it as a separate class rather than as a flag on the FEM body keeps
 * the FEM body free of a mode it would otherwise have to check for.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/bending_constraint.hpp>
#include <SushiEngine/physics/constraints/distance_projection.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A soft body whose internal constraint is the distance constraint.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class MassSpringModel : public SoftBodyBase<T>
        {
            public:
                using SoftBodyBase<T>::particles;

                /**
                 * @brief The springs, swept in this order every substep.
                 *
                 * Index order is the sweep order, and the sweep order is what a
                 * Gauss-Seidel result depends on — so it is a property of the body's
                 * construction and never of a container's iteration (§0.5).
                 */
                std::vector<XpbdDistanceConstraintT<T>> springs;

                /**
                 * @brief The bending stencils, swept after the springs (§9.1).
                 *
                 * Empty for a body that does not bend, which is not a special case
                 * but the whole of what "bending stiffness zero" means here: a
                 * cloth authored without bending gets no bending constraints at
                 * all, so its solve is the identical sequence of identical
                 * projections it was before bending existed. That is a stronger
                 * guarantee than a zero coefficient, which would still cost a pass
                 * and could still perturb a sum.
                 */
                std::vector<XpbdBendingConstraintT<T>> bending;

                /** @brief How fast the body bleeds off velocity, per second. */
                T damping = 0;

                /**
                 * @brief Adds a spring between two particles at their current separation.
                 *
                 * Rest length is measured rather than passed, because a lattice's rest
                 * length *is* the distance its particles were built at, and a caller
                 * recomputing it is a caller that can get it wrong for the diagonals.
                 *
                 * @param a          First particle index.
                 * @param b          Second particle index.
                 * @param compliance XPBD compliance; zero is a rigid link.
                 */
                void add_spring(std::uint32_t a, std::uint32_t b, T compliance)
                {
                    if (a >= particles.size() || b >= particles.size() || a == b)
                        return;
                    XpbdDistanceConstraintT<T> spring;
                    spring.a = a;
                    spring.b = b;
                    spring.local_anchor_a = Vector3T<T>{T(0), T(0), T(0)};
                    spring.local_anchor_b = Vector3T<T>{T(0), T(0), T(0)};
                    spring.rest_length =
                        length(particles[b].position - particles[a].position);
                    spring.compliance = compliance;
                    springs.push_back(spring);
                    lambda_.push_back(T(0));
                }

                /**
                 * @brief One Gauss-Seidel sweep of every spring, in index order.
                 *
                 * The multipliers are cleared at the top of the sweep rather than once
                 * per tick, matching what the FEM body does with its elements: an
                 * internal constraint's multiplier is a per-substep quantity, and one
                 * that compounded across substeps would make the body stiffen as the
                 * substep count rose — the exact step-size dependence XPBD exists to
                 * remove.
                 *
                 * @param h The substep duration, in seconds.
                 */
                void project_constraints(T h) noexcept override
                {
                    lambda_.assign(springs.size(), T(0));
                    const XpbdDistanceProjectionT<T> projection;
                    for (std::size_t i = 0; i < springs.size(); ++i)
                        projection(springs[i], particles.data(), lambda_[i], h);

                    // After the springs, because bending is the weaker effect and a
                    // stencil projected against a stretched sheet would be answering
                    // a shape the springs are about to undo.
                    if (bending.empty())
                        return;
                    bending_lambda_.assign(bending.size(), T(0));
                    const XpbdBendingProjectionT<T> bend;
                    for (std::size_t i = 0; i < bending.size(); ++i)
                        bend(bending[i], particles.data(), bending_lambda_[i], h);
                }

                /** @brief How fast the body bleeds off velocity, per second. */
                T damping_rate() const noexcept override { return damping; }

                /**
                 * @brief Adds a bending stencil across a shared edge.
                 *
                 * @param edge_a,edge_b     The shared edge's particles.
                 * @param opposite_a,opposite_b The two vertices opposite it.
                 * @param compliance        XPBD compliance; zero is a rigid hinge.
                 * @return False for a degenerate stencil, which is not added.
                 */
                bool add_bending(std::uint32_t edge_a, std::uint32_t edge_b,
                                 std::uint32_t opposite_a, std::uint32_t opposite_b, T compliance)
                {
                    const std::uint32_t index[4] = {edge_a, edge_b, opposite_a, opposite_b};
                    for (int i = 0; i < 4; ++i)
                        if (index[i] >= particles.size())
                            return false;

                    XpbdBendingConstraintT<T> constraint;
                    if (!build_bending_constraint(particles[edge_a].position,
                                                  particles[edge_b].position,
                                                  particles[opposite_a].position,
                                                  particles[opposite_b].position, constraint))
                        return false;
                    for (int i = 0; i < 4; ++i)
                        constraint.particle[i] = index[i];
                    constraint.compliance = compliance;
                    bending.push_back(constraint);
                    return true;
                }

            private:
                /** @brief One accumulated Lagrange multiplier per spring. */
                std::vector<T> lambda_;

                /** @brief One accumulated Lagrange multiplier per bending stencil. */
                std::vector<T> bending_lambda_;
        };

        /**
         * @brief Links every distinct edge of a tetrahedral mesh, once each.
         *
         * The coarse tier is built from the same cooked lattice the fine tier
         * simulates — that is what makes a level-of-detail swap a swap rather than a
         * second asset — so the topology this takes is a tetrahedron list, not a
         * grid. A tetrahedron's six edges span it completely, so an edge lattice
         * resists stretch and shear everywhere the elements did; what it stops
         * resisting is volume, which is the fidelity the tier is trading away.
         *
         * Edges are deduplicated and added in ascending `(min, max)` order rather
         * than in the order elements happen to name them. A tetrahedral mesh shares
         * most of its edges between four to six elements, and a spring added once
         * per element that touches it would be that many times too stiff — the same
         * failure §9.6's contact set is deduplicated to avoid. The ascending order
         * then makes the Gauss-Seidel sweep a function of the topology alone (§0.5).
         *
         * @param model       The model to add springs to; its particles set rest lengths.
         * @param tetrahedra  Four particle indices per element.
         * @param count       How many elements @p tetrahedra holds.
         * @param compliance  XPBD compliance of every spring.
         */
        template <typename T>
        inline void link_tetrahedron_edges(MassSpringModel<T>& model,
                                           const std::uint32_t* tetrahedra, std::size_t count,
                                           T compliance)
        {
            std::vector<std::pair<std::uint32_t, std::uint32_t>> edges;
            edges.reserve(count * 6);
            for (std::size_t t = 0; t < count; ++t)
            {
                const std::uint32_t* corner = tetrahedra + t * 4;
                for (int i = 0; i < 4; ++i)
                    for (int j = i + 1; j < 4; ++j)
                    {
                        const std::uint32_t a = corner[i] < corner[j] ? corner[i] : corner[j];
                        const std::uint32_t b = corner[i] < corner[j] ? corner[j] : corner[i];
                        edges.emplace_back(a, b);
                    }
            }

            std::sort(edges.begin(), edges.end());
            edges.erase(std::unique(edges.begin(), edges.end()), edges.end());
            for (const std::pair<std::uint32_t, std::uint32_t>& edge : edges)
                model.add_spring(edge.first, edge.second, compliance);
        }

        /**
         * @brief Wires a row-major cloth grid: structural, shear, and bending (§9.1).
         *
         * The distance topology is exactly `soft/cloth.hpp`'s — a link to the right
         * and below, and both diagonals of each quad — so a grid built here and one
         * built there settle identically when @p bending_stiffness is zero.
         *
         * **Bending stencils are the triangulation's interior edges.** The grid is
         * drawn as two triangles per quad, `(v00, v10, v01)` and `(v01, v10, v11)`
         * (the winding `Render::build_grid_indices` and the deformable shading pass
         * both use), and every edge shared by two of those triangles is a hinge. Per
         * quad that is its own diagonal, plus its top edge where there is a quad
         * above and its left edge where there is one to the left — which reaches
         * each interior edge exactly once, from the quad below-right of it. Counting
         * them from both sides instead would make every hinge twice as stiff as
         * authored, the same doubling §9.6's contact set is deduplicated to avoid.
         *
         * **Zero @p bending_stiffness adds no constraints at all.** Not a zero
         * coefficient on a constraint that still runs: the sweep is then the same
         * sequence of the same projections in the same order it was before this
         * function existed, so "bending off" reproduces the old behaviour by
         * construction rather than by the arithmetic happening to cancel.
         *
         * @param model                The model; its particles must already be placed
         *                             row-major, `row * cols + col`.
         * @param rows                 Grid rows (>= 1).
         * @param cols                 Grid columns (>= 1).
         * @param compliance           XPBD compliance of the distance constraints.
         * @param bending_stiffness    Resistance to folding; zero disables bending
         *                             entirely, larger is stiffer.
         */
        template <typename T>
        inline void link_cloth_grid(MassSpringModel<T>& model, std::size_t rows, std::size_t cols,
                                    T compliance, T bending_stiffness)
        {
            const auto at = [cols](std::size_t row, std::size_t col)
            { return std::uint32_t(row * cols + col); };

            for (std::size_t row = 0; row < rows; ++row)
                for (std::size_t col = 0; col < cols; ++col)
                {
                    if (col + 1 < cols)
                        model.add_spring(at(row, col), at(row, col + 1), compliance);
                    if (row + 1 < rows)
                        model.add_spring(at(row, col), at(row + 1, col), compliance);
                    if (row + 1 < rows && col + 1 < cols)
                    {
                        model.add_spring(at(row, col), at(row + 1, col + 1), compliance);
                        model.add_spring(at(row, col + 1), at(row + 1, col), compliance);
                    }
                }

            if (!(bending_stiffness > T(0)) || rows < 2 || cols < 2)
                return;
            const T bending_compliance = T(1) / bending_stiffness;

            for (std::size_t row = 0; row + 1 < rows; ++row)
                for (std::size_t col = 0; col + 1 < cols; ++col)
                {
                    const std::uint32_t v00 = at(row, col);
                    const std::uint32_t v01 = at(row, col + 1);
                    const std::uint32_t v10 = at(row + 1, col);
                    const std::uint32_t v11 = at(row + 1, col + 1);

                    // The quad's own diagonal, shared by its two triangles.
                    model.add_bending(v10, v01, v00, v11, bending_compliance);
                    // Its top edge, shared with the quad above.
                    if (row > 0)
                        model.add_bending(v00, v01, v10, at(row - 1, col + 1), bending_compliance);
                    // Its left edge, shared with the quad to the left.
                    if (col > 0)
                        model.add_bending(v00, v10, v01, at(row + 1, col - 1), bending_compliance);
                }
        }
    } // namespace Physics
} // namespace SushiEngine
