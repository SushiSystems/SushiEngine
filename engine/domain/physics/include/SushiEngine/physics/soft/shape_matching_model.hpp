/**************************************************************************/
/* shape_matching_model.hpp                                               */
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
 * @file shape_matching_model.hpp
 * @brief Müller et al.'s shape matching: §3.3's third `ISoftBodyModel`, and
 *        §9.7's coarsest deformable tier.
 *
 * There is no topology here at all — no elements, no springs, no neighbour list.
 * Every substep the body asks one question: *if my rest shape were rotated and
 * translated to best fit where my particles are now, where would each particle
 * be?* Each particle is then pulled toward that goal. The best fit is the
 * rigid transform minimising the summed squared distance, and its rotation is
 * the orthogonal factor of the polar decomposition of the cross-covariance
 * between rest offsets and current offsets.
 *
 * **Why this is the right coarsest tier.** Its cost is two passes over the
 * particles and one 3x3 decomposition, independent of how the body is connected,
 * and it cannot become unstable however hard it is hit: the goal shape is the
 * rest shape by construction, so the worst it can do is snap back too fast. A
 * spring lattice under the same abuse tangles, and a tetrahedral body inverts.
 * What it gives up is everything local — it has no notion of one end of a body
 * deforming differently from the other, which is exactly the fidelity a distant
 * body does not need.
 *
 * **The fit is unweighted.** Every particle counts once rather than by mass. A
 * pinned particle (`inv_mass == 0`) therefore anchors the fit like any other and
 * is simply never moved by the correction, which is the behaviour a pinned
 * shape-matched body should have; weighting by mass would need a pinned
 * particle's weight to be infinite, and there is no honest finite number for it.
 *
 * **The stiffness is a rate, not a fraction.** The textbook formulation moves
 * each particle a fixed fraction of the way to its goal per iteration, which
 * makes the body stiffer the more substeps it is given — the step-size
 * dependence §0.2 exists to reject. Written as an exponential relaxation
 * (`1 - exp(-rate * h)`) the same authored number means the same recovery in the
 * same wall-clock time whatever the substep count, which is the property every
 * other constraint in this engine already has.
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/soft_body_model.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The orthogonal factor of a 3x3 matrix's polar decomposition.
         *
         * Higham's iteration, `R <- (R + R^-T) / 2`, which converges quadratically
         * to the nearest rotation and needs no eigen-solve. Preferred over building
         * `sqrt(A^T A)` because that squares the condition number of a matrix this
         * routine is specifically asked about when it is nearly singular — a body
         * squashed flat is exactly when a coarse tier must not produce a garbage
         * rotation.
         *
         * @param a   The matrix to decompose.
         * @param out Receives the rotation; untouched when @p a cannot be decomposed.
         * @return False when @p a is singular or inverted (`det <= 0`), where no
         *         rotation is the right answer and the caller should skip the substep.
         */
        template <typename T>
        inline bool polar_rotation(const FEMMatrix3<T>& a, FEMMatrix3<T>& out) noexcept
        {
            if (!(determinant(a) > T(0)))
                return false;

            FEMMatrix3<T> r = a;
            // Quadratic convergence reaches machine precision in well under a dozen
            // steps; the cap is a guard against a pathological input, not a budget.
            for (int iteration = 0; iteration < 24; ++iteration)
            {
                FEMMatrix3<T> inverse;
                if (!invert_fem_matrix3(r, inverse))
                    return false;
                const FEMMatrix3<T> inverse_transpose = transpose(inverse);

                FEMMatrix3<T> next;
                next.column0 = (r.column0 + inverse_transpose.column0) * T(0.5);
                next.column1 = (r.column1 + inverse_transpose.column1) * T(0.5);
                next.column2 = (r.column2 + inverse_transpose.column2) * T(0.5);

                const Vector3T<T> delta0 = next.column0 - r.column0;
                const Vector3T<T> delta1 = next.column1 - r.column1;
                const Vector3T<T> delta2 = next.column2 - r.column2;
                r = next;
                if (dot(delta0, delta0) + dot(delta1, delta1) + dot(delta2, delta2) <
                    T(1e-24))
                    break;
            }

            out = r;
            return true;
        }

        /**
         * @brief A soft body that keeps its silhouette and nothing else.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class ShapeMatchingModel : public SoftBodyBase<T>
        {
            public:
                using SoftBodyBase<T>::particles;

                /**
                 * @brief Where each particle belongs, in the body's own frame.
                 *
                 * Parallel to @ref particles. The body's *shape*, not its pose: the
                 * fit recovers the pose every substep, so these never have to be
                 * updated as the body moves or spins.
                 */
                std::vector<Vector3T<T>> rest_positions;

                /**
                 * @brief How fast a displaced particle returns to its goal, per second.
                 *
                 * A relaxation rate: over a substep of `h` seconds the particle closes
                 * `1 - exp(-rate * h)` of the gap. Large values approach a rigid body,
                 * small ones a slow-settling blob. Zero leaves the body in free fall.
                 */
                T recovery_rate = T(60);

                /** @brief How fast the body bleeds off velocity, per second. */
                T damping = 0;

                /**
                 * @brief Copies the current particle positions in as the rest shape.
                 *
                 * Called once after the particles are placed. Separate from
                 * construction because the fine-to-coarse transfer of §9.7 fills the
                 * particles first and then declares the shape they are resting in.
                 */
                void capture_rest_shape()
                {
                    rest_positions.resize(particles.size());
                    for (std::size_t i = 0; i < particles.size(); ++i)
                        rest_positions[i] = particles[i].position;
                }

                /**
                 * @brief Fits the rest shape to the current particles and pulls them toward it.
                 * @param h The substep duration, in seconds.
                 */
                void project_constraints(T h) noexcept override
                {
                    const std::size_t count = particles.size();
                    if (count == 0 || rest_positions.size() != count || !(recovery_rate > T(0)))
                        return;

                    const T inverse_count = T(1) / T(count);
                    Vector3T<T> rest_centroid{T(0), T(0), T(0)};
                    Vector3T<T> current_centroid{T(0), T(0), T(0)};
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        rest_centroid = rest_centroid + rest_positions[i];
                        current_centroid = current_centroid + particles[i].position;
                    }
                    rest_centroid = rest_centroid * inverse_count;
                    current_centroid = current_centroid * inverse_count;

                    // The cross-covariance `sum (x_i - c) (r_i - c0)^T`, accumulated as
                    // three columns because that is the shape every 3x3 in this
                    // directory is stored in.
                    FEMMatrix3<T> covariance;
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const Vector3T<T> current = particles[i].position - current_centroid;
                        const Vector3T<T> rest = rest_positions[i] - rest_centroid;
                        covariance.column0 = covariance.column0 + current * rest.x;
                        covariance.column1 = covariance.column1 + current * rest.y;
                        covariance.column2 = covariance.column2 + current * rest.z;
                    }

                    FEMMatrix3<T> rotation;
                    if (!polar_rotation(covariance, rotation))
                        return; // collapsed or turned inside out: no fit to speak of

                    const T blend = T(1) - std::exp(-recovery_rate * h);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        RigidBodyT<T>& particle = particles[i];
                        if (!(particle.inv_mass > T(0)))
                            continue;
                        const Vector3T<T> rest = rest_positions[i] - rest_centroid;
                        const Vector3T<T> goal = current_centroid + rotation.column0 * rest.x +
                                                 rotation.column1 * rest.y +
                                                 rotation.column2 * rest.z;
                        particle.position =
                            particle.position + (goal - particle.position) * blend;
                    }
                }

                /** @brief How fast the body bleeds off velocity, per second. */
                T damping_rate() const noexcept override { return damping; }
        };
    } // namespace Physics
} // namespace SushiEngine
