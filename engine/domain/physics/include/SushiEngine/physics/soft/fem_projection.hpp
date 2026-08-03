/**************************************************************************/
/* fem_projection.hpp                                                    */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file fem_projection.hpp
 * @brief §9.1's two constraints: the deformation gradient, their values and
 *        gradients, and the XPBD position update they drive.
 *
 * Macklin & Müller 2021, "A Constraint-based Formulation of Stable Neo-Hookean
 * Materials". Two constraints per tetrahedron, replacing today's distance
 * lattice:
 *
 * - **Deviatoric** (resists shape change): `C = ||F||_Frobenius` — the norm
 *   itself, not `||F|| - sqrt(3)`; see `evaluate_deviatoric_constraint` for why
 *   the rest offset would break the material.
 * - **Hydrostatic** (resists volume change): `C = det(F) - 1 - mu/lambda`.
 *
 * where `F = Ds * Dm^-1` is the deformation gradient: `Ds`'s columns are the
 * tetrahedron's three *current* edge vectors from its first vertex, `Dm^-1`'s
 * columns are the cooked rest-state inverse (`fem_element.hpp`). Both
 * constraints and their gradients fall out of `F` alone, which is the whole
 * appeal of this formulation: no per-material-model branch, no separate
 * strain-energy derivative to keep in sync with the constraint it corrects.
 *
 * **Why this doesn't need a general 3x3 matrix type.** Every quantity here is
 * either a `Vector3T<T>` (one column of `F`, or one gradient) or built from
 * three of them, so the whole file is written the way `geometry/shapes.hpp`
 * and `collision/manifold.hpp` already are — components and cross/dot
 * products — rather than introducing a matrix library for the one file that
 * would use it.
 *
 * **The gradient derivation, worth recording because a sign or an index slip
 * here is invisible until a cantilever bends the wrong amount.** `F`'s j-th
 * column is `Ds * (Dm^-1's j-th column)`, i.e. `e1*c_j.x + e2*c_j.y + e3*c_j.z`
 * where `e1/e2/e3` are the current edges and `c_j` is `Dm^-1`'s j-th column.
 * Differentiating a scalar function of `F` with respect to vertex `k` (`k` in
 * 1..3) therefore always reduces to a sum over `F`'s columns weighted by that
 * column's `k`-th *row* component of `Dm^-1` — `c_0[k-1], c_1[k-1], c_2[k-1]`
 * — because vertex `k` only ever enters through edge `e_k`, linearly, with
 * that coefficient. Vertex 0's gradient is minus the sum of the other three,
 * since every edge is `x_k - x_0` and `C` cannot depend on a rigid
 * translation of the whole element. This one pattern produces both
 * constraints' gradients below; only the "outer" derivative changes
 * (`F/||F||` for the deviatoric term, the cross products that differentiate a
 * determinant for the hydrostatic one).
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief A 3x3 matrix as three columns — exactly `Dm^-1`'s own storage shape. */
        template <typename T>
        struct FEMMatrix3
        {
            Vector3T<T> column0{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> column1{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> column2{Vector3T<T>{T(0), T(0), T(0)}};
        };

        /**
         * @brief The deformation gradient `F = Ds * Dm^-1` for one tetrahedron.
         *
         * @param edge1 Current edge from vertex 0 to vertex 1.
         * @param edge2 Current edge from vertex 0 to vertex 2.
         * @param edge3 Current edge from vertex 0 to vertex 3.
         * @param rest_inverse_column0 `Dm^-1`'s column 0.
         * @param rest_inverse_column1 `Dm^-1`'s column 1.
         * @param rest_inverse_column2 `Dm^-1`'s column 2.
         */
        template <typename T>
        inline FEMMatrix3<T> tetrahedron_deformation_gradient(
            const Vector3T<T>& edge1, const Vector3T<T>& edge2, const Vector3T<T>& edge3,
            const Vector3T<T>& rest_inverse_column0, const Vector3T<T>& rest_inverse_column1,
            const Vector3T<T>& rest_inverse_column2) noexcept
        {
            FEMMatrix3<T> f;
            f.column0 = edge1 * rest_inverse_column0.x + edge2 * rest_inverse_column0.y +
                       edge3 * rest_inverse_column0.z;
            f.column1 = edge1 * rest_inverse_column1.x + edge2 * rest_inverse_column1.y +
                       edge3 * rest_inverse_column1.z;
            f.column2 = edge1 * rest_inverse_column2.x + edge2 * rest_inverse_column2.y +
                       edge3 * rest_inverse_column2.z;
            return f;
        }

        /** @brief The Frobenius norm of a deformation gradient. */
        template <typename T>
        inline T frobenius_norm(const FEMMatrix3<T>& f) noexcept
        {
            return std::sqrt(dot(f.column0, f.column0) + dot(f.column1, f.column1) +
                             dot(f.column2, f.column2));
        }

        /** @brief The determinant of a deformation gradient. */
        template <typename T>
        inline T determinant(const FEMMatrix3<T>& f) noexcept
        {
            return dot(f.column0, cross(f.column1, f.column2));
        }

        /**
         * @brief Inverts a 3x3 matrix given as three columns.
         *
         * The standard cofactor identity — for `M = [c0|c1|c2]`, `M^-1`'s *rows* are
         * `(c1 x c2)`, `(c2 x c0)`, `(c0 x c1)`, each divided by `det(M)` — assembled
         * back into columns here so the result is a `FEMMatrix3` in the same
         * column-major shape every other matrix in this file is. §9.1 never needs
         * this (the rest inverse arrives pre-cooked), but §9.4's plasticity does: it
         * has to invert the *current*, evolving shape matrix at runtime, which
         * nothing bakes in advance.
         *
         * @param m   The matrix to invert.
         * @param out Receives the inverse; untouched when @p m is singular.
         * @return False when @p m's determinant is too close to zero to invert
         *         safely — a degenerate (collapsed or inverted) element.
         */
        template <typename T>
        inline bool invert_fem_matrix3(const FEMMatrix3<T>& m, FEMMatrix3<T>& out) noexcept
        {
            const T det = determinant(m);
            if (!(det > T(1e-24)) && !(det < T(-1e-24)))
                return false;
            const T inverse_det = T(1) / det;
            const Vector3T<T> inverse_row0 = cross(m.column1, m.column2) * inverse_det;
            const Vector3T<T> inverse_row1 = cross(m.column2, m.column0) * inverse_det;
            const Vector3T<T> inverse_row2 = cross(m.column0, m.column1) * inverse_det;
            out.column0 = Vector3T<T>{inverse_row0.x, inverse_row1.x, inverse_row2.x};
            out.column1 = Vector3T<T>{inverse_row0.y, inverse_row1.y, inverse_row2.y};
            out.column2 = Vector3T<T>{inverse_row0.z, inverse_row1.z, inverse_row2.z};
            return true;
        }

        /**
         * @brief The transpose of a 3x3 matrix given as three columns.
         *
         * Column `i` of the result is row `i` of the input, which for this storage
         * shape is a gather of one component from each column. Needed by the polar
         * decomposition `shape_matching_model.hpp` runs — its Newton iteration is
         * written in terms of `M^-T` — and kept beside the inverse it composes with
         * rather than in the one file that uses it, so a second caller does not
         * arrive at a second spelling.
         *
         * @param m The matrix to transpose.
         * @return Its transpose.
         */
        template <typename T>
        inline FEMMatrix3<T> transpose(const FEMMatrix3<T>& m) noexcept
        {
            FEMMatrix3<T> out;
            out.column0 = Vector3T<T>{m.column0.x, m.column1.x, m.column2.x};
            out.column1 = Vector3T<T>{m.column0.y, m.column1.y, m.column2.y};
            out.column2 = Vector3T<T>{m.column0.z, m.column1.z, m.column2.z};
            return out;
        }

        /** @brief One constraint's value and the gradient it hands each of the tetrahedron's four vertices. */
        template <typename T>
        struct FEMConstraintEvaluation
        {
            T value = 0;
            /** @brief Gradient with respect to vertex 0, 1, 2, 3, in that order. */
            Vector3T<T> gradient[4] = {Vector3T<T>{T(0), T(0), T(0)}, Vector3T<T>{T(0), T(0), T(0)},
                                       Vector3T<T>{T(0), T(0), T(0)}, Vector3T<T>{T(0), T(0), T(0)}};
        };

        /**
         * @brief The deviatoric constraint: `C = ||F||_Frobenius`.
         *
         * The norm itself, *not* `||F|| - sqrt(3)`. With compliance `1/mu`, XPBD's
         * force is `-mu * C * dC/dx = -mu * F * Dm^-T` — exactly the first
         * Piola-Kirchhoff stress of the stable neo-Hookean deviatoric energy
         * `mu/2 * tr(F^T F)`, whose difference from `mu/2 (tr(F^T F) - 3)` is a
         * constant that no force ever sees. Subtracting `sqrt(3)` to make the
         * value read zero at rest looks tidier but scales every force by
         * `(||F|| - sqrt(3)) / ||F||` — vanishing at small strain, which made a
         * cantilever measure roughly 9x the Euler-Bernoulli deflection (§16.19),
         * and it un-balances the hydrostatic constraint's `mu/lambda` offset,
         * whose whole purpose is to cancel this term's rest-state pull
         * (`-mu * I` against `+mu * cof(F)`, Smith et al.'s `-mu (J - 1)` term
         * in constraint form). The value is `sqrt(3)` for any pure rotation
         * (`F` orthogonal), and the rest state is in equilibrium *because* the
         * two constraints fight to a standstill there, not because each is
         * separately zero.
         *
         * @param f The deformation gradient.
         * @param rest_inverse_column0/1/2 `Dm^-1`'s three columns (the gradient
         *        needs them again, alongside `f`).
         */
        template <typename T>
        inline FEMConstraintEvaluation<T> evaluate_deviatoric_constraint(
            const FEMMatrix3<T>& f, const Vector3T<T>& rest_inverse_column0,
            const Vector3T<T>& rest_inverse_column1,
            const Vector3T<T>& rest_inverse_column2) noexcept
        {
            FEMConstraintEvaluation<T> result;
            const T norm = frobenius_norm(f);
            result.value = norm;

            // A collapsed element (every current edge zero) has no defined
            // direction to correct along; leaving the gradient at zero is a
            // guard against dividing by zero, not a claim that nothing needs
            // fixing — a genuinely collapsed tetrahedron is a cooking or a
            // fracture-budget failure elsewhere, not something this constraint
            // alone can recover from.
            if (!(norm > T(1e-9)))
                return result;

            const T inverse_norm = T(1) / norm;
            const Vector3T<T>& c0 = rest_inverse_column0;
            const Vector3T<T>& c1 = rest_inverse_column1;
            const Vector3T<T>& c2 = rest_inverse_column2;
            result.gradient[1] =
                (f.column0 * c0.x + f.column1 * c1.x + f.column2 * c2.x) * inverse_norm;
            result.gradient[2] =
                (f.column0 * c0.y + f.column1 * c1.y + f.column2 * c2.y) * inverse_norm;
            result.gradient[3] =
                (f.column0 * c0.z + f.column1 * c1.z + f.column2 * c2.z) * inverse_norm;
            result.gradient[0] =
                (result.gradient[1] + result.gradient[2] + result.gradient[3]) * T(-1);
            return result;
        }

        /**
         * @brief The hydrostatic constraint: `C = det(F) - 1 - mu/lambda`.
         *
         * @param f The deformation gradient.
         * @param rest_inverse_column0/1/2 `Dm^-1`'s three columns.
         * @param mu_over_lambda The material's `mu/lambda` ratio (§9.1's offset,
         *        the "stable" neo-Hookean formulation's correction so the
         *        deviatoric and hydrostatic terms do not fight each other at rest).
         */
        template <typename T>
        inline FEMConstraintEvaluation<T> evaluate_hydrostatic_constraint(
            const FEMMatrix3<T>& f, const Vector3T<T>& rest_inverse_column0,
            const Vector3T<T>& rest_inverse_column1, const Vector3T<T>& rest_inverse_column2,
            T mu_over_lambda) noexcept
        {
            FEMConstraintEvaluation<T> result;
            result.value = determinant(f) - T(1) - mu_over_lambda;

            // The determinant's gradient with respect to one column, holding the
            // other two fixed, is the cross product of those other two — the
            // standard identity `det(F) = column_i . (column_j x column_k)` is
            // linear in `column_i`, for any cyclic choice of i/j/k.
            const Vector3T<T> d_by_column0 = cross(f.column1, f.column2);
            const Vector3T<T> d_by_column1 = cross(f.column2, f.column0);
            const Vector3T<T> d_by_column2 = cross(f.column0, f.column1);

            const Vector3T<T>& c0 = rest_inverse_column0;
            const Vector3T<T>& c1 = rest_inverse_column1;
            const Vector3T<T>& c2 = rest_inverse_column2;
            result.gradient[1] = d_by_column0 * c0.x + d_by_column1 * c1.x + d_by_column2 * c2.x;
            result.gradient[2] = d_by_column0 * c0.y + d_by_column1 * c1.y + d_by_column2 * c2.y;
            result.gradient[3] = d_by_column0 * c0.z + d_by_column1 * c1.z + d_by_column2 * c2.z;
            result.gradient[0] =
                (result.gradient[1] + result.gradient[2] + result.gradient[3]) * T(-1);
            return result;
        }

        /**
         * @brief Projects one XPBD constraint shared by four particles.
         *
         * The N-body generalization of the two-body update every constraint in
         * this engine already uses (Müller et al., XPBD): the position
         * correction is `delta_lambda * inv_mass[i] * gradient[i]` for each
         * particle `i`, where `delta_lambda` is chosen so the constraint moves
         * toward zero at the rate its compliance allows.
         *
         * @param bodies      The owning solver's particle array.
         * @param vertex      The four particle indices this constraint touches.
         * @param evaluation  The constraint's current value and gradient.
         * @param compliance  `1/(stiffness * rest_volume)`; zero is fully rigid.
         * @param accumulated_lambda The constraint's running multiplier; updated in place.
         * @param h           Sub-step duration, in seconds.
         */
        template <typename T>
        inline void apply_fem_constraint(RigidBodyT<T>* bodies, const std::uint32_t vertex[4],
                                         const FEMConstraintEvaluation<T>& evaluation, T compliance,
                                         T& accumulated_lambda, T h) noexcept
        {
            T denominator = 0;
            for (int i = 0; i < 4; ++i)
                denominator += bodies[vertex[i]].inv_mass *
                              dot(evaluation.gradient[i], evaluation.gradient[i]);

            const T alpha_tilde = compliance / (h * h);
            denominator += alpha_tilde;
            if (!(denominator > T(1e-12)))
                return; // every particle pinned, or a zero gradient: nothing to correct

            const T delta_lambda =
                (-evaluation.value - alpha_tilde * accumulated_lambda) / denominator;
            accumulated_lambda += delta_lambda;

            for (int i = 0; i < 4; ++i)
            {
                const T weight = bodies[vertex[i]].inv_mass;
                if (weight <= T(0))
                    continue;
                bodies[vertex[i]].position =
                    bodies[vertex[i]].position + evaluation.gradient[i] * (delta_lambda * weight);
            }
        }

        /**
         * @brief Projects one tetrahedron's deviatoric constraint in place.
         *
         * @param bodies   The owning solver's particle array.
         * @param element  The element; its `deviatoric_lambda` is updated, and its own
         *                 @ref FEMTetrahedronT::mu is the material this reads.
         * @param h        Sub-step duration, in seconds.
         */
        template <typename T>
        inline void project_fem_deviatoric(RigidBodyT<T>* bodies, FEMTetrahedronT<T>& element,
                                           T h) noexcept
        {
            const T mu = element.mu;
            const Vector3T<T>& x0 = bodies[element.vertex[0]].position;
            const Vector3T<T> edge1 = bodies[element.vertex[1]].position - x0;
            const Vector3T<T> edge2 = bodies[element.vertex[2]].position - x0;
            const Vector3T<T> edge3 = bodies[element.vertex[3]].position - x0;
            const FEMMatrix3<T> f = tetrahedron_deformation_gradient(
                edge1, edge2, edge3, element.plastic_inverse_column_0,
                element.plastic_inverse_column_1, element.plastic_inverse_column_2);
            const FEMConstraintEvaluation<T> evaluation = evaluate_deviatoric_constraint(
                f, element.plastic_inverse_column_0, element.plastic_inverse_column_1,
                element.plastic_inverse_column_2);
            const T compliance =
                element.rest_volume > T(0) && mu > T(0) ? T(1) / (mu * element.rest_volume) : T(0);
            apply_fem_constraint(bodies, element.vertex, evaluation, compliance,
                                 element.deviatoric_lambda, h);
        }

        /**
         * @brief Projects one tetrahedron's hydrostatic constraint in place.
         *
         * The Lame `lambda` is reparameterized to `lambda + mu` before it becomes
         * the constraint's stiffness and offset — Smith et al. 2018's consistency
         * correction, carried into the constraint form. Linearizing this model's
         * total force `mu*F + (lambda'*(J - 1) - mu) * cof(F)` about the rest
         * state gives effective Lame parameters `(mu, lambda' - mu)`: the
         * deviatoric term's own `-mu * cof(F)` counterweight eats one `mu` out of
         * the volumetric response, and passing the raw `lambda` through would
         * leave the material measurably softer than the `(E, nu)` it was asked
         * to be. With `lambda' = lambda + mu` the linearization lands on
         * `2*mu*strain + lambda*tr(strain)*I` exactly.
         *
         * @param bodies   The owning solver's particle array.
         * @param element  The element; its `hydrostatic_lambda` is updated, and its own
         *                 @ref FEMTetrahedronT::mu and @ref FEMTetrahedronT::lambda are
         *                 the material this reads.
         * @param h        Sub-step duration, in seconds.
         */
        template <typename T>
        inline void project_fem_hydrostatic(RigidBodyT<T>* bodies, FEMTetrahedronT<T>& element,
                                            T h) noexcept
        {
            const T mu = element.mu;
            const T lambda = element.lambda;
            const Vector3T<T>& x0 = bodies[element.vertex[0]].position;
            const Vector3T<T> edge1 = bodies[element.vertex[1]].position - x0;
            const Vector3T<T> edge2 = bodies[element.vertex[2]].position - x0;
            const Vector3T<T> edge3 = bodies[element.vertex[3]].position - x0;
            const FEMMatrix3<T> f = tetrahedron_deformation_gradient(
                edge1, edge2, edge3, element.plastic_inverse_column_0,
                element.plastic_inverse_column_1, element.plastic_inverse_column_2);
            const T stable_lambda = lambda + mu;
            const T mu_over_lambda = stable_lambda > T(0) ? mu / stable_lambda : T(0);
            const FEMConstraintEvaluation<T> evaluation = evaluate_hydrostatic_constraint(
                f, element.plastic_inverse_column_0, element.plastic_inverse_column_1,
                element.plastic_inverse_column_2, mu_over_lambda);
            const T compliance = element.rest_volume > T(0) && stable_lambda > T(0)
                                     ? T(1) / (stable_lambda * element.rest_volume)
                                     : T(0);
            apply_fem_constraint(bodies, element.vertex, evaluation, compliance,
                                 element.hydrostatic_lambda, h);
        }
    } // namespace Physics
} // namespace SushiEngine
