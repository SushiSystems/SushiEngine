/**************************************************************************/
/* bending_constraint.hpp                                                 */
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
 * @file bending_constraint.hpp
 * @brief §9.1's bending constraint — the thing today's cloth lacks (§1.2 item 13).
 *
 * Without it a cloth grid is a set of distance constraints and nothing else, so
 * it resists being stretched and offers no resistance whatsoever to being
 * folded: a flag creases into a sharp corner and stays there, a sheet dropped on
 * a sphere wraps it like foil rather than draping. Bending is what makes a
 * fabric a fabric rather than a chainmail net.
 *
 * **Not the dihedral-angle formulation, and the reason matters.** The textbook
 * position-based bending constraint is `C = acos(dot(n1, n2)) - angle_rest`,
 * whose gradient carries a factor of `1 / sqrt(1 - dot(n1, n2)^2)`. For a flat
 * stencil the two triangle normals are exactly opposite, that dot product is
 * exactly minus one, and the factor is a division by zero — sitting precisely
 * on the configuration a piece of cloth spends almost all of its life at and
 * every piece of cloth *starts* at. The numerator vanishes there too, so the
 * correction has a finite limit, but computing it means dividing one quantity
 * losing precision by another quantity losing precision, and the cancellation
 * is worst exactly where cloth needs the constraint to be most reliable.
 *
 * **The isometric formulation instead** (Bergou et al., "A quadratic bending
 * model for inextensible surfaces"), stated in the way that makes it obvious:
 * four points forming two triangles about a shared edge are coplanar exactly
 * when there are weights `w`, summing to zero, with `sum(w_i * x_i) = 0`. Those
 * weights depend only on the rest shape. So `v = sum(w_i * x_i)` is *the*
 * measure of how far the stencil has bent out of the arrangement it rests in —
 * it is identically zero for any rigid motion, any uniform scale, and any
 * in-plane shear, and grows with the fold. It needs no trigonometry, no
 * normals, no normalization of anything that can be degenerate, and its
 * gradient is a constant times a unit vector.
 *
 * The gradients are `w_i * v / |v|`, so the generalized inverse mass is
 * `sum(w_i^2 * inverse_mass_i)` — a constant per constraint but for the masses.
 * That is as cheap as a distance constraint over four particles.
 *
 * **The one limitation, stated rather than hidden.** For a stencil whose rest
 * shape is already creased, this measures the *magnitude* of the fold and not
 * its direction, so such a stencil resists flattening and resists folding
 * further, but does not by itself resist snapping through to the mirror-image
 * crease. Flat rest shapes — every cloth this engine builds — have no such
 * configuration to snap to, and a permanently creased rest shape that needs the
 * sign held wants a dihedral constraint with the singularity handled, which is
 * a different constraint and not this one pretending.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Two triangles about a shared edge, and how hard they resist folding.
         *
         * Trivially copyable and free of pointers, like every other constraint
         * descriptor here, so it can be stored in a flat array and handed to a
         * device solver unchanged.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct XPBDBendingConstraintT
        {
            /** @brief The scalar element type, so a solver can derive its precision. */
            using Real = T;

            /**
             * @brief The stencil's four particles.
             *
             * `particle[0]` and `particle[1]` are the shared edge; `particle[2]` and
             * `particle[3]` are the two vertices opposite it, one per triangle.
             * The projection never relies on that reading — it only ever uses the
             * weights — but the builder does, and a caller reordering them would
             * get a constraint that resists the wrong thing.
             */
            std::uint32_t particle[4] = {0, 0, 0, 0};

            /**
             * @brief The rest shape's coplanarity weights, summing to zero.
             *
             * Normalized so their absolute values sum to two, which makes @ref
             * XPBDBendingProjectionT's constraint value a length — the distance the
             * stencil has moved out of its rest arrangement — rather than a number
             * whose scale depends on how the mesh was tessellated. An author's
             * compliance then means the same thing on a coarse sheet and a fine one.
             */
            T weight[4] = {T(0), T(0), T(0), T(0)};

            /**
             * @brief `|sum(weight_i * rest_position_i)|`, zero for a flat rest shape.
             *
             * Subtracted from the current value so the constraint is satisfied at
             * rest whatever the rest shape is, rather than only for flat ones.
             */
            T rest_deviation = 0;

            /** @brief XPBD compliance; zero is a perfectly stiff hinge. */
            T compliance = 0;
        };

        /** @brief The boundary bending constraint: fixed to `Scalar`. */
        using XPBDBendingConstraint = XPBDBendingConstraintT<Scalar>;

        /**
         * @brief Builds a bending constraint's rest weights from four rest positions.
         *
         * The weights are the coplanarity relation of the rest stencil: `x3`
         * expressed in the barycentric frame of the triangle `(x0, x1, x2)` gives
         * `x3 = a*x0 + b*x1 + c*x2` with `a + b + c = 1`, so `(a, b, c, -1)` sums to
         * zero and annihilates the rest positions. A rest shape that is *not* flat
         * has no such relation exactly, so `x3` is projected onto the triangle's
         * plane first — which is the same answer when it is flat, and the nearest
         * one when it is not.
         *
         * @param x0,x1  The shared edge's rest positions.
         * @param x2,x3  The two opposite vertices' rest positions.
         * @param out    Receives the weights and @ref XPBDBendingConstraintT::rest_deviation.
         * @return False for a degenerate stencil, leaving @p out untouched.
         */
        template <typename T>
        inline bool build_bending_constraint(const Vector3T<T>& x0, const Vector3T<T>& x1,
                                             const Vector3T<T>& x2, const Vector3T<T>& x3,
                                             XPBDBendingConstraintT<T>& out) noexcept
        {
            const Vector3T<T> edge1 = x1 - x0;
            const Vector3T<T> edge2 = x2 - x0;
            const Vector3T<T> normal = cross(edge1, edge2);
            const T twice_area = length(normal);
            if (!(twice_area > T(1e-12)))
                return false; // the two "triangles" are a line

            const Vector3T<T> unit_normal = normal * (T(1) / twice_area);
            const Vector3T<T> offset = x3 - x0;
            const Vector3T<T> planar = offset - unit_normal * dot(unit_normal, offset);

            // Barycentric coordinates of the projected point, by the ratio of the
            // sub-triangle areas it forms — signed against the same normal, so a
            // point outside the triangle gets the negative weight it should.
            const T beta = dot(cross(planar, edge2), unit_normal) / twice_area;
            const T gamma = dot(cross(edge1, planar), unit_normal) / twice_area;
            const T alpha = T(1) - beta - gamma;

            T weight[4] = {alpha, beta, gamma, T(-1)};
            T magnitude = 0;
            for (int i = 0; i < 4; ++i)
                magnitude += weight[i] < T(0) ? -weight[i] : weight[i];
            if (!(magnitude > T(1e-12)))
                return false;

            const T scale = T(2) / magnitude;
            Vector3T<T> deviation{T(0), T(0), T(0)};
            const Vector3T<T> rest[4] = {x0, x1, x2, x3};
            for (int i = 0; i < 4; ++i)
            {
                out.weight[i] = weight[i] * scale;
                deviation = deviation + rest[i] * out.weight[i];
            }
            out.rest_deviation = length(deviation);
            return true;
        }

        /**
         * @brief One XPBD iteration of a bending constraint.
         *
         * A captureless functor, so it is device-copyable — the same shape
         * `XPBDDistanceProjectionT` has, for the same reason.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct XPBDBendingProjectionT
        {
            /**
             * @brief Applies one iteration of constraint @p c.
             *
             * @param c       The constraint being satisfied.
             * @param bodies  The particle array, updated in place.
             * @param lambda  This constraint's accumulated Lagrange multiplier; the
             *                caller resets it to zero once per step.
             * @param h       The sub-step duration, in seconds (> 0).
             */
            void operator()(const XPBDBendingConstraintT<T>& c, RigidBodyT<T>* bodies, T& lambda,
                            T h) const
            {
                Vector3T<T> deviation{T(0), T(0), T(0)};
                for (int i = 0; i < 4; ++i)
                    deviation = deviation + bodies[c.particle[i]].position * c.weight[i];

                const T magnitude = length(deviation);
                if (!(magnitude > T(1e-10)))
                    return; // exactly flat: at rest for a flat stencil, and no direction to push

                const T error = magnitude - c.rest_deviation;
                const Vector3T<T> direction = deviation * (T(1) / magnitude);

                // Each gradient is `weight_i * direction`, a unit vector scaled by a
                // constant, so the generalized inverse mass needs no cross products
                // and no lever arms — these are point masses and the constraint has
                // no angular part at all.
                T generalized_mass = 0;
                for (int i = 0; i < 4; ++i)
                    generalized_mass +=
                        c.weight[i] * c.weight[i] * bodies[c.particle[i]].inv_mass;
                if (!(generalized_mass > T(0)))
                    return; // every particle in the stencil is pinned

                const T alpha_tilde = h > T(0) ? c.compliance / (h * h) : T(0);
                const T delta_lambda =
                    (-error - alpha_tilde * lambda) / (generalized_mass + alpha_tilde);
                lambda += delta_lambda;

                for (int i = 0; i < 4; ++i)
                {
                    RigidBodyT<T>& body = bodies[c.particle[i]];
                    if (!(body.inv_mass > T(0)))
                        continue;
                    body.position =
                        body.position + direction * (c.weight[i] * body.inv_mass * delta_lambda);
                }
            }
        };

        /** @brief The boundary bending projection: fixed to `Scalar`. */
        using XPBDBendingProjection = XPBDBendingProjectionT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
