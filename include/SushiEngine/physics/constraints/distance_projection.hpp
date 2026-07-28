/**************************************************************************/
/* distance_projection.hpp                                                */
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
 * @file distance_projection.hpp
 * @brief How a distance constraint corrects the two bodies it holds.
 *
 * A projection belongs with its constraint, not with a solver: §3.2 gives
 * `physics/constraints` the joint descriptors *and their projections*, while
 * `physics/solver` owns only the schedule. The practical consequence is what moved
 * it here — the projection used to live in `xpbd_solver.hpp`, which includes
 * SushiRuntime, so a host solver could not use the same arithmetic without dragging
 * in the runtime it exists to avoid. Two solvers running *different* arithmetic
 * cannot be held to a conformance suite, which would have made §4.4 unenforceable.
 *
 * The functor is captureless, so it crosses into device code untouched, and it is
 * the pattern every future constraint kind follows: a trivially-copyable descriptor
 * plus a captureless projection with this signature, registered with the solver
 * (§4.2).
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Projects one compliant rigid-body distance constraint (XPBD).
         *
         * Implements the generalized positional constraint of Müller et al.,
         * "Detailed Rigid Body Simulation with XPBD": the attachment points are
         * pulled toward `rest_length` apart, with the correction split between the
         * two bodies' linear and angular degrees of freedom by their generalized
         * inverse mass, and scaled by the constraint's compliance so stiffness is
         * independent of iteration count and step size. `compliance == 0` recovers
         * an infinitely stiff (hard) constraint — the rigid-body generalization of
         * `DistanceProjection`. A captureless functor, so it is device-copyable.
         */
        template <typename T>
        struct XpbdDistanceProjectionT
        {
            /**
             * @brief Applies one XPBD iteration of constraint @p c.
             * @param c       The constraint being satisfied.
             * @param bodies  The rigid-body array, updated in place.
             * @param lambda  This constraint's accumulated Lagrange multiplier for the
             *                current step; the caller resets it to zero once per step.
             * @param h       The sub-step duration used for this step, in seconds (> 0).
             */
            void operator()(const XpbdDistanceConstraintT<T>& c, RigidBodyT<T>* bodies,
                            T& lambda, T h) const
            {
                RigidBodyT<T>& body_a = bodies[c.a];
                RigidBodyT<T>& body_b = bodies[c.b];

                const Vector3T<T> anchor_a = rotate(body_a.orientation, c.local_anchor_a);
                const Vector3T<T> anchor_b = rotate(body_b.orientation, c.local_anchor_b);
                const Vector3T<T> p1 = body_a.position + anchor_a;
                const Vector3T<T> p2 = body_b.position + anchor_b;
                const Vector3T<T> d = p2 - p1;
                const T len = length(d);
                if (len <= T(1e-8))
                    return;
                const Vector3T<T> n = d * (T(1) / len);
                const T error = len - c.rest_length;

                // r x n, expressed in each body's own local frame (see rotate()'s
                // doc comment: R(a x b) = (Ra) x (Rb), so rotating the world cross
                // product back by the body's own orientation gives the same result
                // as crossing the local anchor with the locally-expressed normal).
                const Vector3T<T> rxn_a =
                    rotate(conjugate(body_a.orientation), cross(anchor_a, n));
                const Vector3T<T> rxn_b =
                    rotate(conjugate(body_b.orientation), cross(anchor_b, n));

                const Vector3T<T> iixn_a{body_a.inv_inertia.x * rxn_a.x,
                                  body_a.inv_inertia.y * rxn_a.y,
                                  body_a.inv_inertia.z * rxn_a.z};
                const Vector3T<T> iixn_b{body_b.inv_inertia.x * rxn_b.x,
                                  body_b.inv_inertia.y * rxn_b.y,
                                  body_b.inv_inertia.z * rxn_b.z};

                const T w = body_a.inv_mass + body_b.inv_mass +
                                 dot(rxn_a, iixn_a) + dot(rxn_b, iixn_b);
                if (w <= T(0))
                    return;

                const T alpha_tilde = h > T(0) ? c.compliance / (h * h) : T(0);
                const T delta_lambda = (-error - alpha_tilde * lambda) / (w + alpha_tilde);
                lambda += delta_lambda;

                const Vector3T<T> impulse = n * delta_lambda;
                body_a.position = body_a.position - impulse * body_a.inv_mass;
                body_b.position = body_b.position + impulse * body_b.inv_mass;

                body_a.orientation = apply_angular_correction(
                    body_a.orientation, iixn_a * (-delta_lambda));
                body_b.orientation = apply_angular_correction(
                    body_b.orientation, iixn_b * delta_lambda);
            }
        };

        /**
         * @brief The boundary distance projection: `XpbdDistanceProjectionT` fixed to `Scalar`.
         */
        using XpbdDistanceProjection = XpbdDistanceProjectionT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
