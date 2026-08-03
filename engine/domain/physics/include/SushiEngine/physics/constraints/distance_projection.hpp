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
         *
         * The generalized inverse mass and both impulses go through
         * `core/rigid_body.hpp`'s shared helpers rather than being spelled out here.
         * That is not tidying: this projection used to carry its own copy, and the
         * copy applied its angular correction as a **body-local** vector while
         * `apply_angular_correction` left-multiplies and therefore expects a
         * world-frame one. The correct delta is `R (I_local^-1 R^T (r x p))`; the
         * missing rotation back left every rotated body turning about the wrong
         * axis, so a distance constraint on a tumbling body did not conserve angular
         * momentum. It was invisible to cloth and particles, whose inverse inertia is
         * zero, and invisible to the conformance suite, because both implementations
         * shared the one wrong formula. One formulation, used everywhere, cannot
         * disagree with itself — which is the same argument §1.3 made for unifying
         * the plane and pair contact paths.
         */
        template <typename T>
        struct XPBDDistanceProjectionT
        {
            /**
             * @brief Applies one XPBD iteration of constraint @p c.
             * @param c       The constraint being satisfied.
             * @param bodies  The rigid-body array, updated in place.
             * @param lambda  This constraint's accumulated Lagrange multiplier for the
             *                current step; the caller resets it to zero once per step.
             * @param h       The sub-step duration used for this step, in seconds (> 0).
             */
            void operator()(const XPBDDistanceConstraintT<T>& c, RigidBodyT<T>* bodies,
                            T& lambda, T h) const
            {
                RigidBodyT<T>& body_a = bodies[c.a];
                RigidBodyT<T>& body_b = bodies[c.b];

                const Vector3T<T> anchor_a = rotate(body_a.orientation, c.local_anchor_a);
                const Vector3T<T> anchor_b = rotate(body_b.orientation, c.local_anchor_b);
                const Vector3T<T> p1 = body_a.position + anchor_a;
                const Vector3T<T> p2 = body_b.position + anchor_b;
                const Vector3T<T> d = p2 - p1;
                const T delta_length = length(d);
                if (delta_length <= T(1e-8))
                    return;
                const Vector3T<T> n = d * (T(1) / delta_length);
                const T error = delta_length - c.rest_length;

                const T w = generalized_inverse_mass(body_a, anchor_a, n) +
                            generalized_inverse_mass(body_b, anchor_b, n);
                if (w <= T(0))
                    return;

                const T alpha_tilde = h > T(0) ? c.compliance / (h * h) : T(0);
                const T delta_lambda = (-error - alpha_tilde * lambda) / (w + alpha_tilde);
                lambda += delta_lambda;

                const Vector3T<T> impulse = n * delta_lambda;
                apply_positional_impulse(body_a, impulse, anchor_a, T(-1));
                apply_positional_impulse(body_b, impulse, anchor_b, T(1));
            }
        };

        /**
         * @brief The boundary distance projection: `XPBDDistanceProjectionT` fixed to `Scalar`.
         */
        using XPBDDistanceProjection = XPBDDistanceProjectionT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
