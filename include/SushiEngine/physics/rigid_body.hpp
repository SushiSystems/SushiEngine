/**************************************************************************/
/* rigid_body.hpp                                                        */
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

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A rigid body's XPBD state: pose, velocity, and generalized mass.
         *
         * Trivially copyable so it lives in a runtime buffer and crosses into device
         * code untouched. `inv_inertia` is the body's inverse inertia tensor expressed
         * diagonally in its own local frame (the common case: a body's inertia tensor
         * is diagonal in the frame aligned with its own principal axes); a component
         * of zero means "cannot be rotated about that axis" the same way `inv_mass ==
         * 0` pins a body's position. `prev_position`/`prev_orientation` hold the
         * pre-solve predicted pose so the velocity update can recover velocity and
         * angular velocity from the position/orientation the constraint solve settled
         * on, per XPBD's core idea (Müller et al., "XPBD: Position-Based Simulation of
         * Compliant Constrained Dynamics").
         */
        struct RigidBody
        {
            Vector3 position;
            Quaternion orientation{};
            Vector3 prev_position;
            Quaternion prev_orientation{};
            Vector3 velocity;
            Vector3 angular_velocity;
            Vector3 inv_inertia;
            Scalar inv_mass = 0;
        };

        /**
         * @brief Applies a rotation correction expressed in @p q's own local frame.
         *
         * The angular equivalent of a positional impulse: `local_delta` is the
         * (small) local-frame rotation vector a constraint projection wants to add,
         * folded in as `q += 0.5 * Quaternion(local_delta, 0) * q`, then renormalized. Used
         * by both the predicted-pose integration below and constraint projections.
         *
         * @param q           The orientation to correct.
         * @param local_delta The rotation vector to apply, in @p q's local frame.
         * @return The corrected, renormalized orientation.
         */
        inline Quaternion apply_angular_correction(const Quaternion& q, const Vector3& local_delta) noexcept
        {
            const Quaternion vq{local_delta.x, local_delta.y, local_delta.z, Scalar(0)};
            const Quaternion dq = mul(vq, q);
            const Quaternion updated{q.x + Scalar(0.5) * dq.x, q.y + Scalar(0.5) * dq.y,
                               q.z + Scalar(0.5) * dq.z, q.w + Scalar(0.5) * dq.w};
            return normalize(updated);
        }

        /**
         * @brief Predicts a body's pose for one XPBD sub-step, before constraint solving.
         *
         * Semi-implicit Euler on velocity and position/orientation, exactly the "predict"
         * half of XPBD: external forces integrate into velocity, velocity integrates into
         * a predicted pose, and the pre-predict pose is stashed for the velocity update
         * afterward. A body with `inv_mass == 0` (or a zero `inv_inertia` axis) does not
         * move along that degree of freedom, matching a pinned/fixed body.
         *
         * @param body                 The body to predict; updated in place.
         * @param linear_acceleration  External acceleration for this sub-step (e.g. gravity).
         * @param h                    Sub-step duration, in seconds (> 0).
         */
        inline void predict(RigidBody& body, Vector3 linear_acceleration, Scalar h) noexcept
        {
            body.prev_position = body.position;
            body.prev_orientation = body.orientation;

            if (body.inv_mass > Scalar(0))
            {
                body.velocity = body.velocity + linear_acceleration * h;
                body.position = body.position + body.velocity * h;
            }

            if (body.inv_inertia.x > Scalar(0) || body.inv_inertia.y > Scalar(0) ||
                body.inv_inertia.z > Scalar(0))
            {
                body.orientation =
                    apply_angular_correction(body.orientation, body.angular_velocity * h);
            }
        }

        /**
         * @brief Recovers velocity and angular velocity from the solved pose.
         *
         * The second half of one XPBD sub-step: velocity is the position delta over
         * @p h; angular velocity is derived from the orientation delta the same way,
         * taking the shorter rotational path (sign-correcting so the quaternion delta's
         * scalar part is non-negative).
         *
         * @param body The body whose pose was just solved; velocities updated in place.
         * @param h    Sub-step duration, in seconds (> 0).
         */
        inline void update_velocity(RigidBody& body, Scalar h) noexcept
        {
            if (h <= Scalar(0))
                return;

            body.velocity = (body.position - body.prev_position) * (Scalar(1) / h);

            const Quaternion delta = mul(body.orientation, conjugate(body.prev_orientation));
            const Scalar sign = delta.w < Scalar(0) ? Scalar(-1) : Scalar(1);
            body.angular_velocity = Vector3{delta.x, delta.y, delta.z} * (sign * Scalar(2) / h);
        }
    } // namespace Physics
} // namespace SushiEngine
