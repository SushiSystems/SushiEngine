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

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>

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
        template <typename T>
        struct RigidBodyT
        {
            Vector3T<T> position;
            QuaternionT<T> orientation{};
            Vector3T<T> prev_position;
            QuaternionT<T> prev_orientation{};
            Vector3T<T> velocity;
            Vector3T<T> angular_velocity;
            Vector3T<T> inv_inertia;
            T inv_mass = 0;
            T drag_coefficient = 0; /**< Quadratic drag k: -k|v|v acceleration each predict; 0 disables. */

            /**
             * @brief Offset from the body's authored origin to its centre of mass.
             *
             * `position` is the centre of mass — that is what a rigid body rotates
             * about, and the projection above assumes it. An authored model's origin
             * is rarely there (a car's origin is between its wheels, its centre of
             * mass is not), so the two are related by this offset and
             * @ref body_origin converts. Keeping the solver in centre-of-mass space
             * and converting at the boundary is what stops every projection from
             * having to know the difference.
             */
            Vector3T<T> center_of_mass_local;

            /** @brief Index into the scene's material table; 0 is the default material. */
            std::uint32_t material_index = 0;

            /** @brief `BodyFlags` bits: kinematic, static, sleeping, trigger, and so on. */
            std::uint32_t flags = BodyFlags::dynamic_body;

            /**
             * @brief Exponentially-smoothed motion, the metric sleeping reads.
             *
             * Smoothed rather than instantaneous because a body at the top of a
             * bounce is momentarily still without having settled, and a raw speed
             * test would put it to sleep in mid-air.
             */
            T motion_measure = 0;

            /** @brief Seconds this body has been below the sleep threshold. */
            T sleep_timer = 0;

            /** @brief The island this body was assigned to this tick; 0 when unassigned. */
            std::uint32_t island_index = 0;

            /** @brief Padding so the struct's size is the same in every translation unit. */
            std::uint32_t reserved_ = 0;
        };

        /**
         * @brief The body's authored origin, derived from its centre-of-mass pose.
         *
         * The inverse of what the scene does when it admits a body: the simulation
         * stores the centre of mass, the renderer and the ECS want the origin the
         * artist authored. Both directions live next to each other so they cannot
         * drift apart.
         *
         * @tparam T The scalar element type.
         * @param body The body whose origin is wanted.
         * @return The world-space position of the body's authored origin.
         */
        template <typename T>
        inline Vector3T<T> body_origin(const RigidBodyT<T>& body) noexcept
        {
            return body.position - rotate(body.orientation, body.center_of_mass_local);
        }

        /**
         * @brief The centre-of-mass position for a body placed at @p origin.
         *
         * @tparam T The scalar element type.
         * @param origin              The authored origin's world position.
         * @param orientation         The body's orientation.
         * @param center_of_mass_local The body-local centre-of-mass offset.
         * @return The world-space centre of mass to store in `position`.
         */
        template <typename T>
        inline Vector3T<T> center_of_mass_position(
            const Vector3T<T>& origin, const QuaternionT<T>& orientation,
            const Vector3T<T>& center_of_mass_local) noexcept
        {
            return origin + rotate(orientation, center_of_mass_local);
        }

        /**
         * @brief The boundary rigid body: `RigidBodyT` fixed to `Scalar`.
         *
         * The default precision every existing solver, bridge, and demo uses. A
         * simulation running in a runtime-selected precision instantiates
         * `RigidBodyT<double>` (or `<float>`) directly instead of this alias.
         */
        using RigidBody = RigidBodyT<Scalar>;

        /**
         * @brief Applies a rotation correction expressed in the world frame.
         *
         * The angular equivalent of a positional impulse: `world_delta` is the (small)
         * world-frame rotation vector a projection wants to add, folded in as
         * `q += 0.5 * Quaternion(world_delta, 0) * q`, then renormalized. The
         * left-multiplication is what makes it world-frame rather than body-frame, and
         * it is why @ref predict can hand it a world-frame angular velocity directly and
         * @ref update_velocity can recover one from a left-multiplied orientation delta.
         *
         * @tparam T The scalar element type.
         * @param q           The orientation to correct.
         * @param world_delta The rotation vector to apply, in world space.
         * @return The corrected, renormalized orientation.
         */
        template <typename T>
        inline QuaternionT<T> apply_angular_correction(const QuaternionT<T>& q,
                                                       const Vector3T<T>& world_delta) noexcept
        {
            const QuaternionT<T> vq{world_delta.x, world_delta.y, world_delta.z, T(0)};
            const QuaternionT<T> dq = mul(vq, q);
            const QuaternionT<T> updated{q.x + T(0.5) * dq.x, q.y + T(0.5) * dq.y,
                               q.z + T(0.5) * dq.z, q.w + T(0.5) * dq.w};
            return normalize(updated);
        }

        /**
         * @brief Predicts a body's pose for one XPBD sub-step, before constraint solving.
         *
         * Semi-implicit Euler on velocity and position/orientation, exactly the "predict"
         * half of XPBD: external forces integrate into velocity, velocity integrates into
         * a predicted pose, and the pre-predict pose is stashed for the velocity update
         * afterward. A body with `inv_mass == 0` (or a zero `inv_inertia` axis) does not
         * move along that degree of freedom, matching a pinned/fixed body; a body
         * flagged static or sleeping is skipped entirely, which is the difference
         * between "infinitely heavy" and "not simulated".
         *
         * @tparam T The scalar element type.
         * @param body                 The body to predict; updated in place.
         * @param linear_acceleration  External acceleration for this sub-step (e.g. gravity).
         * @param h                    Sub-step duration, in seconds (> 0).
         */
        template <typename T>
        inline void predict(RigidBodyT<T>& body, Vector3T<T> linear_acceleration, T h) noexcept
        {
            // A static or sleeping body is not integrated at all. Returning before
            // the previous-pose stash matters: leaving prev_* alone is what makes a
            // later update_velocity on the same body a no-op rather than a
            // spurious zero-velocity write.
            if (!is_simulated(body.flags))
                return;

            body.prev_position = body.position;
            body.prev_orientation = body.orientation;

            if (body.inv_mass > T(0))
            {
                body.velocity = body.velocity + linear_acceleration * h;
                // Quadratic aerodynamic drag, -k|v|v: opposes motion and grows with the
                // square of speed, so a body reaches terminal velocity under gravity instead
                // of accelerating without bound. Applied after the external acceleration and
                // before the position update (semi-implicit, so it stays stable).
                if (body.drag_coefficient > T(0))
                {
                    const T speed = length(body.velocity);
                    if (speed > T(0))
                        body.velocity =
                            body.velocity - body.velocity * (body.drag_coefficient * speed * h);
                }
                body.position = body.position + body.velocity * h;
            }

            if (body.inv_inertia.x > T(0) || body.inv_inertia.y > T(0) ||
                body.inv_inertia.z > T(0))
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
         * @tparam T The scalar element type.
         * A static or sleeping body is skipped, matching @ref predict: it was never
         * integrated, so there is no pose delta to read a velocity out of.
         *
         * @param body The body whose pose was just solved; velocities updated in place.
         * @param h    Sub-step duration, in seconds (> 0).
         */
        template <typename T>
        inline void update_velocity(RigidBodyT<T>& body, T h) noexcept
        {
            if (h <= T(0) || !is_simulated(body.flags))
                return;

            body.velocity = (body.position - body.prev_position) * (T(1) / h);

            const QuaternionT<T> delta = mul(body.orientation, conjugate(body.prev_orientation));
            const T sign = delta.w < T(0) ? T(-1) : T(1);
            body.angular_velocity = Vector3T<T>{delta.x, delta.y, delta.z} * (sign * T(2) / h);
        }
    } // namespace Physics
} // namespace SushiEngine
