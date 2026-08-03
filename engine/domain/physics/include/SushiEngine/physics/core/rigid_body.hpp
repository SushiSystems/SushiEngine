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

#include <cmath>
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
         * 0` pins a body's position. `previous_position`/`previous_orientation` hold the
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
            Vector3T<T> previous_position;
            QuaternionT<T> previous_orientation{};
            Vector3T<T> velocity;
            Vector3T<T> angular_velocity;
            Vector3T<T> inv_inertia;
            T inv_mass = 0;
            T drag_coefficient = 0; /**< Quadratic drag k: -k|v|v acceleration each predict; 0 disables. */

            /**
             * @brief This body's own external acceleration, added to the uniform one.
             *
             * The place a non-uniform field lands. `StepParameters::gravity` is a
             * single vector because that is what a solver can apply without knowing
             * where a body is; a planetary field, a wind volume or a local force
             * region is *not* uniform, and the scene above samples it per body and
             * folds it in here — which is what §5.1 and `StepParameters` already say
             * should happen, now with somewhere for it to happen.
             *
             * Per body per **tick**, not per substep, and that is forced rather than
             * chosen: `predict` runs on the device inside one composition, so there
             * is no point inside the substep loop at which a host sampler could be
             * called. A body travels at most the substep schedule's motion budget in
             * a tick, over which any field smooth enough to be worth sampling has
             * not meaningfully changed.
             */
            Vector3T<T> external_acceleration;

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
         * @brief Applies a body's world-space inverse inertia to a world-space vector.
         *
         * The inertia is stored as a body-local diagonal, so the vector is rotated
         * into the body frame, scaled per axis, and rotated back — the similarity
         * transform `R I_local^-1 R^T` without ever forming the matrix. Rotating
         * *back* is the step that is easy to drop, and dropping it leaves a
         * correction that is the right magnitude in the wrong direction for every
         * body that is not axis-aligned.
         *
         * @tparam T The scalar element type.
         * @param body         The body whose inertia is applied.
         * @param world_vector The world-frame vector (a torque or an `r x p`).
         * @return The world-frame result.
         */
        template <typename T>
        inline Vector3T<T> apply_world_inverse_inertia(const RigidBodyT<T>& body,
                                                       const Vector3T<T>& world_vector) noexcept
        {
            const Vector3T<T> local = rotate(conjugate(body.orientation), world_vector);
            const Vector3T<T> scaled{local.x * body.inv_inertia.x, local.y * body.inv_inertia.y,
                                     local.z * body.inv_inertia.z};
            return rotate(body.orientation, scaled);
        }

        /**
         * @brief The generalized inverse mass a body presents along @p direction at @p lever.
         *
         * `inv_mass + (r x n) . I^-1 (r x n)`: the linear share plus the angular
         * share the lever arm exposes. Every positional and velocity projection in
         * the engine divides by the sum of this over the two bodies, which is what
         * makes a light body yield to a heavy one and an off-centre push spend part
         * of itself as a turn. A body with no rotational freedom returns its inverse
         * mass unchanged, so the angular path is a strict extension of the purely
         * linear one rather than a separate case.
         *
         * A body that is not simulated presents *none*: a static body is immovable
         * by definition, and a sleeping one is immovable by decision (§13.2). Saying
         * so here rather than in each projection is what makes "asleep" mean the
         * same thing to the distance constraint, the contact solve and every joint
         * P3 adds — and what stops a settled stack being quietly pushed around by a
         * constraint that never asked whether its bodies were awake.
         *
         * @tparam T The scalar element type.
         * @param body      The body.
         * @param lever     World-space vector from the centre of mass to the point.
         * @param direction Unit world-space direction the impulse acts along.
         */
        template <typename T>
        inline T generalized_inverse_mass(const RigidBodyT<T>& body, const Vector3T<T>& lever,
                                          const Vector3T<T>& direction) noexcept
        {
            if (!is_simulated(body.flags))
                return T(0);
            const Vector3T<T> torque_axis = cross(lever, direction);
            return body.inv_mass + dot(torque_axis, apply_world_inverse_inertia(body, torque_axis));
        }

        /**
         * @brief Applies a positional impulse to a body's pose at a lever arm.
         *
         * @p sign is +1 for the body an impulse pushes along and -1 for the other,
         * so a pair separates with one call each and one shared impulse vector.
         *
         * @tparam T The scalar element type.
         * @param body    The body to move; pose updated in place.
         * @param impulse The world-space positional impulse.
         * @param lever   World-space vector from the centre of mass to the point.
         * @param sign    +1 or -1.
         */
        template <typename T>
        inline void apply_positional_impulse(RigidBodyT<T>& body, const Vector3T<T>& impulse,
                                             const Vector3T<T>& lever, T sign) noexcept
        {
            if (!is_simulated(body.flags))
                return;
            body.position = body.position + impulse * (sign * body.inv_mass);
            const Vector3T<T> rotation =
                apply_world_inverse_inertia(body, cross(lever, impulse * sign));
            if (dot(rotation, rotation) > T(0))
                body.orientation = apply_angular_correction(body.orientation, rotation);
        }

        /**
         * @brief The generalized inverse mass a body presents to a pure rotation about @p axis.
         *
         * `axis . I^-1 axis` — the angular half of @ref generalized_inverse_mass with
         * no lever arm, because an angular constraint does not act at a point. A
         * hinge holding two bodies' axes parallel, a twist limit, and a motor driving
         * a door open are all this quantity, and every one of them divides by the sum
         * of it over the two bodies for the same reason a positional row does: so the
         * body that is easier to turn takes more of the correction.
         *
         * Reports zero for a body that is not simulated, matching
         * @ref generalized_inverse_mass, so "asleep" and "static" mean the same thing
         * to an angular row as they do to a positional one.
         *
         * @tparam T The scalar element type.
         * @param body The body.
         * @param axis Unit world-space rotation axis.
         */
        template <typename T>
        inline T angular_inverse_mass(const RigidBodyT<T>& body,
                                      const Vector3T<T>& axis) noexcept
        {
            if (!is_simulated(body.flags))
                return T(0);
            return dot(axis, apply_world_inverse_inertia(body, axis));
        }

        /**
         * @brief Applies a pure angular positional impulse to a body's orientation.
         *
         * The angular counterpart of @ref apply_positional_impulse: no lever arm, so
         * the body turns without translating. @p sign is +1 for one body of a pair
         * and -1 for the other, exactly as it is there, so a joint's angular row is
         * two calls sharing one impulse vector.
         *
         * @tparam T The scalar element type.
         * @param body    The body to turn; orientation updated in place.
         * @param impulse The world-space angular impulse (an axis times a magnitude).
         * @param sign    +1 or -1.
         */
        template <typename T>
        inline void apply_angular_impulse(RigidBodyT<T>& body, const Vector3T<T>& impulse,
                                          T sign) noexcept
        {
            if (!is_simulated(body.flags))
                return;
            const Vector3T<T> rotation =
                apply_world_inverse_inertia(body, impulse * sign);
            if (dot(rotation, rotation) > T(0))
                body.orientation = apply_angular_correction(body.orientation, rotation);
        }

        /**
         * @brief Applies a pure angular velocity impulse to a body.
         *
         * What a velocity-mode motor and joint friction spend: an impulse that
         * changes only the angular velocity, saturated by the drive's force limit
         * before it gets here.
         *
         * @tparam T The scalar element type.
         * @param body    The body; angular velocity updated in place.
         * @param impulse The world-space angular impulse.
         * @param sign    +1 or -1.
         */
        template <typename T>
        inline void apply_angular_velocity_impulse(RigidBodyT<T>& body,
                                                   const Vector3T<T>& impulse,
                                                   T sign) noexcept
        {
            if (!is_simulated(body.flags))
                return;
            body.angular_velocity =
                body.angular_velocity + apply_world_inverse_inertia(body, impulse * sign);
        }

        /**
         * @brief Applies a velocity impulse to a body at a lever arm.
         *
         * The velocity-level counterpart of @ref apply_positional_impulse, and the
         * one XPBD needs for the quantities its positional projection cannot
         * express: restitution and dynamic friction (§7.4).
         *
         * @tparam T The scalar element type.
         * @param body    The body; velocities updated in place.
         * @param impulse The world-space impulse.
         * @param lever   World-space vector from the centre of mass to the point.
         * @param sign    +1 or -1.
         */
        template <typename T>
        inline void apply_velocity_impulse(RigidBodyT<T>& body, const Vector3T<T>& impulse,
                                           const Vector3T<T>& lever, T sign) noexcept
        {
            if (!is_simulated(body.flags))
                return;
            body.velocity = body.velocity + impulse * (sign * body.inv_mass);
            body.angular_velocity =
                body.angular_velocity +
                apply_world_inverse_inertia(body, cross(lever, impulse * sign));
        }

        /**
         * @brief The world-space velocity of the point @p lever away from the centre of mass.
         *
         * `v + w x r`. What a contact actually measures: the two bodies' centres may
         * be barely moving while the surfaces at the contact slide past each other.
         */
        template <typename T>
        inline Vector3T<T> point_velocity(const RigidBodyT<T>& body,
                                          const Vector3T<T>& lever) noexcept
        {
            return body.velocity + cross(body.angular_velocity, lever);
        }

        /**
         * @brief Turns @p q by the world-frame angular velocity @p omega over @p h, exactly.
         *
         * The exponential map, and **not** @ref apply_angular_correction's first-order
         * form, because the two are used for different sizes of rotation and only one of
         * them is small. A constraint correction is a fraction of a degree and the
         * first-order form is the standard, cheap, correct choice for it. A *prediction*
         * is a whole sub-step of free rotation, and for a wheel that is not small: a car
         * at 100 km/h spins its wheels at 82 rad/s, which is a tenth of a radian per
         * sub-step even at §11.1's effective rate.
         *
         * That difference is not cosmetic, and the arithmetic says why. The first-order
         * step normalizes `q + ½(ωh)q`, whose vector part is `θ/2` before normalization
         * and `sin(atan(θ/2))` after — a rotation *smaller* than `θ = |ω|h`. The velocity
         * recovery then reads the shortened angle back, so each sub-step multiplies the
         * angular velocity by `1/sqrt(1 + (θ/2)²)`. Measured before this was written: a
         * free body with no constraints at all, spinning at 50 rad/s at 480 Hz, was down
         * to 33 rad/s within a second — a third of a wheel's speed lost to nothing but
         * the integrator, and a drivetrain tuned against it would have been tuned against
         * a leak.
         *
         * Left-multiplied, so @p omega is read in the world frame — the same convention
         * @ref apply_angular_correction states and @ref update_velocity inverts.
         *
         * @tparam T The scalar element type.
         * @param q     The orientation to advance.
         * @param omega The world-frame angular velocity, in radians per second.
         * @param h     The sub-step duration, in seconds.
         * @return The advanced, renormalized orientation.
         */
        template <typename T>
        inline QuaternionT<T> integrate_orientation(const QuaternionT<T>& q,
                                                    const Vector3T<T>& omega, T h) noexcept
        {
            const T rate = length(omega);
            const T angle = rate * h;
            // Below this the exact and first-order forms agree to the last bit a `float`
            // can hold, and the division below has nothing left to normalize.
            if (!(angle > T(1e-12)))
                return q;

            const Vector3T<T> axis = omega * (T(1) / rate);
            const T half = angle * T(0.5);
            const T sine = std::sin(half);
            const QuaternionT<T> delta{axis.x * sine, axis.y * sine, axis.z * sine,
                                       std::cos(half)};
            return normalize(mul(delta, q));
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
            // the previous-pose stash matters: leaving previous_* alone is what makes a
            // later update_velocity on the same body a no-op rather than a
            // spurious zero-velocity write.
            if (!is_simulated(body.flags))
                return;

            body.previous_position = body.position;
            body.previous_orientation = body.orientation;

            if (body.inv_mass > T(0))
            {
                body.velocity =
                    body.velocity + (linear_acceleration + body.external_acceleration) * h;
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
                    integrate_orientation(body.orientation, body.angular_velocity, h);
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

            body.velocity = (body.position - body.previous_position) * (T(1) / h);

            const QuaternionT<T> delta =
                mul(body.orientation, conjugate(body.previous_orientation));
            const T sign = delta.w < T(0) ? T(-1) : T(1);
            const Vector3T<T> vector = Vector3T<T>{delta.x, delta.y, delta.z} * sign;
            // The logarithmic map, the exact inverse of `integrate_orientation`'s
            // exponential one. `2·vec(q)/h` is its small-angle approximation and reads
            // `2·sin(θ/2)` where the rotation was `θ` — which is where a fast body's
            // spin was leaking away before this was written; see `integrate_orientation`.
            const T sine = length(vector);
            if (!(sine > T(1e-12)))
            {
                body.angular_velocity = vector * (T(2) / h);
                return;
            }
            const T angle = T(2) * std::atan2(sine, delta.w * sign);
            body.angular_velocity = vector * (angle / (sine * h));
        }

        /**
         * @brief Updates the smoothed motion the sleeping decision reads.
         *
         * Smoothed rather than instantaneous, because the quantity wanted is "has
         * this settled" and not "is it moving right now": a body at the apex of a
         * bounce is momentarily still, a body in a stack jitters about zero, and an
         * unsmoothed test puts the first to sleep in mid-air and never puts the
         * second to sleep at all.
         *
         * The measure combines linear speed with rotational speed scaled by a
         * length, so a body spinning on the spot is not mistaken for a still one.
         * The length is the body's own radius of gyration, recovered from its
         * inertia — which is exactly the number that says how far its mass is from
         * its axis, so a long plank rotating slowly reads as the substantial motion
         * it is and a marble doing the same does not.
         *
         * The blend is `exp(-dt / tau)`, so the smoothing is a time constant rather
         * than a per-call fraction: a scene that changes its substep count does not
         * thereby change when its bodies fall asleep, which would make the sleeping
         * decision depend on the quality dial (§6.2).
         *
         * @tparam T The scalar element type.
         * @param body The body to measure.
         * @param dt   Time since the last update, in seconds.
         * @param tau  The smoothing time constant, in seconds.
         */
        template <typename T>
        inline void update_motion_measure(RigidBodyT<T>& body, T dt, T tau = T(0.1)) noexcept
        {
            if (dt <= T(0))
                return;

            const T linear = std::sqrt(dot(body.velocity, body.velocity));
            const T angular = std::sqrt(dot(body.angular_velocity, body.angular_velocity));

            // Radius of gyration from the smallest non-zero inverse inertia: r^2 =
            // I / m, and the *largest* inertia is the axis that most resists being
            // turned, so it is the one that says how big this body effectively is.
            T gyration = T(0);
            if (body.inv_mass > T(0))
            {
                const T inverse[3] = {body.inv_inertia.x, body.inv_inertia.y, body.inv_inertia.z};
                T smallest_inverse = T(0);
                for (int axis = 0; axis < 3; ++axis)
                    if (inverse[axis] > T(0) &&
                        (smallest_inverse <= T(0) || inverse[axis] < smallest_inverse))
                        smallest_inverse = inverse[axis];
                if (smallest_inverse > T(0))
                    gyration = std::sqrt(body.inv_mass / smallest_inverse);
            }

            const T motion = linear + angular * gyration;
            const T blend = std::exp(-dt / (tau > T(0) ? tau : T(1)));
            body.motion_measure = body.motion_measure * blend + motion * (T(1) - blend);
        }
    } // namespace Physics
} // namespace SushiEngine
