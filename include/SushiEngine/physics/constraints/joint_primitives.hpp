/**************************************************************************/
/* joint_primitives.hpp                                                   */
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
 * @file joint_primitives.hpp
 * @brief The four rows every joint in the library is built from.
 *
 * Müller et al. 2020, *Detailed Rigid Body Simulation with XPBD*, §3.3–3.4: a joint
 * is not a bespoke solve, it is a handful of **generic constraint rows** — one
 * positional, one angular, and their velocity-level counterparts — applied to
 * quantities the joint kind computes. A hinge and a cone-twist run the *same*
 * arithmetic on different violations. That is the whole reason `joint_projection.hpp`
 * can carry seven kinds in eight lines each.
 *
 * ### The one sign convention, stated once
 *
 * Every row here takes a **world-space violation vector** `v` and treats it as the
 * constraint function `C = |v|` whose gradient acts along `+v̂` on body **b** and
 * `-v̂` on body **a**. Written that way the correction is always
 * `Δλ = -C / (w + α̃)`, the impulse is always `v̂ Δλ`, and it is always applied to `a`
 * with sign −1 and to `b` with sign +1 — identical to `XpbdDistanceProjectionT`,
 * which is the point: a joint row and a distance constraint that disagreed about the
 * sense of a correction would be two formulations of the same thing, and §1.3 already
 * records what that costs.
 *
 * So a caller's job is to produce `v` such that *reducing it satisfies the joint*, and
 * the sign lives in `v`'s direction rather than in a separate argument. A limit
 * violated at its lower bound and one violated at its upper bound differ only in
 * which way `v` points.
 *
 * ### Saturation
 *
 * A drive has a force limit and a structural row does not, so every row takes a
 * `max_lambda` and a value at or below zero means unbounded. Clamping the *per-substep*
 * multiplier rather than an accumulated one is what makes the limit a force limit:
 * `λ = F h²`, so a bound on λ at a fixed `h` is a bound on force, and a bound on a
 * multiplier accumulated over a varying substep count would be a bound on nothing an
 * author can name.
 *
 * ### Why the load is folded here
 *
 * §10.4's `force = λ v̂ / h²` is exact and free — the multiplier is already in hand —
 * so every row folds its share into the caller's @ref JointLoad rather than returning
 * a number the caller has to remember to convert. A joint whose force readout depended
 * on each kind remembering to accumulate would be a joint whose break threshold worked
 * for six kinds and silently not for the seventh.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The force and torque a joint's rows have carried this substep.
         *
         * Accumulated across the rows of one joint in one substep, then folded into
         * the descriptor's running sums. A separate value rather than writing the
         * descriptor directly so the rows stay unaware of what they are attached to —
         * which is what lets the same row serve a joint, and later a soft-body
         * attachment or a vehicle beam.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointLoad
        {
            /** @brief Sum of `λ v̂ / h²` over the positional rows, in newtons. */
            Vector3T<T> force;

            /** @brief Sum of `λ v̂ / h²` over the angular rows, in newton-metres. */
            Vector3T<T> torque;
        };

        /**
         * @brief Both joint frames, resolved into world space.
         *
         * Computed once per joint per substep and shared by every row, because the
         * frames are what all of them are statements about and re-deriving them per
         * row would be four quaternion rotations to learn the same thing.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointWorldFrames
        {
            /** @brief Lever arm from body @c a's centre of mass to the attachment point. */
            Vector3T<T> lever_a;

            /** @brief Lever arm from body @c b's centre of mass to the attachment point. */
            Vector3T<T> lever_b;

            /** @brief The attachment point on body @c a, in world space. */
            Vector3T<T> point_a;

            /** @brief The attachment point on body @c b, in world space. */
            Vector3T<T> point_b;

            /** @brief Body @c a's joint frame orientation, in world space. */
            QuaternionT<T> basis_a{};

            /** @brief Body @c b's joint frame orientation, in world space. */
            QuaternionT<T> basis_b{};

            /** @brief Body @c a's primary axis (its frame's x), in world space. */
            Vector3T<T> axis_a;

            /** @brief Body @c b's primary axis (its frame's x), in world space. */
            Vector3T<T> axis_b;
        };

        /**
         * @brief Resolves a joint's two frames against the bodies' current poses.
         *
         * @tparam T The scalar element type.
         * @param joint  The joint.
         * @param body_a The first body.
         * @param body_b The second body.
         * @return The world-space frames every row reads.
         */
        template <typename T>
        inline JointWorldFrames<T> resolve_joint_frames(const JointConstraintT<T>& joint,
                                                        const RigidBodyT<T>& body_a,
                                                        const RigidBodyT<T>& body_b) noexcept
        {
            JointWorldFrames<T> frames;
            frames.lever_a = rotate(body_a.orientation, joint.local_anchor_a);
            frames.lever_b = rotate(body_b.orientation, joint.local_anchor_b);
            frames.point_a = body_a.position + frames.lever_a;
            frames.point_b = body_b.position + frames.lever_b;
            frames.basis_a = mul(body_a.orientation, joint.local_basis_a);
            frames.basis_b = mul(body_b.orientation, joint.local_basis_b);
            const Vector3T<T> x{T(1), T(0), T(0)};
            frames.axis_a = rotate(frames.basis_a, x);
            frames.axis_b = rotate(frames.basis_b, x);
            return frames;
        }

        /**
         * @brief The exact rotation vector of a quaternion: `axis * angle`.
         *
         * The logarithmic map rather than the small-angle `2 vec(q)`, because a joint
         * frame can be a long way from aligned — a ragdoll's arm authored at rest and
         * dropped into a scene, a door hanging past its limit after a teleport — and
         * the small-angle form under-reports a large misalignment by exactly the
         * factor that would leave it never converging.
         *
         * Sign-normalized to the shorter path: a quaternion and its negation are the
         * same rotation, and taking the long way round a full turn is not a
         * correction any joint means.
         *
         * @tparam T The scalar element type.
         * @param q The rotation.
         * @return Its axis times its angle, in radians; zero for the identity.
         */
        template <typename T>
        inline Vector3T<T> joint_rotation_vector(const QuaternionT<T>& q) noexcept
        {
            Vector3T<T> vec{q.x, q.y, q.z};
            T w = q.w;
            if (w < T(0))
            {
                vec = vec * T(-1);
                w = -w;
            }
            const T s = length(vec);
            if (!(s > T(1e-12)))
                return Vector3T<T>{T(0), T(0), T(0)};
            const T angle = T(2) * std::atan2(s, w);
            return vec * (angle / s);
        }

        /**
         * @brief Body @c b's joint frame expressed relative to body @c a's.
         *
         * Everything angular is a statement about this one quaternion: a fixed joint
         * wants it to be the identity, a hinge wants its swing part to be, a
         * cone-twist bounds both parts. Deriving each kind's angular violation from
         * one relative rotation is what keeps them from disagreeing about which way
         * round the pair is measured.
         *
         * @tparam T The scalar element type.
         * @param frames The resolved world frames.
         * @return The rotation taking @c a's frame to @c b's, expressed in @c a's frame.
         */
        template <typename T>
        inline QuaternionT<T> joint_relative_rotation(const JointWorldFrames<T>& frames) noexcept
        {
            return normalize(mul(conjugate(frames.basis_a), frames.basis_b));
        }

        /**
         * @brief Splits a relative rotation into a twist about x and a swing off it.
         *
         * The standard swing–twist decomposition, `q = swing * twist`, with the twist
         * about the joint frame's x axis (§ `joint.hpp`'s convention). Exact rather
         * than approximate: the twist part is the projection of @p relative onto the
         * one-parameter subgroup of rotations about x, and the swing is what remains.
         *
         * This is the decomposition every angular joint kind is expressed in. A hinge
         * is "swing must vanish"; a cone-twist is "swing is bounded and twist is
         * ranged"; a fixed joint is both bounded at zero. Deriving them from one
         * decomposition rather than from per-kind axis algebra is why a hinge's limit
         * and a cone-twist's twist limit are the same eight lines.
         *
         * @tparam T The scalar element type.
         * @param relative The relative rotation, from @ref joint_relative_rotation.
         * @param swing    Receives the swing part (a rotation about an axis ⟂ x).
         * @param twist    Receives the twist part (a rotation about x).
         */
        template <typename T>
        inline void joint_swing_twist(const QuaternionT<T>& relative, QuaternionT<T>& swing,
                                      QuaternionT<T>& twist) noexcept
        {
            const T x = relative.x;
            const T w = relative.w;
            const T magnitude = std::sqrt(x * x + w * w);

            // A half turn about an axis perpendicular to x leaves both components at
            // zero, so the twist is genuinely undetermined — every twist is as
            // consistent with the pose as any other. The identity is chosen rather
            // than normalizing a zero, which would be a division that produced a
            // direction out of nothing.
            if (!(magnitude > T(1e-12)))
            {
                twist = QuaternionT<T>{T(0), T(0), T(0), T(1)};
                swing = relative;
                return;
            }

            const T inverse = T(1) / magnitude;
            twist = QuaternionT<T>{x * inverse, T(0), T(0), w * inverse};
            swing = normalize(mul(relative, conjugate(twist)));
        }

        /**
         * @brief The signed twist angle of a relative rotation, in radians.
         *
         * `2 atan2(x, w)`, sign-normalized so the result lies in `(-π, π]`. This is
         * the hinge's angle and the cone-twist's twist coordinate — the number a
         * limit brackets and a position motor targets.
         *
         * @tparam T The scalar element type.
         * @param relative The relative rotation, from @ref joint_relative_rotation.
         */
        template <typename T>
        inline T joint_twist_angle(const QuaternionT<T>& relative) noexcept
        {
            T x = relative.x;
            T w = relative.w;
            if (w < T(0))
            {
                x = -x;
                w = -w;
            }
            return T(2) * std::atan2(x, w);
        }

        /**
         * @brief Clamps a multiplier to a drive's force limit.
         *
         * @tparam T The scalar element type.
         * @param lambda     The multiplier the row computed.
         * @param max_lambda The bound; at or below zero means unbounded.
         * @return The multiplier to actually apply.
         */
        template <typename T>
        inline T clamp_joint_lambda(T lambda, T max_lambda) noexcept
        {
            if (!(max_lambda > T(0)))
                return lambda;
            if (lambda > max_lambda)
                return max_lambda;
            if (lambda < -max_lambda)
                return -max_lambda;
            return lambda;
        }

        /**
         * @brief One positional row: pulls two attachment points along @p violation.
         *
         * @tparam T The scalar element type.
         * @param body_a     The first body; pose corrected in place.
         * @param body_b     The second body; pose corrected in place.
         * @param lever_a    World lever arm from @p body_a's centre of mass.
         * @param lever_b    World lever arm from @p body_b's centre of mass.
         * @param violation  The world-space violation; reducing it satisfies the row.
         * @param compliance XPBD compliance; zero is rigid.
         * @param h          The substep duration, in seconds (> 0).
         * @param max_lambda The force limit as `F h²`; at or below zero is unbounded.
         * @param load       Receives `λ v̂ / h²` folded into its force.
         * @return The multiplier applied, for a caller that needs it directly.
         */
        template <typename T>
        inline T apply_joint_positional_row(RigidBodyT<T>& body_a, RigidBodyT<T>& body_b,
                                           const Vector3T<T>& lever_a,
                                           const Vector3T<T>& lever_b,
                                           const Vector3T<T>& violation, T compliance, T h,
                                           T max_lambda, JointLoad<T>& load) noexcept
        {
            const T c = length(violation);
            if (!(c > T(1e-10)))
                return T(0);
            const Vector3T<T> n = violation * (T(1) / c);

            const T w = generalized_inverse_mass(body_a, lever_a, n) +
                        generalized_inverse_mass(body_b, lever_b, n);
            if (!(w > T(0)))
                return T(0);

            const T alpha = h > T(0) ? compliance / (h * h) : T(0);
            const T lambda = clamp_joint_lambda(-c / (w + alpha), max_lambda);

            const Vector3T<T> impulse = n * lambda;
            apply_positional_impulse(body_a, impulse, lever_a, T(-1));
            apply_positional_impulse(body_b, impulse, lever_b, T(1));

            if (h > T(0))
                load.force = load.force + impulse * (T(1) / (h * h));
            return lambda;
        }

        /**
         * @brief One angular row: turns the two bodies to reduce @p violation.
         *
         * No lever arm, because an angular constraint does not act at a point: the
         * correction is split by the two bodies' inverse inertia about the violation
         * axis alone.
         *
         * @tparam T The scalar element type.
         * @param body_a     The first body; orientation corrected in place.
         * @param body_b     The second body; orientation corrected in place.
         * @param violation  The world-space rotation violation (axis times angle).
         * @param compliance XPBD compliance; zero is rigid.
         * @param h          The substep duration, in seconds (> 0).
         * @param max_lambda The torque limit as `τ h²`; at or below zero is unbounded.
         * @param load       Receives `λ v̂ / h²` folded into its torque.
         * @return The multiplier applied.
         */
        template <typename T>
        inline T apply_joint_angular_row(RigidBodyT<T>& body_a, RigidBodyT<T>& body_b,
                                        const Vector3T<T>& violation, T compliance, T h,
                                        T max_lambda, JointLoad<T>& load) noexcept
        {
            const T theta = length(violation);
            if (!(theta > T(1e-10)))
                return T(0);
            const Vector3T<T> n = violation * (T(1) / theta);

            const T w = angular_inverse_mass(body_a, n) + angular_inverse_mass(body_b, n);
            if (!(w > T(0)))
                return T(0);

            const T alpha = h > T(0) ? compliance / (h * h) : T(0);
            const T lambda = clamp_joint_lambda(-theta / (w + alpha), max_lambda);

            const Vector3T<T> impulse = n * lambda;
            apply_angular_impulse(body_a, impulse, T(-1));
            apply_angular_impulse(body_b, impulse, T(1));

            if (h > T(0))
                load.torque = load.torque + impulse * (T(1) / (h * h));
            return lambda;
        }

        /**
         * @brief One angular velocity row: drives the relative spin about @p axis.
         *
         * A velocity-mode motor, and — with a target of zero — joint friction. Runs
         * in the velocity pass, after the pose solve has been read back as a velocity,
         * because a rate is not something a positional projection can express.
         *
         * @tparam T The scalar element type.
         * @param body_a      The first body; angular velocity updated in place.
         * @param body_b      The second body; angular velocity updated in place.
         * @param axis        Unit world-space axis the rate is measured about.
         * @param target_rate The relative angular rate to hold, in radians per second.
         * @param max_impulse The torque limit as `τ h`; at or below zero is unbounded.
         * @param h           The substep duration, in seconds (> 0).
         * @param load        Receives `impulse / h` folded into its torque.
         */
        template <typename T>
        inline void apply_joint_angular_velocity_row(RigidBodyT<T>& body_a,
                                                     RigidBodyT<T>& body_b,
                                                     const Vector3T<T>& axis, T target_rate,
                                                     T max_impulse, T h,
                                                     JointLoad<T>& load) noexcept
        {
            const T w = angular_inverse_mass(body_a, axis) + angular_inverse_mass(body_b, axis);
            if (!(w > T(0)))
                return;

            const T rate = dot(body_b.angular_velocity - body_a.angular_velocity, axis);
            const T error = rate - target_rate;
            if (!(std::abs(error) > T(1e-12)))
                return;

            const T magnitude = clamp_joint_lambda(-error / w, max_impulse);
            const Vector3T<T> impulse = axis * magnitude;
            apply_angular_velocity_impulse(body_a, impulse, T(-1));
            apply_angular_velocity_impulse(body_b, impulse, T(1));

            if (h > T(0))
                load.torque = load.torque + impulse * (T(1) / h);
        }

        /**
         * @brief One linear velocity row: drives the relative slide along @p axis.
         *
         * The linear counterpart of @ref apply_joint_angular_velocity_row — a
         * suspension damper, or a slider's friction. Measured between the two
         * *attachment points* rather than the two centres of mass, because a body
         * turning about its centre still slides at the joint.
         *
         * @tparam T The scalar element type.
         * @param body_a      The first body; velocity updated in place.
         * @param body_b      The second body; velocity updated in place.
         * @param lever_a     World lever arm from @p body_a's centre of mass.
         * @param lever_b     World lever arm from @p body_b's centre of mass.
         * @param axis        Unit world-space axis the rate is measured along.
         * @param target_rate The relative speed to hold, in metres per second.
         * @param max_impulse The force limit as `F h`; at or below zero is unbounded.
         * @param h           The substep duration, in seconds (> 0).
         * @param load        Receives `impulse / h` folded into its force.
         */
        template <typename T>
        inline void apply_joint_linear_velocity_row(RigidBodyT<T>& body_a, RigidBodyT<T>& body_b,
                                                    const Vector3T<T>& lever_a,
                                                    const Vector3T<T>& lever_b,
                                                    const Vector3T<T>& axis, T target_rate,
                                                    T max_impulse, T h,
                                                    JointLoad<T>& load) noexcept
        {
            const T w = generalized_inverse_mass(body_a, lever_a, axis) +
                        generalized_inverse_mass(body_b, lever_b, axis);
            if (!(w > T(0)))
                return;

            const T rate = dot(point_velocity(body_b, lever_b) - point_velocity(body_a, lever_a),
                               axis);
            const T error = rate - target_rate;
            if (!(std::abs(error) > T(1e-12)))
                return;

            const T magnitude = clamp_joint_lambda(-error / w, max_impulse);
            const Vector3T<T> impulse = axis * magnitude;
            apply_velocity_impulse(body_a, impulse, lever_a, T(-1));
            apply_velocity_impulse(body_b, impulse, lever_b, T(1));

            if (h > T(0))
                load.force = load.force + impulse * (T(1) / h);
        }
    } // namespace Physics
} // namespace SushiEngine
