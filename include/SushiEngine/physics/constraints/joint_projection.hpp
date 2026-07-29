/**************************************************************************/
/* joint_projection.hpp                                                   */
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
 * @file joint_projection.hpp
 * @brief The joint library: seven kinds, one registration list, no switch.
 *
 * §4.2's rule for a new constraint kind is that it adds a descriptor, a projection,
 * and **one registration line**, and edits nothing that already exists. The mechanism
 * here is the one `narrowphase_dispatch.hpp` already established for shapes: a type
 * list, folded. A joint kind is a traits struct naming its @ref JointKind and its two
 * projections; @ref JointKindList is the registration; the fold below turns the list
 * into the dispatch. Adding an eighth kind is a new traits struct in a new place and
 * one name added to the list — no function body anywhere is touched, which is
 * precisely what a `switch` could not promise.
 *
 * ### Why a fold and not a function-pointer table
 *
 * The narrowphase dispatches through a table of function pointers because it runs on
 * the host. A joint projection runs **inside the solve graph**, on the device, and
 * SYCL device code has no indirect calls — a table of pointers would compile and then
 * fail to link, or worse, work on the CPU backend and not on a GPU. The fold is
 * resolved at compile time into a chain of equality tests against a compile-time
 * constant, which is device-legal, and its cost is a handful of integer compares
 * against arithmetic that is two square roots and a quaternion product deep.
 *
 * ### The rows, and what each kind spends
 *
 * Every kind is composed from `joint_primitives.hpp`'s four rows applied to
 * quantities derived from one relative rotation and one anchor offset. The table is
 * the whole library:
 *
 * | Kind | Positional rows | Velocity rows |
 * |---|---|---|
 * | `Fixed` | attach, lock all rotation | — |
 * | `Ball` | attach | — |
 * | `Hinge` | attach, lock swing, twist limit, twist drive | twist drive/friction |
 * | `Slider` | lock all rotation, lock perpendicular offset, axial limit, axial drive | axial drive/damper |
 * | `Distance` | axial range along the anchor line | — |
 * | `ConeTwist` | attach, swing cone, twist limit, twist drive | twist drive/friction |
 * | `SixDegreeOfFreedom` | per-axis offset limits, swing cone, twist limit, twist drive | twist drive/friction |
 *
 * ### One refinement of §10.1, stated rather than hidden
 *
 * §10.1's table gives `BallJoint` "swing and twist limits, cone angle" — which is
 * exactly `ConeTwistJoint`'s row. Two kinds that differ only in whether the author
 * remembered to enable a limit are two names for one thing, so `Ball` here is the
 * pure spherical joint (attachment only, rotation free) and a ball joint with limits
 * *is* a `ConeTwist`. The library is smaller by one kind and no capability is lost.
 *
 * `GearJoint` and `RackJoint` are deliberately absent. They couple two *accumulated*
 * rotations, which means carrying an unwrapped angle across ticks — state no other
 * joint needs — and their use is the differential and the steering rack. §10.5 has
 * already ruled that a drivetrain is solved as an independent one-dimensional
 * multibody chain coupled through a torque constraint, precisely because forcing it
 * through the three-dimensional solver is both slower and less accurate. So the gear
 * belongs with the powertrain in §11.4, not here, and putting it here would be
 * building the tool §10.5 says not to use.
 */

#include <cmath>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/constraints/joint_primitives.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The angular state every rotational row reads, derived once.
         *
         * The relative rotation, its swing/twist split, and the signed twist angle.
         * Computed once per joint per substep because four rows of a hinge or a
         * cone-twist are four statements about the same decomposition.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointAngularState
        {
            QuaternionT<T> relative{}; /**< Body @c b's frame relative to @c a's, in @c a's frame. */
            QuaternionT<T> swing{};    /**< The part about an axis perpendicular to the primary one. */
            QuaternionT<T> twist{};    /**< The part about the primary axis. */
            T twist_angle = 0;         /**< The signed twist, in radians, in `(-π, π]`. */
        };

        /** @brief Derives the angular state from the resolved frames. */
        template <typename T>
        inline JointAngularState<T> resolve_joint_angular_state(
            const JointWorldFrames<T>& frames) noexcept
        {
            JointAngularState<T> state;
            state.relative = joint_relative_rotation(frames);
            joint_swing_twist(state.relative, state.swing, state.twist);
            state.twist_angle = joint_twist_angle(state.relative);
            return state;
        }

        /**
         * @brief Folds one group of rows' load into a joint's running accounting.
         *
         * Two quantities from one addition, because they answer different questions.
         * The **vector sums** give the mean load once divided by the sample count: which
         * way and how hard the mount is being pulled, which is the readout an inspector
         * shows and the rigid-body half of *mukavemet*. The **peaks** give the worst
         * instant, which is what a break threshold has to be measured against —
         * averaging a load whose direction reverses measures the net pull, and what
         * tears a mount out is the magnitude (`joint.hpp`, @ref
         * JointConstraintT::break_force).
         *
         * Called once per row group rather than once per substep, so a substep's
         * positional rows and its velocity rows contribute separately. That is the
         * honest reading of "the worst instant": the two groups run at different points
         * in the schedule and against different state, so they are different instants.
         *
         * @tparam T The scalar element type.
         * @param joint The joint whose accounting to update.
         * @param load  The group's load, as its rows folded it.
         */
        template <typename T>
        inline void fold_joint_load(JointConstraintT<T>& joint, const JointLoad<T>& load) noexcept
        {
            joint.force_sum = joint.force_sum + load.force;
            joint.torque_sum = joint.torque_sum + load.torque;

            const T force = length(load.force);
            if (force > joint.peak_force)
                joint.peak_force = force;
            const T torque = length(load.torque);
            if (torque > joint.peak_torque)
                joint.peak_torque = torque;
        }

        /** @brief The multiplier bound a drive's force limit implies at substep @p h. */
        template <typename T>
        inline T joint_drive_lambda_bound(const JointMotorT<T>& motor, T h) noexcept
        {
            return motor.max_force > T(0) ? motor.max_force * h * h : T(0);
        }

        /** @brief The impulse bound a drive's force limit implies at substep @p h. */
        template <typename T>
        inline T joint_drive_impulse_bound(const JointMotorT<T>& motor, T h) noexcept
        {
            return motor.max_force > T(0) ? motor.max_force * h : T(0);
        }

        // ----------------------------------------------------------------------
        // The shared rows. Each is one statement about the joint, and every kind
        // below is a short list of them.
        // ----------------------------------------------------------------------

        /** @brief The two attachment points coincide: three translational degrees removed. */
        template <typename T>
        inline void joint_row_attach(JointConstraintT<T>& joint,
                                     const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                     RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
        {
            apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                       frames.point_b - frames.point_a, joint.compliance, h,
                                       T(0), load);
        }

        /** @brief The two joint frames coincide in orientation: three rotational degrees removed. */
        template <typename T>
        inline void joint_row_lock_rotation(JointConstraintT<T>& joint,
                                            const JointWorldFrames<T>& frames,
                                            const JointAngularState<T>& angular,
                                            RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                            JointLoad<T>& load) noexcept
        {
            const Vector3T<T> local = joint_rotation_vector(angular.relative);
            apply_joint_angular_row(body_a, body_b, rotate(frames.basis_a, local),
                                    joint.compliance, h, T(0), load);
        }

        /** @brief The primary axes stay parallel: two rotational degrees removed (the hinge). */
        template <typename T>
        inline void joint_row_lock_swing(JointConstraintT<T>& joint,
                                         const JointWorldFrames<T>& frames,
                                         const JointAngularState<T>& angular,
                                         RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                         JointLoad<T>& load) noexcept
        {
            const Vector3T<T> local = joint_rotation_vector(angular.swing);
            apply_joint_angular_row(body_a, body_b, rotate(frames.basis_a, local),
                                    joint.compliance, h, T(0), load);
        }

        /** @brief The twist stays inside `twist_limit`, projected only when it does not. */
        template <typename T>
        inline void joint_row_twist_limit(JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          const JointAngularState<T>& angular,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            const T violation = joint_limit_violation(joint.twist_limit, angular.twist_angle);
            if (violation == T(0))
                return;
            apply_joint_angular_row(body_a, body_b, frames.axis_a * violation,
                                    joint.twist_limit.compliance, h, T(0), load);
        }

        /**
         * @brief The primary axes stay within a cone of `swing_limit.upper`.
         *
         * Only the upper bound is read, per @ref JointConstraintT::swing_limit: a
         * swing is an unsigned angle off an axis, and a lower bound on it would say
         * the joint must stay bent.
         */
        template <typename T>
        inline void joint_row_swing_limit(JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          const JointAngularState<T>& angular,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            if (!joint.swing_limit.enabled)
                return;
            const Vector3T<T> local = joint_rotation_vector(angular.swing);
            const T angle = length(local);
            const T upper = joint.swing_limit.upper > T(0) ? joint.swing_limit.upper : T(0);
            if (!(angle > upper))
                return;
            const Vector3T<T> violation = local * ((angle - upper) / angle);
            apply_joint_angular_row(body_a, body_b, rotate(frames.basis_a, violation),
                                    joint.swing_limit.compliance, h, T(0), load);
        }

        /** @brief The anchor offset has no component off the primary axis (the slider). */
        template <typename T>
        inline void joint_row_lock_perpendicular(JointConstraintT<T>& joint,
                                                 const JointWorldFrames<T>& frames,
                                                 RigidBodyT<T>& body_a, RigidBodyT<T>& body_b,
                                                 T h, JointLoad<T>& load) noexcept
        {
            const Vector3T<T> offset = frames.point_b - frames.point_a;
            const Vector3T<T> perpendicular =
                offset - frames.axis_a * dot(offset, frames.axis_a);
            apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                       perpendicular, joint.compliance, h, T(0), load);
        }

        /** @brief The offset along the primary axis stays inside `linear_limit`. */
        template <typename T>
        inline void joint_row_axial_limit(JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            const Vector3T<T> offset = frames.point_b - frames.point_a;
            const T along = dot(offset, frames.axis_a);
            const T violation = joint_limit_violation(joint.linear_limit, along);
            if (violation == T(0))
                return;
            apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                       frames.axis_a * violation, joint.linear_limit.compliance,
                                       h, T(0), load);
        }

        /**
         * @brief The anchor separation stays inside `linear_limit` (the distance joint).
         *
         * Along the line between the anchors rather than along the primary axis,
         * which is what makes it a rope rather than a rail: the direction is wherever
         * the two ends happen to be. A separation of zero leaves the direction
         * undetermined, so the row is skipped rather than normalizing a zero.
         */
        template <typename T>
        inline void joint_row_distance_range(JointConstraintT<T>& joint,
                                             const JointWorldFrames<T>& frames,
                                             RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                             JointLoad<T>& load) noexcept
        {
            const Vector3T<T> offset = frames.point_b - frames.point_a;
            const T separation = length(offset);
            if (!(separation > T(1e-10)))
                return;
            const T violation = joint_limit_violation(joint.linear_limit, separation);
            if (violation == T(0))
                return;
            apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                       offset * (violation / separation), joint.compliance, h,
                                       T(0), load);
        }

        /** @brief A position-mode drive holding the twist at the motor's target. */
        template <typename T>
        inline void joint_row_twist_drive(JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          const JointAngularState<T>& angular,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            if (joint.motor.mode != JointMotorMode::Position)
                return;
            const T error = angular.twist_angle - joint.motor.target;
            apply_joint_angular_row(body_a, body_b, frames.axis_a * error,
                                    joint.motor.compliance, h,
                                    joint_drive_lambda_bound(joint.motor, h), load);
        }

        /** @brief A position-mode drive holding the axial offset at the motor's target. */
        template <typename T>
        inline void joint_row_axial_drive(JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            if (joint.motor.mode != JointMotorMode::Position)
                return;
            const Vector3T<T> offset = frames.point_b - frames.point_a;
            const T error = dot(offset, frames.axis_a) - joint.motor.target;
            apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                       frames.axis_a * error, joint.motor.compliance, h,
                                       joint_drive_lambda_bound(joint.motor, h), load);
        }

        /** @brief A velocity-mode drive on the twist rate; with target zero, hinge friction. */
        template <typename T>
        inline void joint_row_twist_rate_drive(JointConstraintT<T>& joint,
                                               const JointWorldFrames<T>& frames,
                                               RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                               JointLoad<T>& load) noexcept
        {
            if (joint.motor.mode != JointMotorMode::Velocity)
                return;
            apply_joint_angular_velocity_row(body_a, body_b, frames.axis_a, joint.motor.target,
                                             joint_drive_impulse_bound(joint.motor, h), h, load);
        }

        /** @brief A velocity-mode drive on the slide rate; with target zero, a damper. */
        template <typename T>
        inline void joint_row_axial_rate_drive(JointConstraintT<T>& joint,
                                               const JointWorldFrames<T>& frames,
                                               RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                               JointLoad<T>& load) noexcept
        {
            if (joint.motor.mode != JointMotorMode::Velocity)
                return;
            apply_joint_linear_velocity_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                            frames.axis_a, joint.motor.target,
                                            joint_drive_impulse_bound(joint.motor, h), h, load);
        }

        /**
         * @brief Per-axis offset limits in body @c a's joint frame (the general case).
         *
         * A disabled limit leaves that axis free; `lower == upper` locks it. Which is
         * why the general joint needs no per-axis mode word: free, limited and locked
         * are three readings of one range, and a fourth word saying which reading was
         * meant would be a word that could disagree with the numbers.
         */
        template <typename T>
        inline void joint_row_frame_limits(JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
        {
            const Vector3T<T> offset = frames.point_b - frames.point_a;

            const JointLimitT<T>* limits[3] = {&joint.linear_limit, &joint.linear_limit_y,
                                               &joint.linear_limit_z};
            const Vector3T<T> axes[3] = {
                frames.axis_a, rotate(frames.basis_a, Vector3T<T>{T(0), T(1), T(0)}),
                rotate(frames.basis_a, Vector3T<T>{T(0), T(0), T(1)})};

            for (int axis = 0; axis < 3; ++axis)
            {
                const T along = dot(offset, axes[axis]);
                const T violation = joint_limit_violation(*limits[axis], along);
                if (violation == T(0))
                    continue;
                apply_joint_positional_row(body_a, body_b, frames.lever_a, frames.lever_b,
                                           axes[axis] * violation, limits[axis]->compliance, h,
                                           T(0), load);
            }
        }

        // ----------------------------------------------------------------------
        // The kinds. Each is a traits struct: a kind tag and three projections —
        // angular, linear, velocity. The split is not organizational; see
        // `JointProjectionT` for why the two positional groups cannot share one
        // frame resolution.
        // ----------------------------------------------------------------------

        /** @brief All six degrees of freedom removed; a compliant one is a flexible weld. */
        struct FixedJointTraits
        {
            static constexpr JointKind kind = JointKind::Fixed;

            template <typename T>
            static void project_angular(JointConstraintT<T>& joint,
                                        const JointWorldFrames<T>& frames,
                                        const JointAngularState<T>& angular,
                                        RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                        JointLoad<T>& load) noexcept
            {
                joint_row_lock_rotation(joint, frames, angular, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_attach(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>&, const JointWorldFrames<T>&,
                                           RigidBodyT<T>&, RigidBodyT<T>&, T,
                                           JointLoad<T>&) noexcept
            {
            }
        };

        /** @brief The pure spherical joint: attached, rotation free. Limits make it a cone-twist. */
        struct BallJointTraits
        {
            static constexpr JointKind kind = JointKind::Ball;

            template <typename T>
            static void project_angular(JointConstraintT<T>&, const JointWorldFrames<T>&,
                                        const JointAngularState<T>&, RigidBodyT<T>&,
                                        RigidBodyT<T>&, T, JointLoad<T>&) noexcept
            {
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_attach(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>&, const JointWorldFrames<T>&,
                                           RigidBodyT<T>&, RigidBodyT<T>&, T,
                                           JointLoad<T>&) noexcept
            {
            }
        };

        /** @brief One surviving rotation, limited and drivable. The car door. */
        struct HingeJointTraits
        {
            static constexpr JointKind kind = JointKind::Hinge;

            template <typename T>
            static void project_angular(JointConstraintT<T>& joint,
                                        const JointWorldFrames<T>& frames,
                                        const JointAngularState<T>& angular,
                                        RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                        JointLoad<T>& load) noexcept
            {
                joint_row_lock_swing(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_limit(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_drive(joint, frames, angular, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_attach(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
            {
                joint_row_twist_rate_drive(joint, frames, body_a, body_b, h, load);
            }
        };

        /** @brief One surviving translation, limited and drivable. Suspension travel. */
        struct SliderJointTraits
        {
            static constexpr JointKind kind = JointKind::Slider;

            template <typename T>
            static void project_angular(JointConstraintT<T>& joint,
                                        const JointWorldFrames<T>& frames,
                                        const JointAngularState<T>& angular,
                                        RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                        JointLoad<T>& load) noexcept
            {
                joint_row_lock_rotation(joint, frames, angular, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_lock_perpendicular(joint, frames, body_a, body_b, h, load);
                joint_row_axial_limit(joint, frames, body_a, body_b, h, load);
                joint_row_axial_drive(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
            {
                joint_row_axial_rate_drive(joint, frames, body_a, body_b, h, load);
            }
        };

        /** @brief A range along the line between the anchors: a rope, or a two-sided strut. */
        struct DistanceJointTraits
        {
            static constexpr JointKind kind = JointKind::Distance;

            template <typename T>
            static void project_angular(JointConstraintT<T>&, const JointWorldFrames<T>&,
                                        const JointAngularState<T>&, RigidBodyT<T>&,
                                        RigidBodyT<T>&, T, JointLoad<T>&) noexcept
            {
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_distance_range(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>&, const JointWorldFrames<T>&,
                                           RigidBodyT<T>&, RigidBodyT<T>&, T,
                                           JointLoad<T>&) noexcept
            {
            }
        };

        /** @brief Attached, with a swing cone and a twist range. Ragdolls. */
        struct ConeTwistJointTraits
        {
            static constexpr JointKind kind = JointKind::ConeTwist;

            template <typename T>
            static void project_angular(JointConstraintT<T>& joint,
                                        const JointWorldFrames<T>& frames,
                                        const JointAngularState<T>& angular,
                                        RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                        JointLoad<T>& load) noexcept
            {
                joint_row_swing_limit(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_limit(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_drive(joint, frames, angular, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_attach(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
            {
                joint_row_twist_rate_drive(joint, frames, body_a, body_b, h, load);
            }
        };

        /** @brief Every axis free, limited, or locked. Reached for when no named kind says it. */
        struct SixDegreeOfFreedomJointTraits
        {
            static constexpr JointKind kind = JointKind::SixDegreeOfFreedom;

            template <typename T>
            static void project_angular(JointConstraintT<T>& joint,
                                        const JointWorldFrames<T>& frames,
                                        const JointAngularState<T>& angular,
                                        RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                        JointLoad<T>& load) noexcept
            {
                joint_row_swing_limit(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_limit(joint, frames, angular, body_a, body_b, h, load);
                joint_row_twist_drive(joint, frames, angular, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_linear(JointConstraintT<T>& joint,
                                       const JointWorldFrames<T>& frames, RigidBodyT<T>& body_a,
                                       RigidBodyT<T>& body_b, T h, JointLoad<T>& load) noexcept
            {
                joint_row_frame_limits(joint, frames, body_a, body_b, h, load);
            }

            template <typename T>
            static void project_velocities(JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
            {
                joint_row_twist_rate_drive(joint, frames, body_a, body_b, h, load);
            }
        };

        /**
         * @brief The registration list: the one place a joint kind is declared to exist.
         *
         * §4.2's "one registration line". A new kind is appended here and nowhere
         * else; the folds below derive the dispatch from it, so no existing function
         * body changes to admit one.
         *
         * @tparam Kinds The traits structs, in any order — the fold tests a kind tag,
         *               not a position, so the order carries no meaning.
         */
        template <typename... Kinds>
        struct JointKindList
        {
        };

        /** @brief Every joint kind the solver projects. */
        using JointKinds =
            JointKindList<FixedJointTraits, BallJointTraits, HingeJointTraits, SliderJointTraits,
                          DistanceJointTraits, ConeTwistJointTraits,
                          SixDegreeOfFreedomJointTraits>;

        /** @brief Projects @p joint's angular rows if @c Traits owns its kind. */
        template <typename Traits, typename T>
        inline bool try_project_joint_angular(JointConstraintT<T>& joint,
                                              const JointWorldFrames<T>& frames,
                                              const JointAngularState<T>& angular,
                                              RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                              JointLoad<T>& load) noexcept
        {
            if (joint.kind != Traits::kind)
                return false;
            Traits::project_angular(joint, frames, angular, body_a, body_b, h, load);
            return true;
        }

        /** @brief Projects @p joint's linear rows if @c Traits owns its kind. */
        template <typename Traits, typename T>
        inline bool try_project_joint_linear(JointConstraintT<T>& joint,
                                             const JointWorldFrames<T>& frames,
                                             RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                             JointLoad<T>& load) noexcept
        {
            if (joint.kind != Traits::kind)
                return false;
            Traits::project_linear(joint, frames, body_a, body_b, h, load);
            return true;
        }

        /** @brief Projects @p joint's velocity rows if @c Traits owns its kind. */
        template <typename Traits, typename T>
        inline bool try_project_joint_velocities(JointConstraintT<T>& joint,
                                                 const JointWorldFrames<T>& frames,
                                                 RigidBodyT<T>& body_a, RigidBodyT<T>& body_b,
                                                 T h, JointLoad<T>& load) noexcept
        {
            if (joint.kind != Traits::kind)
                return false;
            Traits::project_velocities(joint, frames, body_a, body_b, h, load);
            return true;
        }

        /** @brief Folds the registration list into the angular dispatch. */
        template <typename... Kinds, typename T>
        inline void dispatch_joint_angular(JointKindList<Kinds...>, JointConstraintT<T>& joint,
                                           const JointWorldFrames<T>& frames,
                                           const JointAngularState<T>& angular,
                                           RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                           JointLoad<T>& load) noexcept
        {
            // A short-circuiting fold: the matching kind stops the chain, and an
            // unrecognized kind simply projects nothing rather than falling into a
            // default branch that would have to invent a behaviour for it.
            (void)(try_project_joint_angular<Kinds>(joint, frames, angular, body_a, body_b, h,
                                                    load) ||
                   ...);
        }

        /** @brief Folds the registration list into the linear dispatch. */
        template <typename... Kinds, typename T>
        inline void dispatch_joint_linear(JointKindList<Kinds...>, JointConstraintT<T>& joint,
                                          const JointWorldFrames<T>& frames,
                                          RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                          JointLoad<T>& load) noexcept
        {
            (void)(try_project_joint_linear<Kinds>(joint, frames, body_a, body_b, h, load) || ...);
        }

        /** @brief Folds the registration list into the velocity dispatch. */
        template <typename... Kinds, typename T>
        inline void dispatch_joint_velocities(JointKindList<Kinds...>, JointConstraintT<T>& joint,
                                              const JointWorldFrames<T>& frames,
                                              RigidBodyT<T>& body_a, RigidBodyT<T>& body_b, T h,
                                              JointLoad<T>& load) noexcept
        {
            (void)(try_project_joint_velocities<Kinds>(joint, frames, body_a, body_b, h, load) ||
                   ...);
        }

        /**
         * @brief Projects one joint's positional rows, whatever kind it is.
         *
         * Runs after the persistent distance constraints and before the contacts, and
         * that order is a decision rather than an accident: a joint is a *structural*
         * constraint and a contact is a *reactive* one, so an assembly is assembled
         * before it is pushed on. Both solvers walk this order and the conformance
         * suite is what keeps them walking the same one.
         *
         * ### Angular first, then linear — and the frames resolved twice
         *
         * The order is Müller et al. 2020 §3.4's: align the frames, then close the
         * attachment, so a substep ends with the anchors coincident rather than with an
         * alignment correction having just pulled them apart.
         *
         * The **re-resolution between the two groups is load-bearing, not tidiness**,
         * and it is worth recording why because the symptom was spectacular. An angular
         * correction rotates a body about its centre of mass, which *moves the
         * attachment point* by the lever arm. If the linear group then reads frames
         * resolved before that rotation, it corrects a gap that has already been partly
         * closed — over-correcting by the lever-arm share every substep, which the next
         * substep's angular row dutifully fixes, which the linear row then
         * over-corrects again. The loop gain is the positional row's angular share,
         * about three quarters for a door-shaped body, on top of a full angular
         * correction: above one. A hinged door built that way left the scene inside
         * twenty substeps, reporting a hinge load of two meganewtons on the way out.
         *
         * Within a group the staleness is harmless, and that is a property of the
         * groups rather than luck: a swing lock and a twist limit act about
         * perpendicular axes, and the three frame limits of the general joint act along
         * perpendicular axes. Rows that cannot see each other need not be ordered
         * against each other.
         *
         * Also where the tick's load accounting is reset. Folded into this projection
         * rather than given a node of its own because it is one branch on a value the
         * kernel has already loaded, and a separate node would be a whole graph node
         * per colour per tick to clear two vectors.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointProjectionT
        {
            /**
             * @param joint         The joint; its load accumulators are updated in place.
             * @param bodies        The body array the slots index; corrected in place.
             * @param h             The substep duration, in seconds.
             * @param first_substep Whether this is the tick's first substep.
             */
            void operator()(JointConstraintT<T>& joint, RigidBodyT<T>* bodies, T h,
                            bool first_substep) const noexcept
            {
                RigidBodyT<T>& body_a = bodies[joint.a];
                RigidBodyT<T>& body_b = bodies[joint.b];

                // A joint whose two bodies are both out of the solve — asleep, static,
                // or one of each — is left entirely alone, accumulators included. Not an
                // optimisation: clearing them would report a settled hinge as carrying
                // *nothing*, when what it is actually carrying is what it was carrying
                // on the last tick anybody computed. A door that hangs at rest reporting
                // zero load is a wrong answer, and it is the answer a reader gets from a
                // sleeping assembly unless the last live measurement survives. This is
                // the same rule §16.6 applies to a contact `End` event, which reports
                // what the contact carried on its last live tick.
                if (!is_simulated(body_a.flags) && !is_simulated(body_b.flags))
                    return;

                if (first_substep)
                {
                    joint.force_sum = Vector3T<T>{T(0), T(0), T(0)};
                    joint.torque_sum = Vector3T<T>{T(0), T(0), T(0)};
                    joint.peak_force = T(0);
                    joint.peak_torque = T(0);
                    joint.force_samples = 0;
                }
                if (!joint_is_active(joint.flags))
                    return;
                JointLoad<T> load{};

                {
                    const JointWorldFrames<T> frames =
                        resolve_joint_frames(joint, body_a, body_b);
                    const JointAngularState<T> angular = resolve_joint_angular_state(frames);
                    dispatch_joint_angular(JointKinds{}, joint, frames, angular, body_a, body_b,
                                           h, load);
                }
                {
                    const JointWorldFrames<T> frames =
                        resolve_joint_frames(joint, body_a, body_b);
                    dispatch_joint_linear(JointKinds{}, joint, frames, body_a, body_b, h, load);
                }

                fold_joint_load(joint, load);
                ++joint.force_samples;
            }
        };

        /**
         * @brief Projects one joint's velocity rows: rate drives, friction, damping.
         *
         * After `update_velocity`, for the same reason the contact velocity pass is
         * there: until the substep's pose change has been read back as a velocity
         * there is no rate for a rate drive to be a statement about.
         *
         * Folds its impulses into the *same* substep's load sums, which the positional
         * pass has already counted a sample for — so a hinge's friction torque appears
         * in its reported load, which is what makes a seized joint's readout say so.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointVelocityProjectionT
        {
            /**
             * @param joint  The joint; its load accumulators are updated in place.
             * @param bodies The body array the slots index; velocities updated in place.
             * @param h      The substep duration, in seconds.
             */
            void operator()(JointConstraintT<T>& joint, RigidBodyT<T>* bodies,
                            T h) const noexcept
            {
                if (!joint_is_active(joint.flags))
                    return;

                RigidBodyT<T>& body_a = bodies[joint.a];
                RigidBodyT<T>& body_b = bodies[joint.b];
                const JointWorldFrames<T> frames = resolve_joint_frames(joint, body_a, body_b);

                JointLoad<T> load{};
                dispatch_joint_velocities(JointKinds{}, joint, frames, body_a, body_b, h, load);

                fold_joint_load(joint, load);
            }
        };

        /** @brief The boundary joint projection: @ref JointProjectionT fixed to `Scalar`. */
        using JointProjection = JointProjectionT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
