/**************************************************************************/
/* joint.hpp                                                              */
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
 * @file joint.hpp
 * @brief The joint descriptor: one POD for every kind in §10.1.
 *
 * A joint is "where and how are these two bodies attached", and §10.1 says that
 * uniformly: **two joint frames**, one in each body's local space. Everything a
 * joint kind differs in — which degrees of freedom it removes, which it limits,
 * which it drives — is then a statement about those two frames, so one descriptor
 * carries them all and the *kind* selects which statements are made.
 *
 * ### Why one descriptor rather than one per kind
 *
 * §4.2 requires that a new joint add a descriptor, a projection, and one
 * registration line, and touch nothing else. That rule is about the *projection*
 * dispatch, which `joint_projection.hpp` satisfies by folding a registration list;
 * it is not a requirement that each kind own a distinct buffer. And distinct buffers
 * would cost real structure: the solve graph emits one node per kind per colour per
 * substep, so eight joint kinds would multiply the compiled node count eightfold to
 * express a difference that lives in eight lines of arithmetic. The kinds share a
 * descriptor and a band, and diverge inside one kernel.
 *
 * The cost is honest and worth stating: every joint carries the fields every kind
 * might need, so a ball joint stores a linear limit it never reads. A joint
 * descriptor is a few hundred bytes and a scene holds hundreds of them, not
 * millions — this is the one place in the physics where clarity is cheaper than
 * packing.
 *
 * ### The frame convention: x is the axis
 *
 * A joint frame's **local x axis is its primary axis** — the hinge's rotation axis,
 * the slider's travel axis, the cone-twist's twist axis. Fixed once, here, because
 * the whole angular vocabulary (`joint_primitives.hpp`'s swing/twist decomposition)
 * is written against a single named axis, and a convention that varied per kind
 * would mean each kind re-deriving which of three axes it meant.
 *
 * ### What the solver writes back
 *
 * @ref JointConstraintT::force_sum and @ref JointConstraintT::torque_sum are
 * *outputs*. XPBD gives Lagrange multipliers rather than forces, and §10.4's
 * recovery — `force = λ n / h²` — is exact and costs nothing beyond the addition, so
 * every row a joint projects folds its share in. That single quantity is what
 * delivers break thresholds, the load readout that is the rigid-body half of
 * *mukavemet*, and motor-effort feedback for a drivetrain.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Which degrees of freedom a joint removes, limits, and drives.
         *
         * A 32-bit underlying type rather than the smallest that fits, because this
         * sits in a device-resident POD beside `std::uint32_t` fields and a narrower
         * enum would buy three bytes of padding rather than three bytes.
         */
        enum class JointKind : std::uint32_t
        {
            /** @brief All six degrees of freedom. A compliant one is a flexible weld. */
            Fixed = 0,

            /** @brief The three translational ones; rotation is free (§10.1's ball). */
            Ball = 1,

            /**
             * @brief Translation plus two rotations: one axis of rotation survives.
             *
             * The car door. Its surviving rotation is limited by @ref
             * JointConstraintT::twist_limit and driven by @ref
             * JointConstraintT::motor.
             */
            Hinge = 2,

            /**
             * @brief All three rotations plus two translations: travel along one axis.
             *
             * Suspension travel. The surviving translation is limited by @ref
             * JointConstraintT::linear_limit.
             */
            Slider = 3,

            /**
             * @brief A range along the line between the two anchors.
             *
             * The generalization of `XpbdDistanceConstraintT`: a minimum and a
             * maximum rather than a single rest length, so a rope goes slack and a
             * strut resists both ways.
             */
            Distance = 4,

            /**
             * @brief Translation removed; rotation bounded by a swing cone and a twist range.
             *
             * Ragdolls. A shoulder is a cone and a forearm is a hinge, and the
             * difference between them is two numbers rather than two joint types.
             */
            ConeTwist = 5,

            /**
             * @brief The general case: every axis free, limited, or locked.
             *
             * Reached for when none of the named kinds says it. Locking an axis is
             * the degenerate limit `lower == upper`, so this needs no separate
             * per-axis mode word.
             */
            SixDegreeOfFreedom = 6
        };

        /** @brief How many kinds @ref JointKind names; the dispatch fold's bound. */
        inline constexpr std::size_t JOINT_KIND_COUNT = 7;

        /** @brief Bit flags on a joint, orthogonal to its kind. */
        namespace JointFlags
        {
            /** @brief Nothing set. */
            inline constexpr std::uint32_t none = 0u;

            /**
             * @brief Whether the joint is projected at all.
             *
             * A disabled joint keeps its slot, its handle and its frames and is
             * simply skipped — which is what the editor needs to let an author
             * silence a joint without destroying the authoring it holds, and what a
             * mechanism needs to release a latch for a tick.
             */
            inline constexpr std::uint32_t enabled = 1u << 0;

            /**
             * @brief Whether the joint has already exceeded a break threshold.
             *
             * Set by the scene, read by the projection, which skips a broken joint.
             * The joint is then removed at the step boundary — a topology change
             * never happens against a running graph (§6.6) — so the flag is what
             * covers the gap between "it broke" and "it is gone".
             */
            inline constexpr std::uint32_t broken = 1u << 1;
        } // namespace JointFlags

        /** @brief Whether @p flags describes a joint the solver should project. */
        inline bool joint_is_active(std::uint32_t flags) noexcept
        {
            return (flags & JointFlags::enabled) != 0 && (flags & JointFlags::broken) == 0;
        }

        /**
         * @brief A bound on one degree of freedom, projected only when violated.
         *
         * An inequality pair rather than two separate limits, because the pair is
         * what a limit *is*: a range the coordinate must stay inside. `lower ==
         * upper` locks the axis, `lower > upper` is an empty range and is rejected by
         * @ref joint_limit_violation rather than being treated as a lock, and a
         * disabled limit is free.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointLimitT
        {
            /** @brief The lowest value the coordinate may take (radians or metres). */
            T lower = 0;

            /** @brief The highest value the coordinate may take (radians or metres). */
            T upper = 0;

            /**
             * @brief XPBD compliance of the limit row, so a hard stop can have give.
             *
             * Zero is a rigid stop. A positive value is a bumper — which is what a
             * suspension's bump stop and a door seal actually are, and expressing
             * them as compliance rather than as a spring keeps them step-size
             * independent like everything else here.
             */
            T compliance = 0;

            /** @brief Whether this limit is enforced; a disabled limit leaves the axis free. */
            bool enabled = false;
        };

        /** @brief What a motor is asked to hold: nothing, a position, or a speed. */
        enum class JointMotorMode : std::uint32_t
        {
            /** @brief No drive. The default, so a joint built by hand is not powered. */
            Disabled = 0,

            /**
             * @brief Hold the coordinate at a target, positionally.
             *
             * A servo. Projected with the positional rows, so its stiffness is the
             * drive compliance and is independent of the substep count.
             */
            Position = 1,

            /**
             * @brief Hold the coordinate's rate at a target, in the velocity pass.
             *
             * A wheel motor, and — with a target of zero — joint *friction*: a hinge
             * that does not swing free is a velocity drive toward standstill with a
             * small force limit. Velocity-level because a rate is not a position, and
             * expressing it positionally would mean remembering where the coordinate
             * was when the substep began.
             */
            Velocity = 2
        };

        /**
         * @brief A drive on a joint's primary axis, with a saturation limit.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointMotorT
        {
            /** @brief The target: an angle or a distance in @ref JointMotorMode::Position,
             *         an angular or linear rate in @ref JointMotorMode::Velocity. */
            T target = 0;

            /**
             * @brief The most force (N) or torque (N·m) the drive may spend.
             *
             * At or below zero means **unsaturated** — an ideal drive that holds its
             * target whatever it costs. That reading is deliberate rather than
             * convenient: a saturated drive is the interesting case and its limit is
             * always authored, while an ideal drive is the one an author reaches for
             * when they have no number in mind. A default-constructed motor is
             * @ref JointMotorMode::Disabled, so this never means "an ideal drive
             * nobody asked for".
             */
            T max_force = 0;

            /** @brief XPBD compliance of the drive row; zero is an infinitely stiff servo. */
            T compliance = 0;

            /** @brief What the drive holds. */
            JointMotorMode mode = JointMotorMode::Disabled;
        };

        /**
         * @brief How far outside its range a coordinate has strayed.
         *
         * The one arithmetic every limit in the library shares, so it is written
         * once: the signed amount by which @p value must be *reduced* to re-enter
         * `[lower, upper]`. Zero when the limit is disabled, when the range is empty,
         * or when the coordinate is inside — and a caller that projects a zero
         * violation does no work, so no separate "is it violated" test is needed.
         *
         * @tparam T The scalar element type.
         * @param limit The bound.
         * @param value The coordinate's current value.
         * @return The signed excess; positive above @p upper, negative below @p lower.
         */
        template <typename T>
        inline T joint_limit_violation(const JointLimitT<T>& limit, T value) noexcept
        {
            if (!limit.enabled || limit.lower > limit.upper)
                return T(0);
            if (value > limit.upper)
                return value - limit.upper;
            if (value < limit.lower)
                return value - limit.lower;
            return T(0);
        }

        /**
         * @brief A joint frame whose local x axis is @p axis.
         *
         * The shortest rotation taking `(1, 0, 0)` to @p axis. Shortest rather than
         * arbitrary because the two frames of a hinge are usually authored from two
         * axes and nothing else, and the perpendicular reference the twist angle is
         * measured from then falls out of the same rule on both sides — so a hinge
         * built this way reads zero twist when the bodies are in their authored
         * relative pose, which is what an author means by "the door is shut".
         *
         * @tparam T The scalar element type.
         * @param axis The primary axis, in the body's local space; normalized here.
         * @return The frame's local orientation.
         */
        template <typename T>
        inline QuaternionT<T> joint_frame_from_axis(const Vector3T<T>& axis) noexcept
        {
            const T len = length(axis);
            if (!(len > T(0)))
                return QuaternionT<T>{T(0), T(0), T(0), T(1)};
            const Vector3T<T> to = axis * (T(1) / len);
            const Vector3T<T> from{T(1), T(0), T(0)};
            const T d = dot(from, to);

            // Antiparallel: the shortest rotation is a half turn and its axis is not
            // determined by the cross product, which vanishes. Any perpendicular axis
            // is as short as any other, so one is chosen deterministically rather
            // than left to whatever the normalization of a zero vector produces.
            if (d < T(-1) + T(1e-9))
                return QuaternionT<T>{T(0), T(0), T(1), T(0)};

            const Vector3T<T> c = cross(from, to);
            return normalize(QuaternionT<T>{c.x, c.y, c.z, T(1) + d});
        }

        /**
         * @brief One joint: two bodies, two frames, and what is held between them.
         *
         * Exposes `a`/`b` in the shape `color_constraints` and `ConstraintStore`
         * expect, so a joint colours against distance constraints and contacts in one
         * union without the colourer knowing joints exist (§6.3).
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        struct JointConstraintT
        {
            /** @brief The scalar element type, so a solver can derive its precision. */
            using Real = T;

            /** @brief First body slot. */
            std::uint32_t a = 0;

            /** @brief Second body slot. */
            std::uint32_t b = 0;

            /** @brief @ref JointFlags bits. */
            std::uint32_t flags = JointFlags::enabled;

            /** @brief Which kind's arithmetic projects this joint. */
            JointKind kind = JointKind::Fixed;

            /** @brief The attachment point on body @c a, in its local frame. */
            Vector3T<T> local_anchor_a;

            /** @brief The attachment point on body @c b, in its local frame. */
            Vector3T<T> local_anchor_b;

            /** @brief Body @c a's joint frame orientation, local; its x axis is the primary axis. */
            QuaternionT<T> local_basis_a{};

            /** @brief Body @c b's joint frame orientation, local; its x axis is the primary axis. */
            QuaternionT<T> local_basis_b{};

            /**
             * @brief XPBD compliance of the joint's *structural* rows.
             *
             * The attachment and the removed rotations, not the limits or the drive —
             * those carry their own, because "the hinge itself has give" and "the
             * hard stop has give" are different authoring statements and a single
             * number could not say both.
             */
            T compliance = 0;

            /** @brief Travel along the primary axis, in metres (slider, distance, 6-DOF). */
            JointLimitT<T> linear_limit;

            /**
             * @brief Travel along the frame's y axis, in metres. Read only by 6-DOF.
             *
             * The named kinds lock the two perpendicular translations structurally,
             * so they have no use for these; the general kind is the one that needs
             * to say "free along y, limited along z". Two extra ranges in the
             * descriptor rather than an array, because a kernel indexing an array of
             * limits by a loop variable and a kernel reading three named fields
             * generate the same code, and only one of them reads as what it is.
             */
            JointLimitT<T> linear_limit_y;

            /** @brief Travel along the frame's z axis, in metres. Read only by 6-DOF. */
            JointLimitT<T> linear_limit_z;

            /** @brief Rotation about the primary axis, in radians (hinge, cone-twist, 6-DOF). */
            JointLimitT<T> twist_limit;

            /**
             * @brief The cone half-angle the primary axis may stray by, in radians.
             *
             * Only @ref JointLimitT::upper is read: a swing is an unsigned angle off
             * an axis, so a lower bound on it would be a statement that the joint
             * must stay *bent*, which no mechanism means. Stated here rather than
             * discovered by an author whose lower bound did nothing.
             */
            JointLimitT<T> swing_limit;

            /** @brief The drive on the primary axis. */
            JointMotorT<T> motor;

            /**
             * @brief Force (N) above which the joint breaks; zero is unbreakable.
             *
             * Compared against @ref peak_force, the largest load any single substep of
             * the tick put through the joint — **not** against the mean.
             *
             * That distinction is the whole difference between a threshold that works
             * and one that never fires, and it is worth recording because the mean was
             * tried first and looked more principled. A hard impact is a large
             * separation the constraint closes in one substep and then *overshoots*,
             * so the next substep's correction points the other way and is very nearly
             * as large. The mean of the two is almost zero: a door yanked two metres
             * off its hinge and snapped back reported 344 N — its own resting weight —
             * while the substeps either side of the snap carried sixteen meganewtons.
             * Averaging a load whose direction reverses measures the *net* pull, and
             * what tears a mount out is the magnitude.
             *
             * The mean is not noisier than the peak for a joint at rest either, which
             * was the worry: a resting hinge's peak and mean differ by about a
             * newton in three hundred, because nothing in a settled joint spikes.
             */
            T break_force = 0;

            /** @brief Torque (N·m) above which the joint breaks; zero is unbreakable. */
            T break_torque = 0;

            /**
             * @brief Sum over the tick's substeps of the force this joint's rows carried.
             *
             * An output. Divide by @ref force_samples for the mean; see
             * @ref joint_force.
             */
            Vector3T<T> force_sum;

            /** @brief Sum over the tick's substeps of the torque this joint's rows carried. */
            Vector3T<T> torque_sum;

            /**
             * @brief The largest force magnitude a single substep of the tick carried.
             *
             * An output, and the quantity @ref break_force is measured against. Kept
             * beside the vector sums rather than instead of them because the two answer
             * different questions: the mean vector says *which way and how hard this
             * mount is being pulled*, which is the load readout an inspector shows, and
             * the peak says *what the worst instant was*, which is what decides whether
             * the mount is still there.
             */
            T peak_force = 0;

            /** @brief The largest torque magnitude a single substep of the tick carried. */
            T peak_torque = 0;

            /**
             * @brief How many substeps contributed to the two sums.
             *
             * Carried in the descriptor rather than inferred from the substep count,
             * because the two can differ: a joint added mid-tick, disabled for part
             * of one, or skipped because both its bodies were asleep contributed to
             * fewer substeps than the tick ran, and dividing by the tick's count
             * would report a load lower than the one the joint actually carried.
             */
            std::uint32_t force_samples = 0;

            /** @brief Padding, so the struct's size is the same in every translation unit. */
            std::uint32_t reserved_ = 0;
        };

        /**
         * @brief The mean force a joint carried over the last tick, in newtons.
         *
         * §10.4's recovery, read out. Zero before the joint has been stepped, which
         * is the honest answer rather than a division by zero.
         *
         * @tparam T The scalar element type.
         * @param joint The joint to read.
         */
        template <typename T>
        inline Vector3T<T> joint_force(const JointConstraintT<T>& joint) noexcept
        {
            if (joint.force_samples == 0)
                return Vector3T<T>{T(0), T(0), T(0)};
            return joint.force_sum * (T(1) / T(joint.force_samples));
        }

        /** @brief The mean torque a joint carried over the last tick, in newton-metres. */
        template <typename T>
        inline Vector3T<T> joint_torque(const JointConstraintT<T>& joint) noexcept
        {
            if (joint.force_samples == 0)
                return Vector3T<T>{T(0), T(0), T(0)};
            return joint.torque_sum * (T(1) / T(joint.force_samples));
        }

        /**
         * @brief Whether a joint's load has passed either break threshold.
         *
         * Both thresholds, one test, because a joint has one lifetime: a mount that
         * shears under torque and one that pulls out under load are the same event to
         * everything downstream of it.
         *
         * Measured against the *peak* substep load rather than the mean, for the reason
         * @ref JointConstraintT::break_force gives at length: a mean over a load whose
         * direction reverses measures the net pull, and what tears a mount out is the
         * magnitude.
         *
         * @tparam T The scalar element type.
         * @param joint The joint to test, after a step.
         * @return True when the joint should be broken.
         */
        template <typename T>
        inline bool joint_should_break(const JointConstraintT<T>& joint) noexcept
        {
            if (joint.force_samples == 0)
                return false;
            if (joint.break_force > T(0) && joint.peak_force > joint.break_force)
                return true;
            return joint.break_torque > T(0) && joint.peak_torque > joint.break_torque;
        }

        /**
         * @brief The boundary joint: @ref JointConstraintT fixed to `Scalar`.
         */
        using JointConstraint = JointConstraintT<Scalar>;

        /** @brief The boundary joint limit. */
        using JointLimit = JointLimitT<Scalar>;

        /** @brief The boundary joint motor. */
        using JointMotor = JointMotorT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
