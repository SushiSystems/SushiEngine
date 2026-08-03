/**************************************************************************/
/* joint_params.hpp                                                       */
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
 * @file joint_params.hpp
 * @brief The joint vocabulary at the boundary: what is held, and what it carries.
 *
 * A mirror of `Physics::JointKind` and its descriptor rather than a using-declaration
 * of them, for the reason `RigidBodyDescription` is not `Physics::RigidBodyT`: this is
 * boundary vocabulary and names no solver type, so a gameplay system that creates a
 * hinge does not thereby depend on the solver that projects one.
 *
 * ### Why this is its own header
 *
 * These types are named by *three* layers that must not depend on each other. The
 * physics seam (`physics_services.hpp`) creates joints from them; the authoring
 * boundary (`simulation.hpp`) stores them on an entity; the assembly asset
 * (`physics_assembly.hpp`) carries them against part indices it has not yet resolved
 * to entities. The seam already includes the boundary, so the vocabulary cannot live
 * in the seam without the boundary reaching upward for it — and duplicating it is how
 * a new limit ends up honoured by a hand-built joint and silently ignored by an
 * authored one.
 *
 * Nothing here names an entity. That is what keeps this header includable from
 * `simulation.hpp`, which is where `EntityId` is defined, and it is not a compromise:
 * *what is held between two bodies* is genuinely separable from *which two bodies*,
 * which is the same split @ref JointParameters makes for the assembly asset's sake.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Which degrees of freedom a joint removes, at the boundary. */
        enum class JointType : std::uint8_t
        {
            Fixed = 0,               /**< All six degrees; a compliant one is a flexible weld. */
            Ball = 1,                /**< Attached, rotation free. Limits make it a cone-twist. */
            Hinge = 2,               /**< One surviving rotation, limited and drivable. A door. */
            Slider = 3,              /**< One surviving translation. Suspension travel. */
            Distance = 4,            /**< A range along the line between the anchors. A rope. */
            ConeTwist = 5,           /**< Attached, with a swing cone and a twist range. Ragdolls. */
            SixDegreeOfFreedom = 6   /**< Every axis free, limited, or locked. The general case. */
        };

        /** @brief How many kinds @ref JointType names; the bound of any list drawn over them. */
        inline constexpr std::size_t JOINT_TYPE_COUNT = 7;

        /** @brief What a joint's drive holds: nothing, a coordinate, or its rate. */
        enum class JointMotorType : std::uint8_t
        {
            Disabled = 0,
            /** @brief A servo on the coordinate. */
            Position = 1,
            /** @brief A rate drive; with a target of zero and a small limit, joint friction. */
            Velocity = 2
        };

        /**
         * @brief A bound on one of a joint's degrees of freedom.
         *
         * `lower == upper` locks the axis and a disabled limit leaves it free, so
         * free/limited/locked are three readings of one range rather than a mode word
         * that could disagree with the numbers.
         */
        struct JointLimitDescription
        {
            Scalar lower = 0;      /**< Radians for an angle, metres for a translation. */
            Scalar upper = 0;
            Scalar compliance = 0; /**< Zero is a rigid stop; positive is a bumper. */
            bool enabled = false;
        };

        /** @brief A drive on a joint's primary axis, with a saturation limit. */
        struct JointMotorDescription
        {
            JointMotorType type = JointMotorType::Disabled;
            Scalar target = 0;     /**< A coordinate, or a rate, per @ref type. */
            /** @brief N or N·m; at or below zero is an unsaturated, ideal drive. */
            Scalar max_force = 0;
            Scalar compliance = 0;

            /**
             * @brief Viscous resistance on the driven coordinate, as a rate in inverse seconds.
             *
             * The other half of a spring-damper drive: a position drive at a compliance is
             * a spring, and a spring alone rings forever. §11.2's suspension strut is one
             * joint rather than two because of this field, and a steering damper or a door
             * closer is this field with @ref type left @ref JointMotorType::Disabled — a
             * pure damper is a real mechanism, not a misconfiguration.
             *
             * A *rate*, so a strut damped at 8 s⁻¹ damps the same amount per second
             * whatever the substep count.
             */
            Scalar damping = 0;
        };

        /**
         * @brief What is held between a joint's two endpoints, whoever they turn out to be.
         *
         * Split from the endpoints deliberately. A joint is *two bodies plus what is
         * held between them*, and the second half is authored in places that do not yet
         * know the first: an assembly asset (§10.2) describes its joints against **part
         * indices** and only learns which entities those parts became when it is
         * instanced. Keeping the parameters in their own value is what lets the asset
         * carry the joint vocabulary rather than a copy of it — and a copy is how a new
         * parameter ends up honoured by a hand-built joint and silently ignored by an
         * assembled one.
         *
         * The two axes are the joint's *primary* axis in each body's local space: the
         * hinge's rotation axis, the slider's travel axis, the cone-twist's twist axis.
         * The frame each implies is the shortest rotation onto that axis, computed the
         * same way on both sides, so a joint whose bodies are in their authored relative
         * pose reads a twist angle of zero — which is what an author means by "the door
         * is shut".
         */
        struct JointParameters
        {
            JointType type = JointType::Fixed;

            Vector3 anchor_a;                    /**< Attachment point on the first body, local. */
            Vector3 anchor_b;                    /**< Attachment point on the second body, local. */
            Vector3 axis_a{Vector3{1, 0, 0}};    /**< Primary axis on the first body, local. */
            Vector3 axis_b{Vector3{1, 0, 0}};    /**< Primary axis on the second body, local. */

            /** @brief Compliance of the structural rows; zero is rigid. */
            Scalar compliance = 0;

            /** @brief Travel along the primary axis (slider), or the anchor range (distance). */
            JointLimitDescription linear_limit;

            /** @brief Rotation about the primary axis (hinge angle, cone-twist twist). */
            JointLimitDescription twist_limit;

            /** @brief The cone half-angle the primary axis may stray by; only `upper` is read. */
            JointLimitDescription swing_limit;

            /** @brief The drive on the primary axis. */
            JointMotorDescription motor;

            /** @brief Force (N) above which the joint breaks; zero is unbreakable. */
            Scalar break_force = 0;

            /** @brief Torque (N·m) above which the joint breaks; zero is unbreakable. */
            Scalar break_torque = 0;
        };

        /**
         * @brief What a joint is carrying, read back after a step.
         *
         * The rigid-body half of *mukavemet*: XPBD settles on Lagrange multipliers and
         * §10.4's recovery turns them into a force and a torque exactly, so "how much
         * load is this mount carrying" is a readout rather than an estimate. The mean
         * over the tick's substeps, not a peak — a single substep's multiplier during a
         * stiff transient is noise.
         */
        struct JointState
        {
            Vector3 force;   /**< Mean over the tick, in newtons, world space. */
            Vector3 torque;  /**< Mean over the tick, in newton-metres, world space. */

            /**
             * @brief The worst single substep's force magnitude, in newtons.
             *
             * What a break threshold is measured against, and what tells a scrape from
             * a crash. Reported alongside the mean rather than instead of it because
             * the two answer different questions: the mean says which way and how hard
             * the mount is being pulled, and the peak says what the worst instant was.
             * An impact's mean is nearly zero — the correction reverses direction the
             * substep after the hit — so a listener that watched only the mean would
             * never see the hit at all.
             */
            Scalar peak_force = 0;

            /** @brief The worst single substep's torque magnitude, in newton-metres. */
            Scalar peak_torque = 0;
        };
    } // namespace Simulation
} // namespace SushiEngine
