/**************************************************************************/
/* ik_full_body.hpp                                                       */
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
 * @file ik_full_body.hpp
 * @brief A general-purpose, multiple-simultaneous-effector CCD solver (design §12.4).
 *
 * The four shipped solvers (`ik_two_bone.hpp`, `ik_look_at.hpp`, `ik_chain.hpp`,
 * `ik_foot_placement.hpp`) are each scoped to a fixed chain shape (a two-bone limb, a
 * look-at cone, one FABRIK chain, one foot). §12.4 named the gap: "No general-purpose
 * FBIK for arbitrary pose retargeting under constraints." `FullBodyIk` is that
 * general case — an arbitrary set of end effectors (hands, feet, head, whatever a
 * caller names), each pulling toward its own target, all solved from a shared root
 * joint (the pelvis/hips, typically) via Cyclic Coordinate Descent (CCD): for each
 * effector, walk its ancestor chain from the joint nearest the tip up to the shared
 * root, rotating each joint in turn to reduce that effector's tip-to-target error,
 * repeated for `iterations` full passes over every effector.
 *
 * This is a real, working general solver, not a stub — but it is also a simpler
 * algorithm than a production "Full Body IK" package (Unity FinalIK, Unreal's Control
 * Rig full-body solves): no joint limits/constraints, no priority weighting between
 * effectors sharing an ancestor (an effector solved later in `effectors` can undo
 * an earlier one's adjustment to a shared joint — CCD's known limitation, mitigated
 * only by running enough `iterations` for the pass to settle), and it recomposes the
 * *whole* skeleton after every single joint rotation (correct, not batched/optimized
 * — the same "correctness first, device path is a device-batched follow-up" choice
 * every other solver in this stack makes). A caller with genuinely conflicting
 * effectors (e.g. both hands sharing the spine) should order or weight them
 * deliberately; this solver does not resolve that automatically.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief One end effector this solver pulls toward a target. */
        struct FullBodyEffector
        {
            std::uint32_t tip_joint = 0;   /**< The joint whose position is driven to @c target. */
            Vector3 target{0.0, 0.0, 0.0}; /**< Object-space goal for @c tip_joint. */
            float weight = 1.0f;           /**< Per-effector blend, in [0, 1]; 0 skips it. */
        };

        /**
         * @brief Multi-effector CCD IK: several simultaneous targets solved from one shared root.
         */
        class FullBodyIk : public IPoseModifier
        {
            public:
                std::vector<FullBodyEffector> effectors; /**< Every effector this solve pulls toward. */
                std::uint32_t root_joint = 0; /**< CCD never rotates past this joint (e.g. the hips). */
                std::uint32_t iterations = 10; /**< Full passes over every effector. */
                float weight = 1.0f;           /**< Overall blend, in [0, 1]; 0 disables the solver. */

                void solve(PoseModifierContext& context) const override
                {
                    if (weight <= 0.0f || effectors.empty())
                        return;

                    for (std::uint32_t iteration = 0; iteration < iterations; ++iteration)
                        for (const FullBodyEffector& effector : effectors)
                            solve_one_effector(context, effector);
                }

            private:
                /**
                 * @brief One CCD pass for one effector: walks its ancestor chain from the
                 * joint nearest the tip up to @ref root_joint, rotating each in turn.
                 */
                void solve_one_effector(PoseModifierContext& context,
                                        const FullBodyEffector& effector) const
                {
                    if (effector.weight <= 0.0f || effector.tip_joint >= context.skeleton.joint_count)
                        return;

                    std::uint32_t joint = context.skeleton.parents[effector.tip_joint];
                    while (joint != NO_PARENT)
                    {
                        rotate_toward(context, joint, effector);
                        if (joint == root_joint)
                            break;
                        joint = context.skeleton.parents[joint];
                    }
                }

                /**
                 * @brief Rotates @p joint so the effector's tip moves toward its target,
                 * pivoting about @p joint's current object-space position.
                 */
                void rotate_toward(PoseModifierContext& context, std::uint32_t joint,
                                   const FullBodyEffector& effector) const
                {
                    const Vector3 pivot = context.position(joint);
                    const Vector3 tip = context.position(effector.tip_joint);
                    const Vector3 to_tip = tip - pivot;
                    const Vector3 to_target = effector.target - pivot;
                    const Scalar tip_length = length(to_tip);
                    const Scalar target_length = length(to_target);
                    if (tip_length <= Scalar(1e-6) || target_length <= Scalar(1e-6))
                        return;

                    const Vector3 tip_direction = to_tip * (Scalar(1) / tip_length);
                    const Vector3 target_direction = to_target * (Scalar(1) / target_length);
                    const Scalar cos_angle = std::min(
                        Scalar(1), std::max(Scalar(-1), dot(tip_direction, target_direction)));
                    Vector3 axis = cross(tip_direction, target_direction);
                    const Scalar axis_length = length(axis);
                    if (axis_length <= Scalar(1e-6))
                        return; // already aligned (or exactly opposed — no well-defined axis)
                    axis = axis * (Scalar(1) / axis_length);

                    const Scalar angle = std::acos(static_cast<double>(cos_angle)) *
                                         static_cast<Scalar>(effector.weight) *
                                         static_cast<Scalar>(weight);
                    if (angle <= Scalar(1e-6))
                        return;

                    const Quaternion delta = quaternion_axis_angle(axis, angle);
                    const Quaternion current_world = context.rotation(joint);
                    context.set_rotation(joint, mul(delta, current_world));
                    // Every later joint on this (and any other) effector's chain reads
                    // context.position()/rotation() next, so the pose must be current before
                    // the next rotate_toward call — the whole-skeleton cost this header's
                    // comment already documents as a deliberate correctness-first choice.
                    context.recompose();
                }
        };
    } // namespace Animation
} // namespace SushiEngine
