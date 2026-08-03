/**************************************************************************/
/* ik_look_at.hpp                                                        */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file ik_look_at.hpp
 * @brief The look-at IK solver — a chain aims at a target (phase A6).
 *
 * Aims the tip joint's forward axis at a target by rotating a chain of joints (spine → chest →
 * head → eyes), distributing the aim across them by per-joint weights so the turn spreads
 * naturally instead of snapping one bone. Because the distributed rotations share one axis, the
 * tip's forward lands exactly on the target (their composition is the full aim rotation), while
 * the turn is clamped to a cone so the head never over-rotates — Unity's `SetLookAtPosition`
 * semantics (design §5.3).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Max joints a look-at chain distributes its aim across. */
        constexpr std::uint32_t MAX_LOOK_AT_JOINTS = 8;

        /**
         * @brief Look-at IK: aims @ref forward_axis of the chain's tip at @ref target.
         *
         * The chain runs base → tip in @ref joints (the tip is the last). Each joint takes a
         * share of the aim by its @ref weights entry; the total turn is capped at @ref cone.
         */
        class LookAtIk : public IPoseModifier
        {
            public:
                std::uint32_t joints[MAX_LOOK_AT_JOINTS] = {0};
                float weights[MAX_LOOK_AT_JOINTS] = {0};
                std::uint32_t joint_count = 0;      /**< Joints in the chain (tip is the last). */
                Vector3 forward_axis{0.0, 0.0, 1.0}; /**< The tip's local axis that should aim. */
                Vector3 target{0.0, 0.0, 0.0};       /**< Object-space point to look at. */
                float cone = 3.14159265358979323846f; /**< Max aim angle (radians). */
                float weight = 1.0f;                 /**< Blend of the correction, in [0, 1]. */

                void solve(PoseModifierContext& context) const override
                {
                    if (weight <= 0.0f || joint_count == 0)
                        return;
                    const std::uint32_t tip = joints[joint_count - 1];

                    const Quaternion tip_rotation = context.rotation(tip);
                    const Vector3 current = normalize(rotate(tip_rotation, forward_axis));
                    const Vector3 to_target = target - context.position(tip);
                    if (length(to_target) < static_cast<Scalar>(1e-6))
                        return;
                    const Vector3 desired = normalize(to_target);

                    const Quaternion aim = detail::rotation_between(current, desired);
                    Scalar angle = static_cast<Scalar>(2.0) *
                                   std::acos(std::min(static_cast<Scalar>(1.0),
                                                      std::max(static_cast<Scalar>(-1.0), aim.w)));
                    if (angle < static_cast<Scalar>(1e-6))
                        return;
                    Vector3 axis{aim.x, aim.y, aim.z};
                    if (length(axis) < static_cast<Scalar>(1e-6))
                        return;
                    axis = normalize(axis);

                    Scalar total = std::min(angle, static_cast<Scalar>(cone)) *
                                   static_cast<Scalar>(weight);

                    float weight_sum = 0.0f;
                    for (std::uint32_t i = 0; i < joint_count; ++i)
                        weight_sum += weights[i];
                    if (weight_sum <= 0.0f)
                        return;

                    // Apply each joint's share about the shared aim axis, base to tip, so their
                    // composition turns the tip's forward by `total` exactly.
                    for (std::uint32_t i = 0; i < joint_count; ++i)
                    {
                        const Scalar share = total * static_cast<Scalar>(weights[i] / weight_sum);
                        if (std::fabs(share) < static_cast<Scalar>(1e-7))
                            continue;
                        const Quaternion delta = quaternion_axis_angle(axis, share);
                        const std::uint32_t joint = joints[i];
                        context.set_rotation(joint, mul(delta, context.rotation(joint)));
                        context.recompose();
                    }
                }
        };
    } // namespace Animation
} // namespace SushiEngine
