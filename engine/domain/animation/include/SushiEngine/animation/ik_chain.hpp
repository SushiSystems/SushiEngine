/**************************************************************************/
/* ik_chain.hpp                                                          */
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
 * @file ik_chain.hpp
 * @brief The FABRIK chain IK solver — tails, tentacles, cables (phase A6).
 *
 * FABRIK (Forward And Backward Reaching Inverse Kinematics) solves an arbitrarily long joint
 * chain to a target by alternating a backward pass (pin the tip to the target, walk to the root
 * keeping bone lengths) and a forward pass (pin the root back, walk to the tip). It is
 * iteration-capped by @ref iterations (the tier knob, design §5.3) so a long chain never blows
 * the frame budget. The solved joint positions are written back as bone-aligning rotations, and
 * a per-solver weight blends the correction in and out.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Max joints one FABRIK chain solves. */
        constexpr std::uint32_t MAX_CHAIN_JOINTS = 16;

        /**
         * @brief FABRIK IK over a joint chain (root → tip) reaching for @ref target.
         *
         * @ref joints lists the chain root to tip; @ref iterations caps the passes; @ref weight
         * blends the correction. Bone lengths are read from the current pose each solve.
         */
        class ChainIk : public IPoseModifier
        {
            public:
                std::uint32_t joints[MAX_CHAIN_JOINTS] = {0};
                std::uint32_t joint_count = 0; /**< Joints in the chain (>= 2). */
                Vector3 target{0.0, 0.0, 0.0}; /**< Object-space goal for the tip. */
                std::uint32_t iterations = 8;  /**< Max forward/backward passes (tier knob). */
                float tolerance = 1e-4f;       /**< Stop when the tip is within this of the target. */
                float weight = 1.0f;           /**< Blend of the correction, in [0, 1]. */

                void solve(PoseModifierContext& context) const override
                {
                    if (weight <= 0.0f || joint_count < 2 || joint_count > MAX_CHAIN_JOINTS)
                        return;

                    Vector3 point[MAX_CHAIN_JOINTS];
                    Scalar bone[MAX_CHAIN_JOINTS];
                    Scalar total_length = 0.0;
                    for (std::uint32_t i = 0; i < joint_count; ++i)
                        point[i] = context.position(joints[i]);
                    for (std::uint32_t i = 0; i + 1 < joint_count; ++i)
                    {
                        bone[i] = length(point[i + 1] - point[i]);
                        total_length += bone[i];
                    }
                    if (total_length < static_cast<Scalar>(1e-6))
                        return;

                    const Vector3 root = point[0];
                    const Vector3 to_target = target - root;
                    if (length(to_target) >= total_length)
                    {
                        // Out of reach: stretch straight toward the target.
                        const Vector3 direction = normalize(to_target);
                        for (std::uint32_t i = 0; i + 1 < joint_count; ++i)
                            point[i + 1] = point[i] + direction * bone[i];
                    }
                    else
                    {
                        const std::uint32_t last = joint_count - 1;
                        for (std::uint32_t iter = 0; iter < iterations; ++iter)
                        {
                            if (length(point[last] - target) < static_cast<Scalar>(tolerance))
                                break;
                            // Backward: pin the tip to the target, walk to the root.
                            point[last] = target;
                            for (std::uint32_t i = last; i > 0; --i)
                            {
                                const Scalar r = length(point[i] - point[i - 1]);
                                const Scalar lambda = r > static_cast<Scalar>(1e-8) ? bone[i - 1] / r : 0.0;
                                point[i - 1] = point[i] * (static_cast<Scalar>(1.0) - lambda) +
                                               point[i - 1] * lambda;
                            }
                            // Forward: pin the root back, walk to the tip.
                            point[0] = root;
                            for (std::uint32_t i = 0; i + 1 < joint_count; ++i)
                            {
                                const Scalar r = length(point[i + 1] - point[i]);
                                const Scalar lambda = r > static_cast<Scalar>(1e-8) ? bone[i] / r : 0.0;
                                point[i + 1] = point[i] * (static_cast<Scalar>(1.0) - lambda) +
                                               point[i + 1] * lambda;
                            }
                        }
                    }

                    // Write the solved positions back as bone-aligning rotations, root to tip.
                    for (std::uint32_t i = 0; i + 1 < joint_count; ++i)
                    {
                        const Vector3 current = context.position(joints[i + 1]) - context.position(joints[i]);
                        const Vector3 solved = point[i + 1] - point[i];
                        if (length(current) < static_cast<Scalar>(1e-6) ||
                            length(solved) < static_cast<Scalar>(1e-6))
                            continue;
                        Quaternion delta =
                            detail::rotation_between(normalize(current), normalize(solved));
                        if (weight < 1.0f)
                        {
                            const Quaternion identity{0.0, 0.0, 0.0, 1.0};
                            delta = slerp(identity, delta, static_cast<Scalar>(weight));
                        }
                        context.set_rotation(joints[i], mul(delta, context.rotation(joints[i])));
                        context.recompose();
                    }
                }
        };
    } // namespace Animation
} // namespace SushiEngine
