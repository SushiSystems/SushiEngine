/**************************************************************************/
/* ik_two_bone.hpp                                                       */
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
 * @file ik_two_bone.hpp
 * @brief The analytic two-bone IK solver — arms and legs (phase A6).
 *
 * A closed-form (law-of-cosines) solver for a three-joint chain (upper, mid, tip): it places
 * the mid joint on the circle where both bone lengths are satisfied, in the plane the pole
 * vector selects, so the tip reaches the target exactly when it is in range and the elbow/knee
 * bends toward the pole. Beyond reach it soft-clamps to full extension. A per-solver weight
 * blends the correction in and out. It edits only the upper and mid joints, so it composes with
 * any pose beneath it (design §5.3).
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
        /**
         * @brief Two-bone IK over a (upper → mid → tip) chain, pole-vector controlled.
         *
         * All positions are object space (the pose's model space). Configure the three joint
         * indices, the @ref target the tip reaches for, the @ref pole the mid bends toward, and
         * the @ref weight the correction blends by.
         */
        class TwoBoneIk : public IPoseModifier
        {
            public:
                std::uint32_t upper = 0; /**< Shoulder / hip joint. */
                std::uint32_t mid = 0;   /**< Elbow / knee joint. */
                std::uint32_t tip = 0;   /**< Wrist / ankle (end effector). */
                Vector3 target{0.0, 0.0, 0.0}; /**< Object-space goal for the tip. */
                Vector3 pole{0.0, 0.0, 1.0};   /**< Object-space hint the mid bends toward. */
                float weight = 1.0f;           /**< Blend of the correction, in [0, 1]. */

                void solve(PoseModifierContext& context) const override
                {
                    if (weight <= 0.0f)
                        return;

                    const Vector3 a = context.position(upper);
                    const Vector3 b0 = context.position(mid);
                    const Vector3 c0 = context.position(tip);
                    const Scalar length_upper = length(b0 - a);
                    const Scalar length_lower = length(c0 - b0);
                    if (length_upper < static_cast<Scalar>(1e-6) ||
                        length_lower < static_cast<Scalar>(1e-6))
                        return;

                    const Vector3 to_target = target - a;
                    Scalar reach_target = length(to_target);
                    if (reach_target < static_cast<Scalar>(1e-6))
                        return;
                    const Vector3 forward = to_target * (static_cast<Scalar>(1.0) / reach_target);

                    // Soft-clamp the effective reach inside the annulus both bones can satisfy.
                    const Scalar reach_max = length_upper + length_lower;
                    const Scalar reach_min = std::fabs(length_upper - length_lower);
                    const Scalar epsilon = static_cast<Scalar>(1e-4);
                    Scalar reach = reach_target;
                    reach = std::min(reach, reach_max - epsilon);
                    reach = std::max(reach, reach_min + epsilon);

                    // The knee's projection onto the target axis, and its height off that axis.
                    const Scalar projection =
                        (length_upper * length_upper - length_lower * length_lower + reach * reach) /
                        (static_cast<Scalar>(2.0) * reach);
                    const Scalar height_sq = length_upper * length_upper - projection * projection;
                    const Scalar height = height_sq > static_cast<Scalar>(0.0) ? std::sqrt(height_sq)
                                                                               : static_cast<Scalar>(0.0);

                    // The bend plane: the pole component perpendicular to the target axis.
                    Vector3 pole_direction = pole - a;
                    Vector3 side = pole_direction - forward * dot(pole_direction, forward);
                    if (length(side) < static_cast<Scalar>(1e-5))
                    {
                        // Fall back to the current bend plane, then to an arbitrary perpendicular.
                        const Vector3 current = (b0 - a) - forward * dot(b0 - a, forward);
                        side = length(current) > static_cast<Scalar>(1e-5)
                                   ? current
                                   : cross(forward, Vector3{0.0, 1.0, 0.0});
                        if (length(side) < static_cast<Scalar>(1e-5))
                            side = cross(forward, Vector3{1.0, 0.0, 0.0});
                    }
                    side = normalize(side);

                    const Vector3 mid_goal = a + forward * projection + side * height;

                    const Quaternionf original_upper = context.local_rotations[upper];
                    const Quaternionf original_mid = context.local_rotations[mid];

                    // Swing the upper bone so a->mid points at the knee goal.
                    const Quaternion rotate_upper =
                        detail::rotation_between(normalize(b0 - a), normalize(mid_goal - a));
                    context.set_rotation(upper, mul(rotate_upper, context.rotation(upper)));
                    context.recompose();

                    // Swing the lower bone so mid->tip points at the target.
                    const Vector3 b1 = context.position(mid);
                    const Vector3 c1 = context.position(tip);
                    const Quaternion rotate_mid =
                        detail::rotation_between(normalize(c1 - b1), normalize(target - b1));
                    context.set_rotation(mid, mul(rotate_mid, context.rotation(mid)));
                    context.recompose();

                    if (weight < 1.0f)
                    {
                        context.local_rotations[upper] =
                            slerp(original_upper, context.local_rotations[upper], weight);
                        context.local_rotations[mid] =
                            slerp(original_mid, context.local_rotations[mid], weight);
                        context.recompose();
                    }
                }
        };
    } // namespace Animation
} // namespace SushiEngine
