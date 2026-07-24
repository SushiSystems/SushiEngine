/**************************************************************************/
/* ik_foot_placement.hpp                                                 */
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
 * @file ik_foot_placement.hpp
 * @brief The foot-placement IK solver — plant a foot on the ground (phase A6).
 *
 * A composite modifier: it casts a ray down from the ankle through the @ref IPoseTaskContext
 * (sampled at extract, one-frame latency — design §5.3), and on a hit it two-bone-solves the
 * leg to place the ankle a foot-height above the ground and re-orients the ankle so the sole
 * follows the surface normal. It reuses @ref TwoBoneIk for the leg, so the leg math lives in one
 * place. Pelvis-height adjustment (a cross-foot concern) is left to a higher-level rig pass.
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/animation/ik_two_bone.hpp>
#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Foot-placement IK: rays to the ground and plants the (hip → knee → ankle) leg.
         *
         * All positions are object space. @ref ground supplies the surface; when it is null or
         * the ray misses, the solver leaves the pose untouched (the animated foot stands).
         */
        class FootPlacementIk : public IPoseModifier
        {
            public:
                std::uint32_t hip = 0;   /**< Upper leg joint. */
                std::uint32_t knee = 0;  /**< Knee joint. */
                std::uint32_t ankle = 0; /**< Ankle joint (the foot's end effector). */
                const IPoseTaskContext* ground = nullptr; /**< Ray provider (extract-sampled). */
                Vector3 pole{0.0, 0.0, 1.0};   /**< Object-space knee bend hint (forward). */
                Vector3 up_axis{0.0, 1.0, 0.0}; /**< The ankle's local axis normal to the sole. */
                float foot_height = 0.0f;      /**< Ankle height above the ground, object units. */
                float ray_height = 1.0f;       /**< How far above the ankle the ray starts. */
                float ray_length = 2.0f;       /**< Ray length downward. */
                float weight = 1.0f;           /**< Blend of the correction, in [0, 1]. */

                void solve(PoseModifierContext& context) const override
                {
                    if (weight <= 0.0f || ground == nullptr)
                        return;

                    const Vector3 ankle_position = context.position(ankle);
                    const Vector3 origin{ankle_position.x, ankle_position.y + ray_height,
                                         ankle_position.z};
                    const Vector3 direction{0.0, -1.0, 0.0};
                    Vector3 hit;
                    Vector3 normal;
                    if (!ground->raycast(origin, direction, hit, normal))
                        return;
                    if (length(normal) < static_cast<Scalar>(1e-6))
                        normal = Vector3{0.0, 1.0, 0.0};
                    normal = normalize(normal);

                    // Only plant when the ground is at or above the animated foot (don't pull the
                    // foot down into the air); a foot already below ground is lifted to it.
                    const Scalar goal_y = hit.y + static_cast<Scalar>(foot_height);

                    TwoBoneIk leg;
                    leg.upper = hip;
                    leg.mid = knee;
                    leg.tip = ankle;
                    leg.target = Vector3{ankle_position.x, goal_y, ankle_position.z};
                    leg.pole = pole;
                    leg.weight = weight;
                    leg.solve(context);

                    // Re-orient the ankle so the sole follows the surface normal.
                    const Quaternion ankle_rotation = context.rotation(ankle);
                    const Vector3 current_up = normalize(rotate(ankle_rotation, up_axis));
                    Quaternion align = detail::rotation_between(current_up, normal);
                    if (weight < 1.0f)
                    {
                        const Quaternion identity{0.0, 0.0, 0.0, 1.0};
                        align = slerp(identity, align, static_cast<Scalar>(weight));
                    }
                    context.set_rotation(ankle, mul(align, ankle_rotation));
                    context.recompose();
                }
        };
    } // namespace Animation
} // namespace SushiEngine
