/**************************************************************************/
/* ragdoll_blend.hpp                                                      */
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
 * @file ragdoll_blend.hpp
 * @brief The animation-to-physics-and-back pose modifier (design §12.4/§11): blends
 * the animated pose toward per-joint physics-body transforms.
 *
 * Design §11 named this exactly: "Physics / XPBD — `IPoseTaskContext`'s physics-backed
 * implementation and future ragdoll blending consume the existing solver; nothing in
 * A0–A9 blocks on it." This is that ragdoll blending, landed as an `IPoseModifier`
 * (design §5.3's seam — a new solver, not a change to the evaluator or the stack).
 *
 * Scope: this header is the **blend**, not the physics. It takes per-joint object-space
 * transforms a caller already resolved from XPBD bodies (the same "caller resolves the
 * world query, the modifier stays a pure function of its inputs" contract
 * `IPoseTaskContext`/`FootPlacementIk` already use) and blends each named joint's local
 * pose toward the physics transform by a per-joint weight — 0 stays pure animation, 1 is
 * pure physics, and anything between is the transition window a hit reaction or death
 * ragdoll needs (weight ramping 0→1 over the transition is the caller's job, e.g. driven
 * by a timer or an impact-force curve; this modifier is a stateless function of whatever
 * weight it is given this frame). It does **not** map joints to `Physics::XPBDSolver`
 * bodies, run inverse dynamics, or blend velocities for a physically-continuous
 * handoff — those are real follow-up work once a gameplay feature needs them (matching
 * §12.4's "prioritize only against an actual gameplay/content need" rule).
 */

#include <algorithm>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief One joint's physics-driven target and how strongly it overrides the pose. */
        struct RagdollJointTarget
        {
            std::uint32_t joint = 0;    /**< Index into the skeleton's cooked joint order. */
            Matrix4 object_space_transform{}; /**< The physics body's pose, in the character's object space (caller-resolved). */
            float weight = 0.0f;        /**< 0 = pure animation, 1 = pure physics, blends the local pose between. */
        };

        /**
         * @brief Blends named joints' local pose toward caller-supplied physics transforms.
         *
         * Runs after every layer blends (design §5.3's ordering), so it corrects the
         * *animated* pose, not a bind pose — matching every other shipped `IPoseModifier`.
         * Because a physics body reports an absolute object-space transform (not a delta
         * the way IK targets are), this modifier converts it to the joint's *local* pose
         * relative to its already-composed parent (`inverse(model[parent]) *
         * desired_object_space_transform`) before blending — so a downstream joint's model
         * space still composes correctly from the topological forward scan (`recompose()`
         * regenerates every affected joint and its descendants, not just the ones this
         * modifier names directly).
         */
        class RagdollBlend : public IPoseModifier
        {
            public:
                /** @brief This frame's per-joint targets; caller-owned, set before `evaluate()`. */
                std::vector<RagdollJointTarget> targets;

                void solve(PoseModifierContext& context) const override
                {
                    bool changed = false;
                    for (const RagdollJointTarget& target : targets)
                    {
                        if (target.weight <= 0.0f ||
                            target.joint >= context.skeleton.joint_count)
                            continue;

                        const std::uint16_t parent_index = context.skeleton.parents[target.joint];
                        const Matrix4 parent_model =
                            parent_index == NO_PARENT ? Matrix4{} : context.model[parent_index];
                        const Matrix4 desired_local =
                            mul(affine_inverse(parent_model), target.object_space_transform);

                        Vector3 desired_t;
                        Quaternion desired_r;
                        Vector3 desired_s;
                        decompose_transform(desired_local, desired_t, desired_r, desired_s);

                        const Vector3f& at = context.local_translations[target.joint];
                        const Quaternionf& ar = context.local_rotations[target.joint];
                        const Vector3f& as = context.local_scales[target.joint];
                        const Vector3 anim_t{at.x, at.y, at.z};
                        const Quaternion anim_r{ar.x, ar.y, ar.z, ar.w};
                        const Vector3 anim_s{as.x, as.y, as.z};

                        const Scalar w = static_cast<Scalar>(
                            std::min(std::max(target.weight, 0.0f), 1.0f));
                        const Vector3 blended_t = lerp(anim_t, desired_t, w);
                        const Quaternion blended_r = nlerp(anim_r, desired_r, w);
                        const Vector3 blended_s = lerp(anim_s, desired_s, w);

                        context.local_translations[target.joint] = Vector3f{
                            static_cast<float>(blended_t.x), static_cast<float>(blended_t.y),
                            static_cast<float>(blended_t.z)};
                        context.local_rotations[target.joint] = Quaternionf{
                            static_cast<float>(blended_r.x), static_cast<float>(blended_r.y),
                            static_cast<float>(blended_r.z), static_cast<float>(blended_r.w)};
                        context.local_scales[target.joint] = Vector3f{
                            static_cast<float>(blended_s.x), static_cast<float>(blended_s.y),
                            static_cast<float>(blended_s.z)};
                        changed = true;
                    }
                    if (changed)
                        context.recompose();
                }
        };
    } // namespace Animation
} // namespace SushiEngine
