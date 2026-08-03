/**************************************************************************/
/* jiggle_bone.hpp                                                       */
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
 * @file jiggle_bone.hpp
 * @brief Physics-driven secondary motion for a single bone tip (design §12.4).
 *
 * Named in §12.4 as never-scoped work ("jiggle bones / physics-driven secondary motion —
 * hair, cloth-adjacent bone chains, soft-body accessories. Not mentioned anywhere in the
 * original plan; no code."). `JiggleBone` is the minimal real version of that idea, the same
 * shape as VRM SpringBone / Unity DynamicBone's single-point-mass model: each configured
 * joint is treated as a pendulum bob hanging off *its parent*, pulled each frame toward the
 * "rigid" position the animation/IK stack already placed it at (@ref PoseModifierContext's
 * @c position before this modifier runs), integrated with damped-spring Verlet, then
 * distance-constrained back onto its bone length so it swings rather than stretches.
 *
 * A joint's own local rotation never moves the joint itself — in a skeletal hierarchy it only
 * orients that joint's *children* (`model[child] = model[joint] * local(child)`; the joint's
 * own position is `model[parent] * local_translation[joint]`, which its own rotation does not
 * appear in at all). So, exactly like `FullBodyIk`'s CCD step rotates an *ancestor* to move an
 * effector's tip, `JiggleBone` rotates the configured joint's **parent** to swing the
 * configured joint's position toward the simulated point — the configured joint is the
 * pendulum bob, its parent is the pivot. This means a joint with siblings sharing that same
 * parent will have its own rest pose disturbed by another sibling's jiggle if both are
 * configured on the same parent — a real limitation, worth flagging exactly like
 * `FullBodyIk`'s shared-ancestor caveat, not silently.
 *
 * The rotation @ref solve writes onto the parent is a fresh correction recomputed every call
 * against whatever @c context.position currently reports for the configured joint — it is
 * not a standing offset the caller must remember to undo. This falls out for free in the
 * normal pipeline: `AnimatorEvaluator::evaluate` reseeds the local pose from the animated
 * clips and recomposes from scratch every single call, before any pose modifier runs, so a
 * jiggle correction from frame N never leaks into frame N+1's "rest" reading — only the
 * persistent spring state (@ref State, position and velocity) carries across frames, exactly
 * like a real spring bone. A caller driving @ref PoseModifierContext by hand (as this file's
 * own demo does) must reproduce that: recompose from the true, unmodified animated pose
 * before each @ref solve call, not from whatever @ref solve left the parent's rotation at.
 *
 * This is a single point-mass per joint, not a constrained multi-segment chain solver — good
 * for a ponytail tip, an earring, a cloak corner, or a chain of independently-configured
 * joints (add one @ref JiggleJoint per joint, parent to child, and each pulls toward *its
 * own* animated rest position, which itself moves with its already-jiggled parent — a cheap
 * approximation of chain dynamics, not a real multi-body solve). Collision, wind, and
 * stretch limits beyond the fixed bone-length constraint are not built.
 *
 * One consequence of the point-mass-on-a-sphere model is worth naming, because it looks like a
 * bug: a parent displacement exactly *along* the bone's own axis by twice its length is a fixed
 * point. The mass ends up diametrically opposite on the constraint sphere, where the spring
 * pull is parallel to the constraint, so the length projection returns it to where it started
 * and it never catches up. Any lateral component at all breaks the symmetry and it swings
 * normally, so real animation never hits it — but a rig that needs axial travel damped must
 * drive the joint's rest length instead of this spring.
 *
 * @ref IPoseModifier::solve is `const` (the stack's documented "stateless configuration"
 * contract), but a spring genuinely needs last frame's position and velocity to integrate —
 * @ref JiggleBone is a deliberate, documented exception: its per-joint simulation state lives
 * in a `mutable` member, exactly the boundary the class comment on `IPoseModifier` already
 * anticipates for anything actually stateful.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief One jiggling joint and its spring-damper tuning. */
        struct JiggleJoint
        {
            std::uint32_t joint = 0; /**< The joint this spring drives. */
            float stiffness = 120.0f; /**< Spring pull back toward the rigid rest position (1/s^2 scale). */
            float damping = 0.9f;     /**< Per-step velocity retention in [0, 1); lower settles faster. */
            Vector3 gravity{0.0, 0.0, 0.0}; /**< Constant object-space acceleration (e.g. sag). */
        };

        /**
         * @brief Spring-damper secondary motion for a set of independently configured joints.
         */
        class JiggleBone : public IPoseModifier
        {
            public:
                std::vector<JiggleJoint> joints; /**< Every joint this instance drives. */

                /**
                 * @brief Sets the simulation step used by the next @ref solve call.
                 * @param seconds Seconds since the previous @ref solve; must be set every frame.
                 */
                void set_delta_time(float seconds) noexcept { dt_ = seconds; }

                /** @brief Resets every joint's spring state, snapping it back to its rest pose next solve. */
                void reset() noexcept { state_.clear(); }

                void solve(PoseModifierContext& context) const override
                {
                    if (state_.size() != joints.size())
                        state_.assign(joints.size(), State{});

                    for (std::size_t i = 0; i < joints.size(); ++i)
                        solve_one(context, joints[i], state_[i]);
                }

            private:
                /** @brief Per-joint Verlet state (previous and current simulated tip position). */
                struct State
                {
                    Vector3 position{0.0, 0.0, 0.0};
                    Vector3 previous_position{0.0, 0.0, 0.0};
                    bool initialized = false;
                };

                /** @brief Integrates and applies one joint's spring for the current frame. */
                void solve_one(PoseModifierContext& context, const JiggleJoint& config, State& state) const
                {
                    if (config.joint >= context.skeleton.joint_count)
                        return;
                    const std::uint32_t parent = context.skeleton.parents[config.joint];
                    if (parent == NO_PARENT)
                        return;

                    const Vector3 parent_position = context.position(parent);
                    const Vector3 rest_position = context.position(config.joint);
                    const Scalar bone_length = length(rest_position - parent_position);
                    if (bone_length <= Scalar(1e-6))
                        return;

                    if (!state.initialized)
                    {
                        state.position = rest_position;
                        state.previous_position = rest_position;
                        state.initialized = true;
                    }

                    const Scalar dt = static_cast<Scalar>(dt_);
                    const Vector3 velocity = (state.position - state.previous_position) *
                                             static_cast<Scalar>(config.damping);
                    const Vector3 spring_accel =
                        (rest_position - state.position) * static_cast<Scalar>(config.stiffness);
                    const Vector3 acceleration = spring_accel + config.gravity;
                    Vector3 new_position = state.position + velocity + acceleration * (dt * dt);

                    // Constrain back onto the fixed bone length so the joint swings, not stretches.
                    Vector3 direction = new_position - parent_position;
                    const Scalar direction_length = length(direction);
                    if (direction_length > Scalar(1e-6))
                        new_position = parent_position + direction * (bone_length / direction_length);
                    else
                        new_position = rest_position;

                    state.previous_position = state.position;
                    state.position = new_position;

                    // Rotate the PARENT (the pivot), not config.joint itself (the bob) — see
                    // this file's header comment on why a joint can never move its own position.
                    const Vector3 rest_direction = normalize(rest_position - parent_position);
                    const Vector3 sim_direction = normalize(new_position - parent_position);
                    const Quaternion delta = detail::rotation_between(rest_direction, sim_direction);
                    const Quaternion current_world = context.rotation(parent);
                    context.set_rotation(parent, mul(delta, current_world));
                    context.recompose();
                }

                mutable std::vector<State> state_;
                float dt_ = 1.0f / 60.0f;
        };
    } // namespace Animation
} // namespace SushiEngine
