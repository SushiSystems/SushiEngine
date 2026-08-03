/**************************************************************************/
/* runtime_retarget.hpp                                                  */
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
 * @file runtime_retarget.hpp
 * @brief Same-session retargeting (design §12.4): a clip drives a rig picked at runtime.
 *
 * `retarget.hpp`'s `retarget_clip`/`mirror_clip` are import-time only — they bake a new clip
 * for a *known* target rig, once, offline. That leaves a real gap named in §12.4: swapping a
 * character model mid-game (a cosmetic outfit change, a boss reusing a player rig, a network
 * replacement asset) has no known target skeleton at cook time, so nothing can be pre-baked.
 * `RuntimeRetargeter` closes that gap: it samples a clip against its *own* source skeleton
 * every frame (via `ClipEvaluator`, no different from any other clip playback), then calls
 * `retarget_pose_frame` (the per-frame delta transfer `retarget_clip` already uses internally,
 * shared rather than duplicated) to produce the pose against a target skeleton chosen at
 * runtime, and composes/skins it exactly like `ClipEvaluator` does.
 *
 * Cost is the honest tradeoff: this redoes the bind-pose-delta transfer every frame instead of
 * once at cook time. For a clip retargeted onto the same rig pair every frame of a whole level,
 * `retarget_clip` once offline is cheaper — this class is for the case where the target rig
 * genuinely is not known until runtime.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/humanoid.hpp>
#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/animation/retarget.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Plays a clip authored for one rig onto a different rig, chosen at runtime.
         *
         * Bind once via @ref bind, then @ref evaluate each frame like any other clip player; the
         * palette it produces is shaped for the *target* skeleton, ready to upload.
         */
        class RuntimeRetargeter
        {
            public:
                /**
                 * @brief Binds the source and target rigs and their canonical bone maps.
                 * @param source_skeleton The rig the clip was authored for.
                 * @param source_avatar   The source rig's bone map (see @ref build_avatar_heuristic).
                 * @param target_skeleton The rig to actually pose and skin.
                 * @param target_avatar   The target rig's bone map.
                 */
                void bind(const SkeletonView& source_skeleton, const Avatar& source_avatar,
                         const SkeletonView& target_skeleton, const Avatar& target_avatar)
                {
                    source_skeleton_ = source_skeleton;
                    source_avatar_ = source_avatar;
                    target_skeleton_ = target_skeleton;
                    target_avatar_ = target_avatar;

                    const std::uint32_t count = target_skeleton.joint_count;
                    target_translations_.assign(count, Vector3f{});
                    target_rotations_.assign(count, Quaternionf{});
                    target_scales_.assign(count, Vector3f{1.0f, 1.0f, 1.0f});
                    model_.assign(count, Mat4{});
                    palette_.assign(count, JointMatrix{});
                    for (std::uint32_t j = 0; j < count; ++j)
                        target_scales_[j] = target_skeleton.bind_scales[j];
                }

                /**
                 * @brief Samples the clip against the source rig, retargets, composes, and skins.
                 * @param clip     The clip to sample (authored for the bound source skeleton).
                 * @param time_seconds Playback time in seconds.
                 * @param loop     Whether the clip loops.
                 */
                void evaluate(const ClipView& clip, float time_seconds, bool loop)
                {
                    source_evaluator_.evaluate(source_skeleton_, clip, time_seconds, loop);
                    retarget_pose_frame(source_evaluator_.local_translations().data(),
                                        source_evaluator_.local_rotations().data(), source_avatar_,
                                        source_skeleton_, target_avatar_, target_skeleton_,
                                        target_translations_.data(), target_rotations_.data());
                    compose_model(target_skeleton_, target_translations_.data(),
                                  target_rotations_.data(), target_scales_.data(), model_.data());
                    for (std::uint32_t i = 0; i < target_skeleton_.joint_count; ++i)
                        palette_[i] =
                            to_joint_matrix(mul(model_[i], to_mat4(target_skeleton_.inverse_bind[i])));
                }

                /** @brief The retargeted local-space translation per joint (target rig). */
                const std::vector<Vector3f>& local_translations() const noexcept
                {
                    return target_translations_;
                }

                /** @brief The retargeted local-space rotation per joint (target rig). */
                const std::vector<Quaternionf>& local_rotations() const noexcept
                {
                    return target_rotations_;
                }

                /** @brief The object-space model matrix per joint (target rig, post-compose). */
                const std::vector<Mat4>& model() const noexcept { return model_; }

                /** @brief The object-space skin palette per joint (target rig), ready to upload. */
                const std::vector<JointMatrix>& palette() const noexcept { return palette_; }

            private:
                SkeletonView source_skeleton_;
                Avatar source_avatar_;
                SkeletonView target_skeleton_;
                Avatar target_avatar_;
                ClipEvaluator source_evaluator_;
                std::vector<Vector3f> target_translations_;
                std::vector<Quaternionf> target_rotations_;
                std::vector<Vector3f> target_scales_;
                std::vector<Mat4> model_;
                std::vector<JointMatrix> palette_;
        };
    } // namespace Animation
} // namespace SushiEngine
