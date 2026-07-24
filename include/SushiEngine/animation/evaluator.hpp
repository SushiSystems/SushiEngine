/**************************************************************************/
/* evaluator.hpp                                                          */
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
 * @file evaluator.hpp
 * @brief The A1 evaluation chain: sample a clip, compose model space, build the palette.
 *
 * @ref ClipEvaluator owns the per-instance pose buffers (the "pose pool" of the design's
 * §5.4, in its simplest single-instance form) and runs the derived, per-frame chain:
 * sample the clip's local pose, forward-scan into model space (`parent[i] < i`), then
 * `skin[i] = model[i] * inverse_bind[i]` into an object-space float palette ready to
 * upload to the joint-palette buffer. Everything here is derived from sim state and
 * recomputed each frame, so it never enters rollback. Host-side and uncompressed in A1;
 * A2 batches this over all instances on the SushiRuntime graph without changing the
 * palette it produces.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Evaluates one skinned instance's pose into an object-space skin palette.
         *
         * Reused across frames: @ref resize is called once when the skeleton is bound, then
         * @ref evaluate (or @ref evaluate_bind) each frame. The palette and model-space
         * matrices are read back after evaluation.
         */
        class ClipEvaluator
        {
            public:
                /**
                 * @brief Sizes the pose buffers to a skeleton's joint count.
                 * @param joint_count Joints the bound skeleton has.
                 */
                void resize(std::uint32_t joint_count)
                {
                    local_translations_.assign(joint_count, Vector3f{});
                    local_rotations_.assign(joint_count, Quaternionf{});
                    local_scales_.assign(joint_count, Vector3f{1.0f, 1.0f, 1.0f});
                    model_.assign(joint_count, Mat4{});
                    palette_.assign(joint_count, JointMatrix{});
                }

                /**
                 * @brief Evaluates the clip at a time into the skin palette.
                 *
                 * Samples the local pose, composes model space, and builds the palette. Falls
                 * back to a joint's bind-pose local transform for any joint the clip does not
                 * animate (when the clip has fewer joints than the skeleton).
                 *
                 * @param skeleton The bound skeleton (drives joint count and inverse-bind).
                 * @param clip     The clip to sample.
                 * @param time_seconds Playback time in seconds.
                 * @param loop     Whether the clip loops.
                 */
                void evaluate(const SkeletonView& skeleton, const ClipView& clip, float time_seconds,
                              bool loop)
                {
                    const std::uint32_t count = skeleton.joint_count;
                    if (model_.size() != count)
                        resize(count);

                    // Seed every joint with its bind-pose local so joints the clip omits still
                    // pose correctly, then overwrite the ones the clip drives.
                    for (std::uint32_t j = 0; j < count; ++j)
                    {
                        local_translations_[j] = skeleton.bind_translations[j];
                        local_rotations_[j] = skeleton.bind_rotations[j];
                        local_scales_[j] = skeleton.bind_scales[j];
                    }
                    if (clip.valid())
                    {
                        const std::uint32_t animated = clip.joint_count < count ? clip.joint_count : count;
                        clip.sample(time_seconds, loop, local_translations_.data(),
                                    local_rotations_.data(), local_scales_.data());
                        (void)animated;
                    }
                    compose_and_skin(skeleton);
                }

                /**
                 * @brief Poses the skeleton at its bind (rest) pose — no clip.
                 * @param skeleton The bound skeleton.
                 */
                void evaluate_bind(const SkeletonView& skeleton)
                {
                    const std::uint32_t count = skeleton.joint_count;
                    if (model_.size() != count)
                        resize(count);
                    for (std::uint32_t j = 0; j < count; ++j)
                    {
                        local_translations_[j] = skeleton.bind_translations[j];
                        local_rotations_[j] = skeleton.bind_rotations[j];
                        local_scales_[j] = skeleton.bind_scales[j];
                    }
                    compose_and_skin(skeleton);
                }

                /** @brief The object-space skin palette (`model * inverse_bind`) per joint. */
                const std::vector<JointMatrix>& palette() const noexcept { return palette_; }

                /** @brief The model-space matrix per joint (bind space, post-compose). */
                const std::vector<Mat4>& model() const noexcept { return model_; }

            private:
                /** @brief Forward-scan model space, then build the object-space skin palette. */
                void compose_and_skin(const SkeletonView& skeleton)
                {
                    const std::uint32_t count = skeleton.joint_count;
                    for (std::uint32_t i = 0; i < count; ++i)
                    {
                        const Vector3f& t = local_translations_[i];
                        const Quaternionf& r = local_rotations_[i];
                        const Vector3f& s = local_scales_[i];
                        const Mat4 local = compose_transform(Vector3{t.x, t.y, t.z},
                                                             Quaternion{r.x, r.y, r.z, r.w},
                                                             Vector3{s.x, s.y, s.z});
                        model_[i] = skeleton.parents[i] == NO_PARENT
                                        ? local
                                        : mul(model_[skeleton.parents[i]], local);
                        palette_[i] = to_joint_matrix(mul(model_[i], to_mat4(skeleton.inverse_bind[i])));
                    }
                }

                std::vector<Vector3f> local_translations_;
                std::vector<Quaternionf> local_rotations_;
                std::vector<Vector3f> local_scales_;
                std::vector<Mat4> model_;
                std::vector<JointMatrix> palette_;
        };
    } // namespace Animation
} // namespace SushiEngine
