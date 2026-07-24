/**************************************************************************/
/* batch_evaluator.hpp                                                   */
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
 * @file batch_evaluator.hpp
 * @brief Batched multi-instance evaluation with bone LOD and update-rate throttling (A2).
 *
 * The A1 @ref ClipEvaluator poses one instance; A2 poses a crowd. @ref BatchEvaluator owns
 * one pooled palette buffer sliced per instance (the design's §5.4 pose pool, CPU form) and
 * evaluates an array of instances into it. Two throttles keep a 1000-instance crowd in
 * budget (design §5.2): a distance-derived **bone LOD** poses only a joint-count prefix, and
 * an **update-rate** ladder re-poses a distant instance at 30 or 15 Hz, reusing its last
 * palette on the skipped frames. Both are driven by fields the render `QualityParams`
 * resolver maps into @ref AnimationBudget, so animation stays inside the tier contract.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/skeleton.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief The tier-resolved animation LOD/throttle knobs (from render QualityParams).
         *
         * Kept here, engine-side and renderer-free, so the batched evaluator never sees a
         * Vulkan type; the renderer's `resolve_quality` fills these from `bone_lod_bias` and
         * the tier's update-rate ladder.
         */
        struct AnimationBudget
        {
            /** @brief Distance thresholds (metres) between LOD levels 0/1/2/3. */
            float lod_distances[3] = {10.0f, 25.0f, 50.0f};

            /** @brief Levels added to the distance-derived LOD (a lower tier poses coarser). */
            std::uint32_t bone_lod_bias = 0;

            /** @brief Update rate (Hz) per LOD level; a distant instance re-poses less often. */
            std::uint32_t update_hz[4] = {60, 30, 15, 15};

            /** @brief The fixed simulation rate the update-rate cadence is measured against. */
            std::uint32_t base_hz = 60;
        };

        /** @brief The LOD a distance resolves to: how many joints to pose and how often. */
        struct InstanceLod
        {
            std::uint32_t joint_count = 0; /**< Joint-count prefix to compose (a valid sub-skeleton). */
            std::uint32_t update_hz = 60;  /**< Re-pose cadence. */
            std::uint32_t level = 0;       /**< The resolved LOD level, for phasing. */
        };

        /**
         * @brief Resolves a skeleton + distance to a bone-LOD prefix and update rate.
         * @param skeleton The instance's skeleton (its LOD table bounds the prefix).
         * @param distance Camera distance to the instance, metres.
         * @param budget   The tier-resolved knobs.
         * @return The joint-count prefix and update cadence to pose this instance at.
         */
        inline InstanceLod select_lod(const SkeletonView& skeleton, float distance,
                                      const AnimationBudget& budget) noexcept
        {
            std::uint32_t level = 0;
            for (int i = 0; i < 3; ++i)
                if (distance > budget.lod_distances[i])
                    level = static_cast<std::uint32_t>(i + 1);
            level += budget.bone_lod_bias;
            if (level > 3)
                level = 3;
            InstanceLod lod;
            lod.level = level;
            // The skeleton's LOD table caps the prefix; a single-LOD skeleton always poses full.
            const std::uint32_t lod_index = level < skeleton.lod_count ? level : skeleton.lod_count - 1;
            lod.joint_count = skeleton.lod_joint_counts != nullptr
                                  ? skeleton.lod_joint_counts[lod_index]
                                  : skeleton.joint_count;
            lod.update_hz = budget.update_hz[level];
            return lod;
        }

        /** @brief One instance to pose this frame. */
        struct BatchInstance
        {
            SkeletonView skeleton;  /**< The rig to pose. */
            ClipView clip;          /**< The clip to sample. */
            float time = 0.0f;      /**< Playback time, seconds. */
            bool loop = true;       /**< Whether the clip loops. */
            float distance = 0.0f;  /**< Camera distance, metres (drives LOD/throttle). */
        };

        /**
         * @brief Poses a crowd into one pooled palette buffer, with LOD and throttling.
         *
         * Reused across frames: @ref reserve once, @ref evaluate each frame. An instance is
         * re-posed only on frames its update rate is due (round-robin phased by instance
         * index so the cost spreads evenly); on skipped frames its palette holds. The palette
         * for instance i is `palette(i)`, @ref max_joints matrices wide.
         */
        class BatchEvaluator
        {
            public:
                /**
                 * @brief Sizes the pool for a maximum instance count and joint count.
                 * @param instances Maximum instances posed in one batch.
                 * @param max_joints Maximum joints any skeleton in the batch has.
                 */
                void reserve(std::size_t instances, std::uint32_t max_joints)
                {
                    max_joints_ = max_joints;
                    palettes_.assign(instances * max_joints, JointMatrix{});
                    posed_.assign(instances, 0);
                }

                /**
                 * @brief Poses every instance due this frame into the pool.
                 * @param items       The instances to pose.
                 * @param count       Number of entries in @p items.
                 * @param frame_index The current fixed-tick index (drives the update cadence).
                 * @param budget      The tier-resolved LOD/throttle knobs.
                 */
                void evaluate(const BatchInstance* items, std::size_t count,
                              std::uint64_t frame_index, const AnimationBudget& budget)
                {
                    if (palettes_.size() < count * max_joints_)
                        reserve(count, max_joints_);
                    posed_evaluations_ = 0;
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const BatchInstance& item = items[i];
                        const InstanceLod lod = select_lod(item.skeleton, item.distance, budget);
                        // The update cadence: re-pose every (base_hz / update_hz) ticks, phased
                        // by the instance index so a crowd's re-poses do not all land together.
                        const std::uint32_t stride =
                            lod.update_hz > 0 ? budget.base_hz / lod.update_hz : 1;
                        const bool first = posed_[i] == 0;
                        const bool due = stride <= 1 || ((frame_index + i) % stride) == 0;
                        if (!first && !due)
                            continue;

                        evaluator_.evaluate(item.skeleton, item.clip, item.time, item.loop);
                        const std::vector<JointMatrix>& source = evaluator_.palette();
                        JointMatrix* destination = &palettes_[i * max_joints_];
                        const std::uint32_t joints =
                            item.skeleton.joint_count < max_joints_ ? item.skeleton.joint_count
                                                                    : max_joints_;
                        for (std::uint32_t j = 0; j < joints; ++j)
                            destination[j] = source[j];
                        posed_[i] = 1;
                        ++posed_evaluations_;
                    }
                }

                /** @brief The palette of one instance (@ref max_joints matrices wide). */
                const JointMatrix* palette(std::size_t instance) const noexcept
                {
                    return &palettes_[instance * max_joints_];
                }

                /** @brief Matrices per instance in the pool. */
                std::uint32_t max_joints() const noexcept { return max_joints_; }

                /** @brief How many instances were actually re-posed on the last @ref evaluate. */
                std::size_t posed_evaluations() const noexcept { return posed_evaluations_; }

            private:
                ClipEvaluator evaluator_;
                std::vector<JointMatrix> palettes_;
                std::vector<std::uint8_t> posed_; /**< Whether each instance has ever been posed. */
                std::uint32_t max_joints_ = 0;
                std::size_t posed_evaluations_ = 0;
        };
    } // namespace Animation
} // namespace SushiEngine
