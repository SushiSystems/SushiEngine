/**************************************************************************/
/* motion_match_sampler.hpp                                              */
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
 * @file motion_match_sampler.hpp
 * @brief The blend-graph wiring `motion_matching.hpp` deliberately left open (design §12.4).
 *
 * `MotionDatabase::find_best` only selects a candidate; its own header comment is explicit
 * that blending the selection into a moving pose is the caller's job. `MotionMatchSampler`
 * is that caller: a small, ready-to-use reference driver that re-searches the database on a
 * fixed interval (not every frame — cheap hysteresis against candidates that tie or nearly
 * tie every tick), and whenever the search picks a new candidate, crossfades from whatever
 * was already playing into it over a configurable duration, using the same two-contribution
 * shape `AnimatorEvaluator::pose_layer` already uses for state transitions (both the
 * outgoing and incoming clip keep advancing their own playback time through the fade, then
 * blend by `nlerp`/`lerp`). A caller is still free to ignore this and drive
 * `MotionDatabase::find_best` directly with their own blend (e.g. feeding it as one more
 * `AnimatorEvaluator` layer) — this header exists so nobody has to write the crossfade
 * bookkeeping from scratch to get a working result.
 *
 * Deliberately not built here: search-triggered foot locking (the underlying database's
 * foot-height feature is a proxy, not true contact phase — see `motion_matching.hpp`), and
 * trajectory-window matching (still out of scope for the database itself). This sampler
 * blends whatever the database selects; it does not change what gets selected.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/motion_matching.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Tuning knobs for @ref MotionMatchSampler::update. */
        struct MotionMatchSamplerConfiguration
        {
            float velocity_weight = 1.0f;         /**< Passed through to @ref MotionDatabase::find_best. */
            float foot_weight = 0.25f;             /**< Passed through to @ref MotionDatabase::find_best. */
            float crossfade_seconds = 0.2f;        /**< Duration of the blend into a newly-selected candidate. */
            float resample_interval_seconds = 0.1f; /**< Minimum time between re-searches (hysteresis). */
        };

        /**
         * @brief Crossfades between motion-matching selections into a single output local pose.
         *
         * Owns two `ClipEvaluator`s (the outgoing and incoming clip) and the output local-pose
         * arrays a caller composes to model space themselves (via the free `compose_model`,
         * exactly like every other pose producer in this stack) — this class does not build a
         * palette or touch model space, keeping it usable both as a standalone locomotion
         * driver and as one contribution folded into a larger `AnimatorEvaluator` layer stack.
         */
        class MotionMatchSampler
        {
            public:
                /**
                 * @brief Binds the skeleton the sampled poses are shaped for.
                 * @param skeleton The rig; local pose arrays are sized to its joint count.
                 */
                void bind(const SkeletonView& skeleton)
                {
                    skeleton_ = skeleton;
                    const std::uint32_t count = skeleton.joint_count;
                    output_translations_.assign(count, Vector3f{});
                    output_rotations_.assign(count, Quaternionf{});
                    output_scales_.assign(count, Vector3f{1.0f, 1.0f, 1.0f});
                }

                /**
                 * @brief Snaps directly onto a candidate with no crossfade — call once at start.
                 * @param database The searchable pool @p candidate_index indexes into.
                 * @param animation_database Where the candidate's clip resolves.
                 * @param candidate_index An index from @ref MotionDatabase::find_best.
                 */
                void reset(const MotionDatabase& database, const IAnimationDatabase& animation_database,
                          std::size_t candidate_index)
                {
                    if (candidate_index >= database.size())
                        return;
                    const MotionCandidate& candidate = database[candidate_index];
                    current_candidate_ = candidate_index;
                    current_clip_ = candidate.clip;
                    current_time_ = candidate.time_seconds;
                    crossfading_ = false;
                    progress_ = 0.0f;
                    resample_timer_ = 0.0f;
                    sample(animation_database, current_clip_, current_time_, current_evaluator_);
                    write_output(current_evaluator_);
                }

                /**
                 * @brief Advances playback, periodically re-searches, and crossfades on a switch.
                 *
                 * @param database The searchable pool to query.
                 * @param animation_database Where selected clips resolve.
                 * @param query The desired feature (see @ref MotionDatabase::find_best).
                 * @param dt Seconds since the previous call.
                 * @param config Tuning knobs; see @ref MotionMatchSamplerConfiguration.
                 */
                void update(const MotionDatabase& database, const IAnimationDatabase& animation_database,
                           const MotionFeature& query, float dt, const MotionMatchSamplerConfiguration& config)
                {
                    if (current_clip_ == INVALID_ASSET)
                    {
                        const std::size_t best =
                            database.find_best(query, config.velocity_weight, config.foot_weight);
                        if (best != MotionDatabase::NOT_FOUND)
                            reset(database, animation_database, best);
                        return;
                    }

                    resample_timer_ += dt;
                    if (resample_timer_ >= config.resample_interval_seconds)
                    {
                        resample_timer_ = 0.0f;
                        const std::size_t best =
                            database.find_best(query, config.velocity_weight, config.foot_weight);
                        if (best != MotionDatabase::NOT_FOUND && best != current_candidate_)
                        {
                            const MotionCandidate& candidate = database[best];
                            previous_clip_ = current_clip_;
                            previous_time_ = current_time_;
                            current_candidate_ = best;
                            current_clip_ = candidate.clip;
                            current_time_ = candidate.time_seconds;
                            crossfading_ = true;
                            progress_ = 0.0f;
                        }
                    }

                    current_time_ += dt;
                    sample(animation_database, current_clip_, current_time_, current_evaluator_);

                    if (crossfading_)
                    {
                        previous_time_ += dt;
                        sample(animation_database, previous_clip_, previous_time_, previous_evaluator_);

                        const float duration = config.crossfade_seconds > 1e-6f
                                                    ? config.crossfade_seconds
                                                    : 1e-6f;
                        progress_ += dt / duration;
                        if (progress_ >= 1.0f)
                        {
                            progress_ = 1.0f;
                            crossfading_ = false;
                        }
                        blend_output(progress_);
                    }
                    else
                    {
                        write_output(current_evaluator_);
                    }
                }

                /** @brief Whether a crossfade is currently blending toward the active candidate. */
                bool crossfading() const noexcept { return crossfading_; }

                /** @brief The candidate index @ref update last selected, or `NOT_FOUND` before the first search. */
                std::size_t current_candidate() const noexcept { return current_candidate_; }

                /** @brief The output local-space translation per joint, valid after @ref update/@ref reset. */
                const std::vector<Vector3f>& local_translations() const noexcept { return output_translations_; }

                /** @brief The output local-space rotation per joint, valid after @ref update/@ref reset. */
                const std::vector<Quaternionf>& local_rotations() const noexcept { return output_rotations_; }

                /** @brief The output local-space scale per joint, valid after @ref update/@ref reset. */
                const std::vector<Vector3f>& local_scales() const noexcept { return output_scales_; }

            private:
                /** @brief Samples one clip's local pose at a time (looping) into an evaluator. */
                void sample(const IAnimationDatabase& animation_database, AssetId clip_id, float time_seconds,
                           ClipEvaluator& evaluator)
                {
                    const ClipView clip = animation_database.clip(clip_id);
                    evaluator.evaluate(skeleton_, clip, time_seconds, /*loop=*/true);
                }

                /** @brief Copies an evaluator's local pose straight into the output arrays. */
                void write_output(const ClipEvaluator& evaluator)
                {
                    output_translations_ = evaluator.local_translations();
                    output_rotations_ = evaluator.local_rotations();
                    output_scales_ = evaluator.local_scales();
                }

                /** @brief Blends previous-into-current local pose by @p t, joint by joint. */
                void blend_output(float t)
                {
                    const std::uint32_t count = skeleton_.joint_count;
                    const std::vector<Vector3f>& previous_t = previous_evaluator_.local_translations();
                    const std::vector<Quaternionf>& previous_r = previous_evaluator_.local_rotations();
                    const std::vector<Vector3f>& previous_s = previous_evaluator_.local_scales();
                    const std::vector<Vector3f>& current_t = current_evaluator_.local_translations();
                    const std::vector<Quaternionf>& current_r = current_evaluator_.local_rotations();
                    const std::vector<Vector3f>& current_s = current_evaluator_.local_scales();
                    for (std::uint32_t j = 0; j < count; ++j)
                    {
                        output_translations_[j] = lerp(previous_t[j], current_t[j], t);
                        output_scales_[j] = lerp(previous_s[j], current_s[j], t);
                        output_rotations_[j] = nlerp(previous_r[j], current_r[j], t);
                    }
                }

                SkeletonView skeleton_;
                ClipEvaluator current_evaluator_;
                ClipEvaluator previous_evaluator_;
                AssetId current_clip_ = INVALID_ASSET;
                AssetId previous_clip_ = INVALID_ASSET;
                float current_time_ = 0.0f;
                float previous_time_ = 0.0f;
                std::size_t current_candidate_ = MotionDatabase::NOT_FOUND;
                bool crossfading_ = false;
                float progress_ = 0.0f;
                float resample_timer_ = 0.0f;
                std::vector<Vector3f> output_translations_;
                std::vector<Quaternionf> output_rotations_;
                std::vector<Vector3f> output_scales_;
        };
    } // namespace Animation
} // namespace SushiEngine
