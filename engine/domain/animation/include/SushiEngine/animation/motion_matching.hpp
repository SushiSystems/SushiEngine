/**************************************************************************/
/* motion_matching.hpp                                                   */
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
 * @file motion_matching.hpp
 * @brief A minimal, honest motion-matching layer (design §12.4) over the existing clip
 * pipeline: a searchable pose database plus nearest-neighbor selection.
 *
 * Scoped deliberately small against the "real" AAA feature (Motion Matching Illustrated,
 * Ubisoft/EA GDC talks — trajectory windows of past/future root motion, learned or
 * curated feature weights, animation-graph blending on selection): this is the
 * **searchable-database-and-nearest-neighbor core**, the part every fuller
 * implementation is built on, without the trajectory-prediction, contact-aware foot
 * locking, or automatic blend-graph wiring those talks add on top. What's here is real
 * and useful on its own (find the closest-matching pose to a desired velocity across a
 * clip library, no hand-authored state machine), not a stub — but calling it "motion
 * matching" without this paragraph would overclaim. The `Skip list` (design §2) named
 * this as needing "a locomotion database pipeline" — this is that pipeline's foundation.
 *
 * @ref MotionDatabase pre-samples a set of clips into a pool of (clip, time, feature)
 * candidates via the existing `ClipEvaluator` (forward-kinematics the database build
 * already has, reused rather than reimplemented); @ref MotionFeature is a small,
 * inspectable feature vector — root velocity plus two optional foot-height proxies (not
 * true ground-contact phase; a real implementation would want a contact/phase signal
 * baked at import, design's future work) — and @ref MotionDatabase::find_best is a
 * brute-force weighted-nearest-neighbor search (fine at hundreds-to-low-thousands of
 * candidates; a k-d tree or similar is the scaling follow-up once a real content set
 * exceeds brute-force's budget, not built speculatively here). Blending the pose once a
 * new candidate is selected is the caller's responsibility — this header supplies the
 * *selection*, not a bespoke blend engine (the existing `AnimatorEvaluator` crossfade
 * machinery, or a caller's own nlerp between two `ClipEvaluator` outputs, already does
 * that job; duplicating it here would be exactly the kind of redundant abstraction the
 * project avoids).
 */

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/evaluator.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief A candidate pose's searchable feature vector.
         *
         * `root_velocity` is model-space, finite-differenced from the root joint's
         * position a small `dt` apart (§ the database build's own step, not a
         * caller-supplied one, so every candidate in a database is measured the same
         * way). `left_foot_height`/`right_foot_height` are named-joint model-space Y at
         * the candidate's time — a *proxy* for ground-contact phase, not the real
         * thing (a flat ground plane at Y=0 makes a low height a reasonable "planted"
         * signal for a simple locomotion set; it says nothing about actual terrain
         * contact). Either is skipped (left at 0) when the caller does not name that
         * joint — see @ref MotionDatabase::add_clip.
         */
        struct MotionFeature
        {
            Vector3f root_velocity{0.0f, 0.0f, 0.0f};
            float left_foot_height = 0.0f;
            float right_foot_height = 0.0f;
        };

        /** @brief One searchable pose: which clip, what time, and its feature vector. */
        struct MotionCandidate
        {
            AssetId clip = INVALID_ASSET;
            float time_seconds = 0.0f;
            MotionFeature feature;
        };

        /**
         * @brief A brute-force-searchable pool of sampled poses from one or more clips.
         */
        class MotionDatabase
        {
            public:
                /** @brief Sentinel returned by @ref find_best when the database is empty. */
                static constexpr std::size_t NOT_FOUND = static_cast<std::size_t>(-1);

                /**
                 * @brief Samples a clip into evenly-spaced candidates and appends them.
                 *
                 * Uses `ClipEvaluator` (the same host forward-kinematics the live preview
                 * and every headless demo already trust) to pose the skeleton at each
                 * sample time, reads the root joint's model-space position there and a
                 * small `finite_difference_dt` later for @ref MotionFeature::root_velocity,
                 * and optionally the named foot joints' model-space height.
                 *
                 * @param clip_id             The clip to sample (must be registered in
                 *                            @p database).
                 * @param database            Where @p clip_id resolves.
                 * @param skeleton            The bound skeleton (drives joint order/count).
                 * @param root_joint          Joint index whose motion defines velocity.
                 * @param left_foot_joint     Joint index for the left-foot height proxy, or
                 *                            -1 to leave it at 0 for every candidate from
                 *                            this clip.
                 * @param right_foot_joint    Same, for the right foot.
                 * @param candidates_per_clip How many evenly-spaced samples across the
                 *                            clip's duration (>= 1; a value of 0 is a no-op).
                 * @param finite_difference_dt Seconds between the two samples the velocity
                 *                            finite-difference reads (default a 60 Hz step).
                 */
                void add_clip(AssetId clip_id, const IAnimationDatabase& database,
                              const SkeletonView& skeleton, std::uint32_t root_joint,
                              int left_foot_joint, int right_foot_joint,
                              std::uint32_t candidates_per_clip,
                              float finite_difference_dt = 1.0f / 60.0f)
                {
                    if (candidates_per_clip == 0)
                        return;
                    const ClipView clip = database.clip(clip_id);
                    if (!clip.valid())
                        return;

                    ClipEvaluator evaluator;
                    for (std::uint32_t i = 0; i < candidates_per_clip; ++i)
                    {
                        const float t = clip.duration * static_cast<float>(i) /
                                       static_cast<float>(candidates_per_clip);

                        evaluator.evaluate(skeleton, clip, t, /*loop=*/true);
                        const Vector3 p0 = model_position(evaluator, root_joint);
                        MotionCandidate candidate;
                        candidate.clip = clip_id;
                        candidate.time_seconds = t;
                        if (left_foot_joint >= 0)
                            candidate.feature.left_foot_height = static_cast<float>(
                                model_position(evaluator, static_cast<std::uint32_t>(
                                                              left_foot_joint))
                                    .y);
                        if (right_foot_joint >= 0)
                            candidate.feature.right_foot_height = static_cast<float>(
                                model_position(evaluator, static_cast<std::uint32_t>(
                                                              right_foot_joint))
                                    .y);

                        evaluator.evaluate(skeleton, clip, t + finite_difference_dt,
                                          /*loop=*/true);
                        const Vector3 p1 = model_position(evaluator, root_joint);
                        const Vector3 velocity =
                            (p1 - p0) * (Scalar(1) / Scalar(finite_difference_dt));
                        candidate.feature.root_velocity =
                            Vector3f{static_cast<float>(velocity.x),
                                    static_cast<float>(velocity.y),
                                    static_cast<float>(velocity.z)};

                        candidates_.push_back(candidate);
                    }
                }

                /** @brief Candidates currently in the database. */
                std::size_t size() const noexcept { return candidates_.size(); }

                /** @brief One candidate by index (see @ref find_best). */
                const MotionCandidate& operator[](std::size_t index) const noexcept
                {
                    return candidates_[index];
                }

                /**
                 * @brief Brute-force weighted-nearest-neighbor search.
                 *
                 * Squared distance over velocity (all three axes) plus the two foot-height
                 * proxies, each independently weighted so a caller can favor matching
                 * velocity over foot placement or vice versa. O(size()) — fine at the
                 * hundreds-to-low-thousands scale a curated locomotion set actually has;
                 * see this file's header comment on a spatial index as the scaling
                 * follow-up, not built here speculatively.
                 *
                 * @param query           The desired feature (typically: the gameplay
                 *                        input's desired velocity, current actual foot
                 *                        heights left at 0 if not tracked).
                 * @param velocity_weight Weight on the squared velocity-vector distance.
                 * @param foot_weight     Weight on each squared foot-height distance.
                 * @return The index of the closest candidate, or @ref NOT_FOUND if empty.
                 */
                std::size_t find_best(const MotionFeature& query, float velocity_weight = 1.0f,
                                      float foot_weight = 0.25f) const noexcept
                {
                    if (candidates_.empty())
                        return NOT_FOUND;
                    std::size_t best = 0;
                    float best_distance =
                        feature_distance(query, candidates_[0].feature, velocity_weight,
                                        foot_weight);
                    for (std::size_t i = 1; i < candidates_.size(); ++i)
                    {
                        const float distance = feature_distance(
                            query, candidates_[i].feature, velocity_weight, foot_weight);
                        if (distance < best_distance)
                        {
                            best_distance = distance;
                            best = i;
                        }
                    }
                    return best;
                }

            private:
                /** @brief Model-space position (translation column) of a joint's matrix. */
                static Vector3 model_position(const ClipEvaluator& evaluator,
                                              std::uint32_t joint) noexcept
                {
                    const Mat4& m = evaluator.model()[joint];
                    return Vector3{m.m[12], m.m[13], m.m[14]};
                }

                static float feature_distance(const MotionFeature& a, const MotionFeature& b,
                                              float velocity_weight, float foot_weight) noexcept
                {
                    const float dvx = a.root_velocity.x - b.root_velocity.x;
                    const float dvy = a.root_velocity.y - b.root_velocity.y;
                    const float dvz = a.root_velocity.z - b.root_velocity.z;
                    const float dlf = a.left_foot_height - b.left_foot_height;
                    const float drf = a.right_foot_height - b.right_foot_height;
                    return velocity_weight * (dvx * dvx + dvy * dvy + dvz * dvz) +
                          foot_weight * (dlf * dlf + drf * drf);
                }

                std::vector<MotionCandidate> candidates_;
        };
    } // namespace Animation
} // namespace SushiEngine
