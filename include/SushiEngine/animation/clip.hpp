/**************************************************************************/
/* clip.hpp                                                               */
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
 * @file clip.hpp
 * @brief The immutable animation clip asset, seen as a flat view over a cooked blob.
 *
 * Phase A1 stores clips *uncompressed*: every joint carries a dense translation,
 * rotation, and scale track resampled to a single uniform rate at import. This is the
 * simplest thing that can drive skinning correctly; the ACL-shaped compression the
 * design calls for lands in A2 behind this same @ref ClipView, so nothing downstream of
 * the view changes when it does. Tracks are laid out frame-major — all joints of one
 * frame are contiguous — so sampling a whole pose touches two adjacent frames as two
 * coalesced runs, the shape the batched evaluator wants.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/clip_compressed.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Maximum frames in one uncompressed clip (asset compile error above). */
        constexpr std::uint32_t MAX_CLIP_FRAMES = 1u << 20;

        /**
         * @brief A non-owning, immutable view of a cooked animation clip.
         *
         * Aliases a byte buffer owned elsewhere (the @ref AnimationDatabase). The three
         * track arrays are each @ref joint_count × @ref frame_count long, indexed
         * `frame * joint_count + joint`. Trivially copyable, so it passes by value into a
         * kernel or the evaluator.
         */
        struct ClipView
        {
            /** @brief Joints the clip animates; must match the target skeleton. */
            std::uint32_t joint_count = 0;

            /** @brief Number of sampled frames (>= 1). */
            std::uint32_t frame_count = 0;

            /** @brief Uniform sample rate in frames per second. */
            float sample_rate = 30.0f;

            /** @brief Clip length in seconds: `(frame_count - 1) / sample_rate`. */
            float duration = 0.0f;

            /** @brief Per-frame, per-joint local translation (`frame * joint_count + joint`). */
            const Vector3f* translations = nullptr;

            /** @brief Per-frame, per-joint local rotation. */
            const Quaternionf* rotations = nullptr;

            /** @brief Per-frame, per-joint local scale. */
            const Vector3f* scales = nullptr;

            /** @brief Storage format: raw dense tracks (A1) or compressed segments (A2). */
            ClipFormat format = ClipFormat::Raw;

            /** @brief The compressed tables, populated only when @ref format is Compressed. */
            CompressedClip compressed{};

            /** @brief Morph-weight tracks the clip drives (blend-shape weights), 0 if none. */
            std::uint32_t morph_track_count = 0;

            /** @brief FNV-1a 64 hash of each morph track's target name (@ref morph_track_count long). */
            const NameHash* morph_names = nullptr;

            /** @brief Per-frame morph weights, `frame * morph_track_count + track`. */
            const float* morph_weights = nullptr;

            /** @brief Generic float tracks the clip drives (property-hash addressed), 0 if none. */
            std::uint32_t generic_track_count = 0;

            /** @brief FNV-1a 64 hash of each generic track's property name (@ref generic_track_count long). */
            const NameHash* generic_names = nullptr;

            /** @brief Per-frame generic values, `frame * generic_track_count + track`. */
            const float* generic_values = nullptr;

            /**
             * @brief Whether the view points at real data.
             * @return True once a loader has populated it.
             */
            bool valid() const noexcept
            {
                return frame_count > 0 &&
                       (translations != nullptr ||
                        (format == ClipFormat::Compressed && compressed.valid()));
            }

            /**
             * @brief Samples the whole local pose at a time, into caller-owned arrays.
             *
             * Finds the bracketing frames, linearly interpolates translation and scale, and
             * neighbourhood-corrected-nlerps rotation. Looping wraps the time cyclically
             * (the last frame blends back to the first); non-looping clamps to the ends.
             * The output arrays must each hold @ref joint_count elements.
             *
             * @param time_seconds Playback time in seconds (any value; wrapped or clamped).
             * @param loop         Whether the clip loops.
             * @param out_translations Receives per-joint local translation.
             * @param out_rotations    Receives per-joint local rotation.
             * @param out_scales       Receives per-joint local scale.
             */
            void sample(float time_seconds, bool loop, Vector3f* out_translations,
                        Quaternionf* out_rotations, Vector3f* out_scales) const noexcept
            {
                if (!valid())
                    return;

                float frame_position;
                std::uint32_t frame0;
                std::uint32_t frame1;
                if (frame_count == 1)
                {
                    frame_position = 0.0f;
                    frame0 = 0;
                    frame1 = 0;
                }
                else if (loop)
                {
                    // Cyclic over frame_count keys: the last blends back into the first.
                    const float span = static_cast<float>(frame_count);
                    float local = std::fmod(time_seconds * sample_rate, span);
                    if (local < 0.0f)
                        local += span;
                    frame_position = local;
                    frame0 = static_cast<std::uint32_t>(local) % frame_count;
                    frame1 = (frame0 + 1) % frame_count;
                }
                else
                {
                    const float last = static_cast<float>(frame_count - 1);
                    float local = time_seconds * sample_rate;
                    if (local < 0.0f)
                        local = 0.0f;
                    if (local > last)
                        local = last;
                    frame_position = local;
                    frame0 = static_cast<std::uint32_t>(local);
                    frame1 = frame0 + 1 < frame_count ? frame0 + 1 : frame0;
                }
                const float alpha = frame_position - std::floor(frame_position);

                if (format == ClipFormat::Compressed)
                {
                    for (std::uint32_t j = 0; j < joint_count; ++j)
                    {
                        out_translations[j] = lerp(compressed.translation_at(frame0, j),
                                                   compressed.translation_at(frame1, j), alpha);
                        out_scales[j] = lerp(compressed.scale_at(frame0, j),
                                             compressed.scale_at(frame1, j), alpha);
                        out_rotations[j] = nlerp(compressed.rotation_at(frame0, j),
                                                 compressed.rotation_at(frame1, j), alpha);
                    }
                    return;
                }

                const std::uint32_t base0 = frame0 * joint_count;
                const std::uint32_t base1 = frame1 * joint_count;
                for (std::uint32_t j = 0; j < joint_count; ++j)
                {
                    out_translations[j] = lerp(translations[base0 + j], translations[base1 + j], alpha);
                    out_scales[j] = lerp(scales[base0 + j], scales[base1 + j], alpha);
                    out_rotations[j] = nlerp(rotations[base0 + j], rotations[base1 + j], alpha);
                }
            }

            /**
             * @brief The index of a morph track by target name hash, or -1.
             * @param name The FNV-1a 64 hash of the morph target name.
             */
            int find_morph(NameHash name) const noexcept
            {
                for (std::uint32_t i = 0; i < morph_track_count; ++i)
                    if (morph_names[i] == name)
                        return static_cast<int>(i);
                return -1;
            }

            /**
             * @brief The index of a generic track by property name hash, or -1.
             * @param name The FNV-1a 64 hash of the property name.
             */
            int find_generic(NameHash name) const noexcept
            {
                for (std::uint32_t i = 0; i < generic_track_count; ++i)
                    if (generic_names[i] == name)
                        return static_cast<int>(i);
                return -1;
            }

            /**
             * @brief Samples every morph-weight track at a time into a caller-owned array.
             * @param time_seconds Playback time (wrapped or clamped like @ref sample).
             * @param loop         Whether the clip loops.
             * @param out_weights  Receives @ref morph_track_count weights.
             */
            void sample_morph(float time_seconds, bool loop, float* out_weights) const noexcept
            {
                sample_float_tracks(morph_weights, morph_track_count, time_seconds, loop, out_weights);
            }

            /**
             * @brief Samples every generic float track at a time into a caller-owned array.
             * @param time_seconds Playback time (wrapped or clamped like @ref sample).
             * @param loop         Whether the clip loops.
             * @param out_values   Receives @ref generic_track_count values.
             */
            void sample_generic(float time_seconds, bool loop, float* out_values) const noexcept
            {
                sample_float_tracks(generic_values, generic_track_count, time_seconds, loop, out_values);
            }

            /**
             * @brief Brackets a time to two frames and the interpolation alpha between them.
             * @param time_seconds Playback time.
             * @param loop         Whether the clip loops.
             * @param frame0        Receives the lower frame.
             * @param frame1        Receives the upper frame.
             * @param alpha         Receives the interpolation weight in [0, 1].
             */
            void bracket(float time_seconds, bool loop, std::uint32_t& frame0, std::uint32_t& frame1,
                         float& alpha) const noexcept
            {
                if (frame_count <= 1)
                {
                    frame0 = frame1 = 0;
                    alpha = 0.0f;
                    return;
                }
                float position;
                if (loop)
                {
                    const float span = static_cast<float>(frame_count);
                    float local = std::fmod(time_seconds * sample_rate, span);
                    if (local < 0.0f)
                        local += span;
                    position = local;
                    frame0 = static_cast<std::uint32_t>(local) % frame_count;
                    frame1 = (frame0 + 1) % frame_count;
                }
                else
                {
                    const float last = static_cast<float>(frame_count - 1);
                    float local = time_seconds * sample_rate;
                    if (local < 0.0f)
                        local = 0.0f;
                    if (local > last)
                        local = last;
                    position = local;
                    frame0 = static_cast<std::uint32_t>(local);
                    frame1 = frame0 + 1 < frame_count ? frame0 + 1 : frame0;
                }
                alpha = position - std::floor(position);
            }

            private:
                /** @brief Samples a frame-major float-track set (`frame * count + track`). */
                void sample_float_tracks(const float* values, std::uint32_t count, float time_seconds,
                                         bool loop, float* out) const noexcept
                {
                    if (values == nullptr || count == 0)
                        return;
                    std::uint32_t frame0;
                    std::uint32_t frame1;
                    float alpha;
                    bracket(time_seconds, loop, frame0, frame1, alpha);
                    const std::uint32_t base0 = frame0 * count;
                    const std::uint32_t base1 = frame1 * count;
                    for (std::uint32_t t = 0; t < count; ++t)
                        out[t] = values[base0 + t] * (1.0f - alpha) + values[base1 + t] * alpha;
                }
        };
    } // namespace Animation
} // namespace SushiEngine
