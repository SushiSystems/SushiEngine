/**************************************************************************/
/* sequence_timeline.hpp                                                 */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

/**
 * @file sequence_timeline.hpp
 * @brief A minimal cinematics/sequencer timeline core (design §12.4).
 *
 * Named in §12.4 as never-scoped work: "no scene-cutscene authoring layer (Unreal Sequencer
 * equivalent) sits above the Animator; multi-character/camera choreography would be
 * hand-wired today." @ref SequenceTimeline is the evaluation core such a layer would sit on
 * top of — the same relationship `motion_matching.hpp`'s `MotionDatabase` has to a full
 * motion-matching feature, or `sequence_timeline.hpp` here has to an actual Sequencer editor
 * window: real and useful standalone (script a scene by constructing tracks and events in
 * code, or read them from a small data file — this header does not care which), but not an
 * authoring UI, which is a genuinely separate, much larger project.
 *
 * Two kinds of track, matching what a scene actually needs to drive:
 *   * @ref SequenceFloatTrack — keyed float curves written into an
 *     `AnimatorParameterBlock` (design's existing `AnimatorEvaluator` parameter seam, e.g.
 *     to blend a camera's field-of-view or an actor's `AnimatorEvaluator` blend-tree
 *     parameter over the scene) via @ref SequenceTimeline::evaluate — a pure function of
 *     time, safe to call from any scrub position, including scrubbing backward in an editor.
 *   * @ref SequenceEvent — one-shot triggers (start a clip, fire a sound cue, cut a camera)
 *     dispatched by @ref SequenceTimeline::advance, which fires every event whose time falls
 *     in `(previous_time, current_time]` — forward playback only; advancing across several
 *     events in one large time step (e.g. a dropped frame) fires all of them, in time order,
 *     not just the last. Scrubbing backward (`current_time < previous_time`) fires nothing;
 *     looping or seeking is the caller's responsibility (call with `previous_time` reset to
 *     match, not something this header guesses at).
 *
 * Deliberately not built here: nested sub-sequences, blend/crossfade between sequence
 * segments, an editor timeline widget, or serialization — a real authoring layer's job, not
 * this evaluation core's.
 */

#include <algorithm>
#include <cstdint>
#include <vector>

#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief One keyframe on a @ref SequenceFloatTrack. */
        struct SequenceFloatKey
        {
            float time = 0.0f;
            float value = 0.0f;
        };

        /**
         * @brief A keyed float curve driving one `AnimatorParameterBlock` slot over the scene.
         *
         * **Keys must ascend in time**, and @ref sort_keys is how a caller that bulk-loaded them
         * guarantees it. Sorting inside @ref sample instead would put a copy and a sort on the
         * per-frame path for a property that is fixed the moment the track is authored — and a
         * timeline is authored once and evaluated every frame, so the cost belongs at authoring.
         * Out-of-order keys are not rejected because there is nowhere to report from: @ref sample
         * is `noexcept` and returns a value, so it walks the keys in the order it finds them and
         * an unsorted track simply reads the wrong segment.
         */
        struct SequenceFloatTrack
        {
            std::uint32_t parameter_index = 0; /**< Slot in the target `AnimatorParameterBlock`. */
            std::vector<SequenceFloatKey> keys;

            /** @brief Sorts @ref keys ascending by time, which @ref sample requires. */
            void sort_keys()
            {
                std::sort(keys.begin(), keys.end(),
                          [](const SequenceFloatKey& a, const SequenceFloatKey& b)
                          { return a.time < b.time; });
            }

            /**
             * @brief Linearly interpolates the track's value at a time, clamped at the ends.
             * @param time_seconds The timeline time to sample.
             * @return The interpolated value, or 0 if the track has no keys.
             */
            float sample(float time_seconds) const noexcept
            {
                if (keys.empty())
                    return 0.0f;
                if (time_seconds <= keys.front().time)
                    return keys.front().value;
                if (time_seconds >= keys.back().time)
                    return keys.back().value;
                for (std::size_t i = 1; i < keys.size(); ++i)
                {
                    if (time_seconds > keys[i].time)
                        continue;
                    const SequenceFloatKey& a = keys[i - 1];
                    const SequenceFloatKey& b = keys[i];
                    const float span = b.time - a.time;
                    const float t = span > 1e-8f ? (time_seconds - a.time) / span : 0.0f;
                    return a.value + (b.value - a.value) * t;
                }
                return keys.back().value;
            }
        };

        /** @brief A one-shot trigger at a fixed timeline time (design §12.4's cue/cut/clip start). */
        struct SequenceEvent
        {
            float time = 0.0f;
            std::uint32_t event_id = 0; /**< Caller-defined; this header does not interpret it. */
        };

        /**
         * @brief A scene's float-parameter tracks and one-shot events over a shared timeline.
         */
        class SequenceTimeline
        {
            public:
                std::vector<SequenceFloatTrack> float_tracks;
                std::vector<SequenceEvent> events;

                /**
                 * @brief Sorts every track's keys, so a bulk-loaded timeline is ready to evaluate.
                 *
                 * Called once after authoring or loading. @ref events needs no equivalent —
                 * @ref advance sorts what it fired rather than what it holds, because only the
                 * crossed subset has to be ordered.
                 */
                void sort_tracks()
                {
                    for (SequenceFloatTrack& track : float_tracks)
                        track.sort_keys();
                }

                /**
                 * @brief Writes every float track's value at @p time_seconds into a parameter block.
                 * @param time_seconds The timeline time to sample.
                 * @param parameters   Receives each track's value at its own `parameter_index`.
                 */
                void evaluate(float time_seconds, AnimatorParameterBlock& parameters) const
                {
                    for (const SequenceFloatTrack& track : float_tracks)
                        parameters.set_float(track.parameter_index, track.sample(time_seconds));
                }

                /**
                 * @brief Collects every event whose time falls in `(previous_time, current_time]`.
                 *
                 * Forward-only: if `current_time <= previous_time` (a scrub backward, or no time
                 * passing this call), nothing fires. Events are appended in time order, so a
                 * caller processing a large step (several events crossed at once) sees them in
                 * the order they would have played at real-time speed.
                 *
                 * @param previous_time The timeline time as of the previous call.
                 * @param current_time  The timeline time now.
                 * @param out_event_ids Receives the fired events' ids, appended (not cleared first).
                 */
                void advance(float previous_time, float current_time,
                            std::vector<std::uint32_t>& out_event_ids) const
                {
                    if (current_time <= previous_time)
                        return;
                    std::vector<const SequenceEvent*> fired;
                    for (const SequenceEvent& event : events)
                        if (event.time > previous_time && event.time <= current_time)
                            fired.push_back(&event);
                    std::sort(fired.begin(), fired.end(),
                             [](const SequenceEvent* a, const SequenceEvent* b)
                             { return a->time < b->time; });
                    for (const SequenceEvent* event : fired)
                        out_event_ids.push_back(event->event_id);
                }
        };
    } // namespace Animation
} // namespace SushiEngine
