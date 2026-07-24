/**************************************************************************/
/* replay.hpp                                                            */
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
 * @file replay.hpp
 * @brief Device-level replay — capture the event stream, play it back through a scripted source.
 *
 * The recorder captures each frame's drained events (already frame-stamped by their source),
 * and replays them by scheduling them back into a @ref ScriptedInputSource on the same frames.
 * This reproduces *mapper* behaviour — the same devices, the same actions — and complements
 * `InputHistory` replay, which reproduces the *sim* from the reduced command stream (§6): the
 * two exist because the tick boundary is the design's central line. Because events are trivially
 * copyable, a game persists the captured stream however it likes; @ref replay_json.hpp offers a
 * JSON format for those that want a file. This header keeps the core dependency-free.
 */

#include <cstddef>
#include <vector>

#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/source.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief Captures a session's input events and replays them into a scripted source.
         *
         * Feed it each frame's events from @ref InputManager::frame_events after `begin_frame`;
         * the events carry the frame they occurred on, so @ref replay_into reproduces the original
         * timing exactly when the recorded source drove one event queue.
         */
        class InputRecorder
        {
            public:
                /**
                 * @brief Appends @p frame_events (one host frame's drained events) to the recording.
                 * @param frame_events The events drained this frame, already frame-stamped.
                 */
                void capture(const std::vector<InputEvent>& frame_events)
                {
                    events_.insert(events_.end(), frame_events.begin(), frame_events.end());
                }

                /** @brief The captured events, in order. */
                const std::vector<InputEvent>& events() const noexcept { return events_; }

                /** @brief Replaces the recording with @p events (e.g. loaded from a file). */
                void set_events(std::vector<InputEvent> events) { events_ = std::move(events); }

                /** @brief The number of captured events. */
                std::size_t size() const noexcept { return events_.size(); }

                /** @brief Whether nothing has been captured. */
                bool empty() const noexcept { return events_.empty(); }

                /** @brief Clears the recording. */
                void clear() noexcept { events_.clear(); }

                /**
                 * @brief Schedules every captured event into @p source on its recorded frame.
                 *
                 * After this, driving the manager's `begin_frame` once per frame replays the whole
                 * session; the scripted source drains each frame's events in recorded order.
                 *
                 * @param source The scripted source to load the recording into.
                 */
                void replay_into(ScriptedInputSource& source) const
                {
                    for (const InputEvent& event : events_)
                        source.schedule(event.frame, event);
                }

            private:
                std::vector<InputEvent> events_;
        };
    } // namespace Input
} // namespace SushiEngine
