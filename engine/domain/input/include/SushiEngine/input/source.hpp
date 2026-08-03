/**************************************************************************/
/* source.hpp                                                            */
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
 * @file source.hpp
 * @brief Where events come from — the source abstraction and its scripted backend.
 *
 * A source has one job: drain its pending input for the current host frame into a
 * caller-owned buffer. Nothing above a source ever queries it for state; state is
 * folded from the drained events by the @ref DeviceRegistry. That inversion is what
 * makes the backends substitutable (the design's Liskov guarantee): the SDL
 * translator, this scripted replayer, and the virtual-control synthesizer are
 * indistinguishable downstream because they only ever produce @ref InputEvent lists.
 *
 * @ref ScriptedInputSource is the header-only test and demo backend — a real
 * implementation, not a mock, per the project's no-mocks rule. It doubles as the
 * reader for recorded event streams (device-level replay) later.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/input/events.hpp>

namespace SushiEngine
{
    namespace Input
    {
        /**
         * @brief A producer of @ref InputEvent records for a host frame.
         *
         * The single method is a drain: it appends this frame's events to @p out and
         * leaves the source empty for the next frame. Implementations must not clear
         * @p out (several sources feed the same buffer in one frame).
         */
        class IInputSource
        {
            public:
                virtual ~IInputSource() = default;

                /**
                 * @brief Drains this source's events for the current host frame.
                 * @param out Buffer the events are appended to; never cleared by the source.
                 */
                virtual void poll(std::vector<InputEvent>& out) = 0;
        };

        /**
         * @brief A source that replays a programmed list of events, one frame at a time.
         *
         * Events are scheduled against a host frame index. Each @ref poll drains the
         * events scheduled for the current frame, then advances the frame counter, so a
         * test scripts a whole session up front and each `begin_frame` on the manager
         * plays one frame of it. @ref enqueue targets the current (not-yet-drained) frame
         * for the common "these happen now" case. The `frame` field of each emitted event
         * is stamped to match the frame it is drained on, so relative-axis accumulation
         * and replay see a coherent timeline.
         */
        class ScriptedInputSource final : public IInputSource
        {
            public:
                /**
                 * @brief Schedules @p event to be drained on host frame @p frame.
                 *
                 * @param frame The host frame index (0-based) the event surfaces on.
                 * @param event The event to emit; its `frame` field is overwritten with @p frame.
                 */
                void schedule(std::uint64_t frame, InputEvent event)
                {
                    event.frame = frame;
                    pending_.push_back(Scheduled{frame, event});
                }

                /**
                 * @brief Schedules @p event for the current frame (the next one @ref poll drains).
                 * @param event The event to emit now.
                 */
                void enqueue(InputEvent event)
                {
                    schedule(current_frame_, event);
                }

                /** @brief The host frame the next @ref poll will drain. */
                std::uint64_t current_frame() const noexcept { return current_frame_; }

                /** @brief Whether any scheduled event remains at or after the current frame. */
                bool empty() const noexcept
                {
                    for (const Scheduled& s : pending_)
                        if (s.frame >= current_frame_)
                            return false;
                    return true;
                }

                /** @brief Drops every scheduled event and rewinds to frame 0. */
                void reset() noexcept
                {
                    pending_.clear();
                    current_frame_ = 0;
                }

                /**
                 * @brief Drains the current frame's events into @p out and advances one frame.
                 * @param out Buffer the frame's events are appended to.
                 */
                void poll(std::vector<InputEvent>& out) override
                {
                    for (const Scheduled& s : pending_)
                        if (s.frame == current_frame_)
                            out.push_back(s.event);
                    ++current_frame_;
                }

            private:
                struct Scheduled
                {
                    std::uint64_t frame;
                    InputEvent event;
                };

                std::vector<Scheduled> pending_;
                std::uint64_t current_frame_ = 0;
        };
    } // namespace Input
} // namespace SushiEngine
