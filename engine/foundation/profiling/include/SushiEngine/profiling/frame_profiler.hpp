/**************************************************************************/
/* frame_profiler.hpp                                                     */
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

#ifndef SUSHIENGINE_PROFILING_FRAME_PROFILER_HPP
#define SUSHIENGINE_PROFILING_FRAME_PROFILER_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace SushiEngine
{
    namespace Profiling
    {
        /** @brief Index of a registered channel; stable for the profiler's lifetime. */
        using ChannelId = std::uint32_t;

        /**
         * @brief The time source: a free function returning nanoseconds on a monotonic
         *        clock. Injected so tests advance time by hand; null selects the real clock.
         */
        using ProfilerClock = std::uint64_t (*)();

        /** @brief One channel's value in a completed frame's snapshot. */
        struct ChannelValue
        {
            std::string name;          /**< The name given at registration. */
            float milliseconds = 0.0f; /**< Inclusive time accumulated this frame. */
            std::uint32_t depth = 0;   /**< Nesting depth of the channel's last scope. */
        };

        /**
         * @brief One completed frame's CPU times, copied out for display.
         *
         * A copy rather than a reference, matching how the GPU timings reach the editor's
         * panels: the reader holds a frame's numbers and cannot reach back into the
         * profiler through them.
         */
        struct FrameProfileSnapshot
        {
            float frame_milliseconds = 0.0f;   /**< begin_frame() to end_frame(). */
            std::vector<ChannelValue> channels; /**< Indexed by ChannelId. */
            /** @brief Completed frames' totals, oldest first, at most HISTORY_FRAMES. */
            std::vector<float> frame_history;
        };

        /**
         * @brief Measures the main thread's frame: named channels, nested scopes, and a
         *        fixed ring of completed frames.
         *
         * Single-threaded by contract — every call comes from the thread that owns the
         * frame loop. A scope costs two clock reads and an add. Channels are registered
         * once at startup and referenced by index so the hot path never hashes a name.
         */
        class FrameProfiler
        {
            public:
                /** @brief Completed frames the history ring holds (four seconds at 60). */
                static constexpr std::size_t HISTORY_FRAMES = 240;

                /**
                 * @brief Builds an empty profiler.
                 * @param clock Nanosecond time source; null selects std::chrono::steady_clock.
                 */
                explicit FrameProfiler(ProfilerClock clock = nullptr);

                FrameProfiler(const FrameProfiler&) = delete;
                FrameProfiler& operator=(const FrameProfiler&) = delete;

                /**
                 * @brief Registers a channel before the first frame.
                 * @param name The display name; stored by copy.
                 * @return The channel's stable index.
                 */
                ChannelId register_channel(const char* name);

                /** @brief Opens a frame: stamps its start and clears every channel. */
                void begin_frame();

                /** @brief Closes the frame and pushes its totals into the history ring. */
                void end_frame();

                /**
                 * @brief Opens a scope on a channel; ScopedTimer is the intended caller.
                 * @param channel A value register_channel returned.
                 */
                void begin_scope(ChannelId channel);

                /**
                 * @brief Closes the innermost open scope on a channel and adds its time.
                 * @param channel The same value the matching begin_scope was given.
                 */
                void end_scope(ChannelId channel);

                /**
                 * @brief Copies the last completed frame and the history out for display.
                 * @return The snapshot; empty channels and zero totals before any frame.
                 */
                FrameProfileSnapshot snapshot() const;

            private:
                struct Channel
                {
                    std::string name;
                    std::uint64_t nanoseconds = 0;
                    std::uint64_t scope_start = 0;
                    std::uint32_t depth = 0;
                };

                std::uint64_t now() const;

                ProfilerClock clock_;
                std::vector<Channel> channels_;
                std::uint64_t frame_start_ = 0;
                std::uint64_t frame_nanoseconds_ = 0;
                std::uint32_t open_scopes_ = 0;
                std::vector<float> history_;
                std::size_t history_next_ = 0;
                std::size_t history_count_ = 0;
        };

        /**
         * @brief RAII scope: adds the elapsed time to its channel on destruction.
         *
         * Constructed on the stack around the code being measured, so an early return
         * cannot leave a scope open.
         */
        class ScopedTimer
        {
            public:
                /**
                 * @brief Opens a scope on the channel.
                 * @param profiler The profiler owning the channel.
                 * @param channel  A value register_channel returned.
                 */
                ScopedTimer(FrameProfiler& profiler, ChannelId channel);
                ~ScopedTimer();

                ScopedTimer(const ScopedTimer&) = delete;
                ScopedTimer& operator=(const ScopedTimer&) = delete;

            private:
                FrameProfiler& profiler_;
                ChannelId channel_;
        };
    } // namespace Profiling
} // namespace SushiEngine

#endif
