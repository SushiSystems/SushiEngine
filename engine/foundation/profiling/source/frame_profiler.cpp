/**************************************************************************/
/* frame_profiler.cpp                                                     */
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

#include <SushiEngine/profiling/frame_profiler.hpp>

#include <chrono>

namespace SushiEngine
{
    namespace Profiling
    {
        namespace
        {
            std::uint64_t steady_clock_nanoseconds()
            {
                return static_cast<std::uint64_t>(
                    std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::steady_clock::now().time_since_epoch())
                        .count());
            }

            float to_milliseconds(std::uint64_t nanoseconds)
            {
                return static_cast<float>(nanoseconds) * 1.0e-6f;
            }
        } // namespace

        FrameProfiler::FrameProfiler(ProfilerClock clock)
            : clock_(clock != nullptr ? clock : &steady_clock_nanoseconds)
        {
            history_.resize(HISTORY_FRAMES, 0.0f);
        }

        std::uint64_t FrameProfiler::now() const
        {
            return clock_();
        }

        ChannelId FrameProfiler::register_channel(const char* name)
        {
            Channel channel;
            channel.name = name;
            channels_.push_back(channel);
            return static_cast<ChannelId>(channels_.size() - 1);
        }

        void FrameProfiler::begin_frame()
        {
            frame_start_ = now();
            open_scopes_ = 0;
            for (Channel& channel : channels_)
                channel.nanoseconds = 0;
        }

        void FrameProfiler::end_frame()
        {
            frame_nanoseconds_ = now() - frame_start_;
            history_[history_next_] = to_milliseconds(frame_nanoseconds_);
            history_next_ = (history_next_ + 1) % HISTORY_FRAMES;
            if (history_count_ < HISTORY_FRAMES)
                ++history_count_;
        }

        void FrameProfiler::begin_scope(ChannelId channel)
        {
            channels_[channel].scope_start = now();
            channels_[channel].depth = open_scopes_;
            ++open_scopes_;
        }

        void FrameProfiler::end_scope(ChannelId channel)
        {
            channels_[channel].nanoseconds += now() - channels_[channel].scope_start;
            --open_scopes_;
        }

        FrameProfileSnapshot FrameProfiler::snapshot() const
        {
            FrameProfileSnapshot snapshot;
            snapshot.frame_milliseconds = to_milliseconds(frame_nanoseconds_);
            snapshot.channels.reserve(channels_.size());
            for (const Channel& channel : channels_)
            {
                ChannelValue value;
                value.name = channel.name;
                value.milliseconds = to_milliseconds(channel.nanoseconds);
                value.depth = channel.depth;
                snapshot.channels.push_back(value);
            }
            snapshot.frame_history.reserve(history_count_);
            for (std::size_t i = 0; i < history_count_; ++i)
            {
                const std::size_t oldest =
                    (history_next_ + HISTORY_FRAMES - history_count_) % HISTORY_FRAMES;
                snapshot.frame_history.push_back(history_[(oldest + i) % HISTORY_FRAMES]);
            }
            return snapshot;
        }

        ScopedTimer::ScopedTimer(FrameProfiler& profiler, ChannelId channel)
            : profiler_(profiler), channel_(channel)
        {
            profiler_.begin_scope(channel_);
        }

        ScopedTimer::~ScopedTimer()
        {
            profiler_.end_scope(channel_);
        }
    } // namespace Profiling
} // namespace SushiEngine
