/**************************************************************************/
/* test_frame_profiler.cpp                                                */
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

#include <gtest/gtest.h>

#include <SushiEngine/profiling/frame_profiler.hpp>

namespace
{
    // The injected clock: a file-local tick counter the tests advance by hand, so every
    // duration below is exact and the suite never sleeps.
    std::uint64_t fake_now_nanoseconds = 0;
    std::uint64_t fake_clock()
    {
        return fake_now_nanoseconds;
    }
} // namespace

TEST(Unit_FrameProfiler, AScopeAddsItsElapsedTimeToItsChannel)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 2'000'000; // 2 ms
    }
    fake_now_nanoseconds += 1'000'000; // 1 ms outside any scope
    profiler.end_frame();
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    EXPECT_FLOAT_EQ(snapshot.channels[tick].milliseconds, 2.0f);
    EXPECT_FLOAT_EQ(snapshot.frame_milliseconds, 3.0f);
}

TEST(Unit_FrameProfiler, TwoScopesOnOneChannelAccumulateWithinTheFrame)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 1'000'000;
    }
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 500'000;
    }
    profiler.end_frame();
    EXPECT_FLOAT_EQ(profiler.snapshot().channels[tick].milliseconds, 1.5f);
}

TEST(Unit_FrameProfiler, ANewFrameClearsTheLastFramesChannelTimes)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
        fake_now_nanoseconds += 1'000'000;
    }
    profiler.end_frame();
    profiler.begin_frame();
    profiler.end_frame();
    EXPECT_FLOAT_EQ(profiler.snapshot().channels[tick].milliseconds, 0.0f);
}

TEST(Unit_FrameProfiler, NestedScopesRecordDepthAndInclusiveTime)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    const SushiEngine::Profiling::ChannelId outer = profiler.register_channel("outer");
    const SushiEngine::Profiling::ChannelId inner = profiler.register_channel("inner");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer outer_timer(profiler, outer);
        fake_now_nanoseconds += 1'000'000;
        {
            SushiEngine::Profiling::ScopedTimer inner_timer(profiler, inner);
            fake_now_nanoseconds += 2'000'000;
        }
    }
    profiler.end_frame();
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    EXPECT_FLOAT_EQ(snapshot.channels[outer].milliseconds, 3.0f); // inclusive
    EXPECT_FLOAT_EQ(snapshot.channels[inner].milliseconds, 2.0f);
    EXPECT_EQ(snapshot.channels[outer].depth, 0u);
    EXPECT_EQ(snapshot.channels[inner].depth, 1u);
}

TEST(Unit_FrameProfiler, TheHistoryRingHoldsTheLast240FramesOldestFirst)
{
    fake_now_nanoseconds = 0;
    SushiEngine::Profiling::FrameProfiler profiler(&fake_clock);
    for (int frame = 0; frame < 300; ++frame)
    {
        profiler.begin_frame();
        fake_now_nanoseconds += 1'000'000 * static_cast<std::uint64_t>(frame + 1);
        profiler.end_frame();
    }
    const SushiEngine::Profiling::FrameProfileSnapshot snapshot = profiler.snapshot();
    ASSERT_EQ(snapshot.frame_history.size(),
              SushiEngine::Profiling::FrameProfiler::HISTORY_FRAMES);
    // Frames 61..300 survive: the oldest entry is frame 61's 61 ms total.
    EXPECT_FLOAT_EQ(snapshot.frame_history.front(), 61.0f);
    EXPECT_FLOAT_EQ(snapshot.frame_history.back(), 300.0f);
}

TEST(Unit_FrameProfiler, TheDefaultClockProducesNonNegativeTimes)
{
    SushiEngine::Profiling::FrameProfiler profiler;
    const SushiEngine::Profiling::ChannelId tick = profiler.register_channel("tick");
    profiler.begin_frame();
    {
        SushiEngine::Profiling::ScopedTimer timer(profiler, tick);
    }
    profiler.end_frame();
    EXPECT_GE(profiler.snapshot().frame_milliseconds, 0.0f);
}
