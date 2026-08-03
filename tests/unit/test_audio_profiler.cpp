/**************************************************************************/
/* test_audio_profiler.cpp                                                */
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

// Unit_Audio: the phase-S9 live-profiler telemetry channel — the seqlock audio→GUI
// snapshot (publish/latest, empty before first publish), the per-bus meters on the mixer,
// and the AudioEngine gather (real/virtual/active voice counts, master peak/RMS tracking
// the signal, per-bus meters, and the downsampled scope). Header-only, no device.

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // Render `blocks` blocks of `n` frames through an engine, returning its latest snapshot.
    AudioProfileSnapshot run(AudioEngine& engine, int n, int blocks)
    {
        std::vector<float> l(static_cast<std::size_t>(n), 0.0f);
        std::vector<float> r(static_cast<std::size_t>(n), 0.0f);
        float* ch[2] = {l.data(), r.data()};
        for (int b = 0; b < blocks; ++b)
            engine.render(ch, 2, n);
        AudioProfileSnapshot s;
        engine.profiler().latest(s);
        return s;
    }
}

// The seqlock channel: empty before first publish, then returns the latest value.
TEST(Unit_Audio, ProfilerSeqlockChannel)
{
    AudioProfiler profiler;
    AudioProfileSnapshot out;
    EXPECT_FALSE(profiler.latest(out)); // nothing published yet
    EXPECT_EQ(profiler.published_count(), 0u);

    AudioProfileSnapshot in;
    in.block_index = 7;
    in.real_voices = 3;
    in.master_peak = 0.5f;
    profiler.publish(in);

    ASSERT_TRUE(profiler.latest(out));
    EXPECT_EQ(out.block_index, 7u);
    EXPECT_EQ(out.real_voices, 3);
    EXPECT_FLOAT_EQ(out.master_peak, 0.5f);
    EXPECT_EQ(profiler.published_count(), 1u);

    in.block_index = 8;
    in.real_voices = 1;
    profiler.publish(in);
    ASSERT_TRUE(profiler.latest(out));
    EXPECT_EQ(out.block_index, 8u); // always the newest
    EXPECT_EQ(out.real_voices, 1);
}

// The engine gathers voice counts and a master meter that tracks the signal.
TEST(Unit_Audio, ProfilerEngineGather)
{
    AudioEngine engine(8, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(48000.0, 512);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    // Silence first: no voices → master meter reads ~0.
    {
        const AudioProfileSnapshot s = run(engine, 512, 4);
        EXPECT_EQ(s.active_voices, 0);
        EXPECT_LT(s.master_peak, 1.0e-4f);
        EXPECT_EQ(s.scope_points, 128);
        EXPECT_GE(s.bus_count, 1);
    }

    // A 2D tone → one real voice and a master meter that reads the signal.
    VoiceDescriptor d;
    d.base_gain = 0.7f;
    d.priority = 10.0f;
    d.bus = master;
    d.spatial = false;
    engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(440.0f, 0.7f)));

    const AudioProfileSnapshot s = run(engine, 512, 8);
    EXPECT_EQ(s.real_voices, 1);
    EXPECT_EQ(s.active_voices, 1);
    EXPECT_EQ(s.virtual_voices, 0);
    EXPECT_GT(s.master_peak, 0.1f);
    EXPECT_GT(s.master_rms, 0.0f);
    EXPECT_GT(s.buses[master].peak, 0.1f); // the master bus meter agrees
    EXPECT_GT(s.block_index, 0u);
}

// More audible voices than the real cap leaves the rest virtual — and the profiler shows it.
TEST(Unit_Audio, ProfilerVirtualVoiceCount)
{
    AudioEngine engine(16, 3); // cap 3 real voices
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(48000.0, 256);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    for (int i = 0; i < 8; ++i)
    {
        VoiceDescriptor d;
        d.base_gain = 0.3f;
        d.priority = static_cast<float>(i); // distinct priorities, all audible
        d.bus = master;
        d.spatial = false;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(220.0f + 20.0f * i, 0.3f)));
    }

    const AudioProfileSnapshot s = run(engine, 256, 4);
    EXPECT_EQ(s.active_voices, 8);
    EXPECT_EQ(s.real_voices, 3);
    EXPECT_EQ(s.virtual_voices, 5);
}

// Disabling profiling stops new publishes.
TEST(Unit_Audio, ProfilerCanBeDisabled)
{
    AudioEngine engine(4, 2);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(48000.0, 256);

    run(engine, 256, 3);
    const std::uint32_t after_on = engine.profiler().published_count();
    EXPECT_GT(after_on, 0u);

    engine.set_profiling(false);
    run(engine, 256, 5);
    EXPECT_EQ(engine.profiler().published_count(), after_on); // frozen
}

// The K-weighted loudness meter (LUFS) reports a plausible value that rises with level.
TEST(Unit_Audio, ProfilerLoudnessMeter)
{
    auto measure_lufs = [](float gain) {
        AudioEngine engine(4, 4);
        const int master = engine.mixer().add_bus(NO_BUS);
        engine.mixer().set_master(master);
        engine.prepare(48000.0, 512);
        VoiceDescriptor d;
        d.base_gain = gain;
        d.bus = master;
        d.spatial = false;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(1000.0f, 1.0f)));
        const AudioProfileSnapshot s = run(engine, 512, 120); // settle the 400 ms window
        return s.master_lufs;
    };

    const float loud = measure_lufs(0.8f);
    const float quiet = measure_lufs(0.2f);
    EXPECT_GT(loud, quiet);         // louder → higher LUFS
    EXPECT_GT(loud, -40.0f);        // a plausible programme loudness
    EXPECT_LT(loud, 6.0f);          // not absurd
    EXPECT_NEAR(loud - quiet, 12.0f, 4.0f); // ~4× amplitude ≈ +12 dB
}

// True (inter-sample) peak is never below the sample peak, and exceeds it for a
// near-Nyquist signal whose real peak falls between samples.
TEST(Unit_Audio, ProfilerTruePeak)
{
    AudioEngine engine(4, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(48000.0, 512);
    VoiceDescriptor d;
    d.base_gain = 0.9f;
    d.bus = master;
    d.spatial = false;
    // A high tone (near a quarter of Nyquist and up) puts energy between samples.
    engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(11000.0f, 0.9f)));

    const AudioProfileSnapshot s = run(engine, 512, 8);
    EXPECT_GT(s.master_true_peak, 0.0f);
    EXPECT_GE(s.master_true_peak, s.master_peak - 1.0e-4f); // true peak never below the sample peak
}
