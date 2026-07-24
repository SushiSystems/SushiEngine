/**************************************************************************/
/* test_audio_mixer.cpp                                                  */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_Audio: the phase-S2 action layer — parameter smoothing / RTPC, the mixer bus
// DAG (insert / post-fader gain / aux send / topological order), and the voice
// manager's virtual/real ranking, distance culling, one-shot lifetime, and panning.
// Pure header-only maths against the real code, no device and no mocks.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

TEST(Unit_Audio, SmoothedValueRampsToTargetAndSnaps)
{
    SmoothedValue value;
    value.configure(0.01, 48000.0); // 10 ms per unit
    value.snap(0.0f);
    value.set_target(1.0f);

    // Not there after one short block, but monotonically approaching.
    float s0 = 0.0f, e0 = 0.0f;
    value.advance_block(64, s0, e0);
    EXPECT_FLOAT_EQ(s0, 0.0f);
    EXPECT_GT(e0, 0.0f);
    EXPECT_LT(e0, 1.0f);

    // Reaches the target after enough samples (>= 480 total for 10 ms at 48 kHz).
    for (int i = 0; i < 100; ++i)
        value.advance_block(64);
    EXPECT_FLOAT_EQ(value.current(), 1.0f);

    value.snap(0.25f);
    EXPECT_FLOAT_EQ(value.current(), 0.25f);
}

TEST(Unit_Audio, RtpcMapsThroughCurveWithClamping)
{
    RtpcCurve curve;
    curve.add_point(0.0f, 0.0f);
    curve.add_point(100.0f, 1.0f);
    EXPECT_FLOAT_EQ(curve.evaluate(50.0f), 0.5f);
    EXPECT_FLOAT_EQ(curve.evaluate(-20.0f), 0.0f); // clamp low
    EXPECT_FLOAT_EQ(curve.evaluate(999.0f), 1.0f); // clamp high

    Rtpc rtpc;
    rtpc.configure(0.0, 48000.0); // instant
    rtpc.set_curve(curve);
    rtpc.snap(0.0f);
    rtpc.set(75.0f);
    EXPECT_FLOAT_EQ(rtpc.value().advance_block(64), 0.75f);
}

TEST(Unit_Audio, MixerAuxSendAddsScaledSignalToTarget)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    const int dry = mixer.add_bus(master);
    const int reverb = mixer.add_bus(master);
    mixer.set_master(master);
    mixer.add_aux_send(dry, reverb, 0.5f);
    mixer.prepare(48000.0, 64);

    std::vector<float> mono(64, 1.0f);
    mixer.begin_block(64);
    mixer.accumulate(dry, mono.data(), 64, 1.0f, 1.0f); // dry = 1.0 both channels
    mixer.process(64);

    // master = dry routed (1.0) + reverb (dry * 0.5 send, routed) = 1.5.
    EXPECT_NEAR(mixer.master_left()[0], 1.5f, 1e-5f);
    EXPECT_NEAR(mixer.master_right()[0], 1.5f, 1e-5f);
}

TEST(Unit_Audio, MixerPostFaderGainScalesOutput)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    const int dry = mixer.add_bus(master);
    mixer.set_master(master);
    mixer.prepare(48000.0, 64);
    mixer.bus_gain(dry).snap(0.5f);

    std::vector<float> mono(64, 1.0f);
    mixer.begin_block(64);
    mixer.accumulate(dry, mono.data(), 64, 1.0f, 1.0f);
    mixer.process(64);

    EXPECT_NEAR(mixer.master_left()[0], 0.5f, 1e-5f);
}

TEST(Unit_Audio, MixerEvaluatesContributorsBeforeConsumers)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    const int dry = mixer.add_bus(master);
    const int reverb = mixer.add_bus(master);
    mixer.add_aux_send(dry, reverb, 0.3f);
    mixer.prepare(48000.0, 64);

    const std::vector<int>& order = mixer.order();
    auto pos = [&](int id) {
        for (std::size_t i = 0; i < order.size(); ++i)
            if (order[i] == id)
                return static_cast<int>(i);
        return -1;
    };
    EXPECT_LT(pos(dry), pos(reverb));   // sender before aux target
    EXPECT_LT(pos(dry), pos(master));   // child before parent
    EXPECT_LT(pos(reverb), pos(master));
}

TEST(Unit_Audio, VoiceManagerCapsRealVoicesByPriority)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.prepare(48000.0, 64);

    VoiceManager voices(8, 3);
    voices.prepare(48000.0, 64);

    int handles[8];
    for (int i = 0; i < 8; ++i)
    {
        VoiceDescriptor d;
        d.base_gain = 1.0f;
        d.priority = static_cast<float>(i); // handle i has priority i
        d.bus = master;
        handles[i] = voices.play(d, std::unique_ptr<VoiceSource>(new ToneSource(440.0f, 1.0f)));
        ASSERT_NE(handles[i], INVALID_VOICE);
    }

    mixer.begin_block(64);
    voices.render(mixer, 64);

    EXPECT_EQ(voices.real_count(), 3);
    // The three highest priorities (7, 6, 5) are real; the rest virtual.
    EXPECT_EQ(voices.state_of(handles[7]), VoiceState::Real);
    EXPECT_EQ(voices.state_of(handles[6]), VoiceState::Real);
    EXPECT_EQ(voices.state_of(handles[5]), VoiceState::Real);
    EXPECT_EQ(voices.state_of(handles[4]), VoiceState::Virtual);
    EXPECT_EQ(voices.state_of(handles[0]), VoiceState::Virtual);
}

TEST(Unit_Audio, VoiceManagerVirtualizesInaudibleDistantVoices)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.prepare(48000.0, 64);

    VoiceManager voices(4, 4); // cap high enough that only audibility culls
    voices.prepare(48000.0, 64);
    voices.set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    VoiceDescriptor music;
    music.base_gain = 1.0f;
    music.priority = 0.0f;
    music.bus = master;
    const int m0 = voices.play(music, std::unique_ptr<VoiceSource>(new ToneSource(220.0f, 1.0f)));
    const int m1 = voices.play(music, std::unique_ptr<VoiceSource>(new ToneSource(330.0f, 1.0f)));

    VoiceDescriptor far;
    far.base_gain = 1.0f;
    far.priority = 100.0f; // even at max priority, silent -> virtual
    far.bus = master;
    far.spatial = true;
    far.position = AudioVec3{1000.0f, 0.0f, 0.0f};
    far.min_distance = 1.0f;
    far.max_distance = 50.0f;
    const int f0 = voices.play(far, std::unique_ptr<VoiceSource>(new ToneSource(660.0f, 1.0f)));

    mixer.begin_block(64);
    voices.render(mixer, 64);

    EXPECT_EQ(voices.real_count(), 2); // only the two audible non-spatial voices
    EXPECT_EQ(voices.state_of(m0), VoiceState::Real);
    EXPECT_EQ(voices.state_of(m1), VoiceState::Real);
    EXPECT_EQ(voices.state_of(f0), VoiceState::Virtual);
}

TEST(Unit_Audio, VoiceManagerCentersPanAndMixesIntoBus)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.prepare(48000.0, 128);

    VoiceManager voices(2, 2);
    voices.prepare(48000.0, 128);

    VoiceDescriptor d;
    d.base_gain = 0.5f;
    d.priority = 1.0f;
    d.bus = master;
    d.pan = 0.0f; // centre -> equal L/R
    voices.play(d, std::unique_ptr<VoiceSource>(new ToneSource(500.0f, 1.0f)));

    double energy = 0.0;
    for (int block = 0; block < 4; ++block)
    {
        mixer.begin_block(128);
        voices.render(mixer, 128);
        mixer.process(128);
        const float* left = mixer.master_left();
        const float* right = mixer.master_right();
        for (int i = 0; i < 128; ++i)
        {
            EXPECT_NEAR(left[i], right[i], 1e-6f); // centred: identical channels
            energy += std::fabs(static_cast<double>(left[i]));
        }
    }
    EXPECT_GT(energy, 0.0); // not silent
}

TEST(Unit_Audio, VoiceManagerFreesFinishedOneShot)
{
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.prepare(48000.0, 64);

    VoiceManager voices(2, 2);
    voices.prepare(48000.0, 64);

    static std::vector<float> sample(100, 0.5f); // 100 samples, non-looping
    VoiceDescriptor d;
    d.base_gain = 1.0f;
    d.priority = 1.0f;
    d.bus = master;
    const int handle =
        voices.play(d, std::unique_ptr<VoiceSource>(new BufferSource(sample.data(), 100, false)));
    ASSERT_NE(handle, INVALID_VOICE);
    EXPECT_EQ(voices.active_count(), 1);

    // 100 samples span two 64-frame blocks; after that the voice frees itself.
    for (int block = 0; block < 3; ++block)
    {
        mixer.begin_block(64);
        voices.render(mixer, 64);
    }
    EXPECT_EQ(voices.active_count(), 0);
    EXPECT_EQ(voices.state_of(handle), VoiceState::Free);
}

TEST(Unit_Audio, AudioEngineRendersBoundedStereo)
{
    AudioEngine engine(8, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    const int dry = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);
    engine.prepare(48000.0, 256);

    for (int i = 0; i < 4; ++i)
    {
        VoiceDescriptor d;
        d.base_gain = 0.2f;
        d.priority = static_cast<float>(i);
        d.bus = dry;
        d.pan = (i % 2 == 0) ? -0.5f : 0.5f;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(200.0f + 50.0f * i, 1.0f)));
    }

    std::vector<float> left(256, 0.0f), right(256, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    double energy = 0.0, peak = 0.0;
    for (int block = 0; block < 8; ++block)
    {
        engine.render(channels, 2, 256);
        for (int i = 0; i < 256; ++i)
        {
            peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
            peak = std::max(peak, std::fabs(static_cast<double>(right[i])));
            energy += std::fabs(static_cast<double>(left[i]));
        }
    }
    EXPECT_EQ(engine.voices().real_count(), 4);
    EXPECT_GT(energy, 0.0);
    EXPECT_LE(peak, 1.0);
}
