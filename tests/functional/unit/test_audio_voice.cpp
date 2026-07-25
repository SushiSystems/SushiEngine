/**************************************************************************/
/* test_audio_voice.cpp                                                 */
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

// Unit_Audio: voice-layer additions — sample-rate conversion (a buffer authored at one
// rate keeps its pitch on a device at another rate), per-voice pitch (ToneSource and
// BufferSource shift by the multiplier), and voice stealing (a full pool evicts the
// least-important voice for a higher-priority newcomer, and refuses a lower one).

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    // Estimate frequency from positive-going zero crossings over a rendered span.
    double estimate_frequency(VoiceSource& src, double sample_rate, int total)
    {
        std::vector<float> buf(512);
        int rendered = 0;
        int crossings = 0;
        float prev = 0.0f;
        bool have_prev = false;
        while (rendered < total)
        {
            const int n = std::min(512, total - rendered);
            src.render(buf.data(), n);
            for (int i = 0; i < n; ++i)
            {
                if (have_prev && prev <= 0.0f && buf[i] > 0.0f)
                    ++crossings;
                prev = buf[i];
                have_prev = true;
            }
            rendered += n;
        }
        const double seconds = total / sample_rate;
        return crossings / seconds;
    }
}

// A buffer authored at 24 kHz plays at its true pitch on a 48 kHz device (SRC).
TEST(Unit_Audio, BufferSourceSampleRateConversion)
{
    const double source_rate = 24000.0;
    const double device_rate = 48000.0;
    const double tone_hz = 1000.0;
    const int src_len = 24000; // 1 s at the source rate
    std::vector<float> data(static_cast<std::size_t>(src_len));
    for (int i = 0; i < src_len; ++i)
        data[static_cast<std::size_t>(i)] =
            static_cast<float>(std::sin(2.0 * 3.14159265358979 * tone_hz * i / source_rate));

    BufferSource src(data.data(), src_len, true, source_rate);
    src.prepare(device_rate, 512);
    const double f = estimate_frequency(src, device_rate, static_cast<int>(device_rate)); // 1 s
    EXPECT_NEAR(f, tone_hz, 15.0); // pitch preserved despite the rate mismatch

    // Without SRC (source_rate = 0 → ratio 1) the same buffer plays an octave up.
    BufferSource naive(data.data(), src_len, true, 0.0);
    naive.prepare(device_rate, 512);
    const double f_naive = estimate_frequency(naive, device_rate, static_cast<int>(device_rate));
    EXPECT_NEAR(f_naive, tone_hz * (device_rate / source_rate), 40.0); // ~2000 Hz
}

// Per-voice pitch shifts a tone by the multiplier.
TEST(Unit_Audio, PitchShiftsTone)
{
    ToneSource tone(440.0f, 0.8f);
    tone.prepare(48000.0, 512);
    tone.set_pitch(2.0f);
    const double f = estimate_frequency(tone, 48000.0, 48000);
    EXPECT_NEAR(f, 880.0, 10.0);
}

// A full pool steals the least-important voice for a higher-priority sound, and
// refuses one that does not outrank the incumbents.
TEST(Unit_Audio, VoiceStealing)
{
    VoiceManager voices(2, 2);
    voices.prepare(48000.0, 512);

    VoiceDescriptor low;
    low.base_gain = 0.5f;
    low.priority = 1.0f;
    low.spatial = false;
    const int a = voices.play(low, std::unique_ptr<VoiceSource>(new ToneSource(200.0f)));
    const int b = voices.play(low, std::unique_ptr<VoiceSource>(new ToneSource(300.0f)));
    ASSERT_NE(a, INVALID_VOICE);
    ASSERT_NE(b, INVALID_VOICE);
    EXPECT_EQ(voices.active_count(), 2);

    // Another low-priority voice cannot displace an equal/again-low incumbent.
    VoiceDescriptor low2 = low;
    low2.base_gain = 0.1f; // quieter → does not outrank
    const int rejected = voices.play(low2, std::unique_ptr<VoiceSource>(new ToneSource(400.0f)));
    EXPECT_EQ(rejected, INVALID_VOICE);
    EXPECT_EQ(voices.active_count(), 2);

    // A high-priority voice steals a slot (pool stays full, but one incumbent is gone).
    VoiceDescriptor high;
    high.base_gain = 0.9f;
    high.priority = 10.0f;
    high.spatial = false;
    const int stolen = voices.play(high, std::unique_ptr<VoiceSource>(new ToneSource(500.0f)));
    EXPECT_NE(stolen, INVALID_VOICE);
    EXPECT_EQ(voices.active_count(), 2);
    EXPECT_EQ(voices.state_of(stolen) != VoiceState::Free, true);
}

// 5.1 output routes discretely: a hard-left bed feeds FL more than FR, the centre in
// between, the LFE carries low-frequency energy — not a copy of the front-left channel.
TEST(Unit_Audio, SurroundOutputRouting)
{
    const double sr = 48000.0;
    const int block = 512;
    AudioEngine engine(8, 4);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(sr, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    VoiceDescriptor d;
    d.base_gain = 0.8f;
    d.priority = 10.0f;
    d.bus = master;
    d.spatial = false;
    d.pan = -1.0f; // hard left
    engine.voices().play(d, std::unique_ptr<VoiceSource>(new ToneSource(120.0f, 0.8f)));

    // 5.1: FL, FR, C, LFE, SL, SR.
    std::vector<std::vector<float>> chans(6, std::vector<float>(block, 0.0f));
    std::vector<float*> ptrs(6);
    for (int c = 0; c < 6; ++c)
        ptrs[c] = chans[c].data();

    double peak[6] = {0, 0, 0, 0, 0, 0};
    for (int b = 0; b < 20; ++b)
    {
        engine.render(ptrs.data(), 6, block);
        for (int c = 0; c < 6; ++c)
            for (int i = 0; i < block; ++i)
                peak[c] = std::fmax(peak[c], std::fabs(static_cast<double>(chans[c][i])));
    }
    EXPECT_GT(peak[0], peak[1]);      // FL (left) louder than FR
    EXPECT_GT(peak[2], 0.0);          // centre carries the bed
    EXPECT_GT(peak[3], 0.0);          // LFE carries the 120 Hz bass
    EXPECT_LT(peak[1], peak[0]);      // FR is NOT a copy of FL
}

// HDR: a loud voice masks quieter ones, culling them to virtual even under the real cap.
TEST(Unit_Audio, HdrLoudnessCulling)
{
    VoiceManager voices(8, 8); // real cap high, so only HDR limits
    voices.prepare(48000.0, 256);
    voices.set_hdr_window(12.0); // 12 dB window

    VoiceDescriptor loud;
    loud.base_gain = 1.0f;
    loud.spatial = false;
    const int loud_h = voices.play(loud, std::unique_ptr<VoiceSource>(new ToneSource(440.0f)));

    VoiceDescriptor quiet;
    quiet.base_gain = 0.1f; // −20 dB, outside the 12 dB window
    quiet.spatial = false;
    const int q1 = voices.play(quiet, std::unique_ptr<VoiceSource>(new ToneSource(500.0f)));
    const int q2 = voices.play(quiet, std::unique_ptr<VoiceSource>(new ToneSource(600.0f)));

    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.set_master(master);
    mixer.prepare(48000.0, 256);
    voices.render(mixer, 256);

    EXPECT_EQ(voices.state_of(loud_h), VoiceState::Real);
    EXPECT_EQ(voices.state_of(q1), VoiceState::Virtual);
    EXPECT_EQ(voices.state_of(q2), VoiceState::Virtual);

    // Widen the window past the difference → the quiet voices come back as real.
    voices.set_hdr_window(60.0);
    voices.render(mixer, 256);
    EXPECT_EQ(voices.state_of(q1), VoiceState::Real);
}

// The lock-free command ring: control-thread enqueues are applied by render (the audio
// thread), and finished voices are reported back — no direct mutation of render state.
TEST(Unit_Audio, CommandRingApplyAndReport)
{
    VoiceManager voices(4, 4);
    voices.prepare(48000.0, 256);
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.set_master(master);
    mixer.prepare(48000.0, 256);

    VoiceDescriptor d;
    d.spatial = false;
    d.bus = master;
    d.base_gain = 0.8f;
    const int h = voices.enqueue_play(d, std::unique_ptr<VoiceSource>(new ToneSource(440.0f)));
    ASSERT_NE(h, INVALID_VOICE);
    EXPECT_EQ(voices.active_count(), 0); // not installed until the audio thread drains

    voices.render(mixer, 256);
    EXPECT_EQ(voices.active_count(), 1);
    EXPECT_EQ(voices.state_of(h), VoiceState::Real);

    // Enqueue a stop; it takes effect on the next render, and the handle then goes stale.
    voices.enqueue_stop(h);
    voices.render(mixer, 256);
    EXPECT_EQ(voices.active_count(), 0);
    EXPECT_EQ(voices.state_of(h), VoiceState::Free); // stale handle is safe

    int finished = INVALID_VOICE;
    bool reported = false;
    while (voices.poll_finished(finished))
        if (finished == h)
            reported = true;
    EXPECT_TRUE(reported);
}

// A finished one-shot is reported and its slot is recycled for the next enqueue.
TEST(Unit_Audio, CommandRingOneShotRecycles)
{
    VoiceManager voices(2, 2);
    voices.prepare(48000.0, 64);
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    mixer.set_master(master);
    mixer.prepare(48000.0, 64);

    static std::vector<float> clip(100, 0.5f); // ~2 blocks of 64, non-looping
    VoiceDescriptor d;
    d.spatial = false;
    d.bus = master;
    const int h = voices.enqueue_play(
        d, std::unique_ptr<VoiceSource>(new BufferSource(clip.data(), 100, false)));
    ASSERT_NE(h, INVALID_VOICE);

    // Render past the clip's end; the one-shot finishes and is reported.
    bool reported = false;
    for (int b = 0; b < 4; ++b)
    {
        voices.render(mixer, 64);
        int finished = INVALID_VOICE;
        while (voices.poll_finished(finished))
            if (finished == h)
                reported = true;
    }
    EXPECT_TRUE(reported);
    EXPECT_EQ(voices.active_count(), 0);

    // The recycled slot serves the next enqueue.
    const int h2 = voices.enqueue_play(d, std::unique_ptr<VoiceSource>(new ToneSource(220.0f)));
    EXPECT_NE(h2, INVALID_VOICE);
}

// Multi-core voice rendering is bit-identical to the single-threaded path (the parallel
// phase is disjoint per voice; the serial mixdown order is unchanged).
TEST(Unit_Audio, MultiCoreRenderMatchesSerial)
{
    const double sr = 48000.0;
    const int block = 256;
    const int voice_count = 16; // above the parallel threshold

    auto run = [&](VoiceRenderPool* pool) {
        AudioEngine engine(64, 64);
        const int master = engine.mixer().add_bus(NO_BUS);
        engine.mixer().set_master(master);
        engine.prepare(sr, block);
        engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});
        engine.voices().set_render_pool(pool);
        for (int i = 0; i < voice_count; ++i)
        {
            VoiceDescriptor d;
            d.base_gain = 0.2f;
            d.priority = static_cast<float>(i);
            d.bus = master;
            d.spatial = false;
            d.pan = (i % 2 == 0) ? -0.3f : 0.3f;
            engine.voices().play(
                d, std::unique_ptr<VoiceSource>(new ToneSource(200.0f + 25.0f * i, 0.2f)));
        }
        std::vector<float> out;
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        float* ch[2] = {l.data(), r.data()};
        for (int b = 0; b < 8; ++b)
        {
            engine.render(ch, 2, block);
            for (int i = 0; i < block; ++i)
                out.push_back(l[i]);
        }
        return out;
    };

    const std::vector<float> serial = run(nullptr);
    VoiceRenderPool pool(3); // 3 workers + the audio thread = 4 lanes
    const std::vector<float> parallel = run(&pool);

    ASSERT_EQ(serial.size(), parallel.size());
    for (std::size_t i = 0; i < serial.size(); ++i)
        EXPECT_EQ(serial[i], parallel[i]); // bit-identical
}
