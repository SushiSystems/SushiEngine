/**************************************************************************/
/* test_audio_dynamics.cpp                                              */
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

// Unit_Audio: the bus dynamics rack — the compressor reduces gain above its threshold and
// leaves quiet signal alone; the sidechain key ducks the bus; the lookahead limiter holds
// the output under its ceiling even on a transient; the gate silences sub-threshold signal.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    void fill_sine(std::vector<float>& l, std::vector<float>& r, double freq, double sr,
                   float amp, double& phase)
    {
        const double inc = 2.0 * 3.14159265358979 * freq / sr;
        for (std::size_t i = 0; i < l.size(); ++i)
        {
            const float s = amp * static_cast<float>(std::sin(phase));
            l[i] = s;
            r[i] = s;
            phase += inc;
        }
    }

    float peak(const std::vector<float>& v)
    {
        float p = 0.0f;
        for (float s : v)
            p = std::fmax(p, std::fabs(s));
        return p;
    }
}

// The compressor pulls a loud signal down and leaves a quiet one essentially untouched.
TEST(Unit_Audio, CompressorReducesLoudSignal)
{
    const double sr = 48000.0;
    const int block = 512;
    CompressorParams p;
    p.threshold_db = -18.0f;
    p.ratio = 4.0f;
    p.attack_seconds = 0.002f;
    p.release_seconds = 0.05f;

    auto settled_peak = [&](float amp) {
        CompressorBusEffect comp(p);
        comp.prepare(sr, block);
        std::vector<float> l(block), r(block);
        double phase = 0.0;
        float out = 0.0f;
        for (int b = 0; b < 40; ++b)
        {
            fill_sine(l, r, 220.0, sr, amp, phase);
            comp.process(l.data(), r.data(), block);
            out = peak(l);
        }
        return out;
    };

    const float quiet_in = 0.1f;   // ~ -20 dB, below threshold
    const float loud_in = 0.7f;    // ~ -3 dB, well above threshold
    const float quiet_out = settled_peak(quiet_in);
    const float loud_out = settled_peak(loud_in);

    EXPECT_NEAR(quiet_out, quiet_in, 0.02f);      // quiet passes ~unchanged
    EXPECT_LT(loud_out, loud_in * 0.85f);         // loud is compressed down
}

// The sidechain key ducks the bus even though the bus's own signal is constant.
TEST(Unit_Audio, CompressorSidechainDucks)
{
    const double sr = 48000.0;
    const int block = 512;
    CompressorParams p;
    p.threshold_db = -24.0f;
    p.ratio = 8.0f;
    p.attack_seconds = 0.002f;
    p.release_seconds = 0.05f;

    CompressorBusEffect comp(p);
    comp.prepare(sr, block);
    float key = 0.0f;
    comp.set_key(&key);

    std::vector<float> l(block), r(block);
    double phase = 0.0;

    // Key silent → no ducking.
    float open_peak = 0.0f;
    for (int b = 0; b < 20; ++b)
    {
        fill_sine(l, r, 300.0, sr, 0.3f, phase);
        comp.process(l.data(), r.data(), block);
        open_peak = peak(l);
    }

    // Key loud → the bus ducks.
    key = 0.9f;
    float ducked_peak = 0.0f;
    for (int b = 0; b < 20; ++b)
    {
        fill_sine(l, r, 300.0, sr, 0.3f, phase);
        comp.process(l.data(), r.data(), block);
        ducked_peak = peak(l);
    }
    EXPECT_LT(ducked_peak, open_peak * 0.7f);
}

// The limiter holds the output under its ceiling on a sustained over-level signal.
TEST(Unit_Audio, LimiterHoldsCeiling)
{
    const double sr = 48000.0;
    const int block = 512;
    const float ceiling_db = -1.0f;
    const float ceiling_lin = std::pow(10.0f, ceiling_db / 20.0f);
    LimiterBusEffect limiter(ceiling_db, 0.003f, 0.05f);
    limiter.prepare(sr, block);

    std::vector<float> l(block), r(block);
    double phase = 0.0;
    float out_peak = 0.0f;
    for (int b = 0; b < 60; ++b)
    {
        fill_sine(l, r, 440.0, sr, 1.5f, phase); // 3.5 dB over full scale
        limiter.process(l.data(), r.data(), block);
        if (b >= 5) // let the lookahead prime
            out_peak = std::fmax(out_peak, peak(l));
    }
    EXPECT_LE(out_peak, ceiling_lin * 1.02f); // never (meaningfully) exceeds the ceiling
}

// The gate silences signal below its threshold and opens for signal above it.
TEST(Unit_Audio, GateSilencesQuiet)
{
    const double sr = 48000.0;
    const int block = 512;
    GateParams gp;
    gp.threshold_db = -40.0f;
    GateBusEffect gate(gp);
    gate.prepare(sr, block);

    std::vector<float> l(block), r(block);
    double phase = 0.0;

    float quiet_out = 0.0f;
    for (int b = 0; b < 40; ++b)
    {
        fill_sine(l, r, 200.0, sr, 0.001f, phase); // ~ -60 dB, below the gate
        gate.process(l.data(), r.data(), block);
        quiet_out = peak(l);
    }
    EXPECT_LT(quiet_out, 0.0005f);

    float loud_out = 0.0f;
    for (int b = 0; b < 40; ++b)
    {
        fill_sine(l, r, 200.0, sr, 0.3f, phase); // above the gate
        gate.process(l.data(), r.data(), block);
        loud_out = peak(l);
    }
    EXPECT_GT(loud_out, 0.2f);
}
