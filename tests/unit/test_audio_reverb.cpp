/**************************************************************************/
/* test_audio_reverb.cpp                                                  */
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

// Unit_Audio: the phase-S5 reverb — the lossless feedback matrices (Householder /
// Hadamard energy preservation + involution), the Jot FDN (bounded/stable, coprime
// line lengths, a measured RT60 that tracks the requested decay, and a darker tail when
// the HF decays faster), the predelay, and the room-geometry RT60 (Sabine/Eyring, the
// crossover, and the shoebox factory). Pure header-only maths against the real code.

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;

namespace
{
    double norm(const float* v, int n)
    {
        double s = 0.0;
        for (int i = 0; i < n; ++i)
            s += static_cast<double>(v[i]) * v[i];
        return std::sqrt(s);
    }

    // Render an impulse through a reverb and estimate RT60 from the T30 decay slope:
    // a linear fit of the energy envelope (dB) between −5 and −35 dB, extrapolated to
    // −60. The standard robust estimator — insensitive to the onset and the noise floor.
    double measure_rt60(IReverb& reverb, double sample_rate, int block)
    {
        const int total = static_cast<int>(sample_rate * 6.0); // 6 s of tail
        std::vector<float> out;
        out.reserve(static_cast<std::size_t>(total));

        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        bool first = true;
        for (int rendered = 0; rendered < total; rendered += block)
        {
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            if (first)
            {
                l[0] = 1.0f;
                r[0] = 1.0f;
                first = false;
            }
            reverb.process(l.data(), r.data(), block);
            for (int i = 0; i < block; ++i)
                out.push_back(0.5f * (l[i] + r[i]));
        }

        // Energy envelope in dB over non-overlapping windows.
        const int win = 1024;
        std::vector<double> t_s, db;
        double peak_db = -1e9;
        for (std::size_t start = 0; start + win <= out.size(); start += win)
        {
            double e = 0.0;
            for (int i = 0; i < win; ++i)
                e += static_cast<double>(out[start + i]) * out[start + i];
            e = std::sqrt(e / win);
            const double d = 20.0 * std::log10(e + 1e-12);
            t_s.push_back((start + win * 0.5) / sample_rate);
            db.push_back(d);
            if (d > peak_db)
                peak_db = d;
        }

        // Least-squares fit over the [-5, -35] dB window, after the peak.
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        int count = 0;
        bool past_peak = false;
        for (std::size_t i = 0; i < db.size(); ++i)
        {
            if (db[i] >= peak_db - 0.5)
                past_peak = true;
            if (!past_peak)
                continue;
            if (db[i] <= peak_db - 5.0 && db[i] >= peak_db - 35.0)
            {
                sx += t_s[i];
                sy += db[i];
                sxx += t_s[i] * t_s[i];
                sxy += t_s[i] * db[i];
                ++count;
            }
        }
        if (count < 3)
            return 0.0;
        const double slope = (count * sxy - sx * sy) / (count * sxx - sx * sx); // dB/s
        if (slope >= 0.0)
            return 0.0;
        return 60.0 / (-slope);
    }

    // High-frequency fraction of a late window of the reverb tail: a crude first-
    // difference high-pass energy over total energy. A darker tail scores lower.
    double late_hf_fraction(IReverb& reverb, double sample_rate, int block)
    {
        const int total = static_cast<int>(sample_rate * 2.0);
        std::vector<float> out;
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        bool first = true;
        for (int rendered = 0; rendered < total; rendered += block)
        {
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            if (first) { l[0] = r[0] = 1.0f; first = false; }
            reverb.process(l.data(), r.data(), block);
            for (int i = 0; i < block; ++i)
                out.push_back(0.5f * (l[i] + r[i]));
        }
        // Late window: the second half, well past the onset.
        const std::size_t begin = out.size() / 2;
        double hf = 0.0, tot = 0.0;
        for (std::size_t i = begin + 1; i < out.size(); ++i)
        {
            const double d = static_cast<double>(out[i]) - out[i - 1];
            hf += d * d;
            tot += static_cast<double>(out[i]) * out[i];
        }
        return tot > 1e-20 ? hf / tot : 0.0;
    }
} // namespace

// The lossless feedback matrices.

TEST(Unit_Audio, HouseholderMatrixIsLossless)
{
    float v[16];
    for (int i = 0; i < 16; ++i)
        v[i] = std::sin(0.3f * i) + 0.5f * std::cos(1.1f * i);
    const double before = norm(v, 16);
    DSP::apply_householder(v, 16);
    EXPECT_NEAR(norm(v, 16), before, 1e-4);
}

TEST(Unit_Audio, HouseholderMatrixIsAnInvolution)
{
    float v[8] = {1.0f, -2.0f, 3.0f, 0.5f, -1.5f, 2.5f, -0.25f, 4.0f};
    float original[8];
    for (int i = 0; i < 8; ++i)
        original[i] = v[i];
    DSP::apply_householder(v, 8);
    DSP::apply_householder(v, 8); // H is symmetric orthogonal → H² = I
    for (int i = 0; i < 8; ++i)
        EXPECT_NEAR(v[i], original[i], 1e-4f);
}

TEST(Unit_Audio, HadamardMatrixIsLossless)
{
    float v[16];
    for (int i = 0; i < 16; ++i)
        v[i] = static_cast<float>((i * 7 % 11) - 5);
    const double before = norm(v, 16);
    DSP::apply_hadamard(v, 16);
    EXPECT_NEAR(norm(v, 16), before, 1e-4);
}

// The FDN reverb.

TEST(Unit_Audio, FDNDelayLinesHaveDistinctCoprimeLengths)
{
    DSP::FDNReverb fdn;
    fdn.prepare(48000.0, 256);
    for (int i = 0; i < DSP::FDNReverb::kLines; ++i)
        for (int j = i + 1; j < DSP::FDNReverb::kLines; ++j)
        {
            // Distinct primes ⇒ pairwise coprime (gcd == 1).
            int a = fdn.line_length(i), b = fdn.line_length(j);
            EXPECT_NE(a, b);
            while (b != 0) { int t = a % b; a = b; b = t; }
            EXPECT_EQ(a, 1);
        }
}

TEST(Unit_Audio, FDNTailIsBoundedAndDecays)
{
    DSP::FDNReverb fdn;
    fdn.prepare(48000.0, 256);
    DSP::FDNTuning t;
    t.decay_time_s = 1.0;
    fdn.set_tuning(t);

    const int block = 256;
    std::vector<float> il(block, 0.0f), ir(block, 0.0f), ol(block), orr(block);
    il[0] = ir[0] = 1.0f;

    double early_energy = 0.0, late_energy = 0.0, peak = 0.0;
    for (int b = 0; b < 400; ++b) // ~2.1 s
    {
        fdn.process(il.data(), ir.data(), ol.data(), orr.data(), block);
        std::fill(il.begin(), il.end(), 0.0f);
        std::fill(ir.begin(), ir.end(), 0.0f);
        for (int i = 0; i < block; ++i)
        {
            const double s = std::fabs(static_cast<double>(ol[i]));
            peak = std::max(peak, s);
            if (b < 40) early_energy += s * s;
            if (b >= 300) late_energy += s * s;
        }
    }
    EXPECT_LT(peak, 4.0);                  // bounded — never blows up
    EXPECT_GT(early_energy, 0.0);          // it actually rang
    EXPECT_LT(late_energy, early_energy);  // and it decayed
}

TEST(Unit_Audio, FDNMeasuredRt60TracksRequestedDecay)
{
    for (double target : {0.8, 2.0})
    {
        FDNReverbEffect reverb;
        reverb.prepare(48000.0, 256);
        I3DL2Reverb p = I3DL2Reverb::generic();
        p.decay_time = static_cast<float>(target);
        p.decay_hf_ratio = 1.0f;   // uniform decay → a clean single-slope measurement
        p.reverb_delay = 0.005f;
        p.room = 0.0f;
        p.reverb = 0.0f;
        p.wet_dry_mix = 100.0f;    // pure wet
        reverb.set_parameters(p);

        const double measured = measure_rt60(reverb, 48000.0, 256);
        // The FDN's decay is set by the loop-gain design; a measured T30 within ±40 %
        // of the target is the expected accuracy for a 16-line network.
        EXPECT_GT(measured, target * 0.6) << "target=" << target;
        EXPECT_LT(measured, target * 1.4) << "target=" << target;
    }
}

TEST(Unit_Audio, FDNDarkerHfRatioProducesDarkerTail)
{
    FDNReverbEffect bright, dark;
    bright.prepare(48000.0, 256);
    dark.prepare(48000.0, 256);

    I3DL2Reverb pb = I3DL2Reverb::generic();
    pb.decay_time = 2.0f; pb.decay_hf_ratio = 1.0f; pb.reverb_delay = 0.005f;
    pb.room = 0.0f; pb.reverb = 0.0f; pb.room_hf = 0.0f; pb.wet_dry_mix = 100.0f;
    I3DL2Reverb pd = pb;
    pd.decay_hf_ratio = 0.2f; // highs decay much faster

    bright.set_parameters(pb);
    dark.set_parameters(pd);

    const double hf_bright = late_hf_fraction(bright, 48000.0, 256);
    const double hf_dark = late_hf_fraction(dark, 48000.0, 256);
    EXPECT_LT(hf_dark, hf_bright); // the fast-HF-decay tail is measurably darker late
}

TEST(Unit_Audio, FDNPredelayDelaysTheOnset)
{
    FDNReverbEffect reverb;
    reverb.prepare(48000.0, 512);
    I3DL2Reverb p = I3DL2Reverb::generic();
    p.reverb_delay = 0.05f; // 50 ms → 2400 samples at 48 kHz
    p.wet_dry_mix = 100.0f; // fully wet: nothing but the (delayed) tail
    p.room = 0.0f; p.reverb = 0.0f;
    reverb.set_parameters(p);

    std::vector<float> l(512, 0.0f), r(512, 0.0f);
    l[0] = r[0] = 1.0f;
    reverb.process(l.data(), r.data(), 512); // first 512 samples < 2400-sample predelay

    double energy = 0.0;
    for (int i = 0; i < 512; ++i)
        energy += std::fabs(static_cast<double>(l[i]));
    EXPECT_LT(energy, 1e-4); // silent until the predelay elapses
}

// Room-geometry RT60.

TEST(Unit_Audio, SabineRt60MatchesTextbookValue)
{
    // V = 1000 m³, S = 600 m², ᾱ = 0.1 → T = 0.161·1000 / (600·0.1) = 2.6833 s.
    EXPECT_NEAR(sabine_rt60(1000.0, 600.0, 0.1), 2.68333, 1e-4);
}

TEST(Unit_Audio, EyringIsShorterThanSabineForAbsorptiveRooms)
{
    const double v = 500.0, s = 400.0, a = 0.5;
    const double sab = sabine_rt60(v, s, a);
    const double eyr = eyring_rt60(v, s, a);
    EXPECT_LT(eyr, sab); // Eyring's −ln(1−ᾱ) > ᾱ for ᾱ > 0, so a shorter RT60
    // reverb_rt60 uses Eyring past the 0.3 crossover.
    EXPECT_NEAR(reverb_rt60(v, s, a), eyr, 1e-9);
    // …and Sabine below it.
    EXPECT_NEAR(reverb_rt60(v, s, 0.1), sabine_rt60(v, s, 0.1), 1e-9);
}

TEST(Unit_Audio, ShoeboxFactoryProducesSaneParameters)
{
    // A large, fairly live hall: low absorption body, more absorptive highs.
    const I3DL2Reverb r = shoebox_reverb(20.0, 30.0, 12.0, 0.12, 0.28);
    EXPECT_GT(r.decay_time, 1.0f);       // a big live room rings
    EXPECT_LT(r.decay_hf_ratio, 1.0f);   // more HF absorption → darker
    EXPECT_GT(r.reverb_delay, 0.0f);
    EXPECT_LT(r.reverb_delay, 0.24f);

    FDNReverbEffect reverb;
    reverb.prepare(48000.0, 256);
    reverb.set_parameters(r);
    const double measured = measure_rt60(reverb, 48000.0, 256);
    EXPECT_GT(measured, r.decay_time * 0.5); // the geometry decay is actually realised
    EXPECT_LT(measured, r.decay_time * 1.6);
}

TEST(Unit_Audio, ReverbBusEffectDrivesReverbThroughTheMixerSeam)
{
    // The full seam: FDNReverbEffect (IReverb) behind ReverbBusEffect (IBusEffect) on a
    // per-zone aux bus. A dry sfx bus sends into it; the master should carry the wet.
    MixerGraph mixer;
    const int master = mixer.add_bus(NO_BUS);
    const int sfx = mixer.add_bus(master);
    const int reverb_bus = mixer.add_bus(master);
    mixer.set_master(master);
    mixer.add_aux_send(sfx, reverb_bus, 0.5f);

    std::unique_ptr<FDNReverbEffect> fx(new FDNReverbEffect());
    fx->set_parameters(I3DL2Reverb::concert_hall());
    mixer.add_insert(reverb_bus, std::unique_ptr<IBusEffect>(new ReverbBusEffect(std::move(fx))));
    mixer.prepare(48000.0, 256);

    std::vector<float> mono(256, 0.0f);
    mono[0] = 1.0f; // an impulse into the dry bus

    double reverb_tail = 0.0, peak = 0.0;
    for (int b = 0; b < 40; ++b)
    {
        mixer.begin_block(256);
        mixer.accumulate(sfx, mono.data(), 256, 1.0f, 1.0f);
        std::fill(mono.begin(), mono.end(), 0.0f); // impulse only in the first block
        mixer.process(256);
        const float* ml = mixer.master_left();
        for (int i = 0; i < 256; ++i)
        {
            peak = std::max(peak, std::fabs(static_cast<double>(ml[i])));
            if (b >= 10) // well after the dry impulse: only the reverb return remains
                reverb_tail += std::fabs(static_cast<double>(ml[i]));
        }
    }
    EXPECT_GT(reverb_tail, 0.0);   // the aux reverb rings on after the dry sound
    EXPECT_LT(peak, 8.0);          // and stays bounded
}
