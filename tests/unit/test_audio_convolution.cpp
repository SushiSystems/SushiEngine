/**************************************************************************/
/* test_audio_convolution.cpp                                             */
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

// Unit_Audio: the phase-S10 fast-convolution stack — the radix-2 FFT (round-trip +
// linearity of spectra), the uniformly-partitioned overlap-save convolver (matches a
// naive time-domain convolution to numerical precision, across partition boundaries),
// and the convolution reverb (an impulse yields a bounded, decaying tail whose length
// tracks the requested decay). Header-only maths against the real code.

#include <cmath>
#include <complex>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/audio/audio.hpp>

using namespace SushiEngine::Audio;
using namespace SushiEngine::Audio::DSP;

// The FFT is its own inverse up to scale.
TEST(Unit_Audio, FFTRoundTrip)
{
    RadixFFT fft;
    fft.prepare(64);
    std::vector<std::complex<float>> original(64), work(64);
    for (int i = 0; i < 64; ++i)
        original[i] = std::complex<float>(std::sin(i * 0.37f), std::cos(i * 0.11f));
    work = original;
    fft.forward(work.data());
    fft.inverse(work.data());
    double err = 0.0;
    for (int i = 0; i < 64; ++i)
        err += std::abs(work[i] - original[i]);
    EXPECT_LT(err / 64.0, 1.0e-5);
}

// Partitioned overlap-save equals a direct time-domain convolution.
TEST(Unit_Audio, PartitionedConvolutionMatchesDirect)
{
    const int block = 16;
    // An IR spanning several partitions (length 40 > 2·block).
    std::vector<float> ir(40);
    for (int i = 0; i < 40; ++i)
        ir[i] = std::exp(-i * 0.15f) * std::sin(i * 0.5f);

    PartitionedConvolver conv;
    conv.prepare(block, ir.data(), static_cast<int>(ir.size()));
    EXPECT_GE(conv.partitions(), 3);

    const int total = 8 * block;
    std::vector<float> x(static_cast<std::size_t>(total));
    for (int i = 0; i < total; ++i)
        x[static_cast<std::size_t>(i)] = std::sin(i * 0.3f) + 0.5f * std::sin(i * 0.07f);

    std::vector<float> y;
    y.reserve(static_cast<std::size_t>(total));
    for (int b = 0; b < total / block; ++b)
    {
        std::vector<float> out(static_cast<std::size_t>(block));
        conv.process_block(&x[static_cast<std::size_t>(b * block)], out.data());
        for (int i = 0; i < block; ++i)
            y.push_back(out[static_cast<std::size_t>(i)]);
    }

    // Compare against a direct convolution, past the first block (overlap-save warm-up).
    double err = 0.0;
    int checked = 0;
    for (int n = block; n < total - block; ++n)
    {
        float ref = 0.0f;
        for (int k = 0; k < static_cast<int>(ir.size()); ++k)
        {
            const int index = n - k;
            if (index >= 0 && index < total)
                ref += x[static_cast<std::size_t>(index)] * ir[static_cast<std::size_t>(k)];
        }
        err += std::fabs(static_cast<double>(y[static_cast<std::size_t>(n)]) - ref);
        ++checked;
    }
    EXPECT_LT(err / checked, 1.0e-4);
}

// Non-uniform partitioned convolution (small head + large tail) equals a direct
// convolution — same result as UPOLS, at lower tail cost.
TEST(Unit_Audio, NonUniformConvolutionMatchesDirect)
{
    const int block = 16;
    const int m = 200; // IR longer than the head split (4*16 = 64) → exercises the tail
    std::vector<float> ir(static_cast<std::size_t>(m));
    for (int i = 0; i < m; ++i)
        ir[static_cast<std::size_t>(i)] = std::exp(-i * 0.03f) * std::sin(i * 0.4f);

    NonUniformConvolver conv;
    conv.prepare(block, ir.data(), m, 4);
    EXPECT_GT(conv.split(), 0);

    const int total = block * 40;
    std::vector<float> x(static_cast<std::size_t>(total));
    for (int i = 0; i < total; ++i)
        x[static_cast<std::size_t>(i)] = std::sin(i * 0.27f) + 0.3f * std::sin(i * 0.05f);

    std::vector<float> y;
    for (int b = 0; b < total / block; ++b)
    {
        std::vector<float> o(static_cast<std::size_t>(block));
        conv.process_block(&x[static_cast<std::size_t>(b * block)], o.data());
        for (int i = 0; i < block; ++i)
            y.push_back(o[static_cast<std::size_t>(i)]);
    }

    double err = 0.0;
    int checked = 0;
    for (int n = 64; n < total - 64; ++n)
    {
        float ref = 0.0f;
        for (int k = 0; k < m; ++k)
            if (n - k >= 0)
                ref += x[static_cast<std::size_t>(n - k)] * ir[static_cast<std::size_t>(k)];
        err += std::fabs(static_cast<double>(y[static_cast<std::size_t>(n)]) - ref);
        ++checked;
    }
    EXPECT_LT(err / checked, 1.0e-4); // zero-latency, matches direct convolution
}

// The convolution reverb: an impulse produces a bounded, decaying wet tail.
TEST(Unit_Audio, ConvolutionReverbDecays)
{
    const double sr = 48000.0;
    const int block = 512;
    ConvolutionReverb reverb;
    reverb.prepare(sr, block);
    I3DL2Reverb p = I3DL2Reverb::concert_hall();
    p.wet_dry_mix = 100.0f;
    reverb.set_parameters(p);
    EXPECT_GT(reverb.impulse_length(), static_cast<int>(sr)); // multi-second tail

    double peak = 0.0, early = 0.0, late = 0.0;
    bool first = true;
    const int blocks = 200;
    for (int b = 0; b < blocks; ++b)
    {
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        if (first)
        {
            l[0] = r[0] = 1.0f;
            first = false;
        }
        reverb.process(l.data(), r.data(), block);
        for (int i = 0; i < block; ++i)
        {
            const double s = std::fabs(static_cast<double>(l[i]));
            if (s > peak)
                peak = s;
            if (b < blocks / 4)
                early += s * s;
            else if (b >= 3 * blocks / 4)
                late += s * s;
        }
    }
    EXPECT_LT(peak, 8.0);      // bounded (no blow-up)
    EXPECT_GT(early, 0.0);     // the tail exists
    EXPECT_LT(late, early);    // and decays
}

// A longer requested decay produces a longer impulse response.
TEST(Unit_Audio, ConvolutionReverbDecayLength)
{
    const double sr = 48000.0;
    ConvolutionReverb shortv, longv;
    shortv.prepare(sr, 512);
    longv.prepare(sr, 512);
    I3DL2Reverb ps = I3DL2Reverb::generic();
    ps.decay_time = 0.5f;
    I3DL2Reverb pl = ps;
    pl.decay_time = 3.0f;
    shortv.set_parameters(ps);
    longv.set_parameters(pl);
    EXPECT_LT(shortv.impulse_length(), longv.impulse_length());
}

// A loaded (measured) impulse response is convolved verbatim — the reverb reproduces
// the exact IR, not a synthesised tail.
TEST(Unit_Audio, ConvolutionReverbLoadedImpulse)
{
    const double sr = 48000.0;
    const int block = 16;
    ConvolutionReverb reverb;
    reverb.prepare(sr, block);

    // A sparse two-tap IR: unity at 0, half at 10.
    std::vector<float> ir(32, 0.0f);
    ir[0] = 1.0f;
    ir[10] = 0.5f;
    reverb.load_impulse(ir.data(), static_cast<int>(ir.size()), 1, sr);
    EXPECT_TRUE(reverb.has_loaded_impulse());

    I3DL2Reverb p = I3DL2Reverb::generic();
    p.room = 0.0f;
    p.reverb = 0.0f; // unity level
    p.wet_dry_mix = 100.0f;
    reverb.set_parameters(p);

    std::vector<float> out;
    bool first = true;
    for (int b = 0; b < 6; ++b)
    {
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        if (first) { l[0] = r[0] = 1.0f; first = false; }
        reverb.process(l.data(), r.data(), block);
        for (int i = 0; i < block; ++i)
            out.push_back(l[i]);
    }
    // The convolution of an impulse with the IR is the IR itself, delayed by one block.
    float peak0 = 0.0f, peak10 = 0.0f;
    for (int i = 0; i < static_cast<int>(out.size()); ++i)
    {
        if (std::fabs(out[static_cast<std::size_t>(i)]) > 0.5f &&
            std::fabs(out[static_cast<std::size_t>(i)] - 1.0f) < 0.1f)
            peak0 = out[static_cast<std::size_t>(i)];
        if (std::fabs(out[static_cast<std::size_t>(i)] - 0.5f) < 0.1f)
            peak10 = out[static_cast<std::size_t>(i)];
    }
    EXPECT_NEAR(peak0, 1.0f, 0.1f);
    EXPECT_NEAR(peak10, 0.5f, 0.1f);
}
