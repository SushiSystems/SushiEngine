/**************************************************************************/
/* audio_convolution_demo.cpp                                             */
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

/**
 * @file audio_convolution_demo.cpp
 * @brief Phase S10 vertical slice: the convolution reverb behind the IReverb seam.
 *
 *   1. Headless self-checks: the partitioned convolver matches a direct convolution; the
 *      convolution reverb produces a bounded, decaying tail; a longer decay rings longer.
 *      No hardware needed (CI check).
 *   2. Best-effort playback: a percussive pattern through the convolution reverb on a
 *      per-zone aux bus — the same seam and wiring the FDN uses, now driven by a
 *      synthesised room impulse response.
 *
 * Exits 0 on success.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/audio.hpp>
#include <sdl/sdl_audio_device.hpp>

using namespace SushiEngine::Audio;

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    // 1. Headless self-checks.
    {
        const int b = 16;
        std::vector<float> ir(40);
        for (int i = 0; i < 40; ++i)
            ir[i] = std::exp(-i * 0.12f) * std::sin(i * 0.6f);
        DSP::PartitionedConvolver conv;
        conv.prepare(b, ir.data(), static_cast<int>(ir.size()));
        std::vector<float> x(8 * b);
        for (int i = 0; i < 8 * b; ++i)
            x[static_cast<std::size_t>(i)] = std::sin(i * 0.25f);
        std::vector<float> y;
        for (int blk = 0; blk < 8; ++blk)
        {
            std::vector<float> out(b);
            conv.process_block(&x[static_cast<std::size_t>(blk * b)], out.data());
            for (int i = 0; i < b; ++i)
                y.push_back(out[static_cast<std::size_t>(i)]);
        }
        double err = 0.0;
        int checked = 0;
        for (int n = b; n < 7 * b; ++n)
        {
            float ref = 0.0f;
            for (int k = 0; k < static_cast<int>(ir.size()); ++k)
                if (n - k >= 0)
                    ref += x[static_cast<std::size_t>(n - k)] * ir[static_cast<std::size_t>(k)];
            err += std::fabs(static_cast<double>(y[static_cast<std::size_t>(n)]) - ref);
            ++checked;
        }
        std::printf("partitioned conv mean err=%.2e\n", err / checked);
        if (!(err / checked < 1.0e-3))
        {
            std::fprintf(stderr, "audio_convolution_demo FAILED: convolver != direct\n");
            return 1;
        }
    }

    auto tail = [&](float decay) -> double {
        ConvolutionReverb rev;
        rev.prepare(sample_rate, block);
        I3DL2Reverb p = I3DL2Reverb::concert_hall();
        p.decay_time = decay;
        p.wet_dry_mix = 100.0f;
        rev.set_parameters(p);
        double peak = 0.0, early = 0.0, late = 0.0;
        bool first = true;
        for (int b = 0; b < 200; ++b)
        {
            std::vector<float> l(block, 0.0f), r(block, 0.0f);
            if (first) { l[0] = r[0] = 1.0f; first = false; }
            rev.process(l.data(), r.data(), block);
            for (int i = 0; i < block; ++i)
            {
                const double s = std::fabs(static_cast<double>(l[i]));
                if (s > peak) peak = s;
                if (b < 50) early += s * s; else if (b >= 150) late += s * s;
            }
        }
        std::printf("conv reverb decay=%.1fs: peak=%.4f early=%.5f late=%.5f\n", decay, peak, early, late);
        if (!(peak < 8.0) || !(early > 0.0) || !(late < early))
        {
            std::fprintf(stderr, "audio_convolution_demo FAILED: tail not bounded/decaying\n");
            return -1.0;
        }
        return early;
    };
    if (tail(1.0f) < 0.0 || tail(3.5f) < 0.0)
        return 1;
    // Longer requested decay → a longer impulse response (a robust, level-independent
    // measure; energy per sample falls as the tail lengthens, so the IR length is what
    // captures "rings longer").
    {
        ConvolutionReverb s, l;
        s.prepare(sample_rate, block);
        l.prepare(sample_rate, block);
        I3DL2Reverb ps = I3DL2Reverb::generic();
        ps.decay_time = 0.5f;
        I3DL2Reverb pl = ps;
        pl.decay_time = 3.0f;
        s.set_parameters(ps);
        l.set_parameters(pl);
        std::printf("conv reverb IR length: 0.5s=%d  3.0s=%d\n", s.impulse_length(),
                    l.impulse_length());
        if (!(l.impulse_length() > s.impulse_length()))
        {
            std::fprintf(stderr, "audio_convolution_demo FAILED: longer decay did not ring longer\n");
            return 1;
        }
    }
    std::printf("headless convolution checks passed\n");

    // 2. Best-effort playback through the convolution reverb aux bus.
    AudioEngine engine(16, 8);
    const int master = engine.mixer().add_bus(NO_BUS);
    const int sfx_bus = engine.mixer().add_bus(master);
    const int reverb_bus = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);
    engine.mixer().add_aux_send(sfx_bus, reverb_bus, 0.7f);
    {
        std::unique_ptr<ConvolutionReverb> fx(new ConvolutionReverb());
        I3DL2Reverb hall = I3DL2Reverb::concert_hall();
        hall.wet_dry_mix = 100.0f;
        fx->set_parameters(hall);
        engine.mixer().add_insert(
            reverb_bus, std::unique_ptr<IBusEffect>(new ReverbBusEffect(std::move(fx))));
    }
    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    static std::vector<float> blip;
    {
        const int length = static_cast<int>(sample_rate * 0.7);
        blip.assign(static_cast<std::size_t>(length), 0.0f);
        const int hit = static_cast<int>(sample_rate * 0.1);
        for (int i = 0; i < hit; ++i)
        {
            const double env = std::exp(-static_cast<double>(i) / (sample_rate * 0.02));
            blip[static_cast<std::size_t>(i)] =
                static_cast<float>(0.5 * env * std::sin(2.0 * 3.14159265 * 523.25 * i / sample_rate));
        }
    }
    {
        VoiceDescriptor d;
        d.base_gain = 0.6f;
        d.priority = 10.0f;
        d.bus = sfx_bus;
        engine.voices().play(d, std::unique_ptr<VoiceSource>(
                                    new BufferSource(blip.data(), static_cast<int>(blip.size()), true)));
    }

    std::vector<float> left(block, 0.0f), right(block, 0.0f);
    float* channels[2] = {left.data(), right.data()};

    SDLAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block;
    if (device.open(desired, engine))
    {
        std::printf("audio device open — a pulse through the CONVOLUTION reverb aux bus\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
        device.close();
    }
    else
    {
        double peak = 0.0;
        for (int b = 0; b < 200; ++b)
        {
            engine.render(channels, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
        }
        std::printf("no audio device (headless) — rendered mix, peak=%.4f\n", peak);
        if (peak > 4.0)
        {
            std::fprintf(stderr, "audio_convolution_demo FAILED: mix unbounded (peak %.4f)\n", peak);
            return 1;
        }
    }

    std::printf("audio_convolution_demo OK\n");
    return 0;
}
