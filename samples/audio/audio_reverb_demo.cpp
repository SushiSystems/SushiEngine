/**************************************************************************/
/* audio_reverb_demo.cpp                                                  */
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
 * @file audio_reverb_demo.cpp
 * @brief Phase S5 vertical slice: the Jot FDN reverb on a per-zone aux bus.
 *
 * The S2 mixer left a placeholder low-pass on its reverb send; this replaces it with
 * the real thing — an order-16 feedback delay network driven by the I3DL2 parameter
 * set, with room-geometry (Sabine/Eyring) RT60. It:
 *
 *   1. Runs headless and self-checks the FDN directly: an impulse produces a bounded,
 *      decaying tail (no blow-up, no infinite ring); a 50 ms predelay leaves the wet
 *      output silent until the tail arrives; a longer requested decay measurably rings
 *      longer than a short one; and the shoebox room factory yields sane I3DL2 parameters.
 *      No hardware needed — this is the CI check.
 *   2. Best-effort plays a percussive pattern through the reverb aux bus so the tail is
 *      audible (headphones or speakers).
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

namespace
{
    // Total absolute output of a reverb's wet response to a single impulse, split into an
    // early and a late window — used to show the tail both exists and decays.
    struct DecayProfile
    {
        double early = 0.0;
        double late = 0.0;
        double peak = 0.0;
        double first_block_energy = 0.0;
    };

    DecayProfile impulse_response(IReverb& reverb, double sample_rate, int block)
    {
        DecayProfile p;
        const int blocks = static_cast<int>(sample_rate * 3.0 / block);
        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        for (int b = 0; b < blocks; ++b)
        {
            std::fill(l.begin(), l.end(), 0.0f);
            std::fill(r.begin(), r.end(), 0.0f);
            if (b == 0)
                l[0] = r[0] = 1.0f;
            reverb.process(l.data(), r.data(), block);
            for (int i = 0; i < block; ++i)
            {
                const double s = std::fabs(static_cast<double>(l[i]));
                p.peak = std::max(p.peak, s);
                if (b == 0)
                    p.first_block_energy += s;
                if (b < blocks / 4)
                    p.early += s * s;
                else if (b >= 3 * blocks / 4)
                    p.late += s * s;
            }
        }
        return p;
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    // 1. Headless self-checks on the FDN reverb itself.

    // A bounded, decaying tail from a long-decay preset.
    {
        FDNReverbEffect reverb;
        reverb.prepare(sample_rate, block);
        I3DL2Reverb p = I3DL2Reverb::concert_hall();
        p.wet_dry_mix = 100.0f;
        p.reverb_delay = 0.01f;
        reverb.set_parameters(p);

        const DecayProfile prof = impulse_response(reverb, sample_rate, block);
        std::printf("concert_hall: peak=%.4f early=%.4f late=%.4f\n",
                    prof.peak, prof.early, prof.late);
        if (prof.peak > 8.0)
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: tail unbounded (peak %.4f)\n", prof.peak);
            return 1;
        }
        if (!(prof.early > 0.0) || !(prof.late < prof.early))
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: tail did not ring-and-decay\n");
            return 1;
        }
    }

    // Predelay: a 50 ms gap leaves the first block (10.7 ms) silent.
    {
        FDNReverbEffect reverb;
        reverb.prepare(sample_rate, block);
        I3DL2Reverb p = I3DL2Reverb::generic();
        p.reverb_delay = 0.05f;
        p.wet_dry_mix = 100.0f;
        reverb.set_parameters(p);

        std::vector<float> l(block, 0.0f), r(block, 0.0f);
        l[0] = r[0] = 1.0f;
        reverb.process(l.data(), r.data(), block);
        double e = 0.0;
        for (int i = 0; i < block; ++i)
            e += std::fabs(static_cast<double>(l[i]));
        std::printf("predelay(50ms): first-block energy=%.6f (want ~0)\n", e);
        if (e > 1e-4)
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: predelay did not delay the onset\n");
            return 1;
        }
    }

    // A longer requested decay rings longer than a short one.
    {
        FDNReverbEffect shortv, longv;
        shortv.prepare(sample_rate, block);
        longv.prepare(sample_rate, block);
        I3DL2Reverb ps = I3DL2Reverb::generic();
        ps.decay_time = 0.5f; ps.decay_hf_ratio = 1.0f; ps.wet_dry_mix = 100.0f;
        ps.reverb_delay = 0.005f; ps.room = 0.0f; ps.reverb = 0.0f;
        I3DL2Reverb pl = ps;
        pl.decay_time = 3.0f;
        shortv.set_parameters(ps);
        longv.set_parameters(pl);

        const double late_short = impulse_response(shortv, sample_rate, block).late;
        const double late_long = impulse_response(longv, sample_rate, block).late;
        std::printf("late energy: 0.5s=%.6f  3.0s=%.6f\n", late_short, late_long);
        if (!(late_long > late_short))
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: longer decay did not ring longer\n");
            return 1;
        }
    }

    // Room-geometry RT60 (Sabine/Eyring) → I3DL2.
    {
        const I3DL2Reverb hall = shoebox_reverb(20.0, 30.0, 12.0, 0.12, 0.28);
        std::printf("shoebox 20x30x12 (a_lo=0.12, a_hi=0.28): DecayTime=%.2fs HFRatio=%.2f predelay=%.1fms\n",
                    hall.decay_time, hall.decay_hf_ratio, hall.reverb_delay * 1000.0f);
        if (!(hall.decay_time > 1.0f) || !(hall.decay_hf_ratio < 1.0f))
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: shoebox RT60 out of expected range\n");
            return 1;
        }
    }

    std::printf("headless reverb checks passed\n");

    // 2. Best-effort audible playback through the mixer aux bus.

    AudioEngine engine(16, 8);
    const int master = engine.mixer().add_bus(NO_BUS);
    const int sfx_bus = engine.mixer().add_bus(master);
    const int reverb_bus = engine.mixer().add_bus(master);
    engine.mixer().set_master(master);

    // The real reverb, on a per-zone aux bus (S5 — replaces the S2 low-pass placeholder).
    engine.mixer().add_aux_send(sfx_bus, reverb_bus, 0.6f);
    {
        std::unique_ptr<FDNReverbEffect> fx(new FDNReverbEffect());
        I3DL2Reverb hall = I3DL2Reverb::concert_hall();
        hall.wet_dry_mix = 100.0f; // aux bus: pure wet, dry goes direct to master
        fx->set_parameters(hall);
        engine.mixer().add_insert(
            reverb_bus, std::unique_ptr<IBusEffect>(new ReverbBusEffect(std::move(fx))));
    }

    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    // A short percussive blip pattern (a decaying tone burst), looped, so the reverb
    // tail is clearly audible between hits.
    static std::vector<float> blip;
    {
        const int length = static_cast<int>(sample_rate * 0.6); // 600 ms cell
        blip.assign(static_cast<std::size_t>(length), 0.0f);
        const int hit = static_cast<int>(sample_rate * 0.12); // 120 ms hit
        for (int i = 0; i < hit; ++i)
        {
            const double env = std::exp(-static_cast<double>(i) / (sample_rate * 0.03));
            blip[i] = static_cast<float>(0.5 * env * std::sin(2.0 * 3.14159265 * 440.0 * i / sample_rate));
        }
    }
    {
        VoiceDescriptor d;
        d.base_gain = 0.6f;
        d.priority = 10.0f;
        d.bus = sfx_bus;
        d.pan = 0.0f;
        engine.voices().play(
            d, std::unique_ptr<VoiceSource>(new BufferSource(blip.data(), static_cast<int>(blip.size()), true)));
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
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch, %d frames/block — playing with reverb\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        device.close();
    }
    else
    {
        // Headless: render a few blocks so the path is still exercised end-to-end.
        double peak = 0.0;
        for (int b = 0; b < 200; ++b)
        {
            engine.render(channels, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
        }
        std::printf("no audio device available (headless) — rendered mix, peak=%.4f\n", peak);
        if (peak > 4.0)
        {
            std::fprintf(stderr, "audio_reverb_demo FAILED: engine mix unbounded (peak %.4f)\n", peak);
            return 1;
        }
    }

    std::printf("audio_reverb_demo OK\n");
    return 0;
}
