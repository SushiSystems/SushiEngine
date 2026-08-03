/**************************************************************************/
/* audio_procedural_demo.cpp                                            */
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

/**
 * @file audio_procedural_demo.cpp
 * @brief Phase S10 vertical slice: procedural SFX — modal impacts and wind.
 *
 *   1. Headless self-checks: each material strike rings then decays; a harder strike is
 *      louder; wind gets louder and brighter with speed. No hardware needed (CI check).
 *   2. Best-effort playback: knocks through wood / metal / glass / membrane on a timer,
 *      over a wind bed whose speed sweeps up and down — all synthesised, no samples.
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
    double energy_of(VoiceSource& source, int blocks, int block)
    {
        double e = 0.0;
        std::vector<float> buffer(static_cast<std::size_t>(block));
        for (int b = 0; b < blocks; ++b)
        {
            source.render(buffer.data(), block);
            for (float s : buffer)
                e += static_cast<double>(s) * s;
        }
        return e;
    }
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block = 512;

    // --- 1. Headless self-checks ------------------------------------------------------
    {
        DSP::ModalResonatorBank bank;
        bank.set_material(1, 400.0f);
        bank.prepare(sample_rate);
        bank.strike(1.0f);
        double early = 0.0, late = 0.0;
        std::vector<float> buffer(block, 0.0f);
        for (int b = 0; b < 40; ++b)
        {
            bank.process_block(buffer.data(), block);
            for (float s : buffer)
            {
                if (b < 4) early += static_cast<double>(s) * s;
                else if (b >= 36) late += static_cast<double>(s) * s;
            }
        }
        std::printf("modal metal: early=%.3f late=%.3f (rings & decays)\n", early, late);
        if (!(early > 0.0) || !(late < early))
        {
            std::fprintf(stderr, "audio_procedural_demo FAILED: modal did not ring-and-decay\n");
            return 1;
        }
    }
    {
        ModalImpactSource soft(1, 400.0f, 0.3f), hard(1, 400.0f, 1.0f);
        soft.prepare(sample_rate, block);
        hard.prepare(sample_rate, block);
        const double es = energy_of(soft, 40, block);
        const double eh = energy_of(hard, 40, block);
        std::printf("modal strike energy: soft=%.3f hard=%.3f\n", es, eh);
        if (!(eh > es))
        {
            std::fprintf(stderr, "audio_procedural_demo FAILED: harder strike not louder\n");
            return 1;
        }
    }
    {
        auto wind_rms = [&](float speed) {
            WindSource w(0xBEEFu);
            w.prepare(sample_rate, block);
            w.set_speed(speed);
            std::vector<float> buffer(block);
            double e = 0.0;
            for (int b = 0; b < 30; ++b)
            {
                w.render(buffer.data(), block);
                for (float s : buffer) e += static_cast<double>(s) * s;
            }
            return std::sqrt(e / (30 * block));
        };
        const double calm = wind_rms(0.1f), gale = wind_rms(1.0f);
        std::printf("wind rms: calm=%.4f gale=%.4f\n", calm, gale);
        if (!(gale > calm))
        {
            std::fprintf(stderr, "audio_procedural_demo FAILED: wind did not rise with speed\n");
            return 1;
        }
    }
    std::printf("headless procedural checks passed\n");

    // --- 2. Best-effort audible playback ----------------------------------------------
    AudioEngine engine(16, 8);
    const int master = engine.mixer().add_bus(NO_BUS);
    engine.mixer().set_master(master);
    engine.prepare(sample_rate, block);
    engine.voices().set_listener(ListenerState{AudioVec3{0.0f, 0.0f, 0.0f}});

    // A persistent wind bed we sweep.
    auto wind = std::unique_ptr<WindSource>(new WindSource(0x5EEDu, 0.04f));
    WindSource* wind_pointer = wind.get();
    {
        VoiceDescriptor d;
        d.base_gain = 0.5f;
        d.priority = 1.0f;
        d.bus = master;
        d.spatial = false;
        engine.voices().play(d, std::move(wind));
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
        std::printf("audio device open — knocks (wood/metal/glass/membrane) over swept wind\n");
        const int materials[4] = {0, 1, 2, 3};
        const float bases[4] = {180.0f, 320.0f, 900.0f, 140.0f};
        const double seconds = 8.0;
        const int total = static_cast<int>(seconds * 1000.0 / 250.0);
        for (int step = 0; step < total; ++step)
        {
            const float phase = static_cast<float>(step) / static_cast<float>(total);
            wind_pointer->set_speed(0.2f + 0.8f * (0.5f - 0.5f * std::cos(phase * 6.2831853f)));
            VoiceDescriptor d;
            d.base_gain = 0.6f;
            d.priority = 10.0f;
            d.bus = master;
            d.pan = (step % 2 == 0) ? -0.3f : 0.3f;
            const int m = step % 4;
            engine.voices().play(d, std::unique_ptr<VoiceSource>(
                                        new ModalImpactSource(materials[m], bases[m], 0.9f)));
            std::this_thread::sleep_for(std::chrono::milliseconds(250));
        }
        device.close();
    }
    else
    {
        double peak = 0.0;
        for (int b = 0; b < 200; ++b)
        {
            wind_pointer->set_speed(0.5f);
            if (b % 40 == 0)
            {
                VoiceDescriptor d;
                d.base_gain = 0.6f;
                d.priority = 10.0f;
                d.bus = master;
                engine.voices().play(d, std::unique_ptr<VoiceSource>(new ModalImpactSource(1, 320.0f, 0.9f)));
            }
            engine.render(channels, 2, block);
            for (int i = 0; i < block; ++i)
                peak = std::max(peak, std::fabs(static_cast<double>(left[i])));
        }
        std::printf("no audio device (headless) — rendered, peak=%.4f\n", peak);
        if (peak > 8.0)
        {
            std::fprintf(stderr, "audio_procedural_demo FAILED: mix unbounded (peak %.4f)\n", peak);
            return 1;
        }
    }

    std::printf("audio_procedural_demo OK\n");
    return 0;
}
