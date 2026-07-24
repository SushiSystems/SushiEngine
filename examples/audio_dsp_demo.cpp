/**************************************************************************/
/* audio_dsp_demo.cpp                                                    */
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
 * @file audio_dsp_demo.cpp
 * @brief Phase S1 vertical slice: play, filter, and mix a sine through the block graph.
 *
 * Stands up the DSP core: two sine sources (a low tone and a high tone) are summed,
 * low-passed by an RBJ biquad, and gain-staged — the classic "generate → mix → filter"
 * chain — pulled one block at a time through the @ref BlockGraph. It:
 *
 *   1. Runs the graph headless and self-checks the output: bounded, finite, and an RMS
 *      that proves the low tone passed while the high tone was removed by the filter
 *      (a broken or bypassed filter would leave a measurably larger RMS). No hardware
 *      needed — this is the CI check.
 *   2. Best-effort opens the SDL2 device and plays the chain for a moment through a
 *      @ref GraphRenderer, with the denormal guard set for the callback. A headless
 *      host simply skips this.
 *
 * Exits 0 on success.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

#include <SushiEngine/audio/device.hpp>
#include <SushiEngine/audio/dsp/dsp.hpp>
#include <sdl/sdl_audio_device.hpp>

using SushiEngine::Audio::AudioStreamFormat;
using SushiEngine::Audio::IAudioRenderer;
using SushiEngine::Audio::SdlAudioDevice;

namespace Dsp = SushiEngine::Audio::Dsp;

namespace
{
    /**
     * @brief Builds the demo chain into @p graph and prepares it.
     *
     * Sine(200 Hz) + Sine(5000 Hz) → Mix → biquad low-pass(700 Hz) → gain, with the
     * gain node as the graph output. The 5000 Hz tone sits well above the cutoff, so a
     * working filter removes it and only the 200 Hz tone survives.
     *
     * @param graph            The graph to populate.
     * @param sample_rate      The stream sample rate in Hz.
     * @param max_block_frames The largest block that will be processed.
     */
    void build_graph(Dsp::BlockGraph& graph, double sample_rate, int max_block_frames)
    {
        const Dsp::NodeId low = graph.add_node(std::unique_ptr<Dsp::Node>(new Dsp::SineNode(200.0f, 0.5f)));
        const Dsp::NodeId high = graph.add_node(std::unique_ptr<Dsp::Node>(new Dsp::SineNode(5000.0f, 0.5f)));
        const Dsp::NodeId mix = graph.add_node(std::unique_ptr<Dsp::Node>(new Dsp::MixNode(2)));
        const Dsp::NodeId filter = graph.add_node(std::unique_ptr<Dsp::Node>(new Dsp::BiquadNode()));
        const Dsp::NodeId gain = graph.add_node(std::unique_ptr<Dsp::Node>(new Dsp::GainNode(1.0f)));

        graph.node_as<Dsp::BiquadNode>(filter)->filter().set_low_pass(700.0, 0.707, sample_rate);

        graph.connect(low, 0, mix, 0);
        graph.connect(high, 0, mix, 1);
        graph.connect(mix, 0, filter, 0);
        graph.connect(filter, 0, gain, 0);
        graph.set_graph_output(gain, 0);

        graph.prepare(sample_rate, max_block_frames);
    }

    /**
     * @brief An @ref IAudioRenderer that pulls a @ref BlockGraph and fans it to all channels.
     *
     * The graph is prepared before the device opens, so @ref render only runs the
     * (allocation-free) block pass and copies the mono output to each channel, under the
     * denormal guard the audio thread needs.
     */
    class GraphRenderer final : public IAudioRenderer
    {
        public:
            explicit GraphRenderer(Dsp::BlockGraph& graph) noexcept : graph_(graph) {}

            void render(float* const* channels, int channel_count, int frame_count) noexcept override
            {
                Dsp::ScopedNoDenormals guard;
                graph_.process(frame_count);
                const float* mono = graph_.output_buffer();
                for (int c = 0; c < channel_count; ++c)
                {
                    if (mono != nullptr)
                    {
                        for (int f = 0; f < frame_count; ++f)
                            channels[c][f] = mono[f];
                    }
                    else
                    {
                        for (int f = 0; f < frame_count; ++f)
                            channels[c][f] = 0.0f;
                    }
                }
                blocks_.fetch_add(1, std::memory_order_relaxed);
            }

            std::uint64_t blocks() const noexcept { return blocks_.load(std::memory_order_relaxed); }

        private:
            Dsp::BlockGraph& graph_;
            std::atomic<std::uint64_t> blocks_{0};
    };
} // namespace

int main()
{
    const double sample_rate = 48000.0;
    const int block_frames = 512;

    // (1) Headless: run the chain and measure the settled output.
    Dsp::BlockGraph headless;
    build_graph(headless, sample_rate, block_frames);

    const int warmup_blocks = 50;
    const int measured_blocks = 200;
    double sum_squares = 0.0;
    double peak = 0.0;
    long long measured_samples = 0;

    for (int b = 0; b < warmup_blocks + measured_blocks; ++b)
    {
        headless.process(block_frames);
        const float* out = headless.output_buffer();
        if (out == nullptr)
        {
            std::fprintf(stderr, "audio_dsp_demo FAILED: null graph output\n");
            return 1;
        }
        if (b < warmup_blocks)
            continue;
        for (int i = 0; i < block_frames; ++i)
        {
            const double s = static_cast<double>(out[i]);
            if (!(s == s) || s > 2.0 || s < -2.0) // NaN or out of any sane range
            {
                std::fprintf(stderr, "audio_dsp_demo FAILED: bad sample %f\n", s);
                return 1;
            }
            sum_squares += s * s;
            const double a = s < 0.0 ? -s : s;
            if (a > peak)
                peak = a;
            ++measured_samples;
        }
    }

    const double rms = std::sqrt(sum_squares / static_cast<double>(measured_samples));
    std::printf("headless chain: rms=%.4f peak=%.4f over %lld samples\n", rms, peak, measured_samples);

    // A lone 0.5-amplitude 200 Hz sine has RMS ~0.354; if the 5 kHz tone were NOT
    // filtered out, the summed RMS would be ~0.5. The band proves the filter worked.
    if (rms < 0.28 || rms > 0.42)
    {
        std::fprintf(stderr, "audio_dsp_demo FAILED: rms %.4f outside [0.28, 0.42] "
                             "(filter not attenuating the high tone?)\n", rms);
        return 1;
    }
    if (peak > 0.99)
    {
        std::fprintf(stderr, "audio_dsp_demo FAILED: peak %.4f too hot\n", peak);
        return 1;
    }

    // (2) Best-effort playback.
    Dsp::BlockGraph play;
    build_graph(play, sample_rate, 2048);
    GraphRenderer renderer(play);

    SdlAudioDevice device;
    AudioStreamFormat desired;
    desired.sample_rate = 48000;
    desired.channel_count = 2;
    desired.block_frames = block_frames;

    if (device.open(desired, renderer))
    {
        const AudioStreamFormat obtained = device.format();
        std::printf("audio device open: %d Hz, %d ch, %d frames/block — playing the chain\n",
                    obtained.sample_rate, obtained.channel_count, obtained.block_frames);
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        device.close();
        std::printf("device rendered %llu blocks\n",
                    static_cast<unsigned long long>(renderer.blocks()));
    }
    else
    {
        std::printf("no audio device available (headless) — chain verified in software\n");
    }

    std::printf("audio_dsp_demo OK\n");
    return 0;
}
