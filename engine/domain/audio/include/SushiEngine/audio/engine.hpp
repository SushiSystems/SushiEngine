/**************************************************************************/
/* engine.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_ENGINE_HPP
#define SUSHIENGINE_AUDIO_ENGINE_HPP

/**
 * @file engine.hpp
 * @brief The audio engine: the render-plane entry point that ties voices to the mixer.
 *
 * @ref AudioEngine is the @ref IAudioRenderer the device drives (§12). Its @ref render
 * is the top of the audio-render plane: it sets the denormal guard for the whole
 * callback, clears the mixer accumulators, folds the voice manager's real voices into
 * the buses, runs the bus graph, and fans the stereo master out to the device
 * channels. Everything below it is allocation-, lock-, and syscall-free.
 *
 * This is the S2 capstone; parameter changes reach it through the atomics on
 * @ref SmoothedValue / voice descriptors. The batched command ring (§0) that lets the
 * control thread also *start and stop* voices mid-stream layers on in a later phase —
 * for now voices are set up before the device opens.
 */

#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/audio/channel_layout.hpp>
#include <SushiEngine/audio/device.hpp>
#include <SushiEngine/audio/dsp/denormals.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/profiler.hpp>
#include <SushiEngine/audio/spatializer.hpp>
#include <SushiEngine/audio/voice_manager.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief Owns a voice manager and a mixer and renders them as one @ref IAudioRenderer.
         *
         * Build the mixer's bus topology and the voices through @ref mixer and
         * @ref voices, call @ref prepare once, then hand the engine to an
         * @ref IAudioDevice — or call @ref render directly for headless processing.
         */
        class AudioEngine final : public IAudioRenderer
        {
            public:
                /**
                 * @brief Constructs the engine and its voice pool.
                 * @param voice_pool_capacity Maximum simultaneously-active voices.
                 * @param max_real_voices     Maximum voices rendered per block.
                 */
                AudioEngine(int voice_pool_capacity, int max_real_voices)
                    : voices_(voice_pool_capacity, max_real_voices)
                {
                }

                /** @brief The mixer bus graph, for building topology and setting bus gains. */
                MixerGraph& mixer() noexcept { return mixer_; }

                /** @brief The voice manager, for starting/stopping voices and setting the listener. */
                VoiceManager& voices() noexcept { return voices_; }

                /** @brief The binaural spatializer spatial voices are rendered through. */
                BinauralSpatializer& spatializer() noexcept { return spatializer_; }

                /**
                 * @brief The live-profiler telemetry channel (§11).
                 *
                 * The audio thread publishes an @ref AudioProfileSnapshot here at the end of
                 * every @ref render; the GUI (or a test) reads the latest via
                 * @ref AudioProfiler::latest. Enabled by default; @ref set_profiling(false)
                 * skips the (small) per-block gather when no one is listening.
                 */
                AudioProfiler& profiler() noexcept { return profiler_; }

                /** @brief Enables or disables the per-block profiler gather (default on). */
                void set_profiling(bool enabled) noexcept { profiling_ = enabled; }

                /**
                 * @brief Sets the ambisonic order of the scene bus (call before @ref prepare).
                 * @param order The order (1..3 typical; higher sharpens localisation at more cost).
                 */
                void set_ambisonic_order(int order) noexcept { ambisonic_order_ = order; }

                /**
                 * @brief Prepares the mixer, spatializer, and voice manager for a run.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block that will be rendered.
                 */
                void prepare(double sample_rate, int max_block_frames)
                {
                    sample_rate_ = sample_rate;
                    max_block_ = max_block_frames;
                    mixer_.prepare(sample_rate, max_block_frames);
                    spatializer_.configure(ambisonic_order_, sample_rate, max_block_frames);
                    voices_.prepare(sample_rate, max_block_frames);
                    voices_.set_spatializer(&spatializer_);
                    binaural_left_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                    binaural_right_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                    // One-pole low-pass at ~120 Hz for the LFE channel (surround output).
                    lfe_coeff_ = 1.0f - std::exp(-2.0f * 3.14159265358979f * 120.0f /
                                                 static_cast<float>(sample_rate));
                    lfe_lp_ = 0.0f;
                    // ITU-R BS.1770 K-weighting (a ~+4 dB high-shelf then a ~38 Hz high-pass)
                    // for the loudness (LUFS) meter.
                    kweight_shelf_.set_high_shelf(1500.0, 4.0, sample_rate);
                    kweight_hp_.set_high_pass(38.0, 0.5, sample_rate);
                    kweight_shelf_.reset();
                    kweight_hp_.reset();
                    loudness_mean_square_ = 0.0f;
                }

                /**
                 * @brief Renders one block: voices → mixer → device channels.
                 *
                 * Fans the stereo master to the output: channel 0 = left, channel 1 =
                 * right, any further channels get the left as a safe default until the
                 * multichannel decode of §4 lands. A mono device gets the left channel.
                 *
                 * A device may hand a larger block than @ref prepare was told (the OS
                 * mixer's buffer can exceed the requested size); the internal render is
                 * clamped to the prepared maximum and any surplus device samples are
                 * zero-filled, so an under-sized `prepare` degrades to a brief silence
                 * rather than a buffer overrun. Size `prepare` at or above the device
                 * block to avoid that.
                 *
                 * @param channels      The device's planar output buffers.
                 * @param channel_count The device channel count.
                 * @param frame_count   Number of samples this block.
                 */
                void render(float* const* channels, int channel_count, int frame_count) noexcept override
                {
                    DSP::ScopedNoDenormals guard;

                    const int n = frame_count < max_block_ ? frame_count : max_block_;

                    spatializer_.begin_block(n);
                    mixer_.begin_block(n);
                    voices_.render(mixer_, n); // spatial → spatializer, non-spatial → mixer
                    mixer_.process(n);

                    const float* left = mixer_.master_left();
                    const float* right = mixer_.master_right();

                    if (channel_count <= 2)
                    {
                        // Headphone / stereo path: analytic binaural decode of the scene bus,
                        // summed with the non-spatial stereo master.
                        DSP::SIMD::fill(binaural_left_.data(), n, 0.0f);
                        DSP::SIMD::fill(binaural_right_.data(), n, 0.0f);
                        spatializer_.decode_binaural(binaural_left_.data(), binaural_right_.data(), n);
                        for (int c = 0; c < channel_count; ++c)
                        {
                            const float* master = (c == 1) ? right : left;
                            const float* binaural =
                                (c == 1) ? binaural_right_.data() : binaural_left_.data();
                            for (int i = 0; i < n; ++i)
                                channels[c][i] = master[i] + binaural[i];
                            for (int i = n; i < frame_count; ++i)
                                channels[c][i] = 0.0f;
                        }
                    }
                    else
                    {
                        // Discrete multichannel surround: decode the scene bus to each real
                        // speaker direction, pan the non-spatial stereo bed across the speakers
                        // by their left/right position, and derive the LFE from the bass sum.
                        int speaker_count = 0;
                        const OutputSpeaker* speakers =
                            speakers_for(layout_for(channel_count), speaker_count);
                        for (int c = 0; c < channel_count; ++c)
                        {
                            float* out = channels[c];
                            for (int i = 0; i < frame_count; ++i)
                                out[i] = 0.0f;
                            if (c >= speaker_count)
                                continue;
                            const OutputSpeaker& s = speakers[c];
                            if (s.lfe)
                            {
                                for (int i = 0; i < n; ++i)
                                {
                                    const float bass = left[i] + right[i];
                                    lfe_lp_ += lfe_coeff_ * (bass - lfe_lp_);
                                    out[i] = lfe_lp_ * 0.7f;
                                }
                                continue;
                            }
                            // Non-spatial bed: pan L/R by the speaker's lateral position, with
                            // rears/sides a touch lower.
                            const float wl = 0.5f + 0.5f * s.y;
                            const float wr = 1.0f - wl;
                            const float front = s.x >= 0.0f ? 1.0f : 0.75f;
                            for (int i = 0; i < n; ++i)
                                out[i] = front * (left[i] * wl + right[i] * wr);
                            // Spatial scene: true surround decode onto this speaker.
                            spatializer_.decode_direction(s.x, s.y, s.z, out, n);
                        }
                    }

                    if (profiling_)
                        publish_profile(channels, channel_count, n);
                }

            private:
                /**
                 * @brief Gathers the block's telemetry and publishes it to the profiler.
                 *
                 * Meters the true device output (channel 0, i.e. master + binaural), reads
                 * the voice-manager population and the mixer's per-bus meters, and stores a
                 * downsampled scope of the output. Called at the end of @ref render.
                 */
                void publish_profile(float* const* channels, int channel_count, int n) noexcept
                {
                    AudioProfileSnapshot s;
                    s.block_index = ++block_index_;
                    s.real_voices = voices_.real_count();
                    s.active_voices = voices_.active_count();
                    s.virtual_voices = s.active_voices - s.real_voices;

                    const float* out = channel_count > 0 ? channels[0] : nullptr;
                    float peak = 0.0f;
                    double sum_sq = 0.0;
                    if (out != nullptr)
                    {
                        for (int i = 0; i < n; ++i)
                        {
                            const float a = std::fabs(out[i]);
                            if (a > peak) peak = a;
                            sum_sq += static_cast<double>(out[i]) * out[i];
                        }
                    }
                    s.master_peak = peak;
                    s.master_rms = n > 0 ? static_cast<float>(std::sqrt(sum_sq / n)) : 0.0f;

                    // True (inter-sample) peak: 4× oversample with a Catmull-Rom spline and
                    // take the largest magnitude, so a peak that falls between samples (which
                    // a D/A reconstructs and can clip) is caught.
                    float true_peak = peak;
                    if (out != nullptr)
                    {
                        for (int i = 0; i + 1 < n; ++i)
                        {
                            const float pm1 = i > 0 ? out[i - 1] : out[i];
                            const float p0 = out[i];
                            const float p1 = out[i + 1];
                            const float p2 = (i + 2 < n) ? out[i + 2] : out[i + 1];
                            for (int s3 = 1; s3 < 4; ++s3)
                            {
                                const float t = 0.25f * static_cast<float>(s3);
                                const float v =
                                    0.5f * ((2.0f * p0) + (-pm1 + p1) * t +
                                            (2.0f * pm1 - 5.0f * p0 + 4.0f * p1 - p2) * t * t +
                                            (-pm1 + 3.0f * p0 - 3.0f * p1 + p2) * t * t * t);
                                const float a = std::fabs(v);
                                if (a > true_peak)
                                    true_peak = a;
                            }
                        }
                    }
                    s.master_true_peak = true_peak;
                    s.cpu_load = 0.0f; // wall-clock timing is measured by the host, not the RT path

                    // Momentary loudness (LUFS): K-weight the output, mean-square it, and
                    // average over a ~400 ms window (ITU-R BS.1770 gating omitted for a live
                    // meter). LUFS = −0.691 + 10·log10(mean square).
                    if (out != nullptr && n > 0)
                    {
                        double ms = 0.0;
                        for (int i = 0; i < n; ++i)
                        {
                            const float k = kweight_hp_.process(kweight_shelf_.process(out[i]));
                            ms += static_cast<double>(k) * k;
                        }
                        ms /= n;
                        const float a = std::exp(-static_cast<float>(n) /
                                                 (0.4f * static_cast<float>(sample_rate_ > 0 ? sample_rate_ : 48000.0)));
                        loudness_mean_square_ = a * loudness_mean_square_ + (1.0f - a) * static_cast<float>(ms);
                        s.master_lufs = -0.691f + 10.0f * std::log10(loudness_mean_square_ + 1.0e-12f);
                    }

                    int buses = mixer_.bus_count();
                    if (buses > PROFILE_MAX_BUSES)
                        buses = PROFILE_MAX_BUSES;
                    s.bus_count = buses;
                    for (int b = 0; b < buses; ++b)
                    {
                        s.buses[b].peak = mixer_.bus_peak(b);
                        s.buses[b].rms = mixer_.bus_rms(b);
                    }

                    if (out != nullptr && n > 0)
                    {
                        const int points = n < PROFILE_SCOPE_POINTS ? n : PROFILE_SCOPE_POINTS;
                        s.scope_points = points;
                        for (int p = 0; p < points; ++p)
                            s.scope[p] = out[(p * n) / points];
                    }

                    profiler_.publish(s);
                }

                MixerGraph mixer_;
                BinauralSpatializer spatializer_;
                VoiceManager voices_;
                AudioProfiler profiler_;
                bool profiling_ = true;
                std::uint64_t block_index_ = 0;
                int ambisonic_order_ = 3;
                int max_block_ = 0;
                double sample_rate_ = 48000.0;
                std::vector<float> binaural_left_;
                std::vector<float> binaural_right_;
                float lfe_coeff_ = 0.0f;
                float lfe_lp_ = 0.0f;
                DSP::Biquad kweight_shelf_;
                DSP::Biquad kweight_hp_;
                float loudness_mean_square_ = 0.0f;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
