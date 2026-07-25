/**************************************************************************/
/* spatializer.hpp                                                       */
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

#ifndef SUSHIENGINE_AUDIO_SPATIALIZER_HPP
#define SUSHIENGINE_AUDIO_SPATIALIZER_HPP

/**
 * @file spatializer.hpp
 * @brief The ambisonic scene bus and its binaural decode — head-tracked 3D audio.
 *
 * This is the rendering core of §4. Each source is **encoded** into a shared
 * ambisonic bus with its spherical-harmonic gains — cheap, per-source — so any number
 * of sources collapse into one fixed field. The field is then **decoded once** to a
 * fixed layout of virtual loudspeakers, and each virtual speaker is rendered to the
 * two ears through an analytic head model: an interaural time difference (a fractional
 * delay per ear, from the Woodworth spherical-head formula) and a head-shadow
 * low-pass on the far ear. Because the speaker layout is fixed, the number of ear
 * renders is constant no matter how many sources play — the scalability property that
 * makes this the right structure for a game.
 *
 * Head tracking is free: the caller encodes each source in **head-relative**
 * coordinates (@ref head_relative_direction rotates a world direction into the
 * listener's frame), so turning the head re-aims every source with no extra state.
 *
 * The analytic HRTF is self-contained (no measured data needed) and gives solid
 * left/right localisation and externalisation; a measured-HRTF / SOFA + MagLS path is
 * a fidelity upgrade that slots in behind this same encode → decode seam later.
 */

#include <cmath>
#include <cstddef>
#include <vector>

#include <SushiEngine/audio/dsp/filters/one_pole.hpp>
#include <SushiEngine/audio/dsp/fractional_delay.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/dsp/spherical_harmonics.hpp>
#include <SushiEngine/audio/hrtf.hpp>
#include <SushiEngine/audio/magls.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief Rotates a world-space direction into the listener's head frame.
         *
         * Projects onto the listener's basis: front = @p forward, left =
         * `up × forward`, up = @p up. The result's components are (front, left, up),
         * the frame the encoder expects. @p forward and @p up should be unit and
         * roughly orthogonal.
         *
         * @param rel_x,rel_y,rel_z       The world direction from listener to source.
         * @param fwd_x,fwd_y,fwd_z       The listener's forward vector.
         * @param up_x,up_y,up_z          The listener's up vector.
         * @param out_x,out_y,out_z       Set to the head-relative (front, left, up) direction.
         */
        inline void head_relative_direction(float rel_x, float rel_y, float rel_z,
                                            float fwd_x, float fwd_y, float fwd_z,
                                            float up_x, float up_y, float up_z,
                                            float& out_x, float& out_y, float& out_z) noexcept
        {
            // left = up × forward
            const float left_x = up_y * fwd_z - up_z * fwd_y;
            const float left_y = up_z * fwd_x - up_x * fwd_z;
            const float left_z = up_x * fwd_y - up_y * fwd_x;
            out_x = rel_x * fwd_x + rel_y * fwd_y + rel_z * fwd_z;   // front
            out_y = rel_x * left_x + rel_y * left_y + rel_z * left_z; // left
            out_z = rel_x * up_x + rel_y * up_y + rel_z * up_z;      // up
        }

        /**
         * @brief An ambisonic scene bus with an analytic binaural decode.
         *
         * @ref configure once, then per block: @ref begin_block, @ref encode each
         * source (head-relative direction), and @ref decode_binaural to the stereo ears.
         */
        class BinauralSpatializer
        {
            public:
                /**
                 * @brief Sets up the bus, the virtual speaker layout, and the head model.
                 * @param order            Ambisonic order (1..3 typical; higher = sharper).
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block decoded.
                 * @param head_radius_m    Head radius for the ITD (default 0.0875 m).
                 */
                void configure(int order, double sample_rate, int max_block_frames,
                               float head_radius_m = 0.0875f)
                {
                    order_ = order;
                    sample_rate_ = sample_rate;
                    max_block_ = max_block_frames;
                    channels_ = Dsp::ambisonic_channel_count(order_);

                    bus_.assign(static_cast<std::size_t>(channels_), {});
                    for (std::vector<float>& channel : bus_)
                        channel.assign(static_cast<std::size_t>(max_block_), 0.0f);
                    speaker_scratch_.assign(static_cast<std::size_t>(max_block_), 0.0f);
                    ear_scratch_.assign(static_cast<std::size_t>(max_block_), 0.0f);

                    build_speakers(head_radius_m);
                    calibrate();
                    load_hrir_into_speakers();
                }

                /** @brief The number of ambisonic channels. */
                int channel_count() const noexcept { return channels_; }

                /**
                 * @brief Selects a measured-HRTF database for the binaural decode.
                 *
                 * With a database set, @ref decode_binaural convolves each virtual speaker's
                 * signal through that direction's measured HRIR pair instead of the analytic
                 * ITD/shadow/timbre model — the pinna and torso cues of a real head. Pass
                 * `nullptr` to fall back to the analytic model. Safe to call after @ref configure;
                 * the per-speaker impulse responses are (re)loaded immediately.
                 *
                 * @param database The HRIR provider, or `nullptr` for the analytic path.
                 */
                void set_hrtf_database(const IHrtfDatabase* database)
                {
                    hrtf_ = database;
                    load_hrir_into_speakers();
                }

                /** @brief Whether a measured-HRTF database is active. */
                bool uses_measured_hrtf() const noexcept { return hrtf_ != nullptr; }

                /**
                 * @brief Selects a Magnitude-Least-Squares decoder for the binaural output.
                 *
                 * When set (and valid), @ref decode_binaural applies the decoder's per-channel
                 * FIRs directly to the ambisonic bus — the highest-fidelity path, taking priority
                 * over both the per-speaker measured HRIR and the analytic model. Pass `nullptr` to
                 * fall back. The decoder is borrowed and must outlive the spatializer.
                 *
                 * @param decoder The MagLS decoder, or `nullptr`.
                 */
                void set_magls_decoder(MaglsBinauralDecoder* decoder) noexcept { magls_ = decoder; }

                /** @brief Whether a MagLS decoder is active. */
                bool uses_magls() const noexcept { return magls_ != nullptr && magls_->valid(); }

                /** @brief Zeroes the ambisonic bus for a fresh block. */
                void begin_block(int frame_count) noexcept
                {
                    if (frame_count > max_block_)
                        frame_count = max_block_;
                    for (std::vector<float>& channel : bus_)
                        Dsp::Simd::fill(channel.data(), frame_count, 0.0f);
                    encoded_ = false;
                }

                /** @brief Whether any source was encoded into the bus this block. */
                bool has_content() const noexcept { return encoded_; }

                /**
                 * @brief Encodes a mono source into the ambisonic bus at a head-relative direction.
                 * @param mono        The source block.
                 * @param frame_count Number of samples.
                 * @param dir_x,dir_y,dir_z The head-relative (front, left, up) direction.
                 * @param gain        Linear gain for the source.
                 */
                void encode(const float* mono, int frame_count, float dir_x, float dir_y, float dir_z,
                            float gain) noexcept
                {
                    float gains[Dsp::MAX_AMBISONIC_CHANNELS];
                    Dsp::ambisonic_encode_gains(order_, dir_x, dir_y, dir_z, gains);
                    for (int ch = 0; ch < channels_; ++ch)
                        Dsp::Simd::mix_accumulate(bus_[static_cast<std::size_t>(ch)].data(), mono,
                                                  frame_count, gain * gains[ch]);
                    encoded_ = true;
                }

                /**
                 * @brief Decodes the bus to the two ears and accumulates into @p left / @p right.
                 *
                 * Each virtual speaker's signal is the projection of the bus onto that
                 * speaker's direction; it is then delayed and shadowed per ear and summed.
                 * The outputs are **added** to the buffers (the caller clears them), so the
                 * binaural field can be mixed alongside a non-spatial stereo bus.
                 *
                 * @param left        Left-ear output (accumulated).
                 * @param right       Right-ear output (accumulated).
                 * @param frame_count Number of samples.
                 */
                void decode_binaural(float* left, float* right, int frame_count) noexcept
                {
                    if (frame_count > max_block_)
                        frame_count = max_block_;
                    if (!encoded_)
                        return; // nothing spatial this block: leave the ear buffers as-is

                    if (magls_ != nullptr && magls_->valid())
                    {
                        // Highest-fidelity path: decode the bus straight to the ears with the
                        // per-channel MagLS FIRs, bypassing the virtual-speaker layout entirely.
                        bus_ptrs_.resize(static_cast<std::size_t>(channels_));
                        for (int ch = 0; ch < channels_; ++ch)
                            bus_ptrs_[static_cast<std::size_t>(ch)] =
                                bus_[static_cast<std::size_t>(ch)].data();
                        magls_->process(bus_ptrs_.data(), channels_, frame_count, left, right);
                        return;
                    }

                    for (Speaker& speaker : speakers_)
                    {
                        // Speaker signal = projection of the bus onto the speaker direction.
                        Dsp::Simd::fill(speaker_scratch_.data(), frame_count, 0.0f);
                        for (int ch = 0; ch < channels_; ++ch)
                            Dsp::Simd::mix_accumulate(speaker_scratch_.data(),
                                                      bus_[static_cast<std::size_t>(ch)].data(),
                                                      frame_count, decode_scale_ * speaker.gains[static_cast<std::size_t>(ch)]);

                        if (hrtf_ != nullptr)
                        {
                            // Measured path: the HRIR pair carries the full ITD, shadow, and
                            // pinna/torso spectrum, so it replaces the analytic ear model.
                            speaker.left_hrir.process_block(speaker_scratch_.data(), left,
                                                            frame_count);
                            speaker.right_hrir.process_block(speaker_scratch_.data(), right,
                                                             frame_count);
                            continue;
                        }

                        // Apply the direction spectral cue (front/back + elevation) before the
                        // per-ear ITD/shadow, so rear/elevated sources are timbrally distinct.
                        for (int i = 0; i < frame_count; ++i)
                            speaker_scratch_[static_cast<std::size_t>(i)] =
                                speaker.timbre.process(speaker_scratch_[static_cast<std::size_t>(i)]);

                        accumulate_ear(speaker.left_delay, speaker.left_shadow, speaker.left_is_far,
                                       speaker.left_itd, speaker.left_gain, speaker.left_cutoff,
                                       left, frame_count);
                        accumulate_ear(speaker.right_delay, speaker.right_shadow, speaker.right_is_far,
                                       speaker.right_itd, speaker.right_gain, speaker.right_cutoff,
                                       right, frame_count);
                    }
                }

                /**
                 * @brief Decodes the ambisonic bus to one real loudspeaker direction (surround).
                 *
                 * The discrete-speaker counterpart of @ref decode_binaural: projects the
                 * scene bus onto @p (ux,uy,uz) and **adds** the result into @p out — used to
                 * render true multichannel surround (one call per output speaker) instead of
                 * folding everything to the two ears. Head-relative directions, so a head
                 * turn rotates the surround field just like the binaural path.
                 *
                 * @param ux,uy,uz    The speaker's unit direction (x front, y left, z up).
                 * @param out         The speaker's mono output, accumulated into.
                 * @param frame_count Number of samples.
                 */
                void decode_direction(float ux, float uy, float uz, float* out, int frame_count) noexcept
                {
                    if (frame_count > max_block_)
                        frame_count = max_block_;
                    if (!encoded_)
                        return;
                    float gains[Dsp::MAX_AMBISONIC_CHANNELS];
                    Dsp::ambisonic_encode_gains(order_, ux, uy, uz, gains);
                    for (int ch = 0; ch < channels_; ++ch)
                        Dsp::Simd::mix_accumulate(out, bus_[static_cast<std::size_t>(ch)].data(),
                                                  frame_count, decode_scale_ * gains[ch]);
                }

            private:
                struct Speaker
                {
                    std::vector<float> gains; // encode gains at this speaker's direction (ACN)
                    float dir_x = 0.0f;       // unit direction (front) for the HRIR lookup
                    float dir_y = 0.0f;       // unit direction (left)
                    float dir_z = 0.0f;       // unit direction (up)
                    HrirConvolver left_hrir;  // measured-HRTF path (when a database is set)
                    HrirConvolver right_hrir;
                    Dsp::Biquad timbre;       // direction spectral cue: front/back + elevation
                    Dsp::FractionalDelayLine left_delay;
                    Dsp::FractionalDelayLine right_delay;
                    Dsp::OnePole left_shadow;
                    Dsp::OnePole right_shadow;
                    float left_itd = 1.0f;
                    float right_itd = 1.0f;
                    float left_gain = 1.0f;
                    float right_gain = 1.0f;
                    float left_cutoff = 20000.0f;
                    float right_cutoff = 20000.0f;
                    bool left_is_far = false;
                    bool right_is_far = false;
                };

                void accumulate_ear(Dsp::FractionalDelayLine& delay, Dsp::OnePole& shadow, bool is_far,
                                    float itd, float gain, float cutoff, float* out,
                                    int frame_count) noexcept
                {
                    delay.process_block(speaker_scratch_.data(), ear_scratch_.data(), frame_count, itd, itd);
                    if (is_far)
                    {
                        shadow.set_low_pass(cutoff, sample_rate_);
                        for (int i = 0; i < frame_count; ++i)
                            ear_scratch_[static_cast<std::size_t>(i)] =
                                shadow.process_low_pass(ear_scratch_[static_cast<std::size_t>(i)]);
                    }
                    Dsp::Simd::mix_accumulate(out, ear_scratch_.data(), frame_count, gain);
                }

                void build_speakers(float head_radius_m)
                {
                    const float directions[][3] = {
                        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
                        {1, 1, 1}, {1, 1, -1}, {1, -1, 1}, {1, -1, -1},
                        {-1, 1, 1}, {-1, 1, -1}, {-1, -1, 1}, {-1, -1, -1},
                        {1, 1, 0}, {1, -1, 0}, {-1, 1, 0}, {-1, -1, 0},
                        {1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1},
                        {0, 1, 1}, {0, 1, -1}, {0, -1, 1}, {0, -1, -1}};
                    const int count = static_cast<int>(sizeof(directions) / sizeof(directions[0]));

                    const float c = 343.0f; // speed of sound for the head-model ITD
                    const int itd_headroom = 128;

                    speakers_.clear();
                    speakers_.resize(static_cast<std::size_t>(count));
                    for (int s = 0; s < count; ++s)
                    {
                        Speaker& speaker = speakers_[static_cast<std::size_t>(s)];
                        const float len = std::sqrt(directions[s][0] * directions[s][0] +
                                                    directions[s][1] * directions[s][1] +
                                                    directions[s][2] * directions[s][2]);
                        const float ux = directions[s][0] / len;
                        const float uy = directions[s][1] / len; // left component = lateral
                        const float uz = directions[s][2] / len;

                        speaker.dir_x = ux;
                        speaker.dir_y = uy;
                        speaker.dir_z = uz;
                        speaker.gains.assign(static_cast<std::size_t>(channels_), 0.0f);
                        Dsp::ambisonic_encode_gains(order_, ux, uy, uz, speaker.gains.data());

                        // Direction spectral cue (breaks the front/back and up/down cone of
                        // confusion the ITD alone cannot): the pinna dulls rear sources and
                        // brightens elevated ones. A high-shelf whose gain follows front/back
                        // (ux) and elevation (uz).
                        const float fb_db = ux >= 0.0f ? 2.0f * ux : 8.0f * ux;
                        const float el_db = 3.0f * uz;
                        speaker.timbre.set_high_shelf(5000.0, static_cast<double>(fb_db + el_db),
                                                      sample_rate_);
                        speaker.timbre.reset();

                        speaker.left_delay.prepare(itd_headroom);
                        speaker.right_delay.prepare(itd_headroom);
                        speaker.left_shadow.reset();
                        speaker.right_shadow.reset();

                        // Woodworth ITD from the lateral angle; the far ear lags.
                        const float lateral = uy < -1.0f ? -1.0f : (uy > 1.0f ? 1.0f : uy);
                        const float theta = std::asin(lateral);
                        const float abs_theta = theta < 0.0f ? -theta : theta;
                        const float itd_samples =
                            (head_radius_m / c) * (abs_theta + std::sin(abs_theta)) *
                            static_cast<float>(sample_rate_);
                        const float shadow_amount = abs_theta / 1.5707963f; // 0 front .. 1 side
                        const float far_cutoff = 20000.0f - 18000.0f * shadow_amount;
                        const float far_gain = 1.0f - 0.25f * shadow_amount;

                        const float min_delay = 1.0f;
                        if (lateral >= 0.0f) // left source: left ear near, right ear far
                        {
                            speaker.left_itd = min_delay;
                            speaker.right_itd = min_delay + itd_samples;
                            speaker.left_gain = 1.0f;
                            speaker.right_gain = far_gain;
                            speaker.left_is_far = false;
                            speaker.right_is_far = true;
                            speaker.right_cutoff = far_cutoff;
                        }
                        else // right source: right ear near, left ear far
                        {
                            speaker.right_itd = min_delay;
                            speaker.left_itd = min_delay + itd_samples;
                            speaker.right_gain = 1.0f;
                            speaker.left_gain = far_gain;
                            speaker.right_is_far = false;
                            speaker.left_is_far = true;
                            speaker.left_cutoff = far_cutoff;
                        }
                    }
                }

                // (Re)load each virtual speaker's measured HRIR pair from the active database.
                void load_hrir_into_speakers()
                {
                    if (hrtf_ == nullptr || speakers_.empty())
                        return;
                    const int n = hrtf_->ir_length();
                    if (n <= 0)
                        return;
                    std::vector<float> left_ir(static_cast<std::size_t>(n), 0.0f);
                    std::vector<float> right_ir(static_cast<std::size_t>(n), 0.0f);
                    for (Speaker& speaker : speakers_)
                    {
                        hrtf_->get_hrir(speaker.dir_x, speaker.dir_y, speaker.dir_z, left_ir.data(),
                                        right_ir.data());
                        speaker.left_hrir.prepare(left_ir.data(), n);
                        speaker.right_hrir.prepare(right_ir.data(), n);
                    }
                }

                // Normalise so a unit source straight ahead reaches ~unit level at each ear.
                void calibrate()
                {
                    float front[Dsp::MAX_AMBISONIC_CHANNELS];
                    Dsp::ambisonic_encode_gains(order_, 1.0f, 0.0f, 0.0f, front);
                    double left_sum = 0.0;
                    for (const Speaker& speaker : speakers_)
                    {
                        double projection = 0.0;
                        for (int ch = 0; ch < channels_; ++ch)
                            projection += static_cast<double>(speaker.gains[static_cast<std::size_t>(ch)]) * front[ch];
                        left_sum += projection * speaker.left_gain;
                    }
                    decode_scale_ = (left_sum > 1e-6) ? static_cast<float>(1.0 / left_sum) : 1.0f;
                }

                int order_ = 3;
                int channels_ = 16;
                double sample_rate_ = 48000.0;
                int max_block_ = 0;
                float decode_scale_ = 1.0f;
                bool encoded_ = false;
                const IHrtfDatabase* hrtf_ = nullptr;
                MaglsBinauralDecoder* magls_ = nullptr;
                std::vector<const float*> bus_ptrs_;
                std::vector<std::vector<float>> bus_;
                std::vector<float> speaker_scratch_;
                std::vector<float> ear_scratch_;
                std::vector<Speaker> speakers_;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
