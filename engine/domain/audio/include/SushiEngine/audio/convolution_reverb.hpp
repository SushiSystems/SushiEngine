/**************************************************************************/
/* convolution_reverb.hpp                                                */
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

#ifndef SUSHIENGINE_AUDIO_CONVOLUTION_REVERB_HPP
#define SUSHIENGINE_AUDIO_CONVOLUTION_REVERB_HPP

/**
 * @file convolution_reverb.hpp
 * @brief A convolution reverb behind the `IReverb` seam (§7, §S10).
 *
 * The interchangeable alternative to the Jot FDN (`reverb.hpp`): where the FDN *models* a
 * decay, this *convolves* the signal with a room impulse response, so it reproduces a
 * specific space exactly — the design's "signature static spaces" path. It plugs into the
 * same @ref IReverb seam and the same per-zone aux bus, so a `ReverbBusEffect` can host
 * either with no other change.
 *
 * With no IR file loaded, the reverb **synthesises** one from the I3DL2 parameters — the
 * standard exponentially-decaying, HF-damped, decorrelated-per-channel noise burst, with a
 * predelay — so it is immediately usable and driven by the same vocabulary as the FDN. The
 * heavy lifting is the partitioned (UPOLS) convolver (`dsp/convolution.hpp`); this layer
 * builds the IR, buffers arbitrary device blocks to the convolver's fixed block, and mixes
 * wet against a delay-aligned dry. Portable `float` DSP, no SDL and no SushiRuntime.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/audio/dsp/convolution.hpp>
#include <SushiEngine/audio/reverb.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief An @ref IReverb that convolves the signal with a (synthesised) room IR.
         *
         * @ref prepare sizes the convolver's block to the graph's max block; @ref set_params
         * (re)builds the impulse response from the I3DL2 set. @ref process convolves a stereo
         * block in place, mixing the wet return over a block-aligned dry per WetDryMix.
         */
        class ConvolutionReverb final : public IReverb
        {
            public:
                void prepare(double sample_rate, int max_block_frames) override
                {
                    sample_rate_ = sample_rate;
                    block_ = next_pow2(max_block_frames < 1 ? 1 : max_block_frames);
                    const std::size_t cap = static_cast<std::size_t>(block_ * 4);
                    for (int c = 0; c < 2; ++c)
                    {
                        in_fifo_[c].assign(cap, 0.0f);
                        out_fifo_[c].assign(cap, 0.0f);
                        dry_delay_[c].assign(static_cast<std::size_t>(block_), 0.0f);
                        in_r_[c] = in_w_[c] = in_count_[c] = 0;
                        out_r_[c] = out_w_[c] = out_count_[c] = 0;
                        dry_pos_[c] = 0;
                    }
                    block_in_[0].assign(static_cast<std::size_t>(block_), 0.0f);
                    block_in_[1].assign(static_cast<std::size_t>(block_), 0.0f);
                    block_out_[0].assign(static_cast<std::size_t>(block_), 0.0f);
                    block_out_[1].assign(static_cast<std::size_t>(block_), 0.0f);
                    apply_params();
                }

                void reset() noexcept override
                {
                    for (int c = 0; c < 2; ++c)
                    {
                        convolver_[c].reset();
                        std::fill(in_fifo_[c].begin(), in_fifo_[c].end(), 0.0f);
                        std::fill(out_fifo_[c].begin(), out_fifo_[c].end(), 0.0f);
                        std::fill(dry_delay_[c].begin(), dry_delay_[c].end(), 0.0f);
                        in_r_[c] = in_w_[c] = in_count_[c] = 0;
                        out_r_[c] = out_w_[c] = out_count_[c] = 0;
                        dry_pos_[c] = 0;
                    }
                    // Prime the output FIFO with one block of silence so wet and the
                    // block-delayed dry stay aligned from the first sample.
                    for (int c = 0; c < 2; ++c)
                        for (int i = 0; i < block_; ++i)
                            push(out_fifo_[c], out_w_[c], out_count_[c], 0.0f);
                }

                void set_params(const I3DL2Reverb& params) override
                {
                    params_ = params;
                    if (sample_rate_ > 0.0)
                        apply_params();
                }

                /** @brief The current I3DL2 parameters. */
                const I3DL2Reverb& params() const noexcept { return params_; }

                /** @brief The IR length in samples (per channel), loaded or synthesised. */
                int impulse_length() const noexcept { return ir_length_; }

                /** @brief Whether a measured impulse response is loaded (vs synthesised). */
                bool has_loaded_impulse() const noexcept { return has_loaded_ir_; }

                /**
                 * @brief Loads a measured impulse response, replacing the synthesised one.
                 *
                 * The reverb then convolves against a *real* recorded space (the reason to
                 * use convolution). The IR is linearly resampled to the device rate if
                 * needed, split to two channels (mono duplicates), and scaled by the I3DL2
                 * Room+Reverb level. Call @ref clear_impulse to return to the synthesised IR.
                 *
                 * @param data           Interleaved IR samples (mono or stereo).
                 * @param frames         Number of frames.
                 * @param channels       1 (mono) or 2 (stereo).
                 * @param ir_sample_rate The IR's sample rate (0 = the device rate).
                 */
                void load_impulse(const float* data, int frames, int channels, double ir_sample_rate)
                {
                    if (frames <= 0 || channels < 1)
                        return;
                    const int ch = channels > 2 ? 2 : channels;
                    const double ratio =
                        (ir_sample_rate > 0.0 && ir_sample_rate != sample_rate_) ? ir_sample_rate / sample_rate_
                                                                                : 1.0;
                    const int out_len = ratio == 1.0 ? frames
                                                     : static_cast<int>(frames / ratio);
                    for (int c = 0; c < 2; ++c)
                    {
                        const int src_ch = c < ch ? c : 0; // mono → both channels
                        raw_ir_[c].assign(static_cast<std::size_t>(out_len), 0.0f);
                        for (int i = 0; i < out_len; ++i)
                        {
                            const double pos = i * ratio;
                            const int i0 = static_cast<int>(pos);
                            const int i1 = i0 + 1 < frames ? i0 + 1 : frames - 1;
                            const float frac = static_cast<float>(pos - i0);
                            const float s0 = data[i0 * channels + src_ch];
                            const float s1 = data[i1 * channels + src_ch];
                            raw_ir_[c][static_cast<std::size_t>(i)] = s0 + frac * (s1 - s0);
                        }
                    }
                    raw_ir_len_ = out_len;
                    has_loaded_ir_ = true;
                    if (sample_rate_ > 0.0)
                        apply_params();
                }

                /** @brief Discards a loaded IR and returns to the synthesised one. */
                void clear_impulse()
                {
                    has_loaded_ir_ = false;
                    raw_ir_len_ = 0;
                    if (sample_rate_ > 0.0)
                        apply_params();
                }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    float* io[2] = {left, right};
                    for (int c = 0; c < 2; ++c)
                    {
                        for (int i = 0; i < frame_count; ++i)
                        {
                            push(in_fifo_[c], in_w_[c], in_count_[c], io[c][i]);

                            // Consume whole blocks as they accumulate.
                            while (in_count_[c] >= block_)
                            {
                                for (int k = 0; k < block_; ++k)
                                    block_in_[c][static_cast<std::size_t>(k)] =
                                        pop(in_fifo_[c], in_r_[c], in_count_[c]);
                                convolver_[c].process_block(block_in_[c].data(), block_out_[c].data());
                                for (int k = 0; k < block_; ++k)
                                    push(out_fifo_[c], out_w_[c], out_count_[c],
                                         block_out_[c][static_cast<std::size_t>(k)]);
                            }

                            const float wet = out_count_[c] > 0 ? pop(out_fifo_[c], out_r_[c], out_count_[c])
                                                               : 0.0f;
                            // Dry delayed by one block so it aligns with the wet's latency.
                            float& slot = dry_delay_[c][static_cast<std::size_t>(dry_pos_[c])];
                            const float dry = slot;
                            slot = io[c][i];
                            if (++dry_pos_[c] >= block_)
                                dry_pos_[c] = 0;

                            io[c][i] = dry_mix_ * dry + wet_mix_ * wet;
                        }
                    }
                }

            private:
                static int next_pow2(int n) noexcept
                {
                    int p = 1;
                    while (p < n)
                        p <<= 1;
                    return p;
                }

                static float clampf(float v, float lo, float hi) noexcept
                {
                    return v < lo ? lo : (v > hi ? hi : v);
                }

                static float db_to_linear(float db) noexcept
                {
                    return static_cast<float>(std::pow(10.0, static_cast<double>(db) / 20.0));
                }

                static void push(std::vector<float>& buf, int& w, int& count, float v) noexcept
                {
                    buf[static_cast<std::size_t>(w)] = v;
                    if (++w >= static_cast<int>(buf.size()))
                        w = 0;
                    ++count;
                }

                static float pop(std::vector<float>& buf, int& r, int& count) noexcept
                {
                    const float v = buf[static_cast<std::size_t>(r)];
                    if (++r >= static_cast<int>(buf.size()))
                        r = 0;
                    --count;
                    return v;
                }

                /** @brief Rebuilds the impulse response and both convolvers from @ref params_. */
                void apply_params()
                {
                    wet_mix_ = clampf(params_.wet_dry_mix, 0.0f, 100.0f) / 100.0f;
                    dry_mix_ = 1.0f - wet_mix_;

                    // A measured IR is used verbatim (scaled by the Room+Reverb level); only
                    // the synthesised path derives its IR from the decay parameters.
                    if (has_loaded_ir_)
                    {
                        const float level = db_to_linear(params_.room + params_.reverb);
                        ir_length_ = raw_ir_len_;
                        std::vector<float> scaled(static_cast<std::size_t>(raw_ir_len_));
                        for (int c = 0; c < 2; ++c)
                        {
                            for (int i = 0; i < raw_ir_len_; ++i)
                                scaled[static_cast<std::size_t>(i)] =
                                    raw_ir_[c][static_cast<std::size_t>(i)] * level;
                            convolver_[c].prepare(block_, scaled.data(), raw_ir_len_);
                        }
                        reset();
                        return;
                    }

                    const float decay = clampf(params_.decay_time, 0.1f, 20.0f);
                    const float hf_ratio = clampf(params_.decay_hf_ratio, 0.1f, 2.0f);
                    const int max_len = static_cast<int>(4.0 * sample_rate_); // cap the CPU
                    ir_length_ = static_cast<int>(decay * static_cast<float>(sample_rate_));
                    if (ir_length_ > max_len)
                        ir_length_ = max_len;
                    if (ir_length_ < block_)
                        ir_length_ = block_;
                    const int predelay = static_cast<int>(
                        clampf(params_.reverb_delay, 0.0f, 0.24f) * static_cast<float>(sample_rate_));

                    const float level = db_to_linear(params_.room + params_.reverb);
                    // Normalise by the RMS of a unit-energy decaying-noise burst so the wet
                    // level tracks the dB request rather than the tail length.
                    const float norm = level / std::sqrt(static_cast<float>(ir_length_) * 0.25f + 1.0f);

                    // A one-pole low-pass darkens the tail when the HF is set to decay faster.
                    const float fc = 1500.0f + 16000.0f * (hf_ratio * 0.5f);
                    const float a = std::exp(-2.0f * 3.14159265358979f * fc /
                                             static_cast<float>(sample_rate_));

                    std::vector<float> ir(static_cast<std::size_t>(ir_length_));
                    for (int c = 0; c < 2; ++c)
                    {
                        std::uint32_t rng = 0x9e3779b9u ^ (c * 0x85ebca6bu + 0x1u);
                        float lp = 0.0f;
                        for (int i = 0; i < ir_length_; ++i)
                        {
                            float sample = 0.0f;
                            if (i >= predelay)
                            {
                                rng ^= rng << 13;
                                rng ^= rng >> 17;
                                rng ^= rng << 5;
                                const float white = static_cast<float>(static_cast<std::int32_t>(rng)) *
                                                    (1.0f / 2147483648.0f);
                                const float t = static_cast<float>(i - predelay) /
                                                static_cast<float>(sample_rate_);
                                const float env = std::exp(-6.9077553f * t / decay); // −60 dB at `decay`
                                sample = white * env;
                            }
                            lp = a * lp + (1.0f - a) * sample; // HF damping
                            ir[static_cast<std::size_t>(i)] = lp * norm;
                        }
                        convolver_[c].prepare(block_, ir.data(), ir_length_);
                    }

                    wet_mix_ = clampf(params_.wet_dry_mix, 0.0f, 100.0f) / 100.0f;
                    dry_mix_ = 1.0f - wet_mix_;
                    reset();
                }

                DSP::PartitionedConvolver convolver_[2];
                I3DL2Reverb params_;
                std::vector<float> in_fifo_[2];
                std::vector<float> out_fifo_[2];
                std::vector<float> dry_delay_[2];
                std::vector<float> block_in_[2];
                std::vector<float> block_out_[2];
                int in_r_[2] = {0, 0}, in_w_[2] = {0, 0}, in_count_[2] = {0, 0};
                int out_r_[2] = {0, 0}, out_w_[2] = {0, 0}, out_count_[2] = {0, 0};
                int dry_pos_[2] = {0, 0};
                std::vector<float> raw_ir_[2]; /**< A loaded (measured) IR, unscaled, per channel. */
                int raw_ir_len_ = 0;
                bool has_loaded_ir_ = false;
                double sample_rate_ = 0.0;
                int block_ = 0;
                int ir_length_ = 0;
                float wet_mix_ = 1.0f;
                float dry_mix_ = 0.0f;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
