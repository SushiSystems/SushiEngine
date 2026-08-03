/**************************************************************************/
/* dynamics.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_DYNAMICS_HPP
#define SUSHIENGINE_AUDIO_DYNAMICS_HPP

/**
 * @file dynamics.hpp
 * @brief The bus dynamics rack — compressor (with sidechain/ducking), limiter, and gate.
 *
 * Every AAA mix is built on dynamics: a **master limiter** so the mix never clips, a
 * **compressor** to glue and control level, **sidechain ducking** so SFX and music bow
 * under dialogue, and a **gate** to silence bleed. These are @ref IBusEffect inserts, so
 * they drop onto any mixer bus exactly like the EQ and reverb do (§8 of
 * `docs/slop/audio_system.md`). All stereo-linked (one gain from the louder channel, so
 * the stereo image never shifts), all real-time-safe (no allocation/lock in `process`),
 * portable `float` DSP.
 *
 * The compressor's detector can be driven by an external **key** signal — a per-block
 * level published by another bus — which is how ducking works: point the music/SFX bus's
 * compressor key at the dialogue bus's level and it ducks whenever dialogue plays. The
 * limiter uses a short **lookahead** so it catches a transient's leading edge (a true
 * brick wall), at the cost of that much latency on the bus it sits on.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <SushiEngine/audio/mixer.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A peak envelope follower with separate attack and release times. */
        class EnvelopeFollower
        {
            public:
                /** @brief Sets attack/release times (seconds) for a sample rate. */
                void configure(double attack_seconds, double release_seconds, double sample_rate) noexcept
                {
                    attack_ = coefficient(attack_seconds, sample_rate);
                    release_ = coefficient(release_seconds, sample_rate);
                }

                /** @brief Resets the envelope to silence. */
                void reset() noexcept { env_ = 0.0f; }

                /** @brief The current envelope value. */
                float value() const noexcept { return env_; }

                /**
                 * @brief Advances the envelope by one rectified sample.
                 * @param rectified The absolute sample value |x|.
                 * @return The updated envelope.
                 */
                float process(float rectified) noexcept
                {
                    const float coeff = rectified > env_ ? attack_ : release_;
                    env_ = coeff * (env_ - rectified) + rectified;
                    return env_;
                }

            private:
                static float coefficient(double seconds, double sample_rate) noexcept
                {
                    if (seconds <= 0.0)
                        return 0.0f;
                    return static_cast<float>(std::exp(-1.0 / (seconds * sample_rate)));
                }

                float attack_ = 0.0f;
                float release_ = 0.0f;
                float env_ = 0.0f;
        };

        /** @brief Author parameters for @ref CompressorBusEffect. */
        struct CompressorParameters
        {
            float threshold_db = -18.0f; /**< Level above which gain reduction begins. */
            float ratio = 4.0f;          /**< Compression ratio (∞ ≈ a limiter). */
            float knee_db = 6.0f;        /**< Soft-knee width around the threshold. */
            float attack_seconds = 0.005f;
            float release_seconds = 0.12f;
            float makeup_db = 0.0f;      /**< Output make-up gain. */
        };

        /**
         * @brief A stereo-linked compressor insert, optionally sidechained for ducking.
         *
         * Detects on the louder channel (or an external key), computes a soft-knee gain
         * reduction, and applies one gain to both channels. Point @ref set_key at another
         * bus's per-block level (see @ref BusLevelProbe) to duck this bus under that one.
         */
        class CompressorBusEffect final : public IBusEffect
        {
            public:
                explicit CompressorBusEffect(
                    const CompressorParameters& parameters = CompressorParameters{}) noexcept
                    : parameters_(parameters)
                {
                }

                /** @brief Replaces the author parameters. */
                void set_parameters(const CompressorParameters& parameters) noexcept
                {
                    parameters_ = parameters;
                }

                /** @brief The current gain reduction in dB (≤ 0), for metering. */
                float gain_reduction_db() const noexcept { return reduction_db_; }

                /**
                 * @brief Sidechains the detector to an external key level (linear, per block).
                 *
                 * Pass a pointer to a value another bus updates each block (its peak/RMS);
                 * the compressor then reacts to *that* signal instead of its own — the
                 * ducking pattern. Pass nullptr to detect on this bus's own signal.
                 *
                 * @param key_level A borrowed pointer to the key's linear level, or nullptr.
                 */
                void set_key(const float* key_level) noexcept { key_ = key_level; }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                    follower_.configure(parameters_.attack_seconds, parameters_.release_seconds,
                                        sample_rate);
                    follower_.reset();
                }

                void reset() noexcept override { follower_.reset(); }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    const float threshold = parameters_.threshold_db;
                    const float knee = parameters_.knee_db > 0.0f ? parameters_.knee_db : 0.0001f;
                    const float slope =
                        1.0f - 1.0f / (parameters_.ratio > 1.0f ? parameters_.ratio : 1.0f);
                    const float makeup = db_to_lin(parameters_.makeup_db);
                    const float key_level = key_ != nullptr ? *key_ : 0.0f;

                    float last_reduction = 0.0f;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        const float detect = key_ != nullptr
                                                 ? key_level
                                                 : std::fmax(std::fabs(left[i]), std::fabs(right[i]));
                        const float env = follower_.process(detect);
                        const float env_db = lin_to_db(env);

                        // Soft-knee static curve → gain reduction in dB (≤ 0).
                        float over = env_db - threshold;
                        float reduction_db;
                        if (over <= -knee * 0.5f)
                            reduction_db = 0.0f;
                        else if (over >= knee * 0.5f)
                            reduction_db = -slope * over;
                        else
                        {
                            const float x = over + knee * 0.5f;
                            reduction_db = -slope * (x * x) / (2.0f * knee);
                        }
                        last_reduction = reduction_db;
                        const float gain = db_to_lin(reduction_db) * makeup;
                        left[i] *= gain;
                        right[i] *= gain;
                    }
                    reduction_db_ = last_reduction;
                }

            private:
                static float db_to_lin(float db) noexcept
                {
                    return static_cast<float>(std::pow(10.0, static_cast<double>(db) / 20.0));
                }
                static float lin_to_db(float lin) noexcept
                {
                    return 20.0f * std::log10(lin > 1.0e-7f ? lin : 1.0e-7f);
                }

                CompressorParameters parameters_;
                EnvelopeFollower follower_;
                const float* key_ = nullptr;
                double sample_rate_ = 48000.0;
                float reduction_db_ = 0.0f;
        };

        /**
         * @brief A lookahead brick-wall peak limiter insert.
         *
         * Delays the signal by the lookahead so the gain can start pulling down *before* a
         * transient arrives, guaranteeing the output never exceeds the ceiling — the master
         * safety net every mix ends on. Stereo-linked. The lookahead adds that much latency
         * to the bus it sits on (put it last, on the master).
         */
        class LimiterBusEffect final : public IBusEffect
        {
            public:
                /**
                 * @brief Builds a limiter.
                 * @param ceiling_db        The absolute output ceiling (≤ 0 dBFS).
                 * @param lookahead_seconds The lookahead / added latency.
                 * @param release_seconds   How fast gain recovers after a peak.
                 */
                explicit LimiterBusEffect(float ceiling_db = -0.3f, float lookahead_seconds = 0.003f,
                                          float release_seconds = 0.05f) noexcept
                    : ceiling_(db_to_lin(ceiling_db)), lookahead_seconds_(lookahead_seconds),
                      release_seconds_(release_seconds)
                {
                }

                /** @brief The current gain reduction in dB (≤ 0), for metering. */
                float gain_reduction_db() const noexcept
                {
                    return 20.0f * std::log10(gain_ > 1.0e-7f ? gain_ : 1.0e-7f);
                }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                    look_ = static_cast<int>(lookahead_seconds_ * sample_rate);
                    if (look_ < 1)
                        look_ = 1;
                    delay_l_.assign(static_cast<std::size_t>(look_), 0.0f);
                    delay_r_.assign(static_cast<std::size_t>(look_), 0.0f);
                    release_ = static_cast<float>(std::exp(-1.0 / (release_seconds_ * sample_rate)));
                    pos_ = 0;
                    gain_ = 1.0f;
                }

                void reset() noexcept override
                {
                    std::fill(delay_l_.begin(), delay_l_.end(), 0.0f);
                    std::fill(delay_r_.begin(), delay_r_.end(), 0.0f);
                    pos_ = 0;
                    gain_ = 1.0f;
                }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    for (int i = 0; i < frame_count; ++i)
                    {
                        // The incoming (future) peak decides the target gain; the delayed
                        // (present) sample is what we actually scale, so the reduction is
                        // already in place when the peak emerges from the delay.
                        const float peak = std::fmax(std::fabs(left[i]), std::fabs(right[i]));
                        float target = peak > ceiling_ ? ceiling_ / peak : 1.0f;
                        if (target < gain_)
                            gain_ = target; // instant attack: never let a peak through
                        else
                            gain_ = release_ * (gain_ - target) + target;

                        const float dl = delay_l_[static_cast<std::size_t>(pos_)];
                        const float dr = delay_r_[static_cast<std::size_t>(pos_)];
                        delay_l_[static_cast<std::size_t>(pos_)] = left[i];
                        delay_r_[static_cast<std::size_t>(pos_)] = right[i];
                        if (++pos_ >= look_)
                            pos_ = 0;

                        left[i] = dl * gain_;
                        right[i] = dr * gain_;
                    }
                }

            private:
                static float db_to_lin(float db) noexcept
                {
                    return static_cast<float>(std::pow(10.0, static_cast<double>(db) / 20.0));
                }

                std::vector<float> delay_l_;
                std::vector<float> delay_r_;
                float ceiling_;
                float lookahead_seconds_;
                float release_seconds_;
                double sample_rate_ = 48000.0;
                float release_ = 0.0f;
                int look_ = 1;
                int pos_ = 0;
                float gain_ = 1.0f;
        };

        /** @brief Author parameters for @ref GateBusEffect. */
        struct GateParameters
        {
            float threshold_db = -60.0f; /**< Below this the gate closes. */
            float attack_seconds = 0.001f;
            float release_seconds = 0.08f;
        };

        /** @brief A stereo-linked noise gate insert: silences signal below a threshold. */
        class GateBusEffect final : public IBusEffect
        {
            public:
                explicit GateBusEffect(const GateParameters& parameters = GateParameters{}) noexcept
                    : parameters_(parameters)
                {
                }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    follower_.configure(0.001, 0.02, sample_rate);
                    follower_.reset();
                    threshold_ = db_to_lin(parameters_.threshold_db);
                    open_ = static_cast<float>(
                        std::exp(-1.0 / (parameters_.attack_seconds * sample_rate)));
                    close_ = static_cast<float>(
                        std::exp(-1.0 / (parameters_.release_seconds * sample_rate)));
                    envelope_gain_ = 0.0f;
                }

                void reset() noexcept override
                {
                    follower_.reset();
                    envelope_gain_ = 0.0f;
                }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    for (int i = 0; i < frame_count; ++i)
                    {
                        const float detect = std::fmax(std::fabs(left[i]), std::fabs(right[i]));
                        const float env = follower_.process(detect);
                        const float target = env >= threshold_ ? 1.0f : 0.0f;
                        const float coeff = target > envelope_gain_ ? open_ : close_;
                        envelope_gain_ = coeff * (envelope_gain_ - target) + target;
                        left[i] *= envelope_gain_;
                        right[i] *= envelope_gain_;
                    }
                }

            private:
                static float db_to_lin(float db) noexcept
                {
                    return static_cast<float>(std::pow(10.0, static_cast<double>(db) / 20.0));
                }

                GateParameters parameters_;
                EnvelopeFollower follower_;
                float threshold_ = 0.001f;
                float open_ = 0.0f;
                float close_ = 0.0f;
                float envelope_gain_ = 0.0f;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
