/**************************************************************************/
/* procedural.hpp                                                         */
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

#ifndef SUSHIENGINE_AUDIO_PROCEDURAL_HPP
#define SUSHIENGINE_AUDIO_PROCEDURAL_HPP

/**
 * @file procedural.hpp
 * @brief Procedural sound sources — modal impacts and wind — as playable voices.
 *
 * The optional in-engine procedural-SFX feature of §S10 (`docs/design/audio_system.md`),
 * exposed as ordinary @ref VoiceSource objects so they route through the whole voice /
 * mixer / spatializer / reverb pipeline like any sampled sound, but synthesised from a
 * handful of parameters with no asset:
 *
 *   - @ref ModalImpactSource — a struck object (a knock, ping, clang) rendered by a modal
 *     resonator bank (`dsp/modal.hpp`). A one-shot: it plays until the ring decays.
 *   - @ref WindSource — continuous wind, the classic recipe: broadband **filtered noise**
 *     whose band and level track the wind speed, plus a high-Q **Aeolian tone** at the
 *     Strouhal frequency `f = St·v/d` (the whistle air makes past an edge), which rises
 *     and sharpens as the wind picks up.
 *
 * Portable `float` DSP, no SDL and no SushiRuntime. The noise is a deterministic xorshift,
 * so a given seed renders the same wind every run (useful for tests and replays).
 */

#include <cmath>
#include <cstdint>

#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/dsp/modal.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief A one-shot struck-object impact synthesised by a modal resonator bank.
         *
         * Construct with a material and base pitch; it strikes on @ref prepare (and on
         * @ref reset), then rings until decayed, at which point @ref render returns false
         * and the voice manager frees it. @ref restrike re-excites a held voice.
         */
        class ModalImpactSource final : public VoiceSource
        {
            public:
                /**
                 * @brief Builds an impact source.
                 * @param material 0 wood, 1 metal, 2 glass, 3 membrane (see @ref DSP::ModalResonatorBank::set_material).
                 * @param base_hz  The fundamental the material's partials scale from.
                 * @param energy   The strike strength (linear).
                 */
                ModalImpactSource(int material, float base_hz, float energy) noexcept
                    : material_(material), base_hz_(base_hz), energy_(energy)
                {
                }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    bank_.set_material(material_, base_hz_);
                    bank_.prepare(sample_rate);
                    bank_.reset();
                    bank_.strike(energy_);
                }

                void reset() noexcept override
                {
                    bank_.reset();
                    bank_.strike(energy_);
                }

                /** @brief Re-excites the bank (a repeated hit on the same object). */
                void restrike(float energy) noexcept { bank_.strike(energy); }

                /** @brief The modal bank, e.g. to author its modes directly. */
                DSP::ModalResonatorBank& bank() noexcept { return bank_; }

                bool render(float* out, int frame_count) noexcept override
                {
                    bank_.process_block(out, frame_count);
                    return bank_.is_ringing();
                }

                bool advance(int frame_count) noexcept override
                {
                    // A virtualised impact keeps decaying silently so it returns at the
                    // right amplitude; render into a small local sink in chunks.
                    float sink[128];
                    int remaining = frame_count;
                    while (remaining > 0)
                    {
                        const int n = remaining < 128 ? remaining : 128;
                        bank_.process_block(sink, n);
                        remaining -= n;
                    }
                    return bank_.is_ringing();
                }

            private:
                DSP::ModalResonatorBank bank_;
                int material_;
                float base_hz_;
                float energy_;
        };

        /**
         * @brief Continuous wind: speed-driven filtered noise plus an Aeolian tone.
         *
         * @ref set_speed (0 = calm .. 1 = gale) drives both layers each control tick: the
         * broadband band-pass opens brighter and louder, and the Strouhal whistle rises in
         * pitch, sharpens (higher Q), and grows. Endless — @ref render always returns true.
         */
        class WindSource final : public VoiceSource
        {
            public:
                /**
                 * @brief Builds a wind source.
                 * @param seed        The noise seed (same seed → same wind).
                 * @param edge_size_m The obstacle size feeding the Strouhal pitch `f = St·v/d`.
                 */
                explicit WindSource(std::uint32_t seed = 0x1234567u, float edge_size_m = 0.05f) noexcept
                    : rng_(seed ? seed : 1u), edge_size_(edge_size_m > 1.0e-3f ? edge_size_m : 1.0e-3f)
                {
                }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                    body_.reset();
                    whistle_.reset();
                    update_filters();
                }

                void reset() noexcept override
                {
                    body_.reset();
                    whistle_.reset();
                }

                /**
                 * @brief Sets the wind speed (control tick).
                 * @param normalized_speed 0 (calm) .. 1 (gale); clamped.
                 */
                void set_speed(float normalized_speed) noexcept
                {
                    speed_ = normalized_speed < 0.0f ? 0.0f : (normalized_speed > 1.0f ? 1.0f : normalized_speed);
                    if (sample_rate_ > 0.0)
                        update_filters();
                }

                bool render(float* out, int frame_count) noexcept override
                {
                    // Broadband body: white noise through a widening, rising band-pass whose
                    // level grows with the square of speed (gustier winds are much louder).
                    const float body_gain = 0.15f + 0.85f * speed_ * speed_;
                    // The Aeolian whistle sits on top, quiet until the wind is up.
                    const float whistle_gain = 0.25f * speed_ * speed_ * speed_;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        const float n = noise();
                        const float body = body_.process(n) * body_gain;
                        const float whistle = whistle_.process(n) * whistle_gain;
                        out[i] = body + whistle;
                    }
                    return true;
                }

                bool advance(int frame_count) noexcept override
                {
                    // Keep the noise stream and filter state advancing so the texture does
                    // not restart when the voice returns from virtual.
                    for (int i = 0; i < frame_count; ++i)
                    {
                        const float n = noise();
                        body_.process(n);
                        whistle_.process(n);
                    }
                    return true;
                }

            private:
                float noise() noexcept
                {
                    // xorshift32 → white noise in [-1, 1].
                    rng_ ^= rng_ << 13;
                    rng_ ^= rng_ >> 17;
                    rng_ ^= rng_ << 5;
                    return static_cast<float>(static_cast<std::int32_t>(rng_)) *
                           (1.0f / 2147483648.0f);
                }

                void update_filters() noexcept
                {
                    // Body: 300 Hz (calm) → 1800 Hz (gale), widening (lower Q gives a broader
                    // rushing band) as it opens up.
                    const double body_hz = 300.0 + 1500.0 * speed_;
                    const double body_q = 0.9 - 0.4 * speed_;
                    body_.set_band_pass(body_hz, body_q, sample_rate_);

                    // Aeolian tone: f = St · v / d. Map the normalized speed to a physical
                    // wind speed (~0..25 m/s), St ≈ 0.2. Clamp below Nyquist.
                    const double wind_ms = 25.0 * speed_;
                    const double strouhal = 0.2;
                    double f = strouhal * wind_ms / edge_size_;
                    const double max_f = sample_rate_ * 0.45;
                    if (f < 50.0) f = 50.0;
                    if (f > max_f) f = max_f;
                    const double whistle_q = 6.0 + 18.0 * speed_; // sharpens with speed
                    whistle_.set_band_pass(f, whistle_q, sample_rate_);
                }

                DSP::Biquad body_;
                DSP::Biquad whistle_;
                std::uint32_t rng_;
                float edge_size_;
                float speed_ = 0.0f;
                double sample_rate_ = 48000.0;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
