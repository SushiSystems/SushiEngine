/**************************************************************************/
/* modal.hpp                                                              */
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

#ifndef SUSHIENGINE_AUDIO_DSP_MODAL_HPP
#define SUSHIENGINE_AUDIO_DSP_MODAL_HPP

/**
 * @file modal.hpp
 * @brief Modal synthesis — a bank of damped resonators struck by an impulse.
 *
 * A struck or scraped object rings at a set of natural frequencies (its **modes**), each
 * a decaying sinusoid whose pitch, decay, and gain are fixed by the object's shape and
 * material (§S10 / §3 of `docs/design/audio_system.md`). Modal synthesis realises exactly
 * that: each mode is a two-pole resonator `H(z) = 1 / (1 − 2r·cosω·z⁻¹ + r²·z⁻²)`, whose
 * impulse response is `rⁿ·sin(ω(n+1))/sinω` — a sinusoid at ω that decays at a rate set
 * by the pole radius `r = exp(−1/(τ·fs))`. Strike it with an impulse and it *rings*;
 * strike it harder (more energy) and it rings louder. A handful of modes gives a
 * convincing, physically-parameterised impact — a wood knock, a glass ping, a metal
 * clang — for a few multiply-adds per sample, no sample data.
 *
 * Portable `float` DSP, no SDL and no SushiRuntime; deterministic (no RNG), so a strike
 * is bit-reproducible. Coefficients are computed in `double`, stored `float`.
 */

#include <cmath>
#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        namespace DSP
        {
            /** @brief One resonant mode: a frequency, a decay time, and a gain. */
            struct ModalMode
            {
                float frequency_hz = 440.0f; /**< Modal (partial) frequency. */
                float decay_seconds = 1.0f;  /**< −60 dB decay time (T60) of this mode. */
                float amplitude = 1.0f;      /**< Relative excitation gain of this mode. */
            };

            /**
             * @brief A bank of two-pole modal resonators summed to one output.
             *
             * Add modes (or a material preset), @ref prepare for a sample rate, then
             * @ref strike to excite and @ref process_block to render the ring. Real
             * voices call it per block; it is silent once every mode has decayed
             * (@ref is_ringing goes false), which lets a one-shot free itself.
             */
            class ModalResonatorBank
            {
                public:
                    /** @brief Removes every mode. */
                    void clear() noexcept { modes_.clear(); }

                    /** @brief Appends one mode. */
                    void add_mode(const ModalMode& mode) { modes_.push_back(Resonator{mode}); }

                    /** @brief The number of modes. */
                    std::size_t mode_count() const noexcept { return modes_.size(); }

                    /**
                     * @brief Computes each mode's resonator coefficients for a sample rate.
                     * @param sample_rate The stream sample rate in Hz.
                     */
                    void prepare(double sample_rate)
                    {
                        sample_rate_ = sample_rate;
                        for (Resonator& m : modes_)
                        {
                            const double w = 2.0 * 3.14159265358979 * m.mode.frequency_hz / sample_rate;
                            // Pole radius from T60: r^(T60·fs) = 10^(-3) → r = 10^(-3/(T60·fs)).
                            const double t60_samples =
                                static_cast<double>(m.mode.decay_seconds) * sample_rate;
                            const double r = t60_samples > 1.0
                                                 ? std::pow(10.0, -3.0 / t60_samples)
                                                 : 0.0;
                            m.a1 = static_cast<float>(2.0 * r * std::cos(w));
                            m.a2 = static_cast<float>(-r * r);
                            // Impulse response is b0·rⁿ·sin((n+1)ω)/sinω; taking b0 = amp·sinω
                            // makes the ring peak at ≈ amp regardless of the decay time (the
                            // (1−r) normalisation of a resonant *filter* would wrongly silence
                            // long, high-Q modes).
                            m.b0 = static_cast<float>(m.mode.amplitude * std::sin(w));
                            m.z1 = 0.0f;
                            m.z2 = 0.0f;
                        }
                    }

                    /** @brief Clears every resonator's state (silences the ring). */
                    void reset() noexcept
                    {
                        for (Resonator& m : modes_)
                        {
                            m.z1 = 0.0f;
                            m.z2 = 0.0f;
                        }
                    }

                    /**
                     * @brief Excites the bank with an impulse of the given energy.
                     * @param energy The strike strength (linear); scales the impulse.
                     */
                    void strike(float energy) noexcept { pending_impulse_ += energy; }

                    /**
                     * @brief Renders one block, summing every mode's ring.
                     * @param out         The mono output block.
                     * @param frame_count Number of samples.
                     */
                    void process_block(float* out, int frame_count) noexcept
                    {
                        for (int i = 0; i < frame_count; ++i)
                            out[i] = 0.0f;
                        for (Resonator& m : modes_)
                        {
                            float z1 = m.z1;
                            float z2 = m.z2;
                            const float a1 = m.a1, a2 = m.a2, b0 = m.b0;
                            // The impulse hits only the first sample of the block.
                            float x = pending_impulse_;
                            for (int i = 0; i < frame_count; ++i)
                            {
                                const float y = b0 * x + a1 * z1 + a2 * z2;
                                z2 = z1;
                                z1 = y;
                                out[i] += y;
                                x = 0.0f;
                            }
                            m.z1 = z1;
                            m.z2 = z2;
                        }
                        pending_impulse_ = 0.0f;
                    }

                    /**
                     * @brief Whether any mode still carries audible energy.
                     * @param threshold The magnitude below which a mode counts as silent.
                     * @return True if the bank is still ringing.
                     */
                    bool is_ringing(float threshold = 1.0e-5f) const noexcept
                    {
                        if (pending_impulse_ != 0.0f)
                            return true;
                        for (const Resonator& m : modes_)
                            if (std::fabs(m.z1) > threshold || std::fabs(m.z2) > threshold)
                                return true;
                        return false;
                    }

                    /**
                     * @brief Fills the bank with a material preset scaled to a base frequency.
                     *
                     * Each material is a fixed set of partial ratios, decay times, and gains
                     * — the fingerprint that makes wood sound like wood and glass like glass.
                     * The ratios are multiplied by @p base_hz, so the same material can voice
                     * a small tap or a large boom.
                     *
                     * @param material 0 wood, 1 metal, 2 glass, 3 membrane (drum-like).
                     * @param base_hz  The fundamental the partial ratios scale from.
                     */
                    void set_material(int material, float base_hz)
                    {
                        clear();
                        struct Partial { float ratio, decay, gain; };
                        switch (material)
                        {
                            case 1: // metal: inharmonic, long ring, bright
                            {
                                const Partial p[] = {{1.0f, 2.2f, 1.0f},   {2.76f, 1.8f, 0.7f},
                                                     {5.40f, 1.4f, 0.5f},  {8.93f, 1.0f, 0.35f},
                                                     {13.3f, 0.7f, 0.22f}, {18.6f, 0.5f, 0.14f}};
                                for (const Partial& q : p)
                                    add_mode(ModalMode{base_hz * q.ratio, q.decay, q.gain});
                                break;
                            }
                            case 2: // glass: high, harmonic-ish, short bright ping
                            {
                                const Partial p[] = {{1.0f, 0.8f, 1.0f},  {2.0f, 0.6f, 0.6f},
                                                     {3.01f, 0.4f, 0.4f}, {4.7f, 0.28f, 0.25f},
                                                     {6.6f, 0.18f, 0.15f}};
                                for (const Partial& q : p)
                                    add_mode(ModalMode{base_hz * q.ratio, q.decay, q.gain});
                                break;
                            }
                            case 3: // membrane: circular-drum modes, fast decay
                            {
                                const Partial p[] = {{1.0f, 0.5f, 1.0f},   {1.59f, 0.4f, 0.7f},
                                                     {2.14f, 0.32f, 0.5f}, {2.30f, 0.3f, 0.45f},
                                                     {2.65f, 0.25f, 0.35f}};
                                for (const Partial& q : p)
                                    add_mode(ModalMode{base_hz * q.ratio, q.decay, q.gain});
                                break;
                            }
                            default: // wood: quasi-harmonic, quick warm knock
                            {
                                const Partial p[] = {{1.0f, 0.35f, 1.0f},  {2.05f, 0.25f, 0.5f},
                                                     {3.1f, 0.18f, 0.3f},  {4.2f, 0.12f, 0.18f}};
                                for (const Partial& q : p)
                                    add_mode(ModalMode{base_hz * q.ratio, q.decay, q.gain});
                                break;
                            }
                        }
                        if (sample_rate_ > 0.0)
                            prepare(sample_rate_);
                    }

                private:
                    struct Resonator
                    {
                        ModalMode mode;
                        float a1 = 0.0f, a2 = 0.0f, b0 = 0.0f;
                        float z1 = 0.0f, z2 = 0.0f;
                    };

                    std::vector<Resonator> modes_;
                    double sample_rate_ = 0.0;
                    float pending_impulse_ = 0.0f;
            };
        } // namespace DSP
    } // namespace Audio
} // namespace SushiEngine

#endif
