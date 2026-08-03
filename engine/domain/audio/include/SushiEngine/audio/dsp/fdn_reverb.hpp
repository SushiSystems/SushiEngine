/**************************************************************************/
/* fdn_reverb.hpp                                                         */
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

#ifndef SUSHIENGINE_AUDIO_DSP_FDN_REVERB_HPP
#define SUSHIENGINE_AUDIO_DSP_FDN_REVERB_HPP

/**
 * @file fdn_reverb.hpp
 * @brief The late-reverberation primitive — a Jot feedback delay network.
 *
 * The portable DSP core's diffuse-tail generator (§3.7 of
 * `docs/slop/audio_system.md`). `N = 16` delay lines are scattered into each other
 * every round trip by a **lossless** mixing matrix (`feedback_matrix.hpp`); the *only*
 * loss in the loop is a **per-line one-pole damping filter**, which is what turns
 * "decay time" and "how much faster the highs decay" into two clean physical knobs
 * (Jot & Chaigne 1991). Mixing sets echo density; damping sets RT60 — decoupled, the
 * property that makes the reverb tunable rather than a bag of magic coefficients.
 *
 * The signal chain, all pre-allocated in @ref prepare and alloc/lock/syscall-free in
 * @ref process:
 *
 * @code
 *   in ─▶ [predelay] ─▶ [4× Schroeder allpass diffusers] ─▶ ┌─ Σ into N lines
 *                                                            │
 *          ┌───────────────────────────────────────────────┘
 *          ▼
 *     N delay lines (coprime prime lengths, slow-modulated read) ─▶ per-line damping
 *          ▲                                                              │
 *          └────────────── lossless feedback matrix ◀─────────────────────┘
 *                                                          │
 *                                        decorrelated L/R output taps ─▶ wet out
 * @endcode
 *
 * **Coprime lengths** keep the modal pile-up (flutter/metallic ringing) low; a **slow
 * per-line delay modulation** — read through the cubic-Lagrange @ref FractionalDelayLine
 * — smears the surviving modes, the standard cure for the "tin can" tail. Because the
 * matrix is orthogonal and every damping filter is strictly contractive (`|H| < 1` at
 * all frequencies for any finite RT60), the network's poles stay inside the unit circle
 * for **any** delay lengths — the FDN cannot ring forever or blow up.
 *
 * This unit knows nothing about games or I3DL2: it takes low-level @ref FDNTuning
 * (decay times, diffusion, room spread) and produces a wet stereo signal. The I3DL2
 * public parameter set, the room-geometry RT60, and the mixer wiring live one layer up
 * in `audio/reverb.hpp`.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/audio/dsp/feedback_matrix.hpp>
#include <SushiEngine/audio/dsp/fractional_delay.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        namespace DSP
        {
            /**
             * @brief Low-level tuning for @ref FDNReverb — the physical decay controls.
             *
             * These are the DSP-core knobs; the game-facing I3DL2 set (`audio/reverb.hpp`)
             * maps onto them. Times are seconds, ratios and 0..1 amounts are unitless.
             */
            struct FDNTuning
            {
                double decay_time_s = 1.6;   ///< Broadband RT60 (decay to −60 dB) at DC.
                double decay_hf_ratio = 0.5; ///< `RT60_hf / RT60_dc`; < 1 → highs decay faster.
                double predelay_s = 0.02;    ///< Gap before the tail begins (ReverbDelay).
                double diffusion = 0.85;     ///< Input allpass density, 0 (sparse) .. 1 (dense).
                double density = 0.85;       ///< Modal density → delay-length spread, 0 .. 1.
                double room_size = 0.7;      ///< Delay mean/length → apparent room size, 0 .. 1.
                double modulation_depth = 1.5; ///< Delay LFO depth in samples (0 disables).
                double modulation_rate = 1.0;  ///< Base delay-LFO rate in Hz.
                FeedbackMatrix matrix = FeedbackMatrix::Householder; ///< Lossless mixing matrix.
            };

            /**
             * @brief An order-16 Jot feedback delay network producing a wet stereo tail.
             *
             * @ref prepare once (off the audio thread) to size every buffer for the
             * largest room; @ref set_tuning any time (off-thread) to recompute delay
             * lengths and filter coefficients; @ref process on the audio thread.
             */
            class FDNReverb
            {
                public:
                    /** @brief The number of delay lines in the network. */
                    static constexpr int kLines = 16;
                    /** @brief The number of input Schroeder-allpass diffusers. */
                    static constexpr int kDiffusers = 4;

                    /**
                     * @brief Allocates all delay buffers and applies the current tuning.
                     *
                     * Lines are sized for the largest room (`room_size = 1`) plus the
                     * modulation headroom, the predelay for a generous ceiling, so
                     * @ref set_tuning can change the room later without reallocating.
                     *
                     * @param sample_rate      The stream sample rate in Hz.
                     * @param max_block_frames The largest block @ref process will be given
                     *                         (reserved for API symmetry; the loop is
                     *                         sample-accurate and needs no block scratch).
                     */
                    void prepare(double sample_rate, int max_block_frames)
                    {
                        (void)max_block_frames;
                        sample_rate_ = sample_rate > 0.0 ? sample_rate : 48000.0;

                        // Size each line for the longest prime it could ever hold: the
                        // biggest-room max delay plus modulation and interpolation headroom.
                        const int line_cap = static_cast<int>(kMaxLineMs * 0.001 * sample_rate_) + 8;
                        for (int i = 0; i < kLines; ++i)
                            lines_[i].prepare(line_cap);

                        const int predelay_cap =
                            static_cast<int>(kMaxPredelayMs * 0.001 * sample_rate_) + 8;
                        predelay_.prepare(predelay_cap);

                        // Fixed short coprime diffuser lengths (in ms), snapped to primes.
                        static const double diffuser_ms[kDiffusers] = {4.77, 6.61, 9.31, 12.13};
                        for (int i = 0; i < kDiffusers; ++i)
                        {
                            const int target = static_cast<int>(diffuser_ms[i] * 0.001 * sample_rate_);
                            diffuser_length_[i] = next_prime(target < 3 ? 3 : target);
                            diffusers_[i].prepare(diffuser_length_[i] + 4);
                        }

                        build_output_taps();
                        apply_tuning();
                        reset();
                    }

                    /**
                     * @brief Publishes new tuning and recomputes lengths/coefficients.
                     *
                     * Off-thread work (prime search, `pow`, filter design); the audio
                     * thread only ever reads the resulting coefficients. Safe to call
                     * before @ref prepare (the values are stored and applied then).
                     */
                    void set_tuning(const FDNTuning& tuning)
                    {
                        tuning_ = tuning;
                        if (sample_rate_ > 0.0)
                            apply_tuning();
                    }

                    /** @brief The current tuning. */
                    const FDNTuning& tuning() const noexcept { return tuning_; }

                    /**
                     * @brief Sets an overall linear gain on the wet output.
                     * @param gain The multiplier applied to both output channels.
                     */
                    void set_output_gain(float gain) noexcept { output_gain_ = gain; }

                    /** @brief Clears every delay line, filter, and modulation phase. */
                    void reset() noexcept
                    {
                        for (int i = 0; i < kLines; ++i)
                        {
                            lines_[i].reset();
                            damping_[i].reset();
                            lfo_phase_[i] = static_cast<float>(i) * 0.37f; // spread the phases
                        }
                        for (int i = 0; i < kDiffusers; ++i)
                            diffusers_[i].reset();
                        predelay_.reset();
                    }

                    /** @brief The prime sample length of delay line @p i (for tests/diagnostics). */
                    int line_length(int i) const noexcept { return line_length_[i]; }

                    /**
                     * @brief Renders one wet stereo block from a stereo input.
                     *
                     * Produces the reverb tail only (no dry path); the caller mixes dry
                     * and wet. Output buffers must not alias the inputs.
                     *
                     * @param in_left     Left input, @p frame_count samples.
                     * @param in_right    Right input, @p frame_count samples.
                     * @param out_left    Left wet output.
                     * @param out_right   Right wet output.
                     * @param frame_count Number of samples.
                     */
                    void process(const float* in_left, const float* in_right,
                                 float* out_left, float* out_right, int frame_count) noexcept
                    {
                        // Per-block delay-modulation ramp: evaluate the LFO at the block's
                        // start and end, then read each line's delay linearly between them
                        // (a slowly time-varying read is the tail de-metaliser).
                        float delay_cur[kLines];
                        float delay_step[kLines];
                        const float depth = static_cast<float>(tuning_.modulation_depth);
                        for (int i = 0; i < kLines; ++i)
                        {
                            const float start_mod = depth * std::sin(lfo_phase_[i]);
                            const float phase_end = lfo_phase_[i] + lfo_inc_[i] * static_cast<float>(frame_count);
                            const float end_mod = depth * std::sin(phase_end);
                            const float d0 = static_cast<float>(line_length_[i]) + start_mod;
                            const float d1 = static_cast<float>(line_length_[i]) + end_mod;
                            delay_cur[i] = d0;
                            delay_step[i] = (frame_count > 1)
                                                ? (d1 - d0) / static_cast<float>(frame_count - 1)
                                                : 0.0f;
                            lfo_phase_[i] = wrap_two_pi(phase_end);
                        }

                        const float g_ap = diffuser_gain_;

                        for (int n = 0; n < frame_count; ++n)
                        {
                            float x = 0.5f * (in_left[n] + in_right[n]);

                            // Predelay (read-before-write = a pure delay of predelay_length_).
                            const float pre = predelay_.read(static_cast<float>(predelay_length_));
                            predelay_.push(x);
                            x = pre;

                            // Input diffusion: a chain of Schroeder allpasses.
                            for (int a = 0; a < kDiffusers; ++a)
                            {
                                const float d =
                                    diffusers_[a].read(static_cast<float>(diffuser_length_[a]));
                                const float v = x + g_ap * d;
                                diffusers_[a].push(v);
                                x = d - g_ap * v;
                            }

                            // Read the network state, damp it, scatter it, feed it back.
                            float s[kLines];
                            float f[kLines];
                            for (int i = 0; i < kLines; ++i)
                            {
                                s[i] = lines_[i].read(delay_cur[i]);
                                f[i] = damping_[i].process(s[i]);
                                delay_cur[i] += delay_step[i];
                            }

                            apply_feedback_matrix(tuning_.matrix, f, kLines);

                            for (int i = 0; i < kLines; ++i)
                                lines_[i].push(input_gain_[i] * x + f[i]);

                            // Decorrelated stereo output from the (pre-damping) line outputs.
                            float yl = 0.0f, yr = 0.0f;
                            for (int i = 0; i < kLines; ++i)
                            {
                                yl += tap_left_[i] * s[i];
                                yr += tap_right_[i] * s[i];
                            }
                            out_left[n] = output_gain_ * yl;
                            out_right[n] = output_gain_ * yr;
                        }
                    }

                private:
                    /** @brief A one-pole with independent DC and Nyquist gains: `y = b0·x + a1·y'`. */
                    struct DampingFilter
                    {
                        float b0 = 1.0f;
                        float a1 = 0.0f;
                        float z = 0.0f;

                        void reset() noexcept { z = 0.0f; }

                        float process(float x) noexcept
                        {
                            z = b0 * x + a1 * z;
                            return z;
                        }
                    };

                    // Sizing ceilings (ms) that bound the pre-allocated buffers.
                    static constexpr double kMaxLineMs = 110.0;
                    static constexpr double kMaxPredelayMs = 250.0;

                    static bool is_prime(int n) noexcept
                    {
                        if (n < 2)
                            return false;
                        if (n % 2 == 0)
                            return n == 2;
                        for (int d = 3; d * d <= n; d += 2)
                            if (n % d == 0)
                                return false;
                        return true;
                    }

                    static int next_prime(int n) noexcept
                    {
                        if (n < 2)
                            return 2;
                        while (!is_prime(n))
                            ++n;
                        return n;
                    }

                    static float wrap_two_pi(float phase) noexcept
                    {
                        const float two_pi = 6.28318530717958647692f;
                        while (phase >= two_pi)
                            phase -= two_pi;
                        return phase;
                    }

                    /** @brief Fixed input/output tap gains (signs decorrelate; magnitude 1/√N). */
                    void build_output_taps() noexcept
                    {
                        const float norm = 1.0f / std::sqrt(static_cast<float>(kLines));
                        for (int i = 0; i < kLines; ++i)
                        {
                            input_gain_[i] = ((i & 1) == 0 ? +norm : -norm);
                            // Two different sign patterns → the L and R taps are decorrelated.
                            tap_left_[i] = ((i & 1) == 0 ? +norm : -norm);
                            tap_right_[i] = ((i & 2) == 0 ? +norm : -norm);
                        }
                    }

                    /** @brief Recomputes delay lengths, damping filters, diffusion, and LFO rates. */
                    void apply_tuning() noexcept
                    {
                        // Geometrically-spaced delay lengths snapped to distinct primes
                        // (distinct primes are pairwise coprime — the modal-spread goal).
                        const double room = clamp01(tuning_.room_size);
                        const double density = clamp01(tuning_.density);
                        const double mean_ms = 15.0 + 45.0 * room;          // 15..60 ms
                        const double spread = 0.25 + 0.60 * density;         // fractional width
                        const double d_min_ms = mean_ms * (1.0 - spread * 0.5);
                        const double d_max_ms = mean_ms * (1.0 + spread * 0.5);
                        const double ratio = d_max_ms / d_min_ms;

                        int previous = 2;
                        for (int i = 0; i < kLines; ++i)
                        {
                            const double t = (kLines > 1) ? static_cast<double>(i) / (kLines - 1) : 0.0;
                            const double ms = d_min_ms * std::pow(ratio, t);
                            int target = static_cast<int>(ms * 0.001 * sample_rate_);
                            if (target <= previous)
                                target = previous + 1;
                            line_length_[i] = next_prime(target);
                            previous = line_length_[i];
                        }

                        // Per-line Jot damping: solve the one-pole so its DC gain is the
                        // round-trip loss for RT60_dc and its Nyquist gain the loss for
                        // RT60_hf. Both gains are < 1, so each line is strictly contractive.
                        const double t_dc = tuning_.decay_time_s > 1e-3 ? tuning_.decay_time_s : 1e-3;
                        double t_hf = t_dc * (tuning_.decay_hf_ratio > 1e-3 ? tuning_.decay_hf_ratio : 1e-3);
                        if (t_hf < 1e-3)
                            t_hf = 1e-3;
                        for (int i = 0; i < kLines; ++i)
                        {
                            const double d = static_cast<double>(line_length_[i]);
                            const double g_dc = loop_gain(d, t_dc);
                            const double g_hf = loop_gain(d, t_hf);
                            const double a1 = (g_dc - g_hf) / (g_dc + g_hf);
                            damping_[i].b0 = static_cast<float>(g_dc * (1.0 - a1));
                            damping_[i].a1 = static_cast<float>(a1);
                        }

                        // Diffusion amount → allpass coefficient (0 .. 0.7, the stable/natural range).
                        diffuser_gain_ = static_cast<float>(0.70 * clamp01(tuning_.diffusion));

                        // Predelay in samples (clamped to the prepared ceiling).
                        int pd = static_cast<int>(tuning_.predelay_s * sample_rate_);
                        const int pd_cap = static_cast<int>(kMaxPredelayMs * 0.001 * sample_rate_);
                        if (pd < 1)
                            pd = 1;
                        if (pd > pd_cap)
                            pd = pd_cap;
                        predelay_length_ = pd;

                        // Slightly different LFO rate per line so the modulation never
                        // phase-locks across lines (each smears an independent set of modes).
                        const double base = tuning_.modulation_rate;
                        for (int i = 0; i < kLines; ++i)
                        {
                            const double rate = base * (0.7 + 0.6 * (static_cast<double>(i) / (kLines - 1)));
                            lfo_inc_[i] = static_cast<float>(6.28318530717958647692 * rate / sample_rate_);
                        }
                    }

                    /** @brief Round-trip gain of one pass through a `d`-sample line for RT60 `t`. */
                    double loop_gain(double d, double t) const noexcept
                    {
                        // g such that g^(fs·t/d) = 10^-3  →  g = 10^(−3·d / (fs·t)).
                        double g = std::pow(10.0, -3.0 * d / (sample_rate_ * t));
                        if (g < 1e-4)
                            g = 1e-4;
                        if (g > 0.9999)
                            g = 0.9999;
                        return g;
                    }

                    static double clamp01(double v) noexcept { return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v); }

                    double sample_rate_ = 0.0;
                    FDNTuning tuning_;

                    FractionalDelayLine lines_[kLines];
                    DampingFilter damping_[kLines];
                    int line_length_[kLines] = {};
                    float input_gain_[kLines] = {};
                    float tap_left_[kLines] = {};
                    float tap_right_[kLines] = {};
                    float lfo_phase_[kLines] = {};
                    float lfo_inc_[kLines] = {};

                    FractionalDelayLine diffusers_[kDiffusers];
                    int diffuser_length_[kDiffusers] = {};
                    float diffuser_gain_ = 0.0f;

                    FractionalDelayLine predelay_;
                    int predelay_length_ = 1;

                    float output_gain_ = 1.0f;
            };
        } // namespace DSP
    } // namespace Audio
} // namespace SushiEngine

#endif
