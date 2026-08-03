/**************************************************************************/
/* occlusion.hpp                                                          */
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

#ifndef SUSHIENGINE_AUDIO_OCCLUSION_HPP
#define SUSHIENGINE_AUDIO_OCCLUSION_HPP

/**
 * @file occlusion.hpp
 * @brief The per-voice occlusion DSP: two blockage scalars + three-band transmission
 *        into a muffled, attenuated dry signal and a reduced reverb send.
 *
 * This is the render half of §6 of `docs/slop/audio_system.md`. The geometry layer
 * (`acoustic_geometry.hpp`) computes *what* is blocked; this turns that into *sound*.
 * The design carries two author-or-geometry scalars per emitter, routed to separate DSP:
 *
 *   - **obstruction** — the direct path is blocked but the reverberant field is not (a
 *     pillar between you and the source). Degrades the **dry** signal only: a level drop
 *     and a low-pass (sound bends round the obstacle, losing highs), while the reverb send
 *     is untouched.
 *   - **occlusion** — both the direct and the reverberant path are blocked (a solid wall).
 *     Degrades **dry + wet**: the dry attenuation of obstruction, *and* it pulls the
 *     reverb send down too (@ref wet_scale), because the room behind the wall no longer
 *     reaches you.
 *
 * The **three-band transmission** (from the materials a blocked path pierced) colours the
 * dry signal: because walls pass lows far more readily than highs, a blocked source turns
 * bassy. Applied as two shelves around the mid band plus a mid-band trim.
 *
 * Every driver is slewed by a @ref SmoothedValue — the design's **edge-diffraction
 * coefficient**, not the raw ray-block percentage — so a source slipping behind cover
 * fades and darkens smoothly instead of clicking. Coefficients are recomputed per block
 * from the slewed drivers (cheap, and the drivers move slowly). Portable `float` DSP, no
 * SDL and no SushiRuntime.
 */

#include <cmath>

#include <SushiEngine/audio/acoustic_material.hpp>
#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/parameter.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief Tunable curves mapping the blockage scalars to dry level and cutoff. */
        struct OcclusionCurves
        {
            float slew_seconds = 0.03f;    /**< Edge-diffraction slew of every driver (~30 ms). */
            float low_shelf_hz = 700.0f;   /**< Crossover below which the low transmission band acts. */
            float high_shelf_hz = 3500.0f; /**< Crossover above which the high transmission band acts. */
            float open_cutoff_hz = 20000.0f; /**< Dry low-pass cutoff when fully unblocked. */
            float blocked_cutoff_hz = 700.0f; /**< Dry low-pass cutoff when fully blocked (edge diffraction). */
            float transmission_floor = 1.0e-4f; /**< Smallest band gain, so shelves stay finite (−80 dB). */
        };

        /**
         * @brief The per-voice occlusion processor: obstruction/occlusion → dry DSP + wet scale.
         *
         * One instance per spatial voice, alongside @ref SourcePropagation. Call
         * @ref set_targets each control frame with the geometry-derived scalars, then
         * @ref process one block: it filters the dry buffer in place and returns the
         * reverb-send scale to apply this block.
         */
        class OcclusionFilter
        {
            public:
                /**
                 * @brief Allocates state for a run.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block processed (unused; kept for symmetry).
                 */
                void prepare(double sample_rate, int max_block_frames) noexcept
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                    const double s = curves_.slew_seconds;
                    dry_blockage_.configure(s, sample_rate);
                    occlusion_.configure(s, sample_rate);
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        transmission_[b].configure(s, sample_rate);
                    reset();
                }

                /** @brief Clears filter state and snaps every driver to fully open. */
                void reset() noexcept
                {
                    dry_blockage_.snap(0.0f);
                    occlusion_.snap(0.0f);
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        transmission_[b].snap(1.0f);
                    low_pass_.reset();
                    low_shelf_.reset();
                    high_shelf_.reset();
                }

                /** @brief Replaces the mapping curves (call before @ref prepare). */
                void set_curves(const OcclusionCurves& curves) noexcept { curves_ = curves; }

                /**
                 * @brief Publishes the blockage state for the source (control frame).
                 * @param obstruction  Direct-path blockage in [0, 1] (dry only).
                 * @param occlusion    Direct+reverb blockage in [0, 1] (dry and wet).
                 * @param transmission Three-band leak of the blocked path (1 = fully open).
                 */
                void set_targets(float obstruction, float occlusion,
                                 const float transmission[ACOUSTIC_BAND_COUNT]) noexcept
                {
                    obstruction = clamp01(obstruction);
                    occlusion = clamp01(occlusion);
                    // Both block the direct path; a wall (occlusion) implies at least as
                    // much direct blockage as a pillar (obstruction).
                    const float direct = obstruction > occlusion ? obstruction : occlusion;
                    dry_blockage_.set_target(direct);
                    occlusion_.set_target(occlusion);
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        transmission_[b].set_target(clampf(transmission[b], curves_.transmission_floor, 1.0f));
                }

                /**
                 * @brief Filters one dry block in place and returns the reverb-send scale.
                 *
                 * The dry signal is coloured by the three-band transmission (as two shelves
                 * around a mid trim), low-passed by the edge-diffraction cutoff, and level-
                 * scaled — all from the slewed drivers. The returned scale (1 − occlusion)
                 * is what the caller multiplies its reverb aux-send by, so a wall mutes the
                 * room while a pillar leaves it ringing.
                 *
                 * @param block       The dry mono block, filtered in place.
                 * @param frame_count Number of samples.
                 * @return The reverb-send scale in [0, 1] for this block.
                 */
                float process(float* block, int frame_count) noexcept
                {
                    const float direct = dry_blockage_.advance_block(frame_count);
                    const float occlusion = occlusion_.advance_block(frame_count);
                    float trans[ACOUSTIC_BAND_COUNT];
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        trans[b] = transmission_[b].advance_block(frame_count);

                    // Per-band dry gain: the unblocked fraction passes at full, the blocked
                    // fraction passes at the material's transmission — so a partly-covered
                    // source stays present, a fully-walled one drops to its leak.
                    float band_gain[ACOUSTIC_BAND_COUNT];
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        band_gain[b] = (1.0f - direct) + direct * trans[b];

                    const float mid = band_gain[1] > 1.0e-6f ? band_gain[1] : 1.0e-6f;
                    const float low_db = clampf(linear_to_db(band_gain[0] / mid), -60.0f, 24.0f);
                    const float high_db = clampf(linear_to_db(band_gain[2] / mid), -60.0f, 24.0f);

                    const float cutoff =
                        curves_.open_cutoff_hz * (1.0f - direct) + curves_.blocked_cutoff_hz * direct;
                    const double nyq = sample_rate_ * 0.49;
                    low_pass_.set_low_pass(cutoff < nyq ? cutoff : nyq, 0.70710678, sample_rate_);
                    low_shelf_.set_low_shelf(curves_.low_shelf_hz, low_db, sample_rate_);
                    high_shelf_.set_high_shelf(curves_.high_shelf_hz, high_db, sample_rate_);

                    for (int i = 0; i < frame_count; ++i)
                    {
                        float v = block[i];
                        v = low_pass_.process(v);
                        v = low_shelf_.process(v);
                        v = high_shelf_.process(v);
                        block[i] = v * mid;
                    }

                    return 1.0f - occlusion;
                }

                /** @brief The current (slewed) reverb-send scale, without advancing. */
                float wet_scale() const noexcept { return 1.0f - occlusion_.current(); }

            private:
                static float clamp01(float v) noexcept { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }
                static float clampf(float v, float lo, float hi) noexcept
                {
                    return v < lo ? lo : (v > hi ? hi : v);
                }
                static float linear_to_db(float g) noexcept
                {
                    return 20.0f * std::log10(g > 1.0e-6f ? g : 1.0e-6f);
                }

                OcclusionCurves curves_;
                SmoothedValue dry_blockage_;
                SmoothedValue occlusion_;
                SmoothedValue transmission_[ACOUSTIC_BAND_COUNT];
                DSP::Biquad low_pass_;
                DSP::Biquad low_shelf_;
                DSP::Biquad high_shelf_;
                double sample_rate_ = 48000.0;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
