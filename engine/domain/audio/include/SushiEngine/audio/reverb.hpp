/**************************************************************************/
/* reverb.hpp                                                            */
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

#ifndef SUSHIENGINE_AUDIO_REVERB_HPP
#define SUSHIENGINE_AUDIO_REVERB_HPP

/**
 * @file reverb.hpp
 * @brief The game-facing reverb layer: the I3DL2 API, the @ref IReverb seam, and the
 *        room-geometry RT60 that feeds them.
 *
 * The DSP core (`dsp/fdn_reverb.hpp`) knows only decay times and delay spreads. This
 * layer is the vocabulary a sound designer and the engine actually speak (§7, §13 of
 * `docs/slop/audio_system.md`):
 *
 *   - @ref I3DL2Reverb — the de-facto **I3DL2 / EAX** parameter set (Room, RoomHF,
 *     DecayTime, DecayHFRatio, Reflections, Reverb, Diffusion, Density, …). Presets and
 *     a room-geometry factory build one; @ref FDNReverbEffect maps it onto the FDN.
 *   - @ref IReverb — the interchangeable-reverb seam. An FDN today; a convolution
 *     reverb (Gardner NUPC, for signature static spaces) later — same seam, no change
 *     at the call sites. @ref ReverbBusEffect adapts any @ref IReverb into the mixer's
 *     @ref IBusEffect so it drops onto a per-zone **aux bus** (§8).
 *   - @ref sabine_rt60 / @ref eyring_rt60 / @ref reverb_rt60 and @ref shoebox_reverb —
 *     derive the decay from geometry (Sabine below ᾱ = 0.3, Eyring above, the Resonance
 *     dual-formula recipe), so a room's reverb follows its size and materials.
 *
 * The **early reflections** of I3DL2 (Reflections / ReflectionsDelay) are the
 * image-source method over acoustic geometry — that is S7 (it needs the acoustic BVH).
 * Here those fields are carried on the struct but not yet rendered; the FDN late field,
 * its predelay (ReverbDelay), and the frequency-dependent decay are what S5 delivers.
 */

#include <cmath>
#include <memory>
#include <vector>

#include <SushiEngine/audio/dsp/fdn_reverb.hpp>
#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/reverb_params.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief The interchangeable-reverb seam (§13).
         *
         * A reverb algorithm: prepared once off the audio thread, driven by the I3DL2
         * parameter set, and processed **in place** on a stereo buffer on the audio
         * thread. FDN and (later) convolution both implement it, so the aux-bus wiring
         * never depends on which one is in use.
         */
        class IReverb
        {
            public:
                virtual ~IReverb() = default;

                /** @brief Allocates and sizes state (off the audio thread). */
                virtual void prepare(double sample_rate, int max_block_frames) = 0;

                /** @brief Clears the tail and all filter state. */
                virtual void reset() noexcept {}

                /** @brief Publishes new I3DL2 parameters (off the audio thread). */
                virtual void set_params(const I3DL2Reverb& params) = 0;

                /**
                 * @brief Processes one stereo block in place (dry in → dry+wet out).
                 * @param left        Left channel, @p frame_count samples.
                 * @param right       Right channel, @p frame_count samples.
                 * @param frame_count Number of samples.
                 */
                virtual void process(float* left, float* right, int frame_count) noexcept = 0;
        };

        /**
         * @brief An @ref IReverb backed by the Jot FDN, with the I3DL2 → FDN mapping.
         *
         * Maps the I3DL2 set onto @ref DSP::FDNReverb tuning, renders the wet tail, and
         * shapes it with a high-shelf that carries both RoomHF (the authored HF level)
         * and a gentle **Jot-style tonal correction** — a touch of HF lift that offsets
         * the extra high-frequency loss the damping filters add to a fast-decaying tail,
         * so the reverb keeps its air. WetDryMix blends the result back over the dry.
         */
        class FDNReverbEffect final : public IReverb
        {
            public:
                void prepare(double sample_rate, int max_block_frames) override
                {
                    sample_rate_ = sample_rate;
                    max_block_ = max_block_frames;
                    fdn_.prepare(sample_rate, max_block_frames);
                    wet_left_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                    wet_right_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                    apply_params();
                }

                void reset() noexcept override
                {
                    fdn_.reset();
                    shelf_left_.reset();
                    shelf_right_.reset();
                }

                void set_params(const I3DL2Reverb& params) override
                {
                    params_ = params;
                    if (sample_rate_ > 0.0)
                        apply_params();
                }

                /** @brief The current I3DL2 parameters. */
                const I3DL2Reverb& params() const noexcept { return params_; }

                /** @brief The underlying FDN (for diagnostics/tests). */
                DSP::FDNReverb& fdn() noexcept { return fdn_; }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    int n = frame_count;
                    if (n > max_block_)
                        n = max_block_;

                    fdn_.process(left, right, wet_left_.data(), wet_right_.data(), n);

                    for (int i = 0; i < n; ++i)
                        wet_left_[i] = shelf_left_.process(wet_left_[i]);
                    for (int i = 0; i < n; ++i)
                        wet_right_[i] = shelf_right_.process(wet_right_[i]);

                    for (int i = 0; i < n; ++i)
                        left[i] = dry_mix_ * left[i] + wet_mix_ * wet_left_[i];
                    for (int i = 0; i < n; ++i)
                        right[i] = dry_mix_ * right[i] + wet_mix_ * wet_right_[i];
                }

            private:
                void apply_params()
                {
                    DSP::FDNTuning t;
                    t.decay_time_s = clampf(params_.decay_time, 0.1f, 20.0f);
                    t.decay_hf_ratio = clampf(params_.decay_hf_ratio, 0.1f, 2.0f);
                    t.predelay_s = clampf(params_.reverb_delay, 0.0f, 0.24f);
                    t.diffusion = clampf(params_.diffusion, 0.0f, 100.0f) / 100.0f;
                    t.density = clampf(params_.density, 0.0f, 100.0f) / 100.0f;
                    // I3DL2 has no explicit size; a longer decay implies a bigger room, so
                    // scale the delay spread with decay time (saturating past ~3.5 s).
                    t.room_size = clampf(static_cast<float>(t.decay_time_s) / 3.5f, 0.1f, 1.0f);
                    fdn_.set_tuning(t);

                    // Wet level = Room + late Reverb level, in dB.
                    fdn_.set_output_gain(db_to_linear(params_.room + params_.reverb));

                    // The wet high-shelf: RoomHF (authored) plus a Jot tonal-correction lift
                    // that grows as the highs are set to decay faster than the body.
                    const float jot_correction_db =
                        3.0f * (1.0f - clampf(params_.decay_hf_ratio, 0.0f, 1.0f));
                    const float shelf_db = params_.room_hf + jot_correction_db;
                    const double fref = clampf(params_.hf_reference, 1000.0f,
                                               static_cast<float>(sample_rate_ * 0.49));
                    shelf_left_.set_high_shelf(fref, shelf_db, sample_rate_);
                    shelf_right_.set_high_shelf(fref, shelf_db, sample_rate_);
                    shelf_left_.reset();
                    shelf_right_.reset();

                    wet_mix_ = clampf(params_.wet_dry_mix, 0.0f, 100.0f) / 100.0f;
                    dry_mix_ = 1.0f - wet_mix_;
                }

                static float clampf(float v, float lo, float hi) noexcept
                {
                    return v < lo ? lo : (v > hi ? hi : v);
                }

                static float db_to_linear(float db) noexcept
                {
                    return static_cast<float>(std::pow(10.0, static_cast<double>(db) / 20.0));
                }

                DSP::FDNReverb fdn_;
                DSP::Biquad shelf_left_;
                DSP::Biquad shelf_right_;
                std::vector<float> wet_left_;
                std::vector<float> wet_right_;
                I3DL2Reverb params_;
                double sample_rate_ = 0.0;
                int max_block_ = 0;
                float wet_mix_ = 1.0f;
                float dry_mix_ = 0.0f;
        };

        /**
         * @brief Adapts any @ref IReverb into the mixer's @ref IBusEffect.
         *
         * Drop this as an insert on a per-zone reverb **aux bus** (§8): voices aux-send
         * into the bus, this effect turns the summed sends into the wet field, and the
         * bus routes the return to the master. The reverb algorithm behind it is
         * swappable without the mixer knowing.
         */
        class ReverbBusEffect final : public IBusEffect
        {
            public:
                /** @brief Takes ownership of the reverb algorithm to run on the bus. */
                explicit ReverbBusEffect(std::unique_ptr<IReverb> reverb)
                    : reverb_(std::move(reverb))
                {
                }

                /** @brief The wrapped reverb, e.g. to push new I3DL2 parameters. */
                IReverb& reverb() noexcept { return *reverb_; }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    reverb_->prepare(sample_rate, max_block_frames);
                }

                void reset() noexcept override { reverb_->reset(); }

                void process(float* left, float* right, int frame_count) noexcept override
                {
                    reverb_->process(left, right, frame_count);
                }

            private:
                std::unique_ptr<IReverb> reverb_;
        };

        /** @brief The metric Sabine constant, `24·ln(10)/c ≈ 0.161 s/m` (c ≈ 343 m/s). */
        constexpr double kSabineConstant = 0.161;

        /**
         * @brief Sabine RT60 — the classic estimate, best for live rooms (ᾱ < ~0.3).
         * @param volume_m3       Room volume in cubic metres.
         * @param surface_area_m2 Total interior surface area in square metres.
         * @param mean_absorption Area-weighted mean absorption coefficient ᾱ in (0, 1).
         * @return The −60 dB decay time in seconds (0 if the inputs are degenerate).
         */
        inline double sabine_rt60(double volume_m3, double surface_area_m2,
                                  double mean_absorption) noexcept
        {
            if (surface_area_m2 <= 0.0 || mean_absorption <= 0.0)
                return 0.0;
            return kSabineConstant * volume_m3 / (surface_area_m2 * mean_absorption);
        }

        /**
         * @brief Eyring RT60 — the correct estimate for dead rooms (ᾱ > ~0.3).
         *
         * Replaces Sabine's `S·ᾱ` with `−S·ln(1 − ᾱ)`, which stays finite and physical
         * as ᾱ → 1 (Sabine wrongly predicts a non-zero RT60 for a fully absorptive room).
         *
         * @param volume_m3       Room volume in cubic metres.
         * @param surface_area_m2 Total interior surface area in square metres.
         * @param mean_absorption Area-weighted mean absorption coefficient ᾱ in (0, 1).
         * @return The −60 dB decay time in seconds (0 if the inputs are degenerate).
         */
        inline double eyring_rt60(double volume_m3, double surface_area_m2,
                                  double mean_absorption) noexcept
        {
            if (surface_area_m2 <= 0.0 || mean_absorption <= 0.0)
                return 0.0;
            if (mean_absorption >= 0.9999)
                mean_absorption = 0.9999;
            return kSabineConstant * volume_m3 / (-surface_area_m2 * std::log(1.0 - mean_absorption));
        }

        /**
         * @brief RT60 with the Sabine/Eyring crossover (Sabine ≤ 0.3, Eyring above).
         *
         * The Resonance Audio recipe: Sabine is accurate and cheap for live spaces,
         * Eyring is needed once the mean absorption is high enough that Sabine's error
         * (and its non-zero-RT60-at-ᾱ=1 artifact) matters.
         */
        inline double reverb_rt60(double volume_m3, double surface_area_m2,
                                  double mean_absorption) noexcept
        {
            return mean_absorption < 0.3
                       ? sabine_rt60(volume_m3, surface_area_m2, mean_absorption)
                       : eyring_rt60(volume_m3, surface_area_m2, mean_absorption);
        }

        /**
         * @brief Builds an @ref I3DL2Reverb from a shoebox room's geometry and materials.
         *
         * Computes volume and surface area from the dimensions, an RT60 per band from the
         * two absorption coefficients (via @ref reverb_rt60), and derives DecayTime,
         * DecayHFRatio, and a predelay from the **mean free path** `4V/S` (the average
         * distance a ray travels between reflections, `mfp/c` seconds).
         *
         * @param width_m          Room width in metres.
         * @param length_m         Room length in metres.
         * @param height_m         Room height in metres.
         * @param absorption_low   Low-band mean absorption ᾱ (0, 1) — sets the body decay.
         * @param absorption_high  High-band mean absorption ᾱ (0, 1) — sets DecayHFRatio.
         * @return An I3DL2 preset matching the room.
         */
        inline I3DL2Reverb shoebox_reverb(double width_m, double length_m, double height_m,
                                          double absorption_low, double absorption_high)
        {
            const double volume = width_m * length_m * height_m;
            const double surface =
                2.0 * (width_m * length_m + width_m * height_m + length_m * height_m);

            const double rt_low = reverb_rt60(volume, surface, absorption_low);
            const double rt_high = reverb_rt60(volume, surface, absorption_high);

            const double speed_of_sound = 343.0;
            const double mean_free_path = surface > 0.0 ? 4.0 * volume / surface : 0.0;
            const double predelay = mean_free_path / speed_of_sound;

            I3DL2Reverb r;
            r.decay_time = static_cast<float>(rt_low > 0.05 ? rt_low : 0.05);
            r.decay_hf_ratio =
                static_cast<float>(rt_low > 1e-6 ? (rt_high / rt_low) : 1.0);
            if (r.decay_hf_ratio < 0.1f) r.decay_hf_ratio = 0.1f;
            if (r.decay_hf_ratio > 2.0f) r.decay_hf_ratio = 2.0f;
            r.reverb_delay = static_cast<float>(predelay < 0.24 ? predelay : 0.24);
            r.reflections_delay = static_cast<float>(predelay * 0.5);
            return r;
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
