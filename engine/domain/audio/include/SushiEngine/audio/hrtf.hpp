/**************************************************************************/
/* hrtf.hpp                                                              */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_AUDIO_HRTF_HPP
#define SUSHIENGINE_AUDIO_HRTF_HPP

/**
 * @file hrtf.hpp
 * @brief The measured-HRTF seam — an `IHRTFDatabase` and the per-ear HRIR convolver.
 *
 * The fidelity upgrade the analytic head model of `spatializer.hpp` was written to accept:
 * instead of synthesising each virtual speaker's ear signal from a Woodworth ITD and a
 * head-shadow low-pass, a measured **head-related impulse response** (HRIR) pair — the true
 * left/right response of a real head to a source in a given direction — is convolved in.
 * This captures the pinna, torso, and ear-canal cues the analytic model only approximates.
 *
 * @ref IHRTFDatabase is the dependency-free seam: any provider (a baked table, a synthetic
 * set, or the SOFA/HDF5 loader in `sofa_hrtf.hpp`) hands the spatializer an HRIR pair for a
 * head-relative direction. @ref HrirConvolver is the direct-form FIR that applies it. This
 * header carries no third-party dependency and rides the `audio.hpp` umbrella.
 */

#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief A source of measured HRIR pairs, keyed by head-relative direction.
         *
         * The provider owns the measurement grid and the nearest/interpolated lookup; the
         * spatializer only asks for the left/right impulse responses at a direction and
         * convolves them. All impulse responses in one database share @ref ir_length and
         * @ref sample_rate.
         */
        class IHRTFDatabase
        {
            public:
                virtual ~IHRTFDatabase() = default;

                /** @brief The number of taps in each ear's impulse response. */
                virtual int ir_length() const noexcept = 0;

                /** @brief The sample rate the impulse responses were measured/resampled at. */
                virtual double sample_rate() const noexcept = 0;

                /**
                 * @brief Writes the HRIR pair for a head-relative direction.
                 *
                 * @param front       Head-relative front component (+x forward).
                 * @param left        Head-relative left component (+y left).
                 * @param up          Head-relative up component (+z up).
                 * @param left_ir     Filled with @ref ir_length left-ear taps.
                 * @param right_ir    Filled with @ref ir_length right-ear taps.
                 */
                virtual void get_hrir(float front, float left, float up, float* left_ir,
                                      float* right_ir) const noexcept = 0;
        };

        /**
         * @brief A direct-form FIR that convolves a block with one ear's HRIR and accumulates.
         *
         * Holds the taps and a circular history so processing is continuous across blocks
         * (the reverberant tail of one block carries into the next). @ref process_block adds
         * its output to the destination, matching the spatializer's accumulate-into-ears model.
         */
        class HrirConvolver
        {
            public:
                /**
                 * @brief Installs the impulse response and resets the history.
                 * @param taps The impulse response.
                 * @param n    Number of taps.
                 */
                void prepare(const float* taps, int n)
                {
                    taps_.assign(taps, taps + n);
                    history_.assign(static_cast<std::size_t>(n), 0.0f);
                    write_ = 0;
                }

                /** @brief Zeroes the history without changing the taps. */
                void reset() noexcept
                {
                    for (float& h : history_)
                        h = 0.0f;
                    write_ = 0;
                }

                /** @brief The tap count. */
                int length() const noexcept { return static_cast<int>(taps_.size()); }

                /**
                 * @brief Convolves @p in with the impulse response and adds it into @p out.
                 * @param in          Input block.
                 * @param out         Output block, accumulated into.
                 * @param frame_count Number of samples.
                 */
                void process_block(const float* in, float* out, int frame_count) noexcept
                {
                    const int n = static_cast<int>(taps_.size());
                    if (n == 0)
                        return;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        history_[static_cast<std::size_t>(write_)] = in[i];
                        float acc = 0.0f;
                        int idx = write_;
                        for (int t = 0; t < n; ++t)
                        {
                            acc += taps_[static_cast<std::size_t>(t)] *
                                   history_[static_cast<std::size_t>(idx)];
                            --idx;
                            if (idx < 0)
                                idx = n - 1;
                        }
                        out[i] += acc;
                        ++write_;
                        if (write_ >= n)
                            write_ = 0;
                    }
                }

            private:
                std::vector<float> taps_;
                std::vector<float> history_;
                int write_ = 0;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
