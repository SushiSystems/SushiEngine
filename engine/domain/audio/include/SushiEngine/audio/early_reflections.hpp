/**************************************************************************/
/* early_reflections.hpp                                                 */
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

#ifndef SUSHIENGINE_AUDIO_EARLY_REFLECTIONS_HPP
#define SUSHIENGINE_AUDIO_EARLY_REFLECTIONS_HPP

/**
 * @file early_reflections.hpp
 * @brief The image-source early-reflection model and its tapped-delay renderer.
 *
 * The first tens of milliseconds after the direct sound are sparse, discrete reflections
 * off nearby surfaces — and they, not the late tail, are what tell the ear the size and
 * shape of a room (§7 of `docs/slop/audio_system.md`). They are computed by the
 * **image-source method**: a reflection off a wall sounds exactly as if it came from a
 * mirror-image of the source on the far side of that wall, so each wall yields one
 * *image source*, and its contribution is a single delayed, attenuated tap — delay from
 * the image-to-listener distance, gain from spreading loss and the wall's reflectivity,
 * and a direction (which lets the caller place it, or feed the late FDN so the tail
 * inherits the room's texture and reaches density faster).
 *
 * @ref ImageSourceModel computes the taps for a shoebox room (the six walls, first order);
 * @ref EarlyReflections renders them from a delay line with linear interpolation. Portable
 * `float` maths, no SDL and no SushiRuntime.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief One image-source reflection: when it arrives, how loud, and from where. */
        struct ReflectionTap
        {
            float delay_seconds = 0.0f; /**< Arrival delay = image-to-listener distance / c. */
            float gain = 0.0f;          /**< Linear gain (spreading loss × wall reflectivity). */
            AudioVec3 direction;        /**< Unit direction the reflection arrives from (image → listener). */
        };

        /**
         * @brief First-order image sources for an axis-aligned shoebox room.
         *
         * Reflects the source across each of the six walls to get six image sources, and
         * turns each into a @ref ReflectionTap. Reflectivity is the amplitude kept per
         * bounce (≈ √(1 − absorption)); the direct path is *not* included (it is the voice
         * itself).
         */
        class ImageSourceModel
        {
            public:
                /**
                 * @brief Computes the six first-order reflection taps for a shoebox.
                 * @param source        The source world position.
                 * @param listener      The listener world position.
                 * @param room_center   The room centre.
                 * @param room_half     The room half-extents (must be positive).
                 * @param reflectivity  Amplitude kept per wall bounce in [0, 1].
                 * @param speed_of_sound Speed of sound in m/s.
                 * @param out           Filled with up to six taps (cleared first).
                 */
                static void compute(const AudioVec3& source, const AudioVec3& listener,
                                    const AudioVec3& room_center, const AudioVec3& room_half,
                                    float reflectivity, float speed_of_sound,
                                    std::vector<ReflectionTap>& out)
                {
                    out.clear();
                    if (speed_of_sound < 1.0f)
                        speed_of_sound = 343.0f;
                    const float min_x = room_center.x - room_half.x, max_x = room_center.x + room_half.x;
                    const float min_y = room_center.y - room_half.y, max_y = room_center.y + room_half.y;
                    const float min_z = room_center.z - room_half.z, max_z = room_center.z + room_half.z;

                    const AudioVec3 images[6] = {
                        {2.0f * min_x - source.x, source.y, source.z}, // −x wall
                        {2.0f * max_x - source.x, source.y, source.z}, // +x wall
                        {source.x, 2.0f * min_y - source.y, source.z}, // −y wall
                        {source.x, 2.0f * max_y - source.y, source.z}, // +y wall
                        {source.x, source.y, 2.0f * min_z - source.z}, // −z wall
                        {source.x, source.y, 2.0f * max_z - source.z}, // +z wall
                    };

                    for (const AudioVec3& image : images)
                    {
                        const float dx = listener.x - image.x, dy = listener.y - image.y,
                                    dz = listener.z - image.z;
                        const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
                        if (d < 1.0e-3f)
                            continue;
                        ReflectionTap tap;
                        tap.delay_seconds = d / speed_of_sound;
                        tap.gain = reflectivity / d; // spreading loss × one bounce
                        const float inv = 1.0f / d;
                        tap.direction = AudioVec3{dx * inv, dy * inv, dz * inv};
                        out.push_back(tap);
                    }
                }

                /** @brief The amplitude reflectivity for a broadband absorption coefficient. */
                static float reflectivity_from_absorption(float absorption) noexcept
                {
                    const float a = absorption < 0.0f ? 0.0f : (absorption > 1.0f ? 1.0f : absorption);
                    return std::sqrt(1.0f - a);
                }
        };

        /**
         * @brief Renders a set of @ref ReflectionTap taps from one delay line.
         *
         * A single mono delay line the dry source is written into; each tap reads it back
         * at its delay with linear interpolation and sums at its gain, producing the early
         * reflection field. Gains and delays are ramped-in-effect by re-reading fresh taps
         * each block (the taps move slowly as the source/listener move), and a per-tap gain
         * is applied flat within the block — smooth enough for the sparse early field. Sum
         * this into the direct or the reverb input.
         */
        class EarlyReflections
        {
            public:
                /**
                 * @brief Sizes the delay line for the longest reflection.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block processed.
                 * @param max_delay_seconds The longest reflection delay to support.
                 */
                void prepare(double sample_rate, int max_block_frames, float max_delay_seconds)
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                    int cap = static_cast<int>(max_delay_seconds * static_cast<float>(sample_rate)) + 4;
                    if (cap < 8)
                        cap = 8;
                    buffer_.assign(static_cast<std::size_t>(cap), 0.0f);
                    write_ = 0;
                }

                /** @brief Clears the delay line. */
                void reset() noexcept
                {
                    std::fill(buffer_.begin(), buffer_.end(), 0.0f);
                    write_ = 0;
                }

                /** @brief Publishes the taps to render (copied). */
                void set_taps(const std::vector<ReflectionTap>& taps) { taps_ = taps; }

                /** @brief The current taps (diagnostics/tests). */
                const std::vector<ReflectionTap>& taps() const noexcept { return taps_; }

                /**
                 * @brief Renders the early field for one dry block.
                 * @param input       The dry mono source block.
                 * @param output      The early-reflection block (not summed with dry).
                 * @param frame_count Number of samples.
                 */
                void process(const float* input, float* output, int frame_count) noexcept
                {
                    const int cap = static_cast<int>(buffer_.size());
                    if (cap < 8)
                    {
                        for (int i = 0; i < frame_count; ++i)
                            output[i] = 0.0f;
                        return;
                    }
                    for (int i = 0; i < frame_count; ++i)
                    {
                        buffer_[static_cast<std::size_t>(write_)] = input[i];
                        float acc = 0.0f;
                        for (const ReflectionTap& tap : taps_)
                        {
                            const float d = tap.delay_seconds * static_cast<float>(sample_rate_);
                            if (d < 1.0f || d >= static_cast<float>(cap - 1))
                                continue;
                            const int di = static_cast<int>(d);
                            const float frac = d - static_cast<float>(di);
                            int r0 = write_ - di;
                            while (r0 < 0)
                                r0 += cap;
                            int r1 = r0 - 1;
                            while (r1 < 0)
                                r1 += cap;
                            const float s0 = buffer_[static_cast<std::size_t>(r0)];
                            const float s1 = buffer_[static_cast<std::size_t>(r1)];
                            acc += tap.gain * (s0 + frac * (s1 - s0));
                        }
                        output[i] = acc;
                        if (++write_ >= cap)
                            write_ = 0;
                    }
                }

            private:
                std::vector<float> buffer_;
                std::vector<ReflectionTap> taps_;
                double sample_rate_ = 48000.0;
                int write_ = 0;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
