/**************************************************************************/
/* gradient.hpp                                                           */
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

#pragma once

/**
 * @file gradient.hpp
 * @brief A colour-and-alpha gradient over a particle's normalized age, baked to an RGBA LUT.
 *
 * Colour-over-life is the signature VFX property (a spark fading white → orange → red → black,
 * smoke greying and fading out). Following Unity's model, colour keys and alpha keys are
 * authored on **separate** timelines, because an artist tunes fade-out independently of hue.
 * Like @ref AnimationCurve this is an authoring type; only its baked RGBA LUT row crosses into
 * a CompiledEmitter, and both simulation backends sample it identically.
 */

#include <algorithm>
#include <cstddef>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /** @brief One colour stop of a @ref ColorGradient. */
        struct ColorKey
        {
            float time = 0.0f;   /**< Normalized position, in [0, 1]. */
            Vector3 color{Vector3{1, 1, 1}}; /**< Linear RGB at @ref time. */
        };

        /** @brief One alpha stop of a @ref ColorGradient. */
        struct AlphaKey
        {
            float time = 0.0f;  /**< Normalized position, in [0, 1]. */
            float alpha = 1.0f; /**< Opacity at @ref time, in [0, 1]. */
        };

        /**
         * @brief A linear-RGB colour and opacity as a function of normalized age.
         *
         * Colour and alpha are interpolated linearly along their own sorted key lists. An
         * empty channel evaluates to a caller-supplied default (white / opaque). @ref bake
         * produces an interleaved RGBA float row for GPU upload.
         */
        class ColorGradient
        {
            public:
                ColorGradient() = default;

                /**
                 * @brief Constructs a constant, fully-opaque gradient of one colour.
                 * @param constant_color The colour returned everywhere.
                 */
                explicit ColorGradient(const Vector3& constant_color)
                {
                    color_keys_.push_back(ColorKey{0.0f, constant_color});
                    alpha_keys_.push_back(AlphaKey{0.0f, 1.0f});
                }

                /**
                 * @brief Adds a colour stop, keeping the colour keys sorted by time.
                 * @param key The colour stop to insert.
                 */
                void add_color_key(const ColorKey& key)
                {
                    const auto position = std::lower_bound(
                        color_keys_.begin(), color_keys_.end(), key.time,
                        [](const ColorKey& existing, float time) { return existing.time < time; });
                    color_keys_.insert(position, key);
                }

                /**
                 * @brief Adds an alpha stop, keeping the alpha keys sorted by time.
                 * @param key The alpha stop to insert.
                 */
                void add_alpha_key(const AlphaKey& key)
                {
                    const auto position = std::lower_bound(
                        alpha_keys_.begin(), alpha_keys_.end(), key.time,
                        [](const AlphaKey& existing, float time) { return existing.time < time; });
                    alpha_keys_.insert(position, key);
                }

                /** @brief Removes every colour and alpha stop. */
                void clear() noexcept
                {
                    color_keys_.clear();
                    alpha_keys_.clear();
                }

                /** @brief The colour stops, sorted by time. */
                const std::vector<ColorKey>& color_keys() const noexcept { return color_keys_; }

                /** @brief The alpha stops, sorted by time. */
                const std::vector<AlphaKey>& alpha_keys() const noexcept { return alpha_keys_; }

                /** @brief Mutable colour-stop access for an editor; keep them sorted. */
                std::vector<ColorKey>& color_keys() noexcept { return color_keys_; }

                /** @brief Mutable alpha-stop access for an editor; keep them sorted. */
                std::vector<AlphaKey>& alpha_keys() noexcept { return alpha_keys_; }

                /**
                 * @brief Evaluates the linear-RGB colour at a normalized time.
                 * @param time          Normalized age in [0, 1] (clamped).
                 * @param default_color Returned when there are no colour stops.
                 * @return The interpolated colour.
                 */
                Vector3 evaluate_color(float time, const Vector3& default_color = Vector3{1, 1, 1}) const noexcept
                {
                    if (color_keys_.empty())
                        return default_color;
                    if (time <= color_keys_.front().time)
                        return color_keys_.front().color;
                    if (time >= color_keys_.back().time)
                        return color_keys_.back().color;
                    std::size_t upper = 1;
                    while (upper < color_keys_.size() && color_keys_[upper].time < time)
                        ++upper;
                    const ColorKey& a = color_keys_[upper - 1];
                    const ColorKey& b = color_keys_[upper];
                    const float span = b.time - a.time;
                    const float t = span > 0.0f ? (time - a.time) / span : 0.0f;
                    return lerp(a.color, b.color, static_cast<Scalar>(t));
                }

                /**
                 * @brief Evaluates the opacity at a normalized time.
                 * @param time          Normalized age in [0, 1] (clamped).
                 * @param default_alpha Returned when there are no alpha stops.
                 * @return The interpolated opacity.
                 */
                float evaluate_alpha(float time, float default_alpha = 1.0f) const noexcept
                {
                    if (alpha_keys_.empty())
                        return default_alpha;
                    if (time <= alpha_keys_.front().time)
                        return alpha_keys_.front().alpha;
                    if (time >= alpha_keys_.back().time)
                        return alpha_keys_.back().alpha;
                    std::size_t upper = 1;
                    while (upper < alpha_keys_.size() && alpha_keys_[upper].time < time)
                        ++upper;
                    const AlphaKey& a = alpha_keys_[upper - 1];
                    const AlphaKey& b = alpha_keys_[upper];
                    const float span = b.time - a.time;
                    const float t = span > 0.0f ? (time - a.time) / span : 0.0f;
                    return a.alpha + (b.alpha - a.alpha) * t;
                }

                /**
                 * @brief Samples the gradient into a fixed-width interleaved RGBA float LUT.
                 *
                 * Writes @p width RGBA quadruplets (4 * @p width floats) evenly over [0, 1].
                 * The sim shader indexes this by normalized age for colour-over-life.
                 *
                 * @param out   Destination, 4 * @p width floats long.
                 * @param width Number of RGBA samples to write (>= 1).
                 */
                void bake(float* out, std::size_t width) const noexcept
                {
                    if (width == 0)
                        return;
                    const float step = width > 1 ? 1.0f / static_cast<float>(width - 1) : 0.0f;
                    for (std::size_t i = 0; i < width; ++i)
                    {
                        const float time = static_cast<float>(i) * step;
                        const Vector3 color = evaluate_color(time);
                        out[i * 4 + 0] = static_cast<float>(color.x);
                        out[i * 4 + 1] = static_cast<float>(color.y);
                        out[i * 4 + 2] = static_cast<float>(color.z);
                        out[i * 4 + 3] = evaluate_alpha(time);
                    }
                }

            private:
                std::vector<ColorKey> color_keys_;
                std::vector<AlphaKey> alpha_keys_;
        };
    } // namespace Vfx
} // namespace SushiEngine
