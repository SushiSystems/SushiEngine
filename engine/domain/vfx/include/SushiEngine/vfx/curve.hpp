/**************************************************************************/
/* curve.hpp                                                              */
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
 * @file curve.hpp
 * @brief A keyframed scalar curve, evaluated on the CPU and baked to a LUT for the GPU.
 *
 * A curve drives one particle property over a particle's normalized age (size-over-life,
 * drag-over-life, and so on). It is an authoring type — heap-backed keyframes an artist
 * edits — that never itself crosses to the GPU or into an ECS component; only its **baked
 * LUT** (a fixed-width row of floats, produced by @ref bake) does, inside a CompiledEmitter.
 * Both simulation backends sample the same LUT, so a property looks identical whether it is
 * simulated on the GPU or on the deterministic CPU path.
 *
 * Segments interpolate with a cubic Hermite spline over the neighbouring keys' tangents, so
 * an artist gets smooth ease-in/ease-out without authoring bezier handles by hand; a flat
 * (zero-tangent) key degrades to the expected smoothstep-like shape.
 */

#include <algorithm>
#include <cstddef>
#include <vector>

namespace SushiEngine
{
    namespace Vfx
    {
        /** @brief One control point of an @ref AnimationCurve. */
        struct CurveKey
        {
            float time = 0.0f;        /**< Normalized position along the curve, in [0, 1]. */
            float value = 0.0f;       /**< Curve value at @ref time. */
            float in_tangent = 0.0f;  /**< Incoming slope (value units per unit time). */
            float out_tangent = 0.0f; /**< Outgoing slope (value units per unit time). */
        };

        /**
         * @brief A scalar function of normalized age, defined by cubic-Hermite keyframes.
         *
         * Keys are kept sorted by @ref CurveKey::time. A curve with no keys evaluates to a
         * caller-supplied default; a single key is a constant. @ref bake samples the curve
         * at a fixed width for GPU upload.
         */
        class AnimationCurve
        {
            public:
                AnimationCurve() = default;

                /**
                 * @brief Constructs a constant curve.
                 * @param constant_value The value returned everywhere.
                 */
                explicit AnimationCurve(float constant_value)
                {
                    keys_.push_back(CurveKey{0.0f, constant_value, 0.0f, 0.0f});
                }

                /**
                 * @brief Adds a keyframe and keeps the key list sorted by time.
                 * @param key The keyframe to insert.
                 */
                void add_key(const CurveKey& key)
                {
                    const auto position = std::lower_bound(
                        keys_.begin(), keys_.end(), key.time,
                        [](const CurveKey& existing, float time) { return existing.time < time; });
                    keys_.insert(position, key);
                }

                /** @brief Removes every keyframe. */
                void clear() noexcept
                {
                    keys_.clear();
                }

                /** @brief The keyframes, sorted by time. */
                const std::vector<CurveKey>& keys() const noexcept
                {
                    return keys_;
                }

                /** @brief Mutable keyframe access for an editor; the caller must keep them sorted. */
                std::vector<CurveKey>& keys() noexcept
                {
                    return keys_;
                }

                /** @brief Whether the curve has no keyframes. */
                bool empty() const noexcept
                {
                    return keys_.empty();
                }

                /**
                 * @brief Evaluates the curve at a normalized time.
                 *
                 * Clamps @p time to [0, 1] and to the key range: before the first key it
                 * returns the first key's value, after the last the last key's value. Between
                 * two keys it evaluates the cubic Hermite segment scaled by the segment width.
                 *
                 * @param time          Normalized age in [0, 1] (clamped).
                 * @param default_value Returned when the curve has no keys.
                 * @return The interpolated curve value.
                 */
                float evaluate(float time, float default_value = 0.0f) const noexcept
                {
                    if (keys_.empty())
                        return default_value;
                    if (time <= keys_.front().time)
                        return keys_.front().value;
                    if (time >= keys_.back().time)
                        return keys_.back().value;

                    std::size_t upper = 1;
                    while (upper < keys_.size() && keys_[upper].time < time)
                        ++upper;
                    const CurveKey& a = keys_[upper - 1];
                    const CurveKey& b = keys_[upper];
                    const float span = b.time - a.time;
                    if (span <= 0.0f)
                        return b.value;
                    const float t = (time - a.time) / span;
                    const float t2 = t * t;
                    const float t3 = t2 * t;
                    // Cubic Hermite basis; tangents are in value/time, so scale by the span.
                    const float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
                    const float h10 = t3 - 2.0f * t2 + t;
                    const float h01 = -2.0f * t3 + 3.0f * t2;
                    const float h11 = t3 - t2;
                    return h00 * a.value + h10 * span * a.out_tangent + h01 * b.value +
                           h11 * span * b.in_tangent;
                }

                /**
                 * @brief Samples the curve into a fixed-width LUT for GPU upload.
                 *
                 * Writes @p width evenly-spaced samples over [0, 1] into @p out. The sim
                 * shader indexes this row by normalized age and reads back the same value the
                 * CPU backend would compute, so the two domains agree.
                 *
                 * @param out           Destination, @p width floats long.
                 * @param width         Number of samples to write (>= 1).
                 * @param default_value Value used when the curve is empty.
                 */
                void bake(float* out, std::size_t width, float default_value = 0.0f) const noexcept
                {
                    if (width == 0)
                        return;
                    if (width == 1)
                    {
                        out[0] = evaluate(0.0f, default_value);
                        return;
                    }
                    const float step = 1.0f / static_cast<float>(width - 1);
                    for (std::size_t i = 0; i < width; ++i)
                        out[i] = evaluate(static_cast<float>(i) * step, default_value);
                }

            private:
                std::vector<CurveKey> keys_;
        };
    } // namespace Vfx
} // namespace SushiEngine
