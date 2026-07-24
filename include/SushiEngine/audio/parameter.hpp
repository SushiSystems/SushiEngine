/**************************************************************************/
/* parameter.hpp                                                         */
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

#ifndef SUSHIENGINE_AUDIO_PARAMETER_HPP
#define SUSHIENGINE_AUDIO_PARAMETER_HPP

/**
 * @file parameter.hpp
 * @brief Parameter smoothing and RTPC — the control-plane → audio-plane value bridge.
 *
 * Every mix parameter a game changes at runtime (a bus fader, a voice gain, a filter
 * cutoff) crosses the thread boundary the same way (see `docs/design/audio_system.md`
 * §8): the control thread publishes a **target** into an atomic, and the audio thread
 * **ramps toward it** — never a raw jump, which would click. @ref SmoothedValue is
 * that primitive; @ref Rtpc layers the "game variable → authored curve → target"
 * mapping (real-time parameter control) on top of it, evaluating the curve on the
 * control thread so the audio thread only ever slews.
 */

#include <atomic>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief A value the control thread targets and the audio thread slews toward.
         *
         * @ref set_target publishes atomically from any thread; @ref advance_block runs
         * on the audio thread and moves the current value toward the target at a
         * constant, configured rate, bracketing the block as `[start, end]` so a caller
         * can apply a click-free gain ramp. @ref snap sets both immediately, for
         * initialization.
         */
        class SmoothedValue
        {
            public:
                /**
                 * @brief Sets how fast the value slews.
                 *
                 * The rate is a constant slope: a change of one unit takes
                 * @p smoothing_seconds to complete, so the audio thread never steps.
                 *
                 * @param smoothing_seconds Time to traverse one unit of value (0 = instant).
                 * @param sample_rate       The stream sample rate in Hz.
                 */
                void configure(double smoothing_seconds, double sample_rate) noexcept
                {
                    if (smoothing_seconds <= 0.0 || sample_rate <= 0.0)
                        max_step_per_sample_ = 1.0e30f;
                    else
                        max_step_per_sample_ =
                            static_cast<float>(1.0 / (smoothing_seconds * sample_rate));
                }

                /** @brief Publishes a new target (any thread). */
                void set_target(float value) noexcept
                {
                    target_.store(value, std::memory_order_relaxed);
                }

                /** @brief Sets target and current value immediately (initialization). */
                void snap(float value) noexcept
                {
                    target_.store(value, std::memory_order_relaxed);
                    current_ = value;
                }

                /** @brief The current audio-thread value (last @ref advance_block result). */
                float current() const noexcept { return current_; }

                /** @brief The published target. */
                float target() const noexcept { return target_.load(std::memory_order_relaxed); }

                /**
                 * @brief Advances the value across one block (audio thread).
                 * @param frame_count The block length in samples.
                 * @param out_start   Set to the value at the block start (before advancing).
                 * @param out_end     Set to the value at the block end (after advancing).
                 */
                void advance_block(int frame_count, float& out_start, float& out_end) noexcept
                {
                    out_start = current_;
                    const float target = target_.load(std::memory_order_relaxed);
                    const float delta = target - current_;
                    const float max_change = max_step_per_sample_ * static_cast<float>(frame_count);
                    if (std::fabs(delta) <= max_change)
                        current_ = target;
                    else
                        current_ += (delta > 0.0f ? max_change : -max_change);
                    out_end = current_;
                }

                /**
                 * @brief Advances the value one block and returns the end value.
                 * @param frame_count The block length in samples.
                 * @return The value at the block end.
                 */
                float advance_block(int frame_count) noexcept
                {
                    float start = 0.0f, end = 0.0f;
                    advance_block(frame_count, start, end);
                    return end;
                }

            private:
                std::atomic<float> target_{0.0f};
                float current_{0.0f};
                float max_step_per_sample_{1.0e30f};
        };

        /**
         * @brief A clamped piecewise-linear curve mapping a game value to a parameter.
         *
         * Breakpoints are kept sorted by input; @ref evaluate interpolates linearly
         * between them and clamps to the end values outside the range — the shape a
         * sound designer authors to turn "engine RPM" or "health" into a gain, a cutoff,
         * or a send level.
         */
        class RtpcCurve
        {
            public:
                /**
                 * @brief Adds a breakpoint, keeping the curve sorted by input.
                 * @param input  The game-value coordinate.
                 * @param output The mapped parameter value at @p input.
                 */
                void add_point(float input, float output)
                {
                    Point point{input, output};
                    std::size_t i = 0;
                    while (i < points_.size() && points_[i].input < input)
                        ++i;
                    points_.insert(points_.begin() + static_cast<std::ptrdiff_t>(i), point);
                }

                /** @brief Whether any breakpoints have been added. */
                bool empty() const noexcept { return points_.empty(); }

                /**
                 * @brief Maps @p input through the curve.
                 * @param input The game value.
                 * @return The interpolated output, clamped to the end breakpoints.
                 */
                float evaluate(float input) const noexcept
                {
                    if (points_.empty())
                        return input;
                    if (input <= points_.front().input)
                        return points_.front().output;
                    if (input >= points_.back().input)
                        return points_.back().output;
                    for (std::size_t i = 1; i < points_.size(); ++i)
                    {
                        if (input <= points_[i].input)
                        {
                            const Point& a = points_[i - 1];
                            const Point& b = points_[i];
                            const float span = b.input - a.input;
                            const float t = span > 0.0f ? (input - a.input) / span : 0.0f;
                            return a.output + t * (b.output - a.output);
                        }
                    }
                    return points_.back().output;
                }

            private:
                struct Point
                {
                    float input;
                    float output;
                };

                std::vector<Point> points_;
            };

        /**
         * @brief A real-time parameter control: game value → curve → smoothed target.
         *
         * The game calls @ref set with a raw value (RPM, speed, health); the curve maps
         * it to a parameter value on the control thread, which is published as the
         * @ref SmoothedValue target. The audio thread reads @ref value and slews. This
         * keeps all curve evaluation off the audio thread and all clicks out of the mix.
         */
        class Rtpc
        {
            public:
                /**
                 * @brief Configures the slew rate of the underlying smoothed value.
                 * @param smoothing_seconds Time to traverse one unit of the mapped value.
                 * @param sample_rate       The stream sample rate in Hz.
                 */
                void configure(double smoothing_seconds, double sample_rate) noexcept
                {
                    value_.configure(smoothing_seconds, sample_rate);
                }

                /** @brief Installs the game-value → parameter mapping curve. */
                void set_curve(RtpcCurve curve) { curve_ = std::move(curve); }

                /**
                 * @brief Sets the raw game value; maps it and publishes the target (control thread).
                 * @param game_value The raw game-side value.
                 */
                void set(float game_value) noexcept { value_.set_target(mapped(game_value)); }

                /** @brief Sets the raw game value and snaps the smoothed value to it (initialization). */
                void snap(float game_value) noexcept { value_.snap(mapped(game_value)); }

                /** @brief The underlying smoothed value the audio thread reads/advances. */
                SmoothedValue& value() noexcept { return value_; }

            private:
                float mapped(float game_value) const noexcept
                {
                    return curve_.empty() ? game_value : curve_.evaluate(game_value);
                }

                SmoothedValue value_;
                RtpcCurve curve_;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
