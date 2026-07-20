/**************************************************************************/
/* resolution_controller.cpp                                              */
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

#include "frame/resolution_controller.hpp"

#include <algorithm>
#include <cmath>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            namespace
            {
                /** @brief Weight the newest measurement carries in the running average. */
                constexpr float SMOOTHING = 0.25f;

                /**
                 * @brief Fraction of the budget the frame may exceed before the scale drops.
                 *
                 * A dead band around the target: without it the controller would step
                 * every frame, and every step reallocates the render-resolution targets.
                 */
                constexpr float OVERRUN_TOLERANCE = 1.05f;

                /** @brief Fraction of the budget the frame must be under before it rises. */
                constexpr float HEADROOM_TOLERANCE = 0.85f;

                /** @brief Steps the scale may fall in one frame when the budget is blown. */
                constexpr float FALL_STEPS = 2.0f;

                /** @brief Steps the scale may rise in one frame when there is headroom. */
                constexpr float RISE_STEPS = 1.0f;
            } // namespace

            float ResolutionController::update(const DynamicResolutionSettings& settings,
                                               float milliseconds) noexcept
            {
                const float minimum = std::min(std::max(settings.minimum_scale, 0.25f), 1.0f);
                const float maximum = std::min(std::max(settings.maximum_scale, minimum), 1.0f);
                if (!settings.enabled || settings.target_milliseconds <= 0.0f)
                {
                    scale_ = maximum;
                    smoothed_milliseconds_ = 0.0f;
                    return scale_;
                }

                if (milliseconds > 0.0f)
                    smoothed_milliseconds_ =
                        smoothed_milliseconds_ <= 0.0f
                            ? milliseconds
                            : smoothed_milliseconds_ + (milliseconds - smoothed_milliseconds_) *
                                                           SMOOTHING;

                // With no measurement yet the scale must not drift: the first frames of a
                // view have no completed submit to read a timestamp from.
                if (smoothed_milliseconds_ > 0.0f)
                {
                    const float target = settings.target_milliseconds;
                    if (smoothed_milliseconds_ > target * OVERRUN_TOLERANCE)
                        scale_ -= STEP * FALL_STEPS;
                    else if (smoothed_milliseconds_ < target * HEADROOM_TOLERANCE)
                        scale_ += STEP * RISE_STEPS;
                }

                scale_ = std::min(std::max(scale_, minimum), maximum);
                // Quantise after clamping so the bounds themselves stay reachable exactly.
                scale_ = std::min(std::max(std::round(scale_ / STEP) * STEP, minimum), maximum);
                return scale_;
            }

            std::uint32_t scaled_extent(std::uint32_t extent, float scale) noexcept
            {
                // Full scale must reproduce the extent exactly, odd sizes included: an
                // even-rounded 1023 would silently render a pixel short of the output.
                if (scale >= 1.0f)
                    return extent < 1 ? 1u : extent;
                const float clamped = std::min(std::max(scale, 0.05f), 1.0f);
                const std::uint32_t scaled =
                    static_cast<std::uint32_t>(std::lround(static_cast<double>(extent) * clamped));
                const std::uint32_t even = scaled & ~1u;
                return even < 2 ? 2u : even;
            }
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
