/**************************************************************************/
/* resolution_controller.hpp                                              */
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
 * @file resolution_controller.hpp
 * @brief The governor that holds the frame budget by moving the render resolution.
 *
 * The primary automatic performance lever: it reads the GPU time the profiler measured
 * for the last completed frame and walks the internal render scale toward whatever
 * keeps that time under the budget. The temporal resolve upscales from the chosen
 * extent, which is what makes the change hard to see; without it this would be a
 * visible resolution pop rather than a governor.
 */

#include <cstdint>

#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            /**
             * @brief Chooses this frame's internal render scale from the measured GPU time.
             *
             * Deliberately a plain object with no Vulkan and no graph knowledge: it maps
             * (measured milliseconds, settings) to a scale, so it is testable on its own
             * and the scene view is the only thing that has to know what a scale means.
             */
            class ResolutionController
            {
                public:
                    /**
                     * @brief Folds one frame's measurement into the chosen scale.
                     *
                     * Reacts asymmetrically on purpose: overruns drop the scale quickly
                     * because the budget is already blown, while recovery is gradual so a
                     * single cheap frame cannot start an oscillation. The result is
                     * quantised so the extent only changes in steps the transient pools
                     * can actually reuse an allocation across.
                     *
                     * @param settings     The dynamic-resolution request and its bounds.
                     * @param milliseconds GPU time of the last completed frame; zero or
                     *                     less means no measurement is available yet.
                     * @return The scale the next frame should render at, per axis.
                     */
                    float update(const DynamicResolutionSettings& settings,
                                 float milliseconds) noexcept;

                    /** @brief The scale the last update() settled on. */
                    float scale() const noexcept { return scale_; }

                    /** @brief Returns the controller to full resolution. */
                    void reset() noexcept { scale_ = 1.0f; }

                private:
                    /**
                     * @brief Steps the scale is quantised to.
                     *
                     * Sixteenths: fine enough that a step is invisible through the
                     * temporal resolve, coarse enough that a scale hovering near a
                     * boundary does not reallocate every transient each frame.
                     */
                    static constexpr float STEP = 1.0f / 16.0f;

                    float scale_ = 1.0f;
                    float smoothed_milliseconds_ = 0.0f;
            };

            /**
             * @brief Applies a render scale to one axis.
             *
             * Rounds to an even count so a half-resolution child target (the cloud march)
             * keeps an exact two-to-one relationship with its parent.
             *
             * @param extent The output extent in pixels.
             * @param scale  The scale to apply, clamped to (0, 1].
             * @return The internal render extent, at least 2 pixels.
             */
            std::uint32_t scaled_extent(std::uint32_t extent, float scale) noexcept;
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
