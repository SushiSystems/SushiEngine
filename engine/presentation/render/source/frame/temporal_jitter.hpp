/**************************************************************************/
/* temporal_jitter.hpp                                                    */
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
 * @file temporal_jitter.hpp
 * @brief The sub-pixel offset sequence the temporal resolve accumulates over.
 *
 * Every frame shifts the projection by a fraction of a pixel, so N frames sample N
 * different positions inside each pixel and the resolve averages them — that is where
 * the anti-aliasing comes from, and it is also what lets a smaller render grid recover
 * detail at a larger output grid. The sequence is Halton, whose low discrepancy fills
 * the pixel far more evenly over a short run than a random or regular pattern.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            /**
             * @brief Length of the jitter cycle when rendering at the output resolution.
             *
             * Eight positions converge fast enough that a camera stopping is stable
             * within a few frames, and short enough that the history never has to hold
             * a sample older than the neighbourhood clamp can validate.
             */
            constexpr std::uint32_t JITTER_PHASE_COUNT = 8;

            /**
             * @brief The radical inverse of an index in a given base.
             *
             * The one primitive a Halton sequence is built from: it reflects the index's
             * digits about the radix point, which is what makes successive samples fall
             * in the largest remaining gap rather than clustering.
             *
             * @param index The sample index, from zero.
             * @param base  The sequence base; 2 and 3 give the classic Halton pair.
             * @return The sample in [0, 1).
             */
            float radical_inverse(std::uint32_t index, std::uint32_t base) noexcept;

            /**
             * @brief The sub-pixel jitter for one frame, in normalised device coordinates.
             *
             * The offset is expressed in NDC so it can be added straight into the
             * projection's third column, which shifts the whole frustum by a fraction of
             * a pixel without touching the view or any world-space value.
             *
             * @param frame_index  The monotonic frame counter.
             * @param width        Internal render width in pixels.
             * @param height       Internal render height in pixels.
             * @param scale        Multiplies the offset; below 1 trades aliasing for stability.
             * @param phase_count  Positions in the cycle; more when upscaling, since more
             *                     samples are needed to fill the larger output grid.
             * @param jitter       Receives the NDC offset as {x, y}.
             */
            void frame_jitter(std::uint32_t frame_index, std::uint32_t width,
                              std::uint32_t height, float scale, std::uint32_t phase_count,
                              float jitter[2]) noexcept;
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
