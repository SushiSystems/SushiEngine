/**************************************************************************/
/* box_downscale.hpp                                                     */
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
 * @file box_downscale.hpp
 * @brief A box-filter RGBA8 resample, with no device and no file format knowledge.
 *
 * Every output texel is the unweighted average of the source texels whose box falls under
 * it, which is the simplest resample that does not alias when shrinking an image — a
 * point-sample would silently drop entire rows of pixels between texels once the ratio gets
 * large enough. There is no separate "upscale" case: a target box narrower than one source
 * texel just clamps to a single sample, which degenerates gracefully to nearest-neighbour.
 */

#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Imaging
    {
        /**
         * @brief Box-filter downscales an RGBA8 image to an exact target size.
         * @param source        Tightly packed RGBA8 pixels, @p source_width * @p source_height * 4 bytes.
         * @param source_width  Source width in texels. Must be at least 1.
         * @param source_height Source height in texels. Must be at least 1.
         * @param target_width  Output width in texels. Must be at least 1.
         * @param target_height Output height in texels. Must be at least 1.
         * @return A tightly packed RGBA8 buffer, @p target_width * @p target_height * 4 bytes.
         */
        std::vector<std::uint8_t> box_downscale_rgba8(const std::uint8_t* source,
                                                       std::uint32_t source_width,
                                                       std::uint32_t source_height,
                                                       std::uint32_t target_width,
                                                       std::uint32_t target_height);
    } // namespace Imaging
} // namespace SushiEngine
