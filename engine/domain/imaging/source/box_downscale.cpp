/**************************************************************************/
/* box_downscale.cpp                                                     */
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

#include "SushiEngine/imaging/box_downscale.hpp"

#include <algorithm>
#include <cstddef>

namespace SushiEngine
{
    namespace Imaging
    {
        std::vector<std::uint8_t> box_downscale_rgba8(const std::uint8_t* source,
                                                       std::uint32_t source_width,
                                                       std::uint32_t source_height,
                                                       std::uint32_t target_width,
                                                       std::uint32_t target_height)
        {
            std::vector<std::uint8_t> result(
                std::size_t(target_width) * std::size_t(target_height) * 4);

            for (std::uint32_t ty = 0; ty < target_height; ++ty)
            {
                const std::uint32_t source_y0 = ty * source_height / target_height;
                const std::uint32_t source_y1 =
                    std::max(source_y0 + 1, (ty + 1) * source_height / target_height);

                for (std::uint32_t tx = 0; tx < target_width; ++tx)
                {
                    const std::uint32_t source_x0 = tx * source_width / target_width;
                    const std::uint32_t source_x1 =
                        std::max(source_x0 + 1, (tx + 1) * source_width / target_width);

                    std::uint32_t sum[4] = {0, 0, 0, 0};
                    std::uint32_t count = 0;
                    for (std::uint32_t sy = source_y0; sy < source_y1 && sy < source_height; ++sy)
                    {
                        for (std::uint32_t sx = source_x0; sx < source_x1 && sx < source_width; ++sx)
                        {
                            const std::uint8_t* texel =
                                source + (std::size_t(sy) * source_width + sx) * 4;
                            for (int channel = 0; channel < 4; ++channel)
                                sum[channel] += texel[channel];
                            ++count;
                        }
                    }

                    std::uint8_t* out =
                        result.data() + (std::size_t(ty) * target_width + tx) * 4;
                    for (int channel = 0; channel < 4; ++channel)
                        out[channel] =
                            count > 0 ? static_cast<std::uint8_t>(sum[channel] / count) : 0;
                }
            }

            return result;
        }
    } // namespace Imaging
} // namespace SushiEngine
