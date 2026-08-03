/**************************************************************************/
/* temporal_jitter.cpp                                                    */
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

#include "frame/temporal_jitter.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            float radical_inverse(std::uint32_t index, std::uint32_t base) noexcept
            {
                if (base < 2)
                    return 0.0f;
                float result = 0.0f;
                float fraction = 1.0f / static_cast<float>(base);
                while (index > 0)
                {
                    result += static_cast<float>(index % base) * fraction;
                    index /= base;
                    fraction /= static_cast<float>(base);
                }
                return result;
            }

            void frame_jitter(std::uint32_t frame_index, std::uint32_t width,
                              std::uint32_t height, float scale, std::uint32_t phase_count,
                              float jitter[2]) noexcept
            {
                jitter[0] = 0.0f;
                jitter[1] = 0.0f;
                if (width == 0 || height == 0 || phase_count == 0)
                    return;

                // Halton is defined from index 1: index 0 returns the pixel centre for
                // every base, which would waste one of the few positions in the cycle.
                const std::uint32_t phase = frame_index % phase_count + 1;
                const float offset_x = radical_inverse(phase, 2) - 0.5f;
                const float offset_y = radical_inverse(phase, 3) - 0.5f;

                // A whole pixel spans two units of normalised device space, so a
                // sub-pixel offset in pixels becomes 2/extent of it.
                jitter[0] = offset_x * scale * 2.0f / static_cast<float>(width);
                jitter[1] = offset_y * scale * 2.0f / static_cast<float>(height);
            }
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
