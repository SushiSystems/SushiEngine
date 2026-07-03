/**************************************************************************/
/* vulkan_offscreen.hpp                                                   */
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
 * @file vulkan_offscreen.hpp
 * @brief One-shot offscreen triangle render with host readback.
 *
 * The first pixels the renderer produces. It draws the triangle shaders into a
 * VMA-backed color image using Vulkan 1.3 dynamic rendering, copies the result to a
 * host-visible buffer, and returns two sampled pixels — enough to verify the whole
 * pipeline path (shaders, pipeline, command submission) headlessly, without a
 * window. The editor viewport later renders into the same kind of target and samples
 * it with ImGui instead of reading it back.
 */

#include <cstdint>

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /** @brief One RGBA8 pixel read back from the rendered image. */
            struct Pixel
            {
                std::uint8_t r = 0;
                std::uint8_t g = 0;
                std::uint8_t b = 0;
                std::uint8_t a = 0;
            };

            /** @brief Two sampled pixels from a one-shot offscreen triangle render. */
            struct TriangleRenderResult
            {
                Pixel center; /**< Image center — inside the triangle. */
                Pixel corner; /**< Top-left pixel — outside it (shows the clear color). */
                std::uint32_t width = 0;
                std::uint32_t height = 0;
            };

            /**
             * @brief Renders the triangle once into an offscreen image and reads it back.
             *
             * Builds a transient pipeline and command buffer, submits one draw, waits
             * for it, and samples the center and corner pixels. All resources are
             * created and destroyed within the call. Throws std::runtime_error on any
             * Vulkan failure.
             *
             * @param device The live Vulkan device to render on.
             * @param width  Target width in pixels.
             * @param height Target height in pixels.
             * @return The sampled pixels and the target dimensions.
             */
            TriangleRenderResult render_triangle_offscreen(VulkanDevice& device,
                                                           std::uint32_t width,
                                                           std::uint32_t height);
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
