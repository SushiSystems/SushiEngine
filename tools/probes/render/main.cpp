/**************************************************************************/
/* main.cpp                                                               */
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

// Headless renderer smoke test: bring up the Vulkan device, render the triangle into
// an offscreen image, and read two pixels back. It needs no window, so it validates
// the whole pipeline path (device, shaders, pipeline, submit) — and the vcpkg
// provisioning behind it — in CI without a display. The center pixel lands inside the
// triangle (must differ from the clear color); the corner lands outside it (must show
// the clear color). This is the render analogue of the ECS sandbox's exit-0 check.

#include <cstdint>
#include <cstdio>
#include <exception>

#include "rhi/vulkan/vulkan_device.hpp"
#include "rhi/vulkan/vulkan_offscreen.hpp"

namespace
{
    /** @brief Sum of a pixel's colour channels — a cheap "is it lit?" measure. */
    int brightness(const SushiEngine::Render::Vulkan::Pixel& pixel)
    {
        return int(pixel.r) + int(pixel.g) + int(pixel.b);
    }
}

int main()
{
    try
    {
        SushiEngine::Render::RenderDeviceDesc desc;
        SushiEngine::Render::Vulkan::VulkanDevice device(desc);
        const SushiEngine::Render::DeviceInfo& info = device.info();

        const std::uint32_t version = info.api_version;
        std::printf("device: %s\n", info.name.c_str());
        std::printf("type: %s\n", info.is_discrete ? "discrete" : "integrated/other");
        std::printf("api: %u.%u.%u\n", (version >> 22) & 0x7Fu,
                    (version >> 12) & 0x3FFu, version & 0xFFFu);

        const SushiEngine::Render::Vulkan::TriangleRenderResult frame =
            SushiEngine::Render::Vulkan::render_triangle_offscreen(device, 64, 64);

        std::printf("center rgba: %u %u %u %u\n", frame.center.r, frame.center.g,
                    frame.center.b, frame.center.a);
        std::printf("corner rgba: %u %u %u %u\n", frame.corner.r, frame.corner.g,
                    frame.corner.b, frame.corner.a);

        // The triangle must have covered the center and left the corner cleared.
        const bool center_lit = brightness(frame.center) > brightness(frame.corner) + 60;
        const bool corner_cleared = brightness(frame.corner) < 80;
        const bool ok = center_lit && corner_cleared && frame.center.a == 255;

        std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
        return ok ? 0 : 1;
    }
    catch (const std::exception& error)
    {
        std::printf("RESULT: FAIL (%s)\n", error.what());
        return 1;
    }
}
