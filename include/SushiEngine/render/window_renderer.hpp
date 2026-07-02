/**************************************************************************/
/* window_renderer.hpp                                                    */
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
 * @file window_renderer.hpp
 * @brief The presentation surface a windowed host draws its UI through.
 *
 * The editor programs against IWindowRenderer, never against Vulkan: it owns the
 * graphics device and the swapchain and drives the acquire/clear/submit/present
 * cycle, so the host only opens a frame, records into it, and closes it. The one
 * concession to the graphics API is that begin_frame() hands back an opaque
 * command-buffer handle (a VkCommandBuffer as void*) — the editor's Dear ImGui
 * Vulkan adapter is intrinsically Vulkan and reinterprets it; app and panel code
 * never do. Swapchain recreation on resize is handled internally.
 */

#include <cstdint>
#include <memory>

#include <SushiEngine/render/rhi/device.hpp>

namespace SushiEngine
{
    namespace render
    {
        /** @brief Bring-up request for a windowed renderer. */
        struct WindowRendererDesc
        {
            bool enable_validation = false;
            DevicePreference preference = DevicePreference::HighPerformance;

            /** @brief Instance extensions the window system requires for a surface. */
            std::vector<std::string> required_instance_extensions;

            /** @brief Host hook creating the presentation surface (see SurfaceFactory). */
            SurfaceFactory surface_factory;

            std::uint32_t width = 1280;  /**< Initial framebuffer width in pixels. */
            std::uint32_t height = 720;  /**< Initial framebuffer height in pixels. */
        };

        /**
         * @brief A device plus swapchain that presents a host's frames to a window.
         *
         * One instance per window. begin_frame()/end_frame() bracket the recording
         * of a single presented frame; the renderer owns all synchronization and
         * transparently rebuilds the swapchain when the size it is told differs from
         * the current one.
         */
        class IWindowRenderer
        {
            public:
                virtual ~IWindowRenderer() = default;

                /** @brief Identity of the selected physical device. */
                virtual const DeviceInfo& device_info() const noexcept = 0;

                /** @brief Raw handles the ImGui Vulkan adapter needs to initialize. */
                virtual NativeDeviceHandles native_handles() const noexcept = 0;

                /** @brief Swapchain color format (a VkFormat as an integer). */
                virtual std::uint32_t color_format() const noexcept = 0;

                /** @brief Number of swapchain images (ImGui's ImageCount). */
                virtual std::uint32_t image_count() const noexcept = 0;

                /** @brief Minimum swapchain image count (ImGui's MinImageCount, >= 2). */
                virtual std::uint32_t min_image_count() const noexcept = 0;

                /**
                 * @brief Begins a presented frame, clearing the acquired image.
                 *
                 * Rebuilds the swapchain first if @p width / @p height differ from the
                 * current extent. Acquires the next image, begins a command buffer, and
                 * opens dynamic rendering with a clear so the caller can record UI draws.
                 *
                 * @param width  Current framebuffer width in pixels.
                 * @param height Current framebuffer height in pixels.
                 * @return An opaque command-buffer handle to record into, or nullptr when
                 *         the frame must be skipped this tick (a resize/acquire miss).
                 */
                virtual void* begin_frame(std::uint32_t width, std::uint32_t height) = 0;

                /**
                 * @brief Ends dynamic rendering, submits the frame, and presents it.
                 *
                 * Only valid after a begin_frame() that returned a non-null handle.
                 */
                virtual void end_frame() = 0;

                /** @brief Blocks until the device is idle; call before teardown. */
                virtual void wait_idle() = 0;
        };

        /**
         * @brief Creates the default (Vulkan) windowed renderer.
         *
         * The one place the host names a backend. Throws std::runtime_error if the
         * device or swapchain cannot be created.
         *
         * @param desc Surface hooks, device preference, and initial size.
         * @return An owning handle to the live windowed renderer.
         */
        std::unique_ptr<IWindowRenderer> create_window_renderer(const WindowRendererDesc& desc);
    } // namespace render
} // namespace SushiEngine
