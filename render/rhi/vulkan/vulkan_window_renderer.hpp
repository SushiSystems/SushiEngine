/**************************************************************************/
/* vulkan_window_renderer.hpp                                             */
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
 * @file vulkan_window_renderer.hpp
 * @brief Vulkan implementation of IWindowRenderer: device + swapchain + present.
 *
 * Internal to the render library. Composes a VulkanDevice with a vk-bootstrap
 * swapchain and the per-frame synchronization (image-available/render-finished
 * semaphores, in-flight fences) needed to present with Vulkan 1.3 dynamic
 * rendering. Rebuilds the swapchain when the requested size changes or the
 * present engine reports it out of date.
 */

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <VkBootstrap.h>

#include <SushiEngine/render/window_renderer.hpp>

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief A windowed Vulkan renderer that presents a host's UI frames.
             *
             * Owns its VulkanDevice and swapchain. Double-buffered command
             * submission (@ref FRAMES_IN_FLIGHT frames), with one render-finished
             * semaphore per swapchain image so present never waits on a semaphore
             * still in use. Non-copyable: it owns Vulkan handles.
             */
            class VulkanWindowRenderer final : public IWindowRenderer
            {
                public:
                    explicit VulkanWindowRenderer(const WindowRendererDesc& desc);
                    ~VulkanWindowRenderer() override;

                    VulkanWindowRenderer(const VulkanWindowRenderer&) = delete;
                    VulkanWindowRenderer& operator=(const VulkanWindowRenderer&) = delete;

                    const DeviceInfo& device_info() const noexcept override { return device_.info(); }
                    NativeDeviceHandles native_handles() const noexcept override { return device_.native_handles(); }
                    std::uint32_t color_format() const noexcept override { return format_; }
                    std::uint32_t image_count() const noexcept override
                    {
                        return static_cast<std::uint32_t>(images_.size());
                    }
                    std::uint32_t min_image_count() const noexcept override { return min_image_count_; }

                    void* begin_frame(std::uint32_t width, std::uint32_t height) override;
                    void end_frame() override;
                    void wait_idle() override;
                    std::unique_ptr<ISceneView> create_scene_view() override;

                private:
                    static constexpr std::uint32_t FRAMES_IN_FLIGHT = 2;

                    /** @brief Per-in-flight-frame recording and sync objects. */
                    struct FrameResources
                    {
                        VkCommandPool pool = VK_NULL_HANDLE;
                        VkCommandBuffer cmd = VK_NULL_HANDLE;
                        VkSemaphore image_available = VK_NULL_HANDLE;
                        VkFence in_flight = VK_NULL_HANDLE;
                    };

                    void create_swapchain(std::uint32_t width, std::uint32_t height);
                    void destroy_swapchain();
                    void create_frames();
                    void destroy_frames();

                    VulkanDevice device_;
                    vkb::Swapchain swapchain_{};
                    VkFormat format_ = VK_FORMAT_UNDEFINED;
                    VkExtent2D extent_{};
                    std::uint32_t min_image_count_ = 2;
                    std::vector<VkImage> images_;
                    std::vector<VkImageView> views_;
                    std::vector<VkSemaphore> render_finished_; // one per swapchain image

                    FrameResources frames_[FRAMES_IN_FLIGHT];
                    std::uint32_t frame_index_ = 0;
                    std::uint32_t image_index_ = 0;
                    bool frame_open_ = false;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
