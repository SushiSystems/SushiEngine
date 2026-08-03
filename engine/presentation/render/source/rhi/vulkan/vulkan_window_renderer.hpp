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

#include <memory>

#include <SushiEngine/render/window_renderer.hpp>

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            class AssetLibrary;
        }

        namespace Vulkan
        {
            /**
             * @brief A windowed Vulkan renderer that presents a host's UI frames.
             *
             * Owns its VulkanDevice and swapchain. Double-buffered command
             * submission (@ref FRAMES_IN_FLIGHT frames), with one render-finished
             * semaphore per swapchain image so present never waits on a semaphore
             * still in use. Non-copyable: it owns Vulkan handles.
             *
             * Also the headless case (PLATFORM0 S6): a @ref WindowRendererDescription with no
             * @c surface_factory builds no swapchain and no per-frame sync objects at
             * all — @ref VulkanDevice already supports this (it is exactly how
             * `render_probe`/`render_golden` construct a device), so the only thing
             * this class had to stop doing was throwing when it saw one. A headless
             * instance still builds its asset library and still answers
             * @ref create_scene_view, so a host can render offscreen frames and read
             * them back through `ISceneView::read_output()`; @ref begin_frame,
             * @ref present_scene_view, and @ref end_frame become well-defined no-ops
             * (the same "return null / do nothing" shape they already use for a
             * minimized window) rather than something a headless host must remember
             * never to call.
             */
            class VulkanWindowRenderer final : public IWindowRenderer
            {
                public:
                    explicit VulkanWindowRenderer(const WindowRendererDescription& desc);
                    ~VulkanWindowRenderer() override;

                    VulkanWindowRenderer(const VulkanWindowRenderer&) = delete;
                    VulkanWindowRenderer& operator=(const VulkanWindowRenderer&) = delete;

                    const DeviceInformation& device_info() const noexcept override { return device_.info(); }
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
                    void set_present_mode(PresentMode mode) override;
                    std::unique_ptr<ISceneView> create_scene_view() override;
                    IAssetLibrary& assets() noexcept override;
                    void present_scene_view(ISceneView& view, std::uint32_t slot,
                                            std::uint32_t width, std::uint32_t height) override;

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
                    /**
                     * @brief The Vulkan present mode a pacing choice maps to.
                     * @param mode The authored pacing.
                     * @return The VkPresentModeKHR to ask the surface for.
                     */
                    static VkPresentModeKHR present_mode(PresentMode mode) noexcept;
                    void destroy_swapchain();
                    void create_frames();
                    void destroy_frames();

                    /**
                     * @brief Rebuilds the swapchain if needed, waits the slot's fence, and
                     *        acquires+begins recording the next swapchain image.
                     *
                     * The part begin_frame() and present_scene_view() share: everything up to
                     * but not including a layout transition, since the two callers put the
                     * acquired image into different starting layouts (COLOR_ATTACHMENT_OPTIMAL
                     * to open dynamic rendering, TRANSFER_DST_OPTIMAL to blit into). A no-op
                     * call this frame (frame_acquired_ already true) returns the same buffer
                     * without acquiring twice.
                     *
                     * @param width  Current framebuffer width in pixels.
                     * @param height Current framebuffer height in pixels.
                     * @return The frame's command buffer, or VK_NULL_HANDLE to skip this tick.
                     */
                    VkCommandBuffer acquire(std::uint32_t width, std::uint32_t height);

                    VulkanDevice device_;
                    /** @brief Whether this instance has no surface (no swapchain, ever). */
                    bool headless_ = false;
                    // Held by pointer so the concrete asset library stays out of this
                    // header: it drags in the whole resource stack, and nothing that
                    // includes a window renderer needs to see it.
                    std::unique_ptr<Assets::AssetLibrary> assets_;
                    vkb::Swapchain swapchain_{};
                    VkFormat format_ = VK_FORMAT_UNDEFINED;
                    VkExtent2D extent_{};
                    std::uint32_t min_image_count_ = 2;
                    /** @brief The pacing the current swapchain was built with. */
                    PresentMode present_mode_ = PresentMode::Vsync;
                    std::vector<VkImage> images_;
                    std::vector<VkImageView> views_;
                    std::vector<VkSemaphore> render_finished_; // one per swapchain image

                    FrameResources frames_[FRAMES_IN_FLIGHT];
                    std::uint32_t frame_index_ = 0;
                    std::uint32_t image_index_ = 0;
                    /** @brief Whether this frame's image is acquired and its buffer recording. */
                    bool frame_acquired_ = false;
                    /**
                     * @brief Whether a dynamic-rendering scope is open on the acquired image.
                     *
                     * begin_frame() opens one so the host can record UI draws; end_frame() must
                     * close it and transition to PRESENT_SRC_KHR only when it is the one that
                     * left the image needing that — present_scene_view() does both itself, since
                     * its blit cannot run inside a render scope in the first place.
                     */
                    bool rendering_open_ = false;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
