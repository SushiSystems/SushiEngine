/**************************************************************************/
/* vulkan_device.hpp                                                     */
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
 * @file vulkan_device.hpp
 * @brief The Vulkan implementation of IRenderDevice.
 *
 * Internal to the render library — consumers use the abstract device.hpp. This
 * header pulls in the Vulkan, VMA, and vk-bootstrap types, so only backend
 * translation units include it. It brings up a Vulkan 1.3 instance and device
 * (dynamic rendering + synchronization2) and a VMA allocator, and exposes the raw
 * handles later render stages (swapchain, command lists) are built from.
 */

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>
#include <VkBootstrap.h>

#include <SushiEngine/render/rhi/device.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief A Vulkan 1.3 device with a VMA allocator and a graphics queue.
             *
             * Selects a physical device (by interop UUID when requested, else by
             * preference), creates the logical device with dynamic-rendering and
             * synchronization2 enabled, and stands up a VMA allocator. Destruction
             * releases the allocator, device, and instance in order. Non-copyable:
             * it owns Vulkan handles.
             */
            class VulkanDevice final : public IRenderDevice
            {
                public:
                    /**
                     * @brief Brings up the Vulkan instance, device, and allocator.
                     * @param desc Selection request forwarded from create_render_device().
                     */
                    explicit VulkanDevice(const RenderDeviceDesc& desc);
                    ~VulkanDevice() override;

                    VulkanDevice(const VulkanDevice&) = delete;
                    VulkanDevice& operator=(const VulkanDevice&) = delete;

                    const DeviceInfo& info() const noexcept override { return info_; }
                    NativeDeviceHandles native_handles() const noexcept override;

                    /** @brief The Vulkan instance handle. */
                    VkInstance instance() const noexcept { return instance_.instance; }

                    /**
                     * @brief The presentation surface, or VK_NULL_HANDLE if headless.
                     * @return The VkSurfaceKHR created by the host's surface factory.
                     */
                    VkSurfaceKHR surface() const noexcept { return surface_; }

                    /** @brief The instance's API version, for ImGui/loader setup. */
                    std::uint32_t api_version() const noexcept { return instance_.api_version; }

                    /** @brief The selected physical device handle. */
                    VkPhysicalDevice physical_device() const noexcept { return device_.physical_device; }

                    /** @brief The logical device handle. */
                    VkDevice device() const noexcept { return device_.device; }

                    /** @brief The VMA allocator bound to this device. */
                    VmaAllocator allocator() const noexcept { return allocator_; }

                    /** @brief The graphics-capable queue. */
                    VkQueue graphics_queue() const noexcept { return graphics_queue_; }

                    /** @brief The graphics queue family index. */
                    std::uint32_t graphics_queue_family() const noexcept { return graphics_queue_family_; }

                private:
                    vkb::Instance instance_{};
                    VkSurfaceKHR surface_ = VK_NULL_HANDLE;
                    vkb::Device device_{};
                    VmaAllocator allocator_ = VK_NULL_HANDLE;
                    VkQueue graphics_queue_ = VK_NULL_HANDLE;
                    std::uint32_t graphics_queue_family_ = 0;
                    DeviceInfo info_{};
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
