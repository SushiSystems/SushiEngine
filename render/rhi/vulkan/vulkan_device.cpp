/**************************************************************************/
/* vulkan_device.cpp                                                     */
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

#include "vulkan_device.hpp"

#include <cstring>
#include <stdexcept>
#include <string>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /**
                 * @brief Reads the physical device's identity into a DeviceInfo.
                 *
                 * Uses VkPhysicalDeviceIDProperties for the UUID (the interop match
                 * key) and the base properties for the name and device type.
                 *
                 * @param physical The selected physical device.
                 * @return The filled identity record.
                 */
                DeviceInfo read_device_info(VkPhysicalDevice physical)
                {
                    VkPhysicalDeviceIDProperties id_properties{};
                    id_properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_ID_PROPERTIES;

                    VkPhysicalDeviceProperties2 properties{};
                    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
                    properties.pNext = &id_properties;
                    vkGetPhysicalDeviceProperties2(physical, &properties);

                    DeviceInfo info;
                    info.name = properties.properties.deviceName;
                    info.api_version = properties.properties.apiVersion;
                    info.is_discrete =
                        properties.properties.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU;
                    std::memcpy(info.uuid.data(), id_properties.deviceUUID, info.uuid.size());
                    return info;
                }
            } // namespace

            VulkanDevice::VulkanDevice(const RenderDeviceDesc& desc)
            {
                vkb::InstanceBuilder instance_builder;
                instance_builder.set_app_name("SushiEngine")
                    .set_engine_name("SushiEngine")
                    .require_api_version(1, 3, 0);
                if (desc.enable_validation)
                    instance_builder.request_validation_layers().use_default_debug_messenger();
                for (const std::string& extension : desc.required_instance_extensions)
                    instance_builder.enable_extension(extension.c_str());

                auto instance_result = instance_builder.build();
                if (!instance_result)
                    throw std::runtime_error("SushiEngine: Vulkan instance creation failed: " +
                                             instance_result.error().message());
                instance_ = instance_result.value();

                // Windowed devices are handed a surface by the host so selection can
                // require a present-capable queue; headless devices (render_probe)
                // supply no factory and defer surface initialization.
                if (desc.surface_factory)
                {
                    surface_ = reinterpret_cast<VkSurfaceKHR>(
                        desc.surface_factory(reinterpret_cast<std::uint64_t>(instance_.instance)));
                    if (surface_ == VK_NULL_HANDLE)
                        throw std::runtime_error(
                            "SushiEngine: the surface factory returned no surface");
                }

                // Vulkan 1.3 core features the renderer depends on: dynamic rendering
                // removes render-pass/framebuffer objects, synchronization2 gives the
                // cleaner barrier API. Both are required so device selection rejects a
                // GPU that cannot provide them.
                VkPhysicalDeviceVulkan13Features features_13{};
                features_13.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_VULKAN_1_3_FEATURES;
                features_13.dynamicRendering = VK_TRUE;
                features_13.synchronization2 = VK_TRUE;

                vkb::PhysicalDeviceSelector selector(instance_);
                selector.set_minimum_version(1, 3)
                    .set_required_features_13(features_13);
                if (surface_ != VK_NULL_HANDLE)
                    selector.set_surface(surface_); // require a present-capable device
                else
                    selector.defer_surface_initialization(); // headless bring-up
                if (desc.preference == DevicePreference::LowPower)
                    selector.prefer_gpu_device_type(vkb::PreferredDeviceType::integrated);

                auto physical_result = selector.select();
                if (!physical_result)
                    throw std::runtime_error("SushiEngine: no suitable Vulkan device: " +
                                             physical_result.error().message());
                vkb::PhysicalDevice physical = physical_result.value();

                vkb::DeviceBuilder device_builder(physical);
                auto device_result = device_builder.build();
                if (!device_result)
                    throw std::runtime_error("SushiEngine: Vulkan device creation failed: " +
                                             device_result.error().message());
                device_ = device_result.value();

                auto queue_result = device_.get_queue(vkb::QueueType::graphics);
                if (!queue_result)
                    throw std::runtime_error("SushiEngine: no graphics queue: " +
                                             queue_result.error().message());
                graphics_queue_ = queue_result.value();
                graphics_queue_family_ = device_.get_queue_index(vkb::QueueType::graphics).value();

                VmaAllocatorCreateInfo allocator_info{};
                allocator_info.instance = instance_.instance;
                allocator_info.physicalDevice = device_.physical_device;
                allocator_info.device = device_.device;
                allocator_info.vulkanApiVersion = VK_API_VERSION_1_3;
                if (vmaCreateAllocator(&allocator_info, &allocator_) != VK_SUCCESS)
                    throw std::runtime_error("SushiEngine: VMA allocator creation failed");

                info_ = read_device_info(device_.physical_device);
            }

            VulkanDevice::~VulkanDevice()
            {
                if (allocator_ != VK_NULL_HANDLE)
                    vmaDestroyAllocator(allocator_);
                if (device_.device != VK_NULL_HANDLE)
                    vkb::destroy_device(device_);
                if (surface_ != VK_NULL_HANDLE)
                    vkb::destroy_surface(instance_, surface_);
                if (instance_.instance != VK_NULL_HANDLE)
                    vkb::destroy_instance(instance_);
            }

            NativeDeviceHandles VulkanDevice::native_handles() const noexcept
            {
                NativeDeviceHandles handles;
                handles.instance = instance_.instance;
                handles.physical_device = device_.physical_device;
                handles.device = device_.device;
                handles.graphics_queue = graphics_queue_;
                handles.graphics_queue_family = graphics_queue_family_;
                return handles;
            }
        } // namespace Vulkan

        std::unique_ptr<IRenderDevice> create_render_device(const RenderDeviceDesc& desc)
        {
            return std::unique_ptr<IRenderDevice>(new Vulkan::VulkanDevice(desc));
        }
    } // namespace Render
} // namespace SushiEngine
