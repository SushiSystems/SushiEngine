/**************************************************************************/
/* vulkan_interop_buffer.cpp                                              */
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

/**
 * @file vulkan_interop_buffer.cpp
 * @brief The Vulkan half of the interop seam: an exportable device allocation.
 *
 * The one translation unit that pulls in the platform Vulkan header, which is why the
 * export lives here and not in VulkanDevice: reaching @c vkGetMemoryWin32HandleKHR means
 * reaching windows.h, and every file that includes the device would inherit it.
 *
 * The memory is allocated with vkAllocateMemory rather than through VMA. A suballocated
 * block cannot be exported on its own — the handle names the whole allocation — so an
 * interop buffer is deliberately one dedicated allocation, which is also what an importer
 * expects to bind against.
 */

#if defined(_WIN32)
#define VK_USE_PLATFORM_WIN32_KHR
#endif

#include <vulkan/vulkan.h>

#include <SushiEngine/render/interop.hpp>

#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace
        {
            /**
             * @brief Finds a memory type satisfying the requirements and the wanted flags.
             * @param physical     The physical device whose heaps are searched.
             * @param type_bits    The requirement mask from the buffer.
             * @param wanted       Property flags the type must carry.
             * @return The type index, or UINT32_MAX when none qualifies.
             */
            std::uint32_t find_memory_type(VkPhysicalDevice physical, std::uint32_t type_bits,
                                           VkMemoryPropertyFlags wanted) noexcept
            {
                VkPhysicalDeviceMemoryProperties properties{};
                vkGetPhysicalDeviceMemoryProperties(physical, &properties);
                for (std::uint32_t i = 0; i < properties.memoryTypeCount; ++i)
                {
                    if ((type_bits & (1u << i)) == 0)
                        continue;
                    if ((properties.memoryTypes[i].propertyFlags & wanted) == wanted)
                        return i;
                }
                return 0xFFFFFFFFu;
            }

            /**
             * @brief A dedicated, exportable device allocation with a buffer bound to it.
             *
             * Non-copyable: it owns the buffer, the allocation, and (on Win32) the handle.
             */
            class VulkanInteropBuffer final : public IInteropBuffer
            {
                public:
                    /**
                     * @brief Allocates the buffer and exports its memory.
                     * @param device The live Vulkan device.
                     * @param desc   Size and usage the buffer must satisfy.
                     */
                    VulkanInteropBuffer(Vulkan::VulkanDevice& device,
                                        const InteropBufferDesc& desc)
                        : device_(device), size_(desc.size_bytes)
                    {
                        // The buffer must be told at creation that its memory will be
                        // external; a driver may lay it out differently, so this cannot be
                        // decided after the fact.
                        VkExternalMemoryBufferCreateInfo external{};
                        external.sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_BUFFER_CREATE_INFO;
                        external.handleTypes = HANDLE_TYPE;

                        VkBufferCreateInfo buffer_info{};
                        buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                        buffer_info.pNext = &external;
                        buffer_info.size = desc.size_bytes;
                        buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                            VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                            VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                        if (desc.device_address)
                            buffer_info.usage |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
                        // Simulation output is written by one API and read by the other, so
                        // the graphics and compute families both address it.
                        device.share_across_queues(buffer_info);
                        if (vkCreateBuffer(device.device(), &buffer_info, nullptr, &buffer_) !=
                            VK_SUCCESS)
                            return;

                        VkMemoryRequirements requirements{};
                        vkGetBufferMemoryRequirements(device.device(), buffer_, &requirements);
                        const std::uint32_t type =
                            find_memory_type(device.physical_device(), requirements.memoryTypeBits,
                                             VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
                        if (type == 0xFFFFFFFFu)
                            return;

                        // Dedicated, because the handle names the whole allocation: an
                        // importer that mapped a suballocated block would address memory the
                        // renderer is using for something else.
                        VkMemoryDedicatedAllocateInfo dedicated{};
                        dedicated.sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO;
                        dedicated.buffer = buffer_;

                        VkExportMemoryAllocateInfo export_info{};
                        export_info.sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO;
                        export_info.handleTypes = HANDLE_TYPE;
                        export_info.pNext = &dedicated;

                        VkMemoryAllocateFlagsInfo flags{};
                        flags.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_FLAGS_INFO;
                        flags.flags = desc.device_address
                                          ? VK_MEMORY_ALLOCATE_DEVICE_ADDRESS_BIT
                                          : 0;
                        flags.pNext = &export_info;

                        VkMemoryAllocateInfo allocate{};
                        allocate.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
                        allocate.pNext = &flags;
                        allocate.allocationSize = requirements.size;
                        allocate.memoryTypeIndex = type;
                        if (vkAllocateMemory(device.device(), &allocate, nullptr, &memory_) !=
                            VK_SUCCESS)
                            return;
                        if (vkBindBufferMemory(device.device(), buffer_, memory_, 0) != VK_SUCCESS)
                            return;

                        handle_.allocation_size = requirements.size;
                        handle_.memory_offset = 0;
                        handle_.device_uuid = device.info().uuid;
                        export_handle();
                    }

                    ~VulkanInteropBuffer() override
                    {
#if defined(_WIN32)
                        // The Win32 handle is owned by this object, unlike the POSIX
                        // descriptor, which the importer takes over.
                        if (exported_ != nullptr)
                            CloseHandle(exported_);
#endif
                        if (buffer_ != VK_NULL_HANDLE)
                            vkDestroyBuffer(device_.device(), buffer_, nullptr);
                        if (memory_ != VK_NULL_HANDLE)
                            vkFreeMemory(device_.device(), memory_, nullptr);
                    }

                    VulkanInteropBuffer(const VulkanInteropBuffer&) = delete;
                    VulkanInteropBuffer& operator=(const VulkanInteropBuffer&) = delete;

                    /** @brief Whether the buffer, its allocation, and its export all succeeded. */
                    bool valid() const noexcept
                    {
                        return buffer_ != VK_NULL_HANDLE && memory_ != VK_NULL_HANDLE &&
                               handle_.kind != InteropHandleKind::None;
                    }

                    std::uint64_t size_bytes() const noexcept override { return size_; }

                    InteropMemoryHandle memory_handle() const noexcept override
                    {
                        return handle_;
                    }

                    std::uint64_t native_buffer() const noexcept override
                    {
                        return reinterpret_cast<std::uint64_t>(buffer_);
                    }

                private:
#if defined(_WIN32)
                    static constexpr VkExternalMemoryHandleTypeFlagBits HANDLE_TYPE =
                        VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_WIN32_BIT;
#else
                    static constexpr VkExternalMemoryHandleTypeFlagBits HANDLE_TYPE =
                        VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT;
#endif

                    /** @brief Resolves the platform getter and pulls the handle out. */
                    void export_handle()
                    {
#if defined(_WIN32)
                        auto get_handle = reinterpret_cast<PFN_vkGetMemoryWin32HandleKHR>(
                            vkGetDeviceProcAddr(device_.device(), "vkGetMemoryWin32HandleKHR"));
                        if (get_handle == nullptr)
                            return;
                        VkMemoryGetWin32HandleInfoKHR info{};
                        info.sType = VK_STRUCTURE_TYPE_MEMORY_GET_WIN32_HANDLE_INFO_KHR;
                        info.memory = memory_;
                        info.handleType = HANDLE_TYPE;
                        if (get_handle(device_.device(), &info, &exported_) != VK_SUCCESS)
                            return;
                        handle_.kind = InteropHandleKind::OpaqueWin32;
                        handle_.value = reinterpret_cast<std::uint64_t>(exported_);
#else
                        auto get_handle = reinterpret_cast<PFN_vkGetMemoryFdKHR>(
                            vkGetDeviceProcAddr(device_.device(), "vkGetMemoryFdKHR"));
                        if (get_handle == nullptr)
                            return;
                        VkMemoryGetFdInfoKHR info{};
                        info.sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR;
                        info.memory = memory_;
                        info.handleType = HANDLE_TYPE;
                        int descriptor = -1;
                        if (get_handle(device_.device(), &info, &descriptor) != VK_SUCCESS)
                            return;
                        handle_.kind = InteropHandleKind::OpaqueFd;
                        handle_.value = static_cast<std::uint64_t>(descriptor);
#endif
                    }

                    Vulkan::VulkanDevice& device_;
                    std::uint64_t size_ = 0;
                    VkBuffer buffer_ = VK_NULL_HANDLE;
                    VkDeviceMemory memory_ = VK_NULL_HANDLE;
                    InteropMemoryHandle handle_{};
#if defined(_WIN32)
                    HANDLE exported_ = nullptr;
#endif
            };
        } // namespace

        std::unique_ptr<IInteropBuffer> create_interop_buffer(IRenderDevice& device,
                                                              const InteropBufferDesc& desc)
        {
            Vulkan::VulkanDevice& vulkan = static_cast<Vulkan::VulkanDevice&>(device);
            if (!vulkan.supports_external_memory() || desc.size_bytes == 0)
                return nullptr;

            std::unique_ptr<VulkanInteropBuffer> buffer(new VulkanInteropBuffer(vulkan, desc));
            if (!buffer->valid())
                return nullptr;
            return std::unique_ptr<IInteropBuffer>(buffer.release());
        }
    } // namespace Render
} // namespace SushiEngine
