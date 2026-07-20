/**************************************************************************/
/* transient_pool.cpp                                                     */
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

#include "resources/transient_pool.hpp"

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            using Vulkan::check;

            TexturePool::TexturePool(Vulkan::VulkanDevice& device) : device_(device) {}

            TexturePool::~TexturePool() { clear(); }

            void TexturePool::begin_frame()
            {
                for (std::size_t i = 0; i < entries_.size();)
                {
                    Entry& slot = entries_[i];
                    slot.in_use = false;
                    ++slot.frames_unused;
                    if (slot.frames_unused > RETIRE_FRAMES)
                    {
                        destroy(slot);
                        entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(i));
                        continue;
                    }
                    ++i;
                }
            }

            std::uint32_t TexturePool::acquire(const Graph::TextureDesc& desc)
            {
                for (std::size_t i = 0; i < entries_.size(); ++i)
                {
                    Entry& slot = entries_[i];
                    if (!slot.in_use && Graph::same_texture_desc(slot.desc, desc))
                    {
                        slot.in_use = true;
                        slot.frames_unused = 0;
                        return static_cast<std::uint32_t>(i);
                    }
                }

                Entry slot;
                slot.desc = desc;
                slot.in_use = true;
                slot.frames_unused = 0;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = desc.type;
                image_info.format = desc.format;
                image_info.extent = {desc.width, desc.height, desc.depth};
                image_info.mipLevels = desc.mip_levels;
                image_info.arrayLayers = desc.array_layers;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = desc.usage;
                image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &slot.image,
                                     &slot.allocation, nullptr),
                      "vmaCreateImage(transient)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = slot.image;
                view_info.viewType = desc.view_type;
                view_info.format = desc.format;
                view_info.subresourceRange.aspectMask = desc.aspect;
                view_info.subresourceRange.levelCount = desc.mip_levels;
                view_info.subresourceRange.layerCount = desc.array_layers;
                check(vkCreateImageView(device_.device(), &view_info, nullptr, &slot.view),
                      "vkCreateImageView(transient)");

                if ((desc.aspect & VK_IMAGE_ASPECT_DEPTH_BIT) != 0 &&
                    (desc.aspect & VK_IMAGE_ASPECT_STENCIL_BIT) != 0)
                {
                    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
                    check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                            &slot.sample_view),
                          "vkCreateImageView(transient depth sample)");
                }
                else
                {
                    slot.sample_view = slot.view;
                }

                entries_.push_back(slot);
                return static_cast<std::uint32_t>(entries_.size() - 1);
            }

            void TexturePool::release(std::uint32_t entry_index)
            {
                if (entry_index < entries_.size())
                    entries_[entry_index].in_use = false;
            }

            void TexturePool::clear()
            {
                for (Entry& slot : entries_)
                    destroy(slot);
                entries_.clear();
            }

            void TexturePool::destroy(Entry& slot)
            {
                if (slot.sample_view != VK_NULL_HANDLE && slot.sample_view != slot.view)
                    vkDestroyImageView(device_.device(), slot.sample_view, nullptr);
                if (slot.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), slot.view, nullptr);
                if (slot.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), slot.image, slot.allocation);
                slot.sample_view = VK_NULL_HANDLE;
                slot.view = VK_NULL_HANDLE;
                slot.image = VK_NULL_HANDLE;
                slot.allocation = VK_NULL_HANDLE;
            }

            BufferPool::BufferPool(Vulkan::VulkanDevice& device) : device_(device) {}

            BufferPool::~BufferPool() { clear(); }

            void BufferPool::begin_frame()
            {
                for (std::size_t i = 0; i < entries_.size();)
                {
                    Entry& slot = entries_[i];
                    slot.in_use = false;
                    ++slot.frames_unused;
                    if (slot.frames_unused > RETIRE_FRAMES)
                    {
                        destroy(slot);
                        entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(i));
                        continue;
                    }
                    ++i;
                }
            }

            std::uint32_t BufferPool::acquire(const Graph::BufferDesc& desc)
            {
                for (std::size_t i = 0; i < entries_.size(); ++i)
                {
                    Entry& slot = entries_[i];
                    if (!slot.in_use && Graph::same_buffer_desc(slot.desc, desc))
                    {
                        slot.in_use = true;
                        slot.frames_unused = 0;
                        return static_cast<std::uint32_t>(i);
                    }
                }

                Entry slot;
                slot.desc = desc;
                slot.in_use = true;
                slot.frames_unused = 0;

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = desc.size;
                buffer_info.usage = desc.usage;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = desc.host_visible ? VMA_MEMORY_USAGE_AUTO_PREFER_HOST
                                                : VMA_MEMORY_USAGE_AUTO;
                if (desc.host_visible)
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo allocation_info{};
                check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc, &slot.buffer,
                                      &slot.allocation, &allocation_info),
                      "vmaCreateBuffer(transient)");
                slot.mapped = desc.host_visible ? allocation_info.pMappedData : nullptr;

                entries_.push_back(slot);
                return static_cast<std::uint32_t>(entries_.size() - 1);
            }

            void BufferPool::release(std::uint32_t entry_index)
            {
                if (entry_index < entries_.size())
                    entries_[entry_index].in_use = false;
            }

            void BufferPool::clear()
            {
                for (Entry& slot : entries_)
                    destroy(slot);
                entries_.clear();
            }

            void BufferPool::destroy(Entry& slot)
            {
                if (slot.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.buffer, slot.allocation);
                slot.buffer = VK_NULL_HANDLE;
                slot.allocation = VK_NULL_HANDLE;
                slot.mapped = nullptr;
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
