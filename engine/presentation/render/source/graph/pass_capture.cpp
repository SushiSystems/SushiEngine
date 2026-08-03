/**************************************************************************/
/* pass_capture.cpp                                                       */
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

#include "graph/pass_capture.hpp"

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            namespace
            {
                /** @brief Every copy starts on a multiple of this, as the spec requires. */
                constexpr VkDeviceSize COPY_ALIGNMENT = 16;

                /**
                 * @brief Bytes one texel of an uncompressed format occupies, or zero.
                 *
                 * Deliberately not exhaustive and deliberately not clever: a format absent
                 * from this table is *reported* as un-copyable rather than guessed at, so
                 * the failure mode of an unlisted format is a missing hash the caller is
                 * told about, never a hash computed over the wrong number of bytes.
                 *
                 * Compressed and planar formats are absent on purpose. A block-compressed
                 * image copies by block, not by texel, and no pass in this renderer writes
                 * one.
                 */
                std::uint32_t texel_size(VkFormat format) noexcept
                {
                    switch (format)
                    {
                        case VK_FORMAT_R8_UNORM:
                        case VK_FORMAT_R8_SNORM:
                        case VK_FORMAT_R8_UINT:
                        case VK_FORMAT_R8_SINT:
                        case VK_FORMAT_S8_UINT:
                            return 1;
                        case VK_FORMAT_R8G8_UNORM:
                        case VK_FORMAT_R8G8_SNORM:
                        case VK_FORMAT_R8G8_UINT:
                        case VK_FORMAT_R16_UNORM:
                        case VK_FORMAT_R16_SNORM:
                        case VK_FORMAT_R16_UINT:
                        case VK_FORMAT_R16_SINT:
                        case VK_FORMAT_R16_SFLOAT:
                        case VK_FORMAT_D16_UNORM:
                            return 2;
                        case VK_FORMAT_R8G8B8A8_UNORM:
                        case VK_FORMAT_R8G8B8A8_SRGB:
                        case VK_FORMAT_R8G8B8A8_SNORM:
                        case VK_FORMAT_R8G8B8A8_UINT:
                        case VK_FORMAT_B8G8R8A8_UNORM:
                        case VK_FORMAT_B8G8R8A8_SRGB:
                        case VK_FORMAT_A2B10G10R10_UNORM_PACK32:
                        case VK_FORMAT_A2R10G10B10_UNORM_PACK32:
                        case VK_FORMAT_B10G11R11_UFLOAT_PACK32:
                        case VK_FORMAT_E5B9G9R9_UFLOAT_PACK32:
                        case VK_FORMAT_R16G16_UNORM:
                        case VK_FORMAT_R16G16_SNORM:
                        case VK_FORMAT_R16G16_UINT:
                        case VK_FORMAT_R16G16_SFLOAT:
                        case VK_FORMAT_R32_UINT:
                        case VK_FORMAT_R32_SINT:
                        case VK_FORMAT_R32_SFLOAT:
                        case VK_FORMAT_D32_SFLOAT:
                        case VK_FORMAT_X8_D24_UNORM_PACK32:
                        case VK_FORMAT_D24_UNORM_S8_UINT:
                            return 4;
                        case VK_FORMAT_R16G16B16A16_UNORM:
                        case VK_FORMAT_R16G16B16A16_SNORM:
                        case VK_FORMAT_R16G16B16A16_UINT:
                        case VK_FORMAT_R16G16B16A16_SFLOAT:
                        case VK_FORMAT_R32G32_UINT:
                        case VK_FORMAT_R32G32_SFLOAT:
                            return 8;
                        case VK_FORMAT_R32G32B32A32_UINT:
                        case VK_FORMAT_R32G32B32A32_SINT:
                        case VK_FORMAT_R32G32B32A32_SFLOAT:
                            return 16;
                        default:
                            return 0;
                    }
                }

                /**
                 * @brief The single aspect a capture copies out of an image.
                 *
                 * One region, so one aspect. Depth wins over stencil wherever both are
                 * present: a depth regression is what a renderer's passes actually produce,
                 * and a combined format cannot copy both in one region anyway.
                 */
                VkImageAspectFlags capture_aspect(const TextureDescription& description) noexcept
                {
                    if ((description.aspect & VK_IMAGE_ASPECT_DEPTH_BIT) != 0)
                        return VK_IMAGE_ASPECT_DEPTH_BIT;
                    if ((description.aspect & VK_IMAGE_ASPECT_STENCIL_BIT) != 0)
                        return VK_IMAGE_ASPECT_STENCIL_BIT;
                    return VK_IMAGE_ASPECT_COLOR_BIT;
                }

                /** @brief Bytes mip 0 of @p description occupies, across every layer and slice. */
                VkDeviceSize capture_size(const TextureDescription& description) noexcept
                {
                    // A combined depth/stencil image copies its depth aspect only, and for
                    // the two such formats this renderer can meet, that aspect packs
                    // tightly at 4 bytes per texel (the VK_FORMAT_D32_SFLOAT / _UINT
                    // component size, per the VkBufferImageCopy spec for combined
                    // formats) even though the format as a whole is not a fixed-size
                    // texel format texel_size() can describe — checked ahead of that
                    // table rather than folded into it, so texel_size() keeps meaning
                    // "whole texel size" for every format it does list.
                    const bool depth_only_combined =
                        (description.format == VK_FORMAT_D24_UNORM_S8_UINT ||
                         description.format == VK_FORMAT_D32_SFLOAT_S8_UINT) &&
                        capture_aspect(description) == VK_IMAGE_ASPECT_DEPTH_BIT;
                    const std::uint32_t stride =
                        depth_only_combined ? 4 : texel_size(description.format);
                    if (stride == 0)
                        return 0;
                    return VkDeviceSize(stride) * description.width * description.height *
                           description.depth * description.array_layers;
                }

                /** @brief FNV-1a 64 over a byte range. */
                std::uint64_t hash_bytes(const std::uint8_t* data, std::size_t size) noexcept
                {
                    std::uint64_t hash = 1469598103934665603ull;
                    for (std::size_t i = 0; i < size; ++i)
                    {
                        hash ^= data[i];
                        hash *= 1099511628211ull;
                    }
                    return hash;
                }
            } // namespace

            PassCapture::PassCapture(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                     VkDeviceSize budget)
                : device_(device), budget_(budget)
            {
                stores_.resize(frame_slots == 0 ? 1 : frame_slots);
            }

            bool PassCapture::ensure_allocated(Store& store)
            {
                if (store.buffer != VK_NULL_HANDLE)
                    return true;

                VkBufferCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                info.size = budget_;
                info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

                VmaAllocationCreateInfo allocation{};
                allocation.usage = VMA_MEMORY_USAGE_AUTO;
                allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                   VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo mapping{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &info, &allocation,
                                              &store.buffer, &store.allocation, &mapping),
                              "vmaCreateBuffer(pass capture)");
                store.mapped = mapping.pMappedData;
                return store.mapped != nullptr;
            }

            PassCapture::~PassCapture()
            {
                for (Store& store : stores_)
                    if (store.buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), store.buffer, store.allocation);
            }

            void PassCapture::begin_frame(std::uint32_t slot)
            {
                pending_size_ = 0;
                active_ = 0xFFFFFFFFu;
                if (slot >= stores_.size())
                    return;

                Store& store = stores_[slot];
                if (!ensure_allocated(store))
                    return;

                active_ = slot;
                store.cursor = 0;
                store.recorded = true;
                store.dropped_budget = 0;
                store.dropped_format = 0;
                store.entries.clear();
            }

            bool PassCapture::wants(const TextureDescription& description)
            {
                pending_size_ = 0;
                if (active_ >= stores_.size())
                    return false;
                if (description.width == 0 || description.height == 0 || description.depth == 0 ||
                    description.array_layers == 0)
                    return false;
                // An import the graph does not own may simply not be a legal copy source,
                // and nothing here can retroactively give it that usage.
                Store& store = stores_[active_];
                if ((description.usage & VK_IMAGE_USAGE_TRANSFER_SRC_BIT) == 0)
                {
                    ++store.dropped_format;
                    return false;
                }

                const VkDeviceSize size = capture_size(description);
                if (size == 0)
                {
                    ++store.dropped_format;
                    return false;
                }

                const VkDeviceSize offset =
                    (store.cursor + COPY_ALIGNMENT - 1) / COPY_ALIGNMENT * COPY_ALIGNMENT;
                if (offset + size > budget_)
                {
                    ++store.dropped_budget;
                    return false;
                }

                pending_size_ = size;
                return true;
            }

            void PassCapture::record(VkCommandBuffer command, const char* pass,
                                     const TextureDescription& description, VkImage image)
            {
                if (active_ >= stores_.size() || pending_size_ == 0 || image == VK_NULL_HANDLE)
                    return;
                Store& store = stores_[active_];
                const VkDeviceSize offset =
                    (store.cursor + COPY_ALIGNMENT - 1) / COPY_ALIGNMENT * COPY_ALIGNMENT;
                const VkDeviceSize size = pending_size_;
                pending_size_ = 0;

                VkBufferImageCopy copy{};
                copy.bufferOffset = offset;
                copy.imageSubresource.aspectMask = capture_aspect(description);
                copy.imageSubresource.mipLevel = 0;
                copy.imageSubresource.baseArrayLayer = 0;
                copy.imageSubresource.layerCount = description.array_layers;
                copy.imageExtent = {description.width, description.height, description.depth};
                vkCmdCopyImageToBuffer(command, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                       store.buffer, 1, &copy);

                Entry entry;
                entry.pass = pass != nullptr ? pass : "pass";
                entry.resource = description.name != nullptr ? description.name : "texture";
                entry.description = description;
                entry.offset = offset;
                entry.size = size;
                store.entries.push_back(std::move(entry));
                store.cursor = offset + size;
            }

            bool PassCapture::resolve(std::uint32_t slot, std::vector<CapturedPass>& out) const
            {
                if (slot >= stores_.size())
                    return false;
                const Store& store = stores_[slot];
                if (!store.recorded || store.mapped == nullptr)
                    return false;

                if (store.cursor > 0)
                    vmaInvalidateAllocation(device_.allocator(), store.allocation, 0,
                                            store.cursor);

                out.clear();
                out.reserve(store.entries.size());
                const std::uint8_t* base = static_cast<const std::uint8_t*>(store.mapped);
                for (const Entry& entry : store.entries)
                {
                    CapturedPass captured;
                    captured.pass = entry.pass;
                    captured.resource = entry.resource;
                    captured.width = entry.description.width;
                    captured.height = entry.description.height;
                    captured.depth = entry.description.depth;
                    captured.layers = entry.description.array_layers;
                    captured.format = entry.description.format;
                    captured.hash = hash_bytes(base + entry.offset,
                                               static_cast<std::size_t>(entry.size));
                    out.push_back(std::move(captured));
                }
                return true;
            }

            std::uint32_t PassCapture::dropped_by_budget(std::uint32_t slot) const noexcept
            {
                return slot < stores_.size() ? stores_[slot].dropped_budget : 0;
            }

            std::uint32_t PassCapture::dropped_by_format(std::uint32_t slot) const noexcept
            {
                return slot < stores_.size() ? stores_[slot].dropped_format : 0;
            }

            VkDeviceSize PassCapture::bytes_used(std::uint32_t slot) const noexcept
            {
                return slot < stores_.size() ? stores_[slot].cursor : 0;
            }
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
