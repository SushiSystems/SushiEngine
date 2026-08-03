/**************************************************************************/
/* tile_cache.cpp                                                         */
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

#include "terrain/tile_cache.hpp"

#include <cmath>
#include <cstring>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Terrain
        {
            namespace
            {
                using SushiEngine::Terrain::TILE_SAMPLE_COUNT;
                using SushiEngine::Terrain::TILE_STRIDE;

                /** @brief Bytes one tile's normalized channel occupies. */
                constexpr VkDeviceSize TILE_PAYLOAD_BYTES =
                    static_cast<VkDeviceSize>(TILE_SAMPLE_COUNT) * sizeof(std::uint16_t);
            } // namespace

            TileCache::TileCache(Vulkan::VulkanDevice& device, std::uint32_t slot_count)
                // The staging ring covers TERRAIN_FRAME_SLOTS frames, so that is exactly how
                // many frames may still be reading a slot when eviction looks for a victim.
                : device_(device), residency_(slot_count, TERRAIN_FRAME_SLOTS)
            {
                description_.width = TILE_STRIDE;
                description_.height = TILE_STRIDE;
                description_.depth = 1;
                description_.mip_levels = 1;
                description_.array_layers = slot_count;
                description_.format = TERRAIN_HEIGHT_FORMAT;
                description_.type = VK_IMAGE_TYPE_2D;
                description_.view_type = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
                description_.aspect = VK_IMAGE_ASPECT_COLOR_BIT;
                description_.usage =
                    VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                description_.name = "terrain height slots";

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = TERRAIN_HEIGHT_FORMAT;
                image_info.extent = {TILE_STRIDE, TILE_STRIDE, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = slot_count;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = description_.usage;
                image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                VmaAllocationCreateInfo image_alloc{};
                image_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &image_alloc,
                                             &image_, &allocation_, nullptr),
                              "vmaCreateImage(terrain height slots)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
                view_info.format = TERRAIN_HEIGHT_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = slot_count;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &view_),
                              "vkCreateImageView(terrain height slots)");

                // One staging buffer per frame in flight. Sharing one would let this frame's
                // writes land in a buffer an earlier frame's copy is still reading, which
                // corrupts the tile that copy was for rather than the one being written —
                // a failure that shows up two frames away from its cause.
                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = TILE_PAYLOAD_BYTES * TERRAIN_UPLOADS_PER_FRAME;
                buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

                VmaAllocationCreateInfo buffer_alloc{};
                buffer_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                buffer_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                     VMA_ALLOCATION_CREATE_MAPPED_BIT;
                for (std::uint32_t slot = 0; slot < TERRAIN_FRAME_SLOTS; ++slot)
                {
                    VmaAllocationInfo info{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info,
                                                  &buffer_alloc, &staging_[slot].buffer,
                                                  &staging_[slot].allocation, &info),
                                  "vmaCreateBuffer(terrain staging)");
                    staging_[slot].mapped = info.pMappedData;
                }

                pending_.reserve(TERRAIN_UPLOADS_PER_FRAME);
            }

            TileCache::~TileCache()
            {
                for (std::uint32_t slot = 0; slot < TERRAIN_FRAME_SLOTS; ++slot)
                {
                    if (staging_[slot].buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), staging_[slot].buffer,
                                         staging_[slot].allocation);
                }
                if (view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), view_, nullptr);
                if (image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), image_, allocation_);
            }

            void TileCache::begin_frame(std::uint64_t frame, std::uint32_t slot)
            {
                residency_.begin_frame(frame);
                frame_slot_ = slot % TERRAIN_FRAME_SLOTS;
                pending_.clear();
            }

            std::uint32_t TileCache::stage(const SushiEngine::Terrain::TileAddress& address,
                                           const float* heights_metres)
            {
                if (!can_stage())
                    return SushiEngine::Terrain::INVALID_TILE_SLOT;

                // The quantisation range covers every stored sample, apron included, so no
                // value clips; flooring and ceiling to whole metres keeps the bounds exactly
                // representable, so the range the node record carries and the range the
                // pixels were built against are the same number rather than nearly.
                float lowest = heights_metres[0];
                float highest = heights_metres[0];
                for (std::uint32_t index = 1; index < TILE_SAMPLE_COUNT; ++index)
                {
                    const float height = heights_metres[index];
                    lowest = height < lowest ? height : lowest;
                    highest = height > highest ? height : highest;
                }
                lowest = std::floor(lowest);
                highest = std::ceil(highest);

                const std::uint32_t slot = residency_.insert(address, lowest, highest);
                if (slot == SushiEngine::Terrain::INVALID_TILE_SLOT)
                    return slot;

                const VkDeviceSize offset =
                    static_cast<VkDeviceSize>(pending_.size()) * TILE_PAYLOAD_BYTES;
                std::uint16_t* destination = reinterpret_cast<std::uint16_t*>(
                    static_cast<std::uint8_t*>(staging_[frame_slot_].mapped) + offset);

                const double span = static_cast<double>(highest) - static_cast<double>(lowest);
                const double scale = span > 0.0 ? 65535.0 / span : 0.0;
                for (std::uint32_t index = 0; index < TILE_SAMPLE_COUNT; ++index)
                {
                    const double normalized =
                        (static_cast<double>(heights_metres[index]) -
                         static_cast<double>(lowest)) * scale;
                    const double clamped =
                        normalized < 0.0 ? 0.0 : (normalized > 65535.0 ? 65535.0 : normalized);
                    destination[index] = static_cast<std::uint16_t>(clamped + 0.5);
                }

                pending_.push_back(PendingUpload{offset, slot});
                return slot;
            }

            void TileCache::record_uploads(VkCommandBuffer command)
            {
                if (pending_.empty())
                    return;

                // One region per queued tile, all from the same buffer into the same image:
                // the graph has already put the image in TRANSFER_DST, because the pass that
                // calls this declared it that way.
                std::vector<VkBufferImageCopy> regions;
                regions.reserve(pending_.size());
                for (const PendingUpload& upload : pending_)
                {
                    VkBufferImageCopy region{};
                    region.bufferOffset = upload.offset;
                    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    region.imageSubresource.mipLevel = 0;
                    region.imageSubresource.baseArrayLayer = upload.slot;
                    region.imageSubresource.layerCount = 1;
                    region.imageExtent = {TILE_STRIDE, TILE_STRIDE, 1};
                    regions.push_back(region);
                }
                vkCmdCopyBufferToImage(command, staging_[frame_slot_].buffer, image_,
                                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                       static_cast<std::uint32_t>(regions.size()),
                                       regions.data());
            }

            std::size_t TileCache::image_bytes() const noexcept
            {
                return static_cast<std::size_t>(TILE_PAYLOAD_BYTES) *
                       static_cast<std::size_t>(residency_.capacity());
            }
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
