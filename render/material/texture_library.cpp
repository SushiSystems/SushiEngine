/**************************************************************************/
/* texture_library.cpp                                                    */
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

#include "material/texture_library.hpp"

#include <algorithm>
#include <cstring>

#include <stb_image.h>

#include "resources/descriptor_heap.hpp"
#include "resources/sampler_cache.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            namespace
            {
                /** @brief Bytes one RGBA8 texel occupies. */
                constexpr std::size_t TEXEL_BYTES = 4;

                /**
                 * @brief Frames a host-copy-replaced image is held before release.
                 *
                 * The synchronous host path has no upload fence, so a superseded image is
                 * kept this many frames — comfortably past the double-buffered frames in
                 * flight — before it and its heap slot are reclaimed.
                 */
                constexpr std::uint64_t RETIRE_FRAME_DELAY = 3;

                /**
                 * @brief Bytes a full RGBA8 mip chain of the given size occupies.
                 * @param width  Mip 0 width.
                 * @param height Mip 0 height.
                 * @return Device bytes, mip tail included.
                 */
                std::size_t chain_bytes(std::uint32_t width, std::uint32_t height)
                {
                    std::size_t total = 0;
                    while (width > 0 && height > 0)
                    {
                        total += std::size_t(width) * height * TEXEL_BYTES;
                        if (width == 1 && height == 1)
                            break;
                        width = std::max(1u, width / 2);
                        height = std::max(1u, height / 2);
                    }
                    return total;
                }

                /**
                 * @brief Number of mip levels a texture of this size carries.
                 * @param width  Mip 0 width.
                 * @param height Mip 0 height.
                 * @return Levels down to 1x1, at least 1.
                 */
                std::uint32_t mip_count(std::uint32_t width, std::uint32_t height)
                {
                    std::uint32_t levels = 1;
                    while (width > 1 || height > 1)
                    {
                        width = std::max(1u, width / 2);
                        height = std::max(1u, height / 2);
                        ++levels;
                    }
                    return levels;
                }

                /**
                 * @brief Box-filters an RGBA8 image down by one power of two.
                 * @param source       Source texels.
                 * @param width        Source width.
                 * @param height       Source height.
                 * @param target_width Receives the halved width.
                 * @param target_height Receives the halved height.
                 * @return The downsampled texels.
                 */
                std::vector<std::uint8_t> halve(const std::uint8_t* source, std::uint32_t width,
                                                std::uint32_t height, std::uint32_t& target_width,
                                                std::uint32_t& target_height)
                {
                    target_width = std::max(1u, width / 2);
                    target_height = std::max(1u, height / 2);
                    std::vector<std::uint8_t> output(std::size_t(target_width) * target_height *
                                                     TEXEL_BYTES);
                    for (std::uint32_t y = 0; y < target_height; ++y)
                        for (std::uint32_t x = 0; x < target_width; ++x)
                        {
                            const std::uint32_t x0 = std::min(x * 2, width - 1);
                            const std::uint32_t x1 = std::min(x * 2 + 1, width - 1);
                            const std::uint32_t y0 = std::min(y * 2, height - 1);
                            const std::uint32_t y1 = std::min(y * 2 + 1, height - 1);
                            for (std::size_t channel = 0; channel < TEXEL_BYTES; ++channel)
                            {
                                const std::uint32_t sum =
                                    source[(std::size_t(y0) * width + x0) * TEXEL_BYTES + channel] +
                                    source[(std::size_t(y0) * width + x1) * TEXEL_BYTES + channel] +
                                    source[(std::size_t(y1) * width + x0) * TEXEL_BYTES + channel] +
                                    source[(std::size_t(y1) * width + x1) * TEXEL_BYTES + channel];
                                output[(std::size_t(y) * target_width + x) * TEXEL_BYTES + channel] =
                                    static_cast<std::uint8_t>(sum / 4);
                            }
                        }
                    return output;
                }
            } // namespace

            TextureLibrary::TextureLibrary(Vulkan::VulkanDevice& device,
                                           Resources::DescriptorHeap& heap,
                                           Resources::SamplerCache& samplers,
                                           std::size_t budget_bytes)
                : device_(device), heap_(heap), budget_bytes_(budget_bytes)
            {
                Resources::SamplerDesc desc;
                desc.address_mode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                desc.max_anisotropy = 16.0f;
                desc.max_lod = 16.0f;
                sampler_ = samplers.get(desc);

                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                Vulkan::check(vkCreateCommandPool(device_.device(), &pool_info, nullptr, &pool_),
                              "vkCreateCommandPool(textures)");

                // The host-copy path is taken only when the device offers host image copy
                // and lists SHADER_READ_ONLY among the layouts a host copy may target — so
                // the upload can land in its final sampling layout with no intermediate.
                if (device_.supports_host_image_copy())
                {
                    VkPhysicalDeviceHostImageCopyProperties host_copy{};
                    host_copy.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_HOST_IMAGE_COPY_PROPERTIES;
                    VkPhysicalDeviceProperties2 properties{};
                    properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
                    properties.pNext = &host_copy;
                    vkGetPhysicalDeviceProperties2(device_.physical_device(), &properties);

                    std::vector<VkImageLayout> destination_layouts(host_copy.copyDstLayoutCount);
                    host_copy.pCopyDstLayouts = destination_layouts.data();
                    vkGetPhysicalDeviceProperties2(device_.physical_device(), &properties);
                    for (VkImageLayout layout : destination_layouts)
                        if (layout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)
                            host_upload_ok_ = true;
                }

                defaults_[static_cast<std::uint32_t>(DefaultTexture::White)] =
                    create_default("sushi:white", 255, 255, 255, 255);
                defaults_[static_cast<std::uint32_t>(DefaultTexture::FlatNormal)] =
                    create_default("sushi:flat-normal", 128, 128, 255, 255);
                defaults_[static_cast<std::uint32_t>(DefaultTexture::Black)] =
                    create_default("sushi:black", 0, 0, 0, 0);
                defaults_[static_cast<std::uint32_t>(DefaultTexture::NeutralMaterial)] =
                    create_default("sushi:neutral-material", 255, 255, 255, 255);
            }

            TextureLibrary::~TextureLibrary()
            {
                vkDeviceWaitIdle(device_.device());
                collect_finished();
                for (Pending& pending : pending_)
                {
                    if (pending.fence != VK_NULL_HANDLE)
                        vkDestroyFence(device_.device(), pending.fence, nullptr);
                    if (pending.staging != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), pending.staging,
                                         pending.staging_allocation);
                    if (pending.retired_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), pending.retired_view, nullptr);
                    if (pending.retired_image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), pending.retired_image,
                                        pending.retired_allocation);
                }
                pending_.clear();
                for (Retired& retired : retired_)
                {
                    if (retired.view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), retired.view, nullptr);
                    if (retired.image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), retired.image, retired.allocation);
                }
                retired_.clear();
                for (Entry& entry : entries_)
                    destroy(entry);
                entries_.clear();
                if (pool_ != VK_NULL_HANDLE)
                    vkDestroyCommandPool(device_.device(), pool_, nullptr);
            }

            TextureId TextureLibrary::create_default(const char* name, std::uint8_t r,
                                                     std::uint8_t g, std::uint8_t b,
                                                     std::uint8_t a)
            {
                const std::uint8_t texel[4] = {r, g, b, a};
                return add(name, texel, 1, 1, TextureColorSpace::Linear);
            }

            TextureId TextureLibrary::find(const std::string& key) const
            {
                for (std::size_t i = 0; i < entries_.size(); ++i)
                    if (entries_[i].live && entries_[i].key == key)
                        return static_cast<TextureId>(i);
                return INVALID_TEXTURE;
            }

            std::uint32_t TextureLibrary::budget_scale(std::uint32_t width,
                                                       std::uint32_t height) const
            {
                // Drop whole mip levels until the texture fits what is left of the budget.
                // Returning a shift rather than a boolean is what makes residency
                // continuous: an over-budget texture still appears, just softer.
                std::uint32_t shift = 0;
                while (shift < 8)
                {
                    const std::uint32_t w = std::max(1u, width >> shift);
                    const std::uint32_t h = std::max(1u, height >> shift);
                    if (resident_bytes_ + chain_bytes(w, h) <= budget_bytes_)
                        break;
                    ++shift;
                }
                return shift;
            }

            TextureId TextureLibrary::add(const char* name, const std::uint8_t* pixels,
                                          std::uint32_t width, std::uint32_t height,
                                          TextureColorSpace color_space)
            {
                if (name == nullptr || pixels == nullptr || width == 0 || height == 0)
                    return INVALID_TEXTURE;

                const std::string key(name);
                const TextureId existing = find(key);
                if (existing != INVALID_TEXTURE)
                {
                    ++entries_[existing].references;
                    return existing;
                }

                Entry entry;
                entry.key = key;
                entry.color_space = color_space;
                entry.full_width = width;
                entry.full_height = height;
                entry.references = 1;
                entry.live = true;
                entry.pixels.assign(pixels, pixels + std::size_t(width) * height * TEXEL_BYTES);
                for (std::size_t i = 3; i < entry.pixels.size(); i += TEXEL_BYTES)
                    if (entry.pixels[i] != 255)
                    {
                        entry.transparent = true;
                        break;
                    }

                const std::uint32_t shift = budget_scale(width, height);
                std::vector<std::uint8_t> scaled(entry.pixels);
                std::uint32_t scaled_width = width;
                std::uint32_t scaled_height = height;
                for (std::uint32_t level = 0; level < shift; ++level)
                    scaled = halve(scaled.data(), scaled_width, scaled_height, scaled_width,
                                   scaled_height);

                if (!upload(entry, scaled.data(), scaled_width, scaled_height))
                    return INVALID_TEXTURE;

                entries_.push_back(std::move(entry));
                return static_cast<TextureId>(entries_.size() - 1);
            }

            TextureId TextureLibrary::load(const char* path, TextureColorSpace color_space)
            {
                if (path == nullptr || path[0] == '\0')
                    return INVALID_TEXTURE;

                const std::string key(path);
                const TextureId existing = find(key);
                if (existing != INVALID_TEXTURE)
                {
                    ++entries_[existing].references;
                    return existing;
                }

                int width = 0;
                int height = 0;
                int channels = 0;
                stbi_uc* decoded = stbi_load(path, &width, &height, &channels, 4);
                if (decoded == nullptr || width <= 0 || height <= 0)
                {
                    if (decoded != nullptr)
                        stbi_image_free(decoded);
                    return INVALID_TEXTURE;
                }

                const TextureId id = add(path, decoded, static_cast<std::uint32_t>(width),
                                         static_cast<std::uint32_t>(height), color_space);
                stbi_image_free(decoded);
                if (id != INVALID_TEXTURE)
                {
                    // The file is the source of truth for a streaming upgrade, so the
                    // decoded copy is dropped rather than held in host memory — a 4K map
                    // would otherwise cost 64 MB of RAM for the whole run.
                    entries_[id].from_file = true;
                    entries_[id].pixels.clear();
                    entries_[id].pixels.shrink_to_fit();
                }
                return id;
            }

            bool TextureLibrary::upload(Entry& entry, const std::uint8_t* pixels,
                                        std::uint32_t width, std::uint32_t height)
            {
                const VkFormat format = entry.color_space == TextureColorSpace::Srgb
                                            ? VK_FORMAT_R8G8B8A8_SRGB
                                            : VK_FORMAT_R8G8B8A8_UNORM;
                const std::uint32_t levels = mip_count(width, height);

                VkImage image = VK_NULL_HANDLE;
                VmaAllocation allocation = VK_NULL_HANDLE;
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = format;
                image_info.extent = {width, height, 1};
                image_info.mipLevels = levels;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                // Host copy needs only the host-transfer + sampled usage; the staging
                // fallback needs the queue transfer bits for the buffer copy and the mip
                // blit that reads and writes the image.
                image_info.usage =
                    host_upload_ok_
                        ? (VK_IMAGE_USAGE_HOST_TRANSFER_BIT | VK_IMAGE_USAGE_SAMPLED_BIT)
                        : (VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT |
                           VK_IMAGE_USAGE_SAMPLED_BIT);

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                if (vmaCreateImage(device_.allocator(), &image_info, &alloc, &image, &allocation,
                                   nullptr) != VK_SUCCESS)
                    return false;

                VkImageView view = VK_NULL_HANDLE;
                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = format;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = levels;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &view),
                              "vkCreateImageView(texture)");

                Pending pending;
                if (host_upload_ok_)
                    record_host_upload(image, pixels, width, height, levels);
                else if (!record_staging_upload(image, pixels, width, height, levels, pending))
                {
                    vkDestroyImageView(device_.device(), view, nullptr);
                    vmaDestroyImage(device_.allocator(), image, allocation);
                    return false;
                }

                // Snapshot the superseded resources before the entry adopts the new image.
                const VkImage retired_image = entry.image;
                const VmaAllocation retired_allocation = entry.allocation;
                const VkImageView retired_view = entry.view;
                const std::uint32_t retired_heap_index = entry.heap_index;
                const bool had_previous = retired_view != VK_NULL_HANDLE;
                if (entry.bytes > 0)
                    resident_bytes_ -= entry.bytes;

                entry.image = image;
                entry.allocation = allocation;
                entry.view = view;
                entry.resident_width = width;
                entry.resident_height = height;
                entry.mip_levels = levels;
                entry.bytes = chain_bytes(width, height);
                resident_bytes_ += entry.bytes;

                // A fresh heap slot rather than an in-place rewrite: the slot an in-flight
                // frame's material buffer points at must stay valid until that frame ends.
                entry.heap_index = heap_.allocate_texture(
                    view, sampler_, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

                if (host_upload_ok_)
                {
                    // Synchronous upload: the replaced image may still be sampled by frames
                    // already in flight, so its release waits out a frames-in-flight delay.
                    if (had_previous)
                        retired_.push_back({retired_image, retired_allocation, retired_view,
                                            retired_heap_index,
                                            frame_counter_ + RETIRE_FRAME_DELAY});
                }
                else
                {
                    // The staging upload's own fence guards the superseded image: it stays
                    // alive until the replacing upload completes, so a frame still sampling
                    // it is never left with a dangling view.
                    pending.retired_image = retired_image;
                    pending.retired_allocation = retired_allocation;
                    pending.retired_view = retired_view;
                    if (had_previous)
                        pending.retired_heap_index = retired_heap_index;
                    pending_.push_back(pending);
                }
                return true;
            }

            void TextureLibrary::record_host_upload(VkImage image, const std::uint8_t* pixels,
                                                    std::uint32_t width, std::uint32_t height,
                                                    std::uint32_t levels)
            {
                // One host-side transition to the final sampling layout, then a direct copy
                // of each mip from host memory — the chain is box-filtered on the CPU, so no
                // command buffer, queue submit, or fence is ever touched.
                VkHostImageLayoutTransitionInfo transition{};
                transition.sType = VK_STRUCTURE_TYPE_HOST_IMAGE_LAYOUT_TRANSITION_INFO;
                transition.image = image;
                transition.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                transition.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                transition.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                transition.subresourceRange.levelCount = levels;
                transition.subresourceRange.layerCount = 1;
                Vulkan::check(vkTransitionImageLayout(device_.device(), 1, &transition),
                              "vkTransitionImageLayout(texture)");

                std::vector<std::uint8_t> level_pixels(
                    pixels, pixels + std::size_t(width) * height * TEXEL_BYTES);
                std::uint32_t level_width = width;
                std::uint32_t level_height = height;
                for (std::uint32_t level = 0; level < levels; ++level)
                {
                    VkMemoryToImageCopy region{};
                    region.sType = VK_STRUCTURE_TYPE_MEMORY_TO_IMAGE_COPY;
                    region.pHostPointer = level_pixels.data();
                    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    region.imageSubresource.mipLevel = level;
                    region.imageSubresource.layerCount = 1;
                    region.imageExtent = {level_width, level_height, 1};

                    VkCopyMemoryToImageInfo copy{};
                    copy.sType = VK_STRUCTURE_TYPE_COPY_MEMORY_TO_IMAGE_INFO;
                    copy.dstImage = image;
                    copy.dstImageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                    copy.regionCount = 1;
                    copy.pRegions = &region;
                    Vulkan::check(vkCopyMemoryToImage(device_.device(), &copy),
                                  "vkCopyMemoryToImage(texture)");

                    if (level + 1 < levels)
                        level_pixels = halve(level_pixels.data(), level_width, level_height,
                                             level_width, level_height);
                }
            }

            bool TextureLibrary::record_staging_upload(VkImage image, const std::uint8_t* pixels,
                                                       std::uint32_t width, std::uint32_t height,
                                                       std::uint32_t levels, Pending& pending)
            {
                const VkDeviceSize staging_bytes = VkDeviceSize(width) * height * TEXEL_BYTES;
                VkBufferCreateInfo staging_info{};
                staging_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                staging_info.size = staging_bytes;
                staging_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                VmaAllocationCreateInfo staging_alloc{};
                staging_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                staging_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                      VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo staging_mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &staging_info, &staging_alloc,
                                              &pending.staging, &pending.staging_allocation,
                                              &staging_mapped),
                              "vmaCreateBuffer(texture staging)");
                std::memcpy(staging_mapped.pMappedData, pixels,
                            static_cast<std::size_t>(staging_bytes));

                VkCommandBufferAllocateInfo cmd_info{};
                cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                cmd_info.commandPool = pool_;
                cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmd_info.commandBufferCount = 1;
                Vulkan::check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &pending.cmd),
                              "vkAllocateCommandBuffers(texture)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(pending.cmd, &begin);

                const auto barrier = [&](std::uint32_t base_level, std::uint32_t level_count,
                                         VkImageLayout from, VkImageLayout to,
                                         VkPipelineStageFlags2 source_stage,
                                         VkPipelineStageFlags2 destination_stage,
                                         VkAccessFlags2 source_access,
                                         VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 image_barrier{};
                    image_barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    image_barrier.srcStageMask = source_stage;
                    image_barrier.srcAccessMask = source_access;
                    image_barrier.dstStageMask = destination_stage;
                    image_barrier.dstAccessMask = destination_access;
                    image_barrier.oldLayout = from;
                    image_barrier.newLayout = to;
                    image_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    image_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    image_barrier.image = image;
                    image_barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    image_barrier.subresourceRange.baseMipLevel = base_level;
                    image_barrier.subresourceRange.levelCount = level_count;
                    image_barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &image_barrier;
                    vkCmdPipelineBarrier2(pending.cmd, &dependency);
                };

                barrier(0, levels, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_2_COPY_BIT, 0,
                        VK_ACCESS_2_TRANSFER_WRITE_BIT);

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {width, height, 1};
                vkCmdCopyBufferToImage(pending.cmd, pending.staging, image,
                                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

                // Mip chain by successive halving blits: level N-1 is the source for N,
                // so each level is transitioned to TRANSFER_SRC as soon as it is complete.
                std::uint32_t source_width = width;
                std::uint32_t source_height = height;
                for (std::uint32_t level = 1; level < levels; ++level)
                {
                    barrier(level - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                            VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT,
                            VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                            VK_ACCESS_2_TRANSFER_READ_BIT);

                    const std::uint32_t target_width = std::max(1u, source_width / 2);
                    const std::uint32_t target_height = std::max(1u, source_height / 2);
                    VkImageBlit blit{};
                    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    blit.srcSubresource.mipLevel = level - 1;
                    blit.srcSubresource.layerCount = 1;
                    blit.srcOffsets[1] = {static_cast<std::int32_t>(source_width),
                                          static_cast<std::int32_t>(source_height), 1};
                    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    blit.dstSubresource.mipLevel = level;
                    blit.dstSubresource.layerCount = 1;
                    blit.dstOffsets[1] = {static_cast<std::int32_t>(target_width),
                                          static_cast<std::int32_t>(target_height), 1};
                    vkCmdBlitImage(pending.cmd, image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &blit,
                                   VK_FILTER_LINEAR);
                    source_width = target_width;
                    source_height = target_height;
                }

                barrier(0, levels - 1, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT,
                        VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                        VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                barrier(levels - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT,
                        VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                        VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                vkEndCommandBuffer(pending.cmd);

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                Vulkan::check(
                    vkCreateFence(device_.device(), &fence_info, nullptr, &pending.fence),
                    "vkCreateFence(texture)");

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = pending.cmd;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                Vulkan::check(
                    vkQueueSubmit2(device_.graphics_queue(), 1, &submit, pending.fence),
                    "vkQueueSubmit2(texture)");
                return true;
            }

            void TextureLibrary::collect_finished()
            {
                for (std::size_t i = 0; i < pending_.size();)
                {
                    Pending& pending = pending_[i];
                    if (vkGetFenceStatus(device_.device(), pending.fence) != VK_SUCCESS)
                    {
                        ++i;
                        continue;
                    }
                    if (pending.retired_heap_index != 0xFFFFFFFFu)
                        heap_.free_texture(pending.retired_heap_index);
                    vkDestroyFence(device_.device(), pending.fence, nullptr);
                    vkFreeCommandBuffers(device_.device(), pool_, 1, &pending.cmd);
                    vmaDestroyBuffer(device_.allocator(), pending.staging,
                                     pending.staging_allocation);
                    if (pending.retired_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), pending.retired_view, nullptr);
                    if (pending.retired_image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), pending.retired_image,
                                        pending.retired_allocation);
                    pending_.erase(pending_.begin() + static_cast<std::ptrdiff_t>(i));
                }

                // Host-copy retirements have no fence: release each once the frame counter
                // has passed the frames that could still have been sampling the old image.
                for (std::size_t i = 0; i < retired_.size();)
                {
                    Retired& retired = retired_[i];
                    if (frame_counter_ < retired.retire_frame)
                    {
                        ++i;
                        continue;
                    }
                    if (retired.heap_index != 0xFFFFFFFFu)
                        heap_.free_texture(retired.heap_index);
                    if (retired.view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), retired.view, nullptr);
                    if (retired.image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), retired.image, retired.allocation);
                    retired_.erase(retired_.begin() + static_cast<std::ptrdiff_t>(i));
                }
            }

            void TextureLibrary::update()
            {
                ++frame_counter_;
                collect_finished();
                if (!pending_.empty())
                    return;

                // One upgrade per frame: find the texture furthest below its authored
                // resolution and give it one more mip level, if the budget has room.
                std::size_t best = entries_.size();
                std::uint64_t best_deficit = 0;
                for (std::size_t i = 0; i < entries_.size(); ++i)
                {
                    const Entry& entry = entries_[i];
                    if (!entry.live || (!entry.from_file && entry.pixels.empty()))
                        continue;
                    if (entry.resident_width >= entry.full_width &&
                        entry.resident_height >= entry.full_height)
                        continue;
                    const std::uint64_t deficit =
                        std::uint64_t(entry.full_width) * entry.full_height -
                        std::uint64_t(entry.resident_width) * entry.resident_height;
                    if (deficit > best_deficit)
                    {
                        best_deficit = deficit;
                        best = i;
                    }
                }
                if (best == entries_.size())
                    return;

                Entry& entry = entries_[best];
                const std::uint32_t target_width = std::min(entry.full_width,
                                                            entry.resident_width * 2);
                const std::uint32_t target_height = std::min(entry.full_height,
                                                             entry.resident_height * 2);
                if (resident_bytes_ - entry.bytes + chain_bytes(target_width, target_height) >
                    budget_bytes_)
                    return;

                std::vector<std::uint8_t> source = entry.pixels;
                if (entry.from_file)
                {
                    int width = 0;
                    int height = 0;
                    int channels = 0;
                    stbi_uc* decoded =
                        stbi_load(entry.key.c_str(), &width, &height, &channels, 4);
                    if (decoded == nullptr)
                        return;
                    source.assign(decoded, decoded + std::size_t(width) * height * TEXEL_BYTES);
                    stbi_image_free(decoded);
                }

                std::vector<std::uint8_t> scaled(std::move(source));
                std::uint32_t scaled_width = entry.full_width;
                std::uint32_t scaled_height = entry.full_height;
                while (scaled_width > target_width || scaled_height > target_height)
                    scaled = halve(scaled.data(), scaled_width, scaled_height, scaled_width,
                                   scaled_height);
                upload(entry, scaled.data(), scaled_width, scaled_height);
            }

            void TextureLibrary::release(TextureId texture)
            {
                if (texture == INVALID_TEXTURE || texture >= entries_.size())
                    return;
                Entry& entry = entries_[texture];
                if (!entry.live || entry.references == 0)
                    return;
                if (--entry.references > 0)
                    return;
                destroy(entry);
            }

            void TextureLibrary::destroy(Entry& entry)
            {
                if (!entry.live)
                    return;
                heap_.free_texture(entry.heap_index);
                if (entry.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), entry.view, nullptr);
                if (entry.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), entry.image, entry.allocation);
                if (entry.bytes > 0)
                    resident_bytes_ -= entry.bytes;
                entry.image = VK_NULL_HANDLE;
                entry.view = VK_NULL_HANDLE;
                entry.allocation = VK_NULL_HANDLE;
                entry.bytes = 0;
                entry.live = false;
                entry.pixels.clear();
                entry.pixels.shrink_to_fit();
            }

            std::uint32_t TextureLibrary::heap_index(TextureId texture,
                                                     DefaultTexture fallback) const noexcept
            {
                if (texture != INVALID_TEXTURE && texture < entries_.size() &&
                    entries_[texture].live)
                    return entries_[texture].heap_index;
                return entries_[defaults_[static_cast<std::uint32_t>(fallback)]].heap_index;
            }

            bool TextureLibrary::has_transparency(TextureId texture) const noexcept
            {
                if (texture == INVALID_TEXTURE || texture >= entries_.size())
                    return false;
                return entries_[texture].transparent;
            }
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
