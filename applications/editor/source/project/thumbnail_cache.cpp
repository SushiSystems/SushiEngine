/**************************************************************************/
/* thumbnail_cache.cpp                                                    */
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

#include "thumbnail_cache.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

#include <stb_image.h>

#include <SushiEngine/imaging/box_downscale.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        ThumbnailCache::ThumbnailCache(SushiEngine::Render::NativeDeviceHandles handles,
                                       ImGuiBackend& backend, Console& console)
            : device_(static_cast<VkDevice>(handles.device))
            , allocator_(static_cast<VmaAllocator>(handles.allocator))
            , graphics_queue_(static_cast<VkQueue>(handles.graphics_queue))
            , backend_(backend)
            , console_(console)
        {
            VkCommandPoolCreateInfo pool_info{};
            pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
            pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
            pool_info.queueFamilyIndex = handles.graphics_queue_family;
            if (vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool_) != VK_SUCCESS)
                throw std::runtime_error(
                    "SushiEngine: ThumbnailCache command pool creation failed");

            worker_ = std::thread(&ThumbnailCache::worker_main, this);
        }

        ThumbnailCache::~ThumbnailCache()
        {
            {
                std::lock_guard<std::mutex> lock(requests_mutex_);
                stop_ = true;
            }
            requests_cv_.notify_all();
            if (worker_.joinable())
                worker_.join();

            for (PendingEviction& pending : pending_evictions_)
                destroy_thumbnail(pending.thumbnail);
            pending_evictions_.clear();

            for (auto& entry : resident_.drain())
                destroy_thumbnail(entry.second);

            vkDestroyCommandPool(device_, command_pool_, nullptr);
        }

        std::optional<ImTextureID> ThumbnailCache::texture_for(const std::filesystem::path& path)
        {
            const std::string key = path.string();
            if (ResidentThumbnail* found = resident_.touch(key))
                return found->texture;

            // An upload failure is permanent for this path (see class docs' failure-handling
            // note): never re-enqueue it, which would otherwise decode and re-attempt the
            // upload, and re-log a warning, on every single frame the tile stays visible.
            if (failed_uploads_.find(key) != failed_uploads_.end())
                return std::nullopt;

            if (in_flight_.find(key) == in_flight_.end())
            {
                in_flight_[key] = true;
                {
                    std::lock_guard<std::mutex> lock(requests_mutex_);
                    requests_.push_back(key);
                }
                requests_cv_.notify_one();
            }
            return std::nullopt;
        }

        void ThumbnailCache::update()
        {
            ++frame_counter_;

            // Actually free an evicted thumbnail's Vulkan resources only once enough frames
            // have elapsed since its eviction that no command buffer from a prior frame can
            // still be sampling it (see EVICTION_DELAY_FRAMES' doc comment).
            while (!pending_evictions_.empty() &&
                   frame_counter_ - pending_evictions_.front().evicted_at_frame >=
                       static_cast<std::uint64_t>(EVICTION_DELAY_FRAMES))
            {
                destroy_thumbnail(pending_evictions_.front().thumbnail);
                pending_evictions_.pop_front();
            }

            for (int i = 0; i < UPLOADS_PER_FRAME; ++i)
            {
                DecodedImage decoded;
                {
                    std::lock_guard<std::mutex> lock(ready_mutex_);
                    if (ready_.empty())
                        return;
                    decoded = std::move(ready_.front());
                    ready_.pop_front();
                }
                in_flight_.erase(decoded.path);
                upload_one(decoded);
            }
        }

        void ThumbnailCache::worker_main()
        {
            for (;;)
            {
                std::string path;
                {
                    std::unique_lock<std::mutex> lock(requests_mutex_);
                    requests_cv_.wait(lock, [this] { return stop_ || !requests_.empty(); });
                    if (stop_)
                        return;
                    path = requests_.front();
                    requests_.pop_front();
                }

                // A decode failure (bad path, unsupported format, corrupt file) is silently
                // dropped: the tile keeps showing Phase 1's picture-frame glyph forever for
                // this path, which is the design's stated fallback rather than an error state.
                int width = 0;
                int height = 0;
                int source_channels = 0;
                stbi_uc* pixels =
                    stbi_load(path.c_str(), &width, &height, &source_channels, 4);
                if (pixels == nullptr)
                    continue;

                DecodedImage decoded;
                decoded.path = path;
                decoded.pixels = SushiEngine::Imaging::box_downscale_rgba8(
                    pixels, static_cast<std::uint32_t>(width),
                    static_cast<std::uint32_t>(height), THUMBNAIL_SIZE, THUMBNAIL_SIZE);
                stbi_image_free(pixels);

                std::lock_guard<std::mutex> lock(ready_mutex_);
                ready_.push_back(std::move(decoded));
            }
        }

        void ThumbnailCache::upload_one(const DecodedImage& decoded)
        {
            ResidentThumbnail thumbnail;
            VkImage image = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
            VkSampler sampler = VK_NULL_HANDLE;
            VkBuffer staging = VK_NULL_HANDLE;
            VmaAllocation staging_allocation = VK_NULL_HANDLE;
            VkCommandBuffer command = VK_NULL_HANDLE;
            VkFence fence = VK_NULL_HANDLE;

            try
            {
                const VkDeviceSize byte_size =
                    VkDeviceSize(THUMBNAIL_SIZE) * THUMBNAIL_SIZE * 4;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = VK_FORMAT_R8G8B8A8_UNORM;
                image_info.extent = {THUMBNAIL_SIZE, THUMBNAIL_SIZE, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc_info{};
                alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                if (vmaCreateImage(allocator_, &image_info, &alloc_info, &image, &allocation,
                                   nullptr) != VK_SUCCESS)
                    throw std::runtime_error("vmaCreateImage(thumbnail) failed");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                if (vkCreateImageView(device_, &view_info, nullptr, &view) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateImageView(thumbnail) failed");

                VkSamplerCreateInfo sampler_info{};
                sampler_info.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
                sampler_info.magFilter = VK_FILTER_LINEAR;
                sampler_info.minFilter = VK_FILTER_LINEAR;
                sampler_info.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.addressModeV = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.addressModeW = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_info.maxLod = 1.0f;
                if (vkCreateSampler(device_, &sampler_info, nullptr, &sampler) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateSampler(thumbnail) failed");

                VkBufferCreateInfo staging_info{};
                staging_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                staging_info.size = byte_size;
                staging_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                VmaAllocationCreateInfo staging_alloc_info{};
                staging_alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                staging_alloc_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                            VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo staging_mapped{};
                if (vmaCreateBuffer(allocator_, &staging_info, &staging_alloc_info, &staging,
                                    &staging_allocation, &staging_mapped) != VK_SUCCESS)
                    throw std::runtime_error("vmaCreateBuffer(thumbnail staging) failed");
                std::memcpy(staging_mapped.pMappedData, decoded.pixels.data(),
                           static_cast<std::size_t>(byte_size));
                // VMA is free to back a HOST_ACCESS_SEQUENTIAL_WRITE allocation with
                // non-coherent host-visible memory (nothing here requests HOST_COHERENT), so the
                // GPU copy below is not guaranteed to see the bytes just written without an
                // explicit flush. A documented no-op when the memory turns out to be coherent,
                // so this is always safe to call unconditionally.
                vmaFlushAllocation(allocator_, staging_allocation, 0, VK_WHOLE_SIZE);

                VkCommandBufferAllocateInfo command_alloc{};
                command_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_alloc.commandPool = command_pool_;
                command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_alloc.commandBufferCount = 1;
                if (vkAllocateCommandBuffers(device_, &command_alloc, &command) != VK_SUCCESS)
                    throw std::runtime_error("vkAllocateCommandBuffers(thumbnail) failed");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                vkBeginCommandBuffer(command, &begin);

                VkImageMemoryBarrier2 to_transfer{};
                to_transfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_transfer.srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_transfer.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_transfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                to_transfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_transfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                to_transfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.image = image;
                to_transfer.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo dependency_to_transfer{};
                dependency_to_transfer.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency_to_transfer.imageMemoryBarrierCount = 1;
                dependency_to_transfer.pImageMemoryBarriers = &to_transfer;
                vkCmdPipelineBarrier2(command, &dependency_to_transfer);

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {THUMBNAIL_SIZE, THUMBNAIL_SIZE, 1};
                vkCmdCopyBufferToImage(command, staging, image,
                                       VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);

                VkImageMemoryBarrier2 to_shader_read{};
                to_shader_read.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_shader_read.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_shader_read.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                to_shader_read.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                to_shader_read.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
                to_shader_read.oldLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                to_shader_read.newLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                to_shader_read.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_shader_read.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_shader_read.image = image;
                to_shader_read.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo dependency_to_shader_read{};
                dependency_to_shader_read.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency_to_shader_read.imageMemoryBarrierCount = 1;
                dependency_to_shader_read.pImageMemoryBarriers = &to_shader_read;
                vkCmdPipelineBarrier2(command, &dependency_to_shader_read);

                vkEndCommandBuffer(command);

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                if (vkCreateFence(device_, &fence_info, nullptr, &fence) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateFence(thumbnail) failed");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                if (vkQueueSubmit2(graphics_queue_, 1, &submit, fence) != VK_SUCCESS)
                    throw std::runtime_error("vkQueueSubmit2(thumbnail) failed");

                // Synchronous: at most UPLOADS_PER_FRAME of these run per frame, unlike the
                // render loop's own frames-in-flight uploads, which must never stall a frame.
                vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);
                vkDestroyFence(device_, fence, nullptr);
                vkFreeCommandBuffers(device_, command_pool_, 1, &command);
                vmaDestroyBuffer(allocator_, staging, staging_allocation);

                thumbnail.image = image;
                thumbnail.allocation = allocation;
                thumbnail.view = view;
                thumbnail.sampler = sampler;
                thumbnail.texture = backend_.register_texture(sampler, view);
            }
            catch (const std::exception& error)
            {
                console_.append(LogLevel::Warning, "Thumbnail upload failed for '" +
                                                        decoded.path + "': " + error.what());
                if (fence != VK_NULL_HANDLE)
                    vkDestroyFence(device_, fence, nullptr);
                if (command != VK_NULL_HANDLE)
                    vkFreeCommandBuffers(device_, command_pool_, 1, &command);
                if (staging != VK_NULL_HANDLE)
                    vmaDestroyBuffer(allocator_, staging, staging_allocation);
                if (sampler != VK_NULL_HANDLE)
                    vkDestroySampler(device_, sampler, nullptr);
                if (view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_, view, nullptr);
                if (image != VK_NULL_HANDLE)
                    vmaDestroyImage(allocator_, image, allocation);
                // Permanent: texture_for() must stop re-enqueueing this path (see its own
                // comment and the class docs' failure-handling note).
                failed_uploads_.insert(decoded.path);
                return;
            }

            // ImGui_ImplVulkan_AddTexture (called through register_texture) allocates its
            // descriptor set from ImGuiBackend's pool without going through this class's own
            // try/catch: a pool-exhaustion vkAllocateDescriptorSets failure is swallowed by
            // ImGui's own CheckVkResultFn (which ImGuiBackend never sets), and the call simply
            // returns a null descriptor set rather than throwing. Treat that the same as any
            // other upload failure: log, clean up the resources already created, and never
            // retry this path.
            if (thumbnail.texture == static_cast<ImTextureID>(0))
            {
                console_.append(LogLevel::Warning,
                                 "Thumbnail upload failed for '" + decoded.path +
                                     "': ImGui returned a null texture id (likely descriptor "
                                     "pool exhaustion)");
                destroy_thumbnail(thumbnail);
                failed_uploads_.insert(decoded.path);
                return;
            }

            std::optional<std::pair<std::string, ResidentThumbnail>> evicted =
                resident_.insert(decoded.path, thumbnail);
            if (evicted.has_value())
                pending_evictions_.push_back(
                    PendingEviction{std::move(evicted->second), frame_counter_});
        }

        void ThumbnailCache::destroy_thumbnail(ResidentThumbnail& thumbnail)
        {
            backend_.unregister_texture(thumbnail.texture);
            if (thumbnail.sampler != VK_NULL_HANDLE)
                vkDestroySampler(device_, thumbnail.sampler, nullptr);
            if (thumbnail.view != VK_NULL_HANDLE)
                vkDestroyImageView(device_, thumbnail.view, nullptr);
            if (thumbnail.image != VK_NULL_HANDLE)
                vmaDestroyImage(allocator_, thumbnail.image, thumbnail.allocation);
        }
    } // namespace Editor
} // namespace SushiEngine
