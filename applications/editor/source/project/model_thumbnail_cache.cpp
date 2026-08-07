/**************************************************************************/
/* model_thumbnail_cache.cpp                                              */
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

#include "model_thumbnail_cache.hpp"

#include <cstring>
#include <stdexcept>
#include <utility>

namespace SushiEngine
{
    namespace Editor
    {
        ModelThumbnailCache::ModelThumbnailCache(SushiEngine::Render::IWindowRenderer& renderer,
                                                 ImGuiBackend& backend, Console& console)
            : renderer_(renderer)
            , mesh_renderer_(renderer.create_mesh_thumbnail_renderer())
            , device_(static_cast<VkDevice>(renderer.native_handles().device))
            , allocator_(static_cast<VmaAllocator>(renderer.native_handles().allocator))
            , backend_(backend)
            , console_(console)
        {
        }

        ModelThumbnailCache::~ModelThumbnailCache()
        {
            for (PendingEviction& pending : pending_evictions_)
                destroy_thumbnail(pending.thumbnail);
            pending_evictions_.clear();

            for (auto& entry : resident_.drain())
                destroy_thumbnail(entry.second);
        }

        std::optional<ImTextureID> ModelThumbnailCache::texture_for(
            const std::filesystem::path& path)
        {
            const std::string key = path.string();
            if (ResidentThumbnail* found = resident_.touch(key))
                return found->texture;

            // A load/render failure is permanent for this path (see class docs), matching
            // Phase 2's ThumbnailCache policy exactly.
            if (failed_.find(key) != failed_.end())
                return std::nullopt;

            if (in_flight_.insert(key).second)
                pending_.push_back(key);
            return std::nullopt;
        }

        void ModelThumbnailCache::update()
        {
            ++frame_counter_;

            while (!pending_evictions_.empty() &&
                   frame_counter_ - pending_evictions_.front().evicted_at_frame >=
                       static_cast<std::uint64_t>(EVICTION_DELAY_FRAMES))
            {
                destroy_thumbnail(pending_evictions_.front().thumbnail);
                pending_evictions_.pop_front();
            }

            if (pending_.empty())
                return;

            const std::string path = pending_.front();
            pending_.pop_front();
            in_flight_.erase(path);
            upload_one(path);
        }

        void ModelThumbnailCache::upload_one(const std::string& path)
        {
            SushiEngine::Render::FrameImage image;
            bool rendered = false;
            try
            {
                rendered = mesh_renderer_->render_thumbnail(path.c_str(), THUMBNAIL_SIZE,
                                                            THUMBNAIL_SIZE, image);
            }
            catch (const std::exception&)
            {
                rendered = false;
            }
            if (!rendered)
            {
                // A load/render failure is permanent for this path, silently -- matching
                // ThumbnailCache's decode-failure policy: no log, tile keeps its Phase 1 glyph
                // forever.
                failed_.insert(path);
                return;
            }

            // render_thumbnail succeeding already loaded this model's mesh/material data into
            // the mesh renderer's isolated, no-removal-API asset stack -- the unbounded-growth
            // resource this counter exists to bound -- regardless of whether the GPU-texture
            // upload below (this class's own small resident texture) then succeeds or fails.
            // Advance and check the recreation threshold here, not after the upload, so a run
            // of upload failures under memory/descriptor-pool pressure still counts toward
            // recreation instead of silently disabling it.
            ++models_rendered_since_recreation_;
            if (models_rendered_since_recreation_ >= MODELS_PER_RENDERER_LIFETIME)
                recreate_mesh_renderer();

            ResidentThumbnail thumbnail;
            VkImage vk_image = VK_NULL_HANDLE;
            VmaAllocation allocation = VK_NULL_HANDLE;
            VkImageView view = VK_NULL_HANDLE;
            VkBuffer staging = VK_NULL_HANDLE;
            VmaAllocation staging_allocation = VK_NULL_HANDLE;
            VkCommandPool command_pool = VK_NULL_HANDLE;
            VkCommandBuffer command = VK_NULL_HANDLE;
            VkFence fence = VK_NULL_HANDLE;

            try
            {
                const VkDeviceSize byte_size =
                    VkDeviceSize(THUMBNAIL_SIZE) * THUMBNAIL_SIZE * 4;
                if (image.rgba.size() != static_cast<std::size_t>(byte_size))
                    throw std::runtime_error("render_thumbnail returned an unexpected image size");

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
                if (vmaCreateImage(allocator_, &image_info, &alloc_info, &vk_image, &allocation,
                                   nullptr) != VK_SUCCESS)
                    throw std::runtime_error("vmaCreateImage(model thumbnail) failed");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = vk_image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = VK_FORMAT_R8G8B8A8_UNORM;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                if (vkCreateImageView(device_, &view_info, nullptr, &view) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateImageView(model thumbnail) failed");

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
                    throw std::runtime_error("vmaCreateBuffer(model thumbnail staging) failed");
                std::memcpy(staging_mapped.pMappedData, image.rgba.data(),
                           static_cast<std::size_t>(byte_size));
                vmaFlushAllocation(allocator_, staging_allocation, 0, VK_WHOLE_SIZE);

                const std::uint32_t queue_family =
                    renderer_.native_handles().graphics_queue_family;
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = queue_family;
                if (vkCreateCommandPool(device_, &pool_info, nullptr, &command_pool) !=
                    VK_SUCCESS)
                    throw std::runtime_error("vkCreateCommandPool(model thumbnail) failed");

                VkCommandBufferAllocateInfo command_alloc{};
                command_alloc.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_alloc.commandPool = command_pool;
                command_alloc.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_alloc.commandBufferCount = 1;
                if (vkAllocateCommandBuffers(device_, &command_alloc, &command) != VK_SUCCESS)
                    throw std::runtime_error("vkAllocateCommandBuffers(model thumbnail) failed");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                if (vkBeginCommandBuffer(command, &begin) != VK_SUCCESS)
                    throw std::runtime_error("vkBeginCommandBuffer(model thumbnail) failed");

                VkImageMemoryBarrier2 to_transfer{};
                to_transfer.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                to_transfer.srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                to_transfer.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                to_transfer.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                to_transfer.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                to_transfer.newLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                to_transfer.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                to_transfer.image = vk_image;
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
                vkCmdCopyBufferToImage(command, staging, vk_image,
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
                to_shader_read.image = vk_image;
                to_shader_read.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                VkDependencyInfo dependency_to_shader_read{};
                dependency_to_shader_read.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency_to_shader_read.imageMemoryBarrierCount = 1;
                dependency_to_shader_read.pImageMemoryBarriers = &to_shader_read;
                vkCmdPipelineBarrier2(command, &dependency_to_shader_read);

                if (vkEndCommandBuffer(command) != VK_SUCCESS)
                    throw std::runtime_error("vkEndCommandBuffer(model thumbnail) failed");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                if (vkCreateFence(device_, &fence_info, nullptr, &fence) != VK_SUCCESS)
                    throw std::runtime_error("vkCreateFence(model thumbnail) failed");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                const VkQueue queue =
                    static_cast<VkQueue>(renderer_.native_handles().graphics_queue);
                if (vkQueueSubmit2(queue, 1, &submit, fence) != VK_SUCCESS)
                    throw std::runtime_error("vkQueueSubmit2(model thumbnail) failed");
                vkWaitForFences(device_, 1, &fence, VK_TRUE, UINT64_MAX);

                vkDestroyFence(device_, fence, nullptr);
                vkFreeCommandBuffers(device_, command_pool, 1, &command);
                vkDestroyCommandPool(device_, command_pool, nullptr);
                vmaDestroyBuffer(allocator_, staging, staging_allocation);

                thumbnail.image = vk_image;
                thumbnail.allocation = allocation;
                thumbnail.view = view;
                // No sampler: this vendored ImGui Vulkan backend's AddTexture always uses its
                // own immutable sampler and ignores the one passed in, per Phase 2's own
                // minor-fix precedent -- creating one here would be dead weight.
                thumbnail.texture = backend_.register_texture(nullptr, view);
            }
            catch (const std::exception& error)
            {
                console_.append(LogLevel::Warning,
                                "Model thumbnail upload failed for '" + path + "': " +
                                    error.what());
                if (fence != VK_NULL_HANDLE)
                    vkDestroyFence(device_, fence, nullptr);
                if (command != VK_NULL_HANDLE)
                    vkFreeCommandBuffers(device_, command_pool, 1, &command);
                if (command_pool != VK_NULL_HANDLE)
                    vkDestroyCommandPool(device_, command_pool, nullptr);
                if (staging != VK_NULL_HANDLE)
                    vmaDestroyBuffer(allocator_, staging, staging_allocation);
                if (view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_, view, nullptr);
                if (vk_image != VK_NULL_HANDLE)
                    vmaDestroyImage(allocator_, vk_image, allocation);
                failed_.insert(path);
                return;
            }

            if (thumbnail.texture == static_cast<ImTextureID>(0))
            {
                console_.append(LogLevel::Warning,
                                "Model thumbnail upload failed for '" + path +
                                    "': ImGui returned a null texture id (likely descriptor "
                                    "pool exhaustion)");
                destroy_thumbnail(thumbnail);
                failed_.insert(path);
                return;
            }

            std::optional<std::pair<std::string, ResidentThumbnail>> evicted =
                resident_.insert(path, thumbnail);
            if (evicted.has_value())
                pending_evictions_.push_back(
                    PendingEviction{std::move(evicted->second), frame_counter_});
        }

        void ModelThumbnailCache::recreate_mesh_renderer()
        {
            // Safe with no extra GPU synchronization: render_thumbnail already blocked until
            // its own GPU work completed before returning (see class docs), so nothing tied to
            // the old instance is still in flight by the time this runs.
            //
            // Release the old renderer (and everything its isolated asset stack owns) before
            // constructing the new one, so the two are never resident at once -- at exactly the
            // moment memory pressure from the old one is highest.
            mesh_renderer_.reset();
            mesh_renderer_ = renderer_.create_mesh_thumbnail_renderer();
            models_rendered_since_recreation_ = 0;
        }

        void ModelThumbnailCache::destroy_thumbnail(ResidentThumbnail& thumbnail)
        {
            backend_.unregister_texture(thumbnail.texture);
            if (thumbnail.view != VK_NULL_HANDLE)
                vkDestroyImageView(device_, thumbnail.view, nullptr);
            if (thumbnail.image != VK_NULL_HANDLE)
                vmaDestroyImage(allocator_, thumbnail.image, thumbnail.allocation);
        }
    } // namespace Editor
} // namespace SushiEngine
