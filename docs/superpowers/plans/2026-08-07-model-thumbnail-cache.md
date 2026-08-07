# Model thumbnail cache (Phase 3b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show a real, rendered thumbnail for `.gltf`/`.glb` files in the Project panel's Grid view,
consuming the `IMeshThumbnailRenderer`/`IWindowRenderer::create_mesh_thumbnail_renderer()` primitive
Phase 3a already landed. This is the editor-tier half of model thumbnails; Phase 3a (engine-only,
already merged) added no editor-facing surface at all.

**Architecture:** A new `ModelThumbnailCache`, parallel to Phase 2's `ThumbnailCache` in spirit but
with no background thread — `IMeshThumbnailRenderer::render_thumbnail` is synchronous, blocking,
and must run on the same thread that owns the main renderer's graphics queue (per its own
documented contract), so `ModelThumbnailCache::update()` processes at most one queued model per
frame directly, no worker thread involved. Because the underlying `IMeshThumbnailRenderer`'s
isolated mesh/texture stack has no removal API (a known, documented constraint from Phase 3a) and
therefore grows without bound as distinct models are rendered through it, `ModelThumbnailCache`
periodically destroys and recreates its `IMeshThumbnailRenderer` instance to reclaim that memory —
the chosen mitigation for that constraint, confirmed with the user. Each successful render produces
CPU-side RGBA8 bytes (`FrameImage`) that `ModelThumbnailCache` uploads to its own small, independent
GPU texture (using the main renderer's device directly, not the isolated stack) and registers with
ImGui — those resident thumbnail textures are NOT affected by recreating the underlying
`IMeshThumbnailRenderer`, since they are entirely separate GPU resources this class owns itself.

**Tech Stack:** C++17, Vulkan 1.3 (core sync2), VMA (`vk_mem_alloc.h`), Dear ImGui, GoogleTest is
not applicable here (see Testing note in Task 1).

## Global Constraints

- **Format scope:** real thumbnails apply to `.gltf`/`.glb` only — the same two extensions Phase 3a's
  engine primitive actually supports. `.fbx`/`.obj` entries (recognized by the Project panel's
  classifier but with no loader anywhere in the engine) must never reach `ModelThumbnailCache` at
  all; they keep showing Phase 1's wireframe-cube glyph exactly as before this phase.
- **Scope, as in Phase 2:** Grid view only. List view is unaffected.
- **Thumbnail resolution:** fixed **128×128**, matching `IMeshThumbnailRenderer::render_thumbnail`'s
  `width`/`height` parameters and Phase 2's `ThumbnailCache::THUMBNAIL_SIZE` convention.
- **Resident cache cap:** LRU, **32** resident model thumbnails — much smaller than Phase 2's 256,
  reflecting the far higher per-model cost (a full mesh/material import and a GPU draw, not a
  buffer copy).
- **Per-frame budget:** at most **one** model processed (rendered + uploaded) per frame inside
  `update()` — `render_thumbnail` is fully synchronous and blocks until the GPU finishes, so this is
  the entire per-frame cost, unlike Phase 2's cheap buffer-copy budget of 2.
- **Renderer-recreation threshold:** after **64** models have been successfully rendered through one
  `IMeshThumbnailRenderer` instance, destroy it and create a fresh one via
  `IWindowRenderer::create_mesh_thumbnail_renderer()` — the chosen, user-confirmed mitigation for
  the underlying mesh/texture stack's unbounded growth (see Architecture). This is safe with no
  extra GPU synchronization: `render_thumbnail` already blocks until its own GPU work completes
  before returning, so by the time `update()` regains control after a successful call, nothing tied
  to the old instance is still in flight.
- **Failure handling, matching Phase 2's established policy exactly:** a load/render failure
  (`render_thumbnail` returning `false`) is silently permanent for that path — no log, never
  retried, tile keeps its wireframe-cube glyph forever. An upload failure (the GPU texture creation
  step, after a successful `render_thumbnail` call) is logged once via
  `Console::append(LogLevel::Warning, ...)` and is also permanent for that path.
- **This machine cannot run builds.** No task's implementer or reviewer runs `se build`, `se test`,
  `se editor`, cmake, or ninja. Every task is verified by reading the code and reasoning about it by
  hand — the user builds and tests the whole branch after every task is complete.
- **A few API surfaces below are marked "verify against the live header before writing this exact
  call."** Phase 2's `ThumbnailCache` and Phase 3a's `VulkanMeshThumbnailRenderer` already
  established every Vulkan idiom this plan needs (staging-buffer upload, `ImGuiBackend::register_texture`
  without a sampler per Phase 2's own minor-fix precedent, `LruCache`'s interface, delayed eviction).
  Confirm each cited signature against the live file before using it — none of this plan's own
  research is new; it is transcribing already-proven patterns, but "already proven elsewhere" is not
  the same as "verified in this exact file," so verify.

---

### Task 1: `ModelThumbnailCache`

**Files:**
- Create: `applications/editor/source/project/model_thumbnail_cache.hpp`
- Create: `applications/editor/source/project/model_thumbnail_cache.cpp`

**Interfaces:**
- Consumes: `SushiEngine::Render::IWindowRenderer` (specifically
  `create_mesh_thumbnail_renderer()`, confirmed at
  `engine/presentation/render/include/SushiEngine/render/window_renderer.hpp`),
  `SushiEngine::Render::IMeshThumbnailRenderer::render_thumbnail(const char*, std::uint32_t,
  std::uint32_t, FrameImage&) -> bool` (confirmed at
  `engine/presentation/render/include/SushiEngine/render/mesh_thumbnail_renderer.hpp`),
  `SushiEngine::Render::FrameImage{width, height, rgba}` (confirmed at
  `engine/presentation/render/include/SushiEngine/render/scene_view.hpp`),
  `SushiEngine::Imaging::LruCache<Key, Value>` (Phase 2, `engine/domain/imaging`),
  `SushiEngine::Editor::Console::append`, `SushiEngine::Editor::ImGuiBackend::register_texture`/
  `unregister_texture`.
- Produces: `SushiEngine::Editor::ModelThumbnailCache`, consumed by Task 2 (construction/wiring) and
  Task 3 (`texture_for`/`update` calls from the Grid view):
  - `ModelThumbnailCache(SushiEngine::Render::IWindowRenderer& renderer, ImGuiBackend& backend,
    Console& console)`
  - `~ModelThumbnailCache()`
  - `void update()`
  - `std::optional<ImTextureID> texture_for(const std::filesystem::path& path)`

This is one cohesive class and one cohesive review gate, matching how Phase 2's `ThumbnailCache`
and Phase 3a's `VulkanMeshThumbnailRenderer` were each one task despite being their plans' largest.
There is no unit test for this task — every line touches Vulkan/ImGui, neither of which this
codebase's test binary can reach (the same reasoning Phase 2 and Phase 3a both already established
for their own Vulkan-facing classes).

- [ ] **Step 1: Write the header**

Create `applications/editor/source/project/model_thumbnail_cache.hpp`:

```cpp
/**************************************************************************/
/* model_thumbnail_cache.hpp                                              */
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
 * @file model_thumbnail_cache.hpp
 * @brief Real model content for the Project panel's Grid view tiles.
 *
 * The engine-tier counterpart to Phase 2's ThumbnailCache, for `.gltf`/`.glb` files instead of
 * images: it asks an `IMeshThumbnailRenderer` (Phase 3a) to render a model and read the pixels
 * back, then uploads those pixels to its own small resident GPU texture for ImGui, exactly the
 * way ThumbnailCache uploads a decoded image. Unlike ThumbnailCache, there is no background
 * thread — `IMeshThumbnailRenderer::render_thumbnail` is synchronous and must run on the thread
 * that owns the main renderer's graphics queue, so this class's `update()` does the whole
 * render+upload for at most one model per frame, directly.
 *
 * `IMeshThumbnailRenderer`'s isolated mesh/texture stack has no removal API (documented on
 * `VulkanMeshThumbnailRenderer` itself) and grows without bound as distinct models are rendered
 * through it. This class's mitigation: after a fixed number of successful renders, it destroys
 * its `IMeshThumbnailRenderer` and creates a fresh one, reclaiming that memory. This does not
 * affect any thumbnail already resident here — those are independent GPU textures this class
 * owns directly, unrelated to the mesh renderer's own internal resources.
 *
 * See docs/superpowers/specs/2026-08-07-project-panel-model-thumbnails-design.md.
 */

#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>

#include <imgui.h>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/imaging/lru_cache.hpp>
#include <SushiEngine/render/mesh_thumbnail_renderer.hpp>
#include <SushiEngine/render/window_renderer.hpp>

#include "../core/console.hpp"
#include "../ui/imgui_backend.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Real model thumbnails for the Project panel's Grid view, one render per frame.
         *
         * Non-copyable: it owns a live `IMeshThumbnailRenderer` and a set of Vulkan resources
         * tied to one device. Construction builds the first mesh renderer; destruction frees
         * every resident thumbnail's Vulkan resources.
         */
        class ModelThumbnailCache
        {
            public:
                /**
                 * @brief Builds the cache's first mesh renderer.
                 * @param renderer The window renderer this cache's mesh renderers are created
                 *   from, and whose device its own resident-texture Vulkan resources are built
                 *   against.
                 * @param backend The ImGui Vulkan backend thumbnails are registered with.
                 * @param console Where an upload failure is logged (see class docs).
                 * @throws std::runtime_error if the first mesh renderer or this cache's own
                 *   Vulkan command pool cannot be created.
                 */
                ModelThumbnailCache(SushiEngine::Render::IWindowRenderer& renderer,
                                    ImGuiBackend& backend, Console& console);

                /** @brief Frees every resident thumbnail's Vulkan resources. */
                ~ModelThumbnailCache();

                ModelThumbnailCache(const ModelThumbnailCache&) = delete;
                ModelThumbnailCache& operator=(const ModelThumbnailCache&) = delete;

                /**
                 * @brief Renders and uploads at most one queued model this frame.
                 *
                 * Call once per frame, before any panel that might call @ref texture_for reads
                 * its result for this frame.
                 */
                void update();

                /**
                 * @brief The thumbnail texture for @p path, requesting a render if needed.
                 * @return A texture id if @p path's thumbnail is already resident; otherwise
                 *   @c std::nullopt, having enqueued a render request unless one is already
                 *   pending for the same path. A path whose render or upload failed keeps
                 *   returning @c std::nullopt forever (see class docs on failure handling).
                 */
                std::optional<ImTextureID> texture_for(const std::filesystem::path& path);

            private:
                /** @brief One resident thumbnail's live Vulkan resources plus its ImGui texture id. */
                struct ResidentThumbnail
                {
                    VkImage image = VK_NULL_HANDLE;
                    VmaAllocation allocation = VK_NULL_HANDLE;
                    VkImageView view = VK_NULL_HANDLE;
                    ImTextureID texture = static_cast<ImTextureID>(0);
                };

                /**
                 * @brief An evicted thumbnail's resources, held until the GPU is guaranteed to
                 *   be done sampling it.
                 */
                struct PendingEviction
                {
                    ResidentThumbnail thumbnail;
                    std::uint64_t evicted_at_frame = 0;
                };

                void upload_one(const std::string& path);
                void destroy_thumbnail(ResidentThumbnail& thumbnail);
                void recreate_mesh_renderer();

                static constexpr std::uint32_t THUMBNAIL_SIZE = 128;
                static constexpr std::size_t RESIDENT_CAPACITY = 32;
                static constexpr int EVICTION_DELAY_FRAMES = 4;
                // After this many successful renders through one IMeshThumbnailRenderer, it is
                // destroyed and replaced to reclaim its unbounded-growth isolated asset stack
                // (see class docs).
                static constexpr std::size_t MODELS_PER_RENDERER_LIFETIME = 64;

                SushiEngine::Render::IWindowRenderer& renderer_;
                std::unique_ptr<SushiEngine::Render::IMeshThumbnailRenderer> mesh_renderer_;
                std::size_t models_rendered_since_recreation_ = 0;

                VkDevice device_ = VK_NULL_HANDLE;
                VmaAllocator allocator_ = VK_NULL_HANDLE;
                ImGuiBackend& backend_;
                Console& console_;

                std::deque<std::string> pending_;
                std::unordered_set<std::string> in_flight_;
                std::unordered_set<std::string> failed_;
                std::uint64_t frame_counter_ = 0;
                std::deque<PendingEviction> pending_evictions_;

                // Main-thread-only state; there is no other thread in this class.
                SushiEngine::Imaging::LruCache<std::string, ResidentThumbnail> resident_{
                    RESIDENT_CAPACITY};
        };
    } // namespace Editor
} // namespace SushiEngine
```

- [ ] **Step 2: Write the implementation**

Create `applications/editor/source/project/model_thumbnail_cache.cpp`:

```cpp
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

            ++models_rendered_since_recreation_;
            if (models_rendered_since_recreation_ >= MODELS_PER_RENDERER_LIFETIME)
                recreate_mesh_renderer();
        }

        void ModelThumbnailCache::recreate_mesh_renderer()
        {
            // Safe with no extra GPU synchronization: render_thumbnail already blocked until
            // its own GPU work completed before returning (see class docs), so nothing tied to
            // the old instance is still in flight by the time this runs.
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
```

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build or run this code. Verify each of the following by
reading:

1. **`NativeDeviceHandles::allocator`/`graphics_queue`/`graphics_queue_family`** — confirm these
   three fields exist with these exact names on
   `engine/presentation/render/include/SushiEngine/render/rhi/device.hpp`'s `NativeDeviceHandles`
   struct (Phase 2 added `allocator`; the other two predate it).
2. **`IWindowRenderer::native_handles()`** — confirm this method exists and returns
   `NativeDeviceHandles` by value (so calling it twice, as this file does once in the constructor's
   init list and once inside `upload_one` for the queue family/queue, is cheap and safe — it is a
   plain struct copy, not an expensive call).
3. **`LruCache::drain()`/`touch()`/`insert()`** — confirm the exact signatures against
   `engine/domain/imaging/include/SushiEngine/imaging/lru_cache.hpp` match how this file uses them
   (this is the same class Phase 2's `ThumbnailCache` already uses identically — cross-check against
   `applications/editor/source/project/thumbnail_cache.cpp` if anything looks different).
4. **Cleanup ordering on every construction-failure path in `upload_one`** — confirm no Vulkan
   handle is destroyed twice and none is leaked, by re-tracing each `throw` site against exactly
   which of `vk_image`/`view`/`staging`/`command_pool`/`command`/`fence` had already been created at
   that point (this mirrors the exact same exercise Phase 2's `ThumbnailCache::upload_one` review
   already did for its own throw sites — apply the same method here).
5. **`ImGuiBackend::register_texture`/`unregister_texture`'s exact signatures** — confirm
   `register_texture(void* sampler, void* image_view) -> ImTextureID` and `unregister_texture(ImTextureID)`
   against `applications/editor/source/ui/imgui_backend.hpp`, and confirm passing `nullptr` for the
   sampler argument is valid (Phase 2's minor-fix round already established this is correct for this
   vendored ImGui Vulkan backend).

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/model_thumbnail_cache.hpp \
        applications/editor/source/project/model_thumbnail_cache.cpp
git commit -m "feat(editor): add ModelThumbnailCache, the Project panel's model render/upload pipeline"
```

---

### Task 2: Wire `ModelThumbnailCache` into `EditorContext` and `main.cpp`

**Files:**
- Modify: `applications/editor/source/core/editor_context.hpp`
- Modify: `applications/editor/source/main.cpp`

**Interfaces:**
- Consumes: `SushiEngine::Editor::ModelThumbnailCache` (Task 1).
- Produces: `EditorContext::model_thumbnail_cache` (`ModelThumbnailCache*`, non-owning, following
  the exact same pattern as `EditorContext::thumbnail_cache`), which Task 3 reads from
  `project_panel.cpp`.

- [ ] **Step 1: Forward-declare `ModelThumbnailCache` in `editor_context.hpp`**

In `applications/editor/source/core/editor_context.hpp`, the `namespace Editor` block currently
reads (lines 74-86):

```cpp
    namespace Editor
    {
        /** @brief The live particle-effect preview, owned by main() (see effect_preview.hpp). */
        class EffectPreview;

        /** @brief The live GPU-skinned character preview, owned by main() (see animated_mesh_preview.hpp). */
        class AnimatedMeshPreview;

        /** @brief The live editor audio system, owned by main() (see audio/audio_editor_system.hpp). */
        class AudioEditorSystem;

        /** @brief The Project panel's image thumbnail pipeline, owned by main() (see project/thumbnail_cache.hpp). */
        class ThumbnailCache;
```

Add one more forward declaration after `ThumbnailCache`:

```cpp
        /** @brief The Project panel's image thumbnail pipeline, owned by main() (see project/thumbnail_cache.hpp). */
        class ThumbnailCache;

        /** @brief The Project panel's model thumbnail pipeline, owned by main() (see project/model_thumbnail_cache.hpp). */
        class ModelThumbnailCache;
```

- [ ] **Step 2: Add the field to `EditorContext`**

In the same file, `EditorContext`'s field list currently has (lines 269-274):

```cpp
            // The Project panel's image thumbnail pipeline, owned by main() and injected here
            // so the Grid view can ask for a tile's real thumbnail texture. main() constructs
            // it unconditionally, so unlike this struct's other pointer fields it is always
            // non-null once main() has run past that point; there is no headless-editor case
            // for it today.
            ThumbnailCache* thumbnail_cache = nullptr;
```

Add the new field immediately after it:

```cpp
            // The Project panel's image thumbnail pipeline, owned by main() and injected here
            // so the Grid view can ask for a tile's real thumbnail texture. main() constructs
            // it unconditionally, so unlike this struct's other pointer fields it is always
            // non-null once main() has run past that point; there is no headless-editor case
            // for it today.
            ThumbnailCache* thumbnail_cache = nullptr;

            // The Project panel's model thumbnail pipeline, owned by main() and injected here
            // so the Grid view can ask for a .gltf/.glb tile's real rendered thumbnail texture.
            // main() constructs it unconditionally, matching thumbnail_cache above.
            ModelThumbnailCache* model_thumbnail_cache = nullptr;
```

- [ ] **Step 3: Construct and wire `ModelThumbnailCache` in `main.cpp`**

Add the include near the existing `#include "project/thumbnail_cache.hpp"` line (line 65):

```cpp
#include "project/thumbnail_cache.hpp"
#include "project/model_thumbnail_cache.hpp"
```

Then, at lines 349-356, the file currently reads:

```cpp
        // The Project panel's real-image-thumbnail pipeline. Declared after `imgui` and
        // `renderer` (both already constructed above) so it is destroyed before either of
        // them, in the normal C++ stack-unwind order — its worker thread and every resident
        // Vulkan resource must be torn down while the device and allocator they were built
        // against are still alive.
        SushiEngine::Editor::ThumbnailCache thumbnail_cache(renderer->native_handles(), imgui,
                                                             context.console);
        context.thumbnail_cache = &thumbnail_cache;
```

Add the `ModelThumbnailCache` construction and wiring immediately after that block:

```cpp
        // The Project panel's real-image-thumbnail pipeline. Declared after `imgui` and
        // `renderer` (both already constructed above) so it is destroyed before either of
        // them, in the normal C++ stack-unwind order — its worker thread and every resident
        // Vulkan resource must be torn down while the device and allocator they were built
        // against are still alive.
        SushiEngine::Editor::ThumbnailCache thumbnail_cache(renderer->native_handles(), imgui,
                                                             context.console);
        context.thumbnail_cache = &thumbnail_cache;

        // The Project panel's model-thumbnail pipeline, for the same reason and with the same
        // destruction-order requirement as thumbnail_cache above: it holds a live
        // IMeshThumbnailRenderer and its own resident Vulkan textures, both built against
        // *renderer, so it must be destroyed before renderer is.
        SushiEngine::Editor::ModelThumbnailCache model_thumbnail_cache(*renderer, imgui,
                                                                       context.console);
        context.model_thumbnail_cache = &model_thumbnail_cache;
```

Then, at line 576, the main loop currently reads:

```cpp
            context.thumbnail_cache->update();
```

Add the per-frame call immediately after it:

```cpp
            context.thumbnail_cache->update();
            context.model_thumbnail_cache->update();
```

- [ ] **Step 4: Verify by reading, not by building**

Per Global Constraints, this machine cannot build. Re-read the full span of `main.cpp` from the
`renderer`/`imgui` declarations down through the new `model_thumbnail_cache` declaration and confirm
the C++ stack-unwind order holds: `model_thumbnail_cache` is declared after both `imgui` and
`renderer`, so it is destroyed before either of them, with no explicit destructor call needed
anywhere in `main`. Also confirm `ModelThumbnailCache`'s constructor signature
(`IWindowRenderer&, ImGuiBackend&, Console&`) matches exactly how it's called here
(`*renderer, imgui, context.console`) — `*renderer` dereferences the `std::unique_ptr<IWindowRenderer>
renderer` already in scope, giving the `IWindowRenderer&` the constructor expects.

- [ ] **Step 5: Commit**

```bash
git add applications/editor/source/core/editor_context.hpp \
        applications/editor/source/main.cpp
git commit -m "feat(editor): construct and wire ModelThumbnailCache into the editor's main loop"
```

---

### Task 3: Grid view integration — real model thumbnails for `.gltf`/`.glb`

**Files:**
- Modify: `applications/editor/source/project/project_panel.cpp`

**Interfaces:**
- Consumes: `EditorContext::model_thumbnail_cache` (Task 2), `ModelThumbnailCache::texture_for`
  (Task 1).
- Produces: nothing further downstream — this is the plan's last task.

- [ ] **Step 1: Include the header**

In `applications/editor/source/project/project_panel.cpp`, the include block currently reads
(lines 24-33, per Phase 2's own edit):

```cpp
#include "project_panel.hpp"

#include <SushiEngine/authoring/cook_bake_state.hpp>
#include <SushiEngine/model/import_settings_io.hpp>

#include "prefab_serializer.hpp"
#include "thumbnail_cache.hpp"

#include "../animation/animated_mesh_preview.hpp"
#include "../scene/scene_commands.hpp"
#include "../ui/panel_widgets.hpp"
```

Add the new include next to `thumbnail_cache.hpp`:

```cpp
#include "prefab_serializer.hpp"
#include "thumbnail_cache.hpp"
#include "model_thumbnail_cache.hpp"
```

- [ ] **Step 2: Draw the real thumbnail for `.gltf`/`.glb` entries, gated on extension**

`draw_project_grid_view` currently reads (this exact block was Phase 2's own final edit):

```cpp
                        const EntryCategory category = entry_category(entry.path(), is_dir);
                        std::optional<ImTextureID> thumbnail_texture;
                        if (category == EntryCategory::Image && context.thumbnail_cache != nullptr &&
                            ImGui::IsRectVisible(origin, ImVec2(origin.x + tile_size, origin.y + tile_size)))
                        {
                            // IsRectVisible tests the tile's screen rect against the grid child
                            // window's current clip rect, so a tile scrolled out of view never
                            // requests a decode — a folder full of images scrolled past should
                            // never front-load every one of them.
                            thumbnail_texture = context.thumbnail_cache->texture_for(entry.path());
                        }
```

Replace it with a version that also checks the `EntryCategory::Model` case, gated additionally on
the file extension being `.gltf` or `.glb` (never `.fbx`/`.obj`, which `ModelThumbnailCache` cannot
render and must never be asked to):

```cpp
                        const EntryCategory category = entry_category(entry.path(), is_dir);
                        std::optional<ImTextureID> thumbnail_texture;
                        const bool tile_visible = ImGui::IsRectVisible(
                            origin, ImVec2(origin.x + tile_size, origin.y + tile_size));
                        if (category == EntryCategory::Image && context.thumbnail_cache != nullptr &&
                            tile_visible)
                        {
                            // IsRectVisible tests the tile's screen rect against the grid child
                            // window's current clip rect, so a tile scrolled out of view never
                            // requests a decode — a folder full of images scrolled past should
                            // never front-load every one of them.
                            thumbnail_texture = context.thumbnail_cache->texture_for(entry.path());
                        }
                        else if (category == EntryCategory::Model &&
                                context.model_thumbnail_cache != nullptr && tile_visible)
                        {
                            // Real thumbnails only for the two formats the engine can actually
                            // load (see Global Constraints) -- .fbx/.obj entries fall through to
                            // the wireframe-cube glyph below, exactly as they did before this
                            // task, since ModelThumbnailCache is never asked about them.
                            const std::string extension = to_lower(entry.path().extension().string());
                            if (extension == ".gltf" || extension == ".glb")
                                thumbnail_texture =
                                    context.model_thumbnail_cache->texture_for(entry.path());
                        }
```

The rest of the tile-drawing block (the `if (thumbnail_texture.has_value()) { draw_list->AddImage(...); } else { draw_entry_icon(...); }` that follows) is unchanged — it already treats any populated `thumbnail_texture` the same way regardless of which cache produced it.

`to_lower` is the same helper `entry_category` itself already calls on this file's extension string
(confirmed by `entry_category`'s own body, a few lines above this block in the same file) — no new
helper needed.

- [ ] **Step 3: Verify by reading, not by building**

Per Global Constraints, this machine cannot build or run the editor. Re-read the edited block
against four cases by hand: (1) a `Folder`/`Scene`/`Prefab`/`Audio`/`Code`/`Text`/`Unknown`
entry — both category checks fail, `thumbnail_texture` stays unset, behavior is byte-for-byte the
same as before this task; (2) a `.fbx`/`.obj` `Model` entry — `category == EntryCategory::Model` is
true but the extension check fails, so `context.model_thumbnail_cache->texture_for` is never
called for it, and it falls through to the wireframe-cube glyph exactly as before this task; (3) a
visible `.gltf`/`.glb` `Model` entry whose thumbnail is not yet resident — `texture_for` returns
`std::nullopt`, so the cube glyph still renders this frame; (4) a visible `.gltf`/`.glb` `Model`
entry whose thumbnail resolved — `AddImage` renders the real thumbnail in place of the glyph, using
the same centering/sizing math the `Image` branch already established in Phase 2.

- [ ] **Step 4: Commit**

```bash
git add applications/editor/source/project/project_panel.cpp
git commit -m "feat(editor): show real model thumbnails in the Project panel's Grid view"
```

---

## Manual verification (after the branch builds)

Once the user has built the branch, verification is the same shape Phase 2 already established:
open a Project folder containing several `.gltf`/`.glb` files (at least one large enough to exceed
`MAX_PRIMITIVES` from Phase 3a, and at least one with a mirrored/negative-scale node if one is
available, to confirm Phase 3a's culling fix holds for that case too) alongside at least one `.fbx`
or `.obj` file, and confirm: `.gltf`/`.glb` tiles show a real, right-side-out rendered thumbnail
within a couple of frames of becoming visible; `.fbx`/`.obj` tiles keep showing the wireframe-cube
glyph indefinitely, never attempting a render; scrolling a large folder of models doesn't stall the
editor (at most one model renders per frame); a deliberately corrupted `.glb` file keeps showing the
cube glyph with no crash and no console spam; browsing more than 64 distinct models in one session
triggers at least one mesh-renderer recreation with no visible glitch and no crash, and thumbnails
rendered before the recreation remain visible and correct afterward; editor shutdown after
rendering several model thumbnails does not crash or leak-report under `--validation`.
