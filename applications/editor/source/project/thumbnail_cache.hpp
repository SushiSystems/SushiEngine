/**************************************************************************/
/* thumbnail_cache.hpp                                                    */
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
 * @file thumbnail_cache.hpp
 * @brief Real image content for the Project panel's Grid view tiles.
 *
 * The second editor component that legitimately speaks Vulkan directly (the first is
 * ImGuiBackend) — everything here is the minimal, from-scratch image/view/sampler/staging-buffer
 * path a 128x128 thumbnail needs, deliberately not routed through the renderer's own
 * TextureLibrary, which only ever hands back a bindless heap index rather than the raw
 * VkImageView/VkSampler pair ImGui::Image needs. A background thread decodes and downscales;
 * update() uploads a small budget of finished decodes to the GPU each frame; an LRU keeps
 * resident GPU memory bounded. See docs/superpowers/specs/2026-08-07-project-panel-thumbnails-design.md.
 */

#include <condition_variable>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <imgui.h>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/imaging/lru_cache.hpp>
#include <SushiEngine/render/rhi/device.hpp>

#include "../core/console.hpp"
#include "../ui/imgui_backend.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Decodes, downscales, and GPU-uploads image thumbnails on a background thread.
         *
         * Non-copyable: it owns a live worker thread and a set of Vulkan resources tied to one
         * device. Construction starts the worker; destruction stops it, joins it, and frees
         * every resident thumbnail's Vulkan resources before the device it was built against
         * may be torn down — the caller (main()) must destroy this before the renderer.
         */
        class ThumbnailCache
        {
            public:
                /**
                 * @brief Binds the cache to a device and starts its decode worker.
                 * @param handles The window renderer's native handles (needs @c device,
                 *   @c allocator, @c graphics_queue, @c graphics_queue_family all set).
                 * @param backend The ImGui Vulkan backend thumbnails are registered with.
                 * @param console Where an upload failure is logged (see class docs).
                 * @throws std::runtime_error if the internal Vulkan command pool cannot be
                 *   created.
                 */
                ThumbnailCache(SushiEngine::Render::NativeDeviceHandles handles,
                               ImGuiBackend& backend, Console& console);

                /** @brief Stops and joins the worker, then frees every resident thumbnail. */
                ~ThumbnailCache();

                ThumbnailCache(const ThumbnailCache&) = delete;
                ThumbnailCache& operator=(const ThumbnailCache&) = delete;

                /**
                 * @brief Uploads a small, fixed budget of finished decodes to the GPU.
                 *
                 * Call once per frame, before any panel that might call @ref texture_for reads
                 * its result for this frame.
                 */
                void update();

                /**
                 * @brief The thumbnail texture for @p path, requesting a decode if needed.
                 * @return A texture id if @p path's thumbnail is already resident; otherwise
                 *   @c std::nullopt, having enqueued a decode request unless one is already
                 *   in flight for the same path. A path whose decode failed keeps returning
                 *   @c std::nullopt forever (see class docs on failure handling).
                 */
                std::optional<ImTextureID> texture_for(const std::filesystem::path& path);

            private:
                /** @brief One decoded, downscaled thumbnail awaiting a GPU upload. */
                struct DecodedImage
                {
                    std::string path;
                    std::vector<std::uint8_t> pixels; // THUMBNAIL_SIZE^2 * 4 bytes, RGBA8.
                };

                /** @brief One thumbnail's live Vulkan resources plus its ImGui texture id. */
                struct ResidentThumbnail
                {
                    VkImage image = VK_NULL_HANDLE;
                    VmaAllocation allocation = VK_NULL_HANDLE;
                    VkImageView view = VK_NULL_HANDLE;
                    VkSampler sampler = VK_NULL_HANDLE;
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

                void worker_main();
                void upload_one(const DecodedImage& decoded);
                void destroy_thumbnail(ResidentThumbnail& thumbnail);

                static constexpr std::uint32_t THUMBNAIL_SIZE = 128;
                static constexpr std::size_t RESIDENT_CAPACITY = 256;
                static constexpr int UPLOADS_PER_FRAME = 2;
                // Frames to hold an evicted thumbnail's Vulkan resources before actually
                // destroying them. The renderer's real frames-in-flight count is not threaded
                // through to this class, so this is a conservative fixed upper bound (typical
                // renderers of this kind run 2-3 frames in flight) rather than the exact value.
                static constexpr int EVICTION_DELAY_FRAMES = 4;

                VkDevice device_ = VK_NULL_HANDLE;
                VmaAllocator allocator_ = VK_NULL_HANDLE;
                VkQueue graphics_queue_ = VK_NULL_HANDLE;
                VkCommandPool command_pool_ = VK_NULL_HANDLE;
                ImGuiBackend& backend_;
                Console& console_;

                std::thread worker_;
                std::mutex requests_mutex_;
                std::condition_variable requests_cv_;
                std::deque<std::string> requests_;
                bool stop_ = false;

                std::mutex ready_mutex_;
                std::deque<DecodedImage> ready_;

                // Main-thread-only state; never touched from worker_.
                SushiEngine::Imaging::LruCache<std::string, ResidentThumbnail> resident_{
                    RESIDENT_CAPACITY};
                std::unordered_map<std::string, bool> in_flight_;
                // Paths whose upload permanently failed (as opposed to a decode failure, which
                // never leaves in_flight_ once stbi_load fails on the worker thread). Checked by
                // texture_for() so a permanently-failed path is never re-enqueued every frame.
                std::unordered_set<std::string> failed_uploads_;
                std::uint64_t frame_counter_ = 0;
                std::deque<PendingEviction> pending_evictions_;
        };
    } // namespace Editor
} // namespace SushiEngine
