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
