/**************************************************************************/
/* prefab_thumbnail_cache.hpp                                             */
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
 * @file prefab_thumbnail_cache.hpp
 * @brief Real prefab content for the Project panel's Grid view tiles.
 *
 * The editor-tier counterpart to Phase 3b's ModelThumbnailCache, for `.sushiprefab` files: per
 * request it reads and parses the prefab file, instantiates it into a fresh, disposable
 * `ISimulation`, resolves its assets against an `IPrefabThumbnailRenderer`'s own isolated asset
 * library, walks the resolved entities into a `PrefabThumbnailDraw` array, asks the renderer to
 * draw and read the pixels back, then uploads those pixels to its own small resident GPU
 * texture for ImGui -- exactly the way ModelThumbnailCache uploads a rendered model. Unlike
 * ModelThumbnailCache, a single request also does real orchestration work before the render
 * call: `IPrefabThumbnailRenderer` (`engine/presentation/render`) is forbidden from doing any of
 * that itself, since it lives in a tier below `engine/world` (see its own header for the full
 * rationale). `IMeshThumbnailRenderer`/`IPrefabThumbnailRenderer`'s synchronous, blocking
 * contract carries over unchanged: there is no background thread, and `update()` does the whole
 * build+render+upload sequence for at most one prefab per frame, on the thread that owns the
 * main renderer's graphics queue.
 *
 * `IPrefabThumbnailRenderer`'s isolated mesh/texture stack has no removal API (documented on
 * `VulkanPrefabThumbnailRenderer` itself) and grows without bound as distinct meshes/textures
 * are rendered through it -- the same constraint `IMeshThumbnailRenderer` documents. This
 * class's mitigation is identical to `ModelThumbnailCache`'s: after a fixed number of successful
 * renders, it destroys its `IPrefabThumbnailRenderer` and creates a fresh one, reclaiming that
 * memory. This does not affect any thumbnail already resident here -- those are independent GPU
 * textures this class owns directly, unrelated to the prefab renderer's own internal resources.
 *
 * See docs/superpowers/specs/2026-08-08-project-panel-prefab-thumbnails-cache-design.md.
 */

#include <cstddef>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <imgui.h>
#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/imaging/lru_cache.hpp>
#include <SushiEngine/render/prefab_thumbnail_renderer.hpp>
#include <SushiEngine/render/window_renderer.hpp>

#include "../core/console.hpp"
#include "../ui/imgui_backend.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Real prefab thumbnails for the Project panel's Grid view, one render per frame.
         *
         * Non-copyable: it owns a live `IPrefabThumbnailRenderer` and a set of Vulkan resources
         * tied to one device. Construction builds the first prefab renderer; destruction frees
         * every resident thumbnail's Vulkan resources.
         */
        class PrefabThumbnailCache
        {
            public:
                /**
                 * @brief Builds the cache's first prefab renderer.
                 * @param renderer The window renderer this cache's prefab renderers are created
                 *   from, and whose device its own resident-texture Vulkan resources are built
                 *   against.
                 * @param backend The ImGui Vulkan backend thumbnails are registered with.
                 * @param console Where an upload failure is logged (see class docs).
                 * @throws std::runtime_error if the first prefab renderer cannot be created (e.g.
                 *   this device lacks the descriptor-indexing support it needs, or its pipeline
                 *   fails to build).
                 */
                PrefabThumbnailCache(SushiEngine::Render::IWindowRenderer& renderer,
                                     ImGuiBackend& backend, Console& console);

                /** @brief Frees every resident thumbnail's Vulkan resources. */
                ~PrefabThumbnailCache();

                PrefabThumbnailCache(const PrefabThumbnailCache&) = delete;
                PrefabThumbnailCache& operator=(const PrefabThumbnailCache&) = delete;

                /**
                 * @brief Renders and uploads at most one queued prefab this frame.
                 *
                 * Call once per frame, before any panel that might call @ref texture_for reads
                 * its result for this frame.
                 */
                void update();

                /**
                 * @brief The thumbnail texture for @p path, requesting a render if needed.
                 * @return A texture id if @p path's thumbnail is already resident; otherwise
                 *   @c std::nullopt, having enqueued a render request unless one is already
                 *   pending for the same path. A path whose build/render/upload failed keeps
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

                /**
                 * @brief Reads, instantiates, resolves, and walks @p path into a draw array.
                 * @param path The `.sushiprefab` file to build draws from.
                 * @param out  Cleared, then filled with one entry per mesh-backed entity the
                 *   prefab resolves to.
                 * @return @c false if the file cannot be read/parsed, holds no entity, resolves
                 *   to zero mesh-backed entities, or would exceed the renderer's @c max_draws();
                 *   @c true otherwise. Never throws -- every exception from JSON parsing,
                 *   simulation creation, or asset resolution is caught and converted to @c false.
                 */
                bool build_draws(const std::string& path,
                                 std::vector<SushiEngine::Render::PrefabThumbnailDraw>& out);
                void upload_one(const std::string& path);
                void destroy_thumbnail(ResidentThumbnail& thumbnail);
                void recreate_prefab_renderer();

                static constexpr std::uint32_t THUMBNAIL_SIZE = 128;
                static constexpr std::size_t RESIDENT_CAPACITY = 128;
                static constexpr int EVICTION_DELAY_FRAMES = 4;
                // After this many successful renders through one IPrefabThumbnailRenderer, it is
                // destroyed and replaced to reclaim its unbounded-growth isolated asset stack
                // (see class docs). Smaller than ModelThumbnailCache's 64: a single prefab
                // render can pull in more distinct meshes/textures per call than a single-model
                // render, so the isolated asset stack fills faster per successful render.
                static constexpr std::size_t PREFABS_PER_RENDERER_LIFETIME = 24;

                SushiEngine::Render::IWindowRenderer& renderer_;
                std::unique_ptr<SushiEngine::Render::IPrefabThumbnailRenderer> prefab_renderer_;
                std::size_t prefabs_rendered_since_recreation_ = 0;

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
