/**************************************************************************/
/* prefab_thumbnail_asset_library.hpp                                     */
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
 * @file prefab_thumbnail_asset_library.hpp
 * @brief A minimal, cache-free IAssetLibrary for a single isolated renderer's own use.
 *
 * `Scene::resolve_scene_assets` (what turns a prefab's serialized `mesh_path`/material-path
 * strings into live, drawable handles) takes an `IAssetLibrary&` — there is no lighter-weight
 * resolution path. This is the first `IAssetLibrary` implementation in this codebase besides
 * the main renderer's own `Assets::AssetLibrary`. Unlike that one, it implements none of the
 * by-path caching the interface's docs describe as a nice-to-have for a shared, session-long
 * asset store: this class exists for exactly one isolated renderer's own private mesh/texture
 * stack, and its only caller resolves one prefab at a time, so a caching layer here would add
 * complexity with nothing to amortize.
 */

#include <SushiEngine/render/asset_library_interface.hpp>

#include "../../geometry/mesh_registry.hpp"
#include "../../material/texture_library.hpp"
#include "../../resources/descriptor_heap.hpp"
#include "../../resources/sampler_cache.hpp"
#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief A cache-free IAssetLibrary backed by its own private mesh/texture stack.
             *
             * Non-copyable: owns Vulkan resources. Construction builds an empty stack;
             * destruction frees everything it ever imported.
             */
            class PrefabThumbnailAssetLibrary final : public IAssetLibrary
            {
                public:
                    /**
                     * @brief Builds an empty isolated asset stack against @p device.
                     * @param device The live device this stack's resources are built against.
                     * @throws std::runtime_error if the descriptor heap this stack needs is
                     *   unavailable on this device (no descriptor-indexing support), or if any
                     *   other Vulkan resource creation fails.
                     */
                    explicit PrefabThumbnailAssetLibrary(VulkanDevice& device);
                    ~PrefabThumbnailAssetLibrary() override = default;

                    PrefabThumbnailAssetLibrary(const PrefabThumbnailAssetLibrary&) = delete;
                    PrefabThumbnailAssetLibrary& operator=(const PrefabThumbnailAssetLibrary&) = delete;

                    TextureId load_texture(const char* path, TextureColorSpace color_space) override;
                    void release_texture(TextureId texture) override;
                    std::size_t load_gltf(const char* path, MeshId* meshes, Material* materials,
                                         std::size_t count) override;
                    std::size_t load_gltf_scene(const char* path, ImportedPrimitive* out,
                                               std::size_t capacity) override;
                    std::size_t load_gltf_skinned_mesh(const char* path, std::size_t skin_index,
                                                      MeshId* meshes, Material* materials,
                                                      std::size_t count) override;
                    std::size_t resident_texture_bytes() const noexcept override;
                    std::uint32_t morph_target_count(MeshId mesh) const noexcept override;

                    /**
                     * @brief The mesh registry this stack's imports land in.
                     *
                     * Exposed so a caller that already holds a `MeshId` this library resolved
                     * (e.g. from `Scene::resolve_scene_assets` writing one onto a prefab
                     * entity's Shape) can look up its actual Vulkan vertex/index buffers to
                     * draw it — `IAssetLibrary` itself deliberately exposes no such accessor,
                     * since the interface is meant to stay renderer-implementation-agnostic.
                     */
                    Geometry::MeshRegistry& meshes() noexcept { return meshes_; }

                    /**
                     * @brief This stack's bindless descriptor heap, for a renderer's pipeline
                     *   layout to bind as its second descriptor set (matching
                     *   IMeshThumbnailRenderer's Phase 3a precedent exactly).
                     */
                    Resources::DescriptorHeap& heap() noexcept { return heap_; }

                private:
                    static constexpr std::uint32_t HEAP_TEXTURE_CAPACITY = 256;
                    static constexpr std::uint32_t HEAP_BUFFER_CAPACITY = 16;
                    static constexpr std::size_t TEXTURE_BUDGET_BYTES = 64u * 1024u * 1024u;

                    Resources::SamplerCache samplers_;
                    Resources::DescriptorHeap heap_;
                    Geometry::MeshRegistry meshes_;
                    Assets::TextureLibrary textures_;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
