/**************************************************************************/
/* vulkan_prefab_thumbnail_renderer.hpp                                   */
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
 * @file vulkan_prefab_thumbnail_renderer.hpp
 * @brief The Vulkan implementation of IPrefabThumbnailRenderer.
 *
 * Draws exactly the caller-resolved array of (mesh, material, model matrix) triples it is
 * given, framing the camera itself from the union of every draw's mesh bounding sphere. This
 * class knows nothing about prefabs, JSON,
 * or `ISimulation`/`IWorldEditor` -- that orchestration belongs to whatever calls it (Phase
 * 4b's editor-tier `PrefabThumbnailCache`), since `engine/presentation/render` is forbidden
 * from depending on the `world` tier (`engine/world/simulation`/`engine/world/serialization`);
 * see this class's own header (`SushiEngine/render/prefab_thumbnail_renderer.hpp`) for the
 * full rationale. This renderer's own isolated asset stack, exposed via @ref
 * VulkanPrefabThumbnailRenderer::asset_library, is what a caller resolves a prefab's
 * mesh_path/material-path references against before building the draw array; it persists
 * across calls until this whole renderer is destroyed and replaced -- Phase 4b's
 * PrefabThumbnailCache owns that recreation policy, not this class.
 */

#include <cstddef>
#include <cstdint>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/render/prefab_thumbnail_renderer.hpp>

#include "prefab_thumbnail_asset_library.hpp"
#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /** @brief Vulkan implementation of IPrefabThumbnailRenderer; see the file doc comment. */
            class VulkanPrefabThumbnailRenderer final : public IPrefabThumbnailRenderer
            {
                public:
                    /**
                     * @brief Builds this renderer's isolated asset stack and pipeline.
                     * @param device The live device this renderer's own asset stack and
                     *   offscreen resources are built against.
                     * @throws std::runtime_error if the isolated asset stack or this
                     *   renderer's own pipeline cannot be created.
                     */
                    explicit VulkanPrefabThumbnailRenderer(VulkanDevice& device);

                    /** @brief Frees the offscreen targets, the pipeline, and the isolated
                     *  asset stack. */
                    ~VulkanPrefabThumbnailRenderer() override;

                    VulkanPrefabThumbnailRenderer(const VulkanPrefabThumbnailRenderer&) = delete;
                    VulkanPrefabThumbnailRenderer& operator=(const VulkanPrefabThumbnailRenderer&) = delete;

                    IAssetLibrary& asset_library() noexcept override { return assets_; }

                    bool render_thumbnail(const PrefabThumbnailDraw* draws, std::size_t count,
                                          std::uint32_t width, std::uint32_t height,
                                          FrameImage& out_image) override;

                private:
                    static constexpr std::size_t MAX_PRIMITIVES = 128;
                    static constexpr VkFormat COLOR_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
                    static constexpr VkFormat DEPTH_FORMAT = VK_FORMAT_D32_SFLOAT;

                    /** @brief One draw call's worth of per-entity push-constant data (100 bytes total —
                     *  comfortably under Vulkan's guaranteed 128-byte minimum, this repo's own "house budget"
                     *  for push constants; see the .cpp's render_thumbnail for how @c mvp and @c light_object
                     *  are derived). */
                    struct Push
                    {
                        float mvp[16];
                        float light_object[4];
                        float albedo[4];
                        std::int32_t albedo_texture_index;
                    };

                    void create_pipeline();
                    void destroy_pipeline();
                    void ensure_targets(std::uint32_t width, std::uint32_t height);
                    void destroy_targets();

                    VulkanDevice& device_;
                    PrefabThumbnailAssetLibrary assets_;

                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    VkCommandPool command_pool_ = VK_NULL_HANDLE;

                    VkImage color_image_ = VK_NULL_HANDLE;
                    VmaAllocation color_allocation_ = VK_NULL_HANDLE;
                    VkImageView color_view_ = VK_NULL_HANDLE;
                    VkImage depth_image_ = VK_NULL_HANDLE;
                    VmaAllocation depth_allocation_ = VK_NULL_HANDLE;
                    VkImageView depth_view_ = VK_NULL_HANDLE;
                    std::uint32_t target_width_ = 0;
                    std::uint32_t target_height_ = 0;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
