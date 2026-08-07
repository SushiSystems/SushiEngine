/**************************************************************************/
/* vulkan_mesh_thumbnail_renderer.hpp                                     */
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
 * @file vulkan_mesh_thumbnail_renderer.hpp
 * @brief The Vulkan implementation of IMeshThumbnailRenderer.
 *
 * Owns an asset stack isolated from the main renderer's (its own sampler cache, bindless
 * descriptor heap, mesh registry, and texture library, built the same way
 * Assets::AssetLibrary builds the main renderer's copies) plus one fixed, hand-built
 * graphics pipeline. Unlike the renderer's other pipelines, this one is never rebuilt
 * against different shaders or reused across a diverse pipeline set, so it is created
 * directly with vkCreateGraphicsPipelines rather than through GraphicsPipelineFactory --
 * that class exists to amortize work across many pipelines over a session, which does not
 * apply to a renderer that only ever builds the one pipeline it needs, once.
 */

#include <cstdint>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/render/mesh_thumbnail_renderer.hpp>

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
            /** @brief Vulkan implementation of IMeshThumbnailRenderer; see the file doc comment. */
            class VulkanMeshThumbnailRenderer final : public IMeshThumbnailRenderer
            {
                public:
                    /**
                     * @brief Builds this renderer's isolated asset stack and pipeline.
                     * @param device The live device this renderer's own asset stack and
                     *   offscreen resources are built against.
                     * @throws std::runtime_error on any Vulkan resource creation failure.
                     */
                    explicit VulkanMeshThumbnailRenderer(VulkanDevice& device);

                    /** @brief Frees the offscreen targets, the pipeline, and this renderer's
                     *  own isolated asset stack. */
                    ~VulkanMeshThumbnailRenderer() override;

                    VulkanMeshThumbnailRenderer(const VulkanMeshThumbnailRenderer&) = delete;
                    VulkanMeshThumbnailRenderer& operator=(const VulkanMeshThumbnailRenderer&) = delete;

                    bool render_thumbnail(const char* path, std::uint32_t width,
                                          std::uint32_t height, FrameImage& out_image) override;

                private:
                    // A model with more primitives than this is treated as a load failure --
                    // out_meshes/out_materials are fixed-capacity arrays, not vectors, matching
                    // import_gltf's own existing (unchanged by this plan) signature.
                    static constexpr std::size_t MAX_PRIMITIVES = 64;
                    static constexpr std::uint32_t HEAP_TEXTURE_CAPACITY = 256;
                    static constexpr std::uint32_t HEAP_BUFFER_CAPACITY = 16;
                    static constexpr std::size_t TEXTURE_BUDGET_BYTES = 64u * 1024u * 1024u;
                    static constexpr VkFormat COLOR_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
                    static constexpr VkFormat DEPTH_FORMAT = VK_FORMAT_D32_SFLOAT;

                    /** @brief One draw call's worth of per-primitive push-constant data. */
                    struct Push
                    {
                        float view_projection[16];
                        float albedo[4];
                        std::int32_t albedo_texture_index;
                    };

                    void create_pipeline();
                    void destroy_pipeline();
                    void ensure_targets(std::uint32_t width, std::uint32_t height);
                    void destroy_targets();
                    bool copy_output_to_cpu(std::uint32_t width, std::uint32_t height,
                                            FrameImage& out_image);

                    VulkanDevice& device_;

                    // This renderer's own asset stack, isolated from the main renderer's --
                    // see the class doc comment.
                    Resources::SamplerCache samplers_;
                    Resources::DescriptorHeap heap_;
                    Geometry::MeshRegistry meshes_;
                    Assets::TextureLibrary textures_;

                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    VkCommandPool command_pool_ = VK_NULL_HANDLE;

                    // Offscreen render targets, sized to the largest request seen so far and
                    // reused across calls; recreated only when a larger size is requested.
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
