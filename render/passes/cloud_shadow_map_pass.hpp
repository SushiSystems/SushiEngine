/**************************************************************************/
/* cloud_shadow_map_pass.hpp                                             */
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
 * @file cloud_shadow_map_pass.hpp
 * @brief The W2 unified cloud shadow map: one baked authority for every consumer.
 *
 * Bakes a 768x768 map of the cloud stack's optical depth toward the sun, projected over
 * the T3 field's own flat, wind-neutral tile — replacing both `cloud_ground_shadow`'s
 * private per-pixel march in `sky.frag` (its own six-deck loop, independent of the T3
 * field) and `cloud_shadow_common.glsl`'s weather-map approximation for meshes with one
 * mechanism, cheaper than either. Refreshed a row group at a time across 8 frames, the
 * same amortization cadence as `CloudLightVolumePass` — both exist to track the sun. The
 * image is pass-owned and barriered by hand, exactly as the T3 field and light volume are.
 */

#include "passes/render_pass.hpp"

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            class CloudscapeCompilePass;

            /**
             * @brief Builds and owns the baked cloud shadow map.
             *
             * Non-copyable: it owns an image, a view, and a compute pipeline.
             */
            class CloudShadowMapPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the map image and its bake pipeline.
                     * @param device     The live Vulkan device.
                     * @param shaders    Library the bake compute module comes from.
                     * @param pipelines  Factory the compute pipeline is built through.
                     * @param samplers   Cache providing the map's tiling sampler.
                     * @param cloudscape The pass that owns the density field this bake reads.
                     */
                    CloudShadowMapPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Resources::SamplerCache& samplers,
                                       CloudscapeCompilePass& cloudscape);
                    ~CloudShadowMapPass() override;

                    CloudShadowMapPass(const CloudShadowMapPass&) = delete;
                    CloudShadowMapPass& operator=(const CloudShadowMapPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The baked map every shadow consumer samples. */
                    VkImageView view() const noexcept { return map_.view; }

                    /** @brief The linear, REPEAT sampler the map tiles under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief The baked map: its image, view, and allocation. */
                    struct Map
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                    };

                    /** @brief The bake shader's push block. */
                    struct Push
                    {
                        float tile_meters;
                        std::uint32_t row_start;
                        std::uint32_t row_count;
                    };

                    void create_map();
                    void destroy_map();
                    void create_pipeline();
                    void destroy_pipeline();

                    /** @brief Per-axis resolution: design doc §4.3's 768^2 cloud shadow map. */
                    static constexpr std::uint32_t RESOLUTION = 768;
                    /** @brief How many frames a full refresh cycle spreads across. */
                    static constexpr std::uint32_t AMORTIZE_FRAMES = 8;
                    static constexpr std::uint32_t ROW_COUNT = RESOLUTION / AMORTIZE_FRAMES;

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    CloudscapeCompilePass& cloudscape_;

                    Map map_;
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    std::uint32_t frame_counter_ = 0;
                    bool built_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
