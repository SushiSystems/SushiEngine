/**************************************************************************/
/* cloud_light_volume_pass.hpp                                            */
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
 * @file cloud_light_volume_pass.hpp
 * @brief The W2 amortized light volume: summed cloud density toward the sun, baked.
 *
 * Nubis3's decoupled-lighting trick: instead of every lit view-march sample paying for
 * `cloud_light_march`'s multi-tap cone march, this bakes the same physical quantity
 * (optical depth toward the sun) into a 256x256x32 volume sharing the T3 field's flat,
 * wind-neutral tile and (u, v, height01) addressing. Refreshed a Y-slice group at a time
 * across 8 frames rather than change-gated like `CloudscapeCompilePass` — the sun moves
 * continuously (the diurnal cycle), so there is no "settled" state to gate on; instead
 * the bake amortizes across frames and accepts up to an 8-frame lag as the sun drifts,
 * which is invisible at the sun's own angular rate. The images are private to this pass
 * and barriered by hand, exactly as the atmosphere LUTs and the T3 field are.
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
             * @brief Builds and owns the baked cloud light volume.
             *
             * Non-copyable: it owns an image, a view, and a compute pipeline.
             */
            class CloudLightVolumePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the volume image and its bake pipeline.
                     * @param device     The live Vulkan device.
                     * @param shaders    Library the bake compute module comes from.
                     * @param pipelines  Factory the compute pipeline is built through.
                     * @param samplers   Cache providing the volume's tiling sampler.
                     * @param cloudscape The pass that owns the density field this bake reads.
                     */
                    CloudLightVolumePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                         Resources::GraphicsPipelineFactory& pipelines,
                                         Resources::SamplerCache& samplers,
                                         CloudscapeCompilePass& cloudscape);
                    ~CloudLightVolumePass() override;

                    CloudLightVolumePass(const CloudLightVolumePass&) = delete;
                    CloudLightVolumePass& operator=(const CloudLightVolumePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The baked volume the view march samples for its base lighting. */
                    VkImageView view() const noexcept { return volume_.view; }

                    /** @brief The linear, REPEAT sampler the volume tiles under. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief The baked volume: its image, view, and allocation. */
                    struct Volume
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t width = 0;
                        std::uint32_t height = 0;
                        std::uint32_t depth = 0;
                    };

                    /** @brief The bake shader's push block. */
                    struct Push
                    {
                        float span_meters;
                        std::uint32_t y_slice_start;
                        std::uint32_t y_slice_count;
                    };

                    void create_volume();
                    void destroy_volume();
                    void create_pipeline();
                    void destroy_pipeline();

                    /** @brief Per-axis resolution: matches the T3 field's own XZ/Y split. */
                    static constexpr std::uint32_t RESOLUTION_XZ = 256;
                    static constexpr std::uint32_t RESOLUTION_Y = 32;
                    /** @brief How many frames a full refresh cycle spreads across. */
                    static constexpr std::uint32_t AMORTIZE_FRAMES = 8;
                    static constexpr std::uint32_t Y_SLICE_COUNT = RESOLUTION_Y / AMORTIZE_FRAMES;

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    CloudscapeCompilePass& cloudscape_;

                    Volume volume_;
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
