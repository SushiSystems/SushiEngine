/**************************************************************************/
/* bloom_pass.hpp                                                         */
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
 * @file bloom_pass.hpp
 * @brief Energy-conserving bloom: a progressive down/up-sampled mip pyramid.
 *
 * The pass owns two HDR mip pyramids — the graph exposes no per-mip storage view, so it
 * owns the images and barriers each level by hand, on the hi-Z pass's model. The scene is
 * downsampled with a 13-tap Karis-averaged filter, upsampled back with a 3x3 tent, and the
 * final level is written into a graph-transient bloom target the display transform composites
 * in. Threshold-free by default: the whole HDR image scatters, so a bright pixel bleeds in
 * proportion to its own intensity and no energy is invented at a hard cut.
 */

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "passes/render_pass.hpp"
#include "resources/pipeline_cache.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /** @brief Builds the bloom pyramid and writes it into the frame's bloom target. */
            class BloomPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the down/up compute pipelines and their shared layout.
                     * @param device    The live Vulkan device.
                     * @param shaders   The shader library the compute modules come from.
                     * @param pipelines The factory the compute pipelines are built by.
                     */
                    BloomPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                              Resources::GraphicsPipelineFactory& pipelines);
                    ~BloomPass() override;

                    BloomPass(const BloomPass&) = delete;
                    BloomPass& operator=(const BloomPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief One owned HDR mip pyramid: an image, its views, and its extent. */
                    struct Pyramid
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView sample_view = VK_NULL_HANDLE;
                        std::vector<VkImageView> mip_views;
                    };

                    /** @brief The compute push block, shared by the down and up shaders. */
                    struct Push
                    {
                        float a[4];
                        float b[4];
                    };

                    void create_pipelines();
                    void destroy_pipelines();
                    void create_images(std::uint32_t width, std::uint32_t height);
                    void destroy_images();
                    void create_pyramid(Pyramid& pyramid);
                    void destroy_pyramid(Pyramid& pyramid);

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline down_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline up_pipeline_ = VK_NULL_HANDLE;

                    Pyramid down_;
                    Pyramid up_;
                    std::uint32_t base_width_ = 0;  /**< Half the source width: the pyramid's mip 0. */
                    std::uint32_t base_height_ = 0;
                    std::uint32_t mips_ = 0;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
