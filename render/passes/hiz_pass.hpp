/**************************************************************************/
/* hiz_pass.hpp                                                           */
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
 * @file hiz_pass.hpp
 * @brief The hierarchical-Z pyramid: a nearest-depth mip chain for screen-space tracing.
 *
 * A pass-owned image (not a graph transient — the render graph exposes no per-mip storage
 * view, and each mip must be bound as its own storage image to be written, so this follows
 * the IBL pass's model of owning the image and driving its own mip barriers). Level 0
 * linearises the depth prepass; each finer level is the 2x2 minimum of the one above. The
 * result is sampled with `textureLod` by the SSR trace (§5.3) and, later, Phase 10's GPU
 * occlusion culling — shared infrastructure built once. The image lives at the render
 * extent and is rebuilt when that changes.
 */

#include <cstdint>
#include <vector>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include "passes/render_pass.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            class ShaderLibrary;
            class GraphicsPipelineFactory;
            class SamplerCache;
        }

        namespace Passes
        {
            /** @brief Builds the frame's hierarchical-Z (nearest-depth) pyramid. */
            class HizPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the reduction pipeline (the image is built lazily).
                     * @param device    The live Vulkan device.
                     * @param shaders   The shader catalogue the reduction module comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     */
                    HizPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                            Resources::GraphicsPipelineFactory& pipelines);
                    ~HizPass() override;

                    HizPass(const HizPass&) = delete;
                    HizPass& operator=(const HizPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The full-chain view the SSR trace samples with textureLod. */
                    VkImageView pyramid_view() const noexcept { return sample_view_; }

                    /** @brief Whether a pyramid has been built and is safe to sample. */
                    bool valid() const noexcept { return image_ != VK_NULL_HANDLE; }

                    /** @brief Levels in the current pyramid. */
                    std::uint32_t mip_count() const noexcept { return mips_; }

                private:
                    struct Push
                    {
                        std::uint32_t a[4]; /**< level, dst_w, dst_h, src_w. */
                        std::uint32_t b[4]; /**< src_h. */
                        float c[4];         /**< near. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();
                    void create_image(std::uint32_t width, std::uint32_t height);
                    void destroy_image();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    VkImage image_ = VK_NULL_HANDLE;
                    VmaAllocation allocation_ = VK_NULL_HANDLE;
                    VkImageView sample_view_ = VK_NULL_HANDLE;
                    std::vector<VkImageView> mip_views_;
                    std::uint32_t width_ = 0;
                    std::uint32_t height_ = 0;
                    std::uint32_t mips_ = 0;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
