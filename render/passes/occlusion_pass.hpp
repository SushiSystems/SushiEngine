/**************************************************************************/
/* occlusion_pass.hpp                                                     */
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
 * @file occlusion_pass.hpp
 * @brief The farthest-depth pyramid the GPU-driven cull tests occlusion against.
 *
 * A pass-owned, persistent mip chain of the *maximum* linear view depth in each footprint —
 * the conservative twin of the HizPass nearest-depth pyramid, which is right for reflection
 * marching and wrong for culling. It is built after the depth prepass each frame and read at
 * the *start* of the next frame by the cull pass, so an instance is tested against the depth
 * the last frame actually rendered (the standard single-phase GPU occlusion; the two-phase
 * re-test that removes the one-frame disocclusion latency is a documented later refinement).
 *
 * Because the cull reads it before this frame rebuilds it, the image persists across frames
 * and lives outside the render graph. On the first frame after a resize it holds no valid
 * depth, so the cull clears it to "far" (nothing occludes) via @ref prepare_sampling, which
 * self-corrects the next frame — no popping, no readback.
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
        }

        namespace Passes
        {
            /** @brief Builds the frame's farthest-depth occlusion pyramid for GPU culling. */
            class OcclusionPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the reduction pipeline (the image is built on demand).
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue the reduction module comes from.
                     * @param pipelines The factory owning the compute pipeline.
                     */
                    OcclusionPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                  Resources::GraphicsPipelineFactory& pipelines);
                    ~OcclusionPass() override;

                    OcclusionPass(const OcclusionPass&) = delete;
                    OcclusionPass& operator=(const OcclusionPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief Ensures the pyramid image exists at the given render extent.
                     *
                     * Called before the graph is built, so the cull pass — which registers
                     * before this one but reads the pyramid — always has a valid image to bind.
                     * A (re)created image is marked dirty so the cull clears it before use.
                     *
                     * @param width  The render width to size the pyramid to.
                     * @param height The render height.
                     */
                    void ensure_extent(std::uint32_t width, std::uint32_t height);

                    /**
                     * @brief Makes the pyramid readable by the cull compute this frame.
                     *
                     * Recorded at the top of the cull pass. On a freshly (re)created image it
                     * clears the whole chain to a far distance so nothing occludes until real
                     * depth lands next frame; otherwise it just barriers the previous frame's
                     * build to be visible to the sampling read.
                     *
                     * @param cmd The command buffer the cull pass is recording into.
                     */
                    void prepare_sampling(VkCommandBuffer cmd);

                    /** @brief The full-chain view the cull samples with textureLod. */
                    VkImageView pyramid_view() const noexcept { return sample_view_; }

                    /** @brief Whether an image exists to bind. */
                    bool valid() const noexcept { return image_ != VK_NULL_HANDLE; }

                    /** @brief Levels in the current pyramid. */
                    std::uint32_t mip_count() const noexcept { return mips_; }

                    /** @brief The pyramid's base width. */
                    std::uint32_t width() const noexcept { return width_; }

                    /** @brief The pyramid's base height. */
                    std::uint32_t height() const noexcept { return height_; }

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
                    bool dirty_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
