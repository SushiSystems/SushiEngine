/**************************************************************************/
/* cloud_taa_pass.hpp                                                    */
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
 * @file cloud_taa_pass.hpp
 * @brief W3's dedicated cloud temporal resolve (War Thunder's shipped scheme).
 *
 * `CloudPass`'s march is jittered and half the pixels of the main scene, so it needs
 * its own temporal accumulation before `CloudCompositePass` folds it over the sky —
 * the main `taa_pass_` runs later, at the end of the frame, and only ever sees the
 * already-composited, already-resolved cloud contribution as an ordinary shaded pixel.
 *
 * The resolved colour and a per-pixel history-acceptance weight are pass-owned,
 * ping-ponged by frame parity exactly like `ViewResources`'s main history (see that
 * file), except sized at a *fixed* half of the view's output extent rather than the
 * dynamic internal render extent: the march itself already resamples across whatever
 * `QualityParams::cloud_buffer_scale` and dynamic resolution put it at, the same
 * render/output-extent split `taa.frag` already reconciles for the main resolve, so
 * this history is insulated from both without forcing a resize on every dynamic-
 * resolution step. The images are private to this pass and barriered by hand, exactly
 * as the T3 field, the light volume, and the shadow map are.
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
            /**
             * @brief Builds and owns the cloud buffer's own YCoCg variance-clip TAA.
             *
             * Non-copyable: it owns images, views, and a compute pipeline.
             */
            class CloudTaaPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the resolve pipeline and the first pair of history images.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the resolve compute module comes from.
                     * @param pipelines Factory the compute pipeline is built through.
                     * @param samplers  Cache providing the history/current read samplers.
                     * @param output_width  Initial view output width in pixels.
                     * @param output_height Initial view output height in pixels.
                     */
                    CloudTaaPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                Resources::GraphicsPipelineFactory& pipelines,
                                Resources::SamplerCache& samplers, std::uint32_t output_width,
                                std::uint32_t output_height);
                    ~CloudTaaPass() override;

                    CloudTaaPass(const CloudTaaPass&) = delete;
                    CloudTaaPass& operator=(const CloudTaaPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief Reallocates the history at a new output extent.
                     *
                     * Called from `VulkanSceneView::resize()`, which has already idled the
                     * device before touching `ViewResources` — the same safety this pass's
                     * own history needs, since it is not graph-tracked.
                     * @param output_width  New view output width in pixels.
                     * @param output_height New view output height in pixels.
                     */
                    void resize(std::uint32_t output_width, std::uint32_t output_height);

                    /** @brief This frame's resolved cloud colour: scattered.rgb, transmittance. */
                    VkImageView color_view() const noexcept { return current_color_view_; }

                    /** @brief The clamped, linear sampler the resolved colour is read through. */
                    VkSampler sampler() const noexcept { return sampler_; }

                private:
                    /** @brief One ping-pong slot: its colour and weight images. */
                    struct Slot
                    {
                        VkImage color_image = VK_NULL_HANDLE;
                        VmaAllocation color_allocation = VK_NULL_HANDLE;
                        VkImageView color_view = VK_NULL_HANDLE;
                        VkImage weight_image = VK_NULL_HANDLE;
                        VmaAllocation weight_allocation = VK_NULL_HANDLE;
                        VkImageView weight_view = VK_NULL_HANDLE;
                    };

                    /** @brief The resolve shader's push block. */
                    struct Push
                    {
                        std::uint32_t history_valid;
                        std::uint32_t variance_clip;
                    };

                    void create_history();
                    void destroy_history();
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;

                    Slot slots_[2];
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    VkImageView current_color_view_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;

                    std::uint32_t width_ = 1;
                    std::uint32_t height_ = 1;
                    bool built_ = false;
                    /** @brief Cleared whenever clouds were off last frame, so a stale history is never blended in. */
                    bool history_valid_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
