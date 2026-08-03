/**************************************************************************/
/* shading_rate_pass.hpp                                                  */
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
 * @file shading_rate_pass.hpp
 * @brief Builds the per-tile mask that lets a pass shade below one sample per pixel.
 *
 * It runs between the geometry pass and the sky, which is deliberate on two counts.
 * It is the only ordering that is not circular — the mask wants this frame's motion
 * vectors, and the geometry pass is what writes them — and the sky is the pass worth
 * steering: at planet scale the atmosphere and cloud march dominate the frame, while
 * the geometry in front of them is comparatively free.
 *
 * The pass owns its own descriptor and pipeline layouts rather than the shared scene
 * ones, because it is the only compute consumer here and the shared layout exposes its
 * bindings to the graphics stages only.
 */

#include <cstdint>

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Writes the fragment shading rate image for this frame.
             *
             * Registers nothing when the device cannot steer shading from an image or
             * the frame did not ask for it, in which case the rate handle it would have
             * produced stays invalid and the passes that would have bound it simply do
             * not. Non-copyable: it owns Vulkan layout and pipeline objects.
             */
            class ShadingRatePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the mask pipeline and its layouts.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader module comes from.
                     * @param pipelines Factory the pipeline is built through.
                     */
                    ShadingRatePass(Vulkan::VulkanDevice& device,
                                    Resources::ShaderLibrary& shaders,
                                    Resources::GraphicsPipelineFactory& pipelines);
                    ~ShadingRatePass() override;

                    ShadingRatePass(const ShadingRatePass&) = delete;
                    ShadingRatePass& operator=(const ShadingRatePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief Whether this frame's passes may bind a rate image at all.
                     *
                     * The scene view asks before it declares the rate target, so the
                     * decision is made once rather than repeated in every pass.
                     *
                     * @param frame The frame being built.
                     * @return true when the device supports it and the frame asked for it.
                     */
                    bool enabled(const Frame::FrameContext& frame) const noexcept;

                    /** @brief Pixels one rate texel covers horizontally. */
                    std::uint32_t texel_width() const noexcept;

                    /** @brief Pixels one rate texel covers vertically. */
                    std::uint32_t texel_height() const noexcept;

                private:
                    /** @brief Mirrors the mask shader's push constant block. */
                    struct Push
                    {
                        std::uint32_t extents[4];
                        float thresholds[4];
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
