/**************************************************************************/
/* light_cull_pass.hpp                                                    */
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
 * @file light_cull_pass.hpp
 * @brief Builds the froxel cluster grid this frame's punctual lights are culled into.
 *
 * A compute pass that runs before the opaque pass: for every cluster it writes how many
 * lights touch it and their indices, which the shading pass reads back per pixel. It
 * owns its own descriptor and pipeline layouts rather than the shared scene ones,
 * because it is a compute consumer and the shared set exposes its bindings to the
 * graphics stages only — the same reason ShadingRatePass owns its own.
 *
 * The grid buffers are graph transients (the view creates them, this pass writes them,
 * the opaque pass reads them), so the compute→fragment barrier is derived like any
 * other. The light buffer it reads is host-written before the graph runs and bound
 * directly, exactly as the material and motion arrays are.
 */

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

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Passes
        {
            /**
             * @brief Culls the frame's punctual lights into the froxel cluster grid.
             *
             * Non-copyable: it owns Vulkan layout and pipeline objects.
             */
            class LightCullPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the cull pipeline and its layouts.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader module comes from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param lights    The per-frame light buffer and cluster config.
                     */
                    LightCullPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                  Resources::GraphicsPipelineFactory& pipelines,
                                  Lighting::LightSystem& lights);
                    ~LightCullPass() override;

                    LightCullPass(const LightCullPass&) = delete;
                    LightCullPass& operator=(const LightCullPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief Mirrors the cull shader's push constant block. */
                    struct Push
                    {
                        float right[4];   /**< xyz = unit right, w = tan(fovx/2). */
                        float up[4];      /**< xyz = unit up, w = tan(fovy/2). */
                        float forward[4]; /**< xyz = unit forward, w = light count. */
                        float params[4];  /**< x = near, y = far, zw spare. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Lighting::LightSystem& lights_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
