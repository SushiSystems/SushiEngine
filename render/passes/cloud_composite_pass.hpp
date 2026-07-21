/**************************************************************************/
/* cloud_composite_pass.hpp                                               */
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
 * @file cloud_composite_pass.hpp
 * @brief Resolves the half-resolution cloud march over the sky into one HDR image.
 *
 * The frame's last purely-spatial step. It exists as its own pass because everything
 * after it — the temporal resolve, and in time the whole post-processing stack — needs
 * a complete linear HDR scene to work on, and everything before it produces only part
 * of one.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Composites the cloud target over the sky target.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class CloudCompositePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the composite pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    CloudCompositePass(Vulkan::VulkanDevice& device,
                                       Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Scene::SceneLayout& layout);
                    ~CloudCompositePass() override;

                    CloudCompositePass(const CloudCompositePass&) = delete;
                    CloudCompositePass& operator=(const CloudCompositePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
