/**************************************************************************/
/* fxaa_pass.hpp                                                          */
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
 * @file fxaa_pass.hpp
 * @brief The purely spatial anti-aliasing fallback, run after the display transform.
 *
 * Registers nothing unless the frame asked for it. When it does run it is the last
 * pass to touch the image, taking the tonemapped intermediate the display transform
 * wrote and producing the image the host samples.
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
             * @brief Filters aliased edges out of the encoded image.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class FxaaPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the filter pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    FxaaPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout);
                    ~FxaaPass() override;

                    FxaaPass(const FxaaPass&) = delete;
                    FxaaPass& operator=(const FxaaPass&) = delete;

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
