/**************************************************************************/
/* cloud_pass.hpp                                                         */
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
 * @file cloud_pass.hpp
 * @brief The volumetric cloud march, at half resolution.
 *
 * Writes (scattered.rgb, transmittance) into a half-resolution target the tonemap
 * pass upsamples and composites over the sky. Running at a quarter of the pixels is
 * the standing performance trade for a 32–96 step march; the graph derives the
 * reduced viewport from the target's own extent, so the pass declares nothing about
 * resolution beyond the size it asked for.
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

        namespace Textures
        {
            class CloudNoise;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Marches the cloud decks into the half-resolution cloud target.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class CloudPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the cloud pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param noise     The cloud noise volumes the march samples.
                     */
                    CloudPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                              Resources::GraphicsPipelineFactory& pipelines,
                              Scene::SceneLayout& layout, Textures::CloudNoise& noise);
                    ~CloudPass() override;

                    CloudPass(const CloudPass&) = delete;
                    CloudPass& operator=(const CloudPass&) = delete;

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
                    Textures::CloudNoise& noise_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
