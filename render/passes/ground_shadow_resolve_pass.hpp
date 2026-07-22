/**************************************************************************/
/* ground_shadow_resolve_pass.hpp                                         */
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
 * @file ground_shadow_resolve_pass.hpp
 * @brief Spatially blurs the analytic ground's raw sun-visibility term.
 *
 * The ground (sky.frag's planet ray march) has no motion vectors and no rasterised
 * depth, so unlike mesh shadows it gets no TAA history to hide its PCF noise in, and
 * unlike GTAO it has no real depth to weight a bilateral upsample against. This is
 * the spatial-only stand-in: a wide plain blur over the raw visibility so the raw
 * 12-tap speckle reads as a soft penumbra instead of visible dithering.
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
             * @brief Blurs `frame.targets.ground_shadow` into `ground_shadow_resolved`.
             *
             * Runs unconditionally after the sky pass; cloud_composite_pass reads the
             * resolved result. Non-copyable: it owns a Vulkan pipeline.
             */
            class GroundShadowResolvePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the blur pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    GroundShadowResolvePass(Vulkan::VulkanDevice& device,
                                            Resources::ShaderLibrary& shaders,
                                            Resources::GraphicsPipelineFactory& pipelines,
                                            Scene::SceneLayout& layout);
                    ~GroundShadowResolvePass() override;

                    GroundShadowResolvePass(const GroundShadowResolvePass&) = delete;
                    GroundShadowResolvePass& operator=(const GroundShadowResolvePass&) = delete;

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
