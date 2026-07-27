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
 * @brief The volumetric cloud march, at the tier's dedicated cloud buffer resolution.
 *
 * Writes two MRT attachments at `QualityParams::cloud_buffer_scale` of the render
 * extent: (scattered.rgb, transmittance) and, since W3, the transmittance-weighted
 * mean march depth (`frame.targets.cloud_depth`) `CloudCompositePass` samples the
 * aerial-perspective volume at. `CloudTaaPass` resolves this target's own dedicated
 * temporal history before `CloudCompositePass` composites it over the sky; the graph
 * derives the reduced viewport from the target's own extent, so this pass declares
 * nothing about resolution beyond the size it asked for.
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
            class CloudscapeCompilePass;
            class CloudLightVolumePass;

            /**
             * @brief Marches the baked cloudscape field into the tier-scaled cloud target.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class CloudPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the cloud pipeline.
                     * @param device     The live Vulkan device.
                     * @param shaders    Library the shader modules come from.
                     * @param pipelines  Factory the pipeline is built through.
                     * @param layout     The shared scene descriptor and pipeline layout.
                     * @param cloudscape The pass that owns the near and far cloudscape windows
                     *                   and the near skip field.
                     * @param light_volume The pass that owns the baked near-window light volume.
                     * @param noise      The cloud noise volumes; only the detail volume is
                     *                   read here, for the near-camera-only 811 m erosion
                     *                   and curl warp the T3 bake cannot carry.
                     */
                    CloudPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                              Resources::GraphicsPipelineFactory& pipelines,
                              Scene::SceneLayout& layout, CloudscapeCompilePass& cloudscape,
                              CloudLightVolumePass& light_volume, Textures::CloudNoise& noise);
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
                    CloudscapeCompilePass& cloudscape_;
                    CloudLightVolumePass& light_volume_;
                    Textures::CloudNoise& noise_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
