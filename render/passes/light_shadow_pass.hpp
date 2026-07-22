/**************************************************************************/
/* light_shadow_pass.hpp                                                  */
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
 * @file light_shadow_pass.hpp
 * @brief Renders spot-light shadow maps into the shared punctual atlas.
 *
 * The punctual analogue of ShadowPass: every shadow-casting spot claims a tile in one
 * depth image, and the caster being drawn changes with a viewport and a push constant,
 * never a descriptor rebind — so N spot shadows are one pass, one barrier, one profiler
 * entry. The light matrices are camera-relative, like the sun cascades, and read from
 * the LightSystem's per-frame shadow buffer.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Draws every shadow caster once per spot light that claimed a tile.
             *
             * Registers nothing when no spot claimed a tile this frame. Non-copyable: it
             * owns a Vulkan pipeline.
             */
            class LightShadowPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the punctual shadow pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader module comes from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param meshes    Registry holding the primitives and imported meshes.
                     * @param lights    The light engine's per-frame shadow data.
                     */
                    LightShadowPass(Vulkan::VulkanDevice& device,
                                    Resources::ShaderLibrary& shaders,
                                    Resources::GraphicsPipelineFactory& pipelines,
                                    Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                                    Lighting::LightSystem& lights);
                    ~LightShadowPass() override;

                    LightShadowPass(const LightShadowPass&) = delete;
                    LightShadowPass& operator=(const LightShadowPass&) = delete;

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
                    Geometry::MeshRegistry& meshes_;
                    Lighting::LightSystem& lights_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
