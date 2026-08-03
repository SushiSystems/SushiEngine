/**************************************************************************/
/* ssr_pass.hpp                                                           */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file ssr_pass.hpp
 * @brief Screen-space reflections: the mirror trace over the hi-Z pyramid.
 *
 * A fullscreen pass on the shared scene layout. It reads the lit scene, the depth, the thin
 * roughness/reflectance G-buffer, and the hi-Z pyramid the HizPass owns, traces each smooth
 * surface's mirror ray, and writes the scene with its screen reflections folded in. Rough
 * surfaces and rays that leave the screen keep the IBL term untouched. Runs after the scene
 * is fully composited (sky + clouds) and before the temporal resolve reads it.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

#include "passes/render_pass.hpp"
#include "resources/pipeline_handle.hpp"

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

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Passes
        {
            class HizPass;

            /** @brief Traces and composites screen-space reflections into the scene. */
            class SsrPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the trace pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The shader catalogue the trace module comes from.
                     * @param pipelines The factory owning the pipeline.
                     * @param layout    The shared scene layout the pass binds.
                     * @param hiz       The pyramid pass whose image the trace samples.
                     */
                    SsrPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                            Resources::GraphicsPipelineFactory& pipelines, Scene::SceneLayout& layout,
                            HizPass& hiz);
                    ~SsrPass() override;

                    SsrPass(const SsrPass&) = delete;
                    SsrPass& operator=(const SsrPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    struct Push
                    {
                        float p0[4]; /**< max steps, thickness, roughness cutoff, intensity. */
                        float p1[4]; /**< trace enabled. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    HizPass& hiz_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
