/**************************************************************************/
/* depth_prepass.hpp                                                      */
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
 * @file depth_prepass.hpp
 * @brief Fills the depth buffer before anything is shaded.
 *
 * Two things need it. The screen-space contact shadows cannot run at all without a
 * complete depth buffer *before* shading, and with depth already filled the opaque pass
 * rejects an occluded fragment before running the material shader on it — which at this
 * shader's cost is most of what the extra geometry pass buys back.
 *
 * It runs the same `mesh.vert` the opaque pass does, with no fragment stage. That is a
 * correctness requirement rather than a convenience: the opaque pass then tests against
 * depths it will recompute itself, and only the same shader guarantees the two agree bit
 * for bit. A different depth shader could disagree in the last bit and punch holes.
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
            class MotionSystem;
            class SceneLayout;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Draws every opaque instance as depth only.
             *
             * Non-copyable: it owns Vulkan pipelines.
             */
            class DepthPrepass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the depth-only pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipelines are built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param meshes    Registry holding the primitives and imported meshes.
                     * @param motion    System packing this frame's previous transforms; the
                     *                  shared vertex shader reads it even here, where its
                     *                  result is discarded.
                     */
                    DepthPrepass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines,
                                 Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                                 Scene::MotionSystem& motion);
                    ~DepthPrepass() override;

                    DepthPrepass(const DepthPrepass&) = delete;
                    DepthPrepass& operator=(const DepthPrepass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    void create_pipelines();
                    void destroy_pipelines();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Geometry::MeshRegistry& meshes_;
                    Scene::MotionSystem& motion_;
                    Resources::PipelineHandle mesh_pipeline_;
                    Resources::PipelineHandle line_pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
