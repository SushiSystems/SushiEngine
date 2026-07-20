/**************************************************************************/
/* contact_shadow_pass.hpp                                                */
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
 * @file contact_shadow_pass.hpp
 * @brief Marches the depth buffer toward the sun to recover small-scale occlusion.
 *
 * A shadow cascade texel covers tens of centimetres at best, so the moment two surfaces
 * meet the cascades have nothing to say and the contact reads as floating. This says it,
 * bounded in metres rather than pixels, and hands the shading pass a visibility mask.
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
             * @brief Writes the screen-space contact visibility mask.
             *
             * Registers nothing when the frame did not ask for contact shadows.
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class ContactShadowPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the march pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    ContactShadowPass(Vulkan::VulkanDevice& device,
                                      Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines,
                                      Scene::SceneLayout& layout);
                    ~ContactShadowPass() override;

                    ContactShadowPass(const ContactShadowPass&) = delete;
                    ContactShadowPass& operator=(const ContactShadowPass&) = delete;

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
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
