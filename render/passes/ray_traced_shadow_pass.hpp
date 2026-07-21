/**************************************************************************/
/* ray_traced_shadow_pass.hpp                                             */
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
 * @file ray_traced_shadow_pass.hpp
 * @brief Traces the sun ray per pixel instead of sampling a cascade.
 *
 * Two graph nodes, because they are two things the frame must do in order and nothing
 * else may come between them: the acceleration structure is built, then it is traced.
 * They live in one class because they are one feature — when a later phase wants the
 * same structure for reflections, the build is what moves out.
 *
 * The pass owns its own descriptor and pipeline layouts rather than the shared scene
 * ones. An acceleration structure is a descriptor type nothing else in the renderer
 * binds, and putting it in the layout every pipeline is built against would make a
 * device without ray tracing unable to create any of them.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace RayTracing
        {
            class SceneAccelerator;
        }

        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Writes the traced sun visibility mask, and builds what it traces.
             *
             * Registers nothing when the device cannot trace or the frame did not ask for
             * it. Non-copyable: it owns Vulkan layout and pipeline objects.
             */
            class RayTracedShadowPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the trace pipeline and its layouts.
                     * @param device      The live Vulkan device.
                     * @param shaders     Library the shader modules come from.
                     * @param pipelines   Factory the pipeline is built through.
                     * @param accelerator The structure store this pass builds and traces.
                     */
                    RayTracedShadowPass(Vulkan::VulkanDevice& device,
                                        Resources::ShaderLibrary& shaders,
                                        Resources::GraphicsPipelineFactory& pipelines,
                                        RayTracing::SceneAccelerator& accelerator);
                    ~RayTracedShadowPass() override;

                    RayTracedShadowPass(const RayTracedShadowPass&) = delete;
                    RayTracedShadowPass& operator=(const RayTracedShadowPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief Whether this frame will trace rather than sample cascades.
                     *
                     * The scene view asks before it fills the shadow block, so the flag the
                     * material shader reads agrees with what actually ran.
                     *
                     * @param frame The frame being built.
                     * @return true when the device supports it and the frame asked for it.
                     */
                    bool enabled(const Frame::FrameContext& frame) const noexcept;

                private:
                    /** @brief Mirrors the trace shader's push constant block. */
                    struct Push
                    {
                        float forward[4];
                        float right[4];
                        float up[4];
                        float sun[4];
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    RayTracing::SceneAccelerator& accelerator_;
                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
