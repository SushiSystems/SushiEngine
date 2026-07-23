/**************************************************************************/
/* taa_pass.hpp                                                           */
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
 * @file taa_pass.hpp
 * @brief The temporal resolve: this frame's samples accumulated into the history.
 *
 * Both the anti-aliasing and the upscale, because they are the same mechanism — the
 * jittered sample sequence fills in detail the single frame does not carry, whether the
 * target grid is the same size as the render grid or larger. Registers nothing when the
 * frame is not running temporal anti-aliasing, so the rest of the frame never branches
 * on it.
 */

#include "frame/upscaler.hpp"
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
             * @brief Blends the reprojected history into this frame's scene colour.
             *
             * The engine's own implementation of @c Frame::IUpscaler, and the only one that
             * is always present: it reconstructs the output grid from the jittered render
             * grid whether or not the two are the same size. A vendor backend replaces it by
             * implementing the same interface, which is why the resolve is reached through
             * @ref register_upscale rather than being hard-wired into the frame.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class TaaPass final : public IRenderPass, public Frame::IUpscaler
            {
                public:
                    /**
                     * @brief Builds the resolve pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    TaaPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                            Resources::GraphicsPipelineFactory& pipelines,
                            Scene::SceneLayout& layout);
                    ~TaaPass() override;

                    TaaPass(const TaaPass&) = delete;
                    TaaPass& operator=(const TaaPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    const char* name() const noexcept override;
                    void register_upscale(Graph::RenderGraph& graph,
                                          const Frame::FrameContext& frame,
                                          const Frame::UpscaleInputs& inputs) override;

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
