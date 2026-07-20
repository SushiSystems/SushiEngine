/**************************************************************************/
/* shadow_pass.hpp                                                        */
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
 * @file shadow_pass.hpp
 * @brief Renders the sun's cascaded shadow maps into one atlas.
 *
 * All the cascades share one image, laid out as a two-by-two grid of tiles, and one
 * pass: the cascade being drawn changes with a viewport and a push constant, never with
 * a descriptor rebind or a second attachment. That is what keeps four cascades to one
 * barrier, one binding, and one entry in the profiler.
 *
 * The maps are orthographic and therefore linear in depth, so they do not follow the
 * reverse-Z the camera uses — reverse-Z buys a linear depth nothing.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>

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
            class SamplerCache;
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
             * @brief Draws every shadow caster once per cascade.
             *
             * Registers nothing when the frame fitted no cascades. Non-copyable: it owns
             * a Vulkan pipeline.
             */
            class ShadowPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the cascade pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader module comes from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param meshes    Registry holding the primitives and imported meshes.
                     */
                    ShadowPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes);
                    ~ShadowPass() override;

                    ShadowPass(const ShadowPass&) = delete;
                    ShadowPass& operator=(const ShadowPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /**
                     * @brief The sampler a shading pass reads the atlas through.
                     *
                     * A comparison sampler: the hardware compares each texel against the
                     * reference depth and returns the filtered fraction that passed, so a
                     * single fetch is already a two-by-two percentage-closer filter.
                     *
                     * @param samplers The shared sampler cache.
                     * @return The comparison sampler for the shadow atlas.
                     */
                    static VkSampler atlas_sampler(Resources::SamplerCache& samplers);

                    /**
                     * @brief The sampler the blocker search reads raw atlas depths through.
                     *
                     * Point filtered, deliberately: an interpolated depth across a shadow
                     * edge is the average of two unrelated surfaces and stands for
                     * nothing, so the search would place blockers where none exist.
                     *
                     * @param samplers The shared sampler cache.
                     * @return The plain sampler for the shadow atlas.
                     */
                    static VkSampler atlas_depth_sampler(Resources::SamplerCache& samplers);

                private:
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Geometry::MeshRegistry& meshes_;
                    VkPipeline pipeline_ = VK_NULL_HANDLE;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
