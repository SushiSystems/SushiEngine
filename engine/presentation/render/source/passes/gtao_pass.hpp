/**************************************************************************/
/* gtao_pass.hpp                                                          */
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
 * @file gtao_pass.hpp
 * @brief Ground-truth ambient occlusion: the half-res horizon compute and its resolve.
 *
 * Two graph sub-passes behind one object. A compute dispatch marches the depth prepass for
 * the horizon-based visibility and a view-space bent normal into a half-resolution target,
 * then a fullscreen resolve joint-bilateral-upsamples that to full resolution and turns the
 * bent normal into world space. The opaque shading pass samples the result to darken its
 * ambient/IBL diffuse and to occlude its indirect specular. When the tier disables AO the
 * pass still produces its target — cleared to fully-unoccluded — so the shading pass reads
 * one descriptor either way.
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
            /** @brief Builds and resolves the frame's screen-space ambient occlusion. */
            class GtaoPass : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the compute and resolve pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   The shader catalogue the modules come from.
                     * @param pipelines The factory owning the pipelines.
                     * @param layout    The shared scene layout the resolve pipeline binds.
                     */
                    GtaoPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines, Scene::SceneLayout& layout);
                    ~GtaoPass() override;

                    GtaoPass(const GtaoPass&) = delete;
                    GtaoPass& operator=(const GtaoPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief The compute push block: reconstruction constants and AO knobs. */
                    struct Push
                    {
                        float p0[4]; /**< near, tan_x, tan_y, radius. */
                        float p1[4]; /**< intensity, power, focal, spare. */
                        std::uint32_t p2[4]; /**< half_w, half_h, slices, steps. */
                        float p3[4]; /**< 1/full_w, 1/full_h, full_w, full_h. */
                    };

                    void create_pipelines();
                    void destroy_pipelines();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline compute_pipeline_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle resolve_pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
