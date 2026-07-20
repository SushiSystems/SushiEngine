/**************************************************************************/
/* taa_pass.cpp                                                           */
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

#include "passes/taa_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            TaaPass::TaaPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                create_pipeline();
            }

            TaaPass::~TaaPass() { destroy_pipeline(); }

            void TaaPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("taa.frag"), Frame::HDR_FORMAT));
            }

            void TaaPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void TaaPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void TaaPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!frame.temporal_enabled())
                    return;

                graph.add_pass(
                    "temporal resolve",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.resolved,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(frame.targets.scene, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.velocity,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        // The history is last frame's resolved image, which this frame
                        // must not also write; the two ping-pong, so declaring the read
                        // is all the graph needs to order them.
                        if (frame.history_valid)
                            builder.read(frame.targets.history,
                                         Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        // Clamped addressing everywhere: a reprojection that lands just
                        // outside the image must read its edge, never wrap to the far
                        // side and paint a streak across the screen.
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        const VkDescriptorSet set =
                            frame.descriptors->allocate(frame.layout->set_layout());
                        Scene::SceneSetWriter writer(set);
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.scene), sampler);
                        writer.image(2,
                                     context.sampled_view(frame.history_valid
                                                              ? frame.targets.history
                                                              : frame.targets.scene),
                                     sampler);
                        writer.image(3, context.sampled_view(frame.targets.velocity), sampler);
                        writer.image(4, context.sampled_view(frame.targets.depth), sampler);
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(frame.targets.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.commit(device_.device());

                        frame.layout->bind(cmd, set);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
