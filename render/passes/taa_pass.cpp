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
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void TaaPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            const char* TaaPass::name() const noexcept { return "Temporal (built-in)"; }

            void TaaPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!frame.temporal_enabled())
                    return;

                // The pass's own view of the frame, expressed in the terms every
                // reconstruction backend takes; a vendor upscaler is handed exactly this.
                Frame::UpscaleInputs inputs;
                inputs.color = frame.targets.scene_final;
                inputs.depth = frame.targets.depth;
                inputs.velocity = frame.targets.velocity;
                inputs.history = frame.history_valid ? frame.targets.history
                                                     : Graph::TextureHandle{};
                inputs.output = frame.targets.resolved;
                inputs.scene = frame.targets.uniforms;
                inputs.temporal = frame.targets.temporal;
                inputs.render_width = frame.width;
                inputs.render_height = frame.height;
                inputs.output_width = frame.output_width;
                inputs.output_height = frame.output_height;
                inputs.jitter[0] = frame.jitter[0];
                inputs.jitter[1] = frame.jitter[1];
                inputs.reset = !frame.history_valid;
                register_upscale(graph, frame, inputs);
            }

            void TaaPass::register_upscale(Graph::RenderGraph& graph,
                                           const Frame::FrameContext& frame,
                                           const Frame::UpscaleInputs& inputs)
            {
                graph.add_pass(
                    "temporal resolve",
                    [&frame, inputs](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, inputs.output,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(inputs.color, Graph::TextureAccess::SampledFragment);
                        builder.read(inputs.velocity, Graph::TextureAccess::SampledFragment);
                        builder.read(inputs.depth, Graph::TextureAccess::SampledFragment);
                        // The history is last frame's resolved image, which this frame
                        // must not also write; the two ping-pong, so declaring the read
                        // is all the graph needs to order them.
                        if (inputs.history.valid())
                            builder.read(inputs.history, Graph::TextureAccess::SampledFragment);
                        builder.read(inputs.scene, Graph::BufferAccess::UniformRead);
                        builder.read(inputs.temporal, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, inputs](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        // Clamped addressing everywhere: a reprojection that lands just
                        // outside the image must read its edge, never wrap to the far
                        // side and paint a streak across the screen.
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(inputs.scene),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(inputs.color), sampler);
                        writer.image(2,
                                     context.sampled_view(inputs.history.valid()
                                                              ? inputs.history
                                                              : inputs.color),
                                     sampler);
                        writer.image(3, context.sampled_view(inputs.velocity), sampler);
                        writer.image(4, context.sampled_view(inputs.depth), sampler);
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(inputs.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }

        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
