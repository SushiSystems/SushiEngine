/**************************************************************************/
/* tonemap_pass.cpp                                                       */
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

#include "passes/tonemap_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/post_process_uniforms.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            TonemapPass::TonemapPass(Vulkan::VulkanDevice& device,
                                     Resources::ShaderLibrary& shaders,
                                     Resources::GraphicsPipelineFactory& pipelines,
                                     Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                create_pipeline();
            }

            TonemapPass::~TonemapPass() { destroy_pipeline(); }

            void TonemapPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("tonemap.frag"), Frame::RESOLVE_FORMAT));
            }

            void TonemapPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void TonemapPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void TonemapPass::register_pass(Graph::RenderGraph& graph,
                                            const Frame::FrameContext& frame)
            {
                // The last colour the post chain produced: the depth-of-field or motion-blur
                // output when they ran, the temporal resolve's output or the composited scene
                // when they did not. The view picks it in declare_targets so this pass never
                // learns which effects were in the chain ahead of it.
                // The editor grid, when it ran, composited over the post colour and becomes
                // what the tone map encodes; otherwise the post colour directly.
                const Graph::TextureHandle source = frame.targets.grid.valid()
                                                        ? frame.targets.grid
                                                        : frame.targets.post_color;
                // The bloom pyramid when it ran, else the source as a harmless placeholder for the
                // binding the shader always declares; the shader keys the composite off the block's
                // bloom flag, so the placeholder is never actually added.
                const Graph::TextureHandle bloom =
                    frame.targets.bloom.valid() ? frame.targets.bloom : source;

                graph.add_pass(
                    "tonemap",
                    [&, source, bloom](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.display,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(source, Graph::TextureAccess::SampledFragment);
                        if (frame.targets.bloom.valid())
                            builder.read(bloom, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.post, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, source, bloom](VkCommandBuffer cmd,
                                                  const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Scene::SceneSetWriter writer;
                        writer.image(1, context.sampled_view(source), sampler);
                        writer.image(2, context.sampled_view(bloom), sampler);
                        writer.uniform(Scene::SceneLayout::POST_BINDING,
                                       context.buffer(frame.targets.post),
                                       sizeof(Scene::PostProcessUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
