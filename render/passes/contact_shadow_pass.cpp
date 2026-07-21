/**************************************************************************/
/* contact_shadow_pass.cpp                                                */
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

#include "passes/contact_shadow_pass.hpp"

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
#include "scene/shadow_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            ContactShadowPass::ContactShadowPass(Vulkan::VulkanDevice& device,
                                                 Resources::ShaderLibrary& shaders,
                                                 Resources::GraphicsPipelineFactory& pipelines,
                                                 Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                create_pipeline();
            }

            ContactShadowPass::~ContactShadowPass() { destroy_pipeline(); }

            void ContactShadowPass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("contact_shadow.frag"), Frame::CONTACT_SHADOW_FORMAT));
            }

            void ContactShadowPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void ContactShadowPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ContactShadowPass::register_pass(Graph::RenderGraph& graph,
                                                  const Frame::FrameContext& frame)
            {
                if (!frame.settings.shadows.enabled ||
                    !frame.settings.shadows.contact_shadows ||
                    !frame.targets.contact_shadow.valid())
                    return;

                graph.add_pass(
                    "contact shadows",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.contact_shadow,
                                                 Graph::AttachmentLoad::Discard);
                        // The depth prepass filled this; reading it here is the whole
                        // reason that pass exists.
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.depth), sampler);
                        writer.uniform(Scene::SceneLayout::SHADOW_BINDING,
                                       context.buffer(frame.targets.shadow),
                                       sizeof(Scene::ShadowUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
