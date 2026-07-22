/**************************************************************************/
/* sky_pass.cpp                                                           */
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

#include "passes/sky_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"
#include "passes/shadow_pass.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "textures/cloud_noise.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            SkyPass::SkyPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout, Textures::CloudNoise& noise)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  noise_(noise)
            {
                create_pipeline();
            }

            SkyPass::~SkyPass() { destroy_pipeline(); }

            void SkyPass::create_pipeline()
            {
                Resources::GraphicsPipelineDesc desc = fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("sky.frag"), Frame::HDR_FORMAT);
                // Second MRT slot: the analytic ground's raw, unresolved direct-sun term,
                // held back so the ground-shadow resolve pass can blur its noisy PCF alpha
                // before cloud_composite_pass folds it into the scene.
                desc.color_formats[1] = Frame::HDR_FORMAT;
                desc.color_count = 2;
                // Whether a rate image is actually bound is decided per frame, but the
                // pipeline has to be created knowing one may be, so on a device that
                // supports it the sky pipeline always opts in.
                desc.shading_rate_attachment =
                    device_.supports_shading_rate_image() ? VK_TRUE : VK_FALSE;
                pipeline_ = pipelines_.create(desc);
            }

            void SkyPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void SkyPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void SkyPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                graph.add_pass(
                    "sky",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.composite,
                                                 Graph::AttachmentLoad::Discard);
                        builder.color_attachment(1, frame.targets.ground_shadow,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(frame.targets.hdr, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow_atlas,
                                     Graph::TextureAccess::SampledFragment);
                        // The atmosphere and star march is the frame's heaviest fill, so
                        // it is the pass worth shading below one sample per pixel where
                        // the mask says the difference cannot be seen.
                        builder.shading_rate_attachment(frame.targets.shading_rate,
                                                        device_.shading_rate_texel_width(),
                                                        device_.shading_rate_texel_height());
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.depth), sampler);
                        writer.image(2, context.sampled_view(frame.targets.hdr), sampler);
                        writer.image(3, noise_.shape(), noise_.sampler());
                        writer.image(4, noise_.detail(), noise_.sampler());
                        writer.image(5, noise_.weather(), noise_.sampler());
                        writer.image(6, noise_.cirrus(), noise_.sampler());
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(frame.targets.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.uniform(Scene::SceneLayout::SHADOW_BINDING,
                                       context.buffer(frame.targets.shadow),
                                       sizeof(Scene::ShadowUniforms));
                        writer.image(Scene::SceneLayout::SHADOW_ATLAS_BINDING,
                                     context.sampled_view(frame.targets.shadow_atlas),
                                     ShadowPass::atlas_sampler(*frame.samplers));
                        writer.image(Scene::SceneLayout::SHADOW_DEPTH_BINDING,
                                     context.sampled_view(frame.targets.shadow_atlas),
                                     ShadowPass::atlas_depth_sampler(*frame.samplers));
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
