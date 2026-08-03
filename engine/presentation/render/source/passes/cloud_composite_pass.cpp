/**************************************************************************/
/* cloud_composite_pass.cpp                                               */
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

#include "passes/cloud_composite_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/atmosphere_lut_pass.hpp"
#include "passes/cloud_taa_pass.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "scene/scene_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            CloudCompositePass::CloudCompositePass(Vulkan::VulkanDevice& device,
                                                   Resources::ShaderLibrary& shaders,
                                                   Resources::GraphicsPipelineFactory& pipelines,
                                                   Scene::SceneLayout& layout, CloudTAAPass& cloud_taa,
                                                   AtmosphereLUTPass& atmosphere)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  cloud_taa_(cloud_taa), atmosphere_(atmosphere)
            {
                create_pipeline();
            }

            CloudCompositePass::~CloudCompositePass() { destroy_pipeline(); }

            void CloudCompositePass::create_pipeline()
            {
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("cloud_composite.frag"), Frame::HDR_FORMAT));
            }

            void CloudCompositePass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void CloudCompositePass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void CloudCompositePass::register_pass(Graph::RenderGraph& graph,
                                                   const Frame::FrameContext& frame)
            {
                graph.add_pass(
                    "cloud composite",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.scene,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(frame.targets.composite,
                                     Graph::TextureAccess::SampledFragment);
                        // frame.targets.cloud itself is not read here any more —
                        // CloudTAAPass already resolved it into its own pass-owned
                        // history, which this pass samples directly below — but its
                        // W3 depth sibling still is, for the aerial-perspective lookup.
                        builder.read(frame.targets.cloud_depth,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.ground_shadow_resolved,
                                     Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler =
                            frame.samplers->get(Resources::SamplerDescription{});
                        // Point, not linear: the shader reconstructs the tier-scaled
                        // cloud target itself from four explicit texel taps weighted by
                        // depth agreement (nearest-depth upsample), and needs its own and
                        // the full-resolution scene depth read back exactly, unblended.
                        Resources::SamplerDescription point{};
                        point.filter = VK_FILTER_NEAREST;
                        point.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                        const VkSampler point_sampler = frame.samplers->get(point);
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(frame.targets.composite), sampler);
                        // CloudTAAPass's own resolved history, kept in GENERAL across its
                        // compute resolve like the other pass-owned bakes this same
                        // descriptor set samples elsewhere in the frame.
                        writer.image(2, cloud_taa_.color_view(), point_sampler,
                                    VK_IMAGE_LAYOUT_GENERAL);
                        writer.image(3, context.sampled_view(frame.targets.ground_shadow_resolved),
                                    sampler);
                        writer.image(4, context.sampled_view(frame.targets.depth), point_sampler);
                        writer.image(5, context.sampled_view(frame.targets.cloud_depth),
                                    point_sampler);
                        // The Hillaire aerial-perspective froxel volume: sampled once per
                        // pixel at the cloud march's own weighted-mean depth (binding 5,
                        // reconstructed the same nearest-depth way as the colour) rather
                        // than per march sample — see AERIAL_LUT_BINDING's own doc comment.
                        writer.image(Scene::SceneLayout::AERIAL_LUT_BINDING,
                                    atmosphere_.aerial_view(), sampler, VK_IMAGE_LAYOUT_GENERAL);
                        // The froxel volume above only reaches AERIAL_MAX_DISTANCE (31 km).
                        // A cloud deck seen from the ground runs out to its own base-sphere
                        // tangent — past 100 km — so the horizon strip sits entirely beyond
                        // the volume's last slice and was being hazed as though it were 31 km
                        // away. This LUT is what lets the composite continue the view path
                        // analytically past that range; see cloud_composite.frag.
                        writer.image(Scene::SceneLayout::TRANSMITTANCE_LUT_BINDING,
                                     atmosphere_.transmittance_view(), sampler,
                                     VK_IMAGE_LAYOUT_GENERAL);
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
