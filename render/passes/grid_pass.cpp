/**************************************************************************/
/* grid_pass.cpp                                                          */
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

#include "passes/grid_pass.hpp"

#include <cmath>

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

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                /**
                 * @brief Builds the grid's interplanetary frame for this camera.
                 *
                 * The Sun is subtracted from the eye in double before the float cast, the
                 * same discipline the scene block follows, so the plane stays put at
                 * astronomical distances. The regime weight is the camera's distance from
                 * the dominant body's centre in that body's own radii: below two it is
                 * purely the body's sea level, above eight purely the ecliptic, and the
                 * band between crossfades — expressed in radii so it reads the same on a
                 * moon as on a gas giant. With no dominant body the camera is in deep
                 * space and only the ecliptic grid is meaningful.
                 *
                 * @param environment This frame's sun, planet, and heliocentric frame.
                 * @param eye         The camera world position in the scene frame, metres.
                 * @return The filled push constant.
                 */
                GridPushConstants grid_frame(const Environment& environment, const double eye[3])
                {
                    GridPushConstants push{};

                    const double sun_x = environment.sun_center_metres.x - eye[0];
                    const double sun_y = environment.sun_center_metres.y - eye[1];
                    const double sun_z = environment.sun_center_metres.z - eye[2];
                    push.sun_position[0] = static_cast<float>(sun_x);
                    push.sun_position[1] = static_cast<float>(sun_y);
                    push.sun_position[2] = static_cast<float>(sun_z);

                    double weight = 1.0;
                    if (environment.dominant_body_id >= 0 && environment.planet_surface_visible)
                    {
                        const double dx = environment.planet_center.x - eye[0];
                        const double dy = environment.planet_center.y - eye[1];
                        const double dz = environment.planet_center.z - eye[2];
                        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
                        const double radius = environment.planet.mean_radius();
                        const double radii = radius > 0.0 ? distance / radius : 0.0;
                        const double t = std::fmin(1.0, std::fmax(0.0, (radii - 2.0) / 6.0));
                        weight = t * t * (3.0 - 2.0 * t);
                    }
                    push.sun_position[3] = static_cast<float>(weight);

                    push.ecliptic_normal[0] = static_cast<float>(environment.ecliptic_normal.x);
                    push.ecliptic_normal[1] = static_cast<float>(environment.ecliptic_normal.y);
                    push.ecliptic_normal[2] = static_cast<float>(environment.ecliptic_normal.z);
                    push.ecliptic_reference[0] =
                        static_cast<float>(environment.ecliptic_reference.x);
                    push.ecliptic_reference[1] =
                        static_cast<float>(environment.ecliptic_reference.y);
                    push.ecliptic_reference[2] =
                        static_cast<float>(environment.ecliptic_reference.z);
                    return push;
                }
            } // namespace

            GridPass::GridPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                create_pipeline();
            }

            GridPass::~GridPass() { destroy_pipeline(); }

            void GridPass::create_pipeline()
            {
                // HDR: the grid runs in the post chain before the tone map, so it composites
                // in linear HDR the same way the cloud composite does.
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("grid.frag"), Frame::HDR_FORMAT));
            }

            void GridPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void GridPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void GridPass::register_pass(Graph::RenderGraph& graph,
                                         const Frame::FrameContext& frame)
            {
                // Only when the editor asked for the grid this frame — the view allocates the
                // target and redirects post_color to it, leaving it invalid otherwise.
                if (!frame.targets.grid.valid())
                    return;

                const Graph::TextureHandle source = frame.targets.grid_source;
                const GridPushConstants push = grid_frame(*frame.environment, frame.eye);

                graph.add_pass(
                    "editor grid",
                    [&, source](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.grid,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(source, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, source, push](VkCommandBuffer cmd,
                                                 const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(source), sampler);
                        writer.image(2, context.sampled_view(frame.targets.depth), sampler);
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdPushConstants(cmd, frame.layout->pipeline_layout(),
                                           VK_SHADER_STAGE_VERTEX_BIT |
                                               VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(push), &push);
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
