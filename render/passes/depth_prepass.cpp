/**************************************************************************/
/* depth_prepass.cpp                                                      */
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

#include "passes/depth_prepass.hpp"

#include <cstddef>

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "graph/render_graph.hpp"
#include "passes/depth_only.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "scene/motion_system.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            DepthPrepass::DepthPrepass(Vulkan::VulkanDevice& device,
                                       Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                                       Scene::MotionSystem& motion)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes), motion_(motion)
            {
                create_pipelines();
            }

            DepthPrepass::~DepthPrepass() { destroy_pipelines(); }

            void DepthPrepass::create_pipelines()
            {
                Resources::GraphicsPipelineDesc mesh =
                    depth_only_pipeline_desc(layout_.pipeline_layout(),
                                             shaders_.module("mesh.vert"), Frame::DEPTH_FORMAT);
                // Reverse-Z, matching the camera the opaque pass will test EQUAL against.
                mesh.depth_compare = VK_COMPARE_OP_GREATER;
                mesh_pipeline_ = pipelines_.create(mesh);

                Resources::GraphicsPipelineDesc line = mesh;
                line.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
                line_pipeline_ = pipelines_.create(line);
            }

            void DepthPrepass::destroy_pipelines()
            {
                // The factory owns these pipelines and swaps in their optimized rebuilds;
                // the pass only drops its handles. clear_libraries() frees the pipelines.
                mesh_pipeline_ = Resources::PipelineHandle{};
                line_pipeline_ = Resources::PipelineHandle{};
            }

            void DepthPrepass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void DepthPrepass::register_pass(Graph::RenderGraph& graph,
                                             const Frame::FrameContext& frame)
            {
                graph.add_pass(
                    "depth prepass",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        // Reverse-Z: 0 is the far plane, so that is what the clear writes.
                        // The stencil clears with it and is left alone here — the opaque
                        // pass is what marks the selection.
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Clear, 0.0f, 0);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                        // Nothing samples a depth prepass directly, so without this the
                        // graph would correctly conclude it contributes nothing and cull
                        // it; what it contributes is a depth buffer the passes after it
                        // read as an attachment, which is not a dependency the graph
                        // tracks as a read.
                        builder.set_side_effect();
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        // The shared vertex shader still reads both, and a descriptor a
                        // shader statically uses has to be valid whether or not the
                        // result reaches an attachment.
                        writer.storage(Scene::SceneLayout::MOTION_BINDING, motion_.buffer(),
                                       motion_.buffer_range());
                        writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                       context.buffer(frame.targets.temporal),
                                       sizeof(Scene::TemporalUniforms));
                        writer.commit(cmd, frame.layout->pipeline_layout());
                        frame.layout->bind_heap(cmd);

                        const VkPipelineLayout pipeline_layout = frame.layout->pipeline_layout();
                        const VkDeviceSize zero = 0;

                        const Geometry::Mesh& grid = meshes_.grid();
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, line_pipeline_.get());
                        vkCmdBindVertexBuffers(cmd, 0, 1, &grid.vertices, &zero);
                        const Scene::MeshPushConstants grid_push =
                            depth_only_push(Mat4{}, frame.eye);
                        vkCmdPushConstants(cmd, pipeline_layout, DEPTH_PUSH_STAGES, 0,
                                           sizeof(Scene::MeshPushConstants), &grid_push);
                        vkCmdDraw(cmd, grid.vertex_count, 1, 0, 0);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, mesh_pipeline_.get());
                        VkBuffer bound_vertices = VK_NULL_HANDLE;
                        for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                        {
                            const MeshInstance& instance = frame.draws.instances[i];
                            const bool imported = instance.mesh != INVALID_MESH;
                            const Geometry::Mesh& mesh = imported
                                                             ? meshes_.mesh(instance.mesh)
                                                             : meshes_.primitive(instance.kind);
                            if (mesh.index_count == 0)
                                continue;
                            if (mesh.vertices != bound_vertices)
                            {
                                vkCmdBindVertexBuffers(cmd, 0, 1, &mesh.vertices, &zero);
                                vkCmdBindIndexBuffer(cmd, mesh.indices, 0, VK_INDEX_TYPE_UINT32);
                                bound_vertices = mesh.vertices;
                            }
                            const Mat4 model =
                                imported ? instance.model
                                         : mul(instance.model,
                                               Geometry::shape_scale(instance.kind,
                                                                     instance.shape_params));
                            const Scene::MeshPushConstants push =
                                depth_only_push(model, frame.eye);
                            vkCmdPushConstants(cmd, pipeline_layout, DEPTH_PUSH_STAGES, 0,
                                               sizeof(Scene::MeshPushConstants), &push);
                            vkCmdDrawIndexed(cmd, mesh.index_count, 1, 0, 0, 0);
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
