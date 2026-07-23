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
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/gpu_instance.hpp"
#include "scene/instance_system.hpp"
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
                                       Scene::MotionSystem& motion, Scene::InstanceSystem& instances)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes), motion_(motion), instances_(instances)
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

                // The GPU-driven twin: the same depth-only shape, built against the layout that
                // carries the instance set, and reading mesh_gpu.vert's instance record. Only
                // when the layout exists (the bindless heap is present).
                if (layout_.gpu_pipeline_layout() != VK_NULL_HANDLE)
                {
                    Resources::GraphicsPipelineDesc gpu = depth_only_pipeline_desc(
                        layout_.gpu_pipeline_layout(), shaders_.module("mesh_gpu.vert"),
                        Frame::DEPTH_FORMAT);
                    gpu.depth_compare = VK_COMPARE_OP_GREATER;
                    gpu_mesh_pipeline_ = pipelines_.create(gpu);
                }

                // The depth-only mesh-shader twin: a task + mesh pipeline with no fragment, so
                // the meshlet path culls consistently in the prepass and the opaque pass and
                // never leaves a cluster in depth that opaque did not shade. Only when the
                // meshlet layout exists (the device offers mesh shaders).
                if (layout_.meshlet_pipeline_layout() != VK_NULL_HANDLE)
                {
                    Resources::GraphicsPipelineDesc meshlet = depth_only_pipeline_desc(
                        layout_.meshlet_pipeline_layout(), VK_NULL_HANDLE, Frame::DEPTH_FORMAT);
                    meshlet.depth_compare = VK_COMPARE_OP_GREATER;
                    meshlet.task_shader = shaders_.module("meshlet.task");
                    meshlet.mesh_shader = shaders_.module("meshlet.mesh");
                    meshlet_pipeline_ = pipelines_.create_mesh(meshlet);
                }
            }

            void DepthPrepass::destroy_pipelines()
            {
                // The factory owns these pipelines and swaps in their optimized rebuilds;
                // the pass only drops its handles. clear_libraries() frees the pipelines.
                mesh_pipeline_ = Resources::PipelineHandle{};
                gpu_mesh_pipeline_ = Resources::PipelineHandle{};
                meshlet_pipeline_ = Resources::PipelineHandle{};
            }

            void DepthPrepass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void DepthPrepass::register_pass(Graph::RenderGraph& graph,
                                             const Frame::FrameContext& frame)
            {
                // The GPU-driven path is taken when the cull pass produced this frame's draw
                // commands; the classic per-instance loop is the fallback (and the path when
                // an object is selected, so the editor's outline still masks correctly).
                const bool gpu = gpu_mesh_pipeline_.get() != VK_NULL_HANDLE &&
                                 frame.targets.draw_commands.valid() &&
                                 frame.targets.compacted.valid();

                // The meshlet path draws depth the same way the opaque pass shades it, so the
                // two cull the same clusters and no cluster ends up in depth that opaque did
                // not draw. Matches the opaque pass's meshlet gate exactly.
                const bool meshlet = meshlet_pipeline_.get() != VK_NULL_HANDLE &&
                                     frame.quality.meshlets &&
                                     frame.draws.selected_id == NO_PICK;

                graph.add_pass(
                    "depth prepass",
                    [&frame, gpu](Graph::RenderPassBuilder& builder)
                    {
                        // Reverse-Z: 0 is the far plane, so that is what the clear writes.
                        // The stencil clears with it and is left alone here — the opaque
                        // pass is what marks the selection.
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Clear, 0.0f, 0);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformRead);
                        if (gpu)
                        {
                            // The cull pass wrote both; reading them here derives the
                            // compute→draw-indirect and compute→vertex barriers.
                            builder.read(frame.targets.draw_commands,
                                         Graph::BufferAccess::IndirectRead);
                            builder.read(frame.targets.compacted,
                                         Graph::BufferAccess::StorageRead);
                        }
                        // Nothing samples a depth prepass directly, so without this the
                        // graph would correctly conclude it contributes nothing and cull
                        // it; what it contributes is a depth buffer the passes after it
                        // read as an attachment, which is not a dependency the graph
                        // tracks as a read.
                        builder.set_side_effect();
                    },
                    [this, &frame, gpu, meshlet](VkCommandBuffer cmd,
                                                 const Graph::PassContext& context)
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

                        if (meshlet)
                        {
                            // Depth-only meshlet draw: the same task/mesh cull the opaque pass
                            // runs, so the two agree on which clusters land in depth. One draw
                            // per instance; the mesh shader's motion/material outputs are unused
                            // with no fragment stage, so the indices are left zero.
                            const VkPipelineLayout meshlet_layout = layout_.meshlet_pipeline_layout();
                            Scene::SceneSetWriter meshlet_writer;
                            meshlet_writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                                   context.buffer(frame.targets.uniforms),
                                                   sizeof(Scene::SceneUniforms));
                            meshlet_writer.storage(Scene::SceneLayout::MOTION_BINDING,
                                                   motion_.buffer(), motion_.buffer_range());
                            meshlet_writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                                   context.buffer(frame.targets.temporal),
                                                   sizeof(Scene::TemporalUniforms));
                            meshlet_writer.commit(cmd, meshlet_layout);
                            layout_.bind_meshlet_heap(cmd);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                              meshlet_pipeline_.get());
                            const Vulkan::MeshShaderFunctions& mesh_shader = device_.mesh_shader();

                            for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                            {
                                const MeshInstance& instance = frame.draws.instances[i];
                                const bool imported = instance.mesh != INVALID_MESH;
                                const Geometry::Mesh& mesh = imported
                                                                 ? meshes_.mesh(instance.mesh)
                                                                 : meshes_.primitive(instance.kind);
                                if (mesh.meshlet_count == 0)
                                    continue;

                                const VkDescriptorSet meshlet_set =
                                    frame.descriptors->allocate(layout_.meshlet_set_layout());
                                Resources::DescriptorWriter meshlet_set_writer;
                                meshlet_set_writer.storage_buffer(0, mesh.meshlet_descriptors,
                                                                  VK_WHOLE_SIZE);
                                meshlet_set_writer.storage_buffer(1, mesh.meshlet_vertices,
                                                                  VK_WHOLE_SIZE);
                                meshlet_set_writer.storage_buffer(2, mesh.meshlet_triangles,
                                                                  VK_WHOLE_SIZE);
                                meshlet_set_writer.storage_buffer(3, mesh.vertices, VK_WHOLE_SIZE);
                                meshlet_set_writer.update(device_.device(), meshlet_set);
                                Resources::bind_descriptor_set(
                                    cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, meshlet_layout,
                                    Scene::SceneLayout::INSTANCE_SET, meshlet_set);

                                const Mat4 model =
                                    imported ? instance.model
                                             : mul(instance.model,
                                                   Geometry::shape_scale(instance.kind,
                                                                         instance.shape_params));
                                Scene::MeshletPushConstants push{};
                                for (int m = 0; m < 16; ++m)
                                    push.model[m] = static_cast<float>(model.m[m]);
                                push.model[12] = static_cast<float>(model.m[12] - frame.eye[0]);
                                push.model[13] = static_cast<float>(model.m[13] - frame.eye[1]);
                                push.model[14] = static_cast<float>(model.m[14] - frame.eye[2]);
                                push.meshlet_count = mesh.meshlet_count;
                                vkCmdPushConstants(
                                    cmd, meshlet_layout,
                                    VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT, 0,
                                    sizeof(Scene::MeshletPushConstants), &push);
                                mesh_shader.draw_mesh_tasks(
                                    cmd, (mesh.meshlet_count + 31u) / 32u, 1, 1);
                            }
                            return;
                        }

                        if (gpu)
                        {
                            // Re-push set 0 against the GPU layout, plant its heap and instance
                            // set, then one indirect draw per bucket — the instance count each
                            // carries is the one the cull pass decided.
                            const VkPipelineLayout gpu_layout = layout_.gpu_pipeline_layout();
                            Scene::SceneSetWriter gpu_writer;
                            gpu_writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                               context.buffer(frame.targets.uniforms),
                                               sizeof(Scene::SceneUniforms));
                            gpu_writer.storage(Scene::SceneLayout::MOTION_BINDING, motion_.buffer(),
                                               motion_.buffer_range());
                            gpu_writer.uniform(Scene::SceneLayout::TEMPORAL_BINDING,
                                               context.buffer(frame.targets.temporal),
                                               sizeof(Scene::TemporalUniforms));
                            gpu_writer.commit(cmd, gpu_layout);
                            layout_.bind_gpu_heap(cmd);

                            const VkDescriptorSet instance_set =
                                frame.descriptors->allocate(layout_.instance_set_layout());
                            Resources::DescriptorWriter instance_writer;
                            instance_writer.storage_buffer(0, instances_.instance_buffer(),
                                                           instances_.instance_buffer_range());
                            instance_writer.storage_buffer(
                                1, context.buffer(frame.targets.compacted),
                                frame.gpu_instance_count * sizeof(std::uint32_t));
                            instance_writer.update(device_.device(), instance_set);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                           gpu_layout,
                                                           Scene::SceneLayout::INSTANCE_SET,
                                                           instance_set);

                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                              gpu_mesh_pipeline_.get());
                            const VkBuffer commands = context.buffer(frame.targets.draw_commands);
                            const std::vector<Scene::GpuDrawBucket>& buckets = instances_.buckets();
                            for (std::size_t b = 0; b < buckets.size(); ++b)
                            {
                                const Scene::GpuDrawBucket& bucket = buckets[b];
                                vkCmdBindVertexBuffers(cmd, 0, 1, &bucket.vertices, &zero);
                                vkCmdBindIndexBuffer(cmd, bucket.indices, 0, VK_INDEX_TYPE_UINT32);
                                Scene::GpuDrawPush push{bucket.candidate_base, 0};
                                vkCmdPushConstants(cmd, gpu_layout, VK_SHADER_STAGE_VERTEX_BIT, 0,
                                                   sizeof(Scene::GpuDrawPush), &push);
                                vkCmdDrawIndexedIndirect(
                                    cmd, commands,
                                    b * sizeof(VkDrawIndexedIndirectCommand), 1,
                                    sizeof(VkDrawIndexedIndirectCommand));
                            }
                            return;
                        }

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
