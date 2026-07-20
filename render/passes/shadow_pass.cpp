/**************************************************************************/
/* shadow_pass.cpp                                                        */
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

#include "passes/shadow_pass.hpp"

#include <cstddef>

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "graph/render_graph.hpp"
#include "passes/depth_only.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "scene/shadow_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            ShadowPass::ShadowPass(Vulkan::VulkanDevice& device,
                                   Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  meshes_(meshes)
            {
                create_pipeline();
            }

            ShadowPass::~ShadowPass() { destroy_pipeline(); }

            VkSampler ShadowPass::atlas_sampler(Resources::SamplerCache& samplers)
            {
                Resources::SamplerDesc desc;
                desc.compare_enable = VK_TRUE;
                // The maps store distance from the light, so a texel closer than the
                // reference is what occludes it.
                desc.compare_op = VK_COMPARE_OP_LESS_OR_EQUAL;
                // A tap that lands outside the map must read "lit": the shader already
                // rejects out-of-range lookups, and a white border makes the filter's
                // edge taps agree with it instead of darkening the boundary.
                desc.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
                desc.border_color = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
                return samplers.get(desc);
            }

            VkSampler ShadowPass::atlas_depth_sampler(Resources::SamplerCache& samplers)
            {
                Resources::SamplerDesc desc;
                desc.filter = VK_FILTER_NEAREST;
                desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                // A tap off the edge of the atlas must read the far plane, which is the
                // same as saying nothing blocks there.
                desc.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;
                desc.border_color = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
                return samplers.get(desc);
            }

            void ShadowPass::create_pipeline()
            {
                Resources::GraphicsPipelineDesc desc = depth_only_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("shadow.vert"),
                    Frame::SHADOW_FORMAT);
                // Conventional depth, not reverse-Z: an orthographic projection is linear
                // in depth, so there is no precision to redistribute.
                desc.depth_compare = VK_COMPARE_OP_LESS;
                // Double-sided, matching every other pass in this renderer. Culling front
                // faces to record the back of each object is the older way to avoid
                // self-shadowing, and it pays for it by pulling a large caster's shadow
                // in toward the light by the object's own thickness. The normal offset in
                // shadow_common.glsl solves the same problem without that cost, and it is
                // the only one of the two that also works on an open surface.
                desc.cull_mode = VK_CULL_MODE_NONE;
                pipeline_ = pipelines_.create(desc);
            }

            void ShadowPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void ShadowPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ShadowPass::register_pass(Graph::RenderGraph& graph,
                                           const Frame::FrameContext& frame)
            {
                if (frame.cascade_count == 0 || !frame.targets.shadow_atlas.valid())
                    return;

                const std::uint32_t cascades = frame.cascade_count;
                const std::uint32_t tile = frame.shadow_resolution;

                graph.add_pass(
                    "shadow cascades",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        // One clear over the whole atlas, then each cascade draws into its
                        // own quadrant; a tile no cascade fills stays at the far plane and
                        // reads as lit.
                        builder.depth_stencil_attachment(frame.targets.shadow_atlas,
                                                         Graph::AttachmentLoad::Clear, 1.0f, 0);
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, cascades, tile](VkCommandBuffer cmd,
                                                   const Graph::PassContext& context)
                    {
                        const VkDescriptorSet set =
                            frame.descriptors->allocate(frame.layout->set_layout());
                        Scene::SceneSetWriter writer(set);
                        writer.uniform(Scene::SceneLayout::SHADOW_BINDING,
                                       context.buffer(frame.targets.shadow),
                                       sizeof(Scene::ShadowUniforms));
                        writer.commit(device_.device());
                        frame.layout->bind(cmd, set);

                        const VkPipelineLayout pipeline_layout = frame.layout->pipeline_layout();
                        const VkDeviceSize zero = 0;
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_);

                        for (std::uint32_t cascade = 0; cascade < cascades; ++cascade)
                        {
                            VkViewport viewport{};
                            viewport.x = static_cast<float>((cascade & 1) * tile);
                            viewport.y = static_cast<float>((cascade >> 1) * tile);
                            viewport.width = static_cast<float>(tile);
                            viewport.height = static_cast<float>(tile);
                            viewport.maxDepth = 1.0f;
                            VkRect2D scissor{};
                            scissor.offset = {static_cast<std::int32_t>(viewport.x),
                                              static_cast<std::int32_t>(viewport.y)};
                            scissor.extent = {tile, tile};
                            vkCmdSetViewport(cmd, 0, 1, &viewport);
                            vkCmdSetScissor(cmd, 0, 1, &scissor);

                            VkBuffer bound_vertices = VK_NULL_HANDLE;
                            for (std::size_t i = 0; i < frame.draws.instance_count; ++i)
                            {
                                const MeshInstance& instance = frame.draws.instances[i];
                                if (!instance.material.cast_shadows)
                                    continue;
                                const bool imported = instance.mesh != INVALID_MESH;
                                const Geometry::Mesh& mesh =
                                    imported ? meshes_.mesh(instance.mesh)
                                             : meshes_.primitive(instance.kind);
                                if (mesh.index_count == 0)
                                    continue;
                                if (mesh.vertices != bound_vertices)
                                {
                                    vkCmdBindVertexBuffers(cmd, 0, 1, &mesh.vertices, &zero);
                                    vkCmdBindIndexBuffer(cmd, mesh.indices, 0,
                                                         VK_INDEX_TYPE_UINT32);
                                    bound_vertices = mesh.vertices;
                                }
                                const Mat4 model =
                                    imported ? instance.model
                                             : mul(instance.model,
                                                   Geometry::shape_scale(instance.kind,
                                                                         instance.shape_params));
                                const Scene::MeshPushConstants push = depth_only_push(
                                    model, frame.eye, static_cast<float>(cascade));
                                vkCmdPushConstants(cmd, pipeline_layout, DEPTH_PUSH_STAGES, 0,
                                                   sizeof(Scene::MeshPushConstants), &push);
                                vkCmdDrawIndexed(cmd, mesh.index_count, 1, 0, 0, 0);
                            }
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
