/**************************************************************************/
/* particle_mesh_pass.cpp                                                 */
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

#include "passes/particle_mesh_pass.hpp"

#include <cstddef>

#include <SushiEngine/core/types.hpp>

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "graph/render_graph.hpp"
#include "passes/shadow_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/particle_system.hpp"
#include "scene/shadow_uniforms.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            ParticleMeshPass::ParticleMeshPass(Vulkan::VulkanDevice& device,
                                               Resources::ShaderLibrary& shaders,
                                               Resources::GraphicsPipelineFactory& pipelines,
                                               Scene::ParticleSystem& particles,
                                               const Geometry::MeshRegistry& meshes)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles),
                  meshes_(meshes)
            {
                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = PARTICLE_LIST_BINDING;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                bindings[1].binding = SHADOW_BLOCK_BINDING;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[2].binding = SHADOW_ATLAS_BINDING;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(particle mesh)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(particle mesh)");

                create_pipeline();
            }

            ParticleMeshPass::~ParticleMeshPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ParticleMeshPass::create_pipeline()
            {
                Resources::GraphicsPipelineDesc desc;
                desc.layout = pipeline_layout_;
                desc.vertex_shader = shaders_.module("particle_mesh.vert");
                desc.fragment_shader = shaders_.module("particle_mesh.frag");
                // The registry's own vertex layout, so a mesh particle draws the same buffers a
                // mesh instance does.
                desc.vertex_stride = sizeof(Geometry::MeshVertex);
                desc.attribute_count = 6;
                desc.attributes[0] = {
                    0, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, position))};
                desc.attributes[1] = {
                    1, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, normal))};
                desc.attributes[2] = {
                    2, VK_FORMAT_R32G32B32A32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, tangent))};
                desc.attributes[3] = {
                    3, VK_FORMAT_R32G32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv0))};
                desc.attributes[4] = {
                    4, VK_FORMAT_R32G32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv1))};
                desc.attributes[5] = {
                    5, VK_FORMAT_R8G8B8A8_UNORM,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, color))};
                desc.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
                // Debris tumbles, so a back face is as likely to be the visible one as a front
                // face; culling would blink facets in and out as a piece rotates.
                desc.cull_mode = VK_CULL_MODE_NONE;
                desc.depth_test = VK_TRUE;
                desc.depth_write = VK_TRUE;
                desc.depth_compare = VK_COMPARE_OP_GREATER_OR_EQUAL; // reverse-Z
                desc.color_count = 1;
                desc.color_formats[0] = Frame::HDR_FORMAT;
                desc.depth_format = Frame::DEPTH_FORMAT;
                pipeline_ = pipelines_.create(desc);
            }

            void ParticleMeshPass::destroy_pipeline()
            {
                pipeline_ = Resources::PipelineHandle{};
            }

            void ParticleMeshPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ParticleMeshPass::register_pass(Graph::RenderGraph& graph,
                                                 const Frame::FrameContext& frame)
            {
                if (particles_.mesh_draws().empty() || !frame.quality.gpu_particles)
                    return;

                graph.add_pass(
                    "particle mesh",
                    [&frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.hdr, Graph::AttachmentLoad::Load);
                        // Loaded and written: mesh particles are solid, so they occlude each other
                        // and everything drawn after them through the real depth test.
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Load, 0.0f, 0);
                        builder.read(frame.targets.particle_mesh, Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.particle_mesh_args,
                                     Graph::BufferAccess::IndirectRead);
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow_atlas,
                                     Graph::TextureAccess::SampledFragment);
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        Push push{};
                        const Mat4 view_projection =
                            mul(frame.camera->projection, frame.camera->view);
                        for (int i = 0; i < 16; ++i)
                            push.view_projection[i] = static_cast<float>(view_projection.m[i]);

                        const Environment& environment = *frame.environment;
                        push.sun_direction[0] = static_cast<float>(environment.sun.direction.x);
                        push.sun_direction[1] = static_cast<float>(environment.sun.direction.y);
                        push.sun_direction[2] = static_cast<float>(environment.sun.direction.z);
                        push.sun_direction[3] = static_cast<float>(frame.eye[0]);
                        push.sun_radiance[0] =
                            static_cast<float>(environment.sun.color.x) * environment.sun.intensity;
                        push.sun_radiance[1] =
                            static_cast<float>(environment.sun.color.y) * environment.sun.intensity;
                        push.sun_radiance[2] =
                            static_cast<float>(environment.sun.color.z) * environment.sun.intensity;
                        push.sun_radiance[3] = static_cast<float>(frame.eye[1]);
                        push.ambient[0] = static_cast<float>(environment.ambient.x);
                        push.ambient[1] = static_cast<float>(environment.ambient.y);
                        push.ambient[2] = static_cast<float>(environment.ambient.z);
                        push.ambient[3] = static_cast<float>(frame.eye[2]);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_buffer(PARTICLE_LIST_BINDING,
                                              context.buffer(frame.targets.particle_mesh),
                                              particles_.pool_range());
                        writer.uniform_buffer(SHADOW_BLOCK_BINDING,
                                              context.buffer(frame.targets.shadow),
                                              sizeof(Scene::ShadowUniforms));
                        writer.sampled_image(SHADOW_ATLAS_BINDING,
                                             context.sampled_view(frame.targets.shadow_atlas),
                                             ShadowPass::atlas_sampler(*frame.samplers));
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                       pipeline_layout_, 0, set);

                        const VkBuffer args = context.buffer(frame.targets.particle_mesh_args);
                        const std::uint32_t slice = particles_.mesh_slice();
                        // One draw per slice: a draw binds one mesh, which is the whole reason the
                        // list is sliced per emitter rather than shared like the sprite buckets.
                        for (const Scene::ParticleSystem::MeshDraw& draw : particles_.mesh_draws())
                        {
                            const Geometry::Mesh& mesh = meshes_.mesh(draw.mesh);
                            push.slice[0] = draw.slot * slice;
                            push.slice[1] = slice;
                            vkCmdPushConstants(cmd, pipeline_layout_,
                                               VK_SHADER_STAGE_VERTEX_BIT |
                                                   VK_SHADER_STAGE_FRAGMENT_BIT,
                                               0, sizeof(Push), &push);
                            const VkDeviceSize offset = 0;
                            vkCmdBindVertexBuffers(cmd, 0, 1, &mesh.vertices, &offset);
                            vkCmdBindIndexBuffer(cmd, mesh.indices, 0, VK_INDEX_TYPE_UINT32);
                            vkCmdDrawIndexedIndirect(
                                cmd, args,
                                static_cast<VkDeviceSize>(draw.slot) *
                                    sizeof(VkDrawIndexedIndirectCommand),
                                1, sizeof(VkDrawIndexedIndirectCommand));
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
