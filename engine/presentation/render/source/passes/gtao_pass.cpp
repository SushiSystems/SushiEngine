/**************************************************************************/
/* gtao_pass.cpp                                                          */
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

#include "passes/gtao_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
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
                constexpr std::uint32_t GROUP_SIZE = 8;

                std::uint32_t group_count(std::uint32_t value) noexcept
                {
                    return value == 0 ? 1u : (value + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                /** @brief A nearest-sampled clamp sampler: AO reads must not blur depth edges. */
                Resources::SamplerDesc point_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_NEAREST;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                    return desc;
                }
            } // namespace

            GtaoPass::GtaoPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               Scene::SceneLayout& layout)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout)
            {
                // The compute set: the full-res depth to march (a sampled image) and the
                // half-res AO target it writes (a storage image).
                VkDescriptorSetLayoutBinding bindings[2]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 2;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(gtao)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(gtao)");

                create_pipelines();
            }

            GtaoPass::~GtaoPass()
            {
                destroy_pipelines();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void GtaoPass::create_pipelines()
            {
                compute_pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                              shaders_.module("gtao.comp"));
                // The resolve is an ordinary fullscreen pass on the shared scene layout, so
                // its bent-normal write and depth reads ride the same push-descriptor set 0.
                resolve_pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("gtao_resolve.frag"), Frame::HDR_FORMAT));
            }

            void GtaoPass::destroy_pipelines()
            {
                if (compute_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), compute_pipeline_, nullptr);
                compute_pipeline_ = VK_NULL_HANDLE;
                // The factory owns the resolve pipeline and swaps its optimized rebuild in;
                // dropping the handle is all this pass does.
                resolve_pipeline_ = Resources::PipelineHandle{};
            }

            void GtaoPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void GtaoPass::register_pass(Graph::RenderGraph& graph,
                                         const Frame::FrameContext& frame)
            {
                if (!frame.targets.ao.valid())
                    return;

                // Disabled by tier or authoring: still produce the target the shading pass
                // reads, cleared to fully unoccluded (visibility 1, zero bent normal), so the
                // opaque pass never samples an unwritten image and needs no shader permutation.
                if (!frame.settings.gtao.enabled || !frame.targets.gtao.valid() ||
                    frame.camera == nullptr)
                {
                    Graph::ClearColor open;
                    open.float32[0] = 0.0f;
                    open.float32[1] = 0.0f;
                    open.float32[2] = 0.0f;
                    open.float32[3] = 1.0f;
                    graph.add_pass(
                        "gtao (off)",
                        [&](Graph::RenderPassBuilder& builder)
                        {
                            builder.color_attachment(0, frame.targets.ao,
                                                     Graph::AttachmentLoad::Clear, open);
                        },
                        [](VkCommandBuffer, const Graph::PassContext&) {});
                    return;
                }

                const std::uint32_t half_w = (frame.width + 1) / 2;
                const std::uint32_t half_h = (frame.height + 1) / 2;

                const Mat4& proj = frame.camera->projection;
                Push push{};
                push.p0[0] = frame.camera->near_plane;
                push.p0[1] = proj.m[0] != 0.0 ? static_cast<float>(1.0 / proj.m[0]) : 1.0f;
                push.p0[2] = proj.m[5] != 0.0 ? static_cast<float>(1.0 / proj.m[5]) : 1.0f;
                push.p0[3] = frame.settings.gtao.radius;
                push.p1[0] = frame.settings.gtao.intensity;
                push.p1[1] = frame.settings.gtao.power;
                // Horizontal focal length in pixels: NDC-per-view-slope times half the width.
                push.p1[2] = static_cast<float>(proj.m[0]) * 0.5f * static_cast<float>(frame.width);
                push.p1[3] = 0.0f;
                push.p2[0] = half_w;
                push.p2[1] = half_h;
                push.p2[2] = frame.settings.gtao.slices;
                push.p2[3] = frame.settings.gtao.steps;
                push.p3[0] = 1.0f / static_cast<float>(frame.width);
                push.p3[1] = 1.0f / static_cast<float>(frame.height);
                push.p3[2] = static_cast<float>(frame.width);
                push.p3[3] = static_cast<float>(frame.height);

                const Graph::TextureHandle depth = frame.targets.depth;
                const Graph::TextureHandle gtao = frame.targets.gtao;
                const Graph::TextureHandle ao = frame.targets.ao;

                // Half-res horizon march.
                graph.add_pass(
                    "gtao",
                    [depth, gtao](Graph::RenderPassBuilder& builder)
                    {
                        builder.set_queue(Graph::PassQueue::AsyncCompute);
                        builder.read(depth, Graph::TextureAccess::SampledCompute);
                        builder.write(gtao, Graph::TextureAccess::StorageComputeWrite);
                    },
                    [this, &frame, depth, gtao, push, half_w, half_h](
                        VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.sampled_image(0, context.sampled_view(depth),
                                             frame.samplers->get(point_sampler()));
                        writer.storage_image(1, context.view(gtao));
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, compute_pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, group_count(half_w), group_count(half_h), 1);
                    });

                // Full-res joint-bilateral upsample + bent-normal to world.
                graph.add_pass(
                    "gtao resolve",
                    [depth, gtao, ao, &frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, ao, Graph::AttachmentLoad::Discard);
                        builder.read(depth, Graph::TextureAccess::SampledFragment);
                        builder.read(gtao, Graph::TextureAccess::SampledFragment);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                    },
                    [this, &frame, depth, gtao](VkCommandBuffer cmd,
                                                const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(point_sampler());
                        Scene::SceneSetWriter writer;
                        writer.uniform(Scene::SceneLayout::SCENE_BINDING,
                                       context.buffer(frame.targets.uniforms),
                                       sizeof(Scene::SceneUniforms));
                        writer.image(1, context.sampled_view(depth), sampler);
                        writer.image(2, context.sampled_view(gtao), sampler);
                        writer.commit(cmd, frame.layout->pipeline_layout());

                        frame.layout->bind_heap(cmd);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          resolve_pipeline_.get());
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
