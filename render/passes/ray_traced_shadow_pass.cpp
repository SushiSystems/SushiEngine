/**************************************************************************/
/* ray_traced_shadow_pass.cpp                                             */
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

#include "passes/ray_traced_shadow_pass.hpp"

#include <cmath>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/fullscreen.hpp"
#include "raytracing/scene_accelerator.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                /** @brief Normalises a vector, leaving a degenerate one pointing up. */
                Vector3 unit(const Vector3& v) noexcept
                {
                    const double length = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
                    if (length < 1e-12)
                        return Vector3{0.0, 1.0, 0.0};
                    return Vector3{v.x / length, v.y / length, v.z / length};
                }
            } // namespace

            RayTracedShadowPass::RayTracedShadowPass(
                Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                Resources::GraphicsPipelineFactory& pipelines,
                RayTracing::SceneAccelerator& accelerator)
                : device_(device), shaders_(shaders), pipelines_(pipelines),
                  accelerator_(accelerator)
            {
                if (!accelerator_.available())
                    return;

                VkDescriptorSetLayoutBinding bindings[2]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 2;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(ray shadow)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(ray shadow)");

                create_pipeline();
            }

            RayTracedShadowPass::~RayTracedShadowPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void RayTracedShadowPass::create_pipeline()
            {
                if (pipeline_layout_ == VK_NULL_HANDLE)
                    return;
                pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    pipeline_layout_, shaders_.module("fullscreen.vert"),
                    shaders_.module("ray_shadow.frag"), Frame::CONTACT_SHADOW_FORMAT));
            }

            void RayTracedShadowPass::destroy_pipeline()
            {
                // The factory owns the pipeline and swaps in its optimized rebuild, so
                // the pass drops only its handle; clear_libraries() frees the pipeline.
                pipeline_ = Resources::PipelineHandle{};
            }

            void RayTracedShadowPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            bool RayTracedShadowPass::enabled(const Frame::FrameContext& frame) const noexcept
            {
                return pipeline_.valid() && frame.settings.shadows.enabled &&
                       frame.settings.shadows.ray_traced &&
                       frame.settings.quality == RenderQuality::Ultra;
            }

            void RayTracedShadowPass::register_pass(Graph::RenderGraph& graph,
                                                    const Frame::FrameContext& frame)
            {
                if (!enabled(frame) || !frame.targets.ray_shadow.valid())
                    return;

                // The structure has to exist before anything traces it, and it is neither
                // an image nor a buffer, so the graph cannot order it by declaration. The
                // build is its own node with a side effect, which is what keeps it in the
                // schedule and ahead of the trace that follows it.
                graph.add_pass(
                    "acceleration build",
                    [](Graph::RenderPassBuilder& builder) { builder.set_side_effect(); },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext&)
                    {
                        accelerator_.build(cmd, frame.slot, frame.draws.instances,
                                           frame.draws.instance_count, frame.eye);
                    });

                Push push{};
                const Mat4& view = frame.camera->view;
                const Mat4& projection = frame.camera->projection;
                // The camera basis straight out of the view matrix's rows, with the field
                // of view folded into the two lateral axes — exactly the parameterisation
                // the sky pass builds its rays from, so the two agree on where a pixel is.
                const double tan_half_x = projection.m[0] != 0.0 ? 1.0 / projection.m[0] : 1.0;
                const double tan_half_y =
                    projection.m[5] != 0.0 ? 1.0 / (-projection.m[5]) : 1.0;
                push.forward[0] = static_cast<float>(-view.m[2]);
                push.forward[1] = static_cast<float>(-view.m[6]);
                push.forward[2] = static_cast<float>(-view.m[10]);
                push.forward[3] = frame.camera->near_plane;
                push.right[0] = static_cast<float>(view.m[0] * tan_half_x);
                push.right[1] = static_cast<float>(view.m[4] * tan_half_x);
                push.right[2] = static_cast<float>(view.m[8] * tan_half_x);
                push.right[3] = frame.settings.shadows.distance;
                push.up[0] = static_cast<float>(view.m[1] * tan_half_y);
                push.up[1] = static_cast<float>(view.m[5] * tan_half_y);
                push.up[2] = static_cast<float>(view.m[9] * tan_half_y);
                // The ray's start offset, in metres. Reusing the contact-shadow reach
                // keeps one number describing "how close is too close to trust the
                // reconstructed depth" rather than two that can disagree.
                push.up[3] = frame.settings.shadows.contact_distance * 0.1f;
                const Vector3 sun = unit(frame.environment->sun.direction);
                push.sun[0] = static_cast<float>(sun.x);
                push.sun[1] = static_cast<float>(sun.y);
                push.sun[2] = static_cast<float>(sun.z);

                graph.add_pass(
                    "ray traced shadows",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.ray_shadow,
                                                 Graph::AttachmentLoad::Discard);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                    },
                    [this, &frame, push](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkAccelerationStructureKHR structure =
                            accelerator_.top_level(frame.slot);
                        if (structure == VK_NULL_HANDLE)
                            return; // nothing was built; the target keeps its clear

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);

                        Resources::DescriptorWriter writer;
                        writer.acceleration_structure(0, structure);
                        writer.sampled_image(1, context.sampled_view(frame.targets.depth),
                                             frame.samplers->get(Resources::SamplerDesc{}));
                        writer.update(device_.device(), set);

                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                       pipeline_layout_, 0, set);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_.get());
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_FRAGMENT_BIT,
                                           0, sizeof(Push), &push);
                        vkCmdDraw(cmd, 3, 1, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
