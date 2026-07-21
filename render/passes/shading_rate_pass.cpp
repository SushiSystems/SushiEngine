/**************************************************************************/
/* shading_rate_pass.cpp                                                  */
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

#include "passes/shading_rate_pass.hpp"

#include <algorithm>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
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
                /** @brief Threads one workgroup of the mask shader covers, per axis. */
                constexpr std::uint32_t GROUP_SIZE = 8;

                /**
                 * @brief Rounds a count up to a whole number of groups.
                 * @param value The number of items to cover.
                 * @param group The items one group covers.
                 * @return The group count, at least one.
                 */
                std::uint32_t group_count(std::uint32_t value, std::uint32_t group) noexcept
                {
                    return value == 0 ? 1u : (value + group - 1) / group;
                }
            } // namespace

            ShadingRatePass::ShadingRatePass(Vulkan::VulkanDevice& device,
                                             Resources::ShaderLibrary& shaders,
                                             Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                if (!device_.supports_shading_rate_image())
                    return;

                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                for (std::uint32_t i = 1; i < 3; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(shading rate)");

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
                              "vkCreatePipelineLayout(shading rate)");

                create_pipeline();
            }

            ShadingRatePass::~ShadingRatePass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ShadingRatePass::create_pipeline()
            {
                if (pipeline_layout_ == VK_NULL_HANDLE)
                    return;
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("shading_rate.comp"));
            }

            void ShadingRatePass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void ShadingRatePass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            bool ShadingRatePass::enabled(const Frame::FrameContext& frame) const noexcept
            {
                return pipeline_ != VK_NULL_HANDLE &&
                       frame.settings.variable_rate_shading.enabled;
            }

            std::uint32_t ShadingRatePass::texel_width() const noexcept
            {
                return device_.shading_rate_texel_width();
            }

            std::uint32_t ShadingRatePass::texel_height() const noexcept
            {
                return device_.shading_rate_texel_height();
            }

            void ShadingRatePass::register_pass(Graph::RenderGraph& graph,
                                                const Frame::FrameContext& frame)
            {
                if (!enabled(frame) || !frame.targets.shading_rate.valid())
                    return;

                // With no resolved frame behind it there is nothing to measure contrast
                // against, so the mask reads this frame's lit geometry instead and the
                // shader is told to ignore it and keep every tile at full rate.
                const Graph::TextureHandle luminance =
                    frame.history_valid ? frame.targets.history : frame.targets.hdr;

                Push push{};
                push.extents[0] = group_count(frame.width, texel_width());
                push.extents[1] = group_count(frame.height, texel_height());
                push.extents[2] = texel_width();
                push.extents[3] = texel_height();
                push.thresholds[0] = frame.settings.variable_rate_shading.luminance_threshold;
                push.thresholds[1] = frame.settings.variable_rate_shading.velocity_threshold;
                // The coarsest rate the mask may emit is the smaller of what the device
                // supports and what the quality tier permits: a higher tier caps this at a
                // finer rate (Ultra at one axis is no coarsening at all), so raising the
                // tier visibly buys shading resolution here.
                const float device_coarse = static_cast<float>(
                    std::min(device_.max_fragment_width(), device_.max_fragment_height()));
                const float tier_coarse = static_cast<float>(frame.quality.vrs_max_coarse_axis);
                push.thresholds[2] = std::min(device_coarse, tier_coarse);
                push.thresholds[3] = frame.history_valid ? 1.0f : 0.0f;

                graph.add_pass(
                    "shading rate mask",
                    [&, luminance](Graph::RenderPassBuilder& builder)
                    {
                        builder.write(frame.targets.shading_rate,
                                      Graph::TextureAccess::StorageComputeWrite);
                        builder.read(luminance, Graph::TextureAccess::SampledCompute);
                        builder.read(frame.targets.velocity,
                                     Graph::TextureAccess::SampledCompute);
                    },
                    [this, &frame, luminance, push](VkCommandBuffer cmd,
                                                    const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);

                        VkDescriptorImageInfo images[3]{};
                        images[0].imageView = context.view(frame.targets.shading_rate);
                        images[0].imageLayout = VK_IMAGE_LAYOUT_GENERAL;
                        images[1].sampler = sampler;
                        images[1].imageView = context.sampled_view(luminance);
                        images[1].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                        images[2].sampler = sampler;
                        images[2].imageView = context.sampled_view(frame.targets.velocity);
                        images[2].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;

                        VkWriteDescriptorSet writes[3]{};
                        for (std::uint32_t i = 0; i < 3; ++i)
                        {
                            writes[i].sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                            writes[i].dstSet = set;
                            writes[i].dstBinding = i;
                            writes[i].descriptorCount = 1;
                            writes[i].descriptorType =
                                i == 0 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE
                                       : VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                            writes[i].pImageInfo = &images[i];
                        }
                        vkUpdateDescriptorSets(device_.device(), 3, writes, 0, nullptr);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                pipeline_layout_, 0, 1, &set, 0, nullptr);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, group_count(push.extents[0], GROUP_SIZE),
                                      group_count(push.extents[1], GROUP_SIZE), 1);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
