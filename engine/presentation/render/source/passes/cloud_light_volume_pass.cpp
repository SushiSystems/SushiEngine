/**************************************************************************/
/* cloud_light_volume_pass.cpp                                           */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/cloud_light_volume_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/cloudscape_compile_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_uniforms.hpp"
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
                constexpr VkFormat VOLUME_FORMAT = VK_FORMAT_R32_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 4;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                void transition(VkCommandBuffer cmd, VkImage image, VkImageLayout from,
                                VkImageLayout to, VkPipelineStageFlags2 source,
                                VkPipelineStageFlags2 destination, VkAccessFlags2 source_access,
                                VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination;
                    barrier.dstAccessMask = destination_access;
                    barrier.oldLayout = from;
                    barrier.newLayout = to;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.image = image;
                    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    barrier.subresourceRange.levelCount = 1;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            CloudLightVolumePass::CloudLightVolumePass(Vulkan::VulkanDevice& device,
                                                       Resources::ShaderLibrary& shaders,
                                                       Resources::GraphicsPipelineFactory& pipelines,
                                                       Resources::SamplerCache& samplers,
                                                       CloudscapeCompilePass& cloudscape)
                : device_(device), shaders_(shaders), pipelines_(pipelines), cloudscape_(cloudscape)
            {
                static_assert(RESOLUTION_Y % AMORTIZE_FRAMES == 0,
                             "the Y axis must divide evenly across the amortization window");

                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(cloud light volume)");

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
                              "vkCreatePipelineLayout(cloud light volume)");

                create_volume();

                // Tiles under the same convention as the T3 field's own sampler: the
                // volume wraps in every axis so a march sample straying past the union
                // band's Y edge reads its own boundary back.
                Resources::SamplerDescription sampler_desc{};
                sampler_desc.filter = VK_FILTER_LINEAR;
                sampler_desc.address_mode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler_ = samplers.get(sampler_desc);

                create_pipeline();
            }

            CloudLightVolumePass::~CloudLightVolumePass()
            {
                destroy_pipeline();
                destroy_volume();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CloudLightVolumePass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("cloud_light_volume.comp"));
            }

            void CloudLightVolumePass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void CloudLightVolumePass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
                // A shader edit can change the bake's contents; the amortized refresh
                // will catch up within one cycle regardless, so no forced full rebake.
            }

            void CloudLightVolumePass::create_volume()
            {
                volume_.width = RESOLUTION_XZ;
                volume_.height = RESOLUTION_Y;
                volume_.depth = RESOLUTION_XZ;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = VOLUME_FORMAT;
                image_info.extent = {volume_.width, volume_.height, volume_.depth};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &volume_.image,
                                             &volume_.allocation, nullptr),
                              "vmaCreateImage(cloud light volume)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = volume_.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = VOLUME_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &volume_.view),
                              "vkCreateImageView(cloud light volume)");
            }

            void CloudLightVolumePass::destroy_volume()
            {
                if (volume_.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), volume_.view, nullptr);
                if (volume_.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), volume_.image, volume_.allocation);
                volume_.view = VK_NULL_HANDLE;
                volume_.image = VK_NULL_HANDLE;
                volume_.allocation = VK_NULL_HANDLE;
            }

            void CloudLightVolumePass::register_pass(Graph::RenderGraph& graph,
                                                      const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr || !frame.environment->clouds.enabled)
                    return;

                // Not change-gated like the field: the sun moves every frame the clock
                // advances, so there is no "settled" state — instead a fixed slice group
                // refreshes every frame, cycling the whole volume across
                // AMORTIZE_FRAMES frames regardless of what else changed.
                const std::uint32_t slice_group = frame_counter_ % AMORTIZE_FRAMES;
                ++frame_counter_;
                // A moved near window (or the first bake) invalidates every slice at once, not
                // one group at a time — see CloudscapeCompilePass::near_window_moved.
                const bool first_bake = !built_;
                const bool full_refresh = first_bake || cloudscape_.near_window_moved();
                built_ = true;
                const std::uint32_t slice_start = full_refresh ? 0u : slice_group * Y_SLICE_COUNT;
                const std::uint32_t slice_count = full_refresh ? RESOLUTION_Y : Y_SLICE_COUNT;

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "cloud-light-volume",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The volume is pass-owned and barriered by hand below, exactly
                        // like the T3 field; the one graph resource is the scene uniform
                        // block the bake reads the sun direction and cloud band from.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, uniforms, slice_start, slice_count,
                     first_bake](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const Push push{CloudscapeCompilePass::near_span_meters(), slice_start,
                                        slice_count};

                        transition(cmd, volume_.image,
                                  first_bake ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL,
                                  VK_IMAGE_LAYOUT_GENERAL,
                                  first_bake ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                                             : VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  first_bake ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_image(0, volume_.view);
                        writer.sampled_image(1, cloudscape_.field_view(), cloudscape_.sampler(),
                                             VK_IMAGE_LAYOUT_GENERAL);
                        writer.uniform_buffer(2, context.buffer(uniforms), sizeof(Scene::SceneUniforms));
                        writer.update(device_.device(), set);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, groups(volume_.width), groups(slice_count),
                                     groups(volume_.depth));

                        // Readable by the view march (CloudPass), fragment stage.
                        transition(cmd, volume_.image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                  VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
