/**************************************************************************/
/* cloud_noise.cpp                                                        */
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

#include "textures/cloud_noise.hpp"

#include "resources/descriptor_heap.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Textures
        {
            namespace
            {
                // Resolution is chosen against the tile scale each volume is stretched
                // over (see the genus profiles' shape_scale/detail_scale): the goal is a
                // texel near 50 m of world, so cauliflower billows and fine erosion are
                // actually representable. Shape 128^3 RGBA8 ~ 8 MB, detail 64^3 ~ 1 MB.
                constexpr std::uint32_t SHAPE_RESOLUTION = 128;
                constexpr std::uint32_t DETAIL_RESOLUTION = 64;
                constexpr std::uint32_t CIRRUS_RESOLUTION = 96;
                constexpr std::uint32_t WEATHER_RESOLUTION = 512;
                constexpr VkFormat NOISE_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;

                /** @brief The push block both noise shaders declare. */
                struct NoiseParams
                {
                    std::uint32_t resolution;
                    std::uint32_t kind;
                };

                /**
                 * @brief Rounds a dispatch extent up to whole workgroups.
                 * @param extent Texels along the axis.
                 * @param local  Workgroup size along the axis.
                 * @return The group count that covers @p extent.
                 */
                std::uint32_t groups(std::uint32_t extent, std::uint32_t local) noexcept
                {
                    return (extent + local - 1) / local;
                }
            } // namespace

            CloudNoise::CloudNoise(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                   Resources::GraphicsPipelineFactory& pipelines,
                                   Resources::SamplerCache& samplers,
                                   Resources::DescriptorHeap& heap)
                : device_(device), heap_(heap)
            {
                Resources::SamplerDesc sampler_desc;
                sampler_desc.address_mode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler_ = samplers.get(sampler_desc);

                create_volume(SHAPE, SHAPE_RESOLUTION, true);
                create_volume(DETAIL, DETAIL_RESOLUTION, true);
                create_volume(CIRRUS, CIRRUS_RESOLUTION, true);
                create_volume(WEATHER, WEATHER_RESOLUTION, false);

                // One storage-image binding, written by whichever noise shader is bound.
                VkDescriptorSetLayoutBinding binding{};
                binding.binding = 0;
                binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                binding.descriptorCount = 1;
                binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 1;
                layout_info.pBindings = &binding;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(noise)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(NoiseParams);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(noise)");

                VkDescriptorPoolSize size{};
                size.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                size.descriptorCount = SLOT_COUNT;

                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.maxSets = SLOT_COUNT;
                pool_info.poolSizeCount = 1;
                pool_info.pPoolSizes = &size;
                Vulkan::check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &pool_),
                              "vkCreateDescriptorPool(noise)");

                generate(shaders, pipelines);

                // Registering the finished volumes in the bindless heap is what proves the
                // heap end to end; the material system indexes the same slots later.
                for (Volume& volume : volumes_)
                    volume.heap_index = heap_.allocate_texture(
                        volume.view, sampler_, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            }

            CloudNoise::~CloudNoise()
            {
                for (Volume& volume : volumes_)
                {
                    heap_.free_texture(volume.heap_index);
                    if (volume.view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), volume.view, nullptr);
                    if (volume.image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), volume.image, volume.allocation);
                }
                if (pool_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorPool(device_.device(), pool_, nullptr);
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CloudNoise::create_volume(Slot slot, std::uint32_t resolution,
                                           bool three_dimensional)
            {
                Volume& volume = volumes_[slot];
                volume.resolution = resolution;
                volume.three_dimensional = three_dimensional;
                volume.heap_index = Resources::INVALID_HEAP_INDEX;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = three_dimensional ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D;
                image_info.format = NOISE_FORMAT;
                image_info.extent = {resolution, resolution, three_dimensional ? resolution : 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc,
                                             &volume.image, &volume.allocation, nullptr),
                              "vmaCreateImage(cloud noise)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = volume.image;
                view_info.viewType =
                    three_dimensional ? VK_IMAGE_VIEW_TYPE_3D : VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = NOISE_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &volume.view),
                              "vkCreateImageView(cloud noise)");
            }

            void CloudNoise::generate(Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines)
            {
                const VkPipeline volume_pipeline = pipelines.create_compute(
                    pipeline_layout_, shaders.module("cloud_noise_volume.comp"));
                const VkPipeline weather_pipeline = pipelines.create_compute(
                    pipeline_layout_, shaders.module("cloud_noise_weather.comp"));

                VkDescriptorSet sets[SLOT_COUNT]{};
                VkDescriptorSetLayout layouts[SLOT_COUNT] = {set_layout_, set_layout_, set_layout_,
                                                             set_layout_};
                VkDescriptorSetAllocateInfo set_info{};
                set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                set_info.descriptorPool = pool_;
                set_info.descriptorSetCount = SLOT_COUNT;
                set_info.pSetLayouts = layouts;
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &set_info, sets),
                              "vkAllocateDescriptorSets(noise)");

                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                {
                    VkDescriptorImageInfo image{};
                    image.imageView = volumes_[slot].view;
                    image.imageLayout = VK_IMAGE_LAYOUT_GENERAL;

                    VkWriteDescriptorSet write{};
                    write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                    write.dstSet = sets[slot];
                    write.dstBinding = 0;
                    write.descriptorCount = 1;
                    write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                    write.pImageInfo = &image;
                    vkUpdateDescriptorSets(device_.device(), 1, &write, 0, nullptr);
                }

                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                VkCommandPool command_pool = VK_NULL_HANDLE;
                Vulkan::check(
                    vkCreateCommandPool(device_.device(), &pool_info, nullptr, &command_pool),
                    "vkCreateCommandPool(noise)");

                VkCommandBufferAllocateInfo cmd_info{};
                cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                cmd_info.commandPool = command_pool;
                cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmd_info.commandBufferCount = 1;
                VkCommandBuffer cmd = VK_NULL_HANDLE;
                Vulkan::check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &cmd),
                              "vkAllocateCommandBuffers(noise)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(cmd, &begin), "vkBeginCommandBuffer(noise)");

                const auto transition = [&](VkImage image, VkImageLayout from, VkImageLayout to,
                                            VkPipelineStageFlags2 source_stage,
                                            VkPipelineStageFlags2 destination_stage,
                                            VkAccessFlags2 source_access,
                                            VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source_stage;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination_stage;
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
                };

                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                {
                    Volume& volume = volumes_[slot];
                    transition(volume.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                               VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, 0,
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      volume.three_dimensional ? volume_pipeline
                                                               : weather_pipeline);
                    vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_layout_,
                                            0, 1, &sets[slot], 0, nullptr);

                    NoiseParams params{volume.resolution, slot};
                    vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                       sizeof(NoiseParams), &params);

                    if (volume.three_dimensional)
                        vkCmdDispatch(cmd, groups(volume.resolution, 4), groups(volume.resolution, 4),
                                      groups(volume.resolution, 4));
                    else
                        vkCmdDispatch(cmd, groups(volume.resolution, 8), groups(volume.resolution, 8),
                                      1);

                    transition(volume.image, VK_IMAGE_LAYOUT_GENERAL,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                               VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                }

                Vulkan::check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(noise)");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                VkFence fence = VK_NULL_HANDLE;
                Vulkan::check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence),
                              "vkCreateFence(noise)");

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = cmd;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                Vulkan::check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                              "vkQueueSubmit2(noise)");
                vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX);

                vkDestroyFence(device_.device(), fence, nullptr);
                vkDestroyCommandPool(device_.device(), command_pool, nullptr);
                vkDestroyPipeline(device_.device(), weather_pipeline, nullptr);
                vkDestroyPipeline(device_.device(), volume_pipeline, nullptr);
            }
        } // namespace Textures
    } // namespace Render
} // namespace SushiEngine
