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

#include <algorithm>
#include <cmath>

#include "resources/descriptor_heap.hpp"
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
                // The march's combined carve volume (CloudsV2): tiled over the ~2.4 km carve
                // scale a texel is ~19 m of world, comfortably under the finest erosion
                // octave, so per-sample carving never reads its own lattice. 128^3 RGBA8 ~ 8 MB.
                constexpr std::uint32_t MARCH_RESOLUTION = 128;
                constexpr VkFormat NOISE_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;

                // The march volume is the only one with a mip chain, because it is the only one
                // sampled at a world scale fixed in metres by a march whose own step grows with
                // distance: at a hundred kilometres one integration step spans a kilometre
                // against an 18 m texel, and a point sample there is not a cloud, it is
                // whichever noise peak the lattice landed on. Full chain down to 1^3 — the
                // coarsest levels cost 1/7th of a level and are what let the carve converge to
                // the field's own mean instead of stopping at some arbitrary floor.
                // 128 -> 8 levels; the other four volumes stay single-level.
                constexpr std::uint32_t MARCH_MIP_LEVELS = 8;
                static_assert(MARCH_RESOLUTION == (1u << (MARCH_MIP_LEVELS - 1u)),
                              "the march mip chain must reach 1^3, and each level must halve "
                              "exactly — cloud_noise_mip.comp's 2x2x2 box gather assumes it");

                /** @brief The push block both noise generation shaders declare. */
                struct NoiseParameters
                {
                    std::uint32_t resolution;
                    std::uint32_t kind;
                };

                /** @brief The push block cloud_noise_mip.comp declares. */
                struct MipParameters
                {
                    std::uint32_t resolution; /**< Destination extent; the source is twice it. */
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
                Resources::SamplerDescription sampler_description;
                sampler_description.address_mode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler_ = samplers.get(sampler_description);
                // The same sampler, allowed to reach the march volume's coarser levels. A
                // sampler's maxLod is part of its identity, so this cannot be folded into the
                // one above without also lifting the clamp on four volumes that have exactly
                // one level and would gain nothing from it.
                sampler_description.max_lod = static_cast<float>(MARCH_MIP_LEVELS - 1u);
                march_sampler_ = samplers.get(sampler_description);

                create_volume(SHAPE, SHAPE_RESOLUTION, true, 1);
                create_volume(DETAIL, DETAIL_RESOLUTION, true, 1);
                create_volume(CIRRUS, CIRRUS_RESOLUTION, true, 1);
                create_volume(WEATHER, WEATHER_RESOLUTION, false, 1);
                create_volume(MARCH, MARCH_RESOLUTION, true, MARCH_MIP_LEVELS);

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
                range.size = sizeof(NoiseParameters);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(noise)");

                // The downsample reads one level and writes the next, so it needs a second
                // binding and therefore a layout of its own.
                VkDescriptorSetLayoutBinding mip_bindings[2]{binding, binding};
                mip_bindings[1].binding = 1;

                layout_info.bindingCount = 2;
                layout_info.pBindings = mip_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &mip_set_layout_),
                              "vkCreateDescriptorSetLayout(noise mip)");

                range.size = sizeof(MipParameters);
                pipeline_info.pSetLayouts = &mip_set_layout_;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &mip_pipeline_layout_),
                              "vkCreatePipelineLayout(noise mip)");

                // One set per volume for the generators, plus one per downsampled level.
                const std::uint32_t mip_sets = MARCH_MIP_LEVELS - 1u;
                VkDescriptorPoolSize size{};
                size.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                size.descriptorCount = SLOT_COUNT + mip_sets * 2u;

                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.maxSets = SLOT_COUNT + mip_sets;
                pool_info.poolSizeCount = 1;
                pool_info.pPoolSizes = &size;
                Vulkan::check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &pool_),
                              "vkCreateDescriptorPool(noise)");

                generate(shaders, pipelines);

                // Registering the finished volumes in the bindless heap is what proves the
                // heap end to end; the material system indexes the same slots later.
                for (Volume& volume : volumes_)
                    volume.heap_index = heap_.allocate_texture(
                        volume.view,
                        volume.mip_views.size() > 1 ? march_sampler_ : sampler_,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
            }

            CloudNoise::~CloudNoise()
            {
                for (Volume& volume : volumes_)
                {
                    heap_.free_texture(volume.heap_index);
                    for (VkImageView mip : volume.mip_views)
                        if (mip != VK_NULL_HANDLE)
                            vkDestroyImageView(device_.device(), mip, nullptr);
                    if (volume.view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), volume.view, nullptr);
                    if (volume.image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), volume.image, volume.allocation);
                }
                if (pool_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorPool(device_.device(), pool_, nullptr);
                if (mip_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), mip_pipeline_layout_, nullptr);
                if (mip_set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), mip_set_layout_, nullptr);
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CloudNoise::create_volume(Slot slot, std::uint32_t resolution,
                                           bool three_dimensional, std::uint32_t mip_levels)
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
                image_info.mipLevels = mip_levels;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                // A mip'd volume is also the one whose statistics the carve needs, and those
                // are measured on the host from its finest level (see measure_carve_spread).
                if (mip_levels > 1)
                    image_info.usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

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
                view_info.subresourceRange.levelCount = mip_levels;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &volume.view),
                              "vkCreateImageView(cloud noise)");

                // One single-level view per mip: a storage image binding names exactly one
                // level, and both the generator and the downsample write through these.
                volume.mip_views.resize(mip_levels, VK_NULL_HANDLE);
                view_info.subresourceRange.levelCount = 1;
                for (std::uint32_t level = 0; level < mip_levels; ++level)
                {
                    view_info.subresourceRange.baseMipLevel = level;
                    Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                                    &volume.mip_views[level]),
                                  "vkCreateImageView(cloud noise mip)");
                }
            }

            void CloudNoise::generate(Resources::ShaderLibrary& shaders,
                                      Resources::GraphicsPipelineFactory& pipelines)
            {
                const VkPipeline volume_pipeline = pipelines.create_compute(
                    pipeline_layout_, shaders.module("cloud_noise_volume.comp"));
                const VkPipeline weather_pipeline = pipelines.create_compute(
                    pipeline_layout_, shaders.module("cloud_noise_weather.comp"));
                const VkPipeline mip_pipeline = pipelines.create_compute(
                    mip_pipeline_layout_, shaders.module("cloud_noise_mip.comp"));

                const std::uint32_t mip_sets = MARCH_MIP_LEVELS - 1u;

                VkDescriptorSet sets[SLOT_COUNT]{};
                VkDescriptorSetLayout layouts[SLOT_COUNT];
                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                    layouts[slot] = set_layout_;
                VkDescriptorSetAllocateInfo set_info{};
                set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                set_info.descriptorPool = pool_;
                set_info.descriptorSetCount = SLOT_COUNT;
                set_info.pSetLayouts = layouts;
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &set_info, sets),
                              "vkAllocateDescriptorSets(noise)");

                std::vector<VkDescriptorSet> mip_descriptor_sets(mip_sets, VK_NULL_HANDLE);
                std::vector<VkDescriptorSetLayout> mip_layouts(mip_sets, mip_set_layout_);
                set_info.descriptorSetCount = mip_sets;
                set_info.pSetLayouts = mip_layouts.data();
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &set_info,
                                                       mip_descriptor_sets.data()),
                              "vkAllocateDescriptorSets(noise mip)");

                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                {
                    Resources::DescriptorWriter writer;
                    // Level 0 explicitly: the generators write one level, and for the march
                    // volume the whole-chain view would be an invalid storage binding.
                    writer.storage_image(0, volumes_[slot].mip_views[0]);
                    writer.update(device_.device(), sets[slot]);
                }

                const Volume& march = volumes_[MARCH];
                for (std::uint32_t level = 1; level < MARCH_MIP_LEVELS; ++level)
                {
                    Resources::DescriptorWriter writer;
                    writer.storage_image(0, march.mip_views[level - 1]);
                    writer.storage_image(1, march.mip_views[level]);
                    writer.update(device_.device(), mip_descriptor_sets[level - 1]);
                }

                // Where the finest level lands on the host, so the carve's per-level spread is
                // measured from the volume that shipped rather than assumed from its recipe.
                const VkDeviceSize readback_bytes = VkDeviceSize(MARCH_RESOLUTION) *
                                                    MARCH_RESOLUTION * MARCH_RESOLUTION * 4;
                VkBufferCreateInfo readback_info{};
                readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                readback_info.size = readback_bytes;
                readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

                VmaAllocationCreateInfo readback_alloc{};
                readback_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                readback_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                       VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VkBuffer readback = VK_NULL_HANDLE;
                VmaAllocation readback_allocation = VK_NULL_HANDLE;
                VmaAllocationInfo readback_mapping{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &readback_info, &readback_alloc,
                                              &readback, &readback_allocation, &readback_mapping),
                              "vmaCreateBuffer(cloud noise readback)");

                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                VkCommandPool command_pool = VK_NULL_HANDLE;
                Vulkan::check(
                    vkCreateCommandPool(device_.device(), &pool_info, nullptr, &command_pool),
                    "vkCreateCommandPool(noise)");

                VkCommandBufferAllocateInfo command_info{};
                command_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                command_info.commandPool = command_pool;
                command_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                command_info.commandBufferCount = 1;
                VkCommandBuffer command = VK_NULL_HANDLE;
                Vulkan::check(vkAllocateCommandBuffers(device_.device(), &command_info, &command),
                              "vkAllocateCommandBuffers(noise)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(command, &begin), "vkBeginCommandBuffer(noise)");

                const auto transition = [&](VkImage image, VkImageLayout from, VkImageLayout to,
                                            VkPipelineStageFlags2 source_stage,
                                            VkPipelineStageFlags2 destination_stage,
                                            VkAccessFlags2 source_access,
                                            VkAccessFlags2 destination_access,
                                            std::uint32_t base_level = 0,
                                            std::uint32_t level_count = VK_REMAINING_MIP_LEVELS)
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
                    barrier.subresourceRange.baseMipLevel = base_level;
                    barrier.subresourceRange.levelCount = level_count;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(command, &dependency);
                };

                // Pass one: every volume's finest level, from its own generator.
                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                {
                    Volume& volume = volumes_[slot];
                    // Every level, not only the one written here: a level left UNDEFINED would
                    // still be sampled once the whole chain is handed to the shaders below.
                    transition(volume.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                               VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, 0,
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                    vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      volume.three_dimensional ? volume_pipeline
                                                               : weather_pipeline);
                    Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   pipeline_layout_, 0, sets[slot]);

                    NoiseParameters parameters{volume.resolution, slot};
                    vkCmdPushConstants(command, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                       sizeof(NoiseParameters), &parameters);

                    if (volume.three_dimensional)
                        vkCmdDispatch(command, groups(volume.resolution, 4),
                                      groups(volume.resolution, 4), groups(volume.resolution, 4));
                    else
                        vkCmdDispatch(command, groups(volume.resolution, 8),
                                      groups(volume.resolution, 8), 1);
                }

                // Pass two: the march volume's chain, one level at a time. Each level's write
                // must complete before the next reads it, so the barrier is per level rather
                // than one for the whole chain — which is also why this cannot be folded into
                // the loop above.
                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, mip_pipeline);
                for (std::uint32_t level = 1; level < MARCH_MIP_LEVELS; ++level)
                {
                    transition(march.image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                               VK_ACCESS_2_SHADER_STORAGE_READ_BIT, level - 1, 1);

                    Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   mip_pipeline_layout_, 0,
                                                   mip_descriptor_sets[level - 1]);
                    MipParameters mip_parameters{MARCH_RESOLUTION >> level};
                    vkCmdPushConstants(command, mip_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(MipParameters), &mip_parameters);
                    vkCmdDispatch(command, groups(mip_parameters.resolution, 4),
                                  groups(mip_parameters.resolution, 4),
                                  groups(mip_parameters.resolution, 4));
                }

                // The finest level, on its way to the host for the spread measurement. Copied
                // out of GENERAL rather than transitioned to TRANSFER_SRC_OPTIMAL: the layout
                // is legal for a transfer source, and the level is about to become read-only
                // anyway, so a round trip through a third layout would buy nothing.
                transition(march.image, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_2_COPY_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT, VK_ACCESS_2_TRANSFER_READ_BIT, 0,
                           1);

                VkBufferImageCopy region{};
                region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                region.imageSubresource.layerCount = 1;
                region.imageExtent = {MARCH_RESOLUTION, MARCH_RESOLUTION, MARCH_RESOLUTION};
                vkCmdCopyImageToBuffer(command, march.image, VK_IMAGE_LAYOUT_GENERAL, readback, 1,
                                       &region);

                // Pass three: hand every level of every volume to the shaders that sample them.
                for (std::uint32_t slot = 0; slot < SLOT_COUNT; ++slot)
                    transition(volumes_[slot].image, VK_IMAGE_LAYOUT_GENERAL,
                               VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT,
                               VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                               VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                Vulkan::check(vkEndCommandBuffer(command), "vkEndCommandBuffer(noise)");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                VkFence fence = VK_NULL_HANDLE;
                Vulkan::check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence),
                              "vkCreateFence(noise)");

                VkCommandBufferSubmitInfo command_submit{};
                command_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                command_submit.commandBuffer = command;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &command_submit;
                Vulkan::check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                              "vkQueueSubmit2(noise)");
                vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX);

                vmaInvalidateAllocation(device_.allocator(), readback_allocation, 0,
                                        readback_bytes);
                measure_carve_spread(readback_mapping.pMappedData, MARCH_RESOLUTION);
                vmaDestroyBuffer(device_.allocator(), readback, readback_allocation);

                vkDestroyFence(device_.device(), fence, nullptr);
                vkDestroyCommandPool(device_.device(), command_pool, nullptr);
                vkDestroyPipeline(device_.device(), mip_pipeline, nullptr);
                vkDestroyPipeline(device_.device(), weather_pipeline, nullptr);
                vkDestroyPipeline(device_.device(), volume_pipeline, nullptr);
            }

            /**
             * @brief Measures how much shape each mip level of the carve volume threw away.
             *
             * The carve thresholds channel r at `1 - coverage` and relies on that channel being
             * uniform on [0, 1] for the threshold to keep exactly `coverage` of the sky
             * (cloud_noise_volume.comp pushes it through its own CDF for precisely that). A
             * filtered level is *not* uniform — it is narrower, and at the coarsest levels it is
             * nearly a constant — so thresholding a filtered fetch alone would make every
             * distant column all-cloud or all-clear, which is the "distant cloud renders white"
             * failure this whole change is about. What the march needs instead is the spread of
             * the detail the filter removed, so it can integrate the threshold over it.
             *
             * That spread is exact rather than approximate here, because a mip level of this
             * chain is a **box average over disjoint blocks** — the conditional expectation of
             * the field given its block, hence an orthogonal projection — and the variance of an
             * orthogonal decomposition adds:
             *
             *     Var(field) = Var(level l) + E[Var(field | block of level l)]
             *
             * so the residual is `Var(level 0) - Var(level l)` with no assumption about the
             * noise's spectrum anywhere in it. It is measured from the generated volume rather
             * than derived from the recipe so that changing the recipe cannot silently
             * invalidate it.
             *
             * Two second-order caveats, stated rather than buried. The hardware fetch is
             * trilinear *within* a level, which filters slightly more than the block average, so
             * the real residual is a little larger than this; and each level is re-quantised to
             * UNORM8, which adds a variance floor of (1/255)^2/12 — about 1.3e-6 against a
             * base variance near 1/12, four and a half orders of magnitude down.
             *
             * @param level_zero Tightly packed RGBA8 texels of the finest level.
             * @param resolution That level's extent along each axis.
             */
            void CloudNoise::measure_carve_spread(const void* level_zero,
                                                  std::uint32_t resolution)
            {
                const std::size_t levels = volumes_[MARCH].mip_views.size();
                // Entry 0 is zero by construction — no filter has been applied there — and
                // leaving the whole array zero is also the honest fallback if the readback
                // failed: the carve then falls back to a plain hard threshold rather than
                // working from a fabricated spread.
                march_carve_spread_.assign(levels, 0.0f);
                if (level_zero == nullptr || resolution == 0 || levels == 0)
                    return;

                const auto variance = [](const std::vector<float>& field)
                {
                    double sum = 0.0;
                    double sum_of_squares = 0.0;
                    for (float value : field)
                    {
                        sum += value;
                        sum_of_squares += double(value) * double(value);
                    }
                    const double count = double(field.size());
                    const double mean = sum / count;
                    return std::max(sum_of_squares / count - mean * mean, 0.0);
                };

                // Channel r as the shader reads it: the UNORM byte over 255.
                const auto* texels = static_cast<const std::uint8_t*>(level_zero);
                std::uint32_t extent = resolution;
                std::vector<float> field(std::size_t(extent) * extent * extent);
                for (std::size_t i = 0; i < field.size(); ++i)
                    field[i] = float(texels[i * 4]) * (1.0f / 255.0f);

                const double base_variance = variance(field);

                for (std::size_t level = 1; level < levels; ++level)
                {
                    // A 1^3 level cannot be halved again. Unreachable while the static_assert
                    // above ties the level count to the resolution, and left in because the
                    // alternative to an early exit here is an out-of-range gather.
                    if (extent <= 1)
                        break;

                    // The same 2x2x2 box cloud_noise_mip.comp applies, so the statistic
                    // describes the levels the GPU actually sampled.
                    const std::uint32_t half = extent / 2u;
                    std::vector<float> coarse(std::size_t(half) * half * half);
                    for (std::uint32_t z = 0; z < half; ++z)
                        for (std::uint32_t y = 0; y < half; ++y)
                            for (std::uint32_t x = 0; x < half; ++x)
                            {
                                float sum = 0.0f;
                                for (std::uint32_t dz = 0; dz < 2; ++dz)
                                    for (std::uint32_t dy = 0; dy < 2; ++dy)
                                        for (std::uint32_t dx = 0; dx < 2; ++dx)
                                            sum += field[((std::size_t(z * 2 + dz) * extent +
                                                           (y * 2 + dy)) *
                                                              extent +
                                                          (x * 2 + dx))];
                                coarse[(std::size_t(z) * half + y) * half + x] = sum * 0.125f;
                            }
                    field.swap(coarse);
                    extent = half;
                    march_carve_spread_[level] =
                        float(std::sqrt(std::max(base_variance - variance(field), 0.0)));
                }
            }
        } // namespace Textures
    } // namespace Render
} // namespace SushiEngine
