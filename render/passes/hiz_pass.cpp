/**************************************************************************/
/* hiz_pass.cpp                                                           */
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

#include "passes/hiz_pass.hpp"

#include <algorithm>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
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
                constexpr VkFormat HIZ_FORMAT = VK_FORMAT_R32_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                std::uint32_t mip_levels(std::uint32_t width, std::uint32_t height) noexcept
                {
                    std::uint32_t largest = std::max(width, height);
                    std::uint32_t levels = 1;
                    while (largest > 1)
                    {
                        largest >>= 1;
                        ++levels;
                    }
                    return levels;
                }

                std::uint32_t mip_extent(std::uint32_t base, std::uint32_t level) noexcept
                {
                    return std::max(base >> level, 1u);
                }

                void transition(VkCommandBuffer cmd, VkImage image, std::uint32_t base_mip,
                                std::uint32_t mip_count, VkImageLayout from, VkImageLayout to,
                                VkPipelineStageFlags2 source, VkPipelineStageFlags2 destination,
                                VkAccessFlags2 source_access, VkAccessFlags2 destination_access)
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
                    barrier.subresourceRange.baseMipLevel = base_mip;
                    barrier.subresourceRange.levelCount = mip_count;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }

                Resources::SamplerDesc point_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_NEAREST;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                    desc.max_lod = 16.0f; // reach every mip the pyramid holds
                    return desc;
                }
            } // namespace

            HizPass::HizPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 3;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(hiz)");

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
                              "vkCreatePipelineLayout(hiz)");

                create_pipeline();
            }

            HizPass::~HizPass()
            {
                destroy_image();
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void HizPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_, shaders_.module("hiz.comp"));
            }

            void HizPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void HizPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void HizPass::create_image(std::uint32_t width, std::uint32_t height)
            {
                width_ = width;
                height_ = height;
                mips_ = mip_levels(width, height);

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = HIZ_FORMAT;
                image_info.extent = {width, height, 1};
                image_info.mipLevels = mips_;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &image_,
                                             &allocation_, nullptr),
                              "vmaCreateImage(hiz)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = HIZ_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = mips_;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &sample_view_),
                              "vkCreateImageView(hiz sample)");

                // One single-mip storage view per level: a storage image cannot be written
                // through a multi-mip view.
                mip_views_.resize(mips_, VK_NULL_HANDLE);
                for (std::uint32_t level = 0; level < mips_; ++level)
                {
                    VkImageViewCreateInfo mip_info = view_info;
                    mip_info.subresourceRange.baseMipLevel = level;
                    mip_info.subresourceRange.levelCount = 1;
                    Vulkan::check(vkCreateImageView(device_.device(), &mip_info, nullptr,
                                                    &mip_views_[level]),
                                  "vkCreateImageView(hiz mip)");
                }
            }

            void HizPass::destroy_image()
            {
                for (VkImageView& view : mip_views_)
                    if (view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), view, nullptr);
                mip_views_.clear();
                if (sample_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), sample_view_, nullptr);
                if (image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), image_, allocation_);
                sample_view_ = VK_NULL_HANDLE;
                image_ = VK_NULL_HANDLE;
                allocation_ = VK_NULL_HANDLE;
                width_ = 0;
                height_ = 0;
                mips_ = 0;
            }

            void HizPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!frame.settings.ssr.enabled || frame.camera == nullptr ||
                    !frame.targets.depth.valid())
                    return;

                if (width_ != frame.width || height_ != frame.height)
                {
                    // The extent changed (a resize). The swapchain recreation that drove it
                    // idled the device, so tearing the old pyramid down here is safe.
                    if (image_ != VK_NULL_HANDLE)
                        vkDeviceWaitIdle(device_.device());
                    destroy_image();
                    create_image(frame.width, frame.height);
                }

                const float near_plane = frame.camera->near_plane;
                const Graph::TextureHandle depth = frame.targets.depth;

                graph.add_pass(
                    "hi-z",
                    [depth](Graph::RenderPassBuilder& builder)
                    {
                        // Declaring the depth read is what makes the graph barrier the prepass
                        // depth to shader-read before this compute samples it; the pyramid
                        // itself is pass-owned and barriered by hand below.
                        builder.read(depth, Graph::TextureAccess::SampledCompute);
                    },
                    [this, &frame, depth, near_plane](VkCommandBuffer cmd,
                                                      const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(point_sampler());
                        const VkImageView depth_view = context.sampled_view(depth);

                        // Every level starts undefined (its previous contents are dead) and is
                        // rewritten, so the whole chain transitions to GENERAL for writing.
                        transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);

                        for (std::uint32_t level = 0; level < mips_; ++level)
                        {
                            const std::uint32_t dw = mip_extent(width_, level);
                            const std::uint32_t dh = mip_extent(height_, level);
                            const std::uint32_t sw =
                                level == 0 ? width_ : mip_extent(width_, level - 1);
                            const std::uint32_t sh =
                                level == 0 ? height_ : mip_extent(height_, level - 1);
                            const std::uint32_t src_level = level == 0 ? 0 : level - 1;

                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter writer;
                            writer.sampled_image(0, depth_view, sampler);
                            writer.storage_image(1, mip_views_[src_level]);
                            writer.storage_image(2, mip_views_[level]);
                            writer.update(device_.device(), set);

                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           pipeline_layout_, 0, set);
                            Push push{};
                            push.a[0] = level;
                            push.a[1] = dw;
                            push.a[2] = dh;
                            push.a[3] = sw;
                            push.b[0] = sh;
                            push.c[0] = near_plane;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(dw), groups(dh), 1);

                            // Make this level readable by the next dispatch (which minimises it).
                            if (level + 1 < mips_)
                                transition(cmd, image_, level, 1, VK_IMAGE_LAYOUT_GENERAL,
                                           VK_IMAGE_LAYOUT_GENERAL,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_READ_BIT);
                        }

                        // The whole pyramid is now visible to the passes that sample it (SSR,
                        // and later Phase 10 culling). It stays in GENERAL, which is samplable.
                        transition(cmd, image_, 0, mips_, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
