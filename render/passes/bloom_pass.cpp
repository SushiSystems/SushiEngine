/**************************************************************************/
/* bloom_pass.cpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/bloom_pass.hpp"

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
                constexpr VkFormat BLOOM_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;
                constexpr std::uint32_t MAX_MIPS = 6;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                std::uint32_t mip_extent(std::uint32_t base, std::uint32_t level) noexcept
                {
                    return std::max(base >> level, 1u);
                }

                std::uint32_t pyramid_mips(std::uint32_t width, std::uint32_t height) noexcept
                {
                    std::uint32_t largest = std::max(width, height);
                    std::uint32_t levels = 1;
                    while (largest > 1 && levels < MAX_MIPS)
                    {
                        largest >>= 1;
                        ++levels;
                    }
                    // The upsample recurrence needs at least a coarse and a fine level to walk.
                    return std::max(levels, 2u);
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

                Resources::SamplerDesc linear_sampler() noexcept
                {
                    Resources::SamplerDesc desc;
                    desc.filter = VK_FILTER_LINEAR;
                    desc.mipmap_mode = VK_SAMPLER_MIPMAP_MODE_NEAREST;
                    desc.max_lod = static_cast<float>(MAX_MIPS);
                    return desc;
                }
            } // namespace

            BloomPass::BloomPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                VkDescriptorSetLayoutBinding bindings[3]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
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
                              "vkCreateDescriptorSetLayout(bloom)");

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
                              "vkCreatePipelineLayout(bloom)");

                create_pipelines();
            }

            BloomPass::~BloomPass()
            {
                destroy_images();
                destroy_pipelines();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void BloomPass::create_pipelines()
            {
                down_pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("bloom_down.comp"));
                up_pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("bloom_up.comp"));
            }

            void BloomPass::destroy_pipelines()
            {
                if (down_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), down_pipeline_, nullptr);
                if (up_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), up_pipeline_, nullptr);
                down_pipeline_ = VK_NULL_HANDLE;
                up_pipeline_ = VK_NULL_HANDLE;
            }

            void BloomPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void BloomPass::create_pyramid(Pyramid& pyramid)
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = BLOOM_FORMAT;
                image_info.extent = {base_width_, base_height_, 1};
                image_info.mipLevels = mips_;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc,
                                             &pyramid.image, &pyramid.allocation, nullptr),
                              "vmaCreateImage(bloom)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = pyramid.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = BLOOM_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = mips_;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                                &pyramid.sample_view),
                              "vkCreateImageView(bloom sample)");

                pyramid.mip_views.resize(mips_, VK_NULL_HANDLE);
                for (std::uint32_t level = 0; level < mips_; ++level)
                {
                    VkImageViewCreateInfo mip_info = view_info;
                    mip_info.subresourceRange.baseMipLevel = level;
                    mip_info.subresourceRange.levelCount = 1;
                    Vulkan::check(vkCreateImageView(device_.device(), &mip_info, nullptr,
                                                    &pyramid.mip_views[level]),
                                  "vkCreateImageView(bloom mip)");
                }
            }

            void BloomPass::destroy_pyramid(Pyramid& pyramid)
            {
                for (VkImageView& view : pyramid.mip_views)
                    if (view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), view, nullptr);
                pyramid.mip_views.clear();
                if (pyramid.sample_view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), pyramid.sample_view, nullptr);
                if (pyramid.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), pyramid.image, pyramid.allocation);
                pyramid.sample_view = VK_NULL_HANDLE;
                pyramid.image = VK_NULL_HANDLE;
                pyramid.allocation = VK_NULL_HANDLE;
            }

            void BloomPass::create_images(std::uint32_t width, std::uint32_t height)
            {
                base_width_ = std::max((width + 1) / 2, 1u);
                base_height_ = std::max((height + 1) / 2, 1u);
                mips_ = pyramid_mips(base_width_, base_height_);
                create_pyramid(down_);
                create_pyramid(up_);
            }

            void BloomPass::destroy_images()
            {
                destroy_pyramid(down_);
                destroy_pyramid(up_);
                base_width_ = 0;
                base_height_ = 0;
                mips_ = 0;
            }

            void BloomPass::register_pass(Graph::RenderGraph& graph,
                                          const Frame::FrameContext& frame)
            {
                if (!frame.targets.bloom.valid() || !frame.targets.post_color.valid())
                    return;

                // Size the pyramid to the post extent (the resolved output grid, or the render
                // extent with no resolve), matching the half-extent bloom target the graph made.
                const std::uint32_t post_w = frame.post_width();
                const std::uint32_t post_h = frame.post_height();
                const std::uint32_t want_width = std::max((post_w + 1) / 2, 1u);
                const std::uint32_t want_height = std::max((post_h + 1) / 2, 1u);
                if (base_width_ != want_width || base_height_ != want_height)
                {
                    if (down_.image != VK_NULL_HANDLE)
                        vkDeviceWaitIdle(device_.device());
                    destroy_images();
                    create_images(post_w, post_h);
                }

                const Graph::TextureHandle source = frame.targets.post_color;
                const Graph::TextureHandle bloom = frame.targets.bloom;
                const float radius = 1.0f;

                graph.add_pass(
                    "bloom",
                    [source, bloom](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(source, Graph::TextureAccess::SampledCompute);
                        builder.write(bloom, Graph::TextureAccess::StorageComputeWrite);
                    },
                    [this, &frame, source, bloom, radius](VkCommandBuffer cmd,
                                                          const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(linear_sampler());
                        const VkImageView source_view = context.sampled_view(source);

                        // Both pyramids start undefined and are fully rewritten; move the whole
                        // chain to GENERAL for read/write.
                        transition(cmd, down_.image, 0, mips_, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        transition(cmd, up_.image, 0, mips_, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        Push push{};

                        // Downsample: level 0 from the HDR scene (Karis-averaged), each finer
                        // level from the one above.
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, down_pipeline_);
                        for (std::uint32_t level = 0; level < mips_; ++level)
                        {
                            const std::uint32_t dw = mip_extent(base_width_, level);
                            const std::uint32_t dh = mip_extent(base_height_, level);
                            const VkImageView src_view = level == 0 ? source_view : down_.sample_view;
                            const float src_lod = level == 0 ? 0.0f : static_cast<float>(level - 1);
                            // The graph keeps the level-0 source in shader-read-only; the pass
                            // keeps its own pyramid in GENERAL across the whole build.
                            const VkImageLayout src_layout =
                                level == 0 ? VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
                                           : VK_IMAGE_LAYOUT_GENERAL;

                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter down_writer;
                            down_writer.sampled_image(0, src_view, sampler, src_layout);
                            down_writer.sampled_image(1, src_view, sampler, src_layout);
                            down_writer.storage_image(2, down_.mip_views[level]);
                            down_writer.update(device_.device(), set);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           pipeline_layout_, 0, set);

                            push.a[0] = static_cast<float>(dw);
                            push.a[1] = static_cast<float>(dh);
                            push.a[2] = src_lod;
                            push.a[3] = level == 0 ? 1.0f : 0.0f;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(dw), groups(dh), 1);

                            transition(cmd, down_.image, level, 1, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        }

                        // Upsample: from the coarsest level down, each level adds its own
                        // downsample to the tent-expanded coarser level. The finest step writes
                        // the graph bloom target so the tone map reads it with derived barriers.
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, up_pipeline_);
                        for (std::int32_t level = static_cast<std::int32_t>(mips_) - 2; level >= 0;
                             --level)
                        {
                            const std::uint32_t dw = mip_extent(base_width_, level);
                            const std::uint32_t dh = mip_extent(base_height_, level);
                            const std::uint32_t coarser =
                                static_cast<std::uint32_t>(level) + 1;
                            const bool coarser_is_down =
                                coarser == mips_ - 1; // the top of the chain is a downsample level
                            const VkImageView low_view =
                                coarser_is_down ? down_.sample_view : up_.sample_view;
                            const float low_lod = static_cast<float>(coarser);
                            const bool final_level = level == 0;
                            const VkImageView out_view =
                                final_level ? context.view(bloom)
                                            : up_.mip_views[static_cast<std::uint32_t>(level)];

                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter up_writer;
                            // Both pyramids are the pass's own and stay in GENERAL throughout.
                            up_writer.sampled_image(0, low_view, sampler, VK_IMAGE_LAYOUT_GENERAL);
                            up_writer.sampled_image(1, down_.sample_view, sampler,
                                                    VK_IMAGE_LAYOUT_GENERAL);
                            up_writer.storage_image(2, out_view);
                            up_writer.update(device_.device(), set);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           pipeline_layout_, 0, set);

                            push.a[0] = static_cast<float>(dw);
                            push.a[1] = static_cast<float>(dh);
                            push.a[2] = low_lod;
                            push.a[3] = static_cast<float>(level); // base lod in the down pyramid
                            push.b[0] = radius;
                            vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                               sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(dw), groups(dh), 1);

                            // The just-written up level becomes readable by the next (finer) step;
                            // the finest level wrote the graph target, which the graph barriers.
                            if (!final_level)
                                transition(cmd, up_.image,
                                           static_cast<std::uint32_t>(level), 1,
                                           VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
