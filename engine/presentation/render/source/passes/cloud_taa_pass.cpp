/**************************************************************************/
/* cloud_taa_pass.cpp                                                    */
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

#include "passes/cloud_taa_pass.hpp"

#include <algorithm>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
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
                constexpr VkFormat COLOR_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                // The design doc asks for an R8 history weight; R8_UNORM is not on
                // Vulkan's mandatory storage-image format list, so — like HiZPass,
                // CloudLightVolumePass, and CloudShadowMapPass before it — this bakes
                // its single channel into the codebase's safe R32_SFLOAT precedent.
                constexpr VkFormat WEIGHT_FORMAT = VK_FORMAT_R32_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                void transition(VkCommandBuffer cmd, VkImage image, VkImageLayout old_layout,
                                VkPipelineStageFlags2 source, VkPipelineStageFlags2 destination,
                                VkAccessFlags2 source_access, VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination;
                    barrier.dstAccessMask = destination_access;
                    barrier.oldLayout = old_layout;
                    barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
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

                void create_one(Vulkan::VulkanDevice& device, VkFormat format,
                                std::uint32_t width, std::uint32_t height, VkImage& image,
                                VmaAllocation& allocation, VkImageView& view)
                {
                    VkImageCreateInfo image_info{};
                    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    image_info.imageType = VK_IMAGE_TYPE_2D;
                    image_info.format = format;
                    image_info.extent = {width, height, 1};
                    image_info.mipLevels = 1;
                    image_info.arrayLayers = 1;
                    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                    image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    Vulkan::check(
                        vmaCreateImage(device.allocator(), &image_info, &alloc, &image, &allocation, nullptr),
                        "vmaCreateImage(cloud taa)");

                    VkImageViewCreateInfo view_info{};
                    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                    view_info.image = image;
                    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                    view_info.format = format;
                    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    view_info.subresourceRange.levelCount = 1;
                    view_info.subresourceRange.layerCount = 1;
                    Vulkan::check(vkCreateImageView(device.device(), &view_info, nullptr, &view),
                                  "vkCreateImageView(cloud taa)");
                }

                void destroy_one(Vulkan::VulkanDevice& device, VkImage& image,
                                 VmaAllocation& allocation, VkImageView& view)
                {
                    if (view != VK_NULL_HANDLE)
                        vkDestroyImageView(device.device(), view, nullptr);
                    if (image != VK_NULL_HANDLE)
                        vmaDestroyImage(device.allocator(), image, allocation);
                    view = VK_NULL_HANDLE;
                    image = VK_NULL_HANDLE;
                    allocation = VK_NULL_HANDLE;
                }
            } // namespace

            CloudTAAPass::CloudTAAPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Resources::SamplerCache& samplers, std::uint32_t output_width,
                                       std::uint32_t output_height)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                VkDescriptorSetLayoutBinding bindings[10]{};
                bindings[0] = {0, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[1] = {1, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[2] = {2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[3] = {3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[4] = {4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[5] = {5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[6] = {6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};
                bindings[7] = {7, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT,
                              nullptr};
                bindings[8] = {8, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT,
                              nullptr};
                // The march's mean-depth MRT sibling, for the translation-aware sky
                // reprojection fallback (see cloud_taa.comp's binding 9 comment).
                bindings[9] = {9, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                              VK_SHADER_STAGE_COMPUTE_BIT, nullptr};

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 10;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(cloud taa)");

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
                              "vkCreatePipelineLayout(cloud taa)");

                width_ = std::max<std::uint32_t>(1u, output_width / 2u);
                height_ = std::max<std::uint32_t>(1u, output_height / 2u);
                create_history();

                Resources::SamplerDescription sampler_desc{};
                sampler_ = samplers.get(sampler_desc);

                create_pipeline();
            }

            CloudTAAPass::~CloudTAAPass()
            {
                destroy_pipeline();
                destroy_history();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CloudTAAPass::create_history()
            {
                for (Slot& slot : slots_)
                {
                    create_one(device_, COLOR_FORMAT, width_, height_, slot.color_image,
                              slot.color_allocation, slot.color_view);
                    create_one(device_, WEIGHT_FORMAT, width_, height_, slot.weight_image,
                              slot.weight_allocation, slot.weight_view);
                }
                built_ = false;
                history_valid_ = false;
                current_color_view_ = slots_[0].color_view;
            }

            void CloudTAAPass::destroy_history()
            {
                for (Slot& slot : slots_)
                {
                    destroy_one(device_, slot.color_image, slot.color_allocation, slot.color_view);
                    destroy_one(device_, slot.weight_image, slot.weight_allocation, slot.weight_view);
                }
            }

            void CloudTAAPass::resize(std::uint32_t output_width, std::uint32_t output_height)
            {
                const std::uint32_t new_width = std::max<std::uint32_t>(1u, output_width / 2u);
                const std::uint32_t new_height = std::max<std::uint32_t>(1u, output_height / 2u);
                if (new_width == width_ && new_height == height_)
                    return;
                destroy_history();
                width_ = new_width;
                height_ = new_height;
                create_history();
            }

            void CloudTAAPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_, shaders_.module("cloud_taa.comp"));
            }

            void CloudTAAPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void CloudTAAPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void CloudTAAPass::register_pass(Graph::RenderGraph& graph,
                                             const Frame::FrameContext& frame)
            {
                const bool clouds_on =
                    frame.environment != nullptr && frame.environment->clouds.enabled;
                if (!clouds_on)
                {
                    // Nothing new to resolve; the next enabled frame must not blend a
                    // stale accumulation back in.
                    history_valid_ = false;
                    return;
                }

                // Ping-ponged by frame parity, exactly like ViewResources's main history:
                // this frame writes into write_slot and reads read_slot's last write as
                // the temporal history.
                const std::uint32_t write_slot = frame.index % 2u;
                const std::uint32_t read_slot = 1u - write_slot;
                const bool first_bake = !built_;
                built_ = true;
                const bool history_valid_this_frame = history_valid_;
                history_valid_ = true;

                graph.add_pass(
                    "cloud-taa",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(frame.targets.cloud, Graph::TextureAccess::SampledCompute);
                        builder.read(frame.targets.cloud_depth,
                                     Graph::TextureAccess::SampledCompute);
                        builder.read(frame.targets.velocity, Graph::TextureAccess::SampledCompute);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledCompute);
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformComputeRead);
                        builder.read(frame.targets.temporal, Graph::BufferAccess::UniformComputeRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, write_slot, read_slot, first_bake,
                     history_valid_this_frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        Slot& write = slots_[write_slot];
                        Slot& read = slots_[read_slot];

                        // Every slot alternates cleanly between exactly two roles frame
                        // to frame *while clouds stay continuously enabled* — a slot
                        // moving into the write role was last read by this pass's own
                        // compute history sample, and a slot moving into the read role
                        // was last read by CloudCompositePass's fragment shader (see the
                        // post-dispatch transition below). A disable/re-enable gap can
                        // land the same slot in the write role twice in a row (frame
                        // parity keeps advancing while this pass sits out), which breaks
                        // that fixed-source assumption; !history_valid_this_frame is true
                        // on exactly that first frame back, same as a true first bake, so
                        // both fall back to a conservative "wait on everything" source
                        // instead of guessing wrong.
                        const bool safe_source = first_bake || !history_valid_this_frame;
                        const VkImageLayout old_layout =
                            first_bake ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL;
                        const VkPipelineStageFlags2 safe_stage =
                            first_bake ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                                      : VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
                        const VkAccessFlags2 safe_access =
                            first_bake ? VK_ACCESS_2_NONE
                                      : (VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT);

                        transition(cmd, write.color_image, old_layout,
                                  safe_source ? safe_stage : VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  safe_source ? safe_access : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        transition(cmd, write.weight_image, old_layout,
                                  safe_source ? safe_stage : VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  safe_source ? safe_access : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        transition(cmd, read.color_image, old_layout,
                                  safe_source ? safe_stage : VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  safe_source ? safe_access : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        transition(cmd, read.weight_image, old_layout,
                                  safe_source ? safe_stage : VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  safe_source ? safe_access : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                        const VkSampler point_or_linear = sampler_;
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_image(0, write.color_view);
                        writer.storage_image(1, write.weight_view);
                        writer.sampled_image(2, read.color_view, point_or_linear, VK_IMAGE_LAYOUT_GENERAL);
                        writer.sampled_image(3, read.weight_view, point_or_linear, VK_IMAGE_LAYOUT_GENERAL);
                        writer.sampled_image(4, context.sampled_view(frame.targets.cloud), point_or_linear);
                        writer.sampled_image(5, context.sampled_view(frame.targets.velocity),
                                             point_or_linear);
                        writer.sampled_image(6, context.sampled_view(frame.targets.depth), point_or_linear);
                        writer.uniform_buffer(7, context.buffer(frame.targets.uniforms),
                                              sizeof(Scene::SceneUniforms));
                        writer.uniform_buffer(8, context.buffer(frame.targets.temporal),
                                              sizeof(Scene::TemporalUniforms));
                        writer.sampled_image(9, context.sampled_view(frame.targets.cloud_depth),
                                             point_or_linear);
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        const Push push{history_valid_this_frame ? 1u : 0u,
                                       frame.quality.cloud_variance_clip ? 1u : 0u};
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, groups(width_), groups(height_), 1);

                        // Readable by CloudCompositePass's fragment shader this same frame.
                        transition(cmd, write.color_image, VK_IMAGE_LAYOUT_GENERAL,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                  VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        transition(cmd, write.weight_image, VK_IMAGE_LAYOUT_GENERAL,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                  VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                        current_color_view_ = write.color_view;
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
