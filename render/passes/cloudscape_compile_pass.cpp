/**************************************************************************/
/* cloudscape_compile_pass.cpp                                           */
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

#include "passes/cloudscape_compile_pass.hpp"

#include <cstring>

#include <SushiEngine/render/environment.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_uniforms.hpp"
#include "textures/cloud_noise.hpp"
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
                constexpr VkFormat FIELD_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
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

            CloudscapeCompilePass::CloudscapeCompilePass(Vulkan::VulkanDevice& device,
                                                         Resources::ShaderLibrary& shaders,
                                                         Resources::GraphicsPipelineFactory& pipelines,
                                                         Resources::SamplerCache& samplers,
                                                         Textures::CloudNoise& noise)
                : device_(device), shaders_(shaders), pipelines_(pipelines), noise_(noise)
            {
                // Field bake: one storage-image output, the scene uniform block (for the
                // deck stack), and the four noise volumes the deck loop samples.
                VkDescriptorSetLayoutBinding f_bindings[6]{};
                f_bindings[0].binding = 0;
                f_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                f_bindings[0].descriptorCount = 1;
                f_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                f_bindings[1].binding = 1;
                f_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                f_bindings[1].descriptorCount = 1;
                f_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                for (std::uint32_t i = 2; i < 6; ++i)
                {
                    f_bindings[i].binding = i;
                    f_bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    f_bindings[i].descriptorCount = 1;
                    f_bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo f_layout_info{};
                f_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                f_layout_info.bindingCount = 6;
                f_layout_info.pBindings = f_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &f_layout_info, nullptr,
                                                          &field_layout_),
                              "vkCreateDescriptorSetLayout(cloudscape field)");

                VkPushConstantRange f_range{};
                f_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                f_range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo f_pipeline_info{};
                f_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                f_pipeline_info.setLayoutCount = 1;
                f_pipeline_info.pSetLayouts = &field_layout_;
                f_pipeline_info.pushConstantRangeCount = 1;
                f_pipeline_info.pPushConstantRanges = &f_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &f_pipeline_info, nullptr,
                                                     &field_pipeline_layout_),
                              "vkCreatePipelineLayout(cloudscape field)");

                // Skip-field downsample: storage-image output plus the fine field as input.
                VkDescriptorSetLayoutBinding s_bindings[2]{};
                s_bindings[0].binding = 0;
                s_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                s_bindings[0].descriptorCount = 1;
                s_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                s_bindings[1].binding = 1;
                s_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                s_bindings[1].descriptorCount = 1;
                s_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo s_layout_info{};
                s_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                s_layout_info.bindingCount = 2;
                s_layout_info.pBindings = s_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &s_layout_info, nullptr,
                                                          &skip_layout_),
                              "vkCreateDescriptorSetLayout(cloudscape skip)");

                VkPipelineLayoutCreateInfo s_pipeline_info{};
                s_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                s_pipeline_info.setLayoutCount = 1;
                s_pipeline_info.pSetLayouts = &skip_layout_;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &s_pipeline_info, nullptr,
                                                     &skip_pipeline_layout_),
                              "vkCreatePipelineLayout(cloudscape skip)");

                create_volume(field_, FIELD_RESOLUTION_XZ, FIELD_RESOLUTION_Y, FIELD_RESOLUTION_XZ);
                create_volume(skip_, FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ,
                             FIELD_RESOLUTION_Y / SKIP_DOWNSAMPLE_Y,
                             FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ);

                // The field tiles periodically in every axis (X/Z wrap the flat bake tile,
                // Y wraps too so a march sample that strays a texel past the union band's
                // edge reads its own boundary back instead of the border colour); linear
                // filtering smooths the block boundaries the bake's discrete texels leave.
                Resources::SamplerDesc sampler_desc{};
                sampler_desc.filter = VK_FILTER_LINEAR;
                sampler_desc.address_mode = VK_SAMPLER_ADDRESS_MODE_REPEAT;
                sampler_ = samplers.get(sampler_desc);

                create_pipelines();
            }

            CloudscapeCompilePass::~CloudscapeCompilePass()
            {
                destroy_pipelines();
                destroy_volume(field_);
                destroy_volume(skip_);
                if (field_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), field_pipeline_layout_, nullptr);
                if (skip_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), skip_pipeline_layout_, nullptr);
                if (field_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), field_layout_, nullptr);
                if (skip_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), skip_layout_, nullptr);
            }

            void CloudscapeCompilePass::create_pipelines()
            {
                field_pipeline_ = pipelines_.create_compute(field_pipeline_layout_,
                                                            shaders_.module("cloudscape_field.comp"));
                skip_pipeline_ = pipelines_.create_compute(skip_pipeline_layout_,
                                                           shaders_.module("cloudscape_skip.comp"));
            }

            void CloudscapeCompilePass::destroy_pipelines()
            {
                if (field_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), field_pipeline_, nullptr);
                if (skip_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), skip_pipeline_, nullptr);
                field_pipeline_ = VK_NULL_HANDLE;
                skip_pipeline_ = VK_NULL_HANDLE;
            }

            void CloudscapeCompilePass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
                // A shader edit can change the field's contents, so force a rebake next frame.
                built_ = false;
            }

            void CloudscapeCompilePass::create_volume(Volume& volume, std::uint32_t width,
                                                       std::uint32_t height, std::uint32_t depth)
            {
                volume.width = width;
                volume.height = height;
                volume.depth = depth;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = FIELD_FORMAT;
                image_info.extent = {width, height, depth};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &volume.image,
                                             &volume.allocation, nullptr),
                              "vmaCreateImage(cloudscape field)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = volume.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = FIELD_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &volume.view),
                              "vkCreateImageView(cloudscape field)");
            }

            void CloudscapeCompilePass::destroy_volume(Volume& volume)
            {
                if (volume.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), volume.view, nullptr);
                if (volume.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), volume.image, volume.allocation);
                volume.view = VK_NULL_HANDLE;
                volume.image = VK_NULL_HANDLE;
                volume.allocation = VK_NULL_HANDLE;
            }

            bool CloudscapeCompilePass::cloudscape_changed(const Snapshot& snapshot)
            {
                if (!built_ || std::memcmp(&snapshot, &last_snapshot_, sizeof(Snapshot)) != 0)
                {
                    last_snapshot_ = snapshot;
                    return true;
                }
                return false;
            }

            void CloudscapeCompilePass::register_pass(Graph::RenderGraph& graph,
                                                       const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr || !frame.environment->clouds.enabled)
                    return;
                const Cloudscape& clouds = frame.environment->clouds;

                Snapshot snapshot{};
                for (int i = 0; i < CLOUD_MAX_DECKS; ++i)
                {
                    snapshot.decks[i].enabled = clouds.decks[i].enabled ? 1u : 0u;
                    snapshot.decks[i].genus = static_cast<std::uint32_t>(clouds.decks[i].genus);
                    snapshot.decks[i].coverage_bias = clouds.decks[i].coverage_bias;
                    snapshot.decks[i].density_scale = clouds.decks[i].density_scale;
                }
                snapshot.weather_scale = clouds.weather_scale;

                const bool dirty = cloudscape_changed(snapshot);
                built_ = true;
                // The last bake still describes the current deck stack; nothing to redo.
                if (!dirty)
                    return;

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "cloudscape-compile",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The field/skip images are pass-owned and barriered by hand below;
                        // the one graph resource is the scene uniform block the bake reads
                        // the deck stack from. A side effect keeps the pass from being
                        // culled — it has no graph-tracked write to keep it alive otherwise.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, uniforms](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const Push push{TILE_METERS};

                        transition(cmd, field_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        {
                            const VkDescriptorSet set = frame.descriptors->allocate(field_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_image(0, field_.view);
                            writer.uniform_buffer(1, context.buffer(uniforms),
                                                  sizeof(Scene::SceneUniforms));
                            writer.sampled_image(2, noise_.shape(), noise_.sampler());
                            writer.sampled_image(3, noise_.detail(), noise_.sampler());
                            writer.sampled_image(4, noise_.weather(), noise_.sampler());
                            writer.sampled_image(5, noise_.cirrus(), noise_.sampler());
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, field_pipeline_);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           field_pipeline_layout_, 0, set);
                            vkCmdPushConstants(cmd, field_pipeline_layout_,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push), &push);
                            vkCmdDispatch(cmd, groups(field_.width), groups(field_.height),
                                          groups(field_.depth));
                        }
                        // Readable by the skip-field build below (compute) and, once this
                        // pass ends, by the view march's own full-quality tap (fragment).
                        transition(cmd, field_.image, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                       VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                        transition(cmd, skip_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        {
                            const VkDescriptorSet set = frame.descriptors->allocate(skip_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_image(0, skip_.view);
                            writer.sampled_image(1, field_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, skip_pipeline_);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           skip_pipeline_layout_, 0, set);
                            vkCmdDispatch(cmd, groups(skip_.width), groups(skip_.height),
                                          groups(skip_.depth));
                        }
                        // Readable by the view march (CloudPass), fragment stage.
                        transition(cmd, skip_.image, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
