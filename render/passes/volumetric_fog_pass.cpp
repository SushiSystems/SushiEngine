/**************************************************************************/
/* volumetric_fog_pass.cpp                                                */
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

#include "passes/volumetric_fog_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/atmosphere_lut_pass.hpp"
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
                constexpr VkFormat FOG_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr std::uint32_t FOG_SIZE = 32;
                constexpr std::uint32_t GROUP_SIZE = 8;

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

            VolumetricFogPass::VolumetricFogPass(Vulkan::VulkanDevice& device,
                                                 Resources::ShaderLibrary& shaders,
                                                 Resources::GraphicsPipelineFactory& pipelines,
                                                 AtmosphereLutPass& atmosphere)
                : device_(device), shaders_(shaders), pipelines_(pipelines), atmosphere_(atmosphere)
            {
                VkDescriptorSetLayoutBinding bindings[4]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[3].binding = 3;
                bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[3].descriptorCount = 1;
                bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 4;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(volumetric fog)");

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
                              "vkCreatePipelineLayout(volumetric fog)");

                create_volume();
                create_volume_buffers();
                create_pipeline();
            }

            VolumetricFogPass::~VolumetricFogPass()
            {
                destroy_pipeline();
                destroy_volume();
                destroy_volume_buffers();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void VolumetricFogPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("fog_scatter.comp"));
            }

            void VolumetricFogPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void VolumetricFogPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void VolumetricFogPass::create_volume()
            {
                volume_.size = FOG_SIZE;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = FOG_FORMAT;
                image_info.extent = {FOG_SIZE, FOG_SIZE, FOG_SIZE};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc,
                                             &volume_.image, &volume_.allocation, nullptr),
                              "vmaCreateImage(volumetric fog)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = volume_.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = FOG_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                                &volume_.view),
                              "vkCreateImageView(volumetric fog)");
            }

            void VolumetricFogPass::destroy_volume()
            {
                if (volume_.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), volume_.view, nullptr);
                if (volume_.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), volume_.image, volume_.allocation);
                volume_.view = VK_NULL_HANDLE;
                volume_.image = VK_NULL_HANDLE;
                volume_.allocation = VK_NULL_HANDLE;
            }

            void VolumetricFogPass::create_volume_buffers()
            {
                // A small ring of host-visible uniform buffers for the local fog volumes,
                // cycled per frame so a buffer is never rewritten while a prior submit reads.
                for (std::uint32_t i = 0; i < RING; ++i)
                {
                    VkBufferCreateInfo buffer_info{};
                    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    buffer_info.size = sizeof(VolumesBlock);
                    buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                                  &volume_buffers_[i], &volume_allocations_[i],
                                                  &mapped),
                                  "vmaCreateBuffer(fog volumes)");
                    volume_mapped_[i] = mapped.pMappedData;
                }
            }

            void VolumetricFogPass::destroy_volume_buffers()
            {
                for (std::uint32_t i = 0; i < RING; ++i)
                    if (volume_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), volume_buffers_[i],
                                         volume_allocations_[i]);
            }

            void VolumetricFogPass::register_pass(Graph::RenderGraph& graph,
                                                  const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr)
                    return;
                const FogParams& fog = frame.environment->fog;

                Push push{};
                push.color_density[0] = static_cast<float>(fog.scattering_color.x);
                push.color_density[1] = static_cast<float>(fog.scattering_color.y);
                push.color_density[2] = static_cast<float>(fog.scattering_color.z);
                push.color_density[3] = fog.density;
                push.params[0] = fog.height_falloff;
                push.params[1] = fog.ambient;
                push.params[2] = fog.phase_anisotropy;
                // The author's toggle, dropped by the lowest tier (fog is the least
                // essential atmosphere term).
                const bool fog_on = fog.enabled && frame.quality.volumetric_fog;
                push.params[3] = fog_on ? 1.0f : 0.0f;

                // Pack the local fog volumes camera-relative into this frame's ring buffer.
                const std::uint32_t ring = frame.index % RING;
                VolumesBlock* block = static_cast<VolumesBlock*>(volume_mapped_[ring]);
                const Environment& environment = *frame.environment;
                int volume_count = 0;
                if (fog_on)
                {
                    const int available = environment.fog_volume_count < 0
                                              ? 0
                                              : environment.fog_volume_count;
                    volume_count = available < static_cast<int>(MAX_VOLUMES)
                                       ? available
                                       : static_cast<int>(MAX_VOLUMES);
                    for (int i = 0; i < volume_count; ++i)
                    {
                        const FogVolume& v = environment.fog_volumes[i];
                        block->volume[i * 3 + 0][0] =
                            static_cast<float>(v.center.x - frame.eye[0]);
                        block->volume[i * 3 + 0][1] =
                            static_cast<float>(v.center.y - frame.eye[1]);
                        block->volume[i * 3 + 0][2] =
                            static_cast<float>(v.center.z - frame.eye[2]);
                        block->volume[i * 3 + 0][3] =
                            v.shape == FogVolumeShape::Ellipsoid ? 1.0f : 0.0f;
                        block->volume[i * 3 + 1][0] = static_cast<float>(v.extent.x);
                        block->volume[i * 3 + 1][1] = static_cast<float>(v.extent.y);
                        block->volume[i * 3 + 1][2] = static_cast<float>(v.extent.z);
                        block->volume[i * 3 + 1][3] = v.edge_falloff;
                        block->volume[i * 3 + 2][0] = static_cast<float>(v.color.x);
                        block->volume[i * 3 + 2][1] = static_cast<float>(v.color.y);
                        block->volume[i * 3 + 2][2] = static_cast<float>(v.color.z);
                        block->volume[i * 3 + 2][3] = v.density;
                    }
                }
                block->count[0] = static_cast<float>(volume_count);
                const VkBuffer volume_buffer = volume_buffers_[ring];

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "volumetric-fog",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The volume is pass-owned and barriered by hand; the scene block is
                        // the one graph resource, read for the camera basis and the sun.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, push, uniforms, volume_buffer](
                        VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});

                        transition(cmd, volume_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkBuffer scene_buffer = context.buffer(uniforms);
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_image(0, volume_.view);
                        writer.uniform_buffer(1, scene_buffer, sizeof(Scene::SceneUniforms));
                        writer.sampled_image(2, atmosphere_.transmittance_view(), sampler);
                        writer.uniform_buffer(3, volume_buffer, sizeof(VolumesBlock));
                        writer.update(device_.device(), set);
                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, groups(volume_.size), groups(volume_.size), 1);

                        transition(cmd, volume_.image, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
