/**************************************************************************/
/* cloud_panorama_pass.cpp                                                */
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

#include "passes/cloud_panorama_pass.hpp"


#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/cloud_light_volume_pass.hpp"
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
                constexpr VkFormat PANORAMA_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;

                std::uint32_t groups(std::uint32_t extent) noexcept
                {
                    return (extent + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                void transition(VkCommandBuffer command, VkImage image, VkImageLayout from,
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
                    vkCmdPipelineBarrier2(command, &dependency);
                }
            } // namespace

            CloudPanoramaPass::CloudPanoramaPass(Vulkan::VulkanDevice& device,
                                                 Resources::ShaderLibrary& shaders,
                                                 Resources::GraphicsPipelineFactory& pipelines,
                                                 Resources::SamplerCache& samplers,
                                                 CloudscapeCompilePass& cloudscape,
                                                 CloudLightVolumePass& light_volume)
                : device_(device), shaders_(shaders), pipelines_(pipelines), cloudscape_(cloudscape),
                  light_volume_(light_volume)
            {
                static_assert(HEIGHT % AMORTIZE_FRAMES == 0,
                             "the panorama must divide evenly across the amortization window");

                VkDescriptorSetLayoutBinding bindings[5]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
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
                // The simulation's weather field: the same coverage authority CloudPass's
                // march applies, so the far field agrees with the near field.
                bindings[4].binding = 4;
                bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[4].descriptorCount = 1;
                bindings[4].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 5;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(cloud panorama)");

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
                              "vkCreatePipelineLayout(cloud panorama)");

                create_panorama();

                // Wraps in U (the azimuth axis) and clamps in V (the polar axis) the way
                // any equirectangular map addresses: a sample straying past the pole
                // should hold the pole's own colour rather than jumping to the far side.
                Resources::SamplerDescription sampler_description{};
                sampler_description.filter = VK_FILTER_LINEAR;
                sampler_description.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_ = samplers.get(sampler_description);

                create_pipeline();
            }

            CloudPanoramaPass::~CloudPanoramaPass()
            {
                destroy_pipeline();
                destroy_panorama();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void CloudPanoramaPass::create_pipeline()
            {
                pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("cloud_panorama.comp"));
            }

            void CloudPanoramaPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void CloudPanoramaPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
                // A shader edit can change the bake's contents; the amortized refresh
                // will catch up within one cycle regardless, so no forced full rebake.
            }

            void CloudPanoramaPass::create_panorama()
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = PANORAMA_FORMAT;
                image_info.extent = {WIDTH, HEIGHT, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc,
                                             &panorama_.image, &panorama_.allocation, nullptr),
                              "vmaCreateImage(cloud panorama)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = panorama_.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = PANORAMA_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &view_info, nullptr, &panorama_.view),
                    "vkCreateImageView(cloud panorama)");
            }

            void CloudPanoramaPass::destroy_panorama()
            {
                if (panorama_.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), panorama_.view, nullptr);
                if (panorama_.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), panorama_.image, panorama_.allocation);
                panorama_.view = VK_NULL_HANDLE;
                panorama_.image = VK_NULL_HANDLE;
                panorama_.allocation = VK_NULL_HANDLE;
            }

            void CloudPanoramaPass::register_pass(Graph::RenderGraph& graph,
                                                  const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr || !frame.environment->clouds.enabled)
                    return;

                // Not change-gated: like the light volume and shadow map, this tracks a
                // continuously drifting input (the camera's own position on the sphere,
                // which the bake's azimuth/up basis is built from) rather than a
                // discrete author edit, so a fixed row group refreshes every frame.
                const std::uint32_t row_group = frame_counter_ % AMORTIZE_FRAMES;
                ++frame_counter_;
                const bool first_bake = !built_;
                built_ = true;

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "cloud-panorama",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The panorama is pass-owned and barriered by hand below, exactly
                        // like the T3 field, the light volume, and the shadow map; the one
                        // graph resource is the scene uniform block the bake reads the
                        // camera basis and sun direction from.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, uniforms, row_group,
                     first_bake](VkCommandBuffer command, const Graph::PassContext& context)
                    {
                        const Push push{row_group * ROW_COUNT, ROW_COUNT};

                        transition(command, panorama_.image,
                                  first_bake ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL,
                                  VK_IMAGE_LAYOUT_GENERAL,
                                  first_bake ? VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT
                                             : VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                  VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                  first_bake ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_SAMPLED_READ_BIT,
                                  VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_image(0, panorama_.view);
                        writer.sampled_image(1, cloudscape_.field_view(), cloudscape_.sampler(),
                                             VK_IMAGE_LAYOUT_GENERAL);
                        writer.sampled_image(2, light_volume_.view(), light_volume_.sampler(),
                                             VK_IMAGE_LAYOUT_GENERAL);
                        writer.uniform_buffer(3, context.buffer(uniforms), sizeof(Scene::SceneUniforms));
                        // The far window, not the simulation's raw field: this bake spends most
                        // of its march past the near window, and since the cloudscape bake resolves
                        // coverage per column the far field already *is* the meteorology, resolved.
                        writer.sampled_image(4, cloudscape_.far_view(), cloudscape_.sampler(),
                                             VK_IMAGE_LAYOUT_GENERAL);
                        writer.update(device_.device(), set);
                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdPushConstants(command, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0, sizeof(Push), &push);
                        vkCmdDispatch(command, groups(WIDTH), groups(ROW_COUNT), 1);

                        // Readable by a future fragment-stage consumer (reflection probe
                        // capture); kept in GENERAL like every other bake this frame's
                        // descriptor set samples.
                        transition(command, panorama_.image, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
