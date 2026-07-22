/**************************************************************************/
/* ibl_pass.cpp                                                           */
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

#include "passes/ibl_pass.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "passes/atmosphere_lut_pass.hpp"
#include "passes/volumetric_fog_pass.hpp"
#include "passes/fullscreen.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "scene/scene_layout.hpp"
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
                constexpr std::uint32_t ENVIRONMENT_RESOLUTION = 128;
                constexpr std::uint32_t SPECULAR_RESOLUTION = 128;
                constexpr std::uint32_t SPECULAR_MIPS = 6;
                constexpr std::uint32_t IRRADIANCE_RESOLUTION = 32;
                constexpr std::uint32_t BRDF_RESOLUTION = 256;
                constexpr std::uint32_t PREFILTER_SAMPLES = 128;
                constexpr std::uint32_t IRRADIANCE_SAMPLES = 128;
                constexpr std::uint32_t BRDF_SAMPLES = 512;
                /** @brief Frames that must pass before the chain may be rebuilt again. */
                constexpr std::uint32_t RECAPTURE_INTERVAL = 4;

                /** @brief The push block the prefilter and irradiance shaders share. */
                struct ConvolveParams
                {
                    std::uint32_t resolution;
                    std::uint32_t source_resolution;
                    float parameter; /**< Roughness for prefilter, source mip for irradiance. */
                    std::uint32_t sample_count;
                };

                /** @brief The push block the BRDF LUT shader reads. */
                struct LutParams
                {
                    std::uint32_t resolution;
                    std::uint32_t sample_count;
                };

                /** @brief Per-face sample grid the SH projection integrates over. */
                constexpr std::uint32_t SH_SAMPLE_RESOLUTION = 32;

                /** @brief The push block the SH projection shader reads. */
                struct ShParams
                {
                    std::uint32_t resolution; /**< Per-face sample grid side. */
                    float source_mip;         /**< Environment mip the radiance is read from. */
                };

                /**
                 * @brief The camera basis that renders one cubemap face.
                 *
                 * Matches Vulkan's face convention: the u axis runs left to right and the
                 * v axis top to bottom of the face image, which is exactly the sense the
                 * fullscreen triangle's NDC already has.
                 */
                struct FaceBasis
                {
                    float forward[3];
                    float right[3];
                    float up[3];
                };

                constexpr FaceBasis FACES[6] = {
                    {{1, 0, 0}, {0, 0, -1}, {0, -1, 0}},
                    {{-1, 0, 0}, {0, 0, 1}, {0, -1, 0}},
                    {{0, 1, 0}, {1, 0, 0}, {0, 0, 1}},
                    {{0, -1, 0}, {1, 0, 0}, {0, 0, -1}},
                    {{0, 0, 1}, {1, 0, 0}, {0, -1, 0}},
                    {{0, 0, -1}, {-1, 0, 0}, {0, -1, 0}},
                };

                /**
                 * @brief Records an image layout transition over a subresource range.
                 * @param cmd          The recording command buffer.
                 * @param image        The image to transition.
                 * @param base_mip     First mip level in the range.
                 * @param mip_count    Mip levels in the range.
                 * @param layers       Array layers in the range, from layer 0.
                 * @param from         Current layout.
                 * @param to           Layout to move to.
                 * @param source       Stage that must complete first.
                 * @param destination  Stage that waits.
                 * @param source_access      Accesses to make available.
                 * @param destination_access Accesses to make visible.
                 */
                void transition(VkCommandBuffer cmd, VkImage image, std::uint32_t base_mip,
                                std::uint32_t mip_count, std::uint32_t layers, VkImageLayout from,
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
                    barrier.subresourceRange.baseMipLevel = base_mip;
                    barrier.subresourceRange.levelCount = mip_count;
                    barrier.subresourceRange.layerCount = layers;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }

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

            IblPass::IblPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Resources::SamplerCache& samplers, Scene::SceneLayout& layout,
                             Textures::CloudNoise& noise, AtmosphereLutPass& atmosphere,
                             VolumetricFogPass& fog)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  noise_(noise), atmosphere_(atmosphere), fog_(fog)
            {
                Resources::SamplerDesc sampler_desc;
                sampler_desc.max_lod = static_cast<float>(SPECULAR_MIPS);
                sampler_ = samplers.get(sampler_desc);

                create_cube(environment_, ENVIRONMENT_RESOLUTION,
                            static_cast<std::uint32_t>(std::log2(ENVIRONMENT_RESOLUTION)) + 1,
                            VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                            true);
                create_cube(specular_, SPECULAR_RESOLUTION, SPECULAR_MIPS,
                            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, false);
                create_cube(irradiance_, IRRADIANCE_RESOLUTION, 1,
                            VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT, false);
                create_brdf_lut();

                // The diffuse SH coefficients: a tiny device-local storage buffer the
                // projection compute writes and the shading pass reads. One buffer, not one
                // per slot, exactly like the cubes — the capture is rate-limited and change
                // gated, so it is not rewritten while a previous frame still reads it.
                {
                    VkBufferCreateInfo sh_info{};
                    sh_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    sh_info.size = sh_buffer_bytes();
                    sh_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                    VmaAllocationCreateInfo sh_alloc{};
                    sh_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &sh_info, &sh_alloc,
                                                  &sh_buffer_, &sh_allocation_, nullptr),
                                  "vmaCreateBuffer(ibl sh)");
                }

                // One uniform block per face: the frame's scene block with the camera
                // basis swapped for that face's 90-degree view.
                for (std::uint32_t face = 0; face < 6; ++face)
                {
                    VkBufferCreateInfo buffer_info{};
                    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    buffer_info.size = sizeof(Scene::SceneUniforms);
                    buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                                  &face_uniforms_[face],
                                                  &face_uniform_allocations_[face], &mapped),
                                  "vmaCreateBuffer(ibl face)");
                    face_uniform_mapped_[face] = mapped.pMappedData;
                }

                // Compute layout: sample the source cube, write the destination cube.
                VkDescriptorSetLayoutBinding compute_bindings[2]{};
                compute_bindings[0].binding = 0;
                compute_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                compute_bindings[0].descriptorCount = 1;
                compute_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                compute_bindings[1].binding = 1;
                compute_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                compute_bindings[1].descriptorCount = 1;
                compute_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo compute_layout_info{};
                compute_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                compute_layout_info.bindingCount = 2;
                compute_layout_info.pBindings = compute_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &compute_layout_info,
                                                          nullptr, &compute_layout_),
                              "vkCreateDescriptorSetLayout(ibl)");

                VkPushConstantRange convolve_range{};
                convolve_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                convolve_range.size = sizeof(ConvolveParams);
                VkPipelineLayoutCreateInfo compute_pipeline_info{};
                compute_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                compute_pipeline_info.setLayoutCount = 1;
                compute_pipeline_info.pSetLayouts = &compute_layout_;
                compute_pipeline_info.pushConstantRangeCount = 1;
                compute_pipeline_info.pPushConstantRanges = &convolve_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &compute_pipeline_info,
                                                     nullptr, &compute_pipeline_layout_),
                              "vkCreatePipelineLayout(ibl)");

                VkDescriptorSetLayoutBinding lut_binding{};
                lut_binding.binding = 0;
                lut_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                lut_binding.descriptorCount = 1;
                lut_binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                VkDescriptorSetLayoutCreateInfo lut_layout_info{};
                lut_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                lut_layout_info.bindingCount = 1;
                lut_layout_info.pBindings = &lut_binding;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &lut_layout_info,
                                                          nullptr, &lut_layout_),
                              "vkCreateDescriptorSetLayout(brdf)");

                VkPushConstantRange lut_range{};
                lut_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                lut_range.size = sizeof(LutParams);
                VkPipelineLayoutCreateInfo lut_pipeline_info{};
                lut_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                lut_pipeline_info.setLayoutCount = 1;
                lut_pipeline_info.pSetLayouts = &lut_layout_;
                lut_pipeline_info.pushConstantRangeCount = 1;
                lut_pipeline_info.pPushConstantRanges = &lut_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &lut_pipeline_info, nullptr,
                                                     &lut_pipeline_layout_),
                              "vkCreatePipelineLayout(brdf)");

                // SH projection layout: sample the source cube (binding 0), reduce into the
                // coefficient storage buffer (binding 1). A separate layout from the cube
                // convolutions because the destination is a buffer, not a storage image.
                VkDescriptorSetLayoutBinding sh_bindings[2]{};
                sh_bindings[0].binding = 0;
                sh_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                sh_bindings[0].descriptorCount = 1;
                sh_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                sh_bindings[1].binding = 1;
                sh_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                sh_bindings[1].descriptorCount = 1;
                sh_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                VkDescriptorSetLayoutCreateInfo sh_layout_info{};
                sh_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                sh_layout_info.bindingCount = 2;
                sh_layout_info.pBindings = sh_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &sh_layout_info, nullptr,
                                                          &sh_layout_),
                              "vkCreateDescriptorSetLayout(ibl sh)");

                VkPushConstantRange sh_range{};
                sh_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                sh_range.size = sizeof(ShParams);
                VkPipelineLayoutCreateInfo sh_pipeline_info{};
                sh_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                sh_pipeline_info.setLayoutCount = 1;
                sh_pipeline_info.pSetLayouts = &sh_layout_;
                sh_pipeline_info.pushConstantRangeCount = 1;
                sh_pipeline_info.pPushConstantRanges = &sh_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &sh_pipeline_info, nullptr,
                                                     &sh_pipeline_layout_),
                              "vkCreatePipelineLayout(ibl sh)");

                create_pipelines();
                generate_brdf_lut();
            }

            IblPass::~IblPass()
            {
                destroy_pipelines();
                for (std::uint32_t face = 0; face < 6; ++face)
                    if (face_uniforms_[face] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), face_uniforms_[face],
                                         face_uniform_allocations_[face]);
                if (lut_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), lut_pipeline_layout_, nullptr);
                if (lut_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), lut_layout_, nullptr);
                if (compute_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), compute_pipeline_layout_, nullptr);
                if (compute_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), compute_layout_, nullptr);
                if (sh_buffer_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), sh_buffer_, sh_allocation_);
                if (sh_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), sh_pipeline_layout_, nullptr);
                if (sh_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), sh_layout_, nullptr);
                if (dummy_depth_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), dummy_depth_view_, nullptr);
                if (dummy_depth_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), dummy_depth_,
                                    dummy_depth_allocation_);
                if (brdf_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), brdf_view_, nullptr);
                if (brdf_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), brdf_, brdf_allocation_);
                destroy_cube(irradiance_);
                destroy_cube(specular_);
                destroy_cube(environment_);
            }

            void IblPass::create_cube(Cube& cube, std::uint32_t resolution, std::uint32_t mips,
                                      VkImageUsageFlags usage, bool face_views)
            {
                cube.resolution = resolution;
                cube.mips = mips;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.flags = VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = Frame::HDR_FORMAT;
                image_info.extent = {resolution, resolution, 1};
                image_info.mipLevels = mips;
                image_info.arrayLayers = 6;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = usage;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &cube.image,
                                             &cube.allocation, nullptr),
                              "vmaCreateImage(ibl cube)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = cube.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
                view_info.format = Frame::HDR_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = mips;
                view_info.subresourceRange.layerCount = 6;
                Vulkan::check(
                    vkCreateImageView(device_.device(), &view_info, nullptr, &cube.cube_view),
                    "vkCreateImageView(ibl cube)");

                // Storage images cannot be written through a multi-mip view, so each mip
                // gets its own whole-cube view for the compute passes to bind.
                if ((usage & VK_IMAGE_USAGE_STORAGE_BIT) != 0)
                    for (std::uint32_t level = 0; level < mips; ++level)
                    {
                        VkImageViewCreateInfo mip_info = view_info;
                        mip_info.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
                        mip_info.subresourceRange.baseMipLevel = level;
                        mip_info.subresourceRange.levelCount = 1;
                        Vulkan::check(vkCreateImageView(device_.device(), &mip_info, nullptr,
                                                        &cube.mip_views[level]),
                                      "vkCreateImageView(ibl mip)");
                    }

                if (!face_views)
                    return;
                for (std::uint32_t face = 0; face < 6; ++face)
                {
                    VkImageViewCreateInfo face_info = view_info;
                    face_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                    face_info.subresourceRange.baseMipLevel = 0;
                    face_info.subresourceRange.levelCount = 1;
                    face_info.subresourceRange.baseArrayLayer = face;
                    face_info.subresourceRange.layerCount = 1;
                    Vulkan::check(vkCreateImageView(device_.device(), &face_info, nullptr,
                                                    &cube.face_views[face]),
                                  "vkCreateImageView(ibl face)");
                }
            }

            void IblPass::destroy_cube(Cube& cube)
            {
                for (VkImageView& view : cube.face_views)
                    if (view != VK_NULL_HANDLE)
                    {
                        vkDestroyImageView(device_.device(), view, nullptr);
                        view = VK_NULL_HANDLE;
                    }
                for (VkImageView& view : cube.mip_views)
                    if (view != VK_NULL_HANDLE)
                    {
                        vkDestroyImageView(device_.device(), view, nullptr);
                        view = VK_NULL_HANDLE;
                    }
                if (cube.cube_view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), cube.cube_view, nullptr);
                if (cube.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), cube.image, cube.allocation);
                cube.cube_view = VK_NULL_HANDLE;
                cube.image = VK_NULL_HANDLE;
            }

            void IblPass::create_brdf_lut()
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = VK_FORMAT_R16G16_SFLOAT;
                image_info.extent = {BRDF_RESOLUTION, BRDF_RESOLUTION, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &brdf_,
                                             &brdf_allocation_, nullptr),
                              "vmaCreateImage(brdf lut)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = brdf_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = VK_FORMAT_R16G16_SFLOAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &brdf_view_),
                              "vkCreateImageView(brdf lut)");

                // A 1x1 stand-in for the depth and colour the sky shader expects from the
                // scene: cleared to zero, which reverse-Z reads as "nothing here but sky".
                VkImageCreateInfo dummy_info{};
                dummy_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                dummy_info.imageType = VK_IMAGE_TYPE_2D;
                dummy_info.format = VK_FORMAT_R32_SFLOAT;
                dummy_info.extent = {1, 1, 1};
                dummy_info.mipLevels = 1;
                dummy_info.arrayLayers = 1;
                dummy_info.samples = VK_SAMPLE_COUNT_1_BIT;
                dummy_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                dummy_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                Vulkan::check(vmaCreateImage(device_.allocator(), &dummy_info, &alloc,
                                             &dummy_depth_, &dummy_depth_allocation_, nullptr),
                              "vmaCreateImage(ibl dummy)");

                VkImageViewCreateInfo dummy_view_info = view_info;
                dummy_view_info.image = dummy_depth_;
                dummy_view_info.format = VK_FORMAT_R32_SFLOAT;
                Vulkan::check(vkCreateImageView(device_.device(), &dummy_view_info, nullptr,
                                                &dummy_depth_view_),
                              "vkCreateImageView(ibl dummy)");
            }

            void IblPass::create_pipelines()
            {
                sky_pipeline_ = pipelines_.create(fullscreen_pipeline_desc(
                    layout_.pipeline_layout(), shaders_.module("fullscreen.vert"),
                    shaders_.module("sky.frag"), Frame::HDR_FORMAT));
                prefilter_pipeline_ = pipelines_.create_compute(
                    compute_pipeline_layout_, shaders_.module("ibl_prefilter.comp"));
                irradiance_pipeline_ = pipelines_.create_compute(
                    compute_pipeline_layout_, shaders_.module("ibl_irradiance.comp"));
                sh_pipeline_ = pipelines_.create_compute(
                    sh_pipeline_layout_, shaders_.module("sh_project.comp"));
            }

            void IblPass::destroy_pipelines()
            {
                // The compute pipelines are pass-owned and destroyed here; the sky pipeline
                // is a graphics handle the factory owns (it swaps in the optimized rebuild),
                // so the pass only drops that handle.
                for (VkPipeline* pipeline :
                     {&sh_pipeline_, &irradiance_pipeline_, &prefilter_pipeline_})
                {
                    if (*pipeline != VK_NULL_HANDLE)
                        vkDestroyPipeline(device_.device(), *pipeline, nullptr);
                    *pipeline = VK_NULL_HANDLE;
                }
                sky_pipeline_ = Resources::PipelineHandle{};
            }

            void IblPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
                // The environment depends on the sky shader, so a reloaded sky must be
                // recaptured rather than left showing the previous compile's result.
                captured_ = false;
            }

            void IblPass::generate_brdf_lut()
            {
                const VkPipeline pipeline = pipelines_.create_compute(
                    lut_pipeline_layout_, shaders_.module("brdf_lut.comp"));

                VkDescriptorPoolSize size{};
                size.type = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                size.descriptorCount = 1;
                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.maxSets = 1;
                pool_info.poolSizeCount = 1;
                pool_info.pPoolSizes = &size;
                VkDescriptorPool pool = VK_NULL_HANDLE;
                Vulkan::check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &pool),
                              "vkCreateDescriptorPool(brdf)");

                VkDescriptorSetAllocateInfo set_info{};
                set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                set_info.descriptorPool = pool;
                set_info.descriptorSetCount = 1;
                set_info.pSetLayouts = &lut_layout_;
                VkDescriptorSet set = VK_NULL_HANDLE;
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &set_info, &set),
                              "vkAllocateDescriptorSets(brdf)");

                Resources::DescriptorWriter writer;
                writer.storage_image(0, brdf_view_);
                writer.update(device_.device(), set);

                VkCommandPoolCreateInfo command_pool_info{};
                command_pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                command_pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                command_pool_info.queueFamilyIndex = device_.graphics_queue_family();
                VkCommandPool command_pool = VK_NULL_HANDLE;
                Vulkan::check(vkCreateCommandPool(device_.device(), &command_pool_info, nullptr,
                                                  &command_pool),
                              "vkCreateCommandPool(brdf)");

                VkCommandBufferAllocateInfo cmd_info{};
                cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                cmd_info.commandPool = command_pool;
                cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                cmd_info.commandBufferCount = 1;
                VkCommandBuffer cmd = VK_NULL_HANDLE;
                Vulkan::check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &cmd),
                              "vkAllocateCommandBuffers(brdf)");

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(cmd, &begin), "vkBeginCommandBuffer(brdf)");

                transition(cmd, brdf_, 0, 1, 1, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, 0,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               lut_pipeline_layout_, 0, set);
                LutParams params{BRDF_RESOLUTION, BRDF_SAMPLES};
                vkCmdPushConstants(cmd, lut_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                   sizeof(params), &params);
                vkCmdDispatch(cmd, groups(BRDF_RESOLUTION, 8), groups(BRDF_RESOLUTION, 8), 1);

                transition(cmd, brdf_, 0, 1, 1, VK_IMAGE_LAYOUT_GENERAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                transition(cmd, dummy_depth_, 0, 1, 1, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, VK_PIPELINE_STAGE_2_CLEAR_BIT, 0,
                           VK_ACCESS_2_TRANSFER_WRITE_BIT);
                VkClearColorValue clear{};
                VkImageSubresourceRange range{};
                range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                range.levelCount = 1;
                range.layerCount = 1;
                vkCmdClearColorImage(cmd, dummy_depth_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                     &clear, 1, &range);
                transition(cmd, dummy_depth_, 0, 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                Vulkan::check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(brdf)");

                VkFenceCreateInfo fence_info{};
                fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                VkFence fence = VK_NULL_HANDLE;
                Vulkan::check(vkCreateFence(device_.device(), &fence_info, nullptr, &fence),
                              "vkCreateFence(brdf)");

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = cmd;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                Vulkan::check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, fence),
                              "vkQueueSubmit2(brdf)");
                Vulkan::check(vkWaitForFences(device_.device(), 1, &fence, VK_TRUE, UINT64_MAX),
                              "vkWaitForFences(brdf)");

                vkDestroyFence(device_.device(), fence, nullptr);
                vkDestroyCommandPool(device_.device(), command_pool, nullptr);
                vkDestroyDescriptorPool(device_.device(), pool, nullptr);
                vkDestroyPipeline(device_.device(), pipeline, nullptr);
            }

            bool IblPass::environment_changed(const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr)
                    return false;
                if (!captured_)
                    return true;
                // Rebuilding is expensive relative to what a fractional sun movement
                // changes, so it is rate-limited and gated on a measurable difference.
                if (frame.index - last_capture_frame_ < RECAPTURE_INTERVAL)
                    return false;

                const Vector3 sun = normalize(frame.environment->sun.direction);
                const float dot_product = last_sun_[0] * static_cast<float>(sun.x) +
                                          last_sun_[1] * static_cast<float>(sun.y) +
                                          last_sun_[2] * static_cast<float>(sun.z);
                if (dot_product < 0.99999f)
                    return true;
                if (std::fabs(last_intensity_ - frame.environment->sun.intensity) > 1e-3f)
                    return true;
                const float altitude = static_cast<float>(frame.eye[1]);
                return std::fabs(last_altitude_ - altitude) > 500.0f;
            }

            void IblPass::register_pass(Graph::RenderGraph& graph,
                                        const Frame::FrameContext& frame)
            {
                if (!environment_changed(frame))
                    return;

                const Vector3 sun = normalize(frame.environment->sun.direction);
                last_sun_[0] = static_cast<float>(sun.x);
                last_sun_[1] = static_cast<float>(sun.y);
                last_sun_[2] = static_cast<float>(sun.z);
                last_intensity_ = frame.environment->sun.intensity;
                last_altitude_ = static_cast<float>(frame.eye[1]);
                last_capture_frame_ = frame.index;
                captured_ = true;

                graph.add_pass(
                    "ibl update",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        // The cube chain is this pass's own; what it needs from the graph
                        // is the frame's scene block, which it re-aims per face.
                        builder.read(frame.targets.uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    { record_update(cmd, frame, context); });
            }

            void IblPass::record_update(VkCommandBuffer cmd, const Frame::FrameContext& frame,
                                        const Graph::PassContext& context)
            {
                const Scene::SceneUniforms* source = static_cast<const Scene::SceneUniforms*>(
                    context.mapped(frame.targets.uniforms));
                if (source == nullptr)
                    return;

                // Six 90-degree views of the same sky: the face basis replaces the camera
                // basis, and a tangent of 1 makes the frustum exactly a quadrant.
                for (std::uint32_t face = 0; face < 6; ++face)
                {
                    Scene::SceneUniforms uniforms = *source;
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        uniforms.cam_forward[axis] = FACES[face].forward[axis];
                        uniforms.cam_right[axis] = FACES[face].right[axis];
                        uniforms.cam_up[axis] = FACES[face].up[axis];
                    }
                    // The sky-view LUT is built for the main camera's frame, so the capture
                    // marches the atmosphere per pixel instead of reading it (see sky.frag).
                    uniforms.ibl_params[3] = 0.0f;
                    std::memcpy(face_uniform_mapped_[face], &uniforms, sizeof(uniforms));
                }

                transition(cmd, environment_.image, 0, environment_.mips, 6,
                           VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, 0,
                           VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);

                VkViewport viewport{};
                viewport.width = static_cast<float>(environment_.resolution);
                viewport.height = static_cast<float>(environment_.resolution);
                viewport.maxDepth = 1.0f;
                VkRect2D scissor{};
                scissor.extent = {environment_.resolution, environment_.resolution};

                for (std::uint32_t face = 0; face < 6; ++face)
                {
                    VkRenderingAttachmentInfo attachment{};
                    attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                    attachment.imageView = environment_.face_views[face];
                    attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                    attachment.loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                    attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;

                    VkRenderingInfo rendering{};
                    rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                    rendering.renderArea = scissor;
                    rendering.layerCount = 1;
                    rendering.colorAttachmentCount = 1;
                    rendering.pColorAttachments = &attachment;

                    Scene::SceneSetWriter writer;
                    writer.uniform(Scene::SceneLayout::SCENE_BINDING, face_uniforms_[face],
                                   sizeof(Scene::SceneUniforms));
                    writer.image(1, dummy_depth_view_, sampler_);
                    writer.image(2, dummy_depth_view_, sampler_);
                    writer.image(3, noise_.shape(), noise_.sampler());
                    writer.image(4, noise_.detail(), noise_.sampler());
                    writer.image(5, noise_.weather(), noise_.sampler());
                    writer.image(6, noise_.cirrus(), noise_.sampler());
                    // The captured sky shader reads the atmosphere LUTs like the real sky
                    // pass does, so the prefiltered environment tracks the same scattering.
                    writer.image(Scene::SceneLayout::TRANSMITTANCE_LUT_BINDING,
                                 atmosphere_.transmittance_view(), sampler_);
                    writer.image(Scene::SceneLayout::MULTISCATTER_LUT_BINDING,
                                 atmosphere_.multiscatter_view(), sampler_);
                    // Bound so the descriptor is valid, but the capture keeps the per-pixel
                    // march (ibl_params.w is cleared in the face uniforms below): the
                    // sky-view LUT is built for the main camera, not these cube viewpoints.
                    writer.image(Scene::SceneLayout::SKY_VIEW_LUT_BINDING,
                                 atmosphere_.sky_view_view(), sampler_);
                    // Bound for a valid descriptor; the capture has no geometry, so the
                    // aerial volume (a geometry-only term) is never sampled.
                    writer.image(Scene::SceneLayout::AERIAL_LUT_BINDING,
                                 atmosphere_.aerial_view(), sampler_);
                    // Bound for a valid descriptor; the fog composite is gated off in the
                    // capture (ibl_params.w cleared below), so the volume is never sampled.
                    writer.image(Scene::SceneLayout::FOG_LUT_BINDING, fog_.fog_view(), sampler_);

                    vkCmdBeginRendering(cmd, &rendering);
                    vkCmdSetViewport(cmd, 0, 1, &viewport);
                    vkCmdSetScissor(cmd, 0, 1, &scissor);
                    layout_.bind_heap(cmd);
                    writer.commit(cmd, layout_.pipeline_layout());
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, sky_pipeline_.get());
                    vkCmdDraw(cmd, 3, 1, 0, 0);
                    vkCmdEndRendering(cmd);
                }

                // Mip chain on the environment: the prefilter picks a mip from each
                // sample's solid angle, which is what keeps the sun from aliasing into
                // fireflies across the rough mips.
                transition(cmd, environment_.image, 0, 1, 6,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                           VK_ACCESS_2_TRANSFER_READ_BIT);
                transition(cmd, environment_.image, 1, environment_.mips - 1, 6,
                           VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL,
                           VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT,
                           VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT,
                           VK_ACCESS_2_TRANSFER_WRITE_BIT);

                std::uint32_t source_size = environment_.resolution;
                for (std::uint32_t level = 1; level < environment_.mips; ++level)
                {
                    const std::uint32_t target_size = std::max(1u, source_size / 2);
                    VkImageBlit blit{};
                    blit.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    blit.srcSubresource.mipLevel = level - 1;
                    blit.srcSubresource.layerCount = 6;
                    blit.srcOffsets[1] = {static_cast<std::int32_t>(source_size),
                                          static_cast<std::int32_t>(source_size), 1};
                    blit.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    blit.dstSubresource.mipLevel = level;
                    blit.dstSubresource.layerCount = 6;
                    blit.dstOffsets[1] = {static_cast<std::int32_t>(target_size),
                                          static_cast<std::int32_t>(target_size), 1};
                    vkCmdBlitImage(cmd, environment_.image, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                   environment_.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1,
                                   &blit, VK_FILTER_LINEAR);
                    transition(cmd, environment_.image, level, 1, 6,
                               VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                               VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT,
                               VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT,
                               VK_ACCESS_2_TRANSFER_READ_BIT);
                    source_size = target_size;
                }

                transition(cmd, environment_.image, 0, environment_.mips, 6,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_TRANSFER_READ_BIT,
                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                transition(cmd, specular_.image, 0, specular_.mips, 6, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, 0,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                transition(cmd, irradiance_.image, 0, 1, 6, VK_IMAGE_LAYOUT_UNDEFINED,
                           VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, 0,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                const auto convolve = [&](VkPipeline pipeline, VkImageView destination,
                                          const ConvolveParams& params, std::uint32_t resolution)
                {
                    const VkDescriptorSet set = frame.descriptors->allocate(compute_layout_);
                    Resources::DescriptorWriter writer;
                    writer.sampled_image(0, environment_.cube_view, sampler_);
                    writer.storage_image(1, destination);
                    writer.update(device_.device(), set);

                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline);
                    Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   compute_pipeline_layout_, 0, set);
                    vkCmdPushConstants(cmd, compute_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                                       0, sizeof(ConvolveParams), &params);
                    vkCmdDispatch(cmd, groups(resolution, 8), groups(resolution, 8), 6);
                };

                for (std::uint32_t level = 0; level < specular_.mips; ++level)
                {
                    const std::uint32_t resolution =
                        std::max(1u, specular_.resolution >> level);
                    ConvolveParams params{resolution, environment_.resolution,
                                          static_cast<float>(level) /
                                              static_cast<float>(specular_.mips - 1),
                                          PREFILTER_SAMPLES};
                    convolve(prefilter_pipeline_, specular_.mip_views[level], params, resolution);
                }

                // The irradiance integral is smooth, so it reads a coarse mip of the
                // environment and still converges at this sample count.
                ConvolveParams irradiance_params{irradiance_.resolution,
                                                 environment_.resolution, 3.0f,
                                                 IRRADIANCE_SAMPLES};
                convolve(irradiance_pipeline_, irradiance_.mip_views[0], irradiance_params,
                         irradiance_.resolution);

                // Project the environment radiance into 9 diffuse SH coefficients — the
                // ambient the shading pass reads instead of sampling the irradiance cube.
                // Reuses the environment cube, already shader-readable for the convolutions
                // above, and reduces it in a single workgroup.
                {
                    const VkDescriptorSet sh_set = frame.descriptors->allocate(sh_layout_);
                    Resources::DescriptorWriter writer;
                    writer.sampled_image(0, environment_.cube_view, sampler_);
                    writer.storage_buffer(1, sh_buffer_, sh_buffer_bytes());
                    writer.update(device_.device(), sh_set);

                    const ShParams sh_params{SH_SAMPLE_RESOLUTION, 2.0f};
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, sh_pipeline_);
                    Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   sh_pipeline_layout_, 0, sh_set);
                    vkCmdPushConstants(cmd, sh_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                       sizeof(ShParams), &sh_params);
                    vkCmdDispatch(cmd, 1, 1, 1);
                }

                transition(cmd, specular_.image, 0, specular_.mips, 6, VK_IMAGE_LAYOUT_GENERAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                transition(cmd, irradiance_.image, 0, 1, 6, VK_IMAGE_LAYOUT_GENERAL,
                           VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                // Make the SH coefficients visible to the shading pass's fragment reads. The
                // buffer is a private resource — not graph-tracked, like the cubes — so this
                // barrier is recorded here rather than derived by the render graph.
                VkBufferMemoryBarrier2 sh_barrier{};
                sh_barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
                sh_barrier.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                sh_barrier.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                sh_barrier.dstStageMask = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                sh_barrier.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
                sh_barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                sh_barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                sh_barrier.buffer = sh_buffer_;
                sh_barrier.offset = 0;
                sh_barrier.size = VK_WHOLE_SIZE;
                VkDependencyInfo sh_dependency{};
                sh_dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                sh_dependency.bufferMemoryBarrierCount = 1;
                sh_dependency.pBufferMemoryBarriers = &sh_barrier;
                vkCmdPipelineBarrier2(cmd, &sh_dependency);
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
