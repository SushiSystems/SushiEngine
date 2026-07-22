/**************************************************************************/
/* atmosphere_lut_pass.cpp                                                */
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

#include "passes/atmosphere_lut_pass.hpp"

#include <cmath>
#include <cstring>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
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
                constexpr VkFormat LUT_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr std::uint32_t GROUP_SIZE = 8;
                constexpr std::uint32_t TRANSMITTANCE_WIDTH = 256;
                constexpr std::uint32_t TRANSMITTANCE_HEIGHT = 64;
                constexpr std::uint32_t MULTISCATTER_SIZE = 32;
                constexpr std::uint32_t SKY_VIEW_WIDTH = 192;
                constexpr std::uint32_t SKY_VIEW_HEIGHT = 108;
                constexpr std::uint32_t AERIAL_SIZE = 32;

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

            AtmosphereLutPass::AtmosphereLutPass(Vulkan::VulkanDevice& device,
                                                 Resources::ShaderLibrary& shaders,
                                                 Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                // Transmittance build: one storage-image output, no inputs.
                VkDescriptorSetLayoutBinding t_binding{};
                t_binding.binding = 0;
                t_binding.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                t_binding.descriptorCount = 1;
                t_binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo t_layout_info{};
                t_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                t_layout_info.bindingCount = 1;
                t_layout_info.pBindings = &t_binding;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &t_layout_info, nullptr,
                                                          &transmittance_layout_),
                              "vkCreateDescriptorSetLayout(atmosphere transmittance)");

                // Multiple-scattering build: storage-image output plus the transmittance LUT.
                VkDescriptorSetLayoutBinding m_bindings[2]{};
                m_bindings[0].binding = 0;
                m_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                m_bindings[0].descriptorCount = 1;
                m_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                m_bindings[1].binding = 1;
                m_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                m_bindings[1].descriptorCount = 1;
                m_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo m_layout_info{};
                m_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                m_layout_info.bindingCount = 2;
                m_layout_info.pBindings = m_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &m_layout_info, nullptr,
                                                          &multiscatter_layout_),
                              "vkCreateDescriptorSetLayout(atmosphere multiscatter)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo t_pipeline_info{};
                t_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                t_pipeline_info.setLayoutCount = 1;
                t_pipeline_info.pSetLayouts = &transmittance_layout_;
                t_pipeline_info.pushConstantRangeCount = 1;
                t_pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &t_pipeline_info, nullptr,
                                                     &transmittance_pipeline_layout_),
                              "vkCreatePipelineLayout(atmosphere transmittance)");

                VkPipelineLayoutCreateInfo m_pipeline_info = t_pipeline_info;
                m_pipeline_info.pSetLayouts = &multiscatter_layout_;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &m_pipeline_info, nullptr,
                                                     &multiscatter_pipeline_layout_),
                              "vkCreatePipelineLayout(atmosphere multiscatter)");

                // Sky-view build: storage-image output plus both static LUTs as inputs.
                VkDescriptorSetLayoutBinding s_bindings[3]{};
                s_bindings[0].binding = 0;
                s_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                s_bindings[0].descriptorCount = 1;
                s_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                s_bindings[1].binding = 1;
                s_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                s_bindings[1].descriptorCount = 1;
                s_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                s_bindings[2].binding = 2;
                s_bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                s_bindings[2].descriptorCount = 1;
                s_bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo s_layout_info{};
                s_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                s_layout_info.bindingCount = 3;
                s_layout_info.pBindings = s_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &s_layout_info, nullptr,
                                                          &sky_view_layout_),
                              "vkCreateDescriptorSetLayout(atmosphere sky-view)");

                VkPushConstantRange sky_view_range{};
                sky_view_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                sky_view_range.size = sizeof(SkyViewPush);

                VkPipelineLayoutCreateInfo s_pipeline_info{};
                s_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                s_pipeline_info.setLayoutCount = 1;
                s_pipeline_info.pSetLayouts = &sky_view_layout_;
                s_pipeline_info.pushConstantRangeCount = 1;
                s_pipeline_info.pPushConstantRanges = &sky_view_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &s_pipeline_info, nullptr,
                                                     &sky_view_pipeline_layout_),
                              "vkCreatePipelineLayout(atmosphere sky-view)");

                // Aerial-perspective build: the froxel volume output, the scene uniform
                // block (for the camera basis and the atmosphere, read straight so it
                // matches sky.frag), and both static LUTs.
                VkDescriptorSetLayoutBinding a_bindings[4]{};
                a_bindings[0].binding = 0;
                a_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                a_bindings[0].descriptorCount = 1;
                a_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                a_bindings[1].binding = 1;
                a_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                a_bindings[1].descriptorCount = 1;
                a_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                a_bindings[2].binding = 2;
                a_bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                a_bindings[2].descriptorCount = 1;
                a_bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                a_bindings[3].binding = 3;
                a_bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                a_bindings[3].descriptorCount = 1;
                a_bindings[3].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo a_layout_info{};
                a_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                a_layout_info.bindingCount = 4;
                a_layout_info.pBindings = a_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &a_layout_info, nullptr,
                                                          &aerial_layout_),
                              "vkCreateDescriptorSetLayout(atmosphere aerial)");

                VkPipelineLayoutCreateInfo a_pipeline_info{};
                a_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                a_pipeline_info.setLayoutCount = 1;
                a_pipeline_info.pSetLayouts = &aerial_layout_;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &a_pipeline_info, nullptr,
                                                     &aerial_pipeline_layout_),
                              "vkCreatePipelineLayout(atmosphere aerial)");

                create_image(transmittance_, TRANSMITTANCE_WIDTH, TRANSMITTANCE_HEIGHT);
                create_image(multiscatter_, MULTISCATTER_SIZE, MULTISCATTER_SIZE);
                create_image(sky_view_, SKY_VIEW_WIDTH, SKY_VIEW_HEIGHT);
                create_volume(aerial_, AERIAL_SIZE, AERIAL_SIZE, AERIAL_SIZE);
                create_pipelines();
            }

            AtmosphereLutPass::~AtmosphereLutPass()
            {
                destroy_pipelines();
                destroy_image(transmittance_);
                destroy_image(multiscatter_);
                destroy_image(sky_view_);
                destroy_image(aerial_);
                if (transmittance_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), transmittance_pipeline_layout_,
                                            nullptr);
                if (multiscatter_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), multiscatter_pipeline_layout_,
                                            nullptr);
                if (sky_view_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), sky_view_pipeline_layout_, nullptr);
                if (aerial_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), aerial_pipeline_layout_, nullptr);
                if (transmittance_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), transmittance_layout_, nullptr);
                if (multiscatter_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), multiscatter_layout_, nullptr);
                if (sky_view_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), sky_view_layout_, nullptr);
                if (aerial_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), aerial_layout_, nullptr);
            }

            void AtmosphereLutPass::create_pipelines()
            {
                transmittance_pipeline_ = pipelines_.create_compute(
                    transmittance_pipeline_layout_, shaders_.module("transmittance_lut.comp"));
                multiscatter_pipeline_ = pipelines_.create_compute(
                    multiscatter_pipeline_layout_, shaders_.module("multiscatter_lut.comp"));
                sky_view_pipeline_ = pipelines_.create_compute(
                    sky_view_pipeline_layout_, shaders_.module("sky_view_lut.comp"));
                aerial_pipeline_ = pipelines_.create_compute(
                    aerial_pipeline_layout_, shaders_.module("aerial_perspective.comp"));
            }

            void AtmosphereLutPass::destroy_pipelines()
            {
                if (transmittance_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), transmittance_pipeline_, nullptr);
                if (multiscatter_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), multiscatter_pipeline_, nullptr);
                if (sky_view_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), sky_view_pipeline_, nullptr);
                if (aerial_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), aerial_pipeline_, nullptr);
                transmittance_pipeline_ = VK_NULL_HANDLE;
                multiscatter_pipeline_ = VK_NULL_HANDLE;
                sky_view_pipeline_ = VK_NULL_HANDLE;
                aerial_pipeline_ = VK_NULL_HANDLE;
            }

            void AtmosphereLutPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
                // A shader edit can change the LUT contents, so force a rebuild next frame.
                built_ = false;
            }

            void AtmosphereLutPass::create_image(Lut& lut, std::uint32_t width,
                                                 std::uint32_t height)
            {
                lut.width = width;
                lut.height = height;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = LUT_FORMAT;
                image_info.extent = {width, height, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &lut.image,
                                             &lut.allocation, nullptr),
                              "vmaCreateImage(atmosphere lut)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = lut.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = LUT_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &lut.view),
                              "vkCreateImageView(atmosphere lut)");
            }

            void AtmosphereLutPass::create_volume(Lut& lut, std::uint32_t width,
                                                  std::uint32_t height, std::uint32_t depth)
            {
                lut.width = width;
                lut.height = height;
                lut.depth = depth;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = LUT_FORMAT;
                image_info.extent = {width, height, depth};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &lut.image,
                                             &lut.allocation, nullptr),
                              "vmaCreateImage(atmosphere volume)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = lut.image;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = LUT_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &lut.view),
                              "vkCreateImageView(atmosphere volume)");
            }

            void AtmosphereLutPass::destroy_image(Lut& lut)
            {
                if (lut.view != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), lut.view, nullptr);
                if (lut.image != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), lut.image, lut.allocation);
                lut.view = VK_NULL_HANDLE;
                lut.image = VK_NULL_HANDLE;
                lut.allocation = VK_NULL_HANDLE;
            }

            bool AtmosphereLutPass::medium_changed(const Push& push)
            {
                if (!built_ || std::memcmp(&push, &last_push_, sizeof(Push)) != 0)
                {
                    last_push_ = push;
                    return true;
                }
                return false;
            }

            void AtmosphereLutPass::register_pass(Graph::RenderGraph& graph,
                                                  const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr)
                    return;
                const Environment& environment = *frame.environment;

                Push push{};
                push.rayleigh[0] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.x);
                push.rayleigh[1] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.y);
                push.rayleigh[2] =
                    static_cast<float>(environment.atmosphere.rayleigh_coefficient.z);
                push.rayleigh[3] = environment.atmosphere.mie_coefficient;
                push.heights[0] = environment.atmosphere.rayleigh_scale_height;
                push.heights[1] = environment.atmosphere.mie_scale_height;
                const float bottom =
                    static_cast<float>(environment.planet_surface_reference_metres);
                push.heights[2] = bottom;
                push.heights[3] = bottom + environment.atmosphere.height;
                // Mie extinction is the Mie scattering plus its absorption, the same 1.1
                // factor sky.frag applies in its own view-ray optical depth.
                push.extra[0] = environment.atmosphere.mie_coefficient * 1.1f;

                // The sky-view LUT additionally needs the sun and the planet centre in
                // camera-relative space, and the Mie anisotropy for its phase function.
                SkyViewPush sky_view{};
                std::memcpy(sky_view.rayleigh, push.rayleigh, sizeof(push.rayleigh));
                std::memcpy(sky_view.heights, push.heights, sizeof(push.heights));
                sky_view.extra[0] = push.extra[0];
                sky_view.extra[1] = environment.atmosphere.mie_anisotropy;
                sky_view.center[0] =
                    static_cast<float>(environment.planet_center.x - frame.eye[0]);
                sky_view.center[1] =
                    static_cast<float>(environment.planet_center.y - frame.eye[1]);
                sky_view.center[2] =
                    static_cast<float>(environment.planet_center.z - frame.eye[2]);
                const double sx = environment.sun.direction.x;
                const double sy = environment.sun.direction.y;
                const double sz = environment.sun.direction.z;
                const double sun_length = std::sqrt(sx * sx + sy * sy + sz * sz);
                const double inv = sun_length > 0.0 ? 1.0 / sun_length : 0.0;
                sky_view.sun[0] = static_cast<float>(sx * inv);
                sky_view.sun[1] = static_cast<float>(sy * inv);
                sky_view.sun[2] = static_cast<float>(sz * inv);

                // The transmittance and multiple-scattering LUTs are view-independent, so
                // they rebuild only when the medium or the radii move; the sky-view LUT
                // depends on the camera altitude and the sun, so it rebuilds every frame off
                // whichever static LUTs are current.
                const bool build_static = medium_changed(push);
                built_ = true;

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "atmosphere-lut",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The LUT images are pass-owned and barriered by hand below; the one
                        // graph resource is the scene uniform block the aerial build reads,
                        // declared so the graph barriers the host upload to a compute read.
                        // A side effect keeps the pass from being culled.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, push, sky_view, build_static, uniforms](
                        VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler =
                            frame.samplers->get(Resources::SamplerDesc{});

                        if (build_static)
                        {
                            // Transmittance first: the multiple-scattering build samples it.
                            transition(cmd, transmittance_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                            {
                                const VkDescriptorSet set =
                                    frame.descriptors->allocate(transmittance_layout_);
                                Resources::DescriptorWriter writer;
                                writer.storage_image(0, transmittance_.view);
                                writer.update(device_.device(), set);
                                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                  transmittance_pipeline_);
                                Resources::bind_descriptor_set(
                                    cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                    transmittance_pipeline_layout_, 0, set);
                                vkCmdPushConstants(cmd, transmittance_pipeline_layout_,
                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push),
                                                   &push);
                                vkCmdDispatch(cmd, groups(transmittance_.width),
                                              groups(transmittance_.height), 1);
                            }
                            // Readable by the MS/sky-view builds and the fragment passes.
                            transition(cmd, transmittance_.image, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                            transition(cmd, multiscatter_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                            {
                                const VkDescriptorSet set =
                                    frame.descriptors->allocate(multiscatter_layout_);
                                Resources::DescriptorWriter writer;
                                writer.storage_image(0, multiscatter_.view);
                                writer.sampled_image(1, transmittance_.view, sampler);
                                writer.update(device_.device(), set);
                                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                  multiscatter_pipeline_);
                                Resources::bind_descriptor_set(
                                    cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                    multiscatter_pipeline_layout_, 0, set);
                                vkCmdPushConstants(cmd, multiscatter_pipeline_layout_,
                                                   VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push),
                                                   &push);
                                vkCmdDispatch(cmd, groups(multiscatter_.width),
                                              groups(multiscatter_.height), 1);
                            }
                            // Visible to the sky-view build (compute) and the sky/IBL passes.
                            transition(cmd, multiscatter_.image, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        }

                        // Sky-view LUT: every frame, marched off the current static LUTs.
                        transition(cmd, sky_view_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        {
                            const VkDescriptorSet set =
                                frame.descriptors->allocate(sky_view_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_image(0, sky_view_.view);
                            writer.sampled_image(1, transmittance_.view, sampler);
                            writer.sampled_image(2, multiscatter_.view, sampler);
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                              sky_view_pipeline_);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           sky_view_pipeline_layout_, 0, set);
                            vkCmdPushConstants(cmd, sky_view_pipeline_layout_,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(SkyViewPush),
                                               &sky_view);
                            vkCmdDispatch(cmd, groups(sky_view_.width), groups(sky_view_.height),
                                          1);
                        }
                        transition(cmd, sky_view_.image, VK_IMAGE_LAYOUT_GENERAL,
                                   VK_IMAGE_LAYOUT_GENERAL,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                        // Aerial-perspective froxel volume: every frame, from the scene
                        // uniform block and the current static LUTs.
                        transition(cmd, aerial_.image, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                                   VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                        {
                            const VkBuffer scene_buffer = context.buffer(uniforms);
                            const VkDescriptorSet set =
                                frame.descriptors->allocate(aerial_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_image(0, aerial_.view);
                            writer.uniform_buffer(1, scene_buffer,
                                                  sizeof(Scene::SceneUniforms));
                            writer.sampled_image(2, transmittance_.view, sampler);
                            writer.sampled_image(3, multiscatter_.view, sampler);
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                              aerial_pipeline_);
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                           aerial_pipeline_layout_, 0, set);
                            // One thread per (x, y) column; the shader loops the depth slices.
                            vkCmdDispatch(cmd, groups(aerial_.width), groups(aerial_.height), 1);
                        }
                        transition(cmd, aerial_.image, VK_IMAGE_LAYOUT_GENERAL,
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
