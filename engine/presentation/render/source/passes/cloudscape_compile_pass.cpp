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

#include <cmath>
#include <cstring>

#include <SushiEngine/environment/environment.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "atmosphere/atmosphere_nest.hpp"
#include "passes/weather_field_pass.hpp"
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

            CloudscapeCompilePass::CloudscapeCompilePass(Vulkan::VulkanDevice& device,
                                                         Resources::ShaderLibrary& shaders,
                                                         Resources::GraphicsPipelineFactory& pipelines,
                                                         Resources::SamplerCache& samplers,
                                                         Textures::CloudNoise& noise,
                                                         WeatherFieldPass& weather)
                : device_(device), shaders_(shaders), pipelines_(pipelines), noise_(noise),
                  weather_(weather)
            {
                // Field bake: one storage-image output, the scene uniform block (for the deck
                // stack and the weather field's addressing), the four noise volumes the deck loop
                // samples, the simulation's own field, and the static genus catalogue the derived
                // path resolves a column through.
                VkDescriptorSetLayoutBinding f_bindings[9]{};
                f_bindings[0].binding = 0;
                f_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                f_bindings[0].descriptorCount = 1;
                f_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                f_bindings[1].binding = 1;
                f_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                f_bindings[1].descriptorCount = 1;
                f_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                for (std::uint32_t i = 2; i < 7; ++i)
                {
                    f_bindings[i].binding = i;
                    f_bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    f_bindings[i].descriptorCount = 1;
                    f_bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }
                f_bindings[7].binding = 7;
                f_bindings[7].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                f_bindings[7].descriptorCount = 1;
                f_bindings[7].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                // The regional nest's extinction field. Bound unconditionally so the descriptor
                // set is one shape whether or not a nest exists; the bake reads it only when the
                // scene block says the nest is running.
                f_bindings[8].binding = 8;
                f_bindings[8].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                f_bindings[8].descriptorCount = 1;
                f_bindings[8].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo f_layout_info{};
                f_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                f_layout_info.bindingCount = 9;
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

                // Far sun-depth resolve: writes the published far window, reads the scratch
                // density bake. Two images rather than one read-modify-write, because the march
                // reads texels its neighbours are writing.
                VkDescriptorSetLayoutBinding l_bindings[3]{};
                l_bindings[0].binding = 0;
                l_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                l_bindings[0].descriptorCount = 1;
                l_bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                l_bindings[1].binding = 1;
                l_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                l_bindings[1].descriptorCount = 1;
                l_bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                l_bindings[2].binding = 2;
                l_bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                l_bindings[2].descriptorCount = 1;
                l_bindings[2].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo l_layout_info{};
                l_layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                l_layout_info.bindingCount = 3;
                l_layout_info.pBindings = l_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &l_layout_info, nullptr,
                                                          &far_light_layout_),
                              "vkCreateDescriptorSetLayout(cloudscape far light)");

                VkPushConstantRange l_range{};
                l_range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                l_range.size = sizeof(FarLightPush);

                VkPipelineLayoutCreateInfo l_pipeline_info{};
                l_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                l_pipeline_info.setLayoutCount = 1;
                l_pipeline_info.pSetLayouts = &far_light_layout_;
                l_pipeline_info.pushConstantRangeCount = 1;
                l_pipeline_info.pPushConstantRanges = &l_range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &l_pipeline_info, nullptr,
                                                     &far_light_pipeline_layout_),
                              "vkCreatePipelineLayout(cloudscape far light)");

                create_volume(near_, FIELD_RESOLUTION_XZ, FIELD_RESOLUTION_Y, FIELD_RESOLUTION_XZ);
                create_volume(skip_, FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ,
                             FIELD_RESOLUTION_Y / SKIP_DOWNSAMPLE_Y,
                             FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ);
                create_volume(far_source_, FIELD_RESOLUTION_XZ, FIELD_RESOLUTION_Y,
                              FIELD_RESOLUTION_XZ);
                create_volume(far_, FIELD_RESOLUTION_XZ, FIELD_RESOLUTION_Y, FIELD_RESOLUTION_XZ);

                // CLAMP_TO_EDGE, not REPEAT: the windows do not wrap any more (see
                // cloud_field_window.glsl). A lookup that leaves a window reads its nearest real
                // weather rather than folding back into an unrelated piece of sky a span away,
                // which is exactly the property the near/far cross-fade and the sun marches rely
                // on. Linear filtering smooths the block boundaries the bake's discrete texels
                // leave, as before.
                Resources::SamplerDescription sampler_description{};
                sampler_description.filter = VK_FILTER_LINEAR;
                sampler_description.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_ = samplers.get(sampler_description);

                create_catalogue();
                create_pipelines();
            }

            CloudscapeCompilePass::~CloudscapeCompilePass()
            {
                destroy_pipelines();
                destroy_catalogue();
                destroy_volume(near_);
                destroy_volume(skip_);
                destroy_volume(far_source_);
                destroy_volume(far_);
                if (field_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), field_pipeline_layout_, nullptr);
                if (skip_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), skip_pipeline_layout_, nullptr);
                if (far_light_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), far_light_pipeline_layout_, nullptr);
                if (field_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), field_layout_, nullptr);
                if (skip_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), skip_layout_, nullptr);
                if (far_light_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), far_light_layout_, nullptr);
            }

            void CloudscapeCompilePass::create_catalogue()
            {
                GenusCatalogue catalogue{};
                for (int i = 0; i < CLOUD_GENUS_COUNT; ++i)
                {
                    const CloudGenusProfile profile =
                        cloud_genus_profile(static_cast<CloudGenus>(i));
                    // Same lane assignment the scene block's deck arrays use, so the bake's deck
                    // evaluation does not care which of the two filled them.
                    catalogue.a[i][0] = profile.base_altitude;
                    catalogue.a[i][1] = profile.top_altitude;
                    catalogue.a[i][2] = profile.coverage;
                    catalogue.a[i][3] = profile.density;
                    catalogue.b[i][0] = profile.stratiform;
                    catalogue.b[i][1] = profile.detail_strength;
                    catalogue.b[i][2] = profile.shape_scale;
                    catalogue.b[i][3] = profile.detail_scale;
                    catalogue.c[i][0] = static_cast<float>(profile.wind.x);
                    catalogue.c[i][1] = static_cast<float>(profile.wind.y);
                    catalogue.c[i][2] = static_cast<float>(profile.wind.z);
                    catalogue.c[i][3] =
                        static_cast<float>(static_cast<std::uint32_t>(profile.noise));
                    catalogue.d[i][0] = profile.anvil;
                }

                const CloudGenusThresholds& thresholds = cloud_genus_thresholds();
                catalogue.thresholds[0] = thresholds.convective;
                catalogue.thresholds[1] = thresholds.low_broken;
                catalogue.thresholds[2] = thresholds.middle_overcast;
                catalogue.thresholds[3] = thresholds.high_sheet;
                catalogue.thresholds_tail[0] = thresholds.cumulonimbus_convective;
                catalogue.thresholds_tail[1] = thresholds.cumulonimbus_coverage;
                catalogue.thresholds_tail[2] = thresholds.enable_coverage;

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = sizeof(GenusCatalogue);
                buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc, &catalogue_,
                                              &catalogue_allocation_, &mapped),
                              "vmaCreateBuffer(cloud genus catalogue)");
                if (mapped.pMappedData != nullptr)
                    std::memcpy(mapped.pMappedData, &catalogue, sizeof(GenusCatalogue));
            }

            void CloudscapeCompilePass::destroy_catalogue()
            {
                if (catalogue_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), catalogue_, catalogue_allocation_);
                catalogue_ = VK_NULL_HANDLE;
                catalogue_allocation_ = VK_NULL_HANDLE;
            }

            void CloudscapeCompilePass::create_pipelines()
            {
                field_pipeline_ = pipelines_.create_compute(field_pipeline_layout_,
                                                            shaders_.module("cloudscape_field.comp"));
                skip_pipeline_ = pipelines_.create_compute(skip_pipeline_layout_,
                                                           shaders_.module("cloudscape_skip.comp"));
                far_light_pipeline_ =
                    pipelines_.create_compute(far_light_pipeline_layout_,
                                              shaders_.module("cloudscape_far_light.comp"));
            }

            void CloudscapeCompilePass::destroy_pipelines()
            {
                if (field_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), field_pipeline_, nullptr);
                if (skip_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), skip_pipeline_, nullptr);
                if (far_light_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), far_light_pipeline_, nullptr);
                field_pipeline_ = VK_NULL_HANDLE;
                skip_pipeline_ = VK_NULL_HANDLE;
                far_light_pipeline_ = VK_NULL_HANDLE;
            }

            void CloudscapeCompilePass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
                // A shader edit can change the field's contents, so force a rebake next frame.
                // An in-flight amortized far bake is dropped rather than finished: its earlier
                // slabs were recorded with the old pipeline, and completing it would publish a
                // window that is half one shader and half another.
                built_ = false;
                far_baking_ = false;
                far_queued_ = false;
                far_completing_ = false;
                far_slice_base_ = 0;
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

            void CloudscapeCompilePass::place_window(Window& window, float span,
                                                     std::uint32_t resolution, const double eye[3],
                                                     double wind_x, double wind_z,
                                                     float time_seconds, const float sun[3])
            {
                // Snapped to the window's own texel lattice, in absolute coordinates. That is what
                // makes re-centring free of artefacts: the bake is a pure function of the pattern
                // position, so a window that moved by a whole number of texels reproduces the
                // identical value at the identical world point, and a rebake is invisible. Snap to
                // anything else and every rebake would resample the sky onto a shifted lattice and
                // shimmer.
                const double texel = double(span) / double(resolution);
                const double centre_x = eye[0] + wind_x;
                const double centre_z = eye[2] + wind_z;
                window.pattern_origin_x =
                    std::floor((centre_x - double(span) * 0.5) / texel) * texel;
                window.pattern_origin_z =
                    std::floor((centre_z - double(span) * 0.5) / texel) * texel;
                window.wind_x = wind_x;
                window.wind_z = wind_z;
                window.eye_x = eye[0];
                window.eye_z = eye[2];
                window.time_seconds = time_seconds;
                window.sun[0] = sun[0];
                window.sun[1] = sun[1];
                window.sun[2] = sun[2];
                window.baked = true;
            }

            void CloudscapeCompilePass::window_map(const Window& window, float span,
                                                   const double eye[3], double wind_x,
                                                   double wind_z, float map[4])
            {
                if (!window.baked || span <= 0.0f)
                {
                    map[0] = map[1] = map[2] = map[3] = 0.0f;
                    return;
                }
                // Camera-relative XZ metres -> window UV. The eye and the wind are *this frame's*,
                // not the bake's: that difference is precisely how far the sky has drifted since,
                // and folding it in here is what keeps the pattern moving continuously between
                // rebakes instead of freezing and jumping. Formed in double because both terms are
                // planet-scale and their difference is metres.
                const double inverse_span = 1.0 / double(span);
                map[0] = static_cast<float>(inverse_span);
                map[1] = static_cast<float>(inverse_span);
                map[2] = static_cast<float>((eye[0] + wind_x - window.pattern_origin_x) * inverse_span);
                map[3] = static_cast<float>((eye[2] + wind_z - window.pattern_origin_z) * inverse_span);
            }

            void CloudscapeCompilePass::window_push(const Window& window, float span,
                                                    float weather_scale, bool derive_genus,
                                                    std::uint32_t supersample, Push& push)
            {
                push.pattern_origin[0] = static_cast<float>(window.pattern_origin_x);
                push.pattern_origin[1] = static_cast<float>(window.pattern_origin_z);
                // The same corner with the wind and the eye taken back out: camera-relative scene
                // metres, which is the frame `weather_field_map` addresses the meteorology in.
                push.world_origin[0] =
                    static_cast<float>(window.pattern_origin_x - window.wind_x - window.eye_x);
                push.world_origin[1] =
                    static_cast<float>(window.pattern_origin_z - window.wind_z - window.eye_z);
                push.span_meters = span;
                push.weather_scale = weather_scale;
                push.derive_genus = derive_genus ? 1u : 0u;
                push.supersample = supersample;
                // Per-slab; record_density stamps the slice a dispatch actually starts at.
                push.slab_base = 0u;
            }

            void CloudscapeCompilePass::update_window(const Frame::FrameContext& frame,
                                                      const Environment& environment,
                                                      const Atmosphere::AtmosphereNest* nest,
                                                      Scene::SceneUniforms& uniforms)
            {
                near_dirty_ = false;
                far_dirty_ = false;
                nest_ = nest;

                // The nest's own addressing, for the bake to read condensate through
                // (docs/slop/atmosphere_system.md §7.1). Stamped here rather than in
                // fill_scene_uniforms because the nest is centred on the *simulation's*
                // observer while everything in that block is camera-relative, and this is the
                // one place both are in hand.
                for (int i = 0; i < 4; ++i)
                {
                    uniforms.atmosphere_nest_map[i] = 0.0f;
                    uniforms.atmosphere_nest_parameters[i] = 0.0f;
                }
                if (nest_ != nullptr && nest_->step_count() > 0)
                {
                    const AtmosphereNestSize& size = nest_->size();
                    const double span = double(size.spacing_m) * double(size.cells_x);
                    double origin_x = 0.0;
                    double origin_z = 0.0;
                    nest_->origin(origin_x, origin_z);
                    const double inverse_span = 1.0 / span;
                    uniforms.atmosphere_nest_map[0] = static_cast<float>(inverse_span);
                    uniforms.atmosphere_nest_map[1] = static_cast<float>(inverse_span);
                    // Formed in double: both terms are planet-scale and their difference is
                    // metres, the same discipline every other camera-relative term in the block
                    // follows.
                    uniforms.atmosphere_nest_map[2] =
                        static_cast<float>((frame.eye[0] - origin_x) * inverse_span);
                    uniforms.atmosphere_nest_map[3] =
                        static_cast<float>((frame.eye[2] - origin_z) * inverse_span);
                    uniforms.atmosphere_nest_parameters[0] = size.top_m;
                    // Uploaded already inverted so the bake's altitude -> W is one pow rather
                    // than a divide inside a per-texel loop.
                    uniforms.atmosphere_nest_parameters[1] = 1.0f / ATMOSPHERE_VERTICAL_STRETCH;
                    uniforms.atmosphere_nest_parameters[2] = 1.0f;
                    // The extinction of the authored "fully overcast" water content, from the
                    // authored droplet radius: the scale the baked density states sigma against.
                    const AtmosphereNestParameters& physics = environment.atmosphere_nest;
                    uniforms.atmosphere_nest_parameters[3] =
                        3.0f * std::max(physics.coverage_reference_lwc, 1.0e-6f) /
                        (2.0f * physics.water_density *
                         std::max(physics.droplet_effective_radius, 1.0e-7f));
                }

                if (!environment.clouds.enabled)
                {
                    // A zero scale is the published "no window" state every consumer checks; see
                    // cloud_field_window.glsl. The windows are dropped rather than kept, so
                    // switching the cloudscape back on rebakes against wherever the camera has
                    // gone in the meantime instead of resurrecting a stale placement.
                    for (int i = 0; i < 4; ++i)
                    {
                        uniforms.cloud_field_near[i] = 0.0f;
                        uniforms.cloud_field_far[i] = 0.0f;
                        uniforms.cloud_field_parameters[i] = 0.0f;
                        uniforms.cloud_field_pattern[i] = 0.0f;
                    }
                    near_window_.baked = false;
                    far_window_.baked = false;
                    far_baking_ = false;
                    far_queued_ = false;
                    far_completing_ = false;
                    far_slice_base_ = 0;
                    built_ = false;
                    return;
                }

                const float time_seconds = uniforms.misc[2];

                // "The" wind: deck 0's, the same dominant-deck convention the ground-shadow march
                // and the old sample-time UV scroll both used. With genus resolved per column no
                // single deck owns the sky any more, but the field still advects as one pattern,
                // and the reference column's low deck is the honest choice for it — that is the
                // column the camera is standing in. Accumulated in double: it is a velocity times
                // a monotonically growing clock, and it is what the window's own origin tracks.
                const double wind_x = double(uniforms.cloud_deck_c[0][0]) * double(time_seconds);
                const double wind_z = double(uniforms.cloud_deck_c[0][2]) * double(time_seconds);
                const float sun[3] = {uniforms.sun_dir[0], uniforms.sun_dir[1], uniforms.sun_dir[2]};

                const WeatherField& field = environment.weather_field;
                const bool derive = field.valid() && field.derives_genus;

                Snapshot snapshot{};
                for (int i = 0; i < CLOUD_MAX_DECKS; ++i)
                {
                    snapshot.decks[i].enabled = environment.clouds.decks[i].enabled ? 1u : 0u;
                    snapshot.decks[i].genus =
                        static_cast<std::uint32_t>(environment.clouds.decks[i].genus);
                    snapshot.decks[i].coverage_bias = environment.clouds.decks[i].coverage_bias;
                    snapshot.decks[i].density_scale = environment.clouds.decks[i].density_scale;
                }
                snapshot.weather_scale = environment.clouds.weather_scale;
                // The bake maps its Y axis across exactly this altitude span, so a shell that moved
                // is a field addressed against the wrong altitudes — a rebake, not a cadence.
                snapshot.shell_base = uniforms.cloud_global[1];
                snapshot.shell_top = uniforms.cloud_global[2];
                snapshot.derive_genus = derive ? 1u : 0u;
                // A stepped nest is new condensate, and since §7.1 the bake reads condensate
                // directly -- so this is the trigger that actually fires most of the time once
                // the nest is running, in place of the weather cadence below.
                snapshot.nest_step = nest_ != nullptr ? nest_->step_count() : 0;
                const bool authored_changed = cloudscape_changed(snapshot);

                const auto drifted = [&](const Window& window, float span)
                {
                    if (!window.baked)
                        return true;
                    const double centre_x = window.pattern_origin_x + double(span) * 0.5;
                    const double centre_z = window.pattern_origin_z + double(span) * 0.5;
                    const double dx = (frame.eye[0] + wind_x) - centre_x;
                    const double dz = (frame.eye[2] + wind_z) - centre_z;
                    const double limit = double(span) * REBAKE_DRIFT_FRACTION;
                    return (dx * dx + dz * dz) > (limit * limit);
                };

                near_dirty_ = authored_changed || drifted(near_window_, NEAR_SPAN_METERS) ||
                              (derive && (time_seconds - near_window_.time_seconds) >=
                                             NEAR_WEATHER_INTERVAL_SECONDS);

                // The far bake is amortized: a trigger starts a multi-frame bake against a
                // *pending* placement, and consumers keep the previous window — mapping and
                // contents both — until the completing frame switches the two together. First,
                // advance whatever register_pass recorded last frame.
                if (far_baking_)
                {
                    far_slice_base_ += FAR_BAKE_SLICES_PER_FRAME;
                    if (far_slice_base_ >= far_source_.depth)
                    {
                        far_baking_ = false;
                        far_slice_base_ = 0;
                    }
                }

                // Staleness is judged against the newest placement there is, so an in-flight
                // bake is not restarted by the staleness of the published one; a trigger that
                // fires mid-bake queues the next bake instead, which is what keeps a stepping
                // nest from starving the far window of completions.
                const Window& far_reference = far_baking_ ? far_pending_ : far_window_;
                const float sun_dot = far_reference.sun[0] * sun[0] +
                                      far_reference.sun[1] * sun[1] +
                                      far_reference.sun[2] * sun[2];
                const bool far_stale = authored_changed ||
                                       drifted(far_reference, FAR_SPAN_METERS) ||
                                       (derive && (time_seconds - far_reference.time_seconds) >=
                                                      FAR_WEATHER_INTERVAL_SECONDS) ||
                                       sun_dot < FAR_SUN_COS_TOLERANCE;
                if (far_stale && far_baking_)
                    far_queued_ = true;
                if (!far_baking_ && (far_stale || far_queued_))
                {
                    far_queued_ = false;
                    place_window(far_pending_, FAR_SPAN_METERS, FIELD_RESOLUTION_XZ, frame.eye,
                                 wind_x, wind_z, time_seconds, sun);
                    window_push(far_pending_, FAR_SPAN_METERS, environment.clouds.weather_scale,
                                derive, FAR_SUPERSAMPLE, far_push_);
                    far_baking_ = true;
                    far_slice_base_ = 0;
                }

                // The frame recording the last slab also resolves the sun depth and publishes
                // the pending placement, so the mapping below and the contents it addresses
                // switch in the same frame.
                far_completing_ = far_baking_ && (far_slice_base_ + FAR_BAKE_SLICES_PER_FRAME >=
                                                  far_source_.depth);
                if (far_completing_)
                    far_window_ = far_pending_;
                far_dirty_ = far_baking_;

                // Whether the *placement* changed, as opposed to merely the contents. Only a
                // re-centring invalidates the light volume's and the shadow map's amortized
                // slices, because only then do the already-baked ones describe a different
                // piece of the world than the ones still to come. A weather-cadence rebake
                // leaves the window exactly where it was, so those two keep amortizing.
                near_recentred_ = false;
                if (near_dirty_)
                {
                    const double previous_x = near_window_.pattern_origin_x;
                    const double previous_z = near_window_.pattern_origin_z;
                    const bool was_baked = near_window_.baked;
                    place_window(near_window_, NEAR_SPAN_METERS, FIELD_RESOLUTION_XZ, frame.eye,
                                 wind_x, wind_z, time_seconds, sun);
                    near_recentred_ = !was_baked ||
                                      near_window_.pattern_origin_x != previous_x ||
                                      near_window_.pattern_origin_z != previous_z;
                }
                window_map(near_window_, NEAR_SPAN_METERS, frame.eye, wind_x, wind_z,
                           uniforms.cloud_field_near);
                window_map(far_window_, FAR_SPAN_METERS, frame.eye, wind_x, wind_z,
                           uniforms.cloud_field_far);
                uniforms.cloud_field_parameters[0] = near_window_.baked ? NEAR_SPAN_METERS : 0.0f;
                uniforms.cloud_field_parameters[1] = far_window_.baked ? FAR_SPAN_METERS : 0.0f;
                uniforms.cloud_field_parameters[2] =
                    NEAR_SPAN_METERS / float(FIELD_RESOLUTION_XZ / SKIP_DOWNSAMPLE_XZ);
                uniforms.cloud_field_parameters[3] = FAR_SPAN_METERS / float(FIELD_RESOLUTION_XZ);
                // The pattern frame the march's analytic carve reconstructs world-anchored
                // noise coordinates in (CloudsV2); the same float cast the bake's own push
                // constants make, so the carve and the bake agree to the bit.
                uniforms.cloud_field_pattern[0] =
                    near_window_.baked ? static_cast<float>(near_window_.pattern_origin_x) : 0.0f;
                uniforms.cloud_field_pattern[1] =
                    near_window_.baked ? static_cast<float>(near_window_.pattern_origin_z) : 0.0f;
                uniforms.cloud_field_pattern[2] =
                    far_window_.baked ? static_cast<float>(far_window_.pattern_origin_x) : 0.0f;
                uniforms.cloud_field_pattern[3] =
                    far_window_.baked ? static_cast<float>(far_window_.pattern_origin_z) : 0.0f;

                window_push(near_window_, NEAR_SPAN_METERS, environment.clouds.weather_scale,
                            derive, 1u, near_push_);
                // far_push_ is deliberately not rebuilt here: it was taken from far_pending_
                // when its bake began, and an in-flight bake keeps recording against that
                // placement whatever this frame's triggers decided.
                built_ = true;
            }

            void CloudscapeCompilePass::record_density(VkCommandBuffer command,
                                                       const Frame::FrameContext& frame,
                                                       VkBuffer uniform_buffer, Volume& target,
                                                       const Push& push,
                                                       std::uint32_t slab_base,
                                                       std::uint32_t slab_depth, bool discard)
            {
                // UNDEFINED discards, which is right only for a slab that starts a bake; an
                // amortized continuation owns slices written in earlier frames and must keep
                // them.
                transition(command, target.image,
                           discard ? VK_IMAGE_LAYOUT_UNDEFINED : VK_IMAGE_LAYOUT_GENERAL,
                           VK_IMAGE_LAYOUT_GENERAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                               VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                           discard ? VK_ACCESS_2_NONE : VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                const VkDescriptorSet set = frame.descriptors->allocate(field_layout_);
                Resources::DescriptorWriter writer;
                writer.storage_image(0, target.view);
                writer.uniform_buffer(1, uniform_buffer, sizeof(Scene::SceneUniforms));
                writer.sampled_image(2, noise_.shape(), noise_.sampler());
                writer.sampled_image(3, noise_.detail(), noise_.sampler());
                writer.sampled_image(4, noise_.weather(), noise_.sampler());
                writer.sampled_image(5, noise_.cirrus(), noise_.sampler());
                writer.sampled_image(6, weather_.view(), weather_.sampler());
                writer.uniform_buffer(7, catalogue_, sizeof(GenusCatalogue));
                // Falls back to the weather field's own image when no nest exists: a descriptor
                // has to point at *something* valid, and the shader never reads it in that case
                // because the scene block's nest flag is zero.
                writer.sampled_image(8,
                                     nest_ != nullptr ? nest_->extinction_view() : weather_.view(),
                                     nest_ != nullptr ? nest_->sampler() : weather_.sampler(),
                                     nest_ != nullptr ? VK_IMAGE_LAYOUT_GENERAL
                                                      : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
                writer.update(device_.device(), set);
                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, field_pipeline_);
                Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               field_pipeline_layout_, 0, set);
                Push slab = push;
                slab.slab_base = slab_base;
                vkCmdPushConstants(command, field_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                   sizeof(Push), &slab);
                vkCmdDispatch(command, groups(target.width), groups(target.height),
                              groups(slab_depth));
            }

            void CloudscapeCompilePass::record_skip(VkCommandBuffer command,
                                                    const Frame::FrameContext& frame)
            {
                transition(command, skip_.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                const VkDescriptorSet set = frame.descriptors->allocate(skip_layout_);
                Resources::DescriptorWriter writer;
                writer.storage_image(0, skip_.view);
                writer.sampled_image(1, near_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.update(device_.device(), set);
                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, skip_pipeline_);
                Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               skip_pipeline_layout_, 0, set);
                vkCmdDispatch(command, groups(skip_.width), groups(skip_.height),
                              groups(skip_.depth));
            }

            void CloudscapeCompilePass::record_far_light(VkCommandBuffer command,
                                                         const Frame::FrameContext& frame,
                                                         VkBuffer uniform_buffer)
            {
                transition(command, far_.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                               VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                const FarLightPush push{FAR_SPAN_METERS};
                const VkDescriptorSet set = frame.descriptors->allocate(far_light_layout_);
                Resources::DescriptorWriter writer;
                writer.storage_image(0, far_.view);
                writer.sampled_image(1, far_source_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.uniform_buffer(2, uniform_buffer, sizeof(Scene::SceneUniforms));
                writer.update(device_.device(), set);
                vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, far_light_pipeline_);
                Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               far_light_pipeline_layout_, 0, set);
                vkCmdPushConstants(command, far_light_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                                   0, sizeof(FarLightPush), &push);
                vkCmdDispatch(command, groups(far_.width), groups(far_.height), groups(far_.depth));
            }

            void CloudscapeCompilePass::register_pass(Graph::RenderGraph& graph,
                                                       const Frame::FrameContext& frame)
            {
                if (frame.environment == nullptr || !frame.environment->clouds.enabled)
                    return;
                // update_window already decided; nothing this frame is stale.
                if (!near_dirty_ && !far_dirty_)
                    return;

                const Graph::BufferHandle uniforms = frame.targets.uniforms;
                graph.add_pass(
                    "cloudscape-compile",
                    [uniforms](Graph::RenderPassBuilder& builder)
                    {
                        // The window images are pass-owned and barriered by hand below; the one
                        // graph resource is the scene uniform block the bake reads the deck stack,
                        // the march shell and the weather field's addressing from. A side effect
                        // keeps the pass from being culled — it has no graph-tracked write to keep
                        // it alive otherwise.
                        builder.read(uniforms, Graph::BufferAccess::UniformRead);
                        builder.set_side_effect();
                    },
                    [this, &frame, uniforms](VkCommandBuffer command,
                                             const Graph::PassContext& context)
                    {
                        const VkBuffer uniform_buffer = context.buffer(uniforms);

                        if (near_dirty_)
                        {
                            record_density(command, frame, uniform_buffer, near_, near_push_, 0u,
                                           near_.depth, true);
                            // Readable by the skip build below (compute) and, once this pass ends,
                            // by the view march, the light volume, the shadow map and the panorama.
                            transition(command, near_.image, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                           VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                            record_skip(command, frame);
                            transition(command, skip_.image, VK_IMAGE_LAYOUT_GENERAL,
                                       VK_IMAGE_LAYOUT_GENERAL,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                        }

                        if (far_dirty_)
                        {
                            // One slab of the amortized bake. Only the bake-opening slab may
                            // discard; every later one continues into slices written in
                            // earlier frames.
                            const std::uint32_t slab =
                                std::min(FAR_BAKE_SLICES_PER_FRAME,
                                         far_source_.depth - far_slice_base_);
                            record_density(command, frame, uniform_buffer, far_source_, far_push_,
                                           far_slice_base_, slab, far_slice_base_ == 0u);

                            if (far_completing_)
                            {
                                transition(command, far_source_.image, VK_IMAGE_LAYOUT_GENERAL,
                                           VK_IMAGE_LAYOUT_GENERAL,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                                record_far_light(command, frame, uniform_buffer);
                                transition(command, far_.image, VK_IMAGE_LAYOUT_GENERAL,
                                           VK_IMAGE_LAYOUT_GENERAL,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                               VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                            }
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
