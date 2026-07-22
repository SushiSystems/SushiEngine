/**************************************************************************/
/* sdf_probe_tracer.cpp                                                    */
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

#include "gi/sdf_probe_tracer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "frame/frame_context.hpp"
#include "geometry/mesh_registry.hpp"
#include "gi/probe_volume.hpp"
#include "gi/sdf_clipmap.hpp"
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
        namespace Gi
        {
            namespace
            {
                constexpr VkFormat CLIPMAP_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr std::uint32_t POPULATE_GROUP = 4;
                constexpr std::uint32_t RELIGHT_GROUP = 64;
                constexpr std::int32_t RAY_COUNT = 32;
                constexpr float MAX_TRACE_DISTANCE = 64.0f;
                // When the lattice is stable, a quarter of all probes are relit per frame, so
                // the whole grid refreshes every four frames for dynamic lighting and moving
                // objects. A fraction (not a fixed count) keeps that period as the cascade
                // count scales the probe total.
                constexpr std::int32_t RELIGHT_REFRESH_FRAMES = 4;

                std::uint32_t groups(std::uint32_t extent, std::uint32_t size) noexcept
                {
                    return (extent + size - 1) / size;
                }

                void image_barrier(VkCommandBuffer cmd, VkImage image, VkImageLayout from,
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

            SdfProbeTracer::SdfProbeTracer(Vulkan::VulkanDevice& device,
                                           Resources::ShaderLibrary& shaders,
                                           Resources::GraphicsPipelineFactory& pipelines,
                                           Geometry::MeshRegistry& meshes)
                : device_(device), shaders_(shaders), pipelines_(pipelines), meshes_(meshes),
                  brick_uploaded_(static_cast<std::size_t>(MAX_SDF_BRICKS), false)
            {
                // Populate: the distance clipmap storage image, the primitive array, the
                // clipmap config, the imported-mesh instances, the shared brick atlas, and the
                // emissive clipmap storage image.
                VkDescriptorSetLayoutBinding populate_bindings[6]{};
                populate_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                populate_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                populate_bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                populate_bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                populate_bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                populate_bindings[5].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                for (std::uint32_t i = 0; i < 6; ++i)
                {
                    populate_bindings[i].binding = i;
                    populate_bindings[i].descriptorCount = 1;
                    populate_bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo populate_info{};
                populate_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                populate_info.bindingCount = 6;
                populate_info.pBindings = populate_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &populate_info, nullptr,
                                                          &populate_layout_),
                              "vkCreateDescriptorSetLayout(sdf populate)");

                VkPipelineLayoutCreateInfo populate_pipeline_info{};
                populate_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                populate_pipeline_info.setLayoutCount = 1;
                populate_pipeline_info.pSetLayouts = &populate_layout_;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &populate_pipeline_info,
                                                     nullptr, &populate_pipeline_layout_),
                              "vkCreatePipelineLayout(sdf populate)");

                // Relight: probe SH out, environment SH in, the distance clipmap, the probe and
                // clipmap config blocks, and the emissive clipmap.
                VkDescriptorSetLayoutBinding relight_bindings[6]{};
                relight_bindings[0].binding = 0;
                relight_bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                relight_bindings[1].binding = 1;
                relight_bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                relight_bindings[2].binding = 2;
                relight_bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                relight_bindings[3].binding = 3;
                relight_bindings[3].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                relight_bindings[4].binding = 4;
                relight_bindings[4].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                relight_bindings[5].binding = 5;
                relight_bindings[5].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                for (VkDescriptorSetLayoutBinding& binding : relight_bindings)
                {
                    binding.descriptorCount = 1;
                    binding.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo relight_info{};
                relight_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                relight_info.bindingCount = 6;
                relight_info.pBindings = relight_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &relight_info, nullptr,
                                                          &relight_layout_),
                              "vkCreateDescriptorSetLayout(sdf relight)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(RelightPush);

                VkPipelineLayoutCreateInfo relight_pipeline_info{};
                relight_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                relight_pipeline_info.setLayoutCount = 1;
                relight_pipeline_info.pSetLayouts = &relight_layout_;
                relight_pipeline_info.pushConstantRangeCount = 1;
                relight_pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &relight_pipeline_info,
                                                     nullptr, &relight_pipeline_layout_),
                              "vkCreatePipelineLayout(sdf relight)");

                create_clipmap();
                create_buffers();
                create_pipelines();
            }

            SdfProbeTracer::~SdfProbeTracer()
            {
                destroy_pipelines();
                destroy_buffers();
                destroy_clipmap();
                if (populate_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), populate_pipeline_layout_, nullptr);
                if (relight_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), relight_pipeline_layout_, nullptr);
                if (populate_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), populate_layout_, nullptr);
                if (relight_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), relight_layout_, nullptr);
            }

            void SdfProbeTracer::create_clipmap()
            {
                const std::uint32_t resolution =
                    static_cast<std::uint32_t>(SDF_CLIPMAP_RESOLUTION);

                const auto make_volume = [&](VkImage& image, VmaAllocation& allocation,
                                             VkImageView& view, const char* label)
                {
                    VkImageCreateInfo image_info{};
                    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    image_info.imageType = VK_IMAGE_TYPE_3D;
                    image_info.format = CLIPMAP_FORMAT;
                    image_info.extent = {resolution, resolution, resolution};
                    image_info.mipLevels = 1;
                    image_info.arrayLayers = 1;
                    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                    image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &image,
                                                 &allocation, nullptr),
                                  label);

                    VkImageViewCreateInfo view_info{};
                    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                    view_info.image = image;
                    view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                    view_info.format = CLIPMAP_FORMAT;
                    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    view_info.subresourceRange.levelCount = 1;
                    view_info.subresourceRange.layerCount = 1;
                    Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &view),
                                  label);
                };

                make_volume(clipmap_, clipmap_allocation_, clipmap_view_, "sdf clipmap");
                make_volume(emissive_, emissive_allocation_, emissive_view_, "sdf emissive clipmap");
            }

            void SdfProbeTracer::destroy_clipmap()
            {
                if (clipmap_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), clipmap_view_, nullptr);
                if (clipmap_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), clipmap_, clipmap_allocation_);
                if (emissive_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), emissive_view_, nullptr);
                if (emissive_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), emissive_, emissive_allocation_);
                clipmap_view_ = VK_NULL_HANDLE;
                clipmap_ = VK_NULL_HANDLE;
                emissive_view_ = VK_NULL_HANDLE;
                emissive_ = VK_NULL_HANDLE;
            }

            void SdfProbeTracer::create_buffers()
            {
                const auto make_mapped = [&](VkDeviceSize size, VkBufferUsageFlags usage,
                                             VkBuffer& buffer, VmaAllocation& allocation,
                                             void*& mapped)
                {
                    VkBufferCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    info.size = size;
                    info.usage = usage;
                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo mapped_info{};
                    Vulkan::check(
                        vmaCreateBuffer(device_.allocator(), &info, &alloc, &buffer, &allocation,
                                        &mapped_info),
                        "vmaCreateBuffer(sdf tracer)");
                    mapped = mapped_info.pMappedData;
                };

                for (std::uint32_t i = 0; i < RING; ++i)
                {
                    make_mapped(sizeof(SdfPrimitive) * MAX_SDF_PRIMITIVES,
                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, primitive_buffers_[i],
                                primitive_allocations_[i], primitive_mapped_[i]);
                    make_mapped(sizeof(SdfClipmapConfig), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                clip_config_buffers_[i], clip_config_allocations_[i],
                                clip_config_mapped_[i]);
                    make_mapped(sizeof(ProbeVolumeConfig), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                probe_config_buffers_[i], probe_config_allocations_[i],
                                probe_config_mapped_[i]);
                    make_mapped(sizeof(SdfMeshInstance) * MAX_SDF_MESH_INSTANCES,
                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, mesh_buffers_[i],
                                mesh_allocations_[i], mesh_mapped_[i]);
                }

                // The brick atlas is written from the host once per mesh and read every frame,
                // so it is a single persistent slot store rather than a per-frame ring.
                make_mapped(sizeof(float) * static_cast<VkDeviceSize>(SDF_BRICK_VOXELS) *
                                MAX_SDF_BRICKS,
                            VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, brick_atlas_,
                            brick_atlas_allocation_, brick_atlas_mapped_);
            }

            void SdfProbeTracer::destroy_buffers()
            {
                for (std::uint32_t i = 0; i < RING; ++i)
                {
                    if (primitive_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), primitive_buffers_[i],
                                         primitive_allocations_[i]);
                    if (clip_config_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), clip_config_buffers_[i],
                                         clip_config_allocations_[i]);
                    if (probe_config_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), probe_config_buffers_[i],
                                         probe_config_allocations_[i]);
                    if (mesh_buffers_[i] != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), mesh_buffers_[i],
                                         mesh_allocations_[i]);
                }
                if (brick_atlas_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), brick_atlas_, brick_atlas_allocation_);
                brick_atlas_ = VK_NULL_HANDLE;
            }

            void SdfProbeTracer::create_pipelines()
            {
                populate_pipeline_ = pipelines_.create_compute(
                    populate_pipeline_layout_, shaders_.module("sdf_populate.comp"));
                relight_pipeline_ = pipelines_.create_compute(
                    relight_pipeline_layout_, shaders_.module("sdf_probe_relight.comp"));
            }

            void SdfProbeTracer::destroy_pipelines()
            {
                if (populate_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), populate_pipeline_, nullptr);
                if (relight_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), relight_pipeline_, nullptr);
                populate_pipeline_ = VK_NULL_HANDLE;
                relight_pipeline_ = VK_NULL_HANDLE;
            }

            void SdfProbeTracer::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            std::int32_t SdfProbeTracer::build_mesh_instances(const ProbeRelightInputs& inputs,
                                                              std::uint32_t ring)
            {
                const Frame::SceneDrawList& draws = inputs.frame->draws;
                SdfMeshInstance* out = static_cast<SdfMeshInstance*>(mesh_mapped_[ring]);
                std::int32_t count = 0;
                for (std::size_t i = 0;
                     i < draws.instance_count && count < MAX_SDF_MESH_INSTANCES; ++i)
                {
                    const MeshInstance& instance = draws.instances[i];
                    if (instance.mesh == INVALID_MESH)
                        continue;
                    const std::int32_t slot = static_cast<std::int32_t>(instance.mesh);
                    if (slot < 0 || slot >= MAX_SDF_BRICKS)
                        continue;
                    const MeshSdfBrick* brick = meshes_.mesh_brick(instance.mesh);
                    if (brick == nullptr ||
                        brick->distances.size() < static_cast<std::size_t>(SDF_BRICK_VOXELS))
                        continue;

                    // Upload the brick to its atlas slot the first frame it is needed. A
                    // brand-new mesh cannot be referenced by an in-flight frame, so writing a
                    // fresh slot never races a submit already reading a different one.
                    if (!brick_uploaded_[static_cast<std::size_t>(slot)])
                    {
                        const VkDeviceSize offset =
                            static_cast<VkDeviceSize>(slot) * SDF_BRICK_VOXELS * sizeof(float);
                        float* destination =
                            static_cast<float*>(brick_atlas_mapped_) +
                            static_cast<std::size_t>(slot) * SDF_BRICK_VOXELS;
                        std::memcpy(destination, brick->distances.data(),
                                    sizeof(float) * SDF_BRICK_VOXELS);
                        vmaFlushAllocation(device_.allocator(), brick_atlas_allocation_, offset,
                                           sizeof(float) * SDF_BRICK_VOXELS);
                        brick_uploaded_[static_cast<std::size_t>(slot)] = true;
                    }

                    const float albedo[3] = {static_cast<float>(instance.color.x),
                                             static_cast<float>(instance.color.y),
                                             static_cast<float>(instance.color.z)};
                    const Material& material = instance.material;
                    const float emissive_scale =
                        material.emissive_enabled ? material.emissive_intensity : 0.0f;
                    const float emissive[3] = {
                        static_cast<float>(material.emissive.x) * emissive_scale,
                        static_cast<float>(material.emissive.y) * emissive_scale,
                        static_cast<float>(material.emissive.z) * emissive_scale};
                    fill_sdf_mesh_instance(instance.model, inputs.frame->eye, brick->aabb_min,
                                           brick->aabb_max, slot, albedo, emissive, out[count]);
                    ++count;
                }
                return count;
            }

            void SdfProbeTracer::relight(VkCommandBuffer cmd, const ProbeRelightInputs& inputs)
            {
                if (inputs.frame == nullptr || inputs.config == nullptr || inputs.probe_count == 0)
                    return;

                const std::uint32_t ring = inputs.frame->index % RING;

                // Extract the frame's analytic primitives and imported-mesh instances, and size
                // the clipmap around them.
                SdfPrimitive* primitives = static_cast<SdfPrimitive*>(primitive_mapped_[ring]);
                const std::int32_t primitive_count = build_sdf_primitives(
                    inputs.frame->draws, inputs.frame->eye, primitives, MAX_SDF_PRIMITIVES);
                const std::int32_t mesh_count = build_mesh_instances(inputs, ring);

                SdfClipmapConfig clip_config{};
                configure_sdf_clipmap(inputs.frame->eye, primitive_count, clip_config);
                clip_config.extra[0] = mesh_count;
                std::memcpy(clip_config_mapped_[ring], &clip_config, sizeof(clip_config));
                std::memcpy(probe_config_mapped_[ring], inputs.config, sizeof(ProbeVolumeConfig));

                // Choose the relight window. A full relight is forced when nothing has been
                // relit yet, when any cascade's lattice shifts a cell (every probe index in it
                // then maps to a new world point), or when the sun moves; otherwise a
                // round-robin slice across all cascades keeps the cost down while still
                // refreshing dynamic lighting across a few frames. Each cascade snaps to its
                // own spacing, so the coarse ones cross a cell far less often than the finest.
                std::int32_t center[GI_NUM_CASCADES][3];
                bool force_full = !has_relit_;
                for (std::int32_t cascade = 0; cascade < GI_NUM_CASCADES; ++cascade)
                {
                    const double spacing =
                        static_cast<double>(probe_cascade_spacing(cascade));
                    for (int axis = 0; axis < 3; ++axis)
                    {
                        center[cascade][axis] = static_cast<std::int32_t>(
                            std::floor(inputs.frame->eye[axis] / spacing + 0.5));
                        if (center[cascade][axis] != last_center_cell_[cascade][axis])
                            force_full = true;
                    }
                }

                float sun[3] = {last_sun_[0], last_sun_[1], last_sun_[2]};
                float sun_intensity = last_sun_intensity_;
                if (inputs.frame->environment != nullptr)
                {
                    const Environment& environment = *inputs.frame->environment;
                    sun[0] = static_cast<float>(environment.sun.direction.x);
                    sun[1] = static_cast<float>(environment.sun.direction.y);
                    sun[2] = static_cast<float>(environment.sun.direction.z);
                    sun_intensity = environment.sun.intensity;
                    if (std::fabs(sun[0] - last_sun_[0]) > 1e-4f ||
                        std::fabs(sun[1] - last_sun_[1]) > 1e-4f ||
                        std::fabs(sun[2] - last_sun_[2]) > 1e-4f ||
                        std::fabs(sun_intensity - last_sun_intensity_) > 1e-3f)
                        force_full = true;
                }

                const std::int32_t total = static_cast<std::int32_t>(inputs.probe_count);
                std::int32_t first_probe = 0;
                std::int32_t relight_count = total;
                if (!force_full)
                {
                    first_probe = static_cast<std::int32_t>(relight_offset_) % total;
                    relight_count = std::min(total / RELIGHT_REFRESH_FRAMES, total);
                    relight_count = std::max(relight_count, 1);
                    relight_offset_ =
                        (relight_offset_ + static_cast<std::uint32_t>(relight_count)) %
                        static_cast<std::uint32_t>(total);
                }
                else
                {
                    relight_offset_ = 0;
                }
                has_relit_ = true;
                for (std::int32_t cascade = 0; cascade < GI_NUM_CASCADES; ++cascade)
                    for (int axis = 0; axis < 3; ++axis)
                        last_center_cell_[cascade][axis] = center[cascade][axis];
                last_sun_[0] = sun[0];
                last_sun_[1] = sun[1];
                last_sun_[2] = sun[2];
                last_sun_intensity_ = sun_intensity;

                const VkSampler sampler = inputs.frame->samplers->get(Resources::SamplerDesc{});

                // Populate both clipmaps. Every voxel is rewritten, so the old contents are
                // discarded (UNDEFINED old layout).
                image_barrier(cmd, clipmap_, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                              VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                              VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                image_barrier(cmd, emissive_, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_GENERAL,
                              VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT, VK_ACCESS_2_NONE,
                              VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                {
                    const VkDescriptorSet set = inputs.frame->descriptors->allocate(populate_layout_);
                    Resources::DescriptorWriter writer;
                    writer.storage_image(0, clipmap_view_);
                    writer.storage_buffer(1, primitive_buffers_[ring],
                                          sizeof(SdfPrimitive) * MAX_SDF_PRIMITIVES);
                    writer.uniform_buffer(2, clip_config_buffers_[ring], sizeof(SdfClipmapConfig));
                    writer.storage_buffer(3, mesh_buffers_[ring],
                                          sizeof(SdfMeshInstance) * MAX_SDF_MESH_INSTANCES);
                    writer.storage_buffer(4, brick_atlas_,
                                          sizeof(float) *
                                              static_cast<VkDeviceSize>(SDF_BRICK_VOXELS) *
                                              MAX_SDF_BRICKS);
                    writer.storage_image(5, emissive_view_);
                    writer.update(device_.device(), set);
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, populate_pipeline_);
                    Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   populate_pipeline_layout_, 0, set);
                    const std::uint32_t dispatch =
                        groups(static_cast<std::uint32_t>(SDF_CLIPMAP_RESOLUTION), POPULATE_GROUP);
                    vkCmdDispatch(cmd, dispatch, dispatch, dispatch);
                }
                // Both clipmaps stay in GENERAL and become sampled-readable for the trace.
                image_barrier(cmd, clipmap_, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                              VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                image_barrier(cmd, emissive_, VK_IMAGE_LAYOUT_GENERAL, VK_IMAGE_LAYOUT_GENERAL,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                              VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                              VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);

                // Trace the probes against the clipmap.
                {
                    const VkDescriptorSet set = inputs.frame->descriptors->allocate(relight_layout_);
                    Resources::DescriptorWriter writer;
                    writer.storage_buffer(0, inputs.probe_sh, inputs.probe_sh_bytes);
                    writer.storage_buffer(1, inputs.environment_sh, inputs.environment_sh_bytes);
                    writer.sampled_image(2, clipmap_view_, sampler);
                    writer.uniform_buffer(3, probe_config_buffers_[ring], sizeof(ProbeVolumeConfig));
                    writer.uniform_buffer(4, clip_config_buffers_[ring], sizeof(SdfClipmapConfig));
                    writer.sampled_image(5, emissive_view_, sampler);
                    writer.update(device_.device(), set);

                    RelightPush push{first_probe, relight_count, RAY_COUNT, MAX_TRACE_DISTANCE};
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, relight_pipeline_);
                    Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   relight_pipeline_layout_, 0, set);
                    vkCmdPushConstants(cmd, relight_pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                       sizeof(RelightPush), &push);
                    vkCmdDispatch(cmd, groups(static_cast<std::uint32_t>(relight_count),
                                              RELIGHT_GROUP),
                                  1, 1);
                }
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
