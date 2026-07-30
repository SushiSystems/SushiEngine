/**************************************************************************/
/* atmosphere_nest.cpp                                                    */
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

#include "atmosphere/atmosphere_nest.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

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
        namespace Atmosphere
        {
            namespace
            {
                /**
                 * @brief One stage of the step: its module, its bindings, and its push size.
                 *
                 * `bindings` is one character per binding index, in order, from binding 0:
                 * `u` a uniform buffer, `s` a sampled image, `i` a storage image, `b` a storage
                 * buffer. Every stage's binding 0 is the parameter block. A table rather than ten
                 * hand-written layout blocks because that is genuinely all the difference between
                 * them, and ten near-identical blocks is ten places for a binding index to drift.
                 */
                struct StageDesc
                {
                    const char* module;
                    const char* bindings;
                    std::uint32_t push_size;
                };

                enum Stage : std::uint32_t
                {
                    STAGE_SHIFT = 0,
                    STAGE_ADVECT_VELOCITY,
                    STAGE_ADVECT_SCALARS,
                    STAGE_SURFACE,
                    STAGE_FORCES,
                    STAGE_DIVERGENCE,
                    STAGE_PRESSURE,
                    STAGE_PROJECT,
                    STAGE_MICROPHYSICS,
                    STAGE_EXTINCTION,
                    STAGE_READBACK,
                };

                struct PressurePush
                {
                    std::uint32_t colour;
                };

                struct ReadbackPush
                {
                    std::int32_t mirror_cells;
                };

                struct ShiftPushBlock
                {
                    std::int32_t shift[2];
                    float origin_rel[2];
                    float forcing_scale[2];
                    float forcing_offset[2];
                };

                // Lattice cells before the thermal seed's field repeats. **Mirrors
                // `NEST_SEED_WRAP` in atmosphere_nest_common.glsl**, which masks the lattice index
                // by it; the host wraps the world origin by the same multiple of the lattice
                // spacing, and the two together are what make the wrap invisible. Neither is
                // edited alone.
                constexpr int SEED_WRAP = 256;

                struct ForcePushBlock
                {
                    float origin_rel[2];
                    float forcing_scale[2];
                    float forcing_offset[2];
                };

                /** @brief `atmosphere_surface.comp`'s push block. */
                struct SurfacePushBlock
                {
                    float seed_origin[2];
                    float seed_seconds;
                };

                const StageDesc STAGES[] = {
                    {"atmosphere_shift.comp", "usssssiiiiissi", sizeof(ShiftPushBlock)},
                    {"atmosphere_advect_velocity.comp", "usssiii", 0},
                    {"atmosphere_advect_scalars.comp", "usssssii", 0},
                    {"atmosphere_surface.comp", "uisssss", sizeof(SurfacePushBlock)},
                    {"atmosphere_forces.comp", "uiiiiisis", sizeof(ForcePushBlock)},
                    {"atmosphere_divergence.comp", "usssi", 0},
                    {"atmosphere_pressure.comp", "usi", sizeof(PressurePush)},
                    {"atmosphere_project.comp", "usiii", 0},
                    {"atmosphere_microphysics.comp", "uiii", 0},
                    {"atmosphere_extinction.comp", "usiis", 0},
                    {"atmosphere_readback.comp", "ussssssssbbs", sizeof(ReadbackPush)},
                };

                static_assert(sizeof(STAGES) / sizeof(STAGES[0]) == 11,
                              "the stage table and AtmosphereNest::STAGE_COUNT must agree");

                constexpr VkFormat SCALAR_FORMAT = VK_FORMAT_R32_SFLOAT;
                // Full floats, and the reason is a measurement rather than a preference.
                //
                // Half floats were chosen for the range — mixing ratios are a few grams per
                // kilogram and never approach fp16's ceiling. What decides the format is not the
                // range but the *ratio of a step's tendency to the value it lands on*. The
                // surface latent flux adds 1.3e-3 g/kg per step to a surface layer holding 7.27
                // g/kg, where fp16's spacing is 3.9e-3 g/kg: the increment is a third of one
                // unit in the last place, so every step rounded straight back to where it
                // started and the boundary layer never moistened at all. Measured, not deduced —
                // `atmosphere_probe` reported 7.2695 g/kg unchanged after three thousand steps,
                // which is the value an emulation of the same rounding predicts exactly.
                //
                // The consequence was the whole of the phase's symptom: with vapour frozen and
                // potential temperature (fp32) accumulating heat normally, relative humidity fell
                // under warming alone, the lifting condensation level receded, and no column
                // could ever condense however long it ran.
                //
                // fp32 leaves the increment at some 2 700 units in the last place. The
                // memory-cheaper alternative is to store vapour as a departure from the base
                // state, as `theta` does, which puts the stored magnitude near zero where fp16's
                // spacing is tiny; it needs the base-state transport term in the advection and
                // gives the four channels of one texture two different conventions, so it is
                // recorded as the refinement rather than taken here.
                constexpr VkFormat MOISTURE_FORMAT = VK_FORMAT_R32G32B32A32_SFLOAT;
                constexpr VkFormat EXTINCTION_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                // The surface state. Full floats because the skin temperature is ~290 K and its
                // step is a hundredth of a kelvin: exactly the tendency-to-value ratio that froze
                // the moisture field when it was fp16, and the same arithmetic gives the same
                // answer here.
                constexpr VkFormat SURFACE_FORMAT = VK_FORMAT_R32G32B32A32_SFLOAT;
                // The per-column cloud the surface energy balance shades itself with: an optical
                // depth and the cover it is spread over. Two channels, and full floats for the
                // same reason as everything else on this seam -- 147 KB either way at the shipped
                // tier, so precision is free.
                constexpr VkFormat SHADE_FORMAT = VK_FORMAT_R32G32_SFLOAT;
                // Full floats for the parent solution: it is 64x64 texels, so the format costs
                // 256 KB, and half floats would need a packer on the host for no gain at all.
                constexpr VkFormat FORCING_FORMAT = VK_FORMAT_R32G32B32A32_SFLOAT;

                constexpr std::uint32_t GROUP_3D = 4;
                constexpr std::uint32_t GROUP_2D = 8;

                std::uint32_t groups(std::uint32_t extent, std::uint32_t size) noexcept
                {
                    return (extent + size - 1) / size;
                }

                /**
                 * @brief A blanket compute-to-compute memory barrier between two dispatches.
                 *
                 * Deliberately not per-image. The step is ten dispatches over a dozen images with
                 * a ping-pong in the middle; hand-tracking which subresource each stage read and
                 * wrote would be a dozen barriers per step and a dozen chances to get one wrong,
                 * to save time on a dispatch chain that runs once every couple of seconds of game
                 * time. Every image lives in GENERAL for the nest's whole lifetime, so there are
                 * no layout transitions to schedule either.
                 */
                /**
                 * @brief Brackets a named section of the recording with GPU timestamps.
                 *
                 * Scoped rather than a begin/end pair because the sections of `record_step` are
                 * already braced blocks, and an early `return` between a hand-written pair would
                 * leave a query written but never read — which reports as a wildly wrong duration
                 * on the *next* frame to reuse the slot rather than as a missing one.
                 */
                class TimedSection
                {
                    public:
                        TimedSection(Graph::GpuProfiler* profiler, VkCommandBuffer cmd,
                                     const char* name)
                            : profiler_(profiler), cmd_(cmd),
                              timer_(profiler != nullptr
                                         ? profiler->begin_pass(cmd, name)
                                         : Graph::GpuProfiler::INVALID_TIMER)
                        {
                        }
                        ~TimedSection()
                        {
                            if (profiler_ != nullptr)
                                profiler_->end_pass(cmd_, timer_);
                        }

                        TimedSection(const TimedSection&) = delete;
                        TimedSection& operator=(const TimedSection&) = delete;

                    private:
                        Graph::GpuProfiler* profiler_;
                        VkCommandBuffer cmd_;
                        std::uint32_t timer_;
                };

                void barrier(VkCommandBuffer cmd)
                {
                    VkMemoryBarrier2 memory{};
                    memory.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                    memory.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                    memory.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                    memory.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                    memory.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_SAMPLED_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.memoryBarrierCount = 1;
                    dependency.pMemoryBarriers = &memory;
                    vkCmdPipelineBarrier2(cmd, &dependency);
                }
            } // namespace

            AtmosphereNest::AtmosphereNest(Vulkan::VulkanDevice& device,
                                           Resources::ShaderLibrary& shaders,
                                           Resources::GraphicsPipelineFactory& pipelines,
                                           Resources::SamplerCache& samplers,
                                           const AtmosphereNestSize& size)
                : device_(device), shaders_(shaders), pipelines_(pipelines), size_(size)
            {
                // Edge-clamped: a stencil or a semi-Lagrangian trace that leaves the domain reads
                // the boundary cell, which is where the Davies zone has already nudged the
                // solution toward the parent — so the clamp returns the most nearly correct value
                // available rather than wrapping into the far side of the domain.
                Resources::SamplerDesc sampler_desc{};
                sampler_desc.filter = VK_FILTER_LINEAR;
                sampler_desc.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_ = samplers.get(sampler_desc);

                mirror_columns_.resize(std::size_t(ATMOSPHERE_MIRROR_CELLS) *
                                       std::size_t(ATMOSPHERE_MIRROR_CELLS));
                profile_levels_.resize(std::size_t(ATMOSPHERE_PROFILE_MAX_LEVELS));

                create_volumes();
                create_buffers();
                create_layouts();
                create_pipelines();
                create_commands();

                // One pool per in-flight slot, sized to the sections `record_step` brackets. The
                // profiler reports nothing at all on a device without timestamp support, which
                // is the honest outcome — `step_cost().measured` stays false and the panel says
                // so rather than displaying zeros as though they were a fast step.
                profiler_ = std::make_unique<Graph::GpuProfiler>(
                    device_, MIRROR_SLOTS, std::uint32_t(ATMOSPHERE_TIMED_STAGES));
            }

            AtmosphereNest::~AtmosphereNest()
            {
                if (timeline_ != VK_NULL_HANDLE)
                {
                    // The last step may still be in flight; every resource below is one it could
                    // be reading.
                    VkSemaphoreWaitInfo wait{};
                    wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
                    wait.semaphoreCount = 1;
                    wait.pSemaphores = &timeline_;
                    wait.pValues = &timeline_value_;
                    vkWaitSemaphores(device_.device(), &wait, UINT64_MAX);
                }
                destroy_commands();
                destroy_pipelines();
                destroy_layouts();
                destroy_buffers();
                destroy_volumes();
            }

            void AtmosphereNest::create_volume(Volume& volume, VkFormat format,
                                               std::uint32_t width, std::uint32_t height,
                                               std::uint32_t depth)
            {
                const bool three_dimensional = depth > 1;

                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = three_dimensional ? VK_IMAGE_TYPE_3D : VK_IMAGE_TYPE_2D;
                image_info.format = format;
                image_info.extent = {width, height, depth};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_STORAGE_BIT | VK_IMAGE_USAGE_SAMPLED_BIT |
                                   VK_IMAGE_USAGE_TRANSFER_DST_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc,
                                             &volume.image, &volume.allocation, nullptr),
                              "vmaCreateImage(atmosphere nest)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = volume.image;
                view_info.viewType = three_dimensional ? VK_IMAGE_VIEW_TYPE_3D
                                                       : VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = format;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                                &volume.view),
                              "vkCreateImageView(atmosphere nest)");
            }

            void AtmosphereNest::create_volumes()
            {
                const std::uint32_t x = size_.cells_x;
                const std::uint32_t y = size_.levels;
                const std::uint32_t z = size_.cells_z;

                const auto field = [&](Buffered& buffered, VkFormat format)
                {
                    create_volume(buffered.front, format, x, y, z);
                    create_volume(buffered.back, format, x, y, z);
                };
                // Double-buffered because semi-Lagrangian advection reads the whole field while
                // writing the whole field; single-buffering it would have every cell racing its
                // own upstream neighbours.
                field(wind_x_, SCALAR_FORMAT);
                field(wind_y_, SCALAR_FORMAT);
                field(wind_z_, SCALAR_FORMAT);
                field(theta_, SCALAR_FORMAT);
                field(moisture_, MOISTURE_FORMAT);

                // Single-buffered: the pressure solve is an in-place relaxation (red-black is
                // what makes that safe), and the divergence and extinction are written once and
                // read once.
                create_volume(pressure_, SCALAR_FORMAT, x, y, z);
                create_volume(divergence_, SCALAR_FORMAT, x, y, z);
                create_volume(extinction_, EXTINCTION_FORMAT, x, y, z);
                create_volume(surface_rain_, SCALAR_FORMAT, x, z, 1);
                // 2-D and double-buffered: see the member's note. 147 KB a copy at the shipped
                // tier, which is why it is a vec4 carrying the diagnosed fluxes beside the state
                // rather than a scalar plus three more bindings everywhere.
                create_volume(surface_.front, SURFACE_FORMAT, x, z, 1);
                create_volume(surface_.back, SURFACE_FORMAT, x, z, 1);
                create_volume(cloud_shade_, SHADE_FORMAT, x, z, 1);
                // Its own image rather than a fifth channel of the one below, because there is
                // no fifth channel: the parent's four are full. 16 KB, and the alternative is
                // widening every reader of a format that four other things already agree on.
                create_volume(forcing_vertical_, SCALAR_FORMAT, ATMOSPHERE_FORCING_MAX_CELLS,
                              ATMOSPHERE_FORCING_MAX_CELLS, 1);
                create_volume(forcing_, FORCING_FORMAT, ATMOSPHERE_FORCING_MAX_CELLS,
                              ATMOSPHERE_FORCING_MAX_CELLS, 1);
            }

            void AtmosphereNest::destroy_volumes()
            {
                Volume* volumes[] = {&wind_x_.front, &wind_x_.back, &wind_y_.front, &wind_y_.back,
                                     &wind_z_.front, &wind_z_.back, &theta_.front, &theta_.back,
                                     &moisture_.front, &moisture_.back, &pressure_, &divergence_,
                                     &extinction_, &surface_rain_, &surface_.front,
                                     &surface_.back, &cloud_shade_, &forcing_,
                                     &forcing_vertical_};
                for (Volume* volume : volumes)
                {
                    if (volume->view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), volume->view, nullptr);
                    if (volume->image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), volume->image, volume->allocation);
                    volume->view = VK_NULL_HANDLE;
                    volume->image = VK_NULL_HANDLE;
                    volume->allocation = VK_NULL_HANDLE;
                }
            }

            void AtmosphereNest::create_buffers()
            {
                const auto make = [&](VkBuffer& buffer, VmaAllocation& allocation, void** mapped,
                                      VkDeviceSize size, VkBufferUsageFlags usage, bool host)
                {
                    VkBufferCreateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    info.size = size;
                    info.usage = usage;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    if (host)
                        alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                      VMA_ALLOCATION_CREATE_MAPPED_BIT;

                    VmaAllocationInfo info_out{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &info, &alloc, &buffer,
                                                  &allocation, &info_out),
                                  "vmaCreateBuffer(atmosphere nest)");
                    if (mapped != nullptr)
                        *mapped = info_out.pMappedData;
                };

                make(params_, params_allocation_, &params_mapped_, sizeof(NestParams),
                     VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT, true);

                const VkDeviceSize mirror_bytes = sizeof(AtmosphereMirrorColumn) *
                                                  std::size_t(ATMOSPHERE_MIRROR_CELLS) *
                                                  std::size_t(ATMOSPHERE_MIRROR_CELLS);
                make(mirror_, mirror_allocation_, nullptr, mirror_bytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false);
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                    make(mirror_slots_[slot].buffer, mirror_slots_[slot].allocation,
                         &mirror_slots_[slot].mapped, mirror_bytes,
                         VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);

                // A buffer of its own rather than a second region of the mirror's: the
                // descriptor writer binds from offset zero, and widening it to carry an offset
                // for this one caller would be a worse trade than a 4 KB allocation.
                const VkDeviceSize profile_bytes = sizeof(AtmosphereProfileLevel) *
                                                   std::size_t(ATMOSPHERE_PROFILE_MAX_LEVELS);
                make(profile_, profile_allocation_, nullptr, profile_bytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT, false);
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                    make(mirror_slots_[slot].profile_buffer,
                         mirror_slots_[slot].profile_allocation,
                         &mirror_slots_[slot].profile_mapped, profile_bytes,
                         VK_BUFFER_USAGE_TRANSFER_DST_BIT, true);

                // One allocation for both forcing images: the four-channel parent solution
                // followed by the single-channel vertical motion, copied out as two regions of
                // the same buffer. Five floats a texel over 64x64 is 80 KB.
                make(forcing_staging_, forcing_staging_allocation_, &forcing_staging_mapped_,
                     VkDeviceSize(ATMOSPHERE_FORCING_MAX_CELLS) * ATMOSPHERE_FORCING_MAX_CELLS *
                         5 * sizeof(float),
                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT, true);
            }

            void AtmosphereNest::destroy_buffers()
            {
                const auto drop = [&](VkBuffer& buffer, VmaAllocation& allocation)
                {
                    if (buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), buffer, allocation);
                    buffer = VK_NULL_HANDLE;
                    allocation = VK_NULL_HANDLE;
                };
                drop(params_, params_allocation_);
                drop(mirror_, mirror_allocation_);
                drop(profile_, profile_allocation_);
                drop(forcing_staging_, forcing_staging_allocation_);
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                {
                    drop(mirror_slots_[slot].buffer, mirror_slots_[slot].allocation);
                    drop(mirror_slots_[slot].profile_buffer,
                         mirror_slots_[slot].profile_allocation);
                }
            }

            void AtmosphereNest::create_layouts()
            {
                std::uint32_t uniform_count = 0;
                std::uint32_t sampled_count = 0;
                std::uint32_t storage_image_count = 0;
                std::uint32_t storage_buffer_count = 0;

                for (std::uint32_t stage = 0; stage < STAGE_COUNT; ++stage)
                {
                    const StageDesc& desc = STAGES[stage];
                    VkDescriptorSetLayoutBinding bindings[16]{};
                    std::uint32_t count = 0;
                    for (const char* c = desc.bindings; *c != '\0'; ++c, ++count)
                    {
                        bindings[count].binding = count;
                        bindings[count].descriptorCount = 1;
                        bindings[count].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                        switch (*c)
                        {
                        case 'u':
                            bindings[count].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                            ++uniform_count;
                            break;
                        case 's':
                            bindings[count].descriptorType =
                                VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                            ++sampled_count;
                            break;
                        case 'i':
                            bindings[count].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_IMAGE;
                            ++storage_image_count;
                            break;
                        default:
                            bindings[count].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                            ++storage_buffer_count;
                            break;
                        }
                    }

                    VkDescriptorSetLayoutCreateInfo layout_info{};
                    layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                    layout_info.bindingCount = count;
                    layout_info.pBindings = bindings;
                    Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info,
                                                              nullptr, &layouts_[stage]),
                                  "vkCreateDescriptorSetLayout(atmosphere nest)");

                    VkPushConstantRange range{};
                    range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                    range.size = desc.push_size;

                    VkPipelineLayoutCreateInfo pipeline_info{};
                    pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                    pipeline_info.setLayoutCount = 1;
                    pipeline_info.pSetLayouts = &layouts_[stage];
                    pipeline_info.pushConstantRangeCount = desc.push_size > 0 ? 1u : 0u;
                    pipeline_info.pPushConstantRanges = &range;
                    Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                         &pipeline_layouts_[stage]),
                                  "vkCreatePipelineLayout(atmosphere nest)");
                }

                // Every stage of every step allocates a set from the recording slot's pool, and
                // the pressure relaxation allocates two per sweep — so the capacity here is what
                // bounds how many steps a frame may record, and `step()` derives that bound from
                // this number rather than assuming it fits.
                const std::uint32_t sets = DESCRIPTOR_SETS_PER_SLOT;
                VkDescriptorPoolSize sizes[4]{};
                sizes[0] = {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, uniform_count * sets};
                sizes[1] = {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, sampled_count * sets};
                sizes[2] = {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, storage_image_count * sets};
                sizes[3] = {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,
                            std::max(storage_buffer_count, 1u) * sets};

                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.maxSets = sets;
                pool_info.poolSizeCount = 4;
                pool_info.pPoolSizes = sizes;
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                    Vulkan::check(vkCreateDescriptorPool(device_.device(), &pool_info, nullptr,
                                                         &descriptor_pools_[slot]),
                                  "vkCreateDescriptorPool(atmosphere nest)");
            }

            void AtmosphereNest::destroy_layouts()
            {
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                {
                    if (descriptor_pools_[slot] != VK_NULL_HANDLE)
                        vkDestroyDescriptorPool(device_.device(), descriptor_pools_[slot], nullptr);
                    descriptor_pools_[slot] = VK_NULL_HANDLE;
                }
                for (std::uint32_t stage = 0; stage < STAGE_COUNT; ++stage)
                {
                    if (pipeline_layouts_[stage] != VK_NULL_HANDLE)
                        vkDestroyPipelineLayout(device_.device(), pipeline_layouts_[stage],
                                                nullptr);
                    if (layouts_[stage] != VK_NULL_HANDLE)
                        vkDestroyDescriptorSetLayout(device_.device(), layouts_[stage], nullptr);
                    pipeline_layouts_[stage] = VK_NULL_HANDLE;
                    layouts_[stage] = VK_NULL_HANDLE;
                }
            }

            void AtmosphereNest::create_pipelines()
            {
                for (std::uint32_t stage = 0; stage < STAGE_COUNT; ++stage)
                    stage_pipelines_[stage] = pipelines_.create_compute(
                        pipeline_layouts_[stage], shaders_.module(STAGES[stage].module));
            }

            void AtmosphereNest::destroy_pipelines()
            {
                for (std::uint32_t stage = 0; stage < STAGE_COUNT; ++stage)
                {
                    if (stage_pipelines_[stage] != VK_NULL_HANDLE)
                        vkDestroyPipeline(device_.device(), stage_pipelines_[stage], nullptr);
                    stage_pipelines_[stage] = VK_NULL_HANDLE;
                }
            }

            void AtmosphereNest::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
                // A shader edit changes the physics; re-seed rather than carry a state the new
                // code never produced.
                seeded_ = false;
            }

            void AtmosphereNest::create_commands()
            {
                VkCommandPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                pool_info.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
                pool_info.queueFamilyIndex = device_.graphics_queue_family();
                Vulkan::check(vkCreateCommandPool(device_.device(), &pool_info, nullptr,
                                                  &command_pool_),
                              "vkCreateCommandPool(atmosphere nest)");

                VkCommandBufferAllocateInfo allocate{};
                allocate.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                allocate.commandPool = command_pool_;
                allocate.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                allocate.commandBufferCount = MIRROR_SLOTS;
                Vulkan::check(vkAllocateCommandBuffers(device_.device(), &allocate, commands_),
                              "vkAllocateCommandBuffers(atmosphere nest)");

                // A timeline rather than a fence and a binary semaphore: three scene views each
                // need to wait on the same step, and a binary semaphore can only be waited once.
                VkSemaphoreTypeCreateInfo type{};
                type.sType = VK_STRUCTURE_TYPE_SEMAPHORE_TYPE_CREATE_INFO;
                type.semaphoreType = VK_SEMAPHORE_TYPE_TIMELINE;
                type.initialValue = 0;

                VkSemaphoreCreateInfo semaphore_info{};
                semaphore_info.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
                semaphore_info.pNext = &type;
                Vulkan::check(vkCreateSemaphore(device_.device(), &semaphore_info, nullptr,
                                                &timeline_),
                              "vkCreateSemaphore(atmosphere nest timeline)");
            }

            void AtmosphereNest::destroy_commands()
            {
                if (timeline_ != VK_NULL_HANDLE)
                    vkDestroySemaphore(device_.device(), timeline_, nullptr);
                if (command_pool_ != VK_NULL_HANDLE)
                    vkDestroyCommandPool(device_.device(), command_pool_, nullptr);
                timeline_ = VK_NULL_HANDLE;
                command_pool_ = VK_NULL_HANDLE;
            }

            void AtmosphereNest::origin(double& x, double& z) const noexcept
            {
                x = double(origin_cell_x_) * double(size_.spacing_m);
                z = double(origin_cell_z_) * double(size_.spacing_m);
            }

            float AtmosphereNest::choose_step(const AtmosphereParameters& parameters,
                                              const AtmosphereForcing& forcing) const
            {
                // The horizontal CFL against the fastest wind the parent solution carries — the
                // one bound on flow speed the host actually knows without reading the GPU back.
                // Convection adds to it, so a headroom of the convective scale is included rather
                // than discovered by the solve going unstable.
                float fastest = 0.0f;
                if (forcing.valid())
                {
                    const std::size_t count = std::size_t(forcing.cells_x) *
                                              std::size_t(forcing.cells_z);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        const AtmosphereForcingSample& sample = forcing.samples[i];
                        fastest = std::max(fastest, std::hypot(sample.wind_east_mps,
                                                               sample.wind_north_mps));
                    }
                }
                fastest = std::max(fastest + 4.0f * parameters.convective_velocity_scale, 1.0f);

                const float horizontal = size_.spacing_m / fastest;

                // The vertical CFL, against the updraft the nest is **actually producing**.
                //
                // This used to divide by a flat `10 × convective_velocity_scale` — a
                // thunderstorm core, 20 m/s — and it bound the step at 2.43 s on every grid and
                // in every airmass, three times tighter than the horizontal term ever came to.
                // Measured, a fair-weather domain peaks at 0.02 m/s and a convecting one at
                // 0.03: the assumption was three orders out, and since the transport is
                // semi-Lagrangian and therefore unconditionally stable, what it bought was not
                // stability but a vertical Courant number of 9 × 10⁻⁴ — which is §1.4's
                // *maximally diffusive* regime, the exact defect this nest was built to leave
                // behind. A longer step here is both cheaper and more accurate.
                //
                // So the bound is taken against the readback's own domain peak, with a fourfold
                // headroom for what convection can do between one readback and the next, and the
                // old constant stands until the first readback has something to say. What that
                // buys is a step that is long when nothing is happening and tightens by itself
                // when a storm gets going, rather than one permanently sized for the storm.
                //
                // Measured end-to-end at 2.44 / 5 / 6 / 10 / 20 s: the thermodynamics is
                // unchanged through 6 s in both a quiescent and a convecting airmass (sky
                // coverage within 3 %, cloud base within 2 %), and at 10 s and beyond the domain
                // peak vertical velocity inflates by an order of magnitude and the sky changes
                // with it. `max_step_seconds` is the cap that keeps it the safe side of that.
                const float thinnest = atmosphere_level_thickness(0, size_.levels, size_.top_m);
                const float updraft =
                    measured_updraft_ > 0.0f
                        ? std::max(4.0f * measured_updraft_, 1.0f)
                        : std::max(10.0f * parameters.convective_velocity_scale, 1.0f);
                const float vertical = thinnest / updraft;

                const float cfl = parameters.courant_target * std::min(horizontal, vertical);
                return std::clamp(cfl, parameters.min_step_seconds, parameters.max_step_seconds);
            }

            void AtmosphereNest::upload_parameters(const AtmosphereParameters& p, float dt)
            {
                if (params_mapped_ == nullptr)
                    return;
                NestParams block{};
                block.gas_constant_dry = p.gas_constant_dry;
                block.gas_constant_vapour = p.gas_constant_vapour;
                block.specific_heat_pressure = p.specific_heat_pressure;
                block.latent_heat_vaporization = p.latent_heat_vaporization;
                block.gravity = p.gravity;
                block.reference_pressure = p.reference_pressure;
                block.water_density = p.water_density;
                block.surface_temperature = p.surface_temperature;
                block.surface_pressure = p.surface_pressure;
                block.lapse_rate = p.lapse_rate;
                block.tropopause_altitude = p.tropopause_altitude;
                block.surface_humidity = p.surface_humidity;
                block.humidity_scale_height = p.humidity_scale_height;
                block.free_troposphere_drying = p.free_troposphere_drying;
                block.free_troposphere_exponent = p.free_troposphere_exponent;
                block.eddy_viscosity = p.eddy_viscosity;
                block.boundary_layer_depth = p.boundary_layer_depth_m;
                block.boundary_layer_velocity_scale = p.boundary_layer_velocity_scale;
                block.sponge_depth = p.sponge_depth;
                block.sponge_rate = p.sponge_rate;
                block.boundary_relaxation = p.boundary_relaxation;
                block.thermal_seed_amplitude = p.thermal_seed_amplitude;
                block.thermal_seed_length = p.thermal_seed_length_m;
                block.thermal_seed_period = p.thermal_seed_period_s;
                block.convective_velocity_scale = p.convective_velocity_scale;
                block.cloud_top_longwave_flux = p.cloud_top_longwave_flux;
                block.cloud_water_absorption = p.cloud_water_absorption;
                block.cloud_critical_humidity = p.cloud_critical_humidity;
                block.autoconversion_rate = p.autoconversion_rate;
                block.autoconversion_threshold = p.autoconversion_threshold;
                block.accretion_rate = p.accretion_rate;
                block.accretion_exponent = p.accretion_exponent;
                block.rain_evaporation_rate = p.rain_evaporation_rate;
                block.fall_speed_coefficient = p.fall_speed_coefficient;
                block.fall_speed_exponent = p.fall_speed_exponent;
                block.droplet_effective_radius = p.droplet_effective_radius;
                block.latent_heat_fusion = p.latent_heat_fusion;
                block.freezing_temperature = p.freezing_temperature;
                block.glaciation_temperature = p.glaciation_temperature;
                block.snow_fall_speed_coefficient = p.snow_fall_speed_coefficient;
                block.snow_fall_speed_exponent = p.snow_fall_speed_exponent;
                block.glaciated_autoconversion_factor = p.glaciated_autoconversion_factor;
                block.ice_effective_radius = p.ice_effective_radius;
                block.solar_constant = p.solar_constant;
                block.clear_sky_transmittance = p.clear_sky_transmittance;
                block.surface_albedo = p.surface_albedo;
                block.surface_emissivity = p.surface_emissivity;
                block.surface_heat_capacity = p.surface_heat_capacity;
                block.surface_moisture_availability = p.surface_moisture_availability;
                block.surface_exchange_coefficient = p.surface_exchange_coefficient;
                block.surface_minimum_wind = p.surface_minimum_wind;
                block.solar_elevation_sine = solar_elevation_sine_;
                block.spacing = size_.spacing_m;
                block.domain_top = size_.top_m;
                block.dt = dt;
                block.cells_x = std::int32_t(size_.cells_x);
                block.cells_z = std::int32_t(size_.cells_z);
                block.levels = std::int32_t(size_.levels);
                block.boundary_zone = std::int32_t(p.boundary_zone_cells);
                // No `elapsed` and no `step_index`. Both were here for the thermal seed and both
                // were dead: a frame records several steps against one upload of this block, so
                // neither can distinguish them, and the seed takes its time through the per-step
                // push block instead.
                // Coriolis rides the forcing rather than the authored parameters: it is a
                // property of *where the nest is*, and the simulation is the only party that
                // knows the observer's latitude. Held on the object beside the solar sine for
                // the same reason — the parameter block is uploaded once per step and this is
                // one of its fields, not something to thread through every record call.
                block.coriolis = coriolis_;
                std::memcpy(params_mapped_, &block, sizeof(NestParams));
            }

            VkDescriptorSet AtmosphereNest::allocate(std::uint32_t stage, std::uint32_t slot)
            {
                VkDescriptorSetAllocateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                info.descriptorPool = descriptor_pools_[slot];
                info.descriptorSetCount = 1;
                info.pSetLayouts = &layouts_[stage];
                VkDescriptorSet set = VK_NULL_HANDLE;
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &info, &set),
                              "vkAllocateDescriptorSets(atmosphere nest)");
                return set;
            }

            void AtmosphereNest::prepare_layouts(VkCommandBuffer cmd)
            {
                if (layouts_ready_)
                    return;
                layouts_ready_ = true;

                // Every image spends its whole life in GENERAL: it is read and written by compute
                // in the same step, and sampled by the render tier afterwards, so there is no
                // layout a transition could usefully move it to. One barrier at birth, none
                // after.
                VkImage images[] = {wind_x_.front.image, wind_x_.back.image,
                                    wind_y_.front.image, wind_y_.back.image,
                                    wind_z_.front.image, wind_z_.back.image,
                                    theta_.front.image, theta_.back.image,
                                    moisture_.front.image, moisture_.back.image,
                                    pressure_.image, divergence_.image, extinction_.image,
                                    surface_rain_.image, surface_.front.image,
                                    surface_.back.image, cloud_shade_.image, forcing_.image,
                                    forcing_vertical_.image};

                VkImageMemoryBarrier2 barriers[sizeof(images) / sizeof(images[0])]{};
                for (std::size_t i = 0; i < sizeof(images) / sizeof(images[0]); ++i)
                {
                    barriers[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barriers[i].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                    barriers[i].dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                               VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT;
                    barriers[i].dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT |
                                                VK_ACCESS_2_TRANSFER_WRITE_BIT;
                    barriers[i].oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                    barriers[i].newLayout = VK_IMAGE_LAYOUT_GENERAL;
                    barriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barriers[i].image = images[i];
                    barriers[i].subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    barriers[i].subresourceRange.levelCount = 1;
                    barriers[i].subresourceRange.layerCount = 1;
                }

                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.imageMemoryBarrierCount =
                    std::uint32_t(sizeof(images) / sizeof(images[0]));
                dependency.pImageMemoryBarriers = barriers;
                vkCmdPipelineBarrier2(cmd, &dependency);
            }

            void AtmosphereNest::upload_forcing(VkCommandBuffer cmd,
                                                const AtmosphereForcing& forcing)
            {
                if (forcing_staging_mapped_ == nullptr)
                    return;

                auto* texels = static_cast<float*>(forcing_staging_mapped_);
                const int cells = ATMOSPHERE_FORCING_MAX_CELLS;
                // Where the single-channel vertical-motion region starts in the same buffer.
                const std::size_t vertical_base = std::size_t(cells) * std::size_t(cells) * 4;
                for (int z = 0; z < cells; ++z)
                    for (int x = 0; x < cells; ++x)
                    {
                        AtmosphereForcingSample sample{};
                        if (forcing.valid())
                        {
                            // Resampled up to the fixed image extent on the host, exactly as the
                            // weather field already is: it keeps the image, its view and its
                            // descriptor constant for the nest's whole lifetime whatever a
                            // provider chooses to publish.
                            const int sx = std::min(x * forcing.cells_x / cells,
                                                    forcing.cells_x - 1);
                            const int sz = std::min(z * forcing.cells_z / cells,
                                                    forcing.cells_z - 1);
                            sample = forcing.samples[std::size_t(sz) * std::size_t(forcing.cells_x) +
                                                     std::size_t(sx)];
                        }
                        const std::size_t texel = std::size_t(z) * std::size_t(cells) +
                                                  std::size_t(x);
                        const std::size_t offset = texel * 4;
                        texels[offset + 0] = sample.wind_east_mps;
                        texels[offset + 1] = sample.wind_north_mps;
                        texels[offset + 2] = sample.theta_anomaly_k;
                        texels[offset + 3] = sample.humidity_anomaly;
                        texels[vertical_base + texel] = sample.vertical_velocity_mps;
                    }

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {ATMOSPHERE_FORCING_MAX_CELLS, ATMOSPHERE_FORCING_MAX_CELLS, 1};
                vkCmdCopyBufferToImage(cmd, forcing_staging_, forcing_.image,
                                       VK_IMAGE_LAYOUT_GENERAL, 1, &copy);
                copy.bufferOffset = VkDeviceSize(vertical_base) * sizeof(float);
                vkCmdCopyBufferToImage(cmd, forcing_staging_, forcing_vertical_.image,
                                       VK_IMAGE_LAYOUT_GENERAL, 1, &copy);

                VkMemoryBarrier2 memory{};
                memory.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                memory.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                memory.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                memory.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                memory.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;

                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.memoryBarrierCount = 1;
                dependency.pMemoryBarriers = &memory;
                vkCmdPipelineBarrier2(cmd, &dependency);
            }

            void AtmosphereNest::record_shift(VkCommandBuffer cmd, std::int32_t shift_x,
                                              std::int32_t shift_z,
                                              const AtmosphereForcing& forcing)
            {
                ShiftPushBlock push{};
                push.shift[0] = shift_x;
                push.shift[1] = shift_z;
                double origin_x = 0.0;
                double origin_z = 0.0;
                origin(origin_x, origin_z);
                push.origin_rel[0] = static_cast<float>(origin_x - forcing.observer_x);
                push.origin_rel[1] = static_cast<float>(origin_z - forcing.observer_z);
                push.forcing_scale[0] = forcing.uv_scale_x;
                push.forcing_scale[1] = forcing.uv_scale_z;
                push.forcing_offset[0] = static_cast<float>(double(forcing.uv_offset_x) +
                                                            double(forcing.uv_scale_x) *
                                                                forcing.observer_x);
                push.forcing_offset[1] = static_cast<float>(double(forcing.uv_offset_z) +
                                                            double(forcing.uv_scale_z) *
                                                                forcing.observer_z);

                TimedSection timed(profiler_.get(), cmd, "shift");
                const VkDescriptorSet set = allocate(STAGE_SHIFT, slot_);
                Resources::DescriptorWriter writer;
                writer.uniform_buffer(0, params_, sizeof(NestParams));
                const Volume* sources[] = {&wind_x_.front, &wind_y_.front, &wind_z_.front,
                                           &theta_.front, &moisture_.front};
                const Volume* targets[] = {&wind_x_.back, &wind_y_.back, &wind_z_.back,
                                           &theta_.back, &moisture_.back};
                for (std::uint32_t i = 0; i < 5; ++i)
                    writer.sampled_image(1 + i, sources[i]->view, sampler_,
                                         VK_IMAGE_LAYOUT_GENERAL);
                for (std::uint32_t i = 0; i < 5; ++i)
                    writer.storage_image(6 + i, targets[i]->view, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(11, forcing_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(12, surface_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.storage_image(13, surface_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                writer.update(device_.device(), set);

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                  stage_pipelines_[STAGE_SHIFT]);
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               pipeline_layouts_[STAGE_SHIFT], 0, set);
                vkCmdPushConstants(cmd, pipeline_layouts_[STAGE_SHIFT],
                                   VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(push), &push);
                vkCmdDispatch(cmd, groups(size_.cells_x, GROUP_3D), groups(size_.levels, GROUP_3D),
                              groups(size_.cells_z, GROUP_3D));
                barrier(cmd);

                wind_x_.swap();
                wind_y_.swap();
                wind_z_.swap();
                theta_.swap();
                moisture_.swap();
                surface_.swap();
            }

            void AtmosphereNest::record_step(VkCommandBuffer cmd, const AtmosphereForcing& forcing,
                                             double step_seconds, bool timed)
            {
                const std::uint32_t gx = groups(size_.cells_x, GROUP_3D);
                const std::uint32_t gy = groups(size_.levels, GROUP_3D);
                const std::uint32_t gz = groups(size_.cells_z, GROUP_3D);
                // Only one step of a frame is broken into sections: the query pool holds a fixed
                // number and a frame may record several steps, so timing all of them would spend
                // the budget on the first step and a half and report a breakdown that silently
                // stopped part-way. The outer bracket still measures the whole submission, so
                // `total_ms / steps` remains the honest per-step number.
                Graph::GpuProfiler* const profiler = timed ? profiler_.get() : nullptr;

                const auto begin = [&](std::uint32_t stage, Resources::DescriptorWriter& writer)
                {
                    const VkDescriptorSet set = allocate(stage, slot_);
                    writer.uniform_buffer(0, params_, sizeof(NestParams));
                    return set;
                };
                const auto dispatch = [&](std::uint32_t stage, VkDescriptorSet set,
                                          const void* push, std::uint32_t push_size,
                                          std::uint32_t x, std::uint32_t y, std::uint32_t z)
                {
                    vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                      stage_pipelines_[stage]);
                    Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                   pipeline_layouts_[stage], 0, set);
                    if (push_size > 0)
                        vkCmdPushConstants(cmd, pipeline_layouts_[stage],
                                           VK_SHADER_STAGE_COMPUTE_BIT, 0, push_size, push);
                    vkCmdDispatch(cmd, x, y, z);
                    barrier(cmd);
                };

                // 1. Transport the momentum, then the scalars *with the transported momentum* —
                //    the order a semi-Lagrangian step wants, because the scalars should follow
                //    the flow this step ends with rather than the one it started from.
                {
                    TimedSection timed(profiler, cmd, "advect wind");
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_ADVECT_VELOCITY, writer);
                    writer.sampled_image(1, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(2, wind_y_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(3, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, wind_x_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(5, wind_y_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(6, wind_z_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_ADVECT_VELOCITY, set, nullptr, 0, gx, gy, gz);
                    wind_x_.swap();
                    wind_y_.swap();
                    wind_z_.swap();
                }
                {
                    TimedSection timed(profiler, cmd, "advect scalars");
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_ADVECT_SCALARS, writer);
                    writer.sampled_image(1, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(2, wind_y_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(3, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(4, theta_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(5, moisture_.front.view, sampler_,
                                         VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(6, theta_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(7, moisture_.back.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_ADVECT_SCALARS, set, nullptr, 0, gx, gy, gz);
                    theta_.swap();
                    moisture_.swap();
                }

                // 2. The ground, before anything reads what it hands up. One dispatch per
                //    *column* — 37 000 invocations against the 1.8 million every 3-D stage runs —
                //    solving the surface energy balance against the air the advection just
                //    delivered, which is the state it should be exchanging with.
                {
                    TimedSection timed(profiler, cmd, "surface");
                    SurfacePushBlock push{};
                    double origin_x = 0.0;
                    double origin_z = 0.0;
                    origin(origin_x, origin_z);
                    // The seed's own frame: world, not observer-relative, and wrapped in double
                    // onto a whole number of its lattice cells. Scene-absolute coordinates are
                    // planet-scale — a float32 metre at 6.4e6 m is already half a metre of
                    // quantisation — and the shader masks the lattice index by the same power of
                    // two, so the wrap is exact and leaves no seam.
                    const double lattice =
                        double(std::max(seed_length_m_, 1.0f)) * double(SEED_WRAP);
                    push.seed_origin[0] =
                        static_cast<float>(origin_x - std::floor(origin_x / lattice) * lattice);
                    push.seed_origin[1] =
                        static_cast<float>(origin_z - std::floor(origin_z / lattice) * lattice);
                    push.seed_seconds = static_cast<float>(step_seconds);

                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_SURFACE, writer);
                    writer.storage_image(1, surface_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(2, theta_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(3, moisture_.front.view, sampler_,
                                         VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(4, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(5, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(6, cloud_shade_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_SURFACE, set, &push, sizeof(push),
                             groups(size_.cells_x, GROUP_2D), groups(size_.cells_z, GROUP_2D), 1);
                }

                // 3. Everything that acts on the transported state: buoyancy, Coriolis,
                //    diffusion, the sponge, the surface fluxes the stage above solved, and the
                //    lateral relaxation.
                {
                    TimedSection timed(profiler, cmd, "forces");
                    ForcePushBlock push{};
                    double origin_x = 0.0;
                    double origin_z = 0.0;
                    origin(origin_x, origin_z);
                    push.origin_rel[0] = static_cast<float>(origin_x - forcing.observer_x);
                    push.origin_rel[1] = static_cast<float>(origin_z - forcing.observer_z);
                    push.forcing_scale[0] = forcing.uv_scale_x;
                    push.forcing_scale[1] = forcing.uv_scale_z;
                    push.forcing_offset[0] = static_cast<float>(double(forcing.uv_offset_x) +
                                                                double(forcing.uv_scale_x) *
                                                                    forcing.observer_x);
                    push.forcing_offset[1] = static_cast<float>(double(forcing.uv_offset_z) +
                                                                double(forcing.uv_scale_z) *
                                                                    forcing.observer_z);

                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_FORCES, writer);
                    writer.storage_image(1, wind_x_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, wind_y_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(3, wind_z_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, theta_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(5, moisture_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(6, forcing_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(7, surface_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(8, forcing_vertical_.view, sampler_,
                                         VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_FORCES, set, &push, sizeof(push), gx, gy, gz);
                }

                // 4. The anelastic projection: measure the mass divergence the provisional
                //    velocity carries, solve for the pressure that removes it, remove it.
                {
                    TimedSection timed(profiler, cmd, "divergence");
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_DIVERGENCE, writer);
                    writer.sampled_image(1, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(2, wind_y_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(3, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, divergence_.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_DIVERGENCE, set, nullptr, 0, gx, gy, gz);
                }
                {
                    // Every sweep under one bracket, not one bracket per sweep. What the
                    // measurement has to answer is whether the solve is the large share of the
                    // step its own header supposes, and that is a property of the whole
                    // relaxation — `pressure_iterations` is the knob, so the cost of all of the
                    // sweeps together is the number that decides whether turning it down is
                    // worth anything.
                    TimedSection timed(profiler, cmd, "pressure");
                    for (std::uint32_t sweep = 0; sweep < pressure_sweeps_; ++sweep)
                    {
                        // Red then black. The previous step's solution is left in place as this
                        // one's initial guess: the pressure field a convecting atmosphere needs
                        // changes slowly, so a warm start is worth several sweeps of iteration.
                        for (std::uint32_t colour = 0; colour < 2; ++colour)
                        {
                            PressurePush push{colour};
                            Resources::DescriptorWriter writer;
                            const VkDescriptorSet set = begin(STAGE_PRESSURE, writer);
                            writer.sampled_image(1, divergence_.view, sampler_,
                                                 VK_IMAGE_LAYOUT_GENERAL);
                            writer.storage_image(2, pressure_.view, VK_IMAGE_LAYOUT_GENERAL);
                            writer.update(device_.device(), set);
                            dispatch(STAGE_PRESSURE, set, &push, sizeof(push),
                                     groups(size_.cells_x, GROUP_2D),
                                     groups(size_.cells_z, GROUP_2D), 1);
                        }
                    }
                }
                {
                    TimedSection timed(profiler, cmd, "project");
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_PROJECT, writer);
                    writer.sampled_image(1, pressure_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, wind_x_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(3, wind_y_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, wind_z_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_PROJECT, set, nullptr, 0, gx, gy, gz);
                }

                // 5. The microphysics, on a flow that now transports mass consistently — which
                //    is the precondition for condensate to concentrate where the updraft is.
                {
                    TimedSection timed(profiler, cmd, "microphysics");
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_MICROPHYSICS, writer);
                    writer.storage_image(1, theta_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, moisture_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(3, surface_rain_.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_MICROPHYSICS, set, nullptr, 0, gx, gy, gz);
                }

            }

            void AtmosphereNest::record_clear(VkCommandBuffer cmd)
            {
                // The volumes that hold undefined memory on the seed frame. Three of them are
                // step outputs `atmosphere_shift.comp` does not write, because it seeds the
                // prognostic state — wind, theta, moisture — and on the seed frame no step has
                // run to produce the rest.
                //
                // The surface pair is here for a different reason: the shift *does* write it,
                // but it writes the target from the source, and on the seed frame the source is
                // whatever the allocator handed back. A NaN skin temperature would reach the
                // fourth power in the first longwave term and never come back.
                //
                // Pressure is the one that matters. The relaxation warm-starts from the field
                // it left behind last step, which on the first step is whatever the allocator
                // handed back: undefined contents that a red-black sweep would propagate rather
                // than overwrite, and that a single NaN in would make permanent, since every
                // subsequent step inherits it. The other two only reach the readback — a mirror
                // reporting rain out of uninitialized memory before the first step is a smaller
                // fault than a solver that never recovers, but it is the same fault.
                VkClearColorValue zero{};
                VkImageSubresourceRange range{};
                range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                range.levelCount = 1;
                range.layerCount = 1;
                const VkImage images[] = {pressure_.image, divergence_.image,
                                          surface_rain_.image, surface_.front.image,
                                          surface_.back.image, cloud_shade_.image};
                for (VkImage image : images)
                    vkCmdClearColorImage(cmd, image, VK_IMAGE_LAYOUT_GENERAL, &zero, 1, &range);

                VkMemoryBarrier2 memory{};
                memory.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                memory.srcStageMask = VK_PIPELINE_STAGE_2_CLEAR_BIT;
                memory.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                memory.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                memory.dstAccessMask = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                       VK_ACCESS_2_SHADER_SAMPLED_READ_BIT |
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.memoryBarrierCount = 1;
                dependency.pMemoryBarriers = &memory;
                vkCmdPipelineBarrier2(cmd, &dependency);
            }

            void AtmosphereNest::record_extinction(VkCommandBuffer cmd)
            {
                // What the rest of the engine sees. Recorded separately from the step because a
                // frame that only *shifts* still needs it: the shift moves the lattice, and the
                // cloudscape bake addresses this field through the nest's new origin, so leaving
                // yesterday's extinction behind a moved mapping draws the sky offset by however
                // many cells the camera travelled.
                TimedSection timed(profiler_.get(), cmd, "extinction");
                const VkDescriptorSet set = allocate(STAGE_EXTINCTION, slot_);
                Resources::DescriptorWriter writer;
                writer.uniform_buffer(0, params_, sizeof(NestParams));
                writer.sampled_image(1, moisture_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.storage_image(2, extinction_.view, VK_IMAGE_LAYOUT_GENERAL);
                writer.storage_image(3, cloud_shade_.view, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(4, theta_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.update(device_.device(), set);

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                  stage_pipelines_[STAGE_EXTINCTION]);
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               pipeline_layouts_[STAGE_EXTINCTION], 0, set);
                vkCmdDispatch(cmd, groups(size_.cells_x, GROUP_3D), groups(size_.levels, GROUP_3D),
                              groups(size_.cells_z, GROUP_3D));
                barrier(cmd);
            }

            void AtmosphereNest::record_readback(VkCommandBuffer cmd, std::uint32_t slot)
            {
                TimedSection timed(profiler_.get(), cmd, "readback");
                ReadbackPush push{ATMOSPHERE_MIRROR_CELLS};
                const VkDescriptorSet set = allocate(STAGE_READBACK, slot);
                Resources::DescriptorWriter writer;
                writer.uniform_buffer(0, params_, sizeof(NestParams));
                writer.sampled_image(1, extinction_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(2, moisture_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(3, theta_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(4, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(5, wind_y_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(6, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(7, surface_rain_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.sampled_image(8, divergence_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.storage_buffer(9, mirror_,
                                      sizeof(AtmosphereMirrorColumn) *
                                          std::size_t(ATMOSPHERE_MIRROR_CELLS) *
                                          std::size_t(ATMOSPHERE_MIRROR_CELLS));
                writer.storage_buffer(10, profile_,
                                      sizeof(AtmosphereProfileLevel) *
                                          std::size_t(ATMOSPHERE_PROFILE_MAX_LEVELS));
                writer.sampled_image(11, surface_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                writer.update(device_.device(), set);

                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                  stage_pipelines_[STAGE_READBACK]);
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               pipeline_layouts_[STAGE_READBACK], 0, set);
                vkCmdPushConstants(cmd, pipeline_layouts_[STAGE_READBACK],
                                   VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(push), &push);
                vkCmdDispatch(cmd, groups(ATMOSPHERE_MIRROR_CELLS, GROUP_2D),
                              groups(ATMOSPHERE_MIRROR_CELLS, GROUP_2D), 1);

                VkMemoryBarrier2 memory{};
                memory.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                memory.srcStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                memory.srcAccessMask = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                memory.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
                memory.dstAccessMask = VK_ACCESS_2_TRANSFER_READ_BIT;
                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.memoryBarrierCount = 1;
                dependency.pMemoryBarriers = &memory;
                vkCmdPipelineBarrier2(cmd, &dependency);

                VkBufferCopy copy{};
                copy.size = sizeof(AtmosphereMirrorColumn) *
                            std::size_t(ATMOSPHERE_MIRROR_CELLS) *
                            std::size_t(ATMOSPHERE_MIRROR_CELLS);
                vkCmdCopyBuffer(cmd, mirror_, mirror_slots_[slot].buffer, 1, &copy);

                VkBufferCopy profile_copy{};
                profile_copy.size = sizeof(AtmosphereProfileLevel) *
                                    std::size_t(ATMOSPHERE_PROFILE_MAX_LEVELS);
                vkCmdCopyBuffer(cmd, profile_, mirror_slots_[slot].profile_buffer, 1,
                                &profile_copy);
            }

            void AtmosphereNest::collect_readback()
            {
                std::uint64_t completed = 0;
                if (vkGetSemaphoreCounterValue(device_.device(), timeline_, &completed) !=
                    VK_SUCCESS)
                    return;

                // The newest slot whose step has finished and that has not already been taken.
                // A slot can sit completed for several frames while nothing new is submitted;
                // taking it once is what keeps the published revision meaningful.
                std::uint32_t newest = MIRROR_SLOTS;
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                {
                    const MirrorSlot& candidate = mirror_slots_[slot];
                    if (candidate.timeline_value == 0 || candidate.timeline_value > completed ||
                        candidate.timeline_value <= mirror_taken_)
                        continue;
                    if (newest == MIRROR_SLOTS ||
                        candidate.timeline_value > mirror_slots_[newest].timeline_value)
                        newest = slot;
                }
                if (newest == MIRROR_SLOTS)
                    return;

                const MirrorSlot& source = mirror_slots_[newest];
                // The timestamps travel with the same submission and are readable under exactly
                // the same condition — the slot's timeline value has passed — so this is free
                // here and would be a stall anywhere else.
                publish_cost(newest);
                if (source.mapped == nullptr)
                    return;
                std::memcpy(mirror_columns_.data(), source.mapped,
                            mirror_columns_.size() * sizeof(AtmosphereMirrorColumn));
                if (source.profile_mapped != nullptr)
                    std::memcpy(profile_levels_.data(), source.profile_mapped,
                                profile_levels_.size() * sizeof(AtmosphereProfileLevel));
                mirror_taken_ = source.timeline_value;

                // The vertical CFL's bound, from the field itself. Decayed rather than replaced
                // so a step that has grown long in quiet air does not snap back and forth on the
                // ordinary flicker of a peak taken over a thousand columns; a *rise* takes effect
                // at once, which is the direction that matters.
                float peak = 0.0f;
                for (const AtmosphereMirrorColumn& column : mirror_columns_)
                    peak = std::max(peak, column.extent[2]);
                measured_updraft_ = std::max(peak, measured_updraft_ * 0.95f);

                const double span = double(size_.spacing_m) * double(size_.cells_x);
                mirror_view_.columns = mirror_columns_.data();
                mirror_view_.cells = ATMOSPHERE_MIRROR_CELLS;
                // The shader writes only as many levels as the nest has; a tier below the
                // ceiling leaves the tail of the buffer untouched, so the published count is
                // the nest's own rather than the allocation's.
                mirror_view_.profile = profile_levels_.data();
                mirror_view_.profile_levels =
                    std::min(std::int32_t(size_.levels), std::int32_t(ATMOSPHERE_PROFILE_MAX_LEVELS));
                mirror_view_.revision += 1;
                mirror_view_.simulated_seconds = source.simulated_seconds;
                mirror_view_.uv_scale_x = static_cast<float>(1.0 / span);
                mirror_view_.uv_scale_z = static_cast<float>(1.0 / span);
                mirror_view_.uv_offset_x = static_cast<float>(-source.origin_x / span);
                mirror_view_.uv_offset_z = static_cast<float>(-source.origin_z / span);
            }

            void AtmosphereNest::publish_cost(std::uint32_t slot)
            {
                if (!profiler_ || !profiler_->enabled() || slot >= MIRROR_SLOTS)
                    return;
                // Only from a frame that stepped the physics. A frame that merely re-centres
                // the lattice records a shift, an extinction and a readback and nothing else,
                // and publishing that as the step's cost would report a fraction of one every
                // time the observer crossed a cell — which is exactly when someone is watching.
                if (!mirror_slots_[slot].stepped)
                    return;

                // Waiting is free here and only here: the caller has already established that
                // this submission completed, and a completed submission does not oblige a driver
                // to have made its query results readable yet — `VK_NOT_READY` is a legal answer
                // to a question whose answer is known to exist. Asking for it costs nothing at a
                // point where the work is already done, and the alternative is dropping a
                // measurement for a reason that has nothing to do with the measurement.
                if (!profiler_->resolve(slot, true))
                    return;
                const std::vector<Graph::PassTiming>& timings = profiler_->timings();
                if (timings.empty())
                    return;

                AtmosphereStepCost cost;
                cost.step_index = mirror_slots_[slot].step_index;
                cost.steps = std::max(mirror_slots_[slot].steps, 1u);
                cost.total_ms = timings[0].milliseconds; // the outer bracket, recorded first
                for (std::size_t i = 1;
                     i < timings.size() && cost.count < ATMOSPHERE_TIMED_STAGES; ++i)
                {
                    AtmosphereStageTiming& stage = cost.stages[cost.count++];
                    const std::string& name = timings[i].name;
                    const std::size_t length =
                        std::min(name.size(), sizeof(stage.name) - 1);
                    std::memcpy(stage.name, name.c_str(), length);
                    stage.name[length] = '\0';
                    stage.milliseconds = timings[i].milliseconds;
                }
                cost.measured = true;
                step_cost_ = cost;
            }

            void AtmosphereNest::step(const AtmosphereParameters& parameters,
                                      const AtmosphereForcing& forcing,
                                      const VkSemaphoreSubmitInfo* readers,
                                      std::uint32_t reader_count)
            {
                collect_readback();
                if (!parameters.enabled)
                    return;

                pressure_sweeps_ = std::max(parameters.pressure_iterations, 1u);
                // Held from the forcing rather than passed down through every record call: the
                // parameter block is uploaded once per step and these are two of its fields.
                solar_elevation_sine_ = forcing.solar_elevation_sine;
                seed_length_m_ = parameters.thermal_seed_length_m;
                coriolis_ = forcing.coriolis;

                const float dt = choose_step(parameters, forcing);
                step_seconds_ = dt;
                // The delta against the last clock this nest saw. Three views calling in with
                // the same Environment therefore add the elapsed time once, not three times.
                if (forcing.total_seconds > last_total_seconds_)
                    pending_seconds_ += forcing.total_seconds - last_total_seconds_;
                last_total_seconds_ = forcing.total_seconds;
                // Bounded rather than accumulated: a large time scale or a hitch can ask for
                // more steps than a frame can afford, and weather that is briefly behind is a
                // better outcome than a frame that stalls to simulate an hour of it. Nothing
                // downstream depends on the nest's clock being exact (§3.4).
                const std::uint32_t step_cap = std::max(parameters.max_steps_per_frame, 1u);
                pending_seconds_ = std::min(pending_seconds_, double(dt) * double(step_cap));

                // Where the lattice should be, snapped to whole cells against an absolute
                // origin — the same discipline the cloudscape window uses, and for the same
                // reason: a cell that survives a re-centre is copied, never resampled.
                const double spacing = double(size_.spacing_m);
                const long long desired_x =
                    static_cast<long long>(std::floor(forcing.observer_x / spacing)) -
                    static_cast<long long>(size_.cells_x) / 2;
                const long long desired_z =
                    static_cast<long long>(std::floor(forcing.observer_z / spacing)) -
                    static_cast<long long>(size_.cells_z) / 2;

                const bool seeding = !seeded_;
                const bool shifting = seeded_ && (desired_x != origin_cell_x_ ||
                                                  desired_z != origin_cell_z_);

                // How many steps this frame records, and this is the whole of what
                // `max_steps_per_frame` means. It previously clamped the accumulator above and
                // nothing else, so the frame recorded exactly one step however large it was set:
                // raising it bought more tolerated lag rather than more weather, and the nest's
                // throughput was pinned at one step per frame — which is what made a scene
                // drawing at twelve frames a second simulate weather at twelve frames a second.
                std::uint32_t steps =
                    dt > 0.0f ? std::uint32_t(pending_seconds_ / double(dt)) : 0u;
                steps = std::min(steps, step_cap);

                // ...and bounded again by what the slot's descriptor pool can actually serve,
                // because every stage of every step allocates a set from it and the pressure
                // relaxation allocates two per sweep. Clamping here rather than trusting the
                // pool to be large enough is the difference between simulating slightly less
                // weather and failing an allocation mid-record.
                const std::uint32_t sets_per_step = STAGE_COUNT - 3u + 2u * pressure_sweeps_;
                const std::uint32_t affordable =
                    sets_per_step > 0u ? (DESCRIPTOR_SETS_PER_SLOT - 3u) / sets_per_step : step_cap;
                steps = std::min(steps, std::max(affordable, 1u));

                const bool stepping = steps > 0u;
                if (!seeding && !shifting && !stepping)
                    return;

                const std::uint32_t slot = slot_;
                MirrorSlot& target = mirror_slots_[slot];
                if (target.timeline_value != 0)
                {
                    VkSemaphoreWaitInfo wait{};
                    wait.sType = VK_STRUCTURE_TYPE_SEMAPHORE_WAIT_INFO;
                    wait.semaphoreCount = 1;
                    wait.pSemaphores = &timeline_;
                    wait.pValues = &target.timeline_value;
                    Vulkan::check(vkWaitSemaphores(device_.device(), &wait, UINT64_MAX),
                                  "vkWaitSemaphores(atmosphere nest slot)");
                    // A slot is collected *before* it is overwritten, and this is the only place
                    // that can be guaranteed. The opportunistic sweep at the top of this function
                    // compares each slot's `timeline_value` against the semaphore counter — but a
                    // slot holds only its *most recent* submission's value, and with three slots
                    // in flight the CPU sits exactly three submissions ahead, so the value that
                    // has actually completed is always one the slots no longer carry. Measured:
                    // over a hundred consecutive steps the counter read 95 while the three slots
                    // held 96, 97 and 98, the sweep matched nothing every single time, and the
                    // mirror stayed frozen on the first step it ever collected. Gameplay reads
                    // that mirror (§3.2), so what looked like a profiling gap is a data plane
                    // that only refreshed when something else happened to idle the device.
                    //
                    // Here the wait has just established that this slot's submission is done, so
                    // there is no test to get wrong and nothing to stall on.
                    collect_readback();
                }
                Vulkan::check(vkResetDescriptorPool(device_.device(), descriptor_pools_[slot], 0),
                              "vkResetDescriptorPool(atmosphere nest)");

                const std::int32_t shift_x =
                    static_cast<std::int32_t>(desired_x - origin_cell_x_);
                const std::int32_t shift_z =
                    static_cast<std::int32_t>(desired_z - origin_cell_z_);
                origin_cell_x_ = desired_x;
                origin_cell_z_ = desired_z;

                // The clock each recorded step begins at, before the accumulator below runs it
                // forward over all of them.
                const double first_step_seconds = simulated_seconds_;
                if (stepping)
                {
                    pending_seconds_ -= double(dt) * double(steps);
                    simulated_seconds_ += double(dt) * double(steps);
                    step_index_ += steps;
                }
                upload_parameters(parameters, dt);

                VkCommandBuffer cmd = commands_[slot];
                Vulkan::check(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer(atmosphere)");
                VkCommandBufferBeginInfo begin_info{};
                begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(cmd, &begin_info),
                              "vkBeginCommandBuffer(atmosphere)");

                // Resets this slot's queries; every timestamp below is written into it. The
                // outer section is recorded first and is therefore index 0 of the resolved
                // list, which is what `publish_cost` reads the submission total off.
                profiler_->begin_frame(slot, cmd);
                {
                    TimedSection submission(profiler_.get(), cmd, "submission");

                    prepare_layouts(cmd);
                    upload_forcing(cmd, forcing);

                    if (seeding)
                    {
                        // A shift larger than the domain leaves every cell without a source,
                        // which is exactly "fill everything from the base state and the parent"
                        // — the seed is the degenerate case of the re-centre, not a second code
                        // path.
                        record_clear(cmd);
                        record_shift(cmd, std::int32_t(size_.cells_x), std::int32_t(size_.cells_z),
                                     forcing);
                        seeded_ = true;
                    }
                    else if (shifting)
                    {
                        record_shift(cmd, shift_x, shift_z, forcing);
                    }

                    // Several steps into one command buffer, sharing one upload of the parameter
                    // block — legal because everything in it that varies per step is either
                    // constant across a frame's steps by construction (`dt`, the solar
                    // elevation, the parent solution) or passed per dispatch (the clock the
                    // thermal seed is drawn at).
                    for (std::uint32_t i = 0; i < steps; ++i)
                        record_step(cmd, forcing,
                                    first_step_seconds + double(dt) * double(i), i == 0);
                    // Always, not only after a step: a seed or a shift changes what every cell
                    // holds and where it is, and both the cloudscape bake and the readback
                    // address this through the nest's current origin.
                    record_extinction(cmd);
                    record_readback(cmd, slot);
                }

                Vulkan::check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(atmosphere)");

                ++timeline_value_;
                target.timeline_value = timeline_value_;
                target.simulated_seconds = simulated_seconds_;
                target.stepped = stepping;
                target.step_index = step_index_;
                target.steps = steps;
                origin(target.origin_x, target.origin_z);

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = cmd;

                VkSemaphoreSubmitInfo signal{};
                signal.sType = VK_STRUCTURE_TYPE_SEMAPHORE_SUBMIT_INFO;
                signal.semaphore = timeline_;
                signal.value = timeline_value_;
                signal.stageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;

                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                // The frame's readers, so this step cannot begin overwriting fields the views
                // are still sampling. Submission order would not be enough — a queue orders the
                // *start* of its submissions and promises nothing about one finishing before the
                // next begins.
                submit.waitSemaphoreInfoCount = reader_count;
                submit.pWaitSemaphoreInfos = reader_count > 0 ? readers : nullptr;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                submit.signalSemaphoreInfoCount = 1;
                submit.pSignalSemaphoreInfos = &signal;
                Vulkan::check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, VK_NULL_HANDLE),
                              "vkQueueSubmit2(atmosphere nest)");

                slot_ = (slot_ + 1) % MIRROR_SLOTS;
            }

            AtmosphereMirror AtmosphereNest::atmosphere_mirror() const noexcept
            {
                return mirror_view_;
            }
        } // namespace Atmosphere
    } // namespace Render
} // namespace SushiEngine
