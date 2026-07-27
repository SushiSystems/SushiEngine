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

                struct ForcePushBlock
                {
                    float origin_rel[2];
                    float forcing_scale[2];
                    float forcing_offset[2];
                };

                const StageDesc STAGES[] = {
                    {"atmosphere_shift.comp", "usssssiiiiis", sizeof(ShiftPushBlock)},
                    {"atmosphere_advect_velocity.comp", "usssiii", 0},
                    {"atmosphere_advect_scalars.comp", "usssssii", 0},
                    {"atmosphere_forces.comp", "uiiiiis", sizeof(ForcePushBlock)},
                    {"atmosphere_divergence.comp", "usssi", 0},
                    {"atmosphere_pressure.comp", "usi", sizeof(PressurePush)},
                    {"atmosphere_project.comp", "usiii", 0},
                    {"atmosphere_microphysics.comp", "uiii", 0},
                    {"atmosphere_extinction.comp", "usi", 0},
                    {"atmosphere_readback.comp", "usssssssb", sizeof(ReadbackPush)},
                };

                static_assert(sizeof(STAGES) / sizeof(STAGES[0]) == 10,
                              "the stage table and AtmosphereNest::STAGE_COUNT must agree");

                constexpr VkFormat SCALAR_FORMAT = VK_FORMAT_R32_SFLOAT;
                constexpr VkFormat MOISTURE_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
                constexpr VkFormat EXTINCTION_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;
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

                create_volumes();
                create_buffers();
                create_layouts();
                create_pipelines();
                create_commands();
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
                create_volume(forcing_, FORCING_FORMAT, ATMOSPHERE_FORCING_MAX_CELLS,
                              ATMOSPHERE_FORCING_MAX_CELLS, 1);
            }

            void AtmosphereNest::destroy_volumes()
            {
                Volume* volumes[] = {&wind_x_.front, &wind_x_.back, &wind_y_.front, &wind_y_.back,
                                     &wind_z_.front, &wind_z_.back, &theta_.front, &theta_.back,
                                     &moisture_.front, &moisture_.back, &pressure_, &divergence_,
                                     &extinction_, &surface_rain_, &forcing_};
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

                make(forcing_staging_, forcing_staging_allocation_, &forcing_staging_mapped_,
                     VkDeviceSize(ATMOSPHERE_FORCING_MAX_CELLS) * ATMOSPHERE_FORCING_MAX_CELLS *
                         4 * sizeof(float),
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
                drop(forcing_staging_, forcing_staging_allocation_);
                for (std::uint32_t slot = 0; slot < MIRROR_SLOTS; ++slot)
                    drop(mirror_slots_[slot].buffer, mirror_slots_[slot].allocation);
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

                // Sized for the worst frame: every stage allocating a set per step, plus the
                // pressure sweeps, at the step cap. Generous rather than tight — the pool is
                // reset per slot and a few hundred descriptors cost nothing.
                const std::uint32_t sets = 256;
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
                // The vertical CFL against the thinnest level and a generous updraft: a
                // thunderstorm core runs ten times the "fully convective" reporting scale.
                const float thinnest = atmosphere_level_thickness(0, size_.levels, size_.top_m);
                const float vertical = thinnest /
                                       std::max(10.0f * parameters.convective_velocity_scale, 1.0f);

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
                block.eddy_viscosity = p.eddy_viscosity;
                block.sponge_depth = p.sponge_depth;
                block.sponge_rate = p.sponge_rate;
                block.boundary_relaxation = p.boundary_relaxation;
                block.thermal_seed_amplitude = p.thermal_seed_amplitude;
                block.convective_velocity_scale = p.convective_velocity_scale;
                block.autoconversion_rate = p.autoconversion_rate;
                block.autoconversion_threshold = p.autoconversion_threshold;
                block.accretion_rate = p.accretion_rate;
                block.accretion_exponent = p.accretion_exponent;
                block.rain_evaporation_rate = p.rain_evaporation_rate;
                block.fall_speed_coefficient = p.fall_speed_coefficient;
                block.fall_speed_exponent = p.fall_speed_exponent;
                block.droplet_effective_radius = p.droplet_effective_radius;
                block.surface_sensible_flux = p.surface_sensible_flux;
                block.surface_latent_flux = p.surface_latent_flux;
                block.spacing = size_.spacing_m;
                block.domain_top = size_.top_m;
                block.dt = dt;
                block.elapsed = static_cast<float>(simulated_seconds_);
                block.cells_x = std::int32_t(size_.cells_x);
                block.cells_z = std::int32_t(size_.cells_z);
                block.levels = std::int32_t(size_.levels);
                block.boundary_zone = std::int32_t(p.boundary_zone_cells);
                block.step_index = std::int32_t(step_index_ & 0x7fffffffu);
                // Coriolis rides the forcing rather than the authored parameters: it is a
                // property of *where the nest is*, and the simulation is the only party that
                // knows the observer's latitude.
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
                                    surface_rain_.image, forcing_.image};

                VkImageMemoryBarrier2 barriers[sizeof(images) / sizeof(images[0])]{};
                for (std::size_t i = 0; i < sizeof(images) / sizeof(images[0]); ++i)
                {
                    barriers[i].sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barriers[i].srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                    barriers[i].dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT |
                                               VK_PIPELINE_STAGE_2_COPY_BIT;
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
                        const std::size_t offset = (std::size_t(z) * std::size_t(cells) +
                                                    std::size_t(x)) * 4;
                        texels[offset + 0] = sample.wind_east_mps;
                        texels[offset + 1] = sample.wind_north_mps;
                        texels[offset + 2] = sample.theta_anomaly_k;
                        texels[offset + 3] = sample.humidity_anomaly;
                    }

                VkBufferImageCopy copy{};
                copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                copy.imageSubresource.layerCount = 1;
                copy.imageExtent = {ATMOSPHERE_FORCING_MAX_CELLS, ATMOSPHERE_FORCING_MAX_CELLS, 1};
                vkCmdCopyBufferToImage(cmd, forcing_staging_, forcing_.image,
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
            }

            void AtmosphereNest::record_step(VkCommandBuffer cmd, const AtmosphereForcing& forcing)
            {
                const std::uint32_t gx = groups(size_.cells_x, GROUP_3D);
                const std::uint32_t gy = groups(size_.levels, GROUP_3D);
                const std::uint32_t gz = groups(size_.cells_z, GROUP_3D);

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

                // 2. Everything that acts on the transported state: buoyancy, Coriolis,
                //    diffusion, the sponge, the surface fluxes, and the lateral relaxation.
                {
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
                    writer.update(device_.device(), set);
                    dispatch(STAGE_FORCES, set, &push, sizeof(push), gx, gy, gz);
                }

                // 3. The anelastic projection: measure the mass divergence the provisional
                //    velocity carries, solve for the pressure that removes it, remove it.
                {
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_DIVERGENCE, writer);
                    writer.sampled_image(1, wind_x_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(2, wind_y_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.sampled_image(3, wind_z_.front.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, divergence_.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_DIVERGENCE, set, nullptr, 0, gx, gy, gz);
                }
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
                                 groups(size_.cells_x, GROUP_2D), groups(size_.cells_z, GROUP_2D),
                                 1);
                    }
                }
                {
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_PROJECT, writer);
                    writer.sampled_image(1, pressure_.view, sampler_, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, wind_x_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(3, wind_y_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(4, wind_z_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_PROJECT, set, nullptr, 0, gx, gy, gz);
                }

                // 4. The microphysics, on a flow that now transports mass consistently — which
                //    is the precondition for condensate to concentrate where the updraft is.
                {
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_MICROPHYSICS, writer);
                    writer.storage_image(1, theta_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, moisture_.front.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(3, surface_rain_.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_MICROPHYSICS, set, nullptr, 0, gx, gy, gz);
                }

                // 5. What the rest of the engine sees: optical extinction, and the coarse
                //    column summary gameplay reads back.
                {
                    Resources::DescriptorWriter writer;
                    const VkDescriptorSet set = begin(STAGE_EXTINCTION, writer);
                    writer.sampled_image(1, moisture_.front.view, sampler_,
                                         VK_IMAGE_LAYOUT_GENERAL);
                    writer.storage_image(2, extinction_.view, VK_IMAGE_LAYOUT_GENERAL);
                    writer.update(device_.device(), set);
                    dispatch(STAGE_EXTINCTION, set, nullptr, 0, gx, gy, gz);
                }
            }

            void AtmosphereNest::record_readback(VkCommandBuffer cmd, std::uint32_t slot)
            {
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
                writer.storage_buffer(8, mirror_,
                                      sizeof(AtmosphereMirrorColumn) *
                                          std::size_t(ATMOSPHERE_MIRROR_CELLS) *
                                          std::size_t(ATMOSPHERE_MIRROR_CELLS));
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
                if (source.mapped == nullptr)
                    return;
                std::memcpy(mirror_columns_.data(), source.mapped,
                            mirror_columns_.size() * sizeof(AtmosphereMirrorColumn));
                mirror_taken_ = source.timeline_value;

                const double span = double(size_.spacing_m) * double(size_.cells_x);
                mirror_view_.columns = mirror_columns_.data();
                mirror_view_.cells = ATMOSPHERE_MIRROR_CELLS;
                mirror_view_.revision += 1;
                mirror_view_.simulated_seconds = source.simulated_seconds;
                mirror_view_.uv_scale_x = static_cast<float>(1.0 / span);
                mirror_view_.uv_scale_z = static_cast<float>(1.0 / span);
                mirror_view_.uv_offset_x = static_cast<float>(-source.origin_x / span);
                mirror_view_.uv_offset_z = static_cast<float>(-source.origin_z / span);
            }

            void AtmosphereNest::step(const AtmosphereParameters& parameters,
                                      const AtmosphereForcing& forcing)
            {
                collect_readback();
                if (!parameters.enabled)
                    return;

                pressure_sweeps_ = std::max(parameters.pressure_iterations, 1u);

                const float dt = choose_step(parameters, forcing);
                // The delta against the last clock this nest saw. Three views calling in with
                // the same Environment therefore add the elapsed time once, not three times.
                if (forcing.total_seconds > last_total_seconds_)
                    pending_seconds_ += forcing.total_seconds - last_total_seconds_;
                last_total_seconds_ = forcing.total_seconds;
                // Bounded rather than accumulated: a large time scale or a hitch can ask for
                // more steps than a frame can afford, and weather that is briefly behind is a
                // better outcome than a frame that stalls to simulate an hour of it. Nothing
                // downstream depends on the nest's clock being exact (§3.4).
                pending_seconds_ = std::min(pending_seconds_,
                                            double(dt) * double(parameters.max_steps_per_frame));

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
                const bool stepping = pending_seconds_ >= double(dt);
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
                }
                Vulkan::check(vkResetDescriptorPool(device_.device(), descriptor_pools_[slot], 0),
                              "vkResetDescriptorPool(atmosphere nest)");

                const std::int32_t shift_x =
                    static_cast<std::int32_t>(desired_x - origin_cell_x_);
                const std::int32_t shift_z =
                    static_cast<std::int32_t>(desired_z - origin_cell_z_);
                origin_cell_x_ = desired_x;
                origin_cell_z_ = desired_z;

                if (stepping)
                {
                    pending_seconds_ -= double(dt);
                    simulated_seconds_ += double(dt);
                    ++step_index_;
                }
                upload_parameters(parameters, dt);

                VkCommandBuffer cmd = commands_[slot];
                Vulkan::check(vkResetCommandBuffer(cmd, 0), "vkResetCommandBuffer(atmosphere)");
                VkCommandBufferBeginInfo begin_info{};
                begin_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin_info.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                Vulkan::check(vkBeginCommandBuffer(cmd, &begin_info),
                              "vkBeginCommandBuffer(atmosphere)");

                prepare_layouts(cmd);
                upload_forcing(cmd, forcing);

                if (seeding)
                {
                    // A shift larger than the domain leaves every cell without a source, which
                    // is exactly "fill everything from the base state and the parent" — the
                    // seed is the degenerate case of the re-centre, not a second code path.
                    record_shift(cmd, std::int32_t(size_.cells_x), std::int32_t(size_.cells_z),
                                 forcing);
                    seeded_ = true;
                }
                else if (shifting)
                {
                    record_shift(cmd, shift_x, shift_z, forcing);
                }

                if (stepping)
                    record_step(cmd, forcing);
                record_readback(cmd, slot);

                Vulkan::check(vkEndCommandBuffer(cmd), "vkEndCommandBuffer(atmosphere)");

                ++timeline_value_;
                target.timeline_value = timeline_value_;
                target.simulated_seconds = simulated_seconds_;
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
