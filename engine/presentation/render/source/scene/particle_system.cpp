/**************************************************************************/
/* particle_system.cpp                                                    */
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

#include "scene/particle_system.hpp"

#include <cstring>

#include <SushiEngine/vfx/compiled_emitter.hpp>

#include "geometry/mesh_registry.hpp"
#include "material/texture_library.hpp"
#include "resources/descriptor_heap.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            ParticleSystem::ParticleSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots,
                                           std::uint32_t capacity)
                : device_(device), capacity_(capacity < 1 ? 1 : capacity),
                  emitter_tables_(frame_slots), curve_luts_(frame_slots),
                  gradient_luts_(frame_slots), billboards_(frame_slots)
            {
                // The pool is device-local (a compute read-modify-write target every frame) and
                // is zero-cleared once by the sim pass on its first run, so a particle's life
                // starts at 0 (dead) before anything writes it.
                const VkDeviceSize pool_bytes =
                    static_cast<VkDeviceSize>(capacity_) * sizeof(VFX::GPUParticle);
                grow(pool_,
                     pool_bytes,
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, false,
                     false);
                grow(trails_, trail_range(),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT, false,
                     false);
            }

            ParticleSystem::~ParticleSystem()
            {
                destroy(pool_);
                destroy(trails_);
                for (Allocation& allocation : emitter_tables_)
                    destroy(allocation);
                for (Allocation& allocation : curve_luts_)
                    destroy(allocation);
                for (Allocation& allocation : gradient_luts_)
                    destroy(allocation);
                for (Allocation& allocation : billboards_)
                    destroy(allocation);
            }

            void ParticleSystem::prepare(std::uint32_t slot, std::uint32_t frame_index,
                                         const ParticleEmitterView* emitters, std::size_t count,
                                         const Geometry::MeshRegistry& meshes,
                                         const Assets::TextureLibrary& textures)
            {
                (void)frame_index;
                emitters_.clear();
                mesh_draws_.clear();
                needs_alpha_sort_ = false;
                if (emitters == nullptr || count == 0)
                    return;

                // One shared LUT atlas per frame: an effect's emitters carry the same atlas
                // pointer, so uploading the longest one covers them. (Distinct effects with
                // colliding offsets are a later refinement; the slice runs one effect at a time.)
                const float* curve_source = nullptr;
                const float* gradient_source = nullptr;
                std::uint32_t curve_floats = 0;
                std::uint32_t gradient_floats = 0;
                for (std::size_t i = 0; i < count; ++i)
                {
                    if (emitters[i].curve_lut_floats > curve_floats)
                    {
                        curve_floats = emitters[i].curve_lut_floats;
                        curve_source = emitters[i].curve_luts;
                    }
                    if (emitters[i].gradient_lut_floats > gradient_floats)
                    {
                        gradient_floats = emitters[i].gradient_lut_floats;
                        gradient_source = emitters[i].gradient_luts;
                    }
                }

                curve_bytes_ = curve_floats > 0 ? static_cast<VkDeviceSize>(curve_floats) * sizeof(float)
                                                : sizeof(float);
                gradient_bytes_ = gradient_floats > 0
                                      ? static_cast<VkDeviceSize>(gradient_floats) * sizeof(float)
                                      : sizeof(float);
                grow(curve_luts_[slot], curve_bytes_, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, false);
                grow(gradient_luts_[slot], gradient_bytes_, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true,
                     false);
                if (curve_source != nullptr)
                    std::memcpy(curve_luts_[slot].mapped, curve_source, curve_bytes_);
                if (gradient_source != nullptr)
                    std::memcpy(gradient_luts_[slot].mapped, gradient_source, gradient_bytes_);

                emitters_.reserve(count);
                for (std::size_t i = 0; i < count; ++i)
                {
                    const ParticleEmitterView& view = emitters[i];
                    const auto* compiled = static_cast<const VFX::CompiledEmitter*>(view.compiled);
                    if (compiled == nullptr)
                        continue;

                    GPUEmitter gpu{};
                    for (int m = 0; m < 16; ++m)
                        gpu.model[m] = static_cast<float>(view.model.m[m]);
                    gpu.shape = static_cast<std::uint32_t>(compiled->shape);
                    gpu.shape_flags = compiled->shape_flags;
                    gpu.update_flags = compiled->update_flags;
                    gpu.capacity = capacity_;
                    gpu.shape_radius = compiled->shape_radius;
                    gpu.shape_cone_angle = compiled->shape_cone_angle;
                    gpu.shape_arc = compiled->shape_arc;
                    gpu.drag_coefficient = compiled->drag_coefficient;
                    gpu.box_half_extents[0] = compiled->shape_box_half_extents[0];
                    gpu.box_half_extents[1] = compiled->shape_box_half_extents[1];
                    gpu.box_half_extents[2] = compiled->shape_box_half_extents[2];
                    gpu.turbulence_frequency = compiled->turbulence_frequency;
                    gpu.gravity[0] = compiled->gravity[0];
                    gpu.gravity[1] = compiled->gravity[1];
                    gpu.gravity[2] = compiled->gravity[2];
                    gpu.turbulence_amplitude = compiled->turbulence_amplitude;
                    gpu.color[0] = compiled->color[0];
                    gpu.color[1] = compiled->color[1];
                    gpu.color[2] = compiled->color[2];
                    gpu.pad_color = 0.0f;
                    gpu.lifetime_min = compiled->lifetime_min;
                    gpu.lifetime_max = compiled->lifetime_max;
                    gpu.speed_min = compiled->speed_min;
                    gpu.speed_max = compiled->speed_max;
                    gpu.size_min = compiled->size_min;
                    gpu.size_max = compiled->size_max;
                    gpu.rotation_min = compiled->rotation_min;
                    gpu.rotation_max = compiled->rotation_max;
                    gpu.angular_min = compiled->angular_velocity_min;
                    gpu.angular_max = compiled->angular_velocity_max;
                    gpu.velocity_stretch = compiled->velocity_stretch;
                    gpu.pad_b = 0.0f;
                    gpu.size_curve_lut = compiled->size_curve_lut;
                    gpu.color_gradient_lut = compiled->color_gradient_lut;
                    gpu.spawn_base = ring_cursor_;
                    gpu.spawn_count = view.spawn_count;
                    gpu.seed = view.seed;
                    gpu.frame = frame_index;
                    gpu.flipbook_rows = compiled->flipbook_rows;
                    gpu.flipbook_columns = compiled->flipbook_columns;
                    gpu.blend = static_cast<std::uint32_t>(compiled->blend);
                    gpu.sort = static_cast<std::uint32_t>(compiled->sort);
                    gpu.alignment = static_cast<std::uint32_t>(compiled->alignment);

                    // The particle material. The authored texture is a library id; the shader
                    // wants the bindless slot it currently sits in, and only the library can say.
                    // A texture the heap could not take (no descriptor indexing, or the heap is
                    // full) drops back to the built-in dot rather than to a wrong sample.
                    gpu.render_flags = compiled->render_flags;
                    gpu.soft_fade_distance = compiled->soft_fade_distance;
                    gpu.texture = 0;
                    gpu.pad_material = 0.0f;
                    if ((compiled->render_flags & VFX::RENDER_TEXTURED) != 0)
                    {
                        const std::uint32_t heap_slot = textures.heap_index(
                            static_cast<TextureId>(compiled->texture),
                            Assets::DefaultTexture::White);
                        if (heap_slot == Resources::INVALID_HEAP_INDEX)
                            gpu.render_flags &= ~VFX::RENDER_TEXTURED;
                        else
                            gpu.texture = heap_slot;
                    }

                    // A mesh-aligned emitter claims one of the mesh-draw slices, because one draw
                    // binds one mesh. Past the last slice the emitter simply renders nothing —
                    // dropping it is better than silently drawing its particles as someone else's
                    // mesh, which is what sharing a slice would do.
                    // Force fields, placed into the world here so the shader never touches the
                    // emitter transform: the model matrix moves the centre, its rotation the axis.
                    gpu.force_field_count = compiled->force_field_count;
                    gpu.collision_restitution = compiled->collision_restitution;
                    gpu.collision_friction = compiled->collision_friction;
                    gpu.collision_thickness = compiled->collision_thickness;
                    for (std::uint32_t f = 0; f < VFX::MAX_FORCE_FIELDS; ++f)
                    {
                        float* out = gpu.force_fields[f];
                        if (f >= compiled->force_field_count)
                        {
                            for (int k = 0; k < 12; ++k)
                                out[k] = 0.0f;
                            continue;
                        }
                        const VFX::CompiledForceField& field = compiled->force_fields[f];
                        // Centre: the full transform (rotation, scale, translation).
                        for (int row = 0; row < 3; ++row)
                        {
                            out[row] = gpu.model[row] * field.position[0] +
                                       gpu.model[4 + row] * field.position[1] +
                                       gpu.model[8 + row] * field.position[2] + gpu.model[12 + row];
                        }
                        out[3] = field.strength;
                        // Axis: direction only, so the translation column is left out.
                        for (int row = 0; row < 3; ++row)
                        {
                            out[4 + row] = gpu.model[row] * field.axis[0] +
                                           gpu.model[4 + row] * field.axis[1] +
                                           gpu.model[8 + row] * field.axis[2];
                        }
                        out[7] = field.radius;
                        out[8] = static_cast<float>(static_cast<std::uint32_t>(field.kind));
                        out[9] = field.falloff;
                        out[10] = 0.0f;
                        out[11] = 0.0f;
                    }

                    // Beam endpoints, placed into the world by the same rule as a force-field
                    // centre: the full transform, so moving or turning the emitter carries the
                    // beam with it.
                    const auto to_world = [&gpu](const float local[3], float out[3])
                    {
                        for (int row = 0; row < 3; ++row)
                        {
                            out[row] = gpu.model[row] * local[0] + gpu.model[4 + row] * local[1] +
                                       gpu.model[8 + row] * local[2] + gpu.model[12 + row];
                        }
                    };
                    to_world(compiled->beam_start, gpu.beam_start);
                    to_world(compiled->beam_end, gpu.beam_end);
                    gpu.beam_width = compiled->beam_width;
                    gpu.beam_sag = compiled->beam_sag;
                    gpu.beam_noise_amplitude = compiled->beam_noise_amplitude;
                    gpu.beam_noise_frequency = compiled->beam_noise_frequency;
                    gpu.beam_pad[0] = 0.0f;
                    gpu.beam_pad[1] = 0.0f;

                    gpu.mesh_slot = NO_MESH_SLOT;
                    if (compiled->alignment == VFX::RenderAlignment::Mesh &&
                        mesh_draws_.size() < MAX_MESH_EMITTERS)
                    {
                        const MeshId mesh_id = static_cast<MeshId>(compiled->mesh);
                        const Geometry::Mesh& mesh = meshes.mesh(mesh_id);
                        if (mesh.index_count > 0)
                        {
                            gpu.mesh_slot = static_cast<std::uint32_t>(mesh_draws_.size());
                            mesh_draws_.push_back(
                                MeshDraw{mesh_id, gpu.mesh_slot, mesh.index_count});
                        }
                    }
                    // The depth sort is asked for here rather than per particle because the
                    // bucket it orders is shared: every true-alpha emitter's survivors land in
                    // one list, and a bitonic pass over it costs the whole padded pool whatever
                    // fraction of the pool is actually in it. Excluding one emitter's particles
                    // would therefore save nothing and cost their blending — the only other
                    // bucket is the additive draw, which composites differently. So the sort is
                    // run whenever any alpha emitter wants it, and skipped only when none does.
                    if (compiled->blend == VFX::BlendMode::Alpha &&
                        compiled->sort == VFX::SortMode::ViewDistance)
                        needs_alpha_sort_ = true;
                    emitters_.push_back(gpu);

                    ring_cursor_ = (ring_cursor_ + view.spawn_count) % capacity_;
                }

                if (!emitters_.empty())
                {
                    const VkDeviceSize table_bytes =
                        static_cast<VkDeviceSize>(emitters_.size()) * sizeof(GPUEmitter);
                    grow(emitter_tables_[slot], table_bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true,
                         false);
                    std::memcpy(emitter_tables_[slot].mapped, emitters_.data(), table_bytes);
                }
            }

            void ParticleSystem::prepare_billboards(std::uint32_t slot,
                                                    const ParticleBillboard* billboards,
                                                    std::size_t count)
            {
                billboard_count_ = 0;
                if (billboards == nullptr || count == 0)
                    return;

                const VkDeviceSize bytes =
                    static_cast<VkDeviceSize>(count) * sizeof(VFX::GPUParticle);
                grow(billboards_[slot], bytes, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true, false);
                auto* out = static_cast<VFX::GPUParticle*>(billboards_[slot].mapped);
                for (std::size_t i = 0; i < count; ++i)
                {
                    VFX::GPUParticle particle{};
                    particle.position[0] = static_cast<float>(billboards[i].position.x);
                    particle.position[1] = static_cast<float>(billboards[i].position.y);
                    particle.position[2] = static_cast<float>(billboards[i].position.z);
                    particle.color[0] = static_cast<float>(billboards[i].color.x);
                    particle.color[1] = static_cast<float>(billboards[i].color.y);
                    particle.color[2] = static_cast<float>(billboards[i].color.z);
                    particle.size = billboards[i].size;
                    particle.alpha = billboards[i].alpha;
                    particle.rotation = billboards[i].rotation;
                    particle.life = 1.0f;
                    out[i] = particle;
                }
                billboard_count_ = static_cast<std::uint32_t>(count);
            }

            VkBuffer ParticleSystem::pool() const noexcept { return pool_.buffer; }

            VkBuffer ParticleSystem::trail_buffer() const noexcept { return trails_.buffer; }

            VkDeviceSize ParticleSystem::trail_range() const noexcept
            {
                return static_cast<VkDeviceSize>(capacity_) * TRAIL_POINTS * 4 * sizeof(float);
            }

            VkBuffer ParticleSystem::billboard_buffer(std::uint32_t slot) const noexcept
            {
                return billboards_[slot].buffer;
            }

            VkDeviceSize ParticleSystem::billboard_range() const noexcept
            {
                return static_cast<VkDeviceSize>(billboard_count_) * sizeof(VFX::GPUParticle);
            }

            VkDeviceSize ParticleSystem::pool_range() const noexcept
            {
                return static_cast<VkDeviceSize>(capacity_) * sizeof(VFX::GPUParticle);
            }

            VkBuffer ParticleSystem::emitter_buffer(std::uint32_t slot) const noexcept
            {
                return emitter_tables_[slot].buffer;
            }

            VkDeviceSize ParticleSystem::emitter_range() const noexcept
            {
                return static_cast<VkDeviceSize>(emitters_.size()) * sizeof(GPUEmitter);
            }

            VkBuffer ParticleSystem::curve_lut_buffer(std::uint32_t slot) const noexcept
            {
                return curve_luts_[slot].buffer;
            }

            VkBuffer ParticleSystem::gradient_lut_buffer(std::uint32_t slot) const noexcept
            {
                return gradient_luts_[slot].buffer;
            }

            VkDeviceSize ParticleSystem::curve_lut_range() const noexcept { return curve_bytes_; }

            VkDeviceSize ParticleSystem::gradient_lut_range() const noexcept { return gradient_bytes_; }

            void ParticleSystem::grow(Allocation& target, VkDeviceSize bytes,
                                      VkBufferUsageFlags usage, bool host_visible,
                                      bool zero_initialize)
            {
                if (bytes == 0)
                    bytes = 1;
                if (target.buffer != VK_NULL_HANDLE && target.capacity >= bytes)
                {
                    if (zero_initialize && target.mapped != nullptr)
                        std::memset(target.mapped, 0, static_cast<std::size_t>(bytes));
                    return;
                }
                destroy(target);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = usage;
                buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

                VmaAllocationCreateInfo alloc_info{};
                alloc_info.usage = VMA_MEMORY_USAGE_AUTO;
                if (host_visible)
                    alloc_info.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                       VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo mapped_info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc_info,
                                              &target.buffer, &target.allocation, &mapped_info),
                              "vmaCreateBuffer(particles)");
                target.mapped = host_visible ? mapped_info.pMappedData : nullptr;
                target.capacity = bytes;
                if (zero_initialize && target.mapped != nullptr)
                    std::memset(target.mapped, 0, static_cast<std::size_t>(bytes));
            }

            void ParticleSystem::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target.buffer = VK_NULL_HANDLE;
                target.allocation = VK_NULL_HANDLE;
                target.mapped = nullptr;
                target.capacity = 0;
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
