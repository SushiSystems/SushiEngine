/**************************************************************************/
/* material_system.cpp                                                    */
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

#include "material/material_system.hpp"

#include <algorithm>
#include <cstring>

#include "material/texture_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            namespace
            {
                /** @brief Materials a slot buffer starts out able to hold. */
                constexpr std::size_t INITIAL_CAPACITY = 256;
            } // namespace

            MaterialSystem::MaterialSystem(Vulkan::VulkanDevice& device, TextureLibrary& textures,
                                           std::uint32_t frame_slots)
                : device_(device), textures_(textures)
            {
                slots_.resize(frame_slots);
                for (Slot& slot : slots_)
                    grow(slot, INITIAL_CAPACITY * sizeof(GpuMaterial));
            }

            MaterialSystem::~MaterialSystem()
            {
                for (Slot& slot : slots_)
                    if (slot.buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.buffer, slot.allocation);
                slots_.clear();
            }

            void MaterialSystem::grow(Slot& slot, VkDeviceSize bytes)
            {
                if (bytes <= slot.capacity)
                    return;
                if (slot.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.buffer, slot.allocation);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &slot.buffer, &slot.allocation, &info),
                              "vmaCreateBuffer(materials)");
                slot.mapped = info.pMappedData;
                slot.capacity = bytes;
            }

            void MaterialSystem::begin_frame(std::uint32_t slot)
            {
                current_slot_ = slot;
                packed_.clear();
            }

            void MaterialSystem::set_allowed_lobes(std::uint32_t lobes) noexcept
            {
                // Only the advanced-lobe bits are the tier's to withhold; forcing every
                // other bit on keeps a stray mask from ever stripping a base PBR flag.
                constexpr std::uint32_t ADVANCED_LOBES = MATERIAL_ANISOTROPY |
                                                         MATERIAL_CLEARCOAT | MATERIAL_SHEEN |
                                                         MATERIAL_TRANSMISSION;
                allowed_lobes_ = (lobes & ADVANCED_LOBES) | ~ADVANCED_LOBES;
            }

            std::uint32_t MaterialSystem::push(const Render::Material& material)
            {
                GpuMaterial gpu{};

                gpu.base_color[0] = static_cast<float>(material.albedo.x);
                gpu.base_color[1] = static_cast<float>(material.albedo.y);
                gpu.base_color[2] = static_cast<float>(material.albedo.z);
                gpu.base_color[3] = material.base_alpha;

                // Emissive intensity is folded into the colour here rather than in the
                // shader, so the shader multiplies one term instead of two.
                gpu.emissive[0] = static_cast<float>(material.emissive.x) *
                                  material.emissive_intensity;
                gpu.emissive[1] = static_cast<float>(material.emissive.y) *
                                  material.emissive_intensity;
                gpu.emissive[2] = static_cast<float>(material.emissive.z) *
                                  material.emissive_intensity;
                gpu.emissive[3] = material.normal_scale;

                gpu.surface[0] = material.metallic;
                gpu.surface[1] = std::max(material.roughness, 0.045f);
                gpu.surface[2] = material.occlusion_strength;
                gpu.surface[3] = material.alpha_cutoff;

                gpu.parallax[0] = material.height_scale;
                gpu.parallax[1] = static_cast<float>(material.parallax_steps);
                gpu.parallax[2] = material.ior;
                gpu.parallax[3] = material.detail_normal_scale;

                gpu.main_transform[0] = material.main_transform.tiling_x;
                gpu.main_transform[1] = material.main_transform.tiling_y;
                gpu.main_transform[2] = material.main_transform.offset_x;
                gpu.main_transform[3] = material.main_transform.offset_y;
                gpu.detail_transform[0] = material.detail_transform.tiling_x;
                gpu.detail_transform[1] = material.detail_transform.tiling_y;
                gpu.detail_transform[2] = material.detail_transform.offset_x;
                gpu.detail_transform[3] = material.detail_transform.offset_y;

                gpu.anisotropy_clearcoat[0] = material.anisotropy;
                gpu.anisotropy_clearcoat[1] = material.anisotropy_rotation;
                gpu.anisotropy_clearcoat[2] = material.clearcoat;
                gpu.anisotropy_clearcoat[3] = std::max(material.clearcoat_roughness, 0.02f);

                gpu.sheen[0] = static_cast<float>(material.sheen_color.x);
                gpu.sheen[1] = static_cast<float>(material.sheen_color.y);
                gpu.sheen[2] = static_cast<float>(material.sheen_color.z);
                gpu.sheen[3] = std::max(material.sheen_roughness, 0.05f);

                gpu.transmission[0] = material.transmission;
                gpu.transmission[1] = material.thickness;
                gpu.subsurface[0] = static_cast<float>(material.subsurface_color.x);
                gpu.subsurface[1] = static_cast<float>(material.subsurface_color.y);
                gpu.subsurface[2] = static_cast<float>(material.subsurface_color.z);

                gpu.maps_a[0] = textures_.heap_index(material.albedo_map, DefaultTexture::White);
                gpu.maps_a[1] = textures_.heap_index(material.metallic_roughness_map,
                                                     DefaultTexture::NeutralMaterial);
                gpu.maps_a[2] =
                    textures_.heap_index(material.normal_map, DefaultTexture::FlatNormal);
                gpu.maps_a[3] = textures_.heap_index(material.height_map, DefaultTexture::Black);
                gpu.maps_b[0] = textures_.heap_index(material.occlusion_map,
                                                     DefaultTexture::NeutralMaterial);
                gpu.maps_b[1] = textures_.heap_index(material.emissive_map, DefaultTexture::White);
                gpu.maps_b[2] =
                    textures_.heap_index(material.detail_albedo_map, DefaultTexture::White);
                gpu.maps_b[3] =
                    textures_.heap_index(material.detail_normal_map, DefaultTexture::FlatNormal);
                gpu.maps_c[0] =
                    textures_.heap_index(material.detail_mask_map, DefaultTexture::White);

                std::uint32_t flags = 0;
                if (material.height_map != INVALID_TEXTURE && material.parallax_steps > 0 &&
                    material.height_scale > 0.0f)
                {
                    flags |= MATERIAL_HAS_PARALLAX;
                    if (material.parallax_shadows)
                        flags |= MATERIAL_PARALLAX_SHADOWS;
                    if (material.parallax_silhouette_clip)
                        flags |= MATERIAL_PARALLAX_CLIP;
                }
                if (material.packed_occlusion)
                    flags |= MATERIAL_PACKED_OCCLUSION;
                if (material.emissive_enabled)
                    flags |= MATERIAL_EMISSIVE;
                if (material.detail_albedo_map != INVALID_TEXTURE ||
                    material.detail_normal_map != INVALID_TEXTURE)
                    flags |= MATERIAL_HAS_DETAIL;
                if (material.surface_type == SurfaceType::Cutout)
                    flags |= MATERIAL_CUTOUT;
                if (material.anisotropy != 0.0f)
                    flags |= MATERIAL_ANISOTROPY;
                if (material.clearcoat > 0.0f)
                    flags |= MATERIAL_CLEARCOAT;
                if (material.sheen_color.x > 0.0 || material.sheen_color.y > 0.0 ||
                    material.sheen_color.z > 0.0)
                    flags |= MATERIAL_SHEEN;
                if (material.transmission > 0.0f)
                    flags |= MATERIAL_TRANSMISSION;
                // Strip whichever advanced lobes the current tier withholds; the base PBR
                // flags pass through untouched because set_allowed_lobes forces their bits on.
                flags &= allowed_lobes_;
                gpu.maps_c[1] = flags;

                packed_.push_back(gpu);
                return static_cast<std::uint32_t>(packed_.size() - 1);
            }

            void MaterialSystem::upload()
            {
                Slot& slot = slots_[current_slot_];
                // Always keep at least one entry: a zero-range storage buffer is not a
                // legal descriptor, and a frame with nothing to draw still writes a set.
                const std::size_t count = std::max<std::size_t>(packed_.size(), 1);
                grow(slot, count * sizeof(GpuMaterial));
                if (!packed_.empty())
                    std::memcpy(slot.mapped, packed_.data(),
                                packed_.size() * sizeof(GpuMaterial));
            }

            VkBuffer MaterialSystem::buffer() const noexcept
            {
                return slots_[current_slot_].buffer;
            }

            VkDeviceSize MaterialSystem::buffer_range() const noexcept
            {
                return std::max<VkDeviceSize>(packed_.size() * sizeof(GpuMaterial),
                                              sizeof(GpuMaterial));
            }
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
