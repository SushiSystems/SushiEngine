/**************************************************************************/
/* light_system.cpp                                                       */
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

#include "lighting/light_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "lighting/cluster_config.hpp"
#include "material/texture_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Lighting
        {
            namespace
            {
                /** @brief Lights a slot buffer starts out able to hold. */
                constexpr std::uint32_t INITIAL_CAPACITY = 128;

                /** @brief Tiles per side of the square shadow atlas (4×4 = 16 tiles). */
                constexpr std::uint32_t SHADOW_TILES_PER_SIDE = 4;

                /** @brief Floats one shadow record occupies: a mat4 plus a tile vec4. */
                constexpr std::uint32_t SHADOW_RECORD_FLOATS = 20;

                /** @brief A conventional Vulkan perspective (depth 0→1, no Y-flip).
                 *
                 * Deliberately not the engine's reverse-Z/infinite-far camera projection:
                 * a shadow map wants a bounded far plane and near→0 so the depth atlas
                 * clears to 1 (lit) and a `LESS` compare occludes, matching the sun atlas.
                 * No Y-flip, because the map is sampled by this very matrix (uv = ndc*0.5
                 * + 0.5), so render and sample stay self-consistent.
                 */
                Mat4 shadow_perspective(double fovy, double near_plane, double far_plane)
                {
                    Mat4 m;
                    for (int i = 0; i < 16; ++i)
                        m.m[i] = 0.0;
                    const double f = 1.0 / std::tan(fovy * 0.5);
                    m.m[0] = f;   // aspect is 1 (square tile)
                    m.m[5] = f;
                    m.m[10] = far_plane / (near_plane - far_plane);
                    m.m[11] = -1.0;
                    m.m[14] = (far_plane * near_plane) / (near_plane - far_plane);
                    return m;
                }
            } // namespace

            LightSystem::LightSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                slots_.resize(frame_slots);
                for (Slot& slot : slots_)
                {
                    grow_lights(slot, INITIAL_CAPACITY * GPU_LIGHT_BYTES);

                    // The config block is a fixed-size uniform buffer, allocated once.
                    VkBufferCreateInfo config_info{};
                    config_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    config_info.size = sizeof(LightClusterUniforms);
                    config_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                    VmaAllocationInfo info{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &config_info, &alloc,
                                                  &slot.config, &slot.config_alloc, &info),
                                  "vmaCreateBuffer(light config)");
                    slot.config_mapped = info.pMappedData;

                    grow_shadow(slot, SHADOW_TILES_PER_SIDE * SHADOW_TILES_PER_SIDE *
                                          SHADOW_RECORD_FLOATS * sizeof(float));
                    grow_decals(slot, 32 * GPU_DECAL_BYTES);
                }
            }

            LightSystem::~LightSystem()
            {
                for (Slot& slot : slots_)
                {
                    if (slot.lights != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.lights, slot.lights_alloc);
                    if (slot.config != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.config, slot.config_alloc);
                    if (slot.shadow != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.shadow, slot.shadow_alloc);
                    if (slot.decals != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.decals, slot.decals_alloc);
                }
                slots_.clear();
            }

            void LightSystem::grow_decals(Slot& slot, VkDeviceSize bytes)
            {
                if (bytes <= slot.decals_capacity)
                    return;
                if (slot.decals != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.decals, slot.decals_alloc);

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
                                              &slot.decals, &slot.decals_alloc, &info),
                              "vmaCreateBuffer(decals)");
                slot.decals_mapped = info.pMappedData;
                slot.decals_capacity = bytes;
            }

            void LightSystem::grow_shadow(Slot& slot, VkDeviceSize bytes)
            {
                if (bytes <= slot.shadow_capacity)
                    return;
                if (slot.shadow != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.shadow, slot.shadow_alloc);

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
                                              &slot.shadow, &slot.shadow_alloc, &info),
                              "vmaCreateBuffer(light shadows)");
                slot.shadow_mapped = info.pMappedData;
                slot.shadow_capacity = bytes;
            }

            void LightSystem::grow_lights(Slot& slot, VkDeviceSize bytes)
            {
                if (bytes <= slot.lights_capacity)
                    return;
                if (slot.lights != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.lights, slot.lights_alloc);

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
                                              &slot.lights, &slot.lights_alloc, &info),
                              "vmaCreateBuffer(lights)");
                slot.lights_mapped = info.pMappedData;
                slot.lights_capacity = bytes;
            }

            void LightSystem::begin_frame(std::uint32_t slot)
            {
                current_slot_ = slot;
                packed_.clear();
                decal_packed_.clear();
                light_count_ = 0;
                decal_count_ = 0;
            }

            void LightSystem::pack(const PunctualLight* lights, std::size_t count,
                                   const double eye[3], std::uint32_t max_lights)
            {
                const std::uint32_t clamped =
                    static_cast<std::uint32_t>(std::min<std::size_t>(count, max_lights));
                light_count_ = clamped;
                packed_.assign(static_cast<std::size_t>(clamped) * GPU_LIGHT_FLOATS, 0.0f);

                for (std::uint32_t i = 0; i < clamped; ++i)
                {
                    const PunctualLight& light = lights[i];
                    float* out = packed_.data() + static_cast<std::size_t>(i) * GPU_LIGHT_FLOATS;

                    // Camera-relative position: eye subtracted in double before the float
                    // cast, exactly as a mesh's model translation is, so the light stays
                    // put at planetary range. w carries the range the falloff windows to.
                    out[0] = static_cast<float>(light.position.x - eye[0]);
                    out[1] = static_cast<float>(light.position.y - eye[1]);
                    out[2] = static_cast<float>(light.position.z - eye[2]);
                    out[3] = std::max(light.range, 1e-3f);

                    out[4] = static_cast<float>(light.color.x);
                    out[5] = static_cast<float>(light.color.y);
                    out[6] = static_cast<float>(light.color.z);
                    out[7] = light.intensity;

                    const Vector3 dir = normalize(light.direction);
                    out[8] = static_cast<float>(dir.x);
                    out[9] = static_cast<float>(dir.y);
                    out[10] = static_cast<float>(dir.z);
                    out[11] = static_cast<float>(static_cast<std::uint32_t>(light.type));

                    // The spot's angular falloff is precomputed here so the shader is a
                    // clamp and a multiply: cos(outer) is the cutoff, and the inverse of
                    // the cosine span between the inner and outer cones is the slope that
                    // turns the raw cosine into a [0,1] edge. A point light ignores these.
                    const float cos_outer = std::cos(light.outer_cone);
                    const float cos_inner = std::cos(std::min(light.inner_cone, light.outer_cone));
                    out[12] = cos_outer;
                    out[13] = 1.0f / std::max(cos_inner - cos_outer, 1e-3f);
                    // The shadow index, patched by assign_shadows for casters that claim a
                    // tile; -1 means "unshadowed", which the shader reads as no map.
                    out[14] = -1.0f;
                }
            }

            void LightSystem::pack_decals(const Decal* decals, std::size_t count,
                                          const double eye[3], std::uint32_t max_decals,
                                          const Assets::TextureLibrary& textures)
            {
                const std::uint32_t clamped =
                    static_cast<std::uint32_t>(std::min<std::size_t>(count, max_decals));
                decal_count_ = clamped;
                decal_packed_.assign(static_cast<std::size_t>(clamped) * GPU_DECAL_FLOATS, 0.0f);

                for (std::uint32_t i = 0; i < clamped; ++i)
                {
                    const Decal& decal = decals[i];
                    float* out = decal_packed_.data() + static_cast<std::size_t>(i) * GPU_DECAL_FLOATS;

                    // Camera-relative centre, eye subtracted in double, like a light's.
                    out[0] = static_cast<float>(decal.position.x - eye[0]);
                    out[1] = static_cast<float>(decal.position.y - eye[1]);
                    out[2] = static_cast<float>(decal.position.z - eye[2]);
                    // Bounding-sphere radius the cull tests: the box's half-diagonal.
                    out[3] = static_cast<float>(length(decal.half_extents));

                    out[4] = static_cast<float>(decal.right.x);
                    out[5] = static_cast<float>(decal.right.y);
                    out[6] = static_cast<float>(decal.right.z);
                    out[7] = static_cast<float>(decal.half_extents.x);

                    out[8] = static_cast<float>(decal.up.x);
                    out[9] = static_cast<float>(decal.up.y);
                    out[10] = static_cast<float>(decal.up.z);
                    out[11] = static_cast<float>(decal.half_extents.y);

                    out[12] = static_cast<float>(decal.forward.x);
                    out[13] = static_cast<float>(decal.forward.y);
                    out[14] = static_cast<float>(decal.forward.z);
                    out[15] = static_cast<float>(decal.half_extents.z);

                    out[16] = static_cast<float>(decal.color.x);
                    out[17] = static_cast<float>(decal.color.y);
                    out[18] = static_cast<float>(decal.color.z);
                    out[19] = decal.opacity;

                    // Bindless map indices, resolved like a material's maps. The raw uint
                    // bits are stored in the float lanes (the shader reads them back with
                    // floatBitsToUint), with 0xFFFFFFFF meaning "unset — use the tint / the
                    // surface's own response". Lane 21 (a normal map) is reserved unset: a
                    // projected normal decal needs an orientation-signed tangent blend that
                    // is a later increment.
                    const auto pack_index = [&](std::size_t lane, TextureId map)
                    {
                        std::uint32_t index =
                            (map == INVALID_TEXTURE)
                                ? 0xFFFFFFFFu
                                : textures.heap_index(map, Assets::DefaultTexture::White);
                        std::memcpy(&out[lane], &index, sizeof(std::uint32_t));
                    };
                    pack_index(20, decal.albedo_map);
                    const std::uint32_t no_normal = 0xFFFFFFFFu;
                    std::memcpy(&out[21], &no_normal, sizeof(std::uint32_t));
                    pack_index(22, decal.orm_map);
                    out[23] = 0.0f;
                }
            }

            void LightSystem::set_config(std::uint32_t render_width, std::uint32_t render_height,
                                         float near_plane, float far_plane)
            {
                const float near_clamped = std::max(near_plane, 0.01f);
                const float far_clamped = std::max(far_plane, near_clamped * 1.001f);

                config_.grid[0] = static_cast<float>(CLUSTER_X);
                config_.grid[1] = static_cast<float>(CLUSTER_Y);
                config_.grid[2] = static_cast<float>(CLUSTER_Z);
                config_.grid[3] = static_cast<float>(light_count_);

                // Logarithmic depth slices, the Olsson clustered mapping: slice thickness
                // grows with distance so it tracks the eye's depth precision. The shading
                // pass recovers a fragment's slice with floor(log(z) * scale + bias).
                const float log_ratio = std::log(far_clamped / near_clamped);
                const float scale = static_cast<float>(CLUSTER_Z) / log_ratio;
                const float bias = -std::log(near_clamped) * scale;
                config_.depth[0] = near_clamped;
                config_.depth[1] = far_clamped;
                config_.depth[2] = scale;
                config_.depth[3] = bias;

                config_.screen[0] = static_cast<float>(render_width);
                config_.screen[1] = static_cast<float>(render_height);
                config_.screen[2] = static_cast<float>(render_width) / static_cast<float>(CLUSTER_X);
                config_.screen[3] = static_cast<float>(render_height) / static_cast<float>(CLUSTER_Y);

                config_.counts[0] = static_cast<float>(decal_count_);
            }

            void LightSystem::set_stochastic_shadows(std::uint32_t samples, float max_metres,
                                                     float softness, std::uint32_t field_slot,
                                                     const float origin_voxel[4],
                                                     const std::int32_t resolution[4]) noexcept
            {
                config_.stochastic[0] = static_cast<float>(samples);
                config_.stochastic[1] = max_metres;
                config_.stochastic[2] = softness;
                config_.stochastic[3] = static_cast<float>(field_slot);
                for (int i = 0; i < 4; ++i)
                {
                    config_.sdf_origin[i] = origin_voxel[i];
                    config_.sdf_resolution[i] = static_cast<float>(resolution[i]);
                }
            }

            void LightSystem::assign_shadows(const PunctualLight* lights, std::size_t count,
                                             const double eye[3], std::uint32_t atlas_size,
                                             std::uint32_t max_casters, float near_plane)
            {
                shadow_packed_.clear();
                shadow_tiles_.clear();
                if (atlas_size == 0 || max_casters == 0)
                    return;

                const std::uint32_t tile_capacity =
                    SHADOW_TILES_PER_SIDE * SHADOW_TILES_PER_SIDE;
                const float inv_side = 1.0f / static_cast<float>(SHADOW_TILES_PER_SIDE);
                const std::uint32_t tile_size = atlas_size / SHADOW_TILES_PER_SIDE;
                const double near_d = std::max(static_cast<double>(near_plane), 0.02);

                // The six cube-map faces: a look direction and a valid up for each, in the
                // world axes the shading pass selects a face from (camera-relative only
                // translates, never rotates, so ±X here is ±X there). Order is +X,-X,+Y,-Y,
                // +Z,-Z — matched exactly by cube_shadow_face() in clustered_lighting.glsl.
                static const Vector3 kFaceDir[6] = {{1.0, 0.0, 0.0},  {-1.0, 0.0, 0.0},
                                                    {0.0, 1.0, 0.0},  {0.0, -1.0, 0.0},
                                                    {0.0, 0.0, 1.0},  {0.0, 0.0, -1.0}};
                static const Vector3 kFaceUp[6] = {{0.0, 1.0, 0.0}, {0.0, 1.0, 0.0},
                                                   {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0},
                                                   {0.0, 1.0, 0.0}, {0.0, 1.0, 0.0}};

                const std::uint32_t limit =
                    static_cast<std::uint32_t>(std::min<std::size_t>(count, light_count_));
                std::uint32_t casters_used = 0;
                for (std::uint32_t i = 0; i < limit; ++i)
                {
                    const PunctualLight& light = lights[i];
                    if (!light.casts_shadows)
                        continue;
                    if (light.type != LightType::Spot && light.type != LightType::Point)
                        continue;
                    if (casters_used >= max_casters)
                        break;

                    // A spot claims one tile; a point light claims six — one perspective
                    // face per cube direction, rendered into six atlas tiles and sampled by
                    // the fragment's dominant axis. If six tiles do not remain, skip this
                    // caster (a later one-tile spot may still fit) rather than break.
                    const std::uint32_t faces =
                        (light.type == LightType::Point) ? 6u : 1u;
                    const std::uint32_t first_slot =
                        static_cast<std::uint32_t>(shadow_tiles_.size());
                    if (first_slot + faces > tile_capacity)
                        continue;

                    // Built camera-relative, exactly as the sun cascades are: the eye is
                    // the origin, so the map for a light six million metres out is still
                    // small numbers.
                    const Vector3 pos_rel{light.position.x - eye[0], light.position.y - eye[1],
                                          light.position.z - eye[2]};
                    const double far_d =
                        std::max(static_cast<double>(light.range), near_d * 2.0);

                    for (std::uint32_t f = 0; f < faces; ++f)
                    {
                        const std::uint32_t slot_index =
                            static_cast<std::uint32_t>(shadow_tiles_.size());
                        const std::uint32_t col = slot_index % SHADOW_TILES_PER_SIDE;
                        const std::uint32_t row = slot_index / SHADOW_TILES_PER_SIDE;

                        Vector3 look_dir;
                        Vector3 up;
                        double fov;
                        if (faces == 6u)
                        {
                            look_dir = kFaceDir[f];
                            up = kFaceUp[f];
                            fov = 1.57079632679; // 90°: cube faces tile the sphere exactly
                        }
                        else
                        {
                            look_dir = normalize(light.direction);
                            up = std::abs(look_dir.y) > 0.99 ? Vector3{0.0, 0.0, 1.0}
                                                             : Vector3{0.0, 1.0, 0.0};
                            // Full cone, a touch wider so the edge penumbra is not clipped;
                            // capped below a degenerate 180°.
                            fov = std::min(static_cast<double>(light.outer_cone) * 2.2, 3.05);
                        }
                        const Vector3 center{pos_rel.x + look_dir.x, pos_rel.y + look_dir.y,
                                             pos_rel.z + look_dir.z};
                        const Mat4 view = look_at(pos_rel, center, up);
                        const Mat4 proj = shadow_perspective(fov, near_d, far_d);
                        const Mat4 view_proj = mul(proj, view);

                        const std::size_t base = shadow_packed_.size();
                        shadow_packed_.resize(base + SHADOW_RECORD_FLOATS);
                        float* out = shadow_packed_.data() + base;
                        for (int m = 0; m < 16; ++m)
                            out[m] = static_cast<float>(view_proj.m[m]);
                        // The tile's uv rect in the atlas: the shader maps a light-clip
                        // position to ndc, to [0,1], then into this rect.
                        out[16] = static_cast<float>(col) * inv_side;
                        out[17] = static_cast<float>(row) * inv_side;
                        out[18] = inv_side;
                        out[19] = 0.0f;

                        shadow_tiles_.push_back(
                            ShadowTile{col * tile_size, row * tile_size, tile_size, slot_index});
                    }

                    // The packed light's shadow index (cone.z lane) points at the caster's
                    // first record: a spot's only tile, or a point light's +X face, off
                    // which the shader adds the selected face.
                    packed_[static_cast<std::size_t>(i) * GPU_LIGHT_FLOATS + 14] =
                        static_cast<float>(first_slot);
                    ++casters_used;
                }
            }

            void LightSystem::upload()
            {
                Slot& slot = slots_[current_slot_];
                // Always keep room for at least one light: a zero-range storage buffer is
                // not a legal descriptor, and a frame with no lights still writes the set.
                const std::size_t floats =
                    std::max<std::size_t>(packed_.size(), GPU_LIGHT_FLOATS);
                grow_lights(slot, floats * sizeof(float));
                if (!packed_.empty())
                    std::memcpy(slot.lights_mapped, packed_.data(),
                                packed_.size() * sizeof(float));
                std::memcpy(slot.config_mapped, &config_, sizeof(LightClusterUniforms));

                // Always keep room for one shadow record so the descriptor's range is legal
                // even in a frame with no casters.
                const std::size_t shadow_floats =
                    std::max<std::size_t>(shadow_packed_.size(), SHADOW_RECORD_FLOATS);
                grow_shadow(slot, shadow_floats * sizeof(float));
                if (!shadow_packed_.empty())
                    std::memcpy(slot.shadow_mapped, shadow_packed_.data(),
                                shadow_packed_.size() * sizeof(float));

                const std::size_t decal_floats =
                    std::max<std::size_t>(decal_packed_.size(), GPU_DECAL_FLOATS);
                grow_decals(slot, decal_floats * sizeof(float));
                if (!decal_packed_.empty())
                    std::memcpy(slot.decals_mapped, decal_packed_.data(),
                                decal_packed_.size() * sizeof(float));
            }

            VkBuffer LightSystem::decal_buffer() const noexcept
            {
                return slots_[current_slot_].decals;
            }

            VkDeviceSize LightSystem::decal_buffer_range() const noexcept
            {
                const std::size_t floats =
                    std::max<std::size_t>(decal_packed_.size(), GPU_DECAL_FLOATS);
                return static_cast<VkDeviceSize>(floats * sizeof(float));
            }

            VkBuffer LightSystem::shadow_buffer() const noexcept
            {
                return slots_[current_slot_].shadow;
            }

            VkDeviceSize LightSystem::shadow_buffer_range() const noexcept
            {
                const std::size_t floats =
                    std::max<std::size_t>(shadow_packed_.size(), SHADOW_RECORD_FLOATS);
                return static_cast<VkDeviceSize>(floats * sizeof(float));
            }

            VkBuffer LightSystem::light_buffer() const noexcept
            {
                return slots_[current_slot_].lights;
            }

            VkDeviceSize LightSystem::light_buffer_range() const noexcept
            {
                const std::size_t floats =
                    std::max<std::size_t>(packed_.size(), GPU_LIGHT_FLOATS);
                return static_cast<VkDeviceSize>(floats * sizeof(float));
            }

            VkBuffer LightSystem::config_buffer() const noexcept
            {
                return slots_[current_slot_].config;
            }
        } // namespace Lighting
    } // namespace Render
} // namespace SushiEngine
