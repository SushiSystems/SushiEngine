/**************************************************************************/
/* light_system.hpp                                                       */
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

#pragma once

/**
 * @file light_system.hpp
 * @brief The per-frame GPU state of the clustered light engine.
 *
 * Owns two per-slot buffers, both host-written before the graph runs (so a slot still
 * in flight is never overwritten): the packed punctual-light array the cull pass and
 * the shading pass both read, and the small cluster-config block that tells both how
 * the froxel grid maps to this frame's screen and depth range. Packing mirrors
 * @ref MotionSystem exactly — grow-only per slot, positions made camera-relative in
 * double before the float cast — because a light's position is planet-scale metres for
 * the same reason a mesh's is.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/light.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Assets
        {
            class TextureLibrary;
        }

        namespace Lighting
        {
            /**
             * @brief The cluster-config block, mirroring @c ClusterBlock in the shaders.
             *
             * Flat float lanes so the C++ side cannot disagree with the std140 packing.
             * The grid *dimensions* are compile-time (see @ref cluster_config.hpp); what
             * varies each frame — the light count, the near/far the depth slices span, and
             * the screen tile size — lives here so the shading pass can map a pixel to its
             * cluster and read back the right light list.
             */
            struct LightClusterUniforms
            {
                float grid[4];   /**< x,y,z = CLUSTER_X/Y/Z as float, w = active light count. */
                float depth[4];  /**< near, far, log-slice scale, log-slice bias. */
                float screen[4]; /**< render width, height, tile size x, tile size y (pixels). */
                float counts[4]; /**< x = active decal count, yzw spare. */
            };

            /**
             * @brief Packs this frame's lights and cluster config into GPU buffers.
             *
             * Non-copyable: it owns Vulkan buffers.
             */
            class LightSystem
            {
                public:
                    /**
                     * @brief Allocates one light buffer and one config buffer per frame slot.
                     * @param device      The live Vulkan device.
                     * @param frame_slots How many frames may be in flight.
                     */
                    LightSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~LightSystem();

                    LightSystem(const LightSystem&) = delete;
                    LightSystem& operator=(const LightSystem&) = delete;

                    /**
                     * @brief Starts a frame: selects the slot and clears the packed array.
                     * @param slot Which frame slot is being recorded.
                     */
                    void begin_frame(std::uint32_t slot);

                    /**
                     * @brief Packs the frame's lights, camera-relative against @p eye.
                     *
                     * A count above @p max_lights is clamped: the light buffer never grows
                     * past the tier ceiling, so an over-full scene drops its surplus
                     * (bounded, and the editor can surface it) rather than spending
                     * unbounded cull time.
                     *
                     * @param lights     The scene's punctual lights.
                     * @param count      How many entries @p lights holds.
                     * @param eye        Camera world position, metres; subtracted in double.
                     * @param max_lights Tier ceiling on the packed count.
                     */
                    void pack(const PunctualLight* lights, std::size_t count, const double eye[3],
                              std::uint32_t max_lights);

                    /**
                     * @brief Packs the frame's decals, camera-relative against @p eye.
                     *
                     * The same grow-only, eye-subtracted-in-double packing as @ref pack; the
                     * cull pass tests each decal's bounding sphere against the froxel grid
                     * and the shading pass projects the survivors.
                     *
                     * A decal's optional @c albedo_map / @c orm_map @ref TextureId is
                     * resolved to a bindless heap index here (the same way a material map is),
                     * or the sentinel @c 0xFFFFFFFF where unset, so the shading pass can
                     * sample it straight from the bindless heap.
                     *
                     * @param decals     The scene's projected decals.
                     * @param count      How many entries @p decals holds.
                     * @param eye        Camera world position, metres; subtracted in double.
                     * @param max_decals Ceiling on the packed count.
                     * @param textures   Library that maps a decal's @ref TextureId to a heap slot.
                     */
                    void pack_decals(const Decal* decals, std::size_t count, const double eye[3],
                                     std::uint32_t max_decals,
                                     const Assets::TextureLibrary& textures);

                    /**
                     * @brief Fills the cluster-config block for this frame's grid.
                     *
                     * Computes the logarithmic depth-slice scale/bias and the screen tile
                     * size the shading pass needs, and records the near/far the cull pass
                     * builds its froxel bounds from — one source of truth for both.
                     *
                     * @param render_width  Internal render width in pixels.
                     * @param render_height Internal render height in pixels.
                     * @param near_plane    Nearest view-space distance the grid covers, metres.
                     * @param far_plane     Farthest view-space distance the grid covers, metres.
                     */
                    void set_config(std::uint32_t render_width, std::uint32_t render_height,
                                    float near_plane, float far_plane);

                    /**
                     * @brief Assigns shadow atlas tiles to punctual casters and builds maps.
                     *
                     * Must be called after @ref pack (it patches the packed lights' shadow
                     * index). Walks the packed lights in order and, for each shadow-casting
                     * light up to the caster budget, claims tiles in the 4×4 atlas grid — one
                     * for a spot, six (a perspective cube) for a point light — builds their
                     * camera-relative perspective matrices, and records the tile placements
                     * the shadow pass renders into. A caster whose tiles do not fit the
                     * remaining budget is skipped (shaded unshadowed), not partially placed.
                     *
                     * @param lights      The same array passed to @ref pack.
                     * @param count       Its length.
                     * @param eye         Camera world position; subtracted in double.
                     * @param atlas_size  Side of the square shadow atlas, texels (0 disables).
                     * @param max_casters Shadow-casting spots that may claim a tile.
                     * @param near_plane  Near distance of the spot light projections, metres.
                     */
                    void assign_shadows(const PunctualLight* lights, std::size_t count,
                                        const double eye[3], std::uint32_t atlas_size,
                                        std::uint32_t max_casters, float near_plane);

                    /** @brief One shadow caster's tile placement in the atlas, in texels. */
                    struct ShadowTile
                    {
                        std::uint32_t x = 0;    /**< Tile origin x, texels. */
                        std::uint32_t y = 0;    /**< Tile origin y, texels. */
                        std::uint32_t size = 0; /**< Tile side, texels. */
                        std::uint32_t index = 0;/**< Which shadow record the tile renders. */
                    };

                    /** @brief How many spot casters claimed a tile this frame. */
                    std::uint32_t shadow_caster_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(shadow_tiles_.size());
                    }

                    /** @brief The i-th shadow caster's tile placement. */
                    const ShadowTile& shadow_tile(std::uint32_t i) const noexcept
                    {
                        return shadow_tiles_[i];
                    }

                    /** @brief The buffer holding this slot's shadow matrices + tile rects. */
                    VkBuffer shadow_buffer() const noexcept;

                    /** @brief Bytes of the shadow buffer the descriptor must expose. */
                    VkDeviceSize shadow_buffer_range() const noexcept;

                    /** @brief Copies the packed lights and config into this slot's buffers. */
                    void upload();

                    /** @brief The buffer holding this slot's packed lights. */
                    VkBuffer light_buffer() const noexcept;

                    /** @brief Bytes of the light buffer the descriptor must expose. */
                    VkDeviceSize light_buffer_range() const noexcept;

                    /** @brief The buffer holding this slot's cluster-config block. */
                    VkBuffer config_buffer() const noexcept;

                    /** @brief Bytes of the config buffer the descriptor must expose. */
                    VkDeviceSize config_buffer_range() const noexcept { return sizeof(LightClusterUniforms); }

                    /** @brief How many lights were packed this frame (post-clamp). */
                    std::uint32_t light_count() const noexcept { return light_count_; }

                    /** @brief The buffer holding this slot's packed decals. */
                    VkBuffer decal_buffer() const noexcept;

                    /** @brief Bytes of the decal buffer the descriptor must expose. */
                    VkDeviceSize decal_buffer_range() const noexcept;

                    /** @brief How many decals were packed this frame (post-clamp). */
                    std::uint32_t decal_count() const noexcept { return decal_count_; }

                    /** @brief Near view distance the froxel grid spans this frame, metres. */
                    float cluster_near() const noexcept { return config_.depth[0]; }

                    /** @brief Far view distance the froxel grid spans this frame, metres. */
                    float cluster_far() const noexcept { return config_.depth[1]; }

                private:
                    /** @brief One frame slot's light + config buffers. */
                    struct Slot
                    {
                        VkBuffer lights = VK_NULL_HANDLE;
                        VmaAllocation lights_alloc = VK_NULL_HANDLE;
                        void* lights_mapped = nullptr;
                        VkDeviceSize lights_capacity = 0;

                        VkBuffer config = VK_NULL_HANDLE;
                        VmaAllocation config_alloc = VK_NULL_HANDLE;
                        void* config_mapped = nullptr;

                        VkBuffer shadow = VK_NULL_HANDLE;
                        VmaAllocation shadow_alloc = VK_NULL_HANDLE;
                        void* shadow_mapped = nullptr;
                        VkDeviceSize shadow_capacity = 0;

                        VkBuffer decals = VK_NULL_HANDLE;
                        VmaAllocation decals_alloc = VK_NULL_HANDLE;
                        void* decals_mapped = nullptr;
                        VkDeviceSize decals_capacity = 0;
                    };

                    void grow_lights(Slot& slot, VkDeviceSize bytes);
                    void grow_shadow(Slot& slot, VkDeviceSize bytes);
                    void grow_decals(Slot& slot, VkDeviceSize bytes);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Slot> slots_;
                    std::vector<float> packed_;
                    std::vector<float> decal_packed_;
                    std::vector<float> shadow_packed_;
                    std::vector<ShadowTile> shadow_tiles_;
                    LightClusterUniforms config_{};
                    std::uint32_t light_count_ = 0;
                    std::uint32_t decal_count_ = 0;
                    std::uint32_t current_slot_ = 0;
            };
        } // namespace Lighting
    } // namespace Render
} // namespace SushiEngine
