/**************************************************************************/
/* tile_cache.hpp                                                         */
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
 * @file tile_cache.hpp
 * @brief The height slot pool on the device, and getting tiles into it.
 *
 * The device half of the tile cache (`docs/design/solar_system_overhaul.md` §7.2). The
 * bookkeeping — which slot holds what, eviction, and the rectangle an inheriting node
 * reads through — is `Terrain::TileResidency` and is deliberately not here; this owns the
 * image, the staging, and nothing else.
 *
 * Two shapes are worth stating because they are choices rather than defaults.
 *
 * **The image is a plain array texture, not a sparse one.** Sparse residency is not among
 * the features the device is created with, and an explicit slot pool is simpler, portable,
 * and gives exact control over the memory budget — which is the property §17 is written
 * against.
 *
 * **Uploads are recorded into the frame's command buffer, not submitted on their own.**
 * The image is imported into the render graph, so the transfer-to-vertex-read barrier is
 * derived from the same declarations every other resource's is. A one-shot submit per
 * tile would stall the frame that flew somewhere new, which is exactly the frame that can
 * least afford it.
 *
 * **On the qualified names below.** This file's own namespace is `Render::Terrain`, which
 * hides the engine-wide `SushiEngine::Terrain` from unqualified lookup — so the neutral
 * terrain types are spelled in full here. That is deliberate, not clutter: dropping the
 * qualification does not silently pick the wrong type, it fails to compile, which is the
 * right side of that trade but only if nobody "tidies" it back.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/terrain/tile_address.hpp>
#include <SushiEngine/terrain/tile_residency.hpp>

#include "graph/resource_handle.hpp"
#include "graph/resource_state.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Terrain
        {
            /** @brief Elevation slots: one normalized channel, decoded per node. */
            constexpr VkFormat TERRAIN_HEIGHT_FORMAT = VK_FORMAT_R16_UNORM;

            /** @brief Frames the staging ring covers; matches the view's slot count. */
            constexpr std::uint32_t TERRAIN_FRAME_SLOTS = 3;

            /**
             * @brief Tiles uploaded per frame, at most.
             *
             * A bound rather than a target. Terrain never blocks a frame: what has not
             * arrived is inherited from an ancestor and drawn coarser, so the only cost of
             * a low bound is that detail resolves over several frames instead of one.
             */
            constexpr std::uint32_t TERRAIN_UPLOADS_PER_FRAME = 8;

            /**
             * @brief The device-side height pool and its staging ring.
             *
             * Non-copyable: it owns an image, a view, a sampler-independent allocation, and
             * one staging buffer per frame in flight.
             */
            class TileCache
            {
                public:
                    /**
                     * @brief Creates the slot image and the staging ring.
                     * @param device     The live Vulkan device.
                     * @param slot_count How many tiles may be resident at once.
                     */
                    TileCache(Vulkan::VulkanDevice& device, std::uint32_t slot_count);
                    ~TileCache();

                    TileCache(const TileCache&) = delete;
                    TileCache& operator=(const TileCache&) = delete;

                    /**
                     * @brief Opens a frame: ages the residency and resets this slot's staging.
                     * @param frame Monotonic frame index, so eviction can tell ages apart.
                     * @param slot  Which frame-in-flight slot is being recorded; the staging
                     *              buffer is per slot, because writing one the GPU is still
                     *              copying from would corrupt the tile it is copying.
                     */
                    void begin_frame(std::uint64_t frame, std::uint32_t slot);

                    /**
                     * @brief Binds a node to the deepest resident tile covering it.
                     * @param address The node being drawn.
                     * @param binding Receives the slot, rectangle, and decode range.
                     * @return Whether anything covered it; false means the node cannot draw.
                     */
                    bool bind(const SushiEngine::Terrain::TileAddress& address,
                              SushiEngine::Terrain::TileBinding& binding)
                    {
                        return residency_.bind(address, binding);
                    }

                    /** @brief The slot holding a tile, or INVALID_TILE_SLOT. */
                    std::uint32_t find(const SushiEngine::Terrain::TileAddress& address) const
                    {
                        return residency_.find(address);
                    }

                    /**
                     * @brief Forgets every resident tile, keeping the image.
                     *
                     * For re-pointing the pool at a different body. Carries
                     * @ref SushiEngine::Terrain::TileResidency::clear's precondition
                     * unchanged: the device must be idle.
                     */
                    void forget_all()
                    {
                        residency_.clear();
                        pending_.clear();
                    }

                    /** @brief Whether this frame can still take another upload. */
                    bool can_stage() const noexcept
                    {
                        return pending_.size() < TERRAIN_UPLOADS_PER_FRAME;
                    }

                    /**
                     * @brief Claims a slot for a tile and queues its pixels for upload.
                     *
                     * Quantises the elevations to the slot's normalized channel against the
                     * range it returns through the residency, so the node record's decode
                     * range and the pixels always come from one place.
                     *
                     * @param address        The tile being made resident.
                     * @param heights_metres @ref TILE_SAMPLE_COUNT elevations, apron included.
                     * @return The slot, or INVALID_TILE_SLOT when the frame's upload budget is
                     *         spent or every slot is already bound this frame.
                     */
                    std::uint32_t stage(const SushiEngine::Terrain::TileAddress& address,
                                        const float* heights_metres);

                    /**
                     * @brief Records this frame's queued copies.
                     * @param command The command buffer the upload pass is recording into.
                     */
                    void record_uploads(VkCommandBuffer command);

                    /** @brief How many uploads are queued this frame. */
                    std::size_t pending_uploads() const noexcept { return pending_.size(); }

                    /** @brief The slot image, for importing into the frame graph. */
                    VkImage image() const noexcept { return image_; }

                    /** @brief The array view the shaders sample. */
                    VkImageView view() const noexcept { return view_; }

                    /** @brief The description the graph imports it with. */
                    const Graph::TextureDescription& description() const noexcept
                    {
                        return description_;
                    }

                    /** @brief The layout/stage the graph left it in; it tracks this across frames. */
                    Graph::TextureState& state() noexcept { return state_; }

                    /** @brief How many slots exist. */
                    std::uint32_t slot_count() const noexcept { return residency_.capacity(); }

                    /** @brief How many slots currently hold a tile. */
                    std::uint32_t resident_count() const noexcept
                    {
                        return residency_.resident_count();
                    }

                    /**
                     * @brief Whether a slot holds a tile.
                     * @param slot Slot index, below @ref slot_count.
                     */
                    bool slot_occupied(std::uint32_t slot) const
                    {
                        return residency_.slot_occupied(slot);
                    }

                    /**
                     * @brief The tile a slot holds.
                     *
                     * With @ref slot_occupied, this is how a caller asks "what ground is
                     * currently compiled" — the question an edit has to ask, because the
                     * tiles an edit invalidates are exactly the resident ones it reaches.
                     *
                     * @param slot Slot index, below @ref slot_count; only meaningful while
                     *             @ref slot_occupied says so.
                     */
                    SushiEngine::Terrain::TileAddress slot_address(std::uint32_t slot) const
                    {
                        return residency_.slot_address(slot);
                    }

                    /** @brief Device memory the slot image occupies, bytes. */
                    std::size_t image_bytes() const noexcept;

                private:
                    /** @brief One queued copy: where in the staging buffer, and into which slot. */
                    struct PendingUpload
                    {
                        VkDeviceSize offset = 0;
                        std::uint32_t slot = 0;
                    };

                    /** @brief One frame slot's staging buffer. */
                    struct Staging
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                    };

                    Vulkan::VulkanDevice& device_;
                    SushiEngine::Terrain::TileResidency residency_;
                    VkImage image_ = VK_NULL_HANDLE;
                    VmaAllocation allocation_ = VK_NULL_HANDLE;
                    VkImageView view_ = VK_NULL_HANDLE;
                    Graph::TextureDescription description_{};
                    Graph::TextureState state_{};
                    Staging staging_[TERRAIN_FRAME_SLOTS]{};
                    std::vector<PendingUpload> pending_;
                    std::uint32_t frame_slot_ = 0;
            };
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
