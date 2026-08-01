/**************************************************************************/
/* tile_residency.hpp                                                     */
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
 * @file tile_residency.hpp
 * @brief Which cache slot a node reads its heights from, and where inside it.
 *
 * The bookkeeping half of the tile cache (`docs/slop/solar_system_overhaul.md` §7.2),
 * kept away from the graphics API on purpose. Everything here is arithmetic — a slot
 * table, a least-recently-used clock, and the rectangle a node occupies inside whichever
 * tile actually answers for it — and none of it wants a device to be checked.
 *
 * **Inheritance is the load-bearing part.** A node whose own tile is not resident reads
 * its nearest resident ancestor's slot through a scaled UV rectangle instead. That single
 * mechanism is what makes streaming pop-free (a node draws at lower detail rather than not
 * at all), lets the cache be smaller than the visible set without ever stalling a frame,
 * and gives the quadtree something to draw past the depth the data supports. It is also
 * exactly where an off-by-half-a-texel produces terrain that is subtly misaligned
 * everywhere and miserable to diagnose on a GPU, which is why the rectangle is a pure
 * function with its own test rather than four lines inside an upload path.
 */

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /** @brief The slot index meaning "no slot". */
        constexpr std::uint32_t INVALID_TILE_SLOT = 0xFFFFFFFFu;

        /**
         * @brief The rectangle a node's grid occupies inside a resident tile's image.
         *
         * A node's grid parameter `a` in [0, 1] samples the slot at
         * `offset + a * scale`. The offsets account for the apron: a tile's own grid
         * starts one texel in, so even the identity case is not `(0, 1)`.
         */
        struct TileUvRect
        {
            float offset_s = 0.0f;
            float offset_t = 0.0f;
            float scale_s = 1.0f;
            float scale_t = 1.0f;
        };

        /**
         * @brief The UV rectangle a node's grid occupies inside @p resident's image.
         *
         * @param node     The tile being drawn.
         * @param resident The tile whose image is actually being read; @p node itself, or
         *                 one of its ancestors.
         * @return The rectangle. Identity — the node reading its own tile — is
         *         `offset = 1.5/131`, `scale = 128/131`, which is the apron offset and the
         *         grid's share of the stored image rather than the whole image.
         */
        inline TileUvRect tile_uv_rect(const TileAddress& node,
                                       const TileAddress& resident) noexcept
        {
            const double texels = static_cast<double>(TILE_STRIDE);
            const double cells = static_cast<double>(TILE_GRID_SIZE - 1u);
            const double apron = static_cast<double>(TILE_APRON);

            const TileGridRect inner = tile_grid_rect(node);
            const TileGridRect outer = tile_grid_rect(resident);
            const double span_s = outer.s_maximum - outer.s_minimum;
            const double span_t = outer.t_maximum - outer.t_minimum;

            // Where the node's grid sits inside the resident tile's grid, in [0, 1].
            const double first_s = (inner.s_minimum - outer.s_minimum) / span_s;
            const double last_s = (inner.s_maximum - outer.s_minimum) / span_s;
            const double first_t = (inner.t_minimum - outer.t_minimum) / span_t;
            const double last_t = (inner.t_maximum - outer.t_minimum) / span_t;

            TileUvRect rect;
            rect.offset_s = static_cast<float>((apron + 0.5 + first_s * cells) / texels);
            rect.offset_t = static_cast<float>((apron + 0.5 + first_t * cells) / texels);
            rect.scale_s = static_cast<float>((last_s - first_s) * cells / texels);
            rect.scale_t = static_cast<float>((last_t - first_t) * cells / texels);
            return rect;
        }

        /** @brief What a node was bound to for a frame. */
        struct TileBinding
        {
            /** @brief The slot to sample; @ref INVALID_TILE_SLOT when nothing covers it. */
            std::uint32_t slot = INVALID_TILE_SLOT;

            /** @brief The tile actually resident there: the node's own, or an ancestor's. */
            TileAddress source{};

            /** @brief Where the node's grid sits inside that slot. */
            TileUvRect rect{};

            /** @brief The range the slot's normalized texels decode against, metres. */
            float minimum_metres = 0.0f;
            float maximum_metres = 0.0f;

            /** @brief Whether the node got its own tile rather than an ancestor's. */
            bool exact = false;
        };

        /**
         * @brief The slot table: what is resident, what may be evicted, and what a node reads.
         *
         * Fixed capacity, chosen once from the memory budget. Eviction is least-recently-
         * *bound* rather than least-recently-uploaded, because what matters is whether a
         * slot is still being drawn from, and a slot inherited by twenty descendants is
         * touched by all twenty.
         */
        class TileResidency
        {
            public:
                /**
                 * @brief Creates an empty table.
                 *
                 * @param slot_count       How many slots the cache image carries.
                 * @param frames_in_flight How many frames may be recorded before the
                 *                         oldest of them has certainly finished. A slot
                 *                         drawn from within that window is never evicted:
                 *                         re-pointing it would overwrite an image the
                 *                         device is still reading. One means "this frame
                 *                         only", which is right for a caller that submits
                 *                         and waits, and wrong for every real frame chain.
                 */
                explicit TileResidency(std::uint32_t slot_count,
                                       std::uint32_t frames_in_flight = 1)
                    : slots_(slot_count),
                      retire_frames_(frames_in_flight > 0u ? frames_in_flight : 1u)
                {
                    free_slots_.reserve(slot_count);
                    for (std::uint32_t slot = slot_count; slot > 0u; --slot)
                        free_slots_.push_back(slot - 1u);
                }

                /**
                 * @brief Forgets every tile, returning all slots to the free list.
                 *
                 * For re-pointing the pool at a different body: the slots are anonymous
                 * storage, so the pool survives and only its index is wrong. The image
                 * keeps whatever pixels it held, which is harmless because no node can
                 * bind a slot that has not been staged since.
                 *
                 * @warning The caller must know the device is idle. Unlike eviction, which
                 *          refuses slots inside the in-flight window, this drops the whole
                 *          window on the floor.
                 */
                void clear()
                {
                    lookup_.clear();
                    free_slots_.clear();
                    free_slots_.reserve(slots_.size());
                    for (std::size_t index = slots_.size(); index > 0u; --index)
                    {
                        slots_[index - 1u] = Slot{};
                        free_slots_.push_back(static_cast<std::uint32_t>(index - 1u));
                    }
                }

                /** @brief How many slots exist. */
                std::uint32_t capacity() const noexcept
                {
                    return static_cast<std::uint32_t>(slots_.size());
                }

                /** @brief How many slots currently hold a tile. */
                std::uint32_t resident_count() const noexcept
                {
                    return static_cast<std::uint32_t>(lookup_.size());
                }

                /**
                 * @brief Opens a frame, so eviction can tell this frame's slots from older ones.
                 * @param frame Monotonic frame index.
                 */
                void begin_frame(std::uint64_t frame) noexcept { frame_ = frame; }

                /**
                 * @brief The slot holding a tile, without touching it.
                 * @param address The tile.
                 * @return Its slot, or @ref INVALID_TILE_SLOT.
                 */
                std::uint32_t find(const TileAddress& address) const
                {
                    const auto entry = lookup_.find(tile_address_key(address));
                    return entry == lookup_.end() ? INVALID_TILE_SLOT : entry->second;
                }

                /**
                 * @brief Binds a node to the deepest resident tile that covers it.
                 *
                 * Walks the node and then its ancestors, so a node whose own tile has not
                 * arrived yet draws its parent's surface instead of nothing. Every slot the
                 * walk lands on is touched, which is what keeps an ancestor that many
                 * descendants are inheriting from being evicted underneath them.
                 *
                 * @param address The node being drawn.
                 * @param binding Receives the slot, the rectangle, and the decode range.
                 * @return Whether anything covered the node at all.
                 */
                bool bind(const TileAddress& address, TileBinding& binding)
                {
                    TileAddress walk = address;
                    for (;;)
                    {
                        const auto entry = lookup_.find(tile_address_key(walk));
                        if (entry != lookup_.end())
                        {
                            Slot& slot = slots_[entry->second];
                            slot.last_used_frame = frame_;
                            binding.slot = entry->second;
                            binding.source = walk;
                            binding.rect = tile_uv_rect(address, walk);
                            binding.minimum_metres = slot.minimum_metres;
                            binding.maximum_metres = slot.maximum_metres;
                            binding.exact = walk == address;
                            return true;
                        }
                        if (walk.depth == 0)
                            break;
                        walk = tile_parent(walk);
                    }
                    binding = TileBinding{};
                    return false;
                }

                /**
                 * @brief Claims a slot for a tile, evicting the coldest one when full.
                 *
                 * A tile already resident keeps its slot and has its range updated, which is
                 * what a recompile after an edit needs. Eviction never takes a slot bound
                 * this frame: doing so would pull an image out from under a node already
                 * queued to draw from it.
                 *
                 * @param address        The tile being made resident.
                 * @param minimum_metres The range its normalized texels decode against.
                 * @param maximum_metres The other end of that range.
                 * @return The slot, or @ref INVALID_TILE_SLOT when every slot is in use this
                 *         frame and none may be taken.
                 */
                std::uint32_t insert(const TileAddress& address, float minimum_metres,
                                     float maximum_metres)
                {
                    const std::uint64_t key = tile_address_key(address);
                    const auto existing = lookup_.find(key);
                    if (existing != lookup_.end())
                    {
                        Slot& slot = slots_[existing->second];
                        slot.minimum_metres = minimum_metres;
                        slot.maximum_metres = maximum_metres;
                        slot.last_used_frame = frame_;
                        return existing->second;
                    }

                    std::uint32_t chosen = INVALID_TILE_SLOT;
                    if (!free_slots_.empty())
                    {
                        chosen = free_slots_.back();
                        free_slots_.pop_back();
                    }
                    else
                    {
                        std::uint64_t coldest = frame_;
                        for (std::uint32_t index = 0; index < slots_.size(); ++index)
                        {
                            const Slot& slot = slots_[index];
                            // Not just this frame's slots: every frame still in flight is
                            // reading from the ones it bound, and the device has no idea
                            // an upload is about to land on top of them.
                            if (!slot.occupied ||
                                slot.last_used_frame + retire_frames_ > frame_)
                                continue;
                            if (chosen == INVALID_TILE_SLOT || slot.last_used_frame < coldest)
                            {
                                coldest = slot.last_used_frame;
                                chosen = index;
                            }
                        }
                        if (chosen == INVALID_TILE_SLOT)
                            return INVALID_TILE_SLOT;
                        lookup_.erase(tile_address_key(slots_[chosen].address));
                    }

                    Slot& slot = slots_[chosen];
                    slot.address = address;
                    slot.minimum_metres = minimum_metres;
                    slot.maximum_metres = maximum_metres;
                    slot.last_used_frame = frame_;
                    slot.occupied = true;
                    lookup_[key] = chosen;
                    return chosen;
                }

                /**
                 * @brief Drops a tile, freeing its slot.
                 * @param address The tile to evict.
                 * @return Whether it was resident.
                 */
                bool evict(const TileAddress& address)
                {
                    const auto entry = lookup_.find(tile_address_key(address));
                    if (entry == lookup_.end())
                        return false;
                    const std::uint32_t slot = entry->second;
                    slots_[slot].occupied = false;
                    free_slots_.push_back(slot);
                    lookup_.erase(entry);
                    return true;
                }

                /** @brief The tile a slot holds; only meaningful when it is occupied. */
                TileAddress slot_address(std::uint32_t slot) const
                {
                    return slots_[slot].address;
                }

                /** @brief Whether a slot holds a tile. */
                bool slot_occupied(std::uint32_t slot) const { return slots_[slot].occupied; }

            private:
                struct Slot
                {
                    TileAddress address{};
                    float minimum_metres = 0.0f;
                    float maximum_metres = 0.0f;
                    std::uint64_t last_used_frame = 0;
                    bool occupied = false;
                };

                std::vector<Slot> slots_;
                std::vector<std::uint32_t> free_slots_;
                std::unordered_map<std::uint64_t, std::uint32_t> lookup_;
                std::uint64_t frame_ = 1;
                std::uint64_t retire_frames_ = 1;
        };
    } // namespace Terrain
} // namespace SushiEngine
