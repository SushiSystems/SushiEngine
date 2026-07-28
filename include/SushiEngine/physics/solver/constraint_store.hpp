/**************************************************************************/
/* constraint_store.hpp                                                   */
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
 * @file constraint_store.hpp
 * @brief Where a constraint lives, who it belongs to, and what colour it takes.
 *
 * Split out from the solver because it is not solver-specific. A host solver and a
 * runtime-backed one must agree on *layout* — which colour each constraint took and
 * where in that colour's band it sits — or they cannot be compared at all: the
 * projection is Gauss-Seidel, so a different order is a different answer, and a
 * conformance test between two solvers that ordered their constraints differently
 * would be measuring the ordering rather than the execution.
 *
 * So the ordering rule lives here, once, and both solvers read it. What differs
 * between them is only *how* they walk it, which is exactly what the conformance
 * suite is meant to hold constant.
 *
 * ### The layout
 *
 * The constraint capacity is divided into one fixed band per colour. Within a band
 * the live constraints are dense `[0, count)`, kept dense by swapping the last one
 * into a vacated slot. Density is what lets a colour's kernel iterate its live range
 * with no per-element liveness test — the alternative is a wasted lane for every
 * hole, and holes accumulate as a scene churns.
 *
 * Swapping means a constraint's storage slot moves, so a handle addresses a
 * *handle* slot and this maps it to storage. That indirection is the price of
 * density, and it is paid on the host, once per mutation, rather than on the device
 * once per element per substep.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/solver/incremental_coloring.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief What an insertion did, or why it could not be done.
         *
         * A failure here is a budget being exceeded, not an error: the caller counts
         * it and carries on, because a scene that spawned one body too many should
         * report that and keep running rather than stop.
         */
        struct ConstraintPlacement
        {
            /** @brief The handle the caller keeps; invalid when the insertion failed. */
            ConstraintHandle handle;

            /** @brief The colour taken, meaningful only when @ref handle is valid. */
            std::uint32_t color = 0;

            /** @brief The storage slot to write the descriptor into. */
            std::size_t slot = 0;
        };

        /**
         * @brief What a removal moved, so the caller can mirror it in its storage.
         *
         * The store owns the bookkeeping but not the descriptors: the caller holds
         * those, possibly in a device buffer it cannot read. So a removal reports the
         * copy that has to happen rather than performing it.
         */
        struct ConstraintRemoval
        {
            /** @brief Whether a live constraint was removed at all. */
            bool removed = false;

            /** @brief The slot that was vacated. */
            std::size_t slot = 0;

            /**
             * @brief The slot whose contents must move into @ref slot.
             *
             * Equal to @ref slot when the removed constraint was already the last in
             * its band, in which case nothing needs copying.
             */
            std::size_t moved_from = 0;
        };

        /**
         * @brief Slot allocation, colouring, and band density for a constraint set.
         *
         * Owns no constraint descriptors. That is deliberate: the descriptors may
         * live in a device-resident buffer the host cannot address, so this answers
         * *where* things go and leaves the writing to whoever owns the storage.
         */
        class ConstraintStore
        {
            public:
                /**
                 * @brief Creates a store for a fixed body, constraint and colour budget.
                 *
                 * @param body_capacity       How many body slots constraints may name.
                 * @param constraint_capacity How many constraints may be live at once.
                 * @param color_limit         The colour ceiling; bounded by the mask width.
                 */
                ConstraintStore(std::size_t body_capacity, std::size_t constraint_capacity,
                                std::size_t color_limit)
                    : slots_(constraint_capacity),
                      coloring_(body_capacity, color_limit),
                      color_count_(clamped_color_count(color_limit)),
                      band_capacity_(constraint_capacity /
                                     (color_count_ > 0 ? color_count_ : 1)),
                      band_live_(color_count_, 0),
                      handle_of_slot_(band_capacity_ * color_count_, 0),
                      slot_of_handle_(constraint_capacity, 0),
                      color_of_handle_(constraint_capacity, 0)
                {
                }

                /**
                 * @brief Finds a colour and a slot for a constraint between @p a and @p b.
                 *
                 * The colour is the lowest one free on both bodies; the slot is the
                 * next free position in that colour's band. Either can fail, and both
                 * failures unwind cleanly — in particular a full band gives the colour
                 * back, or the bodies would carry a colour no constraint holds and the
                 * next assignment would skip it for ever.
                 *
                 * @param a The first body's slot index.
                 * @param b The second body's slot index.
                 * @return Where to write the constraint, or an invalid handle.
                 */
                ConstraintPlacement place(std::uint32_t a, std::uint32_t b)
                {
                    ConstraintPlacement placement;

                    const std::uint32_t color = coloring_.assign(a, b);
                    if (color == IncrementalColoring::NO_COLOR)
                        return placement;

                    if (band_live_[color] >= band_capacity_)
                    {
                        coloring_.release(a, b, color);
                        return placement;
                    }

                    const ConstraintHandle handle = slots_.allocate();
                    if (!handle.valid())
                    {
                        coloring_.release(a, b, color);
                        return placement;
                    }

                    const std::size_t slot = std::size_t(color) * band_capacity_ +
                                             band_live_[color];
                    ++band_live_[color];

                    slot_of_handle_[handle.index] = std::uint32_t(slot);
                    color_of_handle_[handle.index] = color;
                    handle_of_slot_[slot] = handle.index;

                    placement.handle = handle;
                    placement.color = color;
                    placement.slot = slot;
                    return placement;
                }

                /**
                 * @brief Releases @p handle, reporting the compaction the caller must mirror.
                 *
                 * @param handle The constraint to remove.
                 * @param a      The removed constraint's first body slot.
                 * @param b      The removed constraint's second body slot.
                 * @return What moved, so the caller can copy its descriptor to match.
                 */
                ConstraintRemoval remove(ConstraintHandle handle, std::uint32_t a,
                                         std::uint32_t b)
                {
                    ConstraintRemoval removal;
                    if (!slots_.alive(handle))
                        return removal;

                    const std::uint32_t color = color_of_handle_[handle.index];
                    const std::size_t slot = slot_of_handle_[handle.index];
                    const std::size_t base = std::size_t(color) * band_capacity_;
                    const std::size_t last = base + band_live_[color] - 1;

                    coloring_.release(a, b, color);

                    if (slot != last)
                    {
                        const std::uint32_t moved = handle_of_slot_[last];
                        handle_of_slot_[slot] = moved;
                        slot_of_handle_[moved] = std::uint32_t(slot);
                    }

                    --band_live_[color];
                    slots_.release(handle);

                    removal.removed = true;
                    removal.slot = slot;
                    removal.moved_from = last;
                    return removal;
                }

                /** @brief Whether @p handle still names a live constraint. */
                bool alive(ConstraintHandle handle) const noexcept
                {
                    return slots_.alive(handle);
                }

                /**
                 * @brief The storage slot @p handle currently occupies.
                 *
                 * The reverse of @ref handle_at, and the reason the indirection is
                 * kept in both directions: a caller that owns the descriptors needs
                 * to find one from a handle without scanning, and a caller walking
                 * storage needs the handle without a lookup table of its own.
                 *
                 * @param handle The constraint to locate.
                 * @return Its storage slot, or @ref capacity when the handle is stale.
                 */
                std::size_t slot_of(ConstraintHandle handle) const noexcept
                {
                    if (!slots_.alive(handle))
                        return handle_of_slot_.size();
                    return slot_of_handle_[handle.index];
                }

                /** @brief The live handle occupying storage slot @p slot. */
                ConstraintHandle handle_at(std::size_t slot) const noexcept
                {
                    if (slot >= handle_of_slot_.size())
                        return ConstraintHandle{};
                    return slots_.handle_of(handle_of_slot_[slot]);
                }

                /** @brief How many colours the layout was built with. */
                std::size_t color_count() const noexcept { return color_count_; }

                /** @brief How many constraints one colour may hold. */
                std::size_t band_capacity() const noexcept { return band_capacity_; }

                /** @brief The first storage slot of colour @p color. */
                std::size_t band_base(std::size_t color) const noexcept
                {
                    return color * band_capacity_;
                }

                /** @brief Live constraints in colour @p color. */
                std::size_t band_size(std::size_t color) const noexcept
                {
                    return color < band_live_.size() ? band_live_[color] : 0;
                }

                /** @brief Live constraints across every colour. */
                std::size_t live_count() const noexcept { return slots_.live_count(); }

                /** @brief The fixed number of constraint slots. */
                std::size_t capacity() const noexcept { return slots_.capacity(); }

                /** @brief How many colours have been used since the last full recolour. */
                std::size_t colors_used() const noexcept
                {
                    return coloring_.highest_used();
                }

            private:
                /** @brief The colour count actually used, bounded by the mask width. */
                static std::size_t clamped_color_count(std::size_t requested) noexcept
                {
                    if (requested == 0)
                        return 1;
                    return requested < IncrementalColoring::MAXIMUM_COLORS
                               ? requested
                               : IncrementalColoring::MAXIMUM_COLORS;
                }

                HandleTable<ConstraintTag> slots_;
                IncrementalColoring coloring_;
                std::size_t color_count_;
                std::size_t band_capacity_;
                std::vector<std::size_t> band_live_;
                std::vector<std::uint32_t> handle_of_slot_;
                std::vector<std::uint32_t> slot_of_handle_;
                std::vector<std::uint32_t> color_of_handle_;
        };
    } // namespace Physics
} // namespace SushiEngine
