/**************************************************************************/
/* contact_store.hpp                                                      */
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
 * @file contact_store.hpp
 * @brief Where a contact lives and what colour it takes, for one tick only.
 *
 * `ConstraintStore`'s counterpart for the kind whose set is rebuilt every tick.
 * The two share a purpose — decide the *layout*, so that a host solver and a
 * runtime-backed one cannot disagree about it — and differ in exactly one respect,
 * which is the whole reason this is a separate type rather than a flag on the other:
 *
 * **A contact has no handle and no lifetime.** Nothing outside a tick refers to it,
 * so there is no slot to keep stable, no swap-remove to mirror, and no generation to
 * validate. What survives a tick is the *manifold*, and that is the caller's to keep
 * (it is keyed by the broadphase pair cache, which is what makes warm starting
 * possible at all). Building handle machinery for something with a one-tick lifetime
 * would be paying the cost of persistence for a thing that is not persistent.
 *
 * ### The union colouring, and how it is layered rather than merged
 *
 * §6.3 requires colouring over the *union* of all constraint kinds: a distance
 * constraint and a contact that share a body must not share a colour, or the two
 * nodes that project them run concurrently and write the same body. But the
 * persistent kinds are coloured once, when they are added, and contacts are coloured
 * afresh every tick — so they cannot simply share one colourer, because clearing it
 * for the contacts would throw away the persistent assignments.
 *
 * So the persistent colouring is *read* rather than joined. Each tick a body's mask
 * starts as whatever the persistent colourer says it holds, and the contacts pile on
 * top. Seeding is lazy — a body's mask is copied the first time a contact names it,
 * stamped with the tick's epoch — because a scene with four thousand body slots and
 * forty contacts should not pay four thousand copies to colour forty things.
 *
 * ### The layout
 *
 * One fixed band per colour, live entries dense from the band's base, exactly as
 * `ConstraintStore` lays constraints out. Density is what lets a colour's kernel
 * iterate `[base, base + count)` with no per-element liveness test, and a fixed base
 * is what lets the solve graph name that band as an immutable buffer region at
 * compile time — the runtime allows a node's binding to change size, never which
 * resource it touches.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/physics/solver/incremental_coloring.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Where a contact was placed, or why it could not be.
         *
         * A failure is a budget being exceeded, not an error: a tick that produced
         * one contact more than the scene budgeted for should report that and resolve
         * the rest, because dropping a contact costs a little penetration and
         * stopping costs the frame.
         */
        struct ContactPlacement
        {
            /** @brief Whether the contact was placed at all. */
            bool placed = false;

            /** @brief The colour taken, meaningful only when @ref placed. */
            std::uint32_t color = 0;

            /** @brief The storage slot to write the contact into. */
            std::size_t slot = 0;
        };

        /**
         * @brief Per-tick colouring and band packing for the contact set.
         *
         * Owns no contact descriptors, for the same reason `ConstraintStore` owns no
         * constraint descriptors: they may live in a device-resident buffer the host
         * cannot address. This answers *where*, and leaves the writing to whoever owns
         * the storage.
         */
        class ContactStore
        {
            public:
                /**
                 * @brief Creates a store for a fixed body, contact and colour budget.
                 *
                 * @param body_capacity    How many body slots a contact may name.
                 * @param contact_capacity How many contacts one tick may hold.
                 * @param color_limit      The colour ceiling; bounded by the mask width.
                 */
                ContactStore(std::size_t body_capacity, std::size_t contact_capacity,
                             std::size_t color_limit)
                    : color_count_(clamped_color_count(color_limit)),
                      band_capacity_(contact_capacity /
                                     (color_count_ > 0 ? color_count_ : 1)),
                      band_live_(color_count_, 0),
                      masks_(body_capacity, 0),
                      stamp_(body_capacity, 0)
                {
                }

                /**
                 * @brief Discards last tick's contacts and starts a fresh set.
                 *
                 * The epoch advances rather than the masks being cleared: clearing
                 * would be a pass over every body slot in the scene every tick, and
                 * the masks that matter are re-seeded from the persistent colouring
                 * on first use anyway. A stamp that does not match the current epoch
                 * *is* an empty mask, which is the same statement for less work.
                 */
                void begin() noexcept
                {
                    for (std::size_t& live : band_live_)
                        live = 0;
                    live_count_ = 0;
                    colors_used_ = 0;
                    ++epoch_;
                }

                /**
                 * @brief Finds a colour and a slot for a contact between @p a and @p b.
                 *
                 * The colour is the lowest one free on both bodies, counting the
                 * colours the persistent constraint kinds already hold — which is what
                 * makes this a colouring of the union and not of the contacts alone.
                 *
                 * @param a          The first body's slot index.
                 * @param b          The second body's slot index, or a value at or past
                 *                   the body capacity for static geometry, which holds
                 *                   no colour because it is never written.
                 * @param persistent The colouring the long-lived constraint kinds use.
                 * @return Where to write the contact, or a placement that did not happen.
                 */
                ContactPlacement place(std::uint32_t a, std::uint32_t b,
                                       const IncrementalColoring& persistent) noexcept
                {
                    ContactPlacement placement;
                    if (a >= masks_.size())
                        return placement;

                    // Static geometry is not a body: it takes no correction, so no
                    // two contacts against it can race, and giving it a mask would
                    // serialize every contact in a scene against the same ground.
                    const bool has_b = b < masks_.size();

                    std::uint64_t taken = mask_for(a, persistent);
                    if (has_b)
                        taken |= mask_for(b, persistent);

                    for (std::uint32_t color = 0; color < color_count_; ++color)
                    {
                        const std::uint64_t bit = std::uint64_t(1) << color;
                        if ((taken & bit) != 0)
                            continue;
                        if (band_live_[color] >= band_capacity_)
                            continue;

                        masks_[a] |= bit;
                        if (has_b)
                            masks_[b] |= bit;

                        placement.placed = true;
                        placement.color = color;
                        placement.slot =
                            std::size_t(color) * band_capacity_ + band_live_[color];

                        ++band_live_[color];
                        ++live_count_;
                        if (color >= colors_used_)
                            colors_used_ = color + 1;
                        return placement;
                    }
                    return placement;
                }

                /** @brief How many colours the layout was built with. */
                std::size_t color_count() const noexcept { return color_count_; }

                /** @brief How many contacts one colour may hold. */
                std::size_t band_capacity() const noexcept { return band_capacity_; }

                /** @brief The first storage slot of colour @p color. */
                std::size_t band_base(std::size_t color) const noexcept
                {
                    return color * band_capacity_;
                }

                /** @brief Contacts placed in colour @p color this tick. */
                std::size_t band_size(std::size_t color) const noexcept
                {
                    return color < band_live_.size() ? band_live_[color] : 0;
                }

                /** @brief Contacts placed this tick, across every colour. */
                std::size_t live_count() const noexcept { return live_count_; }

                /** @brief How many colours this tick's contacts spread over. */
                std::size_t colors_used() const noexcept { return colors_used_; }

                /** @brief The total number of contact slots the layout spans. */
                std::size_t capacity() const noexcept
                {
                    return band_capacity_ * color_count_;
                }

            private:
                /** @brief The colour count actually used, bounded by the mask width. */
                static std::uint32_t clamped_color_count(std::size_t requested) noexcept
                {
                    if (requested == 0)
                        return 1;
                    return std::uint32_t(requested < IncrementalColoring::MAXIMUM_COLORS
                                             ? requested
                                             : IncrementalColoring::MAXIMUM_COLORS);
                }

                /**
                 * @brief Body @p index's mask for this tick, seeded on first use.
                 *
                 * @param index      The body slot; known in range by the caller.
                 * @param persistent The long-lived colouring to seed from.
                 */
                std::uint64_t mask_for(std::uint32_t index,
                                       const IncrementalColoring& persistent) noexcept
                {
                    if (stamp_[index] != epoch_)
                    {
                        masks_[index] = persistent.mask_of(index);
                        stamp_[index] = epoch_;
                    }
                    return masks_[index];
                }

                std::uint32_t color_count_;
                std::size_t band_capacity_;
                std::vector<std::size_t> band_live_;
                std::vector<std::uint64_t> masks_;
                std::vector<std::uint32_t> stamp_;

                // Starts at one so that a store which has never seen `begin()` has no
                // body stamped as seeded — zero is what `stamp_` is filled with.
                std::uint32_t epoch_ = 1;
                std::size_t live_count_ = 0;
                std::size_t colors_used_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
