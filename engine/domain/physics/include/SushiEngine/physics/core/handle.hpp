/**************************************************************************/
/* handle.hpp                                                             */
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
 * @file handle.hpp
 * @brief Generational handles, and the slot table that hands them out.
 *
 * The mutable world reuses a removed body's slot, and a plain index cannot survive
 * that: the next caller to dereference a stale index addresses whatever body took
 * the slot, silently and with no way to notice. A generation counter turns that
 * into a detectable miss — the slot remembers how many times it has been reused,
 * and a handle carrying an older count is refused.
 *
 * The index half stays a plain `std::uint32_t` on purpose. Device buffers are
 * indexed by slot, and the generation is a host-side lifetime concern that never
 * needs to cross into a kernel.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A slot index plus the generation that slot was on when handed out.
         *
         * Trivially copyable and cheap to pass by value. A default-constructed
         * handle is invalid and stays invalid: generation zero is reserved for
         * "never allocated", so a zeroed handle can never collide with a live slot.
         *
         * @tparam Tag An empty type naming what the handle points at, so a
         *             `BodyHandle` cannot be passed where a `ConstraintHandle` is
         *             expected. The tag is never instantiated.
         */
        template <typename Tag>
        struct Handle
        {
            std::uint32_t index = 0;
            std::uint32_t generation = 0;

            /** @brief Whether this handle was ever handed out by a table. */
            bool valid() const noexcept { return generation != 0; }

            /** @brief Whether two handles name the same slot at the same generation. */
            friend bool operator==(const Handle& a, const Handle& b) noexcept
            {
                return a.index == b.index && a.generation == b.generation;
            }

            /** @brief Whether two handles differ in slot or in generation. */
            friend bool operator!=(const Handle& a, const Handle& b) noexcept
            {
                return !(a == b);
            }
        };

        /** @brief Tag type naming a rigid body slot. Never instantiated. */
        struct BodyTag;

        /** @brief Tag type naming a constraint slot. Never instantiated. */
        struct ConstraintTag;

        /**
         * @brief Tag type naming a joint slot. Never instantiated.
         *
         * Its own tag rather than a reuse of @ref ConstraintTag, even though a joint
         * *is* a constraint: the two kinds have separate slot tables, so a joint's
         * index 3 and a distance constraint's index 3 name different things, and a
         * handle that could be passed to either table would resolve to whichever one
         * the call site happened to reach.
         */
        struct JointTag;

        /** @brief A generational handle to a rigid body in a scene. */
        using BodyHandle = Handle<BodyTag>;

        /** @brief A generational handle to a constraint in a scene. */
        using ConstraintHandle = Handle<ConstraintTag>;

        /** @brief A generational handle to a joint in a scene. */
        using JointHandle = Handle<JointTag>;

        /**
         * @brief Allocates and recycles slots, tracking each slot's generation.
         *
         * Owns lifetime only — not the payload. The caller keeps its own parallel
         * array indexed by `Handle::index`, which is what lets the payload live in a
         * device buffer while the bookkeeping stays on the host.
         *
         * Capacity is fixed at construction and never grows. That is not a
         * simplification: an `Execution::Buffer` cannot be resized in place, and a
         * growth would reallocate and move, invalidating the raw pointer every
         * compiled graph node captured (§6.4). Running out of slots is therefore a
         * budgeted, reported event, which @ref allocate signals by returning an
         * invalid handle rather than by throwing.
         *
         * @tparam Tag The handle tag this table hands out.
         */
        template <typename Tag>
        class HandleTable
        {
            public:
                /**
                 * @brief Creates a table with @p capacity slots, all free.
                 * @param capacity The fixed number of slots; never exceeded.
                 */
                explicit HandleTable(std::size_t capacity)
                    : generations_(capacity, 0), live_(capacity, false)
                {
                    free_slots_.reserve(capacity);
                    // Pushed high-index-first so the first allocations come back in
                    // ascending order, which keeps a freshly-filled table's slot
                    // order equal to its insertion order and makes a fresh scene's
                    // buffer layout reproducible.
                    for (std::size_t i = capacity; i > 0; --i)
                        free_slots_.push_back(std::uint32_t(i - 1));
                }

                /**
                 * @brief Takes the next free slot.
                 *
                 * The slot's generation advances, so every handle previously handed
                 * out for it is now stale. Generation zero is skipped on wrap so a
                 * live slot never matches a default-constructed handle.
                 *
                 * @return A handle to the new slot, or an invalid handle when the
                 *         table is full.
                 */
                Handle<Tag> allocate() noexcept
                {
                    if (free_slots_.empty())
                        return Handle<Tag>{};

                    const std::uint32_t index = free_slots_.back();
                    free_slots_.pop_back();

                    std::uint32_t generation = generations_[index] + 1;
                    if (generation == 0)
                        generation = 1;
                    generations_[index] = generation;
                    live_[index] = true;
                    ++live_count_;

                    return Handle<Tag>{index, generation};
                }

                /**
                 * @brief Returns @p handle's slot to the free list.
                 *
                 * A stale or already-released handle is ignored rather than
                 * diagnosed: double-release is the normal shape of "two systems both
                 * think they own this body", and turning it into a crash would only
                 * move the problem.
                 *
                 * @param handle The handle to release.
                 * @return True when a live slot was released by this call.
                 */
                bool release(Handle<Tag> handle) noexcept
                {
                    if (!alive(handle))
                        return false;
                    live_[handle.index] = false;
                    free_slots_.push_back(handle.index);
                    --live_count_;
                    return true;
                }

                /**
                 * @brief Whether @p handle still names the slot it was handed out for.
                 * @param handle The handle to test.
                 * @return True when the slot is live and on @p handle's generation.
                 */
                bool alive(Handle<Tag> handle) const noexcept
                {
                    return handle.valid() && handle.index < generations_.size() &&
                           live_[handle.index] &&
                           generations_[handle.index] == handle.generation;
                }

                /** @brief The fixed number of slots. */
                std::size_t capacity() const noexcept { return generations_.size(); }

                /** @brief How many slots are currently allocated. */
                std::size_t live_count() const noexcept { return live_count_; }

                /** @brief Whether the next @ref allocate would fail. */
                bool full() const noexcept { return free_slots_.empty(); }

                /** @brief Whether slot @p index currently holds a live entry. */
                bool slot_live(std::uint32_t index) const noexcept
                {
                    return index < live_.size() && live_[index];
                }

                /**
                 * @brief Rebuilds the handle for a slot the caller knows is live.
                 *
                 * For a payload that is addressed by slot internally but by handle at
                 * the boundary — a dense array whose entries move — where the slot is
                 * known and the handle is what an API takes. Sound only because the
                 * table is the authority on the generation, so the caller cannot
                 * fabricate one.
                 *
                 * @param index The slot to name.
                 * @return A live handle to it, or an invalid handle when the slot is free.
                 */
                Handle<Tag> handle_of(std::uint32_t index) const noexcept
                {
                    if (!slot_live(index))
                        return Handle<Tag>{};
                    return Handle<Tag>{index, generations_[index]};
                }

            private:
                std::vector<std::uint32_t> generations_;
                // std::vector<bool> is the packed specialization, which is exactly
                // what is wanted here: one bit per slot, scanned rarely.
                std::vector<bool> live_;
                std::vector<std::uint32_t> free_slots_;
                std::size_t live_count_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
