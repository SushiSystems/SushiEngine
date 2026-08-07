/**************************************************************************/
/* islands.hpp                                                            */
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
 * @file islands.hpp
 * @brief Which bodies can affect each other, and which of those groups are asleep.
 *
 * §13.2 calls sleeping "the single largest win in any real scene and the cheapest
 * to implement", and the reason is arithmetic: a warehouse of ten thousand settled
 * crates is ten thousand bodies of which none is moving, and simulating them costs
 * exactly as much as simulating ten thousand moving ones unless something notices.
 * An island is the unit that notices — a connected component of the graph whose
 * edges are constraints and contacts — because a body cannot be disturbed by
 * anything it is not connected to, so a component whose every member has been
 * still for long enough can be dropped whole.
 *
 * Three decisions shape the implementation.
 *
 * **Static bodies conduct nothing.** The ground touches every crate in the
 * warehouse; if edges through it merged islands, the warehouse would be one
 * island, and one crate kicked at the far end would wake all of it. So an edge to
 * a body that never moves is *not* a connectivity edge. It is still a contact and
 * it is still solved; it just cannot carry a disturbance, because the thing in the
 * middle cannot be disturbed.
 *
 * **An island's identity is derived from its contents.** The key is the lowest
 * body index it contains, and the islands are numbered in ascending key order.
 * Not the traversal order, not the order bodies were added — §6.6 derives
 * cross-region edges in ascending region-key order, so a numbering that depended
 * on anything but state would make the composition depend on it too, and with it
 * the results (§12.1).
 *
 * **Sleeping is a property of the island, never of a body alone.** A crate resting
 * against one that is still rolling is not still; it is about to be hit. The
 * per-body motion measure and timer decide when a body is *eligible*, and the
 * island sleeps only when every member is — which is also why waking is the cheap
 * direction: one member disturbed wakes the component, and nothing has to be
 * re-derived to know which bodies that is.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief One island: a contiguous run of the builder's body list. */
        struct Island
        {
            /** @brief The lowest body index in the island; its identity. */
            std::uint32_t key = 0;

            /** @brief Where this island's bodies start in @ref IslandSet::bodies. */
            std::uint32_t first = 0;

            /** @brief How many bodies it has. */
            std::uint32_t count = 0;

            /** @brief Whether the whole island is asleep this tick. */
            bool sleeping = false;
        };

        /**
         * @brief This tick's islands, and the bodies in each.
         *
         * The bodies of island `i` are `bodies[island.first .. island.first + count)`,
         * ascending — one allocation for the whole partition rather than a vector
         * per island, which for ten thousand mostly-sleeping bodies is the
         * difference between a scan and a heap storm.
         */
        struct IslandSet
        {
            std::vector<Island> islands;
            std::vector<std::uint32_t> bodies;

            /**
             * @brief Advances only when the set of *awake* islands changes.
             *
             * The number §6.6's graph composition keys off. A tick in which
             * everything carries on exactly as it was must not recompose, and a
             * tick in which an island fell asleep or woke must. Anything else —
             * bodies moving, contacts appearing between already-awake bodies —
             * leaves this alone, which is what makes `compose_count()` a
             * meaningful thing to assert in a test.
             */
            std::uint64_t revision = 0;

            /** @brief How many islands are awake this tick. */
            std::size_t awake_count = 0;

            /** @brief The largest island's body count, for `PhysicsStatistics`. */
            std::size_t largest = 0;
        };

        /**
         * @brief Builds islands, and decides which of them sleep.
         *
         * Kept as an object rather than a function because it owns the scratch the
         * union-find needs, and a per-tick allocation of that is the kind of cost
         * that makes a feature which is supposed to save time cost some.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class IslandBuilder
        {
            public:
                /**
                 * @brief Starts a tick: every body its own island, no edges yet.
                 * @param body_count How many bodies the scene has.
                 */
                void begin(std::size_t body_count)
                {
                    parent_.resize(body_count);
                    rank_.assign(body_count, 0);
                    for (std::size_t i = 0; i < body_count; ++i)
                        parent_[i] = static_cast<std::uint32_t>(i);
                }

                /**
                 * @brief Connects two bodies, unless one of them conducts nothing.
                 *
                 * @param a     The first body index.
                 * @param b     The second body index.
                 * @param bodies The scene's bodies, for their flags.
                 */
                void connect(std::uint32_t a, std::uint32_t b, const RigidBodyT<T>* bodies)
                {
                    if (a >= parent_.size() || b >= parent_.size())
                        return;
                    if (conducts(bodies[a]) && conducts(bodies[b]))
                        merge(a, b);
                }

                /**
                 * @brief Closes the tick: partitions, decides sleep, writes it back.
                 *
                 * The order matters and is the whole method. Bodies are grouped;
                 * each island's sleep eligibility is the conjunction over its
                 * members; the decision is then written back onto every member's
                 * flags *and* onto its island index, so the solver, the broadphase
                 * and the statistics all read one answer rather than three
                 * consistent-looking ones.
                 *
                 * @param bodies      The scene's bodies; their flags and timers are updated.
                 * @param body_count  How many.
                 * @param dt          The tick's duration, for the sleep timers.
                 * @param motion_threshold Below this smoothed motion a body is eligible.
                 * @param sleep_delay How long it must stay there.
                 * @param out         Receives the partition.
                 */
                void finish(RigidBodyT<T>* bodies, std::size_t body_count, T dt,
                            T motion_threshold, T sleep_delay, IslandSet& out)
                {
                    // Per body: is it *itself* ready to sleep? A static body is
                    // ignored entirely; it is in no island and has nothing to wake.
                    for (std::size_t i = 0; i < body_count; ++i)
                    {
                        RigidBodyT<T>& body = bodies[i];
                        if (!conducts(body))
                            continue;
                        if (body.motion_measure > motion_threshold ||
                            has_any_flag(body.flags, BodyFlags::never_sleep))
                            body.sleep_timer = T(0);
                        else
                            body.sleep_timer += dt;
                    }

                    // Group by root. The root is an arbitrary member, so the island's
                    // key is taken as the lowest index under it — a property of the
                    // set rather than of how the merges happened to run.
                    order_.clear();
                    order_.reserve(body_count);
                    keys_.assign(body_count, 0xFFFFFFFFu);
                    for (std::size_t i = 0; i < body_count; ++i)
                    {
                        if (!conducts(bodies[i]))
                            continue;
                        const std::uint32_t root = find(static_cast<std::uint32_t>(i));
                        if (keys_[root] > i)
                            keys_[root] = static_cast<std::uint32_t>(i);
                        order_.push_back(static_cast<std::uint32_t>(i));
                    }

                    // The key *per body*, resolved once. Calling `find` from inside
                    // a comparator instead costs a pointer chase per comparison, and
                    // in a ten-thousand-body scene that one line was most of the
                    // partition's cost.
                    body_key_.assign(body_count, 0xFFFFFFFFu);
                    for (const std::uint32_t body : order_)
                        body_key_[body] = keys_[find(body)];

                    // Counted, not sorted. An island's key *is* the lowest body index
                    // in it, so walking the bodies in ascending order encounters each
                    // island's key at exactly the moment it first appears — which
                    // hands back the islands already in ascending key order, and the
                    // bodies within each already ascending, for the cost of two
                    // linear passes. The comparison sort this replaced was the last
                    // per-tick `n log n` in a scene of settled bodies, and a settled
                    // scene is supposed to cost nothing.
                    slot_of_key_.assign(body_count, 0xFFFFFFFFu);
                    out.islands.clear();
                    out.awake_count = 0;
                    out.largest = 0;

                    for (const std::uint32_t body : order_)
                    {
                        const std::uint32_t key = body_key_[body];
                        if (slot_of_key_[key] == 0xFFFFFFFFu)
                        {
                            slot_of_key_[key] = static_cast<std::uint32_t>(out.islands.size());
                            // Value-initialized: an `Island` is memcpy'd into a physics
                            // snapshot, and a struct whose padding is indeterminate
                            // yields two different blobs for one logical state — which
                            // §12.3's byte equality is precisely a claim against.
                            Island island{};
                            island.key = key;
                            island.sleeping = true;
                            out.islands.push_back(island);
                        }
                        Island& island = out.islands[slot_of_key_[key]];
                        ++island.count;
                        // One member still moving keeps the island awake: a crate
                        // leaning on one that is still rolling is not at rest, it is
                        // about to be hit.
                        if (bodies[body].sleep_timer < sleep_delay ||
                            has_any_flag(bodies[body].flags, BodyFlags::never_sleep))
                            island.sleeping = false;
                    }

                    std::uint32_t offset = 0;
                    for (Island& island : out.islands)
                    {
                        island.first = offset;
                        offset += island.count;
                        if (island.count > out.largest)
                            out.largest = island.count;
                        if (!island.sleeping)
                            ++out.awake_count;
                    }

                    out.bodies.resize(order_.size());
                    fill_.assign(out.islands.size(), 0);
                    for (const std::uint32_t body : order_)
                    {
                        const std::uint32_t slot = slot_of_key_[body_key_[body]];
                        out.bodies[out.islands[slot].first + fill_[slot]] = body;
                        ++fill_[slot];
                    }

                    // Write the decision back, and notice whether it is news.
                    bool changed = out.islands.size() != previous_awake_.size();
                    for (std::size_t i = 0; i < out.islands.size(); ++i)
                    {
                        const Island& island = out.islands[i];
                        for (std::uint32_t k = 0; k < island.count; ++k)
                        {
                            RigidBodyT<T>& body = bodies[out.bodies[island.first + k]];
                            body.island_index = static_cast<std::uint32_t>(i);
                            if (island.sleeping)
                            {
                                body.flags |= BodyFlags::sleeping;
                                // A sleeping body holds still exactly, rather than
                                // drifting at whatever velocity it fell asleep with.
                                body.velocity = Vector3T<T>{T(0), T(0), T(0)};
                                body.angular_velocity = Vector3T<T>{T(0), T(0), T(0)};
                            }
                            else
                            {
                                body.flags &= ~BodyFlags::sleeping;
                            }
                        }
                        if (!changed && (previous_awake_[i] != !island.sleeping ||
                                         previous_keys_[i] != island.key))
                            changed = true;
                    }
                    // The counter lives on the builder, not on the set: it is a
                    // property of the *history* of the partition, and a caller that
                    // fills a fresh set each tick must still be able to compare this
                    // tick's number with last tick's.
                    if (changed)
                        ++revision_;
                    out.revision = revision_;

                    previous_awake_.resize(out.islands.size());
                    previous_keys_.resize(out.islands.size());
                    for (std::size_t i = 0; i < out.islands.size(); ++i)
                    {
                        previous_awake_[i] = !out.islands[i].sleeping;
                        previous_keys_[i] = out.islands[i].key;
                    }
                }

            private:
                /** @brief Whether a body can carry a disturbance to its neighbours. */
                static bool conducts(const RigidBodyT<T>& body) noexcept
                {
                    return !has_any_flag(body.flags, BodyFlags::static_body) &&
                           body.inv_mass > T(0);
                }

                std::uint32_t find(std::uint32_t index) const noexcept
                {
                    while (parent_[index] != index)
                        index = parent_[index];
                    return index;
                }

                void merge(std::uint32_t a, std::uint32_t b) noexcept
                {
                    std::uint32_t root_a = find(a);
                    std::uint32_t root_b = find(b);
                    if (root_a == root_b)
                        return;
                    // Union by rank, but with the *lower index* winning ties, so the
                    // forest a scene produces is a function of the scene.
                    if (rank_[root_a] < rank_[root_b] ||
                        (rank_[root_a] == rank_[root_b] && root_b < root_a))
                    {
                        const std::uint32_t swap = root_a;
                        root_a = root_b;
                        root_b = swap;
                    }
                    parent_[root_b] = root_a;
                    if (rank_[root_a] == rank_[root_b])
                        ++rank_[root_a];
                }

                std::vector<std::uint32_t> parent_;
                std::vector<std::uint32_t> rank_;
                std::vector<std::uint32_t> keys_;
                std::vector<std::uint32_t> body_key_;
                std::vector<std::uint32_t> slot_of_key_;
                std::vector<std::uint32_t> fill_;
                std::vector<std::uint32_t> order_;
                std::vector<char> previous_awake_;
                std::vector<std::uint32_t> previous_keys_;
                std::uint64_t revision_ = 0;
        };

        /**
         * @brief Wakes a body and, through it, everything it is resting against.
         *
         * The asymmetry that makes sleeping safe: falling asleep is a decision taken
         * once a whole island has earned it, and waking is immediate and needs no
         * decision at all. A gameplay system that teleports one crate calls this,
         * and the stack above that crate is awake on the same tick rather than
         * hanging in the air until something re-derives the islands.
         *
         * @param bodies     The scene's bodies.
         * @param body_count How many.
         * @param body       The body to wake.
         * @param set        The islands as last built.
         */
        template <typename T>
        inline void wake_island(RigidBodyT<T>* bodies, std::size_t body_count,
                                std::uint32_t body, const IslandSet& set) noexcept
        {
            if (body >= body_count)
                return;
            bodies[body].flags &= ~BodyFlags::sleeping;
            bodies[body].sleep_timer = T(0);
            const std::uint32_t island = bodies[body].island_index;
            if (island >= set.islands.size())
                return;
            const Island& record = set.islands[island];
            for (std::uint32_t k = 0; k < record.count; ++k)
            {
                RigidBodyT<T>& member = bodies[set.bodies[record.first + k]];
                member.flags &= ~BodyFlags::sleeping;
                member.sleep_timer = T(0);
            }
        }
    } // namespace Physics
} // namespace SushiEngine
