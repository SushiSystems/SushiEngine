/**************************************************************************/
/* event.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_EVENT_HPP
#define SUSHIENGINE_AUDIO_EVENT_HPP

/**
 * @file event.hpp
 * @brief The event / container model — the sound-designer indirection between "the game
 *        posts an intent" and "these samples play".
 *
 * The game never says "play file X"; it posts an **event ID**, and the event resolves to
 * a sound through authored **containers** (§8, §10 of `docs/slop/audio_system.md`). This
 * indirection is the seam a sound designer owns: swap the footstep set, add variation,
 * blend by surface — no code change. The containers are the classic set:
 *
 *   - **Sound** — a leaf: one media id in the bank.
 *   - **Random** — pick a child at random each time (variation; no two footsteps alike).
 *   - **Sequence** — cycle through children in order across successive posts.
 *   - **Blend** — pick the child a continuous game parameter (an RTPC, 0..1) lands on.
 *   - **Switch** — pick the child a discrete game state selects.
 *
 * The whole tree is **flattened** into a `ContainerNode` array with contiguous child
 * ranges, so it is a trivially-serialisable POD block a @ref SushiEngine::Audio bank bakes
 * (no pointers, no per-node allocation). @ref EventDatabase::resolve walks it to one media
 * id given a @ref ResolveContext (the RNG seed, the blend parameter, the switch value).
 * Random and Sequence draw on a small, self-contained splitmix64 keyed by the node and an
 * invocation counter — reproducible, no global RNG state. Portable, no SDL/runtime.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A handle naming one event in an @ref EventDatabase. */
        using EventId = std::uint32_t;

        /** @brief Returned when an event or media lookup fails. */
        constexpr std::uint32_t INVALID_SOUND = 0xffffffffu;

        /** @brief Which selection rule a @ref ContainerNode applies to its children. */
        enum class ContainerKind : std::uint32_t
        {
            Sound = 0,    /**< Leaf: play @ref ContainerNode::sound. */
            Random = 1,   /**< Pick a random child (by @ref ContainerNode::weight) each post. */
            Sequence = 2, /**< Cycle children in order across posts. */
            Blend = 3,    /**< Cross-fade the two children the blend parameter straddles. */
            Switch = 4,   /**< Pick the child the switch value selects. */
            Layer = 5     /**< Play every child at once (layered). */
        };

        /**
         * @brief One node of a flattened container tree.
         *
         * A leaf carries a @ref sound (media id); a branch carries a contiguous child
         * range `[first_child, first_child + child_count)` into the owning database's node
         * array. A plain POD so the whole tree serialises as a byte block.
         */
        struct ContainerNode
        {
            ContainerKind kind = ContainerKind::Sound;
            std::uint32_t sound = INVALID_SOUND; /**< Leaf: the bank media id. */
            std::uint32_t first_child = 0;       /**< Branch: first child node index. */
            std::uint32_t child_count = 0;       /**< Branch: number of children. */
            float weight = 1.0f;                 /**< Relative selection weight (Random) / layer gain. */
        };

        /** @brief One sound a @ref EventDatabase::resolve_all produced: media, gain, delay. */
        struct ResolvedSound
        {
            std::uint32_t media_id = INVALID_SOUND;
            float gain = 1.0f;          /**< Composite gain down the container tree. */
            float delay_seconds = 0.0f; /**< Onset delay (0 today; reserved for staggered layers). */
        };

        /** @brief An event's entry: its id and the root container node it plays. */
        struct EventDef
        {
            EventId id = 0;
            std::uint32_t root = 0; /**< Root @ref ContainerNode index. */
        };

        /** @brief The per-post game state that steers Blend/Switch selection. */
        struct ResolveContext
        {
            std::uint32_t seed = 0;         /**< Extra entropy mixed into Random selection. */
            float blend = 0.0f;             /**< Blend parameter in [0, 1] for Blend containers. */
            std::uint32_t switch_value = 0; /**< Discrete selector for Switch containers. */
        };

        /**
         * @brief A flattened tree of containers plus the events that root them.
         *
         * Build it with @ref add_sound / @ref add_container and @ref add_event (children
         * must be added before the branch that ranges over them, so the ranges are
         * contiguous), or load one a bank baked. @ref resolve turns an event id into a
         * single media id per the container semantics.
         */
        class EventDatabase
        {
            public:
                /** @brief Adds a Sound leaf for a bank media id; returns its node index. */
                std::uint32_t add_sound(std::uint32_t media_id)
                {
                    ContainerNode n;
                    n.kind = ContainerKind::Sound;
                    n.sound = media_id;
                    nodes_.push_back(n);
                    return static_cast<std::uint32_t>(nodes_.size() - 1);
                }

                /**
                 * @brief Adds a branch container over an already-added contiguous child range.
                 * @param kind        Random / Sequence / Blend / Switch.
                 * @param first_child The first child node index (children must be contiguous).
                 * @param child_count The number of children.
                 * @return The new node's index.
                 */
                std::uint32_t add_container(ContainerKind kind, std::uint32_t first_child,
                                            std::uint32_t child_count)
                {
                    ContainerNode n;
                    n.kind = kind;
                    n.first_child = first_child;
                    n.child_count = child_count;
                    nodes_.push_back(n);
                    return static_cast<std::uint32_t>(nodes_.size() - 1);
                }

                /** @brief Maps an event id to a root node index. */
                void add_event(EventId id, std::uint32_t root_node)
                {
                    events_.push_back(EventDef{id, root_node});
                }

                /** @brief Sets a node's selection weight (Random) / layer gain. */
                void set_weight(std::uint32_t node, float weight)
                {
                    if (node < nodes_.size())
                        nodes_[node].weight = weight;
                }

                /** @brief The flattened node array (for the bank to serialise). */
                const std::vector<ContainerNode>& nodes() const noexcept { return nodes_; }
                /** @brief The event table (for the bank to serialise). */
                const std::vector<EventDef>& events() const noexcept { return events_; }

                /** @brief Replaces the contents from raw arrays (bank load). */
                void assign(std::vector<ContainerNode> nodes, std::vector<EventDef> events)
                {
                    nodes_ = std::move(nodes);
                    events_ = std::move(events);
                }

                /** @brief Whether the database holds any events. */
                bool empty() const noexcept { return events_.empty(); }

                /**
                 * @brief Resolves an event id to a single media id.
                 *
                 * Walks the container tree from the event's root: Random draws a child from
                 * a seeded splitmix64, Sequence advances a per-event counter, Blend maps the
                 * context's blend parameter across the children, and Switch indexes by the
                 * switch value. Returns @ref INVALID_SOUND for an unknown event, an empty
                 * branch, or a malformed tree (guarded against cycles by a depth cap).
                 *
                 * @param event   The posted event id.
                 * @param context The per-post game state (seed, blend, switch).
                 * @return The chosen bank media id, or @ref INVALID_SOUND.
                 */
                std::uint32_t resolve(EventId event, const ResolveContext& context)
                {
                    std::size_t event_index = 0;
                    bool found = false;
                    for (std::size_t i = 0; i < events_.size(); ++i)
                    {
                        if (events_[i].id == event)
                        {
                            event_index = i;
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                        return INVALID_SOUND;

                    std::uint32_t node = events_[event_index].root;
                    const std::uint32_t invocation = sequence_counter(event_index);
                    std::uint64_t rng = splitmix_seed(event, context.seed, invocation);
                    ++sequence_counters_[event_index];

                    for (int depth = 0; depth < 64; ++depth)
                    {
                        if (node >= nodes_.size())
                            return INVALID_SOUND;
                        const ContainerNode& n = nodes_[node];
                        if (n.kind == ContainerKind::Sound)
                            return n.sound;
                        if (n.child_count == 0 || n.first_child >= nodes_.size())
                            return INVALID_SOUND;

                        std::uint32_t pick = 0;
                        switch (n.kind)
                        {
                            case ContainerKind::Random:
                                pick = static_cast<std::uint32_t>(next_random(rng) % n.child_count);
                                break;
                            case ContainerKind::Sequence:
                                pick = invocation % n.child_count;
                                break;
                            case ContainerKind::Blend:
                            {
                                float b = context.blend;
                                b = b < 0.0f ? 0.0f : (b > 1.0f ? 1.0f : b);
                                std::uint32_t idx = static_cast<std::uint32_t>(
                                    b * static_cast<float>(n.child_count));
                                if (idx >= n.child_count)
                                    idx = n.child_count - 1;
                                pick = idx;
                                break;
                            }
                            case ContainerKind::Switch:
                                pick = context.switch_value < n.child_count ? context.switch_value
                                                                            : 0;
                                break;
                            case ContainerKind::Layer:
                                pick = 0; // single-resolve takes the first layer; resolve_all layers all
                                break;
                            case ContainerKind::Sound:
                                return n.sound; // unreachable; handled above
                        }
                        node = n.first_child + pick;
                    }
                    return INVALID_SOUND; // depth cap: malformed (cyclic) tree
                }

                /**
                 * @brief Resolves an event to all the sounds it plays (layers included).
                 *
                 * The richer counterpart of @ref resolve: a Blend container **cross-fades**
                 * the two children its parameter straddles, a Layer container plays **all**
                 * its children at once, and Random weights each child — so one event can
                 * spawn several overlapping sounds with per-sound gains. The bank factory
                 * turns the result into a layered voice.
                 *
                 * @param event   The posted event id.
                 * @param context The per-post game state.
                 * @param out     Filled with one @ref ResolvedSound per sounding leaf (cleared).
                 */
                void resolve_all(EventId event, const ResolveContext& context,
                                 std::vector<ResolvedSound>& out)
                {
                    out.clear();
                    std::size_t event_index = 0;
                    bool found = false;
                    for (std::size_t i = 0; i < events_.size(); ++i)
                        if (events_[i].id == event)
                        {
                            event_index = i;
                            found = true;
                            break;
                        }
                    if (!found)
                        return;

                    const std::uint32_t invocation = sequence_counter(event_index);
                    ++sequence_counters_[event_index];
                    std::uint64_t rng = splitmix_seed(event, context.seed, invocation);
                    collect(events_[event_index].root, 1.0f, context, invocation, rng, out, 0);
                }

                /** @brief Resets the per-event Sequence counters (e.g. on a level reload). */
                void reset_sequences() noexcept
                {
                    for (std::uint32_t& c : sequence_counters_)
                        c = 0;
                }

            private:
                /** @brief Recursively gathers the sounding leaves under a node (for @ref resolve_all). */
                void collect(std::uint32_t node, float gain, const ResolveContext& context,
                             std::uint32_t invocation, std::uint64_t& rng,
                             std::vector<ResolvedSound>& out, int depth)
                {
                    if (depth > 64 || node >= nodes_.size())
                        return;
                    const ContainerNode& n = nodes_[node];
                    if (n.kind == ContainerKind::Sound)
                    {
                        if (n.sound != INVALID_SOUND && gain > 1.0e-4f)
                            out.push_back(ResolvedSound{n.sound, gain, 0.0f});
                        return;
                    }
                    if (n.child_count == 0 || n.first_child >= nodes_.size())
                        return;

                    switch (n.kind)
                    {
                        case ContainerKind::Layer:
                            for (std::uint32_t c = 0; c < n.child_count; ++c)
                            {
                                const std::uint32_t child = n.first_child + c;
                                const float w = child < nodes_.size() ? nodes_[child].weight : 1.0f;
                                collect(child, gain * w, context, invocation, rng, out, depth + 1);
                            }
                            break;
                        case ContainerKind::Sequence:
                            collect(n.first_child + (invocation % n.child_count), gain, context,
                                    invocation, rng, out, depth + 1);
                            break;
                        case ContainerKind::Switch:
                            collect(n.first_child + (context.switch_value < n.child_count
                                                         ? context.switch_value
                                                         : 0),
                                    gain, context, invocation, rng, out, depth + 1);
                            break;
                        case ContainerKind::Blend:
                        {
                            float b = context.blend < 0.0f ? 0.0f : (context.blend > 1.0f ? 1.0f : context.blend);
                            const float position = b * static_cast<float>(n.child_count - 1);
                            std::uint32_t i0 = static_cast<std::uint32_t>(position);
                            if (i0 >= n.child_count - 1 && n.child_count > 1)
                                i0 = n.child_count - 2;
                            const std::uint32_t i1 = n.child_count > 1 ? i0 + 1 : i0;
                            const float frac = position - static_cast<float>(i0);
                            if (n.child_count == 1)
                            {
                                collect(n.first_child, gain, context, invocation, rng, out, depth + 1);
                            }
                            else
                            {
                                // Equal-power cross-fade between the two straddling children.
                                const float gl = std::sqrt(1.0f - frac);
                                const float gu = std::sqrt(frac);
                                collect(n.first_child + i0, gain * gl, context, invocation, rng, out,
                                        depth + 1);
                                collect(n.first_child + i1, gain * gu, context, invocation, rng, out,
                                        depth + 1);
                            }
                            break;
                        }
                        case ContainerKind::Random:
                        default:
                        {
                            // Weighted pick: sum the children's weights, draw within the sum.
                            float total = 0.0f;
                            for (std::uint32_t c = 0; c < n.child_count; ++c)
                            {
                                const std::uint32_t child = n.first_child + c;
                                total += child < nodes_.size() ? nodes_[child].weight : 1.0f;
                            }
                            if (total <= 0.0f)
                                total = 1.0f;
                            const float draw =
                                static_cast<float>(next_random(rng) % 100000u) / 100000.0f * total;
                            float acc = 0.0f;
                            std::uint32_t pick = 0;
                            for (std::uint32_t c = 0; c < n.child_count; ++c)
                            {
                                const std::uint32_t child = n.first_child + c;
                                acc += child < nodes_.size() ? nodes_[child].weight : 1.0f;
                                if (draw <= acc)
                                {
                                    pick = c;
                                    break;
                                }
                                pick = c;
                            }
                            collect(n.first_child + pick, gain, context, invocation, rng, out, depth + 1);
                            break;
                        }
                    }
                }

                std::uint32_t sequence_counter(std::size_t event_index)
                {
                    if (sequence_counters_.size() <= event_index)
                        sequence_counters_.resize(event_index + 1, 0);
                    return sequence_counters_[event_index];
                }

                static std::uint64_t splitmix_seed(std::uint32_t event, std::uint32_t seed,
                                                   std::uint32_t counter) noexcept
                {
                    std::uint64_t s = 0x9e3779b97f4a7c15ull;
                    s ^= static_cast<std::uint64_t>(event) * 0xff51afd7ed558ccdull;
                    s ^= static_cast<std::uint64_t>(seed) * 0xc4ceb9fe1a85ec53ull;
                    s ^= static_cast<std::uint64_t>(counter) + 0x165667b19e3779f9ull;
                    return s;
                }

                static std::uint64_t next_random(std::uint64_t& state) noexcept
                {
                    state += 0x9e3779b97f4a7c15ull;
                    std::uint64_t z = state;
                    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
                    z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
                    return z ^ (z >> 31);
                }

                std::vector<ContainerNode> nodes_;
                std::vector<EventDef> events_;
                std::vector<std::uint32_t> sequence_counters_;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
