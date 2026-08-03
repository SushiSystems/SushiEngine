/**************************************************************************/
/* portals.hpp                                                           */
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

#ifndef SUSHIENGINE_AUDIO_PORTALS_HPP
#define SUSHIENGINE_AUDIO_PORTALS_HPP

/**
 * @file portals.hpp
 * @brief Rooms and portals: indoor propagation as doorway secondary sources.
 *
 * Instead of a wave solve, indoor sound coupling is modelled the way every shipping
 * engine does it (§6 of `docs/slop/audio_system.md`): the world is partitioned into
 * **rooms** joined by **portals** (doorways / openings). A source in another room is not
 * heard through the wall — it is heard *through the doorway*, so each portal on a path
 * from the listener's room to the source's room becomes a **secondary virtual source
 * placed at the opening**, at a level set by the total path length through the portals.
 * That gives correct, cheap doorway diffraction and room-to-room coupling: walk into a
 * corridor and the sound in the next room swings to come from the door you can see, not
 * from straight through the wall.
 *
 * @ref PortalGraph::resolve runs a small shortest-path search (one per doorway out of the
 * listener's room) and returns a @ref PortalSource per reachable opening. The caller
 * spawns / steers a virtual voice at each — the direct voice is muted when the source is
 * in another room (the wall occludes it; `occlusion.hpp`) and replaced by these. Portable
 * `float` maths, no SDL and no SushiRuntime, so it unit-tests against a hand-built graph.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/audio/acoustic_geometry.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A room identifier (an opaque handle the game assigns). */
        using RoomId = std::uint32_t;

        /** @brief Returned by @ref PortalGraph::room_of when a point is in no room. */
        constexpr RoomId INVALID_ROOM = 0xffffffffu;

        /** @brief A convex room, approximated by an axis-aligned box for containment. */
        struct AcousticRoom
        {
            RoomId id = 0;
            AcousticAABB bounds; /**< The room's world box (containment test). */
        };

        /** @brief A doorway/opening joining two rooms. */
        struct AcousticPortal
        {
            RoomId room_a = 0;
            RoomId room_b = 0;
            AudioVec3 center;      /**< The opening's world centre (the virtual-source point). */
            AudioVec3 half_extents; /**< The opening's half size (carried for future width-based diffraction). */
        };

        /** @brief A doorway rendered as a secondary source: where it is and how loud. */
        struct PortalSource
        {
            AudioVec3 position;      /**< The opening's world position (the virtual emitter). */
            float gain = 1.0f;       /**< Level in [0, 1] from the path length through the portal. */
            float path_length = 0.0f; /**< Total listener→portal→…→source distance in metres. */
        };

        /** @brief The outcome of resolving a source against the room/portal graph. */
        struct PortalResolution
        {
            bool same_room = true;         /**< Source and listener share a room → direct path. */
            bool source_reachable = false; /**< A portal path reaches the source's room. */
            std::vector<PortalSource> doorways; /**< One per opening out of the listener's room. */
        };

        /**
         * @brief A room/portal topology that turns cross-room sources into doorway sources.
         *
         * Add rooms and portals, @ref build the adjacency, then @ref resolve a
         * source/listener pair per wall-clock frame. The graph is small (a level's rooms),
         * so the per-doorway shortest path is a plain array Dijkstra.
         */
        class PortalGraph
        {
            public:
                /** @brief Adds a room and returns its slot index. */
                std::size_t add_room(RoomId id, const AcousticAABB& bounds)
                {
                    rooms_.push_back(AcousticRoom{id, bounds});
                    return rooms_.size() - 1;
                }

                /** @brief Adds a portal joining two rooms. */
                void add_portal(RoomId room_a, RoomId room_b, const AudioVec3& center,
                                const AudioVec3& half_extents)
                {
                    portals_.push_back(AcousticPortal{room_a, room_b, center, half_extents});
                }

                /** @brief Removes all rooms and portals. */
                void clear() noexcept
                {
                    rooms_.clear();
                    portals_.clear();
                }

                /** @brief Freezes the topology (no-op today; kept for a future flattened form). */
                void build() noexcept {}

                /** @brief The room containing a point, or @ref INVALID_ROOM (first match wins). */
                RoomId room_of(const AudioVec3& p) const noexcept
                {
                    for (const AcousticRoom& r : rooms_)
                    {
                        if (p.x >= r.bounds.min.x && p.x <= r.bounds.max.x &&
                            p.y >= r.bounds.min.y && p.y <= r.bounds.max.y &&
                            p.z >= r.bounds.min.z && p.z <= r.bounds.max.z)
                            return r.id;
                    }
                    return INVALID_ROOM;
                }

                /**
                 * @brief Resolves a source against the graph into doorway secondary sources.
                 *
                 * If the source shares the listener's room (or either is in no room), the
                 * path is direct and @ref PortalResolution::doorways is empty. Otherwise, for
                 * every portal out of the listener's room, the shortest path to the source's
                 * room *starting with that portal* is found; each becomes a @ref PortalSource
                 * at the opening, its gain the inverse of the total path length (referenced to
                 * @p reference_distance), capped to @p max_doorways loudest.
                 *
                 * @param listener          The listener world position.
                 * @param source            The source world position.
                 * @param reference_distance Distance at which a doorway is at unit gain.
                 * @param max_doorways      Cap on returned openings (loudest kept).
                 * @return The resolution (same-room flag, reachability, doorway list).
                 */
                PortalResolution resolve(const AudioVec3& listener, const AudioVec3& source,
                                         float reference_distance, int max_doorways) const
                {
                    PortalResolution out;
                    const RoomId lroom = room_of(listener);
                    const RoomId sroom = room_of(source);

                    if (lroom == INVALID_ROOM || sroom == INVALID_ROOM || lroom == sroom)
                    {
                        out.same_room = true;
                        out.source_reachable = true;
                        return out;
                    }
                    out.same_room = false;

                    if (reference_distance < 0.01f)
                        reference_distance = 0.01f;

                    for (std::size_t start = 0; start < portals_.size(); ++start)
                    {
                        const AcousticPortal& p0 = portals_[start];
                        if (p0.room_a != lroom && p0.room_b != lroom)
                            continue; // not a doorway out of the listener's room

                        const float total = shortest_via(start, listener, source, sroom);
                        if (total < 0.0f)
                            continue; // no path to the source's room through this doorway

                        PortalSource ps;
                        ps.position = p0.center;
                        ps.path_length = total;
                        ps.gain = reference_distance / (total > reference_distance ? total : reference_distance);
                        out.doorways.push_back(ps);
                        out.source_reachable = true;
                    }

                    // Loudest doorways first, then cap.
                    std::sort(out.doorways.begin(), out.doorways.end(),
                              [](const PortalSource& a, const PortalSource& b) { return a.gain > b.gain; });
                    if (max_doorways >= 0 && static_cast<int>(out.doorways.size()) > max_doorways)
                        out.doorways.resize(static_cast<std::size_t>(max_doorways));
                    return out;
                }

            private:
                static float dist(const AudioVec3& a, const AudioVec3& b) noexcept
                {
                    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
                    return std::sqrt(dx * dx + dy * dy + dz * dz);
                }

                static bool share_room(const AcousticPortal& a, const AcousticPortal& b) noexcept
                {
                    return a.room_a == b.room_a || a.room_a == b.room_b || a.room_b == b.room_a ||
                           a.room_b == b.room_b;
                }

                /**
                 * @brief Shortest listener→source distance whose first hop is portal @p start.
                 *
                 * A plain array Dijkstra over portals: seed only @p start (from the listener),
                 * relax between portals that share a room, and close out at any portal
                 * bordering the source's room (adding the final hop to the source). Returns
                 * the total path length, or −1 if the source's room is unreachable this way.
                 */
                float shortest_via(std::size_t start, const AudioVec3& listener,
                                   const AudioVec3& source, RoomId sroom) const
                {
                    const std::size_t n = portals_.size();
                    std::vector<float> best(n, -1.0f);
                    std::vector<bool> done(n, false);
                    best[start] = dist(listener, portals_[start].center);

                    float result = -1.0f;
                    for (std::size_t iter = 0; iter < n; ++iter)
                    {
                        // Pick the unvisited portal with the smallest tentative distance.
                        std::size_t u = n;
                        float best_u = -1.0f;
                        for (std::size_t i = 0; i < n; ++i)
                        {
                            if (done[i] || best[i] < 0.0f)
                                continue;
                            if (best_u < 0.0f || best[i] < best_u)
                            {
                                best_u = best[i];
                                u = i;
                            }
                        }
                        if (u == n)
                            break;
                        done[u] = true;

                        // If this portal borders the source's room, it can close the path.
                        const AcousticPortal& pu = portals_[u];
                        if (pu.room_a == sroom || pu.room_b == sroom)
                        {
                            const float total = best[u] + dist(pu.center, source);
                            if (result < 0.0f || total < result)
                                result = total;
                        }

                        for (std::size_t v = 0; v < n; ++v)
                        {
                            if (done[v] || v == u || !share_room(pu, portals_[v]))
                                continue;
                            const float cand = best[u] + dist(pu.center, portals_[v].center);
                            if (best[v] < 0.0f || cand < best[v])
                                best[v] = cand;
                        }
                    }
                    return result;
                }

                std::vector<AcousticRoom> rooms_;
                std::vector<AcousticPortal> portals_;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
