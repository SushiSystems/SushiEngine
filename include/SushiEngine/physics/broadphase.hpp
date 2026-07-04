/**************************************************************************/
/* broadphase.hpp                                                        */
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
 * @file broadphase.hpp
 * @brief Sweep-and-prune broadphase: culls the O(n^2) contact pairs to overlapping AABBs.
 *
 * The contact solver's narrowphase is exact but costs one test per pair; running it on
 * every pair is quadratic and wasteful once a scene has more than a handful of bodies
 * (a cloth grid alone is hundreds of particles). `sweep_and_prune` sorts the bodies'
 * axis-aligned bounding boxes along one axis and sweeps a moving front, emitting only the
 * pairs whose boxes actually overlap — so the narrowphase runs on candidates, not on the
 * full cross product. It is pure geometry: no runtime, ECS, or solver dependency.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief An axis-aligned bounding box: its minimum and maximum corners. */
        template <typename T>
        struct Aabb
        {
            Vector3T<T> min;
            Vector3T<T> max;
        };

        /** @brief Whether two AABBs overlap on all three axes. */
        template <typename T>
        inline bool aabb_overlap(const Aabb<T>& a, const Aabb<T>& b) noexcept
        {
            return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y &&
                   b.min.y <= a.max.y && a.min.z <= b.max.z && b.min.z <= a.max.z;
        }

        /**
         * @brief Emits every overlapping AABB pair via sweep-and-prune on the X axis.
         *
         * Sorts the boxes by their minimum X, then sweeps: a moving "active" set holds the
         * boxes still reachable along X, and each new box is tested against only those (a
         * full 3-axis overlap test), so non-overlapping bodies far apart in X are never
         * paired. Output pairs are `(i, j)` indices into @p boxes with `i < j`, cleared
         * first. Worst case is still quadratic for a fully overlapping cluster, but typical
         * spatially-spread scenes are near-linear.
         *
         * @tparam T The scalar element type.
         * @param boxes      One AABB per body.
         * @param out_pairs  Receives the candidate index pairs (cleared on entry).
         */
        template <typename T>
        inline void sweep_and_prune(const std::vector<Aabb<T>>& boxes,
                                    std::vector<std::pair<std::uint32_t, std::uint32_t>>& out_pairs)
        {
            out_pairs.clear();
            const std::size_t count = boxes.size();
            if (count < 2)
                return;

            std::vector<std::uint32_t> order(count);
            for (std::size_t i = 0; i < count; ++i)
                order[i] = static_cast<std::uint32_t>(i);
            std::sort(order.begin(), order.end(),
                      [&](std::uint32_t l, std::uint32_t r) { return boxes[l].min.x < boxes[r].min.x; });

            std::vector<std::uint32_t> active;
            active.reserve(count);
            for (std::size_t s = 0; s < count; ++s)
            {
                const std::uint32_t i = order[s];
                const T min_x = boxes[i].min.x;
                // Drop everything whose X extent ended before this box begins.
                active.erase(std::remove_if(active.begin(), active.end(),
                                            [&](std::uint32_t a) { return boxes[a].max.x < min_x; }),
                             active.end());
                for (const std::uint32_t j : active)
                    if (aabb_overlap(boxes[i], boxes[j]))
                        out_pairs.emplace_back(i < j ? i : j, i < j ? j : i);
                active.push_back(i);
            }
        }
    } // namespace Physics
} // namespace SushiEngine
