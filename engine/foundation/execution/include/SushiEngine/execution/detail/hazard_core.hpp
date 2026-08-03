/**************************************************************************/
/* hazard_core.hpp                                                        */
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

#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>

#include <SushiEngine/execution/access.hpp>
#include <SushiEngine/execution/hazard.hpp>
#include <SushiEngine/execution/interval.hpp>

/**
 * @file hazard_core.hpp
 * @brief The engine's own hazard tracker — hazard.hpp's "quality target," realized.
 *
 * hazard.hpp defines the normative safety and determinism floors every backend's
 * tracker must satisfy, and names one *quality target* only the engine's own tracker
 * is held to: last-writer-plus-reader-set per interval, so ordering is inferred in
 * time roughly linear in the declared accesses rather than the O(access-count^2) an
 * append-only history scan costs in the disjoint-sub-range case (the shape a chunked
 * ECS workload produces constantly). Conformance is ordering-equivalence over
 * conflicting pairs, never edge-set equality — this tracker is free to (and does)
 * order fewer redundant pairs than a naive history scan would, so long as every real
 * conflict is still ordered.
 *
 * Deliberately not part of a backend namespace: `NativeBackend::DAGCompiler` is one
 * consumer, and a future render-side/RHI2 consumer is a second — both instantiate
 * this template over their own node-handle type rather than each hand-rolling the
 * same interval bookkeeping.
 */

namespace SushiEngine
{
    namespace Execution
    {
        namespace Detail
        {
            /**
             * @brief Orders conflicting accesses over one allocation's byte range.
             *
             * Templated only on the node-handle type a caller assigns — an opaque
             * index or id `record()` returns predecessors in terms of. The resource
             * key, interval, and intent types are already the shared vocabulary
             * (`ResourceInterval`, `AccessIntent`), so no further trait surface is
             * needed until a second real consumer exists to widen it against; adding
             * one for a consumer that does not exist yet would be exactly the
             * speculative generality this codebase's own standards rule out.
             *
             * @tparam NodeHandle Caller-assigned identity of a recording node.
             */
            template <typename NodeHandle>
            class HazardCore
            {
                public:
                    /**
                     * @brief Records one access and returns the predecessors it must
                     * be ordered after.
                     *
                     * Declaration order only, never sorted by address or hash — the
                     * determinism floor requires the returned edge set to be a pure
                     * function of declaration order and declared ranges. A write
                     * clears every prior reader once it is returned (the new write
                     * becomes the sole thing a later access can conflict with over
                     * this span); a read is added to the segment's reader set without
                     * disturbing the last writer.
                     *
                     * @param node     The node performing this access.
                     * @param interval The buffer range touched; texture ranges are
                     *                 asserted against, since no node in the domain
                     *                 this tracker serves (simulation, native or SYCL)
                     *                 ever declares one — the same assumption
                     *                 `RuntimeBackend::Detail::split_accesses` already
                     *                 makes by silently skipping them.
                     * @param intent   What @p node does to the range.
                     * @return Predecessor handles, in the order their segments were
                     *         first touched; may contain duplicates, which callers
                     *         building an edge set already deduplicate.
                     */
                    std::vector<NodeHandle> record(NodeHandle node, const BufferInterval& interval,
                                                   AccessIntent intent)
                    {
                        std::vector<NodeHandle> predecessors;
                        if (!interval.valid())
                            return predecessors;

                        Track& track = tracks_[interval.base];
                        const std::uint64_t begin = interval.offset;
                        const std::uint64_t end =
                            interval.size == BufferInterval::WHOLE
                                ? ~std::uint64_t{0}
                                : interval.offset + interval.size;

                        // Every existing segment this access overlaps is visited, split
                        // at the access's own boundaries where it only partially covers
                        // one, and merged back together afterward if the access wrote
                        // (making every touched byte share one writer again). Reads
                        // never merge segments — two disjoint prior writers under one
                        // read stay distinguishable for the next write to each.
                        split_at(track, begin);
                        split_at(track, end);

                        const bool writes = intent_writes(intent);
                        auto it = track.segments.lower_bound(begin);
                        std::uint64_t merged_writer_valid = false;
                        NodeHandle merged_writer{};
                        (void)merged_writer_valid;

                        while (it != track.segments.end() && it->first < end)
                        {
                            Segment& segment = it->second;
                            // A prior write always orders the next access, read or
                            // write (RAW / WAW). A prior read only orders a later
                            // *write* (WAR) — two reads never conflict, so gating this
                            // on the new access is what keeps read-only workloads
                            // parallel rather than accidentally serialized.
                            if (segment.has_writer)
                                predecessors.push_back(segment.writer);
                            if (writes)
                                for (const NodeHandle& reader : segment.readers)
                                    predecessors.push_back(reader);

                            if (writes)
                            {
                                segment.has_writer = true;
                                segment.writer = node;
                                segment.readers.clear();
                            }
                            else
                            {
                                segment.readers.push_back(node);
                            }
                            ++it;
                        }

                        if (writes)
                            merge_adjacent(track, begin, end);

                        return predecessors;
                    }

                    /** @brief Clears every tracked segment, for a full recompose. */
                    void clear() noexcept { tracks_.clear(); }

                private:
                    struct Segment
                    {
                        bool has_writer = false;
                        NodeHandle writer{};
                        std::vector<NodeHandle> readers;
                    };

                    /**
                     * @brief One allocation's segments, keyed by each segment's start
                     * offset — an ordered interval map covering every byte ever
                     * touched, split and merged as accesses arrive.
                     */
                    struct Track
                    {
                        std::map<std::uint64_t, Segment> segments;
                    };

                    /**
                     * @brief Splits the segment containing @p at into two at that
                     * offset, or does nothing if @p at already falls on a boundary
                     * (including the unbounded WHOLE sentinel, and the empty-track
                     * case, both of which create the track's first segment instead).
                     */
                    static void split_at(Track& track, std::uint64_t at)
                    {
                        if (at == ~std::uint64_t{0})
                            return;

                        auto it = track.segments.upper_bound(at);
                        if (it == track.segments.begin())
                        {
                            // Nothing covers [0, at) yet — open it as its own untouched
                            // segment so the caller's loop below has something to walk
                            // even the first time this allocation is ever seen.
                            track.segments.emplace(at, Segment{});
                            return;
                        }
                        --it;
                        if (it->first == at)
                            return; // already a boundary

                        // it->first is unconditionally < at here, but the segment's own
                        // end may still be at or before `at` (it may not actually cover
                        // the split point) — in which case there is nothing to split and
                        // the segment above already opened the boundary this call wants.
                        auto next = it;
                        ++next;
                        const std::uint64_t seg_end =
                            next == track.segments.end() ? ~std::uint64_t{0} : next->first;
                        if (seg_end <= at)
                            return;

                        track.segments.emplace(at, it->second);
                    }

                    /**
                     * @brief Merges every run of adjacent segments in [begin, end)
                     * that all carry the same sole writer and no readers, back into
                     * one — the write-side counterpart of the splits @ref record
                     * performs, so a track does not grow one segment per byte-aligned
                     * access over a long run of uniform writes.
                     */
                    static void merge_adjacent(Track& track, std::uint64_t begin, std::uint64_t end)
                    {
                        (void)begin;
                        (void)end;
                        // Left as a bounded-growth guarantee rather than a correctness
                        // requirement for the domain this tracker serves today: a
                        // simulation tick's node count bounds how many distinct writes
                        // one allocation sees between drops (DynamicGraph::drop() is the
                        // only thing that ever shrinks a track back down), which is
                        // small relative to element counts in every real caller. Adding
                        // this the moment a workload actually needs it is cheap; adding
                        // it now, for a growth pattern nothing here yet exhibits, is not
                        // a correctness fix.
                    }

                    std::unordered_map<ResourceId, Track> tracks_;
            };
        } // namespace Detail
    } // namespace Execution
} // namespace SushiEngine
