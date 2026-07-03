/**************************************************************************/
/* rollback.hpp                                                          */
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
 * @file rollback.hpp
 * @brief SushiLoop's Snapshot layer (M3, docs/slop/SUSHILOOP.md): per-chunk
 *        rollback snapshots and restore.
 *
 * `RollbackBuffer` records, per tick, a byte copy of every live chunk's component
 * columns into a fixed-size ring buffer, and can restore the world to any tick
 * still held in that ring. Chunk identity (a `Chunk*`) is used directly rather
 * than an archetype/index pair, which is why this only supports the scope M3 sets
 * out: **no entity may spawn or be destroyed, and no new archetype/chunk may be
 * created, between a capture and the matching restore.** Rebasing after a
 * structural change is a later milestone's problem (M4's reconciliation layer, most
 * likely); recording every live chunk in full rather than only the ones a tick
 * actually wrote to is this milestone's scope too — real per-write dirty tracking
 * is a follow-on optimization once something upstream (Schedule, CommandBuffer)
 * marks chunks dirty, which nothing does yet.
 *
 * `RollbackBuffer` only captures and restores; deciding *when* to roll back, and
 * replaying ticks forward afterward with recorded input, is the caller's job (a
 * game loop, or later the Net layer's reconciliation) — this stays a storage
 * primitive, the same way `Chunk`/`Archetype` do not know what a system is.
 */

#include <cstddef>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/loop/input.hpp>

namespace SushiEngine
{
    namespace loop
    {
        /**
         * @brief A fixed-capacity ring of per-tick, per-chunk world snapshots.
         *
         * Capturing walks every archetype's every chunk (`World::query` with the
         * empty signature matches all of them) and copies each column's live bytes
         * — `count() * column_size` per column, not the full chunk capacity, since
         * only live rows matter. Restoring writes those bytes straight back into the
         * same `Chunk*`, which is why the no-structural-change constraint above is
         * load-bearing: a `Chunk*` captured this tick must still be the same chunk,
         * at the same address, holding the same entities in the same rows, when a
         * later restore names that tick.
         */
        class RollbackBuffer
        {
            public:
                /**
                 * @brief Creates a ring holding at most @p capacity snapshots.
                 * @param capacity Number of ticks to retain before evicting the oldest.
                 */
                explicit RollbackBuffer(std::size_t capacity) noexcept : capacity_(capacity) {}

                /**
                 * @brief Captures every live chunk's component bytes, tagged with @p tick.
                 *
                 * Evicts the oldest retained snapshot first if already at capacity, so
                 * the ring never holds more than `capacity` ticks and old history falls
                 * off automatically as new ticks are captured.
                 *
                 * @param world The world to snapshot.
                 * @param tick  The tick this snapshot represents.
                 */
                void capture(World& world, TickId tick)
                {
                    Snapshot snapshot;
                    snapshot.tick = tick;

                    for (Archetype* archetype : world.query(Signature{}))
                        for (const std::unique_ptr<Chunk>& chunk_ptr : archetype->chunks())
                        {
                            Chunk& chunk = *chunk_ptr;
                            ChunkSnapshot cs;
                            cs.chunk = &chunk;
                            cs.count = chunk.count();
                            cs.columns.reserve(chunk.column_count());
                            for (std::size_t i = 0; i < chunk.column_count(); ++i)
                            {
                                const std::size_t bytes = chunk.column_size(i) * cs.count;
                                const std::byte* src = chunk.column_at(i);
                                cs.columns.emplace_back(src, src + bytes);
                            }
                            snapshot.chunks.push_back(std::move(cs));
                        }

                    ring_.push_back(std::move(snapshot));
                    if (ring_.size() > capacity_)
                        ring_.pop_front();
                }

                /**
                 * @brief Restores the world to the snapshot captured for @p tick.
                 *
                 * A no-op returning `false` if @p tick has been evicted or was never
                 * captured. Every entity's row binding is left untouched — only
                 * component column bytes and each chunk's live count are overwritten —
                 * per the class's no-structural-change scope.
                 *
                 * @param tick The tick to roll back to.
                 * @return Whether @p tick was found and restored.
                 */
                bool restore(TickId tick) const
                {
                    for (const Snapshot& snapshot : ring_)
                    {
                        if (snapshot.tick != tick)
                            continue;
                        for (const ChunkSnapshot& cs : snapshot.chunks)
                        {
                            cs.chunk->restore_count(cs.count);
                            for (std::size_t i = 0; i < cs.columns.size(); ++i)
                                std::memcpy(cs.chunk->column_at(i), cs.columns[i].data(),
                                           cs.columns[i].size());
                        }
                        return true;
                    }
                    return false;
                }

                /** @brief Whether @p tick is still held in the ring. */
                bool has(TickId tick) const noexcept
                {
                    for (const Snapshot& snapshot : ring_)
                        if (snapshot.tick == tick)
                            return true;
                    return false;
                }

                /** @brief Number of ticks currently retained. */
                std::size_t size() const noexcept { return ring_.size(); }

                /** @brief Maximum number of ticks the ring retains. */
                std::size_t capacity() const noexcept { return capacity_; }

            private:
                /** @brief One chunk's column bytes at the moment of capture. */
                struct ChunkSnapshot
                {
                    Chunk* chunk = nullptr;
                    std::size_t count = 0;
                    std::vector<std::vector<std::byte>> columns;
                };

                /** @brief Every live chunk's state for one tick. */
                struct Snapshot
                {
                    TickId tick = 0;
                    std::vector<ChunkSnapshot> chunks;
                };

                std::size_t capacity_;
                std::deque<Snapshot> ring_;
        };
    } // namespace loop
} // namespace SushiEngine
