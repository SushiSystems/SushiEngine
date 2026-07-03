/**************************************************************************/
/* chunk.hpp                                                              */
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

#include <cstddef>
#include <cstring>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/entity.hpp>

namespace SushiEngine
{
    /**
     * @brief A fixed-capacity block of entities laid out structure-of-arrays.
     *
     * A chunk holds up to @p capacity entities of one archetype. Each component is
     * a separate contiguous column backed by its own runtime allocation, so a
     * column's base pointer is a unique resource the dependency tracker keys on:
     * systems touching different chunks (different base pointers) run in parallel
     * automatically, and systems touching the same column are ordered. This is how
     * chunks become the runtime's unit of parallelism without any bespoke
     * scheduler. The columns are shared USM, so the host seeds and reads them while
     * a device kernel drives the arrays.
     *
     * Entities are packed at the front: a live count tracks how many of the
     * capacity rows are occupied, and removal is an O(1) swap with the last row.
     */
    class Chunk
    {
        public:
            /**
             * @brief Allocates one column per component, each sized to @p capacity.
             * @param runtime  The runtime that owns the column allocations.
             * @param comps    The archetype's components (id and byte size each).
             * @param capacity Maximum number of entities this chunk can hold.
             */
            Chunk(SushiRuntime::API::Runtime& runtime,
                  const std::vector<ComponentInfo>& comps, std::size_t capacity)
                : capacity_(capacity), entities_(capacity)
            {
                columns_.reserve(comps.size());
                for (const ComponentInfo& info : comps)
                {
                    Column col;
                    col.id = info.id;
                    col.size = info.size;
                    col.data = runtime.buffer<std::byte>(capacity * info.size);
                    columns_.push_back(std::move(col));
                }
            }

            /** @brief Number of live entities currently in the chunk. */
            std::size_t count() const noexcept { return count_; }

            /** @brief Maximum number of entities the chunk can hold. */
            std::size_t capacity() const noexcept { return capacity_; }

            /** @brief True when no further entity fits. */
            bool full() const noexcept { return count_ >= capacity_; }

            /**
             * @brief Base pointer of the column for component @p id.
             * @param id The component whose column is wanted.
             * @return The column's base address, or nullptr if absent.
             */
            std::byte* column(ComponentId id) noexcept
            {
                for (Column& c : columns_)
                    if (c.id == id) return c.data.data();
                return nullptr;
            }

            /**
             * @brief Appends a row for entity @p e and returns its index.
             *
             * The caller writes the row's component values after this. Must not be
             * called when the chunk is full.
             *
             * @param e The entity occupying the new row.
             * @return The index of the new row.
             */
            std::size_t allocate_row(Entity e) noexcept
            {
                const std::size_t row = count_++;
                entities_[row] = e;
                return row;
            }

            /**
             * @brief Removes row @p row by swapping the last row into its place.
             *
             * Keeps the live rows packed in O(1): the last row's components and
             * entity are copied over @p row and the count shrinks. The caller must
             * update the moved entity's directory record to point at @p row.
             *
             * @param row The row to remove.
             * @return The entity that was moved into @p row, or a null entity if
             *         @p row was already the last row.
             */
            Entity remove_row(std::size_t row) noexcept
            {
                const std::size_t last = count_ - 1;
                Entity moved{};
                if (row != last)
                {
                    for (Column& c : columns_)
                        std::memcpy(c.data.data() + row * c.size,
                                    c.data.data() + last * c.size, c.size);
                    entities_[row] = entities_[last];
                    moved = entities_[row];
                }
                --count_;
                return moved;
            }

            /**
             * @brief The entity occupying row @p row.
             * @param row A live row index.
             * @return The entity stored there.
             */
            Entity entity_at(std::size_t row) const noexcept { return entities_[row]; }

            /**
             * @brief Number of component columns in this chunk, for generic column walks.
             *
             * `column(id)` requires knowing which component to look up; snapshotting
             * (see `loop/rollback.hpp`) instead needs to walk every column of an
             * arbitrary archetype without knowing its component set ahead of time.
             */
            std::size_t column_count() const noexcept { return columns_.size(); }

            /** @brief The byte size of one element in the @p index'th column. */
            std::size_t column_size(std::size_t index) const noexcept
            {
                return columns_[index].size;
            }

            /** @brief Base pointer of the @p index'th column, addressed by position. */
            std::byte* column_at(std::size_t index) noexcept
            {
                return columns_[index].data.data();
            }

            /**
             * @brief Overwrites the live count directly, bypassing row bookkeeping.
             *
             * For rollback restore only (`Loop::RollbackBuffer::restore`): the caller
             * guarantees no entity has spawned or been destroyed since the snapshot
             * being restored, so every row's entity binding (see `entity_at`) is
             * already correct and only component values are moving back in time, not
             * the live/dead boundary itself. `allocate_row`/`remove_row` are the paths
             * that keep the entity directory in sync with real spawn/destroy; this
             * one does not, and must never be used outside a rollback restore.
             *
             * @param count The live row count to restore.
             */
            void restore_count(std::size_t count) noexcept { count_ = count; }

        private:
            /** @brief One component's contiguous backing array for this chunk. */
            struct Column
            {
                ComponentId id = 0;                       /**< Component identity. */
                std::size_t size = 0;                     /**< Element byte size. */
                SushiRuntime::API::Buffer<std::byte> data; /**< The column storage. */
            };

            std::size_t capacity_ = 0;
            std::size_t count_ = 0;
            std::vector<Entity> entities_;
            std::vector<Column> columns_;
    };
} // namespace SushiEngine
