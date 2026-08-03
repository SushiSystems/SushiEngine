/**************************************************************************/
/* scene_blob_table.hpp                                                   */
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

#ifndef SUSHIENGINE_SCENE_SCENE_BLOB_TABLE_HPP
#define SUSHIENGINE_SCENE_SCENE_BLOB_TABLE_HPP

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace SushiEngine
{
    namespace Scene
    {
        /**
         * @brief Where a scene capture puts the large binary assets it will not inline.
         *
         * A cooked soft-body blob is megabytes of tetrahedra, rest-state inverses and a
         * distance field. A `.sushiscene` file carries it by value so the file stays
         * self-contained, but an in-memory snapshot cannot: an undo stack fifty deep plus
         * a redo stack would hold a hundred copies of every soft body in the scene. So a
         * snapshot stores a content hash and the bytes go here, once, and the two shapes
         * differ only in that one field.
         *
         * An interface rather than the concrete table because the caller owns the
         * lifetime and the policy — an editor session's table outlives its undo stack,
         * and a future disk-backed or reference-counted one substitutes here without
         * touching the serializer.
         */
        class ISceneBlobTable
        {
            public:
                virtual ~ISceneBlobTable() = default;

                /**
                 * @brief Stores @p bytes under @p key, replacing any previous entry.
                 *
                 * @param key   The content hash the snapshot will name these bytes by.
                 * @param bytes The blob; copied, since the caller's own copy is about to
                 *     be destroyed by the very edit the snapshot is guarding against.
                 */
                virtual void put(std::uint64_t key, const std::vector<std::byte>& bytes) = 0;

                /**
                 * @brief Reads the blob stored under @p key.
                 *
                 * @param key The content hash from the snapshot.
                 * @param out Receives the bytes on a hit; left untouched on a miss.
                 * @return True when @p key was present. False means the snapshot outlived
                 *     its table and the asset is unrecoverable — the caller must not
                 *     substitute an empty blob for it.
                 */
                virtual bool get(std::uint64_t key, std::vector<std::byte>& out) const = 0;
        };

        /**
         * @brief The in-memory blob table an editor session owns.
         *
         * Never evicts, and that is the whole point of keying on a content hash: the
         * table holds one entry per *distinct* cooked asset the session has seen, not one
         * per snapshot. Its size is therefore bounded by how many soft bodies the project
         * contains, not by how long the artist has been editing — re-cooking the same
         * mesh twice, or fifty undo steps over one body, all collapse onto one entry. An
         * eviction policy would buy nothing and would instead let a live undo step lose
         * the asset it names.
         *
         * @ref clear is there for the caller that knows a session ended (a project
         * closed, a new scene opened) and can reclaim the memory outright, which is a
         * decision only that caller can make correctly.
         */
        class SceneBlobTable final : public ISceneBlobTable
        {
            public:
                /**
                 * @brief Stores @p bytes under @p key, replacing any previous entry.
                 * @param key   The content hash the snapshot names these bytes by.
                 * @param bytes The blob to keep for as long as this table lives.
                 */
                void put(std::uint64_t key, const std::vector<std::byte>& bytes) override
                {
                    entries_[key] = bytes;
                }

                /**
                 * @brief Reads the blob stored under @p key.
                 * @param key The content hash from the snapshot.
                 * @param out Receives the bytes on a hit; left untouched on a miss.
                 * @return True when @p key was present.
                 */
                bool get(std::uint64_t key, std::vector<std::byte>& out) const override
                {
                    const auto entry = entries_.find(key);
                    if (entry == entries_.end())
                        return false;
                    out = entry->second;
                    return true;
                }

                /** @brief How many distinct blobs the table holds. */
                std::size_t size() const noexcept { return entries_.size(); }

                /**
                 * @brief Drops every entry.
                 *
                 * Invalidates every snapshot that names a blob by hash, so only a caller
                 * that has also discarded those snapshots may call it.
                 */
                void clear() noexcept { entries_.clear(); }

            private:
                std::unordered_map<std::uint64_t, std::vector<std::byte>> entries_;
        };
    } // namespace Scene
} // namespace SushiEngine

#endif
