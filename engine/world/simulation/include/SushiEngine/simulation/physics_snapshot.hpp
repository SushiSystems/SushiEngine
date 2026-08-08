/**************************************************************************/
/* physics_snapshot.hpp                                                   */
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
 * @file physics_snapshot.hpp
 * @brief Writing and reading a physics snapshot's bytes, safely and in one place.
 *
 * A cursor pair rather than a serialization framework. What a physics snapshot holds is
 * arrays of pointer-free, trivially copyable structs — which
 * `docs/design/physics_system.md` §12.3 identifies as the property that makes the whole
 * thing possible without a special case — so writing one is `memcpy` and the only real
 * work is refusing to read past the end of a blob that is not what it claims to be.
 *
 * **This is an in-process snapshot, not a file format.** Nothing writes it to disk and
 * nothing reads one produced by a different build, so a struct gaining a field is not a
 * compatibility event and there is no migration path to maintain. The header carries a
 * magic and a version regardless, because eight bytes is nothing against the failure it
 * prevents: a blob read as the wrong shape produces plausible garbage rather than an
 * error.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>
#include <vector>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Identifies a buffer as a physics snapshot; the first eight bytes. */
        constexpr std::uint32_t PHYSICS_SNAPSHOT_MAGIC = 0x53504859u; // "SPHY"

        /**
         * @brief The layout revision.
         *
         * Bumped whenever what the scene writes changes shape. It is not a
         * compatibility promise — an older blob is refused, never migrated — it is how
         * a blob captured before a rebuild becomes an error instead of a wrong answer.
         */
        constexpr std::uint32_t PHYSICS_SNAPSHOT_VERSION = 2u;

        /**
         * @brief A value with every byte zeroed, padding included.
         *
         * `Row row{}` initializes every *member* and says nothing about the bytes
         * between and after them. Whether a compiler zeroes those is a decision it
         * makes per type: it generally does for a large struct, where one `memset`
         * beats a dozen member-wise stores, and generally does not for a small one.
         *
         * A snapshot row is `memcpy`'d into the blob whole, so "generally" is the wrong
         * word — every byte written has to be a function of the simulation, and a
         * padding byte never is. @ref KinematicTargetRow carries seven bytes of tail
         * padding after its `bool`, and on this compiler `{}` left them alone; they were
         * the whole difference between two byte images of one state, twice, in two
         * different tests. Use this for anything staged for a snapshot.
         *
         * @tparam Row A trivially copyable type.
         * @return A zero-filled @p Row.
         */
        template <typename Row>
        Row zeroed() noexcept
        {
            static_assert(std::is_trivially_copyable<Row>::value,
                          "a snapshot holds plain bytes; give it a pointer-free type");
            Row row;
            std::memset(&row, 0, sizeof(Row));
            return row;
        }

        /**
         * @brief Appends values to a snapshot's byte buffer.
         *
         * Every write is an append, so the writer cannot fail and has no error state.
         * That asymmetry with @ref SnapshotReader is deliberate: producing a snapshot is
         * a local operation on state that exists, while consuming one is parsing input
         * that may be anything at all.
         */
        class SnapshotWriter
        {
            public:
                /**
                 * @brief Starts writing into @p buffer, clearing it and stamping the header.
                 * @param buffer The destination; cleared.
                 */
                explicit SnapshotWriter(std::vector<std::byte>& buffer) : buffer_(buffer)
                {
                    buffer_.clear();
                    write(PHYSICS_SNAPSHOT_MAGIC);
                    write(PHYSICS_SNAPSHOT_VERSION);
                }

                /**
                 * @brief Appends one trivially copyable value.
                 * @tparam T The value's type; must be trivially copyable.
                 * @param value The value to append.
                 */
                template <typename T>
                void write(const T& value)
                {
                    static_assert(std::is_trivially_copyable<T>::value,
                                  "a snapshot holds plain bytes; give it a pointer-free type");
                    const std::size_t offset = buffer_.size();
                    buffer_.resize(offset + sizeof(T));
                    std::memcpy(buffer_.data() + offset, &value, sizeof(T));
                }

                /**
                 * @brief Appends a count followed by that many values.
                 *
                 * The count travels with the array rather than being written separately
                 * by the caller, so a reader cannot be handed a length that belongs to a
                 * different array — which is the one framing mistake a flat blob invites.
                 *
                 * @tparam T The element type; must be trivially copyable.
                 * @param data  The first element, or null when @p count is zero.
                 * @param count How many elements follow.
                 */
                template <typename T>
                void write_array(const T* data, std::size_t count)
                {
                    static_assert(std::is_trivially_copyable<T>::value,
                                  "a snapshot holds plain bytes; give it a pointer-free type");
                    write(std::uint64_t(count));
                    if (count == 0)
                        return;
                    const std::size_t offset = buffer_.size();
                    const std::size_t bytes = sizeof(T) * count;
                    buffer_.resize(offset + bytes);
                    std::memcpy(buffer_.data() + offset, data, bytes);
                }

                /** @brief Appends a vector's contents as a counted array. */
                template <typename T>
                void write_vector(const std::vector<T>& values)
                {
                    write_array(values.data(), values.size());
                }

            private:
                std::vector<std::byte>& buffer_;
        };

        /**
         * @brief Reads values back out of a snapshot's bytes, refusing to overrun.
         *
         * Failure is sticky: once a read runs past the end, every later read fails too
         * and @ref ok stays false. That means a caller can read a whole snapshot and
         * check once at the end rather than after every field — and a caller who forgets
         * to check gets zeroed values rather than whatever happened to be adjacent.
         */
        class SnapshotReader
        {
            public:
                /**
                 * @brief Starts reading @p size bytes at @p data, validating the header.
                 * @param data The blob; may be null when @p size is zero.
                 * @param size Its length in bytes.
                 */
                SnapshotReader(const std::byte* data, std::size_t size) noexcept
                    : data_(data), size_(size)
                {
                    std::uint32_t magic = 0;
                    std::uint32_t version = 0;
                    read(magic);
                    read(version);
                    if (magic != PHYSICS_SNAPSHOT_MAGIC || version != PHYSICS_SNAPSHOT_VERSION)
                        ok_ = false;
                }

                /** @brief Whether every read so far succeeded. */
                bool ok() const noexcept { return ok_; }

                /**
                 * @brief Reads one trivially copyable value.
                 * @tparam T The value's type; must be trivially copyable.
                 * @param out Receives the value; zeroed and left alone on failure.
                 * @return Whether the read succeeded.
                 */
                template <typename T>
                bool read(T& out) noexcept
                {
                    static_assert(std::is_trivially_copyable<T>::value,
                                  "a snapshot holds plain bytes; give it a pointer-free type");
                    if (!ok_ || cursor_ + sizeof(T) > size_)
                    {
                        ok_ = false;
                        return false;
                    }
                    std::memcpy(&out, data_ + cursor_, sizeof(T));
                    cursor_ += sizeof(T);
                    return true;
                }

                /**
                 * @brief Reads a counted array into @p out, replacing its contents.
                 *
                 * The count is checked against the bytes actually remaining before the
                 * vector is resized, so a corrupt or truncated blob cannot ask for an
                 * allocation the buffer could never have held.
                 *
                 * @tparam T The element type; must be trivially copyable.
                 * @param out Receives the elements; emptied on failure.
                 * @return Whether the read succeeded.
                 */
                template <typename T>
                bool read_vector(std::vector<T>& out) noexcept
                {
                    static_assert(std::is_trivially_copyable<T>::value,
                                  "a snapshot holds plain bytes; give it a pointer-free type");
                    out.clear();
                    std::uint64_t count = 0;
                    if (!read(count))
                        return false;
                    const std::size_t bytes = std::size_t(count) * sizeof(T);
                    if (count > 0 && (bytes / sizeof(T) != std::size_t(count) ||
                                      cursor_ + bytes > size_))
                    {
                        ok_ = false;
                        return false;
                    }
                    if (count == 0)
                        return true;
                    out.resize(std::size_t(count));
                    std::memcpy(out.data(), data_ + cursor_, bytes);
                    cursor_ += bytes;
                    return true;
                }

            private:
                const std::byte* data_ = nullptr;
                std::size_t size_ = 0;
                std::size_t cursor_ = 0;
                bool ok_ = true;
        };
    } // namespace Simulation
} // namespace SushiEngine
