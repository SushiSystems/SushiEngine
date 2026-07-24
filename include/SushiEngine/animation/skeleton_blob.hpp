/**************************************************************************/
/* skeleton_blob.hpp                                                      */
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
 * @file skeleton_blob.hpp
 * @brief Cook and load of the relocatable `.sushiskel` skeleton blob.
 *
 * The cook takes a host-side @ref SkeletonDesc — joints in any order, parents by index
 * — and produces a versioned, little-endian, self-contained byte buffer: it
 * topologically sorts the joints so `parent[i] < i` (kills the non-topological
 * hierarchy liability at import), remaps every reference, derives inverse-bind
 * matrices from the bind pose when the source did not supply them, and lays the
 * structure-of-arrays sections out at aligned offsets. The load is the mirror: it
 * validates the header and returns a @ref SkeletonView of raw pointers into the
 * caller-owned bytes, with zero parsing or allocation.
 *
 * This is host / asset-domain code (it uses @c std::vector and reads bind poses in
 * double precision); it never runs on the device.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Magic tag at the head of every `.sushiskel` blob. */
        constexpr char SKELETON_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'S', 'K', 'E', 'L'};

        /** @brief Current `.sushiskel` format version. */
        constexpr std::uint32_t SKELETON_BLOB_VERSION = 2;

        /**
         * @brief The fixed header at offset 0 of a skeleton blob.
         *
         * Every field is a fixed-width little-endian integer; the offsets are byte
         * positions of each SoA section from the blob's start, so the blob is position
         * independent — it can be memory-mapped, copied, or uploaded and the loader
         * rebuilds all pointers from these.
         */
        struct SkeletonBlobHeader
        {
            char magic[8];                             /**< @ref SKELETON_BLOB_MAGIC. */
            std::uint32_t version;                     /**< @ref SKELETON_BLOB_VERSION. */
            std::uint32_t joint_count;                 /**< Joints in the skeleton. */
            std::uint32_t lod_count;                   /**< Bone-LOD levels (>= 1). */
            std::uint32_t total_size;                  /**< Whole blob size in bytes. */
            std::uint32_t parents_offset;              /**< uint16[joint_count]. */
            std::uint32_t bind_translations_offset;    /**< Vector3f[joint_count]. */
            std::uint32_t bind_rotations_offset;       /**< Quaternionf[joint_count]. */
            std::uint32_t bind_scales_offset;          /**< Vector3f[joint_count]. */
            std::uint32_t inverse_bind_offset;         /**< JointMatrix[joint_count]. */
            std::uint32_t joint_names_offset;          /**< NameHash[joint_count]. */
            std::uint32_t lod_joint_counts_offset;     /**< uint16[lod_count]. */
            std::uint32_t name_offsets_offset;         /**< uint32[joint_count]: name start per joint. */
            std::uint32_t name_data_offset;            /**< Concatenated null-terminated names. */
            std::uint32_t name_data_size;              /**< Bytes in the name string blob. */
        };

        /** @brief One joint as authored, before the cook sorts and remaps it. */
        struct JointDesc
        {
            std::string name;                          /**< Hashed into the blob at cook. */
            int parent = -1;                           /**< Index into the desc's joints, -1 for a root. */
            Vector3f bind_translation{};               /**< Local bind translation. */
            Quaternionf bind_rotation{};               /**< Local bind rotation (defaults to identity). */
            Vector3f bind_scale{1.0f, 1.0f, 1.0f};     /**< Local bind scale. */
            JointMatrix inverse_bind{};                /**< Object-to-joint at bind; derived if not supplied. */
        };

        /** @brief A skeleton as authored: the input to @ref build_skeleton_blob. */
        struct SkeletonDesc
        {
            std::vector<JointDesc> joints;

            /**
             * @brief Whether @ref JointDesc::inverse_bind carries real matrices.
             *
             * glTF supplies inverse-bind matrices, so the importer sets this true. A
             * hand-authored skeleton leaves it false and the cook derives them from the
             * bind pose (forward-composing globals, then inverting), which is exactly what
             * a well-formed glTF's matrices already equal.
             */
            bool has_inverse_bind = false;
        };

        namespace detail
        {
            /** @brief Rounds @p value up to the next multiple of @p alignment. */
            inline std::size_t align_up(std::size_t value, std::size_t alignment) noexcept
            {
                return (value + alignment - 1) & ~(alignment - 1);
            }

            /** @brief Depth of a joint in the desc hierarchy, or -1 if the parent chain cycles. */
            inline int joint_depth(const std::vector<JointDesc>& joints, int index) noexcept
            {
                int depth = 0;
                int cursor = joints[static_cast<std::size_t>(index)].parent;
                const int limit = static_cast<int>(joints.size());
                while (cursor >= 0)
                {
                    if (depth > limit)
                        return -1; // cycle
                    ++depth;
                    cursor = joints[static_cast<std::size_t>(cursor)].parent;
                }
                return depth;
            }
        } // namespace detail

        /**
         * @brief Cooks a skeleton description into a relocatable `.sushiskel` blob.
         *
         * Sorts the joints by (depth, original index) so `parent[i] < i` holds for the
         * output — a stable order that keeps siblings in their authored sequence — remaps
         * all parent references, derives inverse-bind matrices from the bind pose when
         * @ref SkeletonDesc::has_inverse_bind is false, and writes the SoA sections at
         * 16-byte-aligned offsets. A single full-detail LOD level is emitted (the bone-LOD
         * ladder is a later phase); the format already carries the array.
         *
         * @param desc The authored skeleton (must have between 1 and @ref MAX_JOINTS joints).
         * @param out  Receives the blob bytes; cleared first. Empty on failure.
         * @param out_order When non-null, receives the sort mapping: `(*out_order)[new_index]`
         *                  is the original desc joint index now at `new_index`. The animation
         *                  importer uses it to resample clip tracks in the blob's joint order.
         * @return True on success; false if the joint count is out of range or the
         *         hierarchy contains a cycle.
         */
        inline bool build_skeleton_blob(const SkeletonDesc& desc, std::vector<std::byte>& out,
                                        std::vector<int>* out_order = nullptr)
        {
            out.clear();
            const std::size_t count = desc.joints.size();
            if (count == 0 || count > MAX_JOINTS)
                return false;

            // Stable topological order: sort indices by (depth, original index). A parent is
            // strictly shallower than its child, so it always sorts earlier => parent[i] < i.
            std::vector<int> order(count);
            std::vector<int> depth(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                order[i] = static_cast<int>(i);
                depth[i] = detail::joint_depth(desc.joints, static_cast<int>(i));
                if (depth[i] < 0)
                    return false; // cycle
            }
            std::stable_sort(order.begin(), order.end(), [&](int a, int b)
            {
                return depth[static_cast<std::size_t>(a)] < depth[static_cast<std::size_t>(b)];
            });

            // remap[old_index] = new_index.
            std::vector<int> remap(count);
            for (std::size_t new_index = 0; new_index < count; ++new_index)
                remap[static_cast<std::size_t>(order[new_index])] = static_cast<int>(new_index);
            if (out_order != nullptr)
                *out_order = order;

            std::vector<std::uint16_t> parents(count);
            std::vector<Vector3f> translations(count);
            std::vector<Quaternionf> rotations(count);
            std::vector<Vector3f> scales(count);
            std::vector<JointMatrix> inverse_bind(count);
            std::vector<NameHash> names(count);
            std::vector<std::string> name_strings(count);
            for (std::size_t new_index = 0; new_index < count; ++new_index)
            {
                const JointDesc& joint = desc.joints[static_cast<std::size_t>(order[new_index])];
                parents[new_index] = joint.parent < 0
                                         ? NO_PARENT
                                         : static_cast<std::uint16_t>(remap[static_cast<std::size_t>(joint.parent)]);
                translations[new_index] = joint.bind_translation;
                rotations[new_index] = joint.bind_rotation;
                scales[new_index] = joint.bind_scale;
                names[new_index] = hash_name(joint.name.c_str());
                name_strings[new_index] = joint.name;
                inverse_bind[new_index] = joint.inverse_bind;
            }

            // Concatenated null-terminated names, plus a byte offset per joint into them.
            std::vector<std::uint32_t> name_offsets(count);
            std::vector<char> name_data;
            for (std::size_t i = 0; i < count; ++i)
            {
                name_offsets[i] = static_cast<std::uint32_t>(name_data.size());
                name_data.insert(name_data.end(), name_strings[i].begin(), name_strings[i].end());
                name_data.push_back('\0');
            }

            // Derive inverse-bind matrices from the bind pose when the source lacked them.
            // parent[i] < i lets globals be a single forward scan.
            if (!desc.has_inverse_bind)
            {
                std::vector<Mat4> global(count);
                for (std::size_t i = 0; i < count; ++i)
                {
                    const Mat4 local = compose_transform(
                        Vector3{translations[i].x, translations[i].y, translations[i].z},
                        Quaternion{rotations[i].x, rotations[i].y, rotations[i].z, rotations[i].w},
                        Vector3{scales[i].x, scales[i].y, scales[i].z});
                    global[i] = parents[i] == NO_PARENT
                                    ? local
                                    : mul(global[parents[i]], local);
                    inverse_bind[i] = to_joint_matrix(affine_inverse(global[i]));
                }
            }

            const std::vector<std::uint16_t> lod_joint_counts{static_cast<std::uint16_t>(count)};
            const std::uint32_t lod_count = 1;

            // Lay out sections at 16-byte-aligned offsets after the header.
            std::size_t cursor = detail::align_up(sizeof(SkeletonBlobHeader), 16);
            const std::size_t parents_offset = cursor;
            cursor = detail::align_up(cursor + parents.size() * sizeof(std::uint16_t), 16);
            const std::size_t translations_offset = cursor;
            cursor = detail::align_up(cursor + translations.size() * sizeof(Vector3f), 16);
            const std::size_t rotations_offset = cursor;
            cursor = detail::align_up(cursor + rotations.size() * sizeof(Quaternionf), 16);
            const std::size_t scales_offset = cursor;
            cursor = detail::align_up(cursor + scales.size() * sizeof(Vector3f), 16);
            const std::size_t inverse_bind_offset = cursor;
            cursor = detail::align_up(cursor + inverse_bind.size() * sizeof(JointMatrix), 16);
            const std::size_t names_offset = cursor;
            cursor = detail::align_up(cursor + names.size() * sizeof(NameHash), 16);
            const std::size_t lod_offset = cursor;
            cursor = detail::align_up(cursor + lod_joint_counts.size() * sizeof(std::uint16_t), 16);
            const std::size_t name_offsets_offset = cursor;
            cursor = detail::align_up(cursor + name_offsets.size() * sizeof(std::uint32_t), 16);
            const std::size_t name_data_offset = cursor;
            cursor = detail::align_up(cursor + name_data.size(), 16);
            const std::size_t total_size = cursor;

            out.assign(total_size, std::byte{0});
            std::byte* base = out.data();

            SkeletonBlobHeader header{};
            std::memcpy(header.magic, SKELETON_BLOB_MAGIC, sizeof(header.magic));
            header.version = SKELETON_BLOB_VERSION;
            header.joint_count = static_cast<std::uint32_t>(count);
            header.lod_count = lod_count;
            header.total_size = static_cast<std::uint32_t>(total_size);
            header.parents_offset = static_cast<std::uint32_t>(parents_offset);
            header.bind_translations_offset = static_cast<std::uint32_t>(translations_offset);
            header.bind_rotations_offset = static_cast<std::uint32_t>(rotations_offset);
            header.bind_scales_offset = static_cast<std::uint32_t>(scales_offset);
            header.inverse_bind_offset = static_cast<std::uint32_t>(inverse_bind_offset);
            header.joint_names_offset = static_cast<std::uint32_t>(names_offset);
            header.lod_joint_counts_offset = static_cast<std::uint32_t>(lod_offset);
            header.name_offsets_offset = static_cast<std::uint32_t>(name_offsets_offset);
            header.name_data_offset = static_cast<std::uint32_t>(name_data_offset);
            header.name_data_size = static_cast<std::uint32_t>(name_data.size());
            std::memcpy(base, &header, sizeof(header));

            std::memcpy(base + parents_offset, parents.data(), parents.size() * sizeof(std::uint16_t));
            std::memcpy(base + translations_offset, translations.data(), translations.size() * sizeof(Vector3f));
            std::memcpy(base + rotations_offset, rotations.data(), rotations.size() * sizeof(Quaternionf));
            std::memcpy(base + scales_offset, scales.data(), scales.size() * sizeof(Vector3f));
            std::memcpy(base + inverse_bind_offset, inverse_bind.data(), inverse_bind.size() * sizeof(JointMatrix));
            std::memcpy(base + names_offset, names.data(), names.size() * sizeof(NameHash));
            std::memcpy(base + lod_offset, lod_joint_counts.data(), lod_joint_counts.size() * sizeof(std::uint16_t));
            std::memcpy(base + name_offsets_offset, name_offsets.data(), name_offsets.size() * sizeof(std::uint32_t));
            if (!name_data.empty())
                std::memcpy(base + name_data_offset, name_data.data(), name_data.size());
            return true;
        }

        /**
         * @brief Validates a byte buffer as a `.sushiskel` blob.
         *
         * Checks the magic, version, that the buffer is large enough for the header and
         * every section it points at, and that the joint count is in range. Cheap enough
         * to run on every load.
         *
         * @param data First byte of the candidate blob.
         * @param size Number of bytes available at @p data.
         * @return True if @p data is a well-formed blob this build can read.
         */
        inline bool validate_skeleton_blob(const std::byte* data, std::size_t size) noexcept
        {
            if (data == nullptr || size < sizeof(SkeletonBlobHeader))
                return false;
            SkeletonBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, SKELETON_BLOB_MAGIC, sizeof(header.magic)) != 0)
                return false;
            if (header.version != SKELETON_BLOB_VERSION)
                return false;
            if (header.joint_count == 0 || header.joint_count > MAX_JOINTS || header.lod_count == 0)
                return false;
            if (header.total_size > size)
                return false;
            const std::uint32_t n = header.joint_count;
            const bool fits =
                header.parents_offset + n * sizeof(std::uint16_t) <= header.total_size &&
                header.bind_translations_offset + n * sizeof(Vector3f) <= header.total_size &&
                header.bind_rotations_offset + n * sizeof(Quaternionf) <= header.total_size &&
                header.bind_scales_offset + n * sizeof(Vector3f) <= header.total_size &&
                header.inverse_bind_offset + n * sizeof(JointMatrix) <= header.total_size &&
                header.joint_names_offset + n * sizeof(NameHash) <= header.total_size &&
                header.lod_joint_counts_offset + header.lod_count * sizeof(std::uint16_t) <= header.total_size &&
                header.name_offsets_offset + n * sizeof(std::uint32_t) <= header.total_size &&
                header.name_data_offset + header.name_data_size <= header.total_size;
            return fits;
        }

        /**
         * @brief Builds a @ref SkeletonView over a validated blob.
         *
         * The view aliases @p data; it stays valid as long as the buffer does and copies
         * nothing. Returns an empty (invalid) view if @p data is not a well-formed blob.
         *
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Number of bytes available at @p data.
         * @return A view of the skeleton, or a default view (see @ref SkeletonView::valid).
         */
        inline SkeletonView load_skeleton_blob(const std::byte* data, std::size_t size) noexcept
        {
            SkeletonView view{};
            if (!validate_skeleton_blob(data, size))
                return view;
            SkeletonBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            view.joint_count = header.joint_count;
            view.lod_count = header.lod_count;
            view.parents = reinterpret_cast<const std::uint16_t*>(data + header.parents_offset);
            view.bind_translations = reinterpret_cast<const Vector3f*>(data + header.bind_translations_offset);
            view.bind_rotations = reinterpret_cast<const Quaternionf*>(data + header.bind_rotations_offset);
            view.bind_scales = reinterpret_cast<const Vector3f*>(data + header.bind_scales_offset);
            view.inverse_bind = reinterpret_cast<const JointMatrix*>(data + header.inverse_bind_offset);
            view.joint_names = reinterpret_cast<const NameHash*>(data + header.joint_names_offset);
            view.lod_joint_counts = reinterpret_cast<const std::uint16_t*>(data + header.lod_joint_counts_offset);
            view.joint_name_offsets = reinterpret_cast<const std::uint32_t*>(data + header.name_offsets_offset);
            view.joint_name_data = reinterpret_cast<const char*>(data + header.name_data_offset);
            return view;
        }
    } // namespace Animation
} // namespace SushiEngine
