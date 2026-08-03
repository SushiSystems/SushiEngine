/**************************************************************************/
/* avatar_mask.hpp                                                       */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file avatar_mask.hpp
 * @brief The avatar-mask asset: per-joint layer weights, addressed by joint name (phase A5).
 *
 * A mask gates which joints a layer writes (design §5.2): the upper-body aim layer over a
 * locomotion base is a mask that admits the spine-and-arms joints and excludes the legs. The
 * asset is authored as a set of named joints with a weight each and a default for the rest,
 * and is skeleton-independent — it is bound to a concrete rig at load by name hash, so one
 * mask serves every skeleton that shares the naming. The relocatable `.sushimask` blob mirrors
 * the skeleton/clip cook: magic, version, aligned sections, offsets not pointers.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp> // detail::align_up

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Magic tag at the head of every `.sushimask` blob. */
        constexpr char MASK_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'M', 'A', 'S', 'K'};

        /** @brief Current `.sushimask` format version. */
        constexpr std::uint32_t MASK_BLOB_VERSION = 1;

        /** @brief One masked joint: its name hash and the layer weight it admits. */
        struct MaskEntryRecord
        {
            NameHash name = 0;
            float weight = 1.0f;
            std::uint32_t pad = 0;
        };

        /** @brief The fixed header at offset 0 of a mask blob. */
        struct MaskBlobHeader
        {
            char magic[8];              /**< @ref MASK_BLOB_MAGIC. */
            std::uint32_t version;      /**< @ref MASK_BLOB_VERSION. */
            std::uint32_t entry_count;  /**< Named joints in the mask. */
            float default_weight;       /**< Weight for joints the mask does not name. */
            std::uint32_t total_size;   /**< Whole blob size in bytes. */
            std::uint32_t entries_offset; /**< MaskEntryRecord[entry_count]. */
            std::uint32_t reserved;     /**< Zero; pads the header. */
        };

        /**
         * @brief A non-owning, immutable view of a cooked avatar mask.
         *
         * Aliases a byte buffer owned elsewhere (the @ref AnimationDatabase). Trivially copyable.
         * @ref resolve binds the mask to a concrete skeleton, filling a per-joint weight array.
         */
        struct MaskView
        {
            std::uint32_t entry_count = 0;      /**< Named joints. */
            float default_weight = 0.0f;        /**< Weight for joints not named by the mask. */
            const MaskEntryRecord* entries = nullptr;

            /** @brief Whether the view points at real data. */
            bool valid() const noexcept { return entries != nullptr; }

            /**
             * @brief The weight this mask admits for a joint name hash.
             * @param name The FNV-1a 64 hash of the joint name.
             * @return The named weight, or @ref default_weight if the mask does not name it.
             */
            float weight_for(NameHash name) const noexcept
            {
                for (std::uint32_t i = 0; i < entry_count; ++i)
                    if (entries[i].name == name)
                        return entries[i].weight;
                return default_weight;
            }

            /**
             * @brief Binds the mask to a skeleton, filling one weight per joint.
             * @param skeleton The rig to resolve against (drives joint order and count).
             * @param out      Receives @c skeleton.joint_count weights; caller-owned.
             */
            void resolve(const SkeletonView& skeleton, float* out) const noexcept
            {
                for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
                    out[j] = valid() ? weight_for(skeleton.joint_names[j]) : 1.0f;
            }
        };

        /** @brief An avatar mask as authored: the input to @ref build_mask_blob. */
        struct MaskDesc
        {
            /** @brief One authored joint entry: the joint name and the weight it admits. */
            struct Entry
            {
                std::string joint;   /**< Joint name (hashed at cook). */
                float weight = 1.0f; /**< Layer weight this joint admits, in [0, 1]. */
            };

            std::vector<Entry> entries;
            float default_weight = 0.0f; /**< Weight for joints the mask does not name. */
        };

        /**
         * @brief Cooks a mask description into a relocatable `.sushimask` blob.
         * @param desc The authored mask.
         * @param out  Receives the blob bytes; cleared first, empty on failure.
         * @return True on success; false only if a section overflows the size type.
         */
        inline bool build_mask_blob(const MaskDesc& desc, std::vector<std::byte>& out)
        {
            out.clear();

            std::vector<MaskEntryRecord> entries;
            entries.reserve(desc.entries.size());
            for (const MaskDesc::Entry& e : desc.entries)
            {
                MaskEntryRecord record;
                record.name = hash_name(e.joint.c_str());
                record.weight = e.weight;
                entries.push_back(record);
            }

            std::size_t cursor = detail::align_up(sizeof(MaskBlobHeader), 16);
            const std::size_t entries_offset = cursor;
            cursor = detail::align_up(cursor + entries.size() * sizeof(MaskEntryRecord), 16);
            const std::size_t total_size = cursor;

            out.assign(total_size, std::byte{0});
            std::byte* base = out.data();

            MaskBlobHeader header{};
            std::memcpy(header.magic, MASK_BLOB_MAGIC, sizeof(header.magic));
            header.version = MASK_BLOB_VERSION;
            header.entry_count = static_cast<std::uint32_t>(entries.size());
            header.default_weight = desc.default_weight;
            header.total_size = static_cast<std::uint32_t>(total_size);
            header.entries_offset = static_cast<std::uint32_t>(entries_offset);
            header.reserved = 0;
            std::memcpy(base, &header, sizeof(header));

            if (!entries.empty())
                std::memcpy(base + entries_offset, entries.data(),
                            entries.size() * sizeof(MaskEntryRecord));
            return true;
        }

        /**
         * @brief Validates and views a `.sushimask` blob.
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Bytes available at @p data.
         * @return The mask view, or a default (invalid) view.
         */
        inline MaskView load_mask_blob(const std::byte* data, std::size_t size) noexcept
        {
            MaskView view{};
            if (data == nullptr || size < sizeof(MaskBlobHeader))
                return view;
            MaskBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, MASK_BLOB_MAGIC, sizeof(header.magic)) != 0 ||
                header.version != MASK_BLOB_VERSION || header.total_size > size)
                return view;
            if (header.entries_offset + header.entry_count * sizeof(MaskEntryRecord) >
                header.total_size)
                return view;
            view.entry_count = header.entry_count;
            view.default_weight = header.default_weight;
            view.entries = reinterpret_cast<const MaskEntryRecord*>(data + header.entries_offset);
            return view;
        }
    } // namespace Animation
} // namespace SushiEngine
