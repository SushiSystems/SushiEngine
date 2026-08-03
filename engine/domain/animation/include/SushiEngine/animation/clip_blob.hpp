/**************************************************************************/
/* clip_blob.hpp                                                          */
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
 * @file clip_blob.hpp
 * @brief Cook and load of the relocatable `.sushianim` uncompressed clip blob.
 *
 * The A1 clip format: three dense, frame-major track arrays (translation, rotation,
 * scale) at a single uniform sample rate. The cook validates the desc and lays the
 * sections out at aligned offsets; the load validates the header and returns a
 * @ref ClipView aliasing the bytes. Host / asset-domain code — it never runs on the
 * device.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp> // detail::align_up

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Magic tag at the head of every `.sushianim` blob. */
        constexpr char CLIP_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'A', 'N', 'I', 'M'};

        /** @brief Current `.sushianim` format version (2 adds morph + generic float tracks). */
        constexpr std::uint32_t CLIP_BLOB_VERSION = 2;

        /** @brief The fixed header at offset 0 of a clip blob. */
        struct ClipBlobHeader
        {
            char magic[8];                       /**< @ref CLIP_BLOB_MAGIC. */
            std::uint32_t version;               /**< @ref CLIP_BLOB_VERSION. */
            std::uint32_t joint_count;           /**< Joints the clip animates. */
            std::uint32_t frame_count;           /**< Sampled frames. */
            float sample_rate;                   /**< Frames per second. */
            std::uint32_t morph_track_count;     /**< Morph-weight tracks. */
            std::uint32_t generic_track_count;   /**< Generic float tracks. */
            std::uint32_t total_size;            /**< Whole blob size in bytes. */
            std::uint32_t translations_offset;   /**< Vector3f[frame_count * joint_count]. */
            std::uint32_t rotations_offset;      /**< Quaternionf[frame_count * joint_count]. */
            std::uint32_t scales_offset;         /**< Vector3f[frame_count * joint_count]. */
            std::uint32_t morph_names_offset;    /**< NameHash[morph_track_count]. */
            std::uint32_t morph_weights_offset;  /**< float[frame_count * morph_track_count]. */
            std::uint32_t generic_names_offset;  /**< NameHash[generic_track_count]. */
            std::uint32_t generic_values_offset; /**< float[frame_count * generic_track_count]. */
        };

        /**
         * @brief An animation clip as authored: the input to @ref build_clip_blob.
         *
         * The three joint track vectors are frame-major, each `frame_count * joint_count` long
         * (element `frame * joint_count + joint`). The optional morph and generic track vectors
         * are frame-major over their own track counts (`frame * track_count + track`).
         */
        struct ClipDesc
        {
            std::uint32_t joint_count = 0;
            std::uint32_t frame_count = 0;
            float sample_rate = 30.0f;
            std::vector<Vector3f> translations;
            std::vector<Quaternionf> rotations;
            std::vector<Vector3f> scales;
            std::vector<std::string> morph_names;    /**< One target name per morph track. */
            std::vector<float> morph_weights;        /**< `frame_count * morph_names.size()`. */
            std::vector<std::string> generic_names;  /**< One property name per generic track. */
            std::vector<float> generic_values;       /**< `frame_count * generic_names.size()`. */
        };

        /**
         * @brief Cooks a clip description into a relocatable `.sushianim` blob.
         * @param desc The authored clip; the three track vectors must each hold exactly
         *             `frame_count * joint_count` elements.
         * @param out  Receives the blob bytes; cleared first, empty on failure.
         * @return True on success; false if the counts are out of range or the track
         *         vectors are the wrong length.
         */
        inline bool build_clip_blob(const ClipDesc& desc, std::vector<std::byte>& out)
        {
            out.clear();
            if (desc.joint_count == 0 || desc.joint_count > MAX_JOINTS)
                return false;
            if (desc.frame_count == 0 || desc.frame_count > MAX_CLIP_FRAMES)
                return false;
            if (desc.sample_rate <= 0.0f)
                return false;
            const std::size_t element_count =
                static_cast<std::size_t>(desc.frame_count) * desc.joint_count;
            if (desc.translations.size() != element_count || desc.rotations.size() != element_count ||
                desc.scales.size() != element_count)
                return false;

            const std::size_t morph_count = desc.morph_names.size();
            const std::size_t generic_count = desc.generic_names.size();
            if (desc.morph_weights.size() != morph_count * desc.frame_count ||
                desc.generic_values.size() != generic_count * desc.frame_count)
                return false;

            // Hash the track names to the identity the runtime addresses them by.
            std::vector<NameHash> morph_hashes(morph_count);
            for (std::size_t i = 0; i < morph_count; ++i)
                morph_hashes[i] = hash_name(desc.morph_names[i].c_str());
            std::vector<NameHash> generic_hashes(generic_count);
            for (std::size_t i = 0; i < generic_count; ++i)
                generic_hashes[i] = hash_name(desc.generic_names[i].c_str());

            std::size_t cursor = detail::align_up(sizeof(ClipBlobHeader), 16);
            const auto section = [&cursor](std::size_t bytes) -> std::size_t
            {
                const std::size_t offset = cursor;
                cursor = detail::align_up(cursor + bytes, 16);
                return offset;
            };
            const std::size_t translations_offset = section(element_count * sizeof(Vector3f));
            const std::size_t rotations_offset = section(element_count * sizeof(Quaternionf));
            const std::size_t scales_offset = section(element_count * sizeof(Vector3f));
            const std::size_t morph_names_offset = section(morph_count * sizeof(NameHash));
            const std::size_t morph_weights_offset =
                section(desc.morph_weights.size() * sizeof(float));
            const std::size_t generic_names_offset = section(generic_count * sizeof(NameHash));
            const std::size_t generic_values_offset =
                section(desc.generic_values.size() * sizeof(float));
            const std::size_t total_size = cursor;

            out.assign(total_size, std::byte{0});
            std::byte* base = out.data();

            ClipBlobHeader header{};
            std::memcpy(header.magic, CLIP_BLOB_MAGIC, sizeof(header.magic));
            header.version = CLIP_BLOB_VERSION;
            header.joint_count = desc.joint_count;
            header.frame_count = desc.frame_count;
            header.sample_rate = desc.sample_rate;
            header.morph_track_count = static_cast<std::uint32_t>(morph_count);
            header.generic_track_count = static_cast<std::uint32_t>(generic_count);
            header.total_size = static_cast<std::uint32_t>(total_size);
            header.translations_offset = static_cast<std::uint32_t>(translations_offset);
            header.rotations_offset = static_cast<std::uint32_t>(rotations_offset);
            header.scales_offset = static_cast<std::uint32_t>(scales_offset);
            header.morph_names_offset = static_cast<std::uint32_t>(morph_names_offset);
            header.morph_weights_offset = static_cast<std::uint32_t>(morph_weights_offset);
            header.generic_names_offset = static_cast<std::uint32_t>(generic_names_offset);
            header.generic_values_offset = static_cast<std::uint32_t>(generic_values_offset);
            std::memcpy(base, &header, sizeof(header));

            const auto copy = [&](std::size_t offset, const void* data, std::size_t bytes)
            {
                if (bytes > 0)
                    std::memcpy(base + offset, data, bytes);
            };
            copy(translations_offset, desc.translations.data(), element_count * sizeof(Vector3f));
            copy(rotations_offset, desc.rotations.data(), element_count * sizeof(Quaternionf));
            copy(scales_offset, desc.scales.data(), element_count * sizeof(Vector3f));
            copy(morph_names_offset, morph_hashes.data(), morph_count * sizeof(NameHash));
            copy(morph_weights_offset, desc.morph_weights.data(),
                 desc.morph_weights.size() * sizeof(float));
            copy(generic_names_offset, generic_hashes.data(), generic_count * sizeof(NameHash));
            copy(generic_values_offset, desc.generic_values.data(),
                 desc.generic_values.size() * sizeof(float));
            return true;
        }

        /**
         * @brief Validates a byte buffer as a `.sushianim` blob.
         * @param data First byte of the candidate blob.
         * @param size Bytes available at @p data.
         * @return True if @p data is a well-formed clip blob this build can read.
         */
        inline bool validate_clip_blob(const std::byte* data, std::size_t size) noexcept
        {
            if (data == nullptr || size < sizeof(ClipBlobHeader))
                return false;
            ClipBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, CLIP_BLOB_MAGIC, sizeof(header.magic)) != 0)
                return false;
            if (header.version != CLIP_BLOB_VERSION)
                return false;
            if (header.joint_count == 0 || header.joint_count > MAX_JOINTS ||
                header.frame_count == 0 || header.sample_rate <= 0.0f)
                return false;
            if (header.total_size > size)
                return false;
            const std::size_t element_count =
                static_cast<std::size_t>(header.frame_count) * header.joint_count;
            const std::size_t morph_elements =
                static_cast<std::size_t>(header.frame_count) * header.morph_track_count;
            const std::size_t generic_elements =
                static_cast<std::size_t>(header.frame_count) * header.generic_track_count;
            const bool fits =
                header.translations_offset + element_count * sizeof(Vector3f) <= header.total_size &&
                header.rotations_offset + element_count * sizeof(Quaternionf) <= header.total_size &&
                header.scales_offset + element_count * sizeof(Vector3f) <= header.total_size &&
                header.morph_names_offset + header.morph_track_count * sizeof(NameHash) <=
                    header.total_size &&
                header.morph_weights_offset + morph_elements * sizeof(float) <= header.total_size &&
                header.generic_names_offset + header.generic_track_count * sizeof(NameHash) <=
                    header.total_size &&
                header.generic_values_offset + generic_elements * sizeof(float) <= header.total_size;
            return fits;
        }

        /**
         * @brief Builds a @ref ClipView over a validated blob.
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Bytes available at @p data.
         * @return A view of the clip, or a default (invalid) view.
         */
        inline ClipView load_clip_blob(const std::byte* data, std::size_t size) noexcept
        {
            ClipView view{};
            if (!validate_clip_blob(data, size))
                return view;
            ClipBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            view.joint_count = header.joint_count;
            view.frame_count = header.frame_count;
            view.sample_rate = header.sample_rate;
            view.duration = header.frame_count > 1
                                ? static_cast<float>(header.frame_count - 1) / header.sample_rate
                                : 0.0f;
            view.translations = reinterpret_cast<const Vector3f*>(data + header.translations_offset);
            view.rotations = reinterpret_cast<const Quaternionf*>(data + header.rotations_offset);
            view.scales = reinterpret_cast<const Vector3f*>(data + header.scales_offset);
            view.morph_track_count = header.morph_track_count;
            view.generic_track_count = header.generic_track_count;
            if (header.morph_track_count > 0)
            {
                view.morph_names = reinterpret_cast<const NameHash*>(data + header.morph_names_offset);
                view.morph_weights = reinterpret_cast<const float*>(data + header.morph_weights_offset);
            }
            if (header.generic_track_count > 0)
            {
                view.generic_names =
                    reinterpret_cast<const NameHash*>(data + header.generic_names_offset);
                view.generic_values =
                    reinterpret_cast<const float*>(data + header.generic_values_offset);
            }
            return view;
        }
    } // namespace Animation
} // namespace SushiEngine
