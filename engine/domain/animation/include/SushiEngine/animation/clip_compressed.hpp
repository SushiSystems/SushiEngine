/**************************************************************************/
/* clip_compressed.hpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
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
 * @file clip_compressed.hpp
 * @brief The ACL-shaped compressed clip: decode-side types a @ref ClipView reads.
 *
 * The compression scheme (design §4.2, after ACL): uniform-time segments, per-segment
 * per-track range reduction, three-smallest-component quaternions, per-segment variable
 * bit rates chosen by an import-time error solver, and constant-track collapse. This
 * header holds only what *decoding* needs — the flat tables a cooked blob lays out and the
 * branch-light per-frame sample. The compressor and blob cook live in
 * @ref clip_compress.hpp; the sampling seam stays @ref ClipView, so nothing downstream of
 * the view knows whether a clip is raw (A1) or compressed (A2).
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Whether a clip's tracks are stored raw (A1) or compressed (A2). */
        enum class ClipFormat : std::uint32_t
        {
            Raw = 0,
            Compressed = 1
        };

        /** @brief Frames per uniform-time compression segment (range reduction unit). */
        constexpr std::uint32_t CLIP_SEGMENT_SIZE = 16;

        /** @brief The channel kinds a compressed clip carries, three per joint. */
        enum class TrackChannel : std::uint32_t
        {
            Rotation = 0,
            Translation = 1,
            Scale = 2
        };

        /**
         * @brief Per (joint, channel) metadata in a compressed clip.
         *
         * A track that never moves collapses to a single value (`constant == 1`); otherwise
         * @ref segment_base indexes the track's first per-segment quantization record.
         */
        struct CompressedTrack
        {
            std::uint32_t constant = 0;     /**< 1 if the track is one value for the whole clip. */
            std::uint32_t segment_base = 0; /**< First segment index (when not constant). */
            float const_value[4] = {0, 0, 0, 0}; /**< The value when constant (xyzw or xyz). */
        };

        /**
         * @brief Per (track, segment) quantization: the range and bit rate the solver chose.
         *
         * The three stored components are dequantized as `minimum + extent * q / (2^bits - 1)`.
         * For rotation the three components are the three smallest of a canonicalized
         * quaternion (the largest is reconstructed at decode); a per-frame 2-bit index names
         * which component was dropped.
         */
        struct CompressedSegment
        {
            float minimum[4] = {0, 0, 0, 0};  /**< Per-component segment minimum. */
            float extent[4] = {0, 0, 0, 0};   /**< Per-component (max - min); 0 => constant. */
            std::uint32_t bits = 16;          /**< Bits per stored component this segment. */
            std::uint32_t bit_offset = 0;     /**< First bit of this segment in the bitstream. */
        };

        namespace detail
        {
            /**
             * @brief Reads @p count bits (LSB-first) at an absolute bit position.
             * @param stream The packed byte stream.
             * @param bit    Absolute bit position to read from.
             * @param count  Number of bits (0..24 in this scheme).
             * @return The unsigned value read.
             */
            inline std::uint32_t read_bits(const std::uint8_t* stream, std::uint64_t bit,
                                           std::uint32_t count) noexcept
            {
                std::uint32_t value = 0;
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    const std::uint64_t position = bit + i;
                    const std::uint8_t byte = stream[position >> 3];
                    const std::uint32_t set = (byte >> (position & 7u)) & 1u;
                    value |= set << i;
                }
                return value;
            }

            /** @brief Dequantizes a @p bits-wide code to `[minimum, minimum + extent]`. */
            inline float dequantize(std::uint32_t code, std::uint32_t bits, float minimum,
                                    float extent) noexcept
            {
                if (bits == 0 || extent == 0.0f)
                    return minimum;
                const float denominator = static_cast<float>((1u << bits) - 1u);
                return minimum + extent * (static_cast<float>(code) / denominator);
            }
        } // namespace detail

        /**
         * @brief A non-owning view of a compressed clip's flat tables (aliases the blob).
         *
         * Trivially copyable. `tracks` is `joint_count * 3` long — for joint j the rotation,
         * translation, and scale tracks are `tracks[j*3 + 0/1/2]`. A sample of one channel at
         * one frame touches exactly one segment record and one bitstream run.
         */
        struct CompressedClip
        {
            std::uint32_t joint_count = 0;
            std::uint32_t frame_count = 0;
            std::uint32_t segment_count = 0;
            const CompressedTrack* tracks = nullptr;
            const CompressedSegment* segments = nullptr;
            const std::uint8_t* bitstream = nullptr;

            /** @brief Whether the view points at real data. */
            bool valid() const noexcept { return tracks != nullptr && frame_count > 0; }

            /**
             * @brief The rotation of a joint at an exact frame.
             * @param frame Frame index in [0, frame_count).
             * @param joint Joint index in [0, joint_count).
             * @return The decoded unit quaternion.
             */
            Quaternionf rotation_at(std::uint32_t frame, std::uint32_t joint) const noexcept
            {
                const CompressedTrack& track = tracks[joint * 3 + 0];
                if (track.constant != 0)
                    return Quaternionf{track.const_value[0], track.const_value[1],
                                       track.const_value[2], track.const_value[3]};
                const std::uint32_t segment = frame / CLIP_SEGMENT_SIZE;
                const std::uint32_t frame_in_segment = frame % CLIP_SEGMENT_SIZE;
                const CompressedSegment& info = segments[track.segment_base + segment];
                const std::uint32_t bits = info.bits;
                const std::uint32_t bits_per_frame = 2 + 3 * bits; // 2-bit dropped index + 3 comps
                std::uint64_t bit = info.bit_offset +
                                    static_cast<std::uint64_t>(frame_in_segment) * bits_per_frame;
                const std::uint32_t dropped = detail::read_bits(bitstream, bit, 2);
                bit += 2;
                float component[3];
                for (int k = 0; k < 3; ++k)
                {
                    const std::uint32_t code = detail::read_bits(bitstream, bit, bits);
                    bit += bits;
                    component[k] = detail::dequantize(code, bits, info.minimum[k], info.extent[k]);
                }
                // Reconstruct the dropped (largest-magnitude) component with a positive sign;
                // a quaternion and its negation are the same rotation, so the canonicalization
                // at compress makes the sign recoverable.
                float squared = component[0] * component[0] + component[1] * component[1] +
                                component[2] * component[2];
                const float largest = std::sqrt(squared < 1.0f ? 1.0f - squared : 0.0f);
                float q[4];
                int c = 0;
                for (int i = 0; i < 4; ++i)
                    q[i] = (i == static_cast<int>(dropped)) ? largest : component[c++];
                return Quaternionf{q[0], q[1], q[2], q[3]};
            }

            /**
             * @brief The translation of a joint at an exact frame.
             * @param frame Frame index in [0, frame_count).
             * @param joint Joint index in [0, joint_count).
             * @return The decoded translation.
             */
            Vector3f translation_at(std::uint32_t frame, std::uint32_t joint) const noexcept
            {
                return vector_at(frame, joint, 1);
            }

            /**
             * @brief The scale of a joint at an exact frame.
             * @param frame Frame index in [0, frame_count).
             * @param joint Joint index in [0, joint_count).
             * @return The decoded scale.
             */
            Vector3f scale_at(std::uint32_t frame, std::uint32_t joint) const noexcept
            {
                return vector_at(frame, joint, 2);
            }

            private:
            /** @brief Shared decode for the two three-component (translation/scale) channels. */
            Vector3f vector_at(std::uint32_t frame, std::uint32_t joint,
                               std::uint32_t channel) const noexcept
            {
                const CompressedTrack& track = tracks[joint * 3 + channel];
                if (track.constant != 0)
                    return Vector3f{track.const_value[0], track.const_value[1],
                                    track.const_value[2]};
                const std::uint32_t segment = frame / CLIP_SEGMENT_SIZE;
                const std::uint32_t frame_in_segment = frame % CLIP_SEGMENT_SIZE;
                const CompressedSegment& info = segments[track.segment_base + segment];
                const std::uint32_t bits = info.bits;
                const std::uint32_t bits_per_frame = 3 * bits;
                std::uint64_t bit = info.bit_offset +
                                    static_cast<std::uint64_t>(frame_in_segment) * bits_per_frame;
                float component[3];
                for (int k = 0; k < 3; ++k)
                {
                    const std::uint32_t code = detail::read_bits(bitstream, bit, bits);
                    bit += bits;
                    component[k] = detail::dequantize(code, bits, info.minimum[k], info.extent[k]);
                }
                return Vector3f{component[0], component[1], component[2]};
            }
        };
    } // namespace Animation
} // namespace SushiEngine
