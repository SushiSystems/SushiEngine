/**************************************************************************/
/* clip_compress.hpp                                                     */
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
 * @file clip_compress.hpp
 * @brief The ACL-shaped clip compressor and the version-2 `.sushianim` cook/load.
 *
 * Cooks a raw @ref ClipDesc into a compressed, relocatable blob: uniform-time segments,
 * per-segment per-track range reduction, three-smallest-component quaternions, and a
 * per-segment variable bit rate chosen by an error solver that measures displacement at a
 * virtual vertex (design §4.2). Constant tracks collapse to one value. The load returns a
 * @ref ClipView whose @ref ClipView::format is Compressed, so the evaluator samples it
 * exactly as it samples a raw clip. Host / asset-domain code.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/clip_compressed.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp> // detail::align_up

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Magic tag at the head of a compressed `.sushianim` blob. */
        constexpr char COMPRESSED_CLIP_MAGIC[8] = {'S', 'U', 'S', 'H', 'A', 'C', 'M', 'P'};

        /** @brief Current compressed-clip format version. */
        constexpr std::uint32_t COMPRESSED_CLIP_VERSION = 1;

        /** @brief The fixed header at offset 0 of a compressed clip blob. */
        struct CompressedClipHeader
        {
            char magic[8];                    /**< @ref COMPRESSED_CLIP_MAGIC. */
            std::uint32_t version;            /**< @ref COMPRESSED_CLIP_VERSION. */
            std::uint32_t joint_count;        /**< Joints the clip animates. */
            std::uint32_t frame_count;        /**< Sampled frames. */
            float sample_rate;                /**< Frames per second. */
            std::uint32_t segment_count;      /**< Uniform-time segments. */
            std::uint32_t track_count;        /**< Entries in the track table (joint_count * 3). */
            std::uint32_t segment_records;    /**< Entries in the segment table. */
            std::uint32_t total_size;         /**< Whole blob size in bytes. */
            std::uint32_t tracks_offset;      /**< CompressedTrack[track_count]. */
            std::uint32_t segments_offset;    /**< CompressedSegment[segment_records]. */
            std::uint32_t bitstream_offset;   /**< Packed quantized components. */
            std::uint32_t bitstream_size;     /**< Bytes of the bitstream. */
            std::uint32_t reserved;           /**< Zero; pads the header. */
        };

        namespace detail
        {
            /** @brief Appends bits LSB-first to a growing byte buffer (mirror of read_bits). */
            struct BitWriter
            {
                std::vector<std::uint8_t> bytes;
                std::uint64_t bit_count = 0;

                void write(std::uint32_t value, std::uint32_t count)
                {
                    for (std::uint32_t i = 0; i < count; ++i)
                    {
                        const std::uint64_t position = bit_count;
                        if ((position >> 3) >= bytes.size())
                            bytes.push_back(0);
                        if ((value >> i) & 1u)
                            bytes[position >> 3] |= static_cast<std::uint8_t>(1u << (position & 7u));
                        ++bit_count;
                    }
                }
            };

            /** @brief Quantizes a value in `[minimum, minimum+extent]` to a @p bits code. */
            inline std::uint32_t quantize(float value, std::uint32_t bits, float minimum,
                                          float extent) noexcept
            {
                if (bits == 0 || extent == 0.0f)
                    return 0;
                const float maximum = static_cast<float>((1u << bits) - 1u);
                float normalized = (value - minimum) / extent;
                if (normalized < 0.0f)
                    normalized = 0.0f;
                if (normalized > 1.0f)
                    normalized = 1.0f;
                return static_cast<std::uint32_t>(normalized * maximum + 0.5f);
            }

            /** @brief Canonical three-smallest form of a quaternion: dropped index + 3 comps. */
            inline void to_three_smallest(const Quaternionf& q, std::uint32_t& dropped,
                                          float component[3]) noexcept
            {
                const float a[4] = {q.x, q.y, q.z, q.w};
                int index = 0;
                float best = std::fabs(a[0]);
                for (int i = 1; i < 4; ++i)
                {
                    const float magnitude = std::fabs(a[i]);
                    if (magnitude > best)
                    {
                        best = magnitude;
                        index = i;
                    }
                }
                const float sign = a[index] < 0.0f ? -1.0f : 1.0f;
                int c = 0;
                for (int i = 0; i < 4; ++i)
                    if (i != index)
                        component[c++] = a[i] * sign;
                dropped = static_cast<std::uint32_t>(index);
            }

            /** @brief Reconstructs a quaternion from its three-smallest form (decode side). */
            inline Quaternionf from_three_smallest(std::uint32_t dropped,
                                                   const float component[3]) noexcept
            {
                const float squared = component[0] * component[0] + component[1] * component[1] +
                                      component[2] * component[2];
                const float largest = std::sqrt(squared < 1.0f ? 1.0f - squared : 0.0f);
                float q[4];
                int c = 0;
                for (int i = 0; i < 4; ++i)
                    q[i] = (i == static_cast<int>(dropped)) ? largest : component[c++];
                return Quaternionf{q[0], q[1], q[2], q[3]};
            }

            /** @brief Max displacement of unit axes rotated by two quaternions (virtual-vertex). */
            inline float rotation_error(const Quaternionf& a, const Quaternionf& b) noexcept
            {
                const Vector3f axes[3] = {Vector3f{1, 0, 0}, Vector3f{0, 1, 0}, Vector3f{0, 0, 1}};
                float worst = 0.0f;
                for (const Vector3f& v : axes)
                {
                    const Vector3f ra = rotate(a, v);
                    const Vector3f rb = rotate(b, v);
                    const float d = length(ra - rb);
                    if (d > worst)
                        worst = d;
                }
                return worst;
            }
        } // namespace detail

        namespace detail
        {
            /** @brief The bit rates the error solver is allowed to choose from, ascending. */
            constexpr std::uint32_t CLIP_BIT_RATES[4] = {8, 11, 14, 16};

            /**
             * @brief Compresses one three-component (translation or scale) track.
             *
             * Collapses a constant track to one value; otherwise, per segment, range-reduces
             * and picks the smallest bit rate whose worst-component error stays under the
             * threshold, then packs each frame's three components.
             */
            inline void compress_vector_track(const std::vector<Vector3f>& values,
                                              std::uint32_t frame_count, std::uint32_t segment_count,
                                              float threshold, CompressedTrack& track,
                                              std::vector<CompressedSegment>& segments,
                                              BitWriter& stream)
            {
                bool constant = true;
                for (std::uint32_t f = 1; f < frame_count && constant; ++f)
                    constant = std::fabs(values[f].x - values[0].x) <= threshold &&
                               std::fabs(values[f].y - values[0].y) <= threshold &&
                               std::fabs(values[f].z - values[0].z) <= threshold;
                if (constant)
                {
                    track.constant = 1;
                    track.const_value[0] = values[0].x;
                    track.const_value[1] = values[0].y;
                    track.const_value[2] = values[0].z;
                    return;
                }

                track.constant = 0;
                track.segment_base = static_cast<std::uint32_t>(segments.size());
                for (std::uint32_t s = 0; s < segment_count; ++s)
                {
                    const std::uint32_t begin = s * CLIP_SEGMENT_SIZE;
                    const std::uint32_t end = std::min(begin + CLIP_SEGMENT_SIZE, frame_count);
                    float minimum[3] = {values[begin].x, values[begin].y, values[begin].z};
                    float maximum[3] = {values[begin].x, values[begin].y, values[begin].z};
                    for (std::uint32_t f = begin; f < end; ++f)
                    {
                        const float v[3] = {values[f].x, values[f].y, values[f].z};
                        for (int k = 0; k < 3; ++k)
                        {
                            minimum[k] = std::min(minimum[k], v[k]);
                            maximum[k] = std::max(maximum[k], v[k]);
                        }
                    }
                    CompressedSegment info;
                    for (int k = 0; k < 3; ++k)
                    {
                        info.minimum[k] = minimum[k];
                        info.extent[k] = maximum[k] - minimum[k];
                    }
                    // Error solver: the smallest rate whose worst reconstruction error holds.
                    std::uint32_t chosen = CLIP_BIT_RATES[3];
                    for (std::uint32_t rate : CLIP_BIT_RATES)
                    {
                        float worst = 0.0f;
                        for (std::uint32_t f = begin; f < end; ++f)
                        {
                            const float v[3] = {values[f].x, values[f].y, values[f].z};
                            for (int k = 0; k < 3; ++k)
                            {
                                const std::uint32_t code =
                                    quantize(v[k], rate, info.minimum[k], info.extent[k]);
                                const float back =
                                    dequantize(code, rate, info.minimum[k], info.extent[k]);
                                worst = std::max(worst, std::fabs(back - v[k]));
                            }
                        }
                        if (worst <= threshold)
                        {
                            chosen = rate;
                            break;
                        }
                    }
                    info.bits = chosen;
                    info.bit_offset = static_cast<std::uint32_t>(stream.bit_count);
                    segments.push_back(info);
                    for (std::uint32_t f = begin; f < end; ++f)
                    {
                        const float v[3] = {values[f].x, values[f].y, values[f].z};
                        for (int k = 0; k < 3; ++k)
                            stream.write(quantize(v[k], chosen, info.minimum[k], info.extent[k]),
                                         chosen);
                    }
                }
            }

            /** @brief Compresses one rotation track (three-smallest + per-frame dropped index). */
            inline void compress_rotation_track(const std::vector<Quaternionf>& values,
                                                std::uint32_t frame_count,
                                                std::uint32_t segment_count, float threshold,
                                                CompressedTrack& track,
                                                std::vector<CompressedSegment>& segments,
                                                BitWriter& stream)
            {
                bool constant = true;
                for (std::uint32_t f = 1; f < frame_count && constant; ++f)
                    constant = rotation_error(values[f], values[0]) <= threshold;
                if (constant)
                {
                    track.constant = 1;
                    track.const_value[0] = values[0].x;
                    track.const_value[1] = values[0].y;
                    track.const_value[2] = values[0].z;
                    track.const_value[3] = values[0].w;
                    return;
                }

                // Precompute the three-smallest form of every frame.
                std::vector<std::uint32_t> dropped(frame_count);
                std::vector<float> component(static_cast<std::size_t>(frame_count) * 3);
                for (std::uint32_t f = 0; f < frame_count; ++f)
                {
                    float c[3];
                    to_three_smallest(values[f], dropped[f], c);
                    component[f * 3 + 0] = c[0];
                    component[f * 3 + 1] = c[1];
                    component[f * 3 + 2] = c[2];
                }

                track.constant = 0;
                track.segment_base = static_cast<std::uint32_t>(segments.size());
                for (std::uint32_t s = 0; s < segment_count; ++s)
                {
                    const std::uint32_t begin = s * CLIP_SEGMENT_SIZE;
                    const std::uint32_t end = std::min(begin + CLIP_SEGMENT_SIZE, frame_count);
                    float minimum[3] = {component[begin * 3 + 0], component[begin * 3 + 1],
                                        component[begin * 3 + 2]};
                    float maximum[3] = {minimum[0], minimum[1], minimum[2]};
                    for (std::uint32_t f = begin; f < end; ++f)
                        for (int k = 0; k < 3; ++k)
                        {
                            minimum[k] = std::min(minimum[k], component[f * 3 + k]);
                            maximum[k] = std::max(maximum[k], component[f * 3 + k]);
                        }
                    CompressedSegment info;
                    for (int k = 0; k < 3; ++k)
                    {
                        info.minimum[k] = minimum[k];
                        info.extent[k] = maximum[k] - minimum[k];
                    }
                    std::uint32_t chosen = CLIP_BIT_RATES[3];
                    for (std::uint32_t rate : CLIP_BIT_RATES)
                    {
                        float worst = 0.0f;
                        for (std::uint32_t f = begin; f < end; ++f)
                        {
                            float back[3];
                            for (int k = 0; k < 3; ++k)
                            {
                                const std::uint32_t code = quantize(component[f * 3 + k], rate,
                                                                    info.minimum[k], info.extent[k]);
                                back[k] = dequantize(code, rate, info.minimum[k], info.extent[k]);
                            }
                            const Quaternionf decoded = from_three_smallest(dropped[f], back);
                            worst = std::max(worst, rotation_error(values[f], decoded));
                        }
                        if (worst <= threshold)
                        {
                            chosen = rate;
                            break;
                        }
                    }
                    info.bits = chosen;
                    info.bit_offset = static_cast<std::uint32_t>(stream.bit_count);
                    segments.push_back(info);
                    for (std::uint32_t f = begin; f < end; ++f)
                    {
                        stream.write(dropped[f], 2);
                        for (int k = 0; k < 3; ++k)
                            stream.write(quantize(component[f * 3 + k], chosen, info.minimum[k],
                                                  info.extent[k]),
                                         chosen);
                    }
                }
            }
        } // namespace detail

        /**
         * @brief Cooks a raw clip into a compressed, relocatable blob.
         * @param desc            The raw clip (dense frame-major tracks; see @ref ClipDesc).
         * @param error_threshold The maximum reconstruction error the solver may leave, in
         *                        the clip's units (a virtual vertex at unit distance for
         *                        rotation). Smaller keeps more bits; ~0.001–0.01 is typical.
         * @param out             Receives the blob bytes; cleared first, empty on failure.
         * @return True on success; false if the counts are out of range or tracks mis-sized.
         */
        inline bool compress_clip(const ClipDesc& desc, float error_threshold,
                                  std::vector<std::byte>& out)
        {
            out.clear();
            if (desc.joint_count == 0 || desc.joint_count > MAX_JOINTS || desc.frame_count == 0 ||
                desc.sample_rate <= 0.0f)
                return false;
            const std::size_t element_count =
                static_cast<std::size_t>(desc.frame_count) * desc.joint_count;
            if (desc.translations.size() != element_count || desc.rotations.size() != element_count ||
                desc.scales.size() != element_count)
                return false;

            const std::uint32_t J = desc.joint_count;
            const std::uint32_t F = desc.frame_count;
            const std::uint32_t segment_count = (F + CLIP_SEGMENT_SIZE - 1) / CLIP_SEGMENT_SIZE;

            std::vector<CompressedTrack> tracks(static_cast<std::size_t>(J) * 3);
            std::vector<CompressedSegment> segments;
            detail::BitWriter stream;

            std::vector<Quaternionf> rotations(F);
            std::vector<Vector3f> translations(F);
            std::vector<Vector3f> scales(F);
            for (std::uint32_t j = 0; j < J; ++j)
            {
                for (std::uint32_t f = 0; f < F; ++f)
                {
                    const std::size_t index = static_cast<std::size_t>(f) * J + j;
                    rotations[f] = desc.rotations[index];
                    translations[f] = desc.translations[index];
                    scales[f] = desc.scales[index];
                }
                detail::compress_rotation_track(rotations, F, segment_count, error_threshold,
                                                tracks[j * 3 + 0], segments, stream);
                detail::compress_vector_track(translations, F, segment_count, error_threshold,
                                              tracks[j * 3 + 1], segments, stream);
                detail::compress_vector_track(scales, F, segment_count, error_threshold,
                                              tracks[j * 3 + 2], segments, stream);
            }

            const std::size_t track_bytes = tracks.size() * sizeof(CompressedTrack);
            const std::size_t segment_bytes = segments.size() * sizeof(CompressedSegment);
            const std::size_t bitstream_bytes = stream.bytes.size();

            std::size_t cursor = detail::align_up(sizeof(CompressedClipHeader), 16);
            const std::size_t tracks_offset = cursor;
            cursor = detail::align_up(cursor + track_bytes, 16);
            const std::size_t segments_offset = cursor;
            cursor = detail::align_up(cursor + segment_bytes, 16);
            const std::size_t bitstream_offset = cursor;
            cursor = detail::align_up(cursor + bitstream_bytes, 16);
            const std::size_t total_size = cursor;

            out.assign(total_size, std::byte{0});
            std::byte* base = out.data();

            CompressedClipHeader header{};
            std::memcpy(header.magic, COMPRESSED_CLIP_MAGIC, sizeof(header.magic));
            header.version = COMPRESSED_CLIP_VERSION;
            header.joint_count = J;
            header.frame_count = F;
            header.sample_rate = desc.sample_rate;
            header.segment_count = segment_count;
            header.track_count = static_cast<std::uint32_t>(tracks.size());
            header.segment_records = static_cast<std::uint32_t>(segments.size());
            header.total_size = static_cast<std::uint32_t>(total_size);
            header.tracks_offset = static_cast<std::uint32_t>(tracks_offset);
            header.segments_offset = static_cast<std::uint32_t>(segments_offset);
            header.bitstream_offset = static_cast<std::uint32_t>(bitstream_offset);
            header.bitstream_size = static_cast<std::uint32_t>(bitstream_bytes);
            std::memcpy(base, &header, sizeof(header));
            if (track_bytes > 0)
                std::memcpy(base + tracks_offset, tracks.data(), track_bytes);
            if (segment_bytes > 0)
                std::memcpy(base + segments_offset, segments.data(), segment_bytes);
            if (bitstream_bytes > 0)
                std::memcpy(base + bitstream_offset, stream.bytes.data(), bitstream_bytes);
            return true;
        }

        /**
         * @brief Whether a byte buffer is a compressed `.sushianim` blob.
         * @param data First byte of the candidate blob.
         * @param size Bytes available at @p data.
         * @return True if @p data is a well-formed compressed clip this build can read.
         */
        inline bool validate_compressed_clip_blob(const std::byte* data, std::size_t size) noexcept
        {
            if (data == nullptr || size < sizeof(CompressedClipHeader))
                return false;
            CompressedClipHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, COMPRESSED_CLIP_MAGIC, sizeof(header.magic)) != 0)
                return false;
            if (header.version != COMPRESSED_CLIP_VERSION)
                return false;
            if (header.joint_count == 0 || header.joint_count > MAX_JOINTS ||
                header.frame_count == 0 || header.sample_rate <= 0.0f)
                return false;
            if (header.total_size > size)
                return false;
            const bool fits =
                header.tracks_offset + header.track_count * sizeof(CompressedTrack) <=
                    header.total_size &&
                header.segments_offset + header.segment_records * sizeof(CompressedSegment) <=
                    header.total_size &&
                header.bitstream_offset + header.bitstream_size <= header.total_size;
            return fits;
        }

        /**
         * @brief Builds a compressed @ref ClipView over a validated blob.
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Bytes available at @p data.
         * @return A view with @ref ClipView::format Compressed, or a default (invalid) view.
         */
        inline ClipView load_compressed_clip_blob(const std::byte* data, std::size_t size) noexcept
        {
            ClipView view{};
            if (!validate_compressed_clip_blob(data, size))
                return view;
            CompressedClipHeader header{};
            std::memcpy(&header, data, sizeof(header));
            view.format = ClipFormat::Compressed;
            view.joint_count = header.joint_count;
            view.frame_count = header.frame_count;
            view.sample_rate = header.sample_rate;
            view.duration = header.frame_count > 1
                                ? static_cast<float>(header.frame_count - 1) / header.sample_rate
                                : 0.0f;
            view.compressed.joint_count = header.joint_count;
            view.compressed.frame_count = header.frame_count;
            view.compressed.segment_count = header.segment_count;
            view.compressed.tracks =
                reinterpret_cast<const CompressedTrack*>(data + header.tracks_offset);
            view.compressed.segments =
                reinterpret_cast<const CompressedSegment*>(data + header.segments_offset);
            view.compressed.bitstream =
                reinterpret_cast<const std::uint8_t*>(data + header.bitstream_offset);
            return view;
        }

        /**
         * @brief Loads a clip blob of either format, dispatching on its magic.
         *
         * The one loader a consumer needs: a raw (`SUSHANIM`) blob yields a raw view, a
         * compressed (`SUSHACMP`) blob a compressed one, and both sample identically through
         * @ref ClipView. Used by @ref AnimationDatabase so a clip's storage is invisible to it.
         *
         * @param data First byte of the blob (must outlive the returned view).
         * @param size Bytes available at @p data.
         * @return The clip view, or a default (invalid) view if @p data is neither format.
         */
        inline ClipView load_any_clip_blob(const std::byte* data, std::size_t size) noexcept
        {
            if (validate_compressed_clip_blob(data, size))
                return load_compressed_clip_blob(data, size);
            return load_clip_blob(data, size);
        }
    } // namespace Animation
} // namespace SushiEngine
