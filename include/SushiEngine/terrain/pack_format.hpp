/**************************************************************************/
/* pack_format.hpp                                                        */
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
 * @file pack_format.hpp
 * @brief The baked terrain asset: its byte layout, its reader, and its refusals.
 *
 * `se planet bake` writes one of these per body per quality tier
 * (`docs/slop/solar_system_overhaul.md` §5.2). This header is the authority on the
 * layout and `cli/sushiengine/services/planet/pack.py` is a transcription of it, for the
 * reason the climatology asset gives for the same arrangement: a schema shared between a
 * Python tool and an engine header would be a third thing to keep in step with both.
 * Drift is caught rather than tolerated — @ref PlanetPack::adopt refuses a blob it does
 * not recognise *whole*, so a mismatch is a loud refusal at load time and never terrain
 * quietly grown on a misread height.
 *
 * Elevations are stored as 16-bit values over a per-tile range rather than as floats.
 * That halves the asset and costs nothing that matters: the quantisation step is the
 * tile's own relief divided by 65535, which is 0.15 m across a tile spanning ten
 * kilometres of altitude and far finer across a typical one. It is also what makes the
 * bake's accuracy claim checkable — the exit criterion is agreement with the source
 * within this bound, which is a number rather than an impression.
 *
 * Every multi-byte field is little-endian, which every platform the engine targets is.
 */

#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

#include <SushiEngine/terrain/cube_sphere.hpp>
#include <SushiEngine/terrain/height_source.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /** @brief The eight magic bytes every planet pack opens with. */
        constexpr char PLANET_PACK_MAGIC[8] = {'S', 'U', 'S', 'H', 'I', 'P', 'L', 'A'};

        /** @brief Layout version; a reader refuses anything else rather than guessing. */
        constexpr std::uint32_t PLANET_PACK_VERSION = 1;

        /** @brief Bytes of fixed header before the provenance string. */
        constexpr std::uint32_t PLANET_PACK_HEADER_BYTES = 64;

        /** @brief Bytes of one index record. */
        constexpr std::uint32_t PLANET_PACK_RECORD_BYTES = 40;

        /**
         * @brief Elevations as 16-bit fractions of the tile's own range.
         *
         * The only codec version 1 defines. The field exists so a compressed codec can be
         * added without a format break — a reader that meets an unknown codec refuses the
         * pack rather than misreading its payload.
         */
        constexpr std::uint16_t PLANET_PACK_CODEC_QUANTISED = 0;

        /** @brief Longest provenance string a reader accepts. */
        constexpr std::uint32_t PLANET_PACK_MAX_PROVENANCE_BYTES = 65536;

        /** @brief What the index says about one stored tile. */
        struct PlanetPackRecord
        {
            std::uint64_t key = 0;    /**< @ref tile_address_key of the tile. */
            std::uint64_t offset = 0; /**< Byte offset of its payload within the blob. */
            std::uint32_t length = 0; /**< Payload bytes. */
            std::uint16_t codec = PLANET_PACK_CODEC_QUANTISED;

            /**
             * @brief The range the 16-bit payload is a fraction of.
             *
             * Covers *every* stored sample, apron included, so an apron value cannot
             * clamp. Distinct from the grid range below, which is what a bounding volume
             * is built from.
             */
            float quantised_minimum_metres = 0.0f;
            float quantised_maximum_metres = 0.0f;

            /** @brief The tile's own grid range, apron excluded: its bounding volume. */
            float grid_minimum_metres = 0.0f;
            float grid_maximum_metres = 0.0f;
        };

        namespace Detail
        {
            /** @brief Reads a little-endian 16-bit word. */
            inline std::uint16_t read_u16(const std::uint8_t* bytes) noexcept
            {
                return static_cast<std::uint16_t>(static_cast<std::uint16_t>(bytes[0]) |
                                                  (static_cast<std::uint16_t>(bytes[1]) << 8));
            }

            /** @brief Reads a little-endian 32-bit word. */
            inline std::uint32_t read_u32(const std::uint8_t* bytes) noexcept
            {
                return static_cast<std::uint32_t>(bytes[0]) |
                       (static_cast<std::uint32_t>(bytes[1]) << 8) |
                       (static_cast<std::uint32_t>(bytes[2]) << 16) |
                       (static_cast<std::uint32_t>(bytes[3]) << 24);
            }

            /** @brief Reads a little-endian 64-bit word. */
            inline std::uint64_t read_u64(const std::uint8_t* bytes) noexcept
            {
                std::uint64_t value = 0;
                for (int index = 7; index >= 0; --index)
                    value = (value << 8) | static_cast<std::uint64_t>(bytes[index]);
                return value;
            }

            /** @brief Reads a little-endian IEEE-754 single. */
            inline float read_f32(const std::uint8_t* bytes) noexcept
            {
                const std::uint32_t word = read_u32(bytes);
                float value = 0.0f;
                std::memcpy(&value, &word, sizeof(value));
                return value;
            }

            /** @brief Reads a little-endian IEEE-754 double. */
            inline double read_f64(const std::uint8_t* bytes) noexcept
            {
                const std::uint64_t word = read_u64(bytes);
                double value = 0.0;
                std::memcpy(&value, &word, sizeof(value));
                return value;
            }

            /** @brief Whether a double is finite and strictly positive. */
            inline bool positive_finite(double value) noexcept
            {
                return value > 0.0 && value < 1.0e300;
            }
        } // namespace Detail

        /**
         * @brief A baked planet asset: a value that adopts bytes, or refuses them.
         *
         * Deliberately cannot open a file *during parsing*: `adopt` takes bytes, so the
         * same object serves the editor, a headless probe, and a unit test driving
         * synthesized bytes — which is how the refusals below are tested on a machine with
         * no asset checked out. @ref load_planet_pack is the one convenience that reads a
         * path, and it is a wrapper rather than a second path into the parser.
         */
        class PlanetPack
        {
            public:
                PlanetPack() = default;

                /**
                 * @brief Takes ownership of a blob if it is a pack this reader understands.
                 *
                 * Validates the whole blob before accepting any of it: magic, version, the
                 * tile geometry it was baked for, the provenance length, that the index
                 * fits, that keys ascend strictly (which is what licenses the binary
                 * search), and that every record's payload lies inside the blob and is the
                 * length its codec requires. A pack that fails any of these leaves this
                 * object unloaded rather than partly loaded.
                 *
                 * @param blob Bytes of the asset; moved from on success, untouched on
                 *             failure so the caller may inspect or report them.
                 * @return Whether the pack was accepted.
                 */
                bool adopt(std::vector<std::uint8_t> blob)
                {
                    if (blob.size() < PLANET_PACK_HEADER_BYTES)
                        return false;
                    const std::uint8_t* base = blob.data();
                    if (std::memcmp(base, PLANET_PACK_MAGIC, sizeof(PLANET_PACK_MAGIC)) != 0)
                        return false;
                    if (Detail::read_u32(base + 8) != PLANET_PACK_VERSION)
                        return false;

                    const std::uint32_t body = Detail::read_u32(base + 12);
                    Ellipsoid ellipsoid;
                    ellipsoid.semi_axis_x = Detail::read_f64(base + 16);
                    ellipsoid.semi_axis_y = Detail::read_f64(base + 24);
                    ellipsoid.semi_axis_z = Detail::read_f64(base + 32);
                    if (!Detail::positive_finite(ellipsoid.semi_axis_x) ||
                        !Detail::positive_finite(ellipsoid.semi_axis_y) ||
                        !Detail::positive_finite(ellipsoid.semi_axis_z))
                        return false;

                    // The tile geometry is in the header rather than assumed, because a
                    // pack baked for a different grid would otherwise be read as garbage
                    // of exactly the right length.
                    if (Detail::read_u32(base + 40) != TILE_GRID_SIZE)
                        return false;
                    if (Detail::read_u32(base + 44) != TILE_APRON)
                        return false;

                    const std::uint8_t data_depth = base[48];
                    if (data_depth > MAX_TILE_DEPTH)
                        return false;

                    const std::uint32_t tile_count = Detail::read_u32(base + 52);
                    const std::uint32_t provenance_length = Detail::read_u32(base + 56);
                    if (provenance_length > PLANET_PACK_MAX_PROVENANCE_BYTES)
                        return false;

                    const std::uint64_t index_offset =
                        static_cast<std::uint64_t>(PLANET_PACK_HEADER_BYTES) + provenance_length;
                    const std::uint64_t index_bytes =
                        static_cast<std::uint64_t>(tile_count) * PLANET_PACK_RECORD_BYTES;
                    if (index_offset + index_bytes > blob.size())
                        return false;

                    std::vector<PlanetPackRecord> records;
                    records.reserve(tile_count);
                    std::uint64_t previous_key = 0;
                    for (std::uint32_t entry = 0; entry < tile_count; ++entry)
                    {
                        const std::uint8_t* cursor =
                            base + index_offset + static_cast<std::uint64_t>(entry) *
                                                      PLANET_PACK_RECORD_BYTES;
                        PlanetPackRecord record;
                        record.key = Detail::read_u64(cursor);
                        record.offset = Detail::read_u64(cursor + 8);
                        record.length = Detail::read_u32(cursor + 16);
                        record.codec = Detail::read_u16(cursor + 20);
                        record.quantised_minimum_metres = Detail::read_f32(cursor + 24);
                        record.quantised_maximum_metres = Detail::read_f32(cursor + 28);
                        record.grid_minimum_metres = Detail::read_f32(cursor + 32);
                        record.grid_maximum_metres = Detail::read_f32(cursor + 36);

                        if (entry > 0 && record.key <= previous_key)
                            return false; // the index must ascend for the lookup to be a search
                        previous_key = record.key;

                        if (record.codec != PLANET_PACK_CODEC_QUANTISED)
                            return false;
                        if (record.length != TILE_SAMPLE_COUNT * 2u)
                            return false;
                        if (record.offset < index_offset + index_bytes)
                            return false; // a payload inside the index would alias it
                        if (record.offset + record.length > blob.size())
                            return false;
                        if (record.quantised_maximum_metres < record.quantised_minimum_metres)
                            return false;
                        records.push_back(record);
                    }

                    blob_ = std::move(blob);
                    records_ = std::move(records);
                    ellipsoid_ = ellipsoid;
                    body_id_ = body;
                    height_data_depth_ = data_depth;
                    provenance_.assign(reinterpret_cast<const char*>(blob_.data() +
                                                                     PLANET_PACK_HEADER_BYTES),
                                       provenance_length);
                    loaded_ = true;
                    return true;
                }

                /** @brief Whether a pack was accepted; false leaves every getter at its default. */
                bool loaded() const noexcept { return loaded_; }

                /** @brief The body's reference surface, metres. */
                const Ellipsoid& ellipsoid() const noexcept { return ellipsoid_; }

                /** @brief The ephemeris body ordinal this was baked for. */
                std::uint32_t body_id() const noexcept { return body_id_; }

                /** @brief The deepest quadtree depth this pack has measured data at. */
                std::uint8_t height_data_depth() const noexcept { return height_data_depth_; }

                /** @brief How many tiles the index carries. */
                std::size_t tile_count() const noexcept { return records_.size(); }

                /**
                 * @brief The whole index, ascending by address key.
                 *
                 * The streamer prioritises against it and the editor reports over it, so
                 * both read the index rather than probing it a tile at a time.
                 */
                const std::vector<PlanetPackRecord>& records() const noexcept
                {
                    return records_;
                }

                /** @brief The sources this asset was baked from, verbatim from the bake. */
                const std::string& provenance() const noexcept { return provenance_; }

                /**
                 * @brief The index record for a tile, or nullptr when it is not stored.
                 * @param address The tile to look up.
                 * @return The record, or nullptr.
                 */
                const PlanetPackRecord* find(const TileAddress& address) const noexcept
                {
                    const std::uint64_t key = tile_address_key(address);
                    std::size_t low = 0;
                    std::size_t high = records_.size();
                    while (low < high)
                    {
                        const std::size_t middle = low + (high - low) / 2;
                        if (records_[middle].key < key)
                            low = middle + 1;
                        else
                            high = middle;
                    }
                    if (low < records_.size() && records_[low].key == key)
                        return &records_[low];
                    return nullptr;
                }

                /**
                 * @brief Decodes a stored tile into a caller's buffer.
                 * @param address        The tile; must be one @ref find returns a record for.
                 * @param heights_metres Receives @ref TILE_SAMPLE_COUNT elevations, metres.
                 * @param statistics     Receives the tile's grid range, apron excluded.
                 * @return false when the tile is not stored, in which case nothing is written.
                 */
                bool read_tile(const TileAddress& address, float* heights_metres,
                               TileStatistics& statistics) const
                {
                    const PlanetPackRecord* record = find(address);
                    if (record == nullptr)
                        return false;

                    const std::uint8_t* payload = blob_.data() + record->offset;
                    const double minimum =
                        static_cast<double>(record->quantised_minimum_metres);
                    const double span = static_cast<double>(record->quantised_maximum_metres) -
                                        minimum;
                    const double scale = span / 65535.0;
                    for (std::uint32_t index = 0; index < TILE_SAMPLE_COUNT; ++index)
                    {
                        const std::uint16_t value = Detail::read_u16(payload + index * 2u);
                        heights_metres[index] =
                            static_cast<float>(minimum + static_cast<double>(value) * scale);
                    }
                    statistics.minimum_metres = record->grid_minimum_metres;
                    statistics.maximum_metres = record->grid_maximum_metres;
                    return true;
                }

            private:
                std::vector<std::uint8_t> blob_;
                std::vector<PlanetPackRecord> records_;
                std::string provenance_;
                Ellipsoid ellipsoid_{};
                std::uint32_t body_id_ = 0;
                std::uint8_t height_data_depth_ = 0;
                bool loaded_ = false;
        };

        /**
         * @brief Reads a pack off disk.
         *
         * A missing or malformed file is not an error to abort a load over: a body with no
         * baked terrain falls back to the analytic ground the sky pass already draws, which
         * is what ships today. Ask the returned object's @ref PlanetPack::loaded whether
         * real data was found — a question about which source is in use, which is worth
         * asking, rather than a status code that invites a caller to give up over it.
         *
         * @param path Path to a `.planet` asset.
         * @return The pack, loaded or not.
         */
        inline PlanetPack load_planet_pack(const std::string& path)
        {
            PlanetPack pack;
            std::ifstream file(path, std::ios::binary);
            if (!file)
                return pack;
            std::vector<std::uint8_t> blob((std::istreambuf_iterator<char>(file)),
                                           std::istreambuf_iterator<char>());
            // The return value is dropped on purpose: a refusal leaves the object unloaded,
            // which is exactly what this function would have returned anyway.
            (void)pack.adopt(std::move(blob));
            return pack;
        }

        /**
         * @brief A baked pack, as a height source.
         *
         * Separate from @ref PlanetPack because they answer different questions: the pack
         * is a format, and this is the policy over it — what to do when the tile asked for
         * is deeper than the pack stores. That policy is to resample the nearest stored
         * ancestor, which is what lets the quadtree descend past the data and is where
         * measurement stops and §6.4's synthesis will later begin.
         */
        class PackHeightSource final : public IHeightSource
        {
            public:
                /**
                 * @brief Binds the source to a loaded pack.
                 * @param pack The pack; must outlive this source.
                 */
                explicit PackHeightSource(const PlanetPack& pack) noexcept : pack_(pack) {}

                /**
                 * @brief How deep this pack's measurements go.
                 * @return The pack's data depth, and zero when nothing is loaded — a body
                 *         with no asset reports no measurement rather than pretending to.
                 */
                std::uint8_t data_depth(const TileAddress&) const override
                {
                    return pack_.loaded() ? pack_.height_data_depth() : std::uint8_t(0);
                }

                /**
                 * @brief The elevation band a tile spans, from the index alone.
                 *
                 * An index lookup, not a decode — which is the point: a quadtree cull tests
                 * far more nodes than it draws, and a bounding volume that cost a tile read
                 * would make culling more expensive than the drawing it saves.
                 *
                 * A tile the pack does not store answers from its nearest stored ancestor,
                 * whose band is derived from coarser samples and can therefore miss a peak
                 * the child would have resolved. It is widened by @ref ANCESTOR_BAND_MARGIN
                 * for that reason: a bounding volume is allowed to be conservative and is
                 * not allowed to be tight-but-wrong.
                 *
                 * @param address        The tile being asked about.
                 * @param minimum_metres Receives the band's floor.
                 * @param maximum_metres Receives its ceiling.
                 * @return false only when no stored tile covers the address at all.
                 */
                bool tile_bounds(const TileAddress& address, float& minimum_metres,
                                 float& maximum_metres) const override
                {
                    if (!pack_.loaded())
                        return false;
                    if (const PlanetPackRecord* record = pack_.find(address))
                    {
                        minimum_metres = record->grid_minimum_metres;
                        maximum_metres = record->grid_maximum_metres;
                        return true;
                    }

                    TileAddress ancestor = address;
                    while (ancestor.depth > 0)
                    {
                        ancestor = tile_parent(ancestor);
                        if (const PlanetPackRecord* record = pack_.find(ancestor))
                        {
                            const float band =
                                record->grid_maximum_metres - record->grid_minimum_metres;
                            const float margin = band * ANCESTOR_BAND_MARGIN + 100.0f;
                            minimum_metres = record->grid_minimum_metres - margin;
                            maximum_metres = record->grid_maximum_metres + margin;
                            return true;
                        }
                    }
                    return false;
                }

                /** @brief How far an inherited band is widened, as a fraction of itself. */
                static constexpr float ANCESTOR_BAND_MARGIN = 0.25f;

                /**
                 * @brief Fills a tile from the pack, resampling an ancestor when needed.
                 *
                 * Allocates a scratch tile only on the resampling path, which is the path a
                 * streaming worker takes off the critical frame; the stored-tile path
                 * decodes straight into the caller's buffer.
                 *
                 * @param address        The tile to fill.
                 * @param heights_metres Receives @ref TILE_SAMPLE_COUNT elevations, metres.
                 * @param statistics     Receives the tile's elevation band.
                 * @return false when no stored tile covers the address, including when no
                 *         pack is loaded at all.
                 */
                bool sample_tile(const TileAddress& address, float* heights_metres,
                                 TileStatistics& statistics) const override
                {
                    if (!pack_.loaded() || !tile_address_valid(address))
                        return false;
                    if (pack_.read_tile(address, heights_metres, statistics))
                        return true;

                    // Find the nearest stored ancestor before allocating anything: the walk
                    // is index lookups, and only the one tile that answers is decoded.
                    TileAddress ancestor = address;
                    bool found = false;
                    while (ancestor.depth > 0)
                    {
                        ancestor = tile_parent(ancestor);
                        if (pack_.find(ancestor) != nullptr)
                        {
                            found = true;
                            break;
                        }
                    }
                    if (!found)
                        return false;

                    std::vector<float> source(TILE_SAMPLE_COUNT, 0.0f);
                    TileStatistics ignored;
                    if (!pack_.read_tile(ancestor, source.data(), ignored))
                        return false;
                    resample_from(ancestor, address, source.data(), heights_metres);
                    statistics = tile_statistics(heights_metres);
                    return true;
                }

            private:
                /**
                 * @brief Fills @p target's samples by interpolating within @p source's tile.
                 *
                 * Both tiles are rectangles in the same face's grid coordinates, so the
                 * mapping is a linear rescale — no projection is involved, which is the
                 * reason the quadtree subdivides the *grid* parameter rather than the face
                 * coordinate.
                 */
                static void resample_from(const TileAddress& ancestor, const TileAddress& target,
                                          const float* source, float* destination) noexcept
                {
                    const TileGridRect outer = tile_grid_rect(ancestor);
                    const double outer_span_s = outer.s_maximum - outer.s_minimum;
                    const double outer_span_t = outer.t_maximum - outer.t_minimum;
                    for (std::uint32_t row = 0; row < TILE_STRIDE; ++row)
                    {
                        for (std::uint32_t column = 0; column < TILE_STRIDE; ++column)
                        {
                            double grid_s = 0.0;
                            double grid_t = 0.0;
                            tile_sample_grid_coordinate(target, column, row, grid_s, grid_t);
                            const double alpha = (grid_s - outer.s_minimum) / outer_span_s;
                            const double beta = (grid_t - outer.t_minimum) / outer_span_t;
                            destination[tile_sample_index(column, row)] =
                                sample_tile_bilinear(source, alpha, beta);
                        }
                    }
                }

                const PlanetPack& pack_;
        };
    } // namespace Terrain
} // namespace SushiEngine
