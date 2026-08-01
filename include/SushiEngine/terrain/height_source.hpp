/**************************************************************************/
/* height_source.hpp                                                      */
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
 * @file height_source.hpp
 * @brief Where a tile's measured elevation comes from.
 *
 * The seam that lets a baked pak, a procedurally generated body, and a
 * higher-resolution regional inset be one code path
 * (`docs/slop/solar_system_overhaul.md` §5.1). No consumer of terrain learns which
 * implementation answered, which is what makes a fictional moon in the same scene as a
 * real one cost nothing structurally.
 *
 * @ref IHeightSource::data_depth is the other half of the contract and the more
 * important one: it says how deep the source's *measurement* goes. Past that depth a
 * height is synthesized rather than resampled, and the whole system's honesty about
 * which is which rests on sources reporting it truthfully.
 */

#include <cstdint>

#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /**
         * @brief The elevation band a tile spans, metres above the reference ellipsoid.
         *
         * Carried beside every tile because it is what a bounding volume is made of: a
         * node's extent is its patch swept between these two, which is what lets terrain
         * be frustum- and occlusion-culled exactly rather than conservatively.
         */
        struct TileStatistics
        {
            float minimum_metres = 0.0f;
            float maximum_metres = 0.0f;
        };

        /**
         * @brief A source of measured elevation for a body.
         *
         * Implementations are expected to be cheap to copy-construct nothing and safe to
         * call from any thread: the streamer calls @ref sample_tile from workers, and the
         * headless server calls it with no renderer present at all.
         */
        class IHeightSource
        {
            public:
                virtual ~IHeightSource() = default;

                /**
                 * @brief How deep this source's measurements go at an address.
                 *
                 * A tile below this depth carries no new information — the source can
                 * still fill it, by resampling its parent, but the detail past this depth
                 * is synthesis and is classed accordingly.
                 *
                 * @param address The tile being asked about.
                 * @return The deepest quadtree depth with real data covering it; zero
                 *         when the source has only a single global level, as a body with
                 *         no terrain at all (a gas giant's one-bar surface) reports.
                 */
                virtual std::uint8_t data_depth(const TileAddress& address) const = 0;

                /**
                 * @brief The elevation band a tile spans, without decoding it.
                 *
                 * A capability with a default rather than an obligation: a source that
                 * cannot answer cheaply returns false and the caller falls back to a
                 * coarser bound, which is always safe because a bounding volume may be
                 * conservative. A source that *can* answer — a pack, whose index carries
                 * the band beside every record — turns a node's bounding volume into an
                 * index lookup instead of a tile decode, which is what makes culling the
                 * quadtree affordable.
                 *
                 * @param address The tile being asked about.
                 * @param minimum_metres Receives the lowest elevation in the tile's grid.
                 * @param maximum_metres Receives the highest.
                 * @return Whether the band was known; on false neither output is written.
                 */
                virtual bool tile_bounds(const TileAddress& address, float& minimum_metres,
                                         float& maximum_metres) const
                {
                    (void)address;
                    (void)minimum_metres;
                    (void)maximum_metres;
                    return false;
                }

                /**
                 * @brief Fills a tile's elevations, apron included.
                 *
                 * @param heights_metres Receives @ref TILE_SAMPLE_COUNT elevations in
                 *                       metres above the reference ellipsoid, row-major,
                 *                       indexed by @ref tile_sample_index.
                 * @param address        The tile to fill; must be valid.
                 * @param statistics     Receives the tile's elevation band.
                 * @return true when the tile was filled; false when the address is
                 *         outside this source's coverage, in which case neither output is
                 *         written and a composite source moves on to the next candidate.
                 */
                virtual bool sample_tile(const TileAddress& address, float* heights_metres,
                                         TileStatistics& statistics) const = 0;
        };

        /**
         * @brief Recomputes a tile's elevation band from its samples.
         *
         * The grid only, never the apron: the apron holds the neighbours' data, and a
         * bounding volume that included it would extend past the tile it bounds.
         *
         * @param heights_metres A filled tile of @ref TILE_SAMPLE_COUNT samples.
         * @return The band the tile's own grid spans.
         */
        inline TileStatistics tile_statistics(const float* heights_metres) noexcept
        {
            TileStatistics statistics;
            statistics.minimum_metres = heights_metres[tile_sample_index(TILE_APRON, TILE_APRON)];
            statistics.maximum_metres = statistics.minimum_metres;
            for (std::uint32_t row = TILE_APRON; row < TILE_APRON + TILE_GRID_SIZE; ++row)
            {
                for (std::uint32_t column = TILE_APRON; column < TILE_APRON + TILE_GRID_SIZE;
                     ++column)
                {
                    const float height = heights_metres[tile_sample_index(column, row)];
                    if (height < statistics.minimum_metres)
                        statistics.minimum_metres = height;
                    if (height > statistics.maximum_metres)
                        statistics.maximum_metres = height;
                }
            }
            return statistics;
        }

        /**
         * @brief Bilinearly samples a filled tile at a normalized position within it.
         *
         * Coordinates are the tile's own [0, 1] square, so a caller converts once from
         * grid coordinates and never has to know where the apron sits. Values slightly
         * outside the square are legal and read the apron, which is what it is for; they
         * are clamped to the stored extent so a caller cannot read past the array.
         *
         * @param heights_metres A filled tile of @ref TILE_SAMPLE_COUNT samples.
         * @param alpha          Position along the tile's s axis; 0 is its s minimum.
         * @param beta           Position along the tile's t axis; 0 is its t minimum.
         * @return The interpolated elevation, metres.
         */
        inline float sample_tile_bilinear(const float* heights_metres, double alpha,
                                          double beta) noexcept
        {
            const double cells = static_cast<double>(TILE_GRID_SIZE - 1u);
            const double limit = static_cast<double>(TILE_STRIDE - 1u);
            double column = static_cast<double>(TILE_APRON) + alpha * cells;
            double row = static_cast<double>(TILE_APRON) + beta * cells;
            column = column < 0.0 ? 0.0 : (column > limit ? limit : column);
            row = row < 0.0 ? 0.0 : (row > limit ? limit : row);

            const std::uint32_t column0 = static_cast<std::uint32_t>(column);
            const std::uint32_t row0 = static_cast<std::uint32_t>(row);
            const std::uint32_t column1 =
                column0 + 1u < TILE_STRIDE ? column0 + 1u : column0;
            const std::uint32_t row1 = row0 + 1u < TILE_STRIDE ? row0 + 1u : row0;
            const double fraction_column = column - static_cast<double>(column0);
            const double fraction_row = row - static_cast<double>(row0);

            const double top =
                static_cast<double>(heights_metres[tile_sample_index(column0, row0)]) *
                    (1.0 - fraction_column) +
                static_cast<double>(heights_metres[tile_sample_index(column1, row0)]) *
                    fraction_column;
            const double bottom =
                static_cast<double>(heights_metres[tile_sample_index(column0, row1)]) *
                    (1.0 - fraction_column) +
                static_cast<double>(heights_metres[tile_sample_index(column1, row1)]) *
                    fraction_column;
            return static_cast<float>(top * (1.0 - fraction_row) + bottom * fraction_row);
        }
    } // namespace Terrain
} // namespace SushiEngine
