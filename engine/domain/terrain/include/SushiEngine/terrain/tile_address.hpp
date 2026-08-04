/**************************************************************************/
/* tile_address.hpp                                                       */
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
 * @file tile_address.hpp
 * @brief Which patch of a body a tile is, and how its samples are laid out.
 *
 * A planet is six square faces, each subdivided as a quadtree, so a tile is named by
 * a face, a depth, and a cell in that face's grid at that depth
 * (`docs/design/solar_system_overhaul.md` §4.3). Everything here is integer arithmetic
 * on that name: parent, child, same-face neighbour, and a packed key. Where a tile
 * *is* in space belongs to `cube_sphere.hpp`, which is also where the neighbour that
 * crosses a face edge lives — crossing a seam is a geometric fact, not an integer one.
 *
 * The sample layout is here rather than beside the source interface because it is part
 * of what a tile *is*: a grid of @ref TILE_GRID_SIZE vertices per side, plus a
 * one-texel apron of the neighbouring tiles' data so that central differences and
 * bilinear filtering never need a second tile resident.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Terrain
    {
        /**
         * @brief The six cube faces, in the cubemap convention.
         *
         * Ordered the way a cubemap orders its layers, so a baked colour or class tile
         * can be laid out as one without a remapping table, and so the face basis in
         * `cube_sphere.hpp` is the one a reader already knows.
         */
        enum class CubeFace : std::uint8_t
        {
            PositiveX = 0,
            NegativeX,
            PositiveY,
            NegativeY,
            PositiveZ,
            NegativeZ,
        };

        /** @brief Number of cube faces; the bound on a @ref CubeFace cast to an index. */
        constexpr std::uint8_t CUBE_FACE_COUNT = 6;

        /**
         * @brief Deepest addressable quadtree level.
         *
         * Twenty is a 0.075 m render cell on Earth — an order of magnitude below what
         * either the data or the synthesis can justify — and is within the precision
         * budget `cube_sphere.hpp`'s difference form holds to (§9.2). It also keeps a
         * cell index inside twenty bits, which is what lets @ref tile_address_key pack a
         * whole address into a single 64-bit word.
         */
        constexpr std::uint8_t MAX_TILE_DEPTH = 20;

        /** @brief Vertices per side of a tile's height grid. */
        constexpr std::uint32_t TILE_GRID_SIZE = 129;

        /**
         * @brief Rings of neighbouring samples stored outside the grid, per side.
         *
         * One ring is what a central-difference normal and a bilinear tap at the tile's
         * edge need. Storing it is what removes the single largest source of seams in a
         * tiled height field, in the format rather than in the shader.
         */
        constexpr std::uint32_t TILE_APRON = 1;

        /** @brief Samples per side of a stored tile, apron included. */
        constexpr std::uint32_t TILE_STRIDE = TILE_GRID_SIZE + 2u * TILE_APRON;

        /** @brief Samples in a stored tile. */
        constexpr std::uint32_t TILE_SAMPLE_COUNT = TILE_STRIDE * TILE_STRIDE;

        /**
         * @brief Which side of a tile a neighbour lies on.
         *
         * Named for the face-coordinate axis and direction rather than for a compass
         * point, because a face's axes have no fixed orientation on the globe.
         */
        enum class TileEdge : std::uint8_t
        {
            NegativeS = 0,
            PositiveS,
            NegativeT,
            PositiveT,
        };

        /** @brief Number of edges a tile has; the bound on a @ref TileEdge cast. */
        constexpr std::uint8_t TILE_EDGE_COUNT = 4;

        /**
         * @brief A tile's name: a face, a quadtree depth, and a cell at that depth.
         *
         * At @c depth the face carries `2^depth` cells per side, so @c x and @c y are
         * both below that. The address says nothing about whether the tile has data —
         * that is `IHeightSource::data_depth`'s answer.
         */
        struct TileAddress
        {
            CubeFace face = CubeFace::PositiveX;
            std::uint8_t depth = 0;
            std::uint32_t x = 0;
            std::uint32_t y = 0;
        };

        /** @brief Whether two addresses name the same tile. */
        inline bool operator==(const TileAddress& a, const TileAddress& b) noexcept
        {
            return a.face == b.face && a.depth == b.depth && a.x == b.x && a.y == b.y;
        }

        /** @brief Whether two addresses name different tiles. */
        inline bool operator!=(const TileAddress& a, const TileAddress& b) noexcept
        {
            return !(a == b);
        }

        /**
         * @brief Cells per side of one face at a depth.
         * @param depth Quadtree depth, at most @ref MAX_TILE_DEPTH.
         * @return `2^depth`.
         */
        inline std::uint32_t tiles_per_side(std::uint8_t depth) noexcept
        {
            return std::uint32_t(1) << depth;
        }

        /**
         * @brief Whether an address names a tile that can exist.
         * @param address The address to check.
         * @return true when the depth is within @ref MAX_TILE_DEPTH and the cell is on
         *         the face at that depth.
         */
        inline bool tile_address_valid(const TileAddress& address) noexcept
        {
            if (address.depth > MAX_TILE_DEPTH)
                return false;
            if (static_cast<std::uint8_t>(address.face) >= CUBE_FACE_COUNT)
                return false;
            const std::uint32_t side = tiles_per_side(address.depth);
            return address.x < side && address.y < side;
        }

        /**
         * @brief The tile one level up that contains this one.
         * @param address A tile below depth zero.
         * @return Its parent; a root face returns itself, so a walk upward terminates
         *         rather than underflowing.
         */
        inline TileAddress tile_parent(const TileAddress& address) noexcept
        {
            if (address.depth == 0)
                return address;
            return TileAddress{address.face, static_cast<std::uint8_t>(address.depth - 1u),
                               address.x >> 1, address.y >> 1};
        }

        /**
         * @brief One of the four tiles this one subdivides into.
         * @param address  The parent tile, above @ref MAX_TILE_DEPTH's floor.
         * @param quadrant 0..3, with bit 0 selecting the +s half and bit 1 the +t half.
         * @return The child address; the parent itself when it is already at the
         *         deepest addressable level.
         */
        inline TileAddress tile_child(const TileAddress& address,
                                      std::uint32_t quadrant) noexcept
        {
            if (address.depth >= MAX_TILE_DEPTH)
                return address;
            return TileAddress{address.face, static_cast<std::uint8_t>(address.depth + 1u),
                               (address.x << 1) | (quadrant & 1u),
                               (address.y << 1) | ((quadrant >> 1) & 1u)};
        }

        /**
         * @brief A tile's neighbour on the same face.
         *
         * The common case, and the exact one: stepping within a face is a cell index
         * plus or minus one. A step that would leave the face is refused here and
         * answered by `tile_neighbour` in `cube_sphere.hpp`, which needs the projection.
         *
         * @param address The tile to step from.
         * @param edge    Which side to step across.
         * @param out     Receives the neighbour when one exists on this face.
         * @return true when the neighbour is on the same face, false when the step
         *         crosses a face edge (@p out is untouched).
         */
        inline bool tile_neighbour_on_face(const TileAddress& address, TileEdge edge,
                                           TileAddress& out) noexcept
        {
            const std::int64_t side = static_cast<std::int64_t>(tiles_per_side(address.depth));
            std::int64_t x = static_cast<std::int64_t>(address.x);
            std::int64_t y = static_cast<std::int64_t>(address.y);
            switch (edge)
            {
                case TileEdge::NegativeS: x -= 1; break;
                case TileEdge::PositiveS: x += 1; break;
                case TileEdge::NegativeT: y -= 1; break;
                case TileEdge::PositiveT: y += 1; break;
            }
            if (x < 0 || y < 0 || x >= side || y >= side)
                return false;
            out = TileAddress{address.face, address.depth, static_cast<std::uint32_t>(x),
                              static_cast<std::uint32_t>(y)};
            return true;
        }

        /**
         * @brief A whole address packed into one word, for hashing and cache keys.
         *
         * Three bits of face, five of depth, and twenty each of cell index — 48 bits,
         * which is why @ref MAX_TILE_DEPTH is what it is. Distinct addresses give
         * distinct keys, so the key may be compared instead of the address.
         *
         * @param address The address to pack; must be valid.
         * @return The packed key.
         */
        inline std::uint64_t tile_address_key(const TileAddress& address) noexcept
        {
            return (static_cast<std::uint64_t>(address.face) << 45) |
                   (static_cast<std::uint64_t>(address.depth) << 40) |
                   (static_cast<std::uint64_t>(address.x) << 20) |
                   static_cast<std::uint64_t>(address.y);
        }

        /**
         * @brief The half-open grid rectangle a tile covers on its face.
         *
         * In *grid* coordinates — the uniform parameter the quadtree subdivides, before
         * `cube_sphere.hpp`'s tangent warp — which is what makes these bounds exact
         * binary fractions and the subdivision exact.
         */
        struct TileGridRect
        {
            double s_minimum = -1.0;
            double s_maximum = 1.0;
            double t_minimum = -1.0;
            double t_maximum = 1.0;
        };

        /**
         * @brief The grid rectangle a tile covers.
         * @param address The tile.
         * @return Its bounds in grid coordinates, each within [-1, 1].
         */
        inline TileGridRect tile_grid_rect(const TileAddress& address) noexcept
        {
            const double side = static_cast<double>(tiles_per_side(address.depth));
            const double step = 2.0 / side;
            TileGridRect rect;
            rect.s_minimum = -1.0 + static_cast<double>(address.x) * step;
            rect.s_maximum = rect.s_minimum + step;
            rect.t_minimum = -1.0 + static_cast<double>(address.y) * step;
            rect.t_maximum = rect.t_minimum + step;
            return rect;
        }

        /**
         * @brief Where a stored sample sits in a tile's flat array.
         * @param column Sample column, below @ref TILE_STRIDE (apron included).
         * @param row    Sample row, below @ref TILE_STRIDE.
         * @return The index into a @ref TILE_SAMPLE_COUNT array, row-major.
         */
        inline std::uint32_t tile_sample_index(std::uint32_t column, std::uint32_t row) noexcept
        {
            return row * TILE_STRIDE + column;
        }

        /**
         * @brief The grid coordinates of a stored sample.
         *
         * Apron samples fall outside the tile's own rectangle by exactly one cell, which
         * is what makes them the neighbouring tiles' edge samples rather than an
         * extrapolation.
         *
         * @param address The tile.
         * @param column  Sample column, below @ref TILE_STRIDE.
         * @param row     Sample row, below @ref TILE_STRIDE.
         * @param grid_s  Receives the s grid coordinate.
         * @param grid_t  Receives the t grid coordinate.
         */
        inline void tile_sample_grid_coordinate(const TileAddress& address,
                                                std::uint32_t column, std::uint32_t row,
                                                double& grid_s, double& grid_t) noexcept
        {
            const TileGridRect rect = tile_grid_rect(address);
            const double cells = static_cast<double>(TILE_GRID_SIZE - 1u);
            const double alpha = (static_cast<double>(column) - static_cast<double>(TILE_APRON)) / cells;
            const double beta = (static_cast<double>(row) - static_cast<double>(TILE_APRON)) / cells;
            grid_s = rect.s_minimum + alpha * (rect.s_maximum - rect.s_minimum);
            grid_t = rect.t_minimum + beta * (rect.t_maximum - rect.t_minimum);
        }
    } // namespace Terrain
} // namespace SushiEngine
