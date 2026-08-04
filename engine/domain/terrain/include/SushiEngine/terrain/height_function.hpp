/**************************************************************************/
/* height_function.hpp                                                    */
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
 * @file height_function.hpp
 * @brief The authoritative definition of a body's ground.
 *
 * Measured elevation from an @ref IHeightSource, composed with a @ref LayerStack's
 * ordered edits. This is the *definition* the rest of the system answers to
 * (`docs/design/solar_system_overhaul.md` §2 T2): the physics collision patches are
 * evaluated from it, the headless server evaluates it with no renderer present, the
 * atmosphere's grid sampling reads it, and the tile compile shader is a port of it held
 * to a stated tolerance by a conformance test rather than assumed to agree.
 *
 * That is why it is plain host C++ with no graphics header anywhere beneath it, and why
 * it evaluates whole tiles rather than points: a point query would invite the
 * per-column sampling pattern that made the shipped weather non-spatial
 * (`atmosphere_system.md` §1.1), and the bulk form is what both the compile and the
 * collision patch actually want.
 *
 * Sub-Nyquist detail synthesis is deliberately absent. It is a separate concern with a
 * separate determinism class, and adding its seam before it has an implementation would
 * be an interface standing in for a decision that has not been made yet.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/terrain/cube_sphere.hpp>
#include <SushiEngine/terrain/height_source.hpp>
#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/tile_address.hpp>

namespace SushiEngine
{
    namespace Terrain
    {
        /**
         * @brief A body's ground: a measured source, its edits, and its reference surface.
         *
         * A view over the three, not an owner of any: the source and the stack outlive it
         * and are shared with the streamer and the editor respectively.
         */
        class HeightFunction
        {
            public:
                /**
                 * @brief Binds the function to what defines this body's ground.
                 * @param source    Where measured elevation comes from.
                 * @param layers    The ordered edits composed over it.
                 * @param ellipsoid The reference surface elevations are measured from.
                 */
                HeightFunction(const IHeightSource& source, const LayerStack& layers,
                               const Ellipsoid& ellipsoid) noexcept
                    : source_(source), layers_(layers), ellipsoid_(ellipsoid)
                {
                }

                /** @brief The reference surface elevations are measured above. */
                const Ellipsoid& ellipsoid() const noexcept { return ellipsoid_; }

                /**
                 * @brief How deep the measurement goes at an address.
                 * @param address The tile being asked about.
                 * @return The source's answer, unmodified: an edit changes the ground but
                 *         never the resolution the ground was measured at.
                 */
                std::uint8_t data_depth(const TileAddress& address) const
                {
                    return source_.data_depth(address);
                }

                /**
                 * @brief Fills a tile with the authoritative elevation, apron included.
                 *
                 * The measured tile first, then — only when a layer actually reaches the
                 * tile — the stack applied per sample and the statistics recomputed. The
                 * overlap test is what keeps an untouched tile at the cost of the source
                 * read alone, which is the common case by a wide margin.
                 *
                 * @param address        The tile to fill; must be valid.
                 * @param heights_metres Receives @ref TILE_SAMPLE_COUNT elevations,
                 *                       metres above the reference ellipsoid.
                 * @param statistics     Receives the tile's elevation band, after edits.
                 * @return true when the tile was filled; false when the source has no
                 *         coverage there, in which case neither output is written.
                 */
                bool evaluate_tile(const TileAddress& address, float* heights_metres,
                                   TileStatistics& statistics) const
                {
                    if (!source_.sample_tile(address, heights_metres, statistics))
                        return false;
                    if (layers_.size() == 0)
                        return true;

                    const Vector3 centre =
                        tile_sample_direction(address, TILE_STRIDE / 2u, TILE_STRIDE / 2u);
                    if (!layers_.overlaps(centre, tile_angular_radius(address)))
                        return true;

                    for (std::uint32_t row = 0; row < TILE_STRIDE; ++row)
                    {
                        for (std::uint32_t column = 0; column < TILE_STRIDE; ++column)
                        {
                            const std::uint32_t index = tile_sample_index(column, row);
                            const Vector3 direction =
                                tile_sample_direction(address, column, row);
                            const double elevation = layers_.apply(
                                direction, static_cast<double>(heights_metres[index]));
                            heights_metres[index] = static_cast<float>(elevation);
                        }
                    }
                    statistics = tile_statistics(heights_metres);
                    return true;
                }

                /**
                 * @brief The surface point a tile sample sits at, elevation included.
                 *
                 * Displaced along the geodetic normal rather than the radial direction,
                 * which is the datum every elevation model states its heights against.
                 * Double precision: this is the host-side form, and the single-precision
                 * render-side form is @ref ellipsoid_point_delta against a node centre.
                 *
                 * @param address          The tile.
                 * @param column           Sample column, below @ref TILE_STRIDE.
                 * @param row              Sample row, below @ref TILE_STRIDE.
                 * @param elevation_metres The sample's elevation.
                 * @return The point in body-fixed metres.
                 */
                Vector3 sample_position(const TileAddress& address, std::uint32_t column,
                                        std::uint32_t row, double elevation_metres) const
                {
                    const Vector3 direction = tile_sample_direction(address, column, row);
                    const Vector3 surface = ellipsoid_point(ellipsoid_, direction);
                    const Vector3 normal = ellipsoid_normal(ellipsoid_, surface);
                    return Vector3{surface.x + normal.x * elevation_metres,
                                   surface.y + normal.y * elevation_metres,
                                   surface.z + normal.z * elevation_metres};
                }

            private:
                const IHeightSource& source_;
                const LayerStack& layers_;
                Ellipsoid ellipsoid_;
        };
    } // namespace Terrain
} // namespace SushiEngine
