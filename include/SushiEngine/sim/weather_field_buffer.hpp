/**************************************************************************/
/* weather_field_buffer.hpp                                               */
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
 * @file weather_field_buffer.hpp
 * @brief Sim-side storage behind `Render::WeatherField`, and how a provider fills it.
 *
 * `Render::WeatherField` borrows its samples (see that file's docs); this is who owns
 * them. Kept in the sim domain rather than the render one because the producer is the
 * simulation and the lifetime is the simulation's to guarantee — the render struct is a
 * view across the seam, the same split `Render::Cloudscape` and the compiler that fills
 * it already follow.
 *
 * The scene-space mapping is stated once, here, because every caller needs to agree on
 * it: **scene +X is east and scene +Z is north** at the observer's geodetic position.
 * That is not a new convention — `RuntimeSimulation`'s rain emitter already drives its
 * lateral drift from `WindSample::eastward_mps`/`northward_mps` into exactly those scene
 * axes. Beyond that the mapping is a flat tangent plane, matching both the grid's own
 * equirectangular lattice (`regional_weather_grid.hpp`'s file docs) and the baked cloud
 * field's flat tile: valid over the few hundred kilometres a march can see, not
 * geodesically exact.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/render/weather_field.hpp>
#include <SushiEngine/sim/regional_weather_grid.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Owns the samples a `Render::WeatherField` points at, and fills them.
         *
         * One buffer lives for as long as the provider that publishes into it, so the
         * borrowed pointer in `Environment` is always valid while the environment can be
         * read. `revision_` only advances when a fill actually happens, which is what lets
         * the renderer skip re-uploading an unchanged field.
         */
        class WeatherFieldBuffer
        {
            public:
                /**
                 * @brief Fills the buffer from a regional grid, decimating to the render cap.
                 *
                 * Walks the render-side cell lattice and reads the grid cell each one lands
                 * on, so a simulation grid finer than `Render::WEATHER_FIELD_MAX_CELLS` is
                 * point-sampled down rather than overrunning the renderer's fixed texture.
                 * The published field keeps the *simulation's* footprint either way — the
                 * renderer's resolution cap changes how finely that footprint is resolved,
                 * never how much of the world it covers.
                 *
                 * @param grid             The grid to publish.
                 * @param observer         Where the scene stands geodetically (scene origin).
                 * @param interpolation_t  0 at the last grid tick, approaching 1 before the next.
                 */
                void fill_from_grid(const RegionalWeatherGrid& grid, const GeodeticPosition& observer,
                                    double interpolation_t)
                {
                    const int grid_x = grid.cell_count_x();
                    const int grid_z = grid.cell_count_z();
                    cells_x_ = std::min(grid_x, Render::WEATHER_FIELD_MAX_CELLS);
                    cells_z_ = std::min(grid_z, Render::WEATHER_FIELD_MAX_CELLS);
                    samples_.resize(std::size_t(cells_x_) * std::size_t(cells_z_) *
                                    std::size_t(CLOUD_LEVEL_COUNT));

                    for (int iz = 0; iz < cells_z_; ++iz)
                        for (int ix = 0; ix < cells_x_; ++ix)
                        {
                            const int source_x = cells_x_ == grid_x ? ix : (ix * grid_x) / cells_x_;
                            const int source_z = cells_z_ == grid_z ? iz : (iz * grid_z) / cells_z_;
                            const WeatherColumn column =
                                grid.column_at_cell(source_x, source_z, interpolation_t);
                            store_column(ix, iz, column);
                        }

                    // Scene metres -> the grid's absolute tangent lattice -> [0, 1] UV. Eastward
                    // scene metres carry the 1/cos(latitude) factor because the lattice is plate
                    // carree (x = R * longitude), so a metre of easting is more than a metre of
                    // lattice x away from the equator; northing needs no such term.
                    double origin_x = 0.0;
                    double origin_z = 0.0;
                    grid.tangent_origin_m(origin_x, origin_z);
                    const double radius = grid.planet_radius_m();
                    const double cell = grid.cell_size_m();
                    const double span_x = cell * double(grid_x);
                    const double span_z = cell * double(grid_z);
                    const double cos_latitude =
                        std::max(std::cos(observer.latitude_radians), MIN_COS_LATITUDE);

                    uv_scale_x_ = 1.0 / (cos_latitude * span_x);
                    uv_scale_z_ = 1.0 / span_z;
                    uv_offset_x_ = (radius * observer.longitude_radians - origin_x) / span_x;
                    uv_offset_z_ = (radius * observer.latitude_radians - origin_z) / span_z;
                    ++revision_;
                }

                /**
                 * @brief Fills the buffer by sampling a point query over a lattice.
                 *
                 * For a provider whose horizontal structure exists but is not stored as a grid
                 * — `IngestedWeather` blends between station reports, so its field is real but
                 * implicit in its query. Sampling that query onto a lattice is the only way to
                 * hand it to the renderer, and it costs `cells * cells` point queries at the
                 * publish cadence, not per frame.
                 *
                 * @param observer        The scene's geodetic anchor; the lattice is centred here.
                 * @param planet_radius_m The body's mean radius, metres — the tangent projection's scale.
                 * @param span_meters     Lattice span per axis on the tangent lattice, metres.
                 * @param cells           Cells per axis; clamped to `Render::WEATHER_FIELD_MAX_CELLS`.
                 * @param sample          Callable `WeatherColumn(const GeodeticPosition&)`.
                 */
                template <typename SampleFn>
                void fill_from_sampler(const GeodeticPosition& observer, double planet_radius_m,
                                       double span_meters, int cells, SampleFn sample)
                {
                    cells_x_ = std::clamp(cells, 1, Render::WEATHER_FIELD_MAX_CELLS);
                    cells_z_ = cells_x_;
                    samples_.resize(std::size_t(cells_x_) * std::size_t(cells_z_) *
                                    std::size_t(CLOUD_LEVEL_COUNT));

                    const double span = std::max(span_meters, 1.0);
                    const double radius = std::max(planet_radius_m, 1.0);
                    const double origin_x = radius * observer.longitude_radians - span * 0.5;
                    const double origin_z = radius * observer.latitude_radians - span * 0.5;
                    const double step = span / double(cells_x_);

                    for (int iz = 0; iz < cells_z_; ++iz)
                        for (int ix = 0; ix < cells_x_; ++ix)
                        {
                            const GeodeticPosition position{
                                (origin_z + (double(iz) + 0.5) * step) / radius,
                                (origin_x + (double(ix) + 0.5) * step) / radius};
                            store_column(ix, iz, sample(position));
                        }

                    const double cos_latitude =
                        std::max(std::cos(observer.latitude_radians), MIN_COS_LATITUDE);
                    uv_scale_x_ = 1.0 / (cos_latitude * span);
                    uv_scale_z_ = 1.0 / span;
                    uv_offset_x_ = 0.5; // the lattice is centred on the observer by construction.
                    uv_offset_z_ = 0.5;
                    ++revision_;
                }

                /**
                 * @brief Fills the buffer with one column repeated everywhere.
                 *
                 * The honest field form of a provider that genuinely has no horizontal
                 * structure — a manually authored deck stack, or a single decoded station
                 * report. A one-cell field is not a degenerate placeholder here: it says
                 * exactly what such a provider knows, and the renderer's clamped addressing
                 * turns it into the uniform sky it should be.
                 *
                 * @param column The column to publish everywhere.
                 */
                void fill_uniform(const WeatherColumn& column)
                {
                    cells_x_ = 1;
                    cells_z_ = 1;
                    samples_.resize(std::size_t(CLOUD_LEVEL_COUNT));
                    store_column(0, 0, column);
                    // Every lookup lands on the single texel's centre, whatever the world
                    // position — a constant map, not an arbitrary one.
                    uv_scale_x_ = 0.0;
                    uv_scale_z_ = 0.0;
                    uv_offset_x_ = 0.5;
                    uv_offset_z_ = 0.5;
                    ++revision_;
                }

                /**
                 * @brief Records the column the renderer's deck stack was compiled from.
                 *
                 * See `Render::WeatherField::reference_coverage`. Set by whoever compiles the
                 * `Cloudscape`, from the same column it compiled — the two must describe the
                 * same point, or the renderer will scale a bake it was never given.
                 *
                 * @param column The reference column.
                 */
                void set_reference_column(const WeatherColumn& column) noexcept
                {
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                        reference_coverage_[level] = column.levels[level].coverage;
                }

                /**
                 * @brief A borrowed view of the current contents, for `Environment`.
                 * @return The view; invalid (`valid() == false`) until a fill has happened.
                 */
                Render::WeatherField view() const noexcept
                {
                    Render::WeatherField field{};
                    if (samples_.empty())
                        return field;
                    field.samples = samples_.data();
                    field.cells_x = cells_x_;
                    field.cells_z = cells_z_;
                    field.level_count = CLOUD_LEVEL_COUNT;
                    field.revision = revision_;
                    field.uv_scale_x = static_cast<float>(uv_scale_x_);
                    field.uv_scale_z = static_cast<float>(uv_scale_z_);
                    field.uv_offset_x = static_cast<float>(uv_offset_x_);
                    field.uv_offset_z = static_cast<float>(uv_offset_z_);
                    field.level_altitudes[0] = static_cast<float>(CLOUD_LEVEL_LOW_CENTER_METERS);
                    field.level_altitudes[1] = static_cast<float>(CLOUD_LEVEL_MID_CENTER_METERS);
                    field.level_altitudes[2] = static_cast<float>(CLOUD_LEVEL_HIGH_CENTER_METERS);
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                        field.reference_coverage[level] = reference_coverage_[level];
                    return field;
                }

            private:
                // Bounds the plate-carree easting factor so a scene authored at the pole
                // cannot divide the mapping by zero; the projection is already meaningless
                // there, and this keeps it finite rather than pretending otherwise.
                static constexpr double MIN_COS_LATITUDE = 0.05;

                void store_column(int ix, int iz, const WeatherColumn& column)
                {
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                    {
                        const std::size_t index =
                            (std::size_t(level) * std::size_t(cells_z_) + std::size_t(iz)) *
                                std::size_t(cells_x_) + std::size_t(ix);
                        Render::WeatherFieldSample& sample = samples_[index];
                        sample.coverage = column.levels[level].coverage;
                        sample.density_scale = column.levels[level].density_scale;
                        sample.convective_fraction = column.levels[level].convective_fraction;
                        sample.precipitation = column.precipitation;
                    }
                }

                std::vector<Render::WeatherFieldSample> samples_;
                float reference_coverage_[CLOUD_LEVEL_COUNT]{};
                int cells_x_ = 0;
                int cells_z_ = 0;
                double uv_scale_x_ = 0.0;
                double uv_scale_z_ = 0.0;
                double uv_offset_x_ = 0.0;
                double uv_offset_z_ = 0.0;
                std::uint32_t revision_ = 0;
        };
    } // namespace Simulation
} // namespace SushiEngine
