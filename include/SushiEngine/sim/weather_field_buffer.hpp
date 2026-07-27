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
 * equirectangular lattice and the baked cloud
 * field's flat tile: valid over the few hundred kilometres a march can see, not
 * geodesically exact.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/weather_field.hpp>
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
                    begin_classification(true);

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
                 * @brief Fills the buffer from the GPU nest's readback mirror.
                 *
                 * The path the simulated atmosphere reaches the render tier by since the nest
                 * moved onto the GPU: the mirror is already a lattice of `WeatherColumn`-shaped
                 * records over the nest's own footprint, so this is a transcription rather than
                 * a resample, and everything downstream — the per-column genus the cloudscape
                 * bake resolves, the compiled deck stack, the fog and wetness coupling — keeps
                 * working against real condensate without knowing where it came from.
                 *
                 * @param mirror   The most recently completed readback.
                 * @param observer The scene's geodetic anchor; the field is addressed relative to it.
                 */
                void fill_from_mirror(const Render::AtmosphereMirror& mirror,
                                      const GeodeticPosition& observer)
                {
                    if (!mirror.valid())
                    {
                        fill_uniform(WeatherColumn{}, true);
                        return;
                    }
                    cells_x_ = std::min(int(mirror.cells), Render::WEATHER_FIELD_MAX_CELLS);
                    cells_z_ = cells_x_;
                    samples_.resize(std::size_t(cells_x_) * std::size_t(cells_z_) *
                                    std::size_t(CLOUD_LEVEL_COUNT));
                    begin_classification(true);

                    for (int iz = 0; iz < cells_z_; ++iz)
                        for (int ix = 0; ix < cells_x_; ++ix)
                        {
                            const int sx = cells_x_ == mirror.cells
                                               ? ix
                                               : (ix * mirror.cells) / cells_x_;
                            const int sz = cells_z_ == mirror.cells
                                               ? iz
                                               : (iz * mirror.cells) / cells_z_;
                            store_column(ix, iz,
                                         column_from_mirror(
                                             mirror.columns[std::size_t(sz) *
                                                                std::size_t(mirror.cells) +
                                                            std::size_t(sx)]));
                        }

                    // The span condensate actually occupies, straight from the nest, replacing
                    // the classifier's genus-profile union. The classifier can only say "a
                    // cumulus reaches 3.2 km" because that is what the catalogue says a cumulus
                    // does; the nest knows where this cloud's top *is*. Stretching the march
                    // shell across the real span rather than the catalogue's is what keeps the
                    // baked field's thirty-two vertical texels on the cloud instead of on empty
                    // stratosphere above it.
                    float lowest_base = 0.0f;
                    float highest_top = 0.0f;
                    bool measured = false;
                    for (int i = 0; i < mirror.cells * mirror.cells; ++i)
                    {
                        const Render::AtmosphereMirrorColumn& source = mirror.columns[i];
                        const float top = source.extent[0];
                        if (top <= 0.0f)
                            continue;
                        const float base = source.surface[3];
                        lowest_base = measured ? std::min(lowest_base, base) : base;
                        highest_top = measured ? std::max(highest_top, top) : top;
                        measured = true;
                    }
                    if (measured && highest_top > lowest_base)
                    {
                        // A little headroom below and above: the nest reports the centres of the
                        // levels that hold condensate, and a cloud's own edge sits inside the
                        // level it was found in.
                        union_base_m_ = std::max(lowest_base - MIRROR_SHELL_MARGIN_M, 0.0f);
                        union_top_m_ = highest_top + MIRROR_SHELL_MARGIN_M;
                    }

                    // The mirror already carries the scene-absolute mapping the nest was
                    // centred with, so it is passed straight through rather than rebuilt: two
                    // derivations of the same lattice is two chances to disagree about where
                    // the weather is.
                    (void)observer;
                    uv_scale_x_ = mirror.uv_scale_x;
                    uv_scale_z_ = mirror.uv_scale_z;
                    uv_offset_x_ = mirror.uv_offset_x;
                    uv_offset_z_ = mirror.uv_offset_z;
                    ++revision_;
                }

                /**
                 * @brief Transcribes one mirror record into the bridge's column contract.
                 *
                 * Shared by the field fill above and by `sample_column`, so a point query and
                 * the published field can never report different weather at the same place.
                 *
                 * @param source One coarse column as the GPU wrote it.
                 * @return The same state in `WeatherColumn`'s units.
                 */
                static WeatherColumn column_from_mirror(
                    const Render::AtmosphereMirrorColumn& source) noexcept
                {
                    WeatherColumn column{};
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                    {
                        column.levels[level].coverage =
                            std::clamp(source.bands[level][0], 0.0f, 1.0f);
                        column.levels[level].density_scale =
                            std::clamp(source.bands[level][1], 0.0f, 2.0f);
                        column.levels[level].convective_fraction =
                            std::clamp(source.bands[level][2], 0.0f, 1.0f);
                        column.levels[level].temperature_offset_c = source.bands[level][3];
                    }
                    // The bridge's precipitation is a [0, 1] intensity and the nest reports
                    // millimetres per hour, which is what a rain gauge reads. Ten is heavy
                    // rain, so it is the scale the fraction is stated against.
                    constexpr float RAIN_REFERENCE_MM_PER_HOUR = 10.0f;
                    column.precipitation =
                        std::clamp(source.surface[0] / RAIN_REFERENCE_MM_PER_HOUR, 0.0f, 1.0f);
                    column.wind_u_mps = source.surface[1];
                    column.wind_v_mps = source.surface[2];
                    column.cloud_base_m = source.surface[3];
                    return column;
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
                 * @param column        The column to publish everywhere.
                 * @param derives_genus Whether this column is meteorology the renderer may
                 *                      resolve a genus from (a decoded station report), or a
                 *                      restatement of an authored deck stack it must not
                 *                      overrule (`StaticWeather`). See
                 *                      `Render::WeatherField::derives_genus`; it is a
                 *                      parameter rather than an inference from the fill shape
                 *                      because both kinds of provider publish one column.
                 */
                void fill_uniform(const WeatherColumn& column, bool derives_genus = false)
                {
                    cells_x_ = 1;
                    cells_z_ = 1;
                    samples_.resize(std::size_t(CLOUD_LEVEL_COUNT));
                    begin_classification(derives_genus);
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
                    field.derives_genus = derives_genus_;
                    // A field nothing was classified out of leaves the span collapsed, which
                    // the renderer reads as "no cloud" -- the same answer an empty deck stack
                    // gives, rather than an inverted span it would have to defend against.
                    const bool classified = union_top_m_ > union_base_m_;
                    field.union_base_m = classified ? union_base_m_ : 0.0f;
                    field.union_top_m = classified ? union_top_m_ : 0.0f;
                    return field;
                }

            private:
                // Bounds the plate-carree easting factor so a scene authored at the pole
                // cannot divide the mapping by zero; the projection is already meaningless
                // there, and this keeps it finite rather than pretending otherwise.
                static constexpr double MIN_COS_LATITUDE = 0.05;

                // Headroom around the condensate the nest reports, metres. The readback names
                // the *centres* of the levels that hold cloud, and a cloud's own edge sits
                // inside the level it was found in, so a shell clamped exactly to those centres
                // would clip its own top and bottom.
                static constexpr float MIRROR_SHELL_MARGIN_M = 400.0f;

                // Resets the per-fill classification state. Called at the top of every fill so
                // the union span describes *this* field and not the union of every field ever
                // published into this buffer.
                void begin_classification(bool derives_genus) noexcept
                {
                    derives_genus_ = derives_genus;
                    union_base_m_ = 0.0f;
                    union_top_m_ = 0.0f;
                }

                // Widens the union span by whatever decks @p column would resolve to, through
                // the same `Render::classify_cloud_genus` the bake resolves them with -- the
                // span has to bound exactly the decks that will be rendered, so it cannot
                // afford a second, independently written opinion about which those are.
                void classify_column(const WeatherColumn& column) noexcept
                {
                    if (!derives_genus_)
                        return;
                    const float enable = Render::cloud_genus_thresholds().enable_coverage;
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                    {
                        const WeatherLevelState& state = column.levels[level];
                        if (state.coverage <= enable)
                            continue;
                        widen(Render::classify_cloud_genus(static_cast<Render::CloudBand>(level),
                                                           state.coverage, state.convective_fraction));
                    }
                    const WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
                    if (Render::cloud_band_towers(low.coverage, low.convective_fraction))
                        widen(Render::CloudGenus::Cumulonimbus);
                }

                void widen(Render::CloudGenus genus) noexcept
                {
                    const Render::CloudGenusProfile profile = Render::cloud_genus_profile(genus);
                    if (union_top_m_ <= union_base_m_)
                    {
                        union_base_m_ = profile.base_altitude;
                        union_top_m_ = profile.top_altitude;
                        return;
                    }
                    union_base_m_ = std::min(union_base_m_, profile.base_altitude);
                    union_top_m_ = std::max(union_top_m_, profile.top_altitude);
                }

                void store_column(int ix, int iz, const WeatherColumn& column)
                {
                    classify_column(column);
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
                // The classification the renderer needs and only this side can take: whether
                // genus is the field's to resolve, and the altitude span the resolved decks
                // occupy across every cell (see `Render::WeatherField`'s own docs).
                bool derives_genus_ = false;
                float union_base_m_ = 0.0f;
                float union_top_m_ = 0.0f;
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
