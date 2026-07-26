/**************************************************************************/
/* regional_weather_grid.hpp                                             */
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
 * @file regional_weather_grid.hpp
 * @brief T2: the regional grid — camera-centered meteorology, ticked every 10-30 s.
 *
 * `docs/slop/weather_and_clouds.md` §5.2: a camera-centered grid of `wind(u, v) /
 * temperature / humidity / cloud water / precipitation` columns, advected by T1's
 * wind, lifted orographically, driven by diurnal convection, and closed by a
 * condensation/precipitation/evaporation cycle.
 *
 * **Two scoped-down dimensions, named rather than silent** (see the CHANGELOG entry
 * for the full reasoning):
 * - **3 vertical levels (`CloudLevel::Low/Mid/High`), not the design doc's 8-16.**
 *   `Render::CloudGenus`'s WMO étage bands are themselves only three wide bins
 *   (high/middle/low+vertical); the only consumer of T2's vertical structure is the
 *   deck-genus bridge (`weather_cloudscape_compiler.hpp`), which cannot use a finer
 *   profile than that today (`Render::CloudDeck` has no continuous altitude curve to
 *   feed). A finer T2 column would be internal detail with no render-visible effect.
 * - **A smaller default horizontal grid** (`DEFAULT_CELL_COUNT`) than 256x256, for
 *   tick cost and test/CI runtime; the algorithm is resolution-agnostic (`nx`/`nz`
 *   are constructor parameters), so raising it later is a one-line change once the
 *   cost is worth paying.
 *
 * **Orographic lift is wired but inert by default.** No terrain height field exists
 * anywhere in the engine today (`Render::PlanetParams` and `sky.frag`'s `relief_normal`
 * are confirmed to be a pure analytic-ellipsoid shading trick, not queryable
 * elevation — see the CHANGELOG). `set_terrain_height_source` accepts a real one when
 * it exists; until then the default returns 0 everywhere, so `w ~= u . grad(terrain)`
 * is always 0 and the lift term is a documented no-op, not fabricated data.
 *
 * **Floating-origin rebase, CSM-texel-snap discipline.** The grid is addressed by
 * integer cell indices into an *absolute* lattice (an equirectangular tangent plane
 * anchored at lat 0/lon 0, the same class of curvature-free regional approximation T1
 * already uses — see `synoptic_weather.hpp`'s file docs), exactly the way
 * `shadow_uniforms.cpp`'s cascade snap floors a *world-space* coordinate rather than
 * one already offset by a moving reference point (see the CHANGELOG entry for the
 * quoted technique). `rebase_if_needed` only moves the grid once the camera has
 * drifted a whole cell width from center, and cells scrolled in from off the previous
 * grid are seeded from a deterministic background climatology (never left at zero,
 * never drawn from the RNG) so a rebase never introduces a visible seam or a
 * replay-breaking source of nondeterminism.
 */

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include <SushiEngine/sim/synoptic_weather.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Default horizontal cell count per axis (see the file docs' scope note). */
        constexpr int DEFAULT_REGIONAL_GRID_CELLS = 64;

        /** @brief Default domain span per axis, metres (the design doc's "~1000 km"). */
        constexpr double DEFAULT_REGIONAL_GRID_DOMAIN_M = 1.0e6;

        /**
         * @brief One grid column's tracked meteorology, indexed by @ref CloudLevel.
         *
         * Trivially copyable, like `Loop::RngState` and `PressureSystem` — the whole
         * grid is a flat array of these, so it can be bulk-copied for the rebase and
         * the previous/current double buffer without a per-field loop.
         */
        struct WeatherCell
        {
            float humidity[CLOUD_LEVEL_COUNT]{};            /**< Relative humidity, roughly [0, 1.2] (allows brief supersaturation). */
            float temperature_offset_c[CLOUD_LEVEL_COUNT]{}; /**< Deviation from a neutral baseline, Celsius. */
            float cloud_water[CLOUD_LEVEL_COUNT]{};          /**< Condensed water proxy, arbitrary units, >= 0. */
            float convective_fraction[CLOUD_LEVEL_COUNT]{};  /**< 0 stratiform -> 1 convective, [0, 1]. */
            float wind_u_mps[CLOUD_LEVEL_COUNT]{};           /**< Eastward wind sampled from T1 this tick. */
            float wind_v_mps[CLOUD_LEVEL_COUNT]{};           /**< Northward wind sampled from T1 this tick. */
            float precipitation = 0.0f;                      /**< Column surface precipitation rate, [0, 1]. */
        };

        /**
         * @brief T2: a camera-centered grid of columns, advected/forced by T1 each tick.
         *
         * `tick()` performs exactly one full step of the design doc's four-part recipe
         * (semi-Lagrangian advection, orographic lift, diurnal convection, moisture
         * closure) for `dt_seconds` — the caller decides the 10-30 s cadence (see
         * `ProceduralWeather`, which ticks this on its own nested
         * `Loop::FixedTimestepClock`); this class is agnostic of that policy and simply
         * advances by whatever `dt_seconds` it is given, which is what keeps it directly
         * unit-testable at any step size.
         */
        class RegionalWeatherGrid
        {
            public:
                /**
                 * @brief Allocates an empty, unseeded grid.
                 * @param nx              Horizontal cell count, X axis (> 0).
                 * @param nz              Horizontal cell count, Z axis (> 0).
                 * @param domain_size_m   Span of the grid along each axis, metres.
                 * @param planet_radius_m The dominant body's mean radius, metres (for the tangent-plane projection).
                 */
                RegionalWeatherGrid(int nx, int nz, double domain_size_m, double planet_radius_m)
                    : nx_(std::max(nx, 1)), nz_(std::max(nz, 1)),
                      cell_size_m_(domain_size_m / double(std::max(nx, 1))),
                      planet_radius_m_(planet_radius_m),
                      cells_(std::size_t(nx_) * std::size_t(nz_)),
                      previous_cells_(std::size_t(nx_) * std::size_t(nz_))
                {
                }

                /**
                 * @brief Supplies a real terrain-height sampler for orographic lift (see file docs).
                 * @param source A pure function of geodetic position to metres above the reference ellipsoid.
                 */
                void set_terrain_height_source(std::function<double(const GeodeticPosition&)> source)
                {
                    terrain_height_fn_ = std::move(source);
                }

                /**
                 * @brief Centers the grid on @p center and fills it from the background climatology.
                 * @param center   Where to center the grid, geodetic.
                 * @param synoptic T1, for the initial wind field.
                 */
                void seed(const GeodeticPosition& center, const SynopticLayer& synoptic)
                {
                    const TangentPoint t = to_tangent(center);
                    origin_cell_x_ = static_cast<long long>(std::floor(t.x / cell_size_m_)) - nx_ / 2;
                    origin_cell_z_ = static_cast<long long>(std::floor(t.z / cell_size_m_)) - nz_ / 2;
                    for (int iz = 0; iz < nz_; ++iz)
                        for (int ix = 0; ix < nx_; ++ix)
                        {
                            const WeatherCell filled =
                                background_cell(geodetic_at(origin_cell_x_, origin_cell_z_, ix, iz), synoptic);
                            cells_[std::size_t(iz) * std::size_t(nx_) + std::size_t(ix)] = filled;
                            previous_cells_[std::size_t(iz) * std::size_t(nx_) + std::size_t(ix)] = filled;
                        }
                }

                /**
                 * @brief Advances the grid by one full tick: rebase, then advect/lift/convect/close.
                 * @param dt_seconds  This tick's real duration (the 10-30 s cadence, not the base fixed step).
                 * @param synoptic    T1, sampled for wind, fronts, and the pressure field.
                 * @param observer    Where the grid should be centered (the camera/sky observer position).
                 * @param julian_date Epoch, for diurnal insolation.
                 */
                void tick(double dt_seconds, const SynopticLayer& synoptic, const GeodeticPosition& observer,
                          double julian_date)
                {
                    rebase_if_needed(observer, synoptic);
                    previous_cells_ = cells_; // the pre-tick state: this tick's advection source and interpolation floor.
                    tick_grid(dt_seconds, synoptic, julian_date);
                }

                /**
                 * @brief The layered column at @p position, blended between the last two ticks.
                 *
                 * Bilinearly interpolated in space (the four nearest cell centers) and
                 * linearly in time (`interpolation_t`, from the caller's tick clock), so a
                 * consumer sampling every render frame sees continuous motion even though
                 * the grid itself only advances every 10-30 s — "interpolated in time
                 * between ticks so nothing steps visibly" (design doc §5.2).
                 *
                 * @param position        Query point, geodetic.
                 * @param interpolation_t 0 at the moment of the last tick, approaching 1 just before the next.
                 * @return The blended layered-column state.
                 */
                WeatherColumn sample_column(const GeodeticPosition& position, double interpolation_t) const
                {
                    interpolation_t = std::clamp(interpolation_t, 0.0, 1.0);
                    const TangentPoint t = to_tangent(position);
                    const double gx = t.x / cell_size_m_ - double(origin_cell_x_);
                    const double gz = t.z / cell_size_m_ - double(origin_cell_z_);

                    WeatherColumn column{};
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                    {
                        const float cloud_water = blend(gx, gz, interpolation_t,
                            [level](const WeatherCell& c) { return c.cloud_water[level]; });
                        const float convective = blend(gx, gz, interpolation_t,
                            [level](const WeatherCell& c) { return c.convective_fraction[level]; });
                        // W6: surface the temperature offset T2 has tracked internally since W4
                        // (see WeatherCell::temperature_offset_c) -- the bridge simply never had
                        // a consumer for it before weather_flight_hazards.hpp's icing_risk().
                        const float temperature = blend(gx, gz, interpolation_t,
                            [level](const WeatherCell& c) { return c.temperature_offset_c[level]; });
                        column.levels[level] = level_state(cloud_water, convective, temperature);
                    }
                    column.precipitation = clamp01(blend(gx, gz, interpolation_t,
                        [](const WeatherCell& c) { return c.precipitation; }));
                    column.wind_u_mps = blend(gx, gz, interpolation_t,
                        [](const WeatherCell& c) { return c.wind_u_mps[0]; });
                    column.wind_v_mps = blend(gx, gz, interpolation_t,
                        [](const WeatherCell& c) { return c.wind_v_mps[0]; });
                    return column;
                }

                /**
                 * @brief The layered column at one cell centre, blended between the last two ticks.
                 *
                 * What @ref sample_column computes, without the bilinear gather: a cell centre
                 * lands exactly on a sample point, so the four-tap interpolation would return
                 * that cell's own value anyway. This is the form the spatial-field publisher
                 * wants (`Render::WeatherField`) — it walks every cell once, and paying a
                 * four-tap gather per cell to reproduce the cell it is standing on would be
                 * pure waste. Temporal interpolation is kept, because that is what stops the
                 * published field from stepping visibly on the 10-30 s tick boundary.
                 *
                 * @param ix              Cell index, X axis; clamped into range.
                 * @param iz              Cell index, Z axis; clamped into range.
                 * @param interpolation_t 0 at the moment of the last tick, approaching 1 just before the next.
                 * @return The blended layered-column state at that cell.
                 */
                WeatherColumn column_at_cell(int ix, int iz, double interpolation_t) const
                {
                    interpolation_t = std::clamp(interpolation_t, 0.0, 1.0);
                    const std::size_t index = std::size_t(std::clamp(iz, 0, nz_ - 1)) * std::size_t(nx_) +
                                              std::size_t(std::clamp(ix, 0, nx_ - 1));
                    const WeatherCell& previous = previous_cells_[index];
                    const WeatherCell& current = cells_[index];
                    const auto mix = [interpolation_t](float a, float b)
                    {
                        return static_cast<float>(a + (b - a) * interpolation_t);
                    };

                    WeatherColumn column{};
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                        column.levels[level] =
                            level_state(mix(previous.cloud_water[level], current.cloud_water[level]),
                                        mix(previous.convective_fraction[level], current.convective_fraction[level]),
                                        mix(previous.temperature_offset_c[level], current.temperature_offset_c[level]));
                    column.precipitation = clamp01(mix(previous.precipitation, current.precipitation));
                    column.wind_u_mps = mix(previous.wind_u_mps[0], current.wind_u_mps[0]);
                    column.wind_v_mps = mix(previous.wind_v_mps[0], current.wind_v_mps[0]);
                    return column;
                }

                /**
                 * @brief The grid's lower-corner position on the absolute tangent lattice, metres.
                 *
                 * The equirectangular tangent plane this class addresses itself in (see the
                 * file docs): `x = R * longitude`, `z = R * latitude`, anchored at lat 0 / lon 0
                 * and independent of any moving reference point. A consumer that needs to map
                 * its own coordinates onto the grid — the spatial-field publisher does — needs
                 * exactly this corner plus @ref cell_size_m.
                 *
                 * @param x Receives the X coordinate of the corner of cell (0, 0), metres.
                 * @param z Receives the Z coordinate of that corner, metres.
                 */
                void tangent_origin_m(double& x, double& z) const noexcept
                {
                    x = double(origin_cell_x_) * cell_size_m_;
                    z = double(origin_cell_z_) * cell_size_m_;
                }

                /** @brief The body radius the tangent projection was built against, metres. */
                double planet_radius_m() const noexcept { return planet_radius_m_; }

                /** @brief Horizontal cell count, X axis — for the editor debug view/serialization. */
                int cell_count_x() const noexcept { return nx_; }
                /** @brief Horizontal cell count, Z axis. */
                int cell_count_z() const noexcept { return nz_; }
                /** @brief Cell width/depth, metres. */
                double cell_size_m() const noexcept { return cell_size_m_; }
                /** @brief The geodetic center of cell (@p ix, @p iz), current origin. */
                GeodeticPosition cell_center(int ix, int iz) const
                {
                    return geodetic_at(origin_cell_x_, origin_cell_z_, ix, iz);
                }
                /** @brief The most recently ticked state of cell (@p ix, @p iz). */
                const WeatherCell& cell(int ix, int iz) const
                {
                    return cells_[std::size_t(iz) * std::size_t(nx_) + std::size_t(ix)];
                }

            private:
                struct TangentPoint { double x; double z; };

                static float clamp01(float v) noexcept { return std::clamp(v, 0.0f, 1.0f); }

                // The one place internal cell state becomes the bridge's per-band contract,
                // shared by the point query and the field publisher so the two can never
                // drift into disagreeing about what a given condensate load looks like.
                static WeatherLevelState level_state(float cloud_water, float convective_fraction,
                                                     float temperature_offset_c) noexcept
                {
                    WeatherLevelState state{};
                    state.coverage = clamp01(cloud_water / CLOUD_WATER_COVERAGE_SCALE);
                    state.density_scale = std::clamp(0.3f + cloud_water / CLOUD_WATER_DENSITY_SCALE, 0.0f, 2.0f);
                    state.convective_fraction = clamp01(convective_fraction);
                    state.temperature_offset_c = temperature_offset_c;
                    return state;
                }

                // Equirectangular tangent plane anchored at (lat 0, lon 0) — an absolute,
                // origin-independent coordinate every rebase snaps against (see file docs).
                TangentPoint to_tangent(const GeodeticPosition& p) const noexcept
                {
                    return TangentPoint{planet_radius_m_ * p.longitude_radians,
                                         planet_radius_m_ * p.latitude_radians};
                }

                GeodeticPosition from_tangent(const TangentPoint& t) const noexcept
                {
                    return GeodeticPosition{t.z / planet_radius_m_, t.x / planet_radius_m_};
                }

                GeodeticPosition geodetic_at(long long origin_x, long long origin_z, int ix, int iz) const noexcept
                {
                    const TangentPoint t{(double(origin_x) + ix + 0.5) * cell_size_m_,
                                          (double(origin_z) + iz + 0.5) * cell_size_m_};
                    return from_tangent(t);
                }

                template <typename FieldFn>
                float sample_bilinear(const std::vector<WeatherCell>& cells, double gx, double gz,
                                      FieldFn field) const
                {
                    const double fx = gx - 0.5;
                    const double fz = gz - 0.5;
                    int ix0 = static_cast<int>(std::floor(fx));
                    int iz0 = static_cast<int>(std::floor(fz));
                    const double tx = fx - double(ix0);
                    const double tz = fz - double(iz0);
                    int ix1 = ix0 + 1;
                    int iz1 = iz0 + 1;
                    ix0 = std::clamp(ix0, 0, nx_ - 1);
                    ix1 = std::clamp(ix1, 0, nx_ - 1);
                    iz0 = std::clamp(iz0, 0, nz_ - 1);
                    iz1 = std::clamp(iz1, 0, nz_ - 1);
                    const float v00 = field(cells[std::size_t(iz0) * std::size_t(nx_) + std::size_t(ix0)]);
                    const float v10 = field(cells[std::size_t(iz0) * std::size_t(nx_) + std::size_t(ix1)]);
                    const float v01 = field(cells[std::size_t(iz1) * std::size_t(nx_) + std::size_t(ix0)]);
                    const float v11 = field(cells[std::size_t(iz1) * std::size_t(nx_) + std::size_t(ix1)]);
                    const double vx0 = v00 * (1.0 - tx) + v10 * tx;
                    const double vx1 = v01 * (1.0 - tx) + v11 * tx;
                    return static_cast<float>(vx0 * (1.0 - tz) + vx1 * tz);
                }

                template <typename FieldFn>
                float blend(double gx, double gz, double interpolation_t, FieldFn field) const
                {
                    const float previous = sample_bilinear(previous_cells_, gx, gz, field);
                    const float current = sample_bilinear(cells_, gx, gz, field);
                    return static_cast<float>(previous + (current - previous) * interpolation_t);
                }

                static double background_humidity(const GeodeticPosition& position) noexcept
                {
                    // A crude subtropical-high dry belt around 30 deg, wetter elsewhere; a
                    // relaxation target only, not a claim of real climatology.
                    const double lat_deg = std::fabs(position.latitude_radians) * 180.0 / 3.14159265358979323846;
                    const double dip = std::exp(-std::pow((lat_deg - 30.0) / 10.0, 2.0));
                    return 0.65 - 0.25 * dip;
                }

                static double solar_elevation_fraction(const GeodeticPosition& position, double julian_date) noexcept
                {
                    // A standalone, self-contained solar-position estimate (declination from a
                    // sinusoidal day-of-year model, hour angle from longitude + fractional day) —
                    // not routed through Astro::Ephemeris, to keep this header free of the astro
                    // dependency; sufficient for a CAPE proxy, not meant to match the rendered sun.
                    constexpr double PI = 3.14159265358979323846;
                    constexpr double OBLIQUITY_RADIANS = 0.4090928; // matches Astro::OBLIQUITY_J2000_RADIANS.
                    const double day_of_year = std::fmod(julian_date - 1721425.5, 365.25);
                    const double declination = OBLIQUITY_RADIANS * std::sin(2.0 * PI * (day_of_year - 81.0) / 365.25);
                    double local_day_fraction = julian_date + 0.5 + position.longitude_radians / (2.0 * PI);
                    local_day_fraction -= std::floor(local_day_fraction);
                    const double hour_angle = 2.0 * PI * (local_day_fraction - 0.5);
                    const double cos_zenith = std::sin(position.latitude_radians) * std::sin(declination) +
                                               std::cos(position.latitude_radians) * std::cos(declination) *
                                                   std::cos(hour_angle);
                    return std::max(0.0, cos_zenith);
                }

                double orographic_lift(const GeodeticPosition& position, const WindSample& wind) const
                {
                    constexpr double EPS_RADIANS = 0.0005;
                    const GeodeticPosition east{position.latitude_radians, position.longitude_radians + EPS_RADIANS};
                    const GeodeticPosition west{position.latitude_radians, position.longitude_radians - EPS_RADIANS};
                    const GeodeticPosition north{position.latitude_radians + EPS_RADIANS, position.longitude_radians};
                    const GeodeticPosition south{position.latitude_radians - EPS_RADIANS, position.longitude_radians};
                    const double cos_lat = std::max(std::cos(position.latitude_radians), 0.05);
                    const double dz_dx = (terrain_height_fn_(east) - terrain_height_fn_(west)) /
                                          (2.0 * EPS_RADIANS * planet_radius_m_ * cos_lat);
                    const double dz_dy = (terrain_height_fn_(north) - terrain_height_fn_(south)) /
                                          (2.0 * EPS_RADIANS * planet_radius_m_);
                    return wind.eastward_mps * dz_dx + wind.northward_mps * dz_dy;
                }

                WeatherCell background_cell(const GeodeticPosition& position, const SynopticLayer& synoptic) const
                {
                    WeatherCell cell{};
                    const double base_rh = background_humidity(position);
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                    {
                        const double level_fraction = double(level) / double(CLOUD_LEVEL_COUNT - 1);
                        const WindSample wind = synoptic.wind_at(position, level_fraction);
                        cell.wind_u_mps[level] = static_cast<float>(wind.eastward_mps);
                        cell.wind_v_mps[level] = static_cast<float>(wind.northward_mps);
                        cell.humidity[level] = static_cast<float>(std::clamp(base_rh - level * 0.12, 0.0, 1.0));
                    }
                    return cell;
                }

                void rebase_if_needed(const GeodeticPosition& observer, const SynopticLayer& synoptic)
                {
                    const TangentPoint camera = to_tangent(observer);
                    const auto desired_x = static_cast<long long>(std::floor(camera.x / cell_size_m_)) - nx_ / 2;
                    const auto desired_z = static_cast<long long>(std::floor(camera.z / cell_size_m_)) - nz_ / 2;
                    if (desired_x == origin_cell_x_ && desired_z == origin_cell_z_)
                        return;

                    const long long shift_x = desired_x - origin_cell_x_;
                    const long long shift_z = desired_z - origin_cell_z_;
                    std::vector<WeatherCell> rebased(std::size_t(nx_) * std::size_t(nz_));
                    for (int iz = 0; iz < nz_; ++iz)
                        for (int ix = 0; ix < nx_; ++ix)
                        {
                            const long long old_ix = ix + shift_x;
                            const long long old_iz = iz + shift_z;
                            WeatherCell filled;
                            if (old_ix >= 0 && old_ix < nx_ && old_iz >= 0 && old_iz < nz_)
                                filled = cells_[std::size_t(old_iz) * std::size_t(nx_) + std::size_t(old_ix)];
                            else
                                filled = background_cell(geodetic_at(desired_x, desired_z, ix, iz), synoptic);
                            rebased[std::size_t(iz) * std::size_t(nx_) + std::size_t(ix)] = filled;
                        }
                    cells_ = rebased;
                    previous_cells_ = rebased; // no stale-interpolation snap-back the tick after a rebase.
                    origin_cell_x_ = desired_x;
                    origin_cell_z_ = desired_z;
                }

                void tick_grid(double dt_seconds, const SynopticLayer& synoptic, double julian_date)
                {
                    constexpr double RH_CONDENSE_THRESHOLD = 0.85;
                    constexpr double CONDENSE_RATE_PER_SECOND = 0.05;
                    constexpr double EVAP_RATE_PER_SECOND = 0.01;
                    constexpr double PRECIP_THRESHOLD = 0.5;
                    constexpr double PRECIP_RATE_PER_SECOND = 0.02;
                    constexpr double RELAX_RATE_PER_SECOND = 0.002;
                    constexpr double LIFT_HUMIDITY_COEFF = 0.15;
                    constexpr double WARM_FRONT_HUMIDITY_PER_SECOND = 0.01;
                    constexpr double COLD_FRONT_HUMIDITY_PER_SECOND = 0.015;
                    constexpr double WARM_FRONT_TEMP_C_PER_SECOND = 0.0006;
                    constexpr double COLD_FRONT_TEMP_C_PER_SECOND = -0.001;
                    constexpr double CONVECTIVE_CLOUD_GROWTH_PER_SECOND = 0.4;

                    for (int iz = 0; iz < nz_; ++iz)
                        for (int ix = 0; ix < nx_; ++ix)
                        {
                            const GeodeticPosition position = geodetic_at(origin_cell_x_, origin_cell_z_, ix, iz);
                            const FrontProximity front = synoptic.front_proximity(position);
                            const double insolation = solar_elevation_fraction(position, julian_date);
                            const double background_rh = background_humidity(position);

                            WeatherCell out{};
                            double precipitation_column = 0.0;
                            for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                            {
                                const double level_fraction = double(level) / double(CLOUD_LEVEL_COUNT - 1);
                                const WindSample wind = synoptic.wind_at(position, level_fraction);
                                out.wind_u_mps[level] = static_cast<float>(wind.eastward_mps);
                                out.wind_v_mps[level] = static_cast<float>(wind.northward_mps);

                                const TangentPoint here = to_tangent(position);
                                const TangentPoint departure{here.x - wind.eastward_mps * dt_seconds,
                                                              here.z - wind.northward_mps * dt_seconds};
                                const double gx = departure.x / cell_size_m_ - double(origin_cell_x_);
                                const double gz = departure.z / cell_size_m_ - double(origin_cell_z_);

                                double humidity = sample_bilinear(previous_cells_, gx, gz,
                                    [level](const WeatherCell& c) { return c.humidity[level]; });
                                double temperature = sample_bilinear(previous_cells_, gx, gz,
                                    [level](const WeatherCell& c) { return c.temperature_offset_c[level]; });
                                double cloud_water = sample_bilinear(previous_cells_, gx, gz,
                                    [level](const WeatherCell& c) { return c.cloud_water[level]; });

                                if (level == 0) // orographic lift only meaningfully forces the boundary layer.
                                    humidity += LIFT_HUMIDITY_COEFF *
                                                std::clamp(orographic_lift(position, wind), -0.05, 0.05) * dt_seconds;

                                humidity += front.warm * WARM_FRONT_HUMIDITY_PER_SECOND * dt_seconds;
                                humidity += front.cold * COLD_FRONT_HUMIDITY_PER_SECOND * dt_seconds;
                                temperature += front.warm * WARM_FRONT_TEMP_C_PER_SECOND * dt_seconds;
                                temperature += front.cold * COLD_FRONT_TEMP_C_PER_SECOND * dt_seconds;
                                humidity += (background_rh - humidity) *
                                            std::clamp(RELAX_RATE_PER_SECOND * dt_seconds, 0.0, 1.0);
                                temperature += (0.0 - temperature) * std::clamp(RELAX_RATE_PER_SECOND * dt_seconds, 0.0, 1.0);

                                double convective_fraction = 0.0;
                                if (level == 0)
                                {
                                    const double cape = std::max(0.0, insolation * humidity - 0.3);
                                    cloud_water += cape * CONVECTIVE_CLOUD_GROWTH_PER_SECOND * dt_seconds;
                                    convective_fraction = std::clamp(cape * 2.0 + front.cold * 0.6, 0.0, 1.0);
                                }
                                else if (level == 1)
                                {
                                    const double cape = std::max(0.0, insolation * humidity - 0.3);
                                    convective_fraction =
                                        std::clamp(cape * 0.5 + double(front.cold) * 0.4 - double(front.warm) * 0.3, 0.0, 1.0);
                                }
                                else
                                {
                                    convective_fraction = std::clamp(double(front.cold) * 0.2, 0.0, 1.0);
                                }

                                if (humidity > RH_CONDENSE_THRESHOLD)
                                {
                                    const double excess = humidity - RH_CONDENSE_THRESHOLD;
                                    const double condensed =
                                        excess * std::clamp(CONDENSE_RATE_PER_SECOND * dt_seconds, 0.0, 1.0);
                                    humidity -= condensed;
                                    cloud_water += condensed * 8.0; // condensate scale: humidity is a [0,1] fraction, cloud_water an arbitrary optical-thickness proxy.
                                }
                                else if (cloud_water > 0.0)
                                {
                                    const double evaporated =
                                        cloud_water * std::clamp(EVAP_RATE_PER_SECOND * dt_seconds, 0.0, 1.0);
                                    cloud_water -= evaporated;
                                    humidity += evaporated * 0.05;
                                }

                                double level_precipitation = 0.0;
                                if (cloud_water > PRECIP_THRESHOLD)
                                {
                                    const double rained =
                                        (cloud_water - PRECIP_THRESHOLD) *
                                        std::clamp(PRECIP_RATE_PER_SECOND * dt_seconds, 0.0, 1.0);
                                    cloud_water -= rained;
                                    level_precipitation = clamp01(static_cast<float>(rained / (PRECIP_THRESHOLD * 0.5)));
                                }
                                precipitation_column = std::max(precipitation_column, level_precipitation);

                                out.humidity[level] = static_cast<float>(std::clamp(humidity, 0.0, 1.3));
                                out.temperature_offset_c[level] = static_cast<float>(temperature);
                                out.cloud_water[level] = static_cast<float>(std::max(0.0, cloud_water));
                                out.convective_fraction[level] = static_cast<float>(convective_fraction);
                            }
                            out.precipitation = clamp01(static_cast<float>(precipitation_column));
                            cells_[std::size_t(iz) * std::size_t(nx_) + std::size_t(ix)] = out;
                        }
                }

                static constexpr float CLOUD_WATER_COVERAGE_SCALE = 1.2f;
                static constexpr float CLOUD_WATER_DENSITY_SCALE = 2.5f;

                int nx_;
                int nz_;
                double cell_size_m_;
                double planet_radius_m_;
                long long origin_cell_x_ = 0;
                long long origin_cell_z_ = 0;
                std::vector<WeatherCell> cells_;
                std::vector<WeatherCell> previous_cells_;
                std::function<double(const GeodeticPosition&)> terrain_height_fn_ =
                    [](const GeodeticPosition&) { return 0.0; };
        };
    } // namespace Simulation
} // namespace SushiEngine
