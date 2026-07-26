/**************************************************************************/
/* weather_provider.hpp                                                  */
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
 * @file weather_provider.hpp
 * @brief The `IWeatherProvider` seam and its implementations, plus the authoring capability.
 *
 * `IWeatherProvider` names one thing — the weather, as a point query and as a field — and
 * nothing about how it got there. `ProceduralWeather` wraps the synoptic layer and the
 * regional grid behind it; `StaticWeather` wraps a fixed, author-set `Render::Cloudscape`
 * (manual authoring as a legitimate, substitutable provider rather than a special case the
 * host branches on); `IngestedWeather` (`sim/ingested_weather.hpp`) fills the same contract
 * from METAR-sourced data.
 *
 * Substitutability here used to be nominal: the interface existed, but the host stored the
 * concrete `ProceduralWeather` and the abstract simulation interface returned it, so no other
 * implementation could actually be installed — see `docs/slop/atmosphere_system.md` §1.6, and
 * the CHANGELOG entry that admitted `IngestedWeather` was written, tested, and unreachable.
 * The two additions that close it are @ref IWeatherProvider::tick (the host can advance time
 * without knowing what it installed) and @ref IWeatherAuthoring (the editor reaches authoring
 * through a capability, not through a concrete type).
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/regional_weather_grid.hpp>
#include <SushiEngine/sim/weather_field_buffer.hpp>
#include <SushiEngine/sim/synoptic_weather.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief The seam the renderer's cloud bridge consumes: "the weather at a point".
         *
         * ISP-narrow on purpose (design doc §9): a consumer that only needs to compile a
         * `Render::Cloudscape` (see `WeatherCloudscapeCompiler`) never sees a synoptic
         * system, a grid cell, or an RNG — only this.
         */
        class IWeatherProvider
        {
            public:
                virtual ~IWeatherProvider() = default;

                /**
                 * @brief The layered-column meteorology at @p position.
                 * @param position Query point, geodetic.
                 * @return The coverage/density/type-mix per level, plus surface wind/precipitation.
                 */
                virtual WeatherColumn sample_column(const GeodeticPosition& position) const = 0;

                /**
                 * @brief Publishes this provider's horizontal structure into @p out.
                 *
                 * The field half of the same contract, and the reason a front can be seen as
                 * a front (`docs/slop/atmosphere_system.md` §1.1): a renderer handed only
                 * `sample_column` can do nothing but apply one column to the whole sky, which
                 * is precisely the defect that made every earlier phase's meteorology
                 * invisible. Every provider can answer this honestly — one with no horizontal
                 * structure publishes a uniform field, which is a true statement about it and
                 * not a stub — so it belongs on the interface rather than behind a capability
                 * query.
                 *
                 * @param observer The scene's geodetic anchor; the field is addressed relative to it.
                 * @param out      Caller-owned storage to fill. The caller also owns the lifetime
                 *                 the borrowed `Render::WeatherField` depends on.
                 */
                virtual void publish_field(const GeodeticPosition& observer,
                                           WeatherFieldBuffer& out) const = 0;

                /**
                 * @brief Advances whatever internal state this provider evolves.
                 *
                 * On the interface because the host must be able to drive time forward without
                 * knowing which provider it installed — the concrete-type dependency that used
                 * to sit here is exactly what made `IngestedWeather` uninstallable despite being
                 * written and tested. A provider with nothing to advance implements this as a
                 * no-op, which is a truthful answer, not a stub.
                 *
                 * @param dt_seconds  Fixed step duration; never wall-clock.
                 * @param observer    Where the simulation should be centred.
                 * @param julian_date Epoch, for climate/diurnal terms.
                 */
                virtual void tick(double dt_seconds, const GeodeticPosition& observer,
                                  double julian_date) = 0;
        };

        /**
         * @brief The optional authoring surface a provider may expose to the editor.
         *
         * Deliberately separate from @ref IWeatherProvider (ISP): a consumer that renders the
         * weather has no business placing pressure systems, and a provider fed by real
         * observations has no meaningful way to honour such a request. The host offers this as
         * a capability — present or absent — instead of widening the one interface every
         * provider must satisfy, which is what previously forced the concrete type into
         * `ISimulation` and locked every other implementation out.
         */
        class IWeatherAuthoring
        {
            public:
                virtual ~IWeatherAuthoring() = default;

                /** @brief The synoptic layer, mutable — place, edit, or remove a system. */
                virtual SynopticLayer& synoptic() noexcept = 0;

                /** @brief The synoptic layer, read-only — the map overlay and serialization. */
                virtual const SynopticLayer& synoptic() const noexcept = 0;

                /**
                 * @brief Seeds a named scenario, replacing the live systems.
                 * @param preset   Which named scenario to seed.
                 * @param observer Where the scenario is centred.
                 */
                virtual void apply_preset(Render::WeatherPreset preset,
                                          const GeodeticPosition& observer) = 0;
        };

        /**
         * @brief Wraps a fixed, author-set `Render::Cloudscape` as an `IWeatherProvider`.
         *
         * The manual-authoring mode's formal shape as a provider: uniform everywhere (a
         * manually authored deck stack has no spatial variation today either), decomposed
         * once at construction by bucketing each enabled deck's genus into its WMO étage
         * (`CloudLevel`) — the same three-way grouping `cloud_genus_profile`'s catalogue
         * already implies (high/middle/low+vertical).
         */
        class StaticWeather final : public IWeatherProvider
        {
            public:
                /** @brief Captures @p clouds' decks as a fixed column. */
                explicit StaticWeather(const Render::Cloudscape& clouds) : column_(decompose(clouds)) {}

                WeatherColumn sample_column(const GeodeticPosition&) const override { return column_; }

                // A manually authored deck stack is uniform everywhere by construction (see
                // the class docs), so the uniform field is not a degraded answer here -- it is
                // the whole truth about this provider.
                void publish_field(const GeodeticPosition&, WeatherFieldBuffer& out) const override
                {
                    out.fill_uniform(column_);
                }

                // A fixed authored sky has no clock of its own; time passing changes nothing
                // about it.
                void tick(double, const GeodeticPosition&, double) override {}

            private:
                static CloudLevel level_for_genus(Render::CloudGenus genus) noexcept
                {
                    switch (genus)
                    {
                    case Render::CloudGenus::Cirrus:
                    case Render::CloudGenus::Cirrocumulus:
                    case Render::CloudGenus::Cirrostratus:
                        return CloudLevel::High;
                    case Render::CloudGenus::Altocumulus:
                    case Render::CloudGenus::Altostratus:
                    case Render::CloudGenus::Nimbostratus:
                        return CloudLevel::Mid;
                    default: // Stratocumulus, Stratus, Cumulus, Cumulonimbus (grouped at its base étage).
                        return CloudLevel::Low;
                    }
                }

                static WeatherColumn decompose(const Render::Cloudscape& clouds)
                {
                    WeatherColumn column{};
                    for (int i = 0; i < Render::CLOUD_MAX_DECKS; ++i)
                    {
                        const Render::CloudDeck& deck = clouds.decks[i];
                        if (!deck.enabled)
                            continue;
                        const Render::CloudGenusProfile profile = Render::cloud_genus_profile(deck.genus);
                        WeatherLevelState& state = column.levels[static_cast<int>(level_for_genus(deck.genus))];
                        const float coverage = std::clamp(profile.coverage + deck.coverage_bias, 0.0f, 1.0f);
                        state.coverage = std::max(state.coverage, coverage);
                        state.density_scale = std::max(state.density_scale,
                                                       std::clamp(deck.density_scale * profile.density * 2.0f, 0.0f, 2.0f));
                        state.convective_fraction = std::max(state.convective_fraction, 1.0f - profile.stratiform);
                    }
                    return column;
                }

                WeatherColumn column_;
        };

        /**
         * @brief T1+T2, wrapped as an `IWeatherProvider`; the default procedural weather.
         *
         * Owns a `SynopticLayer` (T1, ticked every call — analytic and microseconds-cheap)
         * and a `RegionalWeatherGrid` (T2, ticked on its own nested
         * `Loop::FixedTimestepClock` at the design doc's 10-30 s cadence). Both clocks are
         * fed only by the caller-supplied `dt_seconds`, never wall-clock time, so this
         * class inherits `Loop::FixedTimestepClock`'s determinism guarantee directly.
         */
        class ProceduralWeather final : public IWeatherProvider, public IWeatherAuthoring
        {
            public:
                /** @brief T2's tick cadence, seconds — the design doc's 10-30 s window's low end. */
                static constexpr double DEFAULT_GRID_TICK_SECONDS = 15.0;

                /**
                 * @brief Creates a procedural weather provider seeded for reproducible evolution.
                 * @param seed            Any 64-bit seed; identical seeds reproduce identical evolution.
                 * @param planet_radius_m The dominant body's mean radius, metres.
                 * @param grid_nx         T2 horizontal cell count, X axis.
                 * @param grid_nz         T2 horizontal cell count, Z axis.
                 * @param grid_domain_m   T2 domain span per axis, metres.
                 * @param grid_tick_seconds T2's tick cadence, seconds.
                 */
                explicit ProceduralWeather(std::uint64_t seed, double planet_radius_m,
                                           int grid_nx = DEFAULT_REGIONAL_GRID_CELLS,
                                           int grid_nz = DEFAULT_REGIONAL_GRID_CELLS,
                                           double grid_domain_m = DEFAULT_REGIONAL_GRID_DOMAIN_M,
                                           double grid_tick_seconds = DEFAULT_GRID_TICK_SECONDS)
                    : grid_(grid_nx, grid_nz, grid_domain_m, planet_radius_m),
                      grid_clock_(grid_tick_seconds)
                {
                    synoptic_.seed(seed, planet_radius_m);
                }

                /**
                 * @brief Advances T1 every call and T2 whenever its own tick interval elapses.
                 * @param dt_seconds  Fixed step duration; never wall-clock (see file docs).
                 * @param observer    Where the regional grid should be centered.
                 * @param julian_date Epoch, for climate/diurnal terms.
                 */
                void tick(double dt_seconds, const GeodeticPosition& observer,
                          double julian_date) override
                {
                    synoptic_.tick(dt_seconds, julian_date);
                    if (!grid_seeded_)
                    {
                        grid_.seed(observer, synoptic_);
                        grid_seeded_ = true;
                    }
                    grid_clock_.accumulate(dt_seconds);
                    while (grid_clock_.consume_step())
                        grid_.tick(grid_clock_.fixed_dt(), synoptic_, observer, julian_date);
                }

                WeatherColumn sample_column(const GeodeticPosition& position) const override
                {
                    if (!grid_seeded_)
                        return WeatherColumn{};
                    return grid_.sample_column(position, grid_clock_.interpolation());
                }

                void publish_field(const GeodeticPosition& observer,
                                   WeatherFieldBuffer& out) const override
                {
                    // Before the first tick there is no grid to publish; a uniform empty field
                    // is what the sky looked like a moment ago and keeps the renderer from
                    // reading an unseeded lattice.
                    if (!grid_seeded_)
                    {
                        out.fill_uniform(WeatherColumn{});
                        return;
                    }
                    out.fill_from_grid(grid_, observer, grid_clock_.interpolation());
                }

                /** @brief T1, read-only — the editor's synoptic map overlay reads this. */
                const SynopticLayer& synoptic() const noexcept override { return synoptic_; }
                /** @brief T1, mutable — editor authoring (add/remove/edit a system). */
                SynopticLayer& synoptic() noexcept override { return synoptic_; }
                /** @brief T2, read-only — the editor's grid/debug view reads this. */
                const RegionalWeatherGrid& grid() const noexcept { return grid_; }

                /**
                 * @brief Seeds a named scenario (editor preset buttons), replacing the live systems.
                 *
                 * Design doc §6: "preset buttons now seed synoptic states ... instead of
                 * directly setting deck parameters". Each preset places (or omits) one
                 * `PressureSystem` relative to @p observer so the resulting sky then evolves
                 * on its own tick over tick, rather than snapping to a fixed deck mix the way
                 * `Render::cloud_weather_preset` still does for the manual-authoring mode.
                 *
                 * @param preset   Which named scenario to seed.
                 * @param observer Where the scenario is centered (the current sky observer).
                 */
                void apply_preset(Render::WeatherPreset preset,
                                  const GeodeticPosition& observer) override
                {
                    synoptic_.clear_systems();
                    switch (preset)
                    {
                    case Render::WeatherPreset::Clear:
                        break;
                    case Render::WeatherPreset::FairWeather:
                        synoptic_.add_system(scenario_system(observer, false, 8.0, 900000.0, 0.0));
                        break;
                    case Render::WeatherPreset::Overcast:
                        synoptic_.add_system(scenario_system(observer, true, 14.0, 1100000.0, 0.0));
                        break;
                    case Render::WeatherPreset::FrontPassage:
                        // Already mature and several hundred km upstream, heading toward the
                        // observer -- the acceptance-bar demo (design doc §7 W4): the front is
                        // formed and approaching, not yet arrived, so it visibly crosses over
                        // the following authored minutes rather than being present immediately.
                        synoptic_.add_system(scenario_system(observer, true, 26.0, 750000.0, -6.0));
                        break;
                    case Render::WeatherPreset::Storm:
                        synoptic_.add_system(scenario_system(observer, true, 34.0, 600000.0, -2.0));
                        break;
                    default:
                        break;
                    }
                }

            private:
                static PressureSystem scenario_system(const GeodeticPosition& observer, bool is_low,
                                                       double anomaly_hpa, double radius_m,
                                                       double longitude_offset_deg)
                {
                    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
                    PressureSystem system;
                    system.is_low = is_low;
                    system.center_latitude_radians = observer.latitude_radians;
                    system.center_longitude_radians =
                        observer.longitude_radians + longitude_offset_deg * DEGREES_TO_RADIANS;
                    system.heading_radians = 1.5707963267948966; // due east.
                    system.speed_mps = 12.0;
                    system.central_anomaly_hpa = anomaly_hpa;
                    system.radius_major_m = radius_m;
                    system.radius_minor_m = radius_m * 0.75;
                    system.orientation_radians = 0.0;
                    system.deepen_seconds = 0.0; // scenarios start already formed, not mid-genesis.
                    system.mature_seconds = 48.0 * 3600.0;
                    system.fill_seconds = 24.0 * 3600.0;
                    return system;
                }

                SynopticLayer synoptic_;
                RegionalWeatherGrid grid_;
                Loop::FixedTimestepClock grid_clock_;
                bool grid_seeded_ = false;
        };
    } // namespace Simulation
} // namespace SushiEngine
