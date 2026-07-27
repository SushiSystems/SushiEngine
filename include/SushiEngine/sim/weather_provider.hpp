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
 * GPU regional nest behind it; `StaticWeather` wraps a fixed, author-set `Render::Cloudscape`
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

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/atmosphere_forcing_buffer.hpp>
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

                /**
                 * @brief Publishes the parent solution the GPU regional nest is driven by.
                 *
                 * `docs/slop/atmosphere_system.md` §6's Davies nesting, from this side. A
                 * provider that does not drive a nest leaves this a no-op, and the forcing then
                 * never becomes valid — which is exactly what tells the renderer not to build a
                 * nest at all, so a scene with a static or ingested sky pays nothing for one.
                 *
                 * @param observer      Where the nest should be centred, geodetic.
                 * @param total_seconds Total game seconds simulated, monotonic.
                 * @param out           Caller-owned storage to fill; the caller also owns the
                 *                      lifetime the borrowed `Render::AtmosphereForcing` needs.
                 */
                virtual void publish_forcing(const GeodeticPosition& observer, double total_seconds,
                                             AtmosphereForcingBuffer& out) const
                {
                    (void)observer;
                    (void)total_seconds;
                    (void)out;
                }

                /**
                 * @brief Binds the renderer's asynchronous readback of the nest.
                 *
                 * The one thing that flows renderer → simulation (§3.2). Bound once by the host
                 * rather than ferried per frame; a provider that answers from its own state
                 * ignores it, which is a truthful answer and not a stub. Null is legal and means
                 * "answer from the base state", which is what a provider does before the first
                 * readback lands anyway.
                 *
                 * @param mirror The renderer's mirror, or null to unbind.
                 */
                virtual void set_atmosphere_mirror(const Render::IAtmosphereMirror* mirror) noexcept
                {
                    (void)mirror;
                }
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
                // the whole truth about this provider. `derives_genus` is false for the reason
                // `Render::WeatherField` gives: this column was *decomposed from* an authored
                // deck stack, so letting the bake re-derive a genus from it would overrule the
                // author with a round trip through the classifier.
                void publish_field(const GeodeticPosition&, WeatherFieldBuffer& out) const override
                {
                    out.fill_uniform(column_, false);
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
         * @brief T1 plus the GPU regional nest, wrapped as an `IWeatherProvider`.
         *
         * Owns a `SynopticLayer` (T1, ticked every call — analytic and microseconds-cheap) and
         * publishes the forcing that drives T2. It no longer owns T2 itself: the nest is a
         * device-level GPU service (`render/atmosphere/atmosphere_nest.hpp`), because the model
         * `docs/slop/atmosphere_system.md` §6 asks for — anelastic dynamics with a pressure
         * solve, monotone transport at Courant ≈ 1, Kessler microphysics — is not worth writing
         * for a CPU, and the design doc says so outright.
         *
         * What crosses back is @ref set_atmosphere_mirror's asynchronous readback, two or three
         * frames stale, which every query below is answered from. Before the first readback —
         * and in a host that never binds one — the answers come from the base state instead: a
         * clear sky with the synoptic wind, which is a truthful description of an atmosphere
         * that has not been simulated yet rather than a guess dressed up as data.
         */
        class ProceduralWeather final : public IWeatherProvider, public IWeatherAuthoring
        {
            public:
                /**
                 * @brief Creates a procedural weather provider seeded for reproducible evolution.
                 * @param seed            Any 64-bit seed; identical seeds reproduce identical T1 evolution.
                 * @param planet_radius_m The dominant body's mean radius, metres.
                 */
                explicit ProceduralWeather(std::uint64_t seed, double planet_radius_m)
                    : planet_radius_m_(planet_radius_m)
                {
                    synoptic_.seed(seed, planet_radius_m);
                }

                /**
                 * @brief Advances T1. The nest advances itself, on the renderer's own clock.
                 * @param dt_seconds  Fixed step duration; never wall-clock.
                 * @param observer    Where the simulation is centred.
                 * @param julian_date Epoch, for climate/diurnal terms.
                 */
                void tick(double dt_seconds, const GeodeticPosition& observer,
                          double julian_date) override
                {
                    (void)observer;
                    synoptic_.tick(dt_seconds, julian_date);
                }

                WeatherColumn sample_column(const GeodeticPosition& position) const override
                {
                    const Render::AtmosphereMirror mirror = current_mirror();
                    if (!mirror.valid())
                        return base_state_column(position);

                    // Scene-absolute metres for the query point, in the frame the mirror is
                    // addressed in -- the same plate-carree tangent plane every other weather
                    // consumer already uses at this scale.
                    const double cos_latitude =
                        std::max(std::cos(position.latitude_radians), 0.05);
                    const double scene_x =
                        planet_radius_m_ * position.longitude_radians * cos_latitude;
                    const double scene_z = planet_radius_m_ * position.latitude_radians;

                    const double u = double(mirror.uv_scale_x) * scene_x + double(mirror.uv_offset_x);
                    const double v = double(mirror.uv_scale_z) * scene_z + double(mirror.uv_offset_z);
                    // Outside the nest's own footprint the mirror has nothing to say, and its
                    // clamped edge would otherwise smear one boundary column across the rest of
                    // the planet. The synoptic wind is still real out there, so the base state
                    // keeps it.
                    if (u < 0.0 || u > 1.0 || v < 0.0 || v > 1.0)
                        return base_state_column(position);

                    const int ix = std::clamp(int(u * double(mirror.cells)), 0, mirror.cells - 1);
                    const int iz = std::clamp(int(v * double(mirror.cells)), 0, mirror.cells - 1);
                    return WeatherFieldBuffer::column_from_mirror(
                        mirror.columns[std::size_t(iz) * std::size_t(mirror.cells) +
                                       std::size_t(ix)]);
                }

                void publish_field(const GeodeticPosition& observer,
                                   WeatherFieldBuffer& out) const override
                {
                    out.fill_from_mirror(current_mirror(), observer);
                }

                void publish_forcing(const GeodeticPosition& observer, double total_seconds,
                                     AtmosphereForcingBuffer& out) const override
                {
                    (void)total_seconds;
                    out.fill(synoptic_, observer, planet_radius_m_, FORCING_SPAN_METERS,
                             Render::ATMOSPHERE_FORCING_MAX_CELLS);
                }

                void set_atmosphere_mirror(const Render::IAtmosphereMirror* mirror) noexcept override
                {
                    mirror_ = mirror;
                }

                /** @brief T1, read-only — the editor's synoptic map overlay reads this. */
                const SynopticLayer& synoptic() const noexcept override { return synoptic_; }
                /** @brief T1, mutable — editor authoring (add/remove/edit a system). */
                SynopticLayer& synoptic() noexcept override { return synoptic_; }

                /**
                 * @brief Seeds a named scenario (editor preset buttons), replacing the live systems.
                 *
                 * Each preset places (or omits) one `PressureSystem` relative to @p observer so
                 * the resulting sky then evolves on its own, rather than snapping to a fixed deck
                 * mix the way `Render::cloud_weather_preset` still does for manual authoring.
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
                        // observer -- the acceptance-bar demo: the front is formed and
                        // approaching, not yet arrived, so it visibly crosses over the following
                        // authored minutes rather than being present immediately.
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
                /**
                 * @brief The forcing lattice's span, metres.
                 *
                 * Deliberately wider than any nest the renderer might build, so the two are not
                 * coupled: the nest samples the middle of this, and a change to its footprint or
                 * its quality tier needs no corresponding change here. At 64 cells that is ~12 km
                 * per sample against synoptic features hundreds of kilometres across.
                 */
                static constexpr double FORCING_SPAN_METERS = 768000.0;

                Render::AtmosphereMirror current_mirror() const noexcept
                {
                    return mirror_ != nullptr ? mirror_->atmosphere_mirror()
                                              : Render::AtmosphereMirror{};
                }

                /**
                 * @brief What to answer before the nest has reported anything.
                 *
                 * A clear sky with the synoptic wind. Honest rather than convenient: nothing has
                 * been simulated yet, so there is no condensate to report, and inventing a
                 * coverage here would be exactly the fabricated signal the audit in §1 was
                 * written about. The wind is real — T1 is analytic and answers immediately.
                 */
                WeatherColumn base_state_column(const GeodeticPosition& position) const
                {
                    WeatherColumn column{};
                    const WindSample wind = synoptic_.wind_at(position, 0.25);
                    column.wind_u_mps = static_cast<float>(wind.eastward_mps);
                    column.wind_v_mps = static_cast<float>(wind.northward_mps);
                    return column;
                }

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
                double planet_radius_m_ = 6371000.0;
                // Borrowed, never owned: the renderer outlives any single provider install, and
                // a null here is a legal state meaning "answer from the base state".
                const Render::IAtmosphereMirror* mirror_ = nullptr;
        };
    } // namespace Simulation
} // namespace SushiEngine
