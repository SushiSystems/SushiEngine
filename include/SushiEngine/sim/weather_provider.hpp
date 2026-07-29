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
 * nothing about how it got there. `ProceduralWeather` wraps the global dynamical core and the
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
#include <vector>

#include <SushiEngine/atmosphere/climatology.hpp>
#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>
#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/atmosphere_forcing_buffer.hpp>
#include <SushiEngine/sim/weather_field_buffer.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief The seam the renderer's cloud bridge consumes: "the weather at a point".
         *
         * ISP-narrow on purpose (design doc §9): a consumer that only needs to compile a
         * `Render::Cloudscape` (see `WeatherCloudscapeCompiler`) never sees a potential
         * vorticity, a grid cell, or an RNG — only this.
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
                 * @brief Wind at a point and level, m/s.
                 *
                 * On this interface for the same reason @ref publish_field is: every provider
                 * can answer it honestly. A fixed authored sky has one wind at every altitude,
                 * which is the whole truth about a fixed authored sky and not a stub.
                 *
                 * @param position       Query point, geodetic.
                 * @param level_fraction 0 at the surface through 1 at the tropopause.
                 * @return The wind, metres per second.
                 */
                virtual WindSample wind_at(const GeodeticPosition& position,
                                           double level_fraction) const
                {
                    (void)position;
                    (void)level_fraction;
                    return WindSample{};
                }

                /**
                 * @brief Surface pressure anomaly at a point, hPa about the zonal mean.
                 *
                 * The zonal mean rather than a global one, because the mean meridional gradient
                 * is worth an order more than any cyclone (see
                 * `Atmosphere::QuasiGeostrophicCore::pressure_anomaly_hpa`, which states the
                 * measurement) and a consumer asking this wants the low, not the latitude. A
                 * provider with no horizontal structure answers zero, which is true of it.
                 *
                 * @param position Query point, geodetic.
                 * @return Departure from the zonal-mean surface pressure, hectopascals.
                 */
                virtual double pressure_anomaly_hpa(const GeodeticPosition& position) const
                {
                    (void)position;
                    return 0.0;
                }

                /**
                 * @brief Strength of the thermal gradient at a point, K per 100 km.
                 *
                 * "Is there a front here", asked of the field rather than of a list of drawn
                 * fronts. A provider with no horizontal thermal structure answers zero, which is
                 * true of it.
                 *
                 * @param position Query point, geodetic.
                 * @return Magnitude of the horizontal temperature gradient, K/100 km.
                 */
                virtual double frontal_strength_at(const GeodeticPosition& position) const
                {
                    (void)position;
                    return 0.0;
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

                /**
                 * @brief Adds a rotating anomaly to the global flow, which then evolves on its own.
                 *
                 * **The replacement for handing the editor a list of pressure systems to drag.**
                 * There is no longer a list: a low is a feature of a field, not an object, and
                 * what an author can do to a field is disturb it. What is injected is vorticity,
                 * so the disturbance deepens, tilts, sheds a front and moves with the steering
                 * flow instead of translating as a rigid shape — which is strictly more useful
                 * authoring as well as the only kind a dynamical core can honour.
                 *
                 * Cyclonic in whichever hemisphere it lands in, so "place a low" means a low on
                 * either side of the equator.
                 *
                 * @param position      Where to centre it, geodetic.
                 * @param radius_m      e-folding radius, metres. Synoptic scale is 500-1500 km.
                 * @param amplitude_mps Peak rotational wind to scale it to; positive for a low,
                 *                      negative for a high.
                 */
                virtual void inject_vorticity(const GeodeticPosition& position, double radius_m,
                                              double amplitude_mps) = 0;

                /**
                 * @brief Seeds a named scenario as a starting condition, not as an outcome.
                 * @param preset   Which named scenario to seed.
                 * @param observer Where the scenario is centred.
                 */
                virtual void apply_preset(Render::WeatherPreset preset,
                                          const GeodeticPosition& observer) = 0;

                /**
                 * @brief Serializes the evolving state for a scene save.
                 * @return An opaque blob, or empty if this provider has no state worth keeping.
                 */
                virtual std::vector<std::uint8_t> capture_state() const = 0;

                /**
                 * @brief Restores a blob from @ref capture_state.
                 * @param blob The bytes to adopt.
                 * @return Whether it was accepted; false leaves the current state untouched.
                 */
                virtual bool restore_state(const std::vector<std::uint8_t>& blob) = 0;
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
         * Owns the global dynamical core (`Atmosphere::QuasiGeostrophicCore`, §5) and publishes
         * the forcing that drives T2. It no longer owns T2 itself: the nest is a device-level
         * GPU service (`render/atmosphere/atmosphere_nest.hpp`), because the model
         * `docs/slop/atmosphere_system.md` §6 asks for — anelastic dynamics with a pressure
         * solve, monotone transport at Courant ≈ 1, Kessler microphysics — is not worth writing
         * for a CPU, and the design doc says so outright. T1 is, for the mirror-image reason
         * (§3.3): every consumer of it is on this side of the seam.
         *
         * **The two tiers are stepped on different clocks and neither is the frame.** The core
         * takes a step per six minutes of game time and carries the remainder itself, so
         * @ref tick may hand it a whole simulation step and have nothing happen — which is
         * correct, and is why the core exposes `advance` rather than `step`.
         *
         * What crosses back is @ref set_atmosphere_mirror's asynchronous readback, two or three
         * frames stale, which every column query below is answered from. Before the first
         * readback — and in a host that never binds one — the answers come from the base state
         * instead: a clear sky with the core's own wind, which is a truthful description of an
         * atmosphere whose *regional* detail has not been simulated yet rather than a guess
         * dressed up as data.
         */
        class ProceduralWeather final : public IWeatherProvider, public IWeatherAuthoring
        {
            public:
                /**
                 * @brief Creates a procedural weather provider seeded for reproducible evolution.
                 *
                 * The climatology is taken by value rather than read from a path here: this class
                 * owns the dynamics, and which mean state they run on is the caller's decision —
                 * a test wants analytic bands, the editor wants the baked asset, and a body that
                 * is not Earth wants neither. `Simulation::load_climatology` is the one place that
                 * turns a file into one of these.
                 *
                 * @param seed            Any 64-bit seed; identical seeds reproduce identical T1 evolution.
                 * @param planet_radius_m The dominant body's mean radius, metres.
                 * @param climatology     The mean state T1 relaxes toward; analytic bands by default.
                 */
                explicit ProceduralWeather(std::uint64_t seed, double planet_radius_m,
                                           const Atmosphere::Climatology& climatology =
                                               Atmosphere::Climatology())
                    : planet_radius_m_(planet_radius_m),
                      core_(grid_for(), physics_for(planet_radius_m), climatology)
                {
                    core_.seed(seed);
                }

                /**
                 * @brief The mean state this weather is a departure from.
                 *
                 * Exposed so a host can say *which* climatology is running. A scene on analytic
                 * bands when somebody meant it to run on the baked asset is otherwise invisible
                 * until the jet turns out to be in the wrong place.
                 */
                const Atmosphere::Climatology& climatology() const noexcept
                {
                    return core_.climatology();
                }

                /**
                 * @brief Advances T1. The nest advances itself, on the renderer's own clock.
                 *
                 * The core's step is six minutes of game time, so most calls advance nothing and
                 * only accumulate; `advance` owns that remainder rather than making the caller
                 * track it. The Julian date is not passed on: the core's mean state is a
                 * climatology, and the season enters through T0 rather than through the tick.
                 *
                 * @param dt_seconds  Fixed step duration; never wall-clock.
                 * @param observer    Where the simulation is centred.
                 * @param julian_date Epoch, for climate/diurnal terms.
                 */
                void tick(double dt_seconds, const GeodeticPosition& observer,
                          double julian_date) override
                {
                    (void)observer;
                    (void)julian_date;
                    core_.advance(dt_seconds);
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
                    out.fill(core_, observer, planet_radius_m_, FORCING_SPAN_METERS,
                             Render::ATMOSPHERE_FORCING_MAX_CELLS);
                }

                void set_atmosphere_mirror(const Render::IAtmosphereMirror* mirror) noexcept override
                {
                    mirror_ = mirror;
                }

                void inject_vorticity(const GeodeticPosition& position, double radius_m,
                                      double amplitude_mps) override
                {
                    core_.inject_vorticity(
                        Atmosphere::GeographicPosition{position.latitude_radians,
                                                       position.longitude_radians},
                        radius_m, amplitude_mps);
                }

                double pressure_anomaly_hpa(const GeodeticPosition& position) const override
                {
                    return core_.pressure_anomaly_hpa(Atmosphere::GeographicPosition{
                        position.latitude_radians, position.longitude_radians});
                }

                double frontal_strength_at(const GeodeticPosition& position) const override
                {
                    return core_.frontal_strength_at(Atmosphere::GeographicPosition{
                        position.latitude_radians, position.longitude_radians});
                }

                WindSample wind_at(const GeodeticPosition& position,
                                   double level_fraction) const override
                {
                    const Atmosphere::Wind wind = core_.wind_at(
                        Atmosphere::GeographicPosition{position.latitude_radians,
                                                       position.longitude_radians},
                        level_fraction);
                    return WindSample{wind.eastward_mps, wind.northward_mps};
                }

                std::vector<std::uint8_t> capture_state() const override { return core_.capture(); }

                bool restore_state(const std::vector<std::uint8_t>& blob) override
                {
                    return core_.restore(blob);
                }

                /**
                 * @brief Seeds a named scenario (editor preset buttons) as a starting condition.
                 *
                 * **A preset is now an initial condition rather than a script.** It injects an
                 * anomaly upstream of @p observer and the dynamics take it from there, so what
                 * arrives is whatever that disturbance grows into — which is the point of having
                 * a dynamical core, and also means the sky a preset produces is not identical
                 * every time the preset is pressed on a different flow.
                 *
                 * Upstream means west: the core's mean state is a westerly jet, so a disturbance
                 * placed to the west is a disturbance that will arrive.
                 *
                 * @param preset   Which named scenario to seed.
                 * @param observer Where the scenario is centered (the current sky observer).
                 */
                void apply_preset(Render::WeatherPreset preset,
                                  const GeodeticPosition& observer) override
                {
                    switch (preset)
                    {
                    case Render::WeatherPreset::Clear:
                        // Subsidence, and nothing arriving: a high overhead rather than the
                        // absence of weather, because the absence of weather is not something a
                        // dynamical core can be asked for.
                        inject_upstream(observer, 0.0, 1200000.0, -14.0);
                        break;
                    case Render::WeatherPreset::FairWeather:
                        inject_upstream(observer, 0.0, 900000.0, -8.0);
                        break;
                    case Render::WeatherPreset::Overcast:
                        inject_upstream(observer, 0.0, 1100000.0, 14.0);
                        break;
                    case Render::WeatherPreset::FrontPassage:
                        // Placed several hundred kilometres upstream so it visibly *arrives*
                        // over the following authored minutes rather than being present
                        // immediately — the acceptance-bar demo.
                        inject_upstream(observer, -6.0, 750000.0, 26.0);
                        break;
                    case Render::WeatherPreset::Storm:
                        inject_upstream(observer, -2.0, 600000.0, 34.0);
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
                 * written about. The wind is real: T1 has a flow from the moment it is seeded —
                 * the mean jet, before any eddy has grown — so this is the *regional* detail
                 * being absent, not the weather.
                 */
                WeatherColumn base_state_column(const GeodeticPosition& position) const
                {
                    WeatherColumn column{};
                    const WindSample wind = wind_at(position, BASE_STATE_LEVEL);
                    column.wind_u_mps = static_cast<float>(wind.eastward_mps);
                    column.wind_v_mps = static_cast<float>(wind.northward_mps);
                    return column;
                }

                /** @brief Injects an anomaly @p longitude_offset_deg from @p observer. */
                void inject_upstream(const GeodeticPosition& observer, double longitude_offset_deg,
                                     double radius_m, double amplitude_mps)
                {
                    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
                    inject_vorticity(
                        GeodeticPosition{observer.latitude_radians,
                                         observer.longitude_radians +
                                             longitude_offset_deg * DEGREES_TO_RADIANS},
                        radius_m, amplitude_mps);
                }

                /**
                 * @brief The core's grid.
                 *
                 * Not resolved from the render quality tier, and deliberately: §11's C2 measured
                 * that below about 128 latitudes the grid-scale damping eats the most unstable
                 * baroclinic mode and the core stops producing weather at all. A tier that
                 * halved this twice would not be a cheaper atmosphere, it would be no
                 * atmosphere, so the one resolution that works is the one that is built.
                 */
                static Atmosphere::QuasiGeostrophicGridSize grid_for() noexcept
                {
                    return Atmosphere::QuasiGeostrophicGridSize{};
                }

                /** @brief The core's physics, with the body's own radius substituted in. */
                static Atmosphere::QuasiGeostrophicParameters physics_for(double planet_radius_m)
                {
                    Atmosphere::QuasiGeostrophicParameters parameters;
                    parameters.planet_radius_m = planet_radius_m;
                    return parameters;
                }

                /**
                 * @brief Level fraction the base-state wind is reported at.
                 *
                 * Low in the column: this answers "what is the wind where the player is", and
                 * the jet is not where the player is.
                 */
                static constexpr double BASE_STATE_LEVEL = 0.25;

                double planet_radius_m_ = 6371000.0;
                Atmosphere::QuasiGeostrophicCore core_;
                // Borrowed, never owned: the renderer outlives any single provider install, and
                // a null here is a legal state meaning "answer from the base state".
                const Render::IAtmosphereMirror* mirror_ = nullptr;
        };
    } // namespace Simulation
} // namespace SushiEngine
