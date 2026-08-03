/**************************************************************************/
/* seeded_weather.hpp                                                     */
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
 * @file seeded_weather.hpp
 * @brief Manual mode's provider: a whole planet's weather chosen by a seed.
 *
 * `docs/slop/atmosphere_system.md`'s WM-SEED. What this stands in for is not another provider
 * — it is the *absence* of one. Manual mode without an `IWeatherProvider` leaves one
 * hand-authored deck stack applied to every square metre of the body, which from orbit is a
 * uniformly milky sphere; the real Earth is mostly clear ocean with weather laid over it in
 * discrete pieces.
 *
 * The distinction this draws is between weather being **placed** and weather being **grown**:
 *
 * * `ProceduralWeather` grows it. A quasi-geostrophic core evolves a flow, disturbances deepen
 *   and steer, and what the sky does next is not something anybody chose. It costs a dynamical
 *   core and a GPU nest, and it only knows about the 384 km it simulates.
 * * This places it. `Atmosphere::SynopticField` puts a dozen pressure systems on a zonal
 *   climatology from a seed, and the answer is a closed form over the whole body — no grid, no
 *   state, no cost beyond the arithmetic, and defined at every point on the planet rather than
 *   inside a nest. Nothing evolves, and that is the correct behaviour for an authored sky
 *   rather than a limitation of one: an author who typed a seed asked for *that* sky.
 *
 * Both publish the same field through the same interface, so the bake, the fog coupling, the
 * wetness and the wind all read one of them without knowing which — which is what the
 * `IWeatherProvider` seam was for, and what could not be demonstrated while only one
 * implementation was ever installed.
 *
 * **The genus is the field's to resolve here** (`Render::WeatherField::derives_genus`), unlike
 * `StaticWeather`, whose column is an authored deck stack decomposed and must not be
 * round-tripped through the classifier. This column is meteorology: a coverage and a convective
 * fraction that came from a synoptic placement, not from an author naming a cloud. Letting the
 * bake resolve a genus per column is exactly right, and it is what makes a seeded tropical
 * column tower while a seeded midlatitude one layers.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <SushiEngine/atmosphere/synoptic_field.hpp>
#include <SushiEngine/simulation/season.hpp>
#include <SushiEngine/simulation/weather_field_buffer.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief An `IWeatherProvider` whose whole state is a seed.
         *
         * @see seeded_weather.hpp for why this exists and how it differs from `ProceduralWeather`.
         */
        class SeededWeather final : public IWeatherProvider
        {
            public:
                /**
                 * @brief Places a planet's weather.
                 *
                 * @param seed            Any 64-bit value; identical seeds reproduce identical skies.
                 * @param planet_radius_m The body's mean radius, metres.
                 * @param julian_date     Epoch, for the ITCZ's seasonal position.
                 */
                explicit SeededWeather(std::uint64_t seed, double planet_radius_m,
                                       double julian_date = Astro::J2000_JULIAN_DATE)
                    : seed_(seed), planet_radius_m_(std::max(planet_radius_m, 1.0)),
                      year_fraction_(year_fraction_from_julian_date(julian_date)),
                      field_(seed, year_fraction_from_julian_date(julian_date),
                             std::max(planet_radius_m, 1.0))
                {
                }

                /** @brief The seed this sky was placed from. */
                std::uint64_t seed() const noexcept { return seed_; }

                /** @brief Re-places every system for a new seed, keeping the current season. */
                void set_seed(std::uint64_t seed)
                {
                    seed_ = seed;
                    field_.reseed(seed, year_fraction_, planet_radius_m_);
                }

                /**
                 * @brief The placement, for the renderer's planet-scale field to publish.
                 *
                 * The whole of what this provider knows, so unlike every other implementation of
                 * this method it is never null.
                 */
                const Atmosphere::SynopticField* synoptic_field() const noexcept override
                {
                    return &field_;
                }

                WeatherColumn sample_column(const GeodeticPosition& position) const override
                {
                    return column_at(position);
                }

                /**
                 * @brief Samples the placement onto a lattice around the observer.
                 *
                 * The span is the one thing worth arguing about here, and it is deliberately
                 * **local** rather than planetary. A 64-cell field stretched over a whole planet
                 * would put 600 km between samples, so the near window — 32 km across — would
                 * land inside a single cell and read as uniform, which is the defect this class
                 * exists to remove. The planetary answer travels by a different route
                 * (`Render::SynopticFieldView`, evaluated in closed form by the march), so the
                 * lattice is free to cover only what the lattice's consumer can see.
                 */
                void publish_field(const GeodeticPosition& observer,
                                   WeatherFieldBuffer& out) const override
                {
                    out.fill_from_sampler(observer, planet_radius_m_, FIELD_SPAN_METERS,
                                          FIELD_CELLS,
                                          [this](const GeodeticPosition& position) {
                                              return column_at(position);
                                          });
                }

                /**
                 * @brief Advances nothing but the season.
                 *
                 * A placed sky has no dynamics — see the file docs. The date still moves the
                 * convergence zone, because that is a property of where the year is rather than
                 * of anything this class simulates, and re-placing on it costs twelve systems'
                 * worth of arithmetic on the roughly 365 calls a simulated year where the day
                 * actually changes.
                 */
                void tick(double, const GeodeticPosition&, double julian_date) override
                {
                    const double year_fraction = year_fraction_from_julian_date(julian_date);
                    if (std::fabs(year_fraction - year_fraction_) < YEAR_FRACTION_EPSILON)
                        return;
                    year_fraction_ = year_fraction;
                    field_.reseed(seed_, year_fraction_, planet_radius_m_);
                }

                /**
                 * @brief The geostrophic wind implied by the placed systems.
                 *
                 * Honest rather than zero, and nearly free: a low is a rotating anomaly, so the
                 * flow around it is the tangential field its gradient implies — cyclonic in
                 * whichever hemisphere it sits in. This is what the cloud advection and the rain
                 * drift read, so a seeded storm's cloud moves the way its own circulation says
                 * it should rather than standing still.
                 *
                 * The vertical structure is a single low-level answer: @p level_fraction is
                 * accepted and ignored, because a placement carries no shear and inventing one
                 * would be a number pretending to be a measurement.
                 */
                WindSample wind_at(const GeodeticPosition& position, double) const override
                {
                    constexpr double SAMPLE_STEP_RADIANS = 2.0e-3; // ~13 km on Earth.
                    const double cos_latitude =
                        std::max(std::cos(position.latitude_radians), MIN_COS_LATITUDE);

                    // The coverage gradient stands in for the pressure gradient: both are the
                    // same Gaussian bumps with the same centres, so their gradients point the
                    // same way and only the scale differs.
                    const double east =
                        field_.coverage_at(position.latitude_radians,
                                           position.longitude_radians + SAMPLE_STEP_RADIANS) -
                        field_.coverage_at(position.latitude_radians,
                                           position.longitude_radians - SAMPLE_STEP_RADIANS);
                    const double north =
                        field_.coverage_at(position.latitude_radians + SAMPLE_STEP_RADIANS,
                                           position.longitude_radians) -
                        field_.coverage_at(position.latitude_radians - SAMPLE_STEP_RADIANS,
                                           position.longitude_radians);

                    // Geostrophic balance is `f k × v = -(1/rho) grad p`, which solves to
                    // `v = (1/(f rho)) k x grad p`. Coverage is high where pressure is low, so
                    // `grad p` runs *against* the coverage gradient and the rotation comes out
                    // as `v = -(k x grad coverage)` — in components, eastward from the northward
                    // gradient and northward from the negated eastward one.
                    //
                    // **The sign is the whole content of this function** and it is easy to write
                    // backwards: the opposite choice puts an anticyclone around every low, which
                    // still looks like circulation and is wrong everywhere. The check that
                    // settles it is that flow around a northern-hemisphere low is
                    // counterclockwise, so directly north of one the wind blows *west*.
                    //
                    // Flipped across the equator, because `f` is.
                    const double sense = position.latitude_radians >= 0.0 ? 1.0 : -1.0;
                    const double scale = GEOSTROPHIC_WIND_MPS * sense;
                    return WindSample{north * scale, -(east / cos_latitude) * scale};
                }

                /**
                 * @brief The coverage anomaly, restated as a pressure departure.
                 *
                 * Not a simulated pressure — this class simulates nothing — but not a stub
                 * either: the placement *is* a field of highs and lows, and this is the same
                 * field read in the units the panels ask for. A low reads negative.
                 */
                double pressure_anomaly_hpa(const GeodeticPosition& position) const override
                {
                    const double zonal = Atmosphere::synoptic_zonal_coverage(
                        position.latitude_radians, field_.itcz_latitude());
                    const double anomaly =
                        field_.coverage_at(position.latitude_radians,
                                           position.longitude_radians) - zonal;
                    return -anomaly * HPA_PER_COVERAGE;
                }

            private:
                // The lattice the near/far windows are baked from. 384 km at 48 cells is 8 km a
                // sample, which is finer than any synoptic feature this field places and is the
                // same order the GPU nest's own mirror resolves -- so switching between the two
                // providers does not change how sharply the bake sees a front.
                static constexpr double FIELD_SPAN_METERS = 384000.0;
                static constexpr int FIELD_CELLS = 48;

                // Bounds the plate-carree easting factor at the poles, matching
                // `WeatherFieldBuffer`'s own guard rather than introducing a second opinion.
                static constexpr double MIN_COS_LATITUDE = 0.05;

                // A day out of a year: below this the ITCZ has not moved far enough to be worth
                // re-placing twelve systems for, and the core's own season is quantized to whole
                // days anyway.
                static constexpr double YEAR_FRACTION_EPSILON = 1.0 / 365.0;

                // Peak wind around a fully developed system, metres/second, against a coverage
                // gradient of one across the sampling step. Scaled to land in the 10-20 m/s a
                // mature synoptic circulation actually carries at low level.
                static constexpr double GEOSTROPHIC_WIND_MPS = 900.0;

                // A full-amplitude anomaly is about 30 hPa, which is the depth of a strong but
                // not exceptional mid-latitude cyclone -- the same scale `ProceduralWeather`'s
                // own measured preset table lands its injections at.
                static constexpr double HPA_PER_COVERAGE = 30.0;

                /**
                 * @brief The placement, read as the layered column every consumer speaks.
                 *
                 * The vertical split is where the placement stops being able to answer and a
                 * rule takes over, so it is stated rather than tuned: a synoptic system is
                 * deepest at the level its own character names — convection fills the low and
                 * mid levels and throws cirrus off the top, while a stratiform system is a
                 * mid-level sheet with a low deck under it — and the high level always carries
                 * some cirrus, because it nearly always does.
                 */
                WeatherColumn column_at(const GeodeticPosition& position) const
                {
                    const double coverage = field_.coverage_at(position.latitude_radians,
                                                               position.longitude_radians);
                    const double convective = field_.convective_at(position.latitude_radians,
                                                                   position.longitude_radians);
                    const double precipitation = field_.precipitation_at(
                        position.latitude_radians, position.longitude_radians);

                    WeatherColumn column{};
                    WeatherLevelState& low = column.levels[int(CloudLevel::Low)];
                    WeatherLevelState& mid = column.levels[int(CloudLevel::Mid)];
                    WeatherLevelState& high = column.levels[int(CloudLevel::High)];

                    low.coverage = float(std::clamp(coverage, 0.0, 1.0));
                    // A convective sky is broken by construction -- towers with gaps between
                    // them -- so its mid level carries less than its low one; a stratiform sky
                    // is a sheet and carries the same at both.
                    mid.coverage = float(std::clamp(coverage * (1.0 - 0.45 * convective), 0.0, 1.0));
                    // Cirrus outlives whatever made it, so the high level is the least
                    // organised of the three: a broad background term, plus the anvil a deep
                    // convective column throws off.
                    //
                    // The background is scaled by the column's own coverage rather than added
                    // as a constant. An additive term is a plausible *mean* cirrus cover and
                    // the wrong shape, because it is the one term no high can suppress: a
                    // subsiding subtropical column empties its low and mid levels correctly
                    // and keeps a permanent veil over the top, so the place that is supposed
                    // to read as open ocean never does. Scaling lets a high clear the whole
                    // column while leaving an ordinary sky generously capped with cirrus.
                    // Mirrored in cloud.frag's `cloud_globe_envelope`; the two must agree or the
                    // far window's rim becomes a step.
                    high.coverage = float(
                        std::clamp(coverage * (0.30 + 0.25 + 0.45 * convective), 0.0, 1.0));

                    const float convective_f = float(std::clamp(convective, 0.0, 1.0));
                    low.convective_fraction = convective_f;
                    mid.convective_fraction = convective_f * 0.6f;
                    high.convective_fraction = 0.0f; // cirrus is not convective where it sits.

                    // Density tracks coverage rather than being independent of it: a sky that is
                    // more covered is more covered *because* there is more water in it.
                    const float density = float(std::clamp(0.4 + coverage * 1.2, 0.0, 2.0));
                    low.density_scale = density;
                    mid.density_scale = density * 0.8f;
                    high.density_scale = density * 0.35f;

                    column.precipitation = float(std::clamp(precipitation, 0.0, 1.0));
                    // Convection lifts its own base -- a tropical cumulus base sits well above a
                    // frontal stratus one -- and a column with no cloud in it has no base at
                    // all, which is the reading the fog coupling depends on.
                    column.cloud_base_m =
                        low.coverage > 0.02f
                            ? float(400.0 + 1200.0 * convective + 600.0 * (1.0 - coverage))
                            : 0.0f;

                    const WindSample wind = wind_at(position, 0.25);
                    column.wind_u_mps = float(wind.eastward_mps);
                    column.wind_v_mps = float(wind.northward_mps);
                    return column;
                }

                std::uint64_t seed_ = 0;
                double planet_radius_m_ = 6371000.0;
                double year_fraction_ = 0.0;
                Atmosphere::SynopticField field_;
        };
    } // namespace Simulation
} // namespace SushiEngine
