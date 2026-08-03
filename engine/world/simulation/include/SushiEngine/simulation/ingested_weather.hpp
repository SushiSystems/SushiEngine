/**************************************************************************/
/* ingested_weather.hpp                                                   */
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
 * @file ingested_weather.hpp
 * @brief `IngestedWeather`: real-data `IWeatherProvider`, per the design doc's §5.4 seam.
 *
 * `docs/slop/weather_and_clouds.md` §5.4: "`IngestedWeather` implements `IWeatherProvider` from
 * GRIB (GFS/WAFS winds, temperature, humidity) blended toward METARs near airfields -- the
 * X-Plane 12 three-stage blend. The column representation is already identical; nothing
 * downstream changes."
 *
 * **What this phase actually decodes.** GRIB2 is a large, specialized binary format tied to a
 * real external data source (a GFS/WAFS feed this engine has no fetcher for) -- genuinely out
 * of scope for one phase, per the task brief's own instruction to scope that part down to "a
 * clearly-named stub/interface point". That stub is @ref set_background: it accepts a
 * `WeatherColumn` already expressed in the bridge's own contract, standing in for "whatever a
 * real GRIB decoder would eventually produce" -- decoding the binary format itself, and fetching
 * it from a real GFS/WAFS source, is named future work, not fabricated here. METAR, by contrast,
 * *is* fully decoded this phase (see `metar_parser.hpp`): `add_station` takes a raw report
 * string and a real parser turns it into a `WeatherColumn`.
 *
 * **The three-stage blend** (X-Plane 12's own scheme): far from every known station, the sample
 * is pure background (GRIB, or whatever @ref set_background was last given); inside
 * @ref NEAR_STATION_RADIUS_METERS of the nearest station it is pure METAR; between the two
 * radii it linearly blends level-by-level, so an approach into an airfield's own reported
 * weather is continuous rather than a hard cut at some boundary.
 *
 * **The point of this file, regardless of how much real decoding it contains**: `IngestedWeather`
 * is a genuine, LSP-substitutable `IWeatherProvider`. `WeatherCloudscapeCompiler`,
 * `WeatherWorldCoupling`, and anything else built against the seam accept it exactly as they
 * accept `ProceduralWeather` or `StaticWeather` today, with zero changes -- see
 * `tests/unit/test_ingested_weather.cpp`'s `SubstitutesForAnyOtherProvider` case,
 * which runs the identical assertions W4/W5's own provider tests already ran, through this
 * provider instead.
 */

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <SushiEngine/simulation/metar_parser.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>
#include <SushiEngine/simulation/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief GRIB/METAR-sourced `IWeatherProvider`: a background column blended toward the
         * nearest reporting station.
         *
         * Owns no clock and ticks nothing (unlike `ProceduralWeather`): real-data ingestion is
         * inherently a snapshot of the latest fetched/parsed report, refreshed by the caller
         * calling @ref set_background / @ref add_station again whenever new data arrives, not by
         * a fixed-step simulation. The design doc's own multiplayer-determinism note applies
         * here, not to this class's internals: "ingested snapshots are timestamped inputs
         * distributed like any other command" is a distribution concern for whatever future code
         * fetches and broadcasts the raw reports, orthogonal to this class reading the
         * already-received snapshot.
         */
        class IngestedWeather final : public IWeatherProvider
        {
            public:
                /** @brief Inside this radius of a station, the sample is that station's METAR alone. */
                static constexpr double NEAR_STATION_RADIUS_METERS = 15000.0; // ~8 nm.

                /** @brief Beyond this radius of every station, the sample is pure background. */
                static constexpr double FAR_BACKGROUND_RADIUS_METERS = 60000.0; // ~32 nm.

                /**
                 * @brief Sets the regional background column -- the GRIB stub (see file docs).
                 * @param column The background state; a default-constructed `WeatherColumn`
                 *               (clear, calm) is the honest choice until a real decoder exists.
                 */
                void set_background(const WeatherColumn& column) { background_ = column; }

                /** @brief Removes every station added by @ref add_station. */
                void clear_stations() { stations_.clear(); }

                /**
                 * @brief Adds (or replaces, by @p position) a station's latest METAR.
                 *
                 * @param position   The station's geodetic position.
                 * @param raw_metar  The raw report text -- parsed by `parse_metar` immediately, so
                 *                   a malformed report degrades to that station reporting an
                 *                   all-clear column rather than corrupting the provider's state.
                 */
                void add_station(const GeodeticPosition& position, const std::string& raw_metar)
                {
                    Station station;
                    station.position = position;
                    station.column = metar_to_weather_column(parse_metar(raw_metar));
                    stations_.push_back(station);
                }

                /**
                 * @brief Adds (or replaces) a station from an already-decoded column.
                 *
                 * The programmatic equivalent of @ref add_station, for callers that already hold
                 * a `WeatherColumn` (tests, or a future non-METAR observation source) rather than
                 * raw report text.
                 * @param position Station position.
                 * @param column   The station's observed column.
                 */
                void add_station_column(const GeodeticPosition& position, const WeatherColumn& column)
                {
                    Station station;
                    station.position = position;
                    station.column = column;
                    stations_.push_back(station);
                }

                WeatherColumn sample_column(const GeodeticPosition& position) const override
                {
                    if (stations_.empty())
                        return background_;

                    std::size_t nearest_index = 0;
                    double nearest_distance = great_circle_distance_meters(position, stations_[0].position);
                    for (std::size_t i = 1; i < stations_.size(); ++i)
                    {
                        const double distance = great_circle_distance_meters(position, stations_[i].position);
                        if (distance < nearest_distance)
                        {
                            nearest_distance = distance;
                            nearest_index = i;
                        }
                    }

                    if (nearest_distance >= FAR_BACKGROUND_RADIUS_METERS)
                        return background_;
                    if (nearest_distance <= NEAR_STATION_RADIUS_METERS)
                        return stations_[nearest_index].column;

                    const double span = FAR_BACKGROUND_RADIUS_METERS - NEAR_STATION_RADIUS_METERS;
                    const double station_weight =
                        1.0 - (nearest_distance - NEAR_STATION_RADIUS_METERS) / span;
                    return blend(background_, stations_[nearest_index].column, station_weight);
                }

                /**
                 * @brief Publishes the station blend as a real spatial field.
                 *
                 * Unlike `StaticWeather`, this provider genuinely varies horizontally — the
                 * whole point of blending toward the nearest report is that the sky near an
                 * airfield differs from the sky between airfields. That structure lives inside
                 * `sample_column` rather than in a stored grid, so the field is produced by
                 * sampling the query onto a lattice spanning the station-influence radius; any
                 * smaller span would publish a field that is uniform only because it was cut
                 * off before the next station.
                 */
                // Ingested state advances when new observations arrive, not with the clock:
                // this provider owns no simulation to step (see the file docs).
                void tick(double, const GeodeticPosition&, double) override {}

                void publish_field(const GeodeticPosition& observer,
                                   WeatherFieldBuffer& out) const override
                {
                    constexpr int FIELD_CELLS = 32;
                    out.fill_from_sampler(observer, EARTH_RADIUS_METERS,
                                          FAR_BACKGROUND_RADIUS_METERS * 2.0, FIELD_CELLS,
                                          [this](const GeodeticPosition& position)
                                          { return sample_column(position); });
                }

            private:
                // Mean Earth radius. Station blending only needs a sane relative distance, not a
                // geodesic-accurate one, and the published field's tangent projection needs the
                // same scale the distances were measured against -- so the two share one constant
                // rather than each carrying its own.
                static constexpr double EARTH_RADIUS_METERS = 6371000.0;

                struct Station
                {
                    GeodeticPosition position;
                    WeatherColumn column;
                };

                static double great_circle_distance_meters(const GeodeticPosition& a,
                                                            const GeodeticPosition& b) noexcept
                {
                    constexpr double EARTH_RADIUS_M = EARTH_RADIUS_METERS;
                    const double dlat = b.latitude_radians - a.latitude_radians;
                    const double dlon = b.longitude_radians - a.longitude_radians;
                    const double sin_dlat = std::sin(dlat * 0.5);
                    const double sin_dlon = std::sin(dlon * 0.5);
                    const double h = sin_dlat * sin_dlat +
                        std::cos(a.latitude_radians) * std::cos(b.latitude_radians) * sin_dlon * sin_dlon;
                    return 2.0 * EARTH_RADIUS_M * std::asin(std::min(1.0, std::sqrt(h)));
                }

                static WeatherLevelState blend_level(const WeatherLevelState& a, const WeatherLevelState& b,
                                                      double station_weight) noexcept
                {
                    const float t = float(station_weight);
                    WeatherLevelState out;
                    out.coverage = a.coverage + (b.coverage - a.coverage) * t;
                    out.density_scale = a.density_scale + (b.density_scale - a.density_scale) * t;
                    out.convective_fraction =
                        a.convective_fraction + (b.convective_fraction - a.convective_fraction) * t;
                    out.temperature_offset_c =
                        a.temperature_offset_c + (b.temperature_offset_c - a.temperature_offset_c) * t;
                    return out;
                }

                static WeatherColumn blend(const WeatherColumn& background, const WeatherColumn& station,
                                           double station_weight) noexcept
                {
                    WeatherColumn out;
                    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
                        out.levels[level] = blend_level(background.levels[level], station.levels[level], station_weight);
                    const float t = float(station_weight);
                    out.precipitation = background.precipitation + (station.precipitation - background.precipitation) * t;
                    out.wind_u_mps = background.wind_u_mps + (station.wind_u_mps - background.wind_u_mps) * t;
                    out.wind_v_mps = background.wind_v_mps + (station.wind_v_mps - background.wind_v_mps) * t;
                    return out;
                }

                WeatherColumn background_{}; // the GRIB stub's current snapshot; clear/calm until set.
                std::vector<Station> stations_;
        };
    } // namespace Simulation
} // namespace SushiEngine
