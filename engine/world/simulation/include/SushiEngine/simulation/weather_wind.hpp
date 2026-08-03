/**************************************************************************/
/* weather_wind.hpp                                                      */
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
 * @file weather_wind.hpp
 * @brief `weather_wind()`: one sampling function, analytic wind + a local perturbation.
 *
 * `docs/slop/weather_and_clouds.md` §5.3: "one sampling API `weather_wind(position, altitude)`
 * (GoT pattern -- analytic + perturbation, no dense volume)". T1's `wind_at` is already exactly
 * the base half (a geostrophic field, altitude-parameterized by `level_fraction`, evaluable
 * anywhere) -- this file adds only the perturbation term and the position/altitude ->
 * level_fraction mapping a consumer actually wants, so it stays a thin, narrow addition rather
 * than a second wind model.
 *
 * Deliberately a free function over `const IWeatherProvider&`, not a new
 * `IWeatherProvider` virtual: `WeatherColumn` already carries a fixed near-surface wind sample
 * (`wind_u_mps`/`wind_v_mps`) for consumers that only need that, and `StaticWeather` (the
 * manual-authoring provider) has no global core to sample at all -- widening the seam for
 * every provider to support an altitude-continuous field they cannot all honestly serve would
 * trade a narrow interface for a wider, partially-fake one.
 *
 * Turbulence intensity here is scoped to what the engine actually tracks today: the thermal
 * gradient the core has concentrated, which is what a front *is* now that nothing draws one
 * (`IWeatherProvider::frontal_strength_at`). The design doc also names CAPE and terrain
 * roughness as turbulence drivers; CAPE is an internal, per-tick intermediate inside
 * `RegionalWeatherGrid::tick_grid` (not part of `WeatherColumn`'s contract, and not stable
 * between T2 ticks the way a *sampled* signal should be), and terrain roughness has no source
 * anywhere in the engine (the same gap `regional_weather_grid.hpp`'s orographic-lift note
 * documents). Both are honestly left out rather than approximated with a fabricated signal.
 *
 * W6 addition: the perturbation term is now also exposed on its own as @ref wind_gust, so
 * `weather_flight_hazards.hpp`'s `turbulence_intensity()` -- the design doc's "flight-sim
 * payoff" (§5.3) -- can query the gust magnitude directly instead of subtracting the analytic
 * field back out of `weather_wind()`'s combined result.
 */

#include <cmath>

#include <SushiEngine/simulation/weather_provider.hpp>
#include <SushiEngine/simulation/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief The gust perturbation's amplitude ceiling, metres/second, at full front proximity. */
        constexpr double WIND_GUST_CEILING_MPS = 6.0;

        /**
         * @brief The perturbation half of `weather_wind()`, isolated: T1's front-proximity-scaled
         * wobble, with no analytic base field mixed in.
         *
         * Split out from `weather_wind()` itself so a consumer that wants the turbulence signal
         * alone -- `weather_flight_hazards.hpp`'s `turbulence_intensity()`, the flight-sim payoff
         * the design doc names -- does not have to subtract the analytic field back out of the
         * combined sample to recover it. `weather_wind()` below is exactly `wind_at() + wind_gust()`.
         *
         * @param weather          The installed provider, sampled only for the thermal gradient here.
         * @param position         Query point, geodetic.
         * @param altitude_meters  Height above the surface, metres (>= 0).
         * @param time_seconds     The simulation's own elapsed time (see `weather_wind()`'s doc).
         * @return The perturbation-only wind vector, metres/second.
         */
        inline WindSample wind_gust(const IWeatherProvider& weather,
                                    const GeodeticPosition& position, double altitude_meters,
                                    double time_seconds) noexcept
        {
            // Front proximity used to be a distance to a drawn ray. A dynamical core draws
            // nothing, so what stands in for "near an active front" is the thermal gradient the
            // flow has actually concentrated -- five kelvin per hundred kilometres being a
            // strong front, and the background baroclinic zone a small fraction of that.
            constexpr double FULLY_FRONTAL_K_PER_100KM = 5.0;
            const double gradient = weather.frontal_strength_at(position);
            const double turbulence_intensity =
                std::min(gradient / FULLY_FRONTAL_K_PER_100KM, 1.0);

            // A cheap, bounded pseudo-noise: two decorrelated sinusoids of position and time, not
            // a curl-noise field -- there is no dense volume to sample (the whole point of the
            // GoT pattern), just a per-consumer wobble that never repeats on a short period.
            constexpr double FREQUENCY = 0.35;
            const double phase = position.latitude_radians * 311.0 + position.longitude_radians * 173.0 +
                                 altitude_meters * 0.001 + time_seconds * FREQUENCY;
            const double gust_u = std::sin(phase) * WIND_GUST_CEILING_MPS * turbulence_intensity;
            const double gust_v = std::cos(phase * 1.3 + 1.7) * WIND_GUST_CEILING_MPS * turbulence_intensity;
            return WindSample{gust_u, gust_v};
        }

        /**
         * @brief Wind at a point and altitude: T1's analytic field plus a local perturbation.
         *
         * The perturbation (@ref wind_gust) is a deterministic, stateless hash of @p position/
         * @p altitude_meters/@p time_seconds (not an RNG draw -- no state to carry or desync),
         * scaled by front proximity so turbulence intensifies near an active front and is nearly
         * flat air far from one, matching the design doc's "turbulence intensity from front
         * proximity".
         *
         * @param weather          The installed provider, sampled for the base wind and the thermal gradient.
         * @param position         Query point, geodetic.
         * @param altitude_meters  Height above the surface, metres (>= 0).
         * @param time_seconds     The simulation's own elapsed time, so the perturbation animates
         *                         deterministically rather than being a fixed function of position alone.
         * @return The wind vector at that point and altitude, metres/second.
         */
        inline WindSample weather_wind(const IWeatherProvider& weather,
                                       const GeodeticPosition& position, double altitude_meters,
                                       double time_seconds) noexcept
        {
            constexpr double COLUMN_TOP_METERS = 8000.0; // matches T2's tracked column depth.
            const double level_fraction =
                std::min(std::max(altitude_meters, 0.0) / COLUMN_TOP_METERS, 1.0);
            const WindSample base = weather.wind_at(position, level_fraction);
            const WindSample gust = wind_gust(weather, position, altitude_meters, time_seconds);
            return WindSample{base.eastward_mps + gust.eastward_mps, base.northward_mps + gust.northward_mps};
        }
    } // namespace Simulation
} // namespace SushiEngine
