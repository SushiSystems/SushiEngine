/**************************************************************************/
/* weather_flight_hazards.hpp                                            */
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
 * @file weather_flight_hazards.hpp
 * @brief `icing_risk()` / `turbulence_intensity()`: narrow queries over weather state, not a
 * flight model.
 *
 * `docs/slop/weather_and_clouds.md` §7's W6 phase asks for "icing/turbulence exposure to
 * gameplay" and §5.3 names "the flight-sim payoff" as `weather_wind()`'s intended destination.
 * No flight/aircraft/vehicle system exists anywhere in this engine today (confirmed by
 * inspection of `include/SushiEngine/` and `examples/`, the same audit W4 and W5 already ran
 * before making the identical call) -- W4 and W5 both scoped their own flight-model-shaped asks
 * down to "the extension point a future flight model would call" rather than inventing one, and
 * this file follows the same discipline: two small, stateless, pure functions a future
 * flight/vehicle system can call, not a airframe simulation.
 *
 * **Icing.** Airframe icing needs two things at once: liquid water in the air, and a
 * near-freezing surface. The liquid-water half reuses `weather_world_coupling.hpp`'s own
 * `saturation_proxy` derivation (`coverage * density_scale`, clamped) -- the nearest available
 * stand-in for liquid water content the bridge carries, the identical honest approximation W5
 * already made for "the air near the surface is close to saturated". The temperature half is
 * new: `WeatherLevelState::temperature_offset_c` (W6, this phase) surfaces T2's own internal
 * per-level offset, combined here with a standard-atmosphere lapse rate
 * (ISA: 15 C at sea level, -6.5 C/km) to estimate an absolute temperature at the query
 * altitude. This is a real, named approximation, not invented data: it is the same reasoning a
 * pilot uses to forecast icing from a surface temperature and a lapse-rate rule of thumb when no
 * full sounding is available, and it is honestly bounded by what T2 actually tracks -- a
 * per-`CloudLevel`-bucket offset, not a continuous vertical profile.
 *
 * **Turbulence.** `turbulence_intensity()` is a thin wrapper over `weather_wind.hpp`'s
 * `wind_gust()`: the magnitude of the same front-proximity-scaled perturbation `weather_wind()`
 * already adds to the analytic field, normalized by its own amplitude ceiling. No new physics;
 * just a queryable scalar over state W5 already computes, per this phase's own instructions
 * ("build on weather_wind()'s gust magnitude").
 */

#include <algorithm>
#include <cmath>

#include <SushiEngine/sim/synoptic_weather.hpp>
#include <SushiEngine/sim/weather_types.hpp>
#include <SushiEngine/sim/weather_wind.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief ISA standard sea-level temperature, Celsius -- the icing lapse-rate baseline. */
        constexpr double ISA_SEA_LEVEL_TEMPERATURE_C = 15.0;

        /** @brief ISA standard tropospheric lapse rate, Celsius per metre (a fall of 6.5 C/km). */
        constexpr double ISA_LAPSE_RATE_C_PER_METER = -0.0065;

        /**
         * @brief Estimated airframe-icing risk at a column and altitude, [0, 1].
         *
         * Peaks where classic structural icing does -- supercooled liquid water between roughly
         * 0 C and -12 C -- and falls to zero above freezing (no supercooled water) and by -20 C
         * (cold enough that cloud water is predominantly ice crystals, which do not accrete the
         * way supercooled liquid droplets do), scaled by how much condensed water the level
         * actually carries. Zero whenever the level has no coverage, whatever the temperature.
         *
         * @param column          The layered-column state to query, from any `IWeatherProvider`.
         * @param altitude_meters Height above the surface, metres; selects both the `CloudLevel`
         *                        bucket and the ISA lapse-rate correction.
         * @return Icing risk, [0, 1]; not a probability, a relative-severity scalar for a future
         *         flight model to threshold or scale an accretion rate from.
         */
        inline float icing_risk(const WeatherColumn& column, double altitude_meters) noexcept
        {
            const CloudLevel level = cloud_level_for_altitude(altitude_meters);
            const WeatherLevelState& state = column.levels[static_cast<int>(level)];

            const double estimated_temperature_c = ISA_SEA_LEVEL_TEMPERATURE_C +
                ISA_LAPSE_RATE_C_PER_METER * altitude_meters + double(state.temperature_offset_c);

            // A trapezoid over temperature: 0 above +1 C or below -20 C, full weight across the
            // classic -2..-12 C icing band, tapering through the shoulders either side of it.
            double temperature_weight = 0.0;
            if (estimated_temperature_c <= 1.0 && estimated_temperature_c >= -20.0)
            {
                const double rise = std::clamp((1.0 - estimated_temperature_c) / 3.0, 0.0, 1.0);
                const double fall = std::clamp((estimated_temperature_c + 20.0) / 8.0, 0.0, 1.0);
                temperature_weight = std::min(rise, fall);
            }

            const float liquid_water_proxy = std::clamp(state.coverage * state.density_scale, 0.0f, 2.0f);
            const float risk = float(temperature_weight) * std::min(liquid_water_proxy, 1.0f);
            return std::clamp(risk, 0.0f, 1.0f);
        }

        /**
         * @brief Turbulence intensity at a point, altitude, and time: the gust term's own magnitude.
         *
         * `weather_wind()`'s perturbation (`wind_gust()`) already scales with front proximity and
         * animates deterministically over time; this normalizes its instantaneous magnitude by
         * its own amplitude ceiling (`WIND_GUST_CEILING_MPS`) into a bounded [0, 1] scalar a
         * future flight model could threshold ("light chop" vs "moderate") or feed straight into
         * a camera-shake/control-surface-buffet term.
         *
         * @param synoptic         T1, sampled for the gust perturbation (see `wind_gust()`).
         * @param position         Query point, geodetic.
         * @param altitude_meters  Height above the surface, metres (>= 0).
         * @param time_seconds     The simulation's own elapsed time (see `weather_wind()`'s doc).
         * @return Turbulence intensity, [0, 1].
         */
        inline float turbulence_intensity(const SynopticLayer& synoptic, const GeodeticPosition& position,
                                          double altitude_meters, double time_seconds) noexcept
        {
            const WindSample gust = wind_gust(synoptic, position, altitude_meters, time_seconds);
            const double magnitude =
                std::sqrt(gust.eastward_mps * gust.eastward_mps + gust.northward_mps * gust.northward_mps);
            return float(std::clamp(magnitude / WIND_GUST_CEILING_MPS, 0.0, 1.0));
        }
    } // namespace Simulation
} // namespace SushiEngine
