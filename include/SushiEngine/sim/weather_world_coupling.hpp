/**************************************************************************/
/* weather_world_coupling.hpp                                            */
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
 * @file weather_world_coupling.hpp
 * @brief The sim-side bridge: any `IWeatherProvider`'s column state -> `Render::WeatherCoupling`.
 *
 * `docs/slop/weather_and_clouds.md` §5.3 (W5, "coupling weather -> world (beyond clouds)"):
 * fog/turbidity, wet ground, and precipitation intensity should all follow the *same*
 * `WeatherColumn` sample that already drives the compiled `Cloudscape`
 * (`WeatherCloudscapeCompiler`) -- the acceptance bar's "one cause, every symptom". This is
 * that sibling compiler: a pure, stateless function of a `WeatherColumn`, exactly like
 * `WeatherCloudscapeCompiler`, so `RuntimeSimulation::extract()` can call both compilers on
 * one sampled column and hand each result to the render tier through `Render::Environment`.
 *
 * `WeatherColumn` does not carry a dew-point/humidity field directly (see `weather_types.hpp`
 * -- T2's per-level humidity is internal `WeatherCell` state, not part of the bridge contract),
 * so the design doc's "dew-point spread" fog driver is approximated here by the low band's
 * cloud coverage/density -- itself derived from condensed water past T2's relative-humidity
 * threshold, the nearest available proxy for "the air near the surface is close to saturated".
 * A named, honest simplification rather than widening `WeatherColumn`'s contract for a field
 * only this one consumer would use.
 */

#include <algorithm>
#include <cmath>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Compiles a `WeatherColumn` into a ready-to-apply `Render::WeatherCoupling`.
         */
        class WeatherWorldCoupling
        {
            public:
                /**
                 * @brief Compiles @p column into a `WeatherCoupling`.
                 * @param column The layered-column state to react to, from any `IWeatherProvider`.
                 * @return A `WeatherCoupling` ready to assign to `Render::Environment::weather`.
                 */
                Render::WeatherCoupling compile(const WeatherColumn& column) const
                {
                    const WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
                    const float saturation_proxy = std::clamp(low.coverage * low.density_scale, 0.0f, 2.0f);

                    Render::WeatherCoupling coupling;

                    // Sea-level extinction, per metre: heavy rain alone is worth roughly a
                    // typical valley-fog author density (~0.02/m); the saturation proxy adds a
                    // gentler haze under a filled, dense low deck even before it rains.
                    coupling.fog_density_bias =
                        column.precipitation * 0.020f + saturation_proxy * 0.004f;

                    // Extra Mie scattering coefficient, per metre: rain droplets and haze both
                    // scatter far more than clean, dry air (whose sea-level Mie coefficient is
                    // ~21e-6 by default -- see AtmosphereParams::mie_coefficient), so the bias is
                    // scaled to be a multiple of that baseline under a genuine downpour.
                    coupling.turbidity_bias =
                        column.precipitation * 8.0e-5f + saturation_proxy * 1.5e-5f;

                    // No soak-in/dry-out lag modelled -- wetness tracks precipitation
                    // instantaneously each sample, a named simplification (ground_wetness carries
                    // no memory of its own; a puddle-persistence model is real, unbuilt follow-up
                    // scope, not silently assumed away).
                    coupling.ground_wetness = std::clamp(column.precipitation * 1.4f, 0.0f, 1.0f);

                    coupling.precipitation_intensity = std::clamp(column.precipitation, 0.0f, 1.0f);
                    coupling.wind_east_mps = column.wind_u_mps;
                    coupling.wind_north_mps = column.wind_v_mps;
                    return coupling;
                }
        };
    } // namespace Simulation
} // namespace SushiEngine
