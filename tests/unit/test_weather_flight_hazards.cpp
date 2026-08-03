/**************************************************************************/
/* test_weather_flight_hazards.cpp                                       */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

// Unit_WeatherFlightHazards: W6's icing_risk()/turbulence_intensity() query API
// (docs/slop/weather_and_clouds.md §7 W6) in isolation -- hand-built WeatherColumns and a bare
// provider, the same style test_weather_world_coupling.cpp already uses for the pure
// functions W5 added. Pure host maths; no SushiRuntime needed.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/simulation/weather_flight_hazards.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>
#include <SushiEngine/simulation/weather_wind.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

TEST(Unit_WeatherFlightHazards, ClearSkyHasNoIcingRisk)
{
    const WeatherColumn column{}; // no coverage anywhere.
    EXPECT_FLOAT_EQ(icing_risk(column, 1500.0), 0.0f);
}

TEST(Unit_WeatherFlightHazards, FilledColdLowCloudHasHighIcingRisk)
{
    WeatherColumn column{};
    WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
    low.coverage = 1.0f;
    low.density_scale = 1.0f;
    // ISA alone at 1500 m is a mild +5 C; a -6 C offset (a real cold air mass, not an extreme
    // one) pushes the estimated temperature to about -0.75 C -- inside the classic icing band.
    low.temperature_offset_c = -6.0f;

    const float risk = icing_risk(column, 1500.0);
    EXPECT_GT(risk, 0.5f);
}

TEST(Unit_WeatherFlightHazards, WarmColumnHasNoIcingRiskEvenWhenFilled)
{
    WeatherColumn column{};
    WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
    low.coverage = 1.0f;
    low.density_scale = 1.0f;
    // A strong positive offset pushes the estimated temperature well above freezing even at
    // altitude -- liquid water present, but too warm to accrete ice.
    low.temperature_offset_c = 40.0f;

    EXPECT_FLOAT_EQ(icing_risk(column, 1500.0), 0.0f);
}

TEST(Unit_WeatherFlightHazards, VeryColdColumnHasReducedIcingRisk)
{
    // Below the classic supercooled-liquid band (colder than roughly -12 C, tapering to zero by
    // -20 C), cloud water is predominantly ice crystals, which do not accrete on an airframe the
    // way supercooled liquid droplets do -- risk should fall back off, not keep climbing as it
    // gets colder. Both columns are otherwise identical (full coverage/density at 1500 m); only
    // the offset moves the estimated temperature -- -12 C lands inside the peak band, -24 C
    // lands near the cold shoulder where the trapezoid has almost fully tapered.
    WeatherColumn peak_band{};
    WeatherLevelState& peak_low = peak_band.levels[static_cast<int>(CloudLevel::Low)];
    peak_low.coverage = 1.0f;
    peak_low.density_scale = 1.0f;
    peak_low.temperature_offset_c = -12.0f;

    WeatherColumn very_cold = peak_band;
    very_cold.levels[static_cast<int>(CloudLevel::Low)].temperature_offset_c = -24.0f;

    const float peak_risk = icing_risk(peak_band, 1500.0);
    const float very_cold_risk = icing_risk(very_cold, 1500.0);
    EXPECT_LT(very_cold_risk, peak_risk);
}

TEST(Unit_WeatherFlightHazards, IcingRiskScalesWithLiquidWaterContent)
{
    WeatherColumn thin{};
    WeatherLevelState& thin_low = thin.levels[static_cast<int>(CloudLevel::Low)];
    thin_low.coverage = 0.2f;
    thin_low.density_scale = 0.3f;
    thin_low.temperature_offset_c = -12.0f; // held constant, inside the peak icing band.

    WeatherColumn thick = thin;
    thick.levels[static_cast<int>(CloudLevel::Low)].coverage = 1.0f;
    thick.levels[static_cast<int>(CloudLevel::Low)].density_scale = 1.0f;

    EXPECT_GT(icing_risk(thick, 1500.0), icing_risk(thin, 1500.0));
}

TEST(Unit_WeatherFlightHazards, IcingRiskUsesTheAltitudesOwnLevelBucket)
{
    // A cold, filled low band should not leak icing risk into a query at a high-band altitude
    // that carries no coverage of its own.
    WeatherColumn column{};
    WeatherLevelState& low = column.levels[static_cast<int>(CloudLevel::Low)];
    low.coverage = 1.0f;
    low.density_scale = 1.0f;
    low.temperature_offset_c = 0.0f;

    EXPECT_FLOAT_EQ(icing_risk(column, 9000.0), 0.0f);
}

// The three cases below used to build a `SynopticLayer` and hand it around directly. They now
// go through `IWeatherProvider`, which is not a cosmetic change: the turbulence signal used to
// be scaled by the distance to a *drawn* front (`front_proximity`), and is now scaled by the
// thermal gradient the flow has actually concentrated (`frontal_strength_at`). The assertions
// are about the arithmetic that surrounds that signal, so they transfer unchanged; what could
// not transfer is a test that placed a front where it wanted one.
namespace
{
    constexpr double HAZARD_DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
    constexpr double HAZARD_EARTH_RADIUS_M = 6371000.0;
} // namespace

TEST(Unit_WeatherFlightHazards, TurbulenceIntensityIsTheGustMagnitudeNormalized)
{
    ProceduralWeather weather(/*seed=*/99, HAZARD_EARTH_RADIUS_M);

    const GeodeticPosition position{40.0 * HAZARD_DEGREES_TO_RADIANS,
                                    10.0 * HAZARD_DEGREES_TO_RADIANS};

    const WindSample gust = wind_gust(weather, position, 800.0, 12.0);
    const double expected_magnitude =
        std::sqrt(gust.eastward_mps * gust.eastward_mps + gust.northward_mps * gust.northward_mps);
    const float expected = float(std::min(1.0, expected_magnitude / WIND_GUST_CEILING_MPS));

    EXPECT_NEAR(turbulence_intensity(weather, position, 800.0, 12.0), expected, 1e-6f);
}

TEST(Unit_WeatherFlightHazards, TurbulenceIntensityStaysBounded)
{
    ProceduralWeather weather(/*seed=*/7, HAZARD_EARTH_RADIUS_M);

    // Mid-latitudes, where the core's own baroclinic zone lives, plus a strong injected
    // disturbance to sharpen the gradient near it. Twelve hours so the disturbance has been
    // deformed by the flow rather than still being the symmetric blob it was injected as --
    // a symmetric vortex has a circulation but not much of a temperature gradient.
    const GeodeticPosition position{45.0 * HAZARD_DEGREES_TO_RADIANS, 0.0};
    weather.inject_vorticity(position, /*radius_m=*/700000.0, /*amplitude_mps=*/30.0);
    weather.tick(12.0 * 3600.0, position, 2451545.0);

    // Without this the loop below would be vacuously true: zero gradient gives zero gust gives
    // zero intensity, which is inside [0, 1] and proves nothing.
    ASSERT_GT(weather.frontal_strength_at(position), 0.0)
        << "no thermal gradient here, so the bound below would be trivially satisfied";

    for (double t = 0.0; t < 100.0; t += 7.0)
    {
        const float intensity = turbulence_intensity(weather, position, 500.0, t);
        EXPECT_GE(intensity, 0.0f);
        EXPECT_LE(intensity, 1.0f);
    }
}

TEST(Unit_WeatherFlightHazards, WeatherWindIsExactlyTheProvidersWindPlusGust)
{
    ProceduralWeather weather(/*seed=*/55, HAZARD_EARTH_RADIUS_M);

    const GeodeticPosition position{-20.0 * HAZARD_DEGREES_TO_RADIANS,
                                    55.0 * HAZARD_DEGREES_TO_RADIANS};

    // 2 800 m of the 8 000 m column weather_wind() maps altitude onto; spelled out rather than
    // hard-coded to 0.35 so the two halves cannot silently disagree about the column depth.
    constexpr double COLUMN_TOP_METERS = 8000.0;
    const WindSample base = weather.wind_at(position, 2800.0 / COLUMN_TOP_METERS);
    const WindSample gust = wind_gust(weather, position, 2800.0, 30.0);
    const WindSample combined = weather_wind(weather, position, 2800.0, 30.0);

    EXPECT_NEAR(combined.eastward_mps, base.eastward_mps + gust.eastward_mps, 1e-9);
    EXPECT_NEAR(combined.northward_mps, base.northward_mps + gust.northward_mps, 1e-9);
}
