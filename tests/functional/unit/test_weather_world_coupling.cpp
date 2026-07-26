/**************************************************************************/
/* test_weather_world_coupling.cpp                                       */
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

// Unit_WeatherWorldCoupling: docs/slop/weather_and_clouds.md §5.3/W5's world-coupling bridge
// (WeatherWorldCoupling, weather_wind(), and WeatherCloudscapeCompiler's cloud-base
// darkening addition), in isolation from T1/T2 -- hand-built WeatherColumns and a bare
// SynopticLayer, exercising exactly the pure functions RuntimeSimulation::extract() calls.
// Pure host maths; no SushiRuntime needed, same reasoning as the other weather tests.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/synoptic_weather.hpp>
#include <SushiEngine/sim/weather_cloudscape_compiler.hpp>
#include <SushiEngine/sim/weather_wind.hpp>
#include <SushiEngine/sim/weather_world_coupling.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

TEST(Unit_WeatherWorldCoupling, ClearColumnCompilesToAllZeroCoupling)
{
    const WeatherColumn column{}; // every field default: no coverage, no precipitation.
    WeatherWorldCoupling coupling;
    const Render::WeatherCoupling result = coupling.compile(column);

    EXPECT_FLOAT_EQ(result.fog_density_bias, 0.0f);
    EXPECT_FLOAT_EQ(result.turbidity_bias, 0.0f);
    EXPECT_FLOAT_EQ(result.ground_wetness, 0.0f);
    EXPECT_FLOAT_EQ(result.precipitation_intensity, 0.0f);
    EXPECT_FLOAT_EQ(result.wind_east_mps, 0.0f);
    EXPECT_FLOAT_EQ(result.wind_north_mps, 0.0f);
}

TEST(Unit_WeatherWorldCoupling, PrecipitationRaisesFogTurbidityAndWetnessTogether)
{
    // One cause (precipitation), every symptom (design doc §7's acceptance bar): a single
    // rained-out column should simultaneously thicken the fog, raise turbidity, and wet the
    // ground -- not just one of the three.
    WeatherColumn column{};
    column.precipitation = 0.8f;
    column.wind_u_mps = 5.0f;
    column.wind_v_mps = -3.0f;

    WeatherWorldCoupling coupling;
    const Render::WeatherCoupling result = coupling.compile(column);

    EXPECT_GT(result.fog_density_bias, 0.0f);
    EXPECT_GT(result.turbidity_bias, 0.0f);
    EXPECT_GT(result.ground_wetness, 0.0f);
    EXPECT_FLOAT_EQ(result.precipitation_intensity, 0.8f);
    EXPECT_FLOAT_EQ(result.wind_east_mps, 5.0f);
    EXPECT_FLOAT_EQ(result.wind_north_mps, -3.0f);

    // Heavier rain must never produce a smaller symptom -- monotonicity, not just "nonzero".
    WeatherColumn heavier = column;
    heavier.precipitation = 1.0f;
    const Render::WeatherCoupling heavier_result = coupling.compile(heavier);
    EXPECT_GE(heavier_result.fog_density_bias, result.fog_density_bias);
    EXPECT_GE(heavier_result.turbidity_bias, result.turbidity_bias);
    EXPECT_GE(heavier_result.ground_wetness, result.ground_wetness);
}

TEST(Unit_WeatherWorldCoupling, GroundWetnessStaysClamped)
{
    WeatherColumn column{};
    column.precipitation = 1.0f; // the scale factor alone (1.4x) would overshoot 1 unclamped.

    WeatherWorldCoupling coupling;
    const Render::WeatherCoupling result = coupling.compile(column);
    EXPECT_LE(result.ground_wetness, 1.0f);
    EXPECT_GE(result.ground_wetness, 0.0f);
}

TEST(Unit_WeatherCloudscapeCompiler, PrecipitationDarkensOnlyTheLowDeck)
{
    // The design doc's literal recipe ("density += density * weather.a"), applied to the low
    // band only -- the band T2's moisture closure actually rains from.
    WeatherColumn dry{};
    dry.levels[static_cast<int>(CloudLevel::Low)].coverage = 0.6f;
    dry.levels[static_cast<int>(CloudLevel::Low)].density_scale = 0.5f;
    dry.levels[static_cast<int>(CloudLevel::Mid)].coverage = 0.4f;
    dry.levels[static_cast<int>(CloudLevel::Mid)].density_scale = 0.5f;

    WeatherColumn rainy = dry;
    rainy.precipitation = 1.0f;

    WeatherCloudscapeCompiler compiler;
    const Render::Cloudscape dry_clouds = compiler.compile(dry);
    const Render::Cloudscape rainy_clouds = compiler.compile(rainy);

    EXPECT_GT(rainy_clouds.decks[0].density_scale, dry_clouds.decks[0].density_scale)
        << "low deck should darken under precipitation";
    EXPECT_FLOAT_EQ(rainy_clouds.decks[1].density_scale, dry_clouds.decks[1].density_scale)
        << "mid deck should be unaffected by the low band's own rain";
}

TEST(Unit_WeatherWind, MatchesTheAnalyticFieldWhenNoFrontIsNearby)
{
    // Far from any system, front proximity is ~0, so the perturbation term should vanish and
    // weather_wind() should reduce to T1's own analytic wind_at() exactly.
    SynopticLayer synoptic;
    synoptic.seed(1234ull);

    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
    const GeodeticPosition far_from_everything{-60.0 * DEGREES_TO_RADIANS, 170.0 * DEGREES_TO_RADIANS};

    const WindSample analytic = synoptic.wind_at(far_from_everything, /*level_fraction*/ 0.2);
    const WindSample sampled = weather_wind(synoptic, far_from_everything, /*altitude_meters*/ 1600.0,
                                            /*time_seconds*/ 42.0);

    EXPECT_NEAR(sampled.eastward_mps, analytic.eastward_mps, 1e-9);
    EXPECT_NEAR(sampled.northward_mps, analytic.northward_mps, 1e-9);
}

TEST(Unit_WeatherWind, GustsIntensifyNearAFront)
{
    SynopticLayer synoptic;
    synoptic.seed(1ull);

    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
    PressureSystem low;
    low.is_low = true;
    low.center_latitude_radians = 45.0 * DEGREES_TO_RADIANS;
    low.center_longitude_radians = 0.0;
    low.heading_radians = 1.5707963267948966; // due east
    low.speed_mps = 15.0;
    low.central_anomaly_hpa = 28.0;
    low.radius_major_m = 700000.0;
    low.radius_minor_m = 500000.0;
    low.deepen_seconds = 0.0;
    low.mature_seconds = 60.0 * 3600.0;
    low.fill_seconds = 20.0 * 3600.0;
    ASSERT_TRUE(synoptic.add_system(low));

    // A point constructed to sit exactly on the cold front ray (see synoptic_weather.hpp's
    // front_proximity: a fixed-angle ray from the low's centre along its heading), 30% of the
    // way along it -- geometry worked out from the same formula, not a guess, so the proximity
    // assertion below is a sanity check on the construction rather than the real assertion.
    const GeodeticPosition on_front{47.1372 * DEGREES_TO_RADIANS, -3.0217 * DEGREES_TO_RADIANS};
    const GeodeticPosition clear_air{-60.0 * DEGREES_TO_RADIANS, 170.0 * DEGREES_TO_RADIANS};
    ASSERT_GT(synoptic.front_proximity(on_front).cold, 0.9f) << "test point must actually sit near the front";

    const auto gust_magnitude = [&](const GeodeticPosition& position)
    {
        const WindSample analytic = synoptic.wind_at(position, 0.1);
        const WindSample sampled = weather_wind(synoptic, position, 500.0, 10.0);
        const double du = sampled.eastward_mps - analytic.eastward_mps;
        const double dv = sampled.northward_mps - analytic.northward_mps;
        return std::sqrt(du * du + dv * dv);
    };

    EXPECT_GT(gust_magnitude(on_front), gust_magnitude(clear_air));
}
