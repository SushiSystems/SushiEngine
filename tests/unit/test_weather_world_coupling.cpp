/**************************************************************************/
/* test_weather_world_coupling.cpp                                        */
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

// Unit_WeatherWorldCoupling: docs/slop/weather_and_clouds.md §5.3/W5's world-coupling bridge
// (WeatherWorldCoupling, weather_wind(), and WeatherCloudscapeCompiler's cloud-base
// darkening addition), in isolation from T1/T2 -- hand-built WeatherColumns and a bare
// provider, exercising exactly the pure functions RuntimeSimulation::extract() calls.
// Pure host maths; no SushiRuntime needed, same reasoning as the other weather tests.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/simulation/weather_cloudscape_compiler.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>
#include <SushiEngine/simulation/weather_wind.hpp>
#include <SushiEngine/simulation/weather_world_coupling.hpp>

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

TEST(Unit_WeatherWorldCoupling, OvercastAloftIsNotFogButCloudOnTheGroundIs)
{
    // Driving fog from the low deck's coverage times its density turns an ordinary grey
    // overcast at 1 200 m into ~0.008/m of extinction -- a ~370 m whiteout under a sky you can
    // see the ground perfectly well beneath. Coverage aloft says nothing about the air at the
    // surface; cloud *base* does.
    WeatherColumn overcast{};
    overcast.levels[static_cast<int>(CloudLevel::Low)].coverage = 1.0f;
    overcast.levels[static_cast<int>(CloudLevel::Low)].density_scale = 2.0f;
    overcast.cloud_base_m = 1200.0f;

    WeatherWorldCoupling coupling;
    EXPECT_FLOAT_EQ(coupling.compile(overcast).fog_density_bias, 0.0f)
        << "a dry overcast sky overhead is not fog on the ground";

    // The same amount of cloud, sitting on the ground, *is* fog -- that is what fog is.
    WeatherColumn grounded = overcast;
    grounded.cloud_base_m = 1.0f;
    EXPECT_GT(coupling.compile(grounded).fog_density_bias, 0.0f);

    // And it ramps rather than switching, so driving up a hill into the cloud base is a
    // transition rather than a step.
    WeatherColumn halfway = overcast;
    halfway.cloud_base_m = 150.0f;
    const float partial = coupling.compile(halfway).fog_density_bias;
    EXPECT_GT(partial, 0.0f);
    EXPECT_LT(partial, coupling.compile(grounded).fog_density_bias);
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
    const Render::Cloudscape dry_clouds = compiler.compile(dry, Render::Cloudscape{});
    const Render::Cloudscape rainy_clouds = compiler.compile(rainy, Render::Cloudscape{});

    EXPECT_GT(rainy_clouds.decks[0].density_scale, dry_clouds.decks[0].density_scale)
        << "low deck should darken under precipitation";
    EXPECT_FLOAT_EQ(rainy_clouds.decks[1].density_scale, dry_clouds.decks[1].density_scale)
        << "mid deck should be unaffected by the low band's own rain";
}

// The two wind cases below were written against a `SynopticLayer` whose fronts were drawn rays
// from a placed low's centre, which meant a test could put a point *on* a front by solving the
// ray's geometry. A dynamical core draws nothing: a front is wherever the flow has concentrated
// a thermal gradient, and where that is depends on the flow. Both cases are restated to assert
// the relation `wind_gust` actually implements -- the perturbation is proportional to the local
// gradient, bounded by the ceiling -- which is the property the drawn-front construction was
// only ever a way of reaching.
namespace
{
    constexpr double WIND_DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
    constexpr double WIND_EARTH_RADIUS_M = 6371000.0;

    /** @brief Mean perturbation magnitude at a point, averaged over the gust's own phase. */
    double mean_gust_magnitude(const IWeatherProvider& weather, const GeodeticPosition& position)
    {
        // Averaged over time rather than sampled once, because the perturbation is two
        // sinusoids of a position/time phase: a single sample can land near a zero crossing at
        // a strongly frontal point and read lower than a quiet one, which would make an honest
        // comparison fail for a reason that has nothing to do with fronts.
        double total = 0.0;
        constexpr int SAMPLES = 64;
        for (int i = 0; i < SAMPLES; ++i)
        {
            const WindSample gust = wind_gust(weather, position, 500.0, double(i) * 3.7);
            total += std::sqrt(gust.eastward_mps * gust.eastward_mps +
                               gust.northward_mps * gust.northward_mps);
        }
        return total / double(SAMPLES);
    }
} // namespace

TEST(Unit_WeatherWind, PerturbationIsBoundedByTheLocalThermalGradient)
{
    // `weather_wind()` is exactly `wind_at() + wind_gust()`, and the gust is the ceiling scaled
    // by the gradient as a fraction of a full front. So the departure from the provider's own
    // wind can never exceed that fraction of the ceiling. Stated as a bound rather than as an
    // equality far from any system, because such an equality would depend on there being
    // somewhere with no weather at all.
    ProceduralWeather weather(/*seed=*/1234, WIND_EARTH_RADIUS_M);

    const GeodeticPosition position{-60.0 * WIND_DEGREES_TO_RADIANS,
                                    170.0 * WIND_DEGREES_TO_RADIANS};

    constexpr double COLUMN_TOP_METERS = 8000.0;
    const WindSample base = weather.wind_at(position, 1600.0 / COLUMN_TOP_METERS);
    const WindSample sampled = weather_wind(weather, position, /*altitude_meters*/ 1600.0,
                                            /*time_seconds*/ 42.0);

    const double du = sampled.eastward_mps - base.eastward_mps;
    const double dv = sampled.northward_mps - base.northward_mps;
    const double departure = std::sqrt(du * du + dv * dv);

    constexpr double FULLY_FRONTAL_K_PER_100KM = 5.0; // mirrors weather_wind.hpp's own constant.
    const double fraction =
        std::min(weather.frontal_strength_at(position) / FULLY_FRONTAL_K_PER_100KM, 1.0);
    // sqrt(2) because the two components are independent sinusoids of the same amplitude.
    const double bound = WIND_GUST_CEILING_MPS * fraction * 1.4142135623730951 + 1e-9;

    EXPECT_LE(departure, bound);
}

TEST(Unit_WeatherWind, GustsIntensifyWhereTheThermalGradientIsSharper)
{
    ProceduralWeather weather(/*seed=*/1, WIND_EARTH_RADIUS_M);

    // Mid-latitudes with an injected disturbance against the deep tropics. The core's own mean
    // state already puts its baroclinic zone at the former and near-nothing at the latter, and
    // twelve hours of evolution lets the injected anomaly deform the temperature field rather
    // than remain the symmetric blob it was injected as.
    const GeodeticPosition frontal{45.0 * WIND_DEGREES_TO_RADIANS, 0.0};
    const GeodeticPosition quiet{2.0 * WIND_DEGREES_TO_RADIANS,
                                 170.0 * WIND_DEGREES_TO_RADIANS};
    weather.inject_vorticity(frontal, /*radius_m=*/700000.0, /*amplitude_mps=*/28.0);
    weather.tick(12.0 * 3600.0, frontal, 2451545.0);

    // The premise, asserted rather than assumed: if the core did not actually concentrate a
    // gradient here the comparison below would be measuring noise.
    ASSERT_GT(weather.frontal_strength_at(frontal), weather.frontal_strength_at(quiet))
        << "no sharper gradient at the disturbed point; the comparison has no premise";

    EXPECT_GT(mean_gust_magnitude(weather, frontal), mean_gust_magnitude(weather, quiet));
}
