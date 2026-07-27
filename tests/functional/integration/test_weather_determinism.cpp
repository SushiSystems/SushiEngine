/**************************************************************************/
/* test_weather_determinism.cpp                                          */
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

// Integration_WeatherDeterminism / Integration_WeatherFrontCrossing: the W4 acceptance
// bar from docs/slop/weather_and_clouds.md §7 ("replay is bit-exact") and its visible
// half ("a front visibly crosses the region ... stratus sheet ... clearing"). The first
// test is the weather-domain analogue of Integration_DeterministicReplay and
// Integration_ParticleDeterminism: two independently constructed T1+T2 simulations,
// seeded identically and ticked through the same fixed-step/julian-date stream, must
// reach byte-identical state. The second proves T1's front geometry actually sweeps
// past a fixed point as a low crosses it, and that T2/the cloudscape bridge visibly
// react — the acceptance bar's "visibly crosses", not just "exists as inert state".
// Pure host maths; no SushiRuntime needed (same reasoning as the particle-determinism
// test: this isolates the weather domain from SYCL device dispatch).

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/regional_weather_grid.hpp>
#include <SushiEngine/sim/synoptic_weather.hpp>
#include <SushiEngine/sim/weather_cloudscape_compiler.hpp>
#include <SushiEngine/sim/weather_provider.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr double PLANET_RADIUS_M = 6371000.0;
    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;

    // Runs a ProceduralWeather instance through a fixed dt/julian-date stream and
    // returns its full state for comparison.
    struct RunResult
    {
        SynopticState synoptic;
        WeatherColumn columns[4];
    };

    RunResult run(std::uint64_t seed, int steps, double step_seconds)
    {
        ProceduralWeather weather(seed, PLANET_RADIUS_M, /*grid_nx*/ 16, /*grid_nz*/ 16,
                                  /*grid_domain_m*/ 400000.0, /*grid_tick_seconds*/ 20.0);

        const GeodeticPosition observer{40.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
        const double start_julian_date = 2460500.25; // an arbitrary but fixed epoch

        for (int i = 0; i < steps; ++i)
        {
            const double julian_date = start_julian_date + (double(i) * step_seconds) / 86400.0;
            weather.tick(step_seconds, observer, julian_date);
        }

        RunResult result;
        result.synoptic = weather.synoptic().state();
        const GeodeticPosition sample_points[4] = {
            observer,
            GeodeticPosition{observer.latitude_radians + 2.0 * DEGREES_TO_RADIANS, observer.longitude_radians},
            GeodeticPosition{observer.latitude_radians, observer.longitude_radians + 2.0 * DEGREES_TO_RADIANS},
            GeodeticPosition{-20.0 * DEGREES_TO_RADIANS, -50.0 * DEGREES_TO_RADIANS},
        };
        for (int i = 0; i < 4; ++i)
            result.columns[i] = weather.sample_column(sample_points[i]);
        return result;
    }

    void expect_column_equal(const WeatherColumn& a, const WeatherColumn& b, const char* label)
    {
        for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
        {
            EXPECT_EQ(a.levels[level].coverage, b.levels[level].coverage) << label << " level " << level;
            EXPECT_EQ(a.levels[level].density_scale, b.levels[level].density_scale) << label << " level " << level;
            EXPECT_EQ(a.levels[level].convective_fraction, b.levels[level].convective_fraction)
                << label << " level " << level;
        }
        EXPECT_EQ(a.precipitation, b.precipitation) << label;
        EXPECT_EQ(a.wind_u_mps, b.wind_u_mps) << label;
        EXPECT_EQ(a.wind_v_mps, b.wind_v_mps) << label;
    }
}

TEST(Integration_WeatherDeterminism, SameSeedProducesSameStateAcrossIndependentRuns)
{
    // ~500 ticks at 20s each spans ~2.8 simulated hours: several T2 ticks (20s cadence)
    // and enough T1 evolution that a genesis draw is plausible within the window, which
    // is exactly what this test wants to cover (spawn/retire decisions are RNG-driven
    // and must replay identically too, not just the deterministic advection math).
    constexpr int STEPS = 500;
    constexpr double STEP_SECONDS = 20.0;

    const RunResult first = run(0xA5A5A5A5u, STEPS, STEP_SECONDS);
    const RunResult second = run(0xA5A5A5A5u, STEPS, STEP_SECONDS);

    ASSERT_EQ(first.synoptic.system_count, second.synoptic.system_count);
    EXPECT_EQ(first.synoptic.rng.s0, second.synoptic.rng.s0);
    EXPECT_EQ(first.synoptic.rng.s1, second.synoptic.rng.s1);
    EXPECT_EQ(first.synoptic.next_system_id, second.synoptic.next_system_id);
    EXPECT_EQ(first.synoptic.elapsed_seconds, second.synoptic.elapsed_seconds);
    EXPECT_EQ(first.synoptic.seconds_to_next_genesis, second.synoptic.seconds_to_next_genesis);
    for (int i = 0; i < first.synoptic.system_count; ++i)
    {
        const PressureSystem& a = first.synoptic.systems[i];
        const PressureSystem& b = second.synoptic.systems[i];
        EXPECT_EQ(a.id, b.id) << "system " << i;
        EXPECT_EQ(a.phase, b.phase) << "system " << i;
        EXPECT_EQ(a.age_seconds, b.age_seconds) << "system " << i;
        EXPECT_EQ(a.center_latitude_radians, b.center_latitude_radians) << "system " << i;
        EXPECT_EQ(a.center_longitude_radians, b.center_longitude_radians) << "system " << i;
        EXPECT_EQ(a.heading_radians, b.heading_radians) << "system " << i;
        EXPECT_EQ(a.central_anomaly_hpa, b.central_anomaly_hpa) << "system " << i;
        EXPECT_EQ(a.radius_major_m, b.radius_major_m) << "system " << i;
        EXPECT_EQ(a.radius_minor_m, b.radius_minor_m) << "system " << i;
    }

    expect_column_equal(first.columns[0], second.columns[0], "observer");
    expect_column_equal(first.columns[1], second.columns[1], "north");
    expect_column_equal(first.columns[2], second.columns[2], "east");
    expect_column_equal(first.columns[3], second.columns[3], "far away");

    WeatherCloudscapeCompiler compiler;
    const Render::Cloudscape cloudscape_first = compiler.compile(first.columns[0], Render::Cloudscape{});
    const Render::Cloudscape cloudscape_second = compiler.compile(second.columns[0], Render::Cloudscape{});
    for (int deck = 0; deck < Render::CLOUD_MAX_DECKS; ++deck)
    {
        EXPECT_EQ(cloudscape_first.decks[deck].enabled, cloudscape_second.decks[deck].enabled) << "deck " << deck;
        EXPECT_EQ(cloudscape_first.decks[deck].genus, cloudscape_second.decks[deck].genus) << "deck " << deck;
        EXPECT_EQ(cloudscape_first.decks[deck].coverage_bias, cloudscape_second.decks[deck].coverage_bias)
            << "deck " << deck;
        EXPECT_EQ(cloudscape_first.decks[deck].density_scale, cloudscape_second.decks[deck].density_scale)
            << "deck " << deck;
    }
    EXPECT_EQ(cloudscape_first.evolution_rate, cloudscape_second.evolution_rate);
}

TEST(Integration_WeatherFrontCrossing, ColdFrontSweepsPastAFixedPointAndCloudsRespond)
{
    // A single, hand-placed low tracks due east along 45N at typical synoptic speed,
    // starting 10 degrees of longitude (~780 km at this latitude) west of a fixed
    // observation point — the observer's cold front (design doc's stylized front ray,
    // see synoptic_weather.hpp) sweeps past the point roughly midway through the run.
    ProceduralWeather weather(7ull, PLANET_RADIUS_M, /*grid_nx*/ 20, /*grid_nz*/ 20,
                              /*grid_domain_m*/ 500000.0, /*grid_tick_seconds*/ 60.0);

    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 0.0};

    PressureSystem low;
    low.is_low = true;
    low.center_latitude_radians = 45.0 * DEGREES_TO_RADIANS;
    low.center_longitude_radians = -10.0 * DEGREES_TO_RADIANS;
    low.heading_radians = 1.5707963267948966; // due east
    low.curvature_radians_per_second = 0.0;
    low.speed_mps = 15.0;
    low.central_anomaly_hpa = 28.0;
    low.radius_major_m = 700000.0;
    low.radius_minor_m = 500000.0;
    low.orientation_radians = 0.0;
    low.deepen_seconds = 0.0;    // start Mature immediately: a stable, fully-formed system.
    low.mature_seconds = 60.0 * 3600.0;
    low.fill_seconds = 20.0 * 3600.0;

    // Disable random genesis for this scenario so the only weather in play is the one
    // hand-placed system — an unrelated random spawn mid-run would confound the
    // "the front sweeps past, coverage rises then clears" assertion below without
    // changing anything about whether the model is behaving correctly.
    SynopticState state;
    state.rng = Loop::seed_rng(7ull);
    state.systems[0] = low;
    state.system_count = 1;
    state.next_system_id = 2;
    state.seconds_to_next_genesis = 1.0e9;
    weather.synoptic().set_state(state, PLANET_RADIUS_M);

    constexpr double JULIAN_DATE = 2451545.0;
    constexpr double STEP_SECONDS = 900.0; // 15 minutes
    constexpr int STEPS = 4 * 24;          // 24 simulated hours

    WeatherCloudscapeCompiler compiler;
    std::vector<float> front_cold(STEPS);
    std::vector<float> low_coverage(STEPS);
    std::vector<bool> low_deck_active(static_cast<std::size_t>(STEPS), false);

    for (int i = 0; i < STEPS; ++i)
    {
        weather.tick(STEP_SECONDS, observer, JULIAN_DATE + (double(i) * STEP_SECONDS) / 86400.0);
        front_cold[std::size_t(i)] = weather.synoptic().front_proximity(observer).cold;
        const WeatherColumn column = weather.sample_column(observer);
        low_coverage[std::size_t(i)] = column.levels[static_cast<int>(CloudLevel::Low)].coverage;
        low_deck_active[std::size_t(i)] = compiler.compile(column, Render::Cloudscape{}).decks[0].enabled;
    }

    int peak_index = 0;
    for (int i = 1; i < STEPS; ++i)
        if (front_cold[std::size_t(i)] > front_cold[std::size_t(peak_index)])
            peak_index = i;

    // The front must actually sweep past mid-run, not sit permanently near (a static
    // model) or never near (a broken heading/speed) the observer.
    EXPECT_GT(front_cold[std::size_t(peak_index)], 0.5f);
    EXPECT_GT(peak_index, STEPS / 8);
    EXPECT_LT(peak_index, STEPS - STEPS / 8);
    EXPECT_LT(front_cold[0], 0.2f) << "front should not already be on top of the observer at t=0";
    EXPECT_LT(front_cold[std::size_t(STEPS - 1)], front_cold[std::size_t(peak_index)])
        << "front should have moved past (clearing) by the end of the run";

    // The cloudscape bridge must visibly react: low-level coverage measurably higher
    // near the front's closest approach than at the start, where the system is still
    // far to the west, and the compiled deck stack (what CloudscapeCompilePass, T3,
    // would actually bake) is enabled right at that moment — closing the loop from
    // "T2 changed" to "the manual-authoring-equivalent Cloudscape actually changed too".
    EXPECT_GT(low_coverage[std::size_t(peak_index)], low_coverage[0] + 0.05f);
    EXPECT_TRUE(low_deck_active[std::size_t(peak_index)]);
}
