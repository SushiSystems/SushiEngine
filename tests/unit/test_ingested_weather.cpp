/**************************************************************************/
/* test_ingested_weather.cpp                                              */
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

// Unit_IngestedWeather: W6's IngestedWeather provider (docs/design/weather_and_clouds.md §5.4/§7)
// -- the three-stage GRIB-background/METAR-station blend, and the LSP substitutability proof
// the task explicitly asks for: swap it in for any other IWeatherProvider and everything
// downstream (WeatherCloudscapeCompiler, WeatherWorldCoupling) keeps working with zero changes.

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/simulation/ingested_weather.hpp>
#include <SushiEngine/simulation/weather_cloudscape_compiler.hpp>
#include <SushiEngine/simulation/weather_world_coupling.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;

    GeodeticPosition degrees(double lat, double lon)
    {
        return GeodeticPosition{lat * DEGREES_TO_RADIANS, lon * DEGREES_TO_RADIANS};
    }
} // namespace

TEST(Unit_IngestedWeather, FarFromEveryStationReturnsTheBackground)
{
    IngestedWeather provider;
    WeatherColumn background{};
    background.precipitation = 0.2f;
    background.levels[static_cast<int>(CloudLevel::Mid)].coverage = 0.4f;
    provider.set_background(background);

    provider.add_station(degrees(40.0, -73.0), "METAR KJFK 261851Z 27015G25KT 10SM FEW250 24/12 A3005");

    const WeatherColumn sample = provider.sample_column(degrees(-10.0, 150.0)); // far side of the planet.
    EXPECT_FLOAT_EQ(sample.precipitation, background.precipitation);
    EXPECT_FLOAT_EQ(sample.levels[static_cast<int>(CloudLevel::Mid)].coverage,
                    background.levels[static_cast<int>(CloudLevel::Mid)].coverage);
}

TEST(Unit_IngestedWeather, RightAtAStationReturnsThatStationsMetarColumn)
{
    IngestedWeather provider;
    provider.set_background(WeatherColumn{}); // clear background.

    const GeodeticPosition station_position = degrees(40.6398, -73.7789); // KJFK.
    provider.add_station(station_position,
        "METAR KJFK 010000Z 18025G35KT 3SM +TSRA BKN008 OVC015CB 18/17 Q0995");

    const WeatherColumn sample = provider.sample_column(station_position);
    EXPECT_GT(sample.precipitation, 0.5f) << "directly over a heavy-thunderstorm station";
    EXPECT_GT(sample.levels[static_cast<int>(CloudLevel::Low)].coverage, 0.0f);
}

TEST(Unit_IngestedWeather, BetweenTheTwoRadiiBlendsMonotonically)
{
    IngestedWeather provider;
    WeatherColumn background{};
    background.precipitation = 0.0f;
    provider.set_background(background);

    const GeodeticPosition station_position = degrees(0.0, 0.0);
    WeatherColumn station_column{};
    station_column.precipitation = 1.0f;
    provider.add_station_column(station_position, station_column);

    // Walk outward from the station: precipitation should fall from 1 toward 0 and never rise
    // again as distance increases -- a real blend, not a step function or a reversal.
    float previous = provider.sample_column(station_position).precipitation;
    EXPECT_FLOAT_EQ(previous, 1.0f);

    // ~5.5 km per step at the equator: comfortably crosses both NEAR_STATION_RADIUS_METERS
    // (15 km) and FAR_BACKGROUND_RADIUS_METERS (60 km) within the loop below.
    const double step_degrees = 0.05;
    for (int i = 1; i <= 25; ++i)
    {
        const GeodeticPosition probe = degrees(0.0, step_degrees * double(i));
        const float current = provider.sample_column(probe).precipitation;
        EXPECT_LE(current, previous + 1e-6f);
        previous = current;
    }
    EXPECT_FLOAT_EQ(previous, background.precipitation);
}

TEST(Unit_IngestedWeather, NearestOfMultipleStationsWins)
{
    IngestedWeather provider;
    provider.set_background(WeatherColumn{});

    WeatherColumn near_column{};
    near_column.precipitation = 0.9f;
    WeatherColumn far_column{};
    far_column.precipitation = 0.1f;

    provider.add_station_column(degrees(10.0, 10.0), far_column);
    provider.add_station_column(degrees(10.01, 10.0), near_column); // ~1.1 km away.

    const WeatherColumn sample = provider.sample_column(degrees(10.01, 10.0));
    EXPECT_FLOAT_EQ(sample.precipitation, near_column.precipitation);
}

TEST(Unit_IngestedWeather, NoStationsAtAllIsPureBackground)
{
    IngestedWeather provider;
    WeatherColumn background{};
    background.wind_u_mps = 3.0f;
    background.wind_v_mps = -1.5f;
    provider.set_background(background);

    const WeatherColumn sample = provider.sample_column(degrees(51.5, -0.1));
    EXPECT_FLOAT_EQ(sample.wind_u_mps, background.wind_u_mps);
    EXPECT_FLOAT_EQ(sample.wind_v_mps, background.wind_v_mps);
}

// The task brief's explicit ask: prove IngestedWeather is genuinely LSP-substitutable for any
// other IWeatherProvider -- the exact assertions W4/W5 already run against StaticWeather/
// ProceduralWeather output, run here against IngestedWeather instead, through the identical
// downstream compilers with zero changes to either.
TEST(Unit_IngestedWeather, SubstitutesForAnyOtherProviderThroughTheSharedCompilers)
{
    IngestedWeather provider;
    provider.set_background(WeatherColumn{});
    provider.add_station(degrees(48.8566, 2.3522),
        "METAR LFPG 010000Z 27012G20KT 6SM BKN008 OVC015 08/06 A2990");

    const IWeatherProvider& seam = provider; // exactly how RuntimeSimulation holds any provider.
    const WeatherColumn column = seam.sample_column(degrees(48.8566, 2.3522));

    WeatherCloudscapeCompiler cloudscape_compiler;
    const Render::Cloudscape clouds = cloudscape_compiler.compile(column, Render::Cloudscape{});
    EXPECT_TRUE(clouds.enabled);
    EXPECT_TRUE(clouds.decks[0].enabled) << "a filled, cold low deck should compile to an enabled low deck";

    WeatherWorldCoupling world_coupling;
    const Render::WeatherCoupling coupling = world_coupling.compile(column);
    EXPECT_GE(coupling.fog_density_bias, 0.0f);
    EXPECT_GE(coupling.ground_wetness, 0.0f);
    EXPECT_LE(coupling.ground_wetness, 1.0f);

    // Same station, queried from far away: falls back to the (clear) background, so the
    // compiled cloudscape must read as an ordinary clear-column compile -- proving this
    // provider's own internal blend, not just that it returns *some* WeatherColumn.
    const WeatherColumn far_column = seam.sample_column(degrees(-33.8688, 151.2093)); // Sydney.
    const Render::Cloudscape far_clouds = cloudscape_compiler.compile(far_column, Render::Cloudscape{});
    EXPECT_FALSE(far_clouds.decks[0].enabled);
}
