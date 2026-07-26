/**************************************************************************/
/* test_metar_parser.cpp                                                 */
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

// Unit_MetarParser: W6's real METAR text parser (docs/slop/weather_and_clouds.md §5.4/§7)
// against hand-picked, realistic report strings -- wind (including calm and variable), cloud
// layers, temperature/dewpoint (including negative), present-weather/precipitation intensity,
// and the WeatherColumn translation. Pure host string parsing; no SushiRuntime needed.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/metar_parser.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

TEST(Unit_MetarParser, ParsesAFairWeatherReport)
{
    const MetarReport report = parse_metar("METAR KJFK 261851Z 27015G25KT 10SM FEW250 24/12 A3005");
    ASSERT_TRUE(report.valid);

    EXPECT_FALSE(report.wind_calm);
    EXPECT_FALSE(report.wind_variable_direction);
    EXPECT_DOUBLE_EQ(report.wind_direction_degrees, 270.0);
    EXPECT_NEAR(report.wind_speed_mps, 15.0 * 0.514444, 1e-6);
    EXPECT_NEAR(report.wind_gust_mps, 25.0 * 0.514444, 1e-6);

    ASSERT_EQ(report.layer_count, 1);
    EXPECT_EQ(report.layers[0].cover, 'F');
    EXPECT_NEAR(report.layers[0].base_meters, 250.0 * 100.0 * 0.3048, 1e-6);

    ASSERT_TRUE(report.has_temperature);
    EXPECT_DOUBLE_EQ(report.temperature_c, 24.0);
    EXPECT_DOUBLE_EQ(report.dewpoint_c, 12.0);

    EXPECT_FLOAT_EQ(report.precipitation_intensity, 0.0f);
    EXPECT_FALSE(report.thunderstorm);
}

TEST(Unit_MetarParser, ParsesNegativeTemperatureAndCalmWind)
{
    const MetarReport report = parse_metar("METAR ENGM 010600Z 00000KT 9999 BKN020 M02/M08 Q1013");
    ASSERT_TRUE(report.valid);

    EXPECT_TRUE(report.wind_calm);
    ASSERT_TRUE(report.has_temperature);
    EXPECT_DOUBLE_EQ(report.temperature_c, -2.0);
    EXPECT_DOUBLE_EQ(report.dewpoint_c, -8.0);

    ASSERT_EQ(report.layer_count, 1);
    EXPECT_EQ(report.layers[0].cover, 'B');
}

TEST(Unit_MetarParser, ParsesVariableWindAndVerticalVisibility)
{
    const MetarReport report = parse_metar("SPECI LFPG 121230Z VRB03KT 1/4SM FG VV002 05/05");
    ASSERT_TRUE(report.valid);
    EXPECT_TRUE(report.wind_variable_direction);
    EXPECT_NEAR(report.wind_speed_mps, 3.0 * 0.514444, 1e-6);

    ASSERT_EQ(report.layer_count, 1);
    EXPECT_EQ(report.layers[0].cover, 'V');
    EXPECT_NEAR(report.layers[0].base_meters, 2.0 * 100.0 * 0.3048, 1e-6);
}

TEST(Unit_MetarParser, HeavyThunderstormRainOutranksLightDrizzle)
{
    const MetarReport light = parse_metar("METAR TEST 010000Z 18010KT 6SM -DZ BKN012 15/14 A2992");
    const MetarReport heavy = parse_metar("METAR TEST 010000Z 18025G35KT 3SM +TSRA BKN008 OVC015CB 18/17 Q0995");

    ASSERT_TRUE(light.valid);
    ASSERT_TRUE(heavy.valid);
    EXPECT_GT(light.precipitation_intensity, 0.0f);
    EXPECT_GT(heavy.precipitation_intensity, light.precipitation_intensity);
    EXPECT_TRUE(heavy.thunderstorm);
    EXPECT_FALSE(light.thunderstorm);
}

TEST(Unit_MetarParser, VicinityWeatherWeighsLessThanOverheadWeather)
{
    const MetarReport overhead = parse_metar("METAR TEST 010000Z 09008KT 5SM RA SCT018 16/13 A3000");
    const MetarReport vicinity = parse_metar("METAR TEST 010000Z 09008KT 8SM VCSH SCT018 16/13 A3000");

    ASSERT_TRUE(overhead.valid);
    ASSERT_TRUE(vicinity.valid);
    EXPECT_GT(overhead.precipitation_intensity, vicinity.precipitation_intensity);
    EXPECT_GT(vicinity.precipitation_intensity, 0.0f);
}

TEST(Unit_MetarParser, SkyClearReportHasNoLayers)
{
    const MetarReport report = parse_metar("METAR TEST 010000Z 00000KT CAVOK 20/10 A3000");
    ASSERT_TRUE(report.valid);
    EXPECT_EQ(report.layer_count, 0);
    // CAVOK itself is not a recognized group (deliberately -- see the file docs, only the
    // groups the bridge can use are decoded), so `sky_clear` is not asserted true here; the
    // report still parses successfully from its wind/temperature groups alone.
}

TEST(Unit_MetarParser, EmptyOrGarbageStringIsNotValid)
{
    EXPECT_FALSE(parse_metar("").valid);
    EXPECT_FALSE(parse_metar("NOT A METAR REPORT AT ALL").valid);
}

TEST(Unit_MetarParser, MetarToWeatherColumnPlacesLayerInItsAltitudeBucket)
{
    const MetarReport report = parse_metar("METAR TEST 010000Z 27012KT 6SM BKN008 18/16 A3000");
    ASSERT_TRUE(report.valid);
    ASSERT_EQ(report.layer_count, 1);
    ASSERT_LT(report.layers[0].base_meters, CLOUD_LEVEL_LOW_CEILING_METERS);

    const WeatherColumn column = metar_to_weather_column(report);
    EXPECT_GT(column.levels[static_cast<int>(CloudLevel::Low)].coverage, 0.0f);
    EXPECT_FLOAT_EQ(column.levels[static_cast<int>(CloudLevel::Mid)].coverage, 0.0f);
    EXPECT_FLOAT_EQ(column.levels[static_cast<int>(CloudLevel::High)].coverage, 0.0f);
}

TEST(Unit_MetarParser, MetarToWeatherColumnDerivesWindTowardVectorFromFromDirection)
{
    // A due-west wind (blowing FROM 270) blows TOWARD the east: eastward positive, northward ~0.
    const MetarReport report = parse_metar("METAR TEST 010000Z 27020KT 10SM SKC 15/05 A3000");
    ASSERT_TRUE(report.valid);

    const WeatherColumn column = metar_to_weather_column(report);
    EXPECT_GT(column.wind_u_mps, 0.0f);
    EXPECT_NEAR(column.wind_v_mps, 0.0f, 1e-3f);
}

TEST(Unit_MetarParser, MetarToWeatherColumnSurfaceTemperatureOnlyInformsTheLowBand)
{
    const MetarReport report = parse_metar("METAR TEST 010000Z 00000KT 10SM FEW250 SCT180 M10/M15 A3000");
    ASSERT_TRUE(report.valid);
    ASSERT_EQ(report.layer_count, 2);

    const WeatherColumn column = metar_to_weather_column(report);
    // Neither reported layer is in the Low band, but the surface temperature reading must not
    // silently leak into Mid/High, which METAR says nothing about.
    EXPECT_FLOAT_EQ(column.levels[static_cast<int>(CloudLevel::Low)].temperature_offset_c,
                    float(-10.0 - ISA_SEA_LEVEL_TEMPERATURE_C));
    EXPECT_FLOAT_EQ(column.levels[static_cast<int>(CloudLevel::Mid)].temperature_offset_c, 0.0f);
    EXPECT_FLOAT_EQ(column.levels[static_cast<int>(CloudLevel::High)].temperature_offset_c, 0.0f);
}

TEST(Unit_MetarParser, InvalidReportProducesAllClearColumnNotFabricatedWeather)
{
    const WeatherColumn column = metar_to_weather_column(parse_metar(""));
    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
    {
        EXPECT_FLOAT_EQ(column.levels[level].coverage, 0.0f);
        EXPECT_FLOAT_EQ(column.levels[level].density_scale, 0.0f);
    }
    EXPECT_FLOAT_EQ(column.precipitation, 0.0f);
}
