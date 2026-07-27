/**************************************************************************/
/* test_atmosphere_nest.cpp                                               */
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

// Unit_AtmosphereNest: the regional nest's host-side half (docs/slop/atmosphere_system.md §6).
//
// **What these cases are actually for.** The nest itself runs in ten compute shaders, which a
// unit test cannot execute. What it *can* do is pin the relations those shaders are written in
// terms of, because `render/shaders/atmosphere_nest_common.glsl` mirrors them formula for
// formula from `atmosphere_nest.hpp` — GLSL cannot include a C++ header, so the mirror is the
// price of running the model on the GPU at all. A mirror is only trustworthy if one side is
// checkable, so every relation below is asserted against a value a meteorology text will
// confirm rather than against whatever the code happens to return. If the C++ side drifts,
// these fail; if the GLSL side drifts from the C++ side, the sky stops matching what gameplay
// reports, which is the symptom to look for.
//
// The second group covers the seam the GPU state reaches the simulation through: the mirror's
// cold start, and the transcription that turns a readback record into the column contract the
// whole gameplay bridge already speaks in.
//
// This replaces `test_weather_determinism.cpp`, retired with the bit-exact replay guarantee it
// proved — see the design doc §0 and §14, which give that up deliberately in exchange for an
// atmosphere that can leave the fixed-tick domain and run on the GPU.

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/atmosphere_forcing_buffer.hpp>
#include <SushiEngine/sim/weather_field_buffer.hpp>
#include <SushiEngine/sim/weather_provider.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr double DEGREES_TO_RADIANS = 3.14159265358979323846 / 180.0;
    constexpr double EARTH_RADIUS_M = 6371000.0;
} // namespace

// ---- The base state, against the International Standard Atmosphere ----------------------

TEST(Unit_AtmosphereNest, BaseStateMatchesTheStandardAtmosphere)
{
    const Render::AtmosphereParameters p{};

    // Sea level, by definition of the defaults.
    EXPECT_NEAR(Render::atmosphere_base_temperature(p, 0.0f), 288.15f, 1e-3f);
    EXPECT_NEAR(Render::atmosphere_base_pressure(p, 0.0f), 101325.0f, 1.0f);
    // rho = p / (R T). The ISA figure is 1.225 kg/m^3.
    EXPECT_NEAR(Render::atmosphere_base_density(p, 0.0f), 1.225f, 5e-3f);

    // The tropopause: 11 km, -56.5 C, and a pressure the ISA tables give as 22 632 Pa. Half a
    // percent of tolerance covers the gas constant's last digits, not a wrong profile.
    EXPECT_NEAR(Render::atmosphere_base_temperature(p, 11000.0f), 216.65f, 1e-2f);
    EXPECT_NEAR(Render::atmosphere_base_pressure(p, 11000.0f), 22632.0f, 22632.0f * 0.005f);

    // Above it the layer is isothermal, so temperature stops falling but pressure does not.
    EXPECT_NEAR(Render::atmosphere_base_temperature(p, 16000.0f), 216.65f, 1e-2f);
    EXPECT_LT(Render::atmosphere_base_pressure(p, 16000.0f),
              Render::atmosphere_base_pressure(p, 11000.0f));
}

TEST(Unit_AtmosphereNest, PotentialTemperatureIsTheQuantityBuoyancyIsWrittenIn)
{
    const Render::AtmosphereParameters p{};

    // Exner is 1 at the reference pressure by construction, and just above it at the surface
    // because standard sea-level pressure is slightly above 1000 hPa.
    EXPECT_NEAR(Render::atmosphere_exner(p, 0.0f), 1.00378f, 1e-4f);
    // theta = T / Pi, so at the surface it is a shade below the temperature itself.
    EXPECT_NEAR(Render::atmosphere_base_theta(p, 0.0f), 288.15f / 1.00378f, 1e-2f);

    // The point of using potential temperature at all: a dry adiabatic atmosphere has a
    // *constant* theta with height, so buoyancy is a difference from the local base state
    // rather than a difference that has to be corrected for altitude first. The ISA's 6.5 K/km
    // is stabler than dry adiabatic (9.8 K/km), so theta must increase upward.
    EXPECT_GT(Render::atmosphere_base_theta(p, 5000.0f), Render::atmosphere_base_theta(p, 0.0f));
    EXPECT_GT(Render::atmosphere_base_theta(p, 10000.0f),
              Render::atmosphere_base_theta(p, 5000.0f));
}

TEST(Unit_AtmosphereNest, SaturationFollowsMagnusNotAThreshold)
{
    // The relation the shipped system replaced with `if (humidity > 0.85)` (§1.3). Saturation
    // vapour pressure at 20 C is ~2.34 kPa; at 0 C it is ~0.61 kPa. That it nearly *quadruples*
    // over twenty degrees is the whole reason a relative-humidity threshold cannot place a
    // cloud base where a rising parcel actually reaches saturation.
    EXPECT_NEAR(Render::atmosphere_saturation_pressure(273.15f), 611.2f, 1.0f);
    EXPECT_NEAR(Render::atmosphere_saturation_pressure(293.15f), 2337.0f, 10.0f);

    // Saturation mixing ratio at 20 C and sea level: ~14.5 g/kg.
    EXPECT_NEAR(Render::atmosphere_saturation_mixing_ratio(293.15f, 101325.0f), 0.01447f, 1e-4f);
    // Colder air holds far less, which is why a cloud forms when a parcel is lifted and cools.
    EXPECT_LT(Render::atmosphere_saturation_mixing_ratio(273.15f, 101325.0f),
              Render::atmosphere_saturation_mixing_ratio(293.15f, 101325.0f));
}

TEST(Unit_AtmosphereNest, BaseVapourIsNeverSupersaturated)
{
    const Render::AtmosphereParameters p{};
    // The initial state must not start the model already condensing, or every column would
    // produce cloud on its first step regardless of what the dynamics did.
    for (float altitude = 0.0f; altitude <= 18000.0f; altitude += 250.0f)
    {
        const float vapour = Render::atmosphere_base_vapour(p, altitude);
        const float saturation = Render::atmosphere_saturation_mixing_ratio(
            Render::atmosphere_base_temperature(p, altitude),
            Render::atmosphere_base_pressure(p, altitude));
        EXPECT_GE(vapour, 0.0f) << "altitude " << altitude;
        EXPECT_LE(vapour, saturation * 1.0001f) << "altitude " << altitude;
    }
}

// ---- The stretched vertical grid ---------------------------------------------------------

TEST(Unit_AtmosphereNest, VerticalGridIsStretchedAndClosesOnTheDomainTop)
{
    const Render::AtmosphereNestSize size{};
    const std::uint32_t levels = size.levels;

    float previous = -1.0f;
    float total = 0.0f;
    for (std::uint32_t level = 0; level < levels; ++level)
    {
        const float altitude = Render::atmosphere_level_altitude(level, levels, size.top_m);
        EXPECT_GT(altitude, previous) << "level " << level;
        previous = altitude;
        total += Render::atmosphere_level_thickness(level, levels, size.top_m);
    }

    // The thicknesses have to tile the domain exactly, or every vertical derivative is dividing
    // by a spacing that does not add up to the column it claims to span.
    EXPECT_NEAR(total, size.top_m, size.top_m * 1e-4f);

    // Stretched, not uniform: the boundary layer and cloud base get the resolution and the
    // anvil does not. At 48 levels over 18 km that is ~54 m at the surface against ~560 m aloft.
    const float lowest = Render::atmosphere_level_thickness(0, levels, size.top_m);
    const float highest = Render::atmosphere_level_thickness(levels - 1, levels, size.top_m);
    EXPECT_LT(lowest, 120.0f);
    EXPECT_GT(highest, lowest * 4.0f);
}

// ---- The seam the GPU state reaches gameplay through -------------------------------------

TEST(Unit_AtmosphereNest, ColdStartReportsAClearSkyWithRealWind)
{
    // Nothing has been simulated yet, so there is no condensate to report — and inventing a
    // coverage here is exactly the fabricated signal §1's audit was written about. The wind is
    // a different matter: the synoptic layer is analytic and answers immediately.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
    ProceduralWeather weather(/*seed=*/11, EARTH_RADIUS_M);
    weather.apply_preset(Render::WeatherPreset::Storm, observer);
    weather.tick(1.0, observer, 2451545.0);

    const WeatherColumn column = weather.sample_column(observer);
    for (int level = 0; level < CLOUD_LEVEL_COUNT; ++level)
        EXPECT_FLOAT_EQ(column.levels[level].coverage, 0.0f) << "level " << level;
    EXPECT_FLOAT_EQ(column.precipitation, 0.0f);
    EXPECT_GT(std::hypot(column.wind_u_mps, column.wind_v_mps), 0.0f)
        << "a placed synoptic system must produce geostrophic wind without any GPU readback";
}

TEST(Unit_AtmosphereNest, MirrorTranscriptionSpeaksTheBridgesUnits)
{
    Render::AtmosphereMirrorColumn source{};
    source.bands[int(CloudLevel::Low)][0] = 0.8f;   // coverage
    source.bands[int(CloudLevel::Low)][1] = 1.5f;   // density scale
    source.bands[int(CloudLevel::Low)][2] = 0.9f;   // convective fraction
    source.bands[int(CloudLevel::Low)][3] = -3.25f; // temperature offset, C
    source.surface[0] = 5.0f;                       // precipitation, mm/h
    source.surface[1] = 7.0f;                       // eastward wind
    source.surface[2] = -2.0f;                      // northward wind

    const WeatherColumn column = WeatherFieldBuffer::column_from_mirror(source);
    EXPECT_FLOAT_EQ(column.levels[int(CloudLevel::Low)].coverage, 0.8f);
    EXPECT_FLOAT_EQ(column.levels[int(CloudLevel::Low)].density_scale, 1.5f);
    EXPECT_FLOAT_EQ(column.levels[int(CloudLevel::Low)].convective_fraction, 0.9f);
    EXPECT_FLOAT_EQ(column.levels[int(CloudLevel::Low)].temperature_offset_c, -3.25f);
    // The nest reports millimetres per hour, which is what a rain gauge reads; the bridge wants
    // a [0, 1] intensity against heavy rain. Five of ten is a half.
    EXPECT_FLOAT_EQ(column.precipitation, 0.5f);
    EXPECT_FLOAT_EQ(column.wind_u_mps, 7.0f);
    EXPECT_FLOAT_EQ(column.wind_v_mps, -2.0f);

    // Torrential rain saturates rather than reporting an out-of-range intensity.
    source.surface[0] = 40.0f;
    EXPECT_FLOAT_EQ(WeatherFieldBuffer::column_from_mirror(source).precipitation, 1.0f);
}

TEST(Unit_AtmosphereNest, ForcingCarriesTheSynopticStructureTheNestRelaxesToward)
{
    // The parent half of Davies nesting. The load-bearing claim is that it is *not uniform*:
    // a boundary nudged toward one value everywhere gives the nest nothing to build a front
    // out of, which was the shape of the defect this whole phase exists to remove.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
    ProceduralWeather weather(/*seed=*/5, EARTH_RADIUS_M);
    weather.apply_preset(Render::WeatherPreset::FrontPassage, observer);
    weather.tick(1.0, observer, 2451545.0);

    AtmosphereForcingBuffer buffer;
    weather.publish_forcing(observer, /*total_seconds=*/0.0, buffer);
    const Render::AtmosphereForcing forcing = buffer.view(0.0, 0.0, 0.0, 1.0e-4f);

    ASSERT_TRUE(forcing.valid());
    ASSERT_GT(forcing.cells_x, 1);

    float minimum_wind = 1e30f;
    float maximum_wind = -1e30f;
    float widest_theta = 0.0f;
    const std::size_t count = std::size_t(forcing.cells_x) * std::size_t(forcing.cells_z);
    for (std::size_t i = 0; i < count; ++i)
    {
        const Render::AtmosphereForcingSample& sample = forcing.samples[i];
        const float speed = std::hypot(sample.wind_east_mps, sample.wind_north_mps);
        minimum_wind = std::min(minimum_wind, speed);
        maximum_wind = std::max(maximum_wind, speed);
        widest_theta = std::max(widest_theta, std::fabs(sample.theta_anomaly_k));
    }

    EXPECT_GT(maximum_wind - minimum_wind, 0.5f)
        << "a uniform boundary wind cannot advect a front across the nest";
    EXPECT_GT(widest_theta, 0.0f)
        << "a front with no thermal contrast is not a front the nest can sharpen";

    // The lattice is centred on the observer, so scene origin maps to the middle of it.
    const double u = double(forcing.uv_scale_x) * 0.0 + double(forcing.uv_offset_x);
    const double v = double(forcing.uv_scale_z) * 0.0 + double(forcing.uv_offset_z);
    EXPECT_NEAR(u, 0.5, 1e-6);
    EXPECT_NEAR(v, 0.5, 1e-6);
}

TEST(Unit_AtmosphereNest, PublishedFieldFallsBackToClearWithoutAMirror)
{
    // A provider with no mirror bound still has to publish a field the renderer can read; what
    // it must not do is publish one that claims cloud nobody simulated.
    const GeodeticPosition observer{45.0 * DEGREES_TO_RADIANS, 10.0 * DEGREES_TO_RADIANS};
    ProceduralWeather weather(/*seed=*/3, EARTH_RADIUS_M);
    weather.tick(1.0, observer, 2451545.0);

    WeatherFieldBuffer buffer;
    weather.publish_field(observer, buffer);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_TRUE(field.derives_genus);
    // No classified band, so no shell is held open for decks that do not exist.
    EXPECT_FLOAT_EQ(field.union_base_m, 0.0f);
    EXPECT_FLOAT_EQ(field.union_top_m, 0.0f);
}
