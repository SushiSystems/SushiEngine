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
#include <type_traits>

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

TEST(Unit_AtmosphereNest, TheScaleHeightFoldsTheMixingRatioAndNotTheRelativeHumidity)
{
    const Render::AtmosphereParameters p{};
    const float surface = Render::atmosphere_base_vapour(p, 0.0f);

    // What the parameter is named for: one scale height up, the *mixing ratio* is down by e.
    // Applying the exponential to relative humidity instead decayed the profile twice — once
    // through the exponential and again through q_s — and left the free troposphere at 9 % RH.
    // The e-fold is exact only where the exponential governs; above ~4 km the ceiling below
    // takes over and the profile is deliberately drier than a bare exponential.
    EXPECT_NEAR(Render::atmosphere_base_vapour(p, p.humidity_scale_height) / surface,
                std::exp(-1.0f), 1e-3f);
    EXPECT_LT(Render::atmosphere_base_vapour(p, 2.0f * p.humidity_scale_height) / surface,
              std::exp(-2.0f));

    // And the profile that follows: q_s folds over ~3.2 km against the vapour's 2.5, so relative
    // humidity decays gently rather than collapsing. A mid-troposphere near half saturated is
    // what lets a lifted parcel reach its condensation level at all.
    const auto relative_humidity = [&p](float altitude) {
        return Render::atmosphere_base_vapour(p, altitude) /
               Render::atmosphere_saturation_mixing_ratio(
                   Render::atmosphere_base_temperature(p, altitude),
                   Render::atmosphere_base_pressure(p, altitude));
    };
    EXPECT_NEAR(relative_humidity(0.0f), p.surface_humidity, 1e-3f);
    EXPECT_NEAR(relative_humidity(1341.0f), 0.62f, 0.02f);
    EXPECT_NEAR(relative_humidity(5000.0f), 0.50f, 0.03f);

    // And the ceiling that has to sit on top of it. Above ~7 km the ordering of the two scale
    // heights reverses, so the bare exponential climbs back through saturation and starts every
    // run with a global cirrus deck — measured at 81 % RH at 9.5 km before this existed. The
    // Weisman-Klemp shape holds the upper troposphere where a real sounding has it.
    EXPECT_LT(relative_humidity(9569.0f), 0.35f);
    EXPECT_LT(relative_humidity(p.tropopause_altitude), 0.25f);
    EXPECT_NEAR(Render::atmosphere_base_humidity_ceiling(p, p.tropopause_altitude),
                p.surface_humidity * (1.0f - p.free_troposphere_drying), 1e-4f);

    // The ceiling must not touch the ground, or it would be setting the surface humidity rather
    // than capping the profile aloft.
    EXPECT_NEAR(Render::atmosphere_base_humidity_ceiling(p, 0.0f), p.surface_humidity, 1e-6f);
}

TEST(Unit_AtmosphereNest, CloudTopCoolingIsConservativeAndConcentratedAtTheTop)
{
    const Render::AtmosphereParameters p{};

    // Clear air has no cloud-top cooling, which is the whole distinction the term draws: this is
    // the sink a cloud has and the air beside it does not.
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_absorption(p, 0.0f, 0.0f), 0.0f);

    // A deck cannot radiate away more than its top is given, however it is sliced. Summing one
    // level's worth against ten slices of a tenth each has to land on the same number, or the
    // cooling would depend on the vertical resolution rather than on the cloud.
    const float total_path = 0.05f;
    const float whole = Render::atmosphere_cloud_top_absorption(p, 0.0f, total_path);
    float sliced = 0.0f;
    for (int slice = 0; slice < 10; ++slice)
        sliced += Render::atmosphere_cloud_top_absorption(p, float(slice) * total_path * 0.1f,
                                                          total_path * 0.1f);
    EXPECT_NEAR(sliced, whole, whole * 1e-4f);
    EXPECT_LE(whole, p.cloud_top_longwave_flux);

    // And where it lands. The top tenth of the path takes 48 % of the flux — nearly five times a
    // uniform share — which is the mechanism rather than a detail: cooling spread through a deck
    // stabilises it, cooling only its top makes the top denser than what is beneath it and drives
    // the overturning.
    const float top_tenth = Render::atmosphere_cloud_top_absorption(p, 0.0f, total_path * 0.1f);
    EXPECT_GT(top_tenth, 4.0f * 0.1f * whole);

    // Deep inside the cloud there is nothing left to absorb.
    EXPECT_LT(Render::atmosphere_cloud_top_absorption(p, total_path, total_path),
              0.01f * whole);
}

TEST(Unit_AtmosphereNest, CloudTopCoolingClosesInsteadOfRunningAway)
{
    const Render::AtmosphereParameters p{};

    // The distinction this scale draws is between a flux and a sink. A cloud at its environment's
    // temperature loses everything the flux says it does — so nothing about a transient deck, the
    // case the term was calibrated on, changes at all.
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_flux_scale(p, 0.0f), 1.0f);
    // A cloud *warmer* than its environment does not lose more than one. The term is a bound on
    // the sky's return, not a temperature difference to be scaled up.
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_flux_scale(p, 5.0f), 1.0f);

    // And the closure: by the depression the loss is gone, and it never turns into a gain however
    // far past it a column is pushed. This is what bounds the cooling — without it the same
    // 70 W/m² is paid at every temperature, and a deck that persists cools forever.
    EXPECT_FLOAT_EQ(
        Render::atmosphere_cloud_top_flux_scale(p, -p.cloud_top_equilibrium_depression), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_flux_scale(p, -1000.0f), 0.0f);

    // Monotone in between, so a deck approaches its floor rather than switching off at it.
    float previous = 1.0f;
    for (int step = 1; step <= 15; ++step)
    {
        const float scale = Render::atmosphere_cloud_top_flux_scale(p, -float(step));
        EXPECT_LT(scale, previous);
        previous = scale;
    }
}

TEST(Unit_AtmosphereNest, CloudTopEntrainmentMixesWhereTheCoolingIs)
{
    Render::AtmosphereParameters p{};

    // No efficiency, no cooling, or no stable interface: no closure. The last is the
    // double-counting guard — an unstable top is resolved convection's job.
    Render::AtmosphereParameters off = p;
    off.cloud_top_entrainment_efficiency = 0.0f;
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_entrainment(off, 70.0f, 1.0f, 5.0f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_entrainment(p, 0.0f, 1.0f, 5.0f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, -2.0f), 0.0f);

    // The closure's own arithmetic: w_e = A * dF / (rho * c_p * dtheta), against the same
    // numbers the shader is handed. 0.8 * 70 / (1.0 * 1005 * 5) is 11.1 mm/s — the upper end
    // of the measured range for nocturnal stratocumulus, which is what says the default
    // coefficient is a physical quantity and not a tuned one.
    const float expected = p.cloud_top_entrainment_efficiency * 70.0f /
                           (1.0f * p.specific_heat_pressure * 5.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, 5.0f), expected);

    // A stronger inversion entrains less: the jump is the resistance the mixing works against,
    // which is why a deck under a hard subsidence inversion outlives one under a soft one.
    EXPECT_LT(Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, 10.0f),
              Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, 5.0f));

    // The floor: a vanishing inversion asks for a velocity a step can integrate, not infinity.
    EXPECT_FLOAT_EQ(Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, 0.1f),
                    Render::atmosphere_cloud_top_entrainment(p, 70.0f, 1.0f, 0.5f));

    // The default is derived from the flux rather than typed beside it, and this is the arithmetic
    // that ties them: a black body at the base state's 1585 m temperature, cooled by the
    // depression, has given up the flux the term starts from.
    constexpr float STEFAN_BOLTZMANN = 5.670374e-8f;
    const float ambient = Render::atmosphere_base_temperature(p, 1585.0f);
    const float floor_k = ambient - p.cloud_top_equilibrium_depression;
    const float given_up = STEFAN_BOLTZMANN * (std::pow(ambient, 4.0f) - std::pow(floor_k, 4.0f));
    EXPECT_NEAR(given_up, p.cloud_top_longwave_flux, 5.0f);
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

TEST(Unit_AtmosphereNest, TheSpongeCoversTheLevelsThatOscillatedAndNotTheWeather)
{
    const Render::AtmosphereParameters p{};
    const Render::AtmosphereNestSize size{};

    // Shape first: zero and flat where it starts, one at the lid, monotone between.
    const float start = size.top_m - p.sponge_depth;
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, start), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, size.top_m), 1.0f);
    float previous = -1.0f;
    for (int step = 0; step <= 20; ++step)
    {
        const float altitude = start + p.sponge_depth * float(step) / 20.0f;
        const float weight = Render::atmosphere_sponge_weight(p, size.top_m, altitude);
        EXPECT_GE(weight, previous) << "altitude " << altitude;
        EXPECT_GE(weight, 0.0f);
        EXPECT_LE(weight, 1.0f);
        previous = weight;
    }

    // A sponge damps exactly nothing below its lower edge, which is why the depth is the
    // parameter that matters and the rate is not: whatever the rate, this is zero.
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, start - 1.0f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, 0.0f), 0.0f);

    // The measurement this default exists for. A two-cell vertical mode grew to +-13 K at
    // 12.4 km, in levels a 5 km sponge left uncovered; the depth has to reach them. Doubling
    // the rate instead moved it to +-11 K, because zero times anything is zero.
    EXPECT_GT(Render::atmosphere_sponge_weight(p, size.top_m, 12430.0f), 0.05f);
    EXPECT_GT(Render::atmosphere_sponge_weight(p, size.top_m, 12930.0f), 0.05f);

    // And the other side of it: deep enough to reach 12.4 km, not so deep that it replaces the
    // troposphere. An edge at 6 km collapsed the wind between 4.6 and 9 km to 0.02 m/s. The
    // boundary layer, the cloud deck at 1585 m, and the free troposphere below 4 km are the
    // weather this model is for, and the sponge does not touch them.
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, 1585.0f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, 4000.0f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_sponge_weight(p, size.top_m, 6166.0f), 0.0f);
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

TEST(Unit_AtmosphereNest, ProfileIsAFlatFloatRunAndIsAbsentUntilAReadbackCompletes)
{
    // Both halves of this pin the same seam from opposite sides, and both failures are silent.
    //
    // The layout half: `atmosphere_readback.comp` writes a std430 array of `ProfileLevel`, which
    // the host then memcpys into `AtmosphereProfileLevel`. std430 gives an array of a
    // floats-only struct a stride of exactly its size, so the two agree only while this side
    // stays a flat, tightly packed run of floats. A field added here and not there — or a
    // member that is not a float — shifts every level after it, and the result is a profile
    // full of plausible numbers rather than a crash or a compile error.
    static_assert(std::is_standard_layout<Render::AtmosphereProfileLevel>::value,
                  "the profile is copied from a GPU buffer, so its layout must be fixed");
    EXPECT_EQ(sizeof(Render::AtmosphereProfileLevel), 16 * sizeof(float))
        << "the shader's ProfileLevel and this struct are the same bytes seen from either side "
           "of a buffer copy; a field was added to one of them alone";
    EXPECT_EQ(alignof(Render::AtmosphereProfileLevel), alignof(float));

    // The lifetime half: the profile is borrowed from a readback that may not have happened.
    // Publishing a non-null pointer with nothing behind it would be the worse failure, so a
    // mirror that carries no snapshot must say so through both fields at once.
    const Render::AtmosphereMirror cold{};
    EXPECT_FALSE(cold.valid());
    EXPECT_EQ(cold.profile, nullptr);
    EXPECT_EQ(cold.profile_levels, 0);
}

TEST(Unit_AtmosphereNest, SubgridCloudFractionGeneralisesTheAllOrNothingAdjustment)
{
    // The closure's load-bearing property, and the reason it can be adopted without a second,
    // competing condensation path: at a critical humidity of 1 the subgrid distribution has no
    // width, and the partition must reduce *exactly* to condensing the excess over saturation.
    // If it does not, then a scene authored at 1.0 gets some third behaviour belonging to
    // neither scheme, and every comparison against the phase's earlier measurements is void.
    const Render::AtmosphereParameters parameters;
    const float saturation = 0.008f;                // kg/kg, a fair-weather cumulus base
    const float efficiency = 0.5f;                  // near enough the 290 K value

    const Render::AtmosphereCloudPartition dry =
        Render::atmosphere_cloud_partition(0.006f, saturation, efficiency, 1.0f);
    EXPECT_FLOAT_EQ(dry.fraction, 0.0f);
    EXPECT_FLOAT_EQ(dry.condensate, 0.0f);

    const Render::AtmosphereCloudPartition wet =
        Render::atmosphere_cloud_partition(0.010f, saturation, efficiency, 1.0f);
    EXPECT_FLOAT_EQ(wet.fraction, 1.0f);
    EXPECT_FLOAT_EQ(wet.condensate, efficiency * (0.010f - saturation));

    // With a width, the first cloud appears exactly at the authored critical humidity and not
    // before it. This is the number an author sets, so it has to mean what it says.
    const float critical = 0.8f;
    const Render::AtmosphereCloudPartition below = Render::atmosphere_cloud_partition(
        saturation * (critical - 0.01f), saturation, efficiency, critical);
    EXPECT_FLOAT_EQ(below.fraction, 0.0f);
    const Render::AtmosphereCloudPartition above = Render::atmosphere_cloud_partition(
        saturation * (critical + 0.01f), saturation, efficiency, critical);
    EXPECT_GT(above.fraction, 0.0f);
    EXPECT_GT(above.condensate, 0.0f);

    // Half the cell is cloud when the cell mean is exactly saturated — which is the whole
    // difference from the scheme this replaces, where a saturated *mean* was the point cloud
    // began rather than the point it was already half-formed.
    const Render::AtmosphereCloudPartition mean_saturated =
        Render::atmosphere_cloud_partition(saturation, saturation, efficiency, critical);
    EXPECT_FLOAT_EQ(mean_saturated.fraction, 0.5f);
    EXPECT_GT(mean_saturated.condensate, 0.0f);

    // And it closes: at the wet end the partition rejoins the all-or-nothing answer, so a
    // deep saturated cell condenses the same water either way and only the approach differs.
    const float width = efficiency * (1.0f - critical) * saturation;
    const Render::AtmosphereCloudPartition overcast = Render::atmosphere_cloud_partition(
        saturation + width / efficiency, saturation, efficiency, critical);
    EXPECT_FLOAT_EQ(overcast.fraction, 1.0f);
    EXPECT_NEAR(overcast.condensate, width, 1.0e-9f);

    // The efficiency itself: latent heating consumes part of every excess, so a saturation
    // adjustment can never condense the whole of it. Around half at cumulus-base temperature.
    const float measured =
        Render::atmosphere_condensation_efficiency(parameters, saturation, 290.0f);
    EXPECT_GT(measured, 0.3f);
    EXPECT_LT(measured, 0.7f);
    // Colder air holds less vapour, so less heating per unit excess and more of it survives.
    EXPECT_GT(Render::atmosphere_condensation_efficiency(parameters, 0.002f, 260.0f), measured);
}

TEST(Unit_AtmosphereNest, IcePartitionReducesExactlyToLiquidAboveFreezing)
{
    // The phase partition's load-bearing property, and the one that lets it be adopted without
    // re-tuning every warm scene: above the freezing point the blended relations must be
    // *identically* the liquid ones. If they are not, then every measurement this phase took
    // before ice existed is void, and the difference would be invisible — a slightly different
    // sky, with nothing saying why.
    const Render::AtmosphereParameters p;
    const float pressure = 85000.0f; // ~1500 m, where these clouds live

    for (const float t : {300.0f, 290.0f, 280.0f, p.freezing_temperature})
    {
        EXPECT_FLOAT_EQ(Render::atmosphere_ice_fraction(p, t), 0.0f) << "at " << t << " K";
        EXPECT_FLOAT_EQ(Render::atmosphere_saturation_mixing_ratio_phase(p, t, pressure),
                        Render::atmosphere_saturation_mixing_ratio(t, pressure))
            << "at " << t << " K";
        EXPECT_FLOAT_EQ(Render::atmosphere_latent_heat(p, t), p.latent_heat_vaporization);
    }

    // And below the glaciation point it is entirely ice: full fusion on top of vaporization,
    // which is 13 % more heat per kilogram condensed.
    const float cold = p.glaciation_temperature - 5.0f;
    EXPECT_FLOAT_EQ(Render::atmosphere_ice_fraction(p, cold), 1.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_latent_heat(p, cold),
                    p.latent_heat_vaporization + p.latent_heat_fusion);
    EXPECT_NEAR(Render::atmosphere_latent_heat(p, cold) / p.latent_heat_vaporization, 1.133f,
                0.01f);

    // The band between is a ramp, monotone and reaching exactly a half at its midpoint.
    const float middle = 0.5f * (p.freezing_temperature + p.glaciation_temperature);
    EXPECT_FLOAT_EQ(Render::atmosphere_ice_fraction(p, middle), 0.5f);
    EXPECT_GT(Render::atmosphere_ice_fraction(p, middle - 1.0f),
              Render::atmosphere_ice_fraction(p, middle + 1.0f));
}

TEST(Unit_AtmosphereNest, SaturationOverIceIsLowerAndThatGapIsTheBergeronProcess)
{
    // The two Magnus curves must **meet exactly at 0 °C** — that is what makes blending across
    // the mixed-phase band continuous rather than a step every cell crosses.
    EXPECT_NEAR(Render::atmosphere_saturation_pressure(273.15f),
                Render::atmosphere_saturation_pressure_ice(273.15f), 0.5f);
    // Both agree with the textbook triple-point value.
    EXPECT_NEAR(Render::atmosphere_saturation_pressure(273.15f), 611.2f, 1.0f);

    // Below it the ice curve sits lower, and the gap peaks around −12 °C at some 10 %: air in
    // equilibrium with a supercooled droplet is *supersaturated* with respect to a crystal
    // beside it, so the crystal grows at the droplet's expense. That gap is the whole of the
    // Bergeron process, and it is why a cold cloud glaciates and precipitates.
    const float liquid = Render::atmosphere_saturation_pressure(261.15f);   // -12 C
    const float ice = Render::atmosphere_saturation_pressure_ice(261.15f);
    EXPECT_LT(ice, liquid);
    EXPECT_NEAR(1.0f - ice / liquid, 0.10f, 0.02f);
    // Textbook: 2.44 hPa over water and 2.17 hPa over ice at -12 C.
    EXPECT_NEAR(liquid, 244.0f, 5.0f);
    EXPECT_NEAR(ice, 217.0f, 5.0f);

    // What that means for the model: the same air is cloudier when it is colder, on the same
    // water. A cell below the glaciation point condenses against the ice curve, which is well
    // under the liquid one, so its saturation mixing ratio is lower.
    const Render::AtmosphereParameters p;
    const float pressure = 85000.0f;
    const float cold = p.glaciation_temperature - 5.0f;
    EXPECT_LT(Render::atmosphere_saturation_mixing_ratio_phase(p, cold, pressure),
              Render::atmosphere_saturation_mixing_ratio(cold, pressure));

    // The condensation efficiency falls with the phase-blended latent heat, because a cell that
    // releases more heat per kilogram raises its own saturation further and keeps less of the
    // excess. Compared at equal saturation so the phase is the only thing that differs.
    const float saturation = 0.002f;
    EXPECT_LT(Render::atmosphere_condensation_efficiency(p, saturation, cold),
              Render::atmosphere_condensation_efficiency(
                  p, saturation, p.freezing_temperature + 0.1f));
}

TEST(Unit_AtmosphereNest, InsolationIsDimmedAlongTheSlantPathAndNotOnlyByElevation)
{
    const Render::AtmosphereParameters parameters;

    // Below the horizon there is no sun, and the balance has to be able to say so rather than
    // returning a small positive number that would keep the ground warming all night.
    EXPECT_FLOAT_EQ(Render::atmosphere_clear_sky_shortwave(parameters, -0.1f), 0.0f);
    EXPECT_FLOAT_EQ(Render::atmosphere_clear_sky_shortwave(parameters, 0.0f), 0.0f);

    // Overhead: the constant, once through the atmosphere.
    const float zenith = Render::atmosphere_clear_sky_shortwave(parameters, 1.0f);
    EXPECT_NEAR(zenith, parameters.solar_constant * parameters.clear_sky_transmittance, 1.0f);

    // **The load-bearing property.** At a 30 degree sun the geometric factor alone would give
    // half the zenith value; the slant path costs another factor on top, because the beam
    // crosses twice the air. That extra dimming is what makes the morning and evening shoulders
    // steeper than a cosine, and it is why the ground starts warming long after the sky lights.
    const float low = Render::atmosphere_clear_sky_shortwave(parameters, 0.5f);
    EXPECT_LT(low, 0.5f * zenith);
    EXPECT_GT(low, 0.0f);
    // Monotone in elevation, which a Beer path through a non-negative optical depth must be.
    EXPECT_LT(Render::atmosphere_clear_sky_shortwave(parameters, 0.2f), low);
}

TEST(Unit_AtmosphereNest, SurfaceBalanceIsUnconditionallyStableAndLandsItsSteadyState)
{
    // The diurnal cycle's integrator. Two claims are pinned here, and both are the kind that
    // fail silently: a scene simply looks different, and nothing says why.
    Render::AtmosphereParameters parameters;
    const float air = 288.15f;
    const float pressure = 101325.0f;
    const float density = 1.2f;
    const float wind = 2.0f;
    const float vapour = 0.007f;
    const float absorbed = 600.0f; // a bright afternoon, after albedo

    // (1) It reaches a steady state, and the steady state is where the fluxes balance the
    //     radiation -- not merely somewhere nearby. The relaxation time is C/lambda, which at
    //     these values is some 2 800 s, so the loop steps in hundreds rather than ones: being
    //     able to do that *is* the semi-implicit form's guarantee, and a one-second loop here
    //     would only be measuring how many e-foldings it had patience for.
    float skin = air;
    Render::AtmosphereSurfaceBalance state;
    for (int i = 0; i < 4000; ++i)
    {
        state = Render::atmosphere_surface_balance(parameters, skin, air, vapour, pressure,
                                                   density, wind, absorbed, 100.0f);
        skin = state.skin_k;
    }
    EXPECT_NEAR(state.net_radiation - state.sensible - state.latent, 0.0f, 0.05f);
    // A surface under 600 W/m² is warmer than the air above it, and by a lot; that difference is
    // what drives the convection this whole tier exists to produce.
    EXPECT_GT(skin, air + 2.0f);

    // (2) **Unconditional stability**, which is the reason the update is semi-implicit rather
    //     than explicit. A thin slab -- a road, a rock face -- relaxes far faster than the nest
    //     steps, and that is the regime an explicit update cannot survive.
    //
    //     What is claimed is stability, not exactness and not monotonicity. A step is one Newton
    //     iteration on a nonlinear balance -- `q_s` is exponential in the skin -- so a very large
    //     first step overshoots (measured, by 3.3 K on a 16 K approach) and then converges from
    //     the other side. What it never does is grow: the residual falls every step after the
    //     first, and the sequence lands on the same equilibrium the patient loop above found.
    parameters.surface_heat_capacity = 1.0e3f;
    const float huge_step = 1.0e6f;
    float leaping = air;
    float previous_residual = 0.0f;
    float first_residual = 0.0f;
    for (int i = 0; i < 8; ++i)
    {
        const Render::AtmosphereSurfaceBalance leap = Render::atmosphere_surface_balance(
            parameters, leaping, air, vapour, pressure, density, wind, absorbed, huge_step);
        ASSERT_TRUE(std::isfinite(leap.skin_k));
        // Bounded by physics at every step, however large the step: a surface under 600 W/m²
        // is somewhere between the air's temperature and a few tens of kelvin above it.
        EXPECT_GT(leap.skin_k, air - 1.0f);
        EXPECT_LT(leap.skin_k, air + 100.0f);
        const float residual = std::fabs(leap.net_radiation - leap.sensible - leap.latent);
        if (i == 0)
            first_residual = residual;
        else
            EXPECT_LE(residual, previous_residual);
        previous_residual = residual;
        leaping = leap.skin_k;
    }
    EXPECT_NEAR(previous_residual, 0.0f, 0.5f);
    // The answer is a property of the balance, not of how it was stepped there.
    EXPECT_NEAR(leaping, skin, 0.5f);

    // And this is what the semi-implicit form actually buys, stated as arithmetic rather than as
    // an adjective. The explicit update at the same step is the same numerator over `C` alone
    // instead of over `C + dt·lambda`, and with a thin slab and a long step that ratio is five
    // orders of magnitude: the "temperature" it produces is not a large error, it is not a
    // temperature.
    const float explicit_delta = first_residual * huge_step / parameters.surface_heat_capacity;
    EXPECT_GT(explicit_delta, 1.0e4f);

    // (3) After dark the balance runs backwards on its own: no shortwave, so the ground radiates
    //     to a colder sky than it is, cools below the air, and the sensible flux reverses. This
    //     is what *ends* convection in the evening, and the retired `surface_night_flux` was a
    //     constant standing in for it.
    parameters = Render::AtmosphereParameters{};
    const Render::AtmosphereSurfaceBalance night = Render::atmosphere_surface_balance(
        parameters, air, air, vapour, pressure, density, wind, 0.0f, 60.0f);
    EXPECT_LT(night.net_radiation, 0.0f);
    EXPECT_LT(night.skin_k, air);
    EXPECT_LT(night.sensible, 0.0f);

    // (4) Moisture availability is the Bowen ratio's author, and it has to actually author it:
    //     the same radiation over a wet surface must go mostly into evaporation and over a dry
    //     one mostly into heating the air.
    Render::AtmosphereParameters wet = parameters;
    wet.surface_moisture_availability = 1.0f;
    Render::AtmosphereParameters dry = parameters;
    dry.surface_moisture_availability = 0.0f;
    const auto settle = [&](const Render::AtmosphereParameters& p)
    {
        float s = air;
        Render::AtmosphereSurfaceBalance b;
        for (int i = 0; i < 20000; ++i)
        {
            b = Render::atmosphere_surface_balance(p, s, air, vapour, pressure, density, wind,
                                                   absorbed, 1.0f);
            s = b.skin_k;
        }
        return b;
    };
    const Render::AtmosphereSurfaceBalance soaked = settle(wet);
    const Render::AtmosphereSurfaceBalance parched = settle(dry);
    EXPECT_GT(soaked.latent, soaked.sensible);
    EXPECT_FLOAT_EQ(parched.latent, 0.0f);
    // And the wet surface is the cooler one, because evaporation is where its energy went.
    EXPECT_LT(soaked.skin_k, parched.skin_k);
}

TEST(Unit_AtmosphereNest, TheGroundRadiatesToTheCloudBaseAndNotThroughIt)
{
    const Render::AtmosphereParameters p{};
    const float air = 288.0f;
    const float vapour = 0.008f;
    const float pressure = 101325.0f;
    const float density = 1.2f;
    const float wind = 3.0f;

    // A clear sky is the balance without any of this, stated as an equality rather than as an
    // intention: a caller with no cloud field to offer must get exactly what it got before one
    // existed, so the default argument cannot quietly change anybody's weather.
    const Render::AtmosphereSurfaceBalance bare = Render::atmosphere_surface_balance(
        p, air, air, vapour, pressure, density, wind, 0.0f, 60.0f);
    Render::AtmosphereCloudCover clear;
    const Render::AtmosphereSurfaceBalance stated = Render::atmosphere_surface_balance(
        p, air, air, vapour, pressure, density, wind, 0.0f, 60.0f, clear);
    EXPECT_FLOAT_EQ(bare.net_radiation, stated.net_radiation);
    EXPECT_FLOAT_EQ(bare.skin_k, stated.skin_k);

    // A column with cover but no water is still clear. Cover alone is not a radiating surface,
    // and this is the case a cloud field produces constantly as a deck thins out of existence.
    Render::AtmosphereCloudCover empty;
    empty.fraction = 1.0f;
    empty.base_temperature_k = 280.0f;
    EXPECT_FLOAT_EQ(Render::atmosphere_surface_balance(p, air, air, vapour, pressure, density,
                                                       wind, 0.0f, 60.0f, empty)
                        .net_radiation,
                    bare.net_radiation);

    // **The defect this closes.** After dark under a real deck the ground is nearly in radiative
    // balance, because what is over it is a warm near-black body rather than the thin emission of
    // clear air. Brutsaert's clear-sky value under an overcast column has the ground going on
    // losing a hundred watts to a sky that is not there — and it was half of a column that cooled
    // 42.7 K in 72 h and did not stop.
    Render::AtmosphereCloudCover overcast;
    overcast.fraction = 1.0f;
    overcast.base_temperature_k = 283.0f;
    overcast.water_path = 0.05f; // 50 g/m^2, an ordinary stratocumulus
    const Render::AtmosphereSurfaceBalance covered = Render::atmosphere_surface_balance(
        p, air, air, vapour, pressure, density, wind, 0.0f, 60.0f, overcast);
    EXPECT_GT(covered.net_radiation, bare.net_radiation);
    // Not merely larger — the loss all but closes, where the clear-sky value leaves the ground
    // tens of watts short of balance with no way out of it.
    EXPECT_LT(std::fabs(covered.net_radiation), 0.45f * std::fabs(bare.net_radiation));
    EXPECT_LT(bare.net_radiation, -50.0f);

    // Blended by cover rather than switched by it, so a broken sky lands between the two ends.
    Render::AtmosphereCloudCover half = overcast;
    half.fraction = 0.5f;
    const float scattered = Render::atmosphere_surface_balance(p, air, air, vapour, pressure,
                                                               density, wind, 0.0f, 60.0f, half)
                                .net_radiation;
    EXPECT_GT(scattered, bare.net_radiation);
    EXPECT_LT(scattered, covered.net_radiation);

    // And a cold high cirrus returns less than a warm low deck, which is the whole reason the
    // base *temperature* is carried rather than the cover alone.
    Render::AtmosphereCloudCover high = overcast;
    high.base_temperature_k = 240.0f;
    EXPECT_LT(Render::atmosphere_surface_balance(p, air, air, vapour, pressure, density, wind,
                                                  0.0f, 60.0f, high)
                  .net_radiation,
              covered.net_radiation);
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
    // The sun is passed through unexamined here; what this test is about is the horizontal
    // structure of the parent solution, which the surface forcing does not touch.
    const Render::AtmosphereForcing forcing =
        buffer.view(0.0, 0.0, /*total_seconds=*/0.0, /*coriolis=*/1.0e-4f,
                    /*solar_elevation_sine=*/0.9f);

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
    // **The assertion whose absence let a 15 534 m/s boundary wind ship.** The check above is
    // about *structure* and a wildly-scaled field satisfies it perfectly, so nothing here ever
    // looked at the magnitude. A synoptic wind is a wind: tens of metres per second at the top
    // of a deepening low's gradient, not hundreds and not thousandths.
    EXPECT_GT(maximum_wind, 1.0f) << "the parent is becalmed; nothing will advect across the nest";
    EXPECT_LT(maximum_wind, 80.0f)
        << "this is not a wind -- check the geostrophic scale's unit conversion";
    EXPECT_GT(widest_theta, 0.0f)
        << "a front with no thermal contrast is not a front the nest can sharpen";

    // The lattice is centred on the observer, so scene origin maps to the middle of it.
    const double u = double(forcing.uv_scale_x) * 0.0 + double(forcing.uv_offset_x);
    const double v = double(forcing.uv_scale_z) * 0.0 + double(forcing.uv_offset_z);
    EXPECT_NEAR(u, 0.5, 1e-6);
    EXPECT_NEAR(v, 0.5, 1e-6);
}

TEST(Unit_AtmosphereNest, QuasiGeostrophicOmegaLandsAtTheSynopticScaleAndBothWays)
{
    // The large-scale vertical motion the nest cannot generate for itself.
    //
    // **This case used to assert something else, and the difference is the Phase C swap.** Its
    // predecessor pinned Ekman pumping: friction turns the geostrophic wind across the isobars,
    // the turn converges into a low, and the convergent air must go up — so air rose at the
    // centre of the deepest low, full stop, and the test said exactly that. That term is gone.
    // The core diagnoses quasi-geostrophic omega from its own vorticity budget instead, and QG
    // omega is *not* centred on the low: it follows differential vorticity advection, which puts
    // the ascent downstream of the trough and the descent upstream of it. Re-asserting the old
    // claim against the new mechanism would have been asserting the boundary layer's answer
    // about a free-atmosphere quantity, so the claim is restated rather than ported.
    //
    // What survives unchanged is the part that was never about the mechanism: the sign structure
    // and the exponent. Centimetres per second — three orders under a convective updraft and two
    // over nothing — and getting that exponent wrong is the failure mode a "looks about right"
    // review would pass. It is also what caught the geostrophic scale being 735x too large, back
    // when the pumping saturated its own cap *everywhere*.
    const Atmosphere::GeographicPosition centre{45.0 * DEGREES_TO_RADIANS,
                                                10.0 * DEGREES_TO_RADIANS};
    const GeodeticPosition observer{centre.latitude_radians, centre.longitude_radians};

    // The core is built directly rather than reached through `ProceduralWeather`, because what
    // is under test is the buffer's coupling to it over a window of the test's own choosing --
    // 3 000 km rather than the nest's own footprint, since a window small enough to sit inside
    // one circulation would see one sign and make a close-up look like a bug.
    Atmosphere::QuasiGeostrophicParameters physics;
    physics.planet_radius_m = EARTH_RADIUS_M;
    Atmosphere::QuasiGeostrophicCore core(Atmosphere::QuasiGeostrophicGridSize{}, physics);
    core.seed(5);

    // A low and a high, 800 km either side, placed by hand rather than taken from a preset: what
    // is being pinned is the *sign*, and a preset that happened to seed only lows would pass
    // half of it silently.
    const double east_to_radians = 1.0 / (EARTH_RADIUS_M * std::cos(centre.latitude_radians));
    core.inject_vorticity(
        Atmosphere::GeographicPosition{centre.latitude_radians,
                                       centre.longitude_radians - 800000.0 * east_to_radians},
        /*radius_m=*/500000.0, /*amplitude_mps=*/22.0);
    core.inject_vorticity(
        Atmosphere::GeographicPosition{centre.latitude_radians,
                                       centre.longitude_radians + 800000.0 * east_to_radians},
        /*radius_m=*/500000.0, /*amplitude_mps=*/-22.0);

    // Omega is diagnosed *during* a step, from the vorticity tendency across it -- there is no
    // vertical motion to read before the core has been advanced even once, which is a true
    // statement about a diagnosed quantity rather than a limitation. Six hours lets the injected
    // pair start being advected, which is what generates the differential advection in the first
    // place.
    ASSERT_GT(core.advance(6.0 * 3600.0), 0) << "the core did not step; nothing diagnosed omega";

    AtmosphereForcingBuffer buffer;
    constexpr double SPAN_METERS = 3.0e6;
    buffer.fill(core, observer, EARTH_RADIUS_M, SPAN_METERS,
                Render::ATMOSPHERE_FORCING_MAX_CELLS);
    const Render::AtmosphereForcing forcing = buffer.view(0.0, 0.0, 0.0, 1.0e-4f, 0.9f);
    ASSERT_TRUE(forcing.valid());

    float ascent = 0.0f;
    float descent = 0.0f;
    double magnitude = 0.0;
    const std::size_t count = std::size_t(forcing.cells_x) * std::size_t(forcing.cells_z);
    for (std::size_t i = 0; i < count; ++i)
    {
        const float w = forcing.samples[i].vertical_velocity_mps;
        ascent = std::max(ascent, w);
        descent = std::min(descent, w);
        magnitude += std::fabs(double(w));
    }
    magnitude /= double(count);

    // Both signs must appear: mass is conserved, so a field that only rises is a field that
    // creates air. This is the claim the old test made too, and the one that does transfer --
    // it is a statement about continuity rather than about which term drove it.
    EXPECT_GT(ascent, 0.0f) << "nothing rises anywhere: omega is inert";
    EXPECT_LT(descent, 0.0f) << "nothing sinks anywhere: air is being created";

    // Centimetres per second. Below 1e-4 in the mean the term is inert and the acceptance clause
    // it exists for stays open; a synoptic omega that reached tenths of a metre per second would
    // not be synoptic.
    EXPECT_GT(magnitude, 1.0e-4);
    EXPECT_LT(magnitude, 5.0e-2);
    EXPECT_LE(std::max(ascent, -descent), 0.1f)
        << "the buffer's own cap was exceeded, which it cannot be";
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
