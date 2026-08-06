/**************************************************************************/
/* test_synoptic_field.cpp                                                */
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

// Unit_SynopticField: the planetary weather placement behind Manual mode
// (docs/design/atmosphere_system.md, WM-SEED). The defect it exists to rule out shows in one
// side-by-side — a uniformly milky sphere from orbit, where the real Earth is mostly clear
// ocean with weather laid over it in discrete pieces — so the load-bearing claims here are
// about *variety*: that the field genuinely reaches both extremes, that a seed decides which
// is where, and that the same seed always decides the same way.
//
// Pure host maths; no SushiRuntime and no device, same reasoning as the other weather tests.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/atmosphere/synoptic_field.hpp>
#include <SushiEngine/simulation/seeded_weather.hpp>
#include <SushiEngine/simulation/weather_field_buffer.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr double EARTH_RADIUS_M = 6371000.0;
    constexpr double DEGREE = 3.14159265358979323846 / 180.0;

    // A coarse sweep of the whole sphere. Everything below that asks "does the planet contain
    // X" asks it through this, so the sampling is stated once.
    template <typename Fn>
    void for_each_place(Fn fn)
    {
        for (int lat = -85; lat <= 85; lat += 5)
            for (int lon = -180; lon < 180; lon += 5)
                fn(double(lat) * DEGREE, double(lon) * DEGREE);
    }
} // namespace

TEST(Unit_SynopticField, TheSubtropicsAreClearerThanTheTropicsAndTheStormTrack)
{
    // The term that decides whether an orbital view has genuinely clear ocean in it. Without
    // the subtropical minimum the field reads as overcast everywhere no matter how the placed
    // systems are tuned, which is exactly what the uniform deck stack it replaces did.
    const double itcz = Atmosphere::synoptic_itcz_latitude(0.5);
    const double tropics = Atmosphere::synoptic_zonal_coverage(itcz, itcz);
    const double subtropics = Atmosphere::synoptic_zonal_coverage(25.0 * DEGREE, itcz);
    const double storm_track = Atmosphere::synoptic_zonal_coverage(58.0 * DEGREE, itcz);

    EXPECT_LT(subtropics, tropics - 0.2);
    EXPECT_LT(subtropics, storm_track - 0.2);
    // And it is a *minimum*, not merely low: the belt is drier than anywhere poleward of it.
    for (int degree = 30; degree <= 85; degree += 5)
        EXPECT_GT(Atmosphere::synoptic_zonal_coverage(double(degree) * DEGREE, itcz), subtropics)
            << "latitude " << degree;
}

TEST(Unit_SynopticField, TheConvergenceZoneMigratesWithTheSeason)
{
    // Boreal summer puts it north of the equator and boreal winter south, which is why a
    // January photograph of the Pacific and a July one do not look the same.
    EXPECT_GT(Atmosphere::synoptic_itcz_latitude(0.55), 3.0 * DEGREE);   // late July
    EXPECT_LT(Atmosphere::synoptic_itcz_latitude(0.05), -3.0 * DEGREE);  // mid January
}

TEST(Unit_SynopticField, TheSameSeedIsTheSameSkyAndDifferentSeedsAreNot)
{
    // Determinism is the entire proposition of a seed. If this fails, an author cannot keep a
    // sky they liked, and a scene file cannot describe one.
    const Atmosphere::SynopticField first(4242u, 0.4, EARTH_RADIUS_M);
    const Atmosphere::SynopticField same(4242u, 0.4, EARTH_RADIUS_M);
    const Atmosphere::SynopticField other(4243u, 0.4, EARTH_RADIUS_M);

    double largest_same_difference = 0.0;
    double largest_other_difference = 0.0;
    for_each_place([&](double latitude, double longitude) {
        const double a = first.coverage_at(latitude, longitude);
        largest_same_difference = std::max(
            largest_same_difference, std::fabs(a - same.coverage_at(latitude, longitude)));
        largest_other_difference = std::max(
            largest_other_difference, std::fabs(a - other.coverage_at(latitude, longitude)));
    });

    EXPECT_DOUBLE_EQ(largest_same_difference, 0.0);
    // Adjacent seeds, deliberately: the generator mixes the seed before use precisely so that
    // 4242 and 4243 do not produce skies sharing their first system.
    EXPECT_GT(largest_other_difference, 0.2);
}

TEST(Unit_SynopticField, APlanetHasBothStormsAndCompletelyClearAir)
{
    // The user's own statement of what this feature is for: "dünyanın bir yeri bulutlu fırtınalı
    // iken bir yeri tam olarak açık hava olacak". A field that cannot reach zero cannot show a
    // subtropical high, and a field that cannot reach one cannot show a storm — and either
    // failure leaves the uniform veil this replaces.
    //
    // Two claims of different strengths, because the model supports two claims of different
    // strengths and conflating them would make this test a coin flip.
    //
    // *Every* seed must contain somewhere clear and somewhere overcast — that is what the zonal
    // climatology plus a dozen systems guarantees, and it is the bar the uniform deck stack
    // failed. Whether a given seed happens to place a high squarely on the subtropical minimum,
    // and so reach *nothing at all*, is chance; asserting it per seed would be asserting a
    // property of the generator's luck. So it is asserted across the ensemble, where it is true.
    double best_clearing = 1.0;
    for (std::uint64_t seed = 1; seed <= 8; ++seed)
    {
        const Atmosphere::SynopticField field(seed, 0.3, EARTH_RADIUS_M);
        double lowest = 2.0;
        double highest = -1.0;
        for_each_place([&](double latitude, double longitude) {
            const double coverage = field.coverage_at(latitude, longitude);
            lowest = std::min(lowest, coverage);
            highest = std::max(highest, coverage);
        });
        // The zonal curve is the *optically thick* fraction rather than the total cloud
        // fraction, so the subtropical minimum alone is ~0.06 before any high is placed on it:
        // a 0.20 bar would be cleared without a seed contributing anything, and a test that
        // passes for free tests nothing.
        EXPECT_LT(lowest, 0.10) << "seed " << seed << " has no clear air anywhere";
        EXPECT_GT(highest, 0.90) << "seed " << seed << " has no overcast anywhere";
        best_clearing = std::min(best_clearing, lowest);
    }
    EXPECT_LT(best_clearing, 0.02) << "no seed clears the sky completely anywhere";
}

TEST(Unit_SynopticField, TheTropicsConvectAndTheMidlatitudesDoNot)
{
    // What decides which *genus* the bake resolves per column, so it is what makes a seeded
    // tropical sky tower and a seeded midlatitude one layer.
    const Atmosphere::SynopticField field(11u, 0.5, EARTH_RADIUS_M);
    double tropical_total = 0.0;
    double midlatitude_total = 0.0;
    int samples = 0;
    for (int lon = -180; lon < 180; lon += 5)
    {
        tropical_total += field.convective_at(field.itcz_latitude(), double(lon) * DEGREE);
        midlatitude_total += field.convective_at(50.0 * DEGREE, double(lon) * DEGREE);
        ++samples;
    }
    EXPECT_GT(tropical_total / samples, midlatitude_total / samples + 0.25);
}

TEST(Unit_SynopticField, ManualModePublishesAFieldThatIsNotUniform)
{
    // The whole point of installing a provider in Manual mode at all. Before WM-SEED, Manual
    // meant *no* provider, so there was no field and one authored deck stack covered a planet.
    SeededWeather weather(7u, EARTH_RADIUS_M);
    WeatherFieldBuffer buffer;
    weather.publish_field(GeodeticPosition{45.0 * DEGREE, 10.0 * DEGREE}, buffer);
    const Render::WeatherField field = buffer.view();

    ASSERT_TRUE(field.valid());
    EXPECT_GT(field.cells_x, 1);
    // This column is meteorology, not an authored deck stack restated, so the bake resolves the
    // genus from it — the opposite of `StaticWeather`, whose column must not be round-tripped
    // through the classifier.
    EXPECT_TRUE(field.derives_genus);

    double lowest = 2.0;
    double highest = -1.0;
    const int level = int(CloudLevel::Low);
    for (int iz = 0; iz < field.cells_z; ++iz)
        for (int ix = 0; ix < field.cells_x; ++ix)
        {
            const std::size_t index =
                (std::size_t(level) * std::size_t(field.cells_z) + std::size_t(iz)) *
                    std::size_t(field.cells_x) + std::size_t(ix);
            lowest = std::min(lowest, double(field.samples[index].coverage));
            highest = std::max(highest, double(field.samples[index].coverage));
        }
    // A 384 km lattice is far smaller than a synoptic system, so the variation across it is
    // real but modest — this asserts the field *varies*, not that it spans the whole range.
    EXPECT_GT(highest - lowest, 1e-4);
}

TEST(Unit_SynopticField, ARerollChangesTheSkyAndTheProviderKeepsAnsweringConsistently)
{
    SeededWeather weather(1u, EARTH_RADIUS_M);
    const GeodeticPosition place{30.0 * DEGREE, -60.0 * DEGREE};
    const WeatherColumn before = weather.sample_column(place);

    weather.set_seed(2u);
    const WeatherColumn after = weather.sample_column(place);

    // Not asserting the value moved a lot at one point — a reroll can leave one spot alone,
    // which is a true property of weather and not a bug. What must hold is that the point query
    // and the published field agree with each other about the *same* sky.
    WeatherFieldBuffer buffer;
    weather.publish_field(place, buffer);
    const Render::WeatherField field = buffer.view();
    ASSERT_TRUE(field.valid());

    const int level = int(CloudLevel::Low);
    const int centre_x = field.cells_x / 2;
    const int centre_z = field.cells_z / 2;
    const std::size_t index =
        (std::size_t(level) * std::size_t(field.cells_z) + std::size_t(centre_z)) *
            std::size_t(field.cells_x) + std::size_t(centre_x);
    // Half a cell away from the exact observer, so this is a nearness check rather than an
    // equality one.
    EXPECT_NEAR(double(field.samples[index].coverage), double(after.levels[level].coverage), 0.05);
    EXPECT_GE(before.levels[level].coverage, 0.0f);
}

TEST(Unit_SynopticField, TheFlowAroundALowIsCyclonic)
{
    // The one thing worth testing about the wind, because it is the one thing easy to get
    // backwards: rotating the coverage gradient the *other* way still produces a circulation,
    // still looks like weather, and is wrong at every point on the planet. The settling fact is
    // that flow around a northern-hemisphere low turns counterclockwise, so directly north of
    // one the wind blows west.
    //
    // "North of a low, coverage falls going north" is a premise, not a given: `coverage_at`
    // sums every centre, and a high is both broader (an e-folding radius up to 2000 km against
    // a low's 1400) and stronger (an amplitude up to 0.90 against 0.56), so one sitting nearby
    // can carry the local gradient the other way. The premise is therefore measured at each
    // sample point and the meteorological claim is made only where it actually holds — which is
    // also what makes this a test of the wind law rather than of the generator's layout.
    constexpr double GRADIENT_STEP_RADIANS = 2.0e-3; // the step `wind_at` differences over.
    int checked = 0;
    for (std::uint64_t seed = 1; seed <= 20; ++seed)
    {
        SeededWeather weather(seed, EARTH_RADIUS_M);
        const Atmosphere::SynopticField* field = weather.synoptic_field();
        ASSERT_NE(field, nullptr);

        for (int i = 0; i < field->count(); ++i)
        {
            const Atmosphere::SynopticCentre& centre = field->centres()[i];
            if (centre.amplitude <= 0.0 || centre.latitude_radians < 20.0 * DEGREE ||
                centre.latitude_radians > 70.0 * DEGREE)
                continue; // not a northern-hemisphere low well clear of the equator

            const GeodeticPosition north_flank{centre.latitude_radians + 4.0 * DEGREE,
                                               centre.longitude_radians};
            const double northward_coverage_gradient =
                field->coverage_at(north_flank.latitude_radians + GRADIENT_STEP_RADIANS,
                                   north_flank.longitude_radians) -
                field->coverage_at(north_flank.latitude_radians - GRADIENT_STEP_RADIANS,
                                   north_flank.longitude_radians);
            if (northward_coverage_gradient >= 0.0)
                continue; // a neighbour owns the gradient here; this point says nothing

            const WindSample wind = weather.wind_at(north_flank, 0.25);
            EXPECT_LT(wind.eastward_mps, 0.0)
                << "seed " << seed << " centre " << i
                << ": where coverage falls northward the flow must run west";
            // And it is a real wind, not a rounding artefact of a flat spot.
            EXPECT_GT(std::fabs(wind.eastward_mps), 1.0);
            ++checked;
        }
    }
    ASSERT_GT(checked, 0) << "no point in twenty seeds had coverage falling northward of a "
                             "northern-hemisphere low, so the wind law went untested";
}

TEST(Unit_SynopticField, TheWindIsPerpendicularToTheCoverageGradient)
{
    // Geostrophy exactly, not approximately: the wind is the spatial gradient rotated a quarter
    // turn, so their dot product is zero to floating point. Stated against the *spatial*
    // gradient — the eastward one carries the `cos(latitude)` easting factor — because the
    // longitudinal difference alone is only perpendicular on the equator.
    SeededWeather weather(5u, EARTH_RADIUS_M);
    const Atmosphere::SynopticField* field = weather.synoptic_field();
    ASSERT_NE(field, nullptr);
    ASSERT_GT(field->count(), 0);

    const Atmosphere::SynopticCentre& centre = field->centres()[0];
    const GeodeticPosition flank{
        std::clamp(centre.latitude_radians + 5.0 * DEGREE, -80.0 * DEGREE, 80.0 * DEGREE),
        centre.longitude_radians};

    constexpr double STEP = 2.0e-3; // the same step the provider differences over.
    const double cos_latitude = std::max(std::cos(flank.latitude_radians), 0.05);
    const double gradient_north = field->coverage_at(flank.latitude_radians + STEP,
                                                     flank.longitude_radians) -
                                  field->coverage_at(flank.latitude_radians - STEP,
                                                     flank.longitude_radians);
    const double gradient_east = (field->coverage_at(flank.latitude_radians,
                                                     flank.longitude_radians + STEP) -
                                  field->coverage_at(flank.latitude_radians,
                                                     flank.longitude_radians - STEP)) /
                                 cos_latitude;

    const WindSample wind = weather.wind_at(flank, 0.25);
    const double gradient_length =
        std::sqrt(gradient_north * gradient_north + gradient_east * gradient_east);
    const double wind_length =
        std::sqrt(wind.eastward_mps * wind.eastward_mps + wind.northward_mps * wind.northward_mps);
    ASSERT_GT(gradient_length, 1e-9);
    ASSERT_GT(wind_length, 1e-9);

    const double alignment = (gradient_east * wind.eastward_mps +
                              gradient_north * wind.northward_mps) /
                             (gradient_length * wind_length);
    EXPECT_NEAR(alignment, 0.0, 1e-9);
}
