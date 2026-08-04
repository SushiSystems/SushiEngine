/**************************************************************************/
/* test_season.cpp                                                        */
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

// Unit_Season: the path from the simulation's epoch to T0's month axis, and what the global
// core does when it moves (`docs/design/atmosphere_system.md` §4.2).
//
// Three separable claims, and each has failed at least once during development:
//
//  1. The year fraction indexes the *month bin* the data actually sits in. T0 holds twelve
//     fields sampled as twelve equal bins, and calendar months are not equal, so a fraction
//     built from the day of the year drifts out of phase with the data it is selecting.
//  2. A season moves the mean state and nothing else. Potential vorticity and column water are
//     prognostic and must carry straight through.
//  3. The mean state is a function of the *date*, not of the call history. This is the one that
//     §3.4's determinism rests on: a "rebuild once it has moved enough" threshold would make two
//     hosts ticking at different rates diverge, and it would do so silently.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>
#include <SushiEngine/simulation/season.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using SushiEngine::Astro::CalendarDate;
using SushiEngine::Astro::julian_date_from_calendar;
using SushiEngine::Atmosphere::AnalyticClimatology;
using SushiEngine::Atmosphere::Climatology;
using SushiEngine::Atmosphere::GeographicPosition;
using SushiEngine::Atmosphere::QuasiGeostrophicCore;
using SushiEngine::Atmosphere::QuasiGeostrophicGridSize;
using SushiEngine::Atmosphere::QuasiGeostrophicParameters;
using SushiEngine::Simulation::year_fraction_from_julian_date;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief A grid small enough to build many cores in a unit test. */
    QuasiGeostrophicGridSize small_grid()
    {
        QuasiGeostrophicGridSize size;
        size.longitude_cells = 64;
        size.latitude_cells = 32;
        return size;
    }

    /**
     * @brief A climatology with a season the test authors itself.
     *
     * The analytic bands have no season by construction, and the shipped asset may not be
     * checked out, so a blob is synthesized here: the upper wind is written per month so the
     * northern jet is strong in January and weak in July, which is the only property these
     * tests need to be able to see.
     */
    Climatology seasonal_climatology()
    {
        constexpr std::int32_t BANDS = 8;
        constexpr std::int32_t MONTHS = 12;
        std::vector<std::uint8_t> blob;
        const auto put = [&blob](const auto& value) {
            const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
            blob.insert(blob.end(), bytes, bytes + sizeof(value));
        };
        const char magic[4] = {'S', 'E', 'T', '0'};
        blob.insert(blob.end(), magic, magic + 4);
        put(std::uint32_t(1));
        put(BANDS);
        put(MONTHS);
        put(std::int32_t(0)); // no surface fields; they are not what this file is about
        put(std::int32_t(0));
        put(std::uint32_t(0));

        // Upper wind: a cosine in the month, peaking in January (month 0) at 40 m/s and
        // troughing in July at 10. Uniform in latitude, so nothing here depends on the band.
        for (int month = 0; month < MONTHS; ++month)
            for (int band = 0; band < BANDS; ++band)
            {
                (void)band;
                put(float(25.0 + 15.0 * std::cos(2.0 * PI * (month + 0.5) / MONTHS)));
            }
        for (int i = 0; i < BANDS * MONTHS; ++i)
            put(float(5.0));  // lower wind: constant, so the shear follows the upper layer
        for (int i = 0; i < BANDS * MONTHS; ++i)
            put(float(50.0)); // saturation: constant, so moisture is not the variable here

        Climatology climatology;
        EXPECT_TRUE(climatology.adopt(blob));
        return climatology;
    }
} // namespace

// 1. The calendar, onto the month axis.

TEST(Unit_Season, TheMiddleOfAMonthLandsOnTheMiddleOfItsBin)
{
    // The claim that day-of-year would fail: February is 28 days and July is 31, so a
    // fraction built from the day of the year drifts out of phase with the equal bins T0 is
    // sampled as -- worst in the months whose climatology changes fastest.
    for (int month = 1; month <= 12; ++month)
    {
        const int day = (Simulation::days_in_month(2001, month) + 1) / 2;
        const double fraction =
            year_fraction_from_julian_date(julian_date_from_calendar({2001, month, day, 0, 0, 0.0}));
        const double bin_centre = (double(month) - 0.5) / 12.0;
        EXPECT_NEAR(fraction, bin_centre, 0.02) << "month " << month;
    }
}

TEST(Unit_Season, TheYearStartsAtZeroAndNeverReachesOne)
{
    EXPECT_NEAR(year_fraction_from_julian_date(
                    julian_date_from_calendar({2001, 1, 1, 0, 0, 0.0})), 0.0, 1e-12);
    const double new_years_eve = year_fraction_from_julian_date(
        julian_date_from_calendar({2001, 12, 31, 23, 59, 59.0}));
    EXPECT_LT(new_years_eve, 1.0);
    EXPECT_GT(new_years_eve, 0.99);
}

TEST(Unit_Season, LeapYearsChangeFebruaryAndNothingElse)
{
    EXPECT_EQ(Simulation::days_in_month(2024, 2), 29);
    EXPECT_EQ(Simulation::days_in_month(2023, 2), 28);
    EXPECT_EQ(Simulation::days_in_month(1900, 2), 28) << "a century that is not a leap year";
    EXPECT_EQ(Simulation::days_in_month(2000, 2), 29) << "but a fourth century is";
    EXPECT_EQ(Simulation::days_in_month(2024, 7), 31);
    EXPECT_EQ(Simulation::days_in_month(2024, 4), 30);
}

// 2. What the season does to the core.

TEST(Unit_Season, TheSeasonMovesTheMeanStateAndNotTheFlow)
{
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters(), seasonal_climatology());
    ASSERT_TRUE(core.valid());
    core.seed(7);

    // Whatever the flow is, the moment before and the moment after the season moves must be
    // the same flow. A season changes what the flow relaxes *toward*; it is not a force.
    const GeographicPosition sample{0.7, 1.0};
    const double before_pressure = core.pressure_anomaly_hpa(sample);
    const double before_water = core.precipitable_water_at(sample);
    const double before_seconds = core.simulated_seconds();

    ASSERT_TRUE(core.set_year_fraction(0.5));

    EXPECT_DOUBLE_EQ(core.pressure_anomaly_hpa(sample), before_pressure);
    EXPECT_DOUBLE_EQ(core.precipitable_water_at(sample), before_water);
    EXPECT_DOUBLE_EQ(core.simulated_seconds(), before_seconds);
}

TEST(Unit_Season, ASeededCoreCarriesTheJetOfTheSeasonItWasSeededIn)
{
    // Seeding reads the mean state, so the season has to be set before the seed -- otherwise a
    // scene opening in July starts with January's jet and spends its first simulated weeks
    // migrating, which reads as the weather being wrong rather than as a transient.
    const Climatology climatology = seasonal_climatology();
    QuasiGeostrophicCore january(small_grid(), QuasiGeostrophicParameters(), climatology);
    QuasiGeostrophicCore july(small_grid(), QuasiGeostrophicParameters(), climatology);
    ASSERT_TRUE(january.valid());
    ASSERT_TRUE(july.valid());

    january.set_year_fraction(0.0);
    july.set_year_fraction(0.5);
    january.seed(3);
    july.seed(3);

    // The synthesized climatology peaks in January and troughs in July, so the diagnosed jet
    // must follow it -- and by a wide margin, since the authored contrast is 40 against 10.
    EXPECT_GT(january.diagnostics().jet_speed_mps, july.diagnostics().jet_speed_mps * 1.5);
}

TEST(Unit_Season, TheDateAloneDecidesTheMeanStateAndNotHowOftenItWasAsked)
{
    // §3.4. The tempting optimization -- rebuild only once the date has moved "enough" -- makes
    // the mean state depend on the call history, so a host ticking at 60 Hz and one ticking at
    // 6 Hz would diverge. Quantizing instead makes it a pure function of the date, and this is
    // the test that would fail the moment somebody replaces it with a threshold.
    const Climatology climatology = seasonal_climatology();
    QuasiGeostrophicCore few(small_grid(), QuasiGeostrophicParameters(), climatology);
    QuasiGeostrophicCore many(small_grid(), QuasiGeostrophicParameters(), climatology);
    ASSERT_TRUE(few.valid());
    ASSERT_TRUE(many.valid());

    const double start = 0.25;
    const double finish = 0.30;
    few.set_year_fraction(start);
    many.set_year_fraction(start);
    for (int i = 1; i <= 10; ++i)
        few.set_year_fraction(start + (finish - start) * double(i) / 10.0);
    for (int i = 1; i <= 1000; ++i)
        many.set_year_fraction(start + (finish - start) * double(i) / 1000.0);

    EXPECT_DOUBLE_EQ(few.year_fraction(), many.year_fraction());

    few.seed(11);
    many.seed(11);
    EXPECT_DOUBLE_EQ(few.diagnostics().jet_speed_mps, many.diagnostics().jet_speed_mps);
}

TEST(Unit_Season, TheSameDayIsNotRebuiltTwice)
{
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters(), seasonal_climatology());
    ASSERT_TRUE(core.valid());
    core.seed(1);

    // The return value is what lets a caller tick this every frame without wondering: true
    // exactly when the mean state moved.
    EXPECT_TRUE(core.set_year_fraction(0.5));
    EXPECT_FALSE(core.set_year_fraction(0.5));
    EXPECT_FALSE(core.set_year_fraction(0.5 + 0.5 / 365.0 * 0.25)) << "still the same day";
    EXPECT_TRUE(core.set_year_fraction(0.5 + 2.0 / 365.0)) << "two days on";
}

TEST(Unit_Season, TheYearWrapsRatherThanClamping)
{
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters(), seasonal_climatology());
    ASSERT_TRUE(core.valid());
    core.set_year_fraction(0.25);
    const double quarter = core.year_fraction();
    core.set_year_fraction(0.9);
    core.set_year_fraction(1.25);
    EXPECT_DOUBLE_EQ(core.year_fraction(), quarter);
    core.set_year_fraction(0.9);
    core.set_year_fraction(-0.75);
    EXPECT_DOUBLE_EQ(core.year_fraction(), quarter);
}

TEST(Unit_Season, AnAnalyticCoreHasNoSeasonAndSaysSoByNotMoving)
{
    // Analytic latitude bands are a statement about latitude alone, so a body with no baked
    // climatology has a mean state that does not vary through the year. The call must still be
    // safe -- a host does not branch on this.
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters(), Climatology());
    ASSERT_TRUE(core.valid());
    core.seed(5);
    const double winter = core.diagnostics().jet_speed_mps;
    EXPECT_TRUE(core.set_year_fraction(0.5));
    core.seed(5);
    EXPECT_DOUBLE_EQ(core.diagnostics().jet_speed_mps, winter);
}
