/**************************************************************************/
/* test_julian_date.cpp                                                   */
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

// Unit_JulianDate: astronomical time. The J2000 epoch anchors every ephemeris here,
// so the calendar->JD conversion must return exactly 2451545.0 for 2000-01-01 12:00 UT
// and zero Julian centuries there; sidereal time must stay wrapped; and the angle
// wrappers must land in their documented ranges. These are the reference landmarks the
// rest of the astro module is built on, so an error here fans out everywhere.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/julian_date.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_JulianDate, J2000EpochIsExactReferenceValue)
{
    // 2000-01-01 12:00:00 TT is the definition of J2000.0; the conversion must land on
    // the tabulated Julian Date to the bit, since every centuries-since-J2000 term keys
    // off it.
    const CalendarDate j2000{2000, 1, 1, 12, 0, 0.0};
    EXPECT_DOUBLE_EQ(julian_date_from_calendar(j2000), J2000_JULIAN_DATE);
    EXPECT_DOUBLE_EQ(julian_date_from_calendar(j2000), 2451545.0);
}

TEST(Unit_JulianDate, MidnightIsHalfADayBeforeNoon)
{
    // The half-day offset between civil midnight and the JD noon boundary.
    const CalendarDate midnight{2000, 1, 1, 0, 0, 0.0};
    EXPECT_DOUBLE_EQ(julian_date_from_calendar(midnight), 2451544.5);
}

TEST(Unit_JulianDate, KnownGregorianDateMatchesReference)
{
    // 2024-01-01 00:00 UT -> JD 2460310.5 (standard reference).
    const CalendarDate date{2024, 1, 1, 0, 0, 0.0};
    EXPECT_NEAR(julian_date_from_calendar(date), 2460310.5, 1e-6);
}

TEST(Unit_JulianDate, CenturiesAreZeroAtEpochAndOneAfter36525Days)
{
    EXPECT_DOUBLE_EQ(julian_centuries_since_j2000(J2000_JULIAN_DATE), 0.0);
    EXPECT_DOUBLE_EQ(julian_centuries_since_j2000(J2000_JULIAN_DATE + DAYS_PER_JULIAN_CENTURY),
                     1.0);
    EXPECT_DOUBLE_EQ(julian_centuries_since_j2000(J2000_JULIAN_DATE - DAYS_PER_JULIAN_CENTURY),
                     -1.0);
}

TEST(Unit_JulianDate, WrapDegreesLandsInZeroToThreeSixty)
{
    EXPECT_NEAR(wrap_degrees(0.0), 0.0, 1e-12);
    EXPECT_NEAR(wrap_degrees(360.0), 0.0, 1e-12);
    EXPECT_NEAR(wrap_degrees(-90.0), 270.0, 1e-12);
    EXPECT_NEAR(wrap_degrees(450.0), 90.0, 1e-12);
    EXPECT_NEAR(wrap_degrees(-720.5), 359.5, 1e-9);

    for (double d = -2000.0; d <= 2000.0; d += 37.3)
    {
        const double w = wrap_degrees(d);
        EXPECT_GE(w, 0.0);
        EXPECT_LT(w, 360.0);
    }
}

TEST(Unit_JulianDate, WrapRadiansSignedLandsInMinusPiToPi)
{
    constexpr double pi = 3.141592653589793;
    EXPECT_NEAR(wrap_radians_signed(0.0), 0.0, 1e-12);
    EXPECT_NEAR(wrap_radians_signed(2.0 * pi), 0.0, 1e-9);
    EXPECT_NEAR(wrap_radians_signed(3.0 * pi), pi, 1e-9);

    for (double r = -20.0; r <= 20.0; r += 0.37)
    {
        const double w = wrap_radians_signed(r);
        EXPECT_GE(w, -pi - 1e-9);
        EXPECT_LE(w, pi + 1e-9);
    }
}

TEST(Unit_JulianDate, GreenwichSiderealTimeIsWrappedToTwoPi)
{
    constexpr double two_pi = 6.283185307179586;
    for (double jd = J2000_JULIAN_DATE; jd < J2000_JULIAN_DATE + 5.0; jd += 0.13)
    {
        const double gmst = greenwich_mean_sidereal_time(jd);
        EXPECT_GE(gmst, 0.0);
        EXPECT_LT(gmst, two_pi);
    }
}

TEST(Unit_JulianDate, SiderealDayIsShorterThanASolarDay)
{
    // Earth turns 360.98565 deg/solar-day relative to the stars, so GMST after one full
    // solar day has advanced by very nearly a full turn plus the ~1 deg the Sun moved:
    // the sidereal angle returns almost to where it was, offset by ~0.0171 rad.
    const double a = greenwich_mean_sidereal_time(J2000_JULIAN_DATE);
    const double b = greenwich_mean_sidereal_time(J2000_JULIAN_DATE + 1.0);
    double delta = b - a;
    while (delta < 0.0)
        delta += 6.283185307179586;
    EXPECT_NEAR(delta, 0.0172026, 1e-4);
}

TEST(Unit_JulianDate, LocalSiderealTimeAddsEastLongitude)
{
    constexpr double two_pi = 6.283185307179586;
    const double jd = J2000_JULIAN_DATE + 12.3;
    const double gmst = greenwich_mean_sidereal_time(jd);
    const double east = 1.0; // radians east
    double expected = std::fmod(gmst + east, two_pi);
    if (expected < 0.0)
        expected += two_pi;
    EXPECT_NEAR(local_mean_sidereal_time(jd, east), expected, 1e-9);
}

// The inverse. `year_fraction_from_julian_date` reads a month out of an epoch to index T0's
// climatology, so a date that comes back one day out puts the weather in the wrong month --
// and the failure would look like a slightly wrong jet rather than like a calendar bug.

TEST(Unit_JulianDate, CalendarSurvivesTheRoundTripThroughJulianDate)
{
    // Deliberately awkward: a leap day, a century that is *not* a leap year under the
    // Gregorian rule, both ends of a day, and a date before the epoch (where the
    // truncation in the algorithm has to floor rather than round toward zero).
    const CalendarDate dates[] = {
        {2000, 1, 1, 12, 0, 0.0},
        {2026, 7, 29, 3, 45, 30.0},
        {1969, 12, 31, 23, 59, 0.0},
        {2024, 2, 29, 6, 0, 0.0},
        {1900, 3, 1, 0, 0, 0.0},
        {2100, 2, 28, 18, 30, 15.0},
    };
    for (const CalendarDate& date : dates)
    {
        const CalendarDate back =
            calendar_from_julian_date(julian_date_from_calendar(date));
        EXPECT_EQ(back.year, date.year);
        EXPECT_EQ(back.month, date.month);
        EXPECT_EQ(back.day, date.day);
        EXPECT_EQ(back.hour, date.hour);
        EXPECT_EQ(back.minute, date.minute);
        EXPECT_NEAR(back.second, date.second, 0.01);
    }
}

TEST(Unit_JulianDate, TheInverseAgreesWithTheEpochItDefines)
{
    const CalendarDate epoch = calendar_from_julian_date(J2000_JULIAN_DATE);
    EXPECT_EQ(epoch.year, 2000);
    EXPECT_EQ(epoch.month, 1);
    EXPECT_EQ(epoch.day, 1);
    EXPECT_EQ(epoch.hour, 12);
    EXPECT_EQ(epoch.minute, 0);
}

TEST(Unit_JulianDate, HalfADayEarlierIsTheMidnightThatStartsTheSameDate)
{
    // The Julian day rolls over at noon and the civil day at midnight; the conversion has
    // to carry that offset, and getting it backwards moves every date by one.
    const CalendarDate midnight = calendar_from_julian_date(J2000_JULIAN_DATE - 0.5);
    EXPECT_EQ(midnight.year, 2000);
    EXPECT_EQ(midnight.month, 1);
    EXPECT_EQ(midnight.day, 1);
    EXPECT_EQ(midnight.hour, 0);
}
