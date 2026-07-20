/**************************************************************************/
/* julian_date.hpp                                                        */
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

#pragma once

/**
 * @file julian_date.hpp
 * @brief Astronomical time: Julian Date, the J2000 epoch, and sidereal time.
 *
 * Every ephemeris in this module is a function of time expressed as a Julian Date
 * (JD) — a continuous count of days that removes calendar irregularities from the
 * orbital maths. The planetary tables are polynomials in Julian centuries since the
 * J2000.0 standard epoch; the topocentric transform needs Greenwich Mean Sidereal
 * Time (the rotation angle of the Earth about its axis). This header is the single
 * place those quantities are derived, in double precision throughout.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Astro
    {
        /** @brief The Julian Date of the J2000.0 standard epoch (2000-01-01 12:00 TT). */
        constexpr double J2000_JULIAN_DATE = 2451545.0;

        /** @brief Days per Julian century, the unit the planetary element rates use. */
        constexpr double DAYS_PER_JULIAN_CENTURY = 36525.0;

        /** @brief Degrees to radians. */
        constexpr double DEGREES_TO_RADIANS = 0.017453292519943295;

        /** @brief Radians to degrees. */
        constexpr double RADIANS_TO_DEGREES = 57.29577951308232;

        /** @brief Mean obliquity of the ecliptic at J2000.0, radians (23.4392911 deg). */
        constexpr double OBLIQUITY_J2000_RADIANS = 0.40909280422232897;

        /**
         * @brief A civil date and time, the human-facing input the editor authors.
         *
         * Interpreted as Universal Time; the small UT/TT difference is ignored, which
         * is far below the visual accuracy of the approximate ephemerides here.
         */
        struct CalendarDate
        {
            int year = 2000;    /**< Gregorian year (proleptic before 1582). */
            int month = 1;      /**< Month in [1, 12]. */
            int day = 1;        /**< Day of month in [1, 31]. */
            int hour = 12;      /**< Hour in [0, 23]. */
            int minute = 0;     /**< Minute in [0, 59]. */
            double second = 0.0; /**< Second in [0, 60). */
        };

        /**
         * @brief Converts a Gregorian civil date to a Julian Date.
         *
         * Uses the standard Fliegel-Van Flandern algorithm extended with the fractional
         * day from the clock time; valid across the whole proleptic Gregorian calendar.
         *
         * @param date The civil date and time, interpreted as Universal Time.
         * @return The Julian Date (days).
         */
        inline double julian_date_from_calendar(const CalendarDate& date) noexcept
        {
            int year = date.year;
            int month = date.month;
            if (month <= 2)
            {
                year -= 1;
                month += 12;
            }
            const int a = year / 100;
            const int b = 2 - a + a / 4;
            const double day_fraction =
                (static_cast<double>(date.hour) + static_cast<double>(date.minute) / 60.0 +
                 date.second / 3600.0) /
                24.0;
            const double jd =
                std::floor(365.25 * (year + 4716)) + std::floor(30.6001 * (month + 1)) +
                date.day + b - 1524.5 + day_fraction;
            return jd;
        }

        /**
         * @brief Julian centuries elapsed from J2000.0 to a Julian Date.
         * @param julian_date The Julian Date to measure from J2000.0.
         * @return (jd - 2451545.0) / 36525.
         */
        inline double julian_centuries_since_j2000(double julian_date) noexcept
        {
            return (julian_date - J2000_JULIAN_DATE) / DAYS_PER_JULIAN_CENTURY;
        }

        /**
         * @brief Wraps an angle in degrees into the [0, 360) range.
         * @param degrees The angle to wrap.
         * @return The equivalent angle in [0, 360).
         */
        inline double wrap_degrees(double degrees) noexcept
        {
            double wrapped = std::fmod(degrees, 360.0);
            if (wrapped < 0.0)
                wrapped += 360.0;
            return wrapped;
        }

        /**
         * @brief Wraps an angle in radians into the [-pi, pi] range.
         * @param radians The angle to wrap.
         * @return The equivalent angle in [-pi, pi].
         */
        inline double wrap_radians_signed(double radians) noexcept
        {
            constexpr double two_pi = 6.283185307179586;
            double wrapped = std::fmod(radians, two_pi);
            if (wrapped > 3.141592653589793)
                wrapped -= two_pi;
            else if (wrapped < -3.141592653589793)
                wrapped += two_pi;
            return wrapped;
        }

        /**
         * @brief Greenwich Mean Sidereal Time at a Julian Date, in radians.
         *
         * The angle from the Greenwich meridian to the vernal equinox — i.e. how far the
         * Earth has rotated relative to the stars. Uses the IAU 1982 polynomial in the
         * days-since-J2000 form; the returned angle is wrapped to [0, 2pi).
         *
         * @param julian_date The Julian Date (UT).
         * @return GMST in radians, in [0, 2pi).
         */
        inline double greenwich_mean_sidereal_time(double julian_date) noexcept
        {
            const double d = julian_date - J2000_JULIAN_DATE;
            const double t = d / DAYS_PER_JULIAN_CENTURY;
            double gmst_degrees =
                280.46061837 + 360.98564736629 * d + 0.000387933 * t * t - t * t * t / 38710000.0;
            return wrap_degrees(gmst_degrees) * DEGREES_TO_RADIANS;
        }

        /**
         * @brief Local Mean Sidereal Time for an observer, in radians.
         *
         * GMST advanced by the observer's east longitude — the hour circle overhead,
         * which orients the whole celestial sphere for the topocentric transform.
         *
         * @param julian_date          The Julian Date (UT).
         * @param east_longitude_radians Observer east longitude (east positive), radians.
         * @return Local sidereal time in radians, in [0, 2pi).
         */
        inline double local_mean_sidereal_time(double julian_date,
                                               double east_longitude_radians) noexcept
        {
            const double lst = greenwich_mean_sidereal_time(julian_date) + east_longitude_radians;
            constexpr double two_pi = 6.283185307179586;
            double wrapped = std::fmod(lst, two_pi);
            if (wrapped < 0.0)
                wrapped += two_pi;
            return wrapped;
        }
    } // namespace Astro
} // namespace SushiEngine
