/**************************************************************************/
/* season.hpp                                                             */
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
 * @file season.hpp
 * @brief Turns the simulation's epoch into the position in the year T0 is indexed by.
 *
 * **This lives on the simulation side because the atmosphere does not know what a calendar
 * is, and should not learn.** `sushi_atmosphere` is engine-neutral (see its `CMakeLists.txt`):
 * `Climatology` and `QuasiGeostrophicCore` speak in a year fraction, which is a position in an
 * abstract cycle and works for any body with any period. A Julian Date is a fact about Earth's
 * civil calendar, and this is the seam where one becomes the other.
 */

#include <cmath>

#include <SushiEngine/astro/julian_date.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Months in the calendar the climatology's month axis is indexed by. */
        constexpr int MONTHS_PER_YEAR = 12;

        /**
         * @brief Days in @p month of @p year, Gregorian.
         * @param year  Gregorian year, for the leap rule.
         * @param month Month in [1, 12].
         */
        inline int days_in_month(int year, int month) noexcept
        {
            switch (month)
            {
                case 2:
                {
                    const bool leap =
                        (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
                    return leap ? 29 : 28;
                }
                case 4:
                case 6:
                case 9:
                case 11:
                    return 30;
                default:
                    return 31;
            }
        }

        /**
         * @brief Position in the year for T0, in [0, 1).
         *
         * **Measured in months, not in days, and deliberately.** T0's profiles are twelve
         * monthly fields and `Climatology::sample_profile` spreads the year fraction across
         * them as twelve equal bins. Calendar months are not equal — February is 28 days and
         * July is 31 — so a fraction built from the day of the year would drift up to a day and
         * a half out of phase with the bin it is meant to select, with the error largest in
         * exactly the months whose climatology changes fastest. Building it from the month index
         * plus the fraction elapsed *within that month* puts the middle of July on the middle of
         * July's bin, which is where the data it is interpolating actually sits.
         *
         * @param julian_date The simulation epoch, Universal Time.
         * @return Position in the year; 0 is the start of January.
         */
        inline double year_fraction_from_julian_date(double julian_date) noexcept
        {
            const Astro::CalendarDate date = Astro::calendar_from_julian_date(julian_date);
            const double time_of_day =
                (static_cast<double>(date.hour) + static_cast<double>(date.minute) / 60.0 +
                 date.second / 3600.0) /
                24.0;
            const double days = static_cast<double>(days_in_month(date.year, date.month));
            const double within_month =
                (static_cast<double>(date.day - 1) + time_of_day) / days;
            const double fraction =
                (static_cast<double>(date.month - 1) + within_month) /
                static_cast<double>(MONTHS_PER_YEAR);

            // A date on the last instant of December can round to exactly 1.0, which is not in
            // the range this promises; wrap rather than clamp, because the year is a cycle.
            const double wrapped = fraction - std::floor(fraction);
            return wrapped >= 1.0 ? 0.0 : wrapped;
        }
    } // namespace Simulation
} // namespace SushiEngine
