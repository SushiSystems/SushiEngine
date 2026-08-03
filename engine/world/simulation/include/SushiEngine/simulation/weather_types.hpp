/**************************************************************************/
/* weather_types.hpp                                                     */
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
 * @file weather_types.hpp
 * @brief Plain value types shared by the T1 synoptic layer and the T2 regional grid.
 *
 * `docs/slop/weather_and_clouds.md` §3/§5 describes three weather tiers; this file
 * carries the small, dependency-free types the sim-domain tiers (T1, T2) and the
 * `IWeatherProvider` seam pass between each other. Deliberately free of any
 * `Astro::`/`Render::` dependency (the same reason `Render::SkyObserver` keeps a
 * plain `int observer_body` rather than an `Astro::BodyId`) so the weather
 * simulation stays a self-contained sim-domain module; the one conversion point to
 * real render/astro types lives at the `RuntimeSimulation` boundary.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief A point on the dominant body's surface, geodetic radians.
         *
         * Mirrors `Render::SkyObserver`'s latitude/longitude fields exactly (same
         * units, same sign convention: east-positive longitude) so a caller can pass
         * the observer position straight through without a conversion.
         */
        struct GeodeticPosition
        {
            double latitude_radians = 0.0;  /**< Geodetic latitude, [-pi/2, pi/2]. */
            double longitude_radians = 0.0; /**< East longitude, any range (wrapped as needed). */
        };

        /** @brief Wind at a point: eastward/northward components, metres per second. */
        struct WindSample
        {
            double eastward_mps = 0.0;
            double northward_mps = 0.0;
        };

        /**
         * @brief Where a scene's weather comes from.
         *
         * A named mode rather than a boolean, and the difference is not cosmetic. "Procedural
         * weather is on" used to be answered by `static_cast<bool>(weather_provider_)`, which
         * made "no provider at all" the *definition* of the other mode — so Manual could not
         * have a provider, and therefore could not have a weather field, and therefore applied
         * one authored deck stack to an entire planet. See `docs/slop/atmosphere_system.md`'s
         * WM-SEED for what that looked like from orbit.
         *
         * Both modes install a provider now, and the choice is genuinely about *where the sky
         * comes from* rather than about whether anything is running:
         *
         * * @c Manual — placed. `SeededWeather` puts a dozen pressure systems on a zonal
         *   climatology from a seed. Deterministic, defined over the whole body, costs nothing
         *   to run, and does not evolve. What an author asking for a specific sky wants.
         * * @c Procedural — grown. `ProceduralWeather`'s quasi-geostrophic core and the GPU
         *   regional nest. Evolves on its own, and only resolves the nest's own footprint.
         */
        enum class WeatherMode : std::uint32_t
        {
            Manual = 0,
            Procedural,
        };

        /** @brief The stable string form of a @ref WeatherMode, for scene files and panels. */
        inline const char* weather_mode_name(WeatherMode mode) noexcept
        {
            return mode == WeatherMode::Procedural ? "procedural" : "manual";
        }

        /**
         * @brief The three WMO vertical étages T2 tracks per grid column.
         *
         * `Render::CloudGenus`/`cloud_genus_profile` already partitions the ten genera into
         * high/middle/low+vertical bands; T2 does not need a finer vertical discretization
         * than the bridge that ultimately picks a genus per band can use (see
         * `weather_cloudscape_compiler.hpp`), so it tracks exactly these three rather than
         * the design doc's 8-16 raw levels — see the CHANGELOG entry for why.
         */
        enum class CloudLevel : std::uint32_t
        {
            Low = 0,
            Mid,
            High,
            Count,
        };

        /** @brief Number of levels in @ref CloudLevel (excludes @c Count). */
        constexpr int CLOUD_LEVEL_COUNT = static_cast<int>(CloudLevel::Count);

        /**
         * @brief Rough altitude boundaries a @ref CloudLevel bucket spans, metres.
         *
         * `CloudLevel` has no continuous altitude of its own (see the type's own doc: it is
         * three WMO étage buckets, not a profile), but W6's flight-hazard queries and its METAR
         * ingestion both need to place an arbitrary altitude into one of the three buckets the
         * rest of the bridge already speaks in. These thresholds are the same coarse split
         * `cloud_genus_profile`'s own étage bands imply (low genera top out around 1.2-3.2 km,
         * middle genera sit roughly 3-5.5 km, high genera start above ~6.5 km); centralized here
         * once rather than re-guessed at each call site.
         */
        constexpr double CLOUD_LEVEL_LOW_CEILING_METERS = 2500.0;
        constexpr double CLOUD_LEVEL_MID_CEILING_METERS = 6000.0;

        /**
         * @brief Altitude each @ref CloudLevel band is taken to be centred on, metres.
         *
         * The ceilings above answer "which bucket does this altitude fall in", which is what
         * a discrete query wants. A *field* consumer has the opposite problem: given a
         * continuous altitude, how much of each band applies — a march sample climbing
         * through 2 500 m should not see low-level coverage switch off and mid-level switch
         * on in one step. These centres are what the renderer interpolates between
         * (`Render::WeatherField::level_altitudes`), placed mid-band for the two bounded
         * étages and at a representative cirrus altitude for the unbounded top one.
         */
        constexpr double CLOUD_LEVEL_LOW_CENTER_METERS = 1250.0;
        constexpr double CLOUD_LEVEL_MID_CENTER_METERS = 4250.0;
        constexpr double CLOUD_LEVEL_HIGH_CENTER_METERS = 9000.0;

        /**
         * @brief Buckets an altitude into the @ref CloudLevel it falls in.
         * @param altitude_meters Height above the surface, metres.
         * @return @ref CloudLevel::Low / @c Mid / @c High per the thresholds above.
         */
        inline CloudLevel cloud_level_for_altitude(double altitude_meters) noexcept
        {
            if (altitude_meters < CLOUD_LEVEL_LOW_CEILING_METERS)
                return CloudLevel::Low;
            if (altitude_meters < CLOUD_LEVEL_MID_CEILING_METERS)
                return CloudLevel::Mid;
            return CloudLevel::High;
        }

        /**
         * @brief One vertical band's meteorology, as the render bridge consumes it.
         *
         * The narrow slice of T2's internal cell state that actually drives cloud
         * rendering: how much of the sky this band covers, how opaque it reads, and
         * whether it is behaving stratiform (a sheet) or convective (cellular/towering) —
         * `coverage`/`density_scale` map directly onto `Render::CloudDeck::coverage_bias`/
         * `density_scale`, `convective_fraction` selects which genus a deck instantiates.
         */
        struct WeatherLevelState
        {
            float coverage = 0.0f;            /**< Fraction of sky this band covers, [0, 1]. */
            float density_scale = 0.0f;       /**< Opacity/thickness scale, [0, 2]. */
            float convective_fraction = 0.0f; /**< 0 stratiform sheet -> 1 cellular/towering, [0, 1]. */
            /**
             * @brief Deviation from a neutral (ISA-like) baseline temperature at this band, Celsius.
             *
             * W6 addition: T2 (`RegionalWeatherGrid::WeatherCell::temperature_offset_c`) has
             * tracked this internally since W4, but the bridge never surfaced it because nothing
             * downstream needed a temperature signal yet -- see `weather_world_coupling.hpp`'s
             * dew-point-spread scope-down note. `weather_flight_hazards.hpp`'s `icing_risk()` is
             * the first real consumer: it combines this offset with a standard-atmosphere lapse
             * rate at the query altitude to estimate whether a band sits near freezing. Zero
             * (the neutral baseline) is `StaticWeather`'s honest default -- a manually authored
             * sky has no thermometer to report a real deviation from.
             */
            float temperature_offset_c = 0.0f;
        };

        /**
         * @brief The layered-column meteorology at one point, ready for cloudscape compilation.
         *
         * The MSFS/X-Plane-style column representation `docs/slop/weather_and_clouds.md`
         * §5.2/§5.4 calls for: coverage/type/density per level plus surface wind and
         * precipitation. This is `IWeatherProvider`'s entire output contract — the renderer,
         * and everything downstream of it, never sees the synoptic systems or the regional
         * grid directly, only this. `IngestedWeather` (design doc §5.4, `sim/ingested_weather.hpp`,
         * W6) fills the identical struct from GRIB/METAR data, which is the whole point of the
         * seam.
         */
        struct WeatherColumn
        {
            WeatherLevelState levels[CLOUD_LEVEL_COUNT]; /**< Indexed by @ref CloudLevel. */
            float precipitation = 0.0f;   /**< Column surface precipitation rate, [0, 1]. */
            /**
             * @brief Altitude of the lowest cloud in this column, metres; 0 when there is none.
             *
             * `docs/slop/atmosphere_system.md` §1.2 lists cloud base among the things the
             * fifteen floats of this struct could not express, and names that as the ceiling on
             * everything downstream. It is here now because the regional nest actually knows it —
             * a column's base is the altitude its condensate starts at — and because the one
             * consumer that most needed it was reaching for the wrong signal without it: **fog
             * is cloud whose base is at the ground**, and nothing else in this struct can tell
             * an overcast sky at 1 200 m from a hill fogged in at zero.
             */
            float cloud_base_m = 0.0f;
            float wind_u_mps = 0.0f;      /**< Low-level eastward wind, metres/second (future @c weather_wind() seam, W5). */
            float wind_v_mps = 0.0f;      /**< Low-level northward wind, metres/second. */
        };
    } // namespace Simulation
} // namespace SushiEngine
