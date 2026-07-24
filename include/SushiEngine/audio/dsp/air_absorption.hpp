/**************************************************************************/
/* air_absorption.hpp                                                    */
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

#ifndef SUSHIENGINE_AUDIO_DSP_AIR_ABSORPTION_HPP
#define SUSHIENGINE_AUDIO_DSP_AIR_ABSORPTION_HPP

/**
 * @file air_absorption.hpp
 * @brief Atmosphere: speed of sound and ISO 9613-1 air absorption.
 *
 * Distant sounds are not just quieter, they are *duller* — air absorbs high
 * frequencies far faster than low, and by more the farther the sound travels. The
 * ISO 9613-1 model (@ref air_absorption_db_per_meter) gives that frequency-dependent
 * attenuation for the current weather; the renderer does not run a full multi-band
 * filter per sample but instead reduces it to a one-pole low-pass whose corner falls
 * with distance (@ref air_absorption_cutoff), which is cheap and perceptually right
 * (see `docs/design/audio_system.md` §5). @ref speed_of_sound feeds both the
 * absorption and the propagation delay, so delay, Doppler, and dullness stay
 * consistent under the same temperature.
 */

#include <cmath>

namespace SushiEngine
{
    namespace Audio
    {
        namespace Dsp
        {
            /** @brief The atmospheric state absorption and the speed of sound depend on. */
            struct Atmosphere
            {
                float temperature_c = 20.0f;    /**< Air temperature in degrees Celsius. */
                float humidity_percent = 50.0f; /**< Relative humidity in percent [0, 100]. */
                float pressure_kpa = 101.325f;  /**< Ambient pressure in kilopascals. */
            };

            /**
             * @brief The speed of sound for a temperature.
             *
             * The standard linear approximation `c = 331.3 + 0.606·T` (°C), accurate near
             * room temperature and the one the propagation delay uses.
             *
             * @param atmosphere The atmospheric state (only temperature is used).
             * @return The speed of sound in metres per second.
             */
            inline float speed_of_sound(const Atmosphere& atmosphere) noexcept
            {
                return 331.3f + 0.606f * atmosphere.temperature_c;
            }

            /**
             * @brief ISO 9613-1 atmospheric absorption at one frequency.
             *
             * The full standard formula: molar water-vapour concentration from relative
             * humidity and saturation pressure, the oxygen and nitrogen relaxation
             * frequencies, and the resulting classical + relaxational absorption. Computed
             * in `double`.
             *
             * @param frequency_hz The frequency in Hz.
             * @param atmosphere   The atmospheric state.
             * @return The absorption in decibels per metre (≥ 0, increasing with frequency).
             */
            inline double air_absorption_db_per_meter(double frequency_hz,
                                                      const Atmosphere& atmosphere) noexcept
            {
                const double temperature = 273.15 + atmosphere.temperature_c; // Kelvin
                const double t0 = 293.15;   // 20 C reference
                const double t01 = 273.16;  // triple point
                const double pr = 101.325;  // reference pressure, kPa
                const double pa = atmosphere.pressure_kpa;

                // Molar concentration of water vapour (%).
                const double psat_ratio =
                    std::pow(10.0, -6.8346 * std::pow(t01 / temperature, 1.261) + 4.6151);
                const double h = atmosphere.humidity_percent * psat_ratio * (pr / pa);

                // Oxygen and nitrogen relaxation frequencies.
                const double fr_o =
                    (pa / pr) * (24.0 + 4.04e4 * h * (0.02 + h) / (0.391 + h));
                const double fr_n =
                    (pa / pr) * std::pow(temperature / t0, -0.5) *
                    (9.0 + 280.0 * h * std::exp(-4.170 * (std::pow(temperature / t0, -1.0 / 3.0) - 1.0)));

                const double f2 = frequency_hz * frequency_hz;
                const double relax_o = 0.01275 * std::exp(-2239.1 / temperature) / (fr_o + f2 / fr_o);
                const double relax_n = 0.1068 * std::exp(-3352.0 / temperature) / (fr_n + f2 / fr_n);

                const double alpha =
                    8.686 * f2 *
                    (1.84e-11 * (pr / pa) * std::sqrt(temperature / t0) +
                     std::pow(temperature / t0, -2.5) * (relax_o + relax_n));

                return alpha > 0.0 ? alpha : 0.0;
            }

            /**
             * @brief The low-pass corner that models air absorption over a distance.
             *
             * Finds, by bisection, the frequency at which the accumulated absorption over
             * @p distance_meters reaches @p threshold_db — the point where the air has
             * meaningfully rolled the highs off. Near the listener that frequency is above
             * the audible band (the filter is effectively open); far away it falls into the
             * midrange (a muffled, distant sound). Monotonic in distance.
             *
             * @param distance_meters The propagation distance in metres.
             * @param atmosphere      The atmospheric state.
             * @param threshold_db    The attenuation defining the corner (default 3 dB).
             * @return The one-pole low-pass cutoff in Hz, within [500, 20000].
             */
            inline float air_absorption_cutoff(float distance_meters, const Atmosphere& atmosphere,
                                               double threshold_db = 3.0) noexcept
            {
                const double lo_hz = 500.0;
                const double hi_hz = 20000.0;
                if (distance_meters <= 0.0f)
                    return static_cast<float>(hi_hz);

                const double d = static_cast<double>(distance_meters);
                if (air_absorption_db_per_meter(hi_hz, atmosphere) * d <= threshold_db)
                    return static_cast<float>(hi_hz); // even 20 kHz survives: filter open
                if (air_absorption_db_per_meter(lo_hz, atmosphere) * d >= threshold_db)
                    return static_cast<float>(lo_hz); // even 500 Hz is rolled off: very distant

                double lo = lo_hz;
                double hi = hi_hz;
                for (int iteration = 0; iteration < 24; ++iteration)
                {
                    const double mid = 0.5 * (lo + hi);
                    if (air_absorption_db_per_meter(mid, atmosphere) * d < threshold_db)
                        lo = mid;
                    else
                        hi = mid;
                }
                return static_cast<float>(0.5 * (lo + hi));
            }
        } // namespace Dsp
    } // namespace Audio
} // namespace SushiEngine

#endif
