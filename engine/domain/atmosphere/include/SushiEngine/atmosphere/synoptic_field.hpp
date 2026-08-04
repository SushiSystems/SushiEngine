/**************************************************************************/
/* synoptic_field.hpp                                                     */
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
 * @file synoptic_field.hpp
 * @brief Where the weather *is*, at the scale of a planet, as a closed-form function.
 *
 * `docs/design/atmosphere_system.md`'s WM-SEED. The defect this closes is visible in one
 * side-by-side: our Earth from orbit is a uniformly milky sphere, and the real one is mostly
 * *clear* ocean with discrete bands and swirls laid over it. The cause was structural rather
 * than cosmetic — a manually authored sky had no horizontal extent at all, so one deck stack
 * was applied to every square metre of the planet and "somewhere it is stormy, somewhere it is
 * clear" was not a thing the data could express.
 *
 * This is the smallest honest thing that can express it: a **zonal climatology** plus a handful
 * of **seeded pressure centres**. Both halves are deliberate.
 *
 * The zonal term is not decoration. Most of what makes a photograph of Earth recognisable is a
 * function of latitude alone — a cloudy ITCZ, startlingly clear subtropics where the Hadley
 * cell subsides (this is where the deserts and the blue ocean in any orbital photograph are),
 * a cloudy midlatitude storm track, and a moderately cloudy polar cap. That structure is free:
 * it needs no simulation, no seed, and no data, because it is the same every year.
 *
 * The centres are what makes one seed differ from another and what puts a storm *here* and
 * clear air *there*. A pressure system is modelled as a signed Gaussian bump in coverage about
 * a point on the sphere: a low raises coverage, deepens convection and rains; a high suppresses
 * coverage toward zero, which is the half that matters, because a field that cannot reach zero
 * can never produce the clear ocean that is most of an orbital photograph.
 *
 * **Why a closed form rather than a texture.** Two consumers need this answer and they live on
 * opposite sides of the render seam: `Simulation::SeededWeather` samples it per column to
 * publish a weather field, and `cloud.frag`'s planet-scale globe field samples it per march
 * sample out where no baked window reaches. A texture would need a binding the cloud pass does
 * not have; a closed form needs only the centre list, which is twelve vectors. Both sides then
 * evaluate the *same* function from the *same* numbers, so they cannot disagree about where the
 * weather is — and disagreeing is exactly what would show, as a seam at the far window's rim.
 *
 * **What this is not.** It is not a simulation and does not pretend to be: nothing here evolves,
 * conserves anything, or responds to terrain. `ProceduralWeather` is where weather is *grown*.
 * This is where it is *placed*, which is what an author asking for a seed is asking for.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace SushiEngine
{
    namespace Atmosphere
    {
        /**
         * @brief Upper bound on the pressure centres a synoptic field carries.
         *
         * Twelve, and the number is a cost decision with a physical sanity check on it. The
         * cost: the render-side evaluation runs per cloud march sample, and each centre is a
         * dot product and an exponential, so this is the width of an inner loop. The check:
         * twelve systems over a whole planet is about what a real synoptic chart carries at any
         * moment once the features too small to see from orbit are dropped, so the bound is not
         * cutting off structure that would otherwise be visible.
         */
        constexpr int SYNOPTIC_MAX_CENTRES = 12;

        /**
         * @brief One pressure system, as the field's evaluation needs it.
         *
         * Geographic here, because this is the frame the simulation asks its questions in. The
         * render seam carries the same list rotated into scene space
         * (`Render::SynopticField`) so the shader needs no frame of its own.
         */
        struct SynopticCentre
        {
            double latitude_radians = 0.0;  /**< Where the system's centre sits. */
            double longitude_radians = 0.0; /**< Where the system's centre sits. */
            /**
             * @brief Gaussian falloff against the **chord**, not the angle.
             *
             * The weight is `exp(-falloff * (1 - cos θ))` rather than `exp(-(θ/θ_e)²)`. They
             * agree to second order — `1 - cos θ ≈ θ²/2` — and the chord form costs one dot
             * product where the angle form costs an inverse cosine, which matters because this
             * is evaluated per march sample. @ref falloff_for_radius converts.
             */
            double falloff = 0.0;
            /** @brief Signed coverage anomaly at the centre: positive is a low, negative a high. */
            double amplitude = 0.0;
            /** @brief Convective fraction this system brings, [0, 1]; only lows carry it. */
            double convective = 0.0;
            /** @brief Surface precipitation this system brings, [0, 1]; only lows carry it. */
            double precipitation = 0.0;
        };

        /**
         * @brief The chord falloff that puts a system's e-folding edge at @p radius_m.
         *
         * @param radius_m        e-folding radius along the surface, metres.
         * @param planet_radius_m The body's mean radius, metres.
         */
        inline double synoptic_falloff_for_radius(double radius_m, double planet_radius_m) noexcept
        {
            const double angle = std::max(radius_m, 1.0) / std::max(planet_radius_m, 1.0);
            return 2.0 / std::max(angle * angle, 1e-9);
        }

        /**
         * @brief Where the Intertropical Convergence Zone sits at a point in the year.
         *
         * The ITCZ follows the sun's declination with a lag of about a month and a reduced
         * amplitude — it does not reach the tropics, because the ocean's heat capacity holds it
         * back. Roughly ±6° about the equator, north in boreal summer. Small, and worth having
         * anyway: it is why a January photograph of the Pacific and a July one do not look the
         * same, and it costs one sine.
         *
         * @param year_fraction 0 at the start of the year through 1 at its end.
         * @return The ITCZ's mean latitude, radians.
         */
        inline double synoptic_itcz_latitude(double year_fraction) noexcept
        {
            constexpr double TWO_PI = 6.28318530717958647692;
            constexpr double AMPLITUDE_RADIANS = 6.0 * 0.01745329251994329577;
            // The peak lands in late July rather than at the solstice: the phase offset is the
            // month of lag, expressed as a fraction of the year.
            constexpr double PHASE = 0.22;
            return AMPLITUDE_RADIANS * std::sin(TWO_PI * (year_fraction - PHASE));
        }

        /**
         * @brief The zonal-mean cloud fraction at a latitude — the free half of the field.
         *
         * Three Gaussians on a base: a cloudy ITCZ, a clear subtropical belt where the Hadley
         * cell subsides, and a cloudy midlatitude storm track. It lands near 0.64 at the ITCZ,
         * **0.06 in the subtropics**, 0.66 in the storm track, 0.31 at the pole, and 0.30 across
         * ordinary midlatitudes — a fair-weather sky with real gaps in it.
         *
         * The subtropical minimum is the one that earns its place. It is why an orbital
         * photograph has large, genuinely clear ocean in it, and a field without it reads as
         * overcast everywhere no matter how the rest is tuned.
         *
         * **This is the optically thick fraction, not the total cloud fraction**, and the
         * distinction is what the first version of these numbers got wrong. Satellite
         * climatologies report annual-mean *total* cloud near 0.50 globally with a subtropical
         * floor around 0.32, and those were the numbers this function originally carried. They
         * are correct for what they measure and wrong for what this feeds: total cloud fraction
         * counts sub-visual cirrus and broken fields that read as clear sky from orbit, whereas
         * every consumer downstream treats this value as the fraction of sky the carve fills
         * with *opaque, lit* cloud. Publishing 0.32 at the clearest place on the planet
         * therefore drew a third of that sky solid, and the whole globe came out milky — the
         * exact failure the subtropical minimum was added to prevent, reintroduced through the
         * units. The optically thick fraction really does approach ~0.05 under a subsiding
         * Hadley branch, and that is the quantity this now states.
         *
         * @param latitude_radians Signed geographic latitude.
         * @param itcz_latitude    Where the convergence zone currently sits, radians.
         */
        inline double synoptic_zonal_coverage(double latitude_radians,
                                              double itcz_latitude) noexcept
        {
            constexpr double DEGREE = 0.01745329251994329577;
            const double absolute = std::fabs(latitude_radians);

            const double itcz = (latitude_radians - itcz_latitude) / (8.0 * DEGREE);
            const double subtropics = (absolute - 25.0 * DEGREE) / (12.0 * DEGREE);
            const double storm_track = (absolute - 58.0 * DEGREE) / (16.0 * DEGREE);

            const double coverage = 0.30 + 0.34 * std::exp(-itcz * itcz) -
                                    0.24 * std::exp(-subtropics * subtropics) +
                                    0.36 * std::exp(-storm_track * storm_track);
            return std::clamp(coverage, 0.0, 1.0);
        }

        /**
         * @brief The zonal-mean convective fraction at a latitude.
         *
         * The tropics convect and the midlatitudes do not — a tropical sky is towering cumulus
         * and cumulonimbus, a midlatitude one is layered frontal cloud — and that difference is
         * a function of latitude before it is a function of anything else. It is also the term
         * that decides which *genus* the classifier resolves per column, so it is what makes a
         * seeded tropical scene look tropical.
         */
        inline double synoptic_zonal_convective(double latitude_radians,
                                                double itcz_latitude) noexcept
        {
            constexpr double DEGREE = 0.01745329251994329577;
            const double tropics = (latitude_radians - itcz_latitude) / (15.0 * DEGREE);
            return 0.15 + 0.75 * std::exp(-tropics * tropics);
        }

        /**
         * @brief A seeded placement of pressure systems over a planet, and how to read it.
         *
         * Deterministic in the seed: the same seed reproduces the same weather everywhere, on
         * any machine, which is the whole point of an author choosing one. The generator is
         * SplitMix64 rather than `std::mt19937` for exactly that reason — a standard-library
         * distribution's mapping from bits to values is implementation-defined, so two vendors'
         * standard libraries would disagree about what seed 7 means.
         *
         * Placement is weighted toward where systems actually are, rather than uniform over the
         * sphere: most land in the midlatitude storm tracks, some in the tropics, and the rest
         * anywhere. Uniform placement looks obviously wrong from orbit — real cyclones queue up
         * along the jet in both hemispheres and the subtropics stay empty.
         */
        class SynopticField
        {
            public:
                /** @brief An empty field: the zonal climatology and nothing placed on it. */
                SynopticField() = default;

                /**
                 * @brief Places systems for a seed.
                 *
                 * @param seed            Any 64-bit value; identical seeds reproduce identical skies.
                 * @param year_fraction   Where the year is, for the ITCZ's seasonal migration.
                 * @param planet_radius_m The body's mean radius, metres — the scale system radii
                 *                        are stated against.
                 */
                SynopticField(std::uint64_t seed, double year_fraction, double planet_radius_m)
                {
                    reseed(seed, year_fraction, planet_radius_m);
                }

                /** @brief Re-places every system. @see SynopticField(std::uint64_t, double, double) */
                void reseed(std::uint64_t seed, double year_fraction, double planet_radius_m)
                {
                    constexpr double PI = 3.14159265358979323846;
                    constexpr double TWO_PI = 6.28318530717958647692;
                    constexpr double DEGREE = 0.01745329251994329577;

                    itcz_latitude_ = synoptic_itcz_latitude(year_fraction);
                    // The seed is mixed once before use so that adjacent seeds — 1, 2, 3, which
                    // is what an author actually types — produce unrelated skies rather than
                    // skies that share their first system.
                    std::uint64_t state = mix(seed + GOLDEN_GAMMA);
                    count_ = SYNOPTIC_MAX_CENTRES;

                    for (int i = 0; i < count_; ++i)
                    {
                        SynopticCentre& centre = centres_[i];
                        centre.longitude_radians = next_uniform(state) * TWO_PI - PI;

                        // Where it lands. Two thirds go to a storm track, a fifth to the
                        // tropics, and the rest anywhere -- see the class docs for why this is
                        // not uniform over the sphere.
                        const double placement = next_uniform(state);
                        const double hemisphere = next_uniform(state) < 0.5 ? -1.0 : 1.0;
                        if (placement < 0.62)
                            centre.latitude_radians =
                                hemisphere * (38.0 + next_uniform(state) * 26.0) * DEGREE;
                        else if (placement < 0.82)
                            centre.latitude_radians =
                                itcz_latitude_ + (next_uniform(state) - 0.5) * 24.0 * DEGREE;
                        else
                            centre.latitude_radians = (next_uniform(state) - 0.5) * PI * 0.9;

                        // Lows outnumber highs slightly, and are the smaller and stronger of the
                        // two: a mature anticyclone is a broad shallow dome of subsidence, while
                        // a cyclone is compact and deep. The asymmetry is real and it is what
                        // makes the clear areas *large* and the cloudy ones *organised*.
                        const bool low = next_uniform(state) < 0.55;
                        const double radius_m =
                            (low ? 700000.0 + next_uniform(state) * 700000.0
                                 : 1100000.0 + next_uniform(state) * 900000.0);
                        centre.falloff = synoptic_falloff_for_radius(radius_m, planet_radius_m);

                        if (low)
                        {
                            centre.amplitude = 0.26 + next_uniform(state) * 0.30;
                            // A tropical system is deep convection; a midlatitude one is a
                            // frontal sheet with embedded showers. The latitude decides, because
                            // it is what decides in the atmosphere.
                            const double tropical = std::exp(
                                -std::pow((centre.latitude_radians - itcz_latitude_) /
                                              (18.0 * DEGREE), 2.0));
                            centre.convective = 0.20 + 0.70 * tropical;
                            centre.precipitation = 0.25 + 0.55 * centre.amplitude;
                        }
                        else
                        {
                            // Strong enough to take the zonal climatology to zero on its own:
                            // 0.76 is the storm track's own value, so a high sitting on one
                            // still clears it. A high that cannot clear the sky is a high
                            // nobody will ever see.
                            centre.amplitude = -(0.45 + next_uniform(state) * 0.45);
                            centre.convective = 0.0;
                            centre.precipitation = 0.0;
                        }
                    }
                }

                /** @brief The placed systems. */
                const SynopticCentre* centres() const noexcept { return centres_; }

                /** @brief How many of @ref centres are populated. */
                int count() const noexcept { return count_; }

                /** @brief Where the convergence zone sits, radians — the seasonal term. */
                double itcz_latitude() const noexcept { return itcz_latitude_; }

                /**
                 * @brief Total cloud fraction at a place, [0, 1].
                 *
                 * The zonal climatology with every system's signed bump added. Clamped rather
                 * than normalised: two overlapping lows really are cloudier than one, up to the
                 * point where the sky is full, and a high sitting on a low really does cancel
                 * it.
                 */
                double coverage_at(double latitude_radians, double longitude_radians) const
                {
                    double coverage = synoptic_zonal_coverage(latitude_radians, itcz_latitude_);
                    const double dx = std::cos(latitude_radians) * std::cos(longitude_radians);
                    const double dy = std::cos(latitude_radians) * std::sin(longitude_radians);
                    const double dz = std::sin(latitude_radians);
                    for (int i = 0; i < count_; ++i)
                        coverage += centres_[i].amplitude * weight(centres_[i], dx, dy, dz);
                    return std::clamp(coverage, 0.0, 1.0);
                }

                /**
                 * @brief Convective fraction at a place, [0, 1].
                 *
                 * The zonal term, pulled toward whichever low is overhead. Highs contribute
                 * nothing here rather than a negative: an anticyclone does not make the sky
                 * *stratiform*, it makes it empty, and the coverage term already says so.
                 */
                double convective_at(double latitude_radians, double longitude_radians) const
                {
                    const double zonal =
                        synoptic_zonal_convective(latitude_radians, itcz_latitude_);
                    const double dx = std::cos(latitude_radians) * std::cos(longitude_radians);
                    const double dy = std::cos(latitude_radians) * std::sin(longitude_radians);
                    const double dz = std::sin(latitude_radians);

                    double weighted = 0.0;
                    double total = 0.0;
                    for (int i = 0; i < count_; ++i)
                    {
                        if (centres_[i].amplitude <= 0.0)
                            continue;
                        const double w = centres_[i].amplitude * weight(centres_[i], dx, dy, dz);
                        weighted += w * centres_[i].convective;
                        total += w;
                    }
                    if (total <= 1e-6)
                        return std::clamp(zonal, 0.0, 1.0);
                    // The systems' own character, blended toward the zonal background by how
                    // strongly any of them is actually present here.
                    const double presence = std::clamp(total, 0.0, 1.0);
                    return std::clamp(zonal + presence * (weighted / total - zonal), 0.0, 1.0);
                }

                /** @brief Surface precipitation intensity at a place, [0, 1]. */
                double precipitation_at(double latitude_radians, double longitude_radians) const
                {
                    const double dx = std::cos(latitude_radians) * std::cos(longitude_radians);
                    const double dy = std::cos(latitude_radians) * std::sin(longitude_radians);
                    const double dz = std::sin(latitude_radians);
                    double rate = 0.0;
                    for (int i = 0; i < count_; ++i)
                    {
                        if (centres_[i].amplitude <= 0.0)
                            continue;
                        rate += centres_[i].precipitation * weight(centres_[i], dx, dy, dz);
                    }
                    return std::clamp(rate, 0.0, 1.0);
                }

            private:
                // SplitMix64's increment. Also used to mix the incoming seed, so that the
                // sequence for seed n and seed n+1 share no state.
                static constexpr std::uint64_t GOLDEN_GAMMA = 0x9E3779B97F4A7C15ull;

                static std::uint64_t mix(std::uint64_t z) noexcept
                {
                    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
                    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
                    return z ^ (z >> 31);
                }

                /** @brief The next value in [0, 1), advancing @p state. */
                static double next_uniform(std::uint64_t& state) noexcept
                {
                    state += GOLDEN_GAMMA;
                    // 53 bits is exactly what a double's mantissa holds, so this is uniform on
                    // the representable values rather than uniform-then-rounded.
                    return double(mix(state) >> 11) * (1.0 / 9007199254740992.0);
                }

                /** @brief A system's Gaussian weight at a unit direction, [0, 1]. */
                static double weight(const SynopticCentre& centre, double dx, double dy,
                                     double dz) noexcept
                {
                    const double cx = std::cos(centre.latitude_radians) *
                                      std::cos(centre.longitude_radians);
                    const double cy = std::cos(centre.latitude_radians) *
                                      std::sin(centre.longitude_radians);
                    const double cz = std::sin(centre.latitude_radians);
                    const double chord = 1.0 - (dx * cx + dy * cy + dz * cz);
                    return std::exp(-centre.falloff * std::max(chord, 0.0));
                }

                SynopticCentre centres_[SYNOPTIC_MAX_CENTRES]{};
                int count_ = 0;
                double itcz_latitude_ = 0.0;
        };
    } // namespace Atmosphere
} // namespace SushiEngine
