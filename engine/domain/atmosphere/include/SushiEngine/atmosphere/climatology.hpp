/**************************************************************************/
/* climatology.hpp                                                        */
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
 * @file climatology.hpp
 * @brief T0 — the mean state T1 is a departure from (`docs/slop/atmosphere_system.md` §4).
 *
 * **T0 is data, not a model.** Nothing here is simulated: it answers "what is normally true
 * here, at this time of year", and the global core (§5) relaxes toward it while the eddies it
 * grows are, by definition, everything this cannot say.
 *
 * The type is a value, and it does no I/O. `sushi_atmosphere` links nothing (see its
 * `CMakeLists.txt`) and that is not a stylistic preference — the same object has to serve the
 * editor, a headless probe and a unit test, and two of those three have no business opening a
 * file. So a `Climatology` is either constructed analytically or handed bytes somebody else
 * read.
 *
 * **The analytic construction is the real one, not a fallback that fails.** §4 says outright
 * that for a non-Earth body T0 degrades to analytic latitude bands driven by the body's own
 * parameters, and a scene on such a body is not a scene with broken weather. It is also what
 * every core built before T0 existed already used, so the analytic path reproduces C2's and
 * C3's measured behaviour exactly rather than approximately.
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/atmosphere/geographic_position.hpp>

namespace SushiEngine
{
    namespace Atmosphere
    {
        /**
         * @brief The latitude bands T0 degrades to when there is no baked asset.
         *
         * These seven numbers were fields of `QuasiGeostrophicParameters` until T0 existed, and
         * they moved here because they are not parameters *of the dynamics* — they are the
         * cheapest possible statement of a mean state, standing exactly where a real
         * climatology stands. Keeping them in the physics struct made "what the atmosphere is
         * relaxing toward" indistinguishable from "how the atmosphere relaxes", and only one of
         * those two is replaced by an ERA5 bake.
         *
         * The defaults are Earth's, so a core built without a word about climatology behaves
         * the way every core built before this file existed did.
         */
        struct AnalyticClimatology
        {
            /** @brief Peak upper-layer zonal wind of the climatological jet, m/s. */
            double upper_jet_speed_mps = 30.0;

            /**
             * @brief Peak lower-layer zonal wind of the climatological jet, m/s.
             *
             * The *difference* from @ref upper_jet_speed_mps is the vertical shear, and the
             * shear is what makes the mean state unstable. Phillips' criterion puts the
             * critical shear near `beta * L_d^2`, about 8 m/s at these settings, so the
             * default 20 m/s shear is comfortably supercritical — deliberately, because a
             * marginally unstable core takes simulated weeks to produce a storm.
             */
            double lower_jet_speed_mps = 10.0;

            /** @brief Latitude the climatological jet is centred on, radians (both hemispheres). */
            double jet_latitude_radians = 0.7853981633974483; // 45 degrees

            /** @brief Half-width of the jet's Gaussian latitude envelope, radians. */
            double jet_width_radians = 0.2617993877991494; // 15 degrees

            /** @brief Column water an equatorial column holds at saturation, kg/m^2. */
            double equatorial_saturation_kg_per_m2 = 60.0;

            /** @brief Equator-to-pole surface temperature contrast, K — shapes the saturation profile. */
            double equator_to_pole_kelvin = 45.0;

            /**
             * @brief Temperature change that alters saturation by a factor of e, K.
             *
             * Clausius-Clapeyron, as one number: about 7 % of the saturation vapour pressure
             * per kelvin makes this roughly 15 K. It is what puts the water in the tropics
             * without a radiation scheme having to be run to find out.
             */
            double saturation_lapse_kelvin = 15.0;
        };

        /**
         * @brief Shape of a baked climatology's zonal profiles.
         *
         * Latitude bands and calendar months, and nothing else: the three fields T1 relaxes
         * toward are all zonal means, so they are one-dimensional in space no matter how
         * finely the source product resolved them. That is why the whole asset is small enough
         * to be uninteresting — a 1° × 12-month profile is 2 160 numbers, against the 1.5 MB
         * the *running* core's own state costs to save.
         */
        struct ClimatologyProfileGrid
        {
            /**
             * @brief Latitude bands, cell-centred from the south pole to the north.
             *
             * 180 rather than 181: `sample_profile` places band `i` at
             * `(i + 0.5) / bands * 180 - 90`, so 180 bands are the degree cells
             * -89.5, -88.5, ... 89.5 and a band is one degree of latitude. 181 would be
             * one *node* per integer degree, which is not what that formula samples.
             */
            int latitude_bands = 180;
            /** @brief Calendar months. Twelve, in every calendar this will ever bake. */
            int months = 12;
        };

        /**
         * @brief The mean state, as three zonal profiles plus what the surface is made of.
         *
         * **Three of the fields are read by T1 and two are not, and that is stated rather than
         * hidden.** `upper_zonal_wind_mps`, `lower_zonal_wind_mps` and
         * `saturation_kg_per_m2` are exactly what `QuasiGeostrophicCore` builds its mean state
         * from — replacing, one for one, the analytic jet and the single Clausius-Clapeyron
         * e-folding it used before. `land_fraction` and `sea_surface_temperature_kelvin` are
         * for T2's surface properties (§4: surface fluxes, albedo, roughness), which is a GPU
         * seam and a separate piece of work; they are baked here because they come from the
         * same download and the same asset, and the task that reads them is named in the
         * phase list rather than left implied.
         *
         * **Time is a year fraction, not a month index.** A scene played across the end of
         * January must not step to a different jet between two frames, so every query
         * interpolates between the two bracketing months and wraps December into January.
         */
        class Climatology
        {
            public:
                /** @brief The analytic latitude bands, with Earth's defaults. */
                Climatology() = default;

                /** @brief The analytic latitude bands, for a body that is not Earth. */
                explicit Climatology(const AnalyticClimatology& bands) : bands_(bands) {}

                /**
                 * @brief Adopts a baked asset produced by the bake tool.
                 *
                 * Rejects a blob whose magic, version or profile grid it does not recognise,
                 * rather than reading it as far as it can. A half-understood climatology is a
                 * mean state nobody chose, and the weather that grows on it would be wrong in a
                 * way that looks like physics.
                 *
                 * @param blob Bytes of the baked asset.
                 * @return Whether it was accepted. On false this object is left analytic, which
                 *         is a working mean state and not an error state.
                 */
                bool adopt(const std::vector<std::uint8_t>& blob);

                /**
                 * @brief Whether the answers come from baked data or from the analytic bands.
                 *
                 * Worth exposing because the editor should be able to say which mean state the
                 * weather is departing from — a scene running on the analytic bands when
                 * somebody meant it to run on ERA5 is otherwise invisible until the jet is in
                 * the wrong place.
                 */
                bool baked() const noexcept { return baked_; }

                /** @brief The analytic bands, whether or not a bake is overlaid on them. */
                const AnalyticClimatology& analytic_bands() const noexcept { return bands_; }

                /**
                 * @brief Upper-layer climatological zonal wind, m/s. Positive is eastward.
                 * @param latitude_radians Query latitude.
                 * @param year_fraction    Position in the year, [0, 1); wraps.
                 */
                double upper_zonal_wind_mps(double latitude_radians, double year_fraction) const;

                /**
                 * @brief Lower-layer climatological zonal wind, m/s.
                 *
                 * The difference from @ref upper_zonal_wind_mps is the vertical shear, which is
                 * the single number that decides whether the mean state makes storms at all —
                 * so a bake that gets this wrong does not produce slightly wrong weather, it
                 * produces none or far too much.
                 *
                 * @param latitude_radians Query latitude.
                 * @param year_fraction    Position in the year, [0, 1); wraps.
                 */
                double lower_zonal_wind_mps(double latitude_radians, double year_fraction) const;

                /**
                 * @brief Column water a saturated column at this latitude holds, kg/m^2.
                 *
                 * The moisture the core relaxes toward, and the reason its rain falls in the
                 * tropics without a radiation scheme being run to find out where the tropics
                 * are.
                 *
                 * @param latitude_radians Query latitude.
                 * @param year_fraction    Position in the year, [0, 1); wraps.
                 */
                double saturation_kg_per_m2(double latitude_radians, double year_fraction) const;

                /**
                 * @brief Fraction of a cell that is land, [0, 1].
                 *
                 * Not read by T1 — see the class docs. Answers 0 on the analytic path, because
                 * an analytic latitude band has no continents and saying so is truthful, where
                 * inventing a plausible coastline would not be.
                 */
                double land_fraction(const GeographicPosition& position) const;

                /**
                 * @brief Climatological sea surface temperature, K.
                 *
                 * Not read by T1 — see the class docs. Over land, and on the analytic path,
                 * this answers the zonal surface temperature the analytic bands imply, so a
                 * consumer never has to branch on whether a bake was loaded.
                 */
                double sea_surface_temperature_kelvin(const GeographicPosition& position,
                                                      double year_fraction) const;

                /**
                 * @brief What this climatology was baked from, for attribution and for debugging.
                 *
                 * §4 calls the licensing step part of the work rather than an afterthought, so
                 * the provenance travels inside the asset and can be shown in the editor
                 * beside the weather it produced. Empty on the analytic path.
                 */
                const std::string& provenance() const noexcept { return provenance_; }

            private:
                /** @brief Interpolates a month-major profile at a latitude and a year fraction. */
                double sample_profile(const std::vector<float>& profile, double latitude_radians,
                                      double year_fraction) const;

                AnalyticClimatology bands_;
                bool baked_ = false;
                ClimatologyProfileGrid profile_grid_;

                // Month-major: all latitudes of January, then all of February. Queries walk
                // latitude within one month far more often than they walk months, so the two
                // rows an interpolation needs are each contiguous.
                std::vector<float> upper_wind_;
                std::vector<float> lower_wind_;
                std::vector<float> saturation_;

                // The two surface fields, on their own lat/lon grid rather than the zonal one:
                // a land/sea mask that had been zonally averaged would say the Pacific is
                // three-quarters water everywhere along its latitude, which is not a fact about
                // anywhere.
                //
                // The grid is a header field rather than a constant, and the dimensions are
                // checked on load, so the resolution is the bake tool's decision and not this
                // reader's. It bakes 1 degree today -- OISST's native grid, and the scale at
                // which "what is normally under this column" is a meaningful question. It is
                // deliberately not the scale at which a shoreline is placed: a nest cell is
                // 2 km (§6), and the scene's terrain already knows where the coast is.
                int surface_longitude_cells_ = 0;
                int surface_latitude_cells_ = 0;
                std::vector<float> land_fraction_;
                std::vector<float> sea_surface_temperature_;

                std::string provenance_;
        };
    } // namespace Atmosphere
} // namespace SushiEngine
