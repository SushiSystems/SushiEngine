/**************************************************************************/
/* main.cpp                                                               */
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

// Headless driver for the global core: runs simulated weeks in seconds of wall clock and prints
// what the flow is doing.
//
// **Why this exists.** The acceptance for T1 is not a screenshot — it is that baroclinic
// instability *emerges*, at a rate the textbook predicts, and then equilibrates instead of
// running away. That is a measurement over simulated weeks, and the editor is the wrong
// instrument for it: a scene view shows one moment of one hemisphere. This runs the core with
// nothing else brought up, samples the whole-globe diagnostics on a schedule, and fits a growth
// rate to the eddy kinetic energy while it is still exponential.
//
// It is a measuring instrument, not a test: it asserts nothing and returns 0 whenever the core
// ran. `test_quasigeostrophic_core.cpp` is where the transform, the inversion, the Jacobian's
// conservation and the Rossby wave's phase speed are pinned against independent references.

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>
// Header-only, and it depends on nothing but the atmosphere library it is reading an asset for.
// Reused rather than reimplemented here so there is exactly one statement in the tree of what a
// missing climatology means.
#include <SushiEngine/simulation/climatology_asset.hpp>

namespace
{
    using SushiEngine::Atmosphere::AnalyticClimatology;
    using SushiEngine::Atmosphere::Climatology;
    using SushiEngine::Atmosphere::QuasiGeostrophicCore;
    using SushiEngine::Atmosphere::QuasiGeostrophicDiagnostics;
    using SushiEngine::Atmosphere::QuasiGeostrophicGridSize;
    using SushiEngine::Atmosphere::QuasiGeostrophicParameters;

    constexpr double SECONDS_PER_DAY = 86400.0;

    /**
     * @brief Consecutive rising samples after which a fall is read as the peak, not as noise.
     *
     * Below this the window restarts instead of closing, because the early samples belong to the
     * seed shaking down rather than to any growing mode. Four is short enough to leave most of a
     * real growth phase inside the fit and long enough that the shake-down cannot fake it.
     */
    constexpr std::size_t MIN_GROWTH_SAMPLES = 4;

    /** @brief Everything the run is parameterized by; all of it settable from the command line. */
    struct Options
    {
        double days = 60.0;          /**< Simulated days to run. */
        double sample_hours = 12.0;  /**< Simulated hours between samples. */
        std::uint64_t seed = 1;      /**< Determines the initial perturbation's phases. */
        int longitude_cells = 512;   /**< Power of two. */
        int latitude_cells = 256;
        double upper_jet_mps = 30.0;
        double lower_jet_mps = 10.0;
        double perturbation_mps = 1.0;
        std::string series_path;     /**< CSV of one line per sample; empty = none. */

        /**
         * @brief Baked climatology to run on; empty runs the analytic bands.
         *
         * The two are not interchangeable and the run says which it got: the baked mean state
         * is not smooth in latitude and its shear is nearly twice the analytic default's, so a
         * growth rate measured on one is not a growth rate on the other.
         */
        std::string climatology_path;

        /**
         * @brief Grid-scale damping, hours; <= 0 keeps the core's own default.
         *
         * The parameter §11 found is calibrated against the two-cell wave and has to move with
         * the grid. It also has to be re-examined against a mean state this much more unstable,
         * which is what this option is for.
         */
        double damping_hours = 0.0;
    };

    bool parse(int argc, char** argv, Options& options)
    {
        for (int i = 1; i < argc; ++i)
        {
            const char* argument = argv[i];
            const bool has_value = i + 1 < argc;
            const auto value = [&]() { return argv[++i]; };

            if (std::strcmp(argument, "--days") == 0 && has_value)
                options.days = std::atof(value());
            else if (std::strcmp(argument, "--sample-hours") == 0 && has_value)
                options.sample_hours = std::atof(value());
            else if (std::strcmp(argument, "--seed") == 0 && has_value)
                options.seed = std::strtoull(value(), nullptr, 10);
            else if (std::strcmp(argument, "--longitudes") == 0 && has_value)
                options.longitude_cells = std::atoi(value());
            else if (std::strcmp(argument, "--latitudes") == 0 && has_value)
                options.latitude_cells = std::atoi(value());
            else if (std::strcmp(argument, "--upper-jet") == 0 && has_value)
                options.upper_jet_mps = std::atof(value());
            else if (std::strcmp(argument, "--lower-jet") == 0 && has_value)
                options.lower_jet_mps = std::atof(value());
            else if (std::strcmp(argument, "--perturbation") == 0 && has_value)
                options.perturbation_mps = std::atof(value());
            else if (std::strcmp(argument, "--series") == 0 && has_value)
                options.series_path = value();
            else if (std::strcmp(argument, "--climatology") == 0 && has_value)
                options.climatology_path = value();
            else if (std::strcmp(argument, "--damping-hours") == 0 && has_value)
                options.damping_hours = std::atof(value());
            else
            {
                std::printf(
                    "usage: atmosphere_global_probe [--days N] [--sample-hours N] [--seed N]\n"
                    "                               [--longitudes N] [--latitudes N]\n"
                    "                               [--upper-jet M] [--lower-jet M]\n"
                    "                               [--perturbation M] [--series PATH]\n"
                    "                               [--climatology PATH] [--damping-hours H]\n"
                    "\n"
                    "  --upper-jet/--lower-jet describe the *analytic* mean state and are\n"
                    "  ignored once --climatology supplies a real one.\n");
                return false;
            }
        }
        return true;
    }

    /**
     * @brief Least-squares slope of log(energy) against time, over the samples given.
     *
     * The growth rate of a linear instability, which is the one number that says whether the
     * core is unstable for the reason it is supposed to be. Fitted in the log rather than read
     * off two endpoints so a single noisy sample cannot set it.
     */
    double growth_rate_per_day(const std::vector<double>& day, const std::vector<double>& energy)
    {
        if (day.size() < 2)
            return 0.0;

        double mean_day = 0.0;
        double mean_log = 0.0;
        for (std::size_t k = 0; k < day.size(); ++k)
        {
            mean_day += day[k];
            mean_log += std::log(std::max(energy[k], 1e-12));
        }
        mean_day /= double(day.size());
        mean_log /= double(day.size());

        double covariance = 0.0;
        double variance = 0.0;
        for (std::size_t k = 0; k < day.size(); ++k)
        {
            const double dx = day[k] - mean_day;
            covariance += dx * (std::log(std::max(energy[k], 1e-12)) - mean_log);
            variance += dx * dx;
        }
        return variance > 0.0 ? covariance / variance : 0.0;
    }
} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse(argc, argv, options))
        return 0;

    QuasiGeostrophicGridSize size;
    size.longitude_cells = options.longitude_cells;
    size.latitude_cells = options.latitude_cells;

    QuasiGeostrophicParameters parameters;
    parameters.seed_perturbation_mps = options.perturbation_mps;
    if (options.damping_hours > 0.0)
        parameters.grid_scale_damping_seconds = options.damping_hours * 3600.0;

    // The jet is the *mean state*, so it is set on T0 rather than on the physics (§4). The
    // analytic bands are the sweepable case -- the probe's whole job is to vary the shear and
    // watch the growth rate, and a baked climatology is a fixed shear it cannot sweep -- so
    // they stay the default and a real climatology is asked for by name.
    AnalyticClimatology bands;
    bands.upper_jet_speed_mps = options.upper_jet_mps;
    bands.lower_jet_speed_mps = options.lower_jet_mps;

    Climatology climatology(bands);
    if (!options.climatology_path.empty())
    {
        climatology = SushiEngine::Simulation::load_climatology(options.climatology_path);
        if (!climatology.baked())
        {
            // Refused rather than run: a sweep that silently fell back to analytic bands would
            // report a growth rate for a mean state nobody asked for, and it would look fine.
            std::printf("could not read a climatology from '%s'\n",
                        options.climatology_path.c_str());
            return 1;
        }
    }

    QuasiGeostrophicCore core(size, parameters, climatology);
    if (!core.valid())
    {
        std::printf("global core rejected the grid: %d x %d (longitudes must be a power of two,\n"
                    "latitudes even and at least 4)\n",
                    size.longitude_cells, size.latitude_cells);
        return 1;
    }
    core.seed(options.seed);

    const double deformation_radius =
        std::sqrt(parameters.reduced_gravity_mps2 * parameters.layer_depth_m) /
        parameters.reference_coriolis;
    if (climatology.baked())
    {
        // The shear the mean state actually carries, not the one the analytic options describe:
        // a baked profile's peak shear is a property of the data and is what a reader needs in
        // order to compare this run against §11's analytic numbers at all.
        double peak_shear = 0.0;
        double peak_latitude = 0.0;
        for (int degree = -89; degree <= 89; ++degree)
        {
            const double latitude = double(degree) * 3.14159265358979323846 / 180.0;
            const double shear = climatology.upper_zonal_wind_mps(latitude, 0.0) -
                                 climatology.lower_zonal_wind_mps(latitude, 0.0);
            if (std::fabs(shear) > std::fabs(peak_shear))
            {
                peak_shear = shear;
                peak_latitude = double(degree);
            }
        }
        std::printf("mean state: baked climatology, peak shear %.1f m/s at %+.0f deg\n",
                    peak_shear, peak_latitude);
    }
    else
    {
        std::printf("mean state: analytic latitude bands\n");
    }
    std::printf("grid-scale damping %.1f h\n", parameters.grid_scale_damping_seconds / 3600.0);
    std::printf("global core %d x %d, deformation radius %.0f km, shear %.1f m/s\n",
                size.longitude_cells, size.latitude_cells, deformation_radius / 1000.0,
                bands.upper_jet_speed_mps - bands.lower_jet_speed_mps);
    std::printf("%8s %12s %12s %9s %9s %10s %9s %9s %9s\n", "day", "eddy_KE", "zonal_KE", "jet_m/s",
                "jet_lat", "peak_wind", "low_hPa", "water", "rain_mmd");

    std::ofstream series;
    if (!options.series_path.empty())
    {
        series.open(options.series_path);
        series << "day,eddy_kinetic_energy,zonal_kinetic_energy,jet_mps,jet_latitude_degrees,"
                  "peak_wind_mps,lowest_pressure_hpa,precipitable_water,precipitation_mm_per_day,"
                  "peak_ascent_cm_per_s,eddy_enstrophy\n";
    }

    std::vector<double> fit_day;
    std::vector<double> fit_energy;
    bool fit_open = true;

    const double sample_seconds = options.sample_hours * 3600.0;
    const double total_seconds = options.days * SECONDS_PER_DAY;
    const double step = parameters.step_seconds;

    double next_sample = 0.0;
    while (core.simulated_seconds() <= total_seconds)
    {
        if (core.simulated_seconds() >= next_sample)
        {
            const QuasiGeostrophicDiagnostics measured = core.diagnostics();
            const double day = core.simulated_seconds() / SECONDS_PER_DAY;
            std::printf("%8.2f %12.1f %12.1f %9.2f %9.1f %10.2f %9.2f %9.2f %9.3f\n", day,
                        measured.eddy_kinetic_energy_j_per_m2,
                        measured.zonal_kinetic_energy_j_per_m2, measured.jet_speed_mps,
                        measured.jet_latitude_radians * 180.0 / 3.14159265358979323846,
                        measured.peak_wind_mps, measured.lowest_pressure_anomaly_hpa,
                        measured.mean_precipitable_water_kg_per_m2,
                        measured.mean_precipitation_mm_per_day);
            std::fflush(stdout);

            if (series.is_open())
                series << day << ',' << measured.eddy_kinetic_energy_j_per_m2 << ','
                       << measured.zonal_kinetic_energy_j_per_m2 << ',' << measured.jet_speed_mps
                       << ',' << measured.jet_latitude_radians * 180.0 / 3.14159265358979323846
                       << ',' << measured.peak_wind_mps << ','
                       << measured.lowest_pressure_anomaly_hpa << ','
                       << measured.mean_precipitable_water_kg_per_m2 << ','
                       << measured.mean_precipitation_mm_per_day << ','
                       << measured.peak_ascent_mps * 100.0 << ','
                       << measured.potential_enstrophy << '\n';

            // The fit window: it must contain the exponential growth and nothing else.
            //
            // **Closing it matters as much as opening it** — the equilibrated state is a life
            // cycle that spends as much time decaying as growing, so a fit spanning both reports
            // a rate near zero for a core that is demonstrably unstable.
            //
            // **Opening it cannot be a fixed day.** It used to be "day >= 2", which held only
            // because the analytic mean state happened to be growing by then. A real climatology
            // is not smooth in latitude, so the seeded perturbation spends its first few days
            // shedding the part of itself that no mode wants before the unstable mode takes over
            // — energy *falls* first, and a window opened at a fixed day closes again one sample
            // later and reports nothing. So a fall is read as "still shaking down" and restarts
            // the window, until enough growth has accumulated that a fall can only be the peak.
            if (fit_open && day >= 2.0)
            {
                const bool falling = !fit_energy.empty() &&
                                     measured.eddy_kinetic_energy_j_per_m2 < fit_energy.back();
                if (falling && fit_energy.size() >= MIN_GROWTH_SAMPLES)
                {
                    fit_open = false;
                }
                else
                {
                    if (falling)
                    {
                        fit_day.clear();
                        fit_energy.clear();
                    }
                    fit_day.push_back(day);
                    fit_energy.push_back(measured.eddy_kinetic_energy_j_per_m2);
                }
            }

            next_sample += sample_seconds;
        }
        core.step(step);
    }

    const double rate = growth_rate_per_day(fit_day, fit_energy);
    // The energy grows as the square of the amplitude, so the amplitude's e-folding time is
    // twice the energy's — which is the number linear theory quotes.
    std::printf("\neddy energy growth %.4f /day over %zu samples; amplitude e-folding %.2f days\n",
                rate, fit_day.size(), rate > 0.0 ? 2.0 / rate : 0.0);
    std::printf("simulated %.1f days in %llu steps\n", core.simulated_seconds() / SECONDS_PER_DAY,
                static_cast<unsigned long long>(core.step_count()));
    return 0;
}
