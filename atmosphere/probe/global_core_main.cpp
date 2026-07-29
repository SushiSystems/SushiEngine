/**************************************************************************/
/* global_core_main.cpp                                                   */
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

namespace
{
    using SushiEngine::Atmosphere::QuasiGeostrophicCore;
    using SushiEngine::Atmosphere::QuasiGeostrophicDiagnostics;
    using SushiEngine::Atmosphere::QuasiGeostrophicGridSize;
    using SushiEngine::Atmosphere::QuasiGeostrophicParameters;

    constexpr double SECONDS_PER_DAY = 86400.0;

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
            else
            {
                std::printf(
                    "usage: atmosphere_global_probe [--days N] [--sample-hours N] [--seed N]\n"
                    "                               [--longitudes N] [--latitudes N]\n"
                    "                               [--upper-jet M] [--lower-jet M]\n"
                    "                               [--perturbation M] [--series PATH]\n");
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
    parameters.upper_jet_speed_mps = options.upper_jet_mps;
    parameters.lower_jet_speed_mps = options.lower_jet_mps;
    parameters.seed_perturbation_mps = options.perturbation_mps;

    QuasiGeostrophicCore core(size, parameters);
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
    std::printf("global core %d x %d, deformation radius %.0f km, shear %.1f m/s\n",
                size.longitude_cells, size.latitude_cells, deformation_radius / 1000.0,
                parameters.upper_jet_speed_mps - parameters.lower_jet_speed_mps);
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

            // The fit window: opens once the seeded perturbation has shaken down into the mode
            // that is actually going to grow, and **closes the first time the energy falls**.
            // Closing it matters as much as opening it — the equilibrated state is a life cycle
            // that spends as much time decaying as growing, and a fit spanning both reports a
            // rate near zero for a core that is demonstrably unstable.
            if (fit_open && day >= 2.0)
            {
                if (!fit_energy.empty() &&
                    measured.eddy_kinetic_energy_j_per_m2 < fit_energy.back())
                    fit_open = false;
                else
                {
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
