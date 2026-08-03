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

// Headless driver for the regional nest: brings up a Vulkan device with no window, steps the
// nest through hours of simulated weather in seconds of wall clock, and writes what the column
// under the observer actually contains.
//
// **Why this exists.** `docs/slop/atmosphere_system.md` §11's Phase B2c records that every
// hypothesis in that phase reasoned from a screenshot turned out wrong, and every one settled by
// porting the code and sampling it turned out right. The remaining questions it names — whether
// the vertical velocity survives the buoyancy correction, and where the boundary layer's water
// goes — are of the second kind, and until now they could only be asked through the editor: open
// a panel, switch on procedural weather, animate the sky, and wait half an hour of wall clock for
// three hours of weather that the mirror then reports one surface number of. This runs the same
// nest, the same shaders and the same parameters with the editor removed, at whatever rate the
// device manages, and prints the profile (§9.1's diagnostic slice) rather than the reduction.
//
// It is a *measuring instrument*, not a test: it asserts nothing and returns 0 whenever the nest
// ran. `test_atmosphere_nest.cpp` is where the base state, the saturation relations and the grid
// are pinned against textbook values. What this answers is the question a pass/fail cannot —
// "what is it doing, and at what height".

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <SushiEngine/environment/atmosphere_nest.hpp>
#include <SushiEngine/simulation/simulation_settings.hpp>
#include <SushiEngine/render/quality_params.hpp>

#include "atmosphere/atmosphere_nest.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "shader_catalogue.hpp"

namespace
{
    /** @brief Everything the run is parameterized by; all of it settable from the command line. */
    struct Options
    {
        double hours = 3.0;         /**< Simulated hours to run. */
        double sample_minutes = 10.0; /**< Simulated minutes between profile dumps. */
        float solar_sine = 0.9f;    /**< Fixed sine of solar elevation, unless `diurnal`. */
        bool diurnal = false;       /**< Drive the sun through a day/night cycle instead. */
        /**
         * @brief Length of one solar day in the nest's *own* elapsed seconds.
         *
         * 24 h is a real day. Anything shorter reproduces the case the editor is actually in:
         * the sky animates on the simulation's clock while the nest advances at most
         * `max_steps_per_frame` steps a frame and drops the surplus, so a sky running faster
         * than the nest can step gives it a *compressed* day — the same diurnal forcing over
         * less simulated time, and therefore less energy into the boundary layer per apparent
         * day. The Meteorology panel reports the ratio and offers to match the two; this is how
         * the consequence of not doing so is measured.
         */
        double day_seconds = 86400.0;
        float latitude_degrees = 45.0f; /**< Sets the Coriolis parameter. */
        std::string profile_path;   /**< CSV of the observer column's profile; empty = none. */
        std::string series_path;    /**< CSV of one line per sample; empty = none. */
        bool validation = false;    /**< Turn the Vulkan validation layers on. */

        // Overrides on `AtmosphereParameters`, so a hypothesis about one term can be separated
        // from the rest by running with it turned off rather than by arguing about it. NaN means
        // "leave the authored default alone", which keeps a default run identical to the scene's.
        float albedo = std::nanf("");
        float moisture_availability = std::nanf("");
        float heat_capacity = std::nanf("");
        float exchange = std::nanf("");
        float transmittance = std::nanf("");
        float surface_temperature = std::nanf("");
        float autoconversion_threshold = std::nanf("");
        float thermal_seed = std::nanf("");
        float seed_length = std::nanf("");
        float seed_period = std::nanf("");
        float eddy_viscosity = std::nanf("");
        float boundary_layer_depth = std::nanf("");
        float boundary_layer_velocity_scale = std::nanf("");
        float surface_humidity = std::nanf("");
        float critical_humidity = std::nanf("");
        float cloud_top_longwave = std::nanf("");
        float cloud_top_depression = std::nanf("");
        float cloud_top_entrainment = std::nanf("");
        float sponge_depth = std::nanf("");
        float sponge_rate = std::nanf("");
        // The parent solution the Davies zone relaxes toward. Zero is a quiescent airmass,
        // which is the hardest case the nest can be asked for and therefore the default.
        float forcing_humidity_anomaly = 0.0f;
        float forcing_theta_anomaly = 0.0f;
        float forcing_wind_east = 0.0f;
        /**
         * @brief Parent large-scale vertical motion, m/s; negative is subsidence.
         *
         * Zero leaves the term entirely inert, which is the state Phase B3e measured and is
         * therefore the default. A synoptic high delivers a couple of centimetres a second of
         * descent, so -0.02 is the value to reach for.
         */
        float forcing_vertical = 0.0f;
        std::uint32_t pressure_iterations = 0; /**< 0 = leave the default. */
        /**
         * @brief Steps of game time to advance the clock by per call into the nest.
         *
         * 1 asks the nest for one step per call. Higher values hand it a frame's worth of
         * elapsed time at once and exercise `max_steps_per_frame` — which is what the editor
         * does whenever the sky is animated faster than one step per frame, and what the
         * shipped code silently discarded. With a fixed sun the two should agree exactly, since
         * everything that varies within a frame's steps is passed per dispatch.
         */
        std::uint32_t batch = 1;
        /**
         * @brief Force the step length, seconds of game time; 0 leaves `choose_step` alone.
         *
         * Pins `min_step_seconds` and `max_step_seconds` together so the clamp delivers exactly
         * this. The step is what decides the tier's *total* cost — cost per step over seconds of
         * weather bought — and the vertical CFL that picks it is taken against an assumed 20 m/s
         * updraft rather than against anything the model produces, so what a longer step actually
         * costs in accuracy is a question only a measurement answers.
         */
        double dt = 0.0;
        /** @brief Quality tier whose nest discretization to run: low/medium/high/ultra. */
        std::string tier = "high";
    };

    void usage()
    {
        std::printf(
            "atmosphere_probe -- steps the regional nest headlessly and reports its column.\n\n"
            "  --hours <h>          simulated hours to run            (default 3)\n"
            "  --sample <minutes>   simulated minutes between samples (default 10)\n"
            "  --sun <sine>         fixed sine of solar elevation     (default 0.9)\n"
            "  --diurnal            drive the sun through a day/night cycle instead of holding it\n"
            "  --day <hours>        length of that cycle in simulated hours (default 24). Shorter\n"
            "                       reproduces a sky animating faster than the nest can step.\n"
            "  --latitude <deg>     latitude the Coriolis term is taken at (default 45)\n"
            "  --profile <path.csv> write every sampled profile, one row per level\n"
            "  --series <path.csv>  write one row per sample of the column summary\n"
            "  --validation         enable the Vulkan validation layers\n"
            "\nParameter overrides (unset leaves the authored default):\n"
            "\n  The surface, stated as what it *is* -- the fluxes are solved from these:\n"
            "  --albedo <0-1>       shortwave albedo of the surface\n"
            "  --beta <0-1>         moisture availability; the Bowen ratio's real author\n"
            "  --slab <J/m2/K>      surface heat capacity; how far the peak lags solar noon\n"
            "  --exchange <C_H>     bulk transfer coefficient for heat and moisture\n"
            "  --transmittance <0-1> clear-sky atmospheric transmittance at zenith\n"
            "  --surface-temp <K>   base-state surface temperature; lowering it brings the\n"
            "                       freezing level down into the cloud and glaciates it\n"
            "\n  Everything else:\n"
            "  --seed <0-1>         surface heating patchiness; 0 removes the symmetry break\n"
            "  --seed-length <m>    length that patchiness is correlated over; far below the\n"
            "                       cell spacing makes it white in space again\n"
            "  --seed-period <s>    time it is correlated over; far below the step makes it\n"
            "                       white in time again\n"
            "  --autoconversion <kg/kg> cloud water precipitation starts at; lowering it makes\n"
            "                       a thin deck rain, which is how the fall speeds are compared\n"
            "  --eddy <m2/s>        subgrid eddy viscosity\n"
            "  --pbl-depth <m>      cap on the diagnosed mixed-layer depth\n"
            "  --pbl-w <m/s>        mixed-layer turbulent velocity scale (Troen-Mahrt w_s)\n"
            "  --humidity <0-1>     base-state surface relative humidity\n"
            "  --critical <0-1>     relative humidity subgrid cloud begins at; 1 disables the\n"
            "                       subgrid closure and condenses on the cell mean alone\n"
            "  --cloud-top-lw <W/m2> longwave a cloud top at ambient loses; 0 removes the term\n"
            "  --cloud-top-floor <K> how far below ambient that loss shuts off; a large value\n"
            "                       reproduces the unbounded sink it was before it closed\n"
            "  --entrainment <A>    cloud-top entrainment efficiency; 0 removes the closure and\n"
            "                       reproduces the deck that sits on its radiative floor\n"
            "  --parent-humidity <f> parent-solution relative humidity anomaly\n"
            "  --parent-theta <K>   parent-solution potential temperature anomaly\n"
            "  --parent-wind <m/s>  parent-solution eastward wind\n"
            "  --sweeps <n>         red-black pressure sweeps per step\n"
            "  --batch <n>          steps of game time handed to the nest per call (default 1);\n"
            "                       above 1 exercises max_steps_per_frame\n"
            "  --dt <seconds>       force the step length instead of letting the CFL pick it\n"
            "  --tier <name>        nest discretization: low, medium, high (default), ultra\n");
    }

    /**
     * @brief Parses the command line into @p options.
     * @param argc    Argument count.
     * @param argv    Argument vector.
     * @param options Filled in place.
     * @return false when an argument was malformed or `--help` was asked for.
     */
    bool parse(int argc, char** argv, Options& options)
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string argument = argv[i];
            const auto value = [&](const char** out) -> bool
            {
                if (i + 1 >= argc)
                {
                    std::printf("error: %s needs a value\n", argument.c_str());
                    return false;
                }
                *out = argv[++i];
                return true;
            };
            const char* text = nullptr;

            if (argument == "--help" || argument == "-h")
                return false;
            else if (argument == "--diurnal")
                options.diurnal = true;
            else if (argument == "--validation")
                options.validation = true;
            else if (argument == "--hours" && value(&text))
                options.hours = std::atof(text);
            else if (argument == "--day" && value(&text))
                options.day_seconds = std::atof(text) * 3600.0;
            else if (argument == "--sample" && value(&text))
                options.sample_minutes = std::atof(text);
            else if (argument == "--sun" && value(&text))
                options.solar_sine = float(std::atof(text));
            else if (argument == "--latitude" && value(&text))
                options.latitude_degrees = float(std::atof(text));
            else if (argument == "--profile" && value(&text))
                options.profile_path = text;
            else if (argument == "--series" && value(&text))
                options.series_path = text;
            else if (argument == "--albedo" && value(&text))
                options.albedo = float(std::atof(text));
            else if (argument == "--beta" && value(&text))
                options.moisture_availability = float(std::atof(text));
            else if (argument == "--slab" && value(&text))
                options.heat_capacity = float(std::atof(text));
            else if (argument == "--exchange" && value(&text))
                options.exchange = float(std::atof(text));
            else if (argument == "--transmittance" && value(&text))
                options.transmittance = float(std::atof(text));
            else if (argument == "--surface-temp" && value(&text))
                options.surface_temperature = float(std::atof(text));
            else if (argument == "--autoconversion" && value(&text))
                options.autoconversion_threshold = float(std::atof(text));
            else if (argument == "--seed" && value(&text))
                options.thermal_seed = float(std::atof(text));
            else if (argument == "--seed-length" && value(&text))
                options.seed_length = float(std::atof(text));
            else if (argument == "--seed-period" && value(&text))
                options.seed_period = float(std::atof(text));
            else if (argument == "--eddy" && value(&text))
                options.eddy_viscosity = float(std::atof(text));
            else if (argument == "--pbl-depth" && value(&text))
                options.boundary_layer_depth = float(std::atof(text));
            else if (argument == "--pbl-w" && value(&text))
                options.boundary_layer_velocity_scale = float(std::atof(text));
            else if (argument == "--humidity" && value(&text))
                options.surface_humidity = float(std::atof(text));
            else if (argument == "--sponge-depth" && value(&text))
                options.sponge_depth = float(std::atof(text));
            else if (argument == "--sponge-rate" && value(&text))
                options.sponge_rate = float(std::atof(text));
            else if (argument == "--cloud-top-lw" && value(&text))
                options.cloud_top_longwave = float(std::atof(text));
            else if (argument == "--cloud-top-floor" && value(&text))
                options.cloud_top_depression = float(std::atof(text));
            else if (argument == "--entrainment" && value(&text))
                options.cloud_top_entrainment = float(std::atof(text));
            else if (argument == "--critical" && value(&text))
                options.critical_humidity = float(std::atof(text));
            else if (argument == "--parent-humidity" && value(&text))
                options.forcing_humidity_anomaly = float(std::atof(text));
            else if (argument == "--parent-theta" && value(&text))
                options.forcing_theta_anomaly = float(std::atof(text));
            else if (argument == "--parent-wind" && value(&text))
                options.forcing_wind_east = float(std::atof(text));
            else if (argument == "--parent-subsidence" && value(&text))
                options.forcing_vertical = float(std::atof(text));
            else if (argument == "--sweeps" && value(&text))
                options.pressure_iterations = std::uint32_t(std::atoi(text));
            else if (argument == "--batch" && value(&text))
                options.batch = std::uint32_t(std::max(1, std::atoi(text)));
            else if (argument == "--dt" && value(&text))
                options.dt = std::atof(text);
            else if (argument == "--tier" && value(&text))
                options.tier = text;
            else if (text == nullptr)
            {
                std::printf("error: unknown argument '%s'\n", argument.c_str());
                return false;
            }
        }
        return true;
    }

    /**
     * @brief The parent solution the nest's Davies zone relaxes toward.
     *
     * Deliberately quiescent — no wind, no thermal anomaly, no humidity anomaly — so that
     * anything the nest develops is the nest's own convection and not something advected in
     * through its boundary. That is the experiment the open questions call for; a probe that
     * blew a front across the domain would answer a different one.
     */
    std::vector<SushiEngine::Render::AtmosphereForcingSample> uniform_forcing(
        float humidity_anomaly, float theta_anomaly, float wind_east, float vertical)
    {
        const int cells = SushiEngine::Render::ATMOSPHERE_FORCING_MAX_CELLS;
        SushiEngine::Render::AtmosphereForcingSample sample{};
        sample.humidity_anomaly = humidity_anomaly;
        sample.theta_anomaly_k = theta_anomaly;
        sample.wind_east_mps = wind_east;
        sample.vertical_velocity_mps = vertical;
        return std::vector<SushiEngine::Render::AtmosphereForcingSample>(
            std::size_t(cells) * std::size_t(cells), sample);
    }

    /** @brief The largest column peak |w| anywhere in the mirror, m/s. */
    float domain_peak_updraft(const SushiEngine::Render::AtmosphereMirror& mirror)
    {
        float peak = 0.0f;
        const std::size_t count = std::size_t(mirror.cells) * std::size_t(mirror.cells);
        for (std::size_t i = 0; i < count; ++i)
            peak = std::max(peak, mirror.columns[i].extent[2]);
        return peak;
    }

    /**
     * @brief What the whole domain's sky looks like, rather than one column of it.
     *
     * The observer column is a single 2 km cell out of 192², and a cloud field is by nature
     * intermittent — so a run can be reported as clear while a quarter of the domain holds
     * cumulus, purely by where the centre happened to land. "Can I see clouds" is a question
     * about the sky, and the sky is the mirror.
     */
    struct DomainSky
    {
        float cloudy_columns = 0.0f; /**< Fraction of columns holding any cloud at all. */
        float mean_coverage = 0.0f;  /**< Mean low-band coverage over every column. */
        float mean_base_m = 0.0f;    /**< Mean cloud base over the cloudy columns, metres. */
        /**
         * @brief Standard deviation of that coverage across the domain.
         *
         * The mean alone cannot tell a broken cumulus field from a uniform sheet of the same
         * total cloud, and that distinction is the entire question the thermal seed exists to
         * decide: a uniformly heated surface rises as one slab, so a *flat* coverage field is
         * the symptom of a symmetry that never broke. `cloudy_columns` saturates at 1 as soon as
         * every column holds a trace, which is exactly when it stops discriminating; this does
         * not saturate.
         */
        float coverage_sd = 0.0f;
        /**
         * @brief Mean absolute difference between neighbouring columns' coverage.
         *
         * The same field measured at the *shortest* scale the mirror resolves rather than over
         * the whole domain. Together with `coverage_sd` it separates the two ways a sky can be
         * variable: a few large cloud masses give a high deviation and a low roughness, while
         * structure at the lattice scale gives them in equal measure — and structure at the
         * lattice scale is the one thing a grid-mean model has not earned.
         */
        float coverage_roughness = 0.0f;
        /**
         * @brief Fraction of columns whose low-band coverage is exactly 0 or exactly 1.
         *
         * The saturation detector for the two statistics above. `nest_cloud_partition` clamps a
         * level's cloud fraction to exactly 0 or exactly 1 once the cell mean leaves the top-hat
         * distribution's width, and a column of clamped levels publishes a coverage of exactly 0
         * or exactly 1 — so when this reads 1.0 the coverage field is binary, its mean, deviation
         * and roughness are exact rationals of the column count, and all three will hold to four
         * decimal places for as long as every column stays inside its clamp. A frozen sky report
         * with this at 1.0 is the closure pinned, not the mirror stuck.
         */
        float coverage_pinned = 0.0f;
    };

    DomainSky domain_sky(const SushiEngine::Render::AtmosphereMirror& mirror)
    {
        DomainSky sky;
        const std::size_t count = std::size_t(mirror.cells) * std::size_t(mirror.cells);
        if (count == 0)
            return sky;
        std::size_t cloudy = 0;
        std::size_t pinned = 0;
        double coverage = 0.0;
        double base = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const float cover = mirror.columns[i].bands[0][0];
            coverage += double(cover);
            // Exact comparisons on purpose: the closure's clamps write exactly these values,
            // and anything the physics computed in between lands strictly inside (0, 1).
            if (cover == 0.0f || cover == 1.0f)
                ++pinned;
            if (mirror.columns[i].surface[3] > 0.0f)
            {
                ++cloudy;
                base += double(mirror.columns[i].surface[3]);
            }
        }
        sky.coverage_pinned = float(double(pinned) / double(count));
        sky.cloudy_columns = float(double(cloudy) / double(count));
        sky.mean_coverage = float(coverage / double(count));
        sky.mean_base_m = cloudy > 0 ? float(base / double(cloudy)) : 0.0f;

        const std::int32_t side = mirror.cells;
        double variance = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const double d = double(mirror.columns[i].bands[0][0]) - double(sky.mean_coverage);
            variance += d * d;
        }
        sky.coverage_sd = float(std::sqrt(variance / double(count)));

        // Both axes, and only the pairs that exist — the lattice does not wrap, so the domain
        // edge is not a neighbour of the opposite edge.
        double rough = 0.0;
        std::size_t pairs = 0;
        for (std::int32_t z = 0; z < side; ++z)
            for (std::int32_t x = 0; x < side; ++x)
            {
                const double here = double(mirror.columns[std::size_t(z) * std::size_t(side) +
                                                          std::size_t(x)].bands[0][0]);
                if (x + 1 < side)
                {
                    rough += std::fabs(double(mirror.columns[std::size_t(z) * std::size_t(side) +
                                                             std::size_t(x + 1)].bands[0][0]) - here);
                    ++pairs;
                }
                if (z + 1 < side)
                {
                    rough += std::fabs(double(mirror.columns[std::size_t(z + 1) * std::size_t(side) +
                                                             std::size_t(x)].bands[0][0]) - here);
                    ++pairs;
                }
            }
        sky.coverage_roughness = pairs > 0 ? float(rough / double(pairs)) : 0.0f;
        return sky;
    }

    /** @brief Column-integrated water, kg/m² — vapour, cloud and rain against their base state. */
    struct WaterBudget
    {
        double vapour = 0.0;
        double base_vapour = 0.0;
        double condensate = 0.0;
    };

    WaterBudget water_budget(const SushiEngine::Render::AtmosphereMirror& mirror,
                             const SushiEngine::Render::AtmosphereParameters& parameters,
                             const SushiEngine::Render::AtmosphereNestSize& size)
    {
        WaterBudget budget;
        for (std::int32_t k = 0; k < mirror.profile_levels; ++k)
        {
            const SushiEngine::Render::AtmosphereProfileLevel& level = mirror.profile[k];
            const float thickness = SushiEngine::Render::atmosphere_level_thickness(
                std::uint32_t(k), size.levels, size.top_m);
            // Mixing ratio is per kilogram of dry air, so the base-state density is what turns
            // it into the kilograms per square metre a water budget is actually stated in.
            const double mass = double(SushiEngine::Render::atmosphere_base_density(
                                    parameters, level.altitude_m)) *
                                double(thickness);
            budget.vapour += double(level.vapour) * mass;
            budget.base_vapour += double(level.base_vapour) * mass;
            budget.condensate += double(level.cloud_water + level.rain) * mass;
        }
        return budget;
    }

    /**
     * @brief Accumulates the nest's per-stage GPU cost over the steps of a run.
     *
     * A single step's timestamps are noisy — a few tens of microseconds of scheduling jitter on
     * a dispatch chain that runs for a couple of milliseconds — and the question the profile has
     * to settle is a *ratio*: whether the pressure sweeps really are the large share of the step
     * that `atmosphere_pressure.comp`'s own header supposes, since that is what decides whether
     * lowering `pressure_iterations` buys anything. So this means over every step of the run
     * rather than reporting the last one.
     *
     * Keyed by name rather than by index because the set of sections is not fixed: a frame that
     * re-centres the lattice records a shift the next one does not, and a positional accumulator
     * would silently add that shift into whatever section followed it.
     */
    struct CostAccumulator
    {
        std::vector<std::string> names;
        std::vector<double> milliseconds;
        double submission = 0.0;
        /** @brief Measured submissions — the divisor for the per-stage breakdown. */
        std::uint64_t submissions = 0;
        /** @brief Steps across those submissions — the divisor for the submission total. */
        std::uint64_t steps = 0;
        std::uint64_t last_step_index = 0;

        void add(const SushiEngine::Render::AtmosphereStepCost& cost)
        {
            // Each measurement is published once per completed submission, but the probe calls
            // step() twice per sample and reads the cost from whichever slot last completed —
            // so the same step's numbers can be seen more than once.
            if (!cost.measured || cost.step_index == last_step_index)
                return;
            last_step_index = cost.step_index;
            // Two divisors, because the two halves of a measurement have different denominators:
            // the submission bracket covers every step the frame recorded, while the breakdown
            // is the first step alone (the query pool holds a fixed number of sections). Keeping
            // them apart is what makes both means per-step whatever the step cap is set to.
            ++submissions;
            steps += cost.steps > 0 ? cost.steps : 1;
            submission += double(cost.total_ms);
            for (int i = 0; i < cost.count; ++i)
            {
                const std::string name = cost.stages[i].name;
                std::size_t slot = names.size();
                for (std::size_t j = 0; j < names.size(); ++j)
                    if (names[j] == name)
                    {
                        slot = j;
                        break;
                    }
                if (slot == names.size())
                {
                    names.push_back(name);
                    milliseconds.push_back(0.0);
                }
                milliseconds[slot] += double(cost.stages[i].milliseconds);
            }
        }

        /**
         * @brief Prints the breakdown against §12's budget.
         *
         * The per-stage mean divides by the number of *submissions*, not by how many of them
         * recorded that section, so a shift that happens on one frame in fifty is reported as
         * the per-frame cost it actually is rather than as the cost of the frames it appeared
         * on.
         */
        void report(double budget_ms) const
        {
            if (submissions == 0 || steps == 0)
            {
                std::printf("\nno step cost measured (device reports no timestamp support, or "
                            "no step completed)\n");
                return;
            }
            const double total = submission / double(steps);
            std::printf("\nstep cost over %llu steps in %llu submissions, mean milliseconds:\n",
                        static_cast<unsigned long long>(steps),
                        static_cast<unsigned long long>(submissions));
            double sum = 0.0;
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                const double mean = milliseconds[i] / double(submissions);
                sum += mean;
                std::printf("  %-16s %8.3f  %5.1f %%\n", names[i].c_str(), mean,
                            total > 0.0 ? 100.0 * mean / total : 0.0);
            }
            // Stated rather than hidden: a breakdown that adds up to less than the measured
            // submission is dispatches the barriers let overlap, and one that adds up to more is
            // a section's opening timestamp written before the previous section drained. Either
            // way the submission is the number to budget against.
            std::printf("  %-16s %8.3f  (sections sum to %.3f)\n", "SUBMISSION", total, sum);
            std::printf("  budget (doc §12) %8.3f  -- %s\n", budget_ms,
                        total <= budget_ms ? "inside" : "OVER");
        }
    };

    /**
     * @brief Opens @p path for writing, reporting the failure itself.
     * @param stream Stream to open; left closed when @p path is empty.
     * @param path   File to write, or empty to do nothing.
     * @return false only when a non-empty path could not be opened.
     */
    bool open_csv(std::ofstream& stream, const std::string& path)
    {
        if (path.empty())
            return true;
        stream.open(path);
        if (stream.is_open())
            return true;
        std::printf("error: cannot open %s\n", path.c_str());
        return false;
    }

    void write_profile_header(std::ofstream& file)
    {
        file << "simulated_s,level,altitude_m,pressure_pa,temperature_k,"
                "theta_pert_k,vapour,base_vapour,saturation,relative_humidity,"
                "cloud_water,cloud_fraction,rain,wind_e,wind_n,wind_up,extinction,buoyancy,"
                "divergence\n";
    }

    void write_profile(std::ofstream& file, double simulated_seconds,
                       const SushiEngine::Render::AtmosphereMirror& mirror)
    {
        // Enough digits that a difference between two samples is a physical change rather than
        // a rounding of the report. The quantization this probe was built to settle would have
        // been invisible at a default six significant figures.
        file << std::setprecision(9);
        for (std::int32_t k = 0; k < mirror.profile_levels; ++k)
        {
            const SushiEngine::Render::AtmosphereProfileLevel& level = mirror.profile[k];
            file << simulated_seconds << ',' << k << ',' << level.altitude_m << ','
                 << level.pressure_pa << ',' << level.temperature_k << ','
                 << level.theta_perturbation_k << ',' << level.vapour << ','
                 << level.base_vapour << ',' << level.saturation << ','
                 << (level.saturation > 0.0f ? level.vapour / level.saturation : 0.0f) << ','
                 << level.cloud_water << ',' << level.cloud_fraction << ',' << level.rain << ','
                 << level.wind_east_mps << ','
                 << level.wind_north_mps << ',' << level.wind_up_mps << ',' << level.extinction
                 << ',' << level.buoyancy << ',' << level.divergence << '\n';
        }
        file.flush();
    }
} // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse(argc, argv, options))
    {
        usage();
        return 1;
    }

    try
    {
        SushiEngine::Render::RenderDeviceDesc desc;
        desc.enable_validation = options.validation;
        SushiEngine::Render::Vulkan::VulkanDevice device(desc);
        std::printf("device: %s\n", device.info().name.c_str());

        // No watch directory: the probe runs the shaders the binary was built with, so a run is
        // reproducible against a commit rather than against whatever is on disk.
        SushiEngine::Render::Resources::ShaderLibrary shaders(
            device, std::string(), SushiEngine::Render::shader_catalogue(),
            SushiEngine::Render::shader_catalogue_count());
        SushiEngine::Render::Resources::PipelineCache cache(device, std::string());
        SushiEngine::Render::Resources::GraphicsPipelineFactory pipelines(device, cache);
        SushiEngine::Render::Resources::SamplerCache samplers(device);

        // Through the same resolver the host uses — the *atmosphere* tier, not the render
        // tier, which no longer carries the nest — so a probe run at a tier is that tier
        // and not a second opinion about what it means.
        SushiEngine::Simulation::AtmosphereQuality tier =
            SushiEngine::Simulation::AtmosphereQuality::High;
        if (options.tier == "low")
            tier = SushiEngine::Simulation::AtmosphereQuality::Low;
        else if (options.tier == "medium")
            tier = SushiEngine::Simulation::AtmosphereQuality::Medium;
        else if (options.tier == "ultra")
            tier = SushiEngine::Simulation::AtmosphereQuality::Ultra;
        const SushiEngine::Render::AtmosphereNestSize size =
            SushiEngine::Simulation::resolve_atmosphere_quality(tier);
        SushiEngine::Render::Atmosphere::AtmosphereNest nest(device, shaders, pipelines, samplers,
                                                             size);
        std::printf("nest: %ux%ux%u at %.0f m, top %.0f m\n", size.cells_x, size.cells_z,
                    size.levels, double(size.spacing_m), double(size.top_m));

        SushiEngine::Render::AtmosphereParameters parameters;
        const auto override_float = [](float& target, float value)
        {
            if (!std::isnan(value))
                target = value;
        };
        override_float(parameters.surface_albedo, options.albedo);
        override_float(parameters.surface_moisture_availability, options.moisture_availability);
        override_float(parameters.surface_heat_capacity, options.heat_capacity);
        override_float(parameters.surface_exchange_coefficient, options.exchange);
        override_float(parameters.clear_sky_transmittance, options.transmittance);
        override_float(parameters.surface_temperature, options.surface_temperature);
        override_float(parameters.autoconversion_threshold, options.autoconversion_threshold);
        override_float(parameters.thermal_seed_amplitude, options.thermal_seed);
        override_float(parameters.thermal_seed_length_m, options.seed_length);
        override_float(parameters.thermal_seed_period_s, options.seed_period);
        override_float(parameters.eddy_viscosity, options.eddy_viscosity);
        override_float(parameters.boundary_layer_depth_m, options.boundary_layer_depth);
        override_float(parameters.boundary_layer_velocity_scale,
                       options.boundary_layer_velocity_scale);
        override_float(parameters.surface_humidity, options.surface_humidity);
        override_float(parameters.cloud_critical_humidity, options.critical_humidity);
        override_float(parameters.cloud_top_longwave_flux, options.cloud_top_longwave);
        override_float(parameters.cloud_top_equilibrium_depression, options.cloud_top_depression);
        override_float(parameters.cloud_top_entrainment_efficiency, options.cloud_top_entrainment);
        override_float(parameters.sponge_depth, options.sponge_depth);
        override_float(parameters.sponge_rate, options.sponge_rate);
        if (options.pressure_iterations > 0)
            parameters.pressure_iterations = options.pressure_iterations;
        if (options.dt > 0.0)
        {
            // Both ends together, so `choose_step`'s clamp delivers exactly this whatever the
            // CFL would have chosen.
            parameters.min_step_seconds = float(options.dt);
            parameters.max_step_seconds = float(options.dt);
        }
        // The surface is stated by what it *is*, not by the fluxes it delivers: those are solved.
        // "seed" is a fraction of the absorbed shortwave, not a heating rate; it stopped being an
        // additive kick on theta when that turned out to be an unbounded random walk.
        std::printf("surface: albedo %.2f, beta %.2f, slab %.2g J/m2/K, C_H %.4f, tau %.2f\n",
                    double(parameters.surface_albedo),
                    double(parameters.surface_moisture_availability),
                    double(parameters.surface_heat_capacity),
                    double(parameters.surface_exchange_coefficient),
                    double(parameters.clear_sky_transmittance));
        std::printf("forcing: seed %.3f over %.0f m and "
                    "%.0f s, eddy %.0f m2/s, %u sweeps\n",
                    double(parameters.thermal_seed_amplitude),
                    double(parameters.thermal_seed_length_m),
                    double(parameters.thermal_seed_period_s),
                    double(parameters.eddy_viscosity), parameters.pressure_iterations);
        std::printf("closure: mixed layer to %.0f m at w_s %.2f m/s, subgrid cloud from RH %.2f\n",
                    double(parameters.boundary_layer_depth_m),
                    double(parameters.boundary_layer_velocity_scale),
                    double(parameters.cloud_critical_humidity));

        const std::vector<SushiEngine::Render::AtmosphereForcingSample> samples =
            uniform_forcing(options.forcing_humidity_anomaly, options.forcing_theta_anomaly,
                            options.forcing_wind_east, options.forcing_vertical);

        // The nest addresses the parent solution in scene-absolute metres; with the observer at
        // the origin the whole forcing field maps onto the domain and stays there.
        const double span = double(size.spacing_m) * double(size.cells_x);
        SushiEngine::Render::AtmosphereForcing forcing;
        forcing.samples = samples.data();
        forcing.cells_x = SushiEngine::Render::ATMOSPHERE_FORCING_MAX_CELLS;
        forcing.cells_z = SushiEngine::Render::ATMOSPHERE_FORCING_MAX_CELLS;
        forcing.revision = 1;
        forcing.uv_scale_x = float(1.0 / span);
        forcing.uv_scale_z = float(1.0 / span);
        forcing.uv_offset_x = 0.5f;
        forcing.uv_offset_z = 0.5f;
        forcing.observer_x = 0.0;
        forcing.observer_z = 0.0;
        const double EARTH_ROTATION = 7.2921159e-5;
        forcing.coriolis = float(2.0 * EARTH_ROTATION *
                                 std::sin(double(options.latitude_degrees) * 3.14159265358979 /
                                          180.0));

        std::ofstream profile_file;
        std::ofstream series_file;
        if (!open_csv(profile_file, options.profile_path) ||
            !open_csv(series_file, options.series_path))
            return 1;
        if (profile_file.is_open())
            write_profile_header(profile_file);
        if (series_file.is_open())
            series_file << "simulated_s,sun_sine,surface_rh,column_peak_w,domain_peak_w,lcl_m,"
                           "cloud_base_m,cloud_top_m,rain_mm_h,vapour_kg_m2,base_vapour_kg_m2,"
                           "condensate_kg_m2,peak_buoyancy,peak_divergence,peak_cloud_fraction,"
                           "peak_fraction_altitude_m,sky_cloudy_columns,sky_mean_coverage,"
                           "sky_mean_base_m,sky_coverage_sd,sky_coverage_roughness,"
                           "skin_k,sensible_w_m2,latent_w_m2,net_radiation_w_m2,"
                           "sky_coverage_pinned\n";

        const double total_seconds = options.hours * 3600.0;
        const double sample_seconds = std::max(options.sample_minutes * 60.0, 1.0);
        // The step the nest will choose on its *first* call, before any readback has given the
        // vertical CFL something measured to bind against. The clock this probe advances has to
        // be the one the nest consumes: it takes as many steps as the elapsed game time is due,
        // up to `max_steps_per_frame`, and drops the rest — so advancing by one step per call
        // asks for exactly one and can never be the case that drops any, which is what makes a
        // probe run comparable against the editor's rather than a compressed version of it.
        //
        // Re-read from the nest each iteration below, because the step is no longer a constant:
        // it lengthens in a quiet airmass and tightens when convection gets going.
        const float thinnest =
            SushiEngine::Render::atmosphere_level_thickness(0, size.levels, size.top_m);
        double step = double(std::clamp(
            parameters.courant_target * thinnest /
                std::max(10.0f * parameters.convective_velocity_scale, 1.0f),
            parameters.min_step_seconds, parameters.max_step_seconds));
        // What the clock advances by per call. The nest is asked for `batch` steps at a time and
        // will take them all as long as `max_steps_per_frame` allows; anything above that cap it
        // drops, which is the behaviour this option exists to be able to see.
        double advance = step * double(options.batch);
        std::printf("stepping %.2f s of game time per step (initial), %.0f steps for %.1f h, "
                    "%u step(s) per call\n",
                    step, total_seconds / step, options.hours, options.batch);
        // What the tier costs is not the step but the step over the weather it buys, so this is
        // the number to compare across runs with different step lengths.
        std::printf("(a step buys %.2f s of weather; cost per simulated second is the step cost "
                    "divided by that)\n",
                    step);
        if (options.batch > parameters.max_steps_per_frame)
            std::printf("note: --batch %u exceeds max_steps_per_frame %u, so %u step(s) per call "
                        "will be dropped\n",
                        options.batch, parameters.max_steps_per_frame,
                        options.batch - parameters.max_steps_per_frame);

        double next_sample = 0.0;
        std::uint32_t samples_written = 0;
        CostAccumulator cost;
        // The last four are the domain's sky rather than the observer's column, which is the
        // difference between "is there cloud where I am standing" and "is there cloud" — and the
        // last two are the difference between a cloud *field* and a sheet of the same total
        // cloud, which is the question the surface heterogeneity decides.
        // The three after the sun are the surface energy balance's own state, which is no longer
        // an input: skin temperature in Celsius and the two turbulent fluxes it delivers. Their
        // ratio is the Bowen ratio, measured rather than authored, and watching it drift through
        // the day is watching the ground dry out.
        std::printf("\n%10s %8s %8s %8s %8s %10s %9s %8s %8s %9s %8s %8s %9s %9s %8s\n", "sim_s",
                    "sun", "skin_c", "H", "LE", "dom_w", "lcl_m", "water", "cld_frac", "frac_m",
                    "sky_pct", "sky_cov", "sky_sd", "sky_rough", "sky_pin");
        for (double elapsed = 0.0; elapsed <= total_seconds + 0.5 * advance;
             elapsed += advance, advance = std::max(double(nest.step_seconds()), 1e-3) *
                                           double(options.batch))
        {
            forcing.total_seconds = elapsed;
            forcing.solar_elevation_sine =
                options.diurnal
                    ? float(std::sin(2.0 * 3.14159265358979 * elapsed /
                                     std::max(options.day_seconds, 1.0)))
                    : options.solar_sine;
            nest.step(parameters, forcing);
            cost.add(nest.step_cost());

            if (elapsed + 0.5 * advance < next_sample)
                continue;

            // The readback is asynchronous by design (§3.2), so a sample has to wait for the
            // step it is asking about. Idling the device is legal *here* and nowhere else: the
            // renderer must never do this, and the probe has no frame to protect.
            vkDeviceWaitIdle(device.device());
            // Collecting happens at the top of step(); a call with no time left to spend does
            // nothing but publish whatever finished.
            nest.step(parameters, forcing);
            cost.add(nest.step_cost());

            const SushiEngine::Render::AtmosphereMirror mirror = nest.atmosphere_mirror();
            if (!mirror.valid() || mirror.profile == nullptr)
                continue;
            next_sample = elapsed + sample_seconds;

            const SushiEngine::Render::AtmosphereMirrorColumn& column =
                mirror.columns[std::size_t(mirror.cells / 2) * std::size_t(mirror.cells) +
                               std::size_t(mirror.cells / 2)];
            const WaterBudget budget = water_budget(mirror, parameters, size);
            const float domain_peak = domain_peak_updraft(mirror);
            const DomainSky sky = domain_sky(mirror);

            float peak_buoyancy = 0.0f;
            float peak_divergence = 0.0f;
            // The cloudiest level and how high it is — the two numbers this phase turns on.
            // "Some condensate somewhere" was true of every earlier run too; what distinguishes
            // a cumulus field from fog is that the cloud is at the mixed-layer top and not at
            // 19 m, and a column summary cannot say which because it reduces over height.
            float peak_fraction = 0.0f;
            float peak_fraction_altitude = 0.0f;
            for (std::int32_t k = 0; k < mirror.profile_levels; ++k)
            {
                peak_buoyancy =
                    std::max(peak_buoyancy, std::fabs(mirror.profile[k].buoyancy));
                peak_divergence =
                    std::max(peak_divergence, std::fabs(mirror.profile[k].divergence));
                if (mirror.profile[k].cloud_fraction > peak_fraction)
                {
                    peak_fraction = mirror.profile[k].cloud_fraction;
                    peak_fraction_altitude = mirror.profile[k].altitude_m;
                }
            }

            std::printf("%10.0f %8.3f %8.1f %8.0f %8.0f %10.2e %9.0f %8.2f %8.3f %9.0f %8.1f "
                        "%8.3f %9.4f %9.4f %8.3f\n",
                        mirror.simulated_seconds, double(forcing.solar_elevation_sine),
                        double(column.skin[0]) - 273.15, double(column.skin[1]),
                        double(column.skin[2]), double(domain_peak),
                        double(column.extent[3]), budget.vapour, double(peak_fraction),
                        double(peak_fraction_altitude), double(sky.cloudy_columns) * 100.0,
                        double(sky.mean_coverage), double(sky.coverage_sd),
                        double(sky.coverage_roughness), double(sky.coverage_pinned));

            if (profile_file.is_open())
                write_profile(profile_file, mirror.simulated_seconds, mirror);
            if (series_file.is_open())
            {
                series_file << std::setprecision(9) << mirror.simulated_seconds << ','
                            << forcing.solar_elevation_sine << ',' << column.extent[1] << ','
                            << column.extent[2] << ',' << domain_peak << ','
                            << column.extent[3] << ',' << column.surface[3] << ','
                            << column.extent[0] << ',' << column.surface[0] << ','
                            << budget.vapour << ',' << budget.base_vapour << ','
                            << budget.condensate << ',' << peak_buoyancy << ','
                            << peak_divergence << ',' << peak_fraction << ','
                            << peak_fraction_altitude << ',' << sky.cloudy_columns << ','
                            << sky.mean_coverage << ',' << sky.mean_base_m << ','
                            << sky.coverage_sd << ',' << sky.coverage_roughness << ','
                            << column.skin[0] << ',' << column.skin[1] << ','
                            << column.skin[2] << ',' << column.skin[3] << ','
                            << sky.coverage_pinned << '\n';
                series_file.flush();
            }
            ++samples_written;
        }

        vkDeviceWaitIdle(device.device());

        // §12's measured whole-step figure in the GPU's sustained clock state. The step is
        // budgeted against what the machine actually delivers, not against the retired 2 ms
        // aspiration a probe on a boosting-then-throttling GPU could never compare against.
        cost.report(12.3);

        std::printf("\nsimulated %.0f s (%.2f h) in %llu steps, %u samples\n",
                    nest.simulated_seconds(), nest.simulated_seconds() / 3600.0,
                    static_cast<unsigned long long>(nest.step_count()), samples_written);
        std::printf("RESULT: %s\n", samples_written > 0 ? "OK" : "FAIL (no readback completed)");
        return samples_written > 0 ? 0 : 1;
    }
    catch (const std::exception& error)
    {
        std::printf("RESULT: FAIL (%s)\n", error.what());
        return 1;
    }
}
