/**************************************************************************/
/* atmosphere_nest.hpp                                                    */
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
 * @file atmosphere_nest.hpp
 * @brief The regional nest's parameters, vertical grid, and base state — as data.
 *
 * `docs/slop/atmosphere_system.md` §6 and §13. The shipped W4–W6 weather kept thirteen
 * magic constants inside the body of one 110-line tick function (§1.3); this file is the
 * opposite arrangement and the reason the design doc states it twice: **every physical
 * constant lives in @ref AtmosphereParameters, is serialized with the scene, and is
 * editable. None is a function-local constant.** A new microphysics scheme, a
 * different planet, or a per-biome tuning pass is then a data edit rather than a
 * recompile of a loop body.
 *
 * The file is deliberately dependency-free (`<cstdint>` and `<cmath>` only), like
 * `weather_field.hpp` and for the same reason: it is read by the render tier that runs the
 * nest, by the simulation that publishes its forcing, by the editor that authors it, and by
 * the serializer that persists it, and none of those should have to agree on anything else
 * to talk about the atmosphere.
 *
 * **What is mirrored on the GPU, and why that is not a duplication smell.**
 * `render/shaders/atmosphere_nest_common.glsl` restates the vertical grid, the base state,
 * and the thermodynamic relations below. GLSL cannot include a C++ header, so the choice is
 * between mirroring the *formulas* (whose inputs are the uploaded parameters, so there is
 * exactly one set of numbers) or mirroring the *numbers* (so there are two). The formulas
 * are mirrored; the numbers are not, and the pair name each other so neither is edited
 * alone. Everything mirrored here has a test in `test_atmosphere_nest.cpp` pinned to values
 * a meteorology text will confirm, which is what makes the mirror checkable rather than
 * merely asserted.
 */

#include <cmath>
#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief Every constant the regional nest's physics is expressed in terms of.
         *
         * Grouped by what changes them: the first block is the thermodynamics of air and
         * water (a planet-scale property), the second the base state the anelastic
         * approximation linearises about, the third the dynamics' numerics, the fourth the
         * microphysics scheme's rate constants, and the last the surface forcing. Defaults
         * are Earth's, and each is the value the source named in its own comment gives.
         */
        struct AtmosphereParameters
        {
            // ---- Thermodynamics of moist air -----------------------------------------

            /** @brief Specific gas constant of dry air, J/(kg·K). */
            float gas_constant_dry = 287.05f;
            /** @brief Specific gas constant of water vapour, J/(kg·K). */
            float gas_constant_vapour = 461.51f;
            /** @brief Specific heat of dry air at constant pressure, J/(kg·K). */
            float specific_heat_pressure = 1005.0f;
            /** @brief Latent heat of vaporization at 0 °C, J/kg. */
            float latent_heat_vaporization = 2.501e6f;
            /** @brief Gravitational acceleration used by buoyancy and hydrostatics, m/s². */
            float gravity = 9.80665f;
            /** @brief Reference pressure the Exner function is defined against, Pa. */
            float reference_pressure = 100000.0f;
            /** @brief Density of liquid water, kg/m³ — the optical extinction's denominator. */
            float water_density = 1000.0f;

            // ---- Base state ----------------------------------------------------------
            //
            // The anelastic system is linearised about a horizontally uniform, hydrostatic
            // reference profile: this is that profile. Sound waves are filtered out by
            // holding the reference density fixed, which is the whole reason the nest's time
            // step is set by advection and buoyancy rather than by the speed of sound — and
            // therefore the reason a non-hydrostatic model is affordable at all (§2.3).

            /** @brief Reference surface temperature, K (ISA: 15 °C). */
            float surface_temperature = 288.15f;
            /** @brief Reference surface pressure, Pa (ISA). */
            float surface_pressure = 101325.0f;
            /** @brief Tropospheric temperature lapse rate, K/m (ISA). */
            float lapse_rate = 0.0065f;
            /** @brief Altitude the lapse rate gives way to an isothermal layer, m (ISA tropopause). */
            float tropopause_altitude = 11000.0f;
            /** @brief Reference relative humidity at the surface, [0, 1]. */
            float surface_humidity = 0.70f;
            /**
             * @brief e-folding height of the base-state vapour profile, m.
             *
             * Water vapour falls off far faster than air itself does — most of the column's
             * moisture sits in the lowest couple of kilometres, which is exactly why the
             * lifting condensation level lands where it does.
             */
            float humidity_scale_height = 2500.0f;

            // ---- Dynamics ------------------------------------------------------------

            /**
             * @brief Courant number the automatic time step targets.
             *
             * §1.4 is the reason this exists as a target rather than a fixed step: the
             * shipped scheme ran at Courant ≈ 0.02, which is bilinear semi-Lagrangian
             * advection's *maximally diffusive* regime — each tick blended ~98 % of a cell
             * with ~2 % of its neighbour, 240 times per simulated hour, so any front smeared
             * into gradient-free mush within hours. At Courant ≈ 1 a departure point lands
             * near a neighbouring cell centre and the interpolation barely averages at all.
             */
            float courant_target = 0.9f;
            /** @brief Never step longer than this, seconds of game time. */
            float max_step_seconds = 6.0f;
            /** @brief Never step shorter than this, seconds of game time. */
            float min_step_seconds = 0.25f;
            /**
             * @brief Steps a single frame may take before the rest is dropped.
             *
             * The nest is stepped in *game* time, so a large time scale (or a hitch) can ask
             * for many steps at once. Past this the surplus is discarded rather than
             * catching up, because a frame that stalls to simulate an hour of weather is a
             * worse outcome than weather that is briefly behind — and nothing downstream
             * depends on the nest's clock being exact (§3.4).
             */
            std::uint32_t max_steps_per_frame = 4;
            /** @brief Subgrid eddy viscosity/diffusivity, m²/s. Horizontal only. */
            float eddy_viscosity = 40.0f;
            /**
             * @brief Ceiling on how deep the mixed layer may grow, m.
             *
             * The nest resolves 2 km horizontally, and the turbulence that carries heat and
             * moisture up out of the surface layer is two orders below that — so it is
             * parameterized, exactly as it is in every operational model (§2.1's YSU, MYNN).
             * Without it the fluxes accumulate in the lowest level and go nowhere: a 54 m slab
             * reaching +17 K while the level above holds its initial value to the last bit, and
             * a horizontally uniform slab cannot rise, so the pressure projection removes the
             * buoyancy and nothing ever convects. This is what makes the difference between a
             * boundary layer and a hotplate.
             *
             * A *cap*, not the depth: the depth itself is diagnosed per column by the parcel
             * method, so at sunrise the layer is a few tens of metres and grows through the
             * morning as the surface parcel outgrows more of the stratification above it. That
             * growth is the diurnal cycle's mechanism — a shallow layer early keeps the surface
             * moisture concentrated, and deepening carries it to its condensation level — and a
             * fixed depth would dilute it into ten times the air from the first step. What this
             * number stands for is the capping inversion the free troposphere puts on a
             * fair-weather layer.
             */
            float boundary_layer_depth_m = 2500.0f;
            /**
             * @brief Turbulent velocity scale of the mixed layer, m/s.
             *
             * The whole vertical diffusivity profile, in one number. Troen & Mahrt (1986) — the
             * form YSU and its lineage use — sets `K(z) = κ·w_s·z·(1 − z/h)²`, so this is `w_s`:
             * the convective velocity scale, how fast the layer's eddies actually turn over.
             * 1–2 m/s is a fair-weather convective afternoon and the profile it implies peaks at
             * 150–300 m²/s a third of the way up, which is the observed range.
             *
             * **A velocity and not a peak diffusivity, for a measured reason.** A profile
             * normalised to its own peak — the parabola `4·K_peak·f(1 − f)` this replaced — goes
             * as `K_peak·(z/h)` near the ground, so its near-surface mixing *weakens as the layer
             * deepens*. The lowest face is the one that has to carry the entire surface flux out
             * of a 54 m level, so that feedback is exactly backwards: with a 2 500 m layer it left
             * 12 m²/s there, the surface level sat 9 K above the one 80 m above it, the layer
             * never homogenised, and its top reached only 57 % relative humidity after eight
             * hours of heating — no cloud, at any Bowen ratio. Troen–Mahrt's slope does not
             * depend on `h` at all.
             */
            float boundary_layer_velocity_scale = 1.5f;
            /**
             * @brief Depth of the Rayleigh damping layer below the domain top, m.
             *
             * Gravity waves radiated by convection would otherwise reflect off the rigid
             * lid and interfere with the storms that launched them. Standard practice in
             * every cloud model; the depth is the usual quarter of the domain.
             */
            float sponge_depth = 5000.0f;
            /** @brief Peak Rayleigh damping rate at the domain top, 1/s. */
            float sponge_rate = 0.01f;
            /** @brief Width of the Davies lateral relaxation zone, cells. */
            std::uint32_t boundary_zone_cells = 8;
            /**
             * @brief Peak relaxation rate at the outermost boundary cell, 1/s.
             *
             * Davies (1976) nesting: the fields are nudged toward the parent solution with a
             * weight ramping from this at the domain edge to zero at the inner edge of the
             * zone. A ramp rather than a hard injection is what stops the boundary from
             * reflecting everything that reaches it.
             */
            float boundary_relaxation = 0.02f;
            /**
             * @brief Red-black sweeps of the pressure relaxation per step.
             *
             * Each sweep solves the *vertical* exactly (see `atmosphere_pressure.comp`), so
             * these only iterate the horizontal coupling, which the grid's anisotropy makes the
             * weak direction. Twelve leaves a small residual divergence — a slight mass
             * imbalance rather than a wrong answer — and is the knob to raise if it ever proves
             * visible, before reaching for the multigrid the shader's header names.
             */
            std::uint32_t pressure_iterations = 12;
            /**
             * @brief Fractional variation in surface heating across a cell, [0, 1].
             *
             * Convection needs something to break the horizontal symmetry of a uniformly
             * heated surface, or the whole boundary layer rises as one slab and no cell ever
             * forms. Real air has turbulence and real ground has patchy albedo, moisture and
             * cover doing this; a cloud model without a resolved surface layer seeds it
             * explicitly, and every one of them does. 0.4 is a mixed landscape.
             *
             * **A modulation of the surface flux, not an additive kick on θ**, and the
             * difference is not cosmetic. An additive perturbation redrawn every step is a
             * random walk with no bound: it reached −8.76 K in the coldest columns after four
             * hours, saturating them from below, so a few percent of the domain sat in permanent
             * ground fog that the sky then reported as cloud. Scaling the flux is bounded by
             * construction — a heated surface is heated more in one cell and less in the next,
             * never refrigerated — and it switches off at night with the flux it scales, which
             * is exactly when a stable nocturnal layer should not be stirred.
             */
            float thermal_seed_amplitude = 0.4f;
            /**
             * @brief Updraft speed a band reads as fully convective, m/s.
             *
             * Purely a *reporting* scale: it converts the nest's vertical velocity into the
             * `convective_fraction` the genus classifier and the editor readout already speak
             * in. 2 m/s is a solidly convective updraft — a fair-weather cumulus runs a metre
             * or two per second, a thunderstorm ten times that.
             */
            float convective_velocity_scale = 2.0f;

            // ---- Microphysics: Kessler (1969) warm rain ------------------------------

            /**
             * @brief Relative humidity a cell begins to hold cloud at, [0, 1].
             *
             * The closure that makes a grid-mean model able to draw a cumulus at all, and it is
             * a *resolution* correction rather than a tuning knob. A nest cell is 2 km across,
             * so its humidity is a cell mean; a fair-weather cumulus is a 200 m–1 km thermal
             * overshooting the mixed-layer top, saturated inside while the cell around it is
             * not. Condensing only when the mean saturates therefore cannot produce one: it
             * produces nothing at all until the entire 4 km² column saturates, and then it
             * produces fog. That is measured, not supposed — before this existed, every run
             * that made condensate made it at 19 m, and the mixed-layer top topped out at
             * 78–95 % relative humidity and never crossed.
             *
             * So the humidity inside a cell is treated as a distribution about its mean rather
             * than a single value, cloud forms in the part of that distribution which is
             * saturated, and this is where the distribution's dry edge sits: at a cell mean of
             * `critical × q_s` the wettest air in the cell is exactly at saturation and the
             * first cloud appears. 0.8 is the standard value for a boundary-layer scheme at
             * this spacing (Sundqvist 1978; Smith 1990) and the reason every operational model
             * near 2 km carries one of these on top of its boundary-layer scheme.
             *
             * At 1.0 the distribution has no width and the scheme collapses *exactly* onto the
             * all-or-nothing saturation adjustment it generalises, which is what makes this a
             * strict generalisation rather than a second, competing condensation path.
             */
            float cloud_critical_humidity = 0.80f;

            /** @brief Autoconversion rate coefficient k₁, 1/s. */
            float autoconversion_rate = 1.0e-3f;
            /** @brief Cloud water below which autoconversion does not start, kg/kg. */
            float autoconversion_threshold = 1.0e-3f;
            /** @brief Accretion rate coefficient k₂, 1/s. */
            float accretion_rate = 2.2f;
            /** @brief Accretion's rain-mixing-ratio exponent (Kessler's 0.875). */
            float accretion_exponent = 0.875f;
            /** @brief Rain evaporation rate coefficient below cloud base, 1/s. */
            float rain_evaporation_rate = 2.0e-2f;
            /** @brief Terminal fall speed coefficient in V = a·(ρ q_r)^b, SI. */
            float fall_speed_coefficient = 36.34f;
            /** @brief Terminal fall speed exponent b. */
            float fall_speed_exponent = 0.1364f;
            /**
             * @brief Effective cloud droplet radius, m.
             *
             * §7.1's optical extinction divides by this: `σ_ext = 3 ρ q_c / (2 ρ_w r_eff)`.
             * ~6 µm is maritime, ~10 µm continental — the single number that decides whether
             * a given amount of condensate reads as a crisp white cumulus or a thin haze.
             */
            float droplet_effective_radius = 8.0e-6f;
            /**
             * @brief Liquid water content a cell must hold to read as fully overcast, kg/m³.
             *
             * The one knob that separates *how much water the model makes* from *how much water
             * reads as a solid sky*, and it exists because those are two different questions and
             * conflating them makes a wrong answer to either look like a wrong answer to both.
             *
             * A nest cell is 2 km across, so its condensate is a **cell mean**: 1 g/m³ averaged
             * over four square kilometres is a deep solid deck, while a fair-weather cumulus
             * field — a sky that is mostly blue — averages a small fraction of that. Raising
             * this makes a given amount of condensate read as thinner and more scattered without
             * touching the physics that produced it; lowering it does the reverse.
             */
            float coverage_reference_lwc = 0.0015f;

            // ---- Surface forcing -----------------------------------------------------
            //
            // Prescribed in this phase. A real surface energy balance — insolation through
            // `Astro::Ephemeris`, a slab heat capacity, land/sea partitioning — is the next
            // phase's `ISurfaceModel`, and it is what turns these two numbers into the
            // diurnal cycle. Prescribed values are stated as such rather than dressed up:
            // they are what a fair-weather convective afternoon actually delivers.

            /**
             * @brief Surface sensible heat flux at solar noon with the sun overhead, W/m².
             *
             * A *peak*, not a constant. The nest scales it by the sine of the real sun's
             * elevation, which is what makes the diurnal cycle and the seasons the same
             * mechanism: the declination that puts the sun higher in July than in January is
             * already in the ephemeris driving the rendered sun, so a summer afternoon delivers
             * more heat to the ground than a winter one without anything modelling "summer".
             */
            float surface_sensible_flux = 140.0f;
            /** @brief Surface latent (moisture) flux at solar noon with the sun overhead, W/m². */
            float surface_latent_flux = 100.0f;
            /**
             * @brief Net radiative cooling of the surface at night, W/m².
             *
             * Positive; subtracted while the sun is down. This is what *ends* convection in the
             * evening rather than merely starving it: the ground cools, the lowest level cools
             * with it, the boundary layer stabilises and the cumulus field decays. Without a
             * negative term the sky would simply stop growing new cloud and keep what it had.
             */
            float surface_night_flux = 35.0f;

            /** @brief Whether the nest runs at all; off falls back to the authored/classified sky. */
            bool enabled = true;
        };

        /**
         * @brief The nest's discretization at one quality tier.
         *
         * The horizontal *domain* is the same at every tier and only its resolution changes,
         * so raising the tier resolves the same weather more finely instead of simulating a
         * different amount of world — which is what keeps a scene's look consistent across
         * machines. 2 km at High is not an arbitrary step on that ladder: it is the spacing
         * at which convection stops being parameterized and starts being *resolved* (§2.2),
         * and it is the reason the acceptance bar for this phase is a cumulus that grows on
         * its own rather than one that is placed.
         */
        struct AtmosphereNestSize
        {
            std::uint32_t cells_x = 192;  /**< Horizontal cells, X axis. */
            std::uint32_t cells_z = 192;  /**< Horizontal cells, Z axis. */
            std::uint32_t levels = 48;    /**< Stretched vertical levels. */
            float spacing_m = 2000.0f;    /**< Horizontal cell size, metres. */
            float top_m = 18000.0f;       /**< Domain top above the surface, metres. */
        };

        /**
         * @brief The vertical stretch exponent, shared by the C++ and GLSL grid.
         *
         * `z(k) = top · ((k + ½)/N)^s`. At s = 1.5 with 48 levels over 18 km the spacing runs
         * from ~54 m at the surface to ~560 m aloft — fine where the boundary layer and cloud
         * base live, coarse where only the anvil does. A uniform grid would either waste most
         * of its levels above the weather or resolve cloud base at half a kilometre.
         */
        constexpr float ATMOSPHERE_VERTICAL_STRETCH = 1.5f;

        /**
         * @brief Altitude of level @p level's centre, metres above the surface.
         * @param level  Level index, 0 at the ground.
         * @param levels Total level count.
         * @param top_m  Domain top, metres.
         */
        inline float atmosphere_level_altitude(std::uint32_t level, std::uint32_t levels,
                                               float top_m)
        {
            const float fraction = (static_cast<float>(level) + 0.5f) /
                                   static_cast<float>(levels > 0 ? levels : 1);
            return top_m * std::pow(fraction, ATMOSPHERE_VERTICAL_STRETCH);
        }

        /**
         * @brief Thickness of level @p level, metres — the spacing the vertical terms divide by.
         * @param level  Level index.
         * @param levels Total level count.
         * @param top_m  Domain top, metres.
         */
        inline float atmosphere_level_thickness(std::uint32_t level, std::uint32_t levels,
                                                float top_m)
        {
            const float count = static_cast<float>(levels > 0 ? levels : 1);
            const float lower = static_cast<float>(level) / count;
            const float upper = static_cast<float>(level + 1) / count;
            return top_m * (std::pow(upper, ATMOSPHERE_VERTICAL_STRETCH) -
                            std::pow(lower, ATMOSPHERE_VERTICAL_STRETCH));
        }

        /** @brief Base-state temperature at @p altitude_m, K (ISA troposphere then isothermal). */
        inline float atmosphere_base_temperature(const AtmosphereParameters& p, float altitude_m)
        {
            const float capped = altitude_m < p.tropopause_altitude ? altitude_m
                                                                    : p.tropopause_altitude;
            return p.surface_temperature - p.lapse_rate * capped;
        }

        /** @brief Base-state pressure at @p altitude_m, Pa (hydrostatic against the profile above). */
        inline float atmosphere_base_pressure(const AtmosphereParameters& p, float altitude_m)
        {
            const float exponent = p.gravity / (p.gas_constant_dry * p.lapse_rate);
            const float capped = altitude_m < p.tropopause_altitude ? altitude_m
                                                                    : p.tropopause_altitude;
            const float ratio = 1.0f - p.lapse_rate * capped / p.surface_temperature;
            const float troposphere = p.surface_pressure * std::pow(ratio, exponent);
            if (altitude_m <= p.tropopause_altitude)
                return troposphere;
            // Isothermal above the tropopause: pressure decays exponentially with the scale
            // height of the (now constant) temperature there.
            const float tropopause_temperature = atmosphere_base_temperature(p, altitude_m);
            const float scale_height = p.gas_constant_dry * tropopause_temperature / p.gravity;
            return troposphere * std::exp(-(altitude_m - p.tropopause_altitude) / scale_height);
        }

        /** @brief The Exner function `Π = (p/p₀)^(R/c_p)` at @p altitude_m. */
        inline float atmosphere_exner(const AtmosphereParameters& p, float altitude_m)
        {
            return std::pow(atmosphere_base_pressure(p, altitude_m) / p.reference_pressure,
                            p.gas_constant_dry / p.specific_heat_pressure);
        }

        /** @brief Base-state potential temperature at @p altitude_m, K. */
        inline float atmosphere_base_theta(const AtmosphereParameters& p, float altitude_m)
        {
            return atmosphere_base_temperature(p, altitude_m) / atmosphere_exner(p, altitude_m);
        }

        /** @brief Base-state density at @p altitude_m, kg/m³. */
        inline float atmosphere_base_density(const AtmosphereParameters& p, float altitude_m)
        {
            return atmosphere_base_pressure(p, altitude_m) /
                   (p.gas_constant_dry * atmosphere_base_temperature(p, altitude_m));
        }

        /**
         * @brief Saturation vapour pressure over liquid water at @p temperature_k, Pa.
         *
         * Magnus/Teten's form, the one §6 names. This is the relation the shipped system
         * replaced with `if (humidity > 0.85)` (§1.3) — and the reason that mattered is that
         * saturation depends on temperature *exponentially*, so a threshold on relative
         * humidity cannot place a cloud base at the altitude a rising parcel actually reaches
         * saturation.
         */
        inline float atmosphere_saturation_pressure(float temperature_k)
        {
            return 611.2f * std::exp(17.67f * (temperature_k - 273.15f) /
                                     (temperature_k - 29.65f));
        }

        /** @brief Saturation mixing ratio at @p temperature_k and @p pressure_pa, kg/kg. */
        inline float atmosphere_saturation_mixing_ratio(float temperature_k, float pressure_pa)
        {
            const float e_s = atmosphere_saturation_pressure(temperature_k);
            const float denominator = pressure_pa - 0.378f * e_s;
            return denominator > 1.0f ? 0.622f * e_s / denominator : 1.0f;
        }

        /**
         * @brief Base-state vapour mixing ratio at @p altitude_m, kg/kg.
         *
         * The surface relative humidity carried aloft with its own scale height, capped at
         * saturation so the initial state is never supersaturated anywhere.
         */
        inline float atmosphere_base_vapour(const AtmosphereParameters& p, float altitude_m)
        {
            const float temperature = atmosphere_base_temperature(p, altitude_m);
            const float pressure = atmosphere_base_pressure(p, altitude_m);
            const float saturation = atmosphere_saturation_mixing_ratio(temperature, pressure);
            const float humidity =
                p.surface_humidity * std::exp(-altitude_m / (p.humidity_scale_height > 1.0f
                                                                 ? p.humidity_scale_height
                                                                 : 1.0f));
            const float vapour = humidity * saturation;
            return vapour < saturation ? vapour : saturation;
        }

        /** @brief What @ref atmosphere_cloud_partition resolves a cell's water into. */
        struct AtmosphereCloudPartition
        {
            /** @brief Fraction of the cell that is cloud, [0, 1]. */
            float fraction = 0.0f;
            /** @brief Cell-mean condensate the fraction implies, kg/kg. */
            float condensate = 0.0f;
        };

        /**
         * @brief Split a cell's total water into cloud fraction and cell-mean condensate.
         *
         * The subgrid closure, as a function, mirrored from `atmosphere_microphysics.comp` for
         * the same reason every other relation in this file is: GLSL cannot include a C++
         * header, so the formula is stated twice and the *numbers* only once. What that buys
         * here is a closure with a test — the identities below are arithmetic, not opinion, and
         * `test_atmosphere_nest.cpp` pins them.
         *
         * The distribution is a top-hat (Sommeria & Deardorff 1977; Mellor 1977) of half-width
         * `(1 - critical) q_s` about the cell mean, which is the simplest member of the family
         * Smith (1990) generalises with a triangular one. Writing `Q` for how far the mean sits
         * across that width,
         *
         *   - `Q ≤ -1`: the whole cell is subsaturated. No cloud, no condensate.
         *   - `-1 < Q < 1`: partly cloudy. Fraction `(1 + Q)/2`, condensate the mean of the
         *     saturated tail — which is quadratic in `1 + Q`, so the first cloud in a cell is
         *     thin and grows faster than linearly as the cell moistens.
         *   - `Q ≥ 1`: every part of the cell is saturated. Fraction 1, and the condensate is
         *     the whole excess — identically what an all-or-nothing adjustment would give.
         *
         * @param total_water Vapour plus cloud, kg/kg. Rain is a separate, precipitating
         *                    species and is not part of the distribution.
         * @param saturation  Saturation mixing ratio at the *liquid-water* temperature, kg/kg.
         * @param efficiency  The fraction of a nominal excess that actually condenses once its
         *                    own latent heating has raised `q_s`; see
         *                    @ref atmosphere_condensation_efficiency.
         * @param critical    @ref AtmosphereParameters::cloud_critical_humidity.
         */
        inline AtmosphereCloudPartition atmosphere_cloud_partition(float total_water,
                                                                   float saturation,
                                                                   float efficiency,
                                                                   float critical)
        {
            AtmosphereCloudPartition partition;
            const float mean = efficiency * (total_water - saturation);
            const float clamped = critical < 0.0f ? 0.0f : (critical > 1.0f ? 1.0f : critical);
            const float width = efficiency * (1.0f - clamped) * saturation;
            if (!(width > 0.0f))
            {
                // The degenerate limit is the scheme this generalises, and it is reached by the
                // formulas below only in the limit — so it is written out rather than divided by
                // a zero width and rescued afterwards.
                partition.fraction = mean > 0.0f ? 1.0f : 0.0f;
                partition.condensate = mean > 0.0f ? mean : 0.0f;
                return partition;
            }
            const float across = mean / width;
            if (across <= -1.0f)
                return partition;
            if (across >= 1.0f)
            {
                partition.fraction = 1.0f;
                partition.condensate = mean;
                return partition;
            }
            partition.fraction = 0.5f * (1.0f + across);
            partition.condensate = width * (1.0f + across) * (1.0f + across) * 0.25f;
            return partition;
        }

        /**
         * @brief The fraction of a nominal excess that survives its own latent heating, [0, 1].
         *
         * Condensing water warms the parcel, which raises `q_s`, which leaves less excess to
         * condense — so a saturation adjustment is a fixed point rather than a subtraction. One
         * Newton step on `f(δ) = (q_v - δ) - q_s(T + Lδ/c_p)` is exact enough at these time
         * steps, and this is that step's denominator: `1 / (1 + (L/c_p)·dq_s/dT)`. Roughly ½ at
         * 290 K, and falling with temperature, which is why a warm cloud takes twice the water
         * to reach a given condensate that a cold one does.
         *
         * @param p             The thermodynamic constants.
         * @param saturation    Saturation mixing ratio at @p temperature_k, kg/kg.
         * @param temperature_k The temperature the derivative is taken at, K.
         */
        inline float atmosphere_condensation_efficiency(const AtmosphereParameters& p,
                                                        float saturation, float temperature_k)
        {
            const float safe = temperature_k > 1.0f ? temperature_k : 1.0f;
            const float slope = saturation * p.latent_heat_vaporization /
                                (p.gas_constant_vapour * safe * safe);
            return 1.0f / (1.0f + p.latent_heat_vaporization / p.specific_heat_pressure * slope);
        }

        /**
         * @brief One coarse cell of the parent solution the nest's boundary relaxes toward.
         *
         * The T1 side of Davies nesting. Until the global quasi-geostrophic core exists this
         * is sampled from the surviving analytic synoptic layer — a named interim, not a
         * pretence: the wind is real geostrophic flow around real pressure systems, and the
         * two anomalies are what a front's warm and cold sectors do to a column.
         */
        struct AtmosphereForcingSample
        {
            float wind_east_mps = 0.0f;   /**< Parent eastward wind, m/s. */
            float wind_north_mps = 0.0f;  /**< Parent northward wind, m/s. */
            float theta_anomaly_k = 0.0f; /**< Departure from the base-state potential temperature, K. */
            float humidity_anomaly = 0.0f;/**< Departure from the base-state relative humidity, fraction. */
        };

        /**
         * @brief A borrowed view of the parent solution over the nest's footprint.
         *
         * Borrowed for the same reason `WeatherField` is: `Environment` is copied per frame
         * and this changes on the nest's own multi-second cadence. Addressed exactly as
         * `WeatherField` is — a scale and offset from scene-absolute world XZ — so the two
         * cannot disagree about where the weather is.
         */
        struct AtmosphereForcing
        {
            const AtmosphereForcingSample* samples = nullptr; /**< `cells_x * cells_z`; borrowed. */
            std::int32_t cells_x = 0;
            std::int32_t cells_z = 0;
            std::uint32_t revision = 0; /**< Bumped when the contents change; 0 = never published. */

            float uv_scale_x = 0.0f;  /**< Scene-absolute world X metres -> U. */
            float uv_scale_z = 0.0f;  /**< Scene-absolute world Z metres -> V. */
            float uv_offset_x = 0.0f; /**< U at world X = 0. */
            float uv_offset_z = 0.0f; /**< V at world Z = 0. */

            /**
             * @brief Total game seconds the simulation has run, monotonic.
             *
             * The nest is stepped in *game* time, not frames: its stability limit is a property
             * of the atmosphere, and at 1× time scale a step lands every couple of seconds of
             * wall clock. This is the only channel through which the simulation's clock reaches
             * the render tier, and it is why a paused or time-scaled scene behaves correctly
             * without the renderer knowing what either of those is.
             *
             * An *absolute* clock rather than a per-frame delta, deliberately: the editor runs
             * three scene views over one `Environment`, so whichever of them reaches the nest
             * first should do the work and the other two should find nothing to do. The nest
             * takes the difference against what it last saw, which makes stepping idempotent
             * instead of making the caller responsible for calling exactly once.
             */
            double total_seconds = 0.0;

            /** @brief Coriolis parameter `f = 2Ω sin φ` at the nest centre, 1/s. */
            float coriolis = 0.0f;

            /**
             * @brief Sine of the sun's elevation at the nest centre; negative below the horizon.
             *
             * The single number that makes the surface forcing diurnal *and* seasonal. Taken
             * from the same sun the sky is rendered with — `docs/slop/atmosphere_system.md` §1.6
             * records the shipped system reimplementing its own solar-position model, so that
             * "the sun that heats the ground and the sun that is rendered are two different
             * suns"; this is the same sun, read off the environment's own direction vector.
             */
            float solar_elevation_sine = 0.0f;

            /**
             * @brief Where the nest should be centred, scene-absolute metres.
             *
             * The *simulation's observer*, deliberately, and not any view's camera: the editor
             * runs three scene views and there is one atmosphere, so a nest that followed a
             * camera would have to pick one and would drift whenever the user looked through a
             * different window. §6 says "recentres on the player", and this is that. Doubles
             * because the scene coordinate is planet-scale and the snap has to land on the same
             * absolute lattice every time.
             */
            double observer_x = 0.0;
            double observer_z = 0.0;

            /** @brief Whether this forcing carries usable data this frame. */
            bool valid() const noexcept
            {
                return samples != nullptr && cells_x > 0 && cells_z > 0;
            }
        };

        /** @brief Upper bound on a forcing field's cells per axis; sizes the GPU image once. */
        constexpr int ATMOSPHERE_FORCING_MAX_CELLS = 64;

        /** @brief Cells per axis of the query mirror; 32² columns is 64 KB of readback. */
        constexpr int ATMOSPHERE_MIRROR_CELLS = 32;

        /**
         * @brief One coarse column of the mirror, as the GPU writes it.
         *
         * Laid out to match `atmosphere_readback.comp`'s `MirrorColumn` exactly — sixteen
         * floats, no padding — because the two are the same bytes seen from either side of a
         * buffer copy. Deliberately shaped like the `WeatherColumn` the existing gameplay
         * bridge already speaks in rather than like §9.1's `AtmosphereProfile`: the profile is
         * phase E's contract and has consumers written for it, and inventing it early would
         * mean two data planes to keep in step for a phase that needs neither.
         */
        struct AtmosphereMirrorColumn
        {
            /** @brief Per WMO étage: coverage, density scale, convective fraction, temperature offset (°C). */
            float bands[3][4]{};
            /** @brief Precipitation (mm/h), eastward wind (m/s), northward wind (m/s), cloud base (m). */
            float surface[4]{};
            /**
             * @brief Cloud top (m), surface relative humidity (0-1), peak |w| (m/s), LCL (m).
             *
             * Base and top together are what let the march shell be stretched across the span
             * condensate actually occupies, instead of across the nest's whole 18 km domain —
             * which would spend two thirds of the baked field's vertical resolution on empty
             * stratosphere.
             *
             * The other three are diagnostics rather than anything the renderer consumes, and
             * they exist because the fields above can only report *that* a column is clear.
             * The lifting condensation level is the decisive one: it is the altitude a surface
             * parcel must be raised to before it saturates, so a nest whose LCL sits above what
             * its convection can reach will stay clear however long it is heated — and because
             * warming lowers relative humidity, more heating pushes the LCL further away rather
             * than closer. Reported as 0 when the parcel never saturates inside the domain.
             */
            float extent[4]{};
        };

        /** @brief Levels the profile readback carries; the Ultra tier's count is the ceiling. */
        constexpr int ATMOSPHERE_PROFILE_MAX_LEVELS = 64;

        /**
         * @brief One level of the observer column's vertical state, as the GPU writes it.
         *
         * The mirror's columns (@ref AtmosphereMirrorColumn) answer "what is the weather here",
         * which is what gameplay asks and all it asks. They cannot answer *why* a column holds
         * what it holds, because every one of their fields is already a vertical reduction —
         * and a log of exact zeros is precisely the case where the reduction has thrown away
         * the only information worth having. A column that will not condense has its explanation
         * distributed over height: where the vapour is, where the parcel stops rising, at what
         * level the two would have met.
         *
         * So this is the same asynchronous readback, unreduced, for one column: the nest's
         * centre, which is the cell the observer stands on. Sixteen floats a level over at most
         * 64 levels is 4 KB — the profile is free beside the 64 KB of columns it travels with,
         * and it is written by the same dispatch from samples that were already being fetched.
         *
         * It is deliberately *not* §9.1's `AtmosphereProfile`. That contract is phase E's, has
         * consumers written for it, and carries derived quantities (dewpoint, turbulence, icing)
         * this does not. This is the state vector the nest actually integrates, plus the two
         * base-state quantities needed to interpret it — which is what a diagnostic wants and
         * what a gameplay API would be wrong to expose.
         */
        struct AtmosphereProfileLevel
        {
            /** @brief Level centre, metres above the surface. */
            float altitude_m = 0.0f;
            /** @brief Base-state pressure at this level, Pa. */
            float pressure_pa = 0.0f;
            /** @brief Absolute temperature, K — base state plus the perturbation, through Exner. */
            float temperature_k = 0.0f;
            /** @brief The prognostic potential temperature perturbation, K. */
            float theta_perturbation_k = 0.0f;
            /** @brief Water vapour mixing ratio, kg/kg. Total, not a perturbation. */
            float vapour = 0.0f;
            /** @brief Base-state vapour mixing ratio at this level, kg/kg — what it started at. */
            float base_vapour = 0.0f;
            /** @brief Saturation mixing ratio at this level's own temperature, kg/kg. */
            float saturation = 0.0f;
            /** @brief Cloud water mixing ratio, kg/kg. A **cell mean**, not an in-cloud value. */
            float cloud_water = 0.0f;
            /**
             * @brief Fraction of the cell that is cloud, [0, 1].
             *
             * Carried beside the condensate because the two answer different questions and the
             * condensate alone answers neither: 0.1 g/kg spread thinly over a whole 2 km cell is
             * a haze, and the same 0.1 g/kg concentrated in a third of it is a cumulus field with
             * blue between the clouds. Dividing one by the other is the in-cloud water content,
             * which is what the bake's amplitude is derived from.
             */
            float cloud_fraction = 0.0f;
            /** @brief Rain mixing ratio, kg/kg. */
            float rain = 0.0f;
            /** @brief Eastward wind on this cell's face, m/s. */
            float wind_east_mps = 0.0f;
            /** @brief Northward wind on this cell's face, m/s. */
            float wind_north_mps = 0.0f;
            /** @brief Vertical wind on the face below this level, m/s — the one that must be nonzero. */
            float wind_up_mps = 0.0f;
            /** @brief Optical extinction, 1/m — what the cloudscape bake reads. */
            float extinction = 0.0f;
            /**
             * @brief Buoyancy this level's face would feel from the state now held, m/s².
             *
             * Re-evaluated here rather than captured from `atmosphere_forces.comp`, which does
             * not store it: microphysics has since added latent heating to `theta`, so this is
             * the buoyancy of the *end* of the step and not the number the step's own velocity
             * update used. The distinction matters for arithmetic and not for the question this
             * answers, which is whether there is any buoyancy in the column at all.
             */
            float buoyancy = 0.0f;
            /**
             * @brief Mass divergence of the provisional velocity, 1/s.
             *
             * What the pressure solve was asked to remove, not what it left behind: the
             * divergence volume is written once per step before the sweeps and is not
             * recomputed after them. A large value here with a small updraft is the signature
             * of a projection working hard against a term that should never have reached it.
             */
            float divergence = 0.0f;
        };

        /**
         * @brief The asynchronous readback of the nest, as gameplay reads it.
         *
         * `docs/slop/atmosphere_system.md` §3.2's second face: the atmosphere's state lives on
         * the GPU and is written by exactly one path, and *nothing reads it synchronously*.
         * This is a snapshot copied back after a step completed, two or three frames old, which
         * for a medium whose own time scale is minutes is not observable — and that staleness
         * is what lets the atmosphere leave the determinism domain (§3.4) without anything
         * downstream having to care.
         *
         * Addressed exactly as `WeatherField` is, by a scale and offset from scene-absolute
         * world XZ, so the two describe the same world in the same coordinates.
         */
        struct AtmosphereMirror
        {
            const AtmosphereMirrorColumn* columns = nullptr; /**< `cells * cells`; borrowed. */
            std::int32_t cells = 0;
            std::uint32_t revision = 0; /**< Bumped per completed readback; 0 = nothing yet. */

            /**
             * @brief The observer column's unreduced vertical state; `profile_levels` entries.
             *
             * Borrowed, from the same snapshot as @ref columns and therefore of the same age.
             * Null until the first readback completes. Diagnostic (@ref AtmosphereProfileLevel):
             * nothing the renderer draws depends on it.
             */
            const AtmosphereProfileLevel* profile = nullptr;
            std::int32_t profile_levels = 0;

            float uv_scale_x = 0.0f;  /**< Scene-absolute world X metres -> U. */
            float uv_scale_z = 0.0f;  /**< Scene-absolute world Z metres -> V. */
            float uv_offset_x = 0.0f; /**< U at world X = 0. */
            float uv_offset_z = 0.0f; /**< V at world Z = 0. */

            /** @brief Game seconds the nest had simulated when this snapshot was taken. */
            double simulated_seconds = 0.0;

            /** @brief Whether this mirror carries usable data. */
            bool valid() const noexcept { return columns != nullptr && cells > 0; }
        };

        /**
         * @brief The read-only seam the simulation reaches the nest's mirror through.
         *
         * Data flows simulation → renderer everywhere else in the engine; the mirror is the one
         * thing that flows back, and this is deliberately the narrowest possible shape for it:
         * one const accessor, no lifetime transfer, no way to ask the renderer to do anything.
         * The host binds an implementation into the simulation once at startup rather than
         * ferrying a copy every frame — a host that forgets gets the honest base-state fallback
         * (`AtmosphereParameters`' own profile, computable on the CPU) instead of a sky that
         * silently stops evolving.
         */
        class IAtmosphereMirror
        {
            public:
                virtual ~IAtmosphereMirror() = default;

                /**
                 * @brief The most recently completed readback.
                 * @return A borrowed view; invalid until the first step has been read back.
                 */
                virtual AtmosphereMirror atmosphere_mirror() const noexcept = 0;
        };
    } // namespace Render
} // namespace SushiEngine
