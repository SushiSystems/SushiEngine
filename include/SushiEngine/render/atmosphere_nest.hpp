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
             * these only iterate the horizontal coupling — and the grid's anisotropy is
             * deliberate and extreme: 2 km horizontally against 54–560 m vertically, so the
             * vertical coupling outweighs the horizontal by 12× at the domain top and 1 370× at
             * the ground. What is left for these sweeps to do is therefore a small, strongly
             * diagonally dominant correction, and it converges almost immediately.
             *
             * **Four, because two was measured to be the convergence point and this is the
             * margin.** The previous value was twelve, chosen on the reasoning that a solver
             * with no coarse-grid correction leaves smooth horizontal error behind — true of an
             * *isotropic* Poisson problem, and beside the point here, where the horizontal is
             * the weak direction and the smooth mode it leaves is proportionally tiny. Measured
             * end-to-end over six simulated hours: at 2, 4, 8, 12 and 20 sweeps the surface
             * humidity, lifting condensation level, cloud base, column water and sky coverage
             * are *identical to every printed figure*, and the peak divergence — the quantity
             * the solve exists to control — agrees to seven significant figures. At **one**
             * sweep it does not, which is what distinguishes "converged" from "the solve was
             * never doing anything". Repeated with a 25 m/s front and a +4 K parent anomaly,
             * the case whose horizontal gradients this argument is weakest for: divergence
             * still agrees to six figures at 4 sweeps.
             *
             * Costed on a GTX 1060 at the shipped tier, the sweeps are the whole difference:
             * the pressure stage runs 1.3 / 2.7 / 8.0 / 15.3 ms at 2 / 4 / 12 / 20 sweeps, and
             * the step 6.3 / 7.9 / 13.2 / 21.2 ms. Twelve sweeps spent 5.3 ms a step buying a
             * change in the sixth significant figure.
             */
            std::uint32_t pressure_iterations = 4;
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
             * @brief Length the surface heterogeneity is correlated over, metres.
             *
             * The seed used to be white in space: every cell drew independently, so the field
             * had no scale at all and the only structure convection could organise around was
             * one cell wide — which is the scale a grid-mean model is least entitled to believe.
             * Worse, it made the *tier* the correlation length: a Low cell is 4 km and an Ultra
             * cell 1.5 km, so the same scene was seeded with patches of different physical size
             * depending on a graphics setting.
             *
             * Stated in metres, that cannot happen — every tier samples one field, and a coarse
             * tier simply resolves less of it, which is what a coarse grid does to real terrain.
             * 6 km is a few times the depth of a well-developed boundary layer, the scale over
             * which land cover, soil moisture and albedo actually vary, and the spacing a
             * cumulus field's cells self-organise onto. Measured against 24 km, which recovers
             * two thirds of the effect, and against a cell-scale field, which recovers a fifth.
             *
             * Named consequence: the tier spread narrows but does not vanish. Medium, High and
             * Ultra now agree on domain structure to 1.10× and on coverage to 1.06×; **Low does
             * not**, because a 4 km cell samples a 6 km patch with a cell and a half and cannot
             * represent it however the field is defined. That is a resolution limit rather than
             * an inconsistency, and it is the honest form of one.
             *
             * @see thermal_seed_period_s, the same argument on the time axis.
             */
            float thermal_seed_length_m = 6000.0f;
            /**
             * @brief Time the surface heterogeneity is correlated over, seconds.
             *
             * The seed was white in time as well, redrawn independently every step, and this is
             * the axis that carries most of the effect. A step is a few seconds; a warm patch
             * that exists for one step and is gone the next cannot lift anything, because a
             * thermal needs the eddy turnover time of the layer it rises through — h/w* is
             * about twelve minutes for a 1.5 km layer under a 2 m/s convective scale. What an
             * uncorrelated kick does instead is cancel: over N steps it accumulates as sqrt(N)
             * where a persistent one accumulates as N, so at the 150 steps a turnover takes it
             * is an order of magnitude weaker.
             *
             * That is measured, not argued — see `nest_thermal_seed` in
             * atmosphere_nest_common.glsl for the table. The short version is that the seed the
             * nest had produced 1.22× the domain structure of running with *no seed at all*,
             * and this produces 3.72×. A period well under the turnover (60 s) recovers half.
             *
             * 900 s is that turnover time. It is the *lifetime of a patch of warmer ground*,
             * not of a cloud: the cloud's own lifetime comes out of the dynamics. Longer still
             * (3600 s) raises the structure further but the field then barely renews across a
             * whole convective afternoon, and the condensate measured *below* the unseeded run.
             */
            float thermal_seed_period_s = 900.0f;
            /**
             * @brief Updraft speed a band reads as fully convective, m/s.
             *
             * A *reporting* scale: it converts the nest's vertical velocity into the
             * `convective_fraction` the genus classifier and the editor readout already speak
             * in. 2 m/s is a solidly convective updraft — a fair-weather cumulus runs a metre
             * or two per second, a thunderstorm ten times that.
             *
             * It says "purely" no longer, and the qualification is the point. `choose_step` also
             * divided the vertical CFL by ten times this, so one number was quietly setting both
             * how a cloud is *labelled* and how long the nest's time step is — and the second job
             * was invisible from here, which is how a step three times tighter than anything the
             * model needed went unquestioned. The CFL now takes its bound from the updraft the
             * readback actually measures; this remains as the fallback until the first readback
             * lands, and nothing else.
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

            // ---- Ice (Phase B3d) -----------------------------------------------------
            //
            // **A diagnostic phase partition, not a second condensate species**, and the choice
            // is deliberate. Carrying cloud ice and snow as prognostic fields costs another
            // rgba32f volume — 28 MB at the shipped tier — plus its share of the advection
            // (~0.8 ms a step, against a 10 ms step), and buys mixed-phase coexistence: liquid
            // and ice in the *same* cell with a transfer rate between them.
            //
            // At 2 km that coexistence is entirely subgrid. What is resolvable is that a cell
            // colder than freezing saturates at a lower humidity than a warm one, releases more
            // latent heat when it condenses, and precipitates as something that falls at a
            // metre per second rather than seven. All three of those are functions of the
            // temperature this cell already carries, so they are computed from it. What is given
            // up is supercooled water below the glaciation point and the Bergeron transfer's own
            // timescale — recorded, and the two-species scheme is the refinement.
            //
            // Between the two temperatures below, a cell is mixed: saturation, latent heat and
            // fall speed are all blended by the ice fraction rather than switched.

            /** @brief Latent heat of fusion, J/kg. Deposition releases this on top of @ref latent_heat_vaporization. */
            float latent_heat_fusion = 3.337e5f;
            /**
             * @brief Warm edge of the mixed-phase band, K — above this a cloud is all liquid.
             *
             * Not a switch at 0 °C, because a cloud at −5 °C is mostly supercooled water and one
             * at −20 °C is mostly crystals; the band between them is where the two coexist and
             * where the Bergeron process runs. Saturation over ice is *lower* than over liquid,
             * so a cell crossing into this band starts condensing at a humidity it would have
             * been clear at — which is why cold clouds glaciate and precipitate so much more
             * readily than warm ones.
             */
            float freezing_temperature = 273.15f;
            /** @brief Cold edge of the mixed-phase band, K — below this a cloud is all ice. */
            float glaciation_temperature = 253.15f;
            /**
             * @brief Terminal fall speed coefficient for snow in V = a·(ρ q)^b, SI.
             *
             * An order of magnitude under rain's, and that single fact is most of what makes
             * snow look like snow: a crystal aggregate has an enormous cross-section for its
             * mass, so it drifts at around a metre a second where a raindrop of the same water
             * falls at seven. It is also why a snow shower's shaft leans so far downwind.
             */
            float snow_fall_speed_coefficient = 4.8f;
            /** @brief Snow's fall-speed exponent. Flatter than rain's; aggregate size varies less with content. */
            float snow_fall_speed_exponent = 0.08f;
            /**
             * @brief Factor on @ref autoconversion_threshold once a cloud is fully glaciated.
             *
             * Ice crystals aggregate into snow far more readily than droplets coalesce into
             * drizzle — they stick, and they grow by vapour deposition at the expense of any
             * liquid beside them. Lowering the threshold is the cheap statement of that: a cold
             * cloud starts precipitating at a fraction of the condensate a warm one needs, which
             * is why a winter sky produces snow out of a deck that in summer would just sit
             * there.
             */
            float glaciated_autoconversion_factor = 0.25f;
            /**
             * @brief Effective radius of cloud *ice*, m — the optical half of the phase partition.
             *
             * §7.1's extinction divides by the effective radius, so this is nearly four times
             * @ref droplet_effective_radius and glaciated cloud is correspondingly thinner for
             * the same water. That single ratio is what makes a cirrus veil translucent and an
             * anvil's top soft where a liquid cumulus of the same condensate is a wall — a
             * crystal aggregate simply puts far less cross-section in the way per kilogram than
             * the same mass divided into 8 µm droplets.
             *
             * Without it a glaciating updraft's anvil renders as solid as its base, which is the
             * one thing everybody can see is wrong about a cloud model that has no ice.
             */
            float ice_effective_radius = 30.0e-6f;
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

            // ---- Surface energy balance (Phase B3) -----------------------------------
            //
            // **The fluxes are solved, not authored.** They used to be three numbers — a peak
            // sensible flux, a peak latent flux and a night-time cooling rate — scaled by the
            // sine of the sun's elevation, which is a *prescribed diurnal shape* wearing the
            // costume of a diurnal cycle. Everything interesting about a real one is missing
            // from it: the ground has no heat capacity, so the fluxes peak exactly at solar noon
            // instead of lagging it; the surface cannot be warmer or cooler than whatever the
            // author typed, so it cannot respond to the air above it; and the Bowen ratio is
            // fixed by construction, so a boundary layer that dries the ground out keeps
            // moistening at the same rate anyway.
            //
            // What replaces them is the textbook slab: the ground absorbs shortwave, exchanges
            // longwave with the air, and loses what is left as turbulent sensible and latent
            // heat, with its own temperature as the state that closes the loop.
            //
            //     C dT_s/dt = S(1-a) + L_down - e sigma T_s^4 - H - LE
            //     H  = rho c_p C_H |U| (T_s - T_air)
            //     LE = rho L   C_H |U| beta (q_s(T_s) - q_v_air)
            //
            // The lag, the response, and the ratio all fall out of that rather than being typed
            // in. `atmosphere_surface.comp` holds the scheme; these are its data.

            /**
             * @brief Solar irradiance at the top of the atmosphere, W/m².
             *
             * The real one, so that "how bright is the sun here" is a question about geometry
             * and the air rather than about a tuning value. Insolation reaching the ground is
             * this times the sine of the elevation of the sun the sky is *already rendered
             * with* — which is what keeps the model from acquiring a second, private sun — times
             * the transmittance below.
             */
            float solar_constant = 1361.0f;
            /**
             * @brief Clear-sky atmospheric transmittance at zenith, [0, 1].
             *
             * Applied along the slant path, so a low sun is dimmed more than by its elevation
             * alone: `tau^(1/sin(elevation))`. That is what makes the morning and evening
             * shoulders of the diurnal cycle steeper than a cosine, and it is why sunrise heats
             * the ground so much later than it lights the sky.
             */
            float clear_sky_transmittance = 0.75f;
            /**
             * @brief Shortwave albedo of the surface, [0, 1].
             *
             * 0.20 is vegetated land. Fresh snow is 0.8 and open water near noon is 0.06 — the
             * range is the widest of any parameter here, and it is the first thing to reach for
             * when a scene heats too fast or too slowly.
             */
            float surface_albedo = 0.20f;
            /** @brief Longwave emissivity of the surface, [0, 1]. Near 1 for everything natural. */
            float surface_emissivity = 0.96f;
            /**
             * @brief Heat capacity of the surface slab, J/m²/K.
             *
             * How much energy it takes to warm the ground by a degree, and therefore how far the
             * afternoon peak lags solar noon. 1×10⁵ is a few centimetres of soil, which lags by
             * an hour or two. Water is the extreme: a 3 m mixed layer is 1.3×10⁷, which is why
             * a sea surface barely moves over a day and a sea breeze exists at all.
             *
             * The slab is integrated **semi-implicitly**, linearising the fluxes about the skin
             * temperature, so this can be authored arbitrarily small without the step becoming
             * unstable — an explicit slab would need dt < C/lambda, which at these lambda (~85
             * W/m²/K) puts even soil within a factor of a few hundred of the nest's step.
             */
            float surface_heat_capacity = 1.0e5f;
            /**
             * @brief Moisture availability of the surface, [0, 1].
             *
             * The fraction of the saturation deficit the surface can actually supply: 1 is open
             * water or saturated soil, 0 is dry rock or asphalt, and 0.35 is unremarkable
             * vegetated land in summer. **This is the Bowen ratio's real author.** It used to be
             * set by the ratio of two typed-in fluxes; here it is a property of the ground, and
             * the ratio it produces changes through the day as the surface warms.
             */
            float surface_moisture_availability = 0.35f;
            /**
             * @brief Bulk transfer coefficient for heat and moisture, dimensionless.
             *
             * `k²/ln(z/z0)²` for the lowest model level over a given roughness — 0.005 is a
             * ~54 m level over 0.1 m roughness. Neutral: a stability correction belongs with the
             * Monin-Obukhov length, and this model has no surface layer resolved well enough to
             * earn one.
             */
            float surface_exchange_coefficient = 0.005f;
            /**
             * @brief Wind speed the bulk formulae never fall below, m/s.
             *
             * A bulk flux is proportional to the wind, so on a calm morning it would be zero —
             * and a calm morning is exactly when free convection carries the most heat. This
             * stands for the eddies buoyancy drives when the mean wind does not: without it the
             * model has a stable fixed point at "no wind, no flux, no convection, no wind".
             */
            float surface_minimum_wind = 1.0f;

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

        /** @brief One timed section of the nest's step, in the order it is recorded. */
        struct AtmosphereStageTiming
        {
            /**
             * @brief The section's name, owned inline.
             *
             * A buffer rather than a `const char*` because the set of sections is not fixed —
             * a frame that re-centres the lattice records a shift the next one does not — so
             * the name cannot be recovered from a position in a static table, and a pointer
             * into the profiler's own storage would dangle the next time it resolved.
             */
            char name[20]{};
            /** @brief GPU time between the section's two timestamps, milliseconds. */
            float milliseconds = 0.0f;
        };

        /** @brief Sections the step is bracketed into; the ten stages plus the submission total. */
        constexpr int ATMOSPHERE_TIMED_STAGES = 12;

        /**
         * @brief What one step of the nest actually cost on the GPU.
         *
         * §12 budgets the nest at ~2 ms per step and §11's Phase B2c closes by recording that
         * the step "has never been measured" — so every lever named there (fewer pressure
         * sweeps, the async compute queue, a multigrid pressure solve) was ordered by estimate.
         * This is the measurement those estimates were waiting on, and it is deliberately a
         * *per-stage* breakdown rather than a single number: the ordering only means anything if
         * the sweeps really are the large share the shader's own header supposes.
         *
         * Timestamps come from the same query-pool mechanism the render graph profiles passes
         * with, resolved at the point the nest already waits on the step's timeline value — so
         * measuring costs no stall. A device that reports no timestamp support leaves
         * @ref measured false and every number zero rather than reporting a fabricated one.
         *
         * Published only from frames that actually *stepped*. A frame that merely re-centres the
         * lattice records a shift, an extinction and a readback, and calling that a step's cost
         * would report a number four times too small whenever the observer moved.
         */
        struct AtmosphereStepCost
        {
            AtmosphereStageTiming stages[ATMOSPHERE_TIMED_STAGES]{};
            /** @brief Sections filled in @ref stages. */
            int count = 0;
            /**
             * @brief The whole submission, milliseconds — measured, not summed.
             *
             * An outer bracket around the entire command buffer rather than the sum of the
             * sections below, because the two are not the same number and the difference is
             * informative: a section's opening timestamp is written when the command is
             * *reached*, so back-to-back sections can overlap, and a breakdown that adds up to
             * noticeably less than the total is the signature of dispatches the barriers let run
             * concurrently. Budget against this one; attribute with the ones below it.
             */
            float total_ms = 0.0f;
            /** @brief The nest's step index this measurement was taken on. */
            std::uint64_t step_index = 0;
            /**
             * @brief Steps the measured submission recorded; @ref total_ms covers all of them.
             *
             * A frame takes up to `max_steps_per_frame` steps in one submission, so the per-step
             * cost is `total_ms / steps`. The @ref stages breakdown is from the *first* of them
             * only — the query pool is a fixed size and timing every step would spend it
             * part-way through the second.
             */
            std::uint32_t steps = 1;
            /** @brief False until a stepping frame's timestamps have been read back. */
            bool measured = false;
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
         * @brief Saturation vapour pressure over *ice*, Pa (Magnus, WMO coefficients).
         *
         * A different curve from the one above, not a correction to it, and the gap between them
         * is the whole of the Bergeron process: at −12 °C the saturation pressure over ice is
         * about 10 % below the one over supercooled water, so air that is in equilibrium with a
         * droplet is **supersaturated** with respect to a crystal beside it. The crystal grows,
         * the droplet evaporates to feed it, and that is how a mixed-phase cloud turns itself
         * into snow. The two curves meet exactly at 0 °C, which is what makes blending across
         * the mixed-phase band continuous.
         */
        inline float atmosphere_saturation_pressure_ice(float temperature_k)
        {
            return 611.2f * std::exp(22.46f * (temperature_k - 273.15f) /
                                     (temperature_k - 0.53f));
        }

        /**
         * @brief How much of a cell's condensate is ice, [0, 1].
         *
         * A linear ramp across the mixed-phase band rather than a switch at 0 °C, because a
         * cloud at −5 °C is mostly supercooled water and one at −20 °C is mostly crystals. The
         * band is where the two coexist.
         *
         * **This is the whole of the phase partition**, and everything else about ice in the
         * nest is a function of it: the saturation the cell condenses at, the latent heat it
         * releases, how readily it precipitates, and how fast that precipitation falls. Being a
         * function of temperature alone is the scheme's simplification and its named limit —
         * see @ref AtmosphereParameters::latent_heat_fusion for what that gives up.
         */
        inline float atmosphere_ice_fraction(const AtmosphereParameters& p, float temperature_k)
        {
            const float span = p.freezing_temperature - p.glaciation_temperature;
            if (!(span > 0.0f))
                return temperature_k < p.freezing_temperature ? 1.0f : 0.0f;
            const float f = (p.freezing_temperature - temperature_k) / span;
            return f < 0.0f ? 0.0f : (f > 1.0f ? 1.0f : f);
        }

        /**
         * @brief Saturation mixing ratio a cell actually condenses at, kg/kg.
         *
         * The liquid and ice curves blended by @ref atmosphere_ice_fraction. Below the
         * glaciation point this *is* the ice curve, which sits well under the liquid one — so a
         * column that would be clear at +2 °C can be cloudy at −25 °C on the same water. Mirrors
         * `nest_saturation_mixing_ratio_phase` in atmosphere_nest_common.glsl.
         */
        inline float atmosphere_saturation_mixing_ratio_phase(const AtmosphereParameters& p,
                                                              float temperature_k,
                                                              float pressure_pa)
        {
            const float ice = atmosphere_ice_fraction(p, temperature_k);
            if (ice <= 0.0f)
                return atmosphere_saturation_mixing_ratio(temperature_k, pressure_pa);
            const float e_liquid = atmosphere_saturation_pressure(temperature_k);
            const float e_ice = atmosphere_saturation_pressure_ice(temperature_k);
            const float e_s = (1.0f - ice) * e_liquid + ice * e_ice;
            const float denominator = pressure_pa - 0.378f * e_s;
            return denominator > 1.0f ? 0.622f * e_s / denominator : 1.0f;
        }

        /**
         * @brief Latent heat released per kilogram condensed at @p temperature_k, J/kg.
         *
         * Vaporization plus the ice fraction's share of fusion: depositing straight to a crystal
         * releases both, which is 13 % more heat than condensing to a droplet and is why a
         * glaciating updraft gets a second push just as it is running out of the first.
         */
        inline float atmosphere_latent_heat(const AtmosphereParameters& p, float temperature_k)
        {
            return p.latent_heat_vaporization +
                   atmosphere_ice_fraction(p, temperature_k) * p.latent_heat_fusion;
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

        /** @brief What @ref atmosphere_surface_balance leaves the ground holding. */
        struct AtmosphereSurfaceBalance
        {
            /** @brief Skin temperature at the end of the step, K. */
            float skin_k = 0.0f;
            /** @brief Sensible heat flux into the air, W/m². Negative after dusk. */
            float sensible = 0.0f;
            /** @brief Latent heat flux into the air, W/m². */
            float latent = 0.0f;
            /** @brief Net radiation absorbed by the surface, W/m². */
            float net_radiation = 0.0f;
        };

        /**
         * @brief Advance the surface energy balance by one step.
         *
         * `atmosphere_surface.comp`'s scheme, mirrored for the same reason every other relation
         * in this file is: GLSL cannot include a C++ header, so the formula is stated twice and
         * the *numbers* only once. What it buys here is a diurnal cycle with a test — the
         * equilibrium below is arithmetic, not opinion, and `test_atmosphere_nest.cpp` pins it.
         *
         *     C dT_s/dt = R_net - H - LE
         *     H  = rho c_p C_H |U| (T_s - T_air)
         *     LE = rho L   C_H |U| beta (q_s(T_s) - q_v_air)
         *
         * **Semi-implicit**, and that is the part worth reading. Every flux stiffens as the skin
         * warms — the longwave as `4εσT³`, the sensible linearly, the latent through the
         * Clausius–Clapeyron slope — some 85 W/m²/K against a soil slab of 10⁵ J/m²/K, a
         * 20-minute relaxation time. An explicit update needs `dt < 2C/λ`, so an author choosing
         * a thin slab (a road, a rock face) walks into an oscillation that reads as a physics bug
         * and is an integrator bug. Linearising the fluxes about the current skin costs one
         * divide and is unconditionally stable at any `dt`: the increment is the residual over
         * `C + dt·λ` rather than over `C`, so the step cannot outrun the response that opposes
         * it. The explicit form differs only in that denominator, and with a thin slab and a
         * long step the ratio is five orders of magnitude — what it produces is not a large
         * error, it is not a temperature.
         *
         * It is *not* exact in one step, and not monotone either: the balance is nonlinear
         * (`q_s` is exponential in the skin), so a step is one Newton iteration. Measured, a
         * single 10⁶ s step onto a 10³ J/m²/K slab overshoots the equilibrium by 3.3 K on a 16 K
         * approach and converges from the other side within a few more. Both claims are pinned
         * in `test_atmosphere_nest.cpp`, and both replaced a stronger one this comment made
         * before the test was written.
         *
         * Longwave down is Brutsaert (1975): the clear-sky emissivity of air goes as the seventh
         * root of vapour pressure over temperature, so a humid night radiates far more back down
         * than a dry one — which is why a desert freezes after dark and a coast does not.
         *
         * @param p              The authored physics.
         * @param skin_k         Skin temperature at the start of the step, K.
         * @param air_k          Air temperature at the lowest level, K.
         * @param air_vapour     Vapour mixing ratio there, kg/kg.
         * @param pressure_pa    Base-state pressure there, Pa.
         * @param density        Base-state density there, kg/m³.
         * @param wind_mps       The *exchange* velocity, not the mean wind: see the shader for
         *                       why free convection is added in quadrature.
         * @param absorbed_w     Shortwave already absorbed — after albedo, cloud and patchiness.
         * @param dt             Step length, seconds.
         */
        inline AtmosphereSurfaceBalance atmosphere_surface_balance(const AtmosphereParameters& p,
                                                                  float skin_k, float air_k,
                                                                  float air_vapour,
                                                                  float pressure_pa, float density,
                                                                  float wind_mps, float absorbed_w,
                                                                  float dt)
        {
            constexpr float STEFAN_BOLTZMANN = 5.670374e-8f;
            const auto clamp01 = [](float v)
            { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); };

            const float emissivity = clamp01(p.surface_emissivity);
            const float vapour_pressure =
                std::fmax(air_vapour * pressure_pa / (0.622f + air_vapour), 1.0f);
            float air_emissivity =
                1.24f * std::pow(vapour_pressure * 0.01f / std::fmax(air_k, 1.0f), 1.0f / 7.0f);
            air_emissivity = air_emissivity < 0.5f ? 0.5f : (air_emissivity > 1.0f ? 1.0f
                                                                                  : air_emissivity);
            const float longwave_down =
                air_emissivity * STEFAN_BOLTZMANN * air_k * air_k * air_k * air_k;
            const float longwave_up =
                emissivity * STEFAN_BOLTZMANN * skin_k * skin_k * skin_k * skin_k;

            AtmosphereSurfaceBalance out;
            out.net_radiation = absorbed_w + emissivity * longwave_down - longwave_up;

            const float exchange =
                std::fmax(p.surface_exchange_coefficient, 0.0f) * wind_mps * density;
            const float beta = clamp01(p.surface_moisture_availability);
            float sensible = exchange * p.specific_heat_pressure * (skin_k - air_k);
            const float skin_saturation =
                atmosphere_saturation_mixing_ratio(skin_k, pressure_pa);
            const float deficit = skin_saturation - air_vapour;
            float latent = exchange * p.latent_heat_vaporization * beta *
                           (deficit > 0.0f ? deficit : 0.0f);

            const float slope = skin_saturation * p.latent_heat_vaporization /
                                (p.gas_constant_vapour * std::fmax(skin_k * skin_k, 1.0f));
            const float stiffness = 4.0f * emissivity * STEFAN_BOLTZMANN * skin_k * skin_k * skin_k +
                                    exchange * p.specific_heat_pressure +
                                    exchange * p.latent_heat_vaporization * beta * slope;
            const float capacity = std::fmax(p.surface_heat_capacity, 1.0f);
            const float delta =
                (out.net_radiation - sensible - latent) * dt / (capacity + dt * stiffness);

            // Reported at the temperature the step *ends* at, so what the air is handed and what
            // the ground paid for it are the same number to first order.
            out.skin_k = skin_k + delta;
            out.sensible = sensible + exchange * p.specific_heat_pressure * delta;
            out.latent = latent + exchange * p.latent_heat_vaporization * beta * slope * delta;
            return out;
        }

        /**
         * @brief Clear-sky shortwave reaching the surface, W/m².
         *
         * `solar_constant · sin(elevation) · tau^(1/sin(elevation))` — the transmittance along
         * the *slant* path, so a low sun is dimmed by far more than its elevation alone. That is
         * what makes the morning and evening shoulders of the diurnal cycle steeper than a cosine,
         * and why the ground starts warming so much later than the sky lights up. Zero below the
         * horizon. Mirrors `atmosphere_surface.comp`; neither is edited alone.
         */
        inline float atmosphere_clear_sky_shortwave(const AtmosphereParameters& p,
                                                    float elevation_sine)
        {
            if (!(elevation_sine > 0.0f))
                return 0.0f;
            const float tau = p.clear_sky_transmittance < 0.01f
                                  ? 0.01f
                                  : (p.clear_sky_transmittance > 1.0f ? 1.0f
                                                                      : p.clear_sky_transmittance);
            return p.solar_constant * elevation_sine *
                   std::pow(tau, 1.0f / std::fmax(elevation_sine, 0.05f));
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
            // The *phase-blended* latent heat, so a glaciating cell is correctly harder to
            // condense in: it releases 13 % more heat per kilogram, which raises q_s further,
            // which leaves less of the excess surviving. Using the liquid value below freezing
            // would over-condense exactly where the extra heating matters most to the updraft.
            const float latent = atmosphere_latent_heat(p, temperature_k);
            const float slope = saturation * latent / (p.gas_constant_vapour * safe * safe);
            return 1.0f / (1.0f + latent / p.specific_heat_pressure * slope);
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
            /**
             * @brief Large-scale vertical motion, m/s. Positive is ascent.
             *
             * **The one field here that is not a boundary condition.** The three above shape what
             * the nest relaxes toward in its Davies zone; this is applied across the *whole*
             * domain, because it is a motion the nest cannot generate and would be wrong to try
             * to: a 384 km window has no way to know it is sitting under a thousand-kilometre
             * high, and the gentle descent that produces is what dries an airmass.
             *
             * Centimetres per second, and that is not a rounding error next to a 1 m/s updraft —
             * it acts over the whole domain for the whole day rather than over one cell for ten
             * minutes. Phase B3e's acceptance run failed its third clause on exactly this: with
             * no subsidence a saturated layer aloft has no sink at all overnight, so an evening
             * deck grew instead of decaying.
             */
            float vertical_velocity_mps = 0.0f;
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
            /**
             * @brief Skin temperature (K), sensible flux, latent flux, net radiation (W/m²).
             *
             * The surface energy balance's own state and what it did with it, which is not
             * derivable from anything above: the fluxes are no longer authored numbers a caller
             * could look up, and the skin temperature is a prognostic variable of the model with
             * no proxy in the air. It is also the fastest way to read *why* a sky is doing what
             * it is doing — a clear afternoon with a warm skin and a near-zero latent flux is a
             * dry surface, and the same skin with the fluxes reversed is a wet one.
             */
            float skin[4]{};
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
