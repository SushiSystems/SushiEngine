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
            std::uint32_t max_steps_per_frame = 2;
            /** @brief Subgrid eddy viscosity/diffusivity, m²/s. */
            float eddy_viscosity = 40.0f;
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
             * @brief Amplitude of the surface-layer temperature perturbation, K.
             *
             * Convection needs something to break the horizontal symmetry of a uniformly
             * heated surface, or the whole boundary layer rises as one slab and no cell ever
             * forms. Real air has turbulence doing this; a cloud model without a resolved
             * surface layer seeds it explicitly, and every one of them does.
             */
            float thermal_seed_amplitude = 0.12f;
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

            // ---- Surface forcing -----------------------------------------------------
            //
            // Prescribed in this phase. A real surface energy balance — insolation through
            // `Astro::Ephemeris`, a slab heat capacity, land/sea partitioning — is the next
            // phase's `ISurfaceModel`, and it is what turns these two numbers into the
            // diurnal cycle. Prescribed values are stated as such rather than dressed up:
            // they are what a fair-weather convective afternoon actually delivers.

            /** @brief Prescribed surface sensible heat flux, W/m². */
            float surface_sensible_flux = 120.0f;
            /** @brief Prescribed surface latent (moisture) heat flux, W/m². */
            float surface_latent_flux = 90.0f;

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
         * from ~80 m at the surface to ~560 m aloft — fine where the boundary layer and cloud
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
