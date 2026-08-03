/**************************************************************************/
/* quasigeostrophic_core.hpp                                              */
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
 * @file quasigeostrophic_core.hpp
 * @brief T1: the global dynamical core — two-layer moist quasi-geostrophic flow.
 *
 * `docs/slop/atmosphere_system.md` §5. The replacement for `Simulation::SynopticLayer`, which
 * is not a simulation: it translates authored elliptical Gaussians across the sphere and
 * diagnoses a wind from their summed gradient, so a low can only ever do what it was told to
 * do at genesis. What runs here instead is a dynamical core. The prognostic variables are the
 * two layers' potential vorticity and the column's precipitable water; everything else —
 * wind, pressure, vertical motion, fronts, the jet — is diagnosed from them. Relaxation
 * toward a baroclinically unstable mean state keeps the mid-latitudes supplied with available
 * potential energy, and cyclones then grow out of whatever perturbation is present. **Nothing
 * places a low.**
 *
 * **It runs on the CPU, and §3.3 sketched it on the GPU.** The sketch's own escape clause is
 * the reason: T1 is 131 072 cells stepped once per six minutes of game time, and every one of
 * its consumers is host-side — the nest's parent forcing is assembled in host memory, and the
 * gameplay question this tier exists to answer ("what is the weather a thousand kilometres
 * away") is a CPU query. Putting it on the device would buy a step that is already free and
 * pay for it with a readback and its latency. The cost is a few milliseconds once per six
 * simulated minutes, and what it buys back is that the whole tier is testable without a
 * device and deterministic without a driver.
 *
 * **Formulation.** Two layers of equal depth on a cell-centred latitude/longitude grid,
 * periodic in longitude, with the polar cell edges falling exactly on the poles where
 * `cos(latitude)` vanishes — so the boundary condition the elliptic operator needs is the one
 * the metric already supplies, and no pole is a special case.
 *
 *     q_1 = laplacian(psi_1) + f - S (psi_1 - psi_2)          (upper)
 *     q_2 = laplacian(psi_2) + f + S (psi_1 - psi_2)          (lower)
 *     dq_k/dt + J(psi_k, q_k) = -(q_k - q_k_climatology)/tau + D_k
 *     dW/dt  + u_2 . grad(W)  = W w/H + E - P
 *
 * The planetary vorticity `f` sits *inside* `q`, so advecting `q` carries the beta effect with
 * it and there is no separate beta term to get wrong. `S = f^2/(g' H)` is the inverse squared
 * deformation radius; because it multiplies the same layer difference in both equations, the
 * inversion decouples **exactly** into a barotropic and a baroclinic problem, each of which is
 * a Fourier transform in longitude followed by a tridiagonal solve in latitude. That is what
 * makes this core cheap, and it is exact rather than approximate.
 *
 * **Determinism.** Every stochastic decision — the initial perturbation's phases — draws from
 * a `Loop::RNGState` seeded by the caller, never from a clock. Two cores seeded identically
 * and stepped with the same sequence of `dt` stay bit-identical. §3.4's posture says the
 * atmosphere is not part of the rollback snapshot; this is nonetheless deterministic, because
 * a tier that cannot be reproduced cannot be debugged.
 */

#include <cstdint>
#include <memory>
#include <vector>

#include <SushiEngine/atmosphere/climatology.hpp>
#include <SushiEngine/atmosphere/geographic_position.hpp>
#include <SushiEngine/core/random_number_generator.hpp>

namespace SushiEngine
{
    namespace Atmosphere
    {
        /** @brief A horizontal wind vector, metres per second. */
        struct Wind
        {
            double eastward_mps = 0.0;
            double northward_mps = 0.0;
        };

        /**
         * @brief The core's discretization.
         *
         * Separate from @ref QuasiGeostrophicParameters because it is not physics: it is how
         * finely the physics is resolved. Deliberately **not** on any quality tier — the
         * global core's state is scene data (its fields persist in the weather sidecar,
         * whose grid must match to load), its step is host-side and nearly free, and a
         * tier that resized it would make "which machine opened the scene" decide whether
         * the saved weather survives. Only the regional nest's grid is tiered
         * (`resolve_atmosphere_quality`); this one is fixed at the resolution the sidecar
         * was written at, and only a test or a probe passes something smaller.
         */
        struct QuasiGeostrophicGridSize
        {
            /**
             * @brief Cells around a latitude circle. **Must be a power of two.**
             *
             * The longitude transform is radix-2. 512 is §5's resolution, ~78 km at the
             * equator; a test or a headless probe may ask for less.
             */
            int longitude_cells = 512;

            /** @brief Cells from pole to pole. Even, so the equator falls on a cell edge. */
            int latitude_cells = 256;
        };

        /**
         * @brief Every physical constant the global core is expressed in terms of.
         *
         * §3.5's rule, applied: none of these is a function-local constant, all of them are
         * serializable data, and a different planet is a different value of these rather than
         * a different code path.
         */
        struct QuasiGeostrophicParameters
        {
            /** @brief Mean radius of the body, m. */
            double planet_radius_m = 6371000.0;

            /** @brief Sidereal rotation rate, rad/s — the Coriolis parameter's scale. */
            double angular_velocity_rad_per_s = 7.2921159e-5;

            /** @brief Density the streamfunction is converted to a pressure anomaly with, kg/m^3. */
            double air_density_kg_per_m3 = 1.225;

            /** @brief Gravitational acceleration, m/s^2 — the hypsometric conversion's numerator. */
            double gravity_mps2 = 9.80665;

            /**
             * @brief Mean temperature of the layer the two streamfunctions straddle, K.
             *
             * Only the thermal anomaly reads it. The difference between the layers *is* a
             * thickness, and a thickness is a temperature by the hypsometric relation; turning
             * one into the other needs the temperature that thickness is measured about, and
             * mid-troposphere is about 250 K.
             */
            double reference_temperature_kelvin = 250.0;

            /**
             * @brief The constant Coriolis parameter the streamfunction is defined against, 1/s.
             *
             * Quasi-geostrophy defines `psi = geopotential / f0` with a *constant* `f0`; the
             * latitude-varying `f` appears only in the potential vorticity and its advection.
             * Using a local `f` here instead would make the streamfunction singular at the
             * equator, which is the one place this formulation is already weakest.
             */
            double reference_coriolis = 1.0e-4;

            /**
             * @brief Reduced gravity between the two layers, m/s^2.
             *
             * `g' = g * delta_theta / theta`. With a 30 K contrast across a 300 K troposphere
             * this is about 1 m/s^2, and together with @ref layer_depth_m it sets the
             * deformation radius — the scale cyclones come out at.
             */
            double reduced_gravity_mps2 = 0.98;

            /** @brief Depth of each layer, m. Two of these make the troposphere. */
            double layer_depth_m = 5000.0;

            /**
             * @brief Ceiling on the deformation radius, m.
             *
             * `S = f^2/(g'H)` vanishes at the equator, where quasi-geostrophy has no business
             * being anyway (§5's tropical honesty, §14). Flooring `S` at `1/L^2` keeps the
             * baroclinic inversion well conditioned there and states the limit as a number
             * rather than leaving it to be discovered as a solver failure.
             */
            double maximum_deformation_radius_m = 5.0e6;

            /**
             * @brief Time scale the flow is relaxed back toward the climatological state, s.
             *
             * Radiative restoration, in the Held-Suarez sense: it is what continually rebuilds
             * the shear the eddies keep flattening, and therefore what makes the storm track a
             * steady state rather than a single burst. Shorter means more, smaller storms.
             */
            double relaxation_seconds = 1296000.0; // 15 days

            /**
             * @brief Lower-layer Ekman spin-down time, s.
             *
             * Surface drag, as a damping of the lower layer's relative vorticity. Without it
             * the baroclinic instability has no energy sink at large scale and the eddy field
             * grows until the advection scheme is the only thing bounding it.
             */
            double ekman_seconds = 432000.0; // 5 days

            /**
             * @brief Time the grid-scale wave is damped on, s.
             *
             * One number for both directions of the scale-selective damping: the meridional
             * fourth-order operator's coefficient is derived from it, and the zonal spectral
             * filter uses it directly. Set so the two-cell wave decays in under an hour, which
             * is the enstrophy sink a two-dimensional flow needs and does not otherwise have.
             */
            double grid_scale_damping_seconds = 3600.0;

            /**
             * @brief Latitude poleward of which the zonal filter starts truncating, radians.
             *
             * A latitude/longitude grid's zonal spacing shrinks as `cos(latitude)`, and near
             * the pole it would set the time step for the whole globe. The filter removes the
             * zonal wavenumbers that spacing cannot carry, which is the standard lat/lon fix
             * (§5) and is why the step can be chosen from the mid-latitude spacing.
             */
            double filter_reference_latitude_radians = 1.0471975511965976; // 60 degrees

            /** @brief Sharpness of the filter's roll-off; even, and higher is sharper. */
            int filter_order = 8;

            /**
             * @brief Angle surface friction turns the near-surface wind toward low pressure, rad.
             *
             * Applied only to the wind reported at level fraction 0, and with the hemisphere's
             * sign — air spirals into a low counterclockwise in the north and clockwise in the
             * south, so the rotation is not the same rotation on both sides of the equator.
             */
            double surface_friction_radians = 0.4363323129985824; // 25 degrees

            /** @brief Time scale surface evaporation moistens a dry column on, s. */
            double evaporation_seconds = 259200.0; // 3 days

            /** @brief Column relative humidity evaporation drives toward, [0, 1]. */
            double evaporation_relative_humidity = 0.75;

            /** @brief Time scale supersaturated water is rained out on, s. */
            double condensation_seconds = 3600.0;

            /** @brief Depth the column's water is treated as spread over, m — ascent's lever arm. */
            double moisture_depth_m = 3000.0;

            /**
             * @brief Length of one step, seconds of game time.
             *
             * §5's six minutes. The advective limit at 78 km with a 60 m/s jet is nearer twenty,
             * so this is not a stability floor — it is the accuracy the moisture transport wants.
             */
            double step_seconds = 360.0;

            /** @brief Steps a single @ref QuasiGeostrophicCore::advance call may take. */
            int max_steps_per_advance = 8;

            /**
             * @brief Wind scale of the perturbation the initial state is disturbed with, m/s.
             *
             * The seed of every storm the core will ever produce, and nothing else about a
             * storm is authored. Zero gives an exactly zonal state that stays zonal forever,
             * which is a legal — and instructive — configuration.
             */
            double seed_perturbation_mps = 1.0;

            /** @brief Lowest zonal wavenumber the initial perturbation excites. */
            int seed_lowest_wavenumber = 4;

            /** @brief Highest zonal wavenumber the initial perturbation excites. */
            int seed_highest_wavenumber = 12;
        };

        /**
         * @brief Whole-globe measurements of the core's state, for the probe and the editor.
         *
         * Recomputed on request rather than accumulated per step: they are diagnostics, and a
         * diagnostic that costs something every step whether or not anyone reads it is a
         * diagnostic that gets switched off.
         */
        struct QuasiGeostrophicDiagnostics
        {
            /** @brief Area-weighted kinetic energy of the departure from the zonal mean, J/m^2. */
            double eddy_kinetic_energy_j_per_m2 = 0.0;
            /** @brief Area-weighted kinetic energy of the zonal mean flow itself, J/m^2. */
            double zonal_kinetic_energy_j_per_m2 = 0.0;
            /** @brief Strongest upper-layer zonal-mean westerly, m/s — the jet. */
            double jet_speed_mps = 0.0;
            /** @brief Latitude that jet sits at, radians. */
            double jet_latitude_radians = 0.0;
            /** @brief Fastest wind anywhere in either layer, m/s. */
            double peak_wind_mps = 0.0;
            /** @brief Deepest surface pressure anomaly anywhere, hPa about the zonal mean (negative). */
            double lowest_pressure_anomaly_hpa = 0.0;
            /** @brief Area-weighted mean column water, kg/m^2. */
            double mean_precipitable_water_kg_per_m2 = 0.0;
            /** @brief Area-weighted mean precipitation, mm/day. */
            double mean_precipitation_mm_per_day = 0.0;
            /** @brief Strongest mid-level ascent anywhere, m/s. */
            double peak_ascent_mps = 0.0;
            /** @brief Area-weighted potential enstrophy, 1/s^2 — the conserved quantity to watch. */
            double potential_enstrophy = 0.0;
        };

        /**
         * @brief The global core: two layers of potential vorticity, and everything it implies.
         *
         * Owns its fields and its solver tables. Non-copyable — it is a megabyte-scale piece of
         * simulation state with a precomputed factorization attached, and copying one silently
         * is never what a caller meant.
         */
        class QuasiGeostrophicCore
        {
            public:
                /**
                 * @brief Allocates the grid and factors the elliptic operator.
                 *
                 * The tridiagonal factorization depends only on the grid and the stratification,
                 * neither of which changes after construction, so it is computed once here
                 * rather than once per inversion — which is three times a step.
                 *
                 * @param size       The discretization; fixed for the core's lifetime.
                 * @param parameters The physics; fixed for the core's lifetime, because the
                 *                   factorization above is derived from part of it.
                 */
                QuasiGeostrophicCore(const QuasiGeostrophicGridSize& size,
                                     const QuasiGeostrophicParameters& parameters,
                                     const Climatology& climatology = Climatology());
                ~QuasiGeostrophicCore();

                QuasiGeostrophicCore(const QuasiGeostrophicCore&) = delete;
                QuasiGeostrophicCore& operator=(const QuasiGeostrophicCore&) = delete;

                /**
                 * @brief Whether the grid was legal and the core is usable.
                 *
                 * False when @ref QuasiGeostrophicGridSize::longitude_cells was not a power of
                 * two or a dimension was too small to hold the stencils. A core that is not
                 * valid steps to nothing and answers every query with zero.
                 */
                bool valid() const noexcept;

                /**
                 * @brief Sets the state to the climatological jet plus a seeded perturbation.
                 *
                 * The perturbation is a handful of zonal wavenumbers with random phases,
                 * enveloped on the jet and given opposite signs in the two layers, because the
                 * mode that is going to grow is baroclinic and seeding it directly costs days of
                 * simulated time less than waiting for it to emerge from rounding.
                 *
                 * @param seed Any 64-bit value; identical seeds reproduce identical weather.
                 */
                void seed(std::uint64_t seed);

                /**
                 * @brief Advances the state by one step of @p dt_seconds.
                 *
                 * Third-order strong-stability-preserving Runge-Kutta on the potential
                 * vorticity, one elliptic inversion per stage; then the moisture transport, the
                 * zonal filter, and the diagnosed vertical velocity.
                 *
                 * @param dt_seconds Step length, game seconds. Non-positive is a no-op.
                 */
                void step(double dt_seconds);

                /**
                 * @brief Takes as many whole steps as @p elapsed_seconds has accumulated.
                 *
                 * The clock the caller keeps is game time and the step length is a property of
                 * the atmosphere, so the two do not divide evenly; the remainder is carried.
                 * Capped by @ref QuasiGeostrophicParameters::max_steps_per_advance, and time
                 * beyond the cap is dropped rather than owed — a core that has fallen a
                 * simulated day behind should skip it, not spend the next minute catching up.
                 *
                 * @param elapsed_seconds Game seconds since the previous call. Negative is a no-op.
                 * @return Steps actually taken.
                 */
                int advance(double elapsed_seconds);

                /**
                 * @brief Serializes the prognostic state to a self-describing byte blob.
                 *
                 * Everything needed to resume: the two layers' vorticity, the column water, the
                 * clock, and the generator. Not the parameters — those are authored data and
                 * live in the scene beside every other authored value.
                 *
                 * **The vorticity is stored with the planetary part removed and single
                 * precision costs nothing for it.** `q` is dominated by `f`, which is a known
                 * function of latitude and identical in every core; storing `q - f` spends all
                 * seven of a float's digits on the part that varies, and the restore adds `f`
                 * back from the same table the step uses. At 512×256 the blob is 1.5 MB, against
                 * the ten minutes of stepping that reproducing seventy simulated days from a
                 * seed would take.
                 *
                 * @return The blob, or empty if the core is not @ref valid.
                 */
                std::vector<std::uint8_t> capture() const;

                /**
                 * @brief Restores a state previously produced by @ref capture.
                 *
                 * Rejects a blob whose grid does not match this core's rather than resampling
                 * one: the shipped grid is fixed (see @ref QuasiGeostrophicGridSize), so a
                 * mismatch means the blob came from a probe or a differently-built core, and
                 * the honest response is a visible restart, not a silently interpolated
                 * different atmosphere.
                 *
                 * @param blob Bytes from @ref capture.
                 * @return Whether the blob was accepted. On false the core is left untouched.
                 */
                bool restore(const std::vector<std::uint8_t>& blob);

                /**
                 * @brief Adds a Gaussian potential-vorticity anomaly to both layers.
                 *
                 * §5's replacement for "place a low here": what is injected is vorticity, which
                 * then evolves under the real dynamics — deepening, tilting, and moving with the
                 * steering flow — instead of translating as a rigid shape. Cyclonic in the
                 * hemisphere it is placed in, so a caller asking for a low gets a low on either
                 * side of the equator.
                 *
                 * @param position    Where to centre it.
                 * @param radius_m    e-folding radius of the anomaly, m.
                 * @param amplitude_mps Peak rotational wind the anomaly is scaled to produce, m/s;
                 *                      positive for a low, negative for a high.
                 */
                void inject_vorticity(const GeographicPosition& position, double radius_m,
                                      double amplitude_mps);

                /**
                 * @brief The wind at a point, interpolated between the two layers.
                 * @param position       Query point.
                 * @param level_fraction 0 at the surface — the lower layer, turned by friction —
                 *                       through 1 at the tropopause, the upper layer. Clamped.
                 * @return The wind, m/s.
                 */
                Wind wind_at(const GeographicPosition& position, double level_fraction) const;

                /**
                 * @brief Surface pressure anomaly at a point, hPa about the zonal mean.
                 *
                 * `p' = rho f0 (psi_2 - <psi_2>)`, the geostrophic relation the streamfunction is
                 * defined by, so this is the same field the wind is and not an independent one
                 * that could disagree with it.
                 *
                 * **About the zonal mean, like @ref thermal_anomaly_at and
                 * @ref humidity_anomaly_at, and for the same reason.** The lower streamfunction
                 * carries the mean westerly jet, whose own meridional gradient is worth over a
                 * hundred hectopascals from the tropics to the pole — an order more than any
                 * cyclone. Reported against a global mean, this query would answer "how far north
                 * are you" and a low would be an invisible ripple on it.
                 *
                 * @param position Query point.
                 * @return Departure from the zonal-mean surface pressure, hectopascals.
                 */
                double pressure_anomaly_hpa(const GeographicPosition& position) const;

                /**
                 * @brief Column precipitable water at a point, kg/m^2 (equivalently mm).
                 * @param position Query point.
                 * @return The column's water content.
                 */
                double precipitable_water_at(const GeographicPosition& position) const;

                /**
                 * @brief Mid-tropospheric temperature anomaly at a point, K.
                 *
                 * **The departure from the zonal mean, not from a standard atmosphere.** A
                 * limited-area nest already carries the mean state in its own base profile; what
                 * it cannot generate, and therefore what its parent has to supply, is the
                 * synoptic eddy — the warm sector and the cold air behind the front. Handing it
                 * the absolute meridional gradient instead would tell a nest at 60° that it is
                 * twenty kelvin colder than its own base state, permanently.
                 *
                 * The difference between the two streamfunctions is a layer thickness, and a
                 * thickness is a temperature by the hypsometric relation. That is the whole
                 * conversion; there is no separate temperature field to disagree with.
                 *
                 * @param position Query point.
                 * @return Departure from the zonal mean temperature, kelvin.
                 */
                double thermal_anomaly_at(const GeographicPosition& position) const;

                /**
                 * @brief Column humidity anomaly at a point, as a fraction of saturation.
                 *
                 * The moisture counterpart of @ref thermal_anomaly_at and departing from the
                 * same reference for the same reason. Expressed against the column's saturation
                 * rather than in kilograms so that a warm moist tropical anomaly and a cold moist
                 * polar one read as comparable numbers, which is what the nest's relative
                 * humidity wants.
                 *
                 * @param position Query point.
                 * @return Departure from the zonal mean, in units of column saturation.
                 */
                double humidity_anomaly_at(const GeographicPosition& position) const;

                /**
                 * @brief Strength of the thermal gradient at a point, K per 100 km.
                 *
                 * **What replaces asking where the fronts were drawn.** The analytic layer
                 * carried a fixed-angle ray pair from each low's centre and reported proximity
                 * to it; a dynamical core has no such object, because a front is not a thing it
                 * places — it is a place where the temperature gradient has been concentrated by
                 * the flow. So this measures the gradient rather than looking anything up, and a
                 * consumer that wants "is there a front here" thresholds it.
                 *
                 * The scale is physical and discriminates on its own: the background baroclinic
                 * zone runs a few tenths of a kelvin per hundred kilometres, and a genuine
                 * frontal zone runs five or more.
                 *
                 * @param position Query point.
                 * @return Magnitude of the horizontal temperature gradient, K/100 km.
                 */
                double frontal_strength_at(const GeographicPosition& position) const;

                /**
                 * @brief Mid-level vertical velocity at a point, m/s. Positive is ascent.
                 *
                 * The quasi-geostrophic omega, diagnosed from the upper layer's own vorticity
                 * budget rather than from a separate equation, so it is consistent with the
                 * flow by construction. Centimetres per second is the synoptic scale here, and
                 * that is the number the regional nest wants as its large-scale forcing.
                 *
                 * @param position Query point.
                 * @return Vertical velocity, m/s.
                 */
                double vertical_velocity_at(const GeographicPosition& position) const;

                /**
                 * @brief Precipitation rate at a point, mm/h.
                 * @param position Query point.
                 * @return The rate the most recent step condensed water out at.
                 */
                double precipitation_at(const GeographicPosition& position) const;

                /** @brief Whole-globe measurements of the current state. */
                QuasiGeostrophicDiagnostics diagnostics() const;

                /** @brief Total game seconds stepped since @ref seed. */
                double simulated_seconds() const noexcept;

                /** @brief Steps taken since @ref seed. */
                std::uint64_t step_count() const noexcept;

                /** @brief The discretization the core was built with. */
                const QuasiGeostrophicGridSize& size() const noexcept;

                /** @brief The physics the core was built with. */
                const QuasiGeostrophicParameters& parameters() const noexcept;

                /**
                 * @brief The mean state the core relaxes toward (T0, §4).
                 *
                 * Exposed because "which climatology is this weather a departure from" is a
                 * question the editor has to be able to answer: a scene running on the analytic
                 * latitude bands when somebody meant it to run on a real bake is otherwise
                 * invisible until the jet turns up in the wrong place.
                 */
                const Climatology& climatology() const noexcept;

                /**
                 * @brief Moves the mean state to a position in the year (T0, §4).
                 *
                 * **The season is not a force on the flow; it is a change to what the flow is
                 * relaxing toward.** Potential vorticity and column water carry straight through
                 * untouched — only the saturation ceiling and the climatological state are
                 * re-read. A cyclone alive in April does not vanish because the target jet
                 * moved; it finds itself in a slightly different mean flow, which is what a
                 * season is.
                 *
                 * Cheap to call every tick, and meant to be. The value is wrapped into [0, 1)
                 * and then **quantized to whole days**, so the mean state is a pure function of
                 * the date rather than of how often the host called — two hosts ticking at
                 * different rates get identical weather, which §3.4 requires. Rebuilding the two
                 * tables therefore happens about 365 times a simulated year rather than once a
                 * frame, and the call is a comparison the rest of the time.
                 *
                 * Nothing calls this on a core that was never seeded; on an invalid core it does
                 * nothing and answers false.
                 *
                 * @param year_fraction Position in the year; 0 is the start of the first month,
                 *                      and values outside [0, 1) wrap rather than clamp, because
                 *                      the last hour of December is adjacent to the first of
                 *                      January.
                 * @return Whether the mean state actually moved — false when the date resolved to
                 *         the same day, which is the common case and not a failure.
                 */
                bool set_year_fraction(double year_fraction);

                /** @brief The position in the year the mean state is currently built for. */
                double year_fraction() const noexcept;

                /** @brief Centre latitude of row @p index, radians. */
                double latitude_of(int index) const noexcept;

                /** @brief Centre longitude of column @p index, radians in [0, 2*pi). */
                double longitude_of(int index) const noexcept;

                /**
                 * @brief Read-only view of a layer's streamfunction, row-major, longitude fastest.
                 * @param layer 0 for the upper layer, 1 for the lower.
                 * @return `longitude_cells * latitude_cells` values, m^2/s; empty if @p layer is
                 *         out of range or the core is not @ref valid.
                 */
                const std::vector<double>& streamfunction(int layer) const;

                /**
                 * @brief Read-only view of a layer's potential vorticity, same layout.
                 * @param layer 0 for the upper layer, 1 for the lower.
                 * @return The field, 1/s; empty if @p layer is out of range.
                 */
                const std::vector<double>& potential_vorticity(int layer) const;

                /** @brief Read-only view of the column water field, kg/m^2, same layout. */
                const std::vector<double>& precipitable_water() const;

                /** @brief Read-only view of the diagnosed mid-level vertical velocity, m/s. */
                const std::vector<double>& vertical_velocity() const;

            private:
                struct State;
                std::unique_ptr<State> state_;
        };
    } // namespace Atmosphere
} // namespace SushiEngine
