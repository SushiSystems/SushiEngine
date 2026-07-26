/**************************************************************************/
/* synoptic_weather.hpp                                                  */
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
 * @file synoptic_weather.hpp
 * @brief T1: the synoptic layer — analytic, global, deterministic pressure systems.
 *
 * `docs/slop/weather_and_clouds.md` §5.1: N moving elliptical Gaussian pressure
 * systems on the sphere (the DCS dynamic-weather pattern), each with a life cycle
 * (deepen -> mature -> fill), climate-prior-weighted genesis, geostrophic wind, and
 * warm/cold front distance fields on each low's periphery. Pure analytic math over
 * `double` — evaluated anywhere on the body in microseconds, no grid required — so
 * `SynopticLayer::tick` is safe to call every fixed simulation step. Determinism
 * follows `docs/slop/SUSHILOOP.md`'s rule directly: every stochastic decision (when
 * and where a system is born, its initial radii/heading/lifetime) draws from the
 * `Loop::RngState` carried in `SynopticState`, never from wall-clock time, so two
 * `SynopticLayer`s seeded identically and ticked with the same `dt`/`julian_date`
 * sequence stay bit-identical forever.
 *
 * The geometry is deliberately simplified: distances and pressure gradients are
 * computed in a small equirectangular tangent plane local to each system's own
 * center (`x = R * dlon * cos(lat_center)`, `y = R * dlat`), the same class of
 * curvature-free approximation W1's flat cloud-field tile and the shadow map's
 * planet-pole-as-up shortcut already accept at their own regional scale (see
 * CHANGELOG's W1/W2 entries) — valid at synoptic system radii (hundreds of km),
 * not meant to be geodesically exact. Front geometry is similarly stylized: a
 * fixed-angle ray pair from the system center along its heading, not a
 * frontogenesis model — the "distance field feeding T2's shaping" the design doc
 * asks for, without simulating frontal-zone dynamics.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

#include <SushiEngine/loop/rng.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Maximum simultaneously-live synoptic pressure systems. */
        constexpr int MAX_SYNOPTIC_SYSTEMS = 8;

        /** @brief A pressure system's life-cycle stage (design doc: "deepen -> mature -> fill"). */
        enum class PressureSystemPhase : std::uint32_t
        {
            Deepening = 0,
            Mature,
            Filling,
        };

        /**
         * @brief One moving elliptical Gaussian pressure system (a low or a high).
         *
         * Trivially copyable: every field is a plain arithmetic type, so a
         * `SynopticState` (an array of these) can be memcmp'd or byte-copied for the
         * determinism proof and snapshotted for rollback/serialization the same way
         * `Loop::RngState` is.
         */
        struct PressureSystem
        {
            std::uint32_t id = 0;    /**< Stable id, assigned at genesis; never reused while live. */
            bool is_low = true;      /**< Low (cyclonic, cloud-bearing) or high (clearing) pressure. */
            PressureSystemPhase phase = PressureSystemPhase::Deepening;
            double age_seconds = 0.0;
            double deepen_seconds = 0.0; /**< Duration of the Deepening phase. */
            double mature_seconds = 0.0; /**< Duration of the Mature phase. */
            double fill_seconds = 0.0;   /**< Duration of the Filling phase. */
            double center_latitude_radians = 0.0;
            double center_longitude_radians = 0.0;
            double heading_radians = 0.0;  /**< Direction of travel, 0 = north, increasing clockwise. */
            double curvature_radians_per_second = 0.0; /**< Slow steering-flow turn of @ref heading_radians. */
            double speed_mps = 0.0;
            double central_anomaly_hpa = 0.0; /**< Peak |pressure - 1013.25 hPa| once Mature; sign from @ref is_low. */
            double radius_major_m = 0.0;
            double radius_minor_m = 0.0;
            double orientation_radians = 0.0; /**< Rotation of the ellipse's major axis. */
        };

        /**
         * @brief The synoptic layer's full state: every live system plus its RNG and clock.
         *
         * Plain data, trivially copyable end to end (a fixed-size array, not a
         * `std::vector`) so the whole layer can be captured, compared, and restored as
         * one value — the same shape `docs/slop/SUSHILOOP.md`'s rollback snapshot
         * expects of simulation state.
         */
        struct SynopticState
        {
            PressureSystem systems[MAX_SYNOPTIC_SYSTEMS]{};
            int system_count = 0;
            Loop::RngState rng{};
            std::uint32_t next_system_id = 1;
            double elapsed_seconds = 0.0;
            double seconds_to_next_genesis = 0.0;
        };

        /** @brief Warm/cold front proximity at a point, in [0, 1] (1 = on the front line). */
        struct FrontProximity
        {
            float warm = 0.0f;
            float cold = 0.0f;
        };

        /** @brief Wind at a point: eastward/northward components, metres/second. */
        struct WindSample
        {
            double eastward_mps = 0.0;
            double northward_mps = 0.0;
        };

        /**
         * @brief T1: N moving elliptical Gaussian pressure systems over the WGS84 sphere.
         *
         * Owns a `SynopticState` and evolves it on `tick()`; every other method is a
         * pure, `const` evaluation of the current state at an arbitrary point, so a
         * caller (T2, the editor's map overlay, a future debug view) can sample the
         * synoptic field as densely as it likes without touching the simulation.
         */
        class SynopticLayer
        {
            public:
                /** @brief Earth's mean angular velocity, rad/s — the Coriolis parameter's scale. */
                static constexpr double EARTH_ANGULAR_VELOCITY_RAD_PER_S = 7.2921159e-5;

                /** @brief Standard sea-level pressure, hPa — the baseline every anomaly is added to. */
                static constexpr double BASELINE_PRESSURE_HPA = 1013.25;

                /**
                 * @brief Resets the layer to an empty, freshly-seeded state.
                 * @param seed         Any 64-bit seed; identical seeds reproduce identical evolution.
                 * @param planet_radius_m The dominant body's mean radius, metres (WGS84 Earth by default).
                 */
                void seed(std::uint64_t seed, double planet_radius_m = 6371000.0) noexcept
                {
                    state_ = SynopticState{};
                    state_.rng = Loop::seed_rng(seed);
                    planet_radius_m_ = planet_radius_m;
                    state_.seconds_to_next_genesis = draw_genesis_interval();
                }

                /** @brief The live state (read-only) — for the editor map overlay and serialization. */
                const SynopticState& state() const noexcept { return state_; }

                /**
                 * @brief Restores a previously captured state verbatim (scene load, rollback restore).
                 * @param state The state to adopt.
                 * @param planet_radius_m The dominant body's mean radius, metres.
                 */
                void set_state(const SynopticState& state, double planet_radius_m) noexcept
                {
                    state_ = state;
                    planet_radius_m_ = planet_radius_m;
                }

                /**
                 * @brief Advances every system's kinematics/life-cycle and spawns/retires systems.
                 *
                 * @param dt_seconds  Fixed step duration; never wall-clock (see file docs).
                 * @param julian_date Epoch, for the latitude/season climate prior genesis draws.
                 */
                void tick(double dt_seconds, double julian_date) noexcept
                {
                    state_.elapsed_seconds += dt_seconds;

                    int write = 0;
                    for (int i = 0; i < state_.system_count; ++i)
                    {
                        PressureSystem system = state_.systems[i];
                        advance_system(system, dt_seconds);
                        if (system.phase == PressureSystemPhase::Filling &&
                            system.age_seconds >= system.deepen_seconds + system.mature_seconds +
                                                      system.fill_seconds)
                            continue; // dissipated: drop it, compacting the array in place.
                        state_.systems[write++] = system;
                    }
                    state_.system_count = write;

                    state_.seconds_to_next_genesis -= dt_seconds;
                    if (state_.seconds_to_next_genesis <= 0.0 &&
                        state_.system_count < MAX_SYNOPTIC_SYSTEMS)
                    {
                        state_.systems[state_.system_count++] = spawn_system(julian_date);
                        state_.seconds_to_next_genesis = draw_genesis_interval();
                    }
                }

                /**
                 * @brief The summed pressure anomaly field at a point, hPa relative to @ref BASELINE_PRESSURE_HPA.
                 * @param position Query point.
                 * @return `sum_i +/- depth_i * life_scale_i * gaussian_i(position)`.
                 */
                double pressure_anomaly_hpa(const GeodeticPosition& position) const noexcept
                {
                    double total = 0.0;
                    for (int i = 0; i < state_.system_count; ++i)
                        total += system_anomaly(state_.systems[i], position);
                    return total;
                }

                /**
                 * @brief Geostrophic wind at a point and altitude: `k_hat x grad(p)`, hemisphere-signed.
                 *
                 * Adds a surface-friction inward turn that fades out with altitude and a
                 * mid-latitude jet-band speed boost aloft, per the design doc's "hemisphere-
                 * dependent sign + friction turning near the surface + jet-band bias at
                 * altitude".
                 *
                 * @param position    Query point.
                 * @param level_fraction 0 at the surface, 1 at the top of the tracked column
                 *                       (friction fully relaxes, jet bias is fully applied).
                 * @return The wind vector, metres/second.
                 */
                WindSample wind_at(const GeodeticPosition& position, double level_fraction) const noexcept
                {
                    const auto [grad_east, grad_north] = pressure_gradient(position);

                    // Coriolis parameter f = 2*Omega*sin(lat); floored in magnitude so the
                    // geostrophic balance (which is singular at the equator) stays bounded — a
                    // documented simplification, not a physical model of equatorial dynamics.
                    constexpr double MIN_ABS_SIN_LAT = 0.15;
                    double sin_lat = std::sin(position.latitude_radians);
                    const double sign = sin_lat >= 0.0 ? 1.0 : -1.0;
                    if (std::fabs(sin_lat) < MIN_ABS_SIN_LAT)
                        sin_lat = sign * MIN_ABS_SIN_LAT;
                    const double f = 2.0 * EARTH_ANGULAR_VELOCITY_RAD_PER_S * sin_lat;

                    // k_hat x grad(p) in an East-North frame is (-grad_north, grad_east); the
                    // hemisphere sign of f already folds the Northern/Southern rotation sense in.
                    constexpr double WIND_SCALE = 6.0e4; // tuned so a ~30 hPa deepening low reads ~15-20 m/s.
                    double u = -grad_north / f * WIND_SCALE;
                    double v = grad_east / f * WIND_SCALE;

                    // Surface friction: turn the wind inward toward low centers (and outward from
                    // highs) by a fixed angle that relaxes to zero aloft.
                    constexpr double SURFACE_FRICTION_RADIANS = 0.4363323; // 25 degrees
                    const double friction = SURFACE_FRICTION_RADIANS * (1.0 - std::clamp(level_fraction, 0.0, 1.0));
                    const double cos_f = std::cos(friction);
                    const double sin_f = std::sin(friction);
                    const double turned_u = u * cos_f - v * sin_f;
                    const double turned_v = u * sin_f + v * cos_f;

                    // Jet-band bias: a mid-latitude (30-60 deg) eastward boost that grows with
                    // altitude, standing in for the WT `wind_alt_gradient` shape.
                    const double lat_deg = position.latitude_radians * 180.0 / 3.14159265358979323846;
                    const double band = std::exp(-std::pow((std::fabs(lat_deg) - 45.0) / 15.0, 2.0));
                    constexpr double JET_SCALE = 25.0;
                    const double jet_boost = JET_SCALE * band * std::clamp(level_fraction, 0.0, 1.0);

                    return WindSample{turned_u + jet_boost, turned_v};
                }

                /**
                 * @brief Warm/cold front proximity at a point, from every active low's periphery.
                 *
                 * Only `Deepening`/`Mature` lows carry fronts (a filling low's fronts have
                 * occluded/dissipated); the strongest match across all systems wins per type.
                 *
                 * @param position Query point.
                 * @return The maximum warm/cold proximity over every live front.
                 */
                FrontProximity front_proximity(const GeodeticPosition& position) const noexcept
                {
                    FrontProximity result;
                    for (int i = 0; i < state_.system_count; ++i)
                    {
                        const PressureSystem& system = state_.systems[i];
                        if (!system.is_low || system.phase == PressureSystemPhase::Filling)
                            continue;

                        const double lat_c = system.center_latitude_radians;
                        const double cos_lat_c = std::cos(lat_c);
                        double dx = planet_radius_m_ * (position.longitude_radians - system.center_longitude_radians) * cos_lat_c;
                        double dy = planet_radius_m_ * (position.latitude_radians - lat_c);

                        const double hemisphere = lat_c >= 0.0 ? 1.0 : -1.0;
                        constexpr double WARM_OFFSET_RADIANS = 1.0472;  // 60 degrees
                        constexpr double COLD_OFFSET_RADIANS = 2.35619; // 135 degrees
                        const double length = system.radius_major_m * 1.6;
                        constexpr double FRONT_WIDTH_M = 60000.0;

                        result.warm = std::max(result.warm,
                            front_ray_proximity(dx, dy, system.heading_radians + hemisphere * WARM_OFFSET_RADIANS,
                                                length, FRONT_WIDTH_M));
                        result.cold = std::max(result.cold,
                            front_ray_proximity(dx, dy, system.heading_radians - hemisphere * COLD_OFFSET_RADIANS,
                                                length, FRONT_WIDTH_M));
                    }
                    return result;
                }

                /**
                 * @brief Adds a system directly, bypassing genesis (editor authoring: "place a low").
                 * @param system The fully-specified system to add.
                 * @return Whether it was added (false if @ref MAX_SYNOPTIC_SYSTEMS is already live).
                 */
                bool add_system(PressureSystem system) noexcept
                {
                    if (state_.system_count >= MAX_SYNOPTIC_SYSTEMS)
                        return false;
                    if (system.id == 0)
                        system.id = state_.next_system_id++;
                    else
                        state_.next_system_id = std::max(state_.next_system_id, system.id + 1);
                    state_.systems[state_.system_count++] = system;
                    return true;
                }

                /**
                 * @brief Removes the system at @p index (editor authoring), if it exists.
                 * @param index Index into @ref state()'s @ref SynopticState::systems, `[0, system_count)`.
                 */
                void remove_system(int index) noexcept
                {
                    if (index < 0 || index >= state_.system_count)
                        return;
                    for (int i = index; i + 1 < state_.system_count; ++i)
                        state_.systems[i] = state_.systems[i + 1];
                    --state_.system_count;
                }

                /**
                 * @brief Overwrites the system at @p index (editor authoring: drag/edit a low).
                 * @param index  Index into @ref state()'s @ref SynopticState::systems; a no-op if out of range.
                 * @param system The replacement system (its `id` is kept from the original).
                 */
                void set_system(int index, PressureSystem system) noexcept
                {
                    if (index < 0 || index >= state_.system_count)
                        return;
                    system.id = state_.systems[index].id;
                    state_.systems[index] = system;
                }

                /**
                 * @brief Removes every live system without disturbing the RNG stream or clock.
                 *
                 * The editor's preset buttons call this before seeding a named scenario
                 * (`ProceduralWeather::apply_preset`) — genesis keeps drawing from the same
                 * seeded sequence afterward, so a preset click is not itself a determinism
                 * reset point, only a state edit like any other.
                 */
                void clear_systems() noexcept { state_.system_count = 0; }

            private:
                double system_anomaly(const PressureSystem& system, const GeodeticPosition& position) const noexcept
                {
                    const double cos_lat_c = std::cos(system.center_latitude_radians);
                    const double dx = planet_radius_m_ * (position.longitude_radians - system.center_longitude_radians) * cos_lat_c;
                    const double dy = planet_radius_m_ * (position.latitude_radians - system.center_latitude_radians);
                    const double c = std::cos(-system.orientation_radians);
                    const double s = std::sin(-system.orientation_radians);
                    const double rx = dx * c - dy * s;
                    const double ry = dx * s + dy * c;
                    const double ra = std::max(system.radius_major_m, 1.0);
                    const double rb = std::max(system.radius_minor_m, 1.0);
                    const double gaussian = std::exp(-0.5 * (rx * rx / (ra * ra) + ry * ry / (rb * rb)));
                    const double sign = system.is_low ? -1.0 : 1.0;
                    return sign * system.central_anomaly_hpa * life_scale(system) * gaussian;
                }

                static double life_scale(const PressureSystem& system) noexcept
                {
                    switch (system.phase)
                    {
                    case PressureSystemPhase::Deepening:
                        return system.deepen_seconds > 0.0
                                   ? std::clamp(system.age_seconds / system.deepen_seconds, 0.0, 1.0)
                                   : 1.0;
                    case PressureSystemPhase::Mature:
                        return 1.0;
                    case PressureSystemPhase::Filling:
                    default:
                    {
                        const double into_fill = system.age_seconds - system.deepen_seconds - system.mature_seconds;
                        return system.fill_seconds > 0.0
                                   ? std::clamp(1.0 - into_fill / system.fill_seconds, 0.0, 1.0)
                                   : 0.0;
                    }
                    }
                }

                // Analytic gradient of the summed anomaly field, in an East-North frame at
                // `position` — closed form from the same Gaussian `system_anomaly` evaluates,
                // rather than a finite-difference probe, so it stays exact and needs no epsilon
                // tuning.
                std::pair<double, double> pressure_gradient(const GeodeticPosition& position) const noexcept
                {
                    double grad_east = 0.0;
                    double grad_north = 0.0;
                    for (int i = 0; i < state_.system_count; ++i)
                    {
                        const PressureSystem& system = state_.systems[i];
                        const double cos_lat_c = std::cos(system.center_latitude_radians);
                        const double dx = planet_radius_m_ * (position.longitude_radians - system.center_longitude_radians) * cos_lat_c;
                        const double dy = planet_radius_m_ * (position.latitude_radians - system.center_latitude_radians);
                        const double c = std::cos(-system.orientation_radians);
                        const double s = std::sin(-system.orientation_radians);
                        const double rx = dx * c - dy * s;
                        const double ry = dx * s + dy * c;
                        const double ra = std::max(system.radius_major_m, 1.0);
                        const double rb = std::max(system.radius_minor_m, 1.0);
                        const double gaussian = std::exp(-0.5 * (rx * rx / (ra * ra) + ry * ry / (rb * rb)));
                        const double sign = system.is_low ? -1.0 : 1.0;
                        const double amplitude = sign * system.central_anomaly_hpa * life_scale(system) * gaussian;
                        // d(gaussian)/d(rx,ry) = gaussian * (-rx/ra^2, -ry/rb^2); rotate back to East-North.
                        const double drx = -rx / (ra * ra);
                        const double dry = -ry / (rb * rb);
                        const double d_east = drx * c + dry * s;   // inverse rotation (transpose)
                        const double d_north = -drx * s + dry * c;
                        grad_east += amplitude * d_east;
                        grad_north += amplitude * d_north;
                    }
                    return {grad_east, grad_north};
                }

                static float front_ray_proximity(double dx, double dy, double heading_radians,
                                                  double length_m, double width_m) noexcept
                {
                    // Heading 0 = north (+dy), increasing clockwise toward east (+dx).
                    const double dir_x = std::sin(heading_radians);
                    const double dir_y = std::cos(heading_radians);
                    const double along = dx * dir_x + dy * dir_y;
                    const double across = -dx * dir_y + dy * dir_x;
                    if (along < 0.0 || along > length_m)
                    {
                        const double end_dist = along < 0.0 ? std::sqrt(dx * dx + dy * dy)
                                                             : std::sqrt(across * across + (along - length_m) * (along - length_m));
                        return static_cast<float>(std::exp(-0.5 * (end_dist * end_dist) / (width_m * width_m)));
                    }
                    return static_cast<float>(std::exp(-0.5 * (across * across) / (width_m * width_m)));
                }

                void advance_system(PressureSystem& system, double dt_seconds) noexcept
                {
                    system.age_seconds += dt_seconds;
                    if (system.phase == PressureSystemPhase::Deepening &&
                        system.age_seconds >= system.deepen_seconds)
                        system.phase = PressureSystemPhase::Mature;
                    if (system.phase == PressureSystemPhase::Mature &&
                        system.age_seconds >= system.deepen_seconds + system.mature_seconds)
                        system.phase = PressureSystemPhase::Filling;

                    system.heading_radians += system.curvature_radians_per_second * dt_seconds;
                    const double dir_x = std::sin(system.heading_radians);
                    const double dir_y = std::cos(system.heading_radians);
                    constexpr double MAX_ABS_LAT_RADIANS = 1.48353; // ~85 degrees; keeps 1/cos(lat) bounded.
                    system.center_latitude_radians = std::clamp(
                        system.center_latitude_radians + (system.speed_mps * dir_y / planet_radius_m_) * dt_seconds,
                        -MAX_ABS_LAT_RADIANS, MAX_ABS_LAT_RADIANS);
                    const double cos_lat = std::max(std::cos(system.center_latitude_radians), 0.05);
                    system.center_longitude_radians +=
                        (system.speed_mps * dir_x / (planet_radius_m_ * cos_lat)) * dt_seconds;
                }

                double draw_genesis_interval() noexcept
                {
                    // Uniform in [MIN, MAX) hours, converted to seconds — a new system is due
                    // roughly this often (climate-prior weighting happens at spawn, not here).
                    constexpr double MIN_HOURS = 6.0;
                    constexpr double MAX_HOURS = 18.0;
                    const double hours = MIN_HOURS + Loop::next_unit(state_.rng) * (MAX_HOURS - MIN_HOURS);
                    return hours * 3600.0;
                }

                PressureSystem spawn_system(double julian_date) noexcept
                {
                    PressureSystem system;
                    system.id = state_.next_system_id++;

                    // Climate prior: genesis is weighted toward the mid-latitude westerlies
                    // (~30-60 deg) in whichever hemisphere is currently in its winter half of the
                    // year (more baroclinic instability), a coarse stand-in for the design doc's
                    // "latitude bands, season from the ephemeris date; ITCZ, mid-latitude
                    // westerlies, polar easterlies as prior fields".
                    const double day_of_year = std::fmod(julian_date - 1721425.5, 365.25);
                    const double season = std::cos(2.0 * 3.14159265358979323846 * (day_of_year - 355.0) / 365.25); // +1 at N-hemisphere winter solstice
                    const bool northern = Loop::next_unit(state_.rng) < (0.5 + 0.25 * season);
                    const double band_center_deg = 30.0 + Loop::next_unit(state_.rng) * 30.0; // 30-60 deg
                    const double lat_deg = northern ? band_center_deg : -band_center_deg;
                    system.center_latitude_radians = lat_deg * 3.14159265358979323846 / 180.0;
                    system.center_longitude_radians = (Loop::next_unit(state_.rng) * 2.0 - 1.0) * 3.14159265358979323846;

                    system.is_low = Loop::next_unit(state_.rng) < 0.75; // lows dominate cloud-bearing genesis
                    // Prevailing westerlies: eastward heading with modest random spread.
                    system.heading_radians = (1.5707963 /*east*/) + (Loop::next_unit(state_.rng) - 0.5) * 1.2;
                    system.curvature_radians_per_second = (Loop::next_unit(state_.rng) - 0.5) * 2.0e-6;
                    system.speed_mps = 8.0 + Loop::next_unit(state_.rng) * 12.0;

                    system.central_anomaly_hpa = (system.is_low ? 8.0 : 6.0) + Loop::next_unit(state_.rng) * 22.0;
                    system.radius_major_m = 400000.0 + Loop::next_unit(state_.rng) * 800000.0;
                    system.radius_minor_m = system.radius_major_m * (0.55 + Loop::next_unit(state_.rng) * 0.35);
                    system.orientation_radians = Loop::next_unit(state_.rng) * 3.14159265358979323846;

                    system.deepen_seconds = (6.0 + Loop::next_unit(state_.rng) * 18.0) * 3600.0;
                    system.mature_seconds = (12.0 + Loop::next_unit(state_.rng) * 36.0) * 3600.0;
                    system.fill_seconds = (12.0 + Loop::next_unit(state_.rng) * 24.0) * 3600.0;
                    system.phase = PressureSystemPhase::Deepening;
                    system.age_seconds = 0.0;
                    return system;
                }

                SynopticState state_{};
                double planet_radius_m_ = 6371000.0;
        };
    } // namespace Simulation
} // namespace SushiEngine
