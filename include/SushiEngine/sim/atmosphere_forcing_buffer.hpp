/**************************************************************************/
/* atmosphere_forcing_buffer.hpp                                          */
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
 * @file atmosphere_forcing_buffer.hpp
 * @brief Sim-side storage behind `Render::AtmosphereForcing`, and how it is filled.
 *
 * The parent half of Davies nesting (`docs/slop/atmosphere_system.md` §6): the regional nest is
 * a window onto a larger atmosphere it does not simulate, and its lateral boundary relaxes
 * toward whatever this publishes. Owned here for the same reason `WeatherFieldBuffer` is —
 * `Environment` is copied per frame and the payload changes on the nest's own multi-second
 * cadence, so the view across the seam borrows rather than copies.
 *
 * **The parent is now a dynamical core, and every field here is read rather than shaped.** The
 * wind is the core's own geostrophic flow around lows nothing placed; the temperature and
 * moisture anomalies are its eddy fields, taken about the zonal mean because the mean state is
 * what the nest's base profile already carries; and the vertical motion is the
 * quasi-geostrophic omega, diagnosed from the core's vorticity budget. Three interim
 * constructions went out with the analytic layer: a stylized warm/cold sector mask driving the
 * anomalies, four constants describing how strong that sector was, and an Ekman-pumping
 * estimate of the vertical motion built out of the analytic wind's friction turn because
 * nothing better existed. The core supplies all three directly.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>
#include <SushiEngine/render/atmosphere_nest.hpp>
#include <SushiEngine/sim/weather_types.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief Owns the samples a `Render::AtmosphereForcing` points at, and fills them.
         *
         * One buffer lives as long as the provider publishing into it, so the borrowed pointer
         * in `Environment` is valid for as long as the environment can be read.
         */
        class AtmosphereForcingBuffer
        {
            public:
                /**
                 * @brief Samples the global core over the nest's footprint.
                 *
                 * @param core            T1, sampled for wind, eddy anomalies and vertical motion.
                 * @param observer        Where the nest is centred, geodetic.
                 * @param planet_radius_m The body's mean radius, metres.
                 * @param span_meters     The nest's horizontal span, metres.
                 * @param cells           Lattice cells per axis; clamped to the render cap.
                 */
                void fill(const Atmosphere::QuasiGeostrophicCore& core,
                          const GeodeticPosition& observer, double planet_radius_m,
                          double span_meters, int cells)
                {
                    cells_ = std::clamp(cells, 1, Render::ATMOSPHERE_FORCING_MAX_CELLS);
                    samples_.resize(std::size_t(cells_) * std::size_t(cells_));

                    const double span = std::max(span_meters, 1.0);
                    const double radius = std::max(planet_radius_m, 1.0);
                    const double cos_latitude =
                        std::max(std::cos(observer.latitude_radians), MIN_COS_LATITUDE);
                    const double step = span / double(cells_);
                    // The nest's own footprint, in the same flat tangent frame the grid and the
                    // cloud field already accept at this scale.
                    const double origin_east = radius * observer.longitude_radians * cos_latitude -
                                               span * 0.5;
                    const double origin_north = radius * observer.latitude_radians - span * 0.5;

                    for (int z = 0; z < cells_; ++z)
                        for (int x = 0; x < cells_; ++x)
                        {
                            const double east = origin_east + (double(x) + 0.5) * step;
                            const double north = origin_north + (double(z) + 0.5) * step;
                            const Atmosphere::GeographicPosition position{
                                north / radius, east / (radius * cos_latitude)};

                            // The near-surface wind: the nest's boundary is where synoptic flow
                            // enters, and that is a lower-tropospheric property.
                            const Atmosphere::Wind wind = core.wind_at(position, BOUNDARY_LEVEL);

                            Render::AtmosphereForcingSample& sample =
                                samples_[std::size_t(z) * std::size_t(cells_) + std::size_t(x)];
                            sample.wind_east_mps = static_cast<float>(wind.eastward_mps);
                            sample.wind_north_mps = static_cast<float>(wind.northward_mps);
                            // Departures from the core's own zonal mean. A front is where these
                            // have a gradient; nothing here draws one, and the nest builds a
                            // front out of the gradient rather than being handed a mask of one.
                            sample.theta_anomaly_k =
                                static_cast<float>(core.thermal_anomaly_at(position));
                            sample.humidity_anomaly =
                                static_cast<float>(core.humidity_anomaly_at(position));
                            sample.vertical_velocity_mps = static_cast<float>(
                                std::clamp(core.vertical_velocity_at(position),
                                           -MAX_VERTICAL_MPS, MAX_VERTICAL_MPS));
                        }

                    uv_scale_ = 1.0 / span;
                    // Scene-absolute metres relative to the observer, so the mapping matches
                    // `WeatherField`'s convention exactly: +X east, +Z north, and the renderer
                    // folds the observer into the offset in double.
                    uv_offset_ = 0.5;
                    ++revision_;
                }

                /**
                 * @brief A borrowed view of the current contents, for `Environment`.
                 * @param observer_x     Scene-absolute X of the nest's centre, metres.
                 * @param observer_z     Scene-absolute Z of the nest's centre, metres.
                 * @param total_seconds  Total game seconds simulated, monotonic.
                 * @param coriolis       `f = 2 Omega sin(latitude)` at the nest centre, 1/s.
                 * @param solar_elevation_sine Sine of the rendered sun's elevation; negative at night.
                 */
                Render::AtmosphereForcing view(double observer_x, double observer_z,
                                               double total_seconds, float coriolis,
                                               float solar_elevation_sine) const noexcept
                {
                    Render::AtmosphereForcing forcing{};
                    if (samples_.empty())
                        return forcing;
                    forcing.samples = samples_.data();
                    forcing.cells_x = cells_;
                    forcing.cells_z = cells_;
                    forcing.revision = revision_;
                    forcing.uv_scale_x = static_cast<float>(uv_scale_);
                    forcing.uv_scale_z = static_cast<float>(uv_scale_);
                    // The offset is stated at world zero, as `WeatherField` states its own: the
                    // lattice is centred on the observer, so subtracting the observer's own
                    // scaled position is what moves the centre from the origin to where it is.
                    forcing.uv_offset_x =
                        static_cast<float>(uv_offset_ - uv_scale_ * observer_x);
                    forcing.uv_offset_z =
                        static_cast<float>(uv_offset_ - uv_scale_ * observer_z);
                    forcing.total_seconds = total_seconds;
                    forcing.coriolis = coriolis;
                    forcing.solar_elevation_sine = solar_elevation_sine;
                    forcing.observer_x = observer_x;
                    forcing.observer_z = observer_z;
                    return forcing;
                }

            private:
                // Bounds the plate-carree easting factor at the poles, exactly as
                // `WeatherFieldBuffer` does and for the same reason.
                static constexpr double MIN_COS_LATITUDE = 0.05;

                /**
                 * @brief Level fraction the boundary wind is read at.
                 *
                 * Low in the column, because the boundary the nest relaxes toward is where
                 * synoptic air actually enters it, and a jet-level wind would blow the nest's
                 * lateral zone at twice the speed of the air under it.
                 */
                static constexpr double BOUNDARY_LEVEL = 0.25;

                /**
                 * @brief Cap on the vertical motion handed across, m/s.
                 *
                 * Ten centimetres a second is already extreme for a synoptic system. The core's
                 * omega is a diagnosed quantity rather than a prognostic one, so a transient in
                 * the vorticity tendency can spike it for a step; the nest applies this term
                 * across its whole domain and would carry that spike into every column.
                 */
                static constexpr double MAX_VERTICAL_MPS = 0.10;

                std::vector<Render::AtmosphereForcingSample> samples_;
                int cells_ = 0;
                double uv_scale_ = 0.0;
                double uv_offset_ = 0.5;
                std::uint32_t revision_ = 0;
        };
    } // namespace Simulation
} // namespace SushiEngine
