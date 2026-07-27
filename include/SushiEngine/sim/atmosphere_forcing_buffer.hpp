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
 * The parent half of Davies nesting (`docs/slop/atmosphere_system.md` §6): the regional nest
 * is a window onto a larger atmosphere it does not simulate, and its lateral boundary relaxes
 * toward whatever this publishes. Owned here for the same reason `WeatherFieldBuffer` is —
 * `Environment` is copied per frame and the payload changes on the nest's own multi-second
 * cadence, so the view across the seam borrows rather than copies.
 *
 * **The parent is the analytic synoptic layer, and that is an interim.** §5 replaces it with a
 * two-layer moist quasi-geostrophic core in phase C, where cyclogenesis is emergent and fronts
 * are diagnosed from the thermal gradient rather than drawn as a ray pair. Until then the wind
 * fed in here is real geostrophic flow around real moving pressure systems — which is enough
 * for the nest to have weather blowing *through* it rather than merely stewing in place — and
 * the temperature and moisture anomalies are shaped by `SynopticLayer::front_proximity`'s
 * stylized frontal mask. Named as an interim rather than presented as nesting.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

#include <SushiEngine/render/atmosphere_nest.hpp>
#include <SushiEngine/sim/synoptic_weather.hpp>
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
                 * @brief Samples the synoptic layer over the nest's footprint.
                 *
                 * @param synoptic        T1, sampled for wind and frontal structure.
                 * @param observer        Where the nest is centred, geodetic.
                 * @param planet_radius_m The body's mean radius, metres.
                 * @param span_meters     The nest's horizontal span, metres.
                 * @param cells           Lattice cells per axis; clamped to the render cap.
                 */
                void fill(const SynopticLayer& synoptic, const GeodeticPosition& observer,
                          double planet_radius_m, double span_meters, int cells)
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
                            const GeodeticPosition position{
                                north / radius, east / (radius * cos_latitude)};

                            // The near-surface wind: the nest's boundary is where synoptic flow
                            // enters, and that is a lower-tropospheric property.
                            const WindSample wind = synoptic.wind_at(position, 0.25);
                            const FrontProximity front = synoptic.front_proximity(position);

                            Render::AtmosphereForcingSample& sample =
                                samples_[std::size_t(z) * std::size_t(cells_) + std::size_t(x)];
                            sample.wind_east_mps = static_cast<float>(wind.eastward_mps);
                            sample.wind_north_mps = static_cast<float>(wind.northward_mps);
                            // A warm sector is warmer and moister than the base state; a cold
                            // one is colder and drier. Shaping the boundary this way is what
                            // gives the nest something to build a front out of -- it does not
                            // draw the front, it lets one form where the gradient is.
                            sample.theta_anomaly_k =
                                static_cast<float>(front.warm * WARM_SECTOR_THETA_K -
                                                   front.cold * COLD_SECTOR_THETA_K);
                            sample.humidity_anomaly =
                                static_cast<float>(front.warm * WARM_SECTOR_HUMIDITY -
                                                   front.cold * COLD_SECTOR_HUMIDITY);
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

                // The interim synoptic coupling's own shaping, deliberately *not* fields of
                // `AtmosphereParameters`: they describe how the analytic front mask is turned
                // into a boundary anomaly, and they go away with that mask when phase C's real
                // quasi-geostrophic core replaces it. A few degrees across a frontal zone and a
                // tenth of the saturation deficit are what the textbook picture of a warm sector
                // looks like.
                static constexpr double WARM_SECTOR_THETA_K = 3.0;
                static constexpr double COLD_SECTOR_THETA_K = 4.5;
                static constexpr double WARM_SECTOR_HUMIDITY = 0.12;
                static constexpr double COLD_SECTOR_HUMIDITY = 0.08;

                std::vector<Render::AtmosphereForcingSample> samples_;
                int cells_ = 0;
                double uv_scale_ = 0.0;
                double uv_offset_ = 0.5;
                std::uint32_t revision_ = 0;
        };
    } // namespace Simulation
} // namespace SushiEngine
