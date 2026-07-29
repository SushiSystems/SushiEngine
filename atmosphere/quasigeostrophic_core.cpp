/**************************************************************************/
/* quasigeostrophic_core.cpp                                              */
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

#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstring>

#include <SushiEngine/atmosphere/fourier_transform.hpp>

namespace SushiEngine
{
    namespace Atmosphere
    {
        namespace
        {
            constexpr double PI = 3.14159265358979323846;

            /** @brief Seconds in an hour, for the millimetres-per-hour precipitation rate. */
            constexpr double SECONDS_PER_HOUR = 3600.0;

            /** @brief Pascals per hectopascal, so the pressure anomaly is reported in hPa. */
            constexpr double PASCALS_PER_HECTOPASCAL = 100.0;

            /**
             * @brief Peak azimuthal wind of a Gaussian vorticity blob, as a fraction of `zeta*R`.
             *
             * A blob `zeta_0 exp(-r^2/2R^2)` has circulation `2*pi*R^2*zeta_0*(1-exp(-r^2/2R^2))`,
             * whose azimuthal wind peaks near `r = 1.585 R` at `0.451 * zeta_0 * R`. Injecting a
             * *wind* rather than a vorticity is what an author means by "a 20 m/s low", so the
             * inverse of this number is how the request is turned into the field.
             */
            constexpr double GAUSSIAN_BLOB_WIND_FACTOR = 0.451;

            /** @brief Departure points are never allowed further than this fraction of the grid. */
            constexpr double MAX_DEPARTURE_CELLS_FRACTION = 0.25;

            bool power_of_two(int value) noexcept
            {
                return value >= 2 && (value & (value - 1)) == 0;
            }

            /**
             * @brief Limits @p value to the range spanned by the two cells it falls between.
             *
             * What makes the semi-Lagrangian transport monotone: a cubic through four points
             * overshoots near a sharp gradient, and an overshoot in a water field is water that
             * did not exist. Clamping to the bracketing pair costs the extra order only where
             * the field is not smooth, which is exactly where the extra order was wrong.
             */
            double limit_between(double value, double low_sample, double high_sample) noexcept
            {
                const double low = std::min(low_sample, high_sample);
                const double high = std::max(low_sample, high_sample);
                return std::min(std::max(value, low), high);
            }

            /** @brief Catmull-Rom through four equally spaced samples at fractional offset @p t. */
            double cubic(double p0, double p1, double p2, double p3, double t) noexcept
            {
                const double a = -0.5 * p0 + 1.5 * p1 - 1.5 * p2 + 0.5 * p3;
                const double b = p0 - 2.5 * p1 + 2.0 * p2 - 0.5 * p3;
                const double c = -0.5 * p0 + 0.5 * p2;
                return ((a * t + b) * t + c) * t + p1;
            }
        } // namespace

        /**
         * @brief Everything the core owns: its grid, its factorization, its fields, its scratch.
         *
         * Behind a pointer so the public header names no container of a container and no
         * complex number, and so the fields can be reshaped without recompiling every consumer.
         */
        struct QuasiGeostrophicCore::State
        {
            QuasiGeostrophicGridSize size{};
            QuasiGeostrophicParameters parameters{};
            bool valid = false;

            int longitude_cells = 0;
            int latitude_cells = 0;
            int cells = 0;
            int modes = 0; /**< Distinct zonal wavenumbers solved for: `longitude_cells/2 + 1`. */
            double delta_longitude = 0.0;
            double delta_latitude = 0.0;

            // Latitude tables. Every one of these is a function of the row alone, and the inner
            // loops read them rather than recomputing a trigonometric function per cell.
            std::vector<double> cosine;      /**< cos(latitude) at the row centre. */
            std::vector<double> coriolis;    /**< f = 2 Omega sin(latitude). */
            std::vector<double> stretch;     /**< S = f^2/(g'H), floored by the deformation cap. */
            std::vector<double> lower_coefficient; /**< Sub-diagonal of the latitude operator. */
            std::vector<double> upper_coefficient; /**< Super-diagonal of the latitude operator. */
            std::vector<double> saturation;  /**< Column water at saturation, kg/m^2. */
            std::vector<double> climatological_streamfunction[2];

            // The elliptic operator's factorization, one row per (wavenumber, latitude). The
            // Thomas sweep's multipliers depend only on the operator, so they are computed once
            // here instead of three times a step for the rest of the core's life.
            std::vector<double> barotropic_multiplier;
            std::vector<double> barotropic_pivot;
            std::vector<double> baroclinic_multiplier;
            std::vector<double> baroclinic_pivot;

            /** @brief `(m / m_cut(latitude))^order`, the zonal filter's shape without its rate. */
            std::vector<double> filter_shape;

            std::vector<double> potential_vorticity[2];
            std::vector<double> streamfunction[2];
            std::vector<double> climatological_vorticity[2];
            std::vector<double> wind_eastward[2];
            std::vector<double> wind_northward[2];
            std::vector<double> water;
            std::vector<double> omega;
            std::vector<double> precipitation;
            /**
             * @brief Departures from the zonal mean, as fields rather than as a mean to subtract.
             *
             * Kept whole so that a query interpolates them exactly the way it interpolates every
             * other field — subtracting a per-row mean at sample time would need that mean
             * interpolated in latitude too, by a second rule that could disagree with the first.
             */
            std::vector<double> thermal_anomaly;
            std::vector<double> humidity_anomaly;
            /**
             * @brief Surface pressure departure from the zonal mean, hPa.
             *
             * Taken about the same reference as the two above, and the reason is worth stating:
             * the lower streamfunction contains the mean westerly jet, whose meridional gradient
             * is worth well over a hundred hectopascals from the tropics to the pole. Reporting
             * that as an "anomaly" would drown every cyclone in the planetary temperature
             * gradient — measured at 139 hPa across a 40-degree window while a deep low is
             * perhaps 30 — and would make the editor's map a picture of latitude.
             */
            std::vector<double> pressure_anomaly;
            /** @brief Magnitude of the total thermal gradient, K per 100 km — frontal zones. */
            std::vector<double> frontal_strength;

            std::vector<double> initial_vorticity[2];
            std::vector<double> stage_vorticity[2];
            std::vector<double> tendency[2];
            std::vector<double> barotropic;
            std::vector<double> baroclinic;
            std::vector<double> upper_relative_vorticity;
            std::vector<double> scratch;
            std::vector<std::complex<double>> first_spectrum;
            std::vector<std::complex<double>> second_spectrum;
            std::vector<std::complex<double>> transform_work;
            std::vector<std::complex<double>> column;

            FourierTransform transform;
            Loop::RngState rng{};
            double simulated_seconds = 0.0;
            double pending_seconds = 0.0;
            std::uint64_t steps = 0;

            State(const QuasiGeostrophicGridSize& grid, const QuasiGeostrophicParameters& physics)
                : size(grid), parameters(physics), transform(grid.longitude_cells)
            {
                if (!power_of_two(grid.longitude_cells) || grid.latitude_cells < 4 ||
                    (grid.latitude_cells % 2) != 0 || !transform.valid())
                    return;

                longitude_cells = grid.longitude_cells;
                latitude_cells = grid.latitude_cells;
                cells = longitude_cells * latitude_cells;
                modes = longitude_cells / 2 + 1;
                delta_longitude = 2.0 * PI / double(longitude_cells);
                delta_latitude = PI / double(latitude_cells);

                build_latitude_tables();
                build_factorization();
                build_filter();

                for (int layer = 0; layer < 2; ++layer)
                {
                    potential_vorticity[layer].assign(std::size_t(cells), 0.0);
                    streamfunction[layer].assign(std::size_t(cells), 0.0);
                    climatological_vorticity[layer].assign(std::size_t(cells), 0.0);
                    wind_eastward[layer].assign(std::size_t(cells), 0.0);
                    wind_northward[layer].assign(std::size_t(cells), 0.0);
                    initial_vorticity[layer].assign(std::size_t(cells), 0.0);
                    stage_vorticity[layer].assign(std::size_t(cells), 0.0);
                    tendency[layer].assign(std::size_t(cells), 0.0);
                }
                water.assign(std::size_t(cells), 0.0);
                omega.assign(std::size_t(cells), 0.0);
                precipitation.assign(std::size_t(cells), 0.0);
                thermal_anomaly.assign(std::size_t(cells), 0.0);
                humidity_anomaly.assign(std::size_t(cells), 0.0);
                pressure_anomaly.assign(std::size_t(cells), 0.0);
                frontal_strength.assign(std::size_t(cells), 0.0);
                barotropic.assign(std::size_t(cells), 0.0);
                baroclinic.assign(std::size_t(cells), 0.0);
                upper_relative_vorticity.assign(std::size_t(cells), 0.0);
                scratch.assign(std::size_t(cells), 0.0);
                first_spectrum.assign(std::size_t(cells), {});
                second_spectrum.assign(std::size_t(cells), {});
                transform_work.assign(std::size_t(longitude_cells), {});
                column.assign(std::size_t(latitude_cells), {});

                build_climatology();
                valid = true;
            }

            int index(int row, int column_index) const noexcept
            {
                return row * longitude_cells + column_index;
            }

            /**
             * @brief Folds a longitude index into range.
             *
             * The in-range test comes first because almost every call is in range: the advective
             * stencils reach one or two cells and only the two ends of a row leave it, and the
             * Jacobian alone makes sixteen of these per cell per stage. The general fold behind
             * it is still needed — a semi-Lagrangian departure point may be a quarter of the
             * globe away.
             */
            int wrap_longitude(int i) const noexcept
            {
                if (i >= 0 && i < longitude_cells)
                    return i;
                i %= longitude_cells;
                return i < 0 ? i + longitude_cells : i;
            }

            /**
             * @brief Reads @p field at (@p row, @p i), crossing the pole where the row runs out.
             *
             * A grid point beyond the pole is a real grid point on the far side, half a
             * revolution away in longitude — and because the grid has an even number of
             * longitudes, "half a revolution" is an exact index shift rather than an
             * interpolation. This is the whole of the polar boundary treatment for the
             * advective stencils; the elliptic operator needs none at all, because its polar
             * cell edge carries `cos(latitude) = 0` and the flux across it is zero by
             * construction.
             */
            double across_pole(const double* field, int row, int i) const noexcept
            {
                if (row < 0)
                {
                    row = -1 - row;
                    i += longitude_cells / 2;
                }
                else if (row >= latitude_cells)
                {
                    row = 2 * latitude_cells - 1 - row;
                    i += longitude_cells / 2;
                }
                return field[index(row, wrap_longitude(i))];
            }

            void build_latitude_tables()
            {
                cosine.resize(std::size_t(latitude_cells));
                coriolis.resize(std::size_t(latitude_cells));
                stretch.resize(std::size_t(latitude_cells));
                lower_coefficient.resize(std::size_t(latitude_cells));
                upper_coefficient.resize(std::size_t(latitude_cells));
                saturation.resize(std::size_t(latitude_cells));

                const double radius = parameters.planet_radius_m;
                const double stretch_floor =
                    1.0 / (parameters.maximum_deformation_radius_m *
                           parameters.maximum_deformation_radius_m);
                const double stratification =
                    parameters.reduced_gravity_mps2 * parameters.layer_depth_m;

                // Cell edges, including both poles. The polar edges are forced to exactly zero
                // rather than left at the ~1e-17 that cos(pi/2) returns, so "no flux across the
                // pole" is exact and the first and last rows need no special case anywhere.
                std::vector<double> edge_cosine(std::size_t(latitude_cells + 1));
                for (int j = 0; j <= latitude_cells; ++j)
                    edge_cosine[std::size_t(j)] =
                        std::cos(-0.5 * PI + double(j) * delta_latitude);
                edge_cosine.front() = 0.0;
                edge_cosine.back() = 0.0;

                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double latitude = -0.5 * PI + (double(j) + 0.5) * delta_latitude;
                    const double cos_latitude = std::cos(latitude);
                    cosine[std::size_t(j)] = cos_latitude;
                    const double f =
                        2.0 * parameters.angular_velocity_rad_per_s * std::sin(latitude);
                    coriolis[std::size_t(j)] = f;
                    stretch[std::size_t(j)] =
                        std::max(f * f / stratification, stretch_floor);

                    const double scale =
                        1.0 / (radius * radius * cos_latitude * delta_latitude * delta_latitude);
                    lower_coefficient[std::size_t(j)] = edge_cosine[std::size_t(j)] * scale;
                    upper_coefficient[std::size_t(j)] = edge_cosine[std::size_t(j + 1)] * scale;

                    // A latitudinal surface temperature, then Clausius-Clapeyron as a single
                    // e-folding. T0 proper (§4) replaces this with monthly precipitable water;
                    // until then this is the analytic latitude band §4 degrades to.
                    const double sine = std::sin(latitude);
                    const double cooling = parameters.equator_to_pole_kelvin * sine * sine;
                    saturation[std::size_t(j)] =
                        parameters.equatorial_saturation_kg_per_m2 *
                        std::exp(-cooling / parameters.saturation_lapse_kelvin);
                }
            }

            void build_factorization()
            {
                const std::size_t entries = std::size_t(modes) * std::size_t(latitude_cells);
                barotropic_multiplier.resize(entries);
                barotropic_pivot.resize(entries);
                baroclinic_multiplier.resize(entries);
                baroclinic_pivot.resize(entries);

                const double radius = parameters.planet_radius_m;
                for (int m = 0; m < modes; ++m)
                {
                    // The *discrete* eigenvalue of the three-point zonal second difference, not
                    // its continuous limit -m^2. The Jacobian, the Ekman drag and the vertical
                    // velocity all read a vorticity that this inversion produced, so the two
                    // spellings of the Laplacian have to be the same operator or those three
                    // are reading a field that does not quite satisfy the equation they assume.
                    const double zonal = (2.0 * std::cos(double(m) * delta_longitude) - 2.0) /
                                         (delta_longitude * delta_longitude);

                    for (int part = 0; part < 2; ++part)
                    {
                        std::vector<double>& multiplier =
                            part == 0 ? barotropic_multiplier : baroclinic_multiplier;
                        std::vector<double>& pivot = part == 0 ? barotropic_pivot : baroclinic_pivot;

                        double previous_multiplier = 0.0;
                        for (int j = 0; j < latitude_cells; ++j)
                        {
                            const std::size_t slot =
                                std::size_t(m) * std::size_t(latitude_cells) + std::size_t(j);
                            const double cos_latitude = cosine[std::size_t(j)];
                            const double lower = lower_coefficient[std::size_t(j)];
                            const double upper = upper_coefficient[std::size_t(j)];

                            double diagonal =
                                -(lower + upper) +
                                zonal / (radius * radius * cos_latitude * cos_latitude);
                            if (part == 1)
                                diagonal -= 2.0 * stretch[std::size_t(j)];

                            double super = upper;
                            double sub = lower;
                            // The barotropic operator at wavenumber zero is the pure Laplacian,
                            // which is singular: adding a constant to the streamfunction changes
                            // nothing. Pinning the southernmost row makes it solvable, and the
                            // gauge that pin implies is undone afterwards by removing the field's
                            // area-weighted mean — so "anomaly" keeps meaning anomaly.
                            if (part == 0 && m == 0 && j == 0)
                            {
                                diagonal = 1.0;
                                super = 0.0;
                                sub = 0.0;
                            }

                            const double denominator = diagonal - sub * previous_multiplier;
                            pivot[slot] = 1.0 / denominator;
                            previous_multiplier = super * pivot[slot];
                            multiplier[slot] = previous_multiplier;
                        }
                    }
                }
            }

            void build_filter()
            {
                filter_shape.assign(std::size_t(modes) * std::size_t(latitude_cells), 0.0);
                const double reference =
                    std::max(std::cos(parameters.filter_reference_latitude_radians), 1e-3);
                const int order = std::max(2, parameters.filter_order);
                const double highest = double(longitude_cells / 2);

                for (int j = 0; j < latitude_cells; ++j)
                {
                    // Poleward of the reference latitude the zonal spacing has shrunk by
                    // cos(latitude), and the cut-off wavenumber shrinks with it. Equatorward it
                    // is clamped to the grid's own highest wavenumber, so away from the poles
                    // this filter is purely the grid-scale enstrophy sink and touches nothing
                    // the flow actually resolves.
                    const double cut = std::max(1.0, std::min(highest, highest * cosine[std::size_t(j)] / reference));
                    for (int m = 0; m < modes; ++m)
                    {
                        const double ratio = double(m) / cut;
                        double shape = 0.0;
                        if (ratio > 4.0)
                            shape = 1e30; // Beyond here the mode is removed outright.
                        else
                        {
                            shape = 1.0;
                            for (int power = 0; power < order; ++power)
                                shape *= ratio;
                        }
                        filter_shape[std::size_t(m) * std::size_t(latitude_cells) +
                                     std::size_t(j)] = shape;
                    }
                }
            }

            /**
             * @brief Builds the climatological jet and the potential vorticity it implies.
             *
             * The mean state is prescribed as a zonal wind per layer, not as a streamfunction,
             * because the wind is the thing with a physical magnitude to check and the shear
             * between the layers is what decides whether the state is unstable at all.
             */
            void build_climatology()
            {
                const double radius = parameters.planet_radius_m;
                for (int layer = 0; layer < 2; ++layer)
                {
                    climatological_streamfunction[layer].assign(std::size_t(latitude_cells), 0.0);
                    const double speed = layer == 0 ? parameters.upper_jet_speed_mps
                                                    : parameters.lower_jet_speed_mps;
                    // psi = -a * integral(u dlatitude), accumulated from the southern pole.
                    double accumulated = 0.0;
                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        const double latitude = -0.5 * PI + (double(j) + 0.5) * delta_latitude;
                        const double north =
                            (latitude - parameters.jet_latitude_radians) / parameters.jet_width_radians;
                        const double south =
                            (latitude + parameters.jet_latitude_radians) / parameters.jet_width_radians;
                        const double wind =
                            speed * (std::exp(-north * north) + std::exp(-south * south));
                        accumulated -= radius * wind * delta_latitude;
                        climatological_streamfunction[layer][std::size_t(j)] = accumulated;
                    }
                    // The gauge again: a constant in the streamfunction is not a wind, and
                    // leaving the integration's arbitrary offset in would show up as a global
                    // pressure bias the moment anyone asked for an anomaly.
                    double mean = 0.0;
                    double weight = 0.0;
                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        mean += cosine[std::size_t(j)] *
                                climatological_streamfunction[layer][std::size_t(j)];
                        weight += cosine[std::size_t(j)];
                    }
                    mean /= weight;
                    for (int j = 0; j < latitude_cells; ++j)
                        climatological_streamfunction[layer][std::size_t(j)] -= mean;
                }

                for (int layer = 0; layer < 2; ++layer)
                {
                    double* target = climatological_vorticity[layer].data();
                    const double* upper = climatological_streamfunction[0].data();
                    const double* lower = climatological_streamfunction[1].data();
                    const double sign = layer == 0 ? -1.0 : 1.0;
                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        const double* profile = layer == 0 ? upper : lower;
                        const double south =
                            j > 0 ? profile[j - 1] : profile[0]; // zero flux across the pole
                        const double north =
                            j + 1 < latitude_cells ? profile[j + 1] : profile[latitude_cells - 1];
                        const double laplacian =
                            upper_coefficient[std::size_t(j)] * (north - profile[j]) -
                            lower_coefficient[std::size_t(j)] * (profile[j] - south);
                        const double value = laplacian + coriolis[std::size_t(j)] +
                                             sign * stretch[std::size_t(j)] *
                                                 (upper[j] - lower[j]);
                        for (int i = 0; i < longitude_cells; ++i)
                            target[index(j, i)] = value;
                    }
                }
            }

            /**
             * @brief Recovers both streamfunctions from both potential vorticities.
             *
             * The barotropic and baroclinic combinations decouple exactly, because the layer
             * coupling enters the two equations with the same coefficient and opposite signs.
             * Each decoupled problem is then diagonal in zonal wavenumber and tridiagonal in
             * latitude, which is the whole reason this tier costs what it costs.
             */
            void invert(const std::vector<double> source[2], std::vector<double> result[2])
            {
                const double* upper = source[0].data();
                const double* lower = source[1].data();
                double* sum = barotropic.data();
                double* difference = baroclinic.data();

                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double twice_f = 2.0 * coriolis[std::size_t(j)];
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        sum[c] = upper[c] + lower[c] - twice_f;
                        difference[c] = upper[c] - lower[c];
                    }
                }

                // The Laplacian's range excludes constants, so its right-hand side has to have
                // no area-weighted mean or the problem has no solution at all. Removing it is
                // not a correction to the data — it is the projection onto the range, and what
                // is removed is the rounding-level residue of a field that sums to zero exactly.
                double mean = 0.0;
                double weight = 0.0;
                for (int j = 0; j < latitude_cells; ++j)
                {
                    double row_total = 0.0;
                    for (int i = 0; i < longitude_cells; ++i)
                        row_total += sum[index(j, i)];
                    mean += cosine[std::size_t(j)] * row_total;
                    weight += cosine[std::size_t(j)] * double(longitude_cells);
                }
                mean /= weight;
                for (int c = 0; c < cells; ++c)
                    sum[c] -= mean;

                for (int j = 0; j < latitude_cells; ++j)
                    transform.forward_real_pair(sum + j * longitude_cells,
                                                difference + j * longitude_cells,
                                                first_spectrum.data() + j * longitude_cells,
                                                second_spectrum.data() + j * longitude_cells);

                solve_modes(first_spectrum.data(), barotropic_multiplier, barotropic_pivot, true);
                solve_modes(second_spectrum.data(), baroclinic_multiplier, baroclinic_pivot, false);

                for (int j = 0; j < latitude_cells; ++j)
                    transform.inverse_real_pair(first_spectrum.data() + j * longitude_cells,
                                                second_spectrum.data() + j * longitude_cells,
                                                sum + j * longitude_cells,
                                                difference + j * longitude_cells,
                                                transform_work.data());

                // Undo the pinned row's arbitrary gauge.
                double offset = 0.0;
                for (int j = 0; j < latitude_cells; ++j)
                {
                    double row_total = 0.0;
                    for (int i = 0; i < longitude_cells; ++i)
                        row_total += sum[index(j, i)];
                    offset += cosine[std::size_t(j)] * row_total;
                }
                offset /= weight;

                double* first = result[0].data();
                double* second = result[1].data();
                for (int c = 0; c < cells; ++c)
                {
                    const double mean_part = 0.5 * (sum[c] - offset);
                    const double shear_part = 0.5 * difference[c];
                    first[c] = mean_part + shear_part;
                    second[c] = mean_part - shear_part;
                }
            }

            /**
             * @brief Runs the pre-factored tridiagonal sweep for every zonal wavenumber.
             *
             * @param spectrum   Row-major spectra, overwritten with the solution's spectra.
             * @param multiplier The factorization's super-diagonal multipliers.
             * @param pivot      The factorization's reciprocal pivots.
             * @param pinned     Whether wavenumber zero's southernmost row was pinned.
             */
            void solve_modes(std::complex<double>* spectrum, const std::vector<double>& multiplier,
                             const std::vector<double>& pivot, bool pinned)
            {
                for (int m = 0; m < modes; ++m)
                {
                    const std::size_t base = std::size_t(m) * std::size_t(latitude_cells);
                    std::complex<double>* work = column.data();

                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        std::complex<double> value = spectrum[index(j, m)];
                        if (pinned && m == 0 && j == 0)
                            value = std::complex<double>(0.0, 0.0);
                        const double sub = (pinned && m == 0 && j == 0)
                                               ? 0.0
                                               : lower_coefficient[std::size_t(j)];
                        const std::complex<double> previous =
                            j > 0 ? work[j - 1] : std::complex<double>(0.0, 0.0);
                        work[j] = (value - sub * previous) * pivot[base + std::size_t(j)];
                    }
                    for (int j = latitude_cells - 2; j >= 0; --j)
                        work[j] -= multiplier[base + std::size_t(j)] * work[j + 1];

                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        spectrum[index(j, m)] = work[j];
                        // The fields are real, so the modes above the Nyquist wavenumber are the
                        // conjugates of the ones below it. Solving them again would produce the
                        // same numbers at twice the cost.
                        const int mirror = (longitude_cells - m) % longitude_cells;
                        if (mirror != m)
                            spectrum[index(j, mirror)] = std::conj(work[j]);
                    }
                }
            }

            /**
             * @brief The Arakawa Jacobian of @p flow against @p carried, on the sphere.
             *
             * The average of the three second-order Jacobians, which is the discretization that
             * conserves the domain integrals of the advected quantity, of its square, and of the
             * energy — the properties that let a two-dimensional flow be integrated for
             * simulated months without its enstrophy drifting into the truncation error.
             */
            void jacobian(const double* flow, const double* carried, double* result) const
            {
                const double scale = 1.0 / (4.0 * delta_longitude * delta_latitude);
                const double radius = parameters.planet_radius_m;

                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double metric =
                        scale / (radius * radius * cosine[std::size_t(j)]);
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int east = wrap_longitude(i + 1);
                        const int west = wrap_longitude(i - 1);

                        const double flow_e = flow[index(j, east)];
                        const double flow_w = flow[index(j, west)];
                        const double flow_n = across_pole(flow, j + 1, i);
                        const double flow_s = across_pole(flow, j - 1, i);
                        const double flow_ne = across_pole(flow, j + 1, i + 1);
                        const double flow_nw = across_pole(flow, j + 1, i - 1);
                        const double flow_se = across_pole(flow, j - 1, i + 1);
                        const double flow_sw = across_pole(flow, j - 1, i - 1);

                        const double carried_e = carried[index(j, east)];
                        const double carried_w = carried[index(j, west)];
                        const double carried_n = across_pole(carried, j + 1, i);
                        const double carried_s = across_pole(carried, j - 1, i);
                        const double carried_ne = across_pole(carried, j + 1, i + 1);
                        const double carried_nw = across_pole(carried, j + 1, i - 1);
                        const double carried_se = across_pole(carried, j - 1, i + 1);
                        const double carried_sw = across_pole(carried, j - 1, i - 1);

                        const double j1 = (flow_e - flow_w) * (carried_n - carried_s) -
                                          (flow_n - flow_s) * (carried_e - carried_w);
                        const double j2 = flow_e * (carried_ne - carried_se) -
                                          flow_w * (carried_nw - carried_sw) -
                                          flow_n * (carried_ne - carried_nw) +
                                          flow_s * (carried_se - carried_sw);
                        const double j3 = flow_ne * (carried_n - carried_e) -
                                          flow_sw * (carried_w - carried_s) -
                                          flow_nw * (carried_n - carried_w) +
                                          flow_se * (carried_e - carried_s);

                        result[index(j, i)] = metric * (j1 + j2 + j3) / 3.0;
                    }
                }
            }

            /** @brief Relative vorticity of a layer, from the potential vorticity it came with. */
            void relative_vorticity(const std::vector<double> vorticity[2],
                                    const std::vector<double> psi[2], int layer,
                                    double* result) const
            {
                const double* source = vorticity[layer].data();
                const double* upper = psi[0].data();
                const double* lower = psi[1].data();
                const double sign = layer == 0 ? 1.0 : -1.0;
                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double f = coriolis[std::size_t(j)];
                    const double s = stretch[std::size_t(j)];
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        result[c] = source[c] - f + sign * s * (upper[c] - lower[c]);
                    }
                }
            }

            /**
             * @brief The right-hand side of both potential vorticity equations.
             *
             * Advection, relaxation toward the climatological state, Ekman drag on the lower
             * layer, and a meridional fourth-order damping. There is no beta term: the
             * planetary vorticity is inside the advected quantity, which is where it belongs
             * and where it cannot be inconsistent with the advection scheme.
             */
            void compute_tendency(const std::vector<double> vorticity[2],
                                  const std::vector<double> psi[2])
            {
                const double relaxation = 1.0 / std::max(parameters.relaxation_seconds, 1.0);
                const double ekman = 1.0 / std::max(parameters.ekman_seconds, 1.0);
                // The fourth difference of the two-cell wave is 16, so damping it in
                // `grid_scale_damping_seconds` fixes the coefficient without a second knob.
                const double damping =
                    1.0 / (16.0 * std::max(parameters.grid_scale_damping_seconds, 1.0));

                relative_vorticity(vorticity, psi, 1, scratch.data());

                for (int layer = 0; layer < 2; ++layer)
                {
                    const double* source = vorticity[layer].data();
                    const double* climatology = climatological_vorticity[layer].data();
                    double* result = tendency[layer].data();
                    jacobian(psi[layer].data(), source, result);

                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        const double f = coriolis[std::size_t(j)];
                        for (int i = 0; i < longitude_cells; ++i)
                        {
                            const int c = index(j, i);
                            // The fourth difference is taken of the *relative* vorticity: the
                            // planetary part is a fixed function of latitude with a fourth
                            // difference of its own, and damping it would drive a spurious
                            // tendency everywhere the state was already at rest.
                            const double centre = source[c] - f;
                            const double north_one =
                                across_pole(source, j + 1, i) - coriolis_at(j + 1);
                            const double south_one =
                                across_pole(source, j - 1, i) - coriolis_at(j - 1);
                            const double north_two =
                                across_pole(source, j + 2, i) - coriolis_at(j + 2);
                            const double south_two =
                                across_pole(source, j - 2, i) - coriolis_at(j - 2);
                            const double fourth = north_two - 4.0 * north_one + 6.0 * centre -
                                                  4.0 * south_one + south_two;

                            result[c] = -result[c] - relaxation * (source[c] - climatology[c]) -
                                        damping * fourth;
                            if (layer == 1)
                                result[c] -= ekman * scratch[std::size_t(c)];
                        }
                    }
                }
            }

            /** @brief Planetary vorticity of a row that may lie beyond a pole. */
            double coriolis_at(int row) const noexcept
            {
                if (row < 0)
                    row = -1 - row;
                else if (row >= latitude_cells)
                    row = 2 * latitude_cells - 1 - row;
                return coriolis[std::size_t(row)];
            }

            /** @brief Damps the zonal wavenumbers the grid cannot carry, and the grid scale. */
            void apply_filter(double dt)
            {
                const double rate = dt / std::max(parameters.grid_scale_damping_seconds, 1.0);

                for (int j = 0; j < latitude_cells; ++j)
                    transform.forward_real_pair(potential_vorticity[0].data() + j * longitude_cells,
                                                potential_vorticity[1].data() + j * longitude_cells,
                                                first_spectrum.data() + j * longitude_cells,
                                                second_spectrum.data() + j * longitude_cells);

                for (int j = 0; j < latitude_cells; ++j)
                    for (int m = 0; m < modes; ++m)
                    {
                        const double shape =
                            filter_shape[std::size_t(m) * std::size_t(latitude_cells) +
                                         std::size_t(j)];
                        const double exponent = rate * shape;
                        const double factor = exponent > 700.0 ? 0.0 : std::exp(-exponent);
                        const int mirror = (longitude_cells - m) % longitude_cells;
                        first_spectrum[std::size_t(index(j, m))] *= factor;
                        second_spectrum[std::size_t(index(j, m))] *= factor;
                        if (mirror != m)
                        {
                            first_spectrum[std::size_t(index(j, mirror))] *= factor;
                            second_spectrum[std::size_t(index(j, mirror))] *= factor;
                        }
                    }

                for (int j = 0; j < latitude_cells; ++j)
                    transform.inverse_real_pair(first_spectrum.data() + j * longitude_cells,
                                                second_spectrum.data() + j * longitude_cells,
                                                potential_vorticity[0].data() + j * longitude_cells,
                                                potential_vorticity[1].data() + j * longitude_cells,
                                                transform_work.data());
            }

            /** @brief Wind from the streamfunction: `u = -dpsi/dy`, `v = dpsi/dx`. */
            void diagnose_winds()
            {
                const double radius = parameters.planet_radius_m;
                for (int layer = 0; layer < 2; ++layer)
                {
                    const double* psi = streamfunction[layer].data();
                    double* eastward = wind_eastward[layer].data();
                    double* northward = wind_northward[layer].data();
                    for (int j = 0; j < latitude_cells; ++j)
                    {
                        const double zonal_scale =
                            1.0 / (radius * cosine[std::size_t(j)] * 2.0 * delta_longitude);
                        const double meridional_scale = 1.0 / (radius * 2.0 * delta_latitude);
                        for (int i = 0; i < longitude_cells; ++i)
                        {
                            const int c = index(j, i);
                            const double north = across_pole(psi, j + 1, i);
                            const double south = across_pole(psi, j - 1, i);
                            const double east = psi[index(j, wrap_longitude(i + 1))];
                            const double west = psi[index(j, wrap_longitude(i - 1))];
                            eastward[c] = -(north - south) * meridional_scale;
                            northward[c] = (east - west) * zonal_scale;
                        }
                    }
                }
            }

            /**
             * @brief The quasi-geostrophic omega, from the upper layer's own vorticity budget.
             *
             * `w = -(H/f0) * (d(zeta_1)/dt + J(psi_1, zeta_1 + f))`. The tendency is taken by
             * difference across the completed step, so this costs one Jacobian and no elliptic
             * solve: both ends of the step already had their streamfunction recovered for other
             * reasons. Ascent thins the upper layer, which is why the sign is negative — and it
             * is the same relation that spins up a surface low underneath rising air.
             *
             * The tendency taken this way carries the step's non-advective terms with it — the
             * relaxation, the damping, and whatever the zonal filter removed — as if they too
             * were vertical motion. They are not, and at the scales the flow actually occupies
             * they are small; a diagnostic that separated them would need the tendency
             * decomposed rather than differenced, which is a second bookkeeping of the step.
             */
            void diagnose_vertical_velocity(double dt)
            {
                const double* before = upper_relative_vorticity.data();
                double* after = scratch.data();
                relative_vorticity(potential_vorticity, streamfunction, 0, after);

                std::vector<double>& absolute = barotropic; // reused; the inversion is finished
                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double f = coriolis[std::size_t(j)];
                    for (int i = 0; i < longitude_cells; ++i)
                        absolute[std::size_t(index(j, i))] = after[index(j, i)] + f;
                }

                double* advection = baroclinic.data();
                jacobian(streamfunction[0].data(), absolute.data(), advection);

                const double scale = -parameters.layer_depth_m /
                                     std::max(std::fabs(parameters.reference_coriolis), 1e-9);
                double* result = omega.data();
                for (int c = 0; c < cells; ++c)
                    result[c] = scale * ((after[c] - before[c]) / dt + advection[c]);
            }

            /**
             * @brief Transports the column water and applies its sources.
             *
             * Semi-Lagrangian, with a cubic interpolation limited to the range of the four cells
             * bracketing the departure point — so the scheme is accurate where the field is
             * smooth and cannot invent water where it is not. The ascent term is the
             * ageostrophic moisture convergence: the geostrophic flow is non-divergent by
             * construction, so without it a low could never gather the water it rains out.
             */
            void step_moisture(double dt)
            {
                const double radius = parameters.planet_radius_m;
                const double max_cells_x = MAX_DEPARTURE_CELLS_FRACTION * double(longitude_cells);
                const double max_cells_y = MAX_DEPARTURE_CELLS_FRACTION * double(latitude_cells);

                const double* eastward = wind_eastward[1].data();
                const double* northward = wind_northward[1].data();
                const double* source = water.data();
                double* result = scratch.data();

                for (int j = 0; j < latitude_cells; ++j)
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        double shift_x = eastward[c] * dt /
                                         (radius * cosine[std::size_t(j)] * delta_longitude);
                        double shift_y = northward[c] * dt / (radius * delta_latitude);
                        shift_x = std::min(std::max(shift_x, -max_cells_x), max_cells_x);
                        shift_y = std::min(std::max(shift_y, -max_cells_y), max_cells_y);
                        result[c] = interpolate(source, double(i) - shift_x, double(j) - shift_y);
                    }

                const double evaporation = 1.0 / std::max(parameters.evaporation_seconds, 1.0);
                const double condensation = 1.0 / std::max(parameters.condensation_seconds, 1.0);
                const double depth = std::max(parameters.moisture_depth_m, 1.0);
                const double* ascent = omega.data();
                double* target = water.data();
                double* rain = precipitation.data();

                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double saturated = saturation[std::size_t(j)];
                    const double target_water = parameters.evaporation_relative_humidity * saturated;
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        double value = result[c];
                        value += dt * value * ascent[c] / depth;
                        if (value < target_water)
                            value += dt * evaporation * (target_water - value);

                        double condensed = 0.0;
                        if (value > saturated)
                        {
                            condensed = std::min(dt * condensation * (value - saturated),
                                                 value - saturated);
                            value -= condensed;
                        }
                        target[c] = std::max(value, 0.0);
                        rain[c] = condensed / dt * SECONDS_PER_HOUR;
                    }
                }
            }

            /**
             * @brief Refreshes the two anomaly fields the regional nest is forced with.
             *
             * The layer difference is a thickness, and a thickness is a temperature by the
             * hypsometric relation — so the thermal anomaly is not an independent field that
             * could drift out of step with the wind, it is the same field read in another unit.
             * Both are taken about the zonal mean, because the mean state is what a nest already
             * has and the eddy is what it cannot make.
             */
            void update_anomalies()
            {
                const double thermal_scale = parameters.reference_coriolis *
                                             parameters.reference_temperature_kelvin /
                                             std::max(parameters.gravity_mps2 *
                                                          parameters.layer_depth_m, 1e-9);
                const double* upper = streamfunction[0].data();
                const double* lower = streamfunction[1].data();
                const double* column_water = water.data();

                // The hydrostatic conversion from a lower-layer streamfunction to a surface
                // pressure: psi is a geostrophic streamfunction, so rho*f0*psi is the pressure it
                // is in balance with.
                const double pressure_scale = parameters.air_density_kg_per_m3 *
                                              parameters.reference_coriolis /
                                              PASCALS_PER_HECTOPASCAL;

                for (int j = 0; j < latitude_cells; ++j)
                {
                    double mean_shear = 0.0;
                    double mean_water = 0.0;
                    double mean_lower = 0.0;
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        mean_shear += upper[c] - lower[c];
                        mean_water += column_water[c];
                        mean_lower += lower[c];
                    }
                    mean_shear /= double(longitude_cells);
                    mean_water /= double(longitude_cells);
                    mean_lower /= double(longitude_cells);

                    const double saturated = std::max(saturation[std::size_t(j)], 1e-6);
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        thermal_anomaly[std::size_t(c)] =
                            thermal_scale * ((upper[c] - lower[c]) - mean_shear);
                        humidity_anomaly[std::size_t(c)] =
                            (column_water[c] - mean_water) / saturated;
                        pressure_anomaly[std::size_t(c)] =
                            pressure_scale * (lower[c] - mean_lower);
                    }
                }

                // The frontal measure is taken of the *total* thermal field, not of the anomaly
                // above: a front is a concentration of the real temperature gradient, and
                // removing the zonal mean first would erase the background baroclinic zone that
                // frontogenesis concentrates. The two readings are wanted for opposite reasons —
                // one is what the nest is missing, the other is what is actually there.
                const double radius = parameters.planet_radius_m;
                const double per_hundred_kilometres = 1.0e5;
                for (int j = 0; j < latitude_cells; ++j)
                {
                    const double zonal_step =
                        radius * cosine[std::size_t(j)] * delta_longitude * 2.0;
                    const double meridional_step = radius * delta_latitude * 2.0;
                    for (int i = 0; i < longitude_cells; ++i)
                    {
                        const int c = index(j, i);
                        const double east = shear_at(upper, lower, j, wrap_longitude(i + 1));
                        const double west = shear_at(upper, lower, j, wrap_longitude(i - 1));
                        const double north = shear_across_pole(upper, lower, j + 1, i);
                        const double south = shear_across_pole(upper, lower, j - 1, i);
                        const double zonal_gradient = (east - west) / zonal_step;
                        const double meridional_gradient = (north - south) / meridional_step;
                        frontal_strength[std::size_t(c)] =
                            thermal_scale * per_hundred_kilometres *
                            std::sqrt(zonal_gradient * zonal_gradient +
                                      meridional_gradient * meridional_gradient);
                    }
                }
            }

            /** @brief The layer difference — a thickness, and therefore a temperature — at a cell. */
            double shear_at(const double* upper, const double* lower, int row, int i) const noexcept
            {
                const int c = index(row, i);
                return upper[c] - lower[c];
            }

            /** @brief The same, for a row that may lie beyond a pole. */
            double shear_across_pole(const double* upper, const double* lower, int row,
                                     int i) const noexcept
            {
                return across_pole(upper, row, i) - across_pole(lower, row, i);
            }

            /** @brief Limited cubic interpolation of @p field at fractional grid indices. */
            double interpolate(const double* field, double x, double y) const noexcept
            {
                const int base_x = int(std::floor(x));
                const int base_y = int(std::floor(y));
                const double fraction_x = x - double(base_x);
                const double fraction_y = y - double(base_y);

                double rows[4];
                for (int r = 0; r < 4; ++r)
                {
                    const int row = base_y - 1 + r;
                    const double p0 = across_pole(field, row, base_x - 1);
                    const double p1 = across_pole(field, row, base_x);
                    const double p2 = across_pole(field, row, base_x + 1);
                    const double p3 = across_pole(field, row, base_x + 2);
                    rows[r] = limit_between(cubic(p0, p1, p2, p3, fraction_x), p1, p2);
                }
                const double value = cubic(rows[0], rows[1], rows[2], rows[3], fraction_y);
                return limit_between(value, rows[1], rows[2]);
            }

            /** @brief Bilinear sample of a cell-centred field at a geographic point. */
            double sample(const std::vector<double>& field,
                          const GeographicPosition& position) const noexcept
            {
                double longitude = std::fmod(position.longitude_radians, 2.0 * PI);
                if (longitude < 0.0)
                    longitude += 2.0 * PI;
                const double x = longitude / delta_longitude;
                const double y =
                    (std::min(std::max(position.latitude_radians, -0.5 * PI), 0.5 * PI) +
                     0.5 * PI) /
                        delta_latitude -
                    0.5;

                const int base_x = int(std::floor(x));
                const int base_y = int(std::floor(y));
                const double fraction_x = x - double(base_x);
                const double fraction_y = y - double(base_y);
                const double* data = field.data();

                const double south = across_pole(data, base_y, base_x) * (1.0 - fraction_x) +
                                     across_pole(data, base_y, base_x + 1) * fraction_x;
                const double north = across_pole(data, base_y + 1, base_x) * (1.0 - fraction_x) +
                                     across_pole(data, base_y + 1, base_x + 1) * fraction_x;
                return south * (1.0 - fraction_y) + north * fraction_y;
            }
        };

        QuasiGeostrophicCore::QuasiGeostrophicCore(const QuasiGeostrophicGridSize& size,
                                                   const QuasiGeostrophicParameters& parameters)
            : state_(new State(size, parameters))
        {
        }

        QuasiGeostrophicCore::~QuasiGeostrophicCore() = default;

        bool QuasiGeostrophicCore::valid() const noexcept { return state_->valid; }

        void QuasiGeostrophicCore::seed(std::uint64_t seed)
        {
            State& s = *state_;
            if (!s.valid)
                return;

            s.rng = Loop::seed_rng(seed);
            s.simulated_seconds = 0.0;
            s.pending_seconds = 0.0;
            s.steps = 0;

            // The mean state, plus a baroclinic perturbation of a few zonal wavenumbers with
            // random phases. Opposite signs in the two layers because the mode that is going to
            // grow is the one with vertical structure, and seeding it directly is the difference
            // between a storm in simulated days and a storm in simulated weeks.
            const int lowest = std::max(1, s.parameters.seed_lowest_wavenumber);
            const int highest =
                std::max(lowest, std::min(s.parameters.seed_highest_wavenumber, s.modes - 1));
            const int count = highest - lowest + 1;

            std::vector<double> phase(std::size_t(count), 0.0);
            for (int k = 0; k < count; ++k)
                phase[std::size_t(k)] = Loop::next_unit(s.rng) * 2.0 * PI;

            const double amplitude = s.parameters.seed_perturbation_mps *
                                     s.parameters.planet_radius_m *
                                     s.parameters.jet_width_radians / std::max(double(count), 1.0);

            for (int layer = 0; layer < 2; ++layer)
            {
                double* psi = s.streamfunction[layer].data();
                const double sign = layer == 0 ? 1.0 : -1.0;
                for (int j = 0; j < s.latitude_cells; ++j)
                {
                    const double latitude = -0.5 * PI + (double(j) + 0.5) * s.delta_latitude;
                    const double north = (latitude - s.parameters.jet_latitude_radians) /
                                         s.parameters.jet_width_radians;
                    const double south = (latitude + s.parameters.jet_latitude_radians) /
                                         s.parameters.jet_width_radians;
                    const double envelope = std::exp(-north * north) + std::exp(-south * south);
                    const double mean = s.climatological_streamfunction[layer][std::size_t(j)];
                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        const double longitude = double(i) * s.delta_longitude;
                        double disturbance = 0.0;
                        for (int k = 0; k < count; ++k)
                            disturbance += std::cos(double(lowest + k) * longitude +
                                                    phase[std::size_t(k)]);
                        psi[s.index(j, i)] =
                            mean + sign * amplitude * envelope * disturbance;
                    }
                }
            }

            for (int layer = 0; layer < 2; ++layer)
            {
                double* target = s.potential_vorticity[layer].data();
                const double* psi = s.streamfunction[layer].data();
                const double* upper = s.streamfunction[0].data();
                const double* lower = s.streamfunction[1].data();
                const double sign = layer == 0 ? -1.0 : 1.0;
                for (int j = 0; j < s.latitude_cells; ++j)
                {
                    const double f = s.coriolis[std::size_t(j)];
                    const double stretch = s.stretch[std::size_t(j)];
                    const double zonal =
                        1.0 / (s.parameters.planet_radius_m * s.parameters.planet_radius_m *
                               s.cosine[std::size_t(j)] * s.cosine[std::size_t(j)] *
                               s.delta_longitude * s.delta_longitude);
                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        const int c = s.index(j, i);
                        const double east = psi[s.index(j, s.wrap_longitude(i + 1))];
                        const double west = psi[s.index(j, s.wrap_longitude(i - 1))];
                        const double north = s.across_pole(psi, j + 1, i);
                        const double south = s.across_pole(psi, j - 1, i);
                        const double laplacian =
                            zonal * (east - 2.0 * psi[c] + west) +
                            s.upper_coefficient[std::size_t(j)] * (north - psi[c]) -
                            s.lower_coefficient[std::size_t(j)] * (psi[c] - south);
                        target[c] = laplacian + f + sign * stretch * (upper[c] - lower[c]);
                    }
                }
            }

            for (int j = 0; j < s.latitude_cells; ++j)
            {
                const double initial = s.parameters.evaporation_relative_humidity *
                                       s.saturation[std::size_t(j)];
                for (int i = 0; i < s.longitude_cells; ++i)
                    s.water[std::size_t(s.index(j, i))] = initial;
            }
            std::fill(s.omega.begin(), s.omega.end(), 0.0);
            std::fill(s.precipitation.begin(), s.precipitation.end(), 0.0);

            // Restore the invariant every step relies on: the streamfunction is the one the
            // current potential vorticity inverts to, not the one it was built from — those
            // differ by whatever the discrete inversion's own null space absorbed.
            s.invert(s.potential_vorticity, s.streamfunction);
            s.diagnose_winds();
            s.update_anomalies();
        }

        void QuasiGeostrophicCore::step(double dt_seconds)
        {
            State& s = *state_;
            if (!s.valid || dt_seconds <= 0.0)
                return;

            s.relative_vorticity(s.potential_vorticity, s.streamfunction, 0,
                                 s.upper_relative_vorticity.data());

            for (int layer = 0; layer < 2; ++layer)
                s.initial_vorticity[layer] = s.potential_vorticity[layer];

            // Strong-stability-preserving Runge-Kutta, third order: three forward-Euler
            // substeps recombined so that no stage ever leaves the region the first one did.
            // The streamfunction has to be recovered before each, because the advecting flow is
            // the state being advanced.
            s.compute_tendency(s.potential_vorticity, s.streamfunction);
            for (int layer = 0; layer < 2; ++layer)
                for (int c = 0; c < s.cells; ++c)
                    s.stage_vorticity[layer][std::size_t(c)] =
                        s.initial_vorticity[layer][std::size_t(c)] +
                        dt_seconds * s.tendency[layer][std::size_t(c)];

            s.invert(s.stage_vorticity, s.streamfunction);
            s.compute_tendency(s.stage_vorticity, s.streamfunction);
            for (int layer = 0; layer < 2; ++layer)
                for (int c = 0; c < s.cells; ++c)
                    s.stage_vorticity[layer][std::size_t(c)] =
                        0.75 * s.initial_vorticity[layer][std::size_t(c)] +
                        0.25 * (s.stage_vorticity[layer][std::size_t(c)] +
                                dt_seconds * s.tendency[layer][std::size_t(c)]);

            s.invert(s.stage_vorticity, s.streamfunction);
            s.compute_tendency(s.stage_vorticity, s.streamfunction);
            for (int layer = 0; layer < 2; ++layer)
                for (int c = 0; c < s.cells; ++c)
                    s.potential_vorticity[layer][std::size_t(c)] =
                        (1.0 / 3.0) * s.initial_vorticity[layer][std::size_t(c)] +
                        (2.0 / 3.0) * (s.stage_vorticity[layer][std::size_t(c)] +
                                       dt_seconds * s.tendency[layer][std::size_t(c)]);

            s.apply_filter(dt_seconds);
            s.invert(s.potential_vorticity, s.streamfunction);
            s.diagnose_vertical_velocity(dt_seconds);
            s.diagnose_winds();
            s.step_moisture(dt_seconds);
            s.update_anomalies();

            s.simulated_seconds += dt_seconds;
            ++s.steps;
        }

        int QuasiGeostrophicCore::advance(double elapsed_seconds)
        {
            State& s = *state_;
            if (!s.valid || elapsed_seconds <= 0.0)
                return 0;

            const double length = std::max(s.parameters.step_seconds, 1.0);
            s.pending_seconds += elapsed_seconds;

            int taken = 0;
            const int limit = std::max(1, s.parameters.max_steps_per_advance);
            while (s.pending_seconds >= length && taken < limit)
            {
                step(length);
                s.pending_seconds -= length;
                ++taken;
            }
            // Time beyond the cap is dropped rather than carried: a core that has fallen a
            // simulated day behind should return to the present, not spend the next minute of
            // wall clock replaying a day nobody watched.
            if (s.pending_seconds >= length)
                s.pending_seconds = std::fmod(s.pending_seconds, length);
            return taken;
        }

        namespace
        {
            /** @brief Identifies a global-core blob; a scene sidecar is otherwise unlabelled bytes. */
            constexpr char CAPTURE_MAGIC[4] = {'S', 'E', 'Q', 'G'};

            /** @brief Blob layout version. A reader that does not know it declines rather than guesses. */
            constexpr std::uint32_t CAPTURE_VERSION = 1;

            /** @brief Appends @p value's bytes to @p blob. Little-endian, as every target here is. */
            template <typename T> void append(std::vector<std::uint8_t>& blob, const T& value)
            {
                const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
                blob.insert(blob.end(), bytes, bytes + sizeof(T));
            }

            /** @brief Reads @p value from @p blob at @p cursor, advancing it. False if short. */
            template <typename T>
            bool take(const std::vector<std::uint8_t>& blob, std::size_t& cursor, T& value)
            {
                if (cursor + sizeof(T) > blob.size())
                    return false;
                std::memcpy(&value, blob.data() + cursor, sizeof(T));
                cursor += sizeof(T);
                return true;
            }
        } // namespace

        std::vector<std::uint8_t> QuasiGeostrophicCore::capture() const
        {
            const State& s = *state_;
            std::vector<std::uint8_t> blob;
            if (!s.valid)
                return blob;

            blob.reserve(64 + std::size_t(s.cells) * 3 * sizeof(float));
            blob.insert(blob.end(), CAPTURE_MAGIC, CAPTURE_MAGIC + 4);
            append(blob, CAPTURE_VERSION);
            append(blob, std::int32_t(s.longitude_cells));
            append(blob, std::int32_t(s.latitude_cells));
            append(blob, s.steps);
            append(blob, s.simulated_seconds);
            append(blob, s.pending_seconds);
            append(blob, s.rng.s0);
            append(blob, s.rng.s1);

            for (int layer = 0; layer < 2; ++layer)
            {
                const double* source = s.potential_vorticity[layer].data();
                for (int j = 0; j < s.latitude_cells; ++j)
                {
                    const double f = s.coriolis[std::size_t(j)];
                    for (int i = 0; i < s.longitude_cells; ++i)
                        append(blob, float(source[s.index(j, i)] - f));
                }
            }
            for (int c = 0; c < s.cells; ++c)
                append(blob, float(s.water[std::size_t(c)]));
            return blob;
        }

        bool QuasiGeostrophicCore::restore(const std::vector<std::uint8_t>& blob)
        {
            State& s = *state_;
            if (!s.valid || blob.size() < 4)
                return false;
            if (std::memcmp(blob.data(), CAPTURE_MAGIC, 4) != 0)
                return false;

            std::size_t cursor = 4;
            std::uint32_t version = 0;
            std::int32_t longitudes = 0;
            std::int32_t latitudes = 0;
            std::uint64_t steps = 0;
            double simulated = 0.0;
            double pending = 0.0;
            Loop::RngState rng{};
            if (!take(blob, cursor, version) || version != CAPTURE_VERSION)
                return false;
            if (!take(blob, cursor, longitudes) || !take(blob, cursor, latitudes))
                return false;
            if (longitudes != s.longitude_cells || latitudes != s.latitude_cells)
                return false;
            if (!take(blob, cursor, steps) || !take(blob, cursor, simulated) ||
                !take(blob, cursor, pending) || !take(blob, cursor, rng.s0) ||
                !take(blob, cursor, rng.s1))
                return false;
            if (blob.size() - cursor != std::size_t(s.cells) * 3 * sizeof(float))
                return false;

            // Read into the fields only once every header check has passed, so a rejected blob
            // leaves a running atmosphere running rather than half-overwritten.
            for (int layer = 0; layer < 2; ++layer)
            {
                double* target = s.potential_vorticity[layer].data();
                for (int j = 0; j < s.latitude_cells; ++j)
                {
                    const double f = s.coriolis[std::size_t(j)];
                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        float value = 0.0f;
                        take(blob, cursor, value);
                        target[s.index(j, i)] = double(value) + f;
                    }
                }
            }
            for (int c = 0; c < s.cells; ++c)
            {
                float value = 0.0f;
                take(blob, cursor, value);
                s.water[std::size_t(c)] = double(value);
            }

            s.steps = steps;
            s.simulated_seconds = simulated;
            s.pending_seconds = pending;
            s.rng = rng;
            std::fill(s.omega.begin(), s.omega.end(), 0.0);
            std::fill(s.precipitation.begin(), s.precipitation.end(), 0.0);

            // The vertical velocity is diagnosed from a tendency across a step, so it is the one
            // published field a restore cannot reconstruct without taking one. It comes back on
            // the first step; until then the nest is told the air is not moving vertically,
            // which is a truthful description of an atmosphere that has not been stepped.
            s.invert(s.potential_vorticity, s.streamfunction);
            s.relative_vorticity(s.potential_vorticity, s.streamfunction, 0,
                                 s.upper_relative_vorticity.data());
            s.diagnose_winds();
            s.update_anomalies();
            return true;
        }

        void QuasiGeostrophicCore::inject_vorticity(const GeographicPosition& position,
                                                     double radius_m, double amplitude_mps)
        {
            State& s = *state_;
            if (!s.valid || radius_m <= 0.0)
                return;

            const double peak = amplitude_mps /
                                (GAUSSIAN_BLOB_WIND_FACTOR * radius_m) *
                                (position.latitude_radians >= 0.0 ? 1.0 : -1.0);
            const double centre_cosine =
                std::max(std::cos(position.latitude_radians), 0.05);

            for (int layer = 0; layer < 2; ++layer)
            {
                double* target = s.potential_vorticity[layer].data();
                for (int j = 0; j < s.latitude_cells; ++j)
                {
                    const double latitude = -0.5 * PI + (double(j) + 0.5) * s.delta_latitude;
                    const double north =
                        s.parameters.planet_radius_m * (latitude - position.latitude_radians);
                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        double difference =
                            double(i) * s.delta_longitude - position.longitude_radians;
                        difference = std::fmod(difference + PI, 2.0 * PI);
                        if (difference < 0.0)
                            difference += 2.0 * PI;
                        difference -= PI;
                        const double east =
                            s.parameters.planet_radius_m * difference * centre_cosine;
                        const double squared = (east * east + north * north) /
                                               (2.0 * radius_m * radius_m);
                        if (squared > 30.0)
                            continue;
                        target[s.index(j, i)] += peak * std::exp(-squared);
                    }
                }
            }

            s.invert(s.potential_vorticity, s.streamfunction);
            s.diagnose_winds();
            s.update_anomalies();
        }

        Wind QuasiGeostrophicCore::wind_at(const GeographicPosition& position,
                                            double level_fraction) const
        {
            const State& s = *state_;
            Wind wind;
            if (!s.valid)
                return wind;

            const double blend = std::min(std::max(level_fraction, 0.0), 1.0);
            const double upper_east = s.sample(s.wind_eastward[0], position);
            const double upper_north = s.sample(s.wind_northward[0], position);
            const double lower_east = s.sample(s.wind_eastward[1], position);
            const double lower_north = s.sample(s.wind_northward[1], position);

            double eastward = lower_east + (upper_east - lower_east) * blend;
            double northward = lower_north + (upper_north - lower_north) * blend;

            // Surface friction, turning the wind toward low pressure and fading out with height.
            // The sense of the turn follows the hemisphere, because the rotation the friction
            // opposes does.
            const double hemisphere = position.latitude_radians >= 0.0 ? 1.0 : -1.0;
            const double angle =
                hemisphere * s.parameters.surface_friction_radians * (1.0 - blend);
            const double cosine = std::cos(angle);
            const double sine = std::sin(angle);
            wind.eastward_mps = eastward * cosine - northward * sine;
            wind.northward_mps = eastward * sine + northward * cosine;
            return wind;
        }

        double QuasiGeostrophicCore::pressure_anomaly_hpa(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.pressure_anomaly, position) : 0.0;
        }

        double QuasiGeostrophicCore::precipitable_water_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.water, position) : 0.0;
        }

        double QuasiGeostrophicCore::thermal_anomaly_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.thermal_anomaly, position) : 0.0;
        }

        double QuasiGeostrophicCore::humidity_anomaly_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.humidity_anomaly, position) : 0.0;
        }

        double QuasiGeostrophicCore::frontal_strength_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.frontal_strength, position) : 0.0;
        }

        double QuasiGeostrophicCore::vertical_velocity_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.omega, position) : 0.0;
        }

        double QuasiGeostrophicCore::precipitation_at(const GeographicPosition& position) const
        {
            const State& s = *state_;
            return s.valid ? s.sample(s.precipitation, position) : 0.0;
        }

        QuasiGeostrophicDiagnostics QuasiGeostrophicCore::diagnostics() const
        {
            const State& s = *state_;
            QuasiGeostrophicDiagnostics result;
            if (!s.valid)
                return result;

            const double mass = s.parameters.air_density_kg_per_m3 * s.parameters.layer_depth_m;
            double weight_total = 0.0;
            double eddy = 0.0;
            double zonal = 0.0;
            double water_total = 0.0;
            double rain_total = 0.0;
            double enstrophy = 0.0;

            for (int j = 0; j < s.latitude_cells; ++j)
            {
                const double weight = s.cosine[std::size_t(j)];
                weight_total += weight * double(s.longitude_cells);

                double mean_water = 0.0;
                double mean_rain = 0.0;
                for (int i = 0; i < s.longitude_cells; ++i)
                {
                    const int c = s.index(j, i);
                    mean_water += s.water[std::size_t(c)];
                    mean_rain += s.precipitation[std::size_t(c)];
                }
                water_total += weight * mean_water;
                rain_total += weight * mean_rain;

                for (int layer = 0; layer < 2; ++layer)
                {
                    const double* eastward = s.wind_eastward[layer].data();
                    const double* northward = s.wind_northward[layer].data();
                    const double* vorticity = s.potential_vorticity[layer].data();

                    double mean_east = 0.0;
                    double mean_north = 0.0;
                    double mean_vorticity = 0.0;
                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        const int c = s.index(j, i);
                        mean_east += eastward[c];
                        mean_north += northward[c];
                        mean_vorticity += vorticity[c];
                    }
                    mean_east /= double(s.longitude_cells);
                    mean_north /= double(s.longitude_cells);
                    mean_vorticity /= double(s.longitude_cells);

                    if (layer == 0 && mean_east > result.jet_speed_mps)
                    {
                        result.jet_speed_mps = mean_east;
                        result.jet_latitude_radians =
                            -0.5 * PI + (double(j) + 0.5) * s.delta_latitude;
                    }

                    for (int i = 0; i < s.longitude_cells; ++i)
                    {
                        const int c = s.index(j, i);
                        const double east = eastward[c] - mean_east;
                        const double north = northward[c] - mean_north;
                        eddy += weight * mass * 0.5 * (east * east + north * north);
                        zonal += weight * mass * 0.5 *
                                 (mean_east * mean_east + mean_north * mean_north);
                        const double departure = vorticity[c] - mean_vorticity;
                        enstrophy += weight * 0.5 * departure * departure;

                        const double speed = std::sqrt(eastward[c] * eastward[c] +
                                                       northward[c] * northward[c]);
                        result.peak_wind_mps = std::max(result.peak_wind_mps, speed);
                    }
                }

                for (int i = 0; i < s.longitude_cells; ++i)
                {
                    const int c = s.index(j, i);
                    // The eddy field, not the total: the same reference `pressure_anomaly_hpa`
                    // reports at, so "the deepest low in the world" is a low rather than
                    // whichever cell sits furthest poleward down the mean jet's own gradient.
                    result.lowest_pressure_anomaly_hpa = std::min(
                        result.lowest_pressure_anomaly_hpa, s.pressure_anomaly[std::size_t(c)]);
                    result.peak_ascent_mps =
                        std::max(result.peak_ascent_mps, s.omega[std::size_t(c)]);
                }
            }

            result.eddy_kinetic_energy_j_per_m2 = eddy / weight_total;
            result.zonal_kinetic_energy_j_per_m2 = zonal / weight_total;
            result.mean_precipitable_water_kg_per_m2 = water_total / weight_total;
            result.mean_precipitation_mm_per_day = rain_total / weight_total * 24.0;
            result.potential_enstrophy = enstrophy / weight_total;
            return result;
        }

        double QuasiGeostrophicCore::simulated_seconds() const noexcept
        {
            return state_->simulated_seconds;
        }

        std::uint64_t QuasiGeostrophicCore::step_count() const noexcept { return state_->steps; }

        const QuasiGeostrophicGridSize& QuasiGeostrophicCore::size() const noexcept
        {
            return state_->size;
        }

        const QuasiGeostrophicParameters& QuasiGeostrophicCore::parameters() const noexcept
        {
            return state_->parameters;
        }

        double QuasiGeostrophicCore::latitude_of(int index) const noexcept
        {
            const State& s = *state_;
            if (!s.valid || index < 0 || index >= s.latitude_cells)
                return 0.0;
            return -0.5 * PI + (double(index) + 0.5) * s.delta_latitude;
        }

        double QuasiGeostrophicCore::longitude_of(int index) const noexcept
        {
            const State& s = *state_;
            if (!s.valid || index < 0 || index >= s.longitude_cells)
                return 0.0;
            return double(index) * s.delta_longitude;
        }

        const std::vector<double>& QuasiGeostrophicCore::streamfunction(int layer) const
        {
            static const std::vector<double> empty;
            if (!state_->valid || layer < 0 || layer > 1)
                return empty;
            return state_->streamfunction[layer];
        }

        const std::vector<double>& QuasiGeostrophicCore::potential_vorticity(int layer) const
        {
            static const std::vector<double> empty;
            if (!state_->valid || layer < 0 || layer > 1)
                return empty;
            return state_->potential_vorticity[layer];
        }

        const std::vector<double>& QuasiGeostrophicCore::precipitable_water() const
        {
            return state_->water;
        }

        const std::vector<double>& QuasiGeostrophicCore::vertical_velocity() const
        {
            return state_->omega;
        }
    } // namespace Atmosphere
} // namespace SushiEngine
