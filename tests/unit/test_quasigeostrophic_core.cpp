/**************************************************************************/
/* test_quasigeostrophic_core.cpp                                         */
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

// T1, the global dynamical core, against references that are not itself: a naive discrete
// Fourier transform, the potential-vorticity relation written out a second time in the test,
// the wind profile the mean state was *asked* for, and the westward drift beta implies.
//
// **Every check here that can assert a magnitude does.** `docs/design/atmosphere_system.md` §11
// records the synoptic wind running 735 times too fast for the whole life of the shipped
// system, surviving because the one test covering the field asserted that it was non-uniform —
// which a 15 km/s field satisfies perfectly. Structure was checked and magnitude never was.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/atmosphere/fourier_transform.hpp>
#include <SushiEngine/atmosphere/quasigeostrophic_core.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Atmosphere;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief The transform, spelled out at O(N^2) — the reference the fast one is checked against. */
    std::vector<std::complex<double>> naive_transform(const std::vector<std::complex<double>>& input)
    {
        const int length = int(input.size());
        std::vector<std::complex<double>> output(input.size());
        for (int m = 0; m < length; ++m)
        {
            std::complex<double> total(0.0, 0.0);
            for (int i = 0; i < length; ++i)
            {
                const double angle = -2.0 * PI * double(m) * double(i) / double(length);
                total += input[static_cast<std::size_t>(i)] *
                         std::complex<double>(std::cos(angle), std::sin(angle));
            }
            output[static_cast<std::size_t>(m)] = total;
        }
        return output;
    }

    /**
     * @brief A mean state comfortably unstable at the coarse test resolution.
     *
     * 45 m/s over a motionless lower layer, against a Phillips critical shear near 8 m/s. The
     * default 20 m/s shear is supercritical too, but only just, and a marginally unstable core
     * takes simulated weeks to produce the storm several cases below need in eight days.
     */
    AnalyticClimatology supercritical_jet()
    {
        AnalyticClimatology bands;
        bands.upper_jet_speed_mps = 45.0;
        bands.lower_jet_speed_mps = 0.0;
        return bands;
    }

    /** @brief A grid small enough to step many times, large enough to resolve the physics. */
    QuasiGeostrophicGridSize small_grid()
    {
        QuasiGeostrophicGridSize size;
        size.longitude_cells = 128;
        size.latitude_cells = 64;
        return size;
    }

    /**
     * @brief Parameters with every sink and source switched off.
     *
     * What is left is advection and the elliptic inversion, which is the regime the
     * conservation properties of the Arakawa Jacobian are stated for. Switching a term off by
     * pushing its time scale past the age of the run is exact enough for that and keeps the
     * test reading the same code path the real core does.
     */
    QuasiGeostrophicParameters inviscid()
    {
        QuasiGeostrophicParameters parameters;
        parameters.relaxation_seconds = 1e15;
        parameters.ekman_seconds = 1e15;
        parameters.grid_scale_damping_seconds = 1e15;
        parameters.evaporation_seconds = 1e15;
        parameters.condensation_seconds = 1e15;
        return parameters;
    }

    /** @brief Area-weighted mean of a field over the sphere. */
    double area_mean(const QuasiGeostrophicCore& core, const std::vector<double>& field)
    {
        double total = 0.0;
        double weight = 0.0;
        for (int j = 0; j < core.size().latitude_cells; ++j)
        {
            const double cosine = std::cos(core.latitude_of(j));
            for (int i = 0; i < core.size().longitude_cells; ++i)
            {
                total += cosine * field[static_cast<std::size_t>(j * core.size().longitude_cells + i)];
                weight += cosine;
            }
        }
        return total / weight;
    }
} // namespace

TEST(Unit_AtmosphereFourierTransform, ARejectedLengthIsNotSilentlyAccepted)
{
    EXPECT_FALSE(FourierTransform(0).valid());
    EXPECT_FALSE(FourierTransform(1).valid());
    EXPECT_FALSE(FourierTransform(48).valid());
    EXPECT_TRUE(FourierTransform(64).valid());
}

TEST(Unit_AtmosphereFourierTransform, TheFastTransformAgreesWithTheSlowOne)
{
    const int length = 64;
    FourierTransform transform(length);
    ASSERT_TRUE(transform.valid());

    std::vector<std::complex<double>> input(static_cast<std::size_t>(length));
    for (int i = 0; i < length; ++i)
        input[static_cast<std::size_t>(i)] =
            std::complex<double>(std::sin(0.3 * double(i)) + 0.2 * double(i % 7),
                                 std::cos(0.11 * double(i) * double(i)));

    const std::vector<std::complex<double>> reference = naive_transform(input);
    std::vector<std::complex<double>> measured = input;
    transform.forward(measured.data());

    for (int m = 0; m < length; ++m)
    {
        EXPECT_NEAR(measured[static_cast<std::size_t>(m)].real(), reference[static_cast<std::size_t>(m)].real(), 1e-9);
        EXPECT_NEAR(measured[static_cast<std::size_t>(m)].imag(), reference[static_cast<std::size_t>(m)].imag(), 1e-9);
    }
}

TEST(Unit_AtmosphereFourierTransform, TheInverseUndoesTheForward)
{
    const int length = 256;
    FourierTransform transform(length);
    std::vector<std::complex<double>> input(static_cast<std::size_t>(length));
    for (int i = 0; i < length; ++i)
        input[static_cast<std::size_t>(i)] = std::complex<double>(std::sin(0.05 * double(i)), 0.0);

    std::vector<std::complex<double>> roundtrip = input;
    transform.forward(roundtrip.data());
    transform.inverse(roundtrip.data());

    for (int i = 0; i < length; ++i)
        EXPECT_NEAR(roundtrip[static_cast<std::size_t>(i)].real(), input[static_cast<std::size_t>(i)].real(), 1e-12);
}

TEST(Unit_AtmosphereFourierTransform, TwoRealRowsRideOneTransformExactly)
{
    const int length = 128;
    FourierTransform transform(length);

    std::vector<double> first(static_cast<std::size_t>(length));
    std::vector<double> second(static_cast<std::size_t>(length));
    for (int i = 0; i < length; ++i)
    {
        first[static_cast<std::size_t>(i)] = std::sin(0.2 * double(i)) + 3.0;
        second[static_cast<std::size_t>(i)] = std::cos(0.07 * double(i) * double(i));
    }

    // The reference: two separate complex transforms of the same two real rows.
    std::vector<std::complex<double>> reference_first(static_cast<std::size_t>(length));
    std::vector<std::complex<double>> reference_second(static_cast<std::size_t>(length));
    for (int i = 0; i < length; ++i)
    {
        reference_first[static_cast<std::size_t>(i)] = std::complex<double>(first[static_cast<std::size_t>(i)], 0.0);
        reference_second[static_cast<std::size_t>(i)] = std::complex<double>(second[static_cast<std::size_t>(i)], 0.0);
    }
    transform.forward(reference_first.data());
    transform.forward(reference_second.data());

    std::vector<std::complex<double>> paired_first(static_cast<std::size_t>(length));
    std::vector<std::complex<double>> paired_second(static_cast<std::size_t>(length));
    transform.forward_real_pair(first.data(), second.data(), paired_first.data(),
                                paired_second.data());

    for (int m = 0; m < length; ++m)
    {
        EXPECT_NEAR(paired_first[static_cast<std::size_t>(m)].real(), reference_first[static_cast<std::size_t>(m)].real(), 1e-9);
        EXPECT_NEAR(paired_first[static_cast<std::size_t>(m)].imag(), reference_first[static_cast<std::size_t>(m)].imag(), 1e-9);
        EXPECT_NEAR(paired_second[static_cast<std::size_t>(m)].real(), reference_second[static_cast<std::size_t>(m)].real(), 1e-9);
        EXPECT_NEAR(paired_second[static_cast<std::size_t>(m)].imag(), reference_second[static_cast<std::size_t>(m)].imag(), 1e-9);
    }

    std::vector<double> recovered_first(static_cast<std::size_t>(length));
    std::vector<double> recovered_second(static_cast<std::size_t>(length));
    std::vector<std::complex<double>> work(static_cast<std::size_t>(length));
    transform.inverse_real_pair(paired_first.data(), paired_second.data(), recovered_first.data(),
                                recovered_second.data(), work.data());
    for (int i = 0; i < length; ++i)
    {
        EXPECT_NEAR(recovered_first[static_cast<std::size_t>(i)], first[static_cast<std::size_t>(i)], 1e-12);
        EXPECT_NEAR(recovered_second[static_cast<std::size_t>(i)], second[static_cast<std::size_t>(i)], 1e-12);
    }
}

TEST(Unit_QuasiGeostrophicCore, AnIllegalGridIsRejectedRatherThanApproximated)
{
    QuasiGeostrophicGridSize size;
    size.longitude_cells = 100; // not a power of two
    size.latitude_cells = 64;
    EXPECT_FALSE(QuasiGeostrophicCore(size, QuasiGeostrophicParameters()).valid());

    size.longitude_cells = 128;
    size.latitude_cells = 65; // odd: the equator would not land on a cell edge
    EXPECT_FALSE(QuasiGeostrophicCore(size, QuasiGeostrophicParameters()).valid());

    size.latitude_cells = 64;
    EXPECT_TRUE(QuasiGeostrophicCore(size, QuasiGeostrophicParameters()).valid());
}

TEST(Unit_QuasiGeostrophicCore, TheInversionSatisfiesThePotentialVorticityRelation)
{
    // The strongest statement that can be made about an elliptic solver without a second
    // solver to compare it to: the field it returned actually solves the equation. The
    // relation is written out here from the design document rather than shared with the
    // implementation, so a sign error in either one shows up as a residual.
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(core.valid());
    core.seed(7);
    core.step(600.0);

    const int longitudes = core.size().longitude_cells;
    const int latitudes = core.size().latitude_cells;
    const double radius = core.parameters().planet_radius_m;
    const double delta_longitude = 2.0 * PI / double(longitudes);
    const double delta_latitude = PI / double(latitudes);
    const double stratification =
        core.parameters().reduced_gravity_mps2 * core.parameters().layer_depth_m;
    const double stretch_floor = 1.0 / (core.parameters().maximum_deformation_radius_m *
                                        core.parameters().maximum_deformation_radius_m);

    const std::vector<double>& upper = core.streamfunction(0);
    const std::vector<double>& lower = core.streamfunction(1);

    double worst = 0.0;
    double scale = 0.0;
    for (int layer = 0; layer < 2; ++layer)
    {
        const std::vector<double>& psi = core.streamfunction(layer);
        const std::vector<double>& vorticity = core.potential_vorticity(layer);
        const double sign = layer == 0 ? -1.0 : 1.0;

        // The polar rows are skipped: their stencil crosses the pole, and reproducing that
        // indexing here would be copying the implementation rather than checking it.
        for (int j = 1; j + 1 < latitudes; ++j)
        {
            const double latitude = -0.5 * PI + (double(j) + 0.5) * delta_latitude;
            const double cosine = std::cos(latitude);
            const double f = 2.0 * core.parameters().angular_velocity_rad_per_s * std::sin(latitude);
            const double stretch = std::max(f * f / stratification, stretch_floor);
            const double south_edge = std::cos(-0.5 * PI + double(j) * delta_latitude);
            const double north_edge = std::cos(-0.5 * PI + double(j + 1) * delta_latitude);
            const double meridional =
                1.0 / (radius * radius * cosine * delta_latitude * delta_latitude);
            const double zonal =
                1.0 / (radius * radius * cosine * cosine * delta_longitude * delta_longitude);

            for (int i = 0; i < longitudes; ++i)
            {
                const int centre = j * longitudes + i;
                const int east = j * longitudes + (i + 1) % longitudes;
                const int west = j * longitudes + (i + longitudes - 1) % longitudes;
                const int north = (j + 1) * longitudes + i;
                const int south = (j - 1) * longitudes + i;

                const double laplacian =
                    zonal * (psi[static_cast<std::size_t>(east)] - 2.0 * psi[static_cast<std::size_t>(centre)] +
                             psi[static_cast<std::size_t>(west)]) +
                    meridional * (north_edge * (psi[static_cast<std::size_t>(north)] - psi[static_cast<std::size_t>(centre)]) -
                                  south_edge * (psi[static_cast<std::size_t>(centre)] - psi[static_cast<std::size_t>(south)]));
                const double expected =
                    laplacian + f +
                    sign * stretch * (upper[static_cast<std::size_t>(centre)] - lower[static_cast<std::size_t>(centre)]);

                worst = std::max(worst, std::fabs(expected - vorticity[static_cast<std::size_t>(centre)]));
                scale = std::max(scale, std::fabs(vorticity[static_cast<std::size_t>(centre)] - f));
            }
        }
    }

    ASSERT_GT(scale, 1e-8) << "the state is trivial; the residual would prove nothing";
    EXPECT_LT(worst, 1e-6 * scale) << "the recovered streamfunction does not invert to the "
                                      "potential vorticity it was recovered from";
}

TEST(Unit_QuasiGeostrophicCore, ThePrescribedJetComesBackAtThePrescribedSpeed)
{
    // The round trip that the 735x wind bug would have failed: a jet is *asked for* in metres
    // per second, turned into a streamfunction, into a potential vorticity, inverted back, and
    // differenced into a wind. Every constant in that chain has to be right for the number to
    // come back.
    QuasiGeostrophicParameters parameters;
    parameters.seed_perturbation_mps = 0.0; // an exactly zonal state

    // The jet is T0's, not the physics': it is what the atmosphere relaxes *toward*, and it is
    // the thing an ERA5 bake replaces one for one.
    AnalyticClimatology bands;
    bands.upper_jet_speed_mps = 30.0;
    bands.lower_jet_speed_mps = 10.0;

    QuasiGeostrophicGridSize size;
    size.longitude_cells = 64;
    size.latitude_cells = 256; // the jet's width is what needs resolving, not its length

    QuasiGeostrophicCore core(size, parameters, Climatology(bands));
    ASSERT_TRUE(core.valid());
    core.seed(1);

    const GeographicPosition centre{bands.jet_latitude_radians, 1.0};
    const Wind aloft = core.wind_at(centre, 1.0);
    const Wind surface = core.wind_at(centre, 0.0);

    EXPECT_NEAR(aloft.eastward_mps, 30.0, 0.6);
    EXPECT_NEAR(aloft.northward_mps, 0.0, 0.1);

    // At the surface the same wind is turned toward low pressure by the friction angle, which
    // for a purely zonal flow shows up as a northward component of `u * tan(25 degrees)`.
    const double speed = std::sqrt(surface.eastward_mps * surface.eastward_mps +
                                   surface.northward_mps * surface.northward_mps);
    EXPECT_NEAR(speed, 10.0, 0.4);
    EXPECT_NEAR(std::atan2(surface.northward_mps, surface.eastward_mps),
                parameters.surface_friction_radians, 1e-6);

    // And the turn reverses across the equator, because the rotation the friction opposes does.
    const GeographicPosition southern{-bands.jet_latitude_radians, 1.0};
    const Wind southern_surface = core.wind_at(southern, 0.0);
    EXPECT_NEAR(std::atan2(southern_surface.northward_mps, southern_surface.eastward_mps),
                -parameters.surface_friction_radians, 1e-6);
}

// The two cases below split one claim in two, because a purely zonal state carries a pressure
// anomaly of tens of hectopascals that is the mean westerly jet's own meridional gradient and
// not weather. Reported as the total `rho f0 psi_2`, measured across a forty-degree window, it
// comes to 139 hPa while a deep cyclone is perhaps 30 -- so a consumer asking "how deep is the
// low here" would be handed a reading dominated by how far north it stood, and the editor's map
// would be a picture of latitude. The reference is therefore the zonal mean, matching the
// thermal and humidity anomalies it sits beside, and both halves are worth pinning separately:
// the mean state must contribute nothing, and what is left must still be synoptic.

TEST(Unit_QuasiGeostrophicCore, TheMeanJetContributesNothingToThePressureAnomaly)
{
    QuasiGeostrophicParameters parameters;
    parameters.seed_perturbation_mps = 0.0; // an exactly zonal state: the jet, and nothing else
    QuasiGeostrophicCore core(small_grid(), parameters);
    ASSERT_TRUE(core.valid());
    core.seed(1);

    // Not "small": zero to round-off. A zonally symmetric field has no zonal-mean departure by
    // construction, so any residual here is the reference leaking the mean state back in, which
    // would show up as hundreds of hectopascals.
    const QuasiGeostrophicDiagnostics measured = core.diagnostics();
    EXPECT_NEAR(measured.lowest_pressure_anomaly_hpa, 0.0, 1.0e-9);
    EXPECT_NEAR(measured.jet_speed_mps, AnalyticClimatology().upper_jet_speed_mps, 1.0);
}

TEST(Unit_QuasiGeostrophicCore, TheEddyPressureAnomalyIsSynopticAndNotAstronomical)
{
    // Measured on a flow that has actually gone unstable, because the eddy field is the subject
    // and at seed time there is barely one. Same configuration as the cyclogenesis case below
    // (see its comment for why the jet and the damping are retuned at this resolution).
    QuasiGeostrophicParameters parameters;
    parameters.grid_scale_damping_seconds = 8.0 * 3600.0;

    QuasiGeostrophicCore core(small_grid(), parameters, Climatology(supercritical_jet()));
    ASSERT_TRUE(core.valid());
    core.seed(5);

    // `step` rather than `advance`: `advance` is capped at `max_steps_per_advance` steps per
    // call, so a loop that asks it for a day at a time simulates minutes and reports success.
    for (int day = 0; day < 8; ++day)
        for (int i = 0; i < 240; ++i)
            core.step(360.0);

    // A deep mid-latitude cyclone is a few tens of hectopascals below the surrounding flow.
    // Both bounds matter: a system that is too weak reads as no weather, and one that is too
    // strong reads as no weather either, because everything downstream saturates.
    const QuasiGeostrophicDiagnostics measured = core.diagnostics();
    EXPECT_LT(measured.lowest_pressure_anomaly_hpa, -5.0);
    EXPECT_GT(measured.lowest_pressure_anomaly_hpa, -100.0);
}

TEST(Unit_QuasiGeostrophicCore, FreeAdvectionConservesWhatTheJacobianClaimsTo)
{
    // The Arakawa Jacobian's reason for existing: with every source and sink switched off, the
    // domain integrals of the advected quantity and of its square must not drift. A scheme that
    // fails this cannot be integrated for simulated weeks, which is the only duration at which
    // this tier does anything interesting.
    QuasiGeostrophicCore core(small_grid(), inviscid());
    ASSERT_TRUE(core.valid());
    core.seed(3);

    const auto measure = [&core](double& mean, double& square)
    {
        mean = 0.0;
        square = 0.0;
        for (int layer = 0; layer < 2; ++layer)
        {
            const std::vector<double>& vorticity = core.potential_vorticity(layer);
            std::vector<double> squared(vorticity.size());
            for (std::size_t k = 0; k < vorticity.size(); ++k)
                squared[k] = vorticity[k] * vorticity[k];
            mean += area_mean(core, vorticity);
            square += area_mean(core, squared);
        }
    };

    double mean_before = 0.0;
    double square_before = 0.0;
    measure(mean_before, square_before);

    for (int i = 0; i < 240; ++i) // a simulated day
        core.step(360.0);

    double mean_after = 0.0;
    double square_after = 0.0;
    measure(mean_after, square_after);

    EXPECT_LT(std::fabs(mean_after - mean_before), 1e-4 * std::fabs(mean_before) + 1e-12);
    EXPECT_LT(std::fabs(square_after - square_before), 2e-3 * square_before);
}

TEST(Unit_QuasiGeostrophicCore, AVorticityBlobDriftsWestwardBecauseOfBeta)
{
    // Rossby's result, and the one piece of physics in this core that has an unambiguous
    // direction: a vorticity anomaly on a rotating sphere with no mean flow propagates *west*,
    // at a speed set by the planetary vorticity gradient and the anomaly's own scale. The
    // magnitude is bounded rather than equated, because a localized blob is a packet of
    // wavenumbers and not one of them.
    QuasiGeostrophicParameters parameters = inviscid();
    parameters.seed_perturbation_mps = 0.0;

    // No mean flow at all, so the only thing that can move the blob is beta.
    AnalyticClimatology bands;
    bands.upper_jet_speed_mps = 0.0;
    bands.lower_jet_speed_mps = 0.0;

    QuasiGeostrophicGridSize size;
    size.longitude_cells = 128;
    size.latitude_cells = 64;

    QuasiGeostrophicCore core(size, parameters, Climatology(bands));
    ASSERT_TRUE(core.valid());
    core.seed(1);

    const double blob_latitude = 0.7853981633974483;
    const double blob_radius = 1.2e6;
    core.inject_vorticity(GeographicPosition{blob_latitude, PI}, blob_radius, 20.0);

    // Where the low is, as the deepest cell of the hemisphere it was placed in. A centroid over
    // a signed field is not well defined; the extremum is, and it is what "the low moved" means.
    // The search is two-dimensional because the drift is: beta carries an anomaly *northwest* in
    // the northern hemisphere, not west, so a fixed latitude row loses it within a day. The
    // other hemisphere is excluded because injecting a non-zero mean vorticity onto a sphere is
    // impossible, and what the inversion does with the excess is a zonally symmetric response
    // that has nothing to do with the blob.
    const auto find_low = [&core](int& row, int& column)
    {
        const std::vector<double>& psi = core.streamfunction(0);
        double lowest = 1e30;
        row = 0;
        column = 0;
        for (int j = 0; j < core.size().latitude_cells; ++j)
        {
            if (core.latitude_of(j) <= 0.0)
                continue;
            for (int i = 0; i < core.size().longitude_cells; ++i)
            {
                const double value =
                    psi[static_cast<std::size_t>(j * core.size().longitude_cells + i)];
                if (value < lowest)
                {
                    lowest = value;
                    row = j;
                    column = i;
                }
            }
        }
    };

    int start_row = 0;
    int start_column = 0;
    find_low(start_row, start_column);

    // One day, and no longer: past that the packet has dispersed into radiated Rossby waves and
    // the deepest cell is no longer the anomaly that was placed.
    const double duration = 86400.0;
    for (int i = 0; i < 240; ++i)
        core.step(360.0);

    int end_row = 0;
    int end_column = 0;
    find_low(end_row, end_column);

    double drift = core.longitude_of(end_column) - core.longitude_of(start_column);
    if (drift > PI)
        drift -= 2.0 * PI;
    if (drift < -PI)
        drift += 2.0 * PI;

    EXPECT_LT(drift, 0.0) << "the anomaly moved east; beta has the wrong sign";
    EXPECT_GT(core.latitude_of(end_row), core.latitude_of(start_row))
        << "the anomaly did not drift poleward; the beta drift is northwest, not west";

    // Rossby's phase speed, `c = -beta/K^2`, with the packet's own scale standing in for K. It
    // is a packet and not a single wavenumber, so this is asserted as a band — but a band that
    // a wrong planetary radius, a wrong rotation rate or a missing metric factor would miss.
    const double beta = 2.0 * parameters.angular_velocity_rad_per_s *
                        std::cos(blob_latitude) / parameters.planet_radius_m;
    const double expected = beta * blob_radius * blob_radius; // metres per second, westward
    const double travelled = -drift * parameters.planet_radius_m * std::cos(blob_latitude);
    const double measured = travelled / duration;
    EXPECT_GT(measured, 0.4 * expected);
    EXPECT_LT(measured, 2.0 * expected);
}

TEST(Unit_QuasiGeostrophicCore, TheSameSeedProducesTheSameWeather)
{
    QuasiGeostrophicCore first(small_grid(), QuasiGeostrophicParameters());
    QuasiGeostrophicCore second(small_grid(), QuasiGeostrophicParameters());
    first.seed(20260729);
    second.seed(20260729);

    for (int i = 0; i < 60; ++i)
    {
        first.step(360.0);
        second.step(360.0);
    }

    EXPECT_EQ(first.streamfunction(0), second.streamfunction(0));
    EXPECT_EQ(first.potential_vorticity(1), second.potential_vorticity(1));
    EXPECT_EQ(first.precipitable_water(), second.precipitable_water());

    QuasiGeostrophicCore other(small_grid(), QuasiGeostrophicParameters());
    other.seed(20260730);
    for (int i = 0; i < 60; ++i)
        other.step(360.0);
    EXPECT_NE(first.streamfunction(0), other.streamfunction(0));
}

TEST(Unit_QuasiGeostrophicCore, ACapturedCoreResumesWhereItLeftOff)
{
    // What a scene sidecar has to buy: reopening a saved scene continues the weather rather
    // than restarting it. The check is not that the restored fields match — single precision
    // guarantees they will not exactly — but that the two cores *stay* together as they step,
    // which is the property a divergent restore would fail within a day.
    QuasiGeostrophicCore original(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(original.valid());
    original.seed(99);
    for (int i = 0; i < 120; ++i)
        original.step(360.0);

    const std::vector<std::uint8_t> blob = original.capture();
    ASSERT_GT(blob.size(), static_cast<std::size_t>(64));

    QuasiGeostrophicCore resumed(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(resumed.restore(blob));
    EXPECT_EQ(resumed.step_count(), original.step_count());
    EXPECT_NEAR(resumed.simulated_seconds(), original.simulated_seconds(), 1e-9);

    for (int i = 0; i < 120; ++i)
    {
        original.step(360.0);
        resumed.step(360.0);
    }

    const std::vector<double>& expected = original.streamfunction(0);
    const std::vector<double>& measured = resumed.streamfunction(0);
    ASSERT_EQ(expected.size(), measured.size());
    double worst = 0.0;
    double scale = 0.0;
    for (std::size_t k = 0; k < expected.size(); ++k)
    {
        worst = std::max(worst, std::fabs(expected[k] - measured[k]));
        scale = std::max(scale, std::fabs(expected[k]));
    }
    EXPECT_LT(worst, 1e-4 * scale);
}

TEST(Unit_QuasiGeostrophicCore, ARestoreRefusesABlobItCannotHonour)
{
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(core.valid());
    core.seed(1);
    const std::vector<double> before = core.streamfunction(0);

    // A grid that does not match. Resampling one atmosphere onto another grid would be a
    // silent lie about what was saved; refusing is what lets the caller restart it visibly.
    QuasiGeostrophicGridSize other;
    other.longitude_cells = 64;
    other.latitude_cells = 32;
    QuasiGeostrophicCore small(other, QuasiGeostrophicParameters());
    small.seed(1);
    EXPECT_FALSE(core.restore(small.capture()));

    EXPECT_FALSE(core.restore(std::vector<std::uint8_t>()));
    EXPECT_FALSE(core.restore(std::vector<std::uint8_t>(200, 0)));
    // A rejected blob leaves a running atmosphere running.
    EXPECT_EQ(core.streamfunction(0), before);
}

TEST(Unit_QuasiGeostrophicCore, TheThermalAnomalyIsAnEddyAndIsAFewKelvin)
{
    // What the regional nest is handed as its boundary temperature. It must be a *departure
    // from the zonal mean* — so a purely zonal state has none at all — and once eddies exist it
    // must be the few kelvin a frontal zone actually spans, not the tens the pole-to-equator
    // gradient would give if the mean had been left in.
    QuasiGeostrophicParameters zonal;
    zonal.seed_perturbation_mps = 0.0;
    QuasiGeostrophicCore quiet(small_grid(), zonal);
    ASSERT_TRUE(quiet.valid());
    quiet.seed(1);
    for (int j = 0; j < quiet.size().latitude_cells; ++j)
        EXPECT_NEAR(quiet.thermal_anomaly_at(GeographicPosition{quiet.latitude_of(j), 1.0}), 0.0,
                    1e-9);

    QuasiGeostrophicParameters lively;
    lively.grid_scale_damping_seconds = 8.0 * 3600.0;
    QuasiGeostrophicCore core(small_grid(), lively, Climatology(supercritical_jet()));
    core.seed(5);
    for (int day = 0; day < 8; ++day)
        for (int i = 0; i < 240; ++i)
            core.step(360.0);

    double strongest = 0.0;
    double wettest = 0.0;
    for (int j = 0; j < core.size().latitude_cells; ++j)
        for (int i = 0; i < core.size().longitude_cells; ++i)
        {
            const GeographicPosition position{core.latitude_of(j), core.longitude_of(i)};
            strongest = std::max(strongest, std::fabs(core.thermal_anomaly_at(position)));
            wettest = std::max(wettest, std::fabs(core.humidity_anomaly_at(position)));
        }

    EXPECT_GT(strongest, 1.0);
    EXPECT_LT(strongest, 25.0);
    EXPECT_GT(wettest, 0.01);
    EXPECT_LT(wettest, 1.0);
}

TEST(Unit_QuasiGeostrophicCore, MoistureTransportInventsNoWater)
{
    // The monotone limiter's contract, and the reason it is worth the extra order it costs:
    // transport alone may move water and may diffuse it, but it may not create an amount that
    // was not somewhere before, and it may not create a negative one.
    QuasiGeostrophicParameters parameters = inviscid();
    parameters.moisture_depth_m = 1e12; // switches off the ascent-driven convergence
    QuasiGeostrophicCore core(small_grid(), parameters);
    ASSERT_TRUE(core.valid());
    core.seed(11);

    const std::vector<double> before = core.precipitable_water();
    double highest = 0.0;
    for (double value : before)
        highest = std::max(highest, value);
    const double total_before = area_mean(core, before);

    for (int i = 0; i < 240; ++i)
        core.step(360.0);

    const std::vector<double>& after = core.precipitable_water();
    for (double value : after)
    {
        EXPECT_GE(value, 0.0);
        EXPECT_LE(value, highest + 1e-9);
    }
    EXPECT_NEAR(area_mean(core, after), total_before, 0.02 * total_before);
}

TEST(Unit_QuasiGeostrophicCore, CyclonesGrowOutOfTheMeanStateWithNothingPlacingThem)
{
    // The whole point of the tier. Nothing here creates a storm: the initial state is a zonal
    // jet plus a perturbation of a hundredth of its speed, and what grows is whatever the mean
    // state is unstable to. The resolution is coarse for a test, so the grid-scale damping is
    // lengthened to match — at 64 latitudes the deformation radius is two cells wide, and the
    // default damping is calibrated for a grid where it is twelve.
    QuasiGeostrophicParameters parameters;
    parameters.grid_scale_damping_seconds = 8.0 * 3600.0;

    QuasiGeostrophicCore core(small_grid(), parameters, Climatology(supercritical_jet()));
    ASSERT_TRUE(core.valid());
    core.seed(5);

    const double initial = core.diagnostics().eddy_kinetic_energy_j_per_m2;
    ASSERT_GT(initial, 0.0);

    for (int day = 0; day < 8; ++day)
        for (int i = 0; i < 240; ++i)
            core.step(360.0);

    const QuasiGeostrophicDiagnostics measured = core.diagnostics();
    EXPECT_GT(measured.eddy_kinetic_energy_j_per_m2, 3.0 * initial)
        << "the mean state is not producing eddies";
    EXPECT_LT(measured.eddy_kinetic_energy_j_per_m2, measured.zonal_kinetic_energy_j_per_m2)
        << "the eddies overtook the flow that feeds them; something is not being dissipated";

    // The vertical motion the eddies imply. Synoptic ascent is centimetres per second: a core
    // reporting zero has no secondary circulation at all, and one reporting metres per second
    // is reporting convection it does not resolve.
    EXPECT_GT(measured.peak_ascent_mps, 1e-4);
    EXPECT_LT(measured.peak_ascent_mps, 1.0);
}

// Injection. An author disturbs the field and the dynamics take it from there, so what has
// to be right is the disturbance itself: it must be local, it must be the strength it was
// asked for, and it must be a system that could exist.

namespace
{
    /** @brief Where the injection tests place their anomaly. */
    GeographicPosition injection_site()
    {
        GeographicPosition where;
        where.latitude_radians = 45.0 * 3.14159265358979323846 / 180.0;
        where.longitude_radians = 3.14159265358979323846; // 180 degrees east
        return where;
    }
} // namespace

TEST(Unit_QuasiGeostrophicCore, AnInjectedAnomalyIsFeltLocallyAndNotAcrossTheHemisphere)
{
    // **The defect this pins.** A Gaussian blob on its own carries net circulation, and a
    // streamfunction with net circulation grows *logarithmically outward* instead of decaying
    // -- so placing one low would quietly tilt the pressure field of the whole hemisphere, and
    // do it in a way that looks like a plausible synoptic pattern. The injection therefore lays
    // down a broader opposing lobe carrying exactly the core's circulation, and this is the
    // observable consequence of that: far from the anomaly, there is nearly nothing.
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(core.valid());
    core.seed(1);

    const GeographicPosition where = injection_site();
    core.inject_vorticity(where, 700000.0, 12.0);

    const double centre = std::fabs(core.pressure_anomaly_hpa(where));
    ASSERT_GT(centre, 5.0) << "nothing was injected, so the rest of this proves nothing";

    double far = 0.0;
    for (int degrees = 60; degrees <= 120; degrees += 10)
    {
        GeographicPosition away = where;
        away.longitude_radians += double(degrees) * 3.14159265358979323846 / 180.0;
        far = std::max(far, std::fabs(core.pressure_anomaly_hpa(away)));
    }
    EXPECT_LT(far, 0.35 * centre)
        << "an anomaly a quarter of the world away should not rival the one that was placed";
}

TEST(Unit_QuasiGeostrophicCore, AnInjectedAnomalyBlowsAtTheSpeedItWasAskedFor)
{
    // `amplitude_mps` names a peak rotational wind, and the compensating lobe is a wind of its
    // own opposing the core's -- so the conversion has to account for it. It did not at first,
    // and a requested 20 m/s blew at 16.2. That is not a small error; it is the parameter
    // meaning something other than its name.
    QuasiGeostrophicParameters parameters;
    parameters.seed_perturbation_mps = 0.0; // a zonal background, so the anomaly is the signal
    QuasiGeostrophicGridSize size;
    size.longitude_cells = 256;
    size.latitude_cells = 128;
    QuasiGeostrophicCore core(size, parameters);
    ASSERT_TRUE(core.valid());
    core.seed(1);

    const GeographicPosition where = injection_site();
    const double background = core.wind_at(where, 0.5).northward_mps;
    constexpr double REQUESTED = 20.0;
    core.inject_vorticity(where, 700000.0, REQUESTED);

    double peak = 0.0;
    for (int step = 1; step <= 60; ++step)
    {
        GeographicPosition sample = where;
        sample.longitude_radians +=
            double(step) * 0.25 * 3.14159265358979323846 / 180.0;
        peak = std::max(peak, std::fabs(core.wind_at(sample, 0.5).northward_mps - background));
    }
    EXPECT_NEAR(peak, REQUESTED, 0.15 * REQUESTED);
}

TEST(Unit_QuasiGeostrophicCore, AnInjectedLowIsASystemThatCouldExist)
{
    // The editor's default click. A deep mid-latitude cyclone runs -30 to -40 hPa against its
    // surroundings and the deepest ever recorded is near -50, so a default that produced -70 --
    // as it did before the compensation -- was offering an author a system the atmosphere has
    // never made.
    QuasiGeostrophicCore core(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(core.valid());
    core.seed(1);
    core.inject_vorticity(injection_site(), 700000.0, 12.0);

    const double depth = core.diagnostics().lowest_pressure_anomaly_hpa;
    EXPECT_LT(depth, -10.0) << "a low, not a ripple";
    EXPECT_GT(depth, -50.0) << "a low, not a record";
}

TEST(Unit_QuasiGeostrophicCore, InjectingAHighRaisesThePressureAndInjectingALowLowersIt)
{
    // Sign, end to end through the diagnosis -- the cheapest thing to get backwards and the
    // hardest to notice, since either way the map fills with plausible-looking systems.
    QuasiGeostrophicCore low(small_grid(), QuasiGeostrophicParameters());
    QuasiGeostrophicCore high(small_grid(), QuasiGeostrophicParameters());
    ASSERT_TRUE(low.valid());
    ASSERT_TRUE(high.valid());
    low.seed(2);
    high.seed(2);

    const GeographicPosition where = injection_site();
    low.inject_vorticity(where, 700000.0, 12.0);
    high.inject_vorticity(where, 700000.0, -12.0);

    EXPECT_LT(low.pressure_anomaly_hpa(where), -10.0);
    EXPECT_GT(high.pressure_anomaly_hpa(where), 10.0);
}
