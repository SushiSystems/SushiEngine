/**************************************************************************/
/* test_orbital_elements.cpp                                              */
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

// Unit_OrbitalElements: the Keplerian machinery that turns six elements plus a date
// into a heliocentric position. The properties, not hand-computed decimals: linear
// element propagation, Kepler's equation actually satisfied by the returned anomaly, a
// zero-eccentricity orbit tracing a circle of radius a, and the ecliptic<->equatorial
// rotation preserving the equinox axis and round-tripping to the identity.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    // A minimal element row: circular, un-inclined, everything else zero unless overridden.
    KeplerianElements circular_orbit(double semi_major_au, double mean_longitude_degrees)
    {
        KeplerianElements e{};
        e.semi_major_axis_au = semi_major_au;
        e.eccentricity = 0.0;
        e.inclination_degrees = 0.0;
        e.mean_longitude_degrees = mean_longitude_degrees;
        e.longitude_perihelion_degrees = 0.0;
        e.longitude_node_degrees = 0.0;
        return e;
    }
}

TEST(Unit_OrbitalElements, ElementAtPropagatesLinearlyWithRates)
{
    KeplerianElements base{};
    base.semi_major_axis_au = 1.0;
    base.semi_major_axis_rate = 0.1;
    base.eccentricity = 0.02;
    base.eccentricity_rate = -0.001;
    base.inclination_degrees = 3.0;
    base.inclination_rate = 0.5;
    base.mean_longitude_degrees = 100.0;
    base.mean_longitude_rate = 36000.0;

    const KeplerianElements at_two = element_at(base, 2.0);
    EXPECT_DOUBLE_EQ(at_two.semi_major_axis_au, 1.2);
    EXPECT_DOUBLE_EQ(at_two.eccentricity, 0.018);
    EXPECT_DOUBLE_EQ(at_two.inclination_degrees, 4.0);
    EXPECT_DOUBLE_EQ(at_two.mean_longitude_degrees, 100.0 + 72000.0);

    // Zero centuries is the identity.
    const KeplerianElements at_zero = element_at(base, 0.0);
    EXPECT_DOUBLE_EQ(at_zero.semi_major_axis_au, base.semi_major_axis_au);
    EXPECT_DOUBLE_EQ(at_zero.mean_longitude_degrees, base.mean_longitude_degrees);
}

TEST(Unit_OrbitalElements, SolveKeplerSatisfiesTheEquation)
{
    // For every eccentricity and mean anomaly the returned E must satisfy M = E - e sin E.
    for (double e = 0.0; e < 0.3; e += 0.05)
    {
        for (double m = -3.0; m <= 3.0; m += 0.25)
        {
            const double eccentric = solve_kepler(m, e);
            const double residual = eccentric - e * std::sin(eccentric) - m;
            EXPECT_NEAR(residual, 0.0, 1e-10) << "e=" << e << " M=" << m;
        }
    }
}

TEST(Unit_OrbitalElements, SolveKeplerIsIdentityForCircularOrbit)
{
    for (double m = -3.0; m <= 3.0; m += 0.3)
        EXPECT_NEAR(solve_kepler(m, 0.0), m, 1e-12);
}

TEST(Unit_OrbitalElements, CircularOrbitHasConstantRadiusEqualToSemiMajorAxis)
{
    const double a = 1.7;
    for (double l = 0.0; l < 360.0; l += 15.0)
    {
        const Vector3 p = heliocentric_ecliptic_position(circular_orbit(a, l));
        EXPECT_NEAR(length(p), a, 1e-9) << "L=" << l;
        // Un-inclined: the orbit stays in the ecliptic plane (z ~ 0).
        EXPECT_NEAR(p.z, 0.0, 1e-9) << "L=" << l;
    }
}

TEST(Unit_OrbitalElements, CircularOrbitPerihelionPointsAlongPlusX)
{
    // L = perihelion longitude -> mean anomaly 0 -> body at perihelion on +x.
    const Vector3 p = heliocentric_ecliptic_position(circular_orbit(1.0, 0.0));
    EXPECT_NEAR(p.x, 1.0, 1e-9);
    EXPECT_NEAR(p.y, 0.0, 1e-9);
    EXPECT_NEAR(p.z, 0.0, 1e-9);
}

TEST(Unit_OrbitalElements, EclipticToEquatorialLeavesTheEquinoxAxisFixed)
{
    // The rotation is about x (the equinox line), so an x-vector is unchanged.
    const Vector3 x{1.0, 0.0, 0.0};
    const Vector3 rotated = ecliptic_to_equatorial(x);
    EXPECT_TRUE(Harness::approx_equal(rotated, x, 1e-12));
}

TEST(Unit_OrbitalElements, EclipticToEquatorialTiltsTheEclipticPoleByObliquity)
{
    // The ecliptic north pole (0,0,1) tilts to (0, -sin e, cos e) in the equatorial frame.
    const Vector3 pole{0.0, 0.0, 1.0};
    const Vector3 equatorial = ecliptic_to_equatorial(pole);
    EXPECT_NEAR(equatorial.x, 0.0, 1e-12);
    EXPECT_NEAR(equatorial.y, -std::sin(OBLIQUITY_J2000_RADIANS), 1e-12);
    EXPECT_NEAR(equatorial.z, std::cos(OBLIQUITY_J2000_RADIANS), 1e-12);
}

TEST(Unit_OrbitalElements, EclipticEquatorialRoundTripsToIdentity)
{
    const Vector3 v{0.3, -0.7, 0.5};
    EXPECT_TRUE(Harness::approx_equal(equatorial_to_ecliptic(ecliptic_to_equatorial(v)), v, 1e-12));
    EXPECT_TRUE(Harness::approx_equal(ecliptic_to_equatorial(equatorial_to_ecliptic(v)), v, 1e-12));
}

TEST(Unit_OrbitalElements, ObliquityConstantMatchesTwentyThreeDegrees)
{
    EXPECT_NEAR(OBLIQUITY_J2000_RADIANS * RADIANS_TO_DEGREES, 23.4392911, 1e-6);
}
