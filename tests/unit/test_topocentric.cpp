/**************************************************************************/
/* test_topocentric.cpp                                                  */
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

// Unit_Topocentric: the observer's East-Up-South basis and the projection into it. The
// basis must be right-handed orthonormal at every latitude/sidereal-time (several astro
// transforms are exact only because it is), the projection must invert exactly, and the
// geodetic up must point at the equatorial pole from the pole and along the equator at
// the equator.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/topocentric.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

TEST(Unit_Topocentric, BasisIsRightHandedOrthonormalEverywhere)
{
    for (double lst = 0.0; lst < 6.28; lst += 0.7)
    {
        for (double lat = -1.5; lat <= 1.5; lat += 0.5)
        {
            const LocalSkyBasis basis = local_sky_basis(lst, lat);
            EXPECT_TRUE(Harness::is_orthonormal_basis(basis.east, basis.up, basis.south, 1e-9))
                << "lst=" << lst << " lat=" << lat;
        }
    }
}

TEST(Unit_Topocentric, ProjectionRoundTripsThroughLocalAndBack)
{
    const LocalSkyBasis basis = local_sky_basis(2.1, 0.7);
    const Vector3 equatorial{0.3, -0.9, 0.4};
    const Vector3 back = from_local(basis, to_local(basis, equatorial));
    EXPECT_TRUE(Harness::approx_equal(back, equatorial, 1e-12));
}

TEST(Unit_Topocentric, UpProjectsOntoTheLocalVerticalAxis)
{
    const LocalSkyBasis basis = local_sky_basis(1.3, -0.4);
    // The up axis, expressed in local coordinates, is exactly (0, 1, 0).
    const Vector3 local_up = to_local(basis, basis.up);
    EXPECT_TRUE(Harness::approx_equal(local_up, Vector3{0.0, 1.0, 0.0}, 1e-12));
}

TEST(Unit_Topocentric, UpPointsAtThePoleFromTheNorthPole)
{
    // At geodetic latitude +90 deg the local vertical is the equatorial +Z axis,
    // regardless of sidereal time.
    const LocalSkyBasis basis = local_sky_basis(3.9, 1.5707963267948966);
    EXPECT_TRUE(Harness::approx_equal(basis.up, Vector3{0.0, 0.0, 1.0}, 1e-9));
}

TEST(Unit_Topocentric, UpLiesInTheEquatorialPlaneAtTheEquator)
{
    const LocalSkyBasis basis = local_sky_basis(0.0, 0.0);
    // Sidereal time 0, latitude 0: up points along equatorial +X.
    EXPECT_TRUE(Harness::approx_equal(basis.up, Vector3{1.0, 0.0, 0.0}, 1e-9));
    EXPECT_NEAR(basis.up.z, 0.0, 1e-12);
}
