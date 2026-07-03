/**************************************************************************/
/* test_floating_origin_stress.cpp                                       */
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

// Unit_FloatingOriginStress: extends Unit_FloatingOrigin (test_floating_origin.cpp)
// to planet-scale and beyond-planet-scale magnitudes — up to ~1e12, several orders
// past Earth's ~6.378e6 m radius — proving ARCHITECTURE.md §6's claim that the
// local offset stays representable (and round-trips accurately) in single
// precision no matter how far `world` is from the origin, as long as the
// decomposition into sector + local offset is applied. This is still plain
// host-side GoogleTest, no SYCL/runtime involvement, matching test_floating_origin.cpp.

#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>
#include <SushiEngine/core/types.hpp>
#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    constexpr double SECTOR_SIZE = 1024.0;

    // The tolerance a round trip must meet, scaled to the magnitude under test:
    // a Scalar-precision local offset carries roughly its own type's relative
    // precision (~1e-7 for float, ~1e-15 for double), independent of how far the
    // sector itself is from the origin, since the offset is always < SECTOR_SIZE.
    double round_trip_tolerance()
    {
#ifdef SE_SCALAR_DOUBLE
        return SECTOR_SIZE * 1e-12;
#else
        return SECTOR_SIZE * 1e-5;
#endif
    }
}

TEST(Unit_FloatingOriginStress, LocalOffsetStaysSmallAtEarthRadius)
{
    // Earth's mean radius, a representative planet-scale magnitude.
    const WorldVector3 earth_radius{6'378'137.0, 6'378'137.0, 6'378'137.0};
    const FloatingOriginVector3 decomposed = to_floating_origin(earth_radius, SECTOR_SIZE);

    EXPECT_GE(decomposed.local.x, Scalar(0));
    EXPECT_LT(decomposed.local.x, Scalar(SECTOR_SIZE));
    EXPECT_GE(decomposed.local.y, Scalar(0));
    EXPECT_LT(decomposed.local.y, Scalar(SECTOR_SIZE));
    EXPECT_GE(decomposed.local.z, Scalar(0));
    EXPECT_LT(decomposed.local.z, Scalar(SECTOR_SIZE));
}

TEST(Unit_FloatingOriginStress, RoundTripsAtEarthRadius)
{
    const WorldVector3 original{6'378'137.0, -6'378'137.0, 4'200'000.5};
    const FloatingOriginVector3 decomposed = to_floating_origin(original, SECTOR_SIZE);
    const WorldVector3 recomposed = from_floating_origin(decomposed, SECTOR_SIZE);

    const double tol = round_trip_tolerance();
    EXPECT_NEAR(recomposed.x, original.x, tol);
    EXPECT_NEAR(recomposed.y, original.y, tol);
    EXPECT_NEAR(recomposed.z, original.z, tol);
}

TEST(Unit_FloatingOriginStress, RoundTripsAtInterplanetaryScale1e9)
{
    const WorldVector3 original{1'234'567'890.125, -987'654'321.75, 555'555'555.5};
    const FloatingOriginVector3 decomposed = to_floating_origin(original, SECTOR_SIZE);
    const WorldVector3 recomposed = from_floating_origin(decomposed, SECTOR_SIZE);

    EXPECT_GE(decomposed.local.x, Scalar(0));
    EXPECT_LT(decomposed.local.x, Scalar(SECTOR_SIZE));

    const double tol = round_trip_tolerance();
    EXPECT_NEAR(recomposed.x, original.x, tol);
    EXPECT_NEAR(recomposed.y, original.y, tol);
    EXPECT_NEAR(recomposed.z, original.z, tol);
}

TEST(Unit_FloatingOriginStress, RoundTripsAtExtremeScale1e12)
{
    const WorldVector3 original{1'000'000'000'000.25, -1'000'000'000'000.75, 999'999'999'999.5};
    const FloatingOriginVector3 decomposed = to_floating_origin(original, SECTOR_SIZE);
    const WorldVector3 recomposed = from_floating_origin(decomposed, SECTOR_SIZE);

    EXPECT_GE(decomposed.local.x, Scalar(0));
    EXPECT_LT(decomposed.local.x, Scalar(SECTOR_SIZE));
    EXPECT_GE(decomposed.local.y, Scalar(0));
    EXPECT_LT(decomposed.local.y, Scalar(SECTOR_SIZE));

    const double tol = round_trip_tolerance();
    EXPECT_NEAR(recomposed.x, original.x, tol);
    EXPECT_NEAR(recomposed.y, original.y, tol);
    EXPECT_NEAR(recomposed.z, original.z, tol);
}

TEST(Unit_FloatingOriginStress, NegativeExtremeScaleFloorsTowardLowerSector)
{
    const WorldVector3 original{-1'000'000'000'000.0, 0.0, 0.0};
    const FloatingOriginVector3 decomposed = to_floating_origin(original, SECTOR_SIZE);

    // The sector's own corner must be <= the original coordinate.
    const double corner_x = static_cast<double>(decomposed.sector.x) * SECTOR_SIZE;
    EXPECT_LE(corner_x, original.x);
    EXPECT_GT(corner_x + SECTOR_SIZE, original.x);

    const WorldVector3 recomposed = from_floating_origin(decomposed, SECTOR_SIZE);
    EXPECT_NEAR(recomposed.x, original.x, round_trip_tolerance());
}

TEST(Unit_FloatingOriginStress, DistinctNearbyPointsResolveToDistinctOrAdjacentSectors)
{
    // Two points 10 meters apart at planet-scale distance from the origin must
    // still decompose into nearby sectors (never a wild jump), and their local
    // offsets must reflect the same physical separation once re-expressed in a
    // common frame — proving the decomposition does not lose the relationship
    // between nearby points even far from the origin.
    const WorldVector3 a{6'378'137.0, 6'378'137.0, 6'378'137.0};
    const WorldVector3 b{6'378'147.0, 6'378'137.0, 6'378'137.0}; // 10 m along x

    const FloatingOriginVector3 da = to_floating_origin(a, SECTOR_SIZE);
    const FloatingOriginVector3 db = to_floating_origin(b, SECTOR_SIZE);

    EXPECT_LE(std::llabs(static_cast<long long>(db.sector.x - da.sector.x)), 1);

    const WorldVector3 ra = from_floating_origin(da, SECTOR_SIZE);
    const WorldVector3 rb = from_floating_origin(db, SECTOR_SIZE);
    EXPECT_NEAR(rb.x - ra.x, 10.0, round_trip_tolerance());
}
