/**************************************************************************/
/* test_floating_origin.cpp                                              */
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

// Unit_FloatingOrigin: WorldVec3 <-> FloatingOriginVec3 round-tripping. Planet-scale
// ECEF coordinates must decompose into a sector index plus a small local offset, and
// recomposing must return the original position (to Scalar precision), which is the
// whole point of the floating origin: it keeps `local` representable near the origin
// even when `world` is not.

#include <cstdint>
#include <gtest/gtest.h>
#include <SushiEngine/core/types.hpp>
#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    constexpr double SECTOR_SIZE = 1024.0;
}

TEST(Unit_FloatingOrigin, OriginIsSectorZeroWithZeroLocal)
{
    const FloatingOriginVec3 origin = to_floating_origin(WorldVec3{0, 0, 0}, SECTOR_SIZE);

    EXPECT_EQ(origin.sector, (SectorCoord{0, 0, 0}));
    EXPECT_TRUE(Harness::approx_equal(origin.local, Vec3{0, 0, 0}, Scalar(1e-6)));
}

TEST(Unit_FloatingOrigin, LocalOffsetStaysWithinOneSector)
{
    // A position far from the world origin: the local offset must still be small,
    // which is the property that keeps it representable in single precision.
    const WorldVec3 far{6'378'137.0, -1'234'567.5, 42.0};
    const FloatingOriginVec3 decomposed = to_floating_origin(far, SECTOR_SIZE);

    EXPECT_GE(decomposed.local.x, Scalar(0));
    EXPECT_LT(decomposed.local.x, Scalar(SECTOR_SIZE));
    EXPECT_GE(decomposed.local.y, Scalar(0));
    EXPECT_LT(decomposed.local.y, Scalar(SECTOR_SIZE));
    EXPECT_GE(decomposed.local.z, Scalar(0));
    EXPECT_LT(decomposed.local.z, Scalar(SECTOR_SIZE));
}

TEST(Unit_FloatingOrigin, RoundTripsThroughSectorAndBack)
{
    const WorldVec3 original{6'378'137.0, -1'234'567.5, 42.0};

    const FloatingOriginVec3 decomposed = to_floating_origin(original, SECTOR_SIZE);
    const WorldVec3 recomposed = from_floating_origin(decomposed, SECTOR_SIZE);

    EXPECT_NEAR(recomposed.x, original.x, 1e-3);
    EXPECT_NEAR(recomposed.y, original.y, 1e-3);
    EXPECT_NEAR(recomposed.z, original.z, 1e-3);
}

TEST(Unit_FloatingOrigin, NegativeCoordinatesFloorTowardLowerSector)
{
    // -1.0 with a sector size of 1024 belongs to sector -1 (its corner at -1024),
    // not sector 0 — a naive truncating division would misplace it.
    const FloatingOriginVec3 decomposed =
        to_floating_origin(WorldVec3{-1.0, 0.0, 0.0}, SECTOR_SIZE);

    EXPECT_EQ(decomposed.sector.x, -1);
    EXPECT_NEAR(static_cast<double>(decomposed.local.x), SECTOR_SIZE - 1.0, 1e-3);
}
