/**************************************************************************/
/* test_gravity.cpp                                                      */
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

// Unit_Gravity: the summed on-rails gravitational field. The Newtonian one-body pull
// must obey the inverse-square law and point at the source; the Sun must sit at the
// heliocentric origin and Earth ~1 AU out in metres; spheres of influence must size to
// the known ~0.9 million km for Earth; and the field near Earth must be dominated by
// Earth. A single body's surface pull recovers standard surface gravity as a cross-check.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/astro/celestial_bodies.hpp>
#include <SushiEngine/astro/gravity.hpp>
#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/astro/orbital_elements.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Astro;

namespace
{
    double magnitude(const WorldVector3& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }
}

TEST(Unit_Gravity, GravitationalParametersArePositiveWithSunDominant)
{
    for (int index = 0; index < BODY_COUNT; ++index)
        EXPECT_GT(standard_gravitational_parameter(static_cast<BodyId>(index)), 0.0);

    EXPECT_NEAR(standard_gravitational_parameter(BodyId::Sun), 1.32712440018e20, 1e10);
    EXPECT_NEAR(standard_gravitational_parameter(BodyId::Earth), 3.986004418e14, 1e6);
    // The Sun's mu dominates every planet by orders of magnitude.
    EXPECT_GT(standard_gravitational_parameter(BodyId::Sun),
              1e5 * standard_gravitational_parameter(BodyId::Jupiter));
}

TEST(Unit_Gravity, GravitationalParentHierarchy)
{
    EXPECT_EQ(gravitational_parent(BodyId::Moon), BodyId::Earth);
    EXPECT_EQ(gravitational_parent(BodyId::Earth), BodyId::Sun);
    EXPECT_EQ(gravitational_parent(BodyId::Jupiter), BodyId::Sun);
}

TEST(Unit_Gravity, PointGravityObeysInverseSquareAndPointsInward)
{
    const double r = 6.371e6; // Earth mean radius
    const WorldVector3 accel = body_point_gravity(BodyId::Earth, WorldVector3{r, 0.0, 0.0});

    // Pull is along -x (toward the centre) and equals standard surface gravity.
    EXPECT_LT(accel.x, 0.0);
    EXPECT_NEAR(accel.y, 0.0, 1e-12);
    EXPECT_NEAR(accel.z, 0.0, 1e-12);
    EXPECT_NEAR(magnitude(accel),
                standard_gravitational_parameter(BodyId::Earth) / (r * r), 1e-6);

    // Doubling the distance quarters the acceleration.
    const WorldVector3 far_accel = body_point_gravity(BodyId::Earth, WorldVector3{2.0 * r, 0.0, 0.0});
    EXPECT_NEAR(magnitude(far_accel), 0.25 * magnitude(accel), 1e-9);
}

TEST(Unit_Gravity, PointGravityIsGuardedAtTheCentreAndForMasslessBodies)
{
    EXPECT_NEAR(magnitude(body_point_gravity(BodyId::Earth, WorldVector3{0.0, 0.0, 0.0})), 0.0,
                1e-30);
    // BodyId::Count has no mu -> no pull.
    EXPECT_NEAR(magnitude(body_point_gravity(BodyId::Count, WorldVector3{1.0, 0.0, 0.0})), 0.0,
                1e-30);
}

TEST(Unit_Gravity, SunSitsAtOriginAndEarthOneAstronomicalUnitOutInMetres)
{
    const WorldVector3 sun = body_heliocentric_metres(BodyId::Sun, J2000_JULIAN_DATE);
    EXPECT_NEAR(magnitude(sun), 0.0, 1e-6);

    const WorldVector3 earth = body_heliocentric_metres(BodyId::Earth, J2000_JULIAN_DATE);
    EXPECT_NEAR(magnitude(earth), METRES_PER_ASTRONOMICAL_UNIT, 0.02 * METRES_PER_ASTRONOMICAL_UNIT);
}

TEST(Unit_Gravity, SphereOfInfluenceSizesAreKnownValues)
{
    EXPECT_DOUBLE_EQ(sphere_of_influence_radius(BodyId::Sun), 0.0);

    // Earth's SOI is ~0.924 million km; the Moon's ~66000 km.
    EXPECT_NEAR(sphere_of_influence_radius(BodyId::Earth), 9.24e8, 1.0e8);
    EXPECT_NEAR(sphere_of_influence_radius(BodyId::Moon), 6.6e7, 1.0e7);

    for (int index = 1; index < BODY_COUNT; ++index)
        EXPECT_GT(sphere_of_influence_radius(static_cast<BodyId>(index)), 0.0);
}

TEST(Unit_Gravity, FieldNearEarthIsDominatedByEarth)
{
    const WorldVector3 earth = body_heliocentric_metres(BodyId::Earth, J2000_JULIAN_DATE);
    const double offset = 1.0e7; // 10000 km from Earth's centre, inside its SOI
    const WorldVector3 point{earth.x + offset, earth.y, earth.z};

    const WorldVector3 accel = gravity_field(point, J2000_JULIAN_DATE);

    // The dominant pull is back toward Earth's centre: the acceleration is nearly
    // antiparallel to the outward offset direction.
    const double toward_earth = -accel.x; // offset was along +x from Earth
    EXPECT_GT(toward_earth, 0.0);
    // Earth's contribution alone is mu/offset^2 ~ 3.99 m/s^2; the field is close to it.
    EXPECT_NEAR(magnitude(accel),
                standard_gravitational_parameter(BodyId::Earth) / (offset * offset), 0.5);
}

TEST(Unit_Gravity, IntegrateStepIsDeterministicToTheBit)
{
    const StateVector state{WorldVector3{1.5e11, 0.0, 0.0}, WorldVector3{0.0, 2.0e4, 0.0}};
    const StateVector a = integrate_step(state, J2000_JULIAN_DATE, 60.0);
    const StateVector b = integrate_step(state, J2000_JULIAN_DATE, 60.0);
    EXPECT_EQ(a.position.x, b.position.x);
    EXPECT_EQ(a.position.y, b.position.y);
    EXPECT_EQ(a.velocity.x, b.velocity.x);
    EXPECT_EQ(a.velocity.y, b.velocity.y);
}

TEST(Unit_Gravity, IntegrateStepPullsAStationaryBodyTowardTheSun)
{
    // A body at rest on the +x axis, one AU out, is pulled back toward the Sun (-x).
    const StateVector state{WorldVector3{METRES_PER_ASTRONOMICAL_UNIT, 0.0, 0.0},
                            WorldVector3{0.0, 0.0, 0.0}};
    const StateVector next = integrate_step(state, J2000_JULIAN_DATE, 100.0);
    EXPECT_LT(next.velocity.x, 0.0);
    EXPECT_LT(next.position.x, METRES_PER_ASTRONOMICAL_UNIT);
}
