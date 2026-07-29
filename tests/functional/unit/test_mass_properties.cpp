/**************************************************************************/
/* test_mass_properties.cpp                                               */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_MassProperties: mass and inertia derived from a shape and a density.
//
// These were written in P0 and only wired up in P2, which is exactly the interval
// in which an untested closed form goes wrong without anyone noticing. The
// assertions are of two kinds. The primitives are checked against the textbook
// formula written out independently here. The capsule — the one added for P2, and
// the only one with a term that is easy to drop — is checked against its own
// limits: with no segment it must *be* a sphere, and as its caps shrink its
// inertia must approach the cylinder's. A dropped parallel-axis term passes
// neither.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/geometry/mass_properties.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;
    constexpr Real pi = 3.14159265358979323846;
} // namespace

TEST(Unit_MassProperties, SphereMatchesTwoFifthsMassRadiusSquared)
{
    const MassProperties<Real> properties = sphere_mass_properties<Real>(2.0, 500.0);
    const Real mass = (4.0 / 3.0) * pi * 8.0 * 500.0;
    EXPECT_NEAR(properties.mass, mass, 1e-9);
    EXPECT_NEAR(properties.inertia.x, 0.4 * mass * 4.0, 1e-9);
    EXPECT_NEAR(properties.inertia.x, properties.inertia.y, 1e-12);
    EXPECT_NEAR(properties.inertia.y, properties.inertia.z, 1e-12);
}

TEST(Unit_MassProperties, BoxMatchesTheTwelfthOfTheSquaredExtents)
{
    const MassProperties<Real> properties =
        box_mass_properties<Real>(Vector3T<Real>{1.0, 2.0, 3.0}, 100.0);
    const Real mass = 2.0 * 4.0 * 6.0 * 100.0;
    EXPECT_NEAR(properties.mass, mass, 1e-9);
    // I_x = m (height^2 + depth^2) / 12, with full extents.
    EXPECT_NEAR(properties.inertia.x, mass * (16.0 + 36.0) / 12.0, 1e-9);
    EXPECT_NEAR(properties.inertia.z, mass * (4.0 + 16.0) / 12.0, 1e-9);
}

TEST(Unit_MassProperties, ACubeIsIsotropicAndALongBoxIsNot)
{
    const MassProperties<Real> cube =
        box_mass_properties<Real>(Vector3T<Real>{1.0, 1.0, 1.0}, 1.0);
    EXPECT_NEAR(cube.inertia.x, cube.inertia.y, 1e-12);
    EXPECT_NEAR(cube.inertia.y, cube.inertia.z, 1e-12);

    // A plank is easiest to spin about its long axis, which is the property that
    // makes a wrongly-typed inertia tensor look plausible and behave wrongly.
    const MassProperties<Real> plank =
        box_mass_properties<Real>(Vector3T<Real>{4.0, 0.1, 1.0}, 1.0);
    EXPECT_LT(plank.inertia.x, plank.inertia.y);
    EXPECT_LT(plank.inertia.x, plank.inertia.z);
}

TEST(Unit_MassProperties, CylinderIsHalfMassRadiusSquaredAboutItsAxis)
{
    const MassProperties<Real> properties = cylinder_mass_properties<Real>(2.0, 3.0, 10.0);
    const Real mass = pi * 4.0 * 6.0 * 10.0;
    EXPECT_NEAR(properties.mass, mass, 1e-9);
    EXPECT_NEAR(properties.inertia.y, 0.5 * mass * 4.0, 1e-9);
    EXPECT_NEAR(properties.inertia.x, mass * (3.0 * 4.0 + 36.0) / 12.0, 1e-9);
}

TEST(Unit_MassProperties, ACapsuleWithNoSegmentIsASphere)
{
    // The limit that catches a dropped hemisphere term outright.
    const MassProperties<Real> capsule = capsule_mass_properties<Real>(1.5, 0.0, 800.0);
    const MassProperties<Real> sphere = sphere_mass_properties<Real>(1.5, 800.0);
    EXPECT_NEAR(capsule.mass, sphere.mass, 1e-9);
    EXPECT_NEAR(capsule.inertia.x, sphere.inertia.x, 1e-9);
    EXPECT_NEAR(capsule.inertia.y, sphere.inertia.y, 1e-9);
}

TEST(Unit_MassProperties, ACapsuleApproachesItsCylinderAsTheCapsShrink)
{
    // As the radius goes to nothing beside the length, the caps stop mattering and
    // the capsule must converge on the cylinder of the same dimensions. A capsule
    // missing its parallel-axis shift converges on something noticeably smaller.
    Real previous = 1.0;
    for (const Real radius : {0.5, 0.1, 0.02, 0.004})
    {
        const MassProperties<Real> capsule = capsule_mass_properties<Real>(radius, 5.0, 1000.0);
        const MassProperties<Real> cylinder = cylinder_mass_properties<Real>(radius, 5.0, 1000.0);
        const Real error = std::abs(capsule.inertia.x - cylinder.inertia.x) / cylinder.inertia.x;
        EXPECT_LT(error, previous) << "radius " << radius;
        previous = error;
    }
    EXPECT_LT(previous, 0.01);
}

TEST(Unit_MassProperties, ACapsuleIsHarderToTumbleThanToSpin)
{
    const MassProperties<Real> capsule = capsule_mass_properties<Real>(0.3, 2.0, 1000.0);
    EXPECT_GT(capsule.inertia.x, capsule.inertia.y * 5.0);
    EXPECT_NEAR(capsule.inertia.x, capsule.inertia.z, 1e-12);
}

TEST(Unit_MassProperties, TheParallelAxisShiftGrowsWithTheSquareOfTheOffset)
{
    const Vector3T<Real> inertia{1.0, 1.0, 1.0};
    const Vector3T<Real> shifted =
        shift_inertia<Real>(inertia, 2.0, Vector3T<Real>{0.0, 3.0, 0.0});
    EXPECT_NEAR(shifted.x, 1.0 + 2.0 * 9.0, 1e-12);
    EXPECT_NEAR(shifted.y, 1.0, 1e-12) << "an offset along Y does not move the Y axis";
    EXPECT_NEAR(shifted.z, 1.0 + 2.0 * 9.0, 1e-12);
}

TEST(Unit_MassProperties, ZeroSurvivesInversionAsCannotRotate)
{
    // The encoding `RigidBodyT` already uses: zero inverse inertia is an axis the
    // body cannot rotate about, so inverting must not turn it into an infinity.
    const Vector3T<Real> inverse = to_inverse<Real>(Vector3T<Real>{4.0, 0.0, -1.0});
    EXPECT_NEAR(inverse.x, 0.25, 1e-12);
    EXPECT_EQ(inverse.y, 0.0);
    EXPECT_EQ(inverse.z, 0.0);
    EXPECT_EQ(inverse_mass<Real>(0.0), 0.0);
}

TEST(Unit_MassProperties, TheColliderOverloadsCarryTheShapesOwnCentre)
{
    SphereCollider<Real> sphere;
    sphere.center = Vector3T<Real>{1.0, 2.0, 3.0};
    sphere.radius = 1.0;
    EXPECT_NEAR(mass_properties_of(sphere, 1000.0).center_of_mass.y, 2.0, 1e-12);

    CapsuleCollider<Real> capsule;
    capsule.center = Vector3T<Real>{0.0, 5.0, 0.0};
    capsule.radius = 0.4;
    capsule.half_height = 1.0;
    const MassProperties<Real> properties = mass_properties_of(capsule, 1000.0);
    EXPECT_NEAR(properties.center_of_mass.y, 5.0, 1e-12);
    EXPECT_NEAR(properties.mass, capsule_mass_properties<Real>(0.4, 1.0, 1000.0).mass, 1e-12);
}
