/**************************************************************************/
/* test_fem_plasticity.cpp                                                */
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

// Unit_FEMPlasticity: §9.4's permanent dent (physics/soft/fem_plasticity.hpp),
// checked against the property the whole design hinges on — full creep
// (plastic_creep = 1) must make the *current* shape the new rest shape
// exactly, so recomputing the deformation gradient against the just-updated
// plastic inverse reports zero elastic strain — plus the two guards a
// from-scratch accumulator most needs: it must not move at all below yield,
// and it must never overshoot its own ceiling.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/fem_plasticity.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A unit right tetrahedron's rest state: Dm = I, so Dm^-1 = I too. */
    FEMTetrahedron unit_rest_element()
    {
        FEMTetrahedron element;
        element.vertex[0] = 0;
        element.vertex[1] = 1;
        element.vertex[2] = 2;
        element.vertex[3] = 3;
        element.rest_inverse_column_0 = Vector3{1.0, 0.0, 0.0};
        element.rest_inverse_column_1 = Vector3{0.0, 1.0, 0.0};
        element.rest_inverse_column_2 = Vector3{0.0, 0.0, 1.0};
        element.plastic_inverse_column_0 = element.rest_inverse_column_0;
        element.plastic_inverse_column_1 = element.rest_inverse_column_1;
        element.plastic_inverse_column_2 = element.rest_inverse_column_2;
        element.rest_volume = 1.0 / 6.0;
        return element;
    }

    /** @brief Four stationary particles at the given world positions. */
    void place(RigidBody bodies[4], Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3)
    {
        bodies[0].position = p0;
        bodies[1].position = p1;
        bodies[2].position = p2;
        bodies[3].position = p3;
    }

    /** @brief `M` applied to `v`, via `M`'s own columns — the convention this file uses throughout. */
    Vector3 apply(const FEMMatrix3<Scalar>& m, Vector3 v)
    {
        return m.column0 * v.x + m.column1 * v.y + m.column2 * v.z;
    }
} // namespace

// Inverting the identity must return the identity.
TEST(Unit_FEMPlasticity, InvertingTheIdentityReturnsTheIdentity)
{
    FEMMatrix3<Scalar> identity;
    identity.column0 = Vector3{1.0, 0.0, 0.0};
    identity.column1 = Vector3{0.0, 1.0, 0.0};
    identity.column2 = Vector3{0.0, 0.0, 1.0};

    FEMMatrix3<Scalar> inverse;
    ASSERT_TRUE(invert_fem_matrix3(identity, inverse));
    EXPECT_NEAR(inverse.column0.x, 1.0, 1e-12);
    EXPECT_NEAR(inverse.column1.y, 1.0, 1e-12);
    EXPECT_NEAR(inverse.column2.z, 1.0, 1e-12);
}

// M * M^-1 must be the identity for a genuinely non-trivial matrix — scaled
// per axis and sheared, so a bug that only cancels out for a diagonal matrix
// cannot hide here.
TEST(Unit_FEMPlasticity, InvertedMatrixRoundTripsToTheIdentity)
{
    FEMMatrix3<Scalar> m;
    m.column0 = Vector3{2.0, 0.5, 0.0};
    m.column1 = Vector3{0.0, 3.0, 0.0};
    m.column2 = Vector3{0.3, 0.0, 0.7};

    FEMMatrix3<Scalar> inverse;
    ASSERT_TRUE(invert_fem_matrix3(m, inverse));

    const Vector3 e0 = apply(m, inverse.column0);
    const Vector3 e1 = apply(m, inverse.column1);
    const Vector3 e2 = apply(m, inverse.column2);
    EXPECT_NEAR(e0.x, 1.0, 1e-9);
    EXPECT_NEAR(e0.y, 0.0, 1e-9);
    EXPECT_NEAR(e0.z, 0.0, 1e-9);
    EXPECT_NEAR(e1.x, 0.0, 1e-9);
    EXPECT_NEAR(e1.y, 1.0, 1e-9);
    EXPECT_NEAR(e1.z, 0.0, 1e-9);
    EXPECT_NEAR(e2.x, 0.0, 1e-9);
    EXPECT_NEAR(e2.y, 0.0, 1e-9);
    EXPECT_NEAR(e2.z, 1.0, 1e-9);
}

// A deformation that never reaches yield_stress must leave the element
// exactly as it was: no creep, no accumulated strain.
TEST(Unit_FEMPlasticity, BelowYieldNothingMoves)
{
    FEMTetrahedron element = unit_rest_element();
    element.von_mises_stress = 100.0; // below yield, set as if just measured

    SoftBodyMaterial material;
    material.yield_stress = 1000.0;
    material.plastic_creep = 0.5;
    material.maximum_plastic_strain = 1.0;

    RigidBody bodies[4];
    place(bodies, Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}, Vector3{0, 0, 1});

    apply_fem_plasticity<Scalar>(bodies, element, material);

    EXPECT_NEAR(element.plastic_inverse_column_0.x, 1.0, 1e-12);
    EXPECT_NEAR(element.accumulated_plastic_strain, 0.0, 1e-12);
}

// Full creep (plastic_creep = 1) must make the *current* shape the new rest
// shape exactly: recomputing F against the just-updated plastic inverse must
// report zero elastic strain, because that is the whole point of the
// derivation in this file's header comment.
TEST(Unit_FEMPlasticity, FullCreepZeroesTheElasticStrainItJustAbsorbed)
{
    FEMTetrahedron element = unit_rest_element();
    element.von_mises_stress = 1.0e9; // far past yield

    SoftBodyMaterial material;
    material.yield_stress = 1.0;
    material.plastic_creep = 1.0; // instant, full creep
    material.maximum_plastic_strain = 100.0; // effectively unbounded here

    RigidBody bodies[4];
    // A stretched, sheared configuration -- deliberately not symmetric, so a
    // bug that only zeroes out for an axis-aligned stretch cannot hide.
    place(bodies, Vector3{0, 0, 0}, Vector3{1.6, 0.1, 0.0}, Vector3{0.2, 1.4, 0.05},
          Vector3{-0.1, 0.0, 1.3});

    apply_fem_plasticity<Scalar>(bodies, element, material);

    const Vector3 edge1 = bodies[1].position - bodies[0].position;
    const Vector3 edge2 = bodies[2].position - bodies[0].position;
    const Vector3 edge3 = bodies[3].position - bodies[0].position;
    const FEMMatrix3<Scalar> elastic_after = tetrahedron_deformation_gradient(
        edge1, edge2, edge3, element.plastic_inverse_column_0, element.plastic_inverse_column_1,
        element.plastic_inverse_column_2);

    EXPECT_NEAR(elastic_after.column0.x, 1.0, 1e-6);
    EXPECT_NEAR(elastic_after.column0.y, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column0.z, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column1.x, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column1.y, 1.0, 1e-6);
    EXPECT_NEAR(elastic_after.column1.z, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column2.x, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column2.y, 0.0, 1e-6);
    EXPECT_NEAR(elastic_after.column2.z, 1.0, 1e-6);

    // And the rest volume must have followed the new rest shape rather than
    // staying at the original unit tetrahedron's 1/6.
    EXPECT_GT(element.accumulated_plastic_strain, 0.0);
    EXPECT_NE(element.rest_volume, 1.0 / 6.0);
}

// The accumulator must never exceed maximum_plastic_strain, however large a
// single tick's creep would otherwise take it — repeated yielding ticks must
// converge to the ceiling and stop there, not overshoot it once and stay.
TEST(Unit_FEMPlasticity, AccumulatedStrainNeverExceedsItsCeiling)
{
    FEMTetrahedron element = unit_rest_element();
    SoftBodyMaterial material;
    material.yield_stress = 1.0;
    material.plastic_creep = 0.5;
    material.maximum_plastic_strain = 0.05; // a tight ceiling, reached in a few ticks

    RigidBody bodies[4];
    place(bodies, Vector3{0, 0, 0}, Vector3{1.5, 0, 0}, Vector3{0, 1.5, 0}, Vector3{0, 0, 1.5});

    for (int tick = 0; tick < 50; ++tick)
    {
        // Re-measure "stress" each tick the crude way a real element would:
        // any deformation this far from rest is well past this test's yield.
        element.von_mises_stress = 1.0e6;
        apply_fem_plasticity<Scalar>(bodies, element, material);
        ASSERT_LE(element.accumulated_plastic_strain, material.maximum_plastic_strain);
    }
    // And it must have actually reached the ceiling, not stalled short of it.
    EXPECT_NEAR(element.accumulated_plastic_strain, material.maximum_plastic_strain, 1e-6);
}
