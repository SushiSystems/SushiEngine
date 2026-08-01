/**************************************************************************/
/* test_fem_stress.cpp                                                   */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

// Unit_FemStress: §9.3's readout (physics/soft/fem_stress.hpp), checked
// against the one case a from-scratch stress tensor can be checked against
// with total confidence: the small-strain, uniaxial-*strain* condition
// (stretched along one axis, the other two held fixed), whose longitudinal
// stress is the textbook linear-elasticity closed form
// `sigma_xx = (lambda + 2*mu) * epsilon_xx` — the P-wave modulus result,
// independent of anything this file itself computes.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/fem_stress.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief `F` as three unit columns — the identity deformation gradient. */
    FemMatrix3<Scalar> identity_gradient()
    {
        FemMatrix3<Scalar> f;
        f.column0 = Vector3{1.0, 0.0, 0.0};
        f.column1 = Vector3{0.0, 1.0, 0.0};
        f.column2 = Vector3{0.0, 0.0, 1.0};
        return f;
    }
} // namespace

// At rest, F = I: the Green strain, both stress tensors, and the von Mises
// scalar must all be exactly zero — an undeformed element reports no stress.
TEST(Unit_FemStress, RestConfigurationReportsZeroStress)
{
    const FemMatrix3<Scalar> f = identity_gradient();
    const FemSymmetricMatrix3<Scalar> strain = green_lagrange_strain(f);
    EXPECT_NEAR(strain.xx, 0.0, 1e-12);
    EXPECT_NEAR(strain.yy, 0.0, 1e-12);
    EXPECT_NEAR(strain.zz, 0.0, 1e-12);
    EXPECT_NEAR(strain.xy, 0.0, 1e-12);
    EXPECT_NEAR(strain.yz, 0.0, 1e-12);
    EXPECT_NEAR(strain.zx, 0.0, 1e-12);

    const Scalar mu = 1.0e6;
    const Scalar lambda = 2.0e6;
    const FemSymmetricMatrix3<Scalar> stress = second_piola_kirchhoff_stress(strain, mu, lambda);
    EXPECT_NEAR(stress.xx, 0.0, 1e-9);
    EXPECT_NEAR(stress.yy, 0.0, 1e-9);
    EXPECT_NEAR(stress.zz, 0.0, 1e-9);

    const FemSymmetricMatrix3<Scalar> cauchy = cauchy_stress(f, stress);
    EXPECT_NEAR(von_mises_stress(cauchy), 0.0, 1e-9);
}

// A small uniaxial-strain extension along X (the other two axes held fixed —
// F = diag(1+e, 1, 1)) has a known closed-form longitudinal stress and a
// known von Mises value, both independent of this file's own machinery:
// sigma_xx = (lambda + 2*mu) * e, sigma_yy = sigma_zz = lambda * e, and the
// von Mises reduction of that diagonal state is exactly 2*mu*e (to first
// order in e, which a strain this small is well within).
TEST(Unit_FemStress, SmallUniaxialStrainMatchesTheLinearElasticityClosedForm)
{
    const Scalar mu = 1.0e6;
    const Scalar lambda = 2.0e6;
    const Scalar e = 1.0e-4; // small enough that O(e^2) is below the tolerance below

    FemMatrix3<Scalar> f = identity_gradient();
    f.column0 = Vector3{1.0 + e, 0.0, 0.0};

    const FemSymmetricMatrix3<Scalar> strain = green_lagrange_strain(f);
    // E_xx = ((1+e)^2 - 1)/2 = e + e^2/2 ~= e for this e.
    EXPECT_NEAR(strain.xx, e, 1e-3 * e);
    EXPECT_NEAR(strain.yy, 0.0, 1e-12);
    EXPECT_NEAR(strain.zz, 0.0, 1e-12);

    const FemSymmetricMatrix3<Scalar> stress = second_piola_kirchhoff_stress(strain, mu, lambda);
    const FemSymmetricMatrix3<Scalar> cauchy = cauchy_stress(f, stress);

    const Scalar expected_xx = (lambda + Scalar(2) * mu) * e;
    const Scalar expected_yy = lambda * e;
    EXPECT_NEAR(cauchy.xx, expected_xx, expected_xx * 1e-2);
    EXPECT_NEAR(cauchy.yy, expected_yy, std::abs(expected_yy) * 1e-2 + 1e-6);
    EXPECT_NEAR(cauchy.zz, expected_yy, std::abs(expected_yy) * 1e-2 + 1e-6);

    const Scalar expected_von_mises = Scalar(2) * mu * e;
    EXPECT_NEAR(von_mises_stress(cauchy), expected_von_mises, expected_von_mises * 1e-2);
}

// The Green strain — and therefore everything built on it — must be
// unaffected by a pure rotation of F, the same rotation-invariance property
// the deviatoric XPBD constraint has, checked here against the stress readout
// instead: a spinning, undeformed element must report zero stress just as a
// still one does.
TEST(Unit_FemStress, PureRotationStillReportsZeroStress)
{
    // A deformation gradient that is a rotation matrix has, by construction,
    // orthonormal columns; build one directly rather than by rotating a
    // Vector3, since FemMatrix3 has no rotate() of its own.
    const Scalar angle = 0.7;
    const Scalar c = std::cos(angle);
    const Scalar s = std::sin(angle);
    FemMatrix3<Scalar> f;
    f.column0 = Vector3{c, s, 0.0};
    f.column1 = Vector3{-s, c, 0.0};
    f.column2 = Vector3{0.0, 0.0, 1.0};

    const FemSymmetricMatrix3<Scalar> strain = green_lagrange_strain(f);
    EXPECT_NEAR(strain.xx, 0.0, 1e-9);
    EXPECT_NEAR(strain.yy, 0.0, 1e-9);
    EXPECT_NEAR(strain.zz, 0.0, 1e-9);
    EXPECT_NEAR(strain.xy, 0.0, 1e-9);
    EXPECT_NEAR(strain.yz, 0.0, 1e-9);
    EXPECT_NEAR(strain.zx, 0.0, 1e-9);

    const FemSymmetricMatrix3<Scalar> stress =
        second_piola_kirchhoff_stress(strain, 1.0e6, 2.0e6);
    const FemSymmetricMatrix3<Scalar> cauchy = cauchy_stress(f, stress);
    EXPECT_NEAR(von_mises_stress(cauchy), 0.0, 1e-6);
}

// A degenerate (collapsed) F must report zero stress rather than a division
// by zero or a not-a-number, matching the same guard `fem_projection.hpp`'s
// constraint evaluators already use for the same case.
TEST(Unit_FemStress, DegenerateGradientReportsZeroRatherThanFailing)
{
    FemMatrix3<Scalar> f; // default: all-zero columns, det(F) = 0
    const FemSymmetricMatrix3<Scalar> strain = green_lagrange_strain(f);
    const FemSymmetricMatrix3<Scalar> stress =
        second_piola_kirchhoff_stress(strain, 1.0e6, 2.0e6);
    const FemSymmetricMatrix3<Scalar> cauchy = cauchy_stress(f, stress);
    EXPECT_EQ(cauchy.xx, 0.0);
    EXPECT_EQ(cauchy.yy, 0.0);
    EXPECT_EQ(cauchy.zz, 0.0);
    EXPECT_NEAR(von_mises_stress(cauchy), 0.0, 1e-12);
}
