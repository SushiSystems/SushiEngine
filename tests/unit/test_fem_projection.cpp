/**************************************************************************/
/* test_fem_projection.cpp                                                */
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

// Unit_FEMProjection: §9.1's deformation gradient and its two constraints, in
// isolation from any solver. Host-only pure math, held to two kinds of oracle:
//
// 1. **Closed-form cases** whose answer does not depend on this file's own
//    formulas at all: the rest configuration gives F = I; a uniform scale
//    gives F = s*I with a known determinant and Frobenius norm; a pure
//    rotation gives ||F|| = sqrt(3) exactly, for every rotation, which is the
//    whole reason this deviatoric term does not have to filter rotation out.
// 2. **A finite-difference check against the analytic gradient itself** — the
//    one test that would catch a sign or an index slip in the hand-derived
//    formulas in `fem_projection.hpp`'s file comment, independent of whether
//    the closed-form cases above happen to be symmetric enough to hide one.

#include <cmath>
#include <utility>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/fem_projection.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief A unit right tetrahedron's rest state: Dm = I, so Dm^-1 = I too. */
    struct UnitTetRestState
    {
        Vector3 v0{0.0, 0.0, 0.0};
        Vector3 v1{1.0, 0.0, 0.0};
        Vector3 v2{0.0, 1.0, 0.0};
        Vector3 v3{0.0, 0.0, 1.0};
        Vector3 column0{1.0, 0.0, 0.0};
        Vector3 column1{0.0, 1.0, 0.0};
        Vector3 column2{0.0, 0.0, 1.0};
    };

    FEMMatrix3<Scalar> gradient_at(const UnitTetRestState& rest, const Vector3& p0,
                                   const Vector3& p1, const Vector3& p2, const Vector3& p3)
    {
        return tetrahedron_deformation_gradient(p1 - p0, p2 - p0, p3 - p0, rest.column0,
                                                rest.column1, rest.column2);
    }
} // namespace

// The rest configuration itself must map to the identity: the whole point of
// Dm^-1 being the *inverse* rest matrix is that plugging the rest edges back
// in undoes it exactly.
TEST(Unit_FEMProjection, RestConfigurationGivesTheIdentity)
{
    const UnitTetRestState rest;
    const FEMMatrix3<Scalar> f = gradient_at(rest, rest.v0, rest.v1, rest.v2, rest.v3);

    EXPECT_NEAR(f.column0.x, 1.0, 1e-12);
    EXPECT_NEAR(f.column0.y, 0.0, 1e-12);
    EXPECT_NEAR(f.column0.z, 0.0, 1e-12);
    EXPECT_NEAR(f.column1.x, 0.0, 1e-12);
    EXPECT_NEAR(f.column1.y, 1.0, 1e-12);
    EXPECT_NEAR(f.column1.z, 0.0, 1e-12);
    EXPECT_NEAR(f.column2.x, 0.0, 1e-12);
    EXPECT_NEAR(f.column2.y, 0.0, 1e-12);
    EXPECT_NEAR(f.column2.z, 1.0, 1e-12);

    EXPECT_NEAR(double(frobenius_norm(f)), std::sqrt(3.0), 1e-12);
    EXPECT_NEAR(double(determinant(f)), 1.0, 1e-12);

    // At rest, the deviatoric constraint reads `||I||_F = sqrt(3)` — the norm
    // itself, not a rest-zeroed offset. The rest state sits still not because
    // this value is zero but because the pull it produces is cancelled exactly
    // by the hydrostatic constraint's `mu/lambda` offset; see
    // `evaluate_deviatoric_constraint`'s own documentation.
    const auto deviatoric = evaluate_deviatoric_constraint(f, rest.column0, rest.column1,
                                                           rest.column2);
    EXPECT_NEAR(double(deviatoric.value), std::sqrt(3.0), 1e-12);
}

// A uniform scale by `s` gives `F = s*I`: Frobenius norm `s*sqrt(3)`,
// determinant `s^3` — closed forms with nothing to do with this file's own
// gradient code, so they catch a wrong deformation-gradient formula even
// before the gradients are examined.
TEST(Unit_FEMProjection, UniformScaleMatchesClosedForm)
{
    const UnitTetRestState rest;
    const double s = 1.3;
    const Vector3 p0{0.0, 0.0, 0.0};
    const Vector3 p1{s, 0.0, 0.0};
    const Vector3 p2{0.0, s, 0.0};
    const Vector3 p3{0.0, 0.0, s};
    const FEMMatrix3<Scalar> f = gradient_at(rest, p0, p1, p2, p3);

    EXPECT_NEAR(double(frobenius_norm(f)), s * std::sqrt(3.0), 1e-9);
    EXPECT_NEAR(double(determinant(f)), s * s * s, 1e-9);

    const auto deviatoric = evaluate_deviatoric_constraint(f, rest.column0, rest.column1,
                                                           rest.column2);
    EXPECT_NEAR(double(deviatoric.value), s * std::sqrt(3.0), 1e-9);
}

// A pure rotation leaves the deviatoric constraint at its rest value sqrt(3),
// for any angle — rotation invariance, the property that makes this "stable"
// neo-Hookean rather than a naive strain measure that would report a spinning
// body as deformed.
TEST(Unit_FEMProjection, PureRotationLeavesDeviatoricConstraintAtRestValue)
{
    const UnitTetRestState rest;
    // quaternion_axis_angle assumes a unit axis; it does not normalize one for
    // the caller.
    const Vector3 raw_axis{0.3, 0.7, -0.2};
    const Scalar axis_length =
        std::sqrt(raw_axis.x * raw_axis.x + raw_axis.y * raw_axis.y + raw_axis.z * raw_axis.z);
    const Vector3 axis{raw_axis.x / axis_length, raw_axis.y / axis_length,
                       raw_axis.z / axis_length};
    const Quaternion rotation = quaternion_axis_angle(axis, Scalar(0.9));

    const Vector3 p0{5.0, -2.0, 3.0}; // an arbitrary rigid translation too
    const Vector3 p1 = p0 + rotate(rotation, rest.v1 - rest.v0);
    const Vector3 p2 = p0 + rotate(rotation, rest.v2 - rest.v0);
    const Vector3 p3 = p0 + rotate(rotation, rest.v3 - rest.v0);

    const FEMMatrix3<Scalar> f = gradient_at(rest, p0, p1, p2, p3);
    EXPECT_NEAR(double(frobenius_norm(f)), std::sqrt(3.0), 1e-9);
    EXPECT_NEAR(double(determinant(f)), 1.0, 1e-9);

    const auto deviatoric = evaluate_deviatoric_constraint(f, rest.column0, rest.column1,
                                                           rest.column2);
    EXPECT_NEAR(double(deviatoric.value), std::sqrt(3.0), 1e-9);
}

// Both constraints' gradients must sum to zero across the tetrahedron's four
// vertices — translation invariance, since every edge is `x_k - x_0` and no
// constraint here can see a rigid translation of the whole element.
TEST(Unit_FEMProjection, GradientsSumToZeroAcrossTheFourVertices)
{
    const UnitTetRestState rest;
    const Vector3 p0{0.2, -0.1, 0.05};
    const Vector3 p1{1.3, 0.2, -0.1};
    const Vector3 p2{-0.1, 1.1, 0.3};
    const Vector3 p3{0.05, -0.2, 0.9};
    const FEMMatrix3<Scalar> f = gradient_at(rest, p0, p1, p2, p3);

    const auto deviatoric = evaluate_deviatoric_constraint(f, rest.column0, rest.column1,
                                                           rest.column2);
    const auto hydrostatic =
        evaluate_hydrostatic_constraint(f, rest.column0, rest.column1, rest.column2, 0.3);

    for (const auto* evaluation : {&deviatoric, &hydrostatic})
    {
        Vector3 sum{0.0, 0.0, 0.0};
        for (int i = 0; i < 4; ++i)
            sum = sum + evaluation->gradient[i];
        EXPECT_NEAR(double(sum.x), 0.0, 1e-9);
        EXPECT_NEAR(double(sum.y), 0.0, 1e-9);
        EXPECT_NEAR(double(sum.z), 0.0, 1e-9);
    }
}

// The gold-standard check: the analytic gradient must match a central finite
// difference of the constraint value itself, for a generic (non-symmetric,
// non-rest) configuration. This is the one test that would catch a transposed
// index or a dropped sign in the hand-derived formulas, independent of
// whether the closed-form cases above are symmetric enough to hide one.
TEST(Unit_FEMProjection, AnalyticGradientMatchesFiniteDifference)
{
    const UnitTetRestState rest;
    Vector3 p[4] = {{0.15, -0.05, 0.1}, {1.4, 0.3, -0.15}, {-0.2, 1.2, 0.25}, {0.1, -0.1, 1.05}};
    const double epsilon = 1e-6;

    const auto evaluate_both = [&](const Vector3 points[4]) {
        const FEMMatrix3<Scalar> f =
            gradient_at(rest, points[0], points[1], points[2], points[3]);
        const auto deviatoric =
            evaluate_deviatoric_constraint(f, rest.column0, rest.column1, rest.column2);
        const auto hydrostatic = evaluate_hydrostatic_constraint(f, rest.column0, rest.column1,
                                                                  rest.column2, 0.3);
        return std::make_pair(deviatoric, hydrostatic);
    };

    const auto base = evaluate_both(p);

    for (int vertex = 0; vertex < 4; ++vertex)
    {
        for (int axis = 0; axis < 3; ++axis)
        {
            Vector3 perturbed_up[4] = {p[0], p[1], p[2], p[3]};
            Vector3 perturbed_down[4] = {p[0], p[1], p[2], p[3]};
            double* up_component =
                axis == 0 ? &perturbed_up[vertex].x
                          : (axis == 1 ? &perturbed_up[vertex].y : &perturbed_up[vertex].z);
            double* down_component =
                axis == 0 ? &perturbed_down[vertex].x
                          : (axis == 1 ? &perturbed_down[vertex].y : &perturbed_down[vertex].z);
            *up_component += epsilon;
            *down_component -= epsilon;

            const auto up = evaluate_both(perturbed_up);
            const auto down = evaluate_both(perturbed_down);

            const double numeric_deviatoric = (up.first.value - down.first.value) / (2.0 * epsilon);
            const double numeric_hydrostatic =
                (up.second.value - down.second.value) / (2.0 * epsilon);

            const double analytic_deviatoric =
                axis == 0 ? base.first.gradient[vertex].x
                          : (axis == 1 ? base.first.gradient[vertex].y
                                       : base.first.gradient[vertex].z);
            const double analytic_hydrostatic =
                axis == 0 ? base.second.gradient[vertex].x
                          : (axis == 1 ? base.second.gradient[vertex].y
                                       : base.second.gradient[vertex].z);

            EXPECT_NEAR(numeric_deviatoric, analytic_deviatoric, 1e-5)
                << "deviatoric gradient, vertex " << vertex << " axis " << axis;
            EXPECT_NEAR(numeric_hydrostatic, analytic_hydrostatic, 1e-5)
                << "hydrostatic gradient, vertex " << vertex << " axis " << axis;
        }
    }
}

// The Lame parameters must reduce to well-known special cases: at Poisson's
// ratio zero there is no lateral coupling, so lambda is zero and mu is simply
// E/2; incompressible materials (approaching 0.5) drive lambda very large
// relative to mu, which is exactly the "resists volume change far more than
// shape change" behaviour rubber and tissue need.
TEST(Unit_FEMProjection, LameParametersMatchKnownSpecialCases)
{
    SoftBodyMaterial zero_poisson;
    zero_poisson.young_modulus = 1.0e6;
    zero_poisson.poisson_ratio = 0.0;
    const LameParameters<Scalar> at_zero = lame_parameters(zero_poisson);
    EXPECT_NEAR(double(at_zero.lambda), 0.0, 1e-6);
    EXPECT_NEAR(double(at_zero.mu), 0.5e6, 1.0);

    SoftBodyMaterial near_incompressible;
    near_incompressible.young_modulus = 1.0e6;
    near_incompressible.poisson_ratio = 0.499;
    const LameParameters<Scalar> at_limit = lame_parameters(near_incompressible);
    EXPECT_GT(double(at_limit.lambda), double(at_limit.mu) * 50.0);
}

// The material presets must each be internally consistent with §9.2's stated
// ordering — steel and aluminium are stiffer than rubber by orders of
// magnitude, and only the two metals yield and fracture at a finite stress.
TEST(Unit_FEMProjection, PresetsAreOrderedTheWayTheirNamesPromise)
{
    const SoftBodyMaterial rubber = rubber_material<Scalar>();
    const SoftBodyMaterial foam = foam_material<Scalar>();
    const SoftBodyMaterial tissue = soft_tissue_material<Scalar>();
    const SoftBodyMaterial steel = sheet_steel_material<Scalar>();
    const SoftBodyMaterial aluminium = aluminium_material<Scalar>();

    EXPECT_LT(tissue.young_modulus, foam.young_modulus);
    EXPECT_LT(foam.young_modulus, rubber.young_modulus);
    EXPECT_LT(rubber.young_modulus, aluminium.young_modulus);
    EXPECT_LT(aluminium.young_modulus, steel.young_modulus);

    EXPECT_LT(steel.yield_stress, steel.fracture_stress);
    EXPECT_LT(aluminium.yield_stress, aluminium.fracture_stress);
    EXPECT_GT(rubber.fracture_stress, 0.0);
}
