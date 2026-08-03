/**************************************************************************/
/* test_bending_constraint.cpp                                            */
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

// §9.1's bending constraint (P6-H), and P6's acceptance clause for it: with the
// stiffness at zero the cloth must behave exactly as it did before.
//
// The weights are checked first and on their own, because everything else in the
// constraint is one line of arithmetic over them. What they have to be is fixed
// by a property rather than by a formula — they annihilate the rest stencil and
// they sum to zero — so the cases assert the property, which is what would still
// be true if the derivation were rewritten.
//
// Then the two behaviours: a stencil at rest must be left alone (a constraint
// that pushed at rest would make every flat cloth in the engine hum), and a
// folded one must be pushed back toward flat and not past it.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraints/bending_constraint.hpp>
#include <SushiEngine/physics/soft/mass_spring_model.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A flat, symmetric stencil: shared edge along x, opposites either side. */
    void flat_stencil(Vector3 out[4])
    {
        out[0] = Vector3{0, 0, 0};
        out[1] = Vector3{1, 0, 0};
        out[2] = Vector3{Scalar(0.5), Scalar(1), 0};
        out[3] = Vector3{Scalar(0.5), Scalar(-1), 0};
    }

    std::vector<RigidBodyT<Scalar>> particles_at(const Vector3 position[4], Scalar inverse_mass)
    {
        std::vector<RigidBodyT<Scalar>> out(4);
        for (int i = 0; i < 4; ++i)
        {
            out[i].position = position[i];
            out[i].prev_position = position[i];
            out[i].inv_mass = inverse_mass;
            out[i].inv_inertia = Vector3{0, 0, 0};
        }
        return out;
    }

    Vector3 deviation_of(const XpbdBendingConstraint& constraint,
                         const std::vector<RigidBodyT<Scalar>>& particles)
    {
        Vector3 sum{0, 0, 0};
        for (int i = 0; i < 4; ++i)
            sum = sum + particles[constraint.particle[i]].position * constraint.weight[i];
        return sum;
    }

    /** @brief A flat rows x cols sheet in the XY plane, pinned along row zero. */
    MassSpringModel<Scalar> flat_sheet(std::size_t rows, std::size_t cols, Scalar spacing,
                                       Scalar bending_stiffness)
    {
        MassSpringModel<Scalar> model;
        model.particles.resize(rows * cols);
        for (std::size_t row = 0; row < rows; ++row)
            for (std::size_t col = 0; col < cols; ++col)
            {
                RigidBodyT<Scalar>& particle = model.particles[row * cols + col];
                particle.position = Vector3{Scalar(col) * spacing, Scalar(row) * spacing, 0};
                particle.prev_position = particle.position;
                particle.inv_inertia = Vector3{0, 0, 0};
                particle.inv_mass = row == 0 ? Scalar(0) : Scalar(1);
            }
        model.damping = Scalar(1);
        link_cloth_grid(model, rows, cols, Scalar(1e-8), bending_stiffness);
        return model;
    }
} // namespace

TEST(Unit_BendingConstraint, TheWeightsAnnihilateTheRestStencilAndSumToZero)
{
    Vector3 rest[4];
    flat_stencil(rest);

    XpbdBendingConstraint constraint;
    ASSERT_TRUE(build_bending_constraint(rest[0], rest[1], rest[2], rest[3], constraint));

    Scalar sum = 0;
    Vector3 combination{0, 0, 0};
    for (int i = 0; i < 4; ++i)
    {
        sum += constraint.weight[i];
        combination = combination + rest[i] * constraint.weight[i];
    }
    EXPECT_NEAR(double(sum), 0.0, 1e-12) << "the weights must sum to zero, or the constraint "
                                            "would resist translation";
    EXPECT_LT(double(length(combination)), 1e-12)
        << "the weights must annihilate the rest shape, or the constraint would not be at rest";
    EXPECT_NEAR(double(constraint.rest_deviation), 0.0, 1e-12);
}

TEST(Unit_BendingConstraint, IsBlindToRigidMotionAndToUniformScale)
{
    // The three things a bending measure must not see. Any of them registering
    // would make a cloth resist being carried, turned, or having its authored
    // size changed — none of which is bending.
    Vector3 rest[4];
    flat_stencil(rest);
    XpbdBendingConstraint constraint;
    ASSERT_TRUE(build_bending_constraint(rest[0], rest[1], rest[2], rest[3], constraint));
    for (int i = 0; i < 4; ++i)
        constraint.particle[i] = std::uint32_t(i);

    Vector3 moved[4];
    const Scalar angle = Scalar(0.9);
    const Scalar c = Scalar(std::cos(double(angle)));
    const Scalar s = Scalar(std::sin(double(angle)));
    for (int i = 0; i < 4; ++i)
    {
        const Vector3 scaled = rest[i] * Scalar(3);
        moved[i] = Vector3{scaled.x * c - scaled.y * s, scaled.x * s + scaled.y * c, scaled.z} +
                   Vector3{Scalar(100), Scalar(-40), Scalar(7)};
    }

    const std::vector<RigidBodyT<Scalar>> particles = particles_at(moved, Scalar(1));
    EXPECT_LT(double(length(deviation_of(constraint, particles))), 1e-12);
}

TEST(Unit_BendingConstraint, LeavesAFlatStencilExactlyWhereItIs)
{
    Vector3 rest[4];
    flat_stencil(rest);
    XpbdBendingConstraint constraint;
    ASSERT_TRUE(build_bending_constraint(rest[0], rest[1], rest[2], rest[3], constraint));
    for (int i = 0; i < 4; ++i)
        constraint.particle[i] = std::uint32_t(i);

    std::vector<RigidBodyT<Scalar>> particles = particles_at(rest, Scalar(1));
    Scalar lambda = 0;
    XpbdBendingProjection{}(constraint, particles.data(), lambda, Scalar(1.0 / 600.0));

    for (int i = 0; i < 4; ++i)
        EXPECT_LT(double(length(particles[i].position - rest[i])), 1e-12)
            << "particle " << i << " moved at rest";
}

TEST(Unit_BendingConstraint, PushesAFoldedStencilBackTowardFlatWithoutOvershooting)
{
    Vector3 rest[4];
    flat_stencil(rest);
    XpbdBendingConstraint constraint;
    ASSERT_TRUE(build_bending_constraint(rest[0], rest[1], rest[2], rest[3], constraint));
    for (int i = 0; i < 4; ++i)
        constraint.particle[i] = std::uint32_t(i);

    Vector3 folded[4];
    for (int i = 0; i < 4; ++i)
        folded[i] = rest[i];
    folded[3].z = Scalar(0.4); // lift one wing out of the plane

    std::vector<RigidBodyT<Scalar>> particles = particles_at(folded, Scalar(1));
    const Scalar before = length(deviation_of(constraint, particles));
    ASSERT_GT(double(before), 1e-6) << "the fold did not register";

    Scalar lambda = 0;
    XpbdBendingProjection{}(constraint, particles.data(), lambda, Scalar(1.0 / 600.0));
    const Scalar after = length(deviation_of(constraint, particles));

    EXPECT_LT(double(after), double(before)) << "the fold was not reduced";
    // A rigid (zero-compliance) constraint solves in one iteration and must land
    // on the answer rather than swing past it — an overshoot here is a body that
    // oscillates instead of settling.
    EXPECT_LT(double(after), 1e-12);
}

TEST(Unit_BendingConstraint, ConservesTheCentreOfMassOfTheStencil)
{
    // The weights sum to zero and the corrections are mass-weighted, so the
    // correction must add no net momentum. A bending constraint that moved the
    // sheet's centre would make a hanging flag swim sideways.
    Vector3 rest[4];
    flat_stencil(rest);
    XpbdBendingConstraint constraint;
    ASSERT_TRUE(build_bending_constraint(rest[0], rest[1], rest[2], rest[3], constraint));
    for (int i = 0; i < 4; ++i)
        constraint.particle[i] = std::uint32_t(i);

    Vector3 folded[4];
    for (int i = 0; i < 4; ++i)
        folded[i] = rest[i];
    folded[2].z = Scalar(-0.3);
    folded[3].z = Scalar(0.5);

    std::vector<RigidBodyT<Scalar>> particles = particles_at(folded, Scalar(2));
    Vector3 before{0, 0, 0};
    for (const RigidBodyT<Scalar>& particle : particles)
        before = before + particle.position * (Scalar(1) / particle.inv_mass);

    Scalar lambda = 0;
    XpbdBendingProjection{}(constraint, particles.data(), lambda, Scalar(1.0 / 600.0));

    Vector3 after{0, 0, 0};
    for (const RigidBodyT<Scalar>& particle : particles)
        after = after + particle.position * (Scalar(1) / particle.inv_mass);
    EXPECT_LT(double(length(after - before)), 1e-12);
}

TEST(Unit_BendingConstraint, RefusesADegenerateStencil)
{
    const Vector3 collinear[4] = {Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{2, 0, 0},
                                  Vector3{3, 0, 0}};
    XpbdBendingConstraint constraint;
    EXPECT_FALSE(build_bending_constraint(collinear[0], collinear[1], collinear[2], collinear[3],
                                          constraint));
}

TEST(Unit_ClothBending, ZeroStiffnessAddsNoConstraintsAtAll)
{
    // P6's acceptance clause, taken literally. Not "the corrections cancel" —
    // there are no bending constraints, so the substep sweep is the identical
    // sequence of identical projections it was before bending existed.
    const MassSpringModel<Scalar> without = flat_sheet(5, 5, Scalar(0.1), Scalar(0));
    const MassSpringModel<Scalar> with = flat_sheet(5, 5, Scalar(0.1), Scalar(100));

    EXPECT_TRUE(without.bending.empty());
    EXPECT_FALSE(with.bending.empty());
    EXPECT_EQ(without.springs.size(), with.springs.size())
        << "bending must not change the distance topology";
}

TEST(Unit_ClothBending, CountsEveryInteriorEdgeOfTheTriangulationExactlyOnce)
{
    // A rows x cols grid triangulated two triangles per quad has
    // (rows-1)*(cols-1) diagonals, (rows-2)*(cols-1) horizontal interior edges
    // and (rows-1)*(cols-2) vertical ones. Counting a hinge twice would make it
    // twice as stiff as authored, silently.
    const std::size_t rows = 5;
    const std::size_t cols = 4;
    const MassSpringModel<Scalar> model = flat_sheet(rows, cols, Scalar(0.1), Scalar(100));

    const std::size_t expected = (rows - 1) * (cols - 1) + (rows - 2) * (cols - 1) +
                                 (rows - 1) * (cols - 2);
    EXPECT_EQ(model.bending.size(), expected);
}

TEST(Integration_ClothBending, ASheetWithBendingHangsStraighterThanOneWithout)
{
    // The whole point, measured: pinned along its top row and hung under gravity,
    // a sheet that resists folding curls less at its free corner than one that
    // does not. Compared against each other rather than against a number, since
    // what bending *is* is the difference between these two.
    MassSpringModel<Scalar> limp = flat_sheet(6, 6, Scalar(0.1), Scalar(0));
    MassSpringModel<Scalar> stiff = flat_sheet(6, 6, Scalar(0.1), Scalar(2000));
    limp.set_external_acceleration(Vector3{0, 0, Scalar(-9.81)});
    stiff.set_external_acceleration(Vector3{0, 0, Scalar(-9.81)});

    for (int tick = 0; tick < 90; ++tick)
    {
        limp.step(Scalar(1.0 / 60.0), 20);
        stiff.step(Scalar(1.0 / 60.0), 20);
    }

    // The far corner of the free edge: the point furthest from the pinned row,
    // and so the one a lack of bending lets fall furthest out of plane.
    const std::size_t corner = 5 * 6 + 5;
    EXPECT_GT(double(stiff.particles[corner].position.z),
              double(limp.particles[corner].position.z))
        << "the stiffer sheet should have sagged less";
}
