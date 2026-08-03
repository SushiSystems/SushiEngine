/**************************************************************************/
/* test_fem_fracture.cpp                                                  */
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

// Unit_FEMFracture: §9.5's removal mechanics (physics/soft/fem_fracture.hpp)
// and the three guard rails it names explicitly — a per-tick budget, a
// minimum fragment size, and a scene-level cap — checked against
// hand-constructed element sets whose connectivity is known by inspection,
// not against real tetrahedron geometry (fracture's removal-and-connectivity
// logic never reads a position, only the four vertex indices).

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/fem_fracture.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    FEMTetrahedron element_with_vertices(std::uint32_t a, std::uint32_t b, std::uint32_t c,
                                         std::uint32_t d, Scalar stress)
    {
        FEMTetrahedron element;
        element.vertex[0] = a;
        element.vertex[1] = b;
        element.vertex[2] = c;
        element.vertex[3] = d;
        element.von_mises_stress = stress;
        return element;
    }
} // namespace

// A material with no fracture_stress set (<= 0, the field's own "never
// fractures" convention elsewhere in this phase) must remove nothing, however
// high the stress reads.
TEST(Unit_FEMFracture, MaterialThatNeverFracturesRemovesNothing)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 0.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e9));

    FEMFractureBudget budget;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
    EXPECT_EQ(total, 0u);
}

// Stress below the threshold must not fracture.
TEST(Unit_FEMFracture, StressBelowThresholdRemovesNothing)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 1000.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 500.0));

    FEMFractureBudget budget;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
}

// The one and only element, over threshold, with a permissive minimum
// fragment size (nothing has to remain): it must be removed cleanly.
TEST(Unit_FEMFracture, TheLastElementCanFractureAwayEntirely)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));

    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 1u);
    EXPECT_EQ(model.elements.size(), 0u);
    EXPECT_EQ(total, 1u);
}

// Three elements over threshold, a per-tick budget of one: only the first in
// ascending index order is removed this call, and the other two are counted
// as skipped rather than silently dropped.
TEST(Unit_FEMFracture, PerTickBudgetLimitsHowManyFractureAtOnce)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(12);
    // Three separate, unconnected tetrahedra (disjoint vertex ranges), all
    // over threshold, so none of them are held back by the sliver guard.
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));
    model.elements.push_back(element_with_vertices(4, 5, 6, 7, 1.0e6));
    model.elements.push_back(element_with_vertices(8, 9, 10, 11, 1.0e6));

    FEMFractureBudget budget;
    budget.max_fractures_per_tick = 1;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 1u);
    EXPECT_EQ(report.elements_skipped, 2u);
    ASSERT_EQ(model.elements.size(), 2u);
    // The remaining two must be the second and third — the first (lowest
    // index) was the one the budget actually spent itself on.
    EXPECT_EQ(model.elements[0].vertex[0], 4u);
    EXPECT_EQ(model.elements[1].vertex[0], 8u);
}

// A chain of three tetrahedra sharing vertices pairwise (0-1-2-3, 3-4-5-6,
// 6-7-8-9): removing the *middle* one severs the chain into two separate
// two-particle-disjoint pieces of one element each. With a minimum fragment
// size of two elements, that removal must be refused — it would leave two
// slivers of one element apiece — while the two end elements (whose removal
// leaves the other two still connected to each other) are allowed.
TEST(Unit_FEMFracture, MinimumFragmentSizeRefusesASeveringRemoval)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(10);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 50.0));  // below threshold
    model.elements.push_back(element_with_vertices(3, 4, 5, 6, 1.0e6)); // the middle link, over
    model.elements.push_back(element_with_vertices(6, 7, 8, 9, 50.0)); // below threshold

    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 2;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    // The only candidate (the middle element) would leave two one-element
    // pieces, each under the size-2 minimum, so it must be refused.
    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(report.elements_skipped, 1u);
    EXPECT_EQ(model.elements.size(), 3u);
}

// The scene-level cap holds across separate calls, not just within one: once
// it is reached, a later call with fresh candidates removes nothing more.
TEST(Unit_FEMFracture, SceneLevelCapHoldsAcrossCalls)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(8);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));
    model.elements.push_back(element_with_vertices(4, 5, 6, 7, 1.0e6));

    FEMFractureBudget budget;
    budget.max_fractures_per_tick = 10; // not the limiting factor in this test
    budget.minimum_fragment_element_count = 1;
    budget.maximum_total_fractures = 1;
    std::uint32_t total = 0;

    const FEMFractureReport first = apply_fem_fracture<Scalar>(model, budget, total);
    EXPECT_EQ(first.elements_removed, 1u);
    EXPECT_EQ(total, 1u);
    ASSERT_EQ(model.elements.size(), 1u);

    // The one surviving element is still over threshold, but the scene cap
    // was already spent by the first call.
    const FEMFractureReport second = apply_fem_fracture<Scalar>(model, budget, total);
    EXPECT_EQ(second.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
}

// P6-G4: the clause §9.5 names — "its shared vertices are duplicated along the
// crack surface, splitting the topology" — plus the two things that clause implies
// and nothing else does: the boundary a broken body presents afterward, and §8.6
// invariant 4, that a render vertex follows the side of the crack its element
// ended up on.

// Three tetrahedra meeting at particle 0. A and B touch each other *only*
// there — one shared vertex, no shared face — and are held together through C.
// Removing C therefore leaves particle 0 as the single point joining two pieces
// that have nothing else in common, which is precisely the vertex that has to
// become two.
namespace
{
    FiniteElementModel<Scalar> hinge_at_particle_zero()
    {
        FiniteElementModel<Scalar> model;
        model.material.fracture_stress = 100.0;
        model.particles.resize(8);
        for (RigidBodyT<Scalar>& particle : model.particles)
            particle.inv_mass = Scalar(4); // a quarter of a kilogramme each
        model.elements.push_back(element_with_vertices(0, 1, 2, 3, 50.0));  // A, survives
        model.elements.push_back(element_with_vertices(0, 4, 5, 6, 50.0));  // B, survives
        model.elements.push_back(element_with_vertices(0, 1, 4, 7, 1.0e6)); // C, fractures
        return model;
    }
} // namespace

TEST(Unit_FEMFracture, DuplicatesTheVertexTwoPiecesWereHangingFrom)
{
    FiniteElementModel<Scalar> model = hinge_at_particle_zero();
    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    FEMFractureRemap remap;

    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total, &remap);

    ASSERT_EQ(report.elements_removed, 1u);
    EXPECT_EQ(report.vertices_duplicated, 1u);
    ASSERT_EQ(model.particles.size(), 9u);
    ASSERT_EQ(remap.duplicated_from.size(), 1u);
    EXPECT_EQ(remap.duplicated_from[0], 0u);
    EXPECT_EQ(remap.original_particle_count, 8u);

    // One of the two survivors keeps particle 0 and the other takes the copy;
    // which is which is an implementation detail, that they differ is not.
    ASSERT_EQ(model.elements.size(), 2u);
    EXPECT_NE(model.elements[0].vertex[0], model.elements[1].vertex[0]);
    EXPECT_TRUE(model.elements[0].vertex[0] == 0u || model.elements[1].vertex[0] == 0u);
    EXPECT_TRUE(model.elements[0].vertex[0] == 8u || model.elements[1].vertex[0] == 8u);
}

TEST(Unit_FEMFracture, TheSplitDividesTheVertexMassRatherThanCopyingIt)
{
    // A copy would make the body heavier every time it broke, which is the sort
    // of error that only shows up as a car that falls faster after a crash.
    FiniteElementModel<Scalar> model = hinge_at_particle_zero();
    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;

    ASSERT_EQ(apply_fem_fracture<Scalar>(model, budget, total).vertices_duplicated, 1u);
    ASSERT_EQ(model.particles.size(), 9u);

    // Two components of one element each: half the mass apiece, so twice the
    // inverse mass apiece.
    EXPECT_NEAR(double(model.particles[0].inv_mass), 8.0, 1e-12);
    EXPECT_NEAR(double(model.particles[8].inv_mass), 8.0, 1e-12);
}

TEST(Unit_FEMFracture, SplitsNothingWhenTheRemovalOnlyChippedACorner)
{
    // Three disjoint tetrahedra: removing one leaves no vertex holding two
    // pieces together, so the pass must add no particles at all. A splitter that
    // fired on every fracture would grow the particle array without bound.
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(12);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));
    model.elements.push_back(element_with_vertices(4, 5, 6, 7, 50.0));
    model.elements.push_back(element_with_vertices(8, 9, 10, 11, 50.0));

    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    const FEMFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 1u);
    EXPECT_EQ(report.vertices_duplicated, 0u);
    EXPECT_EQ(model.particles.size(), 12u);
}

TEST(Unit_FEMFracture, APinnedVertexStaysPinnedInEveryCopy)
{
    // Mass is divided; being held by the world is not. A copy that came back
    // with a finite mass would fall off the wall the original was nailed to.
    FiniteElementModel<Scalar> model = hinge_at_particle_zero();
    model.particles[0].inv_mass = Scalar(0);
    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;

    ASSERT_EQ(apply_fem_fracture<Scalar>(model, budget, total).vertices_duplicated, 1u);
    ASSERT_EQ(model.particles.size(), 9u);
    EXPECT_EQ(double(model.particles[0].inv_mass), 0.0);
    EXPECT_EQ(double(model.particles[8].inv_mass), 0.0);
}

TEST(Unit_SoftBodySurface, KeepsOnlyTheFacesNamedByOneElement)
{
    // Two tetrahedra sharing a face: eight faces between them, one of which is
    // interior, so six triangles remain. This is the whole rule, and it is the
    // rule a fractured body needs re-applied — the surface it was cooked with
    // describes the shape it had before it broke.
    FiniteElementModel<Scalar> model;
    model.particles.resize(5);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 0.0));
    model.elements.push_back(element_with_vertices(1, 2, 3, 4, 0.0));

    rebuild_soft_body_surface(model);

    EXPECT_EQ(model.surface_indices.size(), 18u);
    ASSERT_EQ(model.surface_vertices.size(), 5u);
    for (std::uint32_t i = 0; i < 5; ++i)
        EXPECT_EQ(model.surface_vertices[i], i) << "surface vertices must be ascending and unique";
}

TEST(Unit_SoftBodySurface, ASingleElementIsAllBoundary)
{
    FiniteElementModel<Scalar> model;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 0.0));

    rebuild_soft_body_surface(model);
    EXPECT_EQ(model.surface_indices.size(), 12u);
}

TEST(Unit_FEMFracture, RebuildsTheBoundaryAroundTheHoleItLeft)
{
    // Two disjoint tetrahedra after the fracture, no face shared: every one of
    // their eight faces is boundary.
    FiniteElementModel<Scalar> model = hinge_at_particle_zero();
    FEMFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;

    ASSERT_EQ(apply_fem_fracture<Scalar>(model, budget, total).elements_removed, 1u);
    EXPECT_EQ(model.surface_indices.size(), 24u);
}
