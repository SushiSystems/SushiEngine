/**************************************************************************/
/* test_fem_fracture.cpp                                                 */
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

// Unit_FemFracture: §9.5's removal mechanics (physics/soft/fem_fracture.hpp)
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
    FemTetrahedron element_with_vertices(std::uint32_t a, std::uint32_t b, std::uint32_t c,
                                         std::uint32_t d, Scalar stress)
    {
        FemTetrahedron element;
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
TEST(Unit_FemFracture, MaterialThatNeverFracturesRemovesNothing)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 0.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e9));

    FemFractureBudget budget;
    std::uint32_t total = 0;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
    EXPECT_EQ(total, 0u);
}

// Stress below the threshold must not fracture.
TEST(Unit_FemFracture, StressBelowThresholdRemovesNothing)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 1000.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 500.0));

    FemFractureBudget budget;
    std::uint32_t total = 0;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
}

// The one and only element, over threshold, with a permissive minimum
// fragment size (nothing has to remain): it must be removed cleanly.
TEST(Unit_FemFracture, TheLastElementCanFractureAwayEntirely)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(4);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));

    FemFractureBudget budget;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    EXPECT_EQ(report.elements_removed, 1u);
    EXPECT_EQ(model.elements.size(), 0u);
    EXPECT_EQ(total, 1u);
}

// Three elements over threshold, a per-tick budget of one: only the first in
// ascending index order is removed this call, and the other two are counted
// as skipped rather than silently dropped.
TEST(Unit_FemFracture, PerTickBudgetLimitsHowManyFractureAtOnce)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(12);
    // Three separate, unconnected tetrahedra (disjoint vertex ranges), all
    // over threshold, so none of them are held back by the sliver guard.
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));
    model.elements.push_back(element_with_vertices(4, 5, 6, 7, 1.0e6));
    model.elements.push_back(element_with_vertices(8, 9, 10, 11, 1.0e6));

    FemFractureBudget budget;
    budget.max_fractures_per_tick = 1;
    budget.minimum_fragment_element_count = 1;
    std::uint32_t total = 0;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

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
TEST(Unit_FemFracture, MinimumFragmentSizeRefusesASeveringRemoval)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(10);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 50.0));  // below threshold
    model.elements.push_back(element_with_vertices(3, 4, 5, 6, 1.0e6)); // the middle link, over
    model.elements.push_back(element_with_vertices(6, 7, 8, 9, 50.0)); // below threshold

    FemFractureBudget budget;
    budget.minimum_fragment_element_count = 2;
    std::uint32_t total = 0;
    const FemFractureReport report = apply_fem_fracture<Scalar>(model, budget, total);

    // The only candidate (the middle element) would leave two one-element
    // pieces, each under the size-2 minimum, so it must be refused.
    EXPECT_EQ(report.elements_removed, 0u);
    EXPECT_EQ(report.elements_skipped, 1u);
    EXPECT_EQ(model.elements.size(), 3u);
}

// The scene-level cap holds across separate calls, not just within one: once
// it is reached, a later call with fresh candidates removes nothing more.
TEST(Unit_FemFracture, SceneLevelCapHoldsAcrossCalls)
{
    FiniteElementModel<Scalar> model;
    model.material.fracture_stress = 100.0;
    model.particles.resize(8);
    model.elements.push_back(element_with_vertices(0, 1, 2, 3, 1.0e6));
    model.elements.push_back(element_with_vertices(4, 5, 6, 7, 1.0e6));

    FemFractureBudget budget;
    budget.max_fractures_per_tick = 10; // not the limiting factor in this test
    budget.minimum_fragment_element_count = 1;
    budget.maximum_total_fractures = 1;
    std::uint32_t total = 0;

    const FemFractureReport first = apply_fem_fracture<Scalar>(model, budget, total);
    EXPECT_EQ(first.elements_removed, 1u);
    EXPECT_EQ(total, 1u);
    ASSERT_EQ(model.elements.size(), 1u);

    // The one surviving element is still over threshold, but the scene cap
    // was already spent by the first call.
    const FemFractureReport second = apply_fem_fracture<Scalar>(model, budget, total);
    EXPECT_EQ(second.elements_removed, 0u);
    EXPECT_EQ(model.elements.size(), 1u);
}
