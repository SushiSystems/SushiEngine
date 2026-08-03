/**************************************************************************/
/* test_graph_coloring.cpp                                               */
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

// Unit_GraphColoring: the greedy edge-colouring that makes a Gauss-Seidel sweep
// parallel. The two properties that matter for correctness are that every colour
// is conflict-free (no shared body within a colour) and that every constraint is
// coloured exactly once; the colour count is the sweep's sequential depth.

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <set>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraints/constraint.hpp>
#include <SushiEngine/physics/soft/fem_element.hpp>
#include <SushiEngine/physics/solver/constraint_store.hpp>
#include <SushiEngine/physics/solver/graph_coloring.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    // Asserts no colour reuses a body and every constraint index appears once.
    void check_partition_is_valid(const ColorBatches& colors,
                                  const std::vector<DistanceConstraint>& constraints)
    {
        std::set<std::uint32_t> seen;
        for (const std::vector<std::uint32_t>& batch : colors)
        {
            std::set<std::uint32_t> bodies;
            for (std::uint32_t index : batch)
            {
                EXPECT_TRUE(seen.insert(index).second) << "constraint " << index << " coloured twice";
                const DistanceConstraint& c = constraints[index];
                EXPECT_TRUE(bodies.insert(c.a).second) << "body " << c.a << " reused within a colour";
                EXPECT_TRUE(bodies.insert(c.b).second) << "body " << c.b << " reused within a colour";
            }
        }
        EXPECT_EQ(seen.size(), constraints.size());
    }
}

TEST(Unit_GraphColoring, ChainPartitionsIntoTwoColors)
{
    constexpr std::uint32_t N = 32;
    std::vector<DistanceConstraint> chain;
    for (std::uint32_t i = 0; i + 1 < N; ++i)
        chain.push_back(DistanceConstraint{i, i + 1, Scalar(1)});

    const ColorBatches colors = color_constraints(chain, N);

    EXPECT_EQ(colors.size(), 2u);
    check_partition_is_valid(colors, chain);
}

TEST(Unit_GraphColoring, StarNeedsOneColorPerEdge)
{
    // Every constraint touches body 0, so no two can share a colour.
    constexpr std::uint32_t N = 6;
    std::vector<DistanceConstraint> star;
    for (std::uint32_t i = 1; i < N; ++i)
        star.push_back(DistanceConstraint{0, i, Scalar(1)});

    const ColorBatches colors = color_constraints(star, N);

    EXPECT_EQ(colors.size(), star.size());
    check_partition_is_valid(colors, star);
}

TEST(Unit_GraphColoring, EmptyConstraintsProduceNoColors)
{
    const ColorBatches colors = color_constraints(std::vector<DistanceConstraint>{}, 4);
    EXPECT_TRUE(colors.empty());
}

// -- The incremental colouring, held to what determinism actually needs ------
//
// §17.5 carries a risk row: "incremental recolouring diverges from a full recolour
// and breaks determinism", with the mitigation named as a test and the fallback
// named as a scheduled full recolour. The row went unwritten for a phase and a half
// because it is a risk rather than a feature, which is exactly the kind of claim that
// stays unexamined until it costs something.
//
// Writing it forces the claim to be stated precisely, and the precise version is not
// the one the row implies. Incremental colouring *does* diverge from a full recolour,
// necessarily and by design: greedy over an insertion order is not greedy over the
// final set, and asserting equality would be asserting that the order a scene was
// built in leaves no trace. What determinism needs is weaker and testable:
//
//   1. the colouring is *valid* — no two constraints in one colour share a body,
//      which is the whole property that lets a colour be projected in parallel;
//   2. the colouring is a *function of the sequence* — the same adds and removes in
//      the same order produce the same colour for the same constraint, every time,
//      on any worker count, because the solve order derives from it (§12.1);
//   3. its quality stays inside greedy's bound, so a long-lived scene cannot drift
//      into needing more colours than a rebuild would — the failure the risk row is
//      really about, since colour count is the sweep's sequential depth.
//
// A scene is built and torn down by a deterministic pseudo-random sequence rather
// than by hand, because the interesting states are the ones nobody would think to
// write down: a removal that swaps a constraint into another slot, a body whose
// colour set empties and refills, a re-add into a gap.

namespace
{
    /** @brief One add/remove decision, and what it acted on. */
    struct ColoringStep
    {
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        bool add = true;
    };

    /**
     * @brief A fixed sequence of adds and removes over @p bodies bodies.
     *
     * Its own generator rather than <random>, whose engines are specified but whose
     * distributions are not — a sequence that differs between standard libraries
     * would make this test's failures unreproducible, which is the opposite of the
     * point.
     */
    std::vector<ColoringStep> coloring_sequence(std::size_t count, std::uint32_t bodies)
    {
        std::vector<ColoringStep> steps;
        std::uint64_t state = 0x9E3779B97F4A7C15ull;
        const auto next = [&state]() -> std::uint32_t
        {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            return std::uint32_t(state >> 32);
        };
        for (std::size_t i = 0; i < count; ++i)
        {
            ColoringStep step;
            step.a = next() % bodies;
            step.b = next() % bodies;
            if (step.a == step.b)
                step.b = (step.b + 1) % bodies;
            // Removals are a third of the traffic, so the store spends most of its
            // life growing but still exercises swap-remove and slot reuse constantly.
            step.add = (next() % 3) != 0;
            steps.push_back(step);
        }
        return steps;
    }

    /** @brief One constraint as the store knows it: its endpoints and its handle. */
    struct ColoredEdge
    {
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        ConstraintHandle handle;
        std::uint32_t color = 0;
    };

    /**
     * @brief Replays @p steps against a store and returns what survived.
     *
     * The colour each surviving constraint holds is the observable this test is
     * about, so it is returned rather than inspected in place.
     */
    std::vector<ColoredEdge> replay(const std::vector<ColoringStep>& steps,
                                    std::size_t bodies, std::size_t capacity,
                                    std::size_t colors, ConstraintStore& store)
    {
        std::vector<ColoredEdge> live;
        for (const ColoringStep& step : steps)
        {
            if (step.add)
            {
                const ConstraintPlacement placed = store.place(step.a, step.b);
                if (!placed.handle.valid())
                    continue; // out of capacity or out of colours; counted, not fatal
                ColoredEdge edge;
                edge.a = step.a;
                edge.b = step.b;
                edge.handle = placed.handle;
                edge.color = placed.color;
                live.push_back(edge);
                continue;
            }

            // Remove the oldest constraint that touches `a`, which is a rule and not
            // a coin toss: a removal that depends on iteration order would make the
            // sequence's outcome depend on the container rather than on the sequence.
            for (std::size_t i = 0; i < live.size(); ++i)
            {
                if (live[i].a != step.a && live[i].b != step.a)
                    continue;
                store.remove(live[i].handle, live[i].a, live[i].b);
                live.erase(live.begin() + std::ptrdiff_t(i));
                break;
            }
        }
        (void)bodies;
        (void)capacity;
        (void)colors;
        return live;
    }

    /** @brief The highest number of constraints any one body carries. */
    std::size_t maximum_degree(const std::vector<ColoredEdge>& live, std::size_t bodies)
    {
        std::vector<std::size_t> degree(bodies, 0);
        for (const ColoredEdge& edge : live)
        {
            ++degree[edge.a];
            ++degree[edge.b];
        }
        std::size_t highest = 0;
        for (const std::size_t d : degree)
            highest = std::max(highest, d);
        return highest;
    }
}

TEST(Unit_IncrementalColoring, ARandomizedAddRemoveSequenceStaysConflictFree)
{
    constexpr std::size_t BODIES = 64;
    constexpr std::size_t CAPACITY = 512;
    constexpr std::size_t COLORS = 16;

    ConstraintStore store(BODIES, CAPACITY, COLORS);
    const std::vector<ColoredEdge> live =
        replay(coloring_sequence(4000, BODIES), BODIES, CAPACITY, COLORS, store);
    ASSERT_FALSE(live.empty());

    // The property the whole colouring exists for, checked against the store's own
    // bands rather than against the returned colours — so a store that agreed with
    // itself but placed a constraint somewhere else would still be caught.
    std::vector<std::set<std::uint32_t>> bodies_in_color(store.color_count());
    for (const ColoredEdge& edge : live)
    {
        ASSERT_LT(edge.color, store.color_count());
        const std::size_t slot = store.slot_of(edge.handle);
        EXPECT_GE(slot, store.band_base(edge.color));
        EXPECT_LT(slot, store.band_base(edge.color) + store.band_size(edge.color));

        EXPECT_TRUE(bodies_in_color[edge.color].insert(edge.a).second)
            << "body " << edge.a << " twice in colour " << edge.color;
        EXPECT_TRUE(bodies_in_color[edge.color].insert(edge.b).second)
            << "body " << edge.b << " twice in colour " << edge.color;
    }

    // Every live constraint is in exactly one band, and the bands hold nothing else.
    std::size_t banded = 0;
    for (std::size_t color = 0; color < store.color_count(); ++color)
        banded += store.band_size(color);
    EXPECT_EQ(banded, live.size());
    EXPECT_EQ(store.live_count(), live.size());
}

TEST(Unit_IncrementalColoring, TheSameSequenceProducesTheSameColouring)
{
    constexpr std::size_t BODIES = 64;
    constexpr std::size_t CAPACITY = 512;
    constexpr std::size_t COLORS = 16;

    const std::vector<ColoringStep> steps = coloring_sequence(4000, BODIES);

    ConstraintStore first_store(BODIES, CAPACITY, COLORS);
    ConstraintStore second_store(BODIES, CAPACITY, COLORS);
    const std::vector<ColoredEdge> first =
        replay(steps, BODIES, CAPACITY, COLORS, first_store);
    const std::vector<ColoredEdge> second =
        replay(steps, BODIES, CAPACITY, COLORS, second_store);

    // Not merely the same colour counts: the same constraint in the same colour in
    // the same slot. This is the claim §12.1 rests on — the solve order is a function
    // of simulation state, and the colouring is part of that state.
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i)
    {
        EXPECT_EQ(first[i].a, second[i].a);
        EXPECT_EQ(first[i].b, second[i].b);
        EXPECT_EQ(first[i].color, second[i].color);
        EXPECT_EQ(first_store.slot_of(first[i].handle),
                  second_store.slot_of(second[i].handle));
    }
    EXPECT_EQ(first_store.colors_used(), second_store.colors_used());
}

TEST(Unit_IncrementalColoring, ItStaysInsideGreedysBoundRatherThanDrifting)
{
    constexpr std::size_t BODIES = 64;
    constexpr std::size_t COLORS = 32;
    // Ample on purpose, and the first thing this test taught. `place()` skips a colour
    // whose band is *full* as well as one that is taken (§16.8), so a tight capacity
    // makes the colour count a measure of band capacity rather than of the colouring:
    // at 512 slots over 32 colours a band holds 16, and the store climbs to the colour
    // ceiling without a single conflict having forced it there. A test that cannot tell
    // those two apart cannot support the claim it is written to support. With 64 bodies
    // and a 32-colour ceiling no colour can ever hold more than 32 constraints, so a
    // band of 64 cannot fill and what remains is the colouring alone.
    constexpr std::size_t CAPACITY = COLORS * 64;

    ConstraintStore store(BODIES, CAPACITY, COLORS);
    const std::vector<ColoredEdge> live =
        replay(coloring_sequence(4000, BODIES), BODIES, CAPACITY, COLORS, store);
    ASSERT_FALSE(live.empty());

    // Greedy edge colouring needs at most `2 * (degree - 1) + 1` colours, and the
    // incremental rule is greedy over the insertion order, so it inherits the bound.
    // A colouring that had drifted — ten thousand adds and removes leaving a scene
    // needing far more colours than its shape requires — would be the real form of
    // §17.5's risk, and the sweep would be that much deeper for it.
    const std::size_t degree = maximum_degree(live, BODIES);
    ASSERT_GT(degree, 0u);
    EXPECT_LE(store.colors_used(), 2 * (degree - 1) + 1);

    // And against the concrete alternative the risk row names as its fallback: a full
    // recolour of the surviving set. Divergence is expected — greedy over an insertion
    // order is not greedy over a final set — so what is asserted is that the
    // incremental result is not *worse* in the way that would matter, by more than the
    // one colour a different order can cost.
    std::vector<DistanceConstraint> rebuilt;
    for (const ColoredEdge& edge : live)
    {
        DistanceConstraint constraint;
        constraint.a = edge.a;
        constraint.b = edge.b;
        rebuilt.push_back(constraint);
    }
    const ColorBatches fresh = color_constraints(rebuilt, BODIES);
    check_partition_is_valid(fresh, rebuilt);
    EXPECT_LE(store.colors_used(), fresh.size() + 1)
        << "the incremental colouring has drifted past a rebuild's depth";
}

TEST(Unit_IncrementalColoring, RemovingAConstraintReleasesItsColourForReuse)
{
    // The narrow case behind the two statistical tests above, stated where a failure
    // can be read: a colour bit is "this body has a constraint of this colour", so
    // removal must clear it. If removal leaked, a body cycled through add and remove
    // would climb the colour ladder until it ran out — and the symptom would be a
    // slowly deepening sweep rather than anything that looks like a bug.
    ConstraintStore store(8, 64, 8);
    for (int cycle = 0; cycle < 32; ++cycle)
    {
        const ConstraintPlacement placed = store.place(0, 1);
        ASSERT_TRUE(placed.handle.valid());
        EXPECT_EQ(placed.color, 0u) << "colour leaked on cycle " << cycle;
        store.remove(placed.handle, 0, 1);
    }
    EXPECT_EQ(store.live_count(), 0u);
}

// -- P6-J1: the same machinery over constraints with more than two endpoints ----
//
// A tetrahedron touches four particles and both of its projections write to all
// four. Coloured as though it were an edge, the two ignored particles are
// unprotected: a colour stops meaning "no two constraints here share a body", and
// the parallel sweep the colouring licenses races on exactly the constraints that
// look independent. Every case below is built so that a two-endpoint reading would
// pass — the conflicts are all in the vertices an edge reading would drop.

namespace
{
    FemTetrahedron tet(std::uint32_t v0, std::uint32_t v1, std::uint32_t v2, std::uint32_t v3)
    {
        FemTetrahedron element;
        element.vertex[0] = v0;
        element.vertex[1] = v1;
        element.vertex[2] = v2;
        element.vertex[3] = v3;
        return element;
    }
} // namespace

TEST(Unit_GraphColoring, TetrahedraSharingOnlyALateVertexStillConflict)
{
    // The regression the whole task is about. These two share particle 7, and they
    // share it in slots 2 and 3 — the ones an `a`/`b` reading never looks at. Given
    // the same colour they would be projected in parallel and both write particle 7.
    std::vector<FemTetrahedron> elements;
    elements.push_back(tet(0, 1, 7, 2));
    elements.push_back(tet(3, 4, 5, 7));

    const ColorBatches colors = color_constraints(elements, 8);

    ASSERT_EQ(colors.size(), 2u) << "the shared particle was not seen, so both fit one colour";
    EXPECT_EQ(colors[0].size(), 1u);
    EXPECT_EQ(colors[1].size(), 1u);
}

TEST(Unit_GraphColoring, DisjointTetrahedraStillShareAColour)
{
    // The other half of the claim, and the one that stops the fix from being "give
    // everything its own colour": elements that genuinely share nothing must still
    // batch together, or the sweep's depth becomes the element count.
    std::vector<FemTetrahedron> elements;
    for (std::uint32_t i = 0; i < 5; ++i)
        elements.push_back(tet(i * 4, i * 4 + 1, i * 4 + 2, i * 4 + 3));

    const ColorBatches colors = color_constraints(elements, 20);

    ASSERT_EQ(colors.size(), 1u) << "disjoint elements were spread over several colours";
    EXPECT_EQ(colors[0].size(), 5u);
}

TEST(Unit_GraphColoring, EveryColourOfATetrahedralLatticeIsConflictFree)
{
    // A lattice, where sharing is the rule rather than the exception: check the
    // property directly over all four vertices instead of trusting a count.
    std::vector<FemTetrahedron> elements;
    const std::uint32_t body_count = 64;
    for (std::uint32_t i = 0; i + 3 < body_count; i += 2)
        elements.push_back(tet(i, i + 1, i + 2, i + 3));

    const ColorBatches colors = color_constraints(elements, body_count);

    std::set<std::uint32_t> seen;
    for (const std::vector<std::uint32_t>& batch : colors)
    {
        std::set<std::uint32_t> bodies;
        for (const std::uint32_t index : batch)
        {
            EXPECT_TRUE(seen.insert(index).second) << "element " << index << " coloured twice";
            for (const std::uint32_t vertex : elements[index].vertex)
                EXPECT_TRUE(bodies.insert(vertex).second)
                    << "particle " << vertex << " reused within a colour";
        }
    }
    EXPECT_EQ(seen.size(), elements.size()) << "an element went uncoloured";
}

TEST(Unit_IncrementalColoring, AnNBodyAssignmentIsFreeOnEveryOneOfItsBodies)
{
    IncrementalColoring coloring(16, 8);

    const std::uint32_t first[4] = {0, 1, 2, 3};
    const std::uint32_t second[4] = {4, 5, 6, 3}; // shares only particle 3
    const std::uint32_t third[4] = {8, 9, 10, 11}; // shares nothing

    EXPECT_EQ(coloring.assign_bodies(first, 4), 0u);
    EXPECT_EQ(coloring.assign_bodies(second, 4), 1u) << "the shared particle did not force a new colour";
    EXPECT_EQ(coloring.assign_bodies(third, 4), 0u) << "a disjoint element should reuse colour 0";

    // Every one of the first element's particles now holds colour 0, not just two.
    for (const std::uint32_t body : first)
        EXPECT_TRUE(coloring.holds(body, 0)) << "particle " << body << " was left unmarked";
}

TEST(Unit_IncrementalColoring, ReleasingAnNBodyConstraintFreesEveryOneOfItsBodies)
{
    IncrementalColoring coloring(16, 8);
    const std::uint32_t bodies[4] = {2, 3, 5, 7};

    ASSERT_EQ(coloring.assign_bodies(bodies, 4), 0u);
    coloring.release_bodies(bodies, 4, 0);
    for (const std::uint32_t body : bodies)
        EXPECT_FALSE(coloring.holds(body, 0)) << "particle " << body << " kept the colour";

    // And the colour is genuinely reusable, which is the observable consequence.
    EXPECT_EQ(coloring.assign_bodies(bodies, 4), 0u);
}

TEST(Unit_IncrementalColoring, ARejectedTakeLeavesNoColourHalfHeld)
{
    // A colour marked on some of a constraint's bodies and not the others is worse
    // than either outcome: nothing owns it, so nothing will ever release it, and
    // every later assignment routes around a colour that is doing no work.
    IncrementalColoring coloring(4, 8);
    const std::uint32_t bodies[3] = {0, 1, 99}; // 99 is past the capacity

    EXPECT_FALSE(coloring.take_bodies(bodies, 3, 2u));
    EXPECT_FALSE(coloring.holds(0, 2)) << "the first body was marked before the range check";
    EXPECT_FALSE(coloring.holds(1, 2));
    EXPECT_EQ(coloring.assign_bodies(bodies, 3), IncrementalColoring::NO_COLOR)
        << "an out-of-range body must refuse rather than colour what it can";
}

TEST(Unit_IncrementalColoring, AnEmptyBodyListIsRefused)
{
    // Refused rather than trivially satisfied: a colour handed to a constraint that
    // constrains nothing is a colour permanently taken on no body.
    IncrementalColoring coloring(4, 8);
    EXPECT_EQ(coloring.assign_bodies(nullptr, 0), IncrementalColoring::NO_COLOR);
    EXPECT_FALSE(coloring.take_bodies(nullptr, 0, 0u));
    EXPECT_EQ(coloring.highest_used(), 0u);
}

TEST(Unit_ConstraintStore, PlacesAndRemovesAFourBodyConstraint)
{
    ConstraintStore store(16, 64, 8);
    const std::uint32_t first[4] = {0, 1, 2, 3};
    const std::uint32_t second[4] = {3, 4, 5, 6}; // shares particle 3

    const ConstraintPlacement a = store.place_bodies(first, 4);
    ASSERT_TRUE(a.handle.valid());
    EXPECT_EQ(a.color, 0u);

    const ConstraintPlacement b = store.place_bodies(second, 4);
    ASSERT_TRUE(b.handle.valid());
    EXPECT_EQ(b.color, 1u) << "the shared particle did not force a second colour";

    store.remove_bodies(a.handle, first, 4);
    EXPECT_FALSE(store.alive(a.handle));
    // Colour 0 is free again on all four of the removed element's particles, so an
    // element overlapping it can take it.
    const ConstraintPlacement c = store.place_bodies(first, 4);
    ASSERT_TRUE(c.handle.valid());
    EXPECT_EQ(c.color, 0u) << "removal did not give the colour back on every particle";
}
