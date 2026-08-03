/**************************************************************************/
/* test_runtime_graph_builder.cpp                                         */
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

// The mutable world and the one-composition tick, together, because they are the
// same claim from two directions: the world may change every tick, and the graph
// must not. The compile-count assertions are the load-bearing ones — every
// performance number this system is held to assumes the graph is built once, and a
// count that climbs is the failure mode that would otherwise be invisible.

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
#include <SushiEngine/physics/solver/runtime_graph_builder.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A small scene budget, so a test allocates megabytes and not gigabytes. */
    PhysicsConfiguration small_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 64;
        configuration.capacities.constraints = 256;
        configuration.capacities.colors = 4;
        configuration.substeps.minimum = 2;
        configuration.substeps.maximum = 4;
        return configuration;
    }

    /** @brief A unit-mass body at @p height with no rotational freedom. */
    RigidBody falling_body(Scalar height)
    {
        RigidBody body;
        body.position = Vector3{0, height, 0};
        body.inv_mass = Scalar(1);
        return body;
    }
}

TEST(Integration_RuntimeGraphBuilder, ABodyFallsUnderGravity)
{
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    const BodyHandle handle = solver.add_body(falling_body(Scalar(10)));
    ASSERT_TRUE(handle.valid());

    StepParameters<Scalar> parameters;
    for (int tick = 0; tick < 10; ++tick)
        solver.step(parameters);

    RigidBody body;
    ASSERT_TRUE(solver.read_body(handle, body));
    EXPECT_LT(double(body.position.y), 10.0) << "gravity must have moved it";
    EXPECT_LT(double(body.velocity.y), 0.0) << "and left it moving downward";
}

TEST(Integration_RuntimeGraphBuilder, TheGraphIsCompiledOnceHoweverTheWorldChanges)
{
    // The whole point of the late-binding design. Bodies and constraints come and go
    // every tick; the compiled structure must not notice.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    StepParameters<Scalar> parameters;
    BodyHandle anchor = solver.add_body(falling_body(Scalar(5)));
    solver.step(parameters);
    const std::size_t after_warmup = solver.statistics().compile_count;
    EXPECT_EQ(after_warmup, 1u);

    for (int tick = 0; tick < 8; ++tick)
    {
        const BodyHandle transient = solver.add_body(falling_body(Scalar(3)));
        ASSERT_TRUE(transient.valid());

        XpbdDistanceConstraint link;
        link.a = std::uint32_t(solver.body_slot(anchor));
        link.b = std::uint32_t(solver.body_slot(transient));
        link.rest_length = Scalar(1);
        const ConstraintHandle constraint = solver.add_constraint(link);
        ASSERT_TRUE(constraint.valid());

        solver.step(parameters);

        EXPECT_TRUE(solver.remove_constraint(constraint));
        EXPECT_TRUE(solver.remove_body(transient));
        solver.step(parameters);
    }

    EXPECT_EQ(solver.statistics().compile_count, after_warmup)
        << "adding and removing bodies must not recompile the solve graph";
    EXPECT_EQ(solver.statistics().compose_count, 0u);
}

TEST(Integration_RuntimeGraphBuilder, AStaleHandleIsRefusedAfterItsSlotIsReused)
{
    // The reason handles carry a generation at all. Without one, the second body
    // silently answers for the first.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    const BodyHandle first = solver.add_body(falling_body(Scalar(1)));
    ASSERT_TRUE(solver.remove_body(first));
    const BodyHandle second = solver.add_body(falling_body(Scalar(2)));

    EXPECT_EQ(first.index, second.index) << "the slot should have been reused";
    EXPECT_NE(first, second);

    RigidBody body;
    EXPECT_FALSE(solver.read_body(first, body)) << "the stale handle must be refused";
    EXPECT_TRUE(solver.read_body(second, body));
    EXPECT_FALSE(solver.remove_body(first));
}

TEST(Integration_RuntimeGraphBuilder, RemovingABodyTakesItsConstraintsWithIt)
{
    // A constraint left naming a freed slot would act on whatever body claimed the
    // slot next — a corruption with no symptom until the simulation looks wrong.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    const BodyHandle a = solver.add_body(falling_body(Scalar(1)));
    const BodyHandle b = solver.add_body(falling_body(Scalar(2)));

    XpbdDistanceConstraint link;
    link.a = std::uint32_t(solver.body_slot(a));
    link.b = std::uint32_t(solver.body_slot(b));
    link.rest_length = Scalar(1);
    const ConstraintHandle constraint = solver.add_constraint(link);
    ASSERT_TRUE(constraint.valid());

    StepParameters<Scalar> parameters;
    solver.step(parameters);
    EXPECT_EQ(solver.statistics().constraints, 1u);

    EXPECT_TRUE(solver.remove_body(b));
    solver.step(parameters);
    EXPECT_EQ(solver.statistics().constraints, 0u);
    EXPECT_FALSE(solver.remove_constraint(constraint));
}

TEST(Integration_RuntimeGraphBuilder, ConstraintsOnASharedBodyTakeDifferentColours)
{
    // The incremental colouring rule, observed through the solver: two constraints
    // meeting at one body can never be projected in parallel, so they must land in
    // different colours without a full recolour having run.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    const BodyHandle hub = solver.add_body(falling_body(Scalar(1)));
    const BodyHandle spoke_a = solver.add_body(falling_body(Scalar(2)));
    const BodyHandle spoke_b = solver.add_body(falling_body(Scalar(3)));

    XpbdDistanceConstraint first;
    first.a = std::uint32_t(solver.body_slot(hub));
    first.b = std::uint32_t(solver.body_slot(spoke_a));
    first.rest_length = Scalar(1);
    ASSERT_TRUE(solver.add_constraint(first).valid());

    XpbdDistanceConstraint second;
    second.a = std::uint32_t(solver.body_slot(hub));
    second.b = std::uint32_t(solver.body_slot(spoke_b));
    second.rest_length = Scalar(1);
    ASSERT_TRUE(solver.add_constraint(second).valid());

    EXPECT_EQ(solver.color_size(0), 1u);
    EXPECT_EQ(solver.color_size(1), 1u);
}

TEST(Integration_RuntimeGraphBuilder, ExceedingTheBodyBudgetIsReportedNotThrown)
{
    // A capacity is a budget, and a budget being exceeded is news, not a crash. The
    // count is what tells an author which number in PhysicsCapacities to raise.
    PhysicsConfiguration configuration = small_scene();
    configuration.capacities.bodies = 2;

    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, configuration);

    EXPECT_TRUE(solver.add_body(falling_body(Scalar(1))).valid());
    EXPECT_TRUE(solver.add_body(falling_body(Scalar(2))).valid());
    EXPECT_FALSE(solver.add_body(falling_body(Scalar(3))).valid());

    StepParameters<Scalar> parameters;
    solver.step(parameters);
    EXPECT_EQ(solver.statistics().capacity_overflows, 1u);
}

TEST(Integration_RuntimeGraphBuilder, ADistanceConstraintPullsTwoBodiesTogether)
{
    // The projection itself still has to work through the new plumbing: the colour
    // bands, the late-bound counts, and the device-resident columns are all between
    // the caller and the same XPBD projection the host mirror tests already trust.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());

    // Started only mildly violated, on purpose. A hard constraint given a large
    // initial violation is not a stiffness test — XPBD converts the position
    // correction into velocity by construction, so a rod initialised three metres
    // from its rest length injects that error as momentum and the pair oscillates
    // for ever. That is the integrator behaving correctly, and asserting against it
    // would be asserting that XPBD is not XPBD.
    RigidBody left = falling_body(Scalar(0));
    left.position = Vector3{Scalar(-0.75), 0, 0};
    RigidBody right = falling_body(Scalar(0));
    right.position = Vector3{Scalar(0.75), 0, 0};

    const BodyHandle a = solver.add_body(left);
    const BodyHandle b = solver.add_body(right);

    XpbdDistanceConstraint link;
    link.a = std::uint32_t(solver.body_slot(a));
    link.b = std::uint32_t(solver.body_slot(b));
    link.rest_length = Scalar(1);
    ASSERT_TRUE(solver.add_constraint(link).valid());

    StepParameters<Scalar> parameters;
    parameters.gravity = Vector3{0, 0, 0};
    solver.step(parameters);

    RigidBody solved_a;
    RigidBody solved_b;
    ASSERT_TRUE(solver.read_body(a, solved_a));
    ASSERT_TRUE(solver.read_body(b, solved_b));

    // Compliance zero and one iteration per substep make the projection exact, so a
    // single tick lands on the rest length rather than approaching it.
    const double separation = double(solved_b.position.x) - double(solved_a.position.x);
    EXPECT_NEAR(separation, 1.0, 1e-9)
        << "the constraint should have pulled them to rest length";

    // And it must stay bounded rather than pumping energy: the pair keeps whatever
    // the first correction gave it, and no more.
    for (int tick = 0; tick < 60; ++tick)
        solver.step(parameters);
    ASSERT_TRUE(solver.read_body(a, solved_a));
    ASSERT_TRUE(solver.read_body(b, solved_b));
    const double late = std::fabs(double(solved_b.position.x) - double(solved_a.position.x));
    EXPECT_LT(late, 1.6) << "a hard constraint must not pump energy into the pair";
}

TEST(Integration_RuntimeGraphBuilder, TheRebalancerIsOffForAPhysicsScene)
{
    // The thermal rebalancer migrates tasks mid-run on a millisecond heartbeat. For
    // a fixed-rate tick the cost is jitter, and a scene that quietly left it running
    // would show up as frame-time noise with no other symptom.
    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    // The runtime's default is off since the 2026-08-01 rebalancer change, so turn
    // it on explicitly: the property under test is that constructing a physics
    // scene switches it off, not whatever the runtime's default happens to be.
    runtime.rebalancer(true);
    ASSERT_TRUE(runtime.advanced().rebalancer_enabled());

    RuntimeGraphBuilder<Scalar> solver(execution, small_scene());
    EXPECT_FALSE(runtime.advanced().rebalancer_enabled())
        << "constructing a physics scene must turn it off";
}

TEST(Integration_RuntimeGraphBuilder, TheSubstepCountFollowsTheFastestBody)
{
    // Derived from simulation state alone, and bounded by the schedule. A still scene
    // must sit at the minimum, and a fast one must climb — otherwise the dial is
    // decorative.
    PhysicsConfiguration configuration = small_scene();
    configuration.substeps.minimum = 1;
    configuration.substeps.maximum = 8;
    configuration.substeps.motion_budget = Scalar(0.05);

    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, configuration);

    RigidBody still = falling_body(Scalar(0));
    const BodyHandle resting = solver.add_body(still);

    StepParameters<Scalar> parameters;
    parameters.gravity = Vector3{0, 0, 0};
    solver.step(parameters);
    solver.step(parameters);
    EXPECT_EQ(solver.statistics().substeps, 1u) << "a still scene must not buy substeps";

    RigidBody quick = still;
    quick.velocity = Vector3{Scalar(100), 0, 0};
    ASSERT_TRUE(solver.write_body(resting, quick));

    solver.step(parameters); // measures the new speed
    solver.step(parameters); // and spends it
    EXPECT_GT(solver.statistics().substeps, 1u);
    EXPECT_LE(solver.statistics().substeps, 8u);
}

/**
 * @brief The soft-body budget scene's *shape*, which the suite can own.
 *
 * `samples/physics/soft_body_budget.cpp` measures section 13.1's number -- one body,
 * 20 000 tetrahedra, 32 substeps, 3 ms/tick -- and reports it rather than asserting
 * it, because a target stated against one desktop-class GPU is not a claim any
 * machine running this suite can be held to. What *is* machine-independent, and what
 * the number is worthless without, is here: that a dense tetrahedral lattice colours
 * cleanly inside the colour ceiling, that every element finds a band, and that a
 * scene of this shape composes once and never again.
 *
 * A smaller lattice than the probe's, at the same density and the same connectivity.
 * Vertex valence -- which is what the colouring is bounded by -- is a property of the
 * decomposition and not of how many cells it is run over, so the colour count this
 * pins is the colour count the probe meets.
 */
TEST(Integration_RuntimeGraphBuilder, ATetrahedralLatticeColoursCleanlyAndComposesOnce)
{
    constexpr int CELLS = 5;
    constexpr int GRID_POINTS = CELLS + 1;
    constexpr int PARTICLES = GRID_POINTS * GRID_POINTS * GRID_POINTS;
    constexpr int COLORS = 48;
    constexpr int CELL_TETRAHEDRA[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 4, 5, 7},
                                           {0, 4, 6, 7}, {0, 2, 6, 7}, {0, 2, 3, 7}};

    PhysicsConfiguration configuration;
    configuration.capacities.bodies = PARTICLES;
    configuration.capacities.constraints = 1;
    configuration.capacities.joints = 0;
    configuration.capacities.contacts = 0;
    // Not the element count. The store hands each colour a fixed band of
    // `elements / colors` slots (`constraint_store.hpp`), so what has to fit is the
    // *busiest* band, and a budget of exactly 750 would give every colour fifteen
    // slots and reject the lattice a fifth of the way in. A colour class is a set of
    // vertex-disjoint tetrahedra, so no band can hold more than a quarter of the
    // particles whatever order they arrive in; that bound is the only sizing that
    // cannot under-shoot. Greedy actually peaks at 48 of the 54 it is given here.
    configuration.capacities.colors = COLORS;
    configuration.capacities.elements = std::size_t(COLORS) * (PARTICLES / 4);
    configuration.substeps.minimum = 2;
    configuration.substeps.maximum = 2;

    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, configuration);

    std::vector<Vector3> rest;
    std::vector<BodyHandle> handles;
    for (int z = 0; z < GRID_POINTS; ++z)
        for (int y = 0; y < GRID_POINTS; ++y)
            for (int x = 0; x < GRID_POINTS; ++x)
            {
                const Vector3 position{Scalar(x) * Scalar(0.05),
                                       Scalar(1) + Scalar(y) * Scalar(0.05),
                                       Scalar(z) * Scalar(0.05)};
                rest.push_back(position);
                RigidBody body;
                body.position = position;
                body.prev_position = position;
                body.inv_mass = Scalar(1);
                handles.push_back(solver.add_body(body));
                ASSERT_TRUE(handles.back().valid());
            }

    const LameParameters<Scalar> lame = lame_parameters(SoftBodyMaterial{});

    std::size_t placed = 0;
    for (int z = 0; z < CELLS; ++z)
        for (int y = 0; y < CELLS; ++y)
            for (int x = 0; x < CELLS; ++x)
                for (const auto& corners : CELL_TETRAHEDRA)
                {
                    FemTetrahedron element;
                    Vector3 corner[4];
                    for (int i = 0; i < 4; ++i)
                    {
                        const int bits = corners[i];
                        const int px = x + (bits & 1);
                        const int py = y + ((bits >> 1) & 1);
                        const int pz = z + ((bits >> 2) & 1);
                        element.vertex[i] =
                            std::uint32_t((pz * GRID_POINTS + py) * GRID_POINTS + px);
                        corner[i] = rest[element.vertex[i]];
                    }

                    FemMatrix3<Scalar> rest_shape;
                    rest_shape.column0 = corner[1] - corner[0];
                    rest_shape.column1 = corner[2] - corner[0];
                    rest_shape.column2 = corner[3] - corner[0];
                    FemMatrix3<Scalar> inverse;
                    ASSERT_TRUE(invert_fem_matrix3(rest_shape, inverse))
                        << "a degenerate element means the lattice itself is wrong";
                    element.rest_inverse_column_0 = inverse.column0;
                    element.rest_inverse_column_1 = inverse.column1;
                    element.rest_inverse_column_2 = inverse.column2;
                    element.plastic_inverse_column_0 = inverse.column0;
                    element.plastic_inverse_column_1 = inverse.column1;
                    element.plastic_inverse_column_2 = inverse.column2;
                    element.rest_volume =
                        std::abs(determinant(rest_shape)) / Scalar(6);
                    element.mu = lame.mu;
                    element.lambda = lame.lambda;

                    ASSERT_TRUE(solver.add_element(element).valid())
                        << "element " << placed
                        << " found no conflict-free colour with room left";
                    ++placed;
                }

    EXPECT_EQ(placed, std::size_t(CELLS * CELLS * CELLS * 6));

    StepParameters<Scalar> parameters;
    solver.step(parameters);
    EXPECT_EQ(solver.statistics().elements, placed);
    const std::size_t after_warmup = solver.statistics().compile_count;

    for (int tick = 0; tick < 4; ++tick)
        solver.step(parameters);

    EXPECT_EQ(solver.statistics().compile_count, after_warmup)
        << "a lattice of elements must not recompile the graph as it runs";
    EXPECT_EQ(solver.statistics().compose_count, 0u);
    EXPECT_LE(solver.statistics().colors, configuration.capacities.colors);
}
