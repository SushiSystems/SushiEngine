/**************************************************************************/
/* test_solver_conformance.cpp                                            */
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

// The shared conformance suite for IConstraintSolver: the same scenes, run against
// every implementation, asserting they agree. This is what lets a device solver
// replace a host one without silently changing behaviour -- the Liskov rule of
// section 4.4, made executable.
//
// The two implementations differ in exactly one respect: HostXpbdSolver projects a
// colour's constraints one after another, RuntimeGraphBuilder projects them in
// parallel on the runtime. Everything else -- which colour each constraint takes,
// where in that colour's band it sits, and the arithmetic itself -- is shared code.
// So what these tests actually prove is the claim graph colouring makes: that
// constraints sharing no body can be projected simultaneously without changing the
// answer.
//
// Written as a typed test so a third implementation is a single line rather than a
// copied file, which is the only way a conformance suite survives contact with a
// second device backend.

#include <cmath>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/solver/runtime_graph_builder.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief The budget every conformance scene runs under. */
    PhysicsConfiguration conformance_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 64;
        configuration.capacities.constraints = 256;
        configuration.capacities.colors = 8;
        configuration.substeps.minimum = 4;
        configuration.substeps.maximum = 16;
        return configuration;
    }

    /**
     * @brief Holds a runtime alive for as long as the solver that borrows it.
     *
     * RuntimeGraphBuilder takes the runtime by reference and its buffers must not
     * outlive it, so the two have to be owned together for a test to hold either.
     */
    struct RuntimeBackedSolver
    {
        SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
        std::unique_ptr<RuntimeGraphBuilder<Scalar>> solver;

        explicit RuntimeBackedSolver(const PhysicsConfiguration& configuration)
            : solver(new RuntimeGraphBuilder<Scalar>(runtime, configuration))
        {
        }

        IConstraintSolver<Scalar>& operator*() { return *solver; }
    };

    struct HostBackedSolver
    {
        std::unique_ptr<HostXpbdSolver<Scalar>> solver;

        explicit HostBackedSolver(const PhysicsConfiguration& configuration)
            : solver(new HostXpbdSolver<Scalar>(configuration))
        {
        }

        IConstraintSolver<Scalar>& operator*() { return *solver; }
    };

    /** @brief A unit-mass body at @p position with no rotational freedom. */
    RigidBody point_body(const Vector3& position)
    {
        RigidBody body;
        body.position = position;
        body.inv_mass = Scalar(1);
        return body;
    }

    /** @brief A body that can also rotate, so the angular half of the projection runs. */
    RigidBody spinning_body(const Vector3& position)
    {
        RigidBody body = point_body(position);
        body.inv_inertia = Vector3{Scalar(1), Scalar(1), Scalar(1)};
        return body;
    }

    /** @brief A rigid link of @p rest between two body slots, anchored off-centre. */
    XpbdDistanceConstraint link(std::size_t a, std::size_t b, Scalar rest,
                                const Vector3& anchor_a = Vector3{},
                                const Vector3& anchor_b = Vector3{})
    {
        XpbdDistanceConstraint constraint;
        constraint.a = std::uint32_t(a);
        constraint.b = std::uint32_t(b);
        constraint.rest_length = rest;
        constraint.local_anchor_a = anchor_a;
        constraint.local_anchor_b = anchor_b;
        return constraint;
    }

    /**
     * @brief How far apart two solvers left the same body.
     *
     * A tolerance rather than exact equality, and the distinction is worth being
     * precise about. The *ordering* is identical by construction, so the two are
     * evaluating the same expressions in the same sequence and could in principle
     * agree bit for bit. What they do not share is the code generator: one path is
     * compiled for the device target and one for the host, and a difference in how a
     * fused multiply-add or a reciprocal is emitted is a legitimate difference that
     * says nothing about the solver. The tolerance is tight enough that a genuine
     * ordering divergence -- which changes a Gauss-Seidel result in the first few
     * digits -- cannot hide under it.
     */
    double disagreement(const RigidBody& a, const RigidBody& b)
    {
        const double dx = double(a.position.x) - double(b.position.x);
        const double dy = double(a.position.y) - double(b.position.y);
        const double dz = double(a.position.z) - double(b.position.z);
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    constexpr double TOLERANCE = 1e-9;
}

/**
 * @brief Runs one scene against both implementations and compares the outcome.
 *
 * Every conformance test has the same shape: build the same world on both, step both
 * the same number of times, and compare every body. Factored so a new scene is a
 * lambda rather than another forty lines of scaffolding.
 */
template <typename BuildScene>
static void expect_solvers_agree(int ticks, const StepParameters<Scalar>& parameters,
                                 BuildScene build)
{
    const PhysicsConfiguration configuration = conformance_scene();

    HostBackedSolver host(configuration);
    RuntimeBackedSolver runtime(configuration);

    const std::vector<BodyHandle> host_bodies = build(*host);
    const std::vector<BodyHandle> runtime_bodies = build(*runtime);
    ASSERT_EQ(host_bodies.size(), runtime_bodies.size());

    for (int tick = 0; tick < ticks; ++tick)
    {
        (*host).step(parameters);
        (*runtime).step(parameters);
    }

    EXPECT_EQ((*host).statistics().constraints, (*runtime).statistics().constraints);
    EXPECT_EQ((*host).statistics().colors, (*runtime).statistics().colors);
    EXPECT_EQ((*host).statistics().substeps, (*runtime).statistics().substeps)
        << "the substep schedule is derived from state, so both must derive the same";

    for (std::size_t i = 0; i < host_bodies.size(); ++i)
    {
        RigidBody from_host;
        RigidBody from_runtime;
        ASSERT_TRUE((*host).read_body(host_bodies[i], from_host));
        ASSERT_TRUE((*runtime).read_body(runtime_bodies[i], from_runtime));
        EXPECT_LT(disagreement(from_host, from_runtime), TOLERANCE)
            << "body " << i << " diverged between implementations";
    }
}

TEST(Integration_SolverConformance, FreeFallAgrees)
{
    // The simplest possible scene, and the one that isolates predict and derive from
    // the projection entirely. If this disagrees, nothing after it is worth reading.
    StepParameters<Scalar> parameters;
    expect_solvers_agree(30, parameters, [](IConstraintSolver<Scalar>& solver)
    {
        std::vector<BodyHandle> handles;
        for (int i = 0; i < 8; ++i)
            handles.push_back(solver.add_body(point_body(Vector3{Scalar(i), Scalar(10), 0})));
        return handles;
    });
}

TEST(Integration_SolverConformance, AChainAgreesAcrossEveryColour)
{
    // A chain is the scene colouring exists for: consecutive links share a body, so
    // they must land in different colours, and the colours must be applied in order.
    // Eight links over four colours exercises the ordering rather than assuming it.
    StepParameters<Scalar> parameters;
    expect_solvers_agree(40, parameters, [](IConstraintSolver<Scalar>& solver)
    {
        std::vector<BodyHandle> handles;
        for (int i = 0; i < 9; ++i)
            handles.push_back(solver.add_body(point_body(Vector3{Scalar(i) * Scalar(0.9), Scalar(5), 0})));

        // The first body is pinned, so the chain hangs rather than falling as a unit.
        RigidBody anchor = point_body(Vector3{0, Scalar(5), 0});
        anchor.inv_mass = 0;
        solver.write_body(handles[0], anchor);

        for (std::size_t i = 0; i + 1 < handles.size(); ++i)
            solver.add_constraint(link(solver.body_slot(handles[i]),
                                       solver.body_slot(handles[i + 1]), Scalar(1)));
        return handles;
    });
}

TEST(Integration_SolverConformance, AHubOfSpokesAgrees)
{
    // The case that fills colours fastest: every constraint meets at one body, so
    // each takes its own colour and none of them may run together. It is also the
    // shape a naive parallel projection gets wrong, which is what makes it worth
    // comparing rather than merely running.
    StepParameters<Scalar> parameters;
    expect_solvers_agree(25, parameters, [](IConstraintSolver<Scalar>& solver)
    {
        std::vector<BodyHandle> handles;

        // Two properties this scene needs, and both were absent from the obvious
        // version of it. The spokes start *inside* their rest length, so every
        // constraint has real work on every substep — laid out at rest length the
        // projection is a no-op and the comparison holds whatever order was used.
        // And the hub is free to move, so the projections are genuinely coupled:
        // pinning it makes each spoke's correction independent of the others, which
        // is correct physics and a useless ordering test.
        handles.push_back(solver.add_body(point_body(Vector3{0, Scalar(6), 0})));

        for (int i = 0; i < 6; ++i)
        {
            const Scalar angle = Scalar(i) * Scalar(1.047);
            handles.push_back(solver.add_body(point_body(
                Vector3{Scalar(std::cos(double(angle))) * Scalar(1.2), Scalar(6),
                        Scalar(std::sin(double(angle))) * Scalar(1.2)})));
        }
        for (std::size_t i = 1; i < handles.size(); ++i)
            solver.add_constraint(link(solver.body_slot(handles[0]),
                                       solver.body_slot(handles[i]), Scalar(1.5)));
        return handles;
    });
}

TEST(Integration_SolverConformance, TheAngularProjectionAgrees)
{
    // Off-centre anchors on bodies that can rotate, so the projection spends part of
    // its correction as a turn. The angular path has its own generalized inverse mass
    // and its own quaternion update, and a divergence there would be invisible in
    // every scene above.
    StepParameters<Scalar> parameters;
    expect_solvers_agree(30, parameters, [](IConstraintSolver<Scalar>& solver)
    {
        std::vector<BodyHandle> handles;
        handles.push_back(solver.add_body(spinning_body(Vector3{Scalar(-1), Scalar(4), 0})));
        handles.push_back(solver.add_body(spinning_body(Vector3{Scalar(1), Scalar(4), 0})));
        solver.add_constraint(link(solver.body_slot(handles[0]),
                                   solver.body_slot(handles[1]), Scalar(1.5),
                                   Vector3{0, Scalar(0.4), 0},
                                   Vector3{0, Scalar(-0.4), 0}));
        return handles;
    });
}

TEST(Integration_SolverConformance, AChurningWorldAgrees)
{
    // Bodies and constraints coming and going, which is what the mutable world is
    // for. It also exercises the part most likely to diverge: a swap-remove moves a
    // constraint to a different slot, so the two implementations must compact their
    // storage identically or their projection order silently drifts apart.
    const PhysicsConfiguration configuration = conformance_scene();
    HostBackedSolver host(configuration);
    RuntimeBackedSolver runtime(configuration);

    StepParameters<Scalar> parameters;

    std::vector<BodyHandle> host_bodies;
    std::vector<BodyHandle> runtime_bodies;
    std::vector<ConstraintHandle> host_links;
    std::vector<ConstraintHandle> runtime_links;

    // Spaced short of the rest length and anchored at one end, so the links are
    // under load throughout. Laid out at exactly the rest length the chain would be
    // in equilibrium, the projection would be a no-op, and the comparison would hold
    // whatever order the two solvers used.
    for (int i = 0; i < 6; ++i)
    {
        RigidBody body = point_body(Vector3{Scalar(i) * Scalar(0.8), Scalar(8), 0});
        if (i == 0)
            body.inv_mass = 0;
        host_bodies.push_back((*host).add_body(body));
        runtime_bodies.push_back((*runtime).add_body(body));
    }
    for (std::size_t i = 0; i + 1 < host_bodies.size(); ++i)
    {
        host_links.push_back((*host).add_constraint(
            link((*host).body_slot(host_bodies[i]), (*host).body_slot(host_bodies[i + 1]),
                 Scalar(1))));
        runtime_links.push_back((*runtime).add_constraint(
            link((*runtime).body_slot(runtime_bodies[i]),
                 (*runtime).body_slot(runtime_bodies[i + 1]), Scalar(1))));
    }

    for (int tick = 0; tick < 12; ++tick)
    {
        (*host).step(parameters);
        (*runtime).step(parameters);
        ASSERT_EQ((*host).statistics().substeps, (*runtime).statistics().substeps)
            << "the substep schedules parted at tick " << tick;
        ASSERT_EQ((*host).statistics().constraints, (*runtime).statistics().constraints)
            << "the constraint sets parted at tick " << tick;

        // Drop the middle link on both, then put an equivalent one back. The removal
        // is the interesting half: it compacts the band by swapping the last entry
        // down, and both must swap the same entry into the same slot.
        if (tick == 4)
        {
            EXPECT_TRUE((*host).remove_constraint(host_links[2]));
            EXPECT_TRUE((*runtime).remove_constraint(runtime_links[2]));
        }
        if (tick == 7)
        {
            (*host).add_constraint(link((*host).body_slot(host_bodies[0]),
                                        (*host).body_slot(host_bodies[5]), Scalar(3)));
            (*runtime).add_constraint(link((*runtime).body_slot(runtime_bodies[0]),
                                           (*runtime).body_slot(runtime_bodies[5]),
                                           Scalar(3)));
        }
    }

    EXPECT_EQ((*host).statistics().constraints, (*runtime).statistics().constraints);
    for (std::size_t i = 0; i < host_bodies.size(); ++i)
    {
        RigidBody from_host;
        RigidBody from_runtime;
        ASSERT_TRUE((*host).read_body(host_bodies[i], from_host));
        ASSERT_TRUE((*runtime).read_body(runtime_bodies[i], from_runtime));
        EXPECT_LT(disagreement(from_host, from_runtime), TOLERANCE)
            << "body " << i << " diverged after the world churned";
    }
}

TEST(Integration_SolverConformance, TheHandleContractIsTheSameOnBoth)
{
    // Not arithmetic: the lifetime rules. A stale handle must be refused by every
    // implementation, or code written against one silently misbehaves on the other.
    const PhysicsConfiguration configuration = conformance_scene();
    HostBackedSolver host(configuration);
    RuntimeBackedSolver runtime(configuration);

    IConstraintSolver<Scalar>* solvers[2] = {&(*host), &(*runtime)};
    for (IConstraintSolver<Scalar>* solver : solvers)
    {
        const BodyHandle first = solver->add_body(point_body(Vector3{0, 0, 0}));
        ASSERT_TRUE(first.valid());
        EXPECT_TRUE(solver->remove_body(first));

        const BodyHandle second = solver->add_body(point_body(Vector3{1, 1, 1}));
        EXPECT_EQ(first.index, second.index);
        EXPECT_NE(first, second);

        RigidBody body;
        EXPECT_FALSE(solver->read_body(first, body));
        EXPECT_TRUE(solver->read_body(second, body));
        EXPECT_FALSE(solver->remove_body(first));
        EXPECT_EQ(solver->body_slot(first), solver->body_capacity());
    }
}

TEST(Integration_SolverConformance, RemovingABodyTakesItsConstraintsOnBoth)
{
    const PhysicsConfiguration configuration = conformance_scene();
    HostBackedSolver host(configuration);
    RuntimeBackedSolver runtime(configuration);

    IConstraintSolver<Scalar>* solvers[2] = {&(*host), &(*runtime)};
    StepParameters<Scalar> parameters;
    for (IConstraintSolver<Scalar>* solver : solvers)
    {
        const BodyHandle a = solver->add_body(point_body(Vector3{0, 0, 0}));
        const BodyHandle b = solver->add_body(point_body(Vector3{1, 0, 0}));
        const ConstraintHandle joined = solver->add_constraint(
            link(solver->body_slot(a), solver->body_slot(b), Scalar(1)));
        ASSERT_TRUE(joined.valid());

        solver->step(parameters);
        EXPECT_EQ(solver->statistics().constraints, 1u);

        EXPECT_TRUE(solver->remove_body(b));
        solver->step(parameters);
        EXPECT_EQ(solver->statistics().constraints, 0u);
        EXPECT_FALSE(solver->remove_constraint(joined));
    }
}
