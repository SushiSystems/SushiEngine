/**************************************************************************/
/* soft_body_budget.cpp                                                   */
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

/**
 * @file soft_body_budget.cpp
 * @brief §13.1's soft-body line, measured: one body, 20 000 tetrahedra, 32 substeps.
 *
 * P6's acceptance list ends with a number — `≤ 3 ms/tick` — and until P6-J2 there was
 * nothing to measure, because the FEM element was not a constraint kind in the device
 * graph and a soft body could only be stepped by a host loop. This is that measurement.
 *
 * **Why a probe and not a test.** §13.1 states its targets against "one desktop-class
 * GPU through SushiRuntime, 60 Hz tick". A test asserting 3 ms would be asserting the
 * machine it happens to run on: the same code on a CPU backend, a laptop, or a shared
 * CI runner produces a different number, and a suite that goes red on hardware is a
 * suite people learn to ignore. So the *shape* of the scene is pinned by the
 * conformance suite, and the *number* is reported here — the arrangement
 * `atmosphere_probe` already uses for §12's step-cost line. This exits non-zero only
 * when the scene could not be built, which is a defect rather than a slow machine.
 *
 * The lattice is Freudenthal's: every cell of a cubic grid cut into six tetrahedra
 * sharing the cell's main diagonal. It is the honest stress case for the colouring —
 * an interior vertex is touched by twenty-four elements, so the greedy colouring needs
 * at least that many colours and the graph carries one node per colour per substep.
 */

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
#include <SushiEngine/physics/solver/runtime_graph_builder.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief Cells per axis; six tetrahedra each, so 15³ × 6 = 20 250 elements. */
    constexpr int CELLS = 15;

    /** @brief Grid points per axis. Not `POINTS`: windef.h already owns that name. */
    constexpr int GRID_POINTS = CELLS + 1;

    /** @brief Particles in the block: one per grid point. */
    constexpr int PARTICLES = GRID_POINTS * GRID_POINTS * GRID_POINTS;

    /** @brief The colour ceiling; see where it is set for why forty-eight. */
    constexpr int COLORS = 48;

    /** @brief Spacing between grid points, in metres. */
    constexpr double SPACING = 0.05;

    /** @brief §13.1's budget for this scene. */
    constexpr double BUDGET_MS = 3.0;

    /** @brief Ticks discarded before timing, so composition and first touch are not counted. */
    constexpr int WARMUP_TICKS = 3;

    /** @brief Ticks timed. */
    constexpr int TIMED_TICKS = 20;

    /** @brief The grid index of point (x, y, z). */
    std::uint32_t point_index(int x, int y, int z)
    {
        return std::uint32_t((z * GRID_POINTS + y) * GRID_POINTS + x);
    }

    /**
     * @brief Freudenthal's six tetrahedra for one cell, as corner-bit triples.
     *
     * Every one contains corners 0 and 7 — the cell's main diagonal — which is what
     * makes the decomposition conforming: two neighbouring cells cut their shared face
     * along the same edge, so no element sees a hanging node.
     */
    constexpr int CELL_TETRAHEDRA[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 4, 5, 7},
                                           {0, 4, 6, 7}, {0, 2, 6, 7}, {0, 2, 3, 7}};

    /** @brief A unit-mass particle at @p position with no rotational freedom. */
    RigidBody particle(const Vector3& position)
    {
        RigidBody body;
        body.position = position;
        body.prev_position = position;
        body.inv_mass = Scalar(1);
        return body;
    }

    /** @brief One element, cooked from its own rest corners (see the conformance suite). */
    FemTetrahedron cook(const std::uint32_t vertex[4], const Vector3 rest[4],
                        const LameParameters<Scalar>& lame)
    {
        FemTetrahedron element;
        for (int i = 0; i < 4; ++i)
            element.vertex[i] = vertex[i];

        FemMatrix3<Scalar> rest_shape;
        rest_shape.column0 = rest[1] - rest[0];
        rest_shape.column1 = rest[2] - rest[0];
        rest_shape.column2 = rest[3] - rest[0];

        FemMatrix3<Scalar> inverse;
        invert_fem_matrix3(rest_shape, inverse);
        element.rest_inverse_column_0 = inverse.column0;
        element.rest_inverse_column_1 = inverse.column1;
        element.rest_inverse_column_2 = inverse.column2;
        element.plastic_inverse_column_0 = inverse.column0;
        element.plastic_inverse_column_1 = inverse.column1;
        element.plastic_inverse_column_2 = inverse.column2;

        element.rest_volume = std::abs(determinant(rest_shape)) / Scalar(6);
        element.mu = lame.mu;
        element.lambda = lame.lambda;
        return element;
    }
}

int main()
{
    PhysicsConfiguration configuration;
    configuration.capacities.bodies = PARTICLES;
    configuration.capacities.constraints = 1;
    configuration.capacities.joints = 0;
    configuration.capacities.contacts = 0;
    // An interior vertex of this lattice is shared by twenty-four elements, and greedy
    // colouring needs at least that many colours to keep every colour conflict-free.
    // Forty-eight leaves headroom without reaching the sixty-four the colour mask can
    // address at all.
    configuration.capacities.colors = COLORS;
    // Not the element count: the store bands the budget evenly across the colours, so
    // the number that has to fit is the busiest band. A colour class is a set of
    // vertex-disjoint tetrahedra, so a quarter of the particles bounds it from above
    // whatever order the elements arrive in. Over-allocating against that bound is the
    // honest thing for a probe to do -- a measurement taken on a scene that quietly
    // dropped a third of its elements would be a measurement of nothing.
    configuration.capacities.elements = std::size_t(COLORS) * (PARTICLES / 4);
    // The target names 32 substeps, so the schedule is pinned rather than derived: a
    // measurement taken at whatever count this tick's motion happened to ask for would
    // not be the measurement §13.1 asked for.
    configuration.substeps.minimum = 32;
    configuration.substeps.maximum = 32;

    SushiRuntime::API::Runtime runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    RuntimeGraphBuilder<Scalar> solver(execution, configuration);

    std::vector<Vector3> rest;
    rest.reserve(std::size_t(GRID_POINTS) * GRID_POINTS * GRID_POINTS);
    for (int z = 0; z < GRID_POINTS; ++z)
        for (int y = 0; y < GRID_POINTS; ++y)
            for (int x = 0; x < GRID_POINTS; ++x)
                rest.push_back(Vector3{Scalar(x * SPACING), Scalar(1.0 + y * SPACING),
                                       Scalar(z * SPACING)});

    std::size_t rejected_bodies = 0;
    std::vector<BodyHandle> handles;
    handles.reserve(rest.size());
    for (const Vector3& position : rest)
    {
        // Kept rather than reconstructed from the slot index: a handle carries a
        // generation, and a guessed one is a handle that silently addresses nothing.
        handles.push_back(solver.add_body(particle(position)));
        if (!handles.back().valid())
            ++rejected_bodies;
    }

    // The top layer is pinned, so the block hangs and every element deforms. A block in
    // free fall would move without a single projection doing work, which is the one way
    // to measure this scene and learn nothing.
    for (int z = 0; z < GRID_POINTS; ++z)
        for (int x = 0; x < GRID_POINTS; ++x)
        {
            const std::uint32_t index = point_index(x, GRID_POINTS - 1, z);
            RigidBody anchor = particle(rest[index]);
            anchor.inv_mass = 0;
            solver.write_body(handles[index], anchor);
        }

    const LameParameters<Scalar> lame = lame_parameters(SoftBodyMaterial{});

    std::size_t placed = 0;
    std::size_t rejected_elements = 0;
    for (int z = 0; z < CELLS; ++z)
        for (int y = 0; y < CELLS; ++y)
            for (int x = 0; x < CELLS; ++x)
                for (const auto& corners : CELL_TETRAHEDRA)
                {
                    std::uint32_t vertex[4];
                    Vector3 corner_rest[4];
                    for (int i = 0; i < 4; ++i)
                    {
                        const int bits = corners[i];
                        vertex[i] = point_index(x + (bits & 1), y + ((bits >> 1) & 1),
                                                z + ((bits >> 2) & 1));
                        corner_rest[i] = rest[vertex[i]];
                    }
                    if (solver.add_element(cook(vertex, corner_rest, lame)).valid())
                        ++placed;
                    else
                        ++rejected_elements;
                }

    StepParameters<Scalar> parameters;
    parameters.delta_time = Scalar(1.0 / 60.0);

    for (int tick = 0; tick < WARMUP_TICKS; ++tick)
        solver.step(parameters);

    double total_ms = 0.0;
    double best_ms = 0.0;
    for (int tick = 0; tick < TIMED_TICKS; ++tick)
    {
        const std::chrono::steady_clock::time_point began =
            std::chrono::steady_clock::now();
        solver.step(parameters);
        const std::chrono::duration<double, std::milli> elapsed =
            std::chrono::steady_clock::now() - began;
        total_ms += elapsed.count();
        if (tick == 0 || elapsed.count() < best_ms)
            best_ms = elapsed.count();
    }

    const double mean_ms = total_ms / double(TIMED_TICKS);

    std::printf("soft_body_budget: section 13.1, one soft body, %d substeps\n",
                int(solver.statistics().substeps));
    std::printf("  elements placed   %zu (rejected %zu)\n", placed, rejected_elements);
    std::printf("  particles         %zu (rejected %zu)\n", rest.size(), rejected_bodies);
    std::printf("  colours used      %zu of %zu\n", solver.statistics().colors,
                configuration.capacities.colors);
    std::printf("  recompositions    %zu (must stay at 0 after the first tick)\n",
                solver.statistics().compose_count);
    std::printf("  mean              %.3f ms/tick\n", mean_ms);
    std::printf("  best              %.3f ms/tick\n", best_ms);
    std::printf("  budget            %.3f ms/tick -> %s\n", BUDGET_MS,
                mean_ms <= BUDGET_MS ? "UNDER" : "OVER");

    // Only a scene that could not be built is a failure. The number above is a property
    // of this machine, and this program's job is to report it, not to grade it.
    if (rejected_elements > 0 || rejected_bodies > 0)
    {
        std::printf("soft_body_budget: FAILED to build the scene\n");
        return 1;
    }
    return 0;
}
