/**************************************************************************/
/* soft_body_half_storage_budget.cpp                                     */
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
 * @file soft_body_half_storage_budget.cpp
 * @brief §6.5's second half, measured: the same soft body, `float` storage
 *        against `sycl::half` storage, one wall-clock number each.
 *
 * `soft_body_budget.cpp` measures §13.1's device-graph line; this measures a
 * different, narrower question P8 actually needs answered before
 * `soft_body_half_storage.hpp`'s type is kept or dropped: does narrowing a
 * cosmetic body's *stored* position and velocity to `sycl::half` between
 * ticks pay for itself, or does the narrow/widen conversion cost more than
 * the bandwidth it saves?
 *
 * **This program does not decide that.** It builds the identical scene twice
 * — same lattice, same material, same particle count — and steps one copy
 * with plain `float` storage and the other through `SoftBodyHalfStorage`'s
 * narrow/widen seam once a tick, timing each the same way
 * `soft_body_budget.cpp` times a tick. The two mean/best numbers below, and
 * their difference, are the measurement §6.5 asks for; reading them and
 * deciding keep-or-drop is a step this program deliberately leaves undone.
 *
 * **What this can and cannot show.** `FiniteElementModel<T>` is the host-only
 * reference solver (`finite_element_model.hpp`'s own file comment): there is
 * no device-resident half-precision buffer for this to measure yet, because
 * the FEM element is not a constraint kind in the shared device solver
 * (`runtime_graph_builder.hpp`) at all. So this measures the one thing that
 * *does* exist today — a single-threaded host loop's cost to narrow and widen
 * an array of positions and velocities once a tick — not the device
 * bandwidth saving §6.5 is actually written about. A positive result here
 * (half pays for itself even paying pure host conversion cost) would be
 * strong evidence for the device case; a negative result here is much weaker
 * evidence against it, since a real device buffer's bandwidth saving has no
 * host analogue. That asymmetry belongs in whatever reads this number.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <SushiEngine/physics/soft/fem_projection.hpp>
#include <SushiEngine/physics/soft/finite_element_model.hpp>
#include <SushiEngine/physics/soft/soft_body_half_storage.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief Cells per axis; six tetrahedra each, so 10³ × 6 = 6 000 elements. */
    constexpr int CELLS = 10;

    /** @brief Grid points per axis. Not `POINTS`: windef.h already owns that name. */
    constexpr int GRID_POINTS = CELLS + 1;

    /** @brief Spacing between grid points, in metres. */
    constexpr float SPACING = 0.05f;

    /** @brief Ticks discarded before timing, so first-touch is not counted. */
    constexpr int WARMUP_TICKS = 3;

    /** @brief Ticks timed. */
    constexpr int TIMED_TICKS = 30;

    /** @brief Sub-steps per tick — matched to `soft_body_budget.cpp`'s target schedule. */
    constexpr std::size_t SUBSTEPS = 32;

    /** @brief The grid index of point (x, y, z). */
    std::uint32_t point_index(int x, int y, int z)
    {
        return std::uint32_t((z * GRID_POINTS + y) * GRID_POINTS + x);
    }

    /**
     * @brief Freudenthal's six tetrahedra for one cell, as corner-bit triples.
     *
     * The same decomposition `soft_body_budget.cpp` uses — every tetrahedron
     * contains corners 0 and 7, the cell's main diagonal, so neighbouring
     * cells agree on how their shared face is cut and no element sees a
     * hanging node.
     */
    constexpr int CELL_TETRAHEDRA[6][4] = {{0, 1, 3, 7}, {0, 1, 5, 7}, {0, 4, 5, 7},
                                           {0, 4, 6, 7}, {0, 2, 6, 7}, {0, 2, 3, 7}};

    /** @brief One element, cooked from its own rest corners. */
    FemTetrahedronT<float> cook(const std::uint32_t vertex[4], const Vector3T<float> rest[4],
                               const LameParameters<float>& lame)
    {
        FemTetrahedronT<float> element;
        for (int i = 0; i < 4; ++i)
            element.vertex[i] = vertex[i];

        FemMatrix3<float> rest_shape;
        rest_shape.column0 = rest[1] - rest[0];
        rest_shape.column1 = rest[2] - rest[0];
        rest_shape.column2 = rest[3] - rest[0];

        FemMatrix3<float> inverse;
        invert_fem_matrix3(rest_shape, inverse);
        element.rest_inverse_column_0 = inverse.column0;
        element.rest_inverse_column_1 = inverse.column1;
        element.rest_inverse_column_2 = inverse.column2;
        element.plastic_inverse_column_0 = inverse.column0;
        element.plastic_inverse_column_1 = inverse.column1;
        element.plastic_inverse_column_2 = inverse.column2;

        element.rest_volume = std::fabs(determinant(rest_shape)) / 6.0f;
        element.mu = lame.mu;
        element.lambda = lame.lambda;
        return element;
    }

    /** @brief Builds the lattice: a block hanging from its own top layer, so it deforms. */
    FiniteElementModel<float> build_scene()
    {
        FiniteElementModel<float> model;
        model.material = rubber_material<float>();

        std::vector<Vector3T<float>> rest;
        rest.reserve(std::size_t(GRID_POINTS) * GRID_POINTS * GRID_POINTS);
        for (int z = 0; z < GRID_POINTS; ++z)
            for (int y = 0; y < GRID_POINTS; ++y)
                for (int x = 0; x < GRID_POINTS; ++x)
                    rest.push_back(Vector3T<float>{x * SPACING, 1.0f + y * SPACING, z * SPACING});

        model.particles.resize(rest.size());
        for (std::size_t i = 0; i < rest.size(); ++i)
        {
            RigidBodyT<float>& particle = model.particles[i];
            particle.position = rest[i];
            particle.prev_position = rest[i];
            particle.orientation = QuaternionT<float>{0, 0, 0, 1};
            particle.prev_orientation = particle.orientation;
            particle.inv_mass = 1.0f;
        }

        // The top layer is pinned, so the block hangs and every element deforms
        // under its own weight rather than free-falling with no internal force
        // at all — the same reason `soft_body_budget.cpp` pins its top layer.
        for (int z = 0; z < GRID_POINTS; ++z)
            for (int x = 0; x < GRID_POINTS; ++x)
                model.particles[point_index(x, GRID_POINTS - 1, z)].inv_mass = 0.0f;

        const LameParameters<float> lame = lame_parameters(model.material);
        model.elements.reserve(std::size_t(CELLS) * CELLS * CELLS * 6);
        for (int z = 0; z < CELLS; ++z)
            for (int y = 0; y < CELLS; ++y)
                for (int x = 0; x < CELLS; ++x)
                    for (const auto& corners : CELL_TETRAHEDRA)
                    {
                        std::uint32_t vertex[4];
                        Vector3T<float> corner_rest[4];
                        for (int i = 0; i < 4; ++i)
                        {
                            const int bits = corners[i];
                            vertex[i] = point_index(x + (bits & 1), y + ((bits >> 1) & 1),
                                                    z + ((bits >> 2) & 1));
                            corner_rest[i] = rest[vertex[i]];
                        }
                        model.elements.push_back(cook(vertex, corner_rest, lame));
                    }

        model.set_external_acceleration(Vector3T<float>{0, -9.81f, 0});
        return model;
    }

    /** @brief One precision's timing: mean and best tick, over @ref TIMED_TICKS ticks. */
    struct TimingResult
    {
        double mean_ms = 0.0;
        double best_ms = 0.0;
    };

    /** @brief Times @p body_of_tick, called once per tick, after @ref WARMUP_TICKS untimed ticks. */
    template <typename TickFunction>
    TimingResult time_ticks(TickFunction&& body_of_tick)
    {
        for (int tick = 0; tick < WARMUP_TICKS; ++tick)
            body_of_tick();

        double total_ms = 0.0;
        double best_ms = 0.0;
        for (int tick = 0; tick < TIMED_TICKS; ++tick)
        {
            const std::chrono::steady_clock::time_point began = std::chrono::steady_clock::now();
            body_of_tick();
            const std::chrono::duration<double, std::milli> elapsed =
                std::chrono::steady_clock::now() - began;
            total_ms += elapsed.count();
            if (tick == 0 || elapsed.count() < best_ms)
                best_ms = elapsed.count();
        }

        TimingResult result;
        result.mean_ms = total_ms / double(TIMED_TICKS);
        result.best_ms = best_ms;
        return result;
    }
} // namespace

int main()
{
    const FiniteElementModel<float> scene = build_scene();
    if (scene.particles.empty() || scene.elements.empty())
    {
        std::printf("soft_body_half_storage_budget: FAILED to build the scene\n");
        return 1;
    }

    constexpr float DELTA_TIME = 1.0f / 60.0f;

    // [FLOAT STORAGE]: the scene as `SoftBodyInstance`'s cosmetic column already
    // steps it today -- no narrow/widen seam anywhere in the loop.
    FiniteElementModel<float> float_storage_model = scene;
    const TimingResult float_storage =
        time_ticks([&]() { float_storage_model.step(DELTA_TIME, SUBSTEPS); });

    // [HALF STORAGE]: the same scene, but its position and velocity live in a
    // `SoftBodyHalfStorage` between ticks. Each timed tick pays for exactly
    // the two seams §6.5 draws -- one widen before the step, one narrow after
    // -- in addition to the identical `float` step in between.
    FiniteElementModel<float> half_storage_model = scene;
    SoftBodyHalfStorage half_storage;
    half_storage.narrow_from(half_storage_model);
    const TimingResult half_storage_timing = time_ticks(
        [&]()
        {
            half_storage.widen_into(half_storage_model);
            half_storage_model.step(DELTA_TIME, SUBSTEPS);
            half_storage.narrow_from(half_storage_model);
        });

    std::printf("soft_body_half_storage_budget: docs/slop/physics_system.md §6.5, second half\n");
    std::printf("  scene: %zu particles, %zu elements, %zu substeps, %d timed ticks\n",
                scene.particles.size(), scene.elements.size(), SUBSTEPS, TIMED_TICKS);
    std::printf("\n");
    std::printf("  [FLOAT STORAGE]  mean %8.4f ms/tick   best %8.4f ms/tick\n",
                float_storage.mean_ms, float_storage.best_ms);
    std::printf("  [HALF  STORAGE]  mean %8.4f ms/tick   best %8.4f ms/tick   "
                "(includes the widen + narrow this precision pays that float does not)\n",
                half_storage_timing.mean_ms, half_storage_timing.best_ms);
    std::printf("\n");
    const double mean_delta_ms = half_storage_timing.mean_ms - float_storage.mean_ms;
    const double mean_delta_percent =
        float_storage.mean_ms > 0.0 ? 100.0 * mean_delta_ms / float_storage.mean_ms : 0.0;
    std::printf("  delta (half - float), mean: %+8.4f ms/tick (%+6.1f%%)\n", mean_delta_ms,
                mean_delta_percent);
    std::printf("\n");
    std::printf("  This number is a measurement, not a verdict -- see this file's own comment\n");
    std::printf("  for what it can and cannot show about the device-buffer case §6.5 is about.\n");

    return 0;
}
