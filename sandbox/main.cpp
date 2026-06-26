/**************************************************************************/
/* main.cpp                                                              */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/* Licensed under the Apache License, Version 2.0.                         */
/**************************************************************************/

// Milestone A sandbox: the smallest "game". It spawns a million entities under a
// constant downward velocity, integrates them on the runtime for a fixed number
// of steps, then reads a few positions back to confirm the head drives its
// battery correctly and prints the per-step cost.

#include <cmath>
#include <cstddef>
#include <cstdio>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

int main()
{
    const std::size_t entities = 1'000'000;
    const std::size_t steps    = 100;
    const Scalar      dt        = Scalar(0.016);
    const Vec3        gravity    = Vec3{Scalar(0), Scalar(-9.81), Scalar(0)};

    Application app(entities, dt);
    World& world = app.world();

    // Seed the shared-USM fields directly from the host: every entity starts at
    // the origin with the same gravity-like velocity.
    for (std::size_t i = 0; i < entities; ++i)
    {
        world.position()[i] = Vec3{};
        world.velocity()[i] = gravity;
    }

    const SushiRuntime::RunReport report = app.run_steps(steps);

    // Read back a small sample. Every entity follows the same path, so position.y
    // after S steps must be gravity.y * dt * S.
    const auto sample = world.position().read_range(0, 4);
    const Scalar expected_y = gravity.y * dt * Scalar(steps);

    bool ok = true;
    for (const Vec3& p : sample)
    {
        if (std::fabs(p.y - expected_y) > Scalar(1e-2))
            ok = false;
    }

    std::printf("entities=%zu steps=%zu dt=%.3f\n", entities, steps, double(dt));
    std::printf("position.y: got %.4f, expected %.4f -> %s\n",
                double(sample.empty() ? 0.0f : sample[0].y),
                double(expected_y),
                ok ? "OK" : "MISMATCH");
    std::printf("run: %.3f ms total, %.4f ms/step (analysis %.3f ms)\n",
                report.total_duration_ms,
                steps ? report.total_duration_ms / double(steps) : 0.0,
                report.graph_analysis_duration_ms);

    return ok ? 0 : 1;
}
