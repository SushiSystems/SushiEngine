/**************************************************************************/
/* main.cpp                                                               */
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

// WP-3 worked example: a particle world driven entirely by the ECS, validated
// against an independent scalar reference. It proves the WP-3 thesis end to end:
//
//   * Components live in archetype chunks; a system declares the components it
//     reads and writes and the runtime's dependency tracker orders the systems —
//     apply_forces writes velocity, integrate reads it (ordered after), while
//     decay_lifetime touches a disjoint component and runs in parallel with both.
//   * The schedule is compiled once and replayed every frame; entities spawn and
//     die each frame through a deferred command buffer applied at the frame
//     barrier, varying the per-chunk entity count with no per-frame recompile
//     (asserted: compile_count == 1).
//   * Every surviving entity's state is checked against a scalar loop running the
//     same systems in the same order, so "it behaves like a game engine" is a
//     verified claim.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;

namespace
{
    // Components. Distinct types so each gets its own component id and column.
    struct Position { Vector3 v; };
    struct Velocity { Vector3 v; };
    struct Mass     { Scalar value; };
    struct Lifetime { Scalar value; };

    constexpr Scalar DT     = Scalar(0.01);
    constexpr Scalar FORCE_Y = Scalar(-9.8);   // a downward force; accel = F / mass
    constexpr std::size_t FRAMES         = 300;
    constexpr std::size_t INITIAL        = 500;
    constexpr std::size_t SPAWN_PER_FRAME = 4;
    constexpr std::size_t CHUNK_CAPACITY = 2048; // one chunk holds the whole world

    // Independent scalar mirror of one entity, keyed by entity slot.
    struct Reference
    {
        Vector3 position;
        Vector3 velocity;
        Scalar mass;
        Scalar lifetime;
    };

    Scalar mass_for(std::uint64_t k)     { return Scalar(1) + Scalar(k % 4); }
    Scalar lifetime_for(std::uint64_t k) { return Scalar(0.3) + Scalar(k % 8) * Scalar(0.1); }
}

int main()
{
    auto runtime = SushiRuntime::API::Runtime::create();
    World world(runtime, CHUNK_CAPACITY);
    Schedule schedule(runtime);

    // One archetype for the whole world; pre-reserve so spawns never allocate a
    // new chunk mid-run (which would force a recompile).
    world.reserve<Position, Velocity, Mass, Lifetime>(CHUNK_CAPACITY);

    // Systems. apply_forces -> integrate is a read-after-write chain on velocity;
    // decay_lifetime is independent and runs in parallel.
    schedule.each<Write<Velocity>, Read<Mass>>("apply_forces",
        [](std::size_t i, Velocity* vel, const Mass* mass)
        {
            vel[i].v.y += (FORCE_Y / mass[i].value) * DT;
        });
    schedule.each<Write<Position>, Read<Velocity>>("integrate",
        [](std::size_t i, Position* pos, const Velocity* vel)
        {
            pos[i].v = pos[i].v + vel[i].v * DT;
        });
    schedule.each<Write<Lifetime>>("decay_lifetime",
        [](std::size_t i, Lifetime* life)
        {
            life[i].value -= DT;
        });

    std::unordered_map<std::uint32_t, Reference> reference;
    std::vector<Entity> alive;
    CommandBuffer commands;
    std::uint64_t spawn_counter = 0;
    std::size_t total_spawned = 0, total_destroyed = 0;

    auto spawn_one = [&]()
    {
        const Scalar m = mass_for(spawn_counter);
        const Scalar l = lifetime_for(spawn_counter);
        ++spawn_counter;
        const Entity e = world.spawn(Position{}, Velocity{}, Mass{m}, Lifetime{l});
        alive.push_back(e);
        reference[e.index] = Reference{Vector3{}, Vector3{}, m, l};
        ++total_spawned;
    };

    for (std::size_t i = 0; i < INITIAL; ++i)
        spawn_one();

    double sim_ms = 0.0;

    for (std::size_t frame = 0; frame < FRAMES; ++frame)
    {
        // ECS systems for this frame.
        const SushiRuntime::RunReport report = schedule.run(world);
        sim_ms += report.total_duration_ms;

        // Scalar reference: same three systems, same order, per entity.
        for (Entity e : alive)
        {
            Reference& r = reference[e.index];
            r.velocity.y += (FORCE_Y / r.mass) * DT;
            r.position = r.position + r.velocity * DT;
            r.lifetime -= DT;
        }

        // Sync point: decide despawns from the reference truth, mirror into the
        // command buffer, and rebuild the live list with the survivors.
        std::vector<Entity> survivors;
        survivors.reserve(alive.size());
        for (Entity e : alive)
        {
            if (reference[e.index].lifetime <= Scalar(0))
            {
                commands.destroy(e);
                ++total_destroyed;
            }
            else
            {
                survivors.push_back(e);
            }
        }
        alive = std::move(survivors);

        // Spawn this frame's new entities (immediate, so we learn their handles).
        for (std::size_t k = 0; k < SPAWN_PER_FRAME; ++k)
            spawn_one();

        // Apply the deferred destroys at the barrier.
        commands.apply(world);
    }

    // Verify every survivor against its reference.
    std::size_t mismatches = 0;
    const Scalar tol = Scalar(0.05);
    for (Entity e : alive)
    {
        if (!world.alive(e)) { ++mismatches; continue; }
        const Reference& r = reference[e.index];
        const Vector3 p = world.get<Position>(e).v;
        const Vector3 v = world.get<Velocity>(e).v;
        const Scalar life = world.get<Lifetime>(e).value;
        if (std::fabs(p.y - r.position.y) > tol ||
            std::fabs(v.y - r.velocity.y) > tol ||
            std::fabs(life - r.lifetime) > tol)
        {
            ++mismatches;
        }
    }

    const std::size_t compiles = schedule.compile_count();

    std::printf("systems=%zu  alive=%zu  spawned=%zu  destroyed=%zu\n",
                schedule.system_count(), alive.size(), total_spawned, total_destroyed);
    std::printf("compile_count=%zu (expected 1)  mismatches=%zu (expected 0)\n",
                compiles, mismatches);
    std::printf("sim: %.3f ms over %zu frames, %.4f ms/frame\n",
                sim_ms, FRAMES, FRAMES ? sim_ms / double(FRAMES) : 0.0);

    const bool ok = (mismatches == 0) && (compiles == 1) && (alive.size() > 0);
    std::printf("RESULT: %s\n", ok ? "OK" : "FAIL");
    return ok ? 0 : 1;
}
