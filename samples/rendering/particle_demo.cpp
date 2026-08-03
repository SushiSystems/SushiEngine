/**************************************************************************/
/* particle_demo.cpp                                                      */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// particle_demo: a headless, self-checking proof of the VFX particle system's CPU-verifiable
// half (docs/design/vfx_particle_system.md). It authors a fire-like effect, compiles it to the
// POD boundary both backends consume, and drives the deterministic CPU backend to prove:
//   - the compiler bakes curves/gradients and clamps the deterministic domain's capacity;
//   - the pool spawns, ages, and retires within its budget;
//   - the simulation is bit-reproducible (two runs byte-identical) and rollback-safe (snapshot,
//     restore, replay reproduces the state byte-for-byte).
// The GPU cosmetic backend is validated visually in the editor; this demo covers what a headless
// run can prove. Exits 0 on success, 1 on any failure.

#include <cstdio>
#include <cstring>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/vfx/deterministic_backend.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Vfx;

namespace
{
    int failures = 0;

    void check(bool condition, const char* message)
    {
        if (!condition)
        {
            std::printf("  FAIL: %s\n", message);
            ++failures;
        }
    }

    // A fire/spark effect exercising every deterministic code path.
    ParticleEffect make_fire_effect()
    {
        EmitterDescriptor emitter;
        emitter.name = "sparks";
        emitter.domain = SimulationDomain::Deterministic;
        emitter.capacity = 400;
        emitter.spawn.rate_per_second = 90.0f;
        emitter.spawn.bursts.push_back(ParticleBurst{0.25f, 30});
        emitter.shape.shape = EmitterShape::Cone;
        emitter.shape.radius = 0.3f;
        emitter.shape.cone_angle_radians = 0.4f;
        emitter.init.lifetime_min = 0.8f;
        emitter.init.lifetime_max = 1.6f;
        emitter.init.speed_min = 2.0f;
        emitter.init.speed_max = 4.5f;
        emitter.init.size_min = 0.04f;
        emitter.init.size_max = 0.12f;
        emitter.gravity.enabled = true;
        emitter.gravity.acceleration = Vector3{0, 4.0, 0}; // buoyant rise
        emitter.drag.enabled = true;
        emitter.drag.coefficient = 0.4f;
        emitter.turbulence.enabled = true;
        emitter.turbulence.frequency = 0.9f;
        emitter.turbulence.amplitude = 1.5f;
        emitter.size_over_life.enabled = true;
        emitter.size_over_life.curve.add_key(CurveKey{0.0f, 0.3f, 0.0f, 1.5f});
        emitter.size_over_life.curve.add_key(CurveKey{0.25f, 1.0f, 0.0f, 0.0f});
        emitter.size_over_life.curve.add_key(CurveKey{1.0f, 0.0f, -1.0f, 0.0f});
        emitter.color_over_life.enabled = true;
        emitter.color_over_life.gradient.add_color_key(ColorKey{0.0f, Vector3{1.0, 0.95, 0.5}});
        emitter.color_over_life.gradient.add_color_key(ColorKey{0.5f, Vector3{1.0, 0.35, 0.1}});
        emitter.color_over_life.gradient.add_color_key(ColorKey{1.0f, Vector3{0.15, 0.05, 0.05}});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{0.0f, 0.0f});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{0.12f, 1.0f});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{1.0f, 0.0f});

        ParticleEffect effect;
        effect.name = "campfire";
        effect.emitters.push_back(emitter);
        return effect;
    }

    const float DT = 1.0f / 60.0f;

    DeterministicEmitterState run(const CompiledEffect& effect, std::uint32_t seed, int steps)
    {
        DeterministicEmitterState state;
        CpuDeterministicBackend::reset(state, seed);
        const Vector3 position{0, 0.5, 0};
        const Quaternion rotation{};
        for (int i = 0; i < steps; ++i)
            CpuDeterministicBackend::step(state, effect.emitters[0], effect, DT, position, rotation);
        return state;
    }
}

int main()
{
    std::printf("particle_demo: authoring + deterministic backend\n");

    const ParticleEffect effect = make_fire_effect();
    EffectDatabase database;
    const AssetId id = database.add(effect);
    const CompiledEffect& compiled = database.compiled(id);

    // Compilation: one emitter, capacity clamped to the deterministic budget, both LUTs baked.
    check(compiled.emitters.size() == 1, "one compiled emitter");
    check(compiled.emitters[0].capacity == 400, "capacity preserved under the deterministic cap");
    check((compiled.emitters[0].update_flags & UPDATE_TURBULENCE) != 0, "turbulence flagged");
    check(compiled.emitters[0].size_curve_lut != NO_LUT, "size curve baked");
    check(compiled.emitters[0].color_gradient_lut != NO_LUT, "colour gradient baked");
    check(!compiled.curve_luts.empty() && !compiled.gradient_luts.empty(), "LUT atlases populated");

    // Spawn/lifetime: the pool fills, stays within budget, and turns over.
    DeterministicEmitterState state = run(compiled, 1234u, 300);
    check(state.alive_count > 0, "the pool has live particles");
    check(state.alive_count <= compiled.emitters[0].capacity, "the pool never exceeds its capacity");
    check(state.spawn_serial > 200u, "many particles spawned over five seconds");

    // Determinism: two independent runs are byte-identical; a different seed diverges.
    const DeterministicEmitterState a = run(compiled, 777u, 240);
    const DeterministicEmitterState b = run(compiled, 777u, 240);
    check(std::memcmp(&a, &b, sizeof(a)) == 0, "same seed reproduces the pool byte-for-byte");
    const DeterministicEmitterState c = run(compiled, 888u, 240);
    check(std::memcmp(&a, &c, sizeof(a)) != 0, "a different seed diverges");

    // Rollback: snapshot mid-run, restore, replay -> byte-identical to the continuous run.
    DeterministicEmitterState live;
    CpuDeterministicBackend::reset(live, 555u);
    DeterministicEmitterState snapshot;
    const Vector3 position{0, 0.5, 0};
    const Quaternion rotation{};
    for (int i = 0; i < 240; ++i)
    {
        if (i == 100)
            snapshot = live;
        CpuDeterministicBackend::step(live, compiled.emitters[0], compiled, DT, position, rotation);
    }
    DeterministicEmitterState replay = snapshot;
    for (int i = 100; i < 240; ++i)
        CpuDeterministicBackend::step(replay, compiled.emitters[0], compiled, DT, position, rotation);
    check(std::memcmp(&replay, &live, sizeof(live)) == 0, "restore + replay reproduces the state");

    if (failures != 0)
    {
        std::printf("particle_demo: %d FAILURE(S)\n", failures);
        return 1;
    }
    std::printf("particle_demo: OK (%u live, %u spawned)\n", state.alive_count, state.spawn_serial);
    return 0;
}
