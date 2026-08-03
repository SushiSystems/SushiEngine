/**************************************************************************/
/* test_particle_determinism.cpp                                          */
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

// Integration_ParticleDeterminism: the deterministic CPU backend's half of the hybrid VFX
// system (docs/design/vfx_particle_system.md §6). It proves the two properties rollback and
// server reconciliation depend on: (1) the same seed and the same fixed-step stream produce a
// byte-identical pool on two independent runs, and (2) a state snapshotted mid-run, then
// restored and replayed, reproduces the continuously-simulated state byte-for-byte. This is the
// particle analogue of Integration_DeterministicReplay; it is the reason the deterministic
// pool is a fixed-size, pointer-free, integer-seeded struct. Pure maths; no runtime needed.

#include <cstring>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/vfx/deterministic_backend.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Vfx;

namespace
{
    constexpr float FIXED_DT = 1.0f / 60.0f;
    constexpr int TOTAL_STEPS = 240;
    constexpr int SNAPSHOT_STEP = 90;

    // A busy emitter that exercises every deterministic code path: a cone shape with speed,
    // gravity, drag, curl-noise turbulence, and both over-life LUTs. If any of these carried
    // hidden nondeterminism (an uninitialised field, a divergent branch), the byte compare fails.
    CompiledEffect make_effect()
    {
        EmitterDescriptor emitter;
        emitter.domain = SimulationDomain::Deterministic;
        emitter.capacity = 512;
        emitter.spawn.rate_per_second = 120.0f;
        emitter.shape.shape = EmitterShape::Cone;
        emitter.shape.radius = 0.4f;
        emitter.shape.cone_angle_radians = 0.5f;
        emitter.init.lifetime_min = 1.2f;
        emitter.init.lifetime_max = 2.4f;
        emitter.init.speed_min = 2.0f;
        emitter.init.speed_max = 5.0f;
        emitter.init.size_min = 0.05f;
        emitter.init.size_max = 0.2f;
        emitter.init.angular_velocity_min = -3.0f;
        emitter.init.angular_velocity_max = 3.0f;
        emitter.gravity.enabled = true;
        emitter.gravity.acceleration = Vector3{0, -3.0, 0};
        emitter.drag.enabled = true;
        emitter.drag.coefficient = 0.3f;
        emitter.turbulence.enabled = true;
        emitter.turbulence.frequency = 0.7f;
        emitter.turbulence.amplitude = 2.5f;
        emitter.spawn.bursts.push_back(ParticleBurst{0.5f, 40});
        emitter.size_over_life.enabled = true;
        emitter.size_over_life.curve.add_key(CurveKey{0.0f, 0.2f, 0.0f, 1.0f});
        emitter.size_over_life.curve.add_key(CurveKey{0.3f, 1.0f, 0.0f, 0.0f});
        emitter.size_over_life.curve.add_key(CurveKey{1.0f, 0.0f, -1.0f, 0.0f});
        emitter.color_over_life.enabled = true;
        emitter.color_over_life.gradient.add_color_key(ColorKey{0.0f, Vector3{1.0, 0.9, 0.4}});
        emitter.color_over_life.gradient.add_color_key(ColorKey{0.5f, Vector3{1.0, 0.3, 0.1}});
        emitter.color_over_life.gradient.add_color_key(ColorKey{1.0f, Vector3{0.1, 0.1, 0.1}});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{0.0f, 0.0f});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{0.1f, 1.0f});
        emitter.color_over_life.gradient.add_alpha_key(AlphaKey{1.0f, 0.0f});

        ParticleEffect effect;
        effect.emitters.push_back(emitter);
        return EmitterCompiler::compile(effect);
    }

    // Advances a fresh pool for @p steps and returns the final state by value.
    DeterministicEmitterState run(const CompiledEffect& effect, std::uint32_t seed, int steps)
    {
        DeterministicEmitterState state;
        CpuDeterministicBackend::reset(state, seed);
        const Vector3 position{2.0, 1.0, -3.0};
        const Quaternion rotation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(0.3));
        for (int i = 0; i < steps; ++i)
            CpuDeterministicBackend::step(state, effect.emitters[0], effect, FIXED_DT, position,
                                          rotation);
        return state;
    }
}

TEST(Integration_ParticleDeterminism, TwoRunsAreByteIdentical)
{
    const CompiledEffect effect = make_effect();
    const DeterministicEmitterState first = run(effect, 20260724u, TOTAL_STEPS);
    const DeterministicEmitterState second = run(effect, 20260724u, TOTAL_STEPS);

    ASSERT_GT(first.alive_count, 0u) << "the run must be non-trivial";
    EXPECT_EQ(first.alive_count, second.alive_count);
    EXPECT_EQ(std::memcmp(&first, &second, sizeof(DeterministicEmitterState)), 0)
        << "identical seed + step stream must reproduce the pool byte-for-byte";

    // A different seed must actually diverge, or byte-identity would be vacuous.
    const DeterministicEmitterState other = run(effect, 99999u, TOTAL_STEPS);
    EXPECT_NE(std::memcmp(&first, &other, sizeof(DeterministicEmitterState)), 0)
        << "a different seed must produce a different pool";
}

TEST(Integration_ParticleDeterminism, SnapshotRestoreReplayReproducesState)
{
    const CompiledEffect effect = make_effect();
    const std::uint32_t seed = 0xC0FFEEu;
    const Vector3 position{2.0, 1.0, -3.0};
    const Quaternion rotation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(0.3));

    // Continuous reference run to TOTAL_STEPS, capturing a snapshot at SNAPSHOT_STEP.
    DeterministicEmitterState live;
    CpuDeterministicBackend::reset(live, seed);
    DeterministicEmitterState snapshot;
    for (int i = 0; i < TOTAL_STEPS; ++i)
    {
        if (i == SNAPSHOT_STEP)
            snapshot = live; // trivially-copyable: a plain byte copy
        CpuDeterministicBackend::step(live, effect.emitters[0], effect, FIXED_DT, position, rotation);
    }

    // Roll back to the snapshot and replay the remaining steps.
    DeterministicEmitterState replay = snapshot;
    for (int i = SNAPSHOT_STEP; i < TOTAL_STEPS; ++i)
        CpuDeterministicBackend::step(replay, effect.emitters[0], effect, FIXED_DT, position, rotation);

    ASSERT_GT(live.alive_count, 0u);
    EXPECT_EQ(std::memcmp(&replay, &live, sizeof(DeterministicEmitterState)), 0)
        << "restore + replay must reproduce the continuously-simulated pool byte-for-byte";
}
