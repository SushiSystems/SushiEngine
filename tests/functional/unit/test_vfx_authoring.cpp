/**************************************************************************/
/* test_vfx_authoring.cpp                                                 */
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

// Unit_ParticleCurve / Unit_ParticleGradient / Unit_EmitterCompiler / Unit_ParticleSpawn:
// the VFX authoring model in isolation (docs/design/vfx_particle_system.md §4). These prove the
// artist-facing types evaluate and bake correctly, that the compiler flattens intent into the
// POD boundary the backends consume (domain capacity clamps, update-flag bitfields, baked LUT
// offsets), and that the deterministic backend's spawn accounting is exact — the properties the
// GPU and CPU simulators both rely on. Pure header-only maths; no runtime needed.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/vfx/deterministic_backend.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Vfx;

namespace
{
    EmitterDescriptor make_basic_emitter()
    {
        EmitterDescriptor emitter;
        emitter.domain = SimulationDomain::Deterministic;
        emitter.capacity = 256;
        emitter.spawn.rate_per_second = 0.0f;
        emitter.shape.shape = EmitterShape::Point;
        emitter.init.lifetime_min = 1.0f;
        emitter.init.lifetime_max = 1.0f;
        emitter.init.speed_min = 0.0f;
        emitter.init.speed_max = 0.0f;
        return emitter;
    }
}

TEST(Unit_ParticleCurve, ConstantAndRampEvaluation)
{
    AnimationCurve constant(0.7f);
    EXPECT_FLOAT_EQ(constant.evaluate(0.0f), 0.7f);
    EXPECT_FLOAT_EQ(constant.evaluate(0.5f), 0.7f);
    EXPECT_FLOAT_EQ(constant.evaluate(1.0f), 0.7f);

    AnimationCurve ramp;
    ramp.add_key(CurveKey{0.0f, 0.0f, 0.0f, 1.0f});
    ramp.add_key(CurveKey{1.0f, 1.0f, 1.0f, 0.0f});
    EXPECT_FLOAT_EQ(ramp.evaluate(0.0f), 0.0f);
    EXPECT_FLOAT_EQ(ramp.evaluate(1.0f), 1.0f);
    // Monotonic increase across the span, and clamped outside [0, 1].
    EXPECT_GT(ramp.evaluate(0.75f), ramp.evaluate(0.25f));
    EXPECT_FLOAT_EQ(ramp.evaluate(-1.0f), 0.0f);
    EXPECT_FLOAT_EQ(ramp.evaluate(2.0f), 1.0f);
}

TEST(Unit_ParticleCurve, BakeSamplesEndpoints)
{
    AnimationCurve ramp;
    ramp.add_key(CurveKey{0.0f, 0.0f, 0.0f, 0.0f});
    ramp.add_key(CurveKey{1.0f, 4.0f, 0.0f, 0.0f});
    float lut[CURVE_LUT_WIDTH];
    ramp.bake(lut, CURVE_LUT_WIDTH, 1.0f);
    EXPECT_FLOAT_EQ(lut[0], 0.0f);
    EXPECT_FLOAT_EQ(lut[CURVE_LUT_WIDTH - 1], 4.0f);
    EXPECT_FLOAT_EQ(sample_curve_lut(lut, 0.0f, 9.0f), 0.0f);
    EXPECT_NEAR(sample_curve_lut(lut, 1.0f, 9.0f), 4.0f, 1e-4f);
    // An empty curve bakes to the default value everywhere.
    AnimationCurve empty;
    float lut2[CURVE_LUT_WIDTH];
    empty.bake(lut2, CURVE_LUT_WIDTH, 2.5f);
    EXPECT_FLOAT_EQ(lut2[0], 2.5f);
    EXPECT_FLOAT_EQ(lut2[CURVE_LUT_WIDTH - 1], 2.5f);
}

TEST(Unit_ParticleGradient, ColorAndAlphaInterpolation)
{
    ColorGradient gradient;
    gradient.add_color_key(ColorKey{0.0f, Vector3{1, 0, 0}});
    gradient.add_color_key(ColorKey{1.0f, Vector3{0, 0, 1}});
    gradient.add_alpha_key(AlphaKey{0.0f, 1.0f});
    gradient.add_alpha_key(AlphaKey{1.0f, 0.0f});

    const Vector3 mid = gradient.evaluate_color(0.5f);
    EXPECT_NEAR(mid.x, 0.5, 1e-4);
    EXPECT_NEAR(mid.z, 0.5, 1e-4);
    EXPECT_NEAR(gradient.evaluate_alpha(0.5f), 0.5f, 1e-4f);
    EXPECT_FLOAT_EQ(gradient.evaluate_alpha(0.0f), 1.0f);
    EXPECT_FLOAT_EQ(gradient.evaluate_alpha(1.0f), 0.0f);

    float lut[GRADIENT_LUT_WIDTH * 4];
    gradient.bake(lut, GRADIENT_LUT_WIDTH);
    float rgba0[4];
    float rgba1[4];
    sample_gradient_lut(lut, 0.0f, rgba0);
    sample_gradient_lut(lut, 1.0f, rgba1);
    EXPECT_NEAR(rgba0[0], 1.0f, 1e-3f); // red at start
    EXPECT_NEAR(rgba0[3], 1.0f, 1e-3f); // opaque at start
    EXPECT_NEAR(rgba1[2], 1.0f, 1e-3f); // blue at end
    EXPECT_NEAR(rgba1[3], 0.0f, 1e-3f); // transparent at end
}

TEST(Unit_EmitterCompiler, DomainCapacityClampAndFlags)
{
    ParticleEffect effect;

    EmitterDescriptor cosmetic = make_basic_emitter();
    cosmetic.domain = SimulationDomain::Cosmetic;
    cosmetic.capacity = 100000;
    effect.emitters.push_back(cosmetic);

    EmitterDescriptor deterministic = make_basic_emitter();
    deterministic.domain = SimulationDomain::Deterministic;
    deterministic.capacity = 100000; // must clamp to the deterministic budget
    deterministic.gravity.enabled = true;
    deterministic.drag.enabled = true;
    deterministic.size_over_life.enabled = true;
    deterministic.size_over_life.curve.add_key(CurveKey{0.0f, 1.0f, 0.0f, 0.0f});
    deterministic.size_over_life.curve.add_key(CurveKey{1.0f, 0.0f, 0.0f, 0.0f});
    deterministic.color_over_life.enabled = true;
    deterministic.color_over_life.gradient.add_color_key(ColorKey{0.0f, Vector3{1, 1, 1}});
    effect.emitters.push_back(deterministic);

    const CompiledEffect compiled = EmitterCompiler::compile(effect);
    ASSERT_EQ(compiled.emitters.size(), 2u);
    EXPECT_EQ(compiled.emitters[0].capacity, MAX_EMITTER_CAPACITY);
    EXPECT_EQ(compiled.emitters[1].capacity, MAX_DETERMINISTIC_PARTICLES);

    const CompiledEmitter& det = compiled.emitters[1];
    EXPECT_TRUE((det.update_flags & UPDATE_GRAVITY) != 0);
    EXPECT_TRUE((det.update_flags & UPDATE_DRAG) != 0);
    EXPECT_TRUE((det.update_flags & UPDATE_SIZE_OVER_LIFE) != 0);
    EXPECT_TRUE((det.update_flags & UPDATE_COLOR_OVER_LIFE) != 0);
    EXPECT_NE(det.size_curve_lut, NO_LUT);
    EXPECT_NE(det.color_gradient_lut, NO_LUT);
    EXPECT_NE(compiled.curve_row(det.size_curve_lut), nullptr);
    EXPECT_NE(compiled.gradient_row(det.color_gradient_lut), nullptr);

    // A disabled module leaves its flag clear and its LUT absent.
    EXPECT_TRUE((compiled.emitters[0].update_flags & UPDATE_SIZE_OVER_LIFE) == 0);
    EXPECT_EQ(compiled.emitters[0].size_curve_lut, NO_LUT);
}

TEST(Unit_EmitterCompiler, BurstPackingAndRangeRepair)
{
    EmitterDescriptor emitter = make_basic_emitter();
    emitter.spawn.bursts.push_back(ParticleBurst{0.0f, 10});
    emitter.spawn.bursts.push_back(ParticleBurst{2.0f, 20});
    // Inverted ranges must be repaired so max >= min.
    emitter.init.lifetime_min = 3.0f;
    emitter.init.lifetime_max = 1.0f;

    ParticleEffect effect;
    effect.emitters.push_back(emitter);
    const CompiledEffect compiled = EmitterCompiler::compile(effect);
    const CompiledEmitter& e = compiled.emitters[0];
    EXPECT_EQ(e.burst_count, 2u);
    EXPECT_FLOAT_EQ(e.burst_time[1], 2.0f);
    EXPECT_EQ(e.burst_amount[1], 20u);
    EXPECT_GE(e.lifetime_max, e.lifetime_min);
}

TEST(Unit_ParticleSpawn, RateAndCapacityAccounting)
{
    EmitterDescriptor emitter = make_basic_emitter();
    emitter.capacity = 64;
    emitter.spawn.rate_per_second = 50.0f;
    emitter.init.lifetime_min = 1.0f;
    emitter.init.lifetime_max = 1.0f;

    ParticleEffect effect;
    effect.emitters.push_back(emitter);
    const CompiledEffect compiled = EmitterCompiler::compile(effect);

    DeterministicEmitterState state;
    CpuDeterministicBackend::reset(state, 42);

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) // 10 seconds
    {
        CpuDeterministicBackend::step(state, compiled.emitters[0], compiled, dt,
                                      Vector3{0, 0, 0}, Quaternion{});
        ASSERT_LE(state.alive_count, compiled.emitters[0].capacity)
            << "pool must never exceed its capacity";
    }
    // Steady state: rate 50/s * lifetime 1s ~ 50 live, under the 64 cap.
    EXPECT_GT(state.alive_count, 30u);
    EXPECT_LE(state.alive_count, 64u);
    EXPECT_GT(state.spawn_serial, 400u); // many spawned over 10 s
}

TEST(Unit_ParticleSpawn, NoSpawnWithoutRateOrBurst)
{
    EmitterDescriptor emitter = make_basic_emitter();
    emitter.spawn.rate_per_second = 0.0f;

    ParticleEffect effect;
    effect.emitters.push_back(emitter);
    const CompiledEffect compiled = EmitterCompiler::compile(effect);

    DeterministicEmitterState state;
    CpuDeterministicBackend::reset(state, 7);
    for (int i = 0; i < 120; ++i)
        CpuDeterministicBackend::step(state, compiled.emitters[0], compiled, 1.0f / 60.0f,
                                      Vector3{0, 0, 0}, Quaternion{});
    EXPECT_EQ(state.alive_count, 0u);
    EXPECT_EQ(state.spawn_serial, 0u);
}
