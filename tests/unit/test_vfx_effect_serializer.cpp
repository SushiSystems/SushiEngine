/**************************************************************************/
/* test_vfx_effect_serializer.cpp                                         */
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

// Unit_EffectSerializer: the `.sushieffect` shape as the only thing standing between an authored
// effect and the next session. Two failure modes are specific to it and invisible everywhere else:
// a module the capture forgets, which is destroyed silently on the next load, and an enumerator
// past the range check the reader applies, which is reverted to the default just as silently. Both
// are tested by enumeration rather than by example, so an enumerator or module added later cannot
// regress them without a red test. Pure host code: the descriptor tree and nlohmann::json.

#include <cstdint>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <SushiEngine/SushiEngine.hpp>

#include "effect_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::VFX;

namespace
{
    /** @brief Captures @p effect, re-parses it through text, and reads it back. */
    ParticleEffect round_trip(const ParticleEffect& effect)
    {
        // Through the serialized text rather than the in-memory json, because a file is what an
        // effect actually survives as and the dump/parse pair is where a type mismatch shows up.
        const nlohmann::json parsed = nlohmann::json::parse(Scene::capture_effect(effect).dump());
        ParticleEffect loaded;
        EXPECT_TRUE(Scene::apply_effect(parsed, loaded));
        return loaded;
    }

    /** @brief An effect of one emitter carrying @p emitter, ready to round-trip. */
    ParticleEffect effect_of(const EmitterDescriptor& emitter)
    {
        ParticleEffect effect;
        effect.name = "Round Trip";
        effect.emitters.push_back(emitter);
        return effect;
    }
}

TEST(Unit_EffectSerializer, ABeamSurvivesTheRoundTripWithEveryFieldIntact)
{
    EmitterDescriptor emitter;
    emitter.render.alignment = RenderAlignment::Beam;
    // Every field set away from its default, so a member the capture drops reads back as the
    // default rather than as the authored value and the assertion below catches it.
    emitter.beam.enabled = true;
    emitter.beam.start = Vector3{-1.5, 2.25, -0.75};
    emitter.beam.end = Vector3{4.5, -3.25, 8.125};
    emitter.beam.width = 0.75f;
    emitter.beam.sag = -1.25f;
    emitter.beam.noise_amplitude = 2.5f;
    emitter.beam.noise_frequency = 12.0f;

    const ParticleEffect loaded = round_trip(effect_of(emitter));
    ASSERT_EQ(loaded.emitters.size(), 1u);
    const EmitterDescriptor& out = loaded.emitters[0];

    EXPECT_EQ(out.render.alignment, RenderAlignment::Beam);
    EXPECT_TRUE(out.beam.enabled);
    EXPECT_DOUBLE_EQ(out.beam.start.x, emitter.beam.start.x);
    EXPECT_DOUBLE_EQ(out.beam.start.y, emitter.beam.start.y);
    EXPECT_DOUBLE_EQ(out.beam.start.z, emitter.beam.start.z);
    EXPECT_DOUBLE_EQ(out.beam.end.x, emitter.beam.end.x);
    EXPECT_DOUBLE_EQ(out.beam.end.y, emitter.beam.end.y);
    EXPECT_DOUBLE_EQ(out.beam.end.z, emitter.beam.end.z);
    EXPECT_FLOAT_EQ(out.beam.width, emitter.beam.width);
    EXPECT_FLOAT_EQ(out.beam.sag, emitter.beam.sag);
    EXPECT_FLOAT_EQ(out.beam.noise_amplitude, emitter.beam.noise_amplitude);
    EXPECT_FLOAT_EQ(out.beam.noise_frequency, emitter.beam.noise_frequency);

    // And the compiler carries the same span through to the record the backends read.
    const CompiledEffect compiled = EmitterCompiler::compile(loaded);
    ASSERT_EQ(compiled.emitters.size(), 1u);
    const CompiledEmitter& baked = compiled.emitters[0];
    EXPECT_EQ(baked.alignment, RenderAlignment::Beam);
    EXPECT_FLOAT_EQ(baked.beam_start[0], static_cast<float>(emitter.beam.start.x));
    EXPECT_FLOAT_EQ(baked.beam_end[2], static_cast<float>(emitter.beam.end.z));
    EXPECT_FLOAT_EQ(baked.beam_width, emitter.beam.width);
    EXPECT_FLOAT_EQ(baked.beam_sag, emitter.beam.sag);
    EXPECT_FLOAT_EQ(baked.beam_noise_amplitude, emitter.beam.noise_amplitude);
    EXPECT_FLOAT_EQ(baked.beam_noise_frequency, emitter.beam.noise_frequency);
}

TEST(Unit_EffectSerializer, EveryEnumeratorSurvivesTheRoundTrip)
{
    // By enumeration rather than by example: the reader range-checks each of these against a
    // count, and a count one short reverts the last enumerator to the default without failing.
    for (std::uint32_t value = 0; value < RENDER_ALIGNMENT_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        emitter.render.alignment = static_cast<RenderAlignment>(value);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].render.alignment), value)
            << "RenderAlignment enumerator " << value << " did not survive";
    }

    for (std::uint32_t value = 0; value < SORT_MODE_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        emitter.render.sort = static_cast<SortMode>(value);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].render.sort), value)
            << "SortMode enumerator " << value << " did not survive";
        // And the compiled record the render backend reads carries the same mode.
        const CompiledEffect compiled = EmitterCompiler::compile(loaded);
        ASSERT_EQ(compiled.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(compiled.emitters[0].sort), value);
    }

    for (std::uint32_t value = 0; value < BLEND_MODE_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        emitter.render.blend = static_cast<BlendMode>(value);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].render.blend), value)
            << "BlendMode enumerator " << value << " did not survive";
    }

    for (std::uint32_t value = 0; value < EMITTER_SHAPE_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        emitter.shape.shape = static_cast<EmitterShape>(value);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].shape.shape), value)
            << "EmitterShape enumerator " << value << " did not survive";
    }

    for (std::uint32_t value = 0; value < SIMULATION_DOMAIN_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        emitter.domain = static_cast<SimulationDomain>(value);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].domain), value)
            << "SimulationDomain enumerator " << value << " did not survive";
    }

    for (std::uint32_t value = 0; value < FORCE_FIELD_KIND_COUNT; ++value)
    {
        EmitterDescriptor emitter;
        ForceFieldModule field;
        field.enabled = true;
        field.kind = static_cast<ForceFieldKind>(value);
        emitter.force_fields.push_back(field);
        const ParticleEffect loaded = round_trip(effect_of(emitter));
        ASSERT_EQ(loaded.emitters.size(), 1u);
        ASSERT_EQ(loaded.emitters[0].force_fields.size(), 1u);
        EXPECT_EQ(static_cast<std::uint32_t>(loaded.emitters[0].force_fields[0].kind), value)
            << "ForceFieldKind enumerator " << value << " did not survive";
    }
}

TEST(Unit_EffectSerializer, AnEnumeratorFromANewerBuildFallsBackToTheDefault)
{
    // The other half of the range check: widening a count must not stop it refusing a value that
    // names no enumerator, which would cast a wild integer into the enum every consumer switches
    // on. One past the last enumerator is the case a newer build actually produces.
    EmitterDescriptor emitter;
    emitter.render.alignment = RenderAlignment::Ribbon;
    nlohmann::json captured = Scene::capture_effect(effect_of(emitter));
    captured["emitters"][0]["render"]["alignment"] = RENDER_ALIGNMENT_COUNT;
    captured["emitters"][0]["render"]["sort"] = SORT_MODE_COUNT;
    captured["emitters"][0]["shape"]["shape"] = EMITTER_SHAPE_COUNT;

    ParticleEffect loaded;
    ASSERT_TRUE(Scene::apply_effect(captured, loaded));
    ASSERT_EQ(loaded.emitters.size(), 1u);
    const EmitterDescriptor defaults;
    EXPECT_EQ(loaded.emitters[0].render.alignment, defaults.render.alignment);
    EXPECT_EQ(loaded.emitters[0].render.sort, defaults.render.sort);
    EXPECT_EQ(loaded.emitters[0].shape.shape, defaults.shape.shape);
}

TEST(Unit_EffectSerializer, TheCollisionSurfaceChoiceSurvivesTheRoundTrip)
{
    // Which surface an emitter collides against is a flag on the collision module, and losing it
    // moves the emitter from the distance field to the depth buffer — the same look until the
    // camera turns away, which is exactly when the two differ.
    EmitterDescriptor emitter;
    emitter.collision.enabled = true;
    emitter.collision.use_distance_field = true;

    const ParticleEffect loaded = round_trip(effect_of(emitter));
    ASSERT_EQ(loaded.emitters.size(), 1u);
    EXPECT_TRUE(loaded.emitters[0].collision.use_distance_field);

    const CompiledEffect compiled = EmitterCompiler::compile(loaded);
    ASSERT_EQ(compiled.emitters.size(), 1u);
    EXPECT_TRUE((compiled.emitters[0].render_flags & RENDER_DISTANCE_FIELD_COLLISION) != 0);
}

TEST(Unit_EffectSerializer, AnEffectIsDepthSortedUnlessItOptsOut)
{
    // The default is the sort rather than no sort, because an unsorted alpha effect composites
    // visibly wrongly and an unset field must not select that. `None` is the opt-out.
    const RenderModule defaults;
    EXPECT_EQ(defaults.sort, SortMode::ViewDistance);

    EmitterDescriptor emitter;
    emitter.render.blend = BlendMode::Additive;
    emitter.render.sort = SortMode::None;
    ParticleEffect effect = effect_of(emitter);
    const CompiledEffect compiled = EmitterCompiler::compile(effect);
    ASSERT_EQ(compiled.emitters.size(), 1u);
    EXPECT_EQ(compiled.emitters[0].sort, SortMode::None);
}
