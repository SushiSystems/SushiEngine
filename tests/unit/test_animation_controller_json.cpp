/**************************************************************************/
/* test_animation_controller_json.cpp                                     */
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

// The Animator's persistence seam: `ControllerDesc` to JSON and back. This is what the editor's
// save/load rides on, and what an undo step *is* — a serialized snapshot restored — so a field
// this layer forgets is a field that silently reverts when the user presses Ctrl+Z.
//
// The strongest assertion available is used deliberately: the desc is compiled to a `.sushictrl`
// blob before *and* after the round trip and the two blobs are compared byte for byte. A
// field-by-field comparison of two descs would have to be written by hand, which means a field
// added to the desc and forgotten in the serializer would also be forgotten in the comparison —
// the test would grow the same blind spot as the code it guards. The blob has no such blind spot:
// if a dropped field changes what the runtime loads, the bytes differ.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_controller_json.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    /**
     * @brief A controller exercising every field the serializer can carry.
     *
     * Every value is deliberately *not* the field's default: a round trip that drops a field
     * still reproduces the default, so a desc built from defaults would pass against a
     * serializer that wrote nothing at all.
     */
    ControllerDesc maximal_controller()
    {
        ControllerDesc desc;

        // One parameter of each type, so `parameter_type_name`/`_from` is exercised whole.
        desc.parameters.push_back({"speed", ParameterType::Float, 2.5f});
        desc.parameters.push_back({"variant", ParameterType::Int, 3.0f});
        desc.parameters.push_back({"moving", ParameterType::Bool, 1.0f});
        desc.parameters.push_back({"jump", ParameterType::Trigger, 0.0f});

        LayerDesc base;
        base.name = "Base";
        base.weight = 1.0f;
        base.mask = INVALID_ASSET;
        base.blend_mode = LayerBlendMode::Override;
        base.default_state = "Walk";

        StateDesc idle;
        idle.name = "Idle";
        idle.clip = 7;
        idle.speed = 0.75f;
        idle.speed_parameter = "speed";
        idle.cycle_offset = 0.125f;
        idle.events.push_back({0.5f, "footstep", 11});
        idle.events.push_back({0.9f, "breath", -4});

        // One transition per comparator, so a mislabelled enum name cannot hide behind the
        // others. `If`/`IfNot` in particular read as the same word in a diff.
        const Comparator comparators[] = {Comparator::Greater, Comparator::Less,
                                          Comparator::Equals,  Comparator::NotEquals,
                                          Comparator::If,      Comparator::IfNot};
        for (std::size_t i = 0; i < sizeof(comparators) / sizeof(comparators[0]); ++i)
        {
            TransitionDesc transition;
            transition.destination = "Walk";
            transition.has_exit_time = (i % 2) == 0;
            transition.exit_time = 0.8f + 0.01f * float(i);
            transition.duration = 0.2f + 0.01f * float(i);
            transition.offset = 0.05f * float(i);
            transition.interruption = i == 0   ? InterruptionSource::None
                                      : i == 1 ? InterruptionSource::CurrentState
                                               : InterruptionSource::NextState;
            transition.conditions.push_back({"speed", comparators[i], 1.5f + float(i)});
            transition.conditions.push_back({"variant", Comparator::Equals, float(i)});
            idle.transitions.push_back(transition);
        }

        // A state driven by a nested blend tree rather than a clip, covering every tree kind
        // through the nesting: a 2D parent whose children are themselves trees.
        StateDesc walk;
        walk.name = "Walk";
        walk.speed = 1.5f;
        walk.cycle_offset = 0.25f;

        auto inner_1d = std::make_shared<BlendTreeNodeDesc>();
        inner_1d->type = BlendTreeType::Simple1D;
        inner_1d->parameter_x = "speed";
        inner_1d->children.push_back({1, nullptr, 0.0f, 0.0f, 0.0f, "", 1.0f});
        inner_1d->children.push_back({2, nullptr, 4.5f, 0.0f, 0.0f, "", 0.5f});

        auto inner_direct = std::make_shared<BlendTreeNodeDesc>();
        inner_direct->type = BlendTreeType::Direct;
        inner_direct->normalize = false;
        inner_direct->children.push_back({3, nullptr, 0.0f, 0.0f, 0.0f, "speed", 2.0f});

        auto root = std::make_shared<BlendTreeNodeDesc>();
        root->type = BlendTreeType::FreeformCartesian2D;
        root->parameter_x = "speed";
        root->parameter_y = "variant";
        root->normalize = true;
        BlendChildDesc nested_a;
        nested_a.child_node = inner_1d;
        nested_a.position_x = -1.0f;
        nested_a.position_y = 0.5f;
        nested_a.speed = 1.25f;
        BlendChildDesc nested_b;
        nested_b.child_node = inner_direct;
        nested_b.position_x = 1.0f;
        nested_b.position_y = -0.5f;
        root->children.push_back(nested_a);
        root->children.push_back(nested_b);
        walk.blend_tree = root;

        base.states.push_back(idle);
        base.states.push_back(walk);

        TransitionDesc any;
        any.destination = "Idle";
        any.duration = 0.1f;
        any.interruption = InterruptionSource::NextState;
        any.conditions.push_back({"jump", Comparator::If, 0.0f});
        base.any_state_transitions.push_back(any);

        // A second layer, additive and masked and weight-driven, so the layer fields that only
        // a non-base layer ever exercises are covered too.
        LayerDesc upper;
        upper.name = "UpperBody";
        upper.weight = 0.4f;
        upper.mask = 42;
        upper.blend_mode = LayerBlendMode::Additive;
        upper.weight_parameter = "speed";
        upper.default_state = "Aim";
        StateDesc aim;
        aim.name = "Aim";
        aim.clip = 9;
        aim.speed = 1.0f;
        upper.states.push_back(aim);

        desc.layers.push_back(base);
        desc.layers.push_back(upper);
        return desc;
    }

    /** @brief The compiled `.sushictrl` bytes for @p desc, empty when the compile refuses. */
    std::vector<std::byte> compile(const ControllerDesc& desc)
    {
        std::vector<std::byte> blob;
        if (!compile_controller_blob(desc, blob))
            blob.clear();
        return blob;
    }
} // namespace

TEST(Unit_AnimationControllerJson,TheCompiledAssetIsByteIdenticalAcrossARoundTrip)
{
    const ControllerDesc authored = maximal_controller();
    const std::vector<std::byte> before = compile(authored);
    ASSERT_FALSE(before.empty()) << "the maximal controller must compile, or the rest is moot";

    const ControllerDesc restored = controller_from_json(controller_to_json(authored));
    const std::vector<std::byte> after = compile(restored);

    ASSERT_EQ(after.size(), before.size());
    EXPECT_EQ(std::memcmp(after.data(), before.data(), before.size()), 0);
}

TEST(Unit_AnimationControllerJson,TheRoundTripSurvivesSerializationToTextAndBack)
{
    // The editor writes a file, not a `nlohmann::json` object. Passing the object straight back
    // would skip the dump/parse pair, and that is where a float written at too few digits stops
    // reading back to the same value — the failure mode is a blend threshold that drifts a
    // little every time the file is saved.
    const ControllerDesc authored = maximal_controller();
    const std::vector<std::byte> before = compile(authored);
    ASSERT_FALSE(before.empty());

    const std::string text = controller_to_json(authored).dump();
    const ControllerDesc restored = controller_from_json(nlohmann::json::parse(text));
    const std::vector<std::byte> after = compile(restored);

    ASSERT_EQ(after.size(), before.size());
    EXPECT_EQ(std::memcmp(after.data(), before.data(), before.size()), 0);

    // And the document is stable: serializing what was read back gives the same text, which is
    // what keeps a save with no edits out of version control's diff.
    EXPECT_EQ(controller_to_json(restored).dump(), text);
}

TEST(Unit_AnimationControllerJson,EveryEnumValueSurvivesItsNameRoundTrip)
{
    // Enum names are written by hand in two switch statements that have to agree, which is
    // exactly the shape a copy-paste error hides in. Walking every value is cheap and is the
    // only way to catch the one that was pasted twice.
    const ParameterType parameter_types[] = {ParameterType::Float, ParameterType::Int,
                                             ParameterType::Bool, ParameterType::Trigger};
    for (const ParameterType type : parameter_types)
        EXPECT_EQ(detail::parameter_type_from(detail::parameter_type_name(type)), type);

    const Comparator comparators[] = {Comparator::Greater, Comparator::Less,
                                      Comparator::Equals,  Comparator::NotEquals,
                                      Comparator::If,      Comparator::IfNot};
    for (const Comparator comparator : comparators)
        EXPECT_EQ(detail::comparator_from(detail::comparator_name(comparator)), comparator);

    const InterruptionSource sources[] = {InterruptionSource::None,
                                          InterruptionSource::CurrentState,
                                          InterruptionSource::NextState};
    for (const InterruptionSource source : sources)
        EXPECT_EQ(detail::interruption_from(detail::interruption_name(source)), source);

    const LayerBlendMode modes[] = {LayerBlendMode::Override, LayerBlendMode::Additive};
    for (const LayerBlendMode mode : modes)
        EXPECT_EQ(detail::blend_mode_from(detail::blend_mode_name(mode)), mode);

    const BlendTreeType tree_types[] = {
        BlendTreeType::Simple1D, BlendTreeType::SimpleDirectional2D,
        BlendTreeType::FreeformDirectional2D, BlendTreeType::FreeformCartesian2D,
        BlendTreeType::Direct};
    for (const BlendTreeType type : tree_types)
        EXPECT_EQ(detail::blend_tree_type_from(detail::blend_tree_type_name(type)), type);

    // Every name is distinct, which is the other half: two values sharing a name round-trip
    // one of them wrongly and the loop above would still pass for the survivor.
    std::vector<std::string> names;
    for (const BlendTreeType type : tree_types)
        names.push_back(detail::blend_tree_type_name(type));
    for (std::size_t i = 0; i < names.size(); ++i)
        for (std::size_t j = i + 1; j < names.size(); ++j)
            EXPECT_NE(names[i], names[j]);
}

TEST(Unit_AnimationControllerJson,AnUnknownEnumNameDegradesToTheDocumentedDefault)
{
    // A document from a newer editor may name a value this build has never heard of. Falling
    // back to the documented default keeps the rest of the controller readable, where throwing
    // would lose a whole authored graph over one field.
    EXPECT_EQ(detail::parameter_type_from("Quaternion"), ParameterType::Float);
    EXPECT_EQ(detail::comparator_from("Approximately"), Comparator::Greater);
    EXPECT_EQ(detail::interruption_from("Anything"), InterruptionSource::None);
    EXPECT_EQ(detail::blend_mode_from("Multiply"), LayerBlendMode::Override);
    EXPECT_EQ(detail::blend_tree_type_from("Simple3D"), BlendTreeType::Simple1D);

    // The names are matched exactly, so a case difference is an unknown name rather than a
    // near-miss silently accepted.
    EXPECT_EQ(detail::comparator_from("less"), Comparator::Greater);
}

TEST(Unit_AnimationControllerJson,AMissingFieldReadsAsItsDefaultRatherThanThrowing)
{
    // The tolerant read the header promises. A hand-edited or older document is the case this
    // exists for, and the assertion is that the *documented* default is what appears — not
    // whatever a value-initialized struct happens to hold, which for `speed` and `exit_time`
    // is a different number.
    const nlohmann::json sparse = nlohmann::json::parse(R"({
        "parameters": [{"name": "speed"}],
        "layers": [{
            "name": "Base",
            "states": [{
                "name": "Idle",
                "transitions": [{"destination": "Walk"}]
            }]
        }]
    })");

    const ControllerDesc desc = controller_from_json(sparse);
    ASSERT_EQ(desc.parameters.size(), 1u);
    EXPECT_EQ(desc.parameters[0].type, ParameterType::Float);
    EXPECT_FLOAT_EQ(desc.parameters[0].default_value, 0.0f);

    ASSERT_EQ(desc.layers.size(), 1u);
    EXPECT_FLOAT_EQ(desc.layers[0].weight, 1.0f);
    EXPECT_EQ(desc.layers[0].mask, INVALID_ASSET);
    EXPECT_EQ(desc.layers[0].blend_mode, LayerBlendMode::Override);
    EXPECT_TRUE(desc.layers[0].default_state.empty());

    ASSERT_EQ(desc.layers[0].states.size(), 1u);
    const StateDesc& state = desc.layers[0].states[0];
    EXPECT_EQ(state.clip, INVALID_ASSET);
    EXPECT_FLOAT_EQ(state.speed, 1.0f);
    EXPECT_FLOAT_EQ(state.cycle_offset, 0.0f);
    EXPECT_EQ(state.blend_tree, nullptr);

    ASSERT_EQ(state.transitions.size(), 1u);
    const TransitionDesc& transition = state.transitions[0];
    EXPECT_FALSE(transition.has_exit_time);
    EXPECT_FLOAT_EQ(transition.exit_time, 1.0f);
    EXPECT_FLOAT_EQ(transition.duration, 0.0f);
    EXPECT_TRUE(transition.conditions.empty());
}

TEST(Unit_AnimationControllerJson,AnEmptyDocumentReadsAsAnEmptyControllerAndNotAsAnError)
{
    // Two shapes a real project produces: a brand-new controller with nothing authored yet, and
    // a document whose arrays are present but empty. Neither is an error.
    const ControllerDesc from_empty_object = controller_from_json(nlohmann::json::object());
    EXPECT_TRUE(from_empty_object.parameters.empty());
    EXPECT_TRUE(from_empty_object.layers.empty());

    const ControllerDesc from_empty_arrays =
        controller_from_json(nlohmann::json::parse(R"({"parameters": [], "layers": []})"));
    EXPECT_TRUE(from_empty_arrays.layers.empty());

    // And it survives the trip back out, so a fresh controller saves and reloads.
    EXPECT_EQ(controller_from_json(controller_to_json(from_empty_object)).layers.size(), 0u);
}

TEST(Unit_AnimationControllerJson,AssetReferencesSerializeAsIdsWithMinusOneForNone)
{
    // The header pins this contract explicitly — ids, with -1 for none — because a project layer
    // above maps ids to paths and would otherwise have to guess what an absent reference is.
    ControllerDesc desc;
    LayerDesc layer;
    layer.name = "Base";
    layer.mask = INVALID_ASSET;
    StateDesc with_clip;
    with_clip.name = "Idle";
    with_clip.clip = 5;
    StateDesc without_clip;
    without_clip.name = "Empty";
    without_clip.clip = INVALID_ASSET;
    layer.states.push_back(with_clip);
    layer.states.push_back(without_clip);
    desc.layers.push_back(layer);

    const nlohmann::json json = controller_to_json(desc);
    EXPECT_EQ(json.at("layers").at(0).at("mask").get<std::int64_t>(), -1);
    EXPECT_EQ(json.at("layers").at(0).at("states").at(0).at("clip").get<std::int64_t>(), 5);
    EXPECT_EQ(json.at("layers").at(0).at("states").at(1).at("clip").get<std::int64_t>(), -1);

    const ControllerDesc restored = controller_from_json(json);
    EXPECT_EQ(restored.layers[0].mask, INVALID_ASSET);
    EXPECT_EQ(restored.layers[0].states[0].clip, AssetId(5));
    EXPECT_EQ(restored.layers[0].states[1].clip, INVALID_ASSET);

    // A null or absent reference is the same answer as -1: a document written by hand is the
    // normal source of the first two.
    EXPECT_EQ(detail::asset_from_json(nlohmann::json()), INVALID_ASSET);
    EXPECT_EQ(detail::asset_from_json(nlohmann::json("not a number")), INVALID_ASSET);
}

TEST(Unit_AnimationControllerJson,ANestedBlendTreeKeepsItsShapeAndNotJustItsLeaves)
{
    // Nesting is the recursive half of the serializer, and a recursion that flattens produces a
    // tree that still blends — just not the authored one. Asserting the shape, depth included,
    // is what distinguishes the two.
    const ControllerDesc restored = controller_from_json(controller_to_json(maximal_controller()));
    ASSERT_GE(restored.layers.size(), 1u);
    ASSERT_GE(restored.layers[0].states.size(), 2u);

    const std::shared_ptr<BlendTreeNodeDesc>& root = restored.layers[0].states[1].blend_tree;
    ASSERT_NE(root, nullptr);
    EXPECT_EQ(root->type, BlendTreeType::FreeformCartesian2D);
    EXPECT_EQ(root->parameter_x, "speed");
    EXPECT_EQ(root->parameter_y, "variant");
    ASSERT_EQ(root->children.size(), 2u);

    // Each child is a node rather than a leaf, and carries its own coordinates.
    ASSERT_NE(root->children[0].child_node, nullptr);
    EXPECT_EQ(root->children[0].clip, INVALID_ASSET);
    EXPECT_FLOAT_EQ(root->children[0].position_x, -1.0f);
    EXPECT_FLOAT_EQ(root->children[0].speed, 1.25f);

    const BlendTreeNodeDesc& inner = *root->children[0].child_node;
    EXPECT_EQ(inner.type, BlendTreeType::Simple1D);
    ASSERT_EQ(inner.children.size(), 2u);
    EXPECT_EQ(inner.children[1].clip, AssetId(2));
    EXPECT_FLOAT_EQ(inner.children[1].threshold, 4.5f);
    EXPECT_EQ(inner.children[1].child_node, nullptr) << "a leaf must not gain a node";

    // `normalize` defaults to true, so the child that authored it false is the one that proves
    // the field is carried rather than reconstructed from the default.
    const BlendTreeNodeDesc& direct = *root->children[1].child_node;
    EXPECT_EQ(direct.type, BlendTreeType::Direct);
    EXPECT_FALSE(direct.normalize);
    ASSERT_EQ(direct.children.size(), 1u);
    EXPECT_EQ(direct.children[0].parameter, "speed");
}

TEST(Unit_AnimationControllerJson,EventsAndAnyStateTransitionsSurviveWithTheirOrder)
{
    // Both are arrays whose *order* is observable — events fire in sequence and transitions are
    // evaluated first-match — so a serializer that round-trips the set but not the sequence
    // produces a controller that behaves differently after a save.
    const ControllerDesc restored = controller_from_json(controller_to_json(maximal_controller()));
    const StateDesc& idle = restored.layers[0].states[0];

    ASSERT_EQ(idle.events.size(), 2u);
    EXPECT_EQ(idle.events[0].name, "footstep");
    EXPECT_FLOAT_EQ(idle.events[0].normalized_time, 0.5f);
    EXPECT_EQ(idle.events[0].payload, 11);
    EXPECT_EQ(idle.events[1].name, "breath");
    EXPECT_EQ(idle.events[1].payload, -4) << "a negative payload must not be read as unsigned";

    ASSERT_EQ(idle.transitions.size(), 6u);
    EXPECT_EQ(idle.transitions[0].conditions[0].comparator, Comparator::Greater);
    EXPECT_EQ(idle.transitions[4].conditions[0].comparator, Comparator::If);
    EXPECT_EQ(idle.transitions[5].conditions[0].comparator, Comparator::IfNot);
    // Two conditions per transition, and both are kept: an AND of two is not an AND of one.
    EXPECT_EQ(idle.transitions[3].conditions.size(), 2u);

    ASSERT_EQ(restored.layers[0].any_state_transitions.size(), 1u);
    EXPECT_EQ(restored.layers[0].any_state_transitions[0].destination, "Idle");
    EXPECT_EQ(restored.layers[0].any_state_transitions[0].interruption,
              InterruptionSource::NextState);
}

TEST(Unit_AnimationControllerJson,LayerOrderAndPerLayerFieldsSurvive)
{
    // Layer order is fold order — a later layer overrides an earlier one — so it is behaviour,
    // not presentation. And the second layer is where the additive/mask/weight-parameter fields
    // live, which a single-layer round trip never touches.
    const ControllerDesc restored = controller_from_json(controller_to_json(maximal_controller()));
    ASSERT_EQ(restored.layers.size(), 2u);
    EXPECT_EQ(restored.layers[0].name, "Base");
    EXPECT_EQ(restored.layers[1].name, "UpperBody");

    const LayerDesc& upper = restored.layers[1];
    EXPECT_FLOAT_EQ(upper.weight, 0.4f);
    EXPECT_EQ(upper.mask, AssetId(42));
    EXPECT_EQ(upper.blend_mode, LayerBlendMode::Additive);
    EXPECT_EQ(upper.weight_parameter, "speed");
    EXPECT_EQ(upper.default_state, "Aim");

    // And the base layer's own default_state, which names a state that is not the first one —
    // the case where dropping the field would still produce a working controller pointing at
    // the wrong state.
    EXPECT_EQ(restored.layers[0].default_state, "Walk");
}
