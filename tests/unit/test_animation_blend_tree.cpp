/**************************************************************************/
/* test_animation_blend_tree.cpp                                          */
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

// Unit_AnimationBlendTree: the five blend-node kinds (design §5.2 step 1) resolved out of a
// compiled controller blob, which is the only form the runtime ever sees them in — so this
// covers the authoring-to-blob compile as much as the weight algebra.
//
// The property that matters more than any single sample point is that a resolver's weights
// partition unity: anything else scales the pose up or down, which reads as a character
// shrinking or exploding rather than as a wrong blend. Each kind is checked at its authored
// sample points *and* swept over a grid for that invariant.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_evaluator.hpp>
#include <SushiEngine/animation/animator_step.hpp>
#include <SushiEngine/animation/blend_tree.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    // A world of "marker" clips: each one parks the root at a known z, so a blended pose's z
    // reads back as the weighted average of whichever clips contributed.
    struct Fixture
    {
        AnimationDatabase database;
        AssetId skeleton = INVALID_ASSET;
        AssetId controller = INVALID_ASSET;

        Fixture()
        {
            SkeletonDescription description;
            JointDescription root;
            root.name = "root";
            root.parent = -1;
            JointDescription child;
            child.name = "child";
            child.parent = 0;
            child.bind_translation = Vector3f{0.0f, 1.0f, 0.0f};
            description.joints = {root, child};
            std::vector<std::byte> blob;
            build_skeleton_blob(description, blob);
            skeleton = database.add_skeleton(std::move(blob));
        }

        AssetId marker_clip(float z)
        {
            ClipDescription clip;
            clip.joint_count = 2;
            clip.frame_count = 1;
            clip.sample_rate = 30.0f;
            clip.translations = {Vector3f{0, 0, z}, Vector3f{0, 1, 0}};
            clip.rotations.assign(2, Quaternionf{0, 0, 0, 1});
            clip.scales.assign(2, Vector3f{1, 1, 1});
            std::vector<std::byte> blob;
            build_clip_blob(clip, blob);
            return database.add_clip(std::move(blob));
        }

        // Wraps a tree in a one-state controller, the only shape the resolver is reachable
        // through: the compile flattens the authored node graph into the blob's index-linked
        // arrays, so a compile bug and a resolver bug are both in scope here.
        bool compile(const std::vector<ParameterDescription>& parameters,
                     std::shared_ptr<BlendTreeNodeDescription> tree)
        {
            ControllerDescription description;
            description.parameters = parameters;
            LayerDescription layer;
            layer.name = "base";
            layer.default_state = "Move";
            StateDescription state;
            state.name = "Move";
            state.blend_tree = std::move(tree);
            layer.states = {state};
            description.layers.push_back(layer);
            std::vector<std::byte> blob;
            if (!compile_controller_blob(description, blob))
                return false;
            controller = database.add_controller(std::move(blob));
            return controller != INVALID_ASSET;
        }

        std::uint32_t resolve(const AnimatorParameterBlock& parameters, BlendContribution* out) const
        {
            const ControllerView view = database.controller(controller);
            const StateRecord& state = view.states[0];
            return resolve_blend_tree(view.nodes, view.children, view.pairs,
                                      static_cast<std::uint32_t>(state.blend_tree), parameters, out,
                                      MAX_BLEND_CONTRIBUTIONS);
        }
    };

    std::shared_ptr<BlendTreeNodeDescription> tree_of(BlendTreeType type, const char* x = "",
                                              const char* y = "")
    {
        auto node = std::make_shared<BlendTreeNodeDescription>();
        node->type = type;
        node->parameter_x = x;
        node->parameter_y = y;
        return node;
    }

    // A clip may appear under more than one child, so its weight is the sum over all of them.
    float weight_of(const BlendContribution* contributions, std::uint32_t count, AssetId clip)
    {
        float total = 0.0f;
        for (std::uint32_t i = 0; i < count; ++i)
            if (contributions[i].clip == clip)
                total += contributions[i].weight;
        return total;
    }

    float weight_sum(const BlendContribution* contributions, std::uint32_t count)
    {
        float total = 0.0f;
        for (std::uint32_t i = 0; i < count; ++i)
            total += contributions[i].weight;
        return total;
    }

    // Sweeps a 2D parameter square well past the authored coordinates and asserts the
    // resolver never leaves the unit-partition property, including outside its own hull.
    void expect_unity_over_a_grid(const Fixture& world, float extent)
    {
        BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
        AnimatorParameterBlock parameters;
        for (int ix = -6; ix <= 6; ++ix)
            for (int iy = -6; iy <= 6; ++iy)
            {
                const float x = static_cast<float>(ix) / 6.0f * extent;
                const float y = static_cast<float>(iy) / 6.0f * extent;
                parameters.set_float(0, x);
                parameters.set_float(1, y);
                const std::uint32_t count = world.resolve(parameters, contributions);
                ASSERT_GT(count, 0u) << "no contribution at (" << x << ", " << y << ")";
                ASSERT_LE(count, MAX_BLEND_CONTRIBUTIONS);
                EXPECT_NEAR(weight_sum(contributions, count), 1.0f, 1e-3f)
                    << "weights do not partition unity at (" << x << ", " << y << ")";
                for (std::uint32_t i = 0; i < count; ++i)
                    EXPECT_GE(contributions[i].weight, -1e-6f) << "a negative weight";
            }
    }
}

TEST(Unit_AnimationBlendTree, OneDimensionalSegmentLerpBetweenSortedThresholds)
{
    Fixture world;
    const AssetId idle = world.marker_clip(0.0f);
    const AssetId walk = world.marker_clip(1.0f);
    const AssetId run = world.marker_clip(2.0f);

    auto tree = tree_of(BlendTreeType::Simple1D, "speed");
    tree->children.push_back(BlendChildDescription{idle, nullptr, 0.0f, 0, 0, "", 1});
    tree->children.push_back(BlendChildDescription{walk, nullptr, 1.0f, 0, 0, "", 1});
    tree->children.push_back(BlendChildDescription{run, nullptr, 2.0f, 0, 0, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"speed", ParameterType::Float, 0.0f}}, tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    const auto at = [&](float speed)
    {
        parameters.set_float(0, speed);
        return world.resolve(parameters, contributions);
    };

    // On a threshold exactly, that child owns the blend outright — no leakage into its
    // neighbours, which would soften every authored pose in the tree.
    std::uint32_t count = at(1.0f);
    EXPECT_NEAR(weight_of(contributions, count, walk), 1.0f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.0f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, run), 0.0f, 1e-5f);

    // Between two thresholds, only those two participate, and linearly.
    count = at(0.25f);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.75f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, walk), 0.25f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, run), 0.0f, 1e-5f);

    count = at(1.5f);
    EXPECT_NEAR(weight_of(contributions, count, walk), 0.5f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, run), 0.5f, 1e-5f);

    // Outside the authored range it clamps to the end child rather than extrapolating.
    count = at(-10.0f);
    EXPECT_NEAR(weight_of(contributions, count, idle), 1.0f, 1e-5f);
    count = at(50.0f);
    EXPECT_NEAR(weight_of(contributions, count, run), 1.0f, 1e-5f);

    for (float speed = -1.0f; speed <= 3.0f; speed += 0.05f)
    {
        count = at(speed);
        ASSERT_GT(count, 0u);
        EXPECT_NEAR(weight_sum(contributions, count), 1.0f, 1e-4f) << "at speed " << speed;
    }
}

TEST(Unit_AnimationBlendTree, OneDimensionalTreeWithASingleChildIsThatChild)
{
    // The degenerate authoring case (one clip in a tree) must not divide by a zero segment.
    Fixture world;
    const AssetId only = world.marker_clip(3.0f);
    auto tree = tree_of(BlendTreeType::Simple1D, "speed");
    tree->children.push_back(BlendChildDescription{only, nullptr, 1.0f, 0, 0, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"speed", ParameterType::Float, 0.0f}}, tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    for (float speed : {-5.0f, 1.0f, 5.0f})
    {
        parameters.set_float(0, speed);
        const std::uint32_t count = world.resolve(parameters, contributions);
        ASSERT_EQ(count, 1u);
        EXPECT_NEAR(contributions[0].weight, 1.0f, 1e-5f);
        EXPECT_EQ(contributions[0].clip, only);
    }
}

TEST(Unit_AnimationBlendTree, FreeformCartesianOwnsItsAuthoredPointsAndPartitionsUnity)
{
    Fixture world;
    const AssetId forward = world.marker_clip(1.0f);
    const AssetId right = world.marker_clip(2.0f);
    const AssetId back = world.marker_clip(3.0f);
    const AssetId left = world.marker_clip(4.0f);

    auto tree = tree_of(BlendTreeType::FreeformCartesian2D, "x", "y");
    tree->children.push_back(BlendChildDescription{forward, nullptr, 0, 0.0f, 1.0f, "", 1});
    tree->children.push_back(BlendChildDescription{right, nullptr, 0, 1.0f, 0.0f, "", 1});
    tree->children.push_back(BlendChildDescription{back, nullptr, 0, 0.0f, -1.0f, "", 1});
    tree->children.push_back(BlendChildDescription{left, nullptr, 0, -1.0f, 0.0f, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"x", ParameterType::Float, 0.0f},
                               ParameterDescription{"y", ParameterType::Float, 0.0f}},
                              tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    const auto at = [&](float x, float y)
    {
        parameters.set_float(0, x);
        parameters.set_float(1, y);
        return world.resolve(parameters, contributions);
    };

    // Standing exactly on an authored sample must play that clip alone.
    struct Corner { float x, y; AssetId clip; const char* label; };
    const Corner corners[4] = {{0.0f, 1.0f, forward, "forward"},
                               {1.0f, 0.0f, right, "right"},
                               {0.0f, -1.0f, back, "back"},
                               {-1.0f, 0.0f, left, "left"}};
    for (const Corner& corner : corners)
    {
        const std::uint32_t count = at(corner.x, corner.y);
        EXPECT_NEAR(weight_of(contributions, count, corner.clip), 1.0f, 2e-3f) << corner.label;
    }

    // Halfway between two neighbours, only those two matter and they share the blend.
    std::uint32_t count = at(0.5f, 0.5f);
    EXPECT_NEAR(weight_of(contributions, count, forward) + weight_of(contributions, count, right),
                1.0f, 2e-3f);
    EXPECT_NEAR(weight_of(contributions, count, back), 0.0f, 2e-3f);

    expect_unity_over_a_grid(world, 2.5f);
}

TEST(Unit_AnimationBlendTree, FreeformDirectionalOwnsItsAuthoredPointsAndPartitionsUnity)
{
    Fixture world;
    const AssetId forward = world.marker_clip(1.0f);
    const AssetId right = world.marker_clip(2.0f);
    const AssetId back = world.marker_clip(3.0f);
    const AssetId left = world.marker_clip(4.0f);

    auto tree = tree_of(BlendTreeType::FreeformDirectional2D, "x", "y");
    tree->children.push_back(BlendChildDescription{forward, nullptr, 0, 0.0f, 1.0f, "", 1});
    tree->children.push_back(BlendChildDescription{right, nullptr, 0, 1.0f, 0.0f, "", 1});
    tree->children.push_back(BlendChildDescription{back, nullptr, 0, 0.0f, -1.0f, "", 1});
    tree->children.push_back(BlendChildDescription{left, nullptr, 0, -1.0f, 0.0f, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"x", ParameterType::Float, 0.0f},
                               ParameterDescription{"y", ParameterType::Float, 0.0f}},
                              tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    parameters.set_float(0, 1.0f);
    parameters.set_float(1, 0.0f);
    std::uint32_t count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, right), 1.0f, 2e-3f);

    // The distinguishing property of the directional kind: it interpolates in polar space,
    // so doubling the magnitude along an authored direction keeps that clip dominant rather
    // than dragging in the perpendicular neighbours.
    parameters.set_float(0, 2.0f);
    parameters.set_float(1, 0.0f);
    count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, right), 1.0f, 2e-3f);
    EXPECT_NEAR(weight_of(contributions, count, forward), 0.0f, 2e-3f);
    EXPECT_NEAR(weight_of(contributions, count, back), 0.0f, 2e-3f);

    expect_unity_over_a_grid(world, 2.5f);
}

TEST(Unit_AnimationBlendTree, SimpleDirectionalSeparatesTheCentreFromTheRing)
{
    Fixture world;
    const AssetId centre = world.marker_clip(0.0f);
    const AssetId forward = world.marker_clip(1.0f);
    const AssetId right = world.marker_clip(2.0f);
    const AssetId left = world.marker_clip(3.0f);

    auto tree = tree_of(BlendTreeType::SimpleDirectional2D, "x", "y");
    tree->children.push_back(BlendChildDescription{centre, nullptr, 0, 0.0f, 0.0f, "", 1});
    tree->children.push_back(BlendChildDescription{forward, nullptr, 0, 0.0f, 1.0f, "", 1});
    tree->children.push_back(BlendChildDescription{right, nullptr, 0, 1.0f, 0.0f, "", 1});
    tree->children.push_back(BlendChildDescription{left, nullptr, 0, -1.0f, 0.0f, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"x", ParameterType::Float, 0.0f},
                               ParameterDescription{"y", ParameterType::Float, 0.0f}},
                              tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    const auto at = [&](float x, float y)
    {
        parameters.set_float(0, x);
        parameters.set_float(1, y);
        return world.resolve(parameters, contributions);
    };

    // At the origin the centre child owns everything — the idle-in-the-middle authoring
    // this kind exists for.
    std::uint32_t count = at(0.0f, 0.0f);
    EXPECT_NEAR(weight_of(contributions, count, centre), 1.0f, 1e-3f);

    // On the ring the centre drops out entirely.
    count = at(1.0f, 0.0f);
    EXPECT_NEAR(weight_of(contributions, count, right), 1.0f, 2e-3f);
    EXPECT_NEAR(weight_of(contributions, count, centre), 0.0f, 2e-3f);

    // Halfway out, the centre and the ring share it — the radial term, not an angular one.
    count = at(0.5f, 0.0f);
    EXPECT_GT(weight_of(contributions, count, centre), 0.05f);
    EXPECT_GT(weight_of(contributions, count, right), 0.05f);

    // Between two ring directions the angular sectors split it between those two only.
    count = at(0.7071f, 0.7071f);
    EXPECT_NEAR(weight_of(contributions, count, right) + weight_of(contributions, count, forward),
                1.0f, 5e-3f);
    EXPECT_NEAR(weight_of(contributions, count, left), 0.0f, 5e-3f);

    expect_unity_over_a_grid(world, 2.5f);
}

TEST(Unit_AnimationBlendTree, DirectModeTakesOneParameterPerChild)
{
    Fixture world;
    const AssetId idle = world.marker_clip(0.0f);
    const AssetId walk = world.marker_clip(1.0f);

    auto tree = tree_of(BlendTreeType::Direct);
    tree->normalize = true;
    tree->children = {BlendChildDescription{idle, nullptr, 0, 0, 0, "w_idle", 1},
                      BlendChildDescription{walk, nullptr, 0, 0, 0, "w_walk", 1}};
    ASSERT_TRUE(world.compile({ParameterDescription{"w_idle", ParameterType::Float, 0.0f},
                               ParameterDescription{"w_walk", ParameterType::Float, 0.0f}},
                              tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;

    // Normalized: the parameters are ratios, so 3:1 is 0.75/0.25 whatever their scale.
    parameters.set_float(0, 3.0f);
    parameters.set_float(1, 1.0f);
    std::uint32_t count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.75f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, walk), 0.25f, 1e-5f);

    parameters.set_float(0, 30.0f);
    parameters.set_float(1, 10.0f);
    count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.75f, 1e-5f) << "scale-invariant";

    // All-zero parameters must not divide by zero or produce NaN weights — the state a
    // caller leaves the block in before it has written anything.
    parameters.set_float(0, 0.0f);
    parameters.set_float(1, 0.0f);
    count = world.resolve(parameters, contributions);
    for (std::uint32_t i = 0; i < count; ++i)
        EXPECT_FALSE(std::isnan(contributions[i].weight)) << "NaN weight at all-zero parameters";
}

TEST(Unit_AnimationBlendTree, NestedTreesScaleTheirChildrenByTheParentWeight)
{
    // A sub-tree's internal weights must be multiplied by the weight its parent gave it, or
    // a nested branch at 50% still poses at full strength.
    Fixture world;
    const AssetId idle = world.marker_clip(0.0f);
    const AssetId walk = world.marker_clip(1.0f);
    const AssetId run = world.marker_clip(2.0f);

    auto sub = tree_of(BlendTreeType::Direct);
    sub->normalize = true;
    sub->children = {BlendChildDescription{walk, nullptr, 0, 0, 0, "w_walk", 1},
                     BlendChildDescription{run, nullptr, 0, 0, 0, "w_run", 1}};
    auto tree = tree_of(BlendTreeType::Simple1D, "speed");
    tree->children = {BlendChildDescription{idle, nullptr, 0.0f, 0, 0, "", 1},
                      BlendChildDescription{INVALID_ASSET, sub, 1.0f, 0, 0, "", 1}};
    ASSERT_TRUE(world.compile({ParameterDescription{"speed", ParameterType::Float, 0.0f},
                               ParameterDescription{"w_walk", ParameterType::Float, 0.0f},
                               ParameterDescription{"w_run", ParameterType::Float, 0.0f}},
                              tree));

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
    AnimatorParameterBlock parameters;
    parameters.set_float(1, 1.0f);
    parameters.set_float(2, 1.0f);

    // Fully inside the sub-tree: its two children split it evenly and idle is excluded.
    parameters.set_float(0, 1.0f);
    std::uint32_t count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, walk), 0.5f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, run), 0.5f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.0f, 1e-5f);

    // Halfway to the sub-tree: it contributes half, split evenly inside — 0.25 each.
    parameters.set_float(0, 0.5f);
    count = world.resolve(parameters, contributions);
    EXPECT_NEAR(weight_of(contributions, count, idle), 0.5f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, walk), 0.25f, 1e-5f);
    EXPECT_NEAR(weight_of(contributions, count, run), 0.25f, 1e-5f);
    EXPECT_NEAR(weight_sum(contributions, count), 1.0f, 1e-5f);
}

TEST(Unit_AnimationBlendTree, ResolutionNeverWritesPastTheCallersCapacity)
{
    // The resolver writes into a fixed array (design §4.5: no allocation inside a tick), so a
    // tree with more children than the caller's capacity must truncate rather than overrun.
    Fixture world;
    auto tree = tree_of(BlendTreeType::Direct);
    tree->normalize = true;
    std::vector<ParameterDescription> parameters;
    for (int i = 0; i < 16; ++i)
    {
        const std::string name = "w" + std::to_string(i);
        parameters.push_back(ParameterDescription{name, ParameterType::Float, 1.0f});
        tree->children.push_back(BlendChildDescription{
            world.marker_clip(static_cast<float>(i)), nullptr, 0, 0, 0, name, 1});
    }
    ASSERT_TRUE(world.compile(parameters, tree));

    const ControllerView view = world.database.controller(world.controller);
    const StateRecord& state = view.states[0];
    AnimatorParameterBlock block;
    for (std::uint32_t i = 0; i < 16; ++i)
        block.set_float(i, 1.0f);

    // Deliberately smaller than the tree, with a sentinel past the end.
    constexpr std::uint32_t CAPACITY = 4;
    BlendContribution contributions[CAPACITY + 1];
    contributions[CAPACITY].clip = 0xABCDEF01u;
    const std::uint32_t count =
        resolve_blend_tree(view.nodes, view.children, view.pairs,
                           static_cast<std::uint32_t>(state.blend_tree), block, contributions,
                           CAPACITY);
    EXPECT_LE(count, CAPACITY);
    EXPECT_EQ(contributions[CAPACITY].clip, 0xABCDEF01u) << "the resolver wrote past capacity";
}

TEST(Unit_AnimationBlendTree, TheEvaluatorPosesAStateByItsResolvedWeights)
{
    // End to end: the weights only matter because the evaluator folds them into a pose. Each
    // marker clip parks the root at its own z, so the composed root z *is* the weighted mean.
    Fixture world;
    const AssetId idle = world.marker_clip(0.0f);
    const AssetId walk = world.marker_clip(1.0f);
    const AssetId run = world.marker_clip(2.0f);

    auto tree = tree_of(BlendTreeType::Simple1D, "speed");
    tree->children.push_back(BlendChildDescription{idle, nullptr, 0.0f, 0, 0, "", 1});
    tree->children.push_back(BlendChildDescription{walk, nullptr, 1.0f, 0, 0, "", 1});
    tree->children.push_back(BlendChildDescription{run, nullptr, 2.0f, 0, 0, "", 1});
    ASSERT_TRUE(world.compile({ParameterDescription{"speed", ParameterType::Float, 0.0f}}, tree));

    const ControllerView controller = world.database.controller(world.controller);
    AnimatorInstance animator{};
    animator.controller = world.controller;
    animator.skeleton = world.skeleton;
    animator_step(controller, world.database, animator, 0.0f); // seeds the layer state

    AnimatorEvaluator evaluator;
    const SkeletonView skeleton = world.database.skeleton(world.skeleton);
    const auto root_z = [&](float speed)
    {
        animator.parameters.set_float(0, speed);
        evaluator.evaluate(controller, world.database, animator, skeleton);
        return static_cast<float>(evaluator.model()[0].m[14]);
    };

    EXPECT_NEAR(root_z(0.0f), 0.0f, 1e-4f);
    EXPECT_NEAR(root_z(1.5f), 1.5f, 1e-4f);
    EXPECT_NEAR(root_z(2.0f), 2.0f, 1e-4f);
    // Monotone in the parameter, which a sign or index error in the segment lookup breaks.
    EXPECT_LT(root_z(0.25f), root_z(0.75f));
}
