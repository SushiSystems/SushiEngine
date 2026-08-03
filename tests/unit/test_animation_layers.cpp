/**************************************************************************/
/* test_animation_layers.cpp                                             */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
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

// Unit_AnimationLayers: layer folding, avatar masks, and additive blending (design §5.2
// step 2) — the mechanism a "shoot while running" or "lean while walking" rig is built out
// of, and the one where a mistake is quiet: a mask that admits a joint it should gate leaves
// the character animating correctly *except* for one limb, which reads as bad content rather
// than as a bug.
//
// Most cases use a flat skeleton (every joint a root) so a joint's model-space translation is
// its local one and the layer arithmetic is read back without the compose in the way; the
// hierarchical cases that follow then pin what masking a parent does to its children.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/additive.hpp>
#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/animator_evaluator.hpp>
#include <SushiEngine/animation/animator_step.hpp>
#include <SushiEngine/animation/avatar_mask.hpp>
#include <SushiEngine/animation/clip_blob.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    const Quaternionf IDENTITY{0.0f, 0.0f, 0.0f, 1.0f};

    struct Fixture
    {
        AnimationDatabase database;
        AssetId skeleton = INVALID_ASSET;
        AssetId controller = INVALID_ASSET;
        std::uint32_t joint_count = 0;
        std::vector<std::string> joint_names;

        // `parents` gives each joint's authored parent; -1 for a root. Passing all roots gives
        // the flat skeleton whose local pose is its model pose.
        void build_skeleton(const std::vector<std::string>& names, const std::vector<int>& parents)
        {
            SkeletonDescription description;
            description.joints.resize(names.size());
            for (std::size_t i = 0; i < names.size(); ++i)
            {
                description.joints[i].name = names[i];
                description.joints[i].parent = parents[i];
            }
            std::vector<std::byte> blob;
            build_skeleton_blob(description, blob);
            skeleton = database.add_skeleton(std::move(blob));
            joint_count = static_cast<std::uint32_t>(names.size());
            joint_names = names;
        }

        // A single-frame clip parking each joint at its given local translation.
        AssetId pose_clip(const std::vector<float>& z_per_joint)
        {
            ClipDescription clip;
            clip.joint_count = joint_count;
            clip.frame_count = 1;
            clip.sample_rate = 30.0f;
            clip.translations.resize(joint_count);
            for (std::uint32_t j = 0; j < joint_count; ++j)
                clip.translations[j] = Vector3f{0.0f, 0.0f, z_per_joint[j]};
            clip.rotations.assign(joint_count, IDENTITY);
            clip.scales.assign(joint_count, Vector3f{1, 1, 1});
            std::vector<std::byte> blob;
            build_clip_blob(clip, blob);
            return database.add_clip(std::move(blob));
        }

        AssetId mask(const std::vector<std::pair<std::string, float>>& entries, float default_weight)
        {
            MaskDescription description;
            description.default_weight = default_weight;
            for (const auto& entry : entries)
                description.entries.push_back(MaskDescription::Entry{entry.first, entry.second});
            std::vector<std::byte> blob;
            if (!build_mask_blob(description, blob))
                return INVALID_ASSET;
            return database.add_mask(std::move(blob));
        }

        // A base layer plus zero or more extra layers, each one clip looping forever.
        struct LayerSpecification
        {
            AssetId clip = INVALID_ASSET;
            float weight = 1.0f;
            AssetId mask = INVALID_ASSET;
            bool additive = false;
        };

        bool compile(const std::vector<LayerSpecification>& layers)
        {
            ControllerDescription description;
            for (std::size_t i = 0; i < layers.size(); ++i)
            {
                LayerDescription layer;
                layer.name = "layer" + std::to_string(i);
                layer.weight = layers[i].weight;
                layer.mask = layers[i].mask;
                layer.blend_mode =
                    layers[i].additive ? LayerBlendMode::Additive : LayerBlendMode::Override;
                StateDescription state;
                state.name = "State";
                state.clip = layers[i].clip;
                layer.states = {state};
                description.layers.push_back(layer);
            }
            std::vector<std::byte> blob;
            if (!compile_controller_blob(description, blob))
                return false;
            controller = database.add_controller(std::move(blob));
            return controller != INVALID_ASSET;
        }

        // Evaluates the whole chain and returns each joint's model-space z, keyed by name so a
        // test never depends on the cook's reordering.
        std::vector<float> evaluate_z()
        {
            const ControllerView view = database.controller(controller);
            AnimatorInstance animator{};
            animator.controller = controller;
            animator.skeleton = skeleton;
            animator_step(view, database, animator, 0.0f);
            const SkeletonView bones = database.skeleton(skeleton);
            AnimatorEvaluator evaluator;
            evaluator.evaluate(view, database, animator, bones);

            std::vector<float> by_authored_index(joint_names.size(), 0.0f);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
                const int index = bones.find_joint(hash_name(joint_names[i].c_str()));
                by_authored_index[i] =
                    index >= 0 ? static_cast<float>(evaluator.model()[index].m[14]) : 0.0f;
            }
            return by_authored_index;
        }
    };

    const std::vector<std::string> FLAT_NAMES = {"a", "b", "c"};
    const std::vector<int> ALL_ROOTS = {-1, -1, -1};
}

TEST(Unit_AnimationLayers, ABaseLayerAloneIsItsOwnClip)
{
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    ASSERT_TRUE(world.compile({{world.pose_clip({1.0f, 2.0f, 3.0f}), 1.0f, INVALID_ASSET, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 1.0f, 1e-4f);
    EXPECT_NEAR(pose[1], 2.0f, 1e-4f);
    EXPECT_NEAR(pose[2], 3.0f, 1e-4f);
}

TEST(Unit_AnimationLayers, TheBaseLayerIgnoresItsOwnAuthoredWeight)
{
    // The base layer is the pose everything else folds into, so a weight below 1 on it would
    // blend against nothing. It must be full strength regardless of what was authored.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    ASSERT_TRUE(world.compile({{world.pose_clip({4.0f, 4.0f, 4.0f}), 0.25f, INVALID_ASSET, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 4.0f, 1e-4f);
}

TEST(Unit_AnimationLayers, AnOverrideLayerInterpolatesTowardsItsPoseByWeight)
{
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({0.0f, 0.0f, 0.0f});
    const AssetId over = world.pose_clip({10.0f, 10.0f, 10.0f});

    // Weight 0 must be an exact no-op, not "almost the base pose".
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false},
                               {over, 0.0f, INVALID_ASSET, false}}));
    EXPECT_NEAR(world.evaluate_z()[0], 0.0f, 1e-6f);

    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false},
                               {over, 0.25f, INVALID_ASSET, false}}));
    EXPECT_NEAR(world.evaluate_z()[0], 2.5f, 1e-4f);

    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false},
                               {over, 1.0f, INVALID_ASSET, false}}));
    EXPECT_NEAR(world.evaluate_z()[0], 10.0f, 1e-4f) << "full weight must replace, not blend";
}

TEST(Unit_AnimationLayers, AMaskDecidesWhichJointsALayerMayWrite)
{
    // The point of a mask: an upper-body layer must leave the legs to the base layer, exactly,
    // rather than nearly.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({1.0f, 1.0f, 1.0f});
    const AssetId over = world.pose_clip({9.0f, 9.0f, 9.0f});
    const AssetId only_b = world.mask({{"b", 1.0f}}, 0.0f);
    ASSERT_NE(only_b, INVALID_ASSET);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {over, 1.0f, only_b, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 1.0f, 1e-6f) << "a gated joint must be untouched";
    EXPECT_NEAR(pose[1], 9.0f, 1e-4f) << "the admitted joint takes the layer";
    EXPECT_NEAR(pose[2], 1.0f, 1e-6f);
}

TEST(Unit_AnimationLayers, AMaskWeightScalesTheLayerPerJoint)
{
    // A mask entry is a weight, not a boolean, and it multiplies the layer weight — the seam
    // a feathered mask (a shoulder half-admitting an arm layer) needs.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({0.0f, 0.0f, 0.0f});
    const AssetId over = world.pose_clip({8.0f, 8.0f, 8.0f});
    const AssetId feathered = world.mask({{"a", 1.0f}, {"b", 0.5f}}, 0.0f);
    ASSERT_NE(feathered, INVALID_ASSET);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {over, 0.5f, feathered, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 4.0f, 1e-4f) << "layer 0.5 x mask 1.0";
    EXPECT_NEAR(pose[1], 2.0f, 1e-4f) << "layer 0.5 x mask 0.5";
    EXPECT_NEAR(pose[2], 0.0f, 1e-6f) << "layer 0.5 x mask 0.0";
}

TEST(Unit_AnimationLayers, AMaskDefaultWeightCoversTheJointsItDoesNotName)
{
    // Authoring a mask by exclusion: name the joints to hold back and let everything else
    // through. A default of 0 (the opposite convention) is what the tests above use.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({0.0f, 0.0f, 0.0f});
    const AssetId over = world.pose_clip({6.0f, 6.0f, 6.0f});
    const AssetId all_but_c = world.mask({{"c", 0.0f}}, 1.0f);
    ASSERT_NE(all_but_c, INVALID_ASSET);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {over, 1.0f, all_but_c, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 6.0f, 1e-4f);
    EXPECT_NEAR(pose[1], 6.0f, 1e-4f);
    EXPECT_NEAR(pose[2], 0.0f, 1e-6f) << "the named joint is the one held back";
}

TEST(Unit_AnimationLayers, AnAdditiveLayerAddsToTheBaseInsteadOfReplacingIt)
{
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({2.0f, 2.0f, 2.0f});
    const AssetId delta = world.pose_clip({1.0f, 1.0f, 1.0f});

    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {delta, 1.0f, INVALID_ASSET, true}}));
    EXPECT_NEAR(world.evaluate_z()[0], 3.0f, 1e-4f) << "base + delta, not base replaced by delta";

    // Scaled by weight, and an exact no-op at zero.
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {delta, 0.5f, INVALID_ASSET, true}}));
    EXPECT_NEAR(world.evaluate_z()[0], 2.5f, 1e-4f);

    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {delta, 0.0f, INVALID_ASSET, true}}));
    EXPECT_NEAR(world.evaluate_z()[0], 2.0f, 1e-6f);
}

TEST(Unit_AnimationLayers, AnAdditiveLayerIsAlsoGatedByItsMask)
{
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({5.0f, 5.0f, 5.0f});
    const AssetId delta = world.pose_clip({2.0f, 2.0f, 2.0f});
    const AssetId only_a = world.mask({{"a", 1.0f}}, 0.0f);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {delta, 1.0f, only_a, true}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 7.0f, 1e-4f);
    EXPECT_NEAR(pose[1], 5.0f, 1e-6f);
    EXPECT_NEAR(pose[2], 5.0f, 1e-6f);
}

TEST(Unit_AnimationLayers, LayersFoldInOrderSoALaterOneWinsTheJointsItShares)
{
    // Layer order is authoring intent: two override layers touching the same joint must
    // resolve to the later one, and a layer must not disturb a joint no layer above it named.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({0.0f, 0.0f, 0.0f});
    const AssetId first = world.pose_clip({1.0f, 1.0f, 1.0f});
    const AssetId second = world.pose_clip({2.0f, 2.0f, 2.0f});
    const AssetId ab = world.mask({{"a", 1.0f}, {"b", 1.0f}}, 0.0f);
    const AssetId only_b = world.mask({{"b", 1.0f}}, 0.0f);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false},
                               {first, 1.0f, ab, false},
                               {second, 1.0f, only_b, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 1.0f, 1e-4f) << "only the first layer named this joint";
    EXPECT_NEAR(pose[1], 2.0f, 1e-4f) << "the later layer wins the shared joint";
    EXPECT_NEAR(pose[2], 0.0f, 1e-6f) << "no layer named this joint";
}

TEST(Unit_AnimationLayers, MaskingAParentStillCarriesItsUntouchedChildren)
{
    // The property a naive per-joint mask implementation gets wrong: a mask decides which
    // joints a layer *writes*, not which joints move. A gated child still follows a parent the
    // layer moved, because the compose runs after the fold.
    Fixture world;
    world.build_skeleton({"root", "child"}, {-1, 0});
    const AssetId base = world.pose_clip({0.0f, 1.0f});
    const AssetId over = world.pose_clip({5.0f, 1.0f});
    const AssetId only_root = world.mask({{"root", 1.0f}}, 0.0f);
    ASSERT_TRUE(world.compile({{base, 1.0f, INVALID_ASSET, false}, {over, 1.0f, only_root, false}}));

    const std::vector<float> pose = world.evaluate_z();
    EXPECT_NEAR(pose[0], 5.0f, 1e-4f) << "the layer moved the root";
    EXPECT_NEAR(pose[1], 6.0f, 1e-4f)
        << "the gated child kept its own local offset but followed the root";
}

TEST(Unit_AnimationLayers, EveryLayerSlotTheControllerAllowsActuallyFolds)
{
    // MAX_LAYERS is a documented capacity (§4.5), so the last slot must work rather than be
    // silently dropped — and one more than that must fail to compile rather than overflow.
    Fixture world;
    world.build_skeleton(FLAT_NAMES, ALL_ROOTS);
    const AssetId base = world.pose_clip({0.0f, 0.0f, 0.0f});

    std::vector<Fixture::LayerSpecification> layers;
    layers.push_back({base, 1.0f, INVALID_ASSET, false});
    for (std::uint32_t i = 1; i < MAX_LAYERS; ++i)
        layers.push_back({world.pose_clip({1.0f, 1.0f, 1.0f}), 1.0f, INVALID_ASSET, true});
    ASSERT_TRUE(world.compile(layers));
    // Each additive layer adds 1, so the last slot's contribution is visible in the total.
    EXPECT_NEAR(world.evaluate_z()[0], static_cast<float>(MAX_LAYERS - 1), 1e-4f);

    layers.push_back({world.pose_clip({1.0f, 1.0f, 1.0f}), 1.0f, INVALID_ASSET, true});
    ControllerDescription too_many;
    for (std::size_t i = 0; i < layers.size(); ++i)
    {
        LayerDescription layer;
        layer.name = "extra" + std::to_string(i);
        StateDescription state;
        state.name = "State";
        state.clip = layers[i].clip;
        layer.states = {state};
        too_many.layers.push_back(layer);
    }
    std::vector<std::byte> blob;
    EXPECT_FALSE(compile_controller_blob(too_many, blob)) << "more layers than MAX_LAYERS";
}

TEST(Unit_AnimationLayers, AdditiveBakingProducesADeltaAgainstTheReferencePose)
{
    // The asset half of additive blending: an authored clip becomes a delta from a reference
    // pose at import (§4.2), so playing the baked clip additively over that same reference
    // reproduces the original — the round trip the format has to satisfy.
    ClipDescription reference;
    reference.joint_count = 2;
    reference.frame_count = 1;
    reference.sample_rate = 30.0f;
    reference.translations = {Vector3f{0, 1, 0}, Vector3f{0, 2, 0}};
    reference.rotations.assign(2, IDENTITY);
    reference.scales.assign(2, Vector3f{1, 1, 1});

    ClipDescription source = reference;
    source.frame_count = 2;
    source.translations = {Vector3f{0, 1, 0}, Vector3f{0, 2, 0},
                           Vector3f{0, 4, 0}, Vector3f{0, 2, 0}};
    source.rotations.assign(4, IDENTITY);
    source.scales.assign(4, Vector3f{1, 1, 1});

    ClipDescription additive;
    ASSERT_TRUE(bake_additive_clip(source, reference, 0, additive));
    ASSERT_EQ(additive.frame_count, source.frame_count);
    ASSERT_EQ(additive.translations.size(), source.translations.size());

    // Frame 0 equals the reference, so its delta is zero; frame 1 moved joint 0 by +3.
    EXPECT_NEAR(additive.translations[0].y, 0.0f, 1e-5f);
    EXPECT_NEAR(additive.translations[1].y, 0.0f, 1e-5f);
    EXPECT_NEAR(additive.translations[2].y, 3.0f, 1e-5f);
    EXPECT_NEAR(additive.translations[3].y, 0.0f, 1e-5f)
        << "a joint that did not move must bake to no delta";

    // And the refusals: a reference frame that does not exist, and a mis-sized source.
    ClipDescription unused;
    EXPECT_FALSE(bake_additive_clip(source, reference, 5, unused));
    ClipDescription broken = source;
    broken.rotations.pop_back();
    EXPECT_FALSE(bake_additive_clip(broken, reference, 0, unused));
}
