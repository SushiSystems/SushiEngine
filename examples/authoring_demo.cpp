/**************************************************************************/
/* authoring_demo.cpp                                                    */
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

// The phase-A9 authoring seams, worked and self-checked headlessly. First, a rich controller —
// parameters, two layers (a blend-tree locomotion base and an additive masked upper layer),
// transitions, conditions, an event — round-trips through JSON: serialize, parse back, recompile,
// and the .sushictrl blob is byte-identical, which is what the editor's save/load and undo/redo
// (a snapshot is a serialized document) ride on. Second, edit-mode scrubbing pins a blend-tree
// state at an explicit time outside the loop and the AnimatorEvaluator poses it, matching play
// mode.

#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/animation/animator_controller_json.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[authoring_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

    // A rich controller for the round-trip: two layers, a blend tree, transitions, an event.
    ControllerDesc build_rich_controller()
    {
        ControllerDesc desc;
        desc.parameters = {ParameterDesc{"speed", ParameterType::Float, 0.0f},
                           ParameterDesc{"moving", ParameterType::Bool, 0.0f},
                           ParameterDesc{"aim", ParameterType::Float, 1.0f}};

        LayerDesc base;
        base.name = "base";
        base.default_state = "Idle";

        StateDesc idle;
        idle.name = "Idle";
        idle.clip = 0;
        TransitionDesc to_move;
        to_move.destination = "Move";
        to_move.duration = 0.15f;
        to_move.conditions.push_back(ConditionDesc{"moving", Comparator::If, 0.0f});
        idle.transitions.push_back(to_move);

        StateDesc move;
        move.name = "Move";
        auto tree = std::make_shared<BlendTreeNodeDesc>();
        tree->type = BlendTreeType::Simple1D;
        tree->parameter_x = "speed";
        tree->children.push_back(BlendChildDesc{1, nullptr, 0.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{2, nullptr, 1.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{3, nullptr, 2.0f, 0, 0, "", 1});
        move.blend_tree = tree;
        move.events.push_back(StateEventDesc{0.5f, "footstep", 7});
        TransitionDesc to_idle;
        to_idle.destination = "Idle";
        to_idle.conditions.push_back(ConditionDesc{"moving", Comparator::IfNot, 0.0f});
        move.transitions.push_back(to_idle);
        base.states = {idle, move};

        LayerDesc upper;
        upper.name = "upper";
        upper.default_state = "Aim";
        upper.mask = 9; // an asset id; compile stores it verbatim
        upper.blend_mode = LayerBlendMode::Additive;
        upper.weight_parameter = "aim";
        StateDesc aim;
        aim.name = "Aim";
        aim.clip = 4;
        upper.states = {aim};

        desc.layers = {base, upper};
        return desc;
    }

    // A marker clip: single frame parking the root at z, so a blend reads back as the weight-blend.
    AssetId make_marker_clip(AnimationDatabase& database, float z)
    {
        ClipDesc clip;
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
}

int main()
{
    // --- JSON round-trip: serialize -> parse -> recompile -> byte-identical blob -----
    {
        const ControllerDesc desc = build_rich_controller();
        std::vector<std::byte> blob_original;
        check(compile_controller_blob(desc, blob_original), "rich controller compiles");

        const nlohmann::json json = controller_to_json(desc);
        const std::string text = json.dump();
        const ControllerDesc parsed = controller_from_json(nlohmann::json::parse(text));

        std::vector<std::byte> blob_roundtrip;
        check(compile_controller_blob(parsed, blob_roundtrip), "round-tripped controller compiles");
        check(blob_original.size() == blob_roundtrip.size() &&
                  std::memcmp(blob_original.data(), blob_roundtrip.data(), blob_original.size()) == 0,
              "controller JSON round-trip reproduces a byte-identical blob");

        // A tiny structural spot-check on the parsed desc (not just the blob).
        check(parsed.layers.size() == 2 && parsed.layers[1].blend_mode == LayerBlendMode::Additive,
              "parsed desc keeps the additive upper layer");
        check(parsed.layers[0].states[1].blend_tree &&
                  parsed.layers[0].states[1].blend_tree->children.size() == 3,
              "parsed desc keeps the 3-child blend tree");
    }

    // --- Edit-mode scrub: pin a blend-tree state at a time and pose it off the loop ---
    {
        AnimationDatabase database;
        SkeletonDesc skeleton_desc;
        JointDesc root; root.name = "root"; root.parent = -1;
        JointDesc child; child.name = "child"; child.parent = 0; child.bind_translation = Vector3f{0, 1, 0};
        skeleton_desc.joints = {root, child};
        std::vector<std::byte> skeleton_blob;
        build_skeleton_blob(skeleton_desc, skeleton_blob);
        const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));

        const AssetId idle = make_marker_clip(database, 0.0f);
        const AssetId walk = make_marker_clip(database, 1.0f);
        const AssetId run = make_marker_clip(database, 2.0f);

        ControllerDesc desc;
        desc.parameters = {ParameterDesc{"speed", ParameterType::Float, 0.0f}};
        LayerDesc layer;
        layer.name = "base";
        layer.default_state = "Move";
        StateDesc move;
        move.name = "Move";
        auto tree = std::make_shared<BlendTreeNodeDesc>();
        tree->type = BlendTreeType::Simple1D;
        tree->parameter_x = "speed";
        tree->children.push_back(BlendChildDesc{idle, nullptr, 0.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{walk, nullptr, 1.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{run, nullptr, 2.0f, 0, 0, "", 1});
        move.blend_tree = tree;
        layer.states = {move};
        desc.layers.push_back(layer);
        std::vector<std::byte> controller_blob;
        compile_controller_blob(desc, controller_blob);
        const AssetId controller_id = database.add_controller(std::move(controller_blob));
        const ControllerView controller = database.controller(controller_id);

        const int move_index = find_state_in_layer(controller, 0, hash_name("Move"));
        check(move_index == 0, "scrub finds the Move state by name");

        // Scrub straight to Move at t=0 with speed 1.5, without ever ticking the animator.
        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = skeleton_id;
        animator.parameters.set_float(0, 1.5f);
        scrub_to_state(controller, animator, 0, move_index, 0.0f);

        AnimatorEvaluator evaluator;
        evaluator.evaluate(controller, database, animator, database.skeleton(skeleton_id));
        const Mat4& root_model = evaluator.model()[0];
        check(nearly(static_cast<float>(root_model.m[14]), 1.5f),
              "scrub poses the blend-tree state to the 1.5 blend off the loop");
    }

    if (failures != 0)
    {
        std::printf("[authoring_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[authoring_demo] OK — controller JSON round-trip (byte-identical blob) and "
                "edit-mode scrub preview verified\n");
    return 0;
}
