/**************************************************************************/
/* blend_tree_demo.cpp                                                   */
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

// The phase-A4 blend trees, worked and self-checked headlessly. Each of the five node kinds
// is exercised through the real compile → load → resolve path: a 1D locomotion blend over a
// speed parameter, a 2D freeform-cartesian and freeform-directional strafe, a simple-directional
// blend around a centre, a direct blend driven parameter-per-child, and a nested tree. The
// resolved weights are checked against the analytic answer (a sample at a child's coordinate
// gives that child weight ~1; weights sum to 1). Then the full AnimatorEvaluator runs a
// blend-tree state end to end and the composed root translation is checked to be the
// weight-blend of the contributing clips.

#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[blend_tree_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

    // A clip whose single frame parks the root at a chosen z, so a blended pose reads back as
    // the weight-blend of the contributing clips' z values (translation blends linearly).
    AssetId make_marker_clip(AnimationDatabase& database, float z)
    {
        ClipDesc clip;
        clip.joint_count = 2;
        clip.frame_count = 1;
        clip.sample_rate = 30.0f;
        clip.translations = {Vector3f{0, 0, z}, Vector3f{0, 1, 0}};
        clip.rotations = {Quaternionf{0, 0, 0, 1}, Quaternionf{0, 0, 0, 1}};
        clip.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
        std::vector<std::byte> blob;
        build_clip_blob(clip, blob);
        return database.add_clip(std::move(blob));
    }

    std::shared_ptr<BlendTreeNodeDesc> leaf_tree(BlendTreeType type, const char* px, const char* py)
    {
        auto node = std::make_shared<BlendTreeNodeDesc>();
        node->type = type;
        node->parameter_x = px ? px : "";
        node->parameter_y = py ? py : "";
        return node;
    }
}

int main()
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

    // Builds a one-state controller wrapping a blend tree, so its compiled node arrays can be
    // resolved directly. Parameters are given by name; the tree's root is the state's motion.
    const auto compile_tree = [&](const std::vector<ParameterDesc>& parameters,
                                  std::shared_ptr<BlendTreeNodeDesc> tree) -> AssetId
    {
        ControllerDesc desc;
        desc.parameters = parameters;
        LayerDesc layer;
        layer.name = "base";
        layer.default_state = "Move";
        StateDesc state;
        state.name = "Move";
        state.blend_tree = std::move(tree);
        layer.states = {state};
        desc.layers.push_back(layer);
        std::vector<std::byte> blob;
        if (!compile_controller_blob(desc, blob))
            return INVALID_ASSET;
        return database.add_controller(std::move(blob));
    };

    // Resolves the (single) state's blend tree against a parameter block into contributions.
    const auto resolve = [&](AssetId controller_id, const AnimatorParameterBlock& parameters,
                             BlendContribution* out) -> std::uint32_t
    {
        const ControllerView view = database.controller(controller_id);
        const StateRecord& state = view.states[0];
        return resolve_blend_tree(view.nodes, view.children, view.pairs,
                                  static_cast<std::uint32_t>(state.blend_tree), parameters, out,
                                  MAX_BLEND_CONTRIBUTIONS);
    };

    // Sums the weight a clip receives across all contributions (a clip may appear more than once).
    const auto weight_of = [](const BlendContribution* c, std::uint32_t n, AssetId clip) -> float
    {
        float w = 0.0f;
        for (std::uint32_t i = 0; i < n; ++i)
            if (c[i].clip == clip)
                w += c[i].weight;
        return w;
    };

    BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];

    // --- 1D locomotion: idle(0) / walk(1) / run(2) over "speed" ----------------------
    {
        auto tree = leaf_tree(BlendTreeType::Simple1D, "speed", nullptr);
        tree->children.push_back(BlendChildDesc{idle, nullptr, 0.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{walk, nullptr, 1.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{run, nullptr, 2.0f, 0, 0, "", 1});
        const AssetId id = compile_tree({ParameterDesc{"speed", ParameterType::Float, 0.0f}}, tree);
        check(id != INVALID_ASSET, "1D tree compiles");

        AnimatorParameterBlock p;
        p.set_float(0, 0.5f);
        std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, idle), 0.5f), "1D speed 0.5 -> half idle");
        check(nearly(weight_of(contributions, n, walk), 0.5f), "1D speed 0.5 -> half walk");

        p.set_float(0, 1.5f);
        n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, walk), 0.5f), "1D speed 1.5 -> half walk");
        check(nearly(weight_of(contributions, n, run), 0.5f), "1D speed 1.5 -> half run");

        p.set_float(0, 5.0f); // clamps above the last threshold
        n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, run), 1.0f), "1D above range -> all run");
    }

    // --- 2D freeform cartesian strafe: four cardinal clips ---------------------------
    const auto strafe_children = [&](std::shared_ptr<BlendTreeNodeDesc>& tree)
    {
        tree->children.push_back(BlendChildDesc{walk, nullptr, 0, 0.0f, 1.0f, "", 1});  // forward
        tree->children.push_back(BlendChildDesc{run, nullptr, 0, 1.0f, 0.0f, "", 1});   // right
        tree->children.push_back(BlendChildDesc{idle, nullptr, 0, 0.0f, -1.0f, "", 1}); // back
        tree->children.push_back(BlendChildDesc{run, nullptr, 0, -1.0f, 0.0f, "", 1});  // left
    };
    {
        auto tree = leaf_tree(BlendTreeType::FreeformCartesian2D, "x", "y");
        strafe_children(tree);
        const AssetId id = compile_tree(
            {ParameterDesc{"x", ParameterType::Float, 0.0f}, ParameterDesc{"y", ParameterType::Float, 0.0f}},
            tree);
        AnimatorParameterBlock p;
        p.set_float(0, 0.0f);
        p.set_float(1, 1.0f); // exactly the forward child
        std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, walk), 1.0f), "cartesian at forward -> all forward clip");

        p.set_float(0, 0.0f);
        p.set_float(1, 0.0f); // centre — weights spread and sum to 1
        n = resolve(id, p, contributions);
        float sum = 0.0f;
        for (std::uint32_t i = 0; i < n; ++i)
            sum += contributions[i].weight;
        check(nearly(sum, 1.0f), "cartesian centre weights sum to 1");
    }

    // --- 2D freeform directional strafe ----------------------------------------------
    {
        auto tree = leaf_tree(BlendTreeType::FreeformDirectional2D, "x", "y");
        strafe_children(tree);
        const AssetId id = compile_tree(
            {ParameterDesc{"x", ParameterType::Float, 0.0f}, ParameterDesc{"y", ParameterType::Float, 0.0f}},
            tree);
        AnimatorParameterBlock p;
        p.set_float(0, 1.0f);
        p.set_float(1, 0.0f); // exactly the right child
        const std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, run), 1.0f, 2e-3f),
              "directional at right -> all right clip");
    }

    // --- 2D simple directional: centre + ring ----------------------------------------
    {
        auto tree = leaf_tree(BlendTreeType::SimpleDirectional2D, "x", "y");
        tree->children.push_back(BlendChildDesc{idle, nullptr, 0, 0.0f, 0.0f, "", 1});  // centre
        tree->children.push_back(BlendChildDesc{walk, nullptr, 0, 0.0f, 1.0f, "", 1});  // forward
        tree->children.push_back(BlendChildDesc{run, nullptr, 0, 1.0f, 0.0f, "", 1});   // right
        tree->children.push_back(BlendChildDesc{walk, nullptr, 0, -1.0f, 0.0f, "", 1}); // left
        const AssetId id = compile_tree(
            {ParameterDesc{"x", ParameterType::Float, 0.0f}, ParameterDesc{"y", ParameterType::Float, 0.0f}},
            tree);
        AnimatorParameterBlock p;
        p.set_float(0, 0.0f);
        p.set_float(1, 0.0f); // origin -> centre owns it
        std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, idle), 1.0f), "simple-directional origin -> centre");

        p.set_float(0, 1.0f);
        p.set_float(1, 0.0f); // exactly the right ring child -> centre weight ~0, ring owns it
        n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, run), 1.0f, 2e-3f),
              "simple-directional at ring -> ring clip");
    }

    // --- Direct: parameter-per-child, normalised -------------------------------------
    {
        auto tree = leaf_tree(BlendTreeType::Direct, nullptr, nullptr);
        tree->normalize = true;
        BlendChildDesc a{idle, nullptr, 0, 0, 0, "w_idle", 1};
        BlendChildDesc b{walk, nullptr, 0, 0, 0, "w_walk", 1};
        tree->children = {a, b};
        const AssetId id =
            compile_tree({ParameterDesc{"w_idle", ParameterType::Float, 0.0f},
                          ParameterDesc{"w_walk", ParameterType::Float, 0.0f}},
                         tree);
        AnimatorParameterBlock p;
        p.set_float(0, 3.0f); // idle weight 3
        p.set_float(1, 1.0f); // walk weight 1 -> normalised 0.75 / 0.25
        const std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, idle), 0.75f), "direct normalises idle to 0.75");
        check(nearly(weight_of(contributions, n, walk), 0.25f), "direct normalises walk to 0.25");
    }

    // --- Nested: a 1D whose top child is a Direct sub-tree ----------------------------
    {
        auto sub = leaf_tree(BlendTreeType::Direct, nullptr, nullptr);
        sub->children = {BlendChildDesc{walk, nullptr, 0, 0, 0, "w_walk", 1},
                         BlendChildDesc{run, nullptr, 0, 0, 0, "w_run", 1}};
        auto tree = leaf_tree(BlendTreeType::Simple1D, "speed", nullptr);
        BlendChildDesc low{idle, nullptr, 0.0f, 0, 0, "", 1};
        BlendChildDesc high{INVALID_ASSET, sub, 1.0f, 0, 0, "", 1};
        tree->children = {low, high};
        const AssetId id = compile_tree({ParameterDesc{"speed", ParameterType::Float, 0.0f},
                                         ParameterDesc{"w_walk", ParameterType::Float, 0.0f},
                                         ParameterDesc{"w_run", ParameterType::Float, 0.0f}},
                                        tree);
        AnimatorParameterBlock p;
        p.set_float(0, 1.0f); // fully the nested Direct sub-tree
        p.set_float(1, 1.0f); // walk
        p.set_float(2, 1.0f); // run -> each 0.5 inside the sub-tree
        const std::uint32_t n = resolve(id, p, contributions);
        check(nearly(weight_of(contributions, n, walk), 0.5f), "nested sub-tree walk 0.5");
        check(nearly(weight_of(contributions, n, run), 0.5f), "nested sub-tree run 0.5");
        check(nearly(weight_of(contributions, n, idle), 0.0f), "nested top child excludes idle");
    }

    // --- End to end: a blend-tree state posed by the evaluator ------------------------
    {
        auto tree = leaf_tree(BlendTreeType::Simple1D, "speed", nullptr);
        tree->children.push_back(BlendChildDesc{idle, nullptr, 0.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{walk, nullptr, 1.0f, 0, 0, "", 1});
        tree->children.push_back(BlendChildDesc{run, nullptr, 2.0f, 0, 0, "", 1});
        const AssetId controller_id =
            compile_tree({ParameterDesc{"speed", ParameterType::Float, 0.0f}}, tree);
        const ControllerView controller = database.controller(controller_id);

        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = skeleton_id;
        animator_step(controller, database, animator, 0.0f); // init the layer state
        animator.parameters.set_float(0, 1.5f);              // half walk (z=1) + half run (z=2)

        AnimatorEvaluator evaluator;
        evaluator.evaluate(controller, database, animator, database.skeleton(skeleton_id));
        const Mat4& root_model = evaluator.model()[0];
        // Column-major translation lives in elements 12/13/14; expect the weight-blend 1.5.
        check(nearly(static_cast<float>(root_model.m[14]), 1.5f),
              "evaluator blends root z to 1.5 at speed 1.5");
    }

    if (failures != 0)
    {
        std::printf("[blend_tree_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[blend_tree_demo] OK — 1D, 2D cartesian/directional, simple-directional, direct, "
                "nested, and evaluator blend all verified\n");
    return 0;
}
