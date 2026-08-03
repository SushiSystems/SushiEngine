/**************************************************************************/
/* layered_animation_demo.cpp                                            */
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

// The phase-A5 layers, masks, and additive baking — the shipped example is an upper-body aim
// layer over a locomotion base, worked and self-checked headlessly. A flat six-joint rig (every
// joint a root, so a joint's composed matrix is its local pose and reads back directly) carries
// a locomotion base pose and an aim pose on a second layer gated by an upper-body avatar mask.
// Three things are proved: an override layer through the mask writes only the masked joints (the
// arm and spine take the aim pose, the legs keep the locomotion pose); an animatable layer
// weight blends the masked joints halfway; and an import-baked additive layer adds its delta —
// a translation and a rotation — onto the base at the masked joint only.

#include <cmath>
#include <cstdio>
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
            std::printf("[layered_animation_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }

    // Joint layout: 0 root, 1 spine, 2 chest, 3 arm (upper body); 4 leg_l, 5 leg_r (lower body).
    constexpr std::uint32_t JOINTS = 6;
    const char* const JOINT_NAMES[JOINTS] = {"root", "spine", "chest", "arm", "leg_l", "leg_r"};

    // A single-frame clip parking every joint's z at base + index, and (optionally) rotating the
    // arm about z, so a folded pose reads back per joint as a plain number.
    AssetId make_pose_clip(AnimationDatabase& database, float base_z, float arm_angle_degrees = 0.0f)
    {
        ClipDesc clip;
        clip.joint_count = JOINTS;
        clip.frame_count = 1;
        clip.sample_rate = 30.0f;
        clip.translations.resize(JOINTS);
        clip.rotations.assign(JOINTS, Quaternionf{0, 0, 0, 1});
        clip.scales.assign(JOINTS, Vector3f{1, 1, 1});
        for (std::uint32_t j = 0; j < JOINTS; ++j)
            clip.translations[j] = Vector3f{0, 0, base_z + static_cast<float>(j)};
        if (arm_angle_degrees != 0.0f)
        {
            const float half = arm_angle_degrees * 0.5f * 3.14159265358979323846f / 180.0f;
            clip.rotations[3] = Quaternionf{0, 0, std::sin(half), std::cos(half)};
        }
        std::vector<std::byte> blob;
        build_clip_blob(clip, blob);
        return database.add_clip(std::move(blob));
    }

    float joint_z(const AnimatorEvaluator& evaluator, std::uint32_t joint)
    {
        return static_cast<float>(evaluator.model()[joint].m[14]);
    }
}

int main()
{
    AnimationDatabase database;

    // A flat rig — every joint a root — so model == local and poses read back directly.
    SkeletonDesc skeleton_desc;
    for (std::uint32_t j = 0; j < JOINTS; ++j)
    {
        JointDesc joint;
        joint.name = JOINT_NAMES[j];
        joint.parent = -1;
        skeleton_desc.joints.push_back(joint);
    }
    std::vector<std::byte> skeleton_blob;
    build_skeleton_blob(skeleton_desc, skeleton_blob);
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const SkeletonView skeleton = database.skeleton(skeleton_id);

    const AssetId loco_clip = make_pose_clip(database, 10.0f);        // base: z = 10 + index
    const AssetId aim_clip = make_pose_clip(database, 100.0f);        // aim:  z = 100 + index

    // Upper-body mask: admit spine/chest/arm at full weight, exclude everything else.
    MaskDesc mask_desc;
    mask_desc.default_weight = 0.0f;
    mask_desc.entries = {{"spine", 1.0f}, {"chest", 1.0f}, {"arm", 1.0f}};
    std::vector<std::byte> mask_blob;
    build_mask_blob(mask_desc, mask_blob);
    const AssetId mask_id = database.add_mask(std::move(mask_blob));
    check(mask_id != INVALID_ASSET, "mask registers");

    // Sanity: the mask resolves to the rig with the expected per-joint weights.
    {
        float weights[JOINTS];
        database.mask(mask_id).resolve(skeleton, weights);
        check(nearly(weights[0], 0.0f) && nearly(weights[1], 1.0f) && nearly(weights[3], 1.0f) &&
                  nearly(weights[4], 0.0f),
              "mask resolves upper-body joints to 1 and lower-body to 0");
    }

    // Builds a two-layer controller: locomotion base + a second layer over the upper-body mask.
    const auto build_layered = [&](AssetId second_clip, LayerBlendMode mode,
                                   const std::string& weight_parameter) -> AssetId
    {
        ControllerDesc desc;
        if (!weight_parameter.empty())
            desc.parameters.push_back(ParameterDesc{weight_parameter, ParameterType::Float, 1.0f});

        LayerDesc base;
        base.name = "base";
        base.default_state = "Loco";
        StateDesc loco;
        loco.name = "Loco";
        loco.clip = loco_clip;
        base.states = {loco};

        LayerDesc upper;
        upper.name = "upper";
        upper.default_state = "Aim";
        upper.mask = mask_id;
        upper.blend_mode = mode;
        upper.weight = 1.0f;
        upper.weight_parameter = weight_parameter;
        StateDesc aim;
        aim.name = "Aim";
        aim.clip = second_clip;
        upper.states = {aim};

        desc.layers = {base, upper};
        std::vector<std::byte> blob;
        if (!compile_controller_blob(desc, blob))
            return INVALID_ASSET;
        return database.add_controller(std::move(blob));
    };

    AnimatorEvaluator evaluator;

    // --- 1) Override through the mask: masked joints take aim, others keep locomotion ---
    {
        const AssetId controller_id = build_layered(aim_clip, LayerBlendMode::Override, "");
        check(controller_id != INVALID_ASSET, "override controller compiles");
        const ControllerView controller = database.controller(controller_id);

        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = skeleton_id;
        animator_step(controller, database, animator, 0.0f); // init both layers

        evaluator.evaluate(controller, database, animator, skeleton);
        check(nearly(joint_z(evaluator, 0), 10.0f), "root keeps locomotion (masked out)");
        check(nearly(joint_z(evaluator, 1), 101.0f), "spine takes aim (masked in)");
        check(nearly(joint_z(evaluator, 3), 103.0f), "arm takes aim (masked in)");
        check(nearly(joint_z(evaluator, 4), 14.0f), "leg keeps locomotion (masked out)");
        check(nearly(joint_z(evaluator, 5), 15.0f), "other leg keeps locomotion (masked out)");
    }

    // --- 2) Animatable layer weight: masked joints blend halfway ----------------------
    {
        const AssetId controller_id = build_layered(aim_clip, LayerBlendMode::Override, "aim_weight");
        const ControllerView controller = database.controller(controller_id);
        const int weight_index = controller.find_parameter(hash_name("aim_weight"));
        check(weight_index == 0, "layer-weight parameter resolves");

        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = skeleton_id;
        animator.parameters.set_float(0, 0.5f);
        animator_step(controller, database, animator, 0.0f); // reads the weight parameter into the layer

        evaluator.evaluate(controller, database, animator, skeleton);
        // spine blends base(11) toward aim(101) by 0.5 -> 56; the legs stay on locomotion.
        check(nearly(joint_z(evaluator, 1), 56.0f), "spine blends halfway at layer weight 0.5");
        check(nearly(joint_z(evaluator, 4), 14.0f), "leg unaffected by the upper-body layer");
    }

    // --- 3) Additive: an import-baked delta adds onto the base at the masked joint -----
    {
        // Source arm pose: shifted +5 in z and rotated 90 deg about z; reference is the rest pose.
        ClipDesc source;
        source.joint_count = JOINTS;
        source.frame_count = 1;
        source.sample_rate = 30.0f;
        source.translations.assign(JOINTS, Vector3f{0, 0, 0});
        source.rotations.assign(JOINTS, Quaternionf{0, 0, 0, 1});
        source.scales.assign(JOINTS, Vector3f{1, 1, 1});
        source.translations[3] = Vector3f{0, 0, 5.0f};
        {
            const float half = 45.0f * 3.14159265358979323846f / 180.0f; // 90 deg about z
            source.rotations[3] = Quaternionf{0, 0, std::sin(half), std::cos(half)};
        }

        ClipDesc reference;
        reference.joint_count = JOINTS;
        reference.frame_count = 1;
        reference.sample_rate = 30.0f;
        reference.translations.assign(JOINTS, Vector3f{0, 0, 0});
        reference.rotations.assign(JOINTS, Quaternionf{0, 0, 0, 1});
        reference.scales.assign(JOINTS, Vector3f{1, 1, 1});

        ClipDesc additive;
        check(bake_additive_clip(source, reference, 0, additive), "additive clip bakes");
        std::vector<std::byte> additive_blob;
        build_clip_blob(additive, additive_blob);
        const AssetId additive_clip = database.add_clip(std::move(additive_blob));

        const AssetId controller_id = build_layered(additive_clip, LayerBlendMode::Additive, "");
        const ControllerView controller = database.controller(controller_id);

        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = skeleton_id;
        animator_step(controller, database, animator, 0.0f);

        evaluator.evaluate(controller, database, animator, skeleton);
        // Arm base z is 13; the additive delta adds +5 -> 18. Legs and root are outside the mask.
        check(nearly(joint_z(evaluator, 3), 18.0f), "additive adds +5 to the arm's base z");
        check(nearly(joint_z(evaluator, 0), 10.0f), "additive leaves the root untouched (masked out)");
        check(nearly(joint_z(evaluator, 4), 14.0f), "additive leaves the leg untouched (masked out)");

        // The arm's rotation is base(identity) composed with the +90 deg-about-z delta, so its
        // model matrix is a pure z-rotation: column-major m[0]=cos(90)=0, m[1]=sin(90)=1.
        const Mat4& arm = evaluator.model()[3];
        check(nearly(static_cast<float>(arm.m[1]), 1.0f, 2e-3f) &&
                  nearly(static_cast<float>(arm.m[0]), 0.0f, 2e-3f),
              "additive rotates the arm 90 deg about z");
    }

    if (failures != 0)
    {
        std::printf("[layered_animation_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[layered_animation_demo] OK — mask-gated override, animatable layer weight, and "
                "import-baked additive all verified\n");
    return 0;
}
