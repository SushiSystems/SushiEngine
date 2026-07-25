/**************************************************************************/
/* runtime_retarget_demo.cpp                                             */
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

// The §12.4 "same-session retarget onto a different rig" gap, worked and self-checked.
// Also the first time `retarget_pose_frame`/`retarget_clip`'s underlying delta-transfer
// algebra gets an actual built-and-run check in this repository (it shipped in phase A8
// with no demo of its own). Two proofs, both hand-derivation-free:
//   * Retargeting a clip onto a *clone* of its own source rig (identical bind pose, so
//     hip_scale == 1 and every bone length matches) must reproduce plain `ClipEvaluator`
//     sampling exactly — retargeting onto an identical rig is the identity transform. This
//     validates the delta-transfer math itself, not just "it runs."
//   * Retargeting the same clip onto a rig with a doubled hip height and a doubled upper-arm
//     length reproduces the *same* joint bend (the retargeted local rotation matches the
//     identity-retarget case bit-for-bit — the bend is proportion-independent) while the
//     retargeted skeleton's own tip position scales with *its own* bone length, not the
//     source's — the textbook "same bend, different reach" retargeting property.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/runtime_retarget.hpp>
#include <SushiEngine/core/types.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[runtime_retarget_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

    // Hips -> LeftUpperArm -> ArmTip. `arm_length` sets both LeftUpperArm's and ArmTip's
    // bind translation length, so the whole chain scales together.
    std::vector<std::byte> make_arm_skeleton(float hip_height, float arm_length)
    {
        SkeletonDesc desc;
        JointDesc hips;
        hips.name = "Hips";
        hips.parent = -1;
        hips.bind_translation = Vector3f{0.0f, hip_height, 0.0f};
        JointDesc arm;
        arm.name = "LeftUpperArm";
        arm.parent = 0;
        arm.bind_translation = Vector3f{arm_length, 0.0f, 0.0f};
        JointDesc tip;
        tip.name = "ArmTip";
        tip.parent = 1;
        tip.bind_translation = Vector3f{arm_length, 0.0f, 0.0f};
        desc.joints = {hips, arm, tip};
        std::vector<std::byte> blob;
        if (!build_skeleton_blob(desc, blob))
            std::printf("[runtime_retarget_demo] FAIL: cook skeleton\n"), ++failures;
        return blob;
    }

    // Frame 0 is bind; frame 1 moves the hips by `hip_delta` and rotates LeftUpperArm by
    // `arm_rotation` (both relative to bind, which is identity/zero here). ArmTip is left at
    // bind on both frames — it plays no canonical bone, so a correct retarget never touches
    // it; only the parent chain should move it.
    std::vector<std::byte> make_arm_clip(float hip_height, float arm_length,
                                         Vector3f hip_delta, Quaternionf arm_rotation)
    {
        const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
        ClipDesc desc;
        desc.joint_count = 3;
        desc.frame_count = 2;
        desc.sample_rate = 1.0f;
        desc.translations = {
            Vector3f{0.0f, hip_height, 0.0f}, Vector3f{arm_length, 0.0f, 0.0f},
            Vector3f{arm_length, 0.0f, 0.0f}, // frame 0 (bind)
            Vector3f{0.0f, hip_height, 0.0f} + hip_delta, Vector3f{arm_length, 0.0f, 0.0f},
            Vector3f{arm_length, 0.0f, 0.0f}, // frame 1
        };
        desc.rotations = {identity, identity, identity, identity, arm_rotation, identity};
        desc.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1},
                       Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
        std::vector<std::byte> blob;
        if (!build_clip_blob(desc, blob))
            std::printf("[runtime_retarget_demo] FAIL: cook clip\n"), ++failures;
        return blob;
    }

    Vector3 model_position(const std::vector<Mat4>& model, std::uint32_t joint)
    {
        return Vector3{model[joint].m[12], model[joint].m[13], model[joint].m[14]};
    }
}

int main()
{
    AnimationDatabase database;

    const AssetId source_skeleton_id =
        database.add_skeleton(make_arm_skeleton(/*hip_height=*/1.0f, /*arm_length=*/1.0f));
    const AssetId clone_skeleton_id =
        database.add_skeleton(make_arm_skeleton(1.0f, 1.0f)); // identical bind to source
    const AssetId scaled_skeleton_id =
        database.add_skeleton(make_arm_skeleton(/*hip_height=*/2.0f, /*arm_length=*/2.0f));
    check(source_skeleton_id != INVALID_ASSET && clone_skeleton_id != INVALID_ASSET &&
             scaled_skeleton_id != INVALID_ASSET,
         "register the three skeletons");

    const SkeletonView source_skeleton = database.skeleton(source_skeleton_id);
    const SkeletonView clone_skeleton = database.skeleton(clone_skeleton_id);
    const SkeletonView scaled_skeleton = database.skeleton(scaled_skeleton_id);

    const Avatar source_avatar = build_avatar_heuristic(source_skeleton);
    const Avatar clone_avatar = build_avatar_heuristic(clone_skeleton);
    const Avatar scaled_avatar = build_avatar_heuristic(scaled_skeleton);
    check(source_avatar.has(HumanBone::Hips) && source_avatar.has(HumanBone::LeftUpperArm),
         "the heuristic maps Hips and LeftUpperArm by name");

    const Vector3f hip_delta{0.0f, 0.0f, 1.0f};
    const Quaternionf arm_rotation = quaternion_axis_angle(Vector3f{0.0f, 0.0f, 1.0f}, 1.5707963f);
    const AssetId clip_id =
        database.add_clip(make_arm_clip(1.0f, 1.0f, hip_delta, arm_rotation));
    check(clip_id != INVALID_ASSET, "register the clip");
    const ClipView clip = database.clip(clip_id);

    // --- Identity check: retargeting onto a bind-identical clone must equal plain sampling.
    ClipEvaluator direct;
    direct.evaluate(source_skeleton, clip, /*time_seconds=*/1.0f, /*loop=*/false);

    RuntimeRetargeter identity_retarget;
    identity_retarget.bind(source_skeleton, source_avatar, clone_skeleton, clone_avatar);
    identity_retarget.evaluate(clip, 1.0f, false);

    bool identity_matches = true;
    for (std::uint32_t j = 0; j < source_skeleton.joint_count; ++j)
    {
        const Vector3 direct_p = model_position(direct.model(), j);
        const Vector3 retarget_p = model_position(identity_retarget.model(), j);
        if (!nearly(direct_p.x, retarget_p.x, 1e-4) || !nearly(direct_p.y, retarget_p.y, 1e-4) ||
            !nearly(direct_p.z, retarget_p.z, 1e-4))
            identity_matches = false;
    }
    check(identity_matches,
         "retargeting onto a bind-identical clone reproduces direct sampling exactly");

    // --- Cross-retarget onto a rig with double hip height and double arm length.
    RuntimeRetargeter scaled_retarget;
    scaled_retarget.bind(source_skeleton, source_avatar, scaled_skeleton, scaled_avatar);
    scaled_retarget.evaluate(clip, 1.0f, false);

    // Same bend: the retargeted LeftUpperArm local rotation must match the identity-retarget
    // case bit-for-bit — the joint angle transfers independent of the target's proportions.
    const Quaternionf identity_arm_rotation = identity_retarget.local_rotations()[1];
    const Quaternionf scaled_arm_rotation = scaled_retarget.local_rotations()[1];
    check(nearly(identity_arm_rotation.x, scaled_arm_rotation.x, 1e-5) &&
             nearly(identity_arm_rotation.y, scaled_arm_rotation.y, 1e-5) &&
             nearly(identity_arm_rotation.z, scaled_arm_rotation.z, 1e-5) &&
             nearly(identity_arm_rotation.w, scaled_arm_rotation.w, 1e-5),
         "the retargeted arm bend is identical regardless of the target rig's proportions");

    // Different reach: the scaled rig's hips moved by 2x the source's translation delta
    // (hip_scale == scaled_hip_height / source_hip_height == 2), and its arm tip sits
    // exactly its *own* two-unit chain length from the (moved) hips, not the source's one.
    const Vector3 source_hips = model_position(direct.model(), 0);
    const Vector3 scaled_hips = model_position(scaled_retarget.model(), 0);
    const Vector3 expected_scaled_hips =
        Vector3{0.0, 2.0, 0.0} + Vector3{static_cast<Scalar>(hip_delta.x) * 2.0,
                                         static_cast<Scalar>(hip_delta.y) * 2.0,
                                         static_cast<Scalar>(hip_delta.z) * 2.0};
    check(nearly(scaled_hips.x, expected_scaled_hips.x, 1e-4) &&
             nearly(scaled_hips.y, expected_scaled_hips.y, 1e-4) &&
             nearly(scaled_hips.z, expected_scaled_hips.z, 1e-4),
         "the scaled rig's hips translate by the source delta scaled by the hip-height ratio");
    (void)source_hips;

    // A 90 deg bend between two equal-length segments makes a right isoceles triangle
    // (Hips -> LeftUpperArm -> ArmTip): the hips-to-tip distance is length * sqrt(2),
    // regardless of which way the bend turns — a rotation-convention-agnostic check that
    // still pins down the target's own (doubled) segment length was actually used, not the
    // source's.
    const Vector3 scaled_tip = model_position(scaled_retarget.model(), 2);
    const double reach = length(scaled_tip - scaled_hips);
    const double expected_reach = 2.0 * std::sqrt(2.0); // segment length 2.0 (the scaled rig's own)
    check(nearly(reach, expected_reach, 1e-3),
         "the scaled rig's tip sits its own (doubled) chain length from its own hips");

    if (failures != 0)
    {
        std::printf("[runtime_retarget_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[runtime_retarget_demo] OK — identity retarget matches direct sampling, "
               "and same-bend/different-reach verified against a differently-proportioned "
               "rig\n");
    return 0;
}
