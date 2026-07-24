/**************************************************************************/
/* retarget_demo.cpp                                                     */
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

// The phase-A8 humanoid avatar + retargeting, worked and self-checked headlessly. Two rigs share
// a canonical naming (Hips/Spine/.../LeftLowerArm/...) but differ in proportions and even bind
// rotations. The heuristic avatar maps each rig's joints to the canonical bones by name. A source
// clip bends the left elbow; three things are proved: retargeting reproduces that same
// bend-from-rest on the target rig despite its different bind pose (the bind-pose-delta transfer);
// joints the clip does not drive keep the target's bind pose; and mirroring turns the left-elbow
// bend into a right-elbow bend of the opposite sense.

#include <cmath>
#include <cstdio>
#include <string>
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
            std::printf("[retarget_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    // Two rotations are the same if their quaternions are equal up to sign (q and -q agree).
    bool same_rotation(const Quaternionf& a, const Quaternionf& b, float eps = 2e-3f)
    {
        return std::fabs(dot(a, b)) > 1.0f - eps;
    }

    Quaternionf rotation_z(float degrees)
    {
        const float half = degrees * 0.5f * 3.14159265358979323846f / 180.0f;
        return Quaternionf{0.0f, 0.0f, std::sin(half), std::cos(half)};
    }

    // A canonical 10-joint humanoid upper body. `elbow_bind` sets the LeftLowerArm bind rotation
    // (to give the target rig a different rest pose), and `arm` scales the arm bone lengths.
    AssetId build_humanoid(AnimationDatabase& database, Quaternionf elbow_bind, float arm)
    {
        struct Bone { const char* name; int parent; Vector3f translation; Quaternionf rotation; };
        const Bone bones[] = {
            {"Hips", -1, {0, 1, 0}, {0, 0, 0, 1}},
            {"Spine", 0, {0, 0.2f, 0}, {0, 0, 0, 1}},
            {"Chest", 1, {0, 0.2f, 0}, {0, 0, 0, 1}},
            {"Head", 2, {0, 0.3f, 0}, {0, 0, 0, 1}},
            {"LeftUpperArm", 2, {-arm, 0, 0}, {0, 0, 0, 1}},
            {"LeftLowerArm", 4, {-arm, 0, 0}, elbow_bind},
            {"LeftHand", 5, {-arm, 0, 0}, {0, 0, 0, 1}},
            {"RightUpperArm", 2, {arm, 0, 0}, {0, 0, 0, 1}},
            {"RightLowerArm", 7, {arm, 0, 0}, {0, 0, 0, 1}},
            {"RightHand", 8, {arm, 0, 0}, {0, 0, 0, 1}},
        };
        SkeletonDesc desc;
        for (const Bone& b : bones)
        {
            JointDesc joint;
            joint.name = b.name;
            joint.parent = b.parent;
            joint.bind_translation = b.translation;
            joint.bind_rotation = b.rotation;
            desc.joints.push_back(joint);
        }
        std::vector<std::byte> blob;
        build_skeleton_blob(desc, blob);
        return database.add_skeleton(std::move(blob));
    }
}

int main()
{
    AnimationDatabase database;

    // Source rig: identity rest pose, unit arm bones. Target rig: elbow rest is bent 30 deg,
    // arms are 1.5x longer — different proportions and a different bind, same naming.
    const AssetId source_id = build_humanoid(database, Quaternionf{0, 0, 0, 1}, 1.0f);
    const AssetId target_id = build_humanoid(database, rotation_z(30.0f), 1.5f);
    const SkeletonView source_skeleton = database.skeleton(source_id);
    const SkeletonView target_skeleton = database.skeleton(target_id);

    const Avatar source_avatar = build_avatar_heuristic(source_skeleton);
    const Avatar target_avatar = build_avatar_heuristic(target_skeleton);
    check(source_avatar.mapped_count() == 10, "heuristic maps every source bone");
    check(target_avatar.has(HumanBone::LeftLowerArm), "target maps the left lower arm");

    const int source_elbow = source_avatar.joint(HumanBone::LeftLowerArm);
    const int right_elbow = source_avatar.joint(HumanBone::RightLowerArm);

    // A two-frame source clip: everything at bind, the left elbow bending 45 deg about z at frame 1.
    ClipDesc clip;
    clip.joint_count = source_skeleton.joint_count;
    clip.frame_count = 2;
    clip.sample_rate = 30.0f;
    const std::uint32_t n = clip.joint_count;
    clip.translations.resize(2 * n);
    clip.rotations.resize(2 * n);
    clip.scales.resize(2 * n);
    for (std::uint32_t f = 0; f < 2; ++f)
        for (std::uint32_t j = 0; j < n; ++j)
        {
            clip.translations[f * n + j] = source_skeleton.bind_translations[j];
            clip.rotations[f * n + j] = source_skeleton.bind_rotations[j];
            clip.scales[f * n + j] = source_skeleton.bind_scales[j];
        }
    const Quaternionf elbow_bend = rotation_z(45.0f);
    clip.rotations[1 * n + source_elbow] = elbow_bend; // source elbow bind is identity, so delta == 45 deg

    // --- Retarget onto the differently-proportioned rig -------------------------------
    {
        ClipDesc retargeted;
        check(retarget_clip(clip, source_avatar, source_skeleton, target_avatar, target_skeleton,
                            retargeted),
              "retarget succeeds");
        const std::uint32_t tn = retargeted.joint_count;
        const int target_elbow = target_avatar.joint(HumanBone::LeftLowerArm);

        // The target elbow's delta from its (30 deg) bind should be the source's 45 deg bend.
        const Quaternionf target_bind = target_skeleton.bind_rotations[target_elbow];
        const Quaternionf posed = retargeted.rotations[1 * tn + target_elbow];
        const Quaternionf delta = normalize(mul(conjugate(target_bind), posed));
        check(same_rotation(delta, elbow_bend), "retargeted elbow reproduces the 45 deg bend-from-rest");

        // A joint the clip does not drive (the head) keeps the target's bind pose.
        const int head = target_avatar.joint(HumanBone::Head);
        const Quaternionf head_posed = retargeted.rotations[1 * tn + head];
        check(same_rotation(head_posed, target_skeleton.bind_rotations[head]),
              "undriven joint keeps the target bind pose");
    }

    // --- Mirror the clip left-to-right ------------------------------------------------
    {
        ClipDesc mirrored;
        check(mirror_clip(clip, source_avatar, source_skeleton, mirrored), "mirror succeeds");

        // The right elbow now bends by the mirror of 45 deg about z (i.e. -45 deg).
        const Quaternionf right_bind = source_skeleton.bind_rotations[right_elbow];
        const Quaternionf right_posed = mirrored.rotations[1 * n + right_elbow];
        const Quaternionf right_delta = normalize(mul(conjugate(right_bind), right_posed));
        check(same_rotation(right_delta, rotation_z(-45.0f)), "mirror moves the bend to the right elbow");

        // The left elbow (whose mirror source, the right elbow, was still) returns to bind.
        const Quaternionf left_posed = mirrored.rotations[1 * n + source_elbow];
        check(same_rotation(left_posed, source_skeleton.bind_rotations[source_elbow]),
              "mirror leaves the left elbow at rest");
    }

    if (failures != 0)
    {
        std::printf("[retarget_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[retarget_demo] OK — heuristic avatar mapping, bind-pose-delta retarget across "
                "proportions, and left/right mirror all verified\n");
    return 0;
}
