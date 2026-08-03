/**************************************************************************/
/* keyframe_demo.cpp                                                     */
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

// The keyframe authoring model — sparse curves, quaternion curves, pose recording, and baking to
// the dense runtime clip — worked and self-checked headlessly. It is the editor's dope-sheet /
// curve-editor data and the "record a live pose into a clip" workflow. Five things are proved:
// scalar curves interpolate constant / linear / cubic and pass through their keys; a quaternion
// curve slerps; recording a moving pose over time and baking reproduces it through the runtime
// ClipView; and morph curves bake into the clip's morph tracks.

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
            std::printf("[keyframe_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(float a, float b, float eps = 1e-3f) { return std::fabs(a - b) <= eps; }
}

int main()
{
    // --- Scalar curve: linear, constant, cubic ---------------------------------------
    {
        ScalarCurve curve;
        curve.insert(0.0f, 0.0f);
        curve.insert(1.0f, 10.0f);
        curve.insert(2.0f, 0.0f);
        curve.mode = InterpolationMode::Linear;
        check(nearly(curve.evaluate(0.5f), 5.0f), "linear interpolates to 5 at t=0.5");
        check(nearly(curve.evaluate(1.5f), 5.0f), "linear interpolates to 5 at t=1.5");
        check(nearly(curve.evaluate(-1.0f), 0.0f), "curve clamps to the first key before the start");
        check(nearly(curve.evaluate(9.0f), 0.0f), "curve clamps to the last key after the end");

        curve.mode = InterpolationMode::Constant;
        check(nearly(curve.evaluate(0.5f), 0.0f), "constant holds the earlier key");

        curve.mode = InterpolationMode::Cubic;
        curve.auto_tangents();
        check(nearly(curve.evaluate(0.0f), 0.0f) && nearly(curve.evaluate(1.0f), 10.0f) &&
                  nearly(curve.evaluate(2.0f), 0.0f),
              "cubic passes through its keys");

        curve.remove_at(1.0f);
        check(curve.keys.size() == 2, "remove drops the key");
    }

    // --- Quaternion curve: slerp -----------------------------------------------------
    {
        QuaternionCurve curve;
        curve.insert(0.0f, Quaternionf{0, 0, 0, 1});
        const float half = 45.0f * 3.14159265358979323846f / 180.0f; // 90 deg about z
        curve.insert(1.0f, Quaternionf{0, 0, std::sin(half), std::cos(half)});
        const Quaternionf mid = curve.evaluate(0.5f); // ~45 deg about z
        const float mid_half = 22.5f * 3.14159265358979323846f / 180.0f;
        const Quaternionf expected{0, 0, std::sin(mid_half), std::cos(mid_half)};
        check(std::fabs(dot(mid, expected)) > 0.999f, "quaternion curve slerps to the half angle");
    }

    // --- Record a moving pose, bake, and read it back through the runtime clip --------
    {
        AnimationDatabase database;
        SkeletonDesc skeleton_desc;
        JointDesc root; root.name = "root"; root.parent = -1;
        JointDesc child; child.name = "child"; child.parent = 0; child.bind_translation = Vector3f{0, 1, 0};
        skeleton_desc.joints = {root, child};
        std::vector<std::byte> skeleton_blob;
        build_skeleton_blob(skeleton_desc, skeleton_blob);
        const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
        const SkeletonView skeleton = database.skeleton(skeleton_id);

        const NameHash names[2] = {skeleton.joint_names[0], skeleton.joint_names[1]};
        ClipAuthoring authoring;
        PoseRecorder recorder;
        recorder.begin(authoring, names, 2);

        // Record the root sliding 0 -> 1 -> 2 in z at t = 0, 0.5, 1.0 (child held).
        const float times[3] = {0.0f, 0.5f, 1.0f};
        const float zs[3] = {0.0f, 1.0f, 2.0f};
        for (int k = 0; k < 3; ++k)
        {
            Vector3f translations[2] = {Vector3f{0, 0, zs[k]}, Vector3f{0, 1, 0}};
            Quaternionf rotations[2] = {Quaternionf{0, 0, 0, 1}, Quaternionf{0, 0, 0, 1}};
            Vector3f scales[2] = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
            recorder.record_pose(times[k], translations, rotations, scales, 2);
        }
        check(nearly(authoring.duration(), 1.0f), "authoring duration is the last key time");

        ClipDesc dense;
        check(authoring.bake(30.0f, dense), "authoring bakes to a dense clip");
        check(dense.frame_count == 31, "bake resamples 1 s at 30 Hz to 31 frames");
        std::vector<std::byte> clip_blob;
        build_clip_blob(dense, clip_blob);
        const AssetId clip_id = database.add_clip(std::move(clip_blob));
        const ClipView clip = database.clip(clip_id);

        // The baked clip, sampled through the runtime view, reproduces the recorded slide.
        Vector3f t[2];
        Quaternionf r[2];
        Vector3f s[2];
        clip.sample(0.25f, false, t, r, s); // between t=0 and t=0.5 -> z ~ 0.5
        check(nearly(t[0].z, 0.5f, 5e-2f), "recorded+baked clip reproduces the mid slide (z~0.5)");
        clip.sample(1.0f, false, t, r, s);
        check(nearly(t[0].z, 2.0f, 5e-2f), "recorded+baked clip reproduces the end slide (z~2.0)");
    }

    // --- Morph curve bakes into the clip's morph tracks ------------------------------
    {
        ClipAuthoring authoring;
        authoring.joints.assign(1, JointChannels{}); // one joint so bake has a rig
        NamedCurve smile;
        smile.name = "smile";
        smile.curve.insert(0.0f, 0.0f);
        smile.curve.insert(1.0f, 1.0f);
        authoring.morphs.push_back(smile);

        ClipDesc dense;
        check(authoring.bake(30.0f, dense), "authoring with a morph bakes");
        check(dense.morph_names.size() == 1 && dense.morph_names[0] == "smile",
              "bake carries the morph track name");

        AnimationDatabase database;
        std::vector<std::byte> blob;
        build_clip_blob(dense, blob);
        const ClipView clip = database.clip(database.add_clip(std::move(blob)));
        float weight;
        clip.sample_morph(0.5f, false, &weight);
        check(nearly(weight, 0.5f, 5e-2f), "baked morph track samples to 0.5 at the midpoint");
    }

    if (failures != 0)
    {
        std::printf("[keyframe_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[keyframe_demo] OK — scalar/quaternion curves, pose recording, dense bake, and "
                "morph curves all verified\n");
    return 0;
}
