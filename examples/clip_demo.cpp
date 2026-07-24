/**************************************************************************/
/* clip_demo.cpp                                                         */
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

// The phase-A1 clip + evaluator spine, worked and self-checked headlessly. A two-joint
// arm (root -> child at +X) plays a one-second clip whose only motion is the root turning
// 90 degrees about Z. Because the child sits one unit down +X, its model-space position
// traces a quarter circle, which gives every sample an analytic answer:
//   * t = 0    -> clip frame 0 == bind pose -> skin palette is identity, child at (1,0,0).
//   * t = 1    -> root turned 90 deg        -> child at (0,1,0).
//   * t = 0.5  -> root turned ~45 deg        -> child near (0.707,0.707,0).
//   * loop back to t = 2 (== 0)             -> child at (1,0,0) again.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    constexpr double PI = 3.14159265358979323846;

    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[clip_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }

    // Model-space position of a joint (translation column of its composed matrix).
    Vector3 joint_position(const ClipEvaluator& evaluator, std::uint32_t joint)
    {
        const Mat4& m = evaluator.model()[joint];
        return Vector3{m.m[12], m.m[13], m.m[14]};
    }
}

int main()
{
    // --- Skeleton: root at the origin, child one unit down +X ------------------------
    SkeletonDesc skeleton_desc;
    JointDesc root;  root.name = "root";  root.parent = -1;
    JointDesc child; child.name = "child"; child.parent = 0;
    child.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
    skeleton_desc.joints = {root, child};

    std::vector<std::byte> skeleton_blob;
    check(build_skeleton_blob(skeleton_desc, skeleton_blob), "cook skeleton");

    // --- Clip: two frames at 1 Hz; the root turns 90 deg about Z from frame 0 to 1 ----
    const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
    const QuaternionT<float> turn = quaternion_axis_angle(Vector3T<float>{0, 0, 1},
                                                          static_cast<float>(PI * 0.5));
    ClipDesc clip_desc;
    clip_desc.joint_count = 2;
    clip_desc.frame_count = 2;
    clip_desc.sample_rate = 1.0f;
    // Frame-major: [frame0: root, child][frame1: root, child].
    clip_desc.translations = {Vector3f{0, 0, 0}, Vector3f{1, 0, 0},
                              Vector3f{0, 0, 0}, Vector3f{1, 0, 0}};
    clip_desc.rotations = {identity, identity, turn, identity};
    clip_desc.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};

    std::vector<std::byte> clip_blob;
    check(build_clip_blob(clip_desc, clip_blob), "cook clip");

    // --- Register both in the database (shared id space) ------------------------------
    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const AssetId clip_id = database.add_clip(std::move(clip_blob));
    check(skeleton_id != INVALID_ASSET && clip_id != INVALID_ASSET, "register assets");
    check(skeleton_id != clip_id, "shared id space is unique across kinds");
    check(database.has_skeleton(skeleton_id) && database.has_clip(clip_id), "asset kinds resolve");
    check(!database.has_clip(skeleton_id) && !database.has_skeleton(clip_id), "kinds do not cross");

    const SkeletonView skeleton = database.skeleton(skeleton_id);
    const ClipView clip = database.clip(clip_id);
    check(clip.frame_count == 2 && nearly(clip.duration, 1.0), "clip metadata");

    ClipEvaluator evaluator;

    // t = 0: clip frame 0 equals the bind pose, so every skin matrix is the identity.
    evaluator.evaluate(skeleton, clip, 0.0f, /*loop=*/true);
    bool identity_skin = true;
    for (std::uint32_t i = 0; i < skeleton.joint_count; ++i)
        for (int k = 0; k < 16; ++k)
        {
            const double id = (k % 5 == 0) ? 1.0 : 0.0;
            if (!nearly(evaluator.palette()[i].m[k], id))
                identity_skin = false;
        }
    check(identity_skin, "t=0 skin palette is identity");
    Vector3 p0 = joint_position(evaluator, 1);
    check(nearly(p0.x, 1.0) && nearly(p0.y, 0.0) && nearly(p0.z, 0.0), "t=0 child at (1,0,0)");

    // t = 1: the root has turned 90 deg, carrying the child to (0,1,0).
    evaluator.evaluate(skeleton, clip, 1.0f, /*loop=*/false);
    Vector3 p1 = joint_position(evaluator, 1);
    check(nearly(p1.x, 0.0) && nearly(p1.y, 1.0) && nearly(p1.z, 0.0), "t=1 child at (0,1,0)");

    // t = 0.5: ~45 deg -> the child is near the diagonal.
    evaluator.evaluate(skeleton, clip, 0.5f, /*loop=*/false);
    Vector3 ph = joint_position(evaluator, 1);
    const double diag = std::sqrt(0.5);
    check(nearly(ph.x, diag, 2e-3) && nearly(ph.y, diag, 2e-3), "t=0.5 child near the diagonal");

    // Looping wraps: t = 2 is frame 0 again.
    evaluator.evaluate(skeleton, clip, 2.0f, /*loop=*/true);
    Vector3 pw = joint_position(evaluator, 1);
    check(nearly(pw.x, 1.0) && nearly(pw.y, 0.0), "loop wrap returns to (1,0,0)");

    // AnimationPlayer advances its cursor deterministically.
    AnimationPlayer player;
    player.clip = clip_id;
    player.skeleton = skeleton_id;
    player.speed = 2.0f;
    for (int i = 0; i < 3; ++i)
        player.advance(0.25f); // 3 steps * 0.25s * 2x = 1.5s
    check(nearly(player.time, 1.5), "player cursor advances by speed*dt");

    // Direct ClipView.sample lands exactly on an authored frame.
    std::vector<Vector3f> st(2);
    std::vector<Quaternionf> sr(2);
    std::vector<Vector3f> ss(2);
    clip.sample(1.0f, false, st.data(), sr.data(), ss.data());
    check(nearly(sr[0].z, turn.z, 1e-4) && nearly(sr[0].w, turn.w, 1e-4), "sample hits frame 1 rotation");

    if (failures != 0)
    {
        std::printf("[clip_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[clip_demo] OK — clip cook/load/sample + evaluator palette verified\n");
    return 0;
}
