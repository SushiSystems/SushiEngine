/**************************************************************************/
/* jiggle_bone_demo.cpp                                                  */
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

// The §12.4 jiggle-bone / physics-driven secondary motion solver, worked and self-checked
// against a hand-built PoseModifierContext — same pattern ragdoll_blend_demo and
// full_body_ik_demo use, no controller/evaluator needed since IPoseModifier is a pure
// function (plus, here, deliberately mutable spring state) of the context. A two-joint
// "root -> jiggle" rig (the jiggle joint hangs 1 unit below root, like a ponytail tip)
// proves:
//   * With the root held still, the jiggle joint settles to (and stays at) its rigid rest
//     position — a spring with nothing disturbing it should not drift or oscillate forever.
//   * The joint-to-parent distance stays pinned to the bind bone length every frame — the
//     Verlet step's distance re-projection is actually being applied, not just the spring.
//   * When the root is suddenly stepped sideways (a parent lurch), the jiggle joint does NOT
//     snap instantly to its new rigid position — real lag, the entire point of a jiggle bone.
//   * Given enough further frames with the root held at its new position, the joint
//     re-settles close to the new rigid rest position — the spring actually converges, it
//     does not just lag forever or diverge.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/jiggle_bone.hpp>
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
            std::printf("[jiggle_bone_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(double a, double b, double eps) { return std::fabs(a - b) <= eps; }

    void bind_pose_context(const SkeletonView& skeleton, std::vector<Vector3f>& t,
                           std::vector<Quaternionf>& r, std::vector<Vector3f>& s,
                           std::vector<Matrix4>& model)
    {
        t.assign(skeleton.joint_count, Vector3f{});
        r.assign(skeleton.joint_count, Quaternionf{});
        s.assign(skeleton.joint_count, Vector3f{1, 1, 1});
        model.assign(skeleton.joint_count, Matrix4{});
        for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
        {
            t[j] = skeleton.bind_translations[j];
            r[j] = skeleton.bind_rotations[j];
            s[j] = skeleton.bind_scales[j];
        }
        compose_model(skeleton, t.data(), r.data(), s.data(), model.data());
    }
}

int main()
{
    SkeletonDescription description;
    JointDescription root;
    root.name = "root";
    root.parent = -1;
    JointDescription tip;
    tip.name = "jiggle";
    tip.parent = 0;
    tip.bind_translation = Vector3f{0.0f, -1.0f, 0.0f};
    description.joints = {root, tip};

    std::vector<std::byte> blob;
    check(build_skeleton_blob(description, blob), "cook 2-joint skeleton");
    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(blob));
    const SkeletonView skeleton = database.skeleton(skeleton_id);

    std::vector<Vector3f> t;
    std::vector<Quaternionf> r;
    std::vector<Vector3f> s;
    std::vector<Matrix4> model;
    bind_pose_context(skeleton, t, r, s, model);
    PoseModifierContext context;
    context.skeleton = skeleton;
    context.local_translations = t.data();
    context.local_rotations = r.data();
    context.local_scales = s.data();
    context.model = model.data();

    JiggleBone jiggle;
    JiggleJoint joint;
    joint.joint = 1;
    joint.stiffness = 200.0f;
    joint.damping = 0.85f;
    joint.gravity = Vector3{0.0, 0.0, 0.0};
    jiggle.joints.push_back(joint);
    const float dt = 1.0f / 60.0f;
    jiggle.set_delta_time(dt);

    auto joint_position = [&]() {
        return Vector3{model[1].m[12], model[1].m[13], model[1].m[14]};
    };
    auto root_position = [&]() {
        return Vector3{model[0].m[12], model[0].m[13], model[0].m[14]};
    };

    const Quaternionf root_identity{0.0f, 0.0f, 0.0f, 1.0f};
    // Every real caller (AnimatorEvaluator::evaluate) reseeds the local pose from the
    // animated clips and recomposes from scratch before running the pose-modifier stack, so a
    // jiggle correction from frame N never leaks into frame N+1's "rest" reading. This harness
    // has no evaluator, so it reproduces that contract by hand: reset the root's rotation to
    // its true (un-jiggled) animated value and recompose before every solve() call below.
    auto animate_frame = [&]() {
        r[0] = root_identity;
        context.recompose();
    };

    // --- Phase 1: root held still, the spring should settle to and stay at rest. ---
    for (int i = 0; i < 60; ++i)
    {
        animate_frame();
        jiggle.solve(context);
    }

    const Vector3 rest_position{0.0, -1.0, 0.0};
    const Vector3 settled = joint_position();
    const double settle_error = length(settled - rest_position);
    check(settle_error < 1e-3, "with the root held still, the joint settles to its rest position");

    const double bone_length_at_rest = length(joint_position() - root_position());
    check(nearly(bone_length_at_rest, 1.0, 1e-4),
         "the joint-to-parent distance stays pinned to the bind bone length at rest");

    std::printf("[jiggle_bone_demo] settle error after 60 frames: %.6f\n", settle_error);

    // --- Phase 2: step the root sideways (a sudden parent lurch) and check for real lag. ---
    t[0] = Vector3f{2.0f, 0.0f, 0.0f};
    animate_frame();
    jiggle.solve(context);

    const Vector3 new_rigid_rest{2.0, -1.0, 0.0}; // where the joint would be with zero lag
    const double lag_after_one_frame = length(joint_position() - new_rigid_rest);
    check(lag_after_one_frame > 0.05,
         "one frame after the root steps, the joint has NOT snapped to the new rigid position");
    std::printf("[jiggle_bone_demo] distance from new rigid rest after 1 frame: %.6f\n",
               lag_after_one_frame);

    const double bone_length_after_step = length(joint_position() - root_position());
    check(nearly(bone_length_after_step, 1.0, 1e-4),
         "the distance constraint holds immediately even mid-lurch");

    // --- Phase 3: hold the root at its new position and let the spring re-settle. ---
    for (int i = 0; i < 300; ++i)
    {
        animate_frame();
        jiggle.solve(context);
    }

    const double resettle_error = length(joint_position() - new_rigid_rest);
    check(resettle_error < 1e-2,
         "given enough frames at the new root position, the joint re-settles close to rest");
    std::printf("[jiggle_bone_demo] resettle error after 300 more frames: %.6f\n", resettle_error);

    if (failures != 0)
    {
        std::printf("[jiggle_bone_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[jiggle_bone_demo] OK — settle-to-rest, distance constraint, lag-on-lurch, and "
        "re-settle all verified\n");
    return 0;
}
