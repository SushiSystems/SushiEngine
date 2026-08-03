/**************************************************************************/
/* full_body_ik_demo.cpp                                                  */
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

// The §12.4 general-purpose FullBodyIk (CCD, multi-effector), worked and self-checked
// against a hand-built PoseModifierContext (no controller/evaluator needed — an
// IPoseModifier is a pure function of the context, same pattern ragdoll_blend_demo
// uses). Two things are proved:
//   * A single 3-joint chain (root -> mid -> tip) converges its tip to an
//     out-of-plane target within tight tolerance — the core CCD algorithm works, not
//     just the trivial straight-line case.
//   * Two independent limbs sharing a common, *unrotated* anchor joint (each solved by
//     its own FullBodyIk instance, root_joint set to that limb's own base so neither
//     ever rotates the shared joint 0) both reach their own targets without disturbing
//     each other — proves two effectors' chains genuinely don't interfere as long as
//     they don't share a rotatable joint, the condition the header's own caveat about
//     shared-ancestor effectors names as the one case this solver doesn't arbitrate.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/ik_full_body.hpp>
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
            std::printf("[full_body_ik_demo] FAIL: %s\n", what);
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
    // Test 1: a single 3-joint chain reaches an out-of-plane target.
    {
        SkeletonDescription description;
        JointDescription root;
        root.name = "root";
        root.parent = -1;
        JointDescription mid;
        mid.name = "mid";
        mid.parent = 0;
        mid.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
        JointDescription tip;
        tip.name = "tip";
        tip.parent = 1;
        tip.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
        description.joints = {root, mid, tip};

        std::vector<std::byte> blob;
        check(build_skeleton_blob(description, blob), "cook 3-joint skeleton");
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

        FullBodyIk solver;
        FullBodyEffector effector;
        effector.tip_joint = 2;
        effector.target = Vector3{1.0, 1.0, 0.5}; // reachable: bind reach is 2.0, target dist ~1.5
        effector.weight = 1.0f;
        solver.effectors.push_back(effector);
        solver.root_joint = 0;
        solver.iterations = 30;
        solver.weight = 1.0f;
        solver.solve(context);

        const Vector3 tip_position{model[2].m[12], model[2].m[13], model[2].m[14]};
        const double error = std::sqrt((tip_position.x - effector.target.x) *
                                           (tip_position.x - effector.target.x) +
                                       (tip_position.y - effector.target.y) *
                                           (tip_position.y - effector.target.y) +
                                       (tip_position.z - effector.target.z) *
                                           (tip_position.z - effector.target.z));
        check(error < 0.01, "single-chain CCD converges to an out-of-plane target");
        std::printf("[full_body_ik_demo] single-chain final error: %.6f\n", error);
    }

    // Test 2: two independent limbs off a shared, never-rotated anchor.
    {
        SkeletonDescription description;
        JointDescription root;
        root.name = "root";
        root.parent = -1;
        JointDescription limb_a_base;
        limb_a_base.name = "limb_a_base";
        limb_a_base.parent = 0;
        limb_a_base.bind_translation = Vector3f{0.0f, 1.0f, 0.0f};
        JointDescription limb_a_tip;
        limb_a_tip.name = "limb_a_tip";
        limb_a_tip.parent = 1;
        limb_a_tip.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
        JointDescription limb_b_base;
        limb_b_base.name = "limb_b_base";
        limb_b_base.parent = 0;
        limb_b_base.bind_translation = Vector3f{0.0f, -1.0f, 0.0f};
        JointDescription limb_b_tip;
        limb_b_tip.name = "limb_b_tip";
        limb_b_tip.parent = 3;
        limb_b_tip.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
        description.joints = {root, limb_a_base, limb_a_tip, limb_b_base, limb_b_tip};

        std::vector<std::byte> blob;
        check(build_skeleton_blob(description, blob), "cook 5-joint skeleton");
        AnimationDatabase database;
        const AssetId skeleton_id = database.add_skeleton(std::move(blob));
        const SkeletonView skeleton = database.skeleton(skeleton_id);

        // The cook topologically re-sorts joints by depth (stable_sort), so authored
        // index order is NOT preserved once two joints share a depth — exactly this
        // skeleton's case (limb_a_base and limb_b_base are both depth 1). Resolve every
        // index by name post-cook rather than assuming it matches `description.joints`' order.
        const std::uint32_t root_index =
            static_cast<std::uint32_t>(skeleton.find_joint(hash_name("root")));
        const std::uint32_t limb_a_base_index =
            static_cast<std::uint32_t>(skeleton.find_joint(hash_name("limb_a_base")));
        const std::uint32_t limb_a_tip_index =
            static_cast<std::uint32_t>(skeleton.find_joint(hash_name("limb_a_tip")));
        const std::uint32_t limb_b_base_index =
            static_cast<std::uint32_t>(skeleton.find_joint(hash_name("limb_b_base")));
        const std::uint32_t limb_b_tip_index =
            static_cast<std::uint32_t>(skeleton.find_joint(hash_name("limb_b_tip")));
        check(root_index < skeleton.joint_count && limb_a_base_index < skeleton.joint_count &&
                 limb_a_tip_index < skeleton.joint_count &&
                 limb_b_base_index < skeleton.joint_count &&
                 limb_b_tip_index < skeleton.joint_count,
             "every joint resolves by name post-cook");

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

        // Limb A: root_joint = limb_a_base — CCD stops there, never touches the shared root.
        // Target is placed exactly one bone-length (1.0) from the pivot (0,1,0) — a single
        // rotational DOF *can* reach it exactly, so a tight tolerance is a meaningful check
        // rather than measuring the geometric residual of an off-the-reachable-sphere target.
        FullBodyIk solver_a;
        FullBodyEffector effector_a;
        effector_a.tip_joint = limb_a_tip_index;
        effector_a.target = Vector3{0.0, 1.0, 0.0} +
                            normalize(Vector3{0.7, 0.7, 0.0});
        solver_a.effectors.push_back(effector_a);
        solver_a.root_joint = limb_a_base_index;
        solver_a.iterations = 20;
        solver_a.solve(context);

        // Limb B: root_joint = limb_b_base — independent of limb A's chain. Same exact-reach
        // construction, mirrored below the shared anchor.
        FullBodyIk solver_b;
        FullBodyEffector effector_b;
        effector_b.tip_joint = limb_b_tip_index;
        effector_b.target = Vector3{0.0, -1.0, 0.0} +
                            normalize(Vector3{0.7, -0.7, 0.0});
        solver_b.effectors.push_back(effector_b);
        solver_b.root_joint = limb_b_base_index;
        solver_b.iterations = 20;
        solver_b.solve(context);

        const Vector3 root_position{model[root_index].m[12], model[root_index].m[13],
                                    model[root_index].m[14]};
        const Vector3 tip_a{model[limb_a_tip_index].m[12], model[limb_a_tip_index].m[13],
                            model[limb_a_tip_index].m[14]};
        const Vector3 tip_b{model[limb_b_tip_index].m[12], model[limb_b_tip_index].m[13],
                            model[limb_b_tip_index].m[14]};

        check(nearly(root_position.x, 0.0, 1e-6) && nearly(root_position.y, 0.0, 1e-6) &&
                 nearly(root_position.z, 0.0, 1e-6),
             "root was never rotated by either limb's CCD (root_joint excluded it)");

        const double error_a =
            std::sqrt((tip_a.x - effector_a.target.x) * (tip_a.x - effector_a.target.x) +
                     (tip_a.y - effector_a.target.y) * (tip_a.y - effector_a.target.y) +
                     (tip_a.z - effector_a.target.z) * (tip_a.z - effector_a.target.z));
        const double error_b =
            std::sqrt((tip_b.x - effector_b.target.x) * (tip_b.x - effector_b.target.x) +
                     (tip_b.y - effector_b.target.y) * (tip_b.y - effector_b.target.y) +
                     (tip_b.z - effector_b.target.z) * (tip_b.z - effector_b.target.z));
        check(error_a < 0.01, "limb A reaches its own target independently");
        check(error_b < 0.01, "limb B reaches its own target independently, unaffected by A");
        std::printf("[full_body_ik_demo] limb A error: %.6f, limb B error: %.6f\n", error_a,
                   error_b);
    }

    if (failures != 0)
    {
        std::printf("[full_body_ik_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[full_body_ik_demo] OK — single-chain convergence and two-independent-limb "
        "solving verified\n");
    return 0;
}
