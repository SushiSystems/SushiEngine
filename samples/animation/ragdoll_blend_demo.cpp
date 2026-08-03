/**************************************************************************/
/* ragdoll_blend_demo.cpp                                                 */
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

// The §12.4/§11 ragdoll-blend pose modifier, worked and self-checked. Same root+child
// rig as clip_demo.cpp (child bind-translated 1 unit down +X). Four things are proved
// directly against a hand-built PoseModifierContext (no controller/evaluator needed —
// RagdollBlend is a pure function of the context, per IPoseModifier's contract):
//   * weight 0 leaves the pose untouched (pure animation).
//   * weight 1 on the child joint places it exactly at the physics-supplied position.
//   * weight 0.5 blends the child halfway between its animated and physics positions.
//   * a target on the PARENT joint (root) still correctly moves the untouched CHILD's
//     model-space position after recompose() — proving the local-pose-edit-then-
//     recompose approach cascades to descendants, not just the named joint (the
//     property a naive "overwrite context.model[joint] directly" implementation would
//     get wrong for anything but a leaf).

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/ragdoll_blend.hpp>
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
            std::printf("[ragdoll_blend_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }

    Vector3 model_position(const Matrix4& m) { return Vector3{m.m[12], m.m[13], m.m[14]}; }
}

int main()
{
    SkeletonDescription skeleton_description;
    JointDescription root;
    root.name = "root";
    root.parent = -1;
    JointDescription child;
    child.name = "child";
    child.parent = 0;
    child.bind_translation = Vector3f{1.0f, 0.0f, 0.0f};
    skeleton_description.joints = {root, child};

    std::vector<std::byte> skeleton_blob;
    check(build_skeleton_blob(skeleton_description, skeleton_blob), "cook skeleton");

    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const SkeletonView skeleton = database.skeleton(skeleton_id);

    auto bind_pose_context = [&](std::vector<Vector3f>& t, std::vector<Quaternionf>& r,
                                 std::vector<Vector3f>& s, std::vector<Matrix4>& model)
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
    };

    // weight 0: pure animation, no change.
    {
        std::vector<Vector3f> t;
        std::vector<Quaternionf> r;
        std::vector<Vector3f> s;
        std::vector<Matrix4> model;
        bind_pose_context(t, r, s, model);
        PoseModifierContext context;
        context.skeleton = skeleton;
        context.local_translations = t.data();
        context.local_rotations = r.data();
        context.local_scales = s.data();
        context.model = model.data();

        RagdollBlend blend;
        RagdollJointTarget target;
        target.joint = 1;
        target.object_space_transform = translation(Vector3{5, 0, 0});
        target.weight = 0.0f;
        blend.targets.push_back(target);
        blend.solve(context);

        const Vector3 p = model_position(model[1]);
        check(nearly(p.x, 1.0) && nearly(p.y, 0.0) && nearly(p.z, 0.0),
             "weight 0 leaves the child at its animated (1,0,0)");
    }

    // weight 1: fully physics-driven.
    {
        std::vector<Vector3f> t;
        std::vector<Quaternionf> r;
        std::vector<Vector3f> s;
        std::vector<Matrix4> model;
        bind_pose_context(t, r, s, model);
        PoseModifierContext context;
        context.skeleton = skeleton;
        context.local_translations = t.data();
        context.local_rotations = r.data();
        context.local_scales = s.data();
        context.model = model.data();

        RagdollBlend blend;
        RagdollJointTarget target;
        target.joint = 1;
        target.object_space_transform = translation(Vector3{5, 0, 0});
        target.weight = 1.0f;
        blend.targets.push_back(target);
        blend.solve(context);

        const Vector3 p = model_position(model[1]);
        check(nearly(p.x, 5.0) && nearly(p.y, 0.0) && nearly(p.z, 0.0),
             "weight 1 places the child exactly at the physics target (5,0,0)");
    }

    // weight 0.5: halfway blend.
    {
        std::vector<Vector3f> t;
        std::vector<Quaternionf> r;
        std::vector<Vector3f> s;
        std::vector<Matrix4> model;
        bind_pose_context(t, r, s, model);
        PoseModifierContext context;
        context.skeleton = skeleton;
        context.local_translations = t.data();
        context.local_rotations = r.data();
        context.local_scales = s.data();
        context.model = model.data();

        RagdollBlend blend;
        RagdollJointTarget target;
        target.joint = 1;
        target.object_space_transform = translation(Vector3{5, 0, 0});
        target.weight = 0.5f;
        blend.targets.push_back(target);
        blend.solve(context);

        const Vector3 p = model_position(model[1]);
        check(nearly(p.x, 3.0) && nearly(p.y, 0.0) && nearly(p.z, 0.0),
             "weight 0.5 blends the child halfway to (3,0,0)");
    }

    // targeting the PARENT cascades correctly to the untouched CHILD.
    {
        std::vector<Vector3f> t;
        std::vector<Quaternionf> r;
        std::vector<Vector3f> s;
        std::vector<Matrix4> model;
        bind_pose_context(t, r, s, model);
        PoseModifierContext context;
        context.skeleton = skeleton;
        context.local_translations = t.data();
        context.local_rotations = r.data();
        context.local_scales = s.data();
        context.model = model.data();

        RagdollBlend blend;
        RagdollJointTarget target;
        target.joint = 0; // root, the child's parent
        target.object_space_transform = translation(Vector3{10, 0, 0});
        target.weight = 1.0f;
        blend.targets.push_back(target);
        blend.solve(context);

        const Vector3 root_p = model_position(model[0]);
        const Vector3 child_p = model_position(model[1]);
        check(nearly(root_p.x, 10.0), "root moves exactly to the physics target (10,0,0)");
        // The child was never named as a target, but its parent moved: recompose() must
        // cascade root's new transform through the child's still-animated local offset
        // (still +1 on X from the root), landing it at (11,0,0), not left behind at (1,0,0).
        check(nearly(child_p.x, 11.0),
             "untouched child cascades with its moved parent to (11,0,0)");
    }

    if (failures != 0)
    {
        std::printf("[ragdoll_blend_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[ragdoll_blend_demo] OK — weight blending and parent-to-child cascade verified\n");
    return 0;
}
