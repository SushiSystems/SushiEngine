/**************************************************************************/
/* ik_demo.cpp                                                           */
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

// The phase-A6 IK / pose-modifier stack, worked and self-checked headlessly. Each solver is
// driven directly over a bind-pose PoseModifierContext (isolating the solver from the animator)
// and checked numerically: two-bone reaches its target and keeps both bone lengths; look-at aims
// the tip's forward axis at its target; FABRIK converges a long chain onto a reachable target;
// foot placement rays to a ground plane and plants the ankle on it, re-oriented to the normal.
// A final case runs a two-bone solver through the AnimatorEvaluator's modifier stack to prove
// the stack integration (compose -> modifiers -> palette).

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
            std::printf("[ik_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    // Builds a joint chain: joint i is the child of i-1, offset from its parent by `bone`.
    AssetId build_chain(AnimationDatabase& database, std::uint32_t count, Vector3f bone)
    {
        SkeletonDesc desc;
        for (std::uint32_t i = 0; i < count; ++i)
        {
            JointDesc joint;
            joint.name = std::string("j") + std::to_string(i);
            joint.parent = static_cast<int>(i) - 1;
            joint.bind_translation = i == 0 ? Vector3f{0, 0, 0} : bone;
            desc.joints.push_back(joint);
        }
        std::vector<std::byte> blob;
        build_skeleton_blob(desc, blob);
        return database.add_skeleton(std::move(blob));
    }

    // A mutable bind-pose context over a skeleton, for driving a solver directly.
    struct PoseScratch
    {
        std::vector<Vector3f> translations;
        std::vector<Quaternionf> rotations;
        std::vector<Vector3f> scales;
        std::vector<Mat4> model;
        SkeletonView skeleton;

        explicit PoseScratch(const SkeletonView& view) : skeleton(view)
        {
            const std::uint32_t n = view.joint_count;
            translations.resize(n);
            rotations.resize(n);
            scales.resize(n);
            model.resize(n);
            for (std::uint32_t j = 0; j < n; ++j)
            {
                translations[j] = view.bind_translations[j];
                rotations[j] = view.bind_rotations[j];
                scales[j] = view.bind_scales[j];
            }
            recompose();
        }

        void recompose()
        {
            compose_model(skeleton, translations.data(), rotations.data(), scales.data(),
                          model.data());
        }

        PoseModifierContext context()
        {
            PoseModifierContext ctx;
            ctx.skeleton = skeleton;
            ctx.local_translations = translations.data();
            ctx.local_rotations = rotations.data();
            ctx.local_scales = scales.data();
            ctx.model = model.data();
            return ctx;
        }

        Vector3 position(std::uint32_t joint) const
        {
            return Vector3{model[joint].m[12], model[joint].m[13], model[joint].m[14]};
        }
    };

    // A flat ground plane at a height, for the foot-placement ray provider.
    struct FlatGround : IPoseTaskContext
    {
        Scalar height = 0.0;
        Vector3 normal{0.0, 1.0, 0.0};
        bool raycast(const Vector3& origin, const Vector3& direction, Vector3& out_hit,
                     Vector3& out_normal) const override
        {
            if (direction.y >= 0.0)
                return false;
            const Scalar t = (height - origin.y) / direction.y;
            if (t < 0.0)
                return false;
            out_hit = Vector3{origin.x + direction.x * t, origin.y + direction.y * t,
                              origin.z + direction.z * t};
            out_normal = normal;
            return true;
        }
    };
}

int main()
{
    AnimationDatabase database;

    // --- Two-bone: reach a target, keep both bone lengths -----------------------------
    {
        const AssetId id = build_chain(database, 3, Vector3f{1, 0, 0}); // shoulder-elbow-hand, reach 2
        PoseScratch scratch(database.skeleton(id));

        TwoBoneIk solver;
        solver.upper = 0;
        solver.mid = 1;
        solver.tip = 2;
        solver.target = Vector3{1.0, 1.0, 0.0}; // distance ~1.414, in reach
        solver.pole = Vector3{1.0, 2.0, 0.0};   // bend the elbow up (+y)
        solver.weight = 1.0f;
        PoseModifierContext ctx = scratch.context();
        solver.solve(ctx);

        const Vector3 hand = scratch.position(2);
        const Scalar reach_error = length(hand - solver.target);
        check(reach_error < 1e-3, "two-bone hand reaches the target");
        const Scalar upper_len = length(scratch.position(1) - scratch.position(0));
        const Scalar lower_len = length(scratch.position(2) - scratch.position(1));
        check(std::fabs(upper_len - 1.0) < 1e-3 && std::fabs(lower_len - 1.0) < 1e-3,
              "two-bone preserves both bone lengths");
        check(scratch.position(1).y > 0.1, "two-bone bends the elbow toward the pole (+y)");
    }

    // --- Two-bone: an out-of-reach target soft-clamps to full extension ---------------
    {
        const AssetId id = build_chain(database, 3, Vector3f{1, 0, 0});
        PoseScratch scratch(database.skeleton(id));
        TwoBoneIk solver;
        solver.upper = 0;
        solver.mid = 1;
        solver.tip = 2;
        solver.target = Vector3{10.0, 0.0, 0.0}; // far out of reach (reach is 2)
        solver.pole = Vector3{1.0, 1.0, 0.0};
        PoseModifierContext ctx = scratch.context();
        solver.solve(ctx);
        const Scalar hand_distance = length(scratch.position(2) - scratch.position(0));
        check(std::fabs(hand_distance - 2.0) < 2e-2, "two-bone out-of-reach extends to near full length");
    }

    // --- Look-at: aim the tip's forward axis at a target ------------------------------
    {
        const AssetId id = build_chain(database, 3, Vector3f{0, 1, 0}); // base-neck-head up the y axis
        PoseScratch scratch(database.skeleton(id));

        LookAtIk solver;
        solver.joint_count = 2;
        solver.joints[0] = 1; // neck
        solver.joints[1] = 2; // head (tip)
        solver.weights[0] = 0.4f;
        solver.weights[1] = 0.6f;
        solver.forward_axis = Vector3{0.0, 0.0, 1.0}; // head looks +z at rest
        solver.target = Vector3{3.0, 2.0, 4.0};
        solver.weight = 1.0f;
        PoseModifierContext ctx = scratch.context();
        solver.solve(ctx);

        const Quaternion head = quaternion_from_matrix(scratch.model[2]);
        const Vector3 forward = normalize(rotate(head, solver.forward_axis));
        const Vector3 desired = normalize(solver.target - scratch.position(2));
        check(dot(forward, desired) > 0.999, "look-at aims the head's forward at the target");
    }

    // --- FABRIK: converge a long chain onto a reachable target ------------------------
    {
        const AssetId id = build_chain(database, 5, Vector3f{1, 0, 0}); // 4 bones, reach 4
        PoseScratch scratch(database.skeleton(id));

        ChainIk solver;
        solver.joint_count = 5;
        for (std::uint32_t i = 0; i < 5; ++i)
            solver.joints[i] = i;
        solver.target = Vector3{2.0, 2.0, 0.0}; // distance ~2.83, in reach
        solver.iterations = 16;
        solver.tolerance = 1e-4f;
        PoseModifierContext ctx = scratch.context();
        solver.solve(ctx);

        const Scalar tip_error = length(scratch.position(4) - solver.target);
        check(tip_error < 1e-2, "FABRIK tip converges onto the target");
    }

    // --- Foot placement: ray to the ground and plant the ankle ------------------------
    {
        // Leg down the -y axis: hip at (0,2,0), knee (0,1,0), ankle (0,0,0).
        SkeletonDesc desc;
        JointDesc hip; hip.name = "hip"; hip.parent = -1; hip.bind_translation = Vector3f{0, 2, 0};
        JointDesc knee; knee.name = "knee"; knee.parent = 0; knee.bind_translation = Vector3f{0, -1, 0};
        JointDesc ankle; ankle.name = "ankle"; ankle.parent = 1; ankle.bind_translation = Vector3f{0, -1, 0};
        desc.joints = {hip, knee, ankle};
        std::vector<std::byte> blob;
        build_skeleton_blob(desc, blob);
        const AssetId id = database.add_skeleton(std::move(blob));
        PoseScratch scratch(database.skeleton(id));

        FlatGround ground;
        ground.height = 0.3; // above the rest foot, so the foot lifts to it (in reach)

        FootPlacementIk solver;
        solver.hip = 0;
        solver.knee = 1;
        solver.ankle = 2;
        solver.ground = &ground;
        solver.pole = Vector3{0.0, 0.0, 1.0};
        solver.foot_height = 0.0f;
        solver.ray_height = 1.0f;
        solver.weight = 1.0f;
        PoseModifierContext ctx = scratch.context();
        solver.solve(ctx);

        const Vector3 planted = scratch.position(2);
        check(std::fabs(planted.y - 0.3) < 1e-2, "foot placement plants the ankle on the ground plane");
    }

    // --- Stack integration: a two-bone modifier through the AnimatorEvaluator ---------
    {
        const AssetId id = build_chain(database, 3, Vector3f{1, 0, 0});
        const SkeletonView skeleton = database.skeleton(id);

        // A trivial controller: one layer, one state, one bind-pose clip.
        ClipDesc clip;
        clip.joint_count = 3;
        clip.frame_count = 1;
        clip.sample_rate = 30.0f;
        clip.translations = {skeleton.bind_translations[0], skeleton.bind_translations[1],
                             skeleton.bind_translations[2]};
        clip.rotations.assign(3, Quaternionf{0, 0, 0, 1});
        clip.scales.assign(3, Vector3f{1, 1, 1});
        std::vector<std::byte> clip_blob;
        build_clip_blob(clip, clip_blob);
        const AssetId clip_id = database.add_clip(std::move(clip_blob));

        ControllerDesc controller_desc;
        LayerDesc layer;
        layer.name = "base";
        layer.default_state = "Rest";
        StateDesc state;
        state.name = "Rest";
        state.clip = clip_id;
        layer.states = {state};
        controller_desc.layers.push_back(layer);
        std::vector<std::byte> controller_blob;
        compile_controller_blob(controller_desc, controller_blob);
        const AssetId controller_id = database.add_controller(std::move(controller_blob));
        const ControllerView controller = database.controller(controller_id);

        AnimatorInstance animator{};
        animator.controller = controller_id;
        animator.skeleton = id;
        animator_step(controller, database, animator, 0.0f);

        TwoBoneIk solver;
        solver.upper = 0;
        solver.mid = 1;
        solver.tip = 2;
        solver.target = Vector3{1.0, 1.0, 0.0};
        solver.pole = Vector3{1.0, 2.0, 0.0};
        const IPoseModifier* stack[] = {&solver};

        AnimatorEvaluator evaluator;
        evaluator.evaluate(controller, database, animator, skeleton, stack, 1);
        const Mat4& hand = evaluator.model()[2];
        const Vector3 hand_position{hand.m[12], hand.m[13], hand.m[14]};
        check(length(hand_position - solver.target) < 1e-3,
              "evaluator pose-modifier stack reaches the target");
    }

    if (failures != 0)
    {
        std::printf("[ik_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[ik_demo] OK — two-bone reach + clamp, look-at aim, FABRIK convergence, foot "
                "placement, and evaluator stack integration all verified\n");
    return 0;
}
