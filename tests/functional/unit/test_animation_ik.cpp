/**************************************************************************/
/* test_animation_ik.cpp                                                 */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
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

// Unit_AnimationIk: the pose-modifier stack (design §5.3) — the analytic two-bone solver,
// look-at, FABRIK, multi-effector CCD, ragdoll blending and jiggle bones, each as an
// `IPoseModifier` over a model-space pose.
//
// Two things here are worth more than the convergence numbers. First, §10's LSP claim: every
// solver must be substitutable in any stack position, which means a zero weight has to be an
// exact no-op and the stack must compose in order — asserted directly rather than assumed.
// Second, the failure modes these solvers have actually shipped with: a solver that moves a
// joint by rotating the joint itself (a no-op in a hierarchy, because a joint's own rotation
// only orients its children), and a solver that strands a child when it repositions a parent.
// Both are quantitative assertions below, not "did not crash" checks.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/ik_chain.hpp>
#include <SushiEngine/animation/ik_foot_placement.hpp>
#include <SushiEngine/animation/ik_full_body.hpp>
#include <SushiEngine/animation/ik_look_at.hpp>
#include <SushiEngine/animation/ik_two_bone.hpp>
#include <SushiEngine/animation/jiggle_bone.hpp>
#include <SushiEngine/animation/pose_modifier.hpp>
#include <SushiEngine/animation/ragdoll_blend.hpp>
#include <SushiEngine/animation/skeleton_blob.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    // A posable rig: owns the cooked skeleton, a mutable local pose seeded from the bind pose,
    // and the composed model matrices a solver reads and writes through.
    //
    // Joint indices are always resolved by name, never by authoring order: the cook sorts by
    // depth, so two joints authored at the same depth do not keep their authored order — the
    // exact assumption that produced garbage results in this subsystem before.
    class Rig
    {
        public:
            Rig(const std::vector<std::string>& names, const std::vector<int>& parents,
                const std::vector<Vector3f>& bind_translations)
            {
                SkeletonDesc description;
                description.joints.resize(names.size());
                for (std::size_t i = 0; i < names.size(); ++i)
                {
                    description.joints[i].name = names[i];
                    description.joints[i].parent = parents[i];
                    description.joints[i].bind_translation = bind_translations[i];
                }
                build_skeleton_blob(description, blob_);
                skeleton_ = load_skeleton_blob(blob_.data(), blob_.size());
                reset();
            }

            bool valid() const noexcept { return skeleton_.valid(); }

            /** @brief Returns the local pose to the bind pose and recomposes. */
            void reset()
            {
                const std::uint32_t count = skeleton_.joint_count;
                translations_.assign(skeleton_.bind_translations, skeleton_.bind_translations + count);
                rotations_.assign(skeleton_.bind_rotations, skeleton_.bind_rotations + count);
                scales_.assign(skeleton_.bind_scales, skeleton_.bind_scales + count);
                model_.assign(count, Mat4{});
                context_.skeleton = skeleton_;
                context_.local_translations = translations_.data();
                context_.local_rotations = rotations_.data();
                context_.local_scales = scales_.data();
                context_.model = model_.data();
                context_.recompose();
            }

            std::uint32_t joint(const char* name) const
            {
                const int index = skeleton_.find_joint(hash_name(name));
                return index >= 0 ? static_cast<std::uint32_t>(index) : 0xFFFFFFFFu;
            }

            PoseModifierContext& context() noexcept { return context_; }
            Vector3 position(const char* name) const { return context_.position(joint(name)); }

            void apply(const IPoseModifier& modifier)
            {
                modifier.solve(context_);
            }

        private:
            std::vector<std::byte> blob_;
            SkeletonView skeleton_{};
            std::vector<Vector3f> translations_;
            std::vector<Quaternionf> rotations_;
            std::vector<Vector3f> scales_;
            std::vector<Mat4> model_;
            PoseModifierContext context_{};
    };

    // A straight arm along +Y: shoulder at the origin, elbow one unit up, wrist two units up.
    Rig straight_arm()
    {
        return Rig({"shoulder", "elbow", "wrist"}, {-1, 0, 1},
                   {Vector3f{0, 0, 0}, Vector3f{0, 1, 0}, Vector3f{0, 1, 0}});
    }

    double distance(const Vector3& a, const Vector3& b)
    {
        const Vector3 d = a - b;
        return std::sqrt(static_cast<double>(d.x * d.x + d.y * d.y + d.z * d.z));
    }

    double magnitude(const Vector3& v)
    {
        return std::sqrt(static_cast<double>(v.x * v.x + v.y * v.y + v.z * v.z));
    }

    Mat4 translation_matrix(double x, double y, double z)
    {
        Mat4 matrix{};
        matrix.m[12] = static_cast<Scalar>(x);
        matrix.m[13] = static_cast<Scalar>(y);
        matrix.m[14] = static_cast<Scalar>(z);
        return matrix;
    }
}

TEST(Unit_AnimationIk, TwoBoneReachesATargetInsideItsRange)
{
    Rig rig = straight_arm();
    ASSERT_TRUE(rig.valid());
    // The arm is 2 units long straight; a target at distance sqrt(2) needs a real bend.
    TwoBoneIk solver;
    solver.upper = rig.joint("shoulder");
    solver.mid = rig.joint("elbow");
    solver.tip = rig.joint("wrist");
    solver.target = Vector3{1.0, 1.0, 0.0};
    solver.pole = Vector3{1.0, 0.5, 0.0};
    solver.weight = 1.0f;

    rig.apply(solver);
    EXPECT_LT(distance(rig.position("wrist"), solver.target), 1e-3)
        << "the analytic solver must land on a reachable target exactly";

    // Bone lengths are a hard invariant — an IK solver that stretches the rig is worse than
    // one that misses.
    EXPECT_NEAR(distance(rig.position("shoulder"), rig.position("elbow")), 1.0, 1e-6);
    EXPECT_NEAR(distance(rig.position("elbow"), rig.position("wrist")), 1.0, 1e-6);
}

TEST(Unit_AnimationIk, TwoBoneStraightensTowardAnUnreachableTargetWithoutStretching)
{
    Rig rig = straight_arm();
    TwoBoneIk solver;
    solver.upper = rig.joint("shoulder");
    solver.mid = rig.joint("elbow");
    solver.tip = rig.joint("wrist");
    solver.target = Vector3{10.0, 0.0, 0.0}; // five times the arm's reach
    solver.pole = Vector3{1.0, 1.0, 0.0};
    solver.weight = 1.0f;

    rig.apply(solver);
    const Vector3 wrist = rig.position("wrist");
    // It aims at the target and extends as far as the bones allow — no further.
    EXPECT_NEAR(distance(rig.position("shoulder"), wrist), 2.0, 2e-2)
        << "an out-of-range target must extend the chain, not stretch past its length";
    EXPECT_NEAR(distance(rig.position("shoulder"), rig.position("elbow")), 1.0, 1e-6);
    EXPECT_NEAR(distance(rig.position("elbow"), wrist), 1.0, 1e-6);
    EXPECT_GT(wrist.x, 1.5) << "the chain did not aim at the target";
}

TEST(Unit_AnimationIk, TheSolverWeightBlendsTheCorrectionAndZeroIsAnExactNoOp)
{
    // §10's LSP claim: any modifier is substitutable in any stack position, which requires a
    // zero weight to leave the pose bit-identical rather than nearly identical.
    Rig rig = straight_arm();
    const Vector3 rest = rig.position("wrist");

    TwoBoneIk solver;
    solver.upper = rig.joint("shoulder");
    solver.mid = rig.joint("elbow");
    solver.tip = rig.joint("wrist");
    solver.target = Vector3{1.0, 1.0, 0.0};
    solver.pole = Vector3{1.0, 0.5, 0.0};

    solver.weight = 0.0f;
    rig.apply(solver);
    EXPECT_EQ(rig.position("wrist").x, rest.x);
    EXPECT_EQ(rig.position("wrist").y, rest.y);
    EXPECT_EQ(rig.position("wrist").z, rest.z);

    // A partial weight lands between the animated pose and the solved one.
    solver.weight = 1.0f;
    Rig solved = straight_arm();
    solved.apply(solver);
    const Vector3 full = solved.position("wrist");

    solver.weight = 0.5f;
    Rig half = straight_arm();
    half.apply(solver);
    const Vector3 partial = half.position("wrist");
    EXPECT_LT(distance(partial, full), distance(rest, full)) << "half weight moved the wrong way";
    EXPECT_GT(distance(partial, full), 1e-4) << "half weight behaved like full weight";
}

TEST(Unit_AnimationIk, TwoBoneRefusesADegenerateChain)
{
    // The preview panel ships with all-zero joint indices until a caller configures it, so a
    // degenerate chain must be a no-op rather than a division by a zero bone length.
    Rig rig = straight_arm();
    const Vector3 rest = rig.position("wrist");
    TwoBoneIk solver;
    solver.upper = 0;
    solver.mid = 0;
    solver.tip = 0;
    solver.target = Vector3{5.0, 5.0, 5.0};
    solver.weight = 1.0f;
    rig.apply(solver);

    for (const char* name : {"shoulder", "elbow", "wrist"})
        EXPECT_FALSE(std::isnan(static_cast<double>(rig.position(name).x)))
            << "NaN escaped the degenerate chain at " << name;
    EXPECT_NEAR(distance(rig.position("wrist"), rest), 0.0, 1e-9);
}

TEST(Unit_AnimationIk, LookAtAimsTheTipAndTheConeCapsHowFar)
{
    // The head/eye solver: the tip's forward axis turns toward the target, and the cone is a
    // hard cap so a character never looks over its own shoulder past what a neck allows.
    Rig rig = Rig({"neck", "head"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{0, 1, 0}});
    ASSERT_TRUE(rig.valid());

    LookAtIk solver;
    solver.joints[0] = rig.joint("neck");
    solver.joints[1] = rig.joint("head");
    solver.weights[0] = 0.5f;
    solver.weights[1] = 0.5f;
    solver.joint_count = 2;
    solver.forward_axis = Vector3{0.0, 0.0, 1.0};
    solver.target = Vector3{5.0, 1.0, 0.0}; // off to the side
    solver.weight = 1.0f;

    // Unclamped, the head's forward axis should end up pointing much closer to the target.
    const Quaternion before = rig.context().rotation(rig.joint("head"));
    const Vector3 aim_before = rotate(before, Vector3{0.0, 0.0, 1.0});
    rig.apply(solver);
    const Quaternion after = rig.context().rotation(rig.joint("head"));
    const Vector3 aim_after = rotate(after, Vector3{0.0, 0.0, 1.0});
    const Vector3 to_target = solver.target - rig.position("head");
    const double want = magnitude(to_target);
    ASSERT_GT(want, 1e-6);
    const double dot_before =
        static_cast<double>(aim_before.x * to_target.x + aim_before.y * to_target.y +
                            aim_before.z * to_target.z) / want;
    const double dot_after =
        static_cast<double>(aim_after.x * to_target.x + aim_after.y * to_target.y +
                            aim_after.z * to_target.z) / want;
    EXPECT_GT(dot_after, dot_before) << "look-at turned away from its target";

    // With a tight cone the same request must be clamped, so it turns strictly less far.
    Rig clamped = Rig({"neck", "head"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{0, 1, 0}});
    LookAtIk narrow = solver;
    narrow.cone = 0.05f;
    clamped.apply(narrow);
    const Vector3 aim_clamped =
        rotate(clamped.context().rotation(clamped.joint("head")), Vector3{0.0, 0.0, 1.0});
    const double dot_clamped =
        static_cast<double>(aim_clamped.x * to_target.x + aim_clamped.y * to_target.y +
                            aim_clamped.z * to_target.z) / want;
    EXPECT_LT(dot_clamped, dot_after) << "the cone did not cap the aim";
    EXPECT_GE(dot_clamped, dot_before - 1e-9) << "the clamp turned it the wrong way";
}

TEST(Unit_AnimationIk, FabrikConvergesAndItsIterationCapIsRealTierKnob)
{
    // The tier table (§6.6) buys quality with iterations, so the cap has to actually bound the
    // work: one iteration must be measurably worse than many, or the knob does nothing.
    const std::vector<std::string> names = {"j0", "j1", "j2", "j3"};
    const std::vector<int> parents = {-1, 0, 1, 2};
    const std::vector<Vector3f> binds = {Vector3f{0, 0, 0}, Vector3f{0, 1, 0}, Vector3f{0, 1, 0},
                                        Vector3f{0, 1, 0}};

    const auto solve_with = [&](std::uint32_t iterations)
    {
        Rig rig(names, parents, binds);
        ChainIk solver;
        solver.joint_count = 4;
        for (std::uint32_t i = 0; i < 4; ++i)
            solver.joints[i] = rig.joint(names[i].c_str());
        solver.target = Vector3{1.5, 1.5, 0.5};
        solver.iterations = iterations;
        solver.tolerance = 1e-6f;
        solver.weight = 1.0f;
        rig.apply(solver);
        return distance(rig.position("j3"), solver.target);
    };

    const double one_pass = solve_with(1);
    const double many_passes = solve_with(32);
    EXPECT_LT(many_passes, 1e-3) << "FABRIK did not converge in 32 passes";
    EXPECT_GT(one_pass, many_passes) << "the iteration cap has no effect";
}

TEST(Unit_AnimationIk, FullBodyCcdConvergesToAnOutOfPlaneTarget)
{
    const std::vector<std::string> names = {"root", "mid", "tip"};
    Rig rig(names, {-1, 0, 1}, {Vector3f{0, 0, 0}, Vector3f{0, 1, 0}, Vector3f{0, 1, 0}});
    ASSERT_TRUE(rig.valid());

    FullBodyIk solver;
    solver.root_joint = rig.joint("root");
    solver.iterations = 40;
    solver.weight = 1.0f;
    FullBodyEffector effector;
    effector.tip_joint = rig.joint("tip");
    effector.target = Vector3{1.0, 1.0, 0.5}; // inside reach, off the bind plane
    effector.weight = 1.0f;
    solver.effectors.push_back(effector);

    rig.apply(solver);
    EXPECT_LT(distance(rig.position("tip"), effector.target), 1e-3);
    EXPECT_NEAR(distance(rig.position("root"), rig.position("mid")), 1.0, 1e-4);
    EXPECT_NEAR(distance(rig.position("mid"), rig.position("tip")), 1.0, 1e-4);
}

TEST(Unit_AnimationIk, TwoIndependentLimbsSolveWithoutDisturbingEachOther)
{
    // The case CCD gets wrong when the effectors share a rotatable ancestor: two limbs off one
    // anchor, each solved by its own instance with `root_joint` set to exclude the anchor, must
    // both reach their targets. The header documents that a *shared rotatable* ancestor is not
    // arbitrated — this pins the arrangement that is supposed to work.
    const std::vector<std::string> names = {"anchor", "left_upper", "left_tip", "right_upper",
                                           "right_tip"};
    Rig rig(names, {-1, 0, 1, 0, 3},
            {Vector3f{0, 0, 0}, Vector3f{-1, 0, 0}, Vector3f{-1, 0, 0}, Vector3f{1, 0, 0},
             Vector3f{1, 0, 0}});
    ASSERT_TRUE(rig.valid());
    // Both limbs are exactly reachable: each is two units long from the anchor.
    const Vector3 left_target{-1.0, 1.0, 0.0};
    const Vector3 right_target{1.0, 1.0, 0.0};

    FullBodyIk left;
    left.root_joint = rig.joint("left_upper");
    left.iterations = 40;
    left.effectors.push_back(FullBodyEffector{rig.joint("left_tip"), left_target, 1.0f});

    FullBodyIk right;
    right.root_joint = rig.joint("right_upper");
    right.iterations = 40;
    right.effectors.push_back(FullBodyEffector{rig.joint("right_tip"), right_target, 1.0f});

    rig.apply(left);
    rig.apply(right);

    EXPECT_LT(distance(rig.position("left_tip"), left_target), 1e-3);
    EXPECT_LT(distance(rig.position("right_tip"), right_target), 1e-3)
        << "the second solve disturbed or was disturbed by the first";
    // The never-rotated anchor stayed put, which is what made the two solves independent.
    EXPECT_NEAR(magnitude(rig.position("anchor")), 0.0, 1e-9);
}

TEST(Unit_AnimationIk, RagdollBlendMovesAJointAndCarriesItsChildrenWithIt)
{
    // The property a naive implementation gets wrong: overwriting `model[joint]` directly
    // leaves every descendant at its old place. Targeting a parent must cascade.
    Rig rig = Rig({"root", "child"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}});
    ASSERT_TRUE(rig.valid());
    ASSERT_NEAR(rig.position("child").x, 1.0, 1e-9);

    RagdollBlend solver;
    RagdollJointTarget target;
    target.joint = rig.joint("root");
    target.object_space_transform = translation_matrix(10.0, 0.0, 0.0);
    target.weight = 1.0f;
    solver.targets.push_back(target);

    rig.apply(solver);
    EXPECT_NEAR(rig.position("root").x, 10.0, 1e-4);
    EXPECT_NEAR(rig.position("child").x, 11.0, 1e-4)
        << "the untargeted child was stranded at its old position";
}

TEST(Unit_AnimationIk, RagdollBlendWeightInterpolatesBetweenAnimationAndPhysics)
{
    const auto solve_at = [](float weight)
    {
        Rig rig = Rig({"root", "child"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}});
        RagdollBlend solver;
        RagdollJointTarget target;
        target.joint = rig.joint("root");
        target.object_space_transform = translation_matrix(10.0, 0.0, 0.0);
        target.weight = weight;
        solver.targets.push_back(target);
        rig.apply(solver);
        return rig.position("root").x;
    };

    EXPECT_NEAR(solve_at(0.0f), 0.0, 1e-9) << "weight 0 must be pure animation";
    EXPECT_NEAR(solve_at(0.5f), 5.0, 1e-4);
    EXPECT_NEAR(solve_at(1.0f), 10.0, 1e-4) << "weight 1 must be pure physics";
}

TEST(Unit_AnimationIk, AJiggleBoneLagsBehindASuddenMoveAndThenSettles)
{
    // The assertion that caught this solver's original no-op: a spring must produce a
    // *measurable* lag. Rotating the configured joint itself moves nothing in a hierarchy,
    // and a "did not crash" check would have passed that version.
    Rig rig = Rig({"root", "bone"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}});
    ASSERT_TRUE(rig.valid());
    const std::uint32_t bone = rig.joint("bone");
    const double rest_length = distance(rig.position("root"), rig.position("bone"));

    JiggleBone solver;
    JiggleJoint spring;
    spring.joint = bone;
    spring.stiffness = 120.0f;
    spring.damping = 0.9f;
    solver.joints.push_back(spring);
    solver.set_delta_time(1.0f / 60.0f);

    // Settle at rest first, so the lag below is caused by the lurch and nothing else.
    for (int frame = 0; frame < 60; ++frame)
    {
        rig.reset();
        rig.apply(solver);
    }
    const Vector3 settled = rig.position("bone");

    // A sudden two-unit lurch of the parent, across the bone's axis so the spring has a lateral
    // component to swing through (see the axial case below). The contract the header documents:
    // the caller recomposes a fresh, un-jiggled animated pose every frame, so the rest pose is
    // re-seeded and only the spring state carries over.
    const auto lurched_rig = [&]
    {
        Rig moved = Rig({"root", "bone"}, {-1, 0}, {Vector3f{0, 2, 0}, Vector3f{1, 0, 0}});
        return moved;
    };
    Rig moved = lurched_rig();
    const Vector3 rigid_rest = moved.position("bone");
    ASSERT_NEAR(distance(rigid_rest, settled), 2.0, 1e-6);

    // One frame after the lurch the bone must still be visibly behind its rigid rest.
    moved.apply(solver);
    const double lag = distance(moved.position("bone"), rigid_rest);
    EXPECT_GT(lag, 0.25) << "the spring produced no lag — it is not moving the joint at all";
    EXPECT_LT(lag, 2.01) << "the spring overshot past the pre-lurch position";

    // Three hundred frames later it has caught up, and the bone length held throughout.
    for (int frame = 0; frame < 300; ++frame)
    {
        Rig step = lurched_rig();
        step.apply(solver);
        EXPECT_NEAR(distance(step.position("root"), step.position("bone")), rest_length, 1e-3)
            << "the distance constraint let the bone stretch at frame " << frame;
        if (frame == 299)
            EXPECT_LT(distance(step.position("bone"), rigid_rest), 1e-2) << "never settled";
    }
}

TEST(Unit_AnimationIk, AJiggleBoneCannotResolveAPurelyAxialLurch)
{
    // A documented limitation, pinned so it is a known shape rather than a surprise. The model
    // is one point mass pulled toward the rest position and then projected back onto the bone
    // length. When the parent moves exactly *along* the bone's own axis by twice its length,
    // the mass sits diametrically opposite on that sphere: the spring pull is parallel to the
    // constraint, so the projection returns it to where it started and the joint never catches
    // up. A caller who needs axial travel handled must drive the joint's rest length, not this
    // spring. Any lateral component at all breaks the symmetry, which the test above covers.
    JiggleBone solver;
    JiggleJoint spring;
    solver.set_delta_time(1.0f / 60.0f);

    Rig probe = Rig({"root", "bone"}, {-1, 0}, {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}});
    spring.joint = probe.joint("bone");
    solver.joints.push_back(spring);
    for (int frame = 0; frame < 30; ++frame)
    {
        probe.reset();
        probe.apply(solver);
    }

    // Lurch the parent along +X, the bone's own direction.
    double final_error = 0.0;
    for (int frame = 0; frame < 120; ++frame)
    {
        Rig step = Rig({"root", "bone"}, {-1, 0}, {Vector3f{2, 0, 0}, Vector3f{1, 0, 0}});
        step.apply(solver);
        final_error = distance(step.position("bone"), Vector3{3.0, 0.0, 0.0});
    }
    EXPECT_NEAR(final_error, 2.0, 1e-3)
        << "the axial fixed point resolved — the model changed, update this test and the header";
}

TEST(Unit_AnimationIk, ModifiersComposeInStackOrder)
{
    // §5.3: the stack is ordered, and a later modifier sees the earlier one's output. Two
    // two-bone solvers with different targets must therefore end at the second one's.
    Rig rig = straight_arm();
    TwoBoneIk first;
    first.upper = rig.joint("shoulder");
    first.mid = rig.joint("elbow");
    first.tip = rig.joint("wrist");
    first.target = Vector3{1.0, 1.0, 0.0};
    first.pole = Vector3{1.0, 0.5, 0.0};

    TwoBoneIk second = first;
    second.target = Vector3{-1.0, 1.0, 0.0};
    second.pole = Vector3{-1.0, 0.5, 0.0};

    const IPoseModifier* stack[2] = {&first, &second};
    for (const IPoseModifier* modifier : stack)
        modifier->solve(rig.context());

    EXPECT_LT(distance(rig.position("wrist"), second.target), 1e-3)
        << "the last modifier in the stack must own the final pose";
}

// A ground provider for foot placement: one plane, so the expected answer is arithmetic rather
// than a second raycaster's opinion. `misses` is the case a foot swung out over a ledge produces,
// and it is the one whose correct behaviour is to leave the animated pose alone.
namespace
{
    class PlaneGround : public IPoseTaskContext
    {
        public:
            PlaneGround(double height, const Vector3& plane_normal)
                : height_(static_cast<Scalar>(height)), normal_(normalize(plane_normal))
            {
            }

            /** @brief Makes every cast miss, as a foot swung out over an edge does. */
            void set_misses(bool misses) noexcept { misses_ = misses; }

            /** @brief How many casts have been made, so a solver's early-outs are observable. */
            int cast_count() const noexcept { return casts_; }

            bool raycast(const Vector3& origin, const Vector3& direction, Vector3& out_hit_point,
                        Vector3& out_normal) const override
            {
                ++casts_;
                if (misses_)
                    return false;
                // The plane passes through (0, height_, 0) with normal_, so a downward ray from
                // the origin meets it where the plane equation is satisfied.
                const Vector3 point_on_plane{0.0, height_, 0.0};
                const Scalar denominator = dot(normal_, direction);
                if (std::fabs(static_cast<double>(denominator)) < 1e-9)
                    return false;
                const Scalar t = dot(normal_, point_on_plane - origin) / denominator;
                if (t < static_cast<Scalar>(0))
                    return false;
                out_hit_point = origin + direction * t;
                out_normal = normal_;
                return true;
            }

        private:
            Scalar height_;
            Vector3 normal_;
            bool misses_ = false;
            mutable int casts_ = 0;
    };

    // A leg along -Y: hip at the origin, knee one unit down, ankle two units down. Standing on a
    // plane at y = -2 is therefore the rest pose, and any other plane height is a correction.
    Rig straight_leg()
    {
        return Rig({"hip", "knee", "ankle"}, {-1, 0, 1},
                   {Vector3f{0, 0, 0}, Vector3f{0, -1, 0}, Vector3f{0, -1, 0}});
    }

    /** @brief A foot-placement solver wired to a leg rig and a ground. */
    FootPlacementIk leg_solver(const Rig& rig, const IPoseTaskContext& ground)
    {
        FootPlacementIk solver;
        solver.hip = rig.joint("hip");
        solver.knee = rig.joint("knee");
        solver.ankle = rig.joint("ankle");
        solver.ground = &ground;
        // The knee bends forward, which for a leg down -Y means the pole is off the leg's axis.
        solver.pole = Vector3{0.0, -1.0, 1.0};
        solver.ray_height = 1.0f;
        solver.ray_length = 4.0f;
        return solver;
    }
} // namespace

TEST(Unit_AnimationIk, AFootLandsOnRaisedGroundInsteadOfSinkingIntoIt)
{
    // The whole point of the modifier: the animation was authored for flat ground, the ground is
    // not flat, and the ankle must end up on the surface the ray found rather than where the clip
    // put it. Asserting the ankle's height against the *plane* is what distinguishes a real solve
    // from a solver that merely moved the foot somewhere.
    Rig rig = straight_leg();
    ASSERT_TRUE(rig.valid());
    const PlaneGround ground(-1.5, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk solver = leg_solver(rig, ground);
    solver.foot_height = 0.1f;

    EXPECT_NEAR(rig.position("ankle").y, -2.0, 1e-6) << "the rest pose stands at y = -2";
    rig.apply(solver);

    // Ground at -1.5 plus a 0.1 foot height puts the ankle at -1.4.
    EXPECT_NEAR(rig.position("ankle").y, -1.4, 1e-3);
    // The leg is shorter now, not stretched: the bones keep their lengths and the knee bends.
    EXPECT_NEAR(distance(rig.position("hip"), rig.position("knee")), 1.0, 1e-4);
    EXPECT_NEAR(distance(rig.position("knee"), rig.position("ankle")), 1.0, 1e-4);
    EXPECT_GT(std::fabs(static_cast<double>(rig.position("knee").z)), 1e-3)
        << "the knee must leave the leg's axis, or nothing bent";
}

TEST(Unit_AnimationIk, AFloatingFootIsPlantedDownIntoADip)
{
    // The other direction, and the one that decided a stale comment. The header used to claim the
    // solver "only plants when the ground is at or above the animated foot"; no such condition is
    // in the code, and adding one would be wrong — a walk cycle authored on flat ground would then
    // hover over every depression rather than stepping into it. So the goal is the ground both
    // ways, and this pins it.
    //
    // The starting pose has to be *bent* for the downward case to be reachable at all, because a
    // straight leg already stands at the limit of its own extension. One solve onto high ground
    // produces that bent pose; the second solve is the one under test.
    Rig rig = straight_leg();
    const PlaneGround high(-1.3, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk lift = leg_solver(rig, high);
    rig.apply(lift);
    ASSERT_NEAR(rig.position("ankle").y, -1.3, 1e-3) << "the setup solve must bend the leg";

    const PlaneGround lower(-1.8, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk plant = leg_solver(rig, lower);
    rig.apply(plant);

    EXPECT_NEAR(rig.position("ankle").y, -1.8, 1e-3);
    EXPECT_NEAR(distance(rig.position("hip"), rig.position("knee")), 1.0, 1e-4);
    EXPECT_NEAR(distance(rig.position("knee"), rig.position("ankle")), 1.0, 1e-4);
}

TEST(Unit_AnimationIk, GroundBeyondTheLegsReachLeavesItStraightRatherThanStretched)
{
    // The case that made the stale comment look plausible: ground far below a fully extended leg.
    // Nothing needs to guard against it, because the two-bone solver extends toward an unreachable
    // target without stretching — so the leg straightens and stops, and the bones keep their
    // lengths. A guard added "for safety" here would be dead code justified by a wrong belief.
    Rig rig = straight_leg();
    const PlaneGround far_below(-2.6, Vector3{0.0, 1.0, 0.0});
    const FootPlacementIk solver = leg_solver(rig, far_below);

    rig.apply(solver);
    EXPECT_NEAR(rig.position("ankle").y, -2.0, 1e-3) << "the leg reaches its own limit, no further";
    EXPECT_NEAR(distance(rig.position("hip"), rig.position("ankle")), 2.0, 1e-4)
        << "and it is straight, not stretched";
    EXPECT_NEAR(distance(rig.position("hip"), rig.position("knee")), 1.0, 1e-4);
    EXPECT_NEAR(distance(rig.position("knee"), rig.position("ankle")), 1.0, 1e-4);
}

TEST(Unit_AnimationIk, TheSoleFollowsASlopesNormal)
{
    // The second half of the modifier, and the one a height-only solver silently omits: on a
    // slope the ankle is re-oriented so the sole lies along the surface. The assertion is on the
    // ankle's up axis after the solve, because that is what the sole is normal to.
    Rig rig = straight_leg();
    const Vector3 slope = normalize(Vector3{0.0, 1.0, 0.4});
    const PlaneGround ground(-1.8, slope);
    FootPlacementIk solver = leg_solver(rig, ground);
    solver.up_axis = Vector3{0.0, 1.0, 0.0};

    rig.apply(solver);

    const Vector3 sole_normal =
        normalize(rotate(rig.context().rotation(rig.joint("ankle")), solver.up_axis));
    EXPECT_NEAR(static_cast<double>(dot(sole_normal, slope)), 1.0, 1e-3)
        << "the sole's normal must align with the ground's";
}

TEST(Unit_AnimationIk, AMissedRayLeavesTheAnimatedFootStanding)
{
    // A foot over a ledge, and the documented behaviour: no hit means no correction. Snapping to
    // the ray's end instead would drop the foot into the void, which is the visible bug this
    // early-out prevents.
    Rig rig = straight_leg();
    PlaneGround ground(-1.0, Vector3{0.0, 1.0, 0.0});
    ground.set_misses(true);
    const FootPlacementIk solver = leg_solver(rig, ground);

    rig.apply(solver);
    EXPECT_NEAR(rig.position("ankle").y, -2.0, 1e-6);
    EXPECT_EQ(ground.cast_count(), 1) << "one cast per solve, and its answer was respected";

    // A null ground is the same answer, and it is what an entity outside any physics scene has.
    Rig no_ground_rig = straight_leg();
    FootPlacementIk unwired = solver;
    unwired.ground = nullptr;
    no_ground_rig.apply(unwired);
    EXPECT_NEAR(no_ground_rig.position("ankle").y, -2.0, 1e-6);
}

TEST(Unit_AnimationIk, FootPlacementWeightBlendsAndZeroCostsNothing)
{
    // The substitutability contract every modifier in this suite is held to: zero weight is an
    // exact no-op, and it must not even ask the ground — a disabled solver that still raycasts is
    // paying for a feature that is off.
    Rig rig = straight_leg();
    PlaneGround ground(-1.5, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk solver = leg_solver(rig, ground);
    solver.weight = 0.0f;

    rig.apply(solver);
    EXPECT_NEAR(rig.position("ankle").y, -2.0, 1e-9);
    EXPECT_EQ(ground.cast_count(), 0) << "a zero-weight solver must not cast";

    // A half weight lands between the animated pose and the full correction.
    Rig half_rig = straight_leg();
    PlaneGround half_ground(-1.5, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk half = leg_solver(half_rig, half_ground);
    half.weight = 0.5f;
    half_rig.apply(half);
    const double half_height = static_cast<double>(half_rig.position("ankle").y);

    Rig full_rig = straight_leg();
    PlaneGround full_ground(-1.5, Vector3{0.0, 1.0, 0.0});
    const FootPlacementIk full = leg_solver(full_rig, full_ground);
    full_rig.apply(full);
    const double full_height = static_cast<double>(full_rig.position("ankle").y);

    EXPECT_LT(half_height, full_height);
    EXPECT_GT(half_height, -2.0);
}

TEST(Unit_AnimationIk, FootPlacementDoesNotAdjustThePelvisAndSaysSo)
{
    // The named limitation, pinned so it is a decision rather than a surprise: the header defers
    // pelvis height to a higher-level rig pass, so a foot planted on ground far above the clip's
    // leaves the hip exactly where the animation put it. A future pelvis pass changing this
    // should have to update this test deliberately.
    Rig rig = straight_leg();
    const Vector3 hip_before = rig.position("hip");
    const PlaneGround ground(-0.5, Vector3{0.0, 1.0, 0.0});
    FootPlacementIk solver = leg_solver(rig, ground);
    solver.ray_height = 2.0f;

    rig.apply(solver);
    EXPECT_NEAR(distance(rig.position("hip"), hip_before), 0.0, 1e-9)
        << "the hip must not move — pelvis adjustment is deliberately not this modifier's job";
    EXPECT_GT(static_cast<double>(rig.position("ankle").y), -2.0);
}
