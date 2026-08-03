/**************************************************************************/
/* motion_matching_demo.cpp                                              */
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

// The §12.4 motion-matching database/search core, worked and self-checked. Three
// one-joint clips with analytic root velocities — idle (0,0,0), walk-forward (0,0,2),
// walk-backward (0,0,-2) — are sampled into one MotionDatabase, then queried with
// desired velocities to prove: an exact-match query returns a candidate from the right
// clip; a query near a clip's velocity but not exactly on it still resolves to the
// nearest neighbor, not a coin flip; and the foot-height term (proxy, not true ground
// contact — see motion_matching.hpp's header comment) actually moves the pick when two
// clips share a velocity but differ in stance height.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/motion_matching.hpp>
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
            std::printf("[motion_matching_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    // A one-joint (root-only) clip whose root moves linearly from (0,0,0) to
    // end_translation over its 1-second duration — a constant analytic velocity of
    // end_translation (since duration is 1s), everywhere except right at the loop
    // boundary (which candidate sampling below stays clear of).
    std::vector<std::byte> make_linear_clip(Vector3f end_translation)
    {
        const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
        ClipDescription desc;
        desc.joint_count = 1;
        desc.frame_count = 2;
        desc.sample_rate = 1.0f;
        desc.translations = {Vector3f{0, 0, 0}, end_translation};
        desc.rotations = {identity, identity};
        desc.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
        std::vector<std::byte> blob;
        if (!build_clip_blob(desc, blob))
            std::printf("[motion_matching_demo] FAIL: cook clip\n"), ++failures;
        return blob;
    }
}

int main()
{
    SkeletonDescription skeleton_desc;
    JointDescription root;
    root.name = "root";
    root.parent = -1;
    skeleton_desc.joints = {root};
    std::vector<std::byte> skeleton_blob;
    check(build_skeleton_blob(skeleton_desc, skeleton_blob), "cook skeleton");

    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    check(skeleton_id != INVALID_ASSET, "register skeleton");
    const SkeletonView skeleton = database.skeleton(skeleton_id);

    const AssetId idle_id = database.add_clip(make_linear_clip(Vector3f{0, 0, 0}));
    const AssetId forward_id = database.add_clip(make_linear_clip(Vector3f{0, 0, 2}));
    const AssetId backward_id = database.add_clip(make_linear_clip(Vector3f{0, 0, -2}));
    check(idle_id != INVALID_ASSET && forward_id != INVALID_ASSET &&
             backward_id != INVALID_ASSET,
         "register clips");

    MotionDatabase motion_db;
    // 5 evenly-spaced candidates across each clip's [0, duration) — stays clear of the
    // loop boundary the finite-difference step would otherwise wrap across.
    motion_db.add_clip(idle_id, database, skeleton, /*root_joint=*/0, -1, -1, 5);
    motion_db.add_clip(forward_id, database, skeleton, 0, -1, -1, 5);
    motion_db.add_clip(backward_id, database, skeleton, 0, -1, -1, 5);
    check(motion_db.size() == 15, "database holds 5 candidates per clip x 3 clips");

    auto clip_of = [&](std::size_t index) { return motion_db[index].clip; };

    // Exact-velocity queries resolve to the matching clip.
    MotionFeature query_idle;
    query_idle.root_velocity = Vector3f{0, 0, 0};
    const std::size_t best_idle = motion_db.find_best(query_idle);
    check(best_idle != MotionDatabase::NOT_FOUND && clip_of(best_idle) == idle_id,
         "query (0,0,0) matches the idle clip");

    MotionFeature query_forward;
    query_forward.root_velocity = Vector3f{0, 0, 2};
    const std::size_t best_forward = motion_db.find_best(query_forward);
    check(best_forward != MotionDatabase::NOT_FOUND && clip_of(best_forward) == forward_id,
         "query (0,0,2) matches the walk-forward clip");

    MotionFeature query_backward;
    query_backward.root_velocity = Vector3f{0, 0, -2};
    const std::size_t best_backward = motion_db.find_best(query_backward);
    check(best_backward != MotionDatabase::NOT_FOUND && clip_of(best_backward) == backward_id,
         "query (0,0,-2) matches the walk-backward clip");

    // A near-forward query (not exactly on any candidate) still resolves to the nearest
    // neighbor, not the farthest or a tie-broken wrong clip.
    MotionFeature query_near_forward;
    query_near_forward.root_velocity = Vector3f{0.1f, 0, 1.6f};
    const std::size_t best_near_forward = motion_db.find_best(query_near_forward);
    check(best_near_forward != MotionDatabase::NOT_FOUND &&
             clip_of(best_near_forward) == forward_id,
         "query near (0,0,1.6) still matches walk-forward, the nearest neighbor");

    // The foot-height term is an independent knob, not dead weight: two clips share
    // the same (zero) root velocity but differ in a "foot" joint's bind height — a
    // crouch-idle vs. a standing-idle. With foot_weight zeroed, velocity alone cannot
    // tell them apart (nearest-neighbor tie, resolved to whichever sorts first); with
    // velocity_weight zeroed, only the foot-height term can, and it must pick correctly.
    SkeletonDescription stance_skeleton_desc;
    JointDescription stance_root;
    stance_root.name = "root";
    stance_root.parent = -1;
    JointDescription foot;
    foot.name = "foot";
    foot.parent = 0;
    stance_skeleton_desc.joints = {stance_root, foot};
    std::vector<std::byte> stance_skeleton_blob;
    check(build_skeleton_blob(stance_skeleton_desc, stance_skeleton_blob),
         "cook stance skeleton");

    AnimationDatabase stance_asset_db;
    const AssetId stance_skeleton_id =
        stance_asset_db.add_skeleton(std::move(stance_skeleton_blob));
    const SkeletonView stance_skeleton = stance_asset_db.skeleton(stance_skeleton_id);

    // Both clips: root stationary (zero velocity everywhere); the foot joint's own
    // *bind* height differs between the two skeletons is not an option (one skeleton
    // here), so the difference is authored as the foot joint's animated Y instead —
    // standing keeps it at 1.0, crouching drops it to 0.2, both held constant.
    const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
    auto make_stance_clip = [&](float foot_height)
    {
        ClipDescription desc;
        desc.joint_count = 2;
        desc.frame_count = 2;
        desc.sample_rate = 1.0f;
        desc.translations = {Vector3f{0, 0, 0}, Vector3f{0, foot_height, 0},
                             Vector3f{0, 0, 0}, Vector3f{0, foot_height, 0}};
        desc.rotations = {identity, identity, identity, identity};
        desc.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1},
                       Vector3f{1, 1, 1}};
        std::vector<std::byte> blob;
        check(build_clip_blob(desc, blob), "cook stance clip");
        return blob;
    };
    const AssetId standing_id = stance_asset_db.add_clip(make_stance_clip(1.0f));
    const AssetId crouching_id = stance_asset_db.add_clip(make_stance_clip(0.2f));

    MotionDatabase stance_db;
    stance_db.add_clip(standing_id, stance_asset_db, stance_skeleton, 0, /*left_foot=*/1,
                       -1, 3);
    stance_db.add_clip(crouching_id, stance_asset_db, stance_skeleton, 0, 1, -1, 3);
    check(stance_db.size() == 6, "stance database holds both clips' candidates");

    MotionFeature stance_query;
    stance_query.root_velocity = Vector3f{0, 0, 0};
    stance_query.left_foot_height = 0.2f; // matches the crouching clip's foot height

    const auto stance_clip_of = [&](std::size_t index) { return stance_db[index].clip; };
    const std::size_t crouch_pick =
        stance_db.find_best(stance_query, /*velocity_weight=*/0.0f, /*foot_weight=*/1.0f);
    check(crouch_pick != MotionDatabase::NOT_FOUND &&
             stance_clip_of(crouch_pick) == crouching_id,
         "with velocity_weight=0, foot height alone picks the crouching clip");

    stance_query.left_foot_height = 1.0f; // matches the standing clip's foot height
    const std::size_t stand_pick =
        stance_db.find_best(stance_query, /*velocity_weight=*/0.0f, /*foot_weight=*/1.0f);
    check(stand_pick != MotionDatabase::NOT_FOUND &&
             stance_clip_of(stand_pick) == standing_id,
         "with velocity_weight=0, foot height alone picks the standing clip");

    if (failures != 0)
    {
        std::printf("[motion_matching_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[motion_matching_demo] OK — %zu-candidate database, nearest-neighbor search "
        "verified against three analytic-velocity clips\n",
        motion_db.size());
    return 0;
}
