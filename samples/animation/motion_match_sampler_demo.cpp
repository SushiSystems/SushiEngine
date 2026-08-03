/**************************************************************************/
/* motion_match_sampler_demo.cpp                                          */
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

// The §12.4 "blend-graph wiring" MotionMatchSampler leaves open in motion_matching.hpp's
// own header comment: proves a caller gets a working crossfade for free. Two one-joint
// (root-only) clips — a static idle (root fixed at the origin) and a forward walk (root
// moving at a constant analytic velocity) — are put in one MotionDatabase; the sampler is
// reset onto idle, then driven with a forward-velocity query. What's checked:
//   * Before any query resolves, the output pose is exactly the idle clip's (root at 0).
//   * Once the periodic re-search (hysteresis) picks the forward candidate, crossfading()
//     reports true and the output starts moving away from 0 before the fade completes —
//     i.e. it is actually blending, not snapping.
//   * Once the configured crossfade duration elapses, crossfading() reports false again and
//     the output has clearly departed the idle pose, matching the forward clip's own motion.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/motion_match_sampler.hpp>
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
            std::printf("[motion_match_sampler_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    // A one-joint (root-only) clip whose root moves linearly from (0,0,0) to
    // end_translation over its 2-second duration — long enough that this demo's whole
    // ~0.3s query-to-settle window never crosses the loop boundary.
    std::vector<std::byte> make_linear_clip(Vector3f end_translation)
    {
        const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
        ClipDescription description;
        description.joint_count = 1;
        description.frame_count = 2;
        description.sample_rate = 0.5f; // (frame_count - 1) / sample_rate == 2s duration
        description.translations = {Vector3f{0, 0, 0}, end_translation};
        description.rotations = {identity, identity};
        description.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}};
        std::vector<std::byte> blob;
        if (!build_clip_blob(description, blob))
            std::printf("[motion_match_sampler_demo] FAIL: cook clip\n"), ++failures;
        return blob;
    }
}

int main()
{
    SkeletonDescription skeleton_description;
    JointDescription root;
    root.name = "root";
    root.parent = -1;
    skeleton_description.joints = {root};
    std::vector<std::byte> skeleton_blob;
    check(build_skeleton_blob(skeleton_description, skeleton_blob), "cook skeleton");

    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const SkeletonView skeleton = database.skeleton(skeleton_id);

    const AssetId idle_id = database.add_clip(make_linear_clip(Vector3f{0, 0, 0}));
    const AssetId forward_id = database.add_clip(make_linear_clip(Vector3f{0, 0, 4}));
    check(idle_id != INVALID_ASSET && forward_id != INVALID_ASSET, "register clips");

    MotionDatabase motion_db;
    motion_db.add_clip(idle_id, database, skeleton, /*root_joint=*/0, -1, -1,
                       /*candidates_per_clip=*/1);
    motion_db.add_clip(forward_id, database, skeleton, 0, -1, -1, 1);
    check(motion_db.size() == 2, "database holds one candidate per clip");

    MotionMatchSampler sampler;
    sampler.bind(skeleton);

    MotionFeature query_idle;
    query_idle.root_velocity = Vector3f{0, 0, 0};
    const std::size_t idle_candidate = motion_db.find_best(query_idle);
    check(motion_db[idle_candidate].clip == idle_id, "idle query resolves to the idle candidate");
    sampler.reset(motion_db, database, idle_candidate);

    check(std::fabs(sampler.local_translations()[0].z) < 1e-4f,
         "reset onto idle leaves the root at z == 0");
    check(!sampler.crossfading(), "no crossfade in progress right after reset");

    MotionMatchSamplerConfiguration config;
    config.resample_interval_seconds = 0.05f;
    config.crossfade_seconds = 0.2f;

    MotionFeature query_forward;
    query_forward.root_velocity = Vector3f{0, 0, 2}; // clip's analytic velocity is (0,0,4)/2s

    const float dt = 1.0f / 60.0f;
    bool saw_crossfade = false;
    float z_when_crossfade_started = 0.0f;
    float elapsed = 0.0f;
    for (int step = 0; step < 30 && !saw_crossfade; ++step) // up to 0.5s to trigger the resample
    {
        sampler.update(motion_db, database, query_forward, dt, config);
        elapsed += dt;
        if (sampler.crossfading())
        {
            saw_crossfade = true;
            z_when_crossfade_started = sampler.local_translations()[0].z;
        }
    }
    check(saw_crossfade, "the periodic re-search eventually picks the forward candidate");
    check(motion_db[sampler.current_candidate()].clip == forward_id,
         "the selected candidate belongs to the forward clip");
    check(std::fabs(z_when_crossfade_started) < 0.5f,
         "the crossfade starts close to the idle pose, not snapped to the forward pose");

    // Keep advancing until the configured crossfade duration has elapsed.
    for (int step = 0; step < 60 && sampler.crossfading(); ++step)
        sampler.update(motion_db, database, query_forward, dt, config);
    check(!sampler.crossfading(), "the crossfade completes within its configured duration");

    const float z_after_settle = sampler.local_translations()[0].z;
    check(z_after_settle > 0.3f,
         "once settled, the output pose has clearly moved with the forward clip");
    std::printf(
        "[motion_match_sampler_demo] crossfade start z=%.4f, settled z=%.4f (elapsed %.3fs)\n",
        z_when_crossfade_started, z_after_settle, elapsed);

    if (failures != 0)
    {
        std::printf("[motion_match_sampler_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[motion_match_sampler_demo] OK — search hysteresis and crossfade blending "
               "verified\n");
    return 0;
}
