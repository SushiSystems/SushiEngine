/**************************************************************************/
/* device_batch_evaluator_demo.cpp                                        */
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

// The A2/§12.3 device-batched evaluator, worked and self-checked. Same root+child arm
// and quarter-circle clip as clip_demo.cpp (root turns 90 deg about Z from frame 0 to
// 1, child sits one unit down +X), so every sample has an analytic answer — but here a
// batch of many instances at different playback times/loop settings is evaluated in
// ONE compiled SushiRuntime graph run, and the device palette is checked against both
// the analytic answer and the host ClipEvaluator (the two evaluation paths must agree
// bit-close, or the device kernel's hand-reproduced sampling algorithm has drifted from
// ClipView::sample's). Also proves compile_count stays 1 across repeated evaluate()
// calls with the same instance count, the same replay-only invariant the rest of the
// engine holds.

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

// The animation-only umbrella, not the full SushiEngine.hpp: this demo needs no audio/
// render/physics module, and isolates it from unrelated breakage elsewhere in the tree
// (see [[concurrent-sessions-shared-repo]] in memory — another session's in-progress
// audio edits can leave audio.hpp non-compiling; this file has no reason to depend on it).
#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/core/types.hpp>

// SYCL-only, kept off the animation.hpp umbrella (like accelerator_sycl.hpp is kept off
// audio.hpp) so non-SYCL consumers (the editor) never pull in SushiRuntime/SYCL headers.
#include <SushiEngine/animation/device_batch_evaluator.hpp>

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
            std::printf("[device_batch_evaluator_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(double a, double b, double eps = 1e-3) { return std::fabs(a - b) <= eps; }

    // The skin matrix's translation column (skin = model * inverse_bind, so this is
    // *not* the joint's model-space position — see the identity-at-bind-pose comment
    // below). Used only to diff host vs. device output: both sides extract the same
    // column from the same kind of matrix, so a nonzero difference here still means a
    // real mismatch between the two evaluators, even though the number itself isn't a
    // world position.
    Vector3 child_position(const JointMatrix& skin)
    {
        return Vector3{static_cast<double>(skin.m[12]), static_cast<double>(skin.m[13]),
                       static_cast<double>(skin.m[14])};
    }
}

int main()
{
    // Same rig and clip as clip_demo.cpp.
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

    const Quaternionf identity{0.0f, 0.0f, 0.0f, 1.0f};
    const QuaternionT<float> turn =
        quaternion_axis_angle(Vector3T<float>{0, 0, 1}, static_cast<float>(PI * 0.5));
    ClipDescription clip_description;
    clip_description.joint_count = 2;
    clip_description.frame_count = 2;
    clip_description.sample_rate = 1.0f;
    clip_description.translations = {Vector3f{0, 0, 0}, Vector3f{1, 0, 0}, Vector3f{0, 0, 0},
                                     Vector3f{1, 0, 0}};
    clip_description.rotations = {identity, identity, turn, identity};
    clip_description.scales = {Vector3f{1, 1, 1}, Vector3f{1, 1, 1}, Vector3f{1, 1, 1},
                               Vector3f{1, 1, 1}};

    std::vector<std::byte> clip_blob;
    check(build_clip_blob(clip_description, clip_blob), "cook clip");

    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const AssetId clip_id = database.add_clip(std::move(clip_blob));
    check(skeleton_id != INVALID_ASSET && clip_id != INVALID_ASSET, "register assets");

    const SkeletonView skeleton = database.skeleton(skeleton_id);
    const ClipView clip = database.clip(clip_id);

    // Device batch: a crowd at different times, one bound skeleton + clip.
    auto runtime = SushiRuntime::API::Runtime::create();
    Execution::Context execution(runtime);
    DeviceBatchEvaluator device(execution);
    check(device.bind_skeleton(skeleton), "bind skeleton");
    const std::uint32_t clip_handle = device.bind_clip(clip);
    check(clip_handle != INVALID_CLIP_HANDLE, "bind clip");

    // 37 instances spread from t=0 to t=2 (past one full loop), a mix of looping and
    // clamped — enough to exercise both branches of the sampling algorithm and to be a
    // real "batch", not a single-instance edge case.
    constexpr std::size_t COUNT = 37;
    std::vector<DeviceInstanceDescription> instances(COUNT);
    for (std::size_t i = 0; i < COUNT; ++i)
    {
        instances[i].clip_handle = clip_handle;
        instances[i].time_seconds = static_cast<float>(i) / static_cast<float>(COUNT - 1) * 2.0f;
        instances[i].loop = (i % 3 != 0) ? 1u : 0u; // most loop, every third clamps
    }
    // Force the two boundary instances' loop flags explicitly, so the spot-checks below
    // (which assume instance 0 and instance COUNT-1 both loop) don't depend on whether
    // COUNT happens to be a multiple of 3.
    instances[0].loop = 1u;
    instances[COUNT - 1].loop = 1u;
    device.set_instances(instances);
    const Execution::RunReport report = device.evaluate();
    (void)report;
    check(device.instance_count() == COUNT, "device batch sized to the instance list");
    check(device.palettes().size() == COUNT * skeleton.joint_count, "palette readback sized correctly");

    // Cross-check every instance against the host ClipEvaluator.
    ClipEvaluator host_evaluator;
    double max_position_error = 0.0;
    for (std::size_t i = 0; i < COUNT; ++i)
    {
        host_evaluator.evaluate(skeleton, clip, instances[i].time_seconds, instances[i].loop != 0);
        const JointMatrix& host_child_skin = host_evaluator.palette()[1];
        const JointMatrix& device_child_skin =
            device.palettes()[i * skeleton.joint_count + 1];

        const Vector3 host_pos = child_position(host_child_skin);
        const Vector3 device_pos = child_position(device_child_skin);
        const double error = std::sqrt((host_pos.x - device_pos.x) * (host_pos.x - device_pos.x) +
                                       (host_pos.y - device_pos.y) * (host_pos.y - device_pos.y) +
                                       (host_pos.z - device_pos.z) * (host_pos.z - device_pos.z));
        max_position_error = std::max(max_position_error, error);

        // Every matrix element, not just the child's model-space position — proves the
        // device kernel's full skin matrix (rotation columns included), not only the
        // translation column this rig happens to make legible.
        for (std::uint32_t j = 0; j < skeleton.joint_count; ++j)
        {
            const JointMatrix& h = host_evaluator.palette()[j];
            const JointMatrix& d = device.palettes()[i * skeleton.joint_count + j];
            for (int k = 0; k < 16; ++k)
                if (!nearly(h.m[k], d.m[k], 1e-3))
                {
                    std::printf(
                        "[device_batch_evaluator_demo] mismatch instance %zu joint %u elem %d: "
                        "host=%f device=%f (t=%f loop=%u)\n",
                        i, j, k, static_cast<double>(h.m[k]), static_cast<double>(d.m[k]),
                        static_cast<double>(instances[i].time_seconds), instances[i].loop);
                    ++failures;
                }
        }
    }
    check(max_position_error < 1e-2, "device batch matches the host evaluator across the crowd");
    std::printf("[device_batch_evaluator_demo] max host/device position error: %.6f\n",
               max_position_error);

    // Spot-check two instances against the analytic answer.
    // Skin = model * inverse_bind, so at the bind pose (frame 0) skin is the IDENTITY,
    // not the child's (1,0,0) position — position lives in the *model* matrix, which
    // this evaluator does not expose (only the palette render actually needs); this is
    // the same distinction clip_demo.cpp draws between its "t=0 skin palette is
    // identity" and separately-computed "t=0 child at (1,0,0)" (via ClipEvaluator's
    // model() accessor, which DeviceBatchEvaluator has no equivalent of).
    auto is_identity = [](const JointMatrix& m)
    {
        for (int k = 0; k < 16; ++k)
        {
            const double expected = (k % 5 == 0) ? 1.0 : 0.0; // diagonal of a 4x4
            if (!nearly(m.m[k], expected, 1e-4))
                return false;
        }
        return true;
    };
    // Instance 0 is t=0 (bind pose): both joints' skin matrices are the identity.
    check(is_identity(device.palettes()[0 * skeleton.joint_count + 0]) &&
             is_identity(device.palettes()[0 * skeleton.joint_count + 1]),
         "device t=0 skin palette is identity");
    // The last instance is t=2, looping -> wraps to frame 0 again -> identity too.
    check(is_identity(device.palettes()[(COUNT - 1) * skeleton.joint_count + 1]),
         "device loop wrap returns to identity skin");

    // Replay-only: re-evaluating the same instance count must not recompile.
    device.set_instances(instances);
    device.evaluate();
    device.set_instances(instances);
    device.evaluate();
    check(device.compile_count() == 1, "device graph compiles once and replays (compile_count == 1)");

    if (failures != 0)
    {
        std::printf("[device_batch_evaluator_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[device_batch_evaluator_demo] OK — %zu-instance batch matches the host evaluator, "
        "compile_count == 1\n",
        COUNT);
    return 0;
}
