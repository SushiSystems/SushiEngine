/**************************************************************************/
/* animation_benchmark.cpp                                              */
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

// The A2 headless animation benchmark: it measures the CPU cost of posing the reference
// crowd — 100 hero characters (80 joints, full rate) plus 1000 crowd instances (throttled)
// — through the batched evaluator against a compressed clip, and reports the compression
// ratio, the batched CPU time per frame, and how the update-rate throttle cuts the per-frame
// pose count. It is the measurement the design's §4 / §9 numbers are read from; a real GPU
// budget adds the skinning pass, which this CPU harness does not run.

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    constexpr std::uint32_t JOINTS = 80;
    constexpr std::uint32_t FRAMES = 300; // 10 s at 30 Hz
    constexpr std::size_t HERO = 100;
    constexpr std::size_t CROWD = 1000;

    Quaternionf axis_angle(float x, float y, float z, float angle)
    {
        const float s = std::sin(angle * 0.5f);
        return Quaternionf{x * s, y * s, z * s, std::cos(angle * 0.5f)};
    }
}

int main()
{
    // --- An 80-joint chain skeleton -------------------------------------------------
    SkeletonDesc skeleton_desc;
    skeleton_desc.joints.resize(JOINTS);
    for (std::uint32_t j = 0; j < JOINTS; ++j)
    {
        skeleton_desc.joints[j].name = "joint_" + std::to_string(j);
        skeleton_desc.joints[j].parent = j == 0 ? -1 : static_cast<int>(j - 1);
        skeleton_desc.joints[j].bind_translation = Vector3f{0.0f, 0.1f, 0.0f};
    }
    std::vector<std::byte> skeleton_blob;
    if (!build_skeleton_blob(skeleton_desc, skeleton_blob))
    {
        std::printf("[animation_benchmark] skeleton cook failed\n");
        return 1;
    }

    // --- A compressed clip: most joints rotate gently, a realistic locomotion shape --
    ClipDesc clip_desc;
    clip_desc.joint_count = JOINTS;
    clip_desc.frame_count = FRAMES;
    clip_desc.sample_rate = 30.0f;
    clip_desc.translations.resize(static_cast<std::size_t>(FRAMES) * JOINTS);
    clip_desc.rotations.resize(static_cast<std::size_t>(FRAMES) * JOINTS);
    clip_desc.scales.resize(static_cast<std::size_t>(FRAMES) * JOINTS);
    for (std::uint32_t f = 0; f < FRAMES; ++f)
    {
        const float t = static_cast<float>(f) / clip_desc.sample_rate;
        for (std::uint32_t j = 0; j < JOINTS; ++j)
        {
            const std::size_t i = static_cast<std::size_t>(f) * JOINTS + j;
            clip_desc.scales[i] = Vector3f{1.0f, 1.0f, 1.0f};
            clip_desc.translations[i] = Vector3f{0.0f, 0.1f, 0.0f};
            const float angle = 0.35f * std::sin(2.0f * 3.14159265f * 0.5f * t + j * 0.15f);
            clip_desc.rotations[i] = axis_angle(0.0f, 0.0f, 1.0f, angle);
        }
    }

    std::vector<std::byte> raw_blob;
    build_clip_blob(clip_desc, raw_blob);
    std::vector<std::byte> compressed_blob;
    if (!compress_clip(clip_desc, 0.002f, compressed_blob))
    {
        std::printf("[animation_benchmark] compression failed\n");
        return 1;
    }
    const std::size_t raw_bytes = raw_blob.size();
    const std::size_t compressed_bytes = compressed_blob.size();
    const double ratio = static_cast<double>(raw_bytes) / compressed_bytes;

    AnimationDatabase database;
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));
    const AssetId clip_id = database.add_clip(std::move(compressed_blob));
    const SkeletonView skeleton = database.skeleton(skeleton_id);
    const ClipView clip = database.clip(clip_id);

    // --- The reference crowd: 100 hero (close), 1000 crowd (far) ---------------------
    std::vector<BatchInstance> instances;
    instances.reserve(HERO + CROWD);
    for (std::size_t i = 0; i < HERO; ++i)
        instances.push_back(BatchInstance{skeleton, clip, static_cast<float>(i) * 0.01f, true, 5.0f});
    for (std::size_t i = 0; i < CROWD; ++i)
        instances.push_back(
            BatchInstance{skeleton, clip, static_cast<float>(i) * 0.013f, true, 60.0f});

    AnimationBudget budget; // High tier: full rate near, 15 Hz far.
    BatchEvaluator batch;
    batch.reserve(instances.size(), JOINTS);

    // Warm every instance once (first frame poses all), then measure steady-state frames.
    batch.evaluate(instances.data(), instances.size(), 0, budget);

    constexpr int MEASURED_FRAMES = 240;
    double total_ms = 0.0;
    std::size_t total_posed = 0;
    for (int frame = 1; frame <= MEASURED_FRAMES; ++frame)
    {
        for (BatchInstance& instance : instances)
            instance.time += 1.0f / 60.0f;
        const auto start = std::chrono::steady_clock::now();
        batch.evaluate(instances.data(), instances.size(), static_cast<std::uint64_t>(frame),
                       budget);
        const auto end = std::chrono::steady_clock::now();
        total_ms += std::chrono::duration<double, std::milli>(end - start).count();
        total_posed += batch.posed_evaluations();
    }

    const double avg_ms = total_ms / MEASURED_FRAMES;
    const double avg_posed = static_cast<double>(total_posed) / MEASURED_FRAMES;

    // A correctness floor so the benchmark also guards the batched path: the hero palette
    // at joint 1 must be a real (non-identity) transform once posed.
    const JointMatrix& hero = batch.palette(0)[1];
    bool hero_posed = false;
    for (int k = 0; k < 16; ++k)
        if (std::fabs(hero.m[k] - ((k % 5 == 0) ? 1.0f : 0.0f)) > 1e-4f)
            hero_posed = true;

    std::printf("[animation_benchmark] %zu instances (%zu hero + %zu crowd), %u joints\n",
                instances.size(), HERO, CROWD, JOINTS);
    std::printf("  clip (%.0f s, %u joints): raw %.1f KB -> compressed %.1f KB (%.1fx)\n",
                FRAMES / clip_desc.sample_rate, JOINTS, raw_bytes / 1024.0,
                compressed_bytes / 1024.0, ratio);
    std::printf("  batched CPU pose: %.3f ms/frame, %.0f/%zu instances re-posed/frame "
                "(update-rate throttle)\n",
                avg_ms, avg_posed, instances.size());

    if (!hero_posed)
    {
        std::printf("[animation_benchmark] FAIL: hero instance was not posed\n");
        return 1;
    }
    std::printf("[animation_benchmark] OK\n");
    return 0;
}
