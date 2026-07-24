/**************************************************************************/
/* animator_demo.cpp                                                    */
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

// The phase-A3 Animator core, worked and self-checked headlessly. A two-state controller
// (Idle <-> Walk over a "moving" bool) drives a character: the transition into Walk
// crossfades, the Walk clip carries the root forward (root motion moves the entity) and a
// footstep event on every loop, and dropping "moving" returns to Idle. Three things are then
// proved: the state machine reaches Walk and moves the entity; the footstep events fire; and
// the whole animator state is bit-reproducible — a fresh replay of the same inputs, and a
// rollback snapshot/restore/replay, both reproduce the final state byte-for-byte.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
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
            std::printf("[animator_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(double a, double b, double eps = 1e-4) { return std::fabs(a - b) <= eps; }

    struct CountingSink : IAnimationEventSink
    {
        int footsteps = 0;
        void on_animation_event(std::uint32_t, const AnimatorEvent& event) override
        {
            if (event.name == hash_name("footstep"))
                ++footsteps;
        }
    };
}

int main()
{
    AnimationDatabase database;

    // --- Skeleton: root + child ------------------------------------------------------
    SkeletonDesc skeleton_desc;
    JointDesc root; root.name = "root"; root.parent = -1;
    JointDesc child; child.name = "child"; child.parent = 0;
    child.bind_translation = Vector3f{0, 1, 0};
    skeleton_desc.joints = {root, child};
    std::vector<std::byte> skeleton_blob;
    build_skeleton_blob(skeleton_desc, skeleton_blob);
    const AssetId skeleton_id = database.add_skeleton(std::move(skeleton_blob));

    // --- Clips: idle (root still) and walk (root advances +2 in z per loop) ----------
    const auto make_clip = [&](bool walking) -> AssetId
    {
        ClipDesc clip;
        clip.joint_count = 2;
        clip.frame_count = 31; // 1 s at 30 Hz
        clip.sample_rate = 30.0f;
        clip.translations.resize(31 * 2);
        clip.rotations.resize(31 * 2, Quaternionf{0, 0, 0, 1});
        clip.scales.resize(31 * 2, Vector3f{1, 1, 1});
        for (std::uint32_t f = 0; f < 31; ++f)
        {
            const float z = walking ? (static_cast<float>(f) / 30.0f) * 2.0f : 0.0f;
            clip.translations[f * 2 + 0] = Vector3f{0, 0, z};   // root
            clip.translations[f * 2 + 1] = Vector3f{0, 1, 0};   // child
        }
        std::vector<std::byte> blob;
        build_clip_blob(clip, blob);
        return database.add_clip(std::move(blob));
    };
    const AssetId idle_clip = make_clip(false);
    const AssetId walk_clip = make_clip(true);

    // --- Controller: Idle <-> Walk over a "moving" bool ------------------------------
    ControllerDesc controller_desc;
    controller_desc.parameters.push_back(ParameterDesc{"moving", ParameterType::Bool, 0.0f});

    LayerDesc layer;
    layer.name = "base";
    layer.default_state = "Idle";

    StateDesc idle;
    idle.name = "Idle";
    idle.clip = idle_clip;
    TransitionDesc to_walk;
    to_walk.destination = "Walk";
    to_walk.duration = 0.1f; // a crossfade into Walk
    to_walk.conditions.push_back(ConditionDesc{"moving", Comparator::If, 0.0f});
    idle.transitions.push_back(to_walk);

    StateDesc walk;
    walk.name = "Walk";
    walk.clip = walk_clip;
    walk.events.push_back(StateEventDesc{0.5f, "footstep", 0});
    TransitionDesc to_idle;
    to_idle.destination = "Idle";
    to_idle.duration = 0.0f; // instant back to Idle
    to_idle.conditions.push_back(ConditionDesc{"moving", Comparator::IfNot, 0.0f});
    walk.transitions.push_back(to_idle);

    layer.states = {idle, walk};
    controller_desc.layers.push_back(layer);

    std::vector<std::byte> controller_blob;
    check(compile_controller_blob(controller_desc, controller_blob), "controller compiles");
    const AssetId controller_id = database.add_controller(std::move(controller_blob));
    check(controller_id != INVALID_ASSET, "controller registers");
    const ControllerView controller = database.controller(controller_id);
    const int moving_index = controller.find_parameter(hash_name("moving"));
    check(moving_index == 0, "parameter index resolves");

    // --- Drive the animator ----------------------------------------------------------
    const float dt = 1.0f / 60.0f;
    CountingSink sink;

    // A scripted input timeline: moving=true for ticks [30,150), else false.
    const auto moving_at = [](int tick) { return tick >= 30 && tick < 150; };

    AnimatorInstance animator{};
    animator.controller = controller_id;
    animator.skeleton = skeleton_id;
    Vector3 position{0, 0, 0};
    Quaternion orientation{0, 0, 0, 1};

    const int TICKS = 200;
    for (int tick = 0; tick < TICKS; ++tick)
    {
        animator.parameters.set_bool(static_cast<std::uint32_t>(moving_index), moving_at(tick));
        animator_step(controller, database, animator, dt);
        apply_root_motion(animator.root_motion, position, orientation);
        drain_events(animator, /*entity=*/1, sink);
    }

    // The character walked, so it advanced in +z; the base clip advances 2 units per second
    // over ~2 s of walking, so it should be well past the origin.
    check(position.z > 2.0, "root motion moved the character forward");
    check(sink.footsteps >= 1, "footstep events fired");
    // After moving drops at tick 150, the animator ends in Idle.
    check(animator.layers[0].current_state == 0, "returned to Idle");

    // --- Determinism: a fresh replay of the same inputs reproduces the final state ----
    AnimatorInstance replay{};
    replay.controller = controller_id;
    replay.skeleton = skeleton_id;
    CountingSink replay_sink;
    for (int tick = 0; tick < TICKS; ++tick)
    {
        replay.parameters.set_bool(static_cast<std::uint32_t>(moving_index), moving_at(tick));
        animator_step(controller, database, replay, dt);
        drain_events(replay, 1, replay_sink);
    }
    check(std::memcmp(&animator, &replay, sizeof(AnimatorInstance)) == 0,
          "deterministic: same inputs reproduce byte-exact animator state");
    check(replay_sink.footsteps == sink.footsteps, "deterministic event count");

    // --- Rollback: snapshot at K, run to T, restore, replay K..T, compare -------------
    AnimatorInstance rb{};
    rb.controller = controller_id;
    rb.skeleton = skeleton_id;
    const int K = 90; // mid-walk
    for (int tick = 0; tick < K; ++tick)
    {
        rb.parameters.set_bool(static_cast<std::uint32_t>(moving_index), moving_at(tick));
        animator_step(controller, database, rb, dt);
    }
    AnimatorInstance snapshot{};
    std::memcpy(&snapshot, &rb, sizeof(AnimatorInstance)); // rollback capture
    for (int tick = K; tick < TICKS; ++tick)
    {
        rb.parameters.set_bool(static_cast<std::uint32_t>(moving_index), moving_at(tick));
        animator_step(controller, database, rb, dt);
    }
    AnimatorInstance after_first{};
    std::memcpy(&after_first, &rb, sizeof(AnimatorInstance));

    std::memcpy(&rb, &snapshot, sizeof(AnimatorInstance)); // rollback restore
    for (int tick = K; tick < TICKS; ++tick)
    {
        rb.parameters.set_bool(static_cast<std::uint32_t>(moving_index), moving_at(tick));
        animator_step(controller, database, rb, dt);
    }
    check(std::memcmp(&after_first, &rb, sizeof(AnimatorInstance)) == 0,
          "rollback: restore + replay reproduces byte-exact state");

    if (failures != 0)
    {
        std::printf("[animator_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[animator_demo] OK — state machine, events, root motion, determinism, rollback "
                "(footsteps=%d, z=%.2f)\n",
                sink.footsteps, position.z);
    return 0;
}
