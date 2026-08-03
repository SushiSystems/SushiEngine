/**************************************************************************/
/* morph_demo.cpp                                                        */
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

// The phase-A7 morph targets and generic tracks, worked and self-checked headlessly. A clip
// carries a morph-weight track ("smile") and a generic float track ("emissive") alongside its
// joint tracks. Four things are proved: the morph and generic tracks round-trip through the
// .sushianim v2 cook/load and sample with interpolation; the sampler maps the clip's morph
// tracks onto a mesh's target order by name (an unmatched target stays zero); the CPU morph
// reference blends base vertices by the sampled weights; and a generic track drives a bound
// float through the IFloatSink registry.

#include <cmath>
#include <cstdio>
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
            std::printf("[morph_demo] FAIL: %s\n", what);
            ++failures;
        }
    }
    bool nearly(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }
}

int main()
{
    AnimationDatabase database;

    // A clip: two joints (identity tracks), a "smile" morph track and an "emissive" generic track
    // over three frames at 30 Hz — smile 0 -> 0.5 -> 1, emissive 1 -> 2 -> 3.
    ClipDesc clip;
    clip.joint_count = 2;
    clip.frame_count = 3;
    clip.sample_rate = 30.0f;
    clip.translations.assign(6, Vector3f{0, 0, 0});
    clip.rotations.assign(6, Quaternionf{0, 0, 0, 1});
    clip.scales.assign(6, Vector3f{1, 1, 1});
    clip.morph_names = {"smile"};
    clip.morph_weights = {0.0f, 0.5f, 1.0f};
    clip.generic_names = {"emissive"};
    clip.generic_values = {1.0f, 2.0f, 3.0f};

    std::vector<std::byte> blob;
    check(build_clip_blob(clip, blob), "clip v2 cooks");
    const AssetId clip_id = database.add_clip(std::move(blob));
    check(clip_id != INVALID_ASSET, "clip registers");
    const ClipView view = database.clip(clip_id);
    check(view.morph_track_count == 1 && view.generic_track_count == 1, "v2 tracks round-trip");

    // --- Sampling: exact frame and mid-frame interpolation ---------------------------
    {
        float weight;
        float value;
        view.sample_morph(1.0f / 30.0f, false, &weight); // frame 1 exactly
        view.sample_generic(1.0f / 30.0f, false, &value);
        check(nearly(weight, 0.5f), "morph samples 0.5 at frame 1");
        check(nearly(value, 2.0f), "generic samples 2.0 at frame 1");

        view.sample_morph(0.5f / 30.0f, false, &weight); // halfway to frame 1
        check(nearly(weight, 0.25f), "morph interpolates to 0.25 mid-frame");
    }

    // --- Mesh target mapping: clip tracks resolve onto the mesh's target order --------
    {
        // The mesh's morph targets, in its order: "smile" then "frown" (the clip has no frown).
        const NameHash target_names[2] = {hash_name("smile"), hash_name("frown")};
        MorphState state;
        sample_morph_state(view, 1.0f / 30.0f, false, target_names, 2, state);
        check(state.count == 2, "morph state matches the mesh target count");
        check(nearly(state.weights[0], 0.5f), "smile target takes the clip's weight");
        check(nearly(state.weights[1], 0.0f), "frown target (absent in clip) stays zero");

        // CPU morph reference: one vertex at the origin, smile pushes +y, frown pushes +x.
        const Vector3f base[1] = {Vector3f{0, 0, 0}};
        const Vector3f smile_delta[1] = {Vector3f{0, 1, 0}};
        const Vector3f frown_delta[1] = {Vector3f{1, 0, 0}};
        MorphTargetDeltas targets[2];
        targets[0].position_deltas = smile_delta;
        targets[1].position_deltas = frown_delta;
        Vector3f morphed[1];
        apply_morph_positions(base, 1, targets, state.weights, state.count, morphed);
        check(nearly(morphed[0].x, 0.0f) && nearly(morphed[0].y, 0.5f) && nearly(morphed[0].z, 0.0f),
              "CPU morph blends the vertex by the smile weight");
    }

    // --- Generic tracks: a bound float receives the sampled value ---------------------
    {
        float emissive = 0.0f;
        GenericBindingRegistry registry;
        registry.bind(hash_name("emissive"), &emissive);
        registry.bind(hash_name("unused"), nullptr); // an unbound-target entry is safely ignored

        apply_generic_tracks(view, 2.0f / 30.0f, false, registry); // frame 2
        check(nearly(emissive, 3.0f), "generic track drives the bound float to 3.0");
        apply_generic_tracks(view, 1.0f / 30.0f, false, registry);
        check(nearly(emissive, 2.0f), "generic track re-drives the bound float to 2.0");
    }

    if (failures != 0)
    {
        std::printf("[morph_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf("[morph_demo] OK — morph + generic v2 tracks, mesh-target mapping, CPU morph blend, "
                "and generic binding registry all verified\n");
    return 0;
}
