/**************************************************************************/
/* sequence_timeline_demo.cpp                                            */
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

// The §12.4 minimal sequencer timeline core, worked and self-checked. A two-key float
// track (a camera field-of-view, parameter slot 0) and three timed events (a clip-start cue,
// a sound cue, a camera cut) prove:
//   * evaluate() is a pure function of time — scrubbing to any time, forward or backward,
//     gives the correct linearly-interpolated value, including clamped-at-the-ends reads.
//   * advance() fires exactly the events crossed in (previous_time, current_time], in time
//     order, including firing MULTIPLE events when a single large step crosses more than one
//     (a dropped-frame-sized step, not just single-tick playback).
//   * advance() fires nothing on a backward scrub (current_time <= previous_time) and nothing
//     on a zero-length step — it is not re-triggered by scrubbing back over already-fired
//     events, matching a real playhead's behavior.

#include <cstdio>
#include <vector>

#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/animation/sequence_timeline.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Animation;

namespace
{
    int failures = 0;
    void check(bool condition, const char* what)
    {
        if (!condition)
        {
            std::printf("[sequence_timeline_demo] FAIL: %s\n", what);
            ++failures;
        }
    }

    bool nearly(float a, float b, float eps) { return (a > b ? a - b : b - a) <= eps; }
}

int main()
{
    SequenceTimeline timeline;

    SequenceFloatTrack fov_track;
    fov_track.parameter_index = 0;
    fov_track.keys = {SequenceFloatKey{0.0f, 60.0f}, SequenceFloatKey{4.0f, 90.0f}};
    timeline.float_tracks.push_back(fov_track);

    timeline.events = {
        SequenceEvent{1.0f, /*event_id=*/10}, // clip start
        SequenceEvent{2.5f, /*event_id=*/20}, // sound cue
        SequenceEvent{4.0f, /*event_id=*/30}, // camera cut
    };

    AnimatorParameterBlock parameters;

    // --- evaluate(): pure function of time, including scrub-backward and clamped ends. ---
    timeline.evaluate(0.0f, parameters);
    check(nearly(parameters.values[0].as_float, 60.0f, 1e-4f), "t=0 reads the first key exactly");

    timeline.evaluate(2.0f, parameters); // halfway through [0,4] -> halfway through [60,90]
    check(nearly(parameters.values[0].as_float, 75.0f, 1e-4f), "t=2 (midpoint) interpolates to 75");

    timeline.evaluate(10.0f, parameters); // past the last key
    check(nearly(parameters.values[0].as_float, 90.0f, 1e-4f), "t=10 clamps to the last key's value");

    timeline.evaluate(-5.0f, parameters); // before the first key
    check(nearly(parameters.values[0].as_float, 60.0f, 1e-4f), "t=-5 clamps to the first key's value");

    timeline.evaluate(0.5f, parameters); // scrubbing backward from t=10 to t=0.5 must still be exact
    check(nearly(parameters.values[0].as_float, 60.0f + 30.0f * (0.5f / 4.0f), 1e-3f),
         "scrubbing to an earlier time still interpolates correctly (no hidden state)");

    // --- advance(): forward-only, exactly the events in (previous, current]. -------------
    std::vector<std::uint32_t> fired;
    timeline.advance(0.0f, 0.5f, fired);
    check(fired.empty(), "no event fires before its own time (0.5 < first event at 1.0)");

    timeline.advance(0.5f, 1.5f, fired);
    check(fired.size() == 1 && fired[0] == 10, "crossing t=1.0 fires exactly the clip-start event");

    timeline.advance(1.5f, 1.5f, fired);
    check(fired.size() == 1, "a zero-length step fires nothing new");

    // A single large step crossing both the sound cue (2.5) and the camera cut (4.0).
    timeline.advance(1.5f, 5.0f, fired);
    check(fired.size() == 3 && fired[1] == 20 && fired[2] == 30,
         "a large step fires every crossed event, in time order, not just the last");

    // Scrubbing backward over already-fired events must not refire them.
    std::vector<std::uint32_t> fired_after_scrub_back;
    timeline.advance(5.0f, 0.0f, fired_after_scrub_back);
    check(fired_after_scrub_back.empty(), "scrubbing backward fires nothing");

    if (failures != 0)
    {
        std::printf("[sequence_timeline_demo] %d check(s) failed\n", failures);
        return 1;
    }
    std::printf(
        "[sequence_timeline_demo] OK — pure-function float-track evaluation and forward-only "
        "event dispatch both verified\n");
    return 0;
}
