/**************************************************************************/
/* test_input_replay_json.cpp                                             */
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

// Unit_InputReplayJSON: the on-disk shape of a recorded input session
// (`input/replay_json.hpp`). It is opt-in for the same reason `bindings_json.hpp` is —
// it is the only replay header that includes nlohmann/json, so the recorder itself
// stays dependency-free — which means the engine deliberately has no consumer for it
// and these cases are what holds the format still.
//
// Two properties carry the file. First, exactness: the values a replay feeds back are
// the values it captured, bit for bit through the text, because a replay whose axis
// drifts by an ULP stops reproducing the session it was recorded from. Second, the
// frame stamp — the one field a plausible-looking format drops, and the only one
// `InputRecorder::replay_into` needs to put an event back on the frame it happened on.
//
// Reads are tolerant on purpose: a truncated or older document loads what it can rather
// than throwing, so a stale replay costs a mismatched session and not a crash.

#include <cstdint>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <SushiEngine/input/events.hpp>
#include <SushiEngine/input/replay.hpp>
#include <SushiEngine/input/replay_json.hpp>
#include <SushiEngine/input/source.hpp>

using namespace SushiEngine::Input;

namespace
{
    /** @brief One event with every field moved off its default. */
    InputEvent event_at(std::uint64_t frame, EventType type, std::uint16_t control)
    {
        InputEvent event;
        event.device = KEYBOARD_DEVICE;
        event.type = type;
        event.control = control;
        event.value = 1.0f;
        event.x = 0.0f;
        event.y = 0.0f;
        event.frame = frame;
        return event;
    }

    /** @brief A session with all three axis-carrying shapes in it: a key, a pointer
     *         move, and an analog stick deflection. */
    std::vector<InputEvent> recorded_session()
    {
        std::vector<InputEvent> events;
        events.push_back(event_at(0, EventType::KeyDown, 42));

        InputEvent motion;
        motion.device = MOUSE_DEVICE;
        motion.type = EventType::MouseMove;
        motion.control = 0;
        motion.value = 0.0f;
        motion.x = 640.5f;
        motion.y = -12.25f;
        motion.frame = 2;
        events.push_back(motion);

        InputEvent stick;
        stick.device = FIRST_GAMEPAD_DEVICE;
        stick.type = EventType::GamepadAxisMotion;
        stick.control = 3;
        // Not a power of two: the value a format that round-trips through a printed
        // decimal with too few digits comes back a little wrong on.
        stick.value = -0.7f;
        stick.frame = 5;
        events.push_back(stick);

        return events;
    }

    void expect_event_equal(const InputEvent& actual, const InputEvent& expected)
    {
        EXPECT_EQ(actual.device, expected.device);
        EXPECT_EQ(actual.type, expected.type);
        EXPECT_EQ(actual.control, expected.control);
        // Exact rather than approximate: a replay is only a replay if it feeds back what
        // it captured, and the format's job is to lose nothing on the way through.
        EXPECT_EQ(actual.value, expected.value);
        EXPECT_EQ(actual.x, expected.x);
        EXPECT_EQ(actual.y, expected.y);
        EXPECT_EQ(actual.frame, expected.frame);
    }
} // namespace

TEST(Unit_InputReplayJSON, ASessionSurvivesTheTextItIsWrittenAs)
{
    const std::vector<InputEvent> session = recorded_session();
    InputRecorder recorder;
    recorder.capture(session);

    // Through the serialized text, not only through the in-memory document: printing and
    // re-reading the numbers is the step the format actually has to survive.
    const std::string text = recording_to_json(recorder).dump();

    InputRecorder loaded;
    recording_from_json(loaded, nlohmann::json::parse(text));

    ASSERT_EQ(loaded.size(), session.size());
    for (std::size_t i = 0; i < session.size(); ++i)
        expect_event_equal(loaded.events()[i], session[i]);
}

TEST(Unit_InputReplayJSON, TheDocumentIsAFlatArrayOfNamedFields)
{
    const nlohmann::json document = events_to_json(recorded_session());

    ASSERT_TRUE(document.is_array());
    ASSERT_EQ(document.size(), 3u);
    // Spelled out once, because the field names are the format: renaming one is a
    // compatible-looking edit that silently reverts every field it touches to a default.
    for (const char* field : {"device", "type", "control", "value", "x", "y", "frame"})
        EXPECT_TRUE(document[0].contains(field)) << field;
    EXPECT_EQ(document[0]["frame"].get<std::uint64_t>(), 0u);
    EXPECT_EQ(document[2]["frame"].get<std::uint64_t>(), 5u);
}

TEST(Unit_InputReplayJSON, ALoadedRecordingReplaysOnTheFramesItWasCapturedOn)
{
    InputRecorder recorder;
    recorder.capture(recorded_session());

    InputRecorder loaded;
    recording_from_json(loaded, recording_to_json(recorder));

    ScriptedInputSource source;
    loaded.replay_into(source);

    // Frames 0, 2 and 5 carry one event each; 1, 3 and 4 carry none. That distribution is
    // exactly what is lost if the `frame` field does not survive the file.
    const std::size_t expected_per_frame[] = {1, 0, 1, 0, 0, 1};
    for (std::size_t frame = 0; frame < 6; ++frame)
    {
        std::vector<InputEvent> drained;
        source.poll(drained);
        EXPECT_EQ(drained.size(), expected_per_frame[frame]) << "frame " << frame;
    }
    EXPECT_TRUE(source.empty());
}

TEST(Unit_InputReplayJSON, ADocumentThatIsNotAnArrayReadsAsAnEmptyRecording)
{
    EXPECT_TRUE(events_from_json(nlohmann::json()).empty());
    EXPECT_TRUE(events_from_json(nlohmann::json::object()).empty());
    EXPECT_TRUE(events_from_json(nlohmann::json("not a recording")).empty());
}

TEST(Unit_InputReplayJSON, AnEventMissingFieldsFallsBackFieldByField)
{
    // What a document written by an older build, or truncated by hand, looks like: the
    // records are there but not every field is.
    const nlohmann::json document = nlohmann::json::parse(R"([{}, {"frame": 7, "x": 3.5}])");

    const std::vector<InputEvent> events = events_from_json(document);

    ASSERT_EQ(events.size(), 2u);
    // A record with nothing in it names no device, rather than defaulting onto the
    // keyboard's slot and injecting a keystroke nobody recorded.
    EXPECT_EQ(events[0].device, INVALID_DEVICE);
    EXPECT_EQ(events[0].frame, 0u);
    EXPECT_EQ(events[0].value, 0.0f);
    EXPECT_EQ(events[1].frame, 7u);
    EXPECT_EQ(events[1].x, 3.5f);
    EXPECT_EQ(events[1].device, INVALID_DEVICE);
}

TEST(Unit_InputReplayJSON, LoadingReplacesTheRecordingRatherThanAppendingToIt)
{
    InputRecorder recorder;
    recorder.capture(recorded_session());
    ASSERT_EQ(recorder.size(), 3u);

    recording_from_json(recorder, nlohmann::json::array());

    EXPECT_TRUE(recorder.empty());
}
