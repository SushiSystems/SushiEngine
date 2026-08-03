/**************************************************************************/
/* test_input_advanced.cpp                                               */
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

// Unit_InputAdvanced: the AAA-completeness features — input buffering (§2.2), the gesture
// recognizer (§2.4), device-level replay (§6), and the text-input channel (§6). All exercised
// headlessly through the real types with scripted input.

#include <algorithm>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/input/gestures.hpp>
#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/input/replay.hpp>
#include <SushiEngine/input/text_input.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent key(EventType type, Key k)
    {
        InputEvent event;
        event.device = KEYBOARD_DEVICE;
        event.type = type;
        event.control = static_cast<std::uint16_t>(k);
        return event;
    }

    InputEvent touch(EventType type, std::uint16_t id, float x, float y)
    {
        InputEvent event;
        event.device = MOUSE_DEVICE;
        event.type = type;
        event.control = id;
        event.x = x;
        event.y = y;
        return event;
    }
}

TEST(Unit_InputAdvanced, BufferedPressSurvivesAWindowOfTicks)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);
    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    // Tick 0: press.
    source.enqueue(key(EventType::KeyDown, Key::Space));
    input.begin_frame();
    TickSample sample = input.consume_tick_sample();
    EXPECT_TRUE(sample.pressed("Jump"));
    EXPECT_TRUE(sample.pressed_within("Jump", 2));

    // Tick 1: released, but still inside a 2-tick buffer window.
    source.enqueue(key(EventType::KeyUp, Key::Space));
    input.begin_frame();
    sample = input.consume_tick_sample();
    EXPECT_FALSE(sample.pressed("Jump"));
    EXPECT_TRUE(sample.pressed_within("Jump", 2));

    // Tick 2: at the edge of the window.
    input.begin_frame();
    sample = input.consume_tick_sample();
    EXPECT_TRUE(sample.pressed_within("Jump", 2));
    EXPECT_FALSE(sample.pressed_within("Jump", 1));

    // Tick 3: the buffer has expired.
    input.begin_frame();
    sample = input.consume_tick_sample();
    EXPECT_FALSE(sample.pressed_within("Jump", 2));
}

TEST(Unit_InputAdvanced, GestureRecognizerReportsATap)
{
    DeviceRegistry registry;
    GestureRecognizer recognizer(registry);

    registry.ingest(touch(EventType::TouchDown, 0, 100.0f, 100.0f));
    EXPECT_TRUE(recognizer.update(0.05f).empty()) << "a press alone is not yet a gesture";

    registry.ingest(touch(EventType::TouchUp, 0, 101.0f, 100.0f));
    const std::vector<Gesture> gestures = recognizer.update(0.05f);
    ASSERT_EQ(gestures.size(), 1u);
    EXPECT_EQ(gestures[0].type, GestureType::Tap);
}

TEST(Unit_InputAdvanced, GestureRecognizerReportsADrag)
{
    DeviceRegistry registry;
    GestureRecognizer recognizer(registry);

    registry.ingest(touch(EventType::TouchDown, 0, 100.0f, 100.0f));
    recognizer.update(0.016f); // establish the down.

    // Move well past the drag threshold: DragBegin.
    registry.ingest(touch(EventType::TouchMove, 0, 140.0f, 100.0f));
    std::vector<Gesture> gestures = recognizer.update(0.016f);
    ASSERT_FALSE(gestures.empty());
    EXPECT_EQ(gestures.front().type, GestureType::DragBegin);

    // Continued motion: DragMove.
    registry.ingest(touch(EventType::TouchMove, 0, 160.0f, 100.0f));
    gestures = recognizer.update(0.016f);
    ASSERT_FALSE(gestures.empty());
    EXPECT_EQ(gestures.front().type, GestureType::DragMove);
    EXPECT_NEAR(gestures.front().delta.x, 20.0f, 1e-3f);

    // Lift: DragEnd.
    registry.ingest(touch(EventType::TouchUp, 0, 160.0f, 100.0f));
    gestures = recognizer.update(0.016f);
    ASSERT_FALSE(gestures.empty());
    EXPECT_EQ(gestures.front().type, GestureType::DragEnd);
}

TEST(Unit_InputAdvanced, GestureRecognizerReportsALongPress)
{
    DeviceRegistry registry;
    GestureRecognizer recognizer(registry);

    registry.ingest(touch(EventType::TouchDown, 0, 50.0f, 50.0f));
    recognizer.update(0.0f);   // down
    recognizer.update(0.30f);  // held 0.30s
    const std::vector<Gesture> gestures = recognizer.update(0.35f); // held 0.65s > 0.60s
    ASSERT_EQ(gestures.size(), 1u);
    EXPECT_EQ(gestures[0].type, GestureType::LongPress);
}

TEST(Unit_InputAdvanced, RecordedStreamReplaysToTheSameActions)
{
    const auto run = [](InputRecorder* recorder, ScriptedInputSource* preloaded) -> std::vector<bool>
    {
        InputManager input;
        ScriptedInputSource own;
        ScriptedInputSource& source = preloaded != nullptr ? *preloaded : own;
        input.add_source(source);
        if (preloaded == nullptr)
        {
            source.schedule(1, key(EventType::KeyDown, Key::Space));
            source.schedule(3, key(EventType::KeyUp, Key::Space));
        }
        InputContext gameplay{"Gameplay"};
        gameplay.add_button("Jump").bind(Key::Space);
        input.push_context(gameplay);

        std::vector<bool> held;
        for (int frame = 0; frame < 5; ++frame)
        {
            input.begin_frame();
            if (recorder != nullptr)
                recorder->capture(input.frame_events());
            held.push_back(input.snapshot().held("Jump"));
        }
        return held;
    };

    InputRecorder recorder;
    const std::vector<bool> original = run(&recorder, nullptr);

    ScriptedInputSource replay_source;
    recorder.replay_into(replay_source);
    const std::vector<bool> replayed = run(nullptr, &replay_source);

    ASSERT_EQ(original.size(), replayed.size());
    for (std::size_t i = 0; i < original.size(); ++i)
        EXPECT_EQ(original[i], replayed[i]) << "frame " << i;
    // Non-trivial: Jump was actually held at some point.
    EXPECT_NE(std::find(original.begin(), original.end(), true), original.end());
}

TEST(Unit_InputAdvanced, TextChannelAppendsAndBackspacesUtf8)
{
    TextInputChannel channel;
    EXPECT_FALSE(channel.active());

    channel.append("ignored"); // inactive: dropped.
    EXPECT_TRUE(channel.text().empty());

    channel.begin();
    channel.append("a");
    channel.append("\xC3\xA9"); // 'é', two UTF-8 bytes.
    EXPECT_EQ(channel.text(), "a\xC3\xA9");

    channel.backspace(); // removes the whole 'é' code point.
    EXPECT_EQ(channel.text(), "a");

    EXPECT_EQ(channel.take(), "a");
    EXPECT_TRUE(channel.text().empty()) << "take() commits and clears";
}
