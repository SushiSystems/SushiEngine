/**************************************************************************/
/* test_input_tick_sample.cpp                                            */
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

// Unit_InputTickSample: the determinism boundary (docs/design/input_manager.md §2.3).
// The fixed-step accumulator runs 0..N ticks per host frame, so raw per-frame action
// state cannot cross into the sim directly. These tests prove the three laws the
// TickSampleAccumulator resolves — sticky edges, one edge per burst, sum-and-clear
// relative axes — plus the quantization that makes a command bit-reproducible. Driven
// through the real InputManager and ScriptedInputSource, no window, no hardware.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/input/input_manager.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent key_event(EventType type, Key key)
    {
        InputEvent event;
        event.device = KEYBOARD_DEVICE;
        event.type = type;
        event.control = static_cast<std::uint16_t>(key);
        return event;
    }

    InputEvent mouse_move_event(float x, float y)
    {
        InputEvent event;
        event.device = MOUSE_DEVICE;
        event.type = EventType::MouseMove;
        event.x = x;
        event.y = y;
        return event;
    }
}

TEST(Unit_InputTickSample, QuantizeAxisIsSymmetricWithExactZero)
{
    EXPECT_EQ(quantize_axis(0.0f), 0);
    EXPECT_EQ(quantize_axis(1.0f), AXIS_QUANTIZED_MAX);
    EXPECT_EQ(quantize_axis(-1.0f), -AXIS_QUANTIZED_MAX);
    EXPECT_EQ(quantize_axis(2.0f), AXIS_QUANTIZED_MAX) << "clamps above 1";
    EXPECT_EQ(quantize_axis(-5.0f), -AXIS_QUANTIZED_MAX) << "clamps below -1";
    // The mapping is a pure function of the clamped input, so it is bit-identical everywhere.
    EXPECT_EQ(quantize_axis(0.5f), quantize_axis(0.5f));
}

TEST(Unit_InputTickSample, ZeroTickTapSurvivesToTheNextTick)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    // Press and release in the same host frame that runs no tick — the level diff alone
    // would lose this. The registry's event-level edge keeps it, sticky until consumed.
    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    source.enqueue(key_event(EventType::KeyUp, Key::Space));
    input.begin_frame(); // zero-tick frame

    const TickSample sample = input.consume_tick_sample();
    EXPECT_TRUE(sample.pressed("Jump")) << "the tap must not vanish";
    EXPECT_FALSE(sample.held("Jump"));
}

TEST(Unit_InputTickSample, StickyEdgeCarriesAcrossSeveralZeroTickFrames)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    input.begin_frame(); // press, no tick
    source.enqueue(key_event(EventType::KeyUp, Key::Space));
    input.begin_frame(); // release, no tick
    input.begin_frame(); // idle, no tick

    const TickSample sample = input.consume_tick_sample();
    EXPECT_TRUE(sample.pressed("Jump"));
    EXPECT_TRUE(sample.released("Jump"));
    EXPECT_FALSE(sample.held("Jump"));
}

TEST(Unit_InputTickSample, BurstSeesOneEdgeButLevelEveryTick)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    // One host frame; the key goes down and stays down.
    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    input.begin_frame();

    // Two ticks in this frame (a hitch), with no accumulate between them.
    const TickSample first = input.consume_tick_sample();
    const TickSample second = input.consume_tick_sample();

    EXPECT_TRUE(first.pressed("Jump")) << "first tick consumes the edge";
    EXPECT_TRUE(first.held("Jump"));
    EXPECT_FALSE(second.pressed("Jump")) << "second tick sees level only";
    EXPECT_TRUE(second.held("Jump"));
}

TEST(Unit_InputTickSample, RelativeAxisSumsAcrossFramesAndClearsOnConsume)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext viewport{"Viewport"};
    viewport.add_axis2d("Look").bind_mouse();
    input.push_context(viewport);

    source.enqueue(mouse_move_event(100.0f, 100.0f));
    input.begin_frame(); // establish reference, delta 0
    source.enqueue(mouse_move_event(110.0f, 102.0f));
    input.begin_frame(); // +10, +2
    source.enqueue(mouse_move_event(115.0f, 102.0f));
    input.begin_frame(); // +5, +0 (two frames, no tick between)

    // The tick sees the sum of the deltas since the last tick.
    const TickSample first = input.consume_tick_sample();
    EXPECT_NEAR(first.axis2d("Look").x, 15.0f, 1e-4f);
    EXPECT_NEAR(first.axis2d("Look").y, 2.0f, 1e-4f);

    // A second tick with no new motion reads zero — the delta was consumed.
    const TickSample second = input.consume_tick_sample();
    EXPECT_NEAR(second.axis2d("Look").x, 0.0f, 1e-4f);
    EXPECT_NEAR(second.axis2d("Look").y, 0.0f, 1e-4f);
}

TEST(Unit_InputTickSample, AbsoluteAxisIsLatestWinsAndPersistsAcrossABurst)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_axis2d("Move").bind(GamepadAxis::LeftStick, Deadzone::radial(0.10f));
    input.push_context(gameplay);

    InputEvent connect;
    connect.device = FIRST_GAMEPAD_DEVICE;
    connect.type = EventType::DeviceConnected;
    source.enqueue(connect);

    InputEvent axis;
    axis.device = FIRST_GAMEPAD_DEVICE;
    axis.type = EventType::GamepadAxisMotion;
    axis.control = static_cast<std::uint16_t>(GamepadAxis::LeftStickX);
    axis.value = 1.0f;
    source.enqueue(axis);
    input.begin_frame();

    // An absolute stick value is a level, not a delta: every tick in a burst reads it, and
    // consuming does not zero it.
    const TickSample first = input.consume_tick_sample();
    const TickSample second = input.consume_tick_sample();
    EXPECT_NEAR(first.axis2d("Move").x, 1.0f, 1e-5f);
    EXPECT_NEAR(second.axis2d("Move").x, 1.0f, 1e-5f);
}
