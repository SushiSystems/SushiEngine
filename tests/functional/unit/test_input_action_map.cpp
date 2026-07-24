/**************************************************************************/
/* test_input_action_map.cpp                                             */
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

// Unit_Input: the header-only action layer, driven end to end through the real
// ScriptedInputSource (no mocks, no window, no hardware — the design's Phase 1
// acceptance). Each test schedules events, folds one or more host frames through the
// InputManager, and asserts the resolved ActionSnapshot. Because device state is
// folded from events, a scripted event is indistinguishable from a hardware one, so
// these tests exercise the exact code path a windowed build runs.

#include <cmath>

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
        event.value = type == EventType::KeyDown ? 1.0f : 0.0f;
        return event;
    }

    InputEvent gamepad_button_event(EventType type, GamepadButton button, DeviceId device = FIRST_GAMEPAD_DEVICE)
    {
        InputEvent event;
        event.device = device;
        event.type = type;
        event.control = static_cast<std::uint16_t>(button);
        event.value = type == EventType::GamepadButtonDown ? 1.0f : 0.0f;
        return event;
    }

    InputEvent gamepad_axis_event(GamepadAxis axis, float value, DeviceId device = FIRST_GAMEPAD_DEVICE)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::GamepadAxisMotion;
        event.control = static_cast<std::uint16_t>(axis);
        event.value = value;
        return event;
    }

    InputEvent connect_event(DeviceId device)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::DeviceConnected;
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

TEST(Unit_Input, ButtonBindingReportsEdgesAndLevel)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    // Frame 0: press. Edge (pressed) and level (held) both true.
    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().pressed("Jump"));
    EXPECT_TRUE(input.snapshot().held("Jump"));
    EXPECT_FALSE(input.snapshot().released("Jump"));

    // Frame 1: still down, no new event. Held stays, edge clears.
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().pressed("Jump"));
    EXPECT_TRUE(input.snapshot().held("Jump"));

    // Frame 2: release. Falling edge, no longer held.
    source.enqueue(key_event(EventType::KeyUp, Key::Space));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump"));
    EXPECT_TRUE(input.snapshot().released("Jump"));
}

TEST(Unit_Input, KeyboardAndGamepadDriveTheSameAction)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space).bind(GamepadButton::South);
    input.push_context(gameplay);

    source.enqueue(connect_event(FIRST_GAMEPAD_DEVICE));
    source.enqueue(gamepad_button_event(EventType::GamepadButtonDown, GamepadButton::South));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().held("Jump")) << "gamepad South must resolve the shared Jump action";

    source.enqueue(gamepad_button_event(EventType::GamepadButtonUp, GamepadButton::South));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump"));
}

TEST(Unit_Input, CompositeAxis2DNormalizesTheDiagonal)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_axis2d("Move").bind_composite(Key::W, Key::S, Key::A, Key::D);
    input.push_context(gameplay);

    // A single cardinal press is unit length.
    source.enqueue(key_event(EventType::KeyDown, Key::D));
    input.begin_frame();
    Vector2 move = input.snapshot().axis2d("Move");
    EXPECT_NEAR(move.x, 1.0f, 1e-5f);
    EXPECT_NEAR(move.y, 0.0f, 1e-5f);

    // W + D is a diagonal, renormalized so it is not faster than a cardinal push.
    source.enqueue(key_event(EventType::KeyDown, Key::W));
    input.begin_frame();
    move = input.snapshot().axis2d("Move");
    EXPECT_NEAR(move.x, 0.70710678f, 1e-4f);
    EXPECT_NEAR(move.y, 0.70710678f, 1e-4f);
    EXPECT_NEAR(std::sqrt(move.x * move.x + move.y * move.y), 1.0f, 1e-4f);
}

TEST(Unit_Input, CompositeAxis1DSubtractsNegativeFromPositive)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_axis1d("Lift").bind_composite(Key::Q, Key::E);
    input.push_context(gameplay);

    source.enqueue(key_event(EventType::KeyDown, Key::E));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Lift"), 1.0f, 1e-5f);

    source.enqueue(key_event(EventType::KeyDown, Key::Q));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Lift"), 0.0f, 1e-5f) << "both held cancels";

    source.enqueue(key_event(EventType::KeyUp, Key::E));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Lift"), -1.0f, 1e-5f);
}

TEST(Unit_Input, RadialDeadzoneZeroesInsideAndRenormalizesOutside)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_axis2d("Move").bind(GamepadAxis::LeftStick, Deadzone::radial(0.20f));
    input.push_context(gameplay);

    source.enqueue(connect_event(FIRST_GAMEPAD_DEVICE));

    // Inside the dead-band: fully suppressed.
    source.enqueue(gamepad_axis_event(GamepadAxis::LeftStickX, 0.10f));
    source.enqueue(gamepad_axis_event(GamepadAxis::LeftStickY, 0.0f));
    input.begin_frame();
    Vector2 move = input.snapshot().axis2d("Move");
    EXPECT_NEAR(move.x, 0.0f, 1e-5f);
    EXPECT_NEAR(move.y, 0.0f, 1e-5f);

    // Full deflection on X reaches 1.0 after renormalization; below 1 it is remapped.
    source.enqueue(gamepad_axis_event(GamepadAxis::LeftStickX, 1.0f));
    input.begin_frame();
    move = input.snapshot().axis2d("Move");
    EXPECT_NEAR(move.x, 1.0f, 1e-5f);

    // Halfway (0.6) maps to (0.6 - 0.2) / (1.0 - 0.2) = 0.5.
    source.enqueue(gamepad_axis_event(GamepadAxis::LeftStickX, 0.6f));
    input.begin_frame();
    move = input.snapshot().axis2d("Move");
    EXPECT_NEAR(move.x, 0.5f, 1e-5f);
}

TEST(Unit_Input, HigherContextMasksLowerByConsumingControls)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Fire").bind(Key::Space);
    InputContext menu{"Menu"};
    menu.add_button("Confirm").bind(Key::Space);

    input.push_context(gameplay);
    input.push_context(menu); // Menu is now highest priority.

    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().held("Confirm")) << "menu owns Space";
    EXPECT_FALSE(input.snapshot().held("Fire")) << "gameplay Space is masked by the menu";

    // Pop the menu and gameplay regains the control.
    input.pop_context();
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().held("Fire"));
}

TEST(Unit_Input, GateSuppressesKeyboardButNotGamepad)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space).bind(GamepadButton::South);
    input.push_context(gameplay);

    source.enqueue(connect_event(FIRST_GAMEPAD_DEVICE));
    source.enqueue(key_event(EventType::KeyDown, Key::Space));
    source.enqueue(gamepad_button_event(EventType::GamepadButtonDown, GamepadButton::South));

    InputGate gate;
    gate.want_capture_keyboard = true; // an ImGui text field owns the keyboard.
    input.set_gate(gate);
    input.begin_frame();

    // The keyboard source is gated off, but the gamepad still drives the action.
    EXPECT_TRUE(input.snapshot().held("Jump"));

    // With the gamepad released and the keyboard still gated, the action goes quiet.
    source.enqueue(gamepad_button_event(EventType::GamepadButtonUp, GamepadButton::South));
    input.set_gate(gate);
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump")) << "gated keyboard cannot revive the action";
}

TEST(Unit_Input, RelativeMouseAxisAccumulatesWithinFrameAndResets)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext viewport{"Viewport"};
    viewport.add_axis2d("Look").bind_mouse();
    input.push_context(viewport);

    // First position establishes the reference; no delta yet.
    source.enqueue(mouse_move_event(100.0f, 100.0f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis2d("Look").x, 0.0f, 1e-5f);

    // Two motions in one frame sum to the total delta since the last frame.
    source.enqueue(mouse_move_event(110.0f, 105.0f));
    source.enqueue(mouse_move_event(115.0f, 105.0f));
    input.begin_frame();
    Vector2 look = input.snapshot().axis2d("Look");
    EXPECT_NEAR(look.x, 15.0f, 1e-5f);
    EXPECT_NEAR(look.y, 5.0f, 1e-5f);

    // A frame with no motion reports zero — a delta is a per-frame quantity.
    input.begin_frame();
    look = input.snapshot().axis2d("Look");
    EXPECT_NEAR(look.x, 0.0f, 1e-5f);
    EXPECT_NEAR(look.y, 0.0f, 1e-5f);
}

TEST(Unit_Input, ChordGatesAButtonUntilItsModifierIsHeld)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext editor{"Editor"};
    editor.add_button("Undo").bind(Key::Z, Key::LeftControl); // Ctrl+Z
    input.push_context(editor);

    // Z alone does nothing — the chord's modifier is not held.
    source.enqueue(key_event(EventType::KeyDown, Key::Z));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Undo"));

    // Add Ctrl and the chord is satisfied.
    source.enqueue(key_event(EventType::KeyDown, Key::LeftControl));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().held("Undo"));
}
