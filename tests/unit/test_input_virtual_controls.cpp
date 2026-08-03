/**************************************************************************/
/* test_input_virtual_controls.cpp                                       */
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

// Unit_InputVirtualControls: touch and virtual controls (docs/design/input_manager.md §2.4).
// A VirtualControlSource synthesizes gamepad-shaped events from pointers, so a virtual stick
// drives the very same "Move" binding a hardware stick would — the design's OCP claim, proved
// by resolving through the unchanged binding path. Also covers mouse-as-pointer-0 and the
// UI::PointerInput feed. Driven by scripted pointer state, no window.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/input/ui_pointer.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent mouse_button(bool down, float x, float y)
    {
        InputEvent event;
        event.device = MOUSE_DEVICE;
        event.type = down ? EventType::MouseButtonDown : EventType::MouseButtonUp;
        event.control = static_cast<std::uint16_t>(MouseButton::Left);
        event.x = x;
        event.y = y;
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

TEST(Unit_InputVirtualControls, VirtualStickDrivesTheUnchangedMoveBinding)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);
    input.registry_mutable().set_mouse_as_pointer(true); // develop the touch UI with a mouse.

    VirtualControlSource controls(input.registry());
    controls.add_stick(VirtualStick{Vector2{100.0f, 100.0f}, 50.0f,
                                    GamepadAxis::LeftStickX, GamepadAxis::LeftStickY});
    input.add_virtual_source(controls);

    // The exact Move binding a keyboard/pad game already has — composite AND stick. The virtual
    // stick resolves through the stick path with no gameplay-code difference.
    InputContext gameplay{"Gameplay"};
    gameplay.add_axis2d("Move")
        .bind_composite(Key::W, Key::S, Key::A, Key::D)
        .bind(GamepadAxis::LeftStick, Deadzone::radial(0.10f));
    input.push_context(gameplay);

    // A pointer at the stick's right edge is full right deflection.
    source.enqueue(mouse_button(true, 150.0f, 100.0f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis2d("Move").x, 1.0f, 1e-3f);
    EXPECT_NEAR(input.snapshot().axis2d("Move").y, 0.0f, 1e-3f);

    // Releasing the pointer re-centres the virtual stick to zero.
    source.enqueue(mouse_button(false, 150.0f, 100.0f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis2d("Move").x, 0.0f, 1e-3f);
}

TEST(Unit_InputVirtualControls, VirtualButtonPressesAGamepadButton)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    VirtualControlSource controls(input.registry());
    controls.add_button(VirtualButton{Vector2{200.0f, 200.0f}, 30.0f, GamepadButton::South});
    input.add_virtual_source(controls);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(GamepadButton::South);
    input.push_context(gameplay);

    // A finger inside the button region presses it — through the same GamepadButton binding.
    source.enqueue(touch(EventType::TouchDown, 1, 200.0f, 205.0f));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().pressed("Jump"));
    EXPECT_TRUE(input.snapshot().held("Jump"));

    // Lifting the finger releases it.
    source.enqueue(touch(EventType::TouchUp, 1, 200.0f, 205.0f));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump"));
    EXPECT_TRUE(input.snapshot().released("Jump"));
}

TEST(Unit_InputVirtualControls, PointerOutsideTheRegionDoesNothing)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    VirtualControlSource controls(input.registry());
    controls.add_button(VirtualButton{Vector2{200.0f, 200.0f}, 30.0f, GamepadButton::South});
    input.add_virtual_source(controls);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(GamepadButton::South);
    input.push_context(gameplay);

    // Far outside the 30px radius: no press.
    source.enqueue(touch(EventType::TouchDown, 0, 400.0f, 400.0f));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump"));
}

TEST(Unit_InputVirtualControls, MouseFeedsPointerZeroAndTheUiPointer)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);
    input.registry_mutable().set_mouse_as_pointer(true);

    InputContext dummy{"Dummy"};
    input.push_context(dummy);

    source.enqueue(mouse_button(true, 40.0f, 60.0f));
    input.begin_frame();

    const DeviceRegistry::Pointer& pointer = input.registry().primary_pointer();
    EXPECT_TRUE(pointer.active);
    EXPECT_NEAR(pointer.position.x, 40.0f, 1e-5f);
    EXPECT_NEAR(pointer.position.y, 60.0f, 1e-5f);

    // The UI adapter reads the same primary pointer.
    const UI::PointerInput ui = ui_pointer(input.registry());
    EXPECT_TRUE(ui.down);
    EXPECT_NEAR(static_cast<float>(ui.position.x), 40.0f, 1e-5f);
    EXPECT_NEAR(static_cast<float>(ui.position.y), 60.0f, 1e-5f);
}
