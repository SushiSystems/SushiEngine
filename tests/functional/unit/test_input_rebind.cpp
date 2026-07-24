/**************************************************************************/
/* test_input_rebind.cpp                                                 */
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

// Unit_InputRebind: runtime rebinding (docs/design/input_manager.md §2.5). The listener
// captures the next control of an expected shape — deaf to the other kinds and to stick
// drift — and cancels on Escape or timeout; the conflict check and apply helper let a UI
// rebind an action without any consumer code changing. All header-only, driven by raw
// event lists, no window.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/input/input_manager.hpp>
#include <SushiEngine/input/rebinding.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent key_down(Key key)
    {
        InputEvent event;
        event.device = KEYBOARD_DEVICE;
        event.type = EventType::KeyDown;
        event.control = static_cast<std::uint16_t>(key);
        return event;
    }

    InputEvent gamepad_axis(GamepadAxis axis, float value)
    {
        InputEvent event;
        event.device = FIRST_GAMEPAD_DEVICE;
        event.type = EventType::GamepadAxisMotion;
        event.control = static_cast<std::uint16_t>(axis);
        event.value = value;
        return event;
    }
}

TEST(Unit_InputRebind, CapturesAButtonPress)
{
    RebindingListener listener;
    listener.begin(RebindShape::Button);
    EXPECT_TRUE(listener.listening());

    const std::vector<InputEvent> events{key_down(Key::J)};
    EXPECT_EQ(listener.update(events), RebindStatus::Captured);
    EXPECT_EQ(listener.captured(), control_of(Key::J));
}

TEST(Unit_InputRebind, ButtonShapeIgnoresAxisNoiseAndAxisShapeIgnoresButtons)
{
    RebindingListener button;
    button.begin(RebindShape::Button);
    EXPECT_EQ(button.update({gamepad_axis(GamepadAxis::LeftStickX, 1.0f)}), RebindStatus::Listening);
    EXPECT_EQ(button.update({key_down(Key::K)}), RebindStatus::Captured);

    RebindingListener axis;
    axis.begin(RebindShape::Axis, 0.5f);
    EXPECT_EQ(axis.update({key_down(Key::K)}), RebindStatus::Listening);
    EXPECT_EQ(axis.update({gamepad_axis(GamepadAxis::RightStickY, 0.9f)}), RebindStatus::Captured);
    EXPECT_EQ(axis.captured(), control_of(GamepadAxis::RightStickY));
}

TEST(Unit_InputRebind, AxisCaptureRequiresDeflectionPastTheThreshold)
{
    RebindingListener listener;
    listener.begin(RebindShape::Axis, 0.5f);
    // Drift below the threshold must not bind the axis.
    EXPECT_EQ(listener.update({gamepad_axis(GamepadAxis::LeftStickX, 0.2f)}), RebindStatus::Listening);
    // A deliberate push past it does.
    EXPECT_EQ(listener.update({gamepad_axis(GamepadAxis::LeftStickX, 0.8f)}), RebindStatus::Captured);
}

TEST(Unit_InputRebind, EscapeCancels)
{
    RebindingListener listener;
    listener.begin(RebindShape::Button);
    EXPECT_EQ(listener.update({key_down(Key::Escape)}), RebindStatus::Cancelled);
}

TEST(Unit_InputRebind, TimeoutCancels)
{
    RebindingListener listener;
    listener.begin(RebindShape::Button, 0.5f, 1.0f);
    EXPECT_EQ(listener.update({}, 0.5f), RebindStatus::Listening);
    EXPECT_EQ(listener.update({}, 0.6f), RebindStatus::Cancelled) << "1.1s elapsed exceeds the 1s timeout";
}

TEST(Unit_InputRebind, ConflictFindsAnotherActionSharingTheControl)
{
    InputContext context{"Gameplay"};
    context.add_button("Jump").bind(Key::Space);
    context.add_button("Crouch").bind(Key::LeftControl);

    // Space is used by Jump: a conflict for anyone else, not for Jump itself.
    EXPECT_NE(binding_conflict(context, control_of(Key::Space), "Crouch"), nullptr);
    EXPECT_EQ(binding_conflict(context, control_of(Key::Space), "Jump"), nullptr);
    // An unused control is free.
    EXPECT_EQ(binding_conflict(context, control_of(Key::Q), "Jump"), nullptr);
}

TEST(Unit_InputRebind, ApplyRebindTakesEffectThroughTheMapper)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(Key::Space);
    input.push_context(gameplay);

    // Rebind Jump from Space to J.
    Action* jump = gameplay.find_action("Jump");
    ASSERT_NE(jump, nullptr);
    set_button_binding(*jump, control_of(Key::J));

    // The old key no longer fires; the new one does — with no consumer change.
    source.enqueue(key_down(Key::Space));
    input.begin_frame();
    EXPECT_FALSE(input.snapshot().held("Jump"));

    source.enqueue(key_down(Key::J));
    input.begin_frame();
    EXPECT_TRUE(input.snapshot().held("Jump"));
}
