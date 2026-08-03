/**************************************************************************/
/* test_input_gamepad.cpp                                                */
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

// Unit_InputGamepad: the Phase 3 gamepad path exercised without SDL or hardware. The
// stable-slot allocator (GamepadSlotTable) is the SDL-free policy the translator wraps, so
// its hot-plug behaviour is unit-tested directly; the registry and mapper are driven by
// scripted controller events, proving connect/disconnect state and analog processing behave
// identically to a real pad (the events are indistinguishable by construction).

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/input/input_manager.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent connect_event(DeviceId device)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::DeviceConnected;
        return event;
    }

    InputEvent disconnect_event(DeviceId device)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::DeviceDisconnected;
        return event;
    }

    InputEvent axis_event(GamepadAxis axis, float value, DeviceId device = FIRST_GAMEPAD_DEVICE)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::GamepadAxisMotion;
        event.control = static_cast<std::uint16_t>(axis);
        event.value = value;
        return event;
    }
}

TEST(Unit_InputGamepad, SlotTableAssignsLowestFreeAndIsIdempotent)
{
    GamepadSlotTable table;
    EXPECT_EQ(table.connect(100), FIRST_GAMEPAD_DEVICE);
    EXPECT_EQ(table.connect(200), FIRST_GAMEPAD_DEVICE + 1);
    EXPECT_EQ(table.connect(100), FIRST_GAMEPAD_DEVICE) << "reconnecting a live instance keeps its slot";

    EXPECT_EQ(table.device_for(200), FIRST_GAMEPAD_DEVICE + 1);
    EXPECT_EQ(table.instance_for(FIRST_GAMEPAD_DEVICE), 100);
    EXPECT_EQ(table.connected_count(), 2);
}

TEST(Unit_InputGamepad, SlotSurvivesUnplugReplugOfTheSameOrdering)
{
    GamepadSlotTable table;
    const DeviceId pad_a = table.connect(100); // slot 0 (device 2)
    const DeviceId pad_b = table.connect(200); // slot 1 (device 3)
    ASSERT_EQ(pad_a, FIRST_GAMEPAD_DEVICE);
    ASSERT_EQ(pad_b, FIRST_GAMEPAD_DEVICE + 1);

    // Unplug B while A stays connected; replugging B takes the lowest free slot, which is
    // still slot 1 — so a binding made against "gamepad 1" is preserved.
    EXPECT_EQ(table.disconnect(200), pad_b);
    EXPECT_EQ(table.device_for(200), INVALID_DEVICE);
    EXPECT_EQ(table.connect(200), pad_b) << "same ordering => same slot";
}

TEST(Unit_InputGamepad, SlotTableFreesTheLowestSlotFirstAfterAGap)
{
    GamepadSlotTable table;
    const DeviceId first = table.connect(100);
    table.connect(200);
    // Disconnect the first pad, leaving slot 0 free while slot 1 stays occupied.
    EXPECT_EQ(table.disconnect(100), first);
    // A new pad reclaims the now-lowest free slot 0.
    EXPECT_EQ(table.connect(300), first);
    EXPECT_EQ(table.instance_for(first), 300);
}

TEST(Unit_InputGamepad, SlotTableRejectsWhenFull)
{
    GamepadSlotTable table;
    for (int i = 0; i < MAX_GAMEPADS; ++i)
        EXPECT_NE(table.connect(1000 + i), INVALID_DEVICE);
    EXPECT_EQ(table.connect(9999), INVALID_DEVICE) << "no slot left";
}

TEST(Unit_InputGamepad, RegistryTracksConnectAndDisconnect)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_button("Jump").bind(GamepadButton::South);
    input.push_context(gameplay);

    source.enqueue(connect_event(FIRST_GAMEPAD_DEVICE));
    input.begin_frame();
    EXPECT_TRUE(input.registry().connected(FIRST_GAMEPAD_DEVICE));
    EXPECT_EQ(input.registry().default_device(DeviceFamily::Gamepad), FIRST_GAMEPAD_DEVICE);

    source.enqueue(disconnect_event(FIRST_GAMEPAD_DEVICE));
    input.begin_frame();
    EXPECT_FALSE(input.registry().connected(FIRST_GAMEPAD_DEVICE));
    EXPECT_EQ(input.registry().default_device(DeviceFamily::Gamepad), INVALID_DEVICE);
}

TEST(Unit_InputGamepad, TriggerAxialDeadzoneRemapsThroughTheMapper)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    gameplay.add_axis1d("Accelerate").bind(GamepadAxis::RightTrigger, Deadzone::axial(0.10f));
    input.push_context(gameplay);

    source.enqueue(connect_event(FIRST_GAMEPAD_DEVICE));

    // Inside the dead-band: zero.
    source.enqueue(axis_event(GamepadAxis::RightTrigger, 0.05f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Accelerate"), 0.0f, 1e-5f);

    // 0.55 remaps to (0.55 - 0.10) / (1.0 - 0.10) = 0.5.
    source.enqueue(axis_event(GamepadAxis::RightTrigger, 0.55f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Accelerate"), 0.5f, 1e-5f);

    // Full press reaches 1.0.
    source.enqueue(axis_event(GamepadAxis::RightTrigger, 1.0f));
    input.begin_frame();
    EXPECT_NEAR(input.snapshot().axis1d("Accelerate"), 1.0f, 1e-5f);
}
