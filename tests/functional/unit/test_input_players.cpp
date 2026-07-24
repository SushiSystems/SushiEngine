/**************************************************************************/
/* test_input_players.cpp                                                */
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

// Unit_InputPlayers: local-multiplayer routing (docs/design/input_manager.md §2.6). One shared
// device state, one shared context, but a mapper per player resolving against its own device
// assignment — so two pads drive two players independently through the same "Move"/"Jump"
// bindings. Also covers "press A to join". Driven headlessly by scripted controller events.

#include <cstdint>

#include <gtest/gtest.h>

#include <SushiEngine/input/input_manager.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    InputEvent connect(DeviceId device)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::DeviceConnected;
        return event;
    }

    InputEvent axis(DeviceId device, GamepadAxis a, float value)
    {
        InputEvent event;
        event.device = device;
        event.type = EventType::GamepadAxisMotion;
        event.control = static_cast<std::uint16_t>(a);
        event.value = value;
        return event;
    }

    InputEvent button(DeviceId device, GamepadButton b, bool down)
    {
        InputEvent event;
        event.device = device;
        event.type = down ? EventType::GamepadButtonDown : EventType::GamepadButtonUp;
        event.control = static_cast<std::uint16_t>(b);
        return event;
    }

    void build_gameplay(InputContext& context)
    {
        context.add_axis2d("Move").bind(GamepadAxis::LeftStick, Deadzone::radial(0.10f));
        context.add_button("Jump").bind(GamepadButton::South);
    }
}

TEST(Unit_InputPlayers, TwoPadsDriveTwoPlayersIndependently)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    // One shared context both players resolve; only the device assignment differs.
    InputContext gameplay{"Gameplay"};
    build_gameplay(gameplay);

    const DeviceId pad0 = FIRST_GAMEPAD_DEVICE;
    const DeviceId pad1 = FIRST_GAMEPAD_DEVICE + 1;

    PlayerRoster roster;
    DeviceAssignment devices0;
    devices0.gamepad = pad0;
    DeviceAssignment devices1;
    devices1.gamepad = pad1;
    PlayerHandle& player0 = roster.add_player(devices0);
    PlayerHandle& player1 = roster.add_player(devices1);
    player0.push_context(gameplay);
    player1.push_context(gameplay);

    // Pad 0 pushes its stick right and holds Jump; pad 1 pushes its stick down, no Jump.
    source.enqueue(connect(pad0));
    source.enqueue(connect(pad1));
    source.enqueue(axis(pad0, GamepadAxis::LeftStickX, 1.0f));
    source.enqueue(button(pad0, GamepadButton::South, true));
    source.enqueue(axis(pad1, GamepadAxis::LeftStickY, 1.0f));

    input.begin_frame();                       // fold events into the shared registry
    roster.update(input.registry(), input.gate());

    // Each player sees only its own pad.
    EXPECT_NEAR(player0.snapshot().axis2d("Move").x, 1.0f, 1e-3f);
    EXPECT_NEAR(player0.snapshot().axis2d("Move").y, 0.0f, 1e-3f);
    EXPECT_TRUE(player0.snapshot().held("Jump"));

    EXPECT_NEAR(player1.snapshot().axis2d("Move").x, 0.0f, 1e-3f);
    EXPECT_NEAR(player1.snapshot().axis2d("Move").y, 1.0f, 1e-3f);
    EXPECT_FALSE(player1.snapshot().held("Jump")) << "player 1's pad did not press Jump";
}

TEST(Unit_InputPlayers, PerPlayerTickSamplesAreIndependent)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    InputContext gameplay{"Gameplay"};
    build_gameplay(gameplay);

    const DeviceId pad0 = FIRST_GAMEPAD_DEVICE;
    const DeviceId pad1 = FIRST_GAMEPAD_DEVICE + 1;
    PlayerRoster roster;
    DeviceAssignment d0;
    d0.gamepad = pad0;
    DeviceAssignment d1;
    d1.gamepad = pad1;
    roster.add_player(d0).push_context(gameplay);
    roster.add_player(d1).push_context(gameplay);

    source.enqueue(connect(pad0));
    source.enqueue(connect(pad1));
    source.enqueue(button(pad0, GamepadButton::South, true));
    input.begin_frame();
    roster.update(input.registry(), input.gate());

    // A tick reduces each player's input separately: only player 0 jumped.
    const TickSample sample0 = roster.player(0).consume_tick_sample();
    const TickSample sample1 = roster.player(1).consume_tick_sample();
    EXPECT_TRUE(sample0.pressed("Jump"));
    EXPECT_FALSE(sample1.pressed("Jump"));
}

TEST(Unit_InputPlayers, JoinCandidatesReportsUnownedPadsPressingTheJoinButton)
{
    InputManager input;
    ScriptedInputSource source;
    input.add_source(source);

    const DeviceId pad0 = FIRST_GAMEPAD_DEVICE;
    const DeviceId pad1 = FIRST_GAMEPAD_DEVICE + 1;

    PlayerRoster roster;
    DeviceAssignment d0;
    d0.gamepad = pad0;
    roster.add_player(d0); // player 0 already owns pad 0.

    // Both pads connect; the unowned pad 1 presses South to join.
    source.enqueue(connect(pad0));
    source.enqueue(connect(pad1));
    source.enqueue(button(pad0, GamepadButton::South, true)); // owned — not a candidate.
    source.enqueue(button(pad1, GamepadButton::South, true));
    input.begin_frame();

    const std::vector<DeviceId> candidates =
        roster.join_candidates(input.registry(), GamepadButton::South);
    ASSERT_EQ(candidates.size(), 1u);
    EXPECT_EQ(candidates[0], pad1);

    // The game claims it for a new player, who then reads that pad.
    PlayerHandle& joined = roster.add_player();
    roster.claim_gamepad(joined.index(), candidates[0]);
    EXPECT_EQ(roster.gamepad_owner(pad1), joined.index());
}
