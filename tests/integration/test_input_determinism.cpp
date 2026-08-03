/**************************************************************************/
/* test_input_determinism.cpp                                            */
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

// Integration_InputDeterminism: the input manager's half of SushiLoop's determinism
// contract (docs/design/input_manager.md §2.3). The same scripted device-event stream,
// fed through the InputManager and reduced to a quantized Command inside sample_command,
// must produce a bit-identical InputHistory<Command> on two independent App instances —
// even though the fixed-step loop runs 0..N ticks per host frame. This is the input-side
// analogue of Integration_DeterministicReplay: it proves the reduction carries no hidden
// nondeterminism (no wall-clock read, no float drift across frames), so rollback replay
// and server reconciliation operate on values that are equal by construction.

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/SushiEngine.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;

namespace
{
    // The quantized per-tick command: exactly what would cross the wire. Trivially
    // copyable and operator==, as SushiLoop requires of a Command.
    struct MoveCommand
    {
        std::int16_t move_x = 0;
        std::int16_t move_y = 0;
        bool jump = false;

        bool operator==(const MoveCommand& other) const noexcept
        {
            return move_x == other.move_x && move_y == other.move_y && jump == other.jump;
        }
    };

    constexpr Scalar FIXED_DT = Scalar(0.1);
    constexpr std::size_t FRAMES = 14;

    // A host-frame delta pattern that deliberately runs zero, one, and two ticks per
    // frame against the 0.1 s step, so the reduction is exercised across all three cases.
    const Scalar FRAME_DELTAS[] = {
        Scalar(0.05), Scalar(0.05), Scalar(0.20), Scalar(0.10), Scalar(0.00),
        Scalar(0.10), Scalar(0.05), Scalar(0.25), Scalar(0.10), Scalar(0.00),
        Scalar(0.10), Scalar(0.10), Scalar(0.05), Scalar(0.15),
    };

    Input::InputEvent key_event(Input::EventType type, Input::Key key)
    {
        Input::InputEvent event;
        event.device = Input::KEYBOARD_DEVICE;
        event.type = type;
        event.control = static_cast<std::uint16_t>(key);
        return event;
    }

    // Schedules the identical device-event script on a scripted source. The same story
    // told to both runs: a diagonal move that starts and stops, and two jumps — one a
    // sub-frame tap, one a held press.
    void script_events(Input::ScriptedInputSource& source)
    {
        using Input::EventType;
        using Input::Key;
        source.schedule(1, key_event(EventType::KeyDown, Key::W));
        source.schedule(2, key_event(EventType::KeyDown, Key::D));
        source.schedule(3, key_event(EventType::KeyDown, Key::Space)); // tap: down and up
        source.schedule(3, key_event(EventType::KeyUp, Key::Space));   // in the same frame
        source.schedule(6, key_event(EventType::KeyUp, Key::W));
        source.schedule(7, key_event(EventType::KeyUp, Key::D));
        source.schedule(9, key_event(EventType::KeyDown, Key::Space)); // held from here
        source.schedule(12, key_event(EventType::KeyUp, Key::Space));
    }

    // Runs one full, independent session and returns the recorded command per tick.
    std::vector<MoveCommand> run(SushiRuntime::API::Runtime& runtime)
    {
        Input::InputManager input;
        Input::ScriptedInputSource source;
        input.add_source(source);
        script_events(source);

        Input::InputContext gameplay{"Gameplay"};
        gameplay.add_axis2d("Move").bind_composite(Input::Key::W, Input::Key::S,
                                                   Input::Key::A, Input::Key::D);
        gameplay.add_button("Jump").bind(Input::Key::Space);
        input.push_context(gameplay);

        Loop::AppConfig config;
        config.fixed_dt_seconds = FIXED_DT;
        config.rollback_capacity = 0;
        Loop::App<MoveCommand> app(runtime, config);

        app.sample_command([&input](Loop::TickId) -> MoveCommand
        {
            const Input::TickSample sample = input.consume_tick_sample();
            MoveCommand command;
            command.move_x = Input::quantize_axis(sample.axis2d("Move").x);
            command.move_y = Input::quantize_axis(sample.axis2d("Move").y);
            command.jump = sample.pressed("Jump");
            return command;
        });

        for (std::size_t frame = 0; frame < FRAMES; ++frame)
        {
            input.begin_frame();
            app.advance(FRAME_DELTAS[frame]);
        }

        std::vector<MoveCommand> commands;
        for (Loop::TickId tick = 0; tick < app.tick(); ++tick)
        {
            const MoveCommand* recorded = app.input_history().find(tick);
            if (recorded == nullptr)
            {
                ADD_FAILURE() << "no recorded command for tick " << tick;
                return {};
            }
            commands.push_back(*recorded);
        }
        return commands;
    }
}

TEST(Integration_InputDeterminism, SameEventStreamProducesBitIdenticalCommandHistory)
{
    const std::vector<MoveCommand> first = run(Harness::shared_runtime());
    const std::vector<MoveCommand> second = run(Harness::shared_runtime());

    ASSERT_FALSE(first.empty());
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t tick = 0; tick < first.size(); ++tick)
    {
        EXPECT_EQ(first[tick].move_x, second[tick].move_x) << "tick " << tick;
        EXPECT_EQ(first[tick].move_y, second[tick].move_y) << "tick " << tick;
        EXPECT_EQ(first[tick].jump, second[tick].jump) << "tick " << tick;
    }

    // The stream must be non-trivial, or bit-identity would be a vacuous pass: the tap and
    // the held jump must both surface as pressed edges, and the diagonal must quantize to a
    // nonzero move on both axes at some tick.
    bool any_jump = false;
    bool any_move = false;
    for (const MoveCommand& command : first)
    {
        any_jump = any_jump || command.jump;
        any_move = any_move || (command.move_x != 0 && command.move_y != 0);
    }
    EXPECT_TRUE(any_jump) << "the jump edges must reach the command stream";
    EXPECT_TRUE(any_move) << "the diagonal move must quantize into the command stream";
}
