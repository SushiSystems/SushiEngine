/**************************************************************************/
/* test_input_bindings_json.cpp                                          */
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

// Unit_InputBindingsJson: binding persistence (docs/design/input_manager.md §2.5). The
// (de)serialization round-trips a rebind (the "survives restart" acceptance, proved by
// re-resolving through the mapper), degrades a stale or partial document to compiled-in
// defaults without throwing, and preserves unknown actions on round-trip. Reads are
// tolerant field-by-field, mirroring the editor's render_settings pattern.

#include <cstdint>

#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <SushiEngine/input/bindings_json.hpp>
#include <SushiEngine/input/input_manager.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Input;

namespace
{
    // The compiled-in default bindings, rebuilt fresh for each context (as a game or editor
    // would define them before overlaying a saved document).
    void build_gameplay_defaults(InputContext& context)
    {
        context.add_axis2d("Move")
            .bind_composite(Key::W, Key::S, Key::A, Key::D)
            .bind(GamepadAxis::LeftStick, Deadzone::radial(0.20f));
        context.add_button("Jump").bind(Key::Space).bind(GamepadButton::South);
        context.add_axis2d("Look").bind_mouse();
    }

    InputEvent key_down(Key key)
    {
        InputEvent event;
        event.device = KEYBOARD_DEVICE;
        event.type = EventType::KeyDown;
        event.control = static_cast<std::uint16_t>(key);
        return event;
    }

    // Resolves whether `action` is held after a single key press, through a fresh manager.
    bool held_after_key(InputContext& context, Key key, const std::string& action)
    {
        InputManager input;
        ScriptedInputSource source;
        input.add_source(source);
        input.push_context(context);
        source.enqueue(key_down(key));
        input.begin_frame();
        return input.snapshot().held(action);
    }
}

TEST(Unit_InputBindingsJson, RebindSurvivesASerializationRoundTrip)
{
    // Author defaults, rebind Jump from Space to J, and serialize — this is a "save".
    InputContext authored{"Gameplay"};
    build_gameplay_defaults(authored);
    Action* jump = authored.find_action("Jump");
    ASSERT_NE(jump, nullptr);
    set_button_binding(*jump, control_of(Key::J)); // replaces the Space binding, keeps South.
    const nlohmann::json document = bindings_to_json(authored);

    // A fresh session rebuilds compiled-in defaults, then loads the document — the "restart".
    InputContext restored{"Gameplay"};
    build_gameplay_defaults(restored);
    bindings_from_json(restored, document);

    EXPECT_TRUE(held_after_key(restored, Key::J, "Jump")) << "the rebind persisted";
    EXPECT_FALSE(held_after_key(restored, Key::Space, "Jump")) << "the old key is gone";
    // A binding the round-trip did not touch is intact.
    EXPECT_TRUE(held_after_key(restored, Key::W, "Move")) << "the WASD composite survived";
}

TEST(Unit_InputBindingsJson, MissingEntryKeepsCompiledInDefaults)
{
    InputContext context{"Gameplay"};
    build_gameplay_defaults(context);

    // A document that omits "Jump" (an action added after the file was written) leaves it default.
    nlohmann::json document = nlohmann::json::object();
    document["Move"] = bindings_to_json(context)["Move"];
    bindings_from_json(context, document);

    EXPECT_TRUE(held_after_key(context, Key::Space, "Jump")) << "Jump kept its default";
}

TEST(Unit_InputBindingsJson, StaleOrPartialDocumentLoadsToDefaultsWithoutThrowing)
{
    InputContext context{"Gameplay"};
    build_gameplay_defaults(context);

    // A non-object document is a no-op.
    EXPECT_NO_THROW(bindings_from_json(context, nlohmann::json::array()));
    EXPECT_TRUE(held_after_key(context, Key::Space, "Jump"));

    // A malformed entry (buttons is a number, not an array) must not corrupt or clear Jump.
    nlohmann::json document = nlohmann::json::object();
    document["Jump"] = nlohmann::json{{"buttons", 42}};
    EXPECT_NO_THROW(bindings_from_json(context, document));
    EXPECT_TRUE(held_after_key(context, Key::Space, "Jump")) << "malformed entry kept defaults";
}

TEST(Unit_InputBindingsJson, UnknownActionsArePreservedOnRoundTrip)
{
    InputContext context{"Gameplay"};
    build_gameplay_defaults(context);

    // A document from another version carries an action this build does not define.
    nlohmann::json previous = nlohmann::json::object();
    previous["LegacyAction"] = nlohmann::json{{"type", 0}, {"buttons", nlohmann::json::array()}};

    const nlohmann::json merged = bindings_to_json(context, previous);
    EXPECT_TRUE(merged.contains("LegacyAction")) << "an unknown action is not dropped";
    EXPECT_TRUE(merged.contains("Jump")) << "known actions are still written";
}
