/**************************************************************************/
/* editor_contexts.hpp                                                   */
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

#ifndef SUSHIENGINE_EDITOR_INPUT_CONTEXTS_HPP
#define SUSHIENGINE_EDITOR_INPUT_CONTEXTS_HPP

/**
 * @file editor_contexts.hpp
 * @brief The editor's shortcut and tool keys, as rebindable input contexts.
 *
 * These replace the ad-hoc `ImGui::IsKeyPressed` polls that were scattered across
 * `main.cpp` and `editor_panels.cpp`: every shortcut is now one binding, in data, resolved
 * once by the `ActionMapper` and gated in one place (the ImGui capture gate). Consumers query
 * the resolved `ActionSnapshot` by action name and never touch a key code, so W/E/R and Ctrl+Z
 * become rebindable from the Preferences page without any consumer change. Both Left and Right
 * Control are bound so either modifier satisfies a `Ctrl+*` shortcut, matching ImGui's `KeyCtrl`.
 */

#include <SushiEngine/input/action_map.hpp>
#include <SushiEngine/input/controls.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Builds the always-active editor shortcuts into @p context.
         *
         * Undo/Redo/Save/Copy/Cut/Paste, each chorded on Control. The ImGui capture gate
         * suppresses these while a text field owns the keyboard, so Ctrl+Z in a rename field is
         * not hijacked — the `!WantTextInput` guard the old polls did by hand, centralized.
         *
         * @param context The (empty) context to populate; the caller owns it.
         */
        inline void build_editor_global_context(Input::InputContext& context)
        {
            using Input::Key;
            context.add_button("Undo").bind(Key::Z, Key::LeftControl).bind(Key::Z, Key::RightControl);
            context.add_button("Redo").bind(Key::Y, Key::LeftControl).bind(Key::Y, Key::RightControl);
            context.add_button("Save").bind(Key::S, Key::LeftControl).bind(Key::S, Key::RightControl);
            context.add_button("Copy").bind(Key::C, Key::LeftControl).bind(Key::C, Key::RightControl);
            context.add_button("Cut").bind(Key::X, Key::LeftControl).bind(Key::X, Key::RightControl);
            context.add_button("Paste").bind(Key::V, Key::LeftControl).bind(Key::V, Key::RightControl);
        }

        /**
         * @brief Builds the viewport tool hotkeys into @p context.
         *
         * The Unity-style transform-tool selectors W/E/R (translate/rotate/scale). Camera flight
         * (WASD while right-mouse is held) stays on the viewport's own `Editor::InputState` seam;
         * the toolbar additionally stands these hotkeys down while right-mouse is held so they do
         * not fight flight, exactly as before.
         *
         * @param context The (empty) context to populate; the caller owns it.
         */
        inline void build_editor_viewport_context(Input::InputContext& context)
        {
            using Input::Key;
            context.add_button("GizmoTranslate").bind(Key::W);
            context.add_button("GizmoRotate").bind(Key::E);
            context.add_button("GizmoScale").bind(Key::R);
        }
    } // namespace Editor
} // namespace SushiEngine

#endif
