/**************************************************************************/
/* input_manager_window.hpp                                               */
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

#pragma once

/**
 * @file input_manager_window.hpp
 * @brief The Input Manager window: what every editor action is bound to.
 *
 * Lists the editor's input contexts and their actions with the live binding, a
 * click-to-rebind flow, a conflict indicator, and a reset to defaults. It also owns the
 * human-readable spelling of a binding, which the menu bar borrows so a menu shortcut
 * label is derived from the real binding instead of a hard-coded string that can rot.
 */

#include <string>

#include <SushiEngine/input/input_manager.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Spells a binding the way a menu or a rebind row shows it, e.g. "Ctrl+S".
         *
         * Exported so the menu bar's shortcut hints come from the live binding table; a menu
         * that spelled its own shortcut would keep claiming one after a rebind changed it.
         *
         * @param binding The binding to describe.
         * @return The display text, empty when the binding names no control.
         */
        std::string input_binding_label(const Input::Binding& binding);

        /**
         * @brief Draws the Input Manager window when its panel flag is set.
         *
         * Rebinds are serialized into the preferences as they are made, so a remapped key
         * survives the session that made it.
         *
         * @param context Editor state; reads the live input contexts, edits preferences.
         */
        void draw_input_manager_window(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
