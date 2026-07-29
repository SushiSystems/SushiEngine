/**************************************************************************/
/* modals.hpp                                                             */
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
 * @file modals.hpp
 * @brief The prompts that stand between a destructive action and losing work.
 *
 * Save-As, the close-the-window confirm, and the replace-the-scene confirm. They are
 * one unit because they are one mechanism: each of them can route through
 * `save_current_scene`, and each has to hand control back to whatever it interrupted —
 * a pending close or a parked New/Open — once the user has decided.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the "Save Scene As" filename prompt when it has been requested.
         *
         * A single-field popup rooted at @ref EditorContext::project_root; on confirm it
         * writes the `.sushiscene` file and records the path so a later Save goes straight
         * to disk.
         *
         * @param context Editor state; edits the save-as buffer and the scene path.
         * @param running Cleared on a successful save that was raised to unblock a pending
         *                window close (see @ref EditorContext::exit_after_save).
         */
        void draw_save_scene_as_modal(EditorContext& context, bool& running);

        /**
         * @brief Draws the unsaved-changes prompt for a requested window close.
         *
         * A no-op unless the close was requested. A clean scene closes immediately; a dirty
         * one offers Save / Don't Save / Cancel, where Save defers to the Save-As modal if
         * the scene has never been written.
         *
         * @param context Editor state.
         * @param running Cleared to end the main loop once the close is confirmed.
         */
        void draw_exit_confirm_modal(EditorContext& context, bool& running);

        /**
         * @brief Draws the unsaved-changes prompt for a parked New/Open Scene request.
         *
         * A no-op unless a scene action is pending. A clean scene runs the action at once;
         * a dirty one offers the same Save / Don't Save / Cancel choice.
         *
         * @param context Editor state; resolves @ref EditorContext::pending_scene_action.
         */
        void draw_scene_action_confirm_modal(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
