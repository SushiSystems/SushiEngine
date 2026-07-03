/**************************************************************************/
/* editor_panels.hpp                                                      */
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

#ifndef SUSHIENGINE_EDITOR_EDITOR_PANELS_HPP
#define SUSHIENGINE_EDITOR_EDITOR_PANELS_HPP

#include <cstdint>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draw the top menu bar (File / GameObject / Window).
         * @param context Shared editor state the menu acts on (world, documents). File >
         *                Exit sets @ref EditorContext::close_requested rather than exiting
         *                directly, so the caller's close-confirm modal gets a chance to run.
         */
        void draw_menu_bar(EditorContext& context);

        /**
         * @brief Save the open scene: straight to @c scene_path if set, else via the
         * Save-As prompt.
         *
         * Shared by the File > Save Scene menu item, the Ctrl+S shortcut, and the
         * unsaved-changes close prompt's Save button, so all three save the same way and
         * agree on when the scene becomes clean (see @ref scene_is_dirty).
         *
         * @param context Shared editor state; saves through the world editor.
         * @return True if the scene was written to an existing path; false if it failed,
         *         or if it deferred to the Save-As prompt because there was no path yet.
         */
        bool save_current_scene(EditorContext& context);

        /**
         * @brief Draw the Hierarchy panel: the live world's entities.
         *
         * Lists the world's entities directly (the world is the single source of truth),
         * with select, create, delete, a search filter, and inline / context-menu rename.
         *
         * @param context Shared editor state; updates @c selected_entity via the world.
         */
        void draw_hierarchy_panel(EditorContext& context);

        /**
         * @brief Draw the Inspector panel for the current selection.
         * @param context Shared editor state; edits the selected entity's name, visibility,
         *                transform, and colour through the world editor.
         */
        void draw_inspector_panel(EditorContext& context);

        /**
         * @brief Draw the Project panel: a filesystem browser rooted at the project.
         *
         * Double-clicking a directory descends into it; double-clicking a file opens it
         * in the text-edit panel via @ref EditorContext::documents.
         *
         * @param context Shared editor state; updates the browse directory and documents.
         */
        void draw_project_panel(EditorContext& context);

        /**
         * @brief Draw the text-edit panel hosting the open documents as tabs.
         * @param context Shared editor state; edits document buffers and saves to disk.
         */
        void draw_text_editor_panel(EditorContext& context);

        /**
         * @brief Draw the playback toolbar (Play / Pause / Step).
         *
         * Toggles @ref EditorContext::play_state and logs the transition; with no runtime
         * wired in the buttons only reflect state, marking the seam a future loop binds to.
         *
         * @param context Shared editor state; updates the playback state.
         */
        void draw_toolbar_panel(EditorContext& context);

        /**
         * @brief Draw the Console panel showing accumulated log lines with a clear button.
         * @param context Shared editor state; reads and clears the console buffer.
         */
        void draw_console_panel(EditorContext& context);

        /**
         * @brief Draw the Statistics panel (frame time, FPS, entity and document counts).
         * @param context Shared editor state, read for scene and document counts.
         */
        void draw_statistics_panel(EditorContext& context);

        /**
         * @brief Draw the bottom status bar (selection, playback state, entity count).
         * @param context Shared editor state, read for the status summary.
         */
        void draw_status_bar(EditorContext& context);

        /**
         * @brief Draw the modal Preferences window when @c show_preferences is set.
         *
         * Edits @ref EditorContext::preferences in place across General / Editor / Scene
         * sections. Any change sets @ref EditorContext::preferences_dirty so the loop
         * persists it and applies the live-effective fields (theme, camera speed). The
         * precision control is compile-time, so it records intent and prompts a rebuild.
         *
         * @param context Shared editor state; edits the preferences aggregate.
         */
        void draw_preferences_window(EditorContext& context);

        /**
         * @brief Draw the modal "Save Scene As" filename prompt when @c show_save_scene_as is set.
         *
         * A single-field popup rooted at @ref EditorContext::project_root; on confirm it
         * writes the `.sushiscene` file via @ref save_scene and records the path in
         * @ref EditorContext::scene_path so a later Save goes straight to disk.
         *
         * @param context Shared editor state; edits the save-as buffer and the scene path.
         * @param running Cleared on a successful save that was raised to unblock a pending
         *                window close (see @ref EditorContext::exit_after_save).
         */
        void draw_save_scene_as_modal(EditorContext& context, bool& running);

        /**
         * @brief Draw the "unsaved changes" confirm prompt when the window close was
         * requested while the scene is dirty.
         *
         * A no-op unless @ref EditorContext::close_requested is set. If the scene is
         * clean, closes immediately (clears @p running). Otherwise offers Save / Don't
         * Save / Cancel; Save routes through @ref save_current_scene, deferring to the
         * Save-As modal (via @ref EditorContext::exit_after_save) if the scene has never
         * been saved.
         *
         * @param context Shared editor state.
         * @param running Cleared to end the main loop once the close is confirmed.
         */
        void draw_exit_confirm_modal(EditorContext& context, bool& running);

        /**
         * @brief Draw the "unsaved changes" confirm prompt for a pending New/Open Scene
         * request made while the current scene was dirty.
         *
         * A no-op unless @ref EditorContext::pending_scene_action is set. If the scene is
         * clean, runs the pending action immediately. Otherwise offers Save / Don't Save /
         * Cancel; Save routes through @ref save_current_scene, deferring to the Save-As
         * modal if the scene has never been saved.
         *
         * @param context Shared editor state.
         */
        void draw_scene_action_confirm_modal(EditorContext& context);

        /**
         * @brief Apply a theme to ImGui's active style.
         *
         * Kept as a free function so both startup (from the loaded preferences) and the
         * Preferences window can apply the same mapping without duplicating it.
         *
         * @param theme The theme to install.
         */
        void apply_theme(EditorTheme theme);

        /**
         * @brief Build the default Unity-style dock layout the first time the editor runs.
         *
         * Splits the dockspace into Hierarchy (left), Inspector (right), Project plus the
         * text editor (bottom), and a central node left empty for a future viewport. Only
         * applied when no persisted layout exists, so user rearrangement survives restarts.
         *
         * @param dockspace_id The id of the root dockspace node to partition.
         */
        void build_default_layout(std::uint32_t dockspace_id);
    } // namespace Editor
} // namespace SushiEngine

#endif
