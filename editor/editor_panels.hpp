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

#include "editor_context.hpp"

namespace sushi::editor
{
    /**
     * @brief Draw the top menu bar (File / GameObject / Window).
     * @param context Shared editor state the menu acts on (world, documents).
     * @param running Set to false when the user chooses File > Exit.
     */
    void draw_menu_bar(EditorContext& context, bool& running);

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
     * @brief Build the default Unity-style dock layout the first time the editor runs.
     *
     * Splits the dockspace into Hierarchy (left), Inspector (right), Project plus the
     * text editor (bottom), and a central node left empty for a future viewport. Only
     * applied when no persisted layout exists, so user rearrangement survives restarts.
     *
     * @param dockspace_id The id of the root dockspace node to partition.
     */
    void build_default_layout(std::uint32_t dockspace_id);
}

#endif
