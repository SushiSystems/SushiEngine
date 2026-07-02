/**************************************************************************/
/* editor_context.hpp                                                     */
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

#ifndef SUSHIENGINE_EDITOR_EDITOR_CONTEXT_HPP
#define SUSHIENGINE_EDITOR_EDITOR_CONTEXT_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/sim/simulation.hpp>

#include "command_history.hpp"
#include "gizmo_controller.hpp"
#include "preferences.hpp"

namespace sushi::editor
{
    /**
     * @brief Which dockable panels are currently shown.
     *
     * Each flag backs one entry in the Window menu and one panel's open state, so
     * closing a panel (its title-bar X) and reopening it from the menu share the
     * same bit. Defaults to everything visible on a fresh layout.
     */
    struct PanelVisibility
    {
        bool scene_view = true;
        bool game_view = true;
        bool hierarchy = true;
        bool inspector = true;
        bool project = true;
        bool text_editor = true;
        bool console = true;
        bool statistics = true;
        bool toolbar = true;
    };

    /**
     * @brief Editor playback state, mirroring a game engine's play controls.
     *
     * The shell has no runtime wired in yet, so this drives only the toolbar's
     * button states and console feedback; it is the seam a future World loop binds
     * to for play/pause/step.
     */
    enum class PlayState
    {
        Stopped,
        Playing,
        Paused
    };
    /**
     * @brief One file open in the text-edit panel.
     *
     * Holds the on-disk path, the live editable buffer, and a dirty flag flipped
     * whenever the buffer diverges from disk so the UI can mark unsaved work and
     * prompt on close.
     */
    struct Document
    {
        std::string path;
        std::string display_name;
        std::string text;
        bool dirty = false;
    };

    /**
     * @brief Shared, mutable editor state passed to every panel each frame.
     *
     * A single aggregate the panels read and write: the scene being edited, the
     * current selection, the project browser root, and the set of open documents.
     * Panels communicate through this struct rather than calling each other, so the
     * hierarchy's selection change is picked up by the inspector next frame with no
     * direct coupling.
     */
    struct EditorContext
    {
        // The live world, owned by main() and injected here; the panels edit it
        // through the IWorldEditor surface. The world is the single source of truth
        // for entities — there is no separate editor-side scene model.
        SushiEngine::sim::ISimulation* simulation = nullptr;

        // The Inspector/gizmo's single "primary" target (the most recently clicked
        // entity). `selected_entities` is the full Hierarchy multi-selection (Ctrl
        // toggles membership, Shift extends a range from `selection_anchor`); a plain
        // click collapses both down to one entity via `select_only`. Anything that
        // edits a single entity (Inspector, the viewport gizmo, Align/Move-to-View)
        // reads `selected_entity`; bulk operations (Hierarchy Delete) read the vector.
        SushiEngine::sim::EntityId selected_entity = SushiEngine::sim::NULL_ENTITY;
        std::vector<SushiEngine::sim::EntityId> selected_entities;
        SushiEngine::sim::EntityId selection_anchor = SushiEngine::sim::NULL_ENTITY;
        SushiEngine::sim::EntityId renaming_entity = SushiEngine::sim::NULL_ENTITY;

        std::string project_root;
        std::string current_directory;

        // The scene currently open, if any (empty means unsaved/new). Save writes
        // here directly; Save As and the Save-As-prompted first save go through
        // `show_save_scene_as`, an inline filename popup rooted at `project_root`.
        std::string scene_path;
        bool show_save_scene_as = false;
        std::string save_scene_as_name;

        // Undo/redo over whole-world snapshots; panels record before a mutation (see
        // CommandHistory) and the menu/keyboard shortcuts drive undo()/redo().
        CommandHistory history;

        // Project panel state: the single selected file/folder (full path, empty if
        // none), the path currently in inline rename, and the name-search filter
        // applied to the current folder's contents.
        std::string selected_project_path;
        std::string renaming_project_path;
        std::string project_filter;

        std::vector<Document> documents;
        int active_document = -1;

        PanelVisibility panels;
        PlayState play_state = PlayState::Stopped;

        std::string hierarchy_filter;

        std::vector<std::string> console_lines;

        std::size_t world_entity_count = 0;

        // Which display the Game view renders, chosen from the resolved cameras.
        std::uint32_t game_display = 0;

        // The active Scene-view transform tool (W/E/R) and axis frame (Local/World),
        // shared between the toolbar that sets them and the viewport that draws the
        // matching gizmo.
        GizmoMode gizmo_mode = GizmoMode::Translate;
        GizmoSpace gizmo_space = GizmoSpace::World;

        // One-shot camera/selection requests raised by the Hierarchy and GameObject
        // menu and serviced by the main loop, which owns the Scene camera and world:
        //   frame  — move the Scene camera to look at the selection (double-click).
        //   align  — move the selection to the Scene camera's pose (Align With View).
        //   moveto — move the selection in front of the Scene camera (Move to View).
        bool frame_selected_requested = false;
        bool align_with_view_requested = false;
        bool move_to_view_requested = false;

        // The persisted editor/project settings and their store. The store is owned by
        // main() and injected; panels read and edit `preferences` and set
        // `preferences_dirty` so the loop persists the change once per frame rather
        // than on every widget tick. `show_preferences` toggles the modal window.
        Preferences preferences;
        IPreferencesStore* preferences_store = nullptr;
        bool preferences_dirty = false;
        bool show_preferences = false;

        bool show_imgui_demo = false;
    };

    /**
     * @brief Append one line to the editor console log.
     *
     * A tiny free function rather than a method so panels depend only on the data
     * aggregate, not on a logging interface; the console panel renders whatever has
     * accumulated. Older lines are trimmed to keep the buffer bounded.
     *
     * @param context Editor state whose console buffer receives the line.
     * @param message Text to record.
     */
    inline void editor_log(EditorContext& context, const std::string& message)
    {
        context.console_lines.push_back(message);
        constexpr std::size_t MAX_CONSOLE_LINES = 1000;
        if (context.console_lines.size() > MAX_CONSOLE_LINES)
        {
            context.console_lines.erase(
                context.console_lines.begin(),
                context.console_lines.begin() +
                    static_cast<std::ptrdiff_t>(context.console_lines.size() -
                                                MAX_CONSOLE_LINES));
        }
    }

    /** @brief Whether @p id is part of the current Hierarchy multi-selection. */
    inline bool is_selected(const EditorContext& context,
                            SushiEngine::sim::EntityId id) noexcept
    {
        return std::find(context.selected_entities.begin(), context.selected_entities.end(),
                         id) != context.selected_entities.end();
    }

    /**
     * @brief Collapses the selection to a single entity (a plain click).
     *
     * Sets both the Inspector/gizmo's `selected_entity` and the Hierarchy's
     * multi-selection to just @p id, and rebases the Shift-range anchor there. Pass
     * `NULL_ENTITY` to clear the selection entirely.
     *
     * @param context Editor state to update.
     * @param id The entity to select alone, or `NULL_ENTITY` to select nothing.
     */
    inline void select_only(EditorContext& context, SushiEngine::sim::EntityId id)
    {
        context.selected_entity = id;
        context.selection_anchor = id;
        context.selected_entities.clear();
        if (id != SushiEngine::sim::NULL_ENTITY)
            context.selected_entities.push_back(id);
    }

    /**
     * @brief Toggles @p id's membership in the multi-selection (a Ctrl+click).
     *
     * Rebases the Shift-range anchor to @p id either way, so a following Shift-click
     * extends from this entity rather than the original plain-click target.
     *
     * @param context Editor state to update.
     * @param id The entity to add or remove from the selection.
     */
    inline void toggle_selected(EditorContext& context, SushiEngine::sim::EntityId id)
    {
        const auto it =
            std::find(context.selected_entities.begin(), context.selected_entities.end(), id);
        if (it != context.selected_entities.end())
        {
            context.selected_entities.erase(it);
            context.selected_entity = context.selected_entities.empty()
                                          ? SushiEngine::sim::NULL_ENTITY
                                          : context.selected_entities.back();
        }
        else
        {
            context.selected_entities.push_back(id);
            context.selected_entity = id;
        }
        context.selection_anchor = id;
    }
}

#endif
