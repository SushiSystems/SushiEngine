/**************************************************************************/
/* scene_commands.hpp                                                     */
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
 * @file scene_commands.hpp
 * @brief What the editor *does* to a scene, separated from what it draws.
 *
 * New, Open, Save, the clipboard, and entity creation — the document model behind the
 * menu bar, the Hierarchy's context menus, the Project browser's double-click, the
 * keyboard shortcuts, and the unsaved-changes prompts. Every one of those surfaces calls
 * the same function here rather than holding its own copy of the sequence, so a scene can
 * only be replaced or saved one way.
 *
 * The two `draw_*_menu_items` functions are in this file rather than with a panel for
 * the same reason: they are the shared *entry points* to these commands, and four
 * surfaces offer them. A menu that lived in one panel would drift from the others.
 */

#include <string>

#include "../core/editor_context.hpp"
#include "prefab_serializer.hpp"
#include "scene_serializer.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Snapshots the editor's date/time/observer into the persisted sky block.
         *
         * Mirrors the Environment panel's Solar System fields, so a save/load round-trips
         * the moment and the place a scene was authored at instead of resetting to noon.
         *
         * @param context Editor state, read for the clock and the observer.
         * @return The sky state to write into the scene file.
         */
        Scene::SceneSkyState capture_sky_state(const EditorContext& context);

        /**
         * @brief Records a scene path at the head of the recent-scenes list.
         *
         * Deduplicates and caps the list, so File ▸ Open Recent stays short and a scene
         * reopened often does not fill it.
         *
         * @param context Editor state; edits `preferences.recent_scenes` and marks it dirty.
         * @param path The scene file just opened or saved.
         */
        void note_recent_scene(EditorContext& context, const std::string& path);

        /**
         * @brief Saves the open scene: straight to `scene_path` if set, else via Save-As.
         *
         * Shared by File ▸ Save Scene, Ctrl+S, the autosave timer, and the unsaved-changes
         * prompts, so all of them agree on when the scene becomes clean.
         *
         * @param context Editor state; saves through the world editor.
         * @return True if the scene was written to an existing path; false if it failed or
         *         deferred to the Save-As prompt because there was no path yet.
         */
        bool save_current_scene(EditorContext& context);

        /**
         * @brief Requests a new scene, deferring to the unsaved-changes prompt if dirty.
         * @param context Editor state; either replaces the scene or parks the request.
         */
        void request_new_scene(EditorContext& context);

        /**
         * @brief Requests a scene open, deferring to the unsaved-changes prompt if dirty.
         * @param context Editor state; either replaces the scene or parks the request.
         * @param path The scene file to open.
         */
        void request_open_scene(EditorContext& context, const std::string& path);

        /**
         * @brief Requests a project switch, deferring to the unsaved-changes prompt when the
         * current scene is dirty rather than discarding it silently.
         * @param context  Editor state.
         * @param new_root The directory to make the new @ref EditorContext::project_root.
         */
        void request_switch_project(EditorContext& context, const std::string& new_root);

        /**
         * @brief Runs whichever scene replacement was parked by a request, then clears it.
         *
         * Called by the unsaved-changes prompt's Save and Don't-Save buttons once the
         * current scene is safe to discard, and directly when it was already clean.
         *
         * @param context Editor state; consumes `pending_scene_action`.
         */
        void perform_pending_scene_action(EditorContext& context);

        /**
         * @brief Snapshots the current Hierarchy selection into the clipboard.
         *
         * Shared by Edit ▸ Copy, Ctrl+C, and every Hierarchy context menu. A no-op with no
         * world or an empty selection.
         *
         * @param context Editor state; reads the selection, writes the clipboard.
         */
        void copy_selection(EditorContext& context);

        /**
         * @brief Copies the selection, then deletes it.
         *
         * Cut, like a text editor's, removes the originals immediately — Paste recreates
         * them from the snapshot rather than "moving" a pending clipboard.
         *
         * @param context Editor state; mutates the world and the selection.
         */
        void cut_selection(EditorContext& context);

        /**
         * @brief Recreates the entities captured in the clipboard.
         *
         * Parent/child relationships among the copied entities are preserved; an entity
         * whose original parent was not copied re-parents to that same external parent
         * (still a paste-in-place), falling back to root if it no longer exists. The new
         * entities become the selection.
         *
         * @param context Editor state; mutates the world and the selection.
         */
        void paste_clipboard(EditorContext& context);

        /**
         * @brief Deep-copies the selection in place and selects the copies (Unity's Ctrl+D).
         *
         * Deliberately does **not** go through the clipboard: duplicating is not copying,
         * and a Ctrl+D that replaced what the user had copied would destroy state the
         * gesture never claimed to touch. Shares the snapshot and instantiate steps with
         * copy/paste, so a duplicate carries exactly the components a paste would.
         *
         * @param context Editor state; records one undo step and moves the selection to
         *                the new entities.
         */
        void duplicate_selection(EditorContext& context);

        /**
         * @brief Destroys every selected entity as one undo step.
         *
         * Shared by the Delete key, Edit ▸ Delete, and the Hierarchy context menus, so all
         * of them delete the whole selection rather than whichever entity was under the
         * cursor.
         *
         * @param context Editor state; mutates the world and clears the selection.
         */
        void delete_selection(EditorContext& context);

        /**
         * @brief Places an instance of @p asset_path's prefab, and selects it.
         *
         * The Scene view's model drop. Reads `<asset_path>.sushiprefab`, the file the model
         * importer writes beside an asset, and builds an instance of it as one undo step.
         *
         * A model that has not been imported yet is reported and nothing is placed. Importing
         * it here is deliberately not done: reading and planning a large glTF inside a
         * mouse-release handler would block the interface for as long as it took, which reads
         * as a hang rather than as work.
         *
         * @param context    Editor state; mutates the world, the selection and the log.
         * @param asset_path The `.gltf` or `.glb` that was dropped.
         * @return Whether an instance was placed.
         */
        bool place_model_instance(EditorContext& context, const std::string& asset_path);

        /**
         * @brief Runs the selection command parked this frame, if any, and clears it.
         *
         * Called once per frame by the main loop after every panel has been drawn, which is
         * the only point at which creating or destroying entities cannot invalidate a walk
         * already in progress. Every entry point — menu, context menu, shortcut — parks its
         * command rather than acting, so all of them are correct for the same reason.
         *
         * @param context Editor state; consumes @c pending_entity_command.
         */
        void run_pending_entity_command(EditorContext& context);

        /**
         * @brief Draws the shared entity-creation menu items.
         *
         * Reused by the Entity menu, the Hierarchy's row and empty-space context menus, and
         * its filtered search view, so no surface can drift out of sync with the others.
         * The new entity becomes the selection.
         *
         * @param context Editor state; records undo and selects what it creates.
         * @param world The world to create in; the items disable themselves when null.
         */
        void draw_create_object_menu_items(EditorContext& context,
                                           Simulation::IWorldEditor* world);

        /**
         * @brief Draws the shared Copy / Cut / Paste / Duplicate / Delete menu items.
         *
         * Shortcut hints come from the live bindings. Selecting one parks it in
         * @c EditorContext::pending_entity_command rather than acting, for the reason given
         * on @ref run_pending_entity_command.
         *
         * @param context Editor state; parks the chosen command.
         * @param world The world the items act on; they disable themselves when null.
         */
        void draw_clipboard_menu_items(EditorContext& context, Simulation::IWorldEditor* world);
    } // namespace Editor
} // namespace SushiEngine
