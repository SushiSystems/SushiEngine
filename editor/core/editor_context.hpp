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

#include <optional>

#include <nlohmann/json.hpp>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/render/material.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/sim/simulation.hpp>

#include "command_history.hpp"
#include "../gizmo/gizmo_controller.hpp"
#include "preferences.hpp"

namespace SushiEngine
{
    namespace Editor
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
            bool environment = true;
            bool rendering = false;
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
         * @brief One entity's full authored state, snapshotted for copy/cut/paste.
         *
         * Captured entirely through `IWorldEditor` getters (see `copy_selection`), so
         * pasting an entity is just replaying the matching setters on a freshly created
         * one — no serialization format and no new engine-side clone primitive needed.
         * `original`/`original_parent` are used only to rebuild internal parent/child
         * relationships within a multi-entity paste; they are not valid after the source
         * entity is gone (e.g. once Cut has deleted it).
         */
        struct ClipboardEntity
        {
            SushiEngine::Simulation::EntityId original = SushiEngine::Simulation::NULL_ENTITY;
            SushiEngine::Simulation::EntityId original_parent = SushiEngine::Simulation::NULL_ENTITY;
            std::string name;
            SushiEngine::Simulation::EntityTransform transform;
            SushiEngine::Vector3 color{};
            bool visible = true;
            bool has_renderer = false;
            bool is_camera = false;
            SushiEngine::Simulation::CameraParams camera_params;
            bool has_physics_body = false;
            SushiEngine::Simulation::PhysicsBodyParams physics_body_params;
            bool has_cloth = false;
            SushiEngine::Simulation::ClothParams cloth_params;
            bool has_shape = false;
            SushiEngine::Simulation::ShapeParams shape_params;
            bool has_collider = false;
            SushiEngine::Simulation::ColliderParams collider_params;
            bool has_ui = false;
            SushiEngine::Simulation::UIElementParams ui_params;
            std::vector<SushiEngine::Simulation::ScriptComponent> scripts;
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
            SushiEngine::Simulation::ISimulation* simulation = nullptr;

            // The renderer's shared asset store, injected by main() so a panel can load
            // a texture or a model without knowing which renderer produced it. Null in a
            // headless editor, which is why every use is guarded.
            SushiEngine::Render::IAssetLibrary* assets = nullptr;

            // The Inspector/gizmo's single "primary" target (the most recently clicked
            // entity). `selected_entities` is the full Hierarchy multi-selection (Ctrl
            // toggles membership, Shift extends a range from `selection_anchor`); a plain
            // click collapses both down to one entity via `select_only`. Anything that
            // edits a single entity (Inspector, the viewport gizmo, Align/Move-to-View)
            // reads `selected_entity`; bulk operations (Hierarchy Delete) read the vector.
            SushiEngine::Simulation::EntityId selected_entity = SushiEngine::Simulation::NULL_ENTITY;
            std::vector<SushiEngine::Simulation::EntityId> selected_entities;
            SushiEngine::Simulation::EntityId selection_anchor = SushiEngine::Simulation::NULL_ENTITY;
            SushiEngine::Simulation::EntityId renaming_entity = SushiEngine::Simulation::NULL_ENTITY;

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

            // The history revision as of the last successful save/load, so `scene_is_dirty`
            // can tell "changed since save" apart from "never changed" without diffing
            // snapshots. Set from `history.revision()` by New/Save/Save As/Open.
            std::uint64_t saved_scene_revision = 0;

            // Set when the OS asks to close the window; the main loop holds the window
            // open and shows a confirm-close modal instead of exiting immediately when the
            // scene is dirty (see `scene_is_dirty`).
            bool close_requested = false;

            // Set when the Save-As modal was opened to unblock a pending close (the scene
            // had never been saved). On a successful save it lets the modal finish the
            // close; on cancel it aborts the pending close instead of leaving it stuck.
            bool exit_after_save = false;

            // A New Scene / Open Scene request raised while the current scene was dirty,
            // parked here so the unsaved-changes prompt can run first. `None` means no
            // action is pending; `pending_scene_open_path` holds the target for `Open`
            // (unused, empty for `New`). Resolved by `perform_pending_scene_action`, which
            // the prompt's Save/Don't Save buttons call once the scene is safe to replace.
            enum class PendingSceneAction
            {
                None,
                New,
                Open
            };
            PendingSceneAction pending_scene_action = PendingSceneAction::None;
            std::string pending_scene_open_path;

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

            // The scene captured at the moment Play was pressed, restored verbatim on
            // Stop (see scene_serializer.hpp's capture_scene/apply_scene) — mirrors
            // Unity's edit-mode-is-never-mutated-by-play-mode guarantee. Empty means no
            // Play session is in progress.
            std::optional<nlohmann::json> play_mode_snapshot;

            // One-shot request from the toolbar's Step button: advance the world exactly
            // one tick this frame regardless of play_state (typically pressed while
            // Paused), then the main loop clears it.
            bool step_requested = false;

            std::string hierarchy_filter;

            // Snapshot of the last Copy/Cut, replayed by Paste (see `ClipboardEntity`).
            // Cut fills this exactly like Copy, then additionally deletes the originals.
            std::vector<ClipboardEntity> clipboard;

            std::vector<std::string> console_lines;

            std::size_t world_entity_count = 0;

            // Which display the Game view renders, chosen from the resolved cameras.
            std::uint32_t game_display = 0;

            // The active Scene-view transform tool (W/E/R) and axis frame (Local/World),
            // shared between the toolbar that sets them and the viewport that draws the
            // matching gizmo.
            GizmoMode gizmo_mode = GizmoMode::Translate;
            GizmoSpace gizmo_space = GizmoSpace::World;

            // One-shot camera/selection requests raised by the Hierarchy and Entity menu
            // and serviced by the main loop, which owns the Scene camera and world:
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

            // How the viewports trade fidelity against frame time: anti-aliasing mode,
            // render scale, the dynamic-resolution governor, and variable-rate shading.
            // Edited in the Environment panel's Rendering section and pushed to both
            // viewports each frame, so the two always agree.
            SushiEngine::Render::RenderSettings render_settings;

            // The internal resolution the Scene view last rendered at, written back by
            // the main loop. It is the only way to see the dynamic-resolution governor
            // work: the scale slider is a request, this is what it settled on.
            std::uint32_t scene_render_width = 0;
            std::uint32_t scene_render_height = 0;

            bool show_imgui_demo = false;

            // Solar-system sky authoring, driven from the Environment panel. The civil
            // epoch and observer position feed the ephemeris every frame (in main()),
            // which repopulates the environment's bodies and stars without touching the
            // world — so scrubbing the date or animating time costs no world re-extract.
            // `sky_accumulated_days` is the running offset of the master epoch past the
            // authored start date, shown in the panel; latitude/longitude re-point the
            // whole celestial sphere. The simulation owns the master epoch now (see
            // ISimulation::julian_date): the main loop drives its flow from `sky_animate`/
            // `sky_days_per_second`, seeks it when `sky_date` changes, and reads it back for
            // the sky, so the sky and the orbital dynamics share one clock.
            bool sky_enabled = true;
            SushiEngine::Astro::CalendarDate sky_date{2026, 7, 4, 21, 0, 0.0};
            double sky_latitude_degrees = 41.0;
            double sky_longitude_degrees = 29.0;
            bool sky_astronomical_sun = true;
            bool sky_animate = false;
            double sky_days_per_second = 0.02;
            double sky_accumulated_days = 0.0;
            // The authored start epoch the sim was last seeked to, so the main loop can
            // detect a `sky_date` edit and re-seek; negative until the first frame seeds it.
            double sky_authored_start_cache = -1.0;
            // A body index (Astro::BodyId) the camera should travel to this frame, set by
            // the environment panel's quick-travel buttons; -1 when none is pending. The
            // main loop consumes it: teleports to the body's sunlit side and resets it.
            int sky_travel_target = -1;

            // Ride-along state so the camera stays attached to a moving planet as time
            // animates: the dominant body index of the previous frame and its scene-frame
            // centre. When the same non-Earth body remains the analytic ground across a
            // frame, the camera is shifted by the body's centre delta so its altitude over
            // the surface is preserved instead of the orbit sliding out from under it.
            // Earth is exempt — its observer point is re-anchored to the scene origin each
            // frame, so the camera already tracks it. -1 marks "no body tracked yet".
            int sky_ride_body = -1;
            SushiEngine::WorldVector3 sky_ride_center{};

            // The catalog of user-defined "script" component types available in the
            // Add Component menu. Each entry is a definition (a type name plus default
            // fields); attaching one copies it onto the entity as an instance. Seeded
            // from scripts found when a scene loads and grown by the New Script dialog,
            // so a project's custom components survive a save/load round-trip.
            std::vector<SushiEngine::Simulation::ScriptComponent> script_catalog;

            // New Script dialog state: the pending class name typed in the modal and
            // whether it is open. Creating a script adds a definition to the catalog and
            // scaffolds a C++ system stub in the project, opened in the Text Editor.
            bool show_new_script = false;
            std::string new_script_name;
            SushiEngine::Simulation::EntityId new_script_target =
                SushiEngine::Simulation::NULL_ENTITY;
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

        /**
         * @brief Whether the scene has unsaved changes.
         *
         * True whenever the undo history has advanced past the revision recorded at the
         * last successful New/Open/Save, so the title bar's "*" and the close-confirm
         * modal always agree with what Ctrl+Z/Y would actually undo back to.
         */
        inline bool scene_is_dirty(const EditorContext& context) noexcept
        {
            return context.history.revision() != context.saved_scene_revision;
        }

        /** @brief Whether @p id is part of the current Hierarchy multi-selection. */
        inline bool is_selected(const EditorContext& context,
                                SushiEngine::Simulation::EntityId id) noexcept
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
        inline void select_only(EditorContext& context, SushiEngine::Simulation::EntityId id)
        {
            context.selected_entity = id;
            context.selection_anchor = id;
            context.selected_entities.clear();
            if (id != SushiEngine::Simulation::NULL_ENTITY)
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
        inline void toggle_selected(EditorContext& context, SushiEngine::Simulation::EntityId id)
        {
            const auto it =
                std::find(context.selected_entities.begin(), context.selected_entities.end(), id);
            if (it != context.selected_entities.end())
            {
                context.selected_entities.erase(it);
                context.selected_entity = context.selected_entities.empty()
                                              ? SushiEngine::Simulation::NULL_ENTITY
                                              : context.selected_entities.back();
            }
            else
            {
                context.selected_entities.push_back(id);
                context.selected_entity = id;
            }
            context.selection_anchor = id;
        }
    } // namespace Editor
} // namespace SushiEngine

#endif
