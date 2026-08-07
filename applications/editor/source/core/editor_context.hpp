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
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <optional>

#include <nlohmann/json.hpp>

#include <SushiEngine/astro/julian_date.hpp>
#include <SushiEngine/authoring/command_history.hpp>
#include <SushiEngine/authoring/game_view_settings.hpp>
#include <SushiEngine/authoring/panel_visibility.hpp>
#include <SushiEngine/authoring/preferences.hpp>
#include <SushiEngine/authoring/soft_body_heat.hpp>
#include <SushiEngine/material/material.hpp>
#include <SushiEngine/render/asset_library_interface.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/simulation/simulation.hpp>
#include <SushiEngine/simulation/simulation_settings.hpp>

#include "scene_blob_table.hpp"

#include "console.hpp"
#include "../gizmo/gizmo_controller.hpp"
#include "meteorology_log.hpp"
#include "physics_overlay_settings.hpp"
#include "panel_state.hpp"

namespace SushiEngine
{
    namespace Input
    {
        class ActionSnapshot;
        class InputManager;
        class InputContext;
    } // namespace Input

    namespace Authoring
    {
        /** @brief The Bake surface's model, owned by main() (see authoring/cook_bake_state.hpp). */
        class CookBakeState;
    } // namespace Authoring

    namespace Editor
    {
        /** @brief The live particle-effect preview, owned by main() (see effect_preview.hpp). */
        class EffectPreview;

        /** @brief The live GPU-skinned character preview, owned by main() (see animated_mesh_preview.hpp). */
        class AnimatedMeshPreview;

        /** @brief The live editor audio system, owned by main() (see audio/audio_editor_system.hpp). */
        class AudioEditorSystem;

        /** @brief The Project panel's image thumbnail pipeline, owned by main() (see project/thumbnail_cache.hpp). */
        class ThumbnailCache;

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
         * Every component the Inspector can author must have a field here, or copy/paste
         * silently strips it from the duplicate. `original`/`original_parent` are used
         * only to rebuild internal parent/child relationships within a multi-entity
         * paste; they are not valid after the source entity is gone (e.g. once Cut has
         * deleted it).
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
            SushiEngine::Render::Material material;
            SushiEngine::Simulation::MaterialTexturePaths material_texture_paths;
            bool is_camera = false;
            SushiEngine::Simulation::CameraParameters camera_parameters;
            bool has_physics_body = false;
            SushiEngine::Simulation::PhysicsBodyParameters physics_body_parameters;
            bool has_cloth = false;
            SushiEngine::Simulation::ClothParameters cloth_parameters;
            /**
             * @brief The soft body, cooked asset included.
             *
             * The blob is copied rather than shared: a pasted entity is a second body
             * from the moment it exists, and a clipboard that handed out a reference
             * would tie the duplicate's lifetime to an original the artist may delete.
             */
            bool has_soft_body = false;
            SushiEngine::Simulation::SoftBodyParameters soft_body_parameters;
            /**
             * @brief The crowd character, its asset paths included.
             *
             * The paths travel with the ids rather than instead of them, because a paste is
             * replayed through `set_crowd_parameters` — which re-registers the skeleton and
             * clip from the paths. So a duplicate is bound to the same rig as its original
             * even when the clipboard outlives the session that filled it.
             */
            bool has_crowd = false;
            SushiEngine::Simulation::CrowdParameters crowd_parameters;
            bool has_light = false;
            SushiEngine::Simulation::LightParameters light_parameters;
            bool has_decal = false;
            SushiEngine::Simulation::DecalParameters decal_parameters;
            bool has_shape = false;
            SushiEngine::Simulation::ShapeParameters shape_parameters;
            bool has_collider = false;
            SushiEngine::Simulation::ColliderParameters collider_parameters;
            bool has_joint = false;
            SushiEngine::Simulation::PhysicsJointParameters joint_parameters;
            bool has_vehicle = false;
            SushiEngine::Simulation::VehicleInstanceParameters vehicle_parameters;
            bool has_particle_emitter = false;
            SushiEngine::Simulation::ParticleEmitterParameters particle_emitter_parameters;
            SushiEngine::VFX::ParticleEffect particle_effect;
            bool has_audio_emitter = false;
            SushiEngine::Simulation::AudioEmitterParameters audio_emitter_parameters;
            bool has_reverb_zone = false;
            SushiEngine::Simulation::ReverbZoneParameters reverb_zone_parameters;
            bool has_audio_listener = false;
            SushiEngine::Simulation::AudioListenerParameters audio_listener_parameters;
            bool surface_anchored = false;
            SushiEngine::Quaternion surface_local_orientation{};
            SushiEngine::Simulation::EntityFrame entity_frame;
            bool has_ui = false;
            SushiEngine::Simulation::UIElementParameters ui_parameters;
            std::vector<SushiEngine::Simulation::ScriptComponent> scripts;
        };

        /**
         * @brief One component's authored values, remembered by Copy Values.
         *
         * Type-erased behind the component's display name, and it is that name which gates
         * the cast back on paste (see `component_editor.hpp`): a Light's values can only
         * ever be read back as a Light's. Erasing rather than enumerating keeps this one
         * field instead of a variant that has to grow a member per component — and lets it
         * hold the components whose parameters are not trivially copyable, like the Decal's
         * map paths.
         */
        struct ComponentValueClipboard
        {
            std::string component;       /**< Display name of the component the values came from. */
            std::shared_ptr<void> values; /**< A copy of that component's parameter aggregate. */
        };

        /** @brief One render pass's GPU time, copied out of a scene view for display. */
        struct GPUPassStatistic
        {
            std::string pass;          /**< The pass name as registered in the render graph. */
            float milliseconds = 0.0f; /**< Measured GPU time of the pass. */
        };

        /**
         * @brief One viewport's per-pass GPU times for the Statistics panel.
         *
         * Copied, not referenced: the scene view owns the timing storage only until its
         * next render, while the Statistics panel reads this after both viewports have
         * already rendered the frame.
         */
        struct ViewportGPUStatistics
        {
            std::string viewport;                 /**< The viewport title ("Scene", "Game"). */
            std::vector<GPUPassStatistic> passes; /**< Per-pass times, in graph order. */
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

            // The live particle-effect preview, owned by main() and injected here so the
            // Particle Editor panel authors it and the Scene viewport renders + gizmos it.
            EffectPreview* particle_preview = nullptr;

            // The live GPU-skinned character preview, owned by main() and injected here so
            // the Statistics panel can report its pose-pool/palette footprint (design
            // `slop/animation_system.md` §12.1). Null in a headless editor.
            AnimatedMeshPreview* animated_mesh_preview = nullptr;

            // The live editor audio system, owned by main() and injected here so the audio
            // panels and the Inspector's audio sections drive and hear the same engine.
            AudioEditorSystem* audio = nullptr;

            // The Bake surface's model, owned by main() and injected here so a panel that
            // brings a mesh into the project (today: the Project panel's glTF open/preview
            // flow) can queue it for cooking without owning a worker thread itself. Null in
            // a headless editor, which is why every use is guarded.
            Authoring::CookBakeState* cook_bake_state = nullptr;

            // The Project panel's image thumbnail pipeline, owned by main() and injected here
            // so the Grid view can ask for a tile's real thumbnail texture. main() constructs
            // it unconditionally, so unlike this struct's other pointer fields it is always
            // non-null once main() has run past that point; there is no headless-editor case
            // for it today.
            ThumbnailCache* thumbnail_cache = nullptr;

            // The asset the Cooking Override modal is open for; empty means closed. Staged
            // here rather than as a modal-local static so a right-click on a second asset
            // while the modal is up retargets it instead of the two fighting over one popup.
            std::string cooking_override_target;

            // The Inspector's Soft Body section stages the source mesh path here between
            // frames -- `SoftBodyParameters::asset` holds cooked bytes by value, not a path, so
            // there is nowhere on the component itself for a text field to write into.
            std::string soft_body_source_path;

            // The Renderer's Mesh combo, mid-transition into Imported mode for the entity
            // named below, before a mesh has actually been imported. ShapeParameters has no
            // "wants imported" field of its own (only mesh != INVALID_MESH means imported),
            // so this bridges the frame gap between picking "Imported" and Load succeeding —
            // reset whenever a different entity's Renderer section is drawn, so it cannot
            // leak a stale "Imported" mode onto an unrelated entity.
            SushiEngine::Simulation::EntityId shape_picker_pending_entity =
                SushiEngine::Simulation::NULL_ENTITY;
            bool shape_picker_wants_imported = false;

            // The Renderer's Source Mesh field stages its typed path here, the same way
            // `soft_body_source_path` below stages the Soft Body section's -- so typing does
            // not commit a `set_shape_parameters` (and the full extract it triggers) on every
            // keystroke. Seeded from `ShapeParameters::mesh_path` whenever the entity above
            // changes, and cleared when the Mesh combo leaves Imported mode, so it never shows
            // a path that does not belong to the entity on screen.
            std::string shape_picker_source_path;

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

            enum class ProjectPickerMode { New, Load };

            bool show_project_picker = false;
            ProjectPickerMode project_picker_mode = ProjectPickerMode::Load;
            std::string project_picker_directory;       // where the modal is currently browsing
            std::string project_picker_new_folder_name; // New mode only

            // The scene currently open, if any (empty means unsaved/new). Save writes
            // here directly; Save As and the Save-As-prompted first save go through
            // `show_save_scene_as`, an inline filename popup rooted at `project_root`.
            std::string scene_path;
            bool show_save_scene_as = false;
            std::string save_scene_as_name;

            // File ▸ Open Scene…: the path prompt's visibility and its typed path
            // (relative paths resolve against `project_root`).
            bool show_open_scene = false;
            std::string open_scene_path;

            // Undo/redo over whole-world snapshots; panels record before a mutation (see
            // Authoring::CommandHistory) and the menu/keyboard shortcuts drive undo()/redo().
            Authoring::CommandHistory history;

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
                Open,
                SwitchProject
            };
            PendingSceneAction pending_scene_action = PendingSceneAction::None;
            std::string pending_scene_open_path;
            std::string pending_project_switch_path; // target for SwitchProject

            // A command against the selection, parked until the frame's panels have all
            // been drawn. Every one of these creates or destroys entities, and the surfaces
            // that offer them (the Hierarchy's row menus, its empty-space menu, the Edit
            // menu, the keyboard shortcuts) are drawn while a local copy of the entity list
            // is being walked — so acting immediately leaves the rest of that walk holding
            // ids of entities that no longer exist. Deferring is not a nicety here; it is
            // the difference between a correct frame and reading a destroyed entity.
            // Resolved once per frame by `run_pending_entity_command`.
            enum class EntityCommand
            {
                None,
                Copy,
                Cut,
                Paste,
                Duplicate,
                Delete
            };
            EntityCommand pending_entity_command = EntityCommand::None;

            // Project panel state: the single selected file/folder (full path, empty if
            // none), the path currently in inline rename, and the name-search filter
            // applied to the current folder's contents.
            std::string selected_project_path;
            std::string renaming_project_path;
            std::string project_filter;

            // The model dropped onto the Scene view this frame, or empty. The viewport reports
            // where a drop landed and nothing more — it draws a rendered image and knows
            // nothing about prefabs, assets or the world — so the main loop reads this after
            // the panel is drawn, places the instance, and clears it. Acting inside the panel
            // would put a world edit in the middle of a walk the panel is in the middle of.
            std::string dropped_model_path;

            // Scratch state for the shared inline-rename field (`inline_rename_field`),
            // which the Hierarchy's tree rows, its filtered search rows, and the Project
            // tiles all draw. Held here rather than as a static inside each panel so the
            // three renames cannot fight over one buffer, and so the state is inspectable.
            // `rename_target` is the opaque key of the row being renamed (an entity id or a
            // path), used only to detect when the target changed and the buffer needs
            // re-seeding; empty means no rename field is live.
            std::string rename_target;
            std::string rename_buffer;

            std::vector<Document> documents;
            int active_document = -1;

            // Index of the document whose close was requested while it still had unsaved
            // changes, or -1 when no prompt is up. The tab stays open until the prompt is
            // answered, so closing a dirty buffer can no longer discard it silently.
            int closing_document = -1;

            Authoring::PanelVisibility panels;

            // Between-frame scratch for the panels that keep any (see panel_state.hpp).
            PanelState panel_state;

            // One-shot request from Window ▸ Reset Layout: the main loop rebuilds the
            // default dock layout and resets `panels` to defaults on the frame it is
            // set, then clears it — the escape hatch for any wedged drag state.
            bool layout_reset_requested = false;

            // Whether the Scene view is maximized over the whole editor viewport
            // (Unity's Shift+Space). Session state, deliberately not persisted: an
            // editor that reopens fullscreen hides its own layout.
            bool scene_view_fullscreen = false;

            PlayState play_state = PlayState::Stopped;

            // The scene captured at the moment Play was pressed, restored verbatim on
            // Stop (see scene_serializer.hpp's capture_scene/apply_scene) — mirrors
            // Unity's edit-mode-is-never-mutated-by-play-mode guarantee. Empty means no
            // Play session is in progress.
            std::optional<nlohmann::json> play_mode_snapshot;

            // The cooked soft-body assets `play_mode_snapshot` names by content hash
            // rather than inlining. Owned beside the snapshot because the two are one
            // thing: a snapshot restored against a different table restores no bodies.
            Scene::SceneBlobTable play_mode_blobs;

            // One-shot request from the toolbar's Step button: advance the world exactly
            // one tick this frame regardless of play_state (typically pressed while
            // Paused), then the main loop clears it.
            bool step_requested = false;

            std::string hierarchy_filter;

            // Snapshot of the last Copy/Cut, replayed by Paste (see `ClipboardEntity`).
            // Cut fills this exactly like Copy, then additionally deletes the originals.
            /**
             * @brief Each emitter entity's own authored effect, keyed by entity.
             *
             * A particle emitter is a **component**: putting one on an entity is what makes that
             * entity a particle system, and the Particle Editor is that component's editing
             * surface. So the effect is authored per entity rather than shared — an entry appears
             * the first time an emitter is edited and is pushed into the world under a name derived
             * from the entity, which keeps one emitter's edits off every other emitter.
             *
             * Effects loaded from `assets/effects` are templates: loading one copies it into the
             * selected entity rather than binding the entity to a shared asset.
             *
             * This is only the panel's scratch copy of the selected entity's effect — the effect
             * itself lives on the component, in the world, which is what lets the scene file
             * round-trip it.
             */
            SushiEngine::VFX::ParticleEffect particle_effect_scratch;

            /** @brief The entity @ref particle_effect_scratch was read from. */
            SushiEngine::Simulation::EntityId particle_effect_entity =
                SushiEngine::Simulation::NULL_ENTITY;

            /** @brief Set when the scratch changed outside a widget edit (a library load). */
            bool particle_effect_dirty = false;

            /**
             * @brief Whether a particle-effect drag currently holds a pending undo step.
             *
             * The particle panel's widgets edit the scratch directly and report no
             * per-widget change, so its undo bracketing is edge-triggered from
             * "the scratch diverged this frame" — this flag is the begin_change /
             * end_change state between those frames.
             */
            bool particle_effect_change_active = false;

            /** @brief The Meteorology panel's CSV logger (see meteorology_log.hpp). */
            MeteorologyLog meteorology_log;

            /**
             * @brief Whether an environment-editing drag holds a pending undo step.
             *
             * Shared by every panel that writes the environment (Environment, Lighting,
             * Meteorology) through `commit_environment_edit`: the panels detect a change
             * by memcmp after all their widgets ran, not per widget, so the bracketing is
             * edge-triggered the same way the particle panel's is.
             */
            bool environment_change_active = false;

            std::vector<ClipboardEntity> clipboard;

            // The last Copy Values from a component header, replayed by Paste Values on a
            // header of the same component. Deliberately separate from `clipboard`: copying
            // a Rigid Body's tuning to another entity must not destroy the entities the user
            // had on the entity clipboard, and Ctrl+V must not paste half an entity.
            ComponentValueClipboard component_clipboard;

            Console console;

            std::size_t world_entity_count = 0;

            /**
             * @brief What the last physics step contained, snapshotted per frame.
             *
             * A copy rather than a reference to the live simulation, matching how the
             * GPU timings and the entity count already reach the panels: the UI reads
             * a frame's worth of numbers and cannot reach back into the world through
             * them.
             */
            SushiEngine::Physics::PhysicsStatistics physics_statistics;

            /**
             * @brief Which §14 physics debug categories the Scene view draws.
             *
             * On the context rather than on the Physics panel that toggles them, because
             * the panel that *sets* them and the viewport that *reads* them are two
             * different windows — and a setting owned by one of them would be a setting the
             * other could not see with the first one closed.
             */
            PhysicsOverlaySettings physics_overlay;

            /**
             * @brief Whether a sleeping island's joints should be dropped from the solve graph.
             *
             * `ISimulation::set_park_sleeping_joints`, staged the same way as everything else
             * a panel sets and main.cpp pushes into the live simulation each frame — off by
             * default, matching the engine-side default (§16.44).
             */
            bool park_sleeping_joints = false;

            /**
             * @brief The driving controls' ramped positions, between frames.
             *
             * A key is a bit and a throttle is not, so the two live here rather than being
             * recomputed from the key each frame: a ramp needs to remember where it got to.
             * On the context rather than inside the driving function because that function
             * is a plain call rather than an object — the same reason the panels' state is
             * held out here (see `drive_selected_vehicle`).
             */
            float drive_throttle = 0.0f;
            float drive_steer = 0.0f;

            // Which display the Game view renders, chosen from the resolved cameras.
            std::uint32_t game_display = 0;

            // The Game view's aspect/orientation/fullscreen toolbar state.
            Authoring::GameViewSettings game_view_settings;

            // The active Scene-view transform tool (W/E/R) and axis frame (Local/World),
            // shared between the toolbar that sets them and the viewport that draws the
            // matching gizmo.
            Authoring::GizmoMode gizmo_mode = Authoring::GizmoMode::Translate;
            Authoring::GizmoSpace gizmo_space = Authoring::GizmoSpace::World;

            // Which of the soft-body debug views the Scene view draws over the selected
            // body (§9.3/§9.4, P6-G5). Off by default: an overlay that is always on is an
            // overlay nobody reads, and this one draws over the surface it is explaining.
            // Scene view only, like the collider overlay — the Game view is what the
            // player sees, and a debug view there would be showing them the workings.
            Authoring::SoftBodyDebugView soft_body_debug_view = Authoring::SoftBodyDebugView::Off;

            // Input (Phase 6): the device-abstracted action layer the editor consumes instead of
            // polling ImGui keys. main() sets these each frame after folding input, so panels read
            // resolved actions (input_snapshot) by name and the Authoring::Preferences page rebinds
            // against the live contexts (via input_manager). All non-owning; main() owns the
            // objects.
            const SushiEngine::Input::ActionSnapshot* input_snapshot = nullptr;
            SushiEngine::Input::InputManager* input_manager = nullptr;
            SushiEngine::Input::InputContext* editor_global_context = nullptr;
            SushiEngine::Input::InputContext* editor_viewport_context = nullptr;

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
            // than on every widget tick. The Authoring::Preferences and Input Manager windows are
            // toggled through `panels` like every other window.
            Authoring::Preferences preferences;
            Authoring::IPreferencesStore* preferences_store = nullptr;
            bool preferences_dirty = false;

            // How the viewports trade fidelity against frame time: anti-aliasing mode,
            // render scale, the dynamic-resolution governor, and variable-rate shading.
            // Edited in the Rendering panel and pushed to both viewports each frame, so
            // the two always agree.
            SushiEngine::Render::RenderSettings render_settings;

            // The simulation-side quality budgets — the atmosphere tier that resolves
            // the nest grid. Its own aggregate, deliberately not part of
            // `render_settings`: selecting a render tier must never rebuild the weather.
            // Edited in the Meteorology panel, persisted per user like render_settings.
            SushiEngine::Simulation::SimulationSettings simulation_settings;

            // The internal resolution the Scene view last rendered at, written back by
            // the main loop. It is the only way to see the dynamic-resolution governor
            // work: the scale slider is a request, this is what it settled on.
            std::uint32_t scene_render_width = 0;
            std::uint32_t scene_render_height = 0;

            // The Scene view's GPU cull counts (survived / considered), read back a frame
            // late by the main loop for the GPU Culling panel. Both zero on the classic
            // path or before the first GPU-driven frame.
            std::uint32_t scene_cull_drawn = 0;
            std::uint32_t scene_cull_tested = 0;

            // Each visible viewport's per-pass GPU times, refilled by the main loop
            // after the viewports render and shown in the Statistics panel.
            std::vector<ViewportGPUStatistics> gpu_statistics;

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
         * aggregate, not on a logging interface. The level defaults to Info so the ~150
         * existing call sites keep meaning what they meant; anything that reports a problem
         * should pass Warning or Error, because a console where everything is Info is a
         * console nobody can filter.
         *
         * @param context Editor state whose console receives the line.
         * @param message Text to record.
         * @param level How much attention the line wants.
         */
        inline void editor_log(EditorContext& context, const std::string& message,
                               LogLevel level = LogLevel::Info)
        {
            context.console.append(level, message);
        }

        /**
         * @brief Says what reading a project moved out of its cooking document.
         *
         * A project written before per-asset cooking overrides lived beside their assets
         * carries them in the document instead, and reading it moves them once. Silence would
         * be the wrong outcome for both halves: an artist whose settings changed file needs to
         * know where they went, and an override whose asset is gone is settings that are lost.
         *
         * Takes the two path lists rather than the bake model, so the context header keeps its
         * forward declaration of `Authoring::CookBakeState` instead of including it.
         *
         * @param context  Editor state whose console receives the lines.
         * @param migrated Asset paths whose `.meta` sidecar now carries the override.
         * @param dropped  Asset paths the document named that are no longer on disk.
         */
        inline void log_cooking_override_migration(EditorContext& context,
                                                   const std::vector<std::string>& migrated,
                                                   const std::vector<std::string>& dropped)
        {
            if (!migrated.empty())
                editor_log(context, "Moved " + std::to_string(migrated.size()) +
                                        " per-asset cooking override(s) into .meta sidecars.");
            for (const std::string& asset_path : dropped)
                editor_log(context,
                           "Dropped the cooking override for '" + asset_path +
                               "', which no longer exists.",
                           LogLevel::Warning);
        }

        /**
         * @brief Says which assets cooked at the project default because their `.meta` failed.
         *
         * A sidecar that will not parse is settings that stopped applying, and an artist who
         * is not told assumes the setting is wrong rather than the file
         * (`docs/design/model_import.md` §8). One line per asset, at Warning, because a cook
         * that used the wrong profile still produced an asset and nothing else reports it.
         *
         * Takes the path list rather than the bake model, so the context header keeps its
         * forward declaration of `Authoring::CookBakeState` instead of including it.
         *
         * @param context     Editor state whose console receives the lines.
         * @param asset_paths Assets whose sidecar could not be read, from
         *                    `Authoring::CookBakeState::take_unreadable_sidecars`.
         */
        inline void log_unreadable_import_sidecars(EditorContext& context,
                                                   const std::vector<std::string>& asset_paths)
        {
            for (const std::string& asset_path : asset_paths)
                editor_log(context,
                           "Could not parse the .meta sidecar for '" + asset_path +
                               "'; cooked it at the project default instead.",
                           LogLevel::Warning);
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
