/**************************************************************************/
/* preferences.hpp                                                        */
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

#ifndef SUSHIENGINE_EDITOR_PREFERENCES_HPP
#define SUSHIENGINE_EDITOR_PREFERENCES_HPP

#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/authoring/game_view_settings.hpp>
#include <SushiEngine/authoring/gizmo_state.hpp>
#include <SushiEngine/authoring/panel_visibility.hpp>
#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/simulation/simulation_settings.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /** @brief The ImGui colour theme applied to the editor. */
        enum class EditorTheme
        {
            Dark,
            Light,
            Classic
        };

        /**
         * @brief The user-configurable editor and project settings, persisted to disk.
         *
         * A plain aggregate of values — no behaviour — so it can be copied, compared, and
         * serialized freely. The store (@ref IPreferencesStore) owns persistence; the
         * Preferences window edits an instance of this; and the editor applies the fields
         * that take effect live (theme, grid, camera speed, snap).
         */
        struct Preferences
        {
            EditorTheme theme = EditorTheme::Dark;

            bool grid_visible = true;
            float camera_move_speed = 6.0f;

            bool snap_enabled = false;
            float snap_translate = 0.25f;
            float snap_rotate_degrees = 15.0f;
            float snap_scale = 0.1f;

            bool autosave = false;

            /** @brief Seconds of continuous dirty time before an autosave fires. */
            float autosave_interval_seconds = 120.0f;

            std::vector<std::string> recent_scenes;
            std::string last_project_root;

            /**
             * @brief The editor's input bindings, as a serialized JSON document.
             *
             * Held as the dumped text of @ref SushiEngine::Input::bindings_to_json so this
             * struct stays free of the JSON dependency; the store nests it as a real object in
             * the preferences file. Empty means "use the compiled-in defaults". The editor
             * loads it into its @c InputContext with @ref SushiEngine::Input::bindings_from_json,
             * so a stale or partial document degrades to defaults rather than failing.
             */
            std::string input_bindings;

            /**
             * @brief The renderer performance/fidelity trade, persisted across sessions.
             *
             * A host setting (not scene data — see @ref SushiEngine::Render::RenderSettings),
             * so it lives here alongside theme and camera speed rather than in the scene file.
             */
            SushiEngine::Render::RenderSettings render_settings;

            /**
             * @brief The simulation-side quality budgets (the atmosphere tier).
             *
             * A host setting like @ref render_settings and persisted for the same
             * reason: a grid resolution is a machine budget, not scene content.
             * Deliberately its own aggregate so no simulation tier can ride a render
             * tier — the coupling this file's owner once suffered from.
             */
            SushiEngine::Simulation::SimulationSettings simulation;

            /**
             * @brief The environment a *new* scene starts from.
             *
             * The environment itself is scene content — every .sushiscene, undo
             * snapshot, and play-mode snapshot carries it, and a loaded scene's
             * environment is never overridden by this. What the preferences keep is
             * only the starting point File ▸ New Scene applies, so an author who
             * always works at the same latitude with the same sky does not re-author
             * it per scene. Serialized in the same full shape the scene file uses
             * (see `SushiEngine/serialization/environment_serializer.hpp`).
             */
            SushiEngine::Render::Environment default_environment;

            /**
             * @brief Which editor windows are open, so the working set survives a restart.
             *
             * The dock *positions* are ImGui's to persist (layout.ini, in the same config
             * directory); this is the open/closed side of the same session state.
             */
            PanelVisibility panels;

            /** @brief The Game view's aspect/orientation/fullscreen toolbar state. */
            GameViewSettings game_view;

            /** @brief The active Scene-view transform tool (W/E/R), restored on start. */
            GizmoMode gizmo_mode = GizmoMode::Translate;

            /** @brief The gizmo's axis frame (Local/World), restored on start. */
            GizmoSpace gizmo_space = GizmoSpace::World;
        };

        /**
         * @brief Persistence for @ref Preferences, abstracted from any file format.
         *
         * The Preferences window and the editor loop depend on this interface, not on a
         * concrete JSON file, so the storage backend can change without touching the UI
         * (dependency inversion). One implementation, @ref JsonPreferencesStore, writes a
         * JSON file under the per-user config directory.
         */
        class IPreferencesStore
        {
            public:
                virtual ~IPreferencesStore() = default;

                /**
                 * @brief Loads the persisted preferences, or defaults when none exist.
                 *
                 * A first run (no file) yields a default-constructed @ref Preferences.
                 *
                 * @return The loaded (or default) preferences.
                 */
                virtual Preferences load() = 0;

                /**
                 * @brief Persists @p preferences, creating the config directory if needed.
                 * @param preferences The settings to write.
                 * @return True on success; false if the file could not be written.
                 */
                virtual bool save(const Preferences& preferences) = 0;

                /**
                 * @brief The absolute path the store reads from and writes to.
                 * @return The preferences file path, for display and diagnostics.
                 */
                virtual std::string path() const = 0;
        };

        /**
         * @brief The per-user editor config directory, as an absolute path.
         *
         * %APPDATA%/SushiEngine on Windows, $XDG_CONFIG_HOME or ~/.config/SushiEngine
         * elsewhere. The one place this resolution lives: the preferences store writes
         * `preferences.json` here, and the editor pins ImGui's `layout.ini` beside it so
         * the dock layout stops depending on the launch directory. The directory itself
         * is created on first save, not by this call.
         *
         * @return The config directory path (the directory may not exist yet).
         */
        std::string user_config_directory();

        /**
         * @brief Creates the default JSON-backed preferences store.
         *
         * Targets `preferences.json` inside @ref user_config_directory — a location
         * distinct from the build-tool config under `cli/`.
         *
         * @return A store owning that path.
         */
        std::unique_ptr<IPreferencesStore> create_preferences_store();

        /**
         * @brief Creates a JSON-backed preferences store at an explicit @p path.
         *
         * The seam the round-trip tests use: same serialization as the default store,
         * pointed at a temporary file instead of the user's real config.
         *
         * @param path The preferences file to read and write.
         * @return A store owning that path.
         */
        std::unique_ptr<IPreferencesStore> create_preferences_store(const std::string& path);
    } // namespace Editor
} // namespace SushiEngine

#endif
