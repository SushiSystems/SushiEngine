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

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/render_settings.hpp>

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
             * @brief The sky/lighting setup edited in the Environment and Lighting panels.
             *
             * This is an editor/host setting, not scene data: it describes how the editor
             * *displays* whatever scene is open, so it lives here (persisted once per user)
             * rather than round-tripping through every .sushiscene file.
             */
            SushiEngine::Render::Environment environment;
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
         * @brief Creates the default JSON-backed preferences store.
         *
         * Resolves the per-user config directory (%APPDATA%/SushiEngine on Windows,
         * $XDG_CONFIG_HOME or ~/.config/SushiEngine elsewhere) and targets
         * `preferences.json` inside it — a location distinct from the build-tool config
         * under `cli/`.
         *
         * @return A store owning that path.
         */
        std::unique_ptr<IPreferencesStore> create_preferences_store();
    } // namespace Editor
} // namespace SushiEngine

#endif
