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

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The precision the live simulation's physics solve runs in.
         *
         * A runtime choice, mapped to `Simulation::Precision`: changing it rebuilds the
         * running simulation in the new precision (the scene is preserved), no rebuild of
         * the binary required. This is distinct from the render/ECS boundary `Scalar`,
         * which remains a build-time option (SE_SCALAR_DOUBLE) reported by
         * @ref current_precision.
         */
        enum class ScalarPrecision
        {
            Single,
            Double
        };

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
         * that take effect live (theme, grid, camera speed, snap, and @ref precision,
         * which rebuilds the running simulation in the chosen physics-solve precision).
         */
        struct Preferences
        {
            ScalarPrecision precision = ScalarPrecision::Single;
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
        };

        /**
         * @brief The precision this binary was compiled with.
         *
         * Reads the SE_SCALAR_DOUBLE build macro, so the Preferences window can compare
         * the user's chosen precision against reality and prompt a rebuild when they
         * differ. Kept inline (rather than in the store) because it is a pure build fact.
         *
         * @return ScalarPrecision::Double under a double build, else ::Single.
         */
        inline ScalarPrecision current_precision() noexcept
        {
    #ifdef SE_SCALAR_DOUBLE
            return ScalarPrecision::Double;
    #else
            return ScalarPrecision::Single;
    #endif
        }

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
                 * A first run (no file) yields a default-constructed @ref Preferences whose
                 * @ref Preferences::precision is seeded from @ref current_precision so the
                 * UI starts consistent with the running binary.
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
