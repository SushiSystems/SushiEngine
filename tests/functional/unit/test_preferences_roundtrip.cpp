/**************************************************************************/
/* test_preferences_roundtrip.cpp                                         */
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

// The preferences store's round-trip contract, pinned for the session state the
// editor's layout persistence rides on: the open window set (PanelVisibility),
// the Game view toolbar (GameViewSettings), and the gizmo tool/space. What the
// editor saves at exit must be exactly what the next launch restores — and a
// missing or empty file must degrade to defaults, never to a throw.

#include <filesystem>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include <SushiEngine/authoring/preferences.hpp>

namespace
{
    namespace fs = std::filesystem;
    using SushiEngine::Editor::EditorTheme;
    using SushiEngine::Editor::GameViewAspectPreset;
    using SushiEngine::Editor::GameViewOrientation;
    using SushiEngine::Editor::GizmoMode;
    using SushiEngine::Editor::GizmoSpace;
    using SushiEngine::Editor::Preferences;

    /** @brief A scratch preferences path in the system temp dir, removed on destruction. */
    class ScratchPreferencesFile
    {
        public:
            explicit ScratchPreferencesFile(const char* name)
                : path_((fs::temp_directory_path() / name).string())
            {
                std::error_code error;
                fs::remove(path_, error);
            }

            ~ScratchPreferencesFile()
            {
                std::error_code error;
                fs::remove(path_, error);
            }

            const std::string& path() const noexcept { return path_; }

        private:
            std::string path_;
    };

    /** @brief A Preferences with every session-state field moved off its default. */
    Preferences non_default_preferences()
    {
        Preferences preferences;
        preferences.theme = EditorTheme::Classic;
        preferences.camera_move_speed = 9.5f;
        preferences.simulation.atmosphere.quality =
            SushiEngine::Simulation::AtmosphereQuality::Ultra;

        preferences.panels.scene_view = false;
        preferences.panels.console = false;
        preferences.panels.environment = false;
        preferences.panels.meteorology = true;
        preferences.panels.audio_mixer = true;
        preferences.panels.audio_authoring = true;
        preferences.panels.animator_graph = true;
        preferences.autosave = true;
        preferences.autosave_interval_seconds = 45.0f;
        preferences.panels.preview = true;
        preferences.panels.preferences = true;
        preferences.panels.input_manager = true;

        preferences.game_view.aspect = GameViewAspectPreset::Ultrawide21x9;
        preferences.game_view.orientation = GameViewOrientation::Portrait;
        preferences.game_view.fullscreen = true;

        preferences.gizmo_mode = GizmoMode::Rotate;
        preferences.gizmo_space = GizmoSpace::Local;
        return preferences;
    }

    void expect_session_state_equal(const Preferences& actual, const Preferences& expected)
    {
        EXPECT_EQ(actual.theme, expected.theme);
        EXPECT_FLOAT_EQ(actual.camera_move_speed, expected.camera_move_speed);
        EXPECT_EQ(actual.simulation.atmosphere.quality, expected.simulation.atmosphere.quality);

        EXPECT_EQ(actual.panels.scene_view, expected.panels.scene_view);
        EXPECT_EQ(actual.panels.game_view, expected.panels.game_view);
        EXPECT_EQ(actual.panels.hierarchy, expected.panels.hierarchy);
        EXPECT_EQ(actual.panels.inspector, expected.panels.inspector);
        EXPECT_EQ(actual.panels.project, expected.panels.project);
        EXPECT_EQ(actual.panels.text_editor, expected.panels.text_editor);
        EXPECT_EQ(actual.panels.console, expected.panels.console);
        EXPECT_EQ(actual.panels.statistics, expected.panels.statistics);
        EXPECT_EQ(actual.panels.animation, expected.panels.animation);
        EXPECT_EQ(actual.panels.animator_graph, expected.panels.animator_graph);
        EXPECT_EQ(actual.panels.animator_preview, expected.panels.animator_preview);
        EXPECT_EQ(actual.panels.environment, expected.panels.environment);
        EXPECT_EQ(actual.panels.rendering, expected.panels.rendering);
        EXPECT_EQ(actual.panels.lighting, expected.panels.lighting);
        EXPECT_EQ(actual.panels.post_process, expected.panels.post_process);
        EXPECT_EQ(actual.panels.meteorology, expected.panels.meteorology);
        EXPECT_EQ(actual.panels.gpu_culling, expected.panels.gpu_culling);
        EXPECT_EQ(actual.panels.physics, expected.panels.physics);
        EXPECT_EQ(actual.panels.preview, expected.panels.preview);
        EXPECT_EQ(actual.panels.audio_mixer, expected.panels.audio_mixer);
        EXPECT_EQ(actual.panels.audio_profiler, expected.panels.audio_profiler);
        EXPECT_EQ(actual.panels.audio_authoring, expected.panels.audio_authoring);
        EXPECT_EQ(actual.autosave, expected.autosave);
        EXPECT_FLOAT_EQ(actual.autosave_interval_seconds, expected.autosave_interval_seconds);
        EXPECT_EQ(actual.panels.preferences, expected.panels.preferences);
        EXPECT_EQ(actual.panels.input_manager, expected.panels.input_manager);

        EXPECT_EQ(actual.game_view.aspect, expected.game_view.aspect);
        EXPECT_EQ(actual.game_view.orientation, expected.game_view.orientation);
        EXPECT_EQ(actual.game_view.fullscreen, expected.game_view.fullscreen);

        EXPECT_EQ(actual.gizmo_mode, expected.gizmo_mode);
        EXPECT_EQ(actual.gizmo_space, expected.gizmo_space);
    }
} // namespace

TEST(Unit_PreferencesRoundTrip, SessionStateSurvivesSaveAndLoad)
{
    const ScratchPreferencesFile file("sushiengine_test_preferences_roundtrip.json");
    const Preferences saved = non_default_preferences();

    ASSERT_TRUE(SushiEngine::Editor::create_preferences_store(file.path())->save(saved));
    const Preferences loaded =
        SushiEngine::Editor::create_preferences_store(file.path())->load();

    expect_session_state_equal(loaded, saved);
}

TEST(Unit_PreferencesRoundTrip, MissingFileYieldsDefaults)
{
    const ScratchPreferencesFile file("sushiengine_test_preferences_missing.json");

    const Preferences loaded =
        SushiEngine::Editor::create_preferences_store(file.path())->load();

    expect_session_state_equal(loaded, Preferences{});
}

TEST(Unit_PreferencesRoundTrip, FileWithoutSessionKeysFallsBackToDefaults)
{
    // An older preferences.json predating the session-state fields: every present
    // key loads, every absent one keeps its compiled-in default instead of failing.
    const ScratchPreferencesFile file("sushiengine_test_preferences_partial.json");
    {
        std::ofstream output(file.path(), std::ios::trunc);
        ASSERT_TRUE(output.is_open());
        output << "{\"theme\": \"classic\"}\n";
    }

    const Preferences loaded =
        SushiEngine::Editor::create_preferences_store(file.path())->load();

    Preferences expected;
    expected.theme = EditorTheme::Classic;
    expect_session_state_equal(loaded, expected);
}
