/**************************************************************************/
/* preferences_window.cpp                                                */
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

#include "preferences_window.hpp"

#include "../ui/editor_panels.hpp"

#include <imgui.h>
#include <misc/cpp/imgui_stdlib.h>

namespace SushiEngine
{
    namespace Editor
    {
        void draw_preferences_window(EditorContext& context)
        {
            if (!context.panels.preferences)
                return;

            ImGui::SetNextWindowSize(ImVec2(460.0f, 420.0f), ImGuiCond_FirstUseEver);
            if (!ImGui::Begin("Preferences", &context.panels.preferences))
            {
                ImGui::End();
                return;
            }

            Authoring::Preferences& preferences = context.preferences;
            bool changed = false;

            if (ImGui::CollapsingHeader("General", ImGuiTreeNodeFlags_DefaultOpen))
            {
                const char* theme_items[] = {"Dark", "Light", "Classic"};
                int theme_index = static_cast<int>(preferences.theme);
                if (ImGui::Combo("Theme", &theme_index, theme_items, 3))
                {
                    preferences.theme = static_cast<Authoring::EditorTheme>(theme_index);
                    apply_theme(preferences.theme);
                    changed = true;
                }
            }

            if (ImGui::CollapsingHeader("Editor", ImGuiTreeNodeFlags_DefaultOpen))
            {
                changed |= ImGui::Checkbox("Autosave", &preferences.autosave);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Saves a dirty scene to its file on the interval below.\n"
                                      "A scene that has never been saved is left alone (no\n"
                                      "surprise Save-As), and a clean scene is never rewritten.");
                if (preferences.autosave)
                    changed |= ImGui::SliderFloat("Autosave interval",
                                                  &preferences.autosave_interval_seconds, 15.0f,
                                                  600.0f, "%.0f s");
                changed |= ImGui::DragFloat("Camera move speed", &preferences.camera_move_speed,
                                            0.1f, 0.1f, 100.0f, "%.1f");
            }

            if (ImGui::CollapsingHeader("Scene", ImGuiTreeNodeFlags_DefaultOpen))
            {
                changed |= ImGui::Checkbox("Show grid", &preferences.grid_visible);
                changed |= ImGui::Checkbox("Snap to grid", &preferences.snap_enabled);
                if (preferences.snap_enabled)
                {
                    changed |= ImGui::DragFloat("Move snap", &preferences.snap_translate,
                                                0.01f, 0.001f, 10.0f, "%.3f");
                    changed |= ImGui::DragFloat("Rotate snap (deg)", &preferences.snap_rotate_degrees,
                                                0.5f, 1.0f, 90.0f, "%.1f");
                    changed |= ImGui::DragFloat("Scale snap", &preferences.snap_scale,
                                                0.01f, 0.001f, 10.0f, "%.3f");
                }
            }

            ImGui::Separator();
            if (ImGui::Button("Configure Input... (Edit > Input Manager)"))
                context.panels.input_manager = true;

            ImGui::Separator();
            if (context.preferences_store != nullptr)
                ImGui::TextDisabled("%s", context.preferences_store->path().c_str());

            if (changed)
                context.preferences_dirty = true;

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
