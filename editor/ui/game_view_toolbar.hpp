/**************************************************************************/
/* game_view_toolbar.hpp                                                  */
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

#ifndef SUSHIENGINE_EDITOR_GAME_VIEW_TOOLBAR_HPP
#define SUSHIENGINE_EDITOR_GAME_VIEW_TOOLBAR_HPP

#include <imgui.h>

#include "../core/game_view_settings.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Game view's aspect/orientation/fullscreen combo row.
         *
         * Shared by the normal (camera-rendering) and the no-camera placeholder path so
         * the two never drift into two different toolbars. Assumes the caller is already
         * inside the panel's `ImGui::Begin`/`ImGui::End` pair.
         *
         * @param settings The settings this row edits in place.
         */
        inline void draw_game_view_toolbar(GameViewSettings& settings)
        {
            static const char* ASPECT_LABELS[] = {"Free Aspect", "Standard (4:3)", "Widescreen (16:9)",
                                                  "Ultrawide (21:9)", "Square (1:1)"};
            int aspect_index = static_cast<int>(settings.aspect);
            ImGui::SetNextItemWidth(170.0f);
            if (ImGui::BeginCombo("##game_view_aspect", ASPECT_LABELS[aspect_index]))
            {
                for (int i = 0; i < IM_ARRAYSIZE(ASPECT_LABELS); ++i)
                {
                    const bool selected = i == aspect_index;
                    if (ImGui::Selectable(ASPECT_LABELS[i], selected))
                        settings.aspect = static_cast<GameViewAspectPreset>(i);
                    if (selected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }

            // Orientation has no effect on a square aspect and none at all on Free, so it
            // is hidden rather than shown disabled — nothing for the user to set either way.
            if (settings.aspect != GameViewAspectPreset::Free &&
                settings.aspect != GameViewAspectPreset::Square1x1)
            {
                ImGui::SameLine();
                const bool portrait = settings.orientation == GameViewOrientation::Portrait;
                static const char* ORIENTATION_LABELS[] = {"Landscape", "Portrait"};
                int orientation_index = portrait ? 1 : 0;
                ImGui::SetNextItemWidth(110.0f);
                if (ImGui::BeginCombo("##game_view_orientation", ORIENTATION_LABELS[orientation_index]))
                {
                    for (int i = 0; i < 2; ++i)
                    {
                        const bool selected = i == orientation_index;
                        if (ImGui::Selectable(ORIENTATION_LABELS[i], selected))
                            settings.orientation = i == 1 ? GameViewOrientation::Portrait
                                                          : GameViewOrientation::Landscape;
                        if (selected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
            }

            ImGui::SameLine();
            ImGui::Checkbox("Fullscreen", &settings.fullscreen);
        }

    } // namespace Editor
} // namespace SushiEngine

#endif
