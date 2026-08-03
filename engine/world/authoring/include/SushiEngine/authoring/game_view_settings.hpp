/**************************************************************************/
/* game_view_settings.hpp                                                 */
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

#ifndef SUSHIENGINE_AUTHORING_GAME_VIEW_SETTINGS_HPP
#define SUSHIENGINE_AUTHORING_GAME_VIEW_SETTINGS_HPP

namespace SushiEngine
{
    namespace Authoring
    {
        /** @brief The Game view's target orientation, applied to every aspect preset but Free. */
        enum class GameViewOrientation
        {
            Landscape,
            Portrait
        };

        /** @brief A Game view aspect/resolution preset (Unity-style), before orientation is applied. */
        enum class GameViewAspectPreset
        {
            Free,        /**< No constraint: the image fills the panel exactly. */
            Standard4x3, /**< 4:3, e.g. 1600x1200 landscape / 1200x1600 portrait. */
            Widescreen16x9, /**< 16:9, e.g. 1920x1080 landscape / 1080x1920 portrait. */
            Ultrawide21x9,  /**< 21:9, e.g. 2560x1080 landscape / 1080x2560 portrait. */
            Square1x1       /**< 1:1, unaffected by orientation. */
        };

        /**
         * @brief The Game view's display settings: aspect/resolution, orientation, fullscreen.
         *
         * Mirrors Unity's Game view toolbar (resolution dropdown + orientation) plus a
         * fullscreen toggle. A plain value type owned by `EditorContext` and read by both
         * the Game view's toolbar (to edit it) and `ViewportPanel::draw` (to constrain the
         * rendered image's aspect ratio).
         */
        struct GameViewSettings
        {
            GameViewAspectPreset aspect = GameViewAspectPreset::Free;
            GameViewOrientation orientation = GameViewOrientation::Landscape;

            // Unity's "Maximize on Play": when set, the Game panel is undocked and
            // resized to cover the whole editor viewport instead of just its dock slot,
            // and the rendered image fills that panel regardless of `aspect`.
            bool fullscreen = false;
        };

        /**
         * @brief Resolves @p preset/@p orientation to a width:height ratio.
         * @param preset      The aspect preset.
         * @param orientation Landscape or portrait; ignored for Square1x1.
         * @param out_ratio   Receives width / height when the preset constrains the aspect.
         * @return False for `Free` (no constraint, @p out_ratio left untouched); true otherwise.
         */
        inline bool resolve_game_view_aspect_ratio(GameViewAspectPreset preset,
                                                   GameViewOrientation orientation,
                                                   float& out_ratio) noexcept
        {
            float ratio;
            switch (preset)
            {
                case GameViewAspectPreset::Standard4x3: ratio = 4.0f / 3.0f; break;
                case GameViewAspectPreset::Widescreen16x9: ratio = 16.0f / 9.0f; break;
                case GameViewAspectPreset::Ultrawide21x9: ratio = 21.0f / 9.0f; break;
                case GameViewAspectPreset::Square1x1: out_ratio = 1.0f; return true;
                case GameViewAspectPreset::Free:
                default: return false;
            }
            out_ratio = orientation == GameViewOrientation::Portrait ? 1.0f / ratio : ratio;
            return true;
        }
    } // namespace Authoring
} // namespace SushiEngine

#endif
