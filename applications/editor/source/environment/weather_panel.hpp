/**************************************************************************/
/* weather_panel.hpp                                                      */
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
 * @file weather_panel.hpp
 * @brief The Environment window: what surrounds the scene, as a level artist sees it.
 *
 * The sun and sky, the fog and its volumes, global illumination, the surface, the
 * stars, the cloud decks and the weather that drives them, and the observer's place
 * on the planet. Everything here is scene content — it round-trips through the
 * `.sushiscene` file and is what a saved level looks like when it is reopened.
 *
 * The nest's *grid* and its physics constants deliberately live next door in the
 * Meteorology panel: those are the simulation's budget and its parameters, not the
 * look of the world, and one panel owning both was how a lighting tweak came to
 * restart the weather.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Environment window: sun, sky, fog, GI, surface, clouds, weather.
         *
         * Writes the world's @ref SushiEngine::Render::Environment through the shared
         * environment-edit bracket, so a slider drag is one undo step and the edit is
         * saved with the scene rather than with the user's preferences.
         *
         * @param context Editor state; reads and writes the world's environment.
         */
        void draw_environment_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
