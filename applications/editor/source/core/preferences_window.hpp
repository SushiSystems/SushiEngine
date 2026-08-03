/**************************************************************************/
/* preferences_window.hpp                                                 */
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
 * @file preferences_window.hpp
 * @brief The Authoring::Preferences window: the editor's own settings, not the scene's.
 *
 * Beside `preferences.{hpp,cpp}` because it is the one surface that edits that
 * aggregate wholesale. Any change here raises `preferences_dirty`, and the main loop
 * both persists the store and applies the fields that take effect live.
 */

#include "editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Authoring::Preferences window when its panel flag is set.
         *
         * Edits @ref EditorContext::preferences in place across its General / Editor /
         * Scene sections. The precision control is compile-time, so it records intent and
         * says a rebuild is needed rather than pretending to switch at runtime.
         *
         * @param context Editor state; edits the preferences aggregate.
         */
        void draw_preferences_window(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
