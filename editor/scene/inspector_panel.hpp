/**************************************************************************/
/* inspector_panel.hpp                                                    */
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
 * @file inspector_panel.hpp
 * @brief The Inspector window: every component of the selected entity.
 *
 * One section per component — transform and reference frame, camera, primitive shape,
 * material, collider and physics body, cloth, light, decal, UI element, scripts, the
 * particle system — each editing the live component in place and bracketing its edit as
 * one undo step. The two largest sections (particle systems and scripts) live in their
 * own translation units and are called from here.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Inspector window for the current primary selection.
         *
         * @param context Editor state; edits the selected entity through the world editor.
         */
        void draw_inspector_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
