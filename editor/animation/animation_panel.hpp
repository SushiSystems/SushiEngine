/**************************************************************************/
/* animation_panel.hpp                                                    */
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

#ifndef SUSHIENGINE_EDITOR_ANIMATION_PANEL_HPP
#define SUSHIENGINE_EDITOR_ANIMATION_PANEL_HPP

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draw the Animation window: record and key the selected Hierarchy object.
         *
         * The authoring surface for the keyframe model (`animation/keyframe.hpp`), Unity's
         * Animation window shape. It targets the entity selected in the Hierarchy: with **Record**
         * on, moving the object (gizmo, physics) keys its transform at the playhead; with Record
         * off, scrubbing or playing evaluates the curves and drives the object live in the Scene
         * view. A transport, an autokey timeline of draggable key diamonds (click to add,
         * right-click to delete), and Bake, which resamples to a dense `.sushianim` on disk. With
         * nothing selected it falls back to abstract named scalar tracks. The panel owns its
         * document in file-static state, like the other panels' widget state.
         *
         * @param context Shared editor state; the selected entity, the world editor, the visibility flag.
         */
        void draw_animation_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine

#endif
