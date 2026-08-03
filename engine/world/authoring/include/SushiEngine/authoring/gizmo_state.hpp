/**************************************************************************/
/* gizmo_state.hpp                                                        */
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

#ifndef SUSHIENGINE_AUTHORING_GIZMO_STATE_HPP
#define SUSHIENGINE_AUTHORING_GIZMO_STATE_HPP

/**
 * @file gizmo_state.hpp
 * @brief The gizmo's mode and axis-frame vocabulary, free of the controller.
 *
 * These two enums are shared editor state — the toolbar sets them, the context
 * holds them, the preferences persist them, and the controller reads them. They
 * live apart from @c gizmo_controller.hpp so a header that only names the state
 * (preferences, context) does not inherit the controller's ImGui dependency.
 */

namespace SushiEngine
{
    namespace Authoring
    {
        /**
         * @brief Which transform channel the viewport gizmo manipulates.
         *
         * Mirrors Unity's W/E/R tools: move, rotate, scale. The active mode is editor
         * state (shared through the context and the toolbar); the controller only reads
         * it to pick which handle set to draw and how a drag maps to the transform.
         */
        enum class GizmoMode
        {
            Translate,
            Rotate,
            Scale
        };

        /**
         * @brief Which frame a Translate/Rotate drag resolves its axes against.
         *
         * World keeps the handles aligned to the world's fixed X/Y/Z. Local aligns them
         * to the selection's own orientation, so dragging X always moves/turns the object
         * along its own facing rather than the world's. Scale always drags in local axes
         * regardless of this setting — a world-aligned scale on a rotated object would
         * shear it, which is never what an author wants.
         */
        enum class GizmoSpace
        {
            Local,
            World
        };
    } // namespace Authoring
} // namespace SushiEngine

#endif
