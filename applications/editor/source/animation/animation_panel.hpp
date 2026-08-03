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

#include <string>

#include <SushiEngine/animation/keyframe.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The Animation window's document: the clip being authored, and the transport.
         *
         * Held by the caller rather than as a static inside the panel, so the clip under
         * authoring is inspectable state with one owner instead of a hidden process-global
         * that no second view or reset could ever reach.
         */
        struct AnimationState
        {
            float time = 0.0f;         /**< Playhead position, in seconds. */
            float sample_rate = 30.0f; /**< Bake rate, in samples per second. */
            float length = 3.0f;       /**< Clip length, in seconds. */
            bool playing = false;      /**< Transport is running. */
            bool recording = false;    /**< Moving the target keys it at the playhead. */
            char save_path[256] = "clip.sushianim"; /**< Bake destination. */
            std::string status;                     /**< Result of the last bake. */

            Simulation::EntityId target = Simulation::NULL_ENTITY; /**< The entity being animated. */
            std::string target_name;
            /** Position/rotation/scale curves of the target's transform. */
            Animation::JointChannels channels;
            bool have_last = false;                    /**< Whether `last_seen` holds a reading. */
            Simulation::EntityTransform last_seen{};   /**< Last frame's transform (change detection). */

            int selected_row = 0;  /**< Highlighted track in the timeline. */
            int selected_key = -1; /**< Highlighted key, or -1 for none. */
        };

        /**
         * @brief Draw the Animation window: record and key the selected Hierarchy object.
         *
         * The authoring surface for the keyframe model (`animation/keyframe.hpp`), Unity's
         * Animation window shape. It targets the entity selected in the Hierarchy: with **Record**
         * on, moving the object (gizmo, physics) keys its transform at the playhead; with Record
         * off, scrubbing or playing evaluates the curves and drives the object live in the Scene
         * view. A transport, an autokey timeline of draggable key diamonds (click to add,
         * right-click to delete), and Bake, which resamples to a dense `.sushianim` on disk. With
         * nothing selected it falls back to abstract named scalar tracks.
         *
         * @param context Shared editor state; the selected entity, the world editor, the visibility flag.
         * @param state   The clip being authored and the transport driving it, owned by the caller.
         */
        void draw_animation_panel(EditorContext& context, AnimationState& state);
    } // namespace Editor
} // namespace SushiEngine

#endif
