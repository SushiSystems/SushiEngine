/**************************************************************************/
/* lighting_panel.hpp                                                     */
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
 * @file lighting_panel.hpp
 * @brief The Lighting window: the punctual lights, the IBL source, and shadow quality.
 *
 * Separate from the render-settings panels next door because it edits a different
 * kind of thing: those describe the machinery drawing the frame, this describes what
 * is lighting it. The sun deliberately is *not* here — it lives in Environment, its
 * single owner, and this panel links there rather than offering a second copy of the
 * same four sliders with a different legal range.
 */

#include "../ui/component_editor.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws a punctual light's editable fields into the current window.
         *
         * Exported because a light is authored in two places — this panel lists every light
         * in the scene, the Inspector shows the selected entity's — and the two must offer the
         * same fields with the same units, ranges and explanations. The domain that owns
         * lights owns their field list, the same way the VFX and audio units own theirs; a
         * second copy in the Inspector is how the two views drift apart.
         *
         * @param editor The section's editor, already scoped to the light being edited: the
         *               Inspector's follows the selection, this panel's addresses one row.
         */
        void draw_light_fields(ComponentEditor<Simulation::LightParams>& editor);

        /**
         * @brief Draws the Lighting window: every light-bearing entity, plus IBL and shadows.
         *
         * Lists the world's punctual lights editable in place (selecting one selects its
         * entity), with a button to add one, alongside the image-based-lighting source and
         * the shadow-rendering controls the light engine reads.
         *
         * @param context Editor state; reads and writes the world's lights and light engine.
         */
        void draw_lighting_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
