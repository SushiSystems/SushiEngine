/**************************************************************************/
/* animator_preview_panel.hpp                                             */
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

#ifndef SUSHIENGINE_EDITOR_ANIMATOR_PREVIEW_PANEL_HPP
#define SUSHIENGINE_EDITOR_ANIMATOR_PREVIEW_PANEL_HPP

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draw the Animator Preview window: mask/IK authoring over the live
         * `AnimatedMeshPreview` (design `slop/animation_system.md` §12.1, part 2).
         *
         * This panel is the UI over `AnimatedMeshPreview`'s controller-driven layered/masked/IK
         * `AnimatorEvaluator` path: a layer list (add a mask-gated override/additive layer
         * looping another clip from the source glTF, built from a per-joint checkbox list
         * against the loaded skeleton; a live weight slider per layer, no recompile; remove a
         * layer), and a two-bone IK section (joint pickers by name, pole-vector fields, a weight
         * slider — the target itself is dragged in the viewport, see `ViewportPanel::draw`'s
         * `ik_gizmo` parameter and `viewport_panel.cpp`'s IK gizmo block). Owns its transient
         * authoring state (the add-layer form's clip/mask/weight fields) in file-static state,
         * like the other panels; `AnimatedMeshPreview` itself owns the compiled/authored
         * controller.
         *
         * @param context Shared editor state; reads `context.animated_mesh_preview` and the
         *                panel visibility flag.
         */
        void draw_animator_preview_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine

#endif
