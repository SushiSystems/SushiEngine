/**************************************************************************/
/* render_settings_panels.hpp                                             */
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
 * @file render_settings_panels.hpp
 * @brief The three windows that author `RenderSettings`: how the frame gets drawn.
 *
 * Rendering (the quality tier, anti-aliasing, render scale, frame delivery), Post
 * Process (the display transform and lens stack), and GPU Culling (the GPU-driven
 * geometry path). One translation unit because they edit one object for one reason —
 * trading fidelity for frame time on this machine — and share the "Tier resolves to"
 * readout that shows what the tier actually gave them.
 *
 * These are per-user settings, not scene content: they persist to `preferences.json`,
 * which is why every one of them raises `preferences_dirty` rather than dirtying the
 * scene.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Rendering window: the quality tier and the frame's shape.
         *
         * The render quality tier (which scales shadows, clouds, material lobes and post
         * effects — and deliberately never the weather), anti-aliasing and its temporal
         * parameters, the render scale and its dynamic governor, variable-rate shading,
         * and frame delivery. The values reach both viewports each frame.
         *
         * @param context Editor state; reads and writes @ref EditorContext::render_settings.
         */
        void draw_rendering_panel(EditorContext& context);

        /**
         * @brief Draws the Post Process window: the display transform and lens stack.
         *
         * The single owner of exposure — both the EV chain the post passes read and the
         * environment's pre-tonemap scene multiplier, labelled as the scene content it is.
         * Then the tone curve, bloom, the colour grade, depth of field, motion blur, and
         * the lens effects, with a tier readout naming which of them the tier permits.
         *
         * @param context Editor state; writes the post block and the scene exposure.
         */
        void draw_post_process_panel(EditorContext& context);

        /**
         * @brief Draws the GPU Culling window: the GPU-driven geometry path's controls.
         *
         * The master enable, the frustum and occlusion toggles, the small-on-screen LOD
         * gate, the frustum freeze for debugging what the cull actually sees, and the live
         * drawn/tested counts read back from the scene view.
         *
         * @param context Editor state; writes the GPU-culling block, reads the cull counts.
         */
        void draw_gpu_culling_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
