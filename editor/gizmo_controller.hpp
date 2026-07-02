/**************************************************************************/
/* gizmo_controller.hpp                                                   */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#ifndef SUSHIENGINE_EDITOR_GIZMO_CONTROLLER_HPP
#define SUSHIENGINE_EDITOR_GIZMO_CONTROLLER_HPP

#include <imgui.h>

#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace sushi::editor
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

    /**
     * @brief Optional grid snapping applied to a gizmo drag, from Preferences.
     *
     * When enabled, a drag's accumulated delta is quantised: translation to a world
     * step, rotation to a degree step, scale to a step. Off by default so free drags
     * are pixel-smooth.
     */
    struct GizmoSnap
    {
        bool enabled = false;          /**< Whether any snapping is applied. */
        float translate = 0.25f;       /**< World-unit step for translation. */
        float rotate_degrees = 15.0f;  /**< Degree step for rotation. */
        float scale = 0.25f;           /**< Step for scale. */
    };

    /**
     * @brief The interactive transform gizmo for a viewport, one responsibility.
     *
     * Draws the handle set for the current @ref GizmoMode at a selection's transform
     * and turns a left-mouse drag into an edit of that transform, projecting through
     * the panel's camera. It owns only the drag state captured at grab time so the
     * pixel-to-world mapping stays stable while the object moves; the panel owns the
     * transform and writes the result back through the world editor. Split out of the
     * panel so a new handle kind is a change here, not in the viewport.
     */
    class GizmoController
    {
        public:
            /** @brief The outcome of one frame of manipulation. */
            struct Result
            {
                bool modified = false;       /**< The transform was edited this frame. */
                bool consumed_click = false; /**< A handle took the click (suppress picking). */
            };

            /**
             * @brief Draws the gizmo and applies a drag to @p transform.
             *
             * @param mode          The active handle set (translate/rotate/scale).
             * @param space         Local or World axis frame (Translate/Rotate only).
             * @param transform     The selection's transform, edited in place on drag.
             * @param camera_view   The panel camera's view and projection this frame.
             * @param image_origin  Top-left of the rendered image in screen pixels.
             * @param width         Image width in pixels.
             * @param height        Image height in pixels.
             * @param hovered       Whether the image is hovered (gates grabbing).
             * @param snap          Optional snapping applied to the drag.
             * @return Whether the transform changed and whether the click was consumed.
             */
            Result manipulate(GizmoMode mode, GizmoSpace space,
                              SushiEngine::sim::EntityTransform& transform,
                              const SushiEngine::render::CameraView& camera_view,
                              const ImVec2& image_origin, float width, float height,
                              bool hovered, const GizmoSnap& snap);

        private:
            // Active-drag state. axis_ is the grabbed axis (0=X,1=Y,2=Z, 3=uniform for
            // scale, -1=none); the rest is captured at grab time.
            int axis_ = -1;
            GizmoMode mode_ = GizmoMode::Translate;
            GizmoSpace space_ = GizmoSpace::World;
            ImVec2 start_mouse_{};
            ImVec2 axis_screen_{};
            SushiEngine::Vec3 axis_world_{};
            SushiEngine::Vec3 start_plane_vector_{};
            float world_per_pixel_ = 0.0f;
            SushiEngine::sim::EntityTransform start_transform_{};
    };
} // namespace sushi::editor

#endif
