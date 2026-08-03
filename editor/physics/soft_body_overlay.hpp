/**************************************************************************/
/* soft_body_overlay.hpp                                                  */
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
 * @file soft_body_overlay.hpp
 * @brief The simulated interior of a soft body, drawn over the scene (P6-G5).
 *
 * A soft body draws as a shaded surface, and the surface is the one part of it that
 * cannot show what the simulation is doing. Two elements deep inside a beam can be
 * past yield while every triangle facing the camera looks exactly as it did at rest,
 * so the questions an author actually has — *is this lattice too coarse where it
 * bends, is that part about to break, is the dent I am looking at permanent* — are
 * all questions about the interior.
 *
 * Three views, one drawing path. The **wireframe** answers a resolution question: a
 * heat map over four elements is four coloured blobs, and knowing that is what stops
 * a re-cook being blamed on the material. The two **heat maps** answer the other two,
 * and what their colours mean lives in `soft_body_heat.hpp` — the reading is the part
 * with a right answer, so it is kept where a test can reach it.
 */

#include <cstddef>
#include <vector>

#include <imgui.h>

#include <SushiEngine/authoring/soft_body_heat.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws a soft body's tetrahedra over the viewport image.
         *
         * Every element contributes its six edges. Shared edges are drawn once per
         * element that owns them rather than deduplicated, which is deliberate for a
         * heat map: an edge between a calm element and a hot one belongs to both, and
         * picking one of the two colours for it would hide the boundary that matters.
         *
         * @param positions    World-space particle positions (`soft_body_surface`'s first output).
         * @param elements     The body's tetrahedra and their readouts.
         * @param material     The body's constitutive parameters, for the colour scale.
         * @param view         Which view to draw; @ref SoftBodyDebugView::Off draws nothing.
         * @param camera_view  The frame's camera.
         * @param image_origin Top-left of the viewport image, in screen pixels.
         * @param width        Image width in pixels.
         * @param height       Image height in pixels.
         * @param draw_list    The list to draw into; a null pointer draws nothing.
         * @return How many edges were drawn; fewer than `6 * elements.size()` when some
         *         fell behind the camera.
         */
        std::size_t draw_soft_body_overlay(
            const std::vector<Vector3>& positions,
            const std::vector<Simulation::SoftBodyElementSample>& elements,
            const Physics::SoftBodyMaterialT<Scalar>& material, SoftBodyDebugView view,
            const Render::CameraView& camera_view, const ImVec2& image_origin, float width,
            float height, ImDrawList* draw_list);
    } // namespace Editor
} // namespace SushiEngine
