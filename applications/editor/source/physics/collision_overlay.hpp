/**************************************************************************/
/* collision_overlay.hpp                                                  */
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
 * @file collision_overlay.hpp
 * @brief The cooked collider, drawn over the scene.
 *
 * §14: *"draws the actual collision geometry as an overlay so 'the collider is not the mesh'
 * is visible."* A number in an inspector saying the collider is three centimetres fatter is
 * useful; seeing *where* is what stops somebody spending an afternoon on an invisible wall.
 *
 * Takes line segments rather than an asset, because turning an asset into segments is a
 * cook-time-shaped job — the hull faces have to be rebuilt from the stored point set — and
 * doing it per frame would be paying that cost at frame rate. @ref CookBakeState rebuilds
 * them when the selection changes and hands the result here.
 */

#include <cstddef>
#include <vector>

#include <imgui.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws @p segments over the viewport image.
         *
         * @param segments    Six floats per segment (two endpoints), in the asset's own frame.
         * @param world       Where the asset sits; identity draws it at the origin.
         * @param camera_view The frame's camera.
         * @param image_origin Top-left of the viewport image, in screen pixels.
         * @param width       Image width in pixels.
         * @param height      Image height in pixels.
         * @param draw_list   The list to draw into; a null pointer draws nothing.
         * @return How many segments were drawn; fewer than were passed when some fell behind
         *         the camera.
         */
        std::size_t draw_collision_overlay(const std::vector<float>& segments, const Matrix4& world,
                                           const SushiEngine::Render::CameraView& camera_view,
                                           const ImVec2& image_origin, float width, float height,
                                           ImDrawList* draw_list);
    } // namespace Editor
} // namespace SushiEngine
