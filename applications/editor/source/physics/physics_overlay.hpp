/**************************************************************************/
/* physics_overlay.hpp                                                    */
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
 * @file physics_overlay.hpp
 * @brief §14's physics debug draw, and the joint gizmo, over the scene.
 *
 * Contacts and their normals, island colouring, sleeping state, broadphase bounds, and
 * the selected entity's joint frame with its limit arc — each its own toggle, because
 * they answer different questions and a view that showed all of them at once would answer
 * none of them.
 *
 * One unit for all of it rather than one per category. They share a projection, a colour
 * convention and a culling rule, and the alternative — five small files each doing the
 * same three things — is how a near-plane rule ends up different in two of them and one
 * overlay draws a stripe across the viewport that the others do not. `viewport_projection.hpp`
 * exists so the projection itself has exactly one definition.
 *
 * Reads the world through `IWorldEditor` and writes nothing. The debug draw exists to say
 * what the simulation is doing; one that could reach back into it would sooner or later
 * be used to.
 */

#include <imgui.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include "../core/physics_overlay_settings.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the enabled physics debug categories over the viewport image.
         *
         * @param world        The world to read; nothing is written.
         * @param selected     The selected entity, whose joint the gizmo is drawn for.
         * @param settings     Which categories to draw.
         * @param camera_view  The frame's camera.
         * @param image_origin Top-left of the viewport image, in screen pixels.
         * @param width        Image width in pixels.
         * @param height       Image height in pixels.
         * @param draw_list    The list to draw into; a null pointer draws nothing.
         */
        void draw_physics_overlay(Simulation::IWorldEditor& world, Simulation::EntityId selected,
                                  const PhysicsOverlaySettings& settings,
                                  const Render::CameraView& camera_view,
                                  const ImVec2& image_origin, float width, float height,
                                  ImDrawList* draw_list);
    } // namespace Editor
} // namespace SushiEngine
