/**************************************************************************/
/* physics_overlay_settings.hpp                                           */
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
 * @file physics_overlay_settings.hpp
 * @brief Which §14 physics debug categories are on.
 *
 * Its own header, apart from the overlay that draws them, for one structural reason:
 * `EditorContext` holds these — the window that toggles them and the viewport that reads
 * them are two different windows, so neither can own them — and `EditorContext` must stay
 * free of ImGui. The serialization tests compile it into a target that has no ImGui at
 * all, and a settings struct is not a reason to change that.
 */

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Which categories of physics debug draw are on.
         *
         * All off by default, the joint gizmo included. The argument for defaulting it on —
         * that it is scoped to the selected entity and therefore costs nothing until one is
         * selected — is true and is not the point. A debug view that is on by default is a
         * debug view everybody turns off once and never sees again, and an editor whose
         * viewport draws things nobody asked for reads as broken. The same rule the Bake
         * window follows: the panel is the readout, not the trigger.
         */
        struct PhysicsOverlaySettings
        {
            /** @brief Contact points and their normals, from the live contact stream. */
            bool contacts = false;

            /** @brief Each body's broadphase bound, as a wire box. */
            bool bounds = false;

            /** @brief Every body tinted by its island, so a settled group reads as one thing. */
            bool islands = false;

            /** @brief Asleep bodies marked, which is what a stack that has settled looks like. */
            bool sleeping = false;

            /** @brief The selected entity's joint: its anchors, its axis, and its limit arc. */
            bool joints = false;

            /** @brief Whether any category is on, so a caller can skip the whole pass. */
            bool any() const noexcept
            {
                return contacts || bounds || islands || sleeping || joints;
            }
        };
    } // namespace Editor
} // namespace SushiEngine
