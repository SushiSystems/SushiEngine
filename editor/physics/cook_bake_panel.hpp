/**************************************************************************/
/* cook_bake_panel.hpp                                                    */
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
 * @file cook_bake_panel.hpp
 * @brief The Bake window: the fidelity dial, what it produced, and how wrong it is.
 *
 * §14's collider and soft-body inspectors, as one window rather than two, because they show
 * the same thing about the same asset: what the cook made and what it cost. Splitting them
 * would mean two panels reading one @ref CookBakeState and an artist checking two places to
 * find out whether a crate is solid.
 *
 * Widgets only. Every decision — which profile applies, whether Re-cook has to evict, when
 * the overlay's geometry is rebuilt — is in @ref CookBakeState, which links no UI and is
 * tested. This file is the part that cannot be tested, and it is kept small for exactly that
 * reason.
 */

#include "../core/editor_context.hpp"
#include "cook_bake_state.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Bake window and polls finished cooks.
         *
         * Polls whether or not the window is open, because a bake the artist started and then
         * closed the window on must still finish and still be there when it is reopened.
         *
         * @param context Editor state (the panel-open flag).
         * @param state   The bake model; polled every frame.
         */
        void draw_cook_bake_panel(EditorContext& context, CookBakeState& state);
    } // namespace Editor
} // namespace SushiEngine
