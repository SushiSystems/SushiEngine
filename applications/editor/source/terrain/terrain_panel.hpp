/**************************************************************************/
/* terrain_panel.hpp                                                      */
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

#ifndef SUSHIENGINE_EDITOR_TERRAIN_PANEL_HPP
#define SUSHIENGINE_EDITOR_TERRAIN_PANEL_HPP

/**
 * @file terrain_panel.hpp
 * @brief The Terrain window: a body's ordered edits, and what the last frame made of them.
 *
 * The authoring surface for @ref SushiEngine::Terrain::LayerStack — the records that
 * decide what the ground *is*, as opposed to what was measured (design
 * `docs/design/solar_system_overhaul.md` §6). It is the layer stack's only editor, and it
 * edits it through @ref SushiEngine::Terrain::ITerrainAuthoring rather than reaching into
 * a stack directly, because an edit that does not also invalidate the compiled ground is
 * an edit that appears to do nothing.
 *
 * The selection readout sits in this window rather than in Statistics for the same
 * reason: node counts, the depth reached and an exhausted budget are read *in response
 * to an edit*, and they need the same handle on the body that the edits do.
 *
 * Widgets only. Every rule about what an edit means — which orders are free, what a
 * changed footprint invalidates, how much recompilation a frame can afford — belongs to
 * the implementation behind the interface, where it is reachable without a UI.
 */

#include <SushiEngine/terrain/layer_stack.hpp>
#include <SushiEngine/terrain/terrain_authoring.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The Terrain window's scratch: the layer being composed, before it exists.
         *
         * Only the *new* layer needs holding. Every layer already in the stack is edited
         * in place — the stack is the model, and a second copy of it here is a second copy
         * to keep in step. A layer being composed has nowhere else to live, since it is
         * not in the stack until the author commits it.
         *
         * Owned by the host (like the animation and bake panels' state) rather than by a
         * static inside the panel, so it is inspectable and so a second Terrain window
         * could never silently share one draft with the first.
         */
        struct TerrainPanelState
        {
            /**
             * @brief The layer being composed, in the same form the stack stores.
             *
             * One representation, so the draft's fields and an existing layer's are drawn
             * by the same code and cannot drift apart. Its footprint radii are seeded on
             * the frame the panel first sees a body, since a radius in radians has no
             * sensible constant default across bodies of different sizes.
             */
            SushiEngine::Terrain::TerrainLayer draft{};

            /**
             * @brief The body @ref draft was seeded against; a change re-seeds it.
             *
             * Negative means "never seeded", which is also the state a host starts in, so
             * the first body to arrive is seeded for without a second flag saying so.
             */
            int draft_body = -1;
        };

        /**
         * @brief Draws the Terrain window: the layer stack, the draft, and the selection.
         *
         * Lists every layer in composition order with its operation, footprint and profile
         * editable in place, offers move/remove per layer and a compose-and-add form
         * underneath, and reports what the last frame's node selection did with the result.
         * Applies each edit immediately through @p terrain, which is what marks the ground
         * it changed for recompilation.
         *
         * @param context Editor state; read for the panel-open flag and written for the log.
         * @param state   The draft layer, held across frames by the host.
         * @param terrain The body's authoring surface, or nullptr when the host's viewport
         *                draws no terrain — in which case the window says so rather than
         *                offering edits nothing would compose.
         */
        void draw_terrain_panel(EditorContext& context, TerrainPanelState& state,
                                SushiEngine::Terrain::ITerrainAuthoring* terrain);
    } // namespace Editor
} // namespace SushiEngine

#endif
