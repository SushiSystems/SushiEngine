/**************************************************************************/
/* animator_graph_panel.hpp                                              */
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

#ifndef SUSHIENGINE_EDITOR_ANIMATOR_GRAPH_PANEL_HPP
#define SUSHIENGINE_EDITOR_ANIMATOR_GRAPH_PANEL_HPP

#include <string>
#include <vector>

#include <imgui.h>

#include <SushiEngine/animation/animator_controller.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The Animator window's document: the controller graph and its layout.
         *
         * Held by the caller rather than as a static inside the panel. The layout (node
         * positions, pan, zoom) sits beside the states because the compiled `.sushictrl`
         * blob carries none of it — it is authoring information, and losing it on every
         * reload would make the graph unusable for anything past a handful of states.
         */
        struct GraphState
        {
            Animation::ControllerDescription controller;
            int layer = 0;                        /**< The layer being edited. */
            std::vector<ImVec2> positions;        /**< One per state in the current layer. */
            std::vector<std::string> clip_paths;  /**< Per-state `.sushianim` motion path. */
            int positions_layer = -1;             /**< Layer `positions` was laid out for. */
            ImVec2 pan{40.0f, 40.0f};             /**< Canvas scroll offset. */
            float zoom = 1.0f;                    /**< Canvas zoom, clamped. */
            ImVec2 entry_pos{20.0f, 20.0f};       /**< Entry node graph position. */
            ImVec2 exit_pos{560.0f, 20.0f};       /**< Exit node graph position. */
            int selected_state = -1;
            int selected_transition_state = -1;   /**< Source state, or -2 for Any-State. */
            int selected_transition = -1;
            bool linking = false;                 /**< A wire is being drawn. */
            bool link_from_drag = false;          /**< The wire started at an output nub. */
            int link_source = -1;                 /**< Source state index, or -2 for Any-State. */
            ImVec2 context_menu_pos{0.0f, 0.0f};  /**< Graph point of the last canvas right-click. */
            char new_state[64] = "New State";
            char new_param[64] = "param";
            char io_path[256] = "controller.json";
            std::string status;                   /**< Result of the last save, load or compile. */
            bool seeded = false;                  /**< Whether the starter graph has been built. */
            /** Target of `clip_paths`' out-of-range fallback, so a stray edit has somewhere to go. */
            std::string fallback_clip_path;
        };

        /**
         * @brief Draw the Animator window: a Mecanim-style state-machine graph editor.
         *
         * Authors a controller (`animation/animator_controller.hpp`'s `ControllerDescription`) as a
         * graph: states are draggable nodes, transitions are arrows, and a parameter panel edits
         * the typed parameters. Add/remove states and transitions, link a source to a destination,
         * pick the default (entry) state, and save/load the whole controller as JSON
         * (`animator_controller_json.hpp`) — the same document the runtime compiles to a
         * `.sushictrl` blob.
         *
         * @param context Shared editor state; read for the panel visibility flag.
         * @param graph   The controller being authored and its layout, owned by the caller.
         */
        void draw_animator_graph_panel(EditorContext& context, GraphState& graph);
    } // namespace Editor
} // namespace SushiEngine

#endif
