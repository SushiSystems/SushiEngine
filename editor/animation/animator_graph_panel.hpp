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

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draw the Animator window: a Mecanim-style state-machine graph editor.
         *
         * Authors a controller (`animation/animator_controller.hpp`'s `ControllerDesc`) as a
         * graph: states are draggable nodes, transitions are arrows, and a parameter panel edits
         * the typed parameters. Add/remove states and transitions, link a source to a destination,
         * pick the default (entry) state, and save/load the whole controller as JSON
         * (`animator_controller_json.hpp`) — the same document the runtime compiles to a
         * `.sushictrl` blob. The panel owns its document in file-static state, like the other
         * panels; node layout lives beside the states there (the blob carries no layout).
         *
         * @param context Shared editor state; read for the panel visibility flag.
         */
        void draw_animator_graph_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine

#endif
