/**************************************************************************/
/* hierarchy_panel.hpp                                                    */
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
 * @file hierarchy_panel.hpp
 * @brief The Hierarchy window: the live world's entities, as a tree.
 *
 * Lists the world directly rather than a model of it — the world is the single source
 * of truth, so a tree row is a view of an entity and nothing caches its name or
 * parent. Selection (click, Ctrl-toggle, Shift-range from an anchor), drag-reparenting
 * and reordering with an insertion line, inline rename, a name filter, and the shared
 * create/clipboard context menus.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Hierarchy window: select, create, reparent, rename, filter.
         *
         * @param context Editor state; updates the selection and mutates the world through
         *                the world editor, recording undo per gesture.
         */
        void draw_hierarchy_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
