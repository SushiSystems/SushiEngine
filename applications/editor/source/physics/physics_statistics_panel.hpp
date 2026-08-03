/**************************************************************************/
/* physics_statistics_panel.hpp                                           */
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
 * @file physics_statistics_panel.hpp
 * @brief The Physics window: what the last step contained, and what it cost.
 *
 * Its own unit rather than another function in `editor_panels.cpp`, following the
 * audio and animation panels: a panel that reads one subsystem's snapshot has no
 * reason to share a translation unit with every other panel in the editor.
 *
 * It reads `EditorContext::physics_statistics`, a per-frame copy, and writes
 * nothing. That is deliberate — this is a readout, and a readout that could reach
 * back into the simulation would sooner or later be used to.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Physics window: counts, budgets, and the graph's health.
         * @param context Editor state (the panel-open flag and the frame's snapshot).
         */
        void draw_physics_statistics_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
