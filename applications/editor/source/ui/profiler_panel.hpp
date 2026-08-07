/**************************************************************************/
/* profiler_panel.hpp                                                     */
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

#include "../core/editor_context.hpp"
#include "../core/panel_state.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Profiler window: frame history, CPU and GPU breakdowns,
         *        renderer counters, memory and system utilization.
         *
         * Reads only the per-frame copies on the context, like every panel. Sections
         * without a wired producer render their values as "n/a" rather than zeros, per
         * `docs/design/profiling_system.md` §8.
         *
         * @param context The shared editor state the panels read.
         * @param state   The panel's own between-frame scratch (pause).
         */
        void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state);
    } // namespace Editor
} // namespace SushiEngine
