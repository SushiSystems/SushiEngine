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

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/profiling/frame_profiler.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The Profiler panel's between-frame state.
         *
         * Defined here rather than in `panel_state.hpp` because Pause has to freeze every
         * section together (CPU, GPU, Renderer, physics), which means holding copies typed
         * in `ViewportGPUStatistics` and `Physics::PhysicsStatistics` — both reached only
         * through `editor_context.hpp`. `panel_state.hpp` is included BY `editor_context.hpp`
         * (`EditorContext::panel_state`), so the reverse include would be circular; this
         * follows the same carve-out `panel_state.hpp` already documents for the animation
         * panels' larger, subsystem-typed state.
         */
        struct ProfilerPanelState
        {
            bool paused = false; /**< Freeze the displayed numbers while comparing. */

            /** Every held copy below is refreshed together from the context, once per
             *  frame, only while not paused — so all sections freeze on the same frame
             *  instead of some following the live context past the pause point. */
            SushiEngine::Profiling::FrameProfileSnapshot held_frame_profile;
            std::vector<ViewportGPUStatistics> held_gpu_statistics;
            std::vector<ViewportRenderStatistics> held_render_statistics;
            std::size_t held_resident_texture_bytes = 0;
            /** Whether the context had an asset library the frame this was held. */
            bool held_has_asset_library = false;
            SushiEngine::Physics::PhysicsStatistics held_physics_statistics;
            SystemMetricsSnapshot held_system_metrics;
        };

        /**
         * @brief Draws the Profiler window: frame history, CPU and GPU breakdowns,
         *        renderer counters, memory and system utilization.
         *
         * Reads only the per-frame copies on the context, like every panel. Sections
         * without a wired producer render their values as "n/a" rather than zeros, per
         * `docs/design/profiling_system.md` §8.
         *
         * @param context The shared editor state the panels read.
         * @param state   The panel's own between-frame scratch (pause and the held snapshot).
         */
        void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state);
    } // namespace Editor
} // namespace SushiEngine
