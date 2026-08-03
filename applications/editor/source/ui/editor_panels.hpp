/**************************************************************************/
/* editor_panels.hpp                                                      */
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

#ifndef SUSHIENGINE_EDITOR_EDITOR_PANELS_HPP
#define SUSHIENGINE_EDITOR_EDITOR_PANELS_HPP

#include <cstdint>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draw the top menu bar (File / Edit / Entity / Window).
         * @param context Shared editor state the menu acts on (world, documents). File >
         *                Exit sets @ref EditorContext::close_requested rather than exiting
         *                directly, so the caller's close-confirm modal gets a chance to run.
         */
        void draw_menu_bar(EditorContext& context);

        /**
         * @brief Draw the toolbar: a fixed strip under the menu bar, not a dockable window.
         *
         * Hosts Play/Pause/Step (which snapshot and restore the scene around a play
         * session), the transform-tool selector, the gizmo axis-frame toggle, and the
         * derived Overall Quality preset (never stored; writes every domain tier). Also
         * resolves the playback shortcuts (Ctrl+P play/stop, Ctrl+Shift+P pause) and the
         * W/E/R tool hotkeys, so every playback and tool entry point lives in one place.
         * Rendered with `BeginViewportSideBar` like the status bar — always present, no
         * close button, which is why it no longer has a @ref Authoring::PanelVisibility flag.
         *
         * @param context Shared editor state; updates playback and gizmo state.
         */
        void draw_toolbar(EditorContext& context);

        /**
         * @brief Draw the Console panel showing accumulated log lines with a clear button.
         * @param context Shared editor state; reads and clears the console buffer.
         */
        void draw_console_panel(EditorContext& context);

        /**
         * @brief Draw the Statistics panel: frame time, FPS, entity and document counts,
         * and each visible viewport's per-pass GPU times.
         * @param context Shared editor state, read for the counts and the GPU statistics
         *                the main loop copied out of the viewports this frame.
         */
        void draw_statistics_panel(EditorContext& context);

        /**
         * @brief Draw the bottom status bar (selection, playback state, entity count).
         * @param context Shared editor state, read for the status summary.
         */
        void draw_status_bar(EditorContext& context);

        /**
         * @brief Apply a theme to ImGui's active style.
         *
         * Kept as a free function so both startup (from the loaded preferences) and the
         * Authoring::Preferences window can apply the same mapping without duplicating it.
         *
         * @param theme The theme to install.
         */
        void apply_theme(Authoring::EditorTheme theme);

        /**
         * @brief Build the default Unity-style dock layout, docking every editor window.
         *
         * Splits the dockspace into Hierarchy (left), the Scene/Game/Preview viewport
         * tabs (centre), Inspector with the settings panels stacked behind it (right),
         * and Project/Console plus the timeline-shaped tools (bottom). Docks all windows
         * — open or closed — so any window opened later lands in its home node instead
         * of floating. Applied when no persisted layout exists (user rearrangement
         * survives restarts) and again on Window ▸ Reset Layout, which tears the node
         * tree down and rebuilds this exact arrangement.
         *
         * @param dockspace_id The id of the root dockspace node to partition.
         */
        void build_default_layout(std::uint32_t dockspace_id);
    } // namespace Editor
} // namespace SushiEngine

#endif
