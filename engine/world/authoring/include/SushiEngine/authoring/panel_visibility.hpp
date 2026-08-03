/**************************************************************************/
/* panel_visibility.hpp                                                   */
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

#ifndef SUSHIENGINE_AUTHORING_PANEL_VISIBILITY_HPP
#define SUSHIENGINE_AUTHORING_PANEL_VISIBILITY_HPP

namespace SushiEngine
{
    namespace Authoring
    {
        /**
         * @brief Which editor windows are currently shown.
         *
         * Each flag backs one entry in the Window menu and one window's open state, so
         * closing a window (its title-bar X) and reopening it from the menu share the
         * same bit. Defaults to the core authoring set (viewports, Hierarchy, Inspector,
         * Project, Console, ...); the domain and diagnostic panels open on demand and
         * land in their dock home (see @ref build_default_layout). The whole struct is
         * persisted in the preferences, so the open set survives a restart.
         *
         * Its own header (rather than a section of the context) because the preferences
         * persist it and the context holds it — two owners of the *type* would otherwise
         * force one to include the other's whole world.
         */
        struct PanelVisibility
        {
            bool scene_view = true;
            bool game_view = true;
            bool hierarchy = true;
            bool inspector = true;
            bool project = true;
            bool text_editor = true;
            bool console = true;
            bool statistics = true;
            bool animation = false;
            bool animator_graph = false;
            bool animator_preview = false;
            bool environment = true;
            bool rendering = false;
            bool lighting = false;
            bool post_process = false;
            bool meteorology = false;
            bool gpu_culling = false;
            bool physics = false;

            /** @brief The Vehicle window: §11's authoring surface. */
            bool vehicle = false;

            /** @brief The Assembly window: §10.2's parts, joints and filter matrix (§14). */
            bool assembly = false;

            /**
             * @brief The Bake window: the fidelity dial and what each cook produced (§14).
             *
             * Off by default. An import cooks whether or not this is open — the panel is the
             * readout, not the trigger — so opening it is something an artist does when a
             * collider looks wrong, which is not most of the time.
             */
            bool bake = false;
            /**
             * @brief The Preview viewport: the one surface anything being authored is shown on.
             *
             * One screen, not one per subject: a previewed effect, a previewed character, and
             * whatever is previewed next are all "the thing being authored, in isolation", and that
             * is a property of the surface rather than of the subject. Nothing previewed belongs to
             * an entity, which is exactly why none of it belongs in the Scene view.
             */
            bool preview = false;
            bool audio_mixer = false;
            bool audio_profiler = false;
            bool audio_authoring = false;
            /** @brief The floating Preferences window (Edit ▸ Preferences...). */
            bool preferences = false;
            /** @brief The floating Input Manager window (Edit ▸ Input Manager...). */
            bool input_manager = false;
        };
    } // namespace Authoring
} // namespace SushiEngine

#endif
