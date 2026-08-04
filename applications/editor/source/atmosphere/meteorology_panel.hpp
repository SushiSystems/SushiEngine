/**************************************************************************/
/* meteorology_panel.hpp                                                  */
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
 * @file meteorology_panel.hpp
 * @brief The Meteorology window: the atmosphere's tier, its physics, and its clock.
 *
 * The regional nest's authoring surface and its readout in one place, which is what
 * the panel's name always promised: the atmosphere quality tier that resolves the
 * nest grid, the surface-energy / boundary-layer / microphysics constants, and the
 * mirror-based diagnostics — including the clock comparison, the readout that tells
 * you whether the sky is running ahead of the air it is supposed to be heating.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Meteorology window: the nest's tier, physics, and diagnostics.
         *
         * The nest runs on the GPU and is read back asynchronously
         * (`docs/design/atmosphere_system.md` §3.2), so every readout here comes from that
         * same mirror — the only honest view of what the simulation actually did.
         *
         * @param context Editor state; reads the mirror through @c assets and writes the
         *                environment's nest parameters through the world editor.
         */
        void draw_meteorology_panel(EditorContext& context);
    } // namespace Editor
} // namespace SushiEngine
