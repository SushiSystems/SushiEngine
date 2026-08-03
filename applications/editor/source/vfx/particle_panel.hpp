/**************************************************************************/
/* particle_panel.hpp                                                     */
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
 * @file particle_panel.hpp
 * @brief The Particle System component section: authoring an effect in the Inspector.
 *
 * A whole authoring surface rather than a handful of fields — emitter shape and
 * rates, the curve and colour ramps, force fields, bursts, the render module and its
 * texture, plus the effect library and the timeline that scrubs the isolated preview.
 * It is the Inspector's largest single component section, which is why it is its own
 * translation unit instead of another few hundred lines inside the Inspector's.
 */

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws the Particle System section for one entity's emitter.
         *
         * Edits the entity's @ref SushiEngine::VFX::ParticleEffect in place, bracketing a
         * drag as one undo step and a discrete edit (a library load, a burst change) as
         * its own. The isolated Preview viewport is pointed at whatever is being edited
         * here, so the two never show different effects.
         *
         * @param context Editor state; owns the effect-edit transaction and the preview.
         * @param world The world holding the entity's emitter.
         * @param entity The entity whose particle system is being authored.
         */
        void draw_particle_system_component(EditorContext& context,
                                            Simulation::IWorldEditor& world,
                                            Simulation::EntityId entity);
    } // namespace Editor
} // namespace SushiEngine
