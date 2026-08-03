/**************************************************************************/
/* vehicle_panel.hpp                                                      */
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
 * @file vehicle_panel.hpp
 * @brief The Vehicle window: authoring a `VehicleAsset`, with its consequences shown.
 *
 * §11's authoring surface. A vehicle is a `Physics::VehicleAsset` — corners, tyres, a
 * drivetrain and a body — and every number in it is one an author can reason about. What
 * an author *cannot* do in their head is the arithmetic that turns those numbers into
 * behaviour, so this panel does it beside the field: a spring rate is shown next to the
 * ride height it produces, a gear ratio next to the road speed it reaches at the limiter,
 * a tyre's friction next to the cornering force it makes at the corner's own load.
 *
 * **The derived column is the whole point.** A vehicle editor that only echoed back what
 * was typed would be a form; the numbers that catch mistakes are `m·g/k`, the speed in
 * sixth, and the load a corner actually carries. Those are the ones a wrong entry moves
 * visibly, and none of them is visible in the field it came from.
 *
 * ### What this panel does not do
 *
 * It edits a *document*, not a selected entity's component, because there is no `Vehicle`
 * component in the ECS yet — a vehicle reaches a scene through `VehicleInstanceT` against
 * a solver, and the authoring record has no owner in the entity world to hang from. When
 * that component exists this panel becomes an inspector over it and nothing else here
 * changes. It also has no viewport: a live vehicle preview needs its own render target
 * (never the Scene view), and that is a separate piece of work rather than a line in this
 * one.
 */

#include <vector>

#include <SushiEngine/physics/vehicle/vehicle_asset.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The vehicle being authored, and what the panel remembers about editing it.
         *
         * Held by the editor's frame loop rather than by the panel, like @ref
         * Authoring::CookBakeState: a panel is a function over state and owning state would make it
         * impossible to draw twice or to serialize what it holds.
         */
        struct VehicleAuthoringState
        {
            /** @brief The document under edit. */
            Physics::VehicleAsset asset;

            /** @brief Which corner the corner editor is showing. */
            int selected_corner = 0;

            /** @brief Which gear the ratio readout reports a road speed for. */
            int selected_gear = 2;

            /**
             * @brief Whether the asset has been given its default four corners yet.
             *
             * A default-constructed `VehicleAsset` has *no* corners, which is correct for
             * the type — a trailer is a vehicle — and unhelpful as a starting point for a
             * car. Filling them in on first draw rather than in the constructor keeps the
             * physics type free of an editor's opinion about what is normal.
             */
            bool seeded = false;

            /**
             * @brief The live vehicle's node positions, reused frame to frame.
             *
             * A member rather than a local, because the Scene tab reads four hundred
             * positions every frame it is open and a fresh vector each time would be four
             * hundred allocations a frame for a view that changes by millimetres.
             */
            std::vector<Vector3> node_positions;
        };

        /**
         * @brief Draws the Vehicle window: the asset, and the behaviour it implies.
         * @param context Editor state (the panel-open flag).
         * @param state   The vehicle under edit.
         */
        void draw_vehicle_panel(EditorContext& context, VehicleAuthoringState& state);
    } // namespace Editor
} // namespace SushiEngine
