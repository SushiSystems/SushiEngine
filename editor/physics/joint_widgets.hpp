/**************************************************************************/
/* joint_widgets.hpp                                                      */
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
 * @file joint_widgets.hpp
 * @brief The one joint editor, shared by everything that authors a joint.
 *
 * A joint is authored in two places — on an entity, in the Inspector's Physics Joint
 * section, and against part indices, in the Assembly editor's joint list — and §10.1's
 * whole argument for a single `JointParams` is that those are the *same* joint. A second
 * copy of the widget list is how a new limit ends up editable in one of them and
 * invisible in the other, which is precisely the failure the shared value type exists to
 * prevent; the widgets have to be shared for the same reason the type is.
 *
 * What is *not* here is the endpoint picker. An entity joint names a partner entity and
 * an assembly joint names two part indices, and those are genuinely different questions
 * with different answers — which is the split `JointParams` already makes, honoured here
 * rather than papered over with a callback.
 */

#include <SushiEngine/simulation/joint_params.hpp>
#include <SushiEngine/simulation/simulation.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Draws one joint limit: whether it is enforced, and the range it allows.
         *
         * Written once for all three of a joint's limits rather than three times, because
         * they differ in exactly two things — the unit and what the range means — and both
         * are parameters. The enabled box gates the rest visually rather than hiding it: a
         * disabled limit whose range vanished would lose the numbers the author is about to
         * turn back on.
         *
         * @param context    Shared editor state; drives the undo bracket.
         * @param world      The world the undo snapshot is taken from.
         * @param label      The limit's display name, also its ImGui id scope.
         * @param limit      The limit to edit, in place.
         * @param degrees    Whether the bounds are angles, shown and typed in degrees.
         * @param tooltip    What this limit bounds, in the joint's own words.
         * @param upper_only Whether only the upper bound is read (the swing cone).
         * @return Whether anything changed this frame.
         */
        bool draw_joint_limit(EditorContext& context, Simulation::IWorldEditor& world,
                              const char* label, Simulation::JointLimitDesc& limit, bool degrees,
                              const char* tooltip, bool upper_only);

        /**
         * @brief Draws everything a joint holds, except which two bodies it holds.
         *
         * The kind, the anchors and axes, the structural give, the three limits and the
         * drive, and the two break thresholds. Which rows appear follows the kind, because
         * a ball joint has no primary axis and offering it a twist limit would be offering
         * a control that changes nothing.
         *
         * @param context Shared editor state; drives the undo bracket.
         * @param world   The world the undo snapshot is taken from.
         * @param params  The joint parameters, edited in place.
         * @return Whether anything changed this frame.
         */
        bool draw_joint_params(EditorContext& context, Simulation::IWorldEditor& world,
                               Simulation::JointParams& params);
    } // namespace Editor
} // namespace SushiEngine
