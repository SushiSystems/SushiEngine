/**************************************************************************/
/* assembly_panel.hpp                                                     */
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
 * @file assembly_panel.hpp
 * @brief The Assembly window: parts, the joints between them, and what they may touch.
 *
 * §14's assembly-editor bullet, and P3's one outstanding item. The joint library, §10.4's
 * load recovery and breakable joints were all built in P3 and none of them could be
 * *authored*, so this is the surface that turns a `PhysicsAssembly` from a type only C++
 * could construct into a document.
 *
 * ### Instancing produces entities, not a hidden object
 *
 * "Instantiate into Scene" creates one ordinary entity per part — a Transform, a Collider
 * and a Rigid Body — and one Physics Joint component per assembly joint. That is the whole
 * design decision in this panel, and it is worth stating why: an assembly instanced as
 * some opaque scene-graph node would be a second kind of thing the Hierarchy, the
 * Inspector, undo, save and the debug draw would each need a case for. Instanced as
 * entities it is *already* all of those, and the parts stay editable afterwards — which is
 * what an author does with a ragdoll five minutes after placing one.
 *
 * The cost is stated too: the instance forgets it was an assembly. Re-instancing does not
 * update the copies already in the scene, because there is nothing linking them back. A
 * prefab relationship is a real feature and it is a scene-system feature, not a physics
 * one.
 *
 * ### The live load readout is over the scene, not over the asset
 *
 * §14 asks for "a live joint-load readout while playing". An asset has no load — only an
 * instance does — so the readout lists the *scene's* joints and what each is carrying.
 * Since instancing produces entities, that list contains this assembly's joints along with
 * every other, which is the correct set for the question "what is about to tear off".
 */

#include <SushiEngine/simulation/physics_assembly.hpp>

#include "../core/editor_context.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The assembly being authored, and what the panel remembers about editing it.
         *
         * Held by the editor's frame loop rather than by the panel, like
         * @ref VehicleAuthoringState: a panel is a function over state, and owning state
         * would make it impossible to draw twice or to serialize what it holds.
         */
        struct AssemblyAuthoringState
        {
            /** @brief The document under edit. */
            Simulation::PhysicsAssembly asset;

            /** @brief Which part the part editor is showing. */
            int selected_part = 0;

            /** @brief Which joint the joint editor is showing. */
            int selected_joint = 0;

            /**
             * @brief Where the next instance is placed, in world metres.
             *
             * On the state rather than derived from the camera, because an author placing
             * three ragdolls wants them somewhere they chose — and because a placement that
             * followed the camera would put two successive instances in different places
             * for no reason the author gave.
             */
            Vector3 instance_position{Vector3{0, 5, 0}};

            /**
             * @brief Whether the asset has been given its starting parts yet.
             *
             * A default-constructed `PhysicsAssembly` has no parts, which is correct for the
             * type and unhelpful as a starting point. Seeding on first draw rather than in
             * the constructor keeps the boundary type free of an editor's opinion about
             * what is normal.
             */
            bool seeded = false;
        };

        /**
         * @brief Draws the Assembly window: the parts, the joints, and the filter matrix.
         * @param context Editor state (the panel-open flag, the world, the undo history).
         * @param state   The assembly under edit.
         */
        void draw_assembly_panel(EditorContext& context, AssemblyAuthoringState& state);
    } // namespace Editor
} // namespace SushiEngine
