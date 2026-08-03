/**************************************************************************/
/* physics_sample_scene.hpp                                               */
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
 * @file physics_sample_scene.hpp
 * @brief A scene that exercises every physics feature an author can reach.
 *
 * The deliverable of the exposure work is that what P0 to P7 built is usable without
 * writing C++, and a claim like that is worth exactly as much as the scene that
 * demonstrates it. This builds one: a stack that settles and falls asleep, two surfaces
 * that differ only in their material and slide differently because of it, a filter pair
 * that passes through each other, a hinged door that tears off its chassis, a pendulum,
 * and a cloth.
 *
 * Built through `IWorldEditor` alone — the same calls the Inspector makes — so anything it
 * can produce, an author can produce by hand, and anything it cannot is a genuine gap
 * rather than a shortcut it took. It is one undo step, and it replaces the scene rather
 * than adding to it, because half a demo mixed into somebody's work is worse than either.
 *
 * ### Not a menu item
 *
 * It was one, briefly, and is not any more: a scene is a *file*, and an engine that ships
 * its demonstration as a hard-coded builder behind a menu is an engine whose demonstration
 * cannot be opened, edited, saved or diffed like the thing it is demonstrating. The
 * destination is a `.sushiscene` in the project, and this builder is what writes it —
 * called from the test suite today, which is also what keeps it honest: a builder no test
 * runs is a builder that rots.
 */

#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief Replaces the world with the physics demonstration scene.
         *
         * Takes the world and nothing else. An `EditorContext` parameter — to record an
         * undo step and clear the selection — would drag ImGui in behind it and make a
         * *scene builder* impossible to use from anything that is not the editor shell,
         * including the generator that writes the file and the suite that tests it.
         * Recording undo and moving a selection are the caller's business; building a world
         * is this function's, and that is the whole of it.
         *
         * @param world The world to build in; everything already in it is destroyed.
         */
        void build_physics_sample_scene(Simulation::IWorldEditor& world);
    } // namespace Editor
} // namespace SushiEngine
