/**************************************************************************/
/* physics_sample_scene.cpp                                               */
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

/**
 * @file physics_sample_scene.cpp
 * @brief Writes the physics demonstration scene to a `.sushiscene` file.
 *
 * A scene is a *file*. Shipping the demonstration as a builder behind a menu item made it
 * the one scene in the engine that could not be opened, edited, saved or diffed like the
 * thing it was demonstrating — so the menu item is gone and this is what replaces it.
 *
 * Generated rather than hand-written, and that is the point rather than a convenience: the
 * file comes out of `build_physics_sample_scene`, which is the same function the
 * integration suite steps and asserts on. A hand-authored JSON blob would be a second
 * definition of the scene, free to drift from the one that is tested, and the drift would
 * show up as a demo that quietly stopped demonstrating something.
 *
 * Usage:
 *
 *     physics_sample_scene <path/to/physics_sample.sushiscene>
 *
 * with no argument writing `physics_sample.sushiscene` beside the working directory.
 */

#include <cstdio>
#include <string>

#include <SushiEngine/sim/simulation.hpp>

#include "scene/physics_sample_scene.hpp"
#include "serialization/scene_serializer.hpp"

int main(int argc, char** argv)
{
    const std::string path =
        argc > 1 ? std::string(argv[1]) : std::string("physics_sample.sushiscene");

    const auto simulation = SushiEngine::Simulation::create_simulation();
    if (simulation == nullptr)
    {
        std::printf("physics_sample_scene: could not create a simulation\n");
        return 1;
    }

    SushiEngine::Simulation::IWorldEditor& world = simulation->world();
    SushiEngine::Editor::build_physics_sample_scene(world);

    // Written before any tick. The file is the *authored* scene — where an author put
    // things — and a scene saved after a second of simulation would ship a stack already
    // settled and a ball already on the floor, which is a scene nobody authored.
    if (!SushiEngine::Editor::save_scene(world, path, nullptr))
    {
        std::printf("physics_sample_scene: could not write '%s'\n", path.c_str());
        return 1;
    }

    std::printf("physics_sample_scene: wrote '%s' (%zu entities)\n", path.c_str(),
                world.entities().size());
    return 0;
}
