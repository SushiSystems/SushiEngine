/**************************************************************************/
/* test_entity_lifecycle.cpp                                              */
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

// entity_lifecycle_system.md Phase 1: `enabled` (Unity's `activeSelf`/`activeInHierarchy`) is a
// real, hierarchical on/off switch, distinct from the render-only local `visible` flag. This
// file covers the flag's own semantics; render, physics and audio gating on it are covered in
// their own tests below and, for physics, in the entities-and-transforms cases further down.

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    const RenderInstance* find_instance(const RenderScene& scene, EntityId id)
    {
        for (const RenderInstance& instance : scene.instances)
            if (instance.id == id)
                return &instance;
        return nullptr;
    }
} // namespace

TEST(Integration_EntityLifecycle, EnabledDefaultsToTrueAndIsSelfOnly)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create("Widget");
    EXPECT_TRUE(world.enabled(id));
    EXPECT_TRUE(world.enabled_in_hierarchy(id));

    world.set_enabled(id, false);
    EXPECT_FALSE(world.enabled(id));
    EXPECT_FALSE(world.enabled_in_hierarchy(id));
}

TEST(Integration_EntityLifecycle, DisablingAnAncestorDisablesTheWholeSubtree)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId root = world.create("Root");
    const EntityId child = world.create("Child");
    world.set_parent(child, root);

    EXPECT_TRUE(world.enabled_in_hierarchy(child));
    world.set_enabled(root, false);
    EXPECT_FALSE(world.enabled_in_hierarchy(child))
        << "a disabled ancestor must disable everything under it";
    // The child's own flag is untouched -- only the hierarchy view changed.
    EXPECT_TRUE(world.enabled(child));
}

TEST(Integration_EntityLifecycle, ReEnablingAParentDoesNotAdoptAChildsOwnDisabledFlag)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId root = world.create("Root");
    const EntityId child = world.create("Child");
    world.set_parent(child, root);
    world.set_enabled(child, false);

    world.set_enabled(root, false);
    world.set_enabled(root, true);

    EXPECT_TRUE(world.enabled_in_hierarchy(root));
    EXPECT_FALSE(world.enabled_in_hierarchy(child))
        << "re-enabling the parent silently re-enabled a child the author had disabled";
}
