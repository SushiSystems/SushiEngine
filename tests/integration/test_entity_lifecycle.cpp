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

TEST(Integration_EntityLifecycle, DisablingAParentHidesItsRenderedDescendants)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    // The shape an imported model has: a bare pivot root over the part that actually draws.
    const EntityId root = world.create("Model");
    const EntityId part = world.create_box("Part");
    ASSERT_NE(root, NULL_ENTITY);
    ASSERT_NE(part, NULL_ENTITY);
    world.set_parent(part, root);
    simulation->tick(simulation->fixed_dt_seconds());
    ASSERT_NE(find_instance(simulation->render_scene(), part), nullptr);

    // Unity's `activeInHierarchy`: disabling the root has to hide the model. Gating on the
    // part's own flag alone would leave every part of an imported model drawing after its
    // root was switched off, which is the whole model still on screen.
    world.set_enabled(root, false);
    simulation->tick(simulation->fixed_dt_seconds());
    EXPECT_EQ(find_instance(simulation->render_scene(), part), nullptr)
        << "the child kept drawing after its parent was disabled";
}

TEST(Integration_EntityLifecycle, ReEnablingAParentLeavesAnIndividuallyDisabledChildDisabled)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId root = world.create("Model");
    const EntityId shown = world.create_box("Shown");
    const EntityId hidden = world.create_box("Hidden");
    world.set_parent(shown, root);
    world.set_parent(hidden, root);
    world.set_enabled(hidden, false);

    // The reason this is a walk and not a write-through on `set_enabled`: the child's own
    // flag is its own. Toggling the parent off and back on must not adopt its children.
    world.set_enabled(root, false);
    world.set_enabled(root, true);
    simulation->tick(simulation->fixed_dt_seconds());

    EXPECT_NE(find_instance(simulation->render_scene(), shown), nullptr);
    EXPECT_EQ(find_instance(simulation->render_scene(), hidden), nullptr)
        << "re-enabling the parent silently re-enabled a child the author had disabled";
    EXPECT_TRUE(world.enabled(root));
    EXPECT_FALSE(world.enabled(hidden));
}

TEST(Integration_EntityLifecycle, VisibleIsLocalOnlyAndDoesNotCascade)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId root = world.create("Model");
    const EntityId part = world.create_box("Part");
    world.set_parent(part, root);

    // `visible` is the render-only local flag (Unity's Renderer.enabled): turning it off on
    // the root must not touch the child's own render contribution, unlike `enabled`, which
    // does cascade (see DisablingAParentHidesItsRenderedDescendants above).
    world.set_visible(root, false);
    simulation->tick(simulation->fixed_dt_seconds());
    EXPECT_NE(find_instance(simulation->render_scene(), part), nullptr)
        << "a parent's local visible flag must not hide a child that has its own";
}

TEST(Integration_EntityLifecycle, DisabledHidesEvenWhenVisibleIsTrue)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Box");
    ASSERT_TRUE(world.visible(id));
    world.set_enabled(id, false);
    simulation->tick(simulation->fixed_dt_seconds());

    EXPECT_EQ(find_instance(simulation->render_scene(), id), nullptr)
        << "enabled=false must hide rendering even though visible is still true";
}

TEST(Integration_EntityLifecycle, DisablingAnEntityRemovesItsRigidBodyFromPhysics)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Box");
    EntityTransform transform = world.transform(id);
    transform.position = Vector3{0, 5, 0};
    world.set_transform(id, transform);
    PhysicsBodyParameters body;
    body.density = Scalar(1000);
    world.set_has_physics_body(id, true);
    world.set_physics_body_parameters(id, body);

    simulation->tick(simulation->fixed_dt_seconds());
    RigidDebugState state;
    ASSERT_TRUE(world.physics_body_debug(id, state))
        << "an enabled entity with a physics body must be tracked by the solver";

    world.set_enabled(id, false);
    simulation->tick(simulation->fixed_dt_seconds());
    EXPECT_FALSE(world.physics_body_debug(id, state))
        << "a disabled entity's rigid body must be removed from the solver";

    world.set_enabled(id, true);
    simulation->tick(simulation->fixed_dt_seconds());
    EXPECT_TRUE(world.physics_body_debug(id, state))
        << "re-enabling must re-admit the body to the solver";
}
