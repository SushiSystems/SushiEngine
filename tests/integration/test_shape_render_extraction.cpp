/**************************************************************************/
/* test_shape_render_extraction.cpp                                       */
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

// A Renderer entity whose ShapeParameters names an imported mesh must extract into
// RenderScene::instances carrying that same Render::MeshId, alongside whatever `kind`
// still says — the render passes are the ones that ignore `kind` when `mesh` is set
// (Render::MeshInstance's own documented rule), not this extraction step. This is the
// one place that claim is checked against the entity that produces it.

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

TEST(Integration_ShapeRenderExtraction, AnImportedMeshSurvivesExtraction)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Prop");
    ASSERT_NE(id, NULL_ENTITY);

    ShapeParameters shape = world.shape_parameters(id);
    shape.mesh_path = "models/car.gltf";
    shape.mesh = Render::MeshId(7);
    world.set_shape_parameters(id, shape);

    simulation->tick(simulation->fixed_dt_seconds());

    const RenderInstance* instance = find_instance(simulation->render_scene(), id);
    ASSERT_NE(instance, nullptr) << "the entity reached no render channel at all";
    EXPECT_EQ(instance->mesh, Render::MeshId(7));
}

TEST(Integration_ShapeRenderExtraction, ABoxWithNoImportedMeshExtractsAsInvalid)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Box");
    ASSERT_NE(id, NULL_ENTITY);
    simulation->tick(simulation->fixed_dt_seconds());

    const RenderInstance* instance = find_instance(simulation->render_scene(), id);
    ASSERT_NE(instance, nullptr);
    EXPECT_EQ(instance->mesh, Render::INVALID_MESH)
        << "a Box Renderer with no imported mesh must not silently draw one";
}

TEST(Integration_ShapeRenderExtraction, HidingAParentHidesItsDescendants)
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

    // Unity's `activeInHierarchy`: hiding the root has to hide the model. Gating on the
    // part's own flag alone would leave every part of an imported model drawing after its
    // root was switched off, which is the whole model still on screen.
    world.set_visible(root, false);
    simulation->tick(simulation->fixed_dt_seconds());
    EXPECT_EQ(find_instance(simulation->render_scene(), part), nullptr)
        << "the child kept drawing after its parent was hidden";
}

TEST(Integration_ShapeRenderExtraction, ShowingAParentLeavesAnIndividuallyHiddenChildHidden)
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
    world.set_visible(hidden, false);

    // The reason this is a walk and not a write-through on `set_visible`: the child's own
    // flag is its own. Toggling the parent off and back on must not adopt its children.
    world.set_visible(root, false);
    world.set_visible(root, true);
    simulation->tick(simulation->fixed_dt_seconds());

    EXPECT_NE(find_instance(simulation->render_scene(), shown), nullptr);
    EXPECT_EQ(find_instance(simulation->render_scene(), hidden), nullptr)
        << "re-showing the parent silently re-showed a child the author had hidden";
    // And the flag itself is still `activeSelf`, untouched by either question above.
    EXPECT_TRUE(world.visible(root));
    EXPECT_FALSE(world.visible(hidden));
}
