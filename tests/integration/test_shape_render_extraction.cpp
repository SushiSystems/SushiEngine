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

