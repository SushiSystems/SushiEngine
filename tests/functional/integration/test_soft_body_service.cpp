/**************************************************************************/
/* test_soft_body_service.cpp                                             */
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

// P6-G3: a soft body from a cooked asset, through the world, to the renderer's
// seam — the whole path §8.6 draws, driven by the real simulation rather than by
// a harness standing in for it. A cook runs first, because a soft body cannot be
// built from numbers the way a cloth grid can and a test that faked the asset
// would be testing a path production does not have.
//
// The claim that matters is §8.6's third invariant: a deformation is visible in
// the render mesh in the *same* tick, with no lag. That is not a promise about
// ordering — it is a claim that there is nothing to fall out of step with,
// because the extract derives the render mesh from the live particles rather
// than copying them into a second place. The last two cases are what makes the
// difference observable: the extracted vertices must equal what the service
// reports at that instant, and they must have *changed* between ticks. Either
// one alone passes trivially — a frozen body satisfies equality, and a body read
// one tick late satisfies change.

#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/soft_body_cooker.hpp>
#include <SushiEngine/sim/simulation.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;
using namespace SushiEngine::Physics::Cooking;

namespace
{
    /** @brief A unit box as a closed, outward-wound triangle mesh. */
    Geometry::TriangleMesh box_mesh(float hx, float hy, float hz)
    {
        Geometry::TriangleMesh mesh;
        const float corners[8][3] = {{-hx, -hy, -hz}, {hx, -hy, -hz}, {hx, hy, -hz},
                                     {-hx, hy, -hz},  {-hx, -hy, hz}, {hx, -hy, hz},
                                     {hx, hy, hz},    {-hx, hy, hz}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        const std::uint32_t faces[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                            {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                            {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief Cooks the box once; the result is shared by every case here. */
    const std::vector<std::byte>& cooked_box()
    {
        static const std::vector<std::byte> bytes = []
        {
            SoftBodyCooker cooker;
            CookingParameters parameters;
            parameters.fidelity = 0.0f;
            parameters.voxel_resolution = 6;
            parameters.target_tetrahedron_count = DERIVE_FROM_FIDELITY;
            parameters.simulation_level_count = 1;
            parameters.distance_field_resolution = 8;
            parameters.surface_conforming_passes = 0;
            parameters.accuracy_lattice_order = 2;
            parameters.cook_soft_body = true;
            parameters.cook_collision = false;

            const Geometry::TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
            std::vector<std::byte> out;
            cooker.cook(box.view(), parameters, nullptr, nullptr, out);
            return out;
        }();
        return bytes;
    }

    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    /** @brief The extracted surface for @p id, or nullptr when it was not extracted. */
    const DeformableInstance* find_surface(const RenderScene& scene, EntityId id)
    {
        for (const DeformableInstance& surface : scene.deformable_instances)
            if (surface.id == id)
                return &surface;
        return nullptr;
    }
} // namespace

TEST(Integration_SoftBodyService, ARefusedAssetDoesNotBecomeAnEntity)
{
    // The asymmetry with create_cloth, asserted rather than commented: a cloth with
    // default parameters is a sheet, while a soft body with no cook is an entity
    // that can never become one. Failing at creation turns a cook failure into an
    // error where it happened instead of a mystery at play time.
    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const std::size_t before = world.entities().size();
    EXPECT_EQ(world.create_soft_body("Broken", std::vector<std::byte>{}), NULL_ENTITY);
    EXPECT_EQ(world.entities().size(), before) << "a refused create must leave nothing behind";
}

TEST(Integration_SoftBodyService, ACookedAssetBecomesADrawableSurface)
{
    ASSERT_FALSE(cooked_box().empty()) << "the cook this whole file depends on did not produce one";

    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_soft_body("Blob", cooked_box());
    ASSERT_NE(id, NULL_ENTITY);
    EXPECT_TRUE(world.has_soft_body(id));

    simulation->tick(simulation->fixed_dt_seconds());

    const DeformableInstance* surface = find_surface(simulation->render_scene(), id);
    ASSERT_NE(surface, nullptr) << "the body reached no render channel at all";
    EXPECT_GT(surface->vertex_count, 0u);
    EXPECT_GT(surface->index_count, 0u);
    EXPECT_EQ(surface->index_count % 3u, 0u) << "a triangle list has a multiple of three indices";

    // The renderer applies first_vertex as a draw-time vertex offset, so every index
    // must address this surface's own range and no other's. An index past the end
    // would read a neighbouring body's vertices — geometry rather than a crash, which
    // is the failure mode worth an assertion.
    const RenderScene& scene = simulation->render_scene();
    for (std::uint32_t i = 0; i < surface->index_count; ++i)
    {
        const std::uint32_t index = scene.deformable_indices[surface->first_index + i];
        EXPECT_LT(index, surface->vertex_count) << "index " << i << " left its own surface";
    }
    EXPECT_LE(std::size_t(surface->first_vertex) + surface->vertex_count,
              scene.deformable_vertices.size());
}

TEST(Integration_SoftBodyService, TwoBodiesGetDisjointSlicesOfTheSharedArrays)
{
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId first = world.create_soft_body("A", cooked_box());
    const EntityId second = world.create_soft_body("B", cooked_box());
    ASSERT_NE(first, NULL_ENTITY);
    ASSERT_NE(second, NULL_ENTITY);

    simulation->tick(simulation->fixed_dt_seconds());

    const RenderScene& scene = simulation->render_scene();
    const DeformableInstance* a = find_surface(scene, first);
    const DeformableInstance* b = find_surface(scene, second);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);

    const bool disjoint = a->first_vertex + a->vertex_count <= b->first_vertex ||
                          b->first_vertex + b->vertex_count <= a->first_vertex;
    EXPECT_TRUE(disjoint) << "two bodies were packed into overlapping vertex ranges";
    EXPECT_LE(std::size_t(b->first_index) + b->index_count, scene.deformable_indices.size());
}

TEST(Integration_SoftBodyService, TheRenderMeshIsTheSimulatedSurfaceAndNotACopyOfIt)
{
    // §8.6 invariant 3, first half. The extract reads the live particles, so what the
    // renderer receives and what a query reports are the same numbers at the same
    // instant — not two copies that a scheduling change could put a tick apart.
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_soft_body("Blob", cooked_box());
    ASSERT_NE(id, NULL_ENTITY);
    for (int tick = 0; tick < 8; ++tick)
        simulation->tick(simulation->fixed_dt_seconds());

    std::vector<Vector3> positions;
    std::vector<std::uint32_t> indices;
    ASSERT_TRUE(world.soft_body_surface(id, positions, indices));

    const RenderScene& scene = simulation->render_scene();
    const DeformableInstance* surface = find_surface(scene, id);
    ASSERT_NE(surface, nullptr);
    ASSERT_EQ(std::size_t(surface->vertex_count), positions.size());
    ASSERT_EQ(std::size_t(surface->index_count), indices.size());

    for (std::size_t v = 0; v < positions.size(); ++v)
    {
        const Vector3& extracted = scene.deformable_vertices[surface->first_vertex + v];
        // Exact, not near. Anything else here would be a copy that had been through
        // an arithmetic step, and an arithmetic step is a place a lag can hide.
        EXPECT_EQ(double(extracted.x), double(positions[v].x)) << "vertex " << v;
        EXPECT_EQ(double(extracted.y), double(positions[v].y)) << "vertex " << v;
        EXPECT_EQ(double(extracted.z), double(positions[v].z)) << "vertex " << v;
    }
    for (std::size_t i = 0; i < indices.size(); ++i)
        EXPECT_EQ(scene.deformable_indices[surface->first_index + i], indices[i]);
}

TEST(Integration_SoftBodyService, ADeformationReachesTheRenderMeshInTheSameTick)
{
    // §8.6 invariant 3, second half — and the half that makes the first one mean
    // something. Equality between the extract and the query is satisfied trivially by
    // a body that never moves, so the same body is also required to have moved.
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_soft_body("Blob", cooked_box());
    ASSERT_NE(id, NULL_ENTITY);

    simulation->tick(simulation->fixed_dt_seconds());
    const DeformableInstance* first_surface = find_surface(simulation->render_scene(), id);
    ASSERT_NE(first_surface, nullptr);
    const std::vector<Vector3> before(
        simulation->render_scene().deformable_vertices.begin() + first_surface->first_vertex,
        simulation->render_scene().deformable_vertices.begin() + first_surface->first_vertex +
            first_surface->vertex_count);

    for (int tick = 0; tick < 20; ++tick)
        simulation->tick(simulation->fixed_dt_seconds());

    std::vector<Vector3> live;
    std::vector<std::uint32_t> live_indices;
    ASSERT_TRUE(world.soft_body_surface(id, live, live_indices));

    const RenderScene& scene = simulation->render_scene();
    const DeformableInstance* surface = find_surface(scene, id);
    ASSERT_NE(surface, nullptr);
    ASSERT_EQ(std::size_t(surface->vertex_count), before.size());

    double largest_move = 0.0;
    for (std::size_t v = 0; v < before.size(); ++v)
    {
        const Vector3& now = scene.deformable_vertices[surface->first_vertex + v];
        const double moved = double(length(now - before[v]));
        if (moved > largest_move)
            largest_move = moved;
        // Still exactly the live state, twenty ticks of falling later.
        EXPECT_EQ(double(now.x), double(live[v].x)) << "vertex " << v;
        EXPECT_EQ(double(now.y), double(live[v].y)) << "vertex " << v;
        EXPECT_EQ(double(now.z), double(live[v].z)) << "vertex " << v;
    }
    EXPECT_GT(largest_move, 1e-6)
        << "nothing moved, so the equality above proves nothing about lag";
}

TEST(Integration_SoftBodyService, DetachingTheBodyRemovesItFromTheRenderChannel)
{
    ASSERT_FALSE(cooked_box().empty());

    const auto simulation = create_simulation();
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_soft_body("Blob", cooked_box());
    ASSERT_NE(id, NULL_ENTITY);
    simulation->tick(simulation->fixed_dt_seconds());
    ASSERT_NE(find_surface(simulation->render_scene(), id), nullptr);

    world.set_has_soft_body(id, false);
    simulation->tick(simulation->fixed_dt_seconds());

    EXPECT_FALSE(world.has_soft_body(id));
    EXPECT_EQ(find_surface(simulation->render_scene(), id), nullptr)
        << "a detached body kept drawing";

    std::vector<Vector3> positions;
    std::vector<std::uint32_t> indices;
    EXPECT_FALSE(world.soft_body_surface(id, positions, indices));
    EXPECT_TRUE(positions.empty());
}
