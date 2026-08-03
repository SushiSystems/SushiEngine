/**************************************************************************/
/* test_mesh_collision.cpp                                                */
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

// Unit_MeshCollision: static triangle-mesh geometry — the cooked hierarchy
// (physics/geometry/mesh_bvh.hpp) and the contacts against it
// (physics/collision/mesh_manifold.hpp).
//
// Two things are being checked and they are quite different in character.
//
// The hierarchy is a data structure, so it is checked against the definition it
// implements: a query must return exactly the triangles whose bounds meet the
// box, no more and no fewer, and it must do so having looked at far fewer than
// all of them — otherwise it is an expensive way to write a loop.
//
// The contacts are checked against the bug they exist to prevent. A crate
// sliding over a tessellated floor must not trip on the seams between triangles:
// every contact it makes must have the floor's normal, never an edge's. That is
// the ghost-collision test, and it is the reason the cooker computes adjacency
// at all.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/mesh_manifold.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI_HALF = 1.57079632679489661923;
    const Quaternion IDENTITY{0.0, 0.0, 0.0, 1.0};

    /** @brief A flat grid of `cells x cells` quads on the y = 0 plane, each split in two. */
    struct Grid
    {
        std::vector<Vector3> vertices;
        std::vector<std::uint32_t> indices;
        std::uint32_t triangle_count = 0;
    };

    Grid make_grid(std::uint32_t cells, Scalar extent)
    {
        Grid grid;
        const std::uint32_t stride = cells + 1;
        for (std::uint32_t z = 0; z <= cells; ++z)
            for (std::uint32_t x = 0; x <= cells; ++x)
            {
                const Scalar fx = -extent + 2.0 * extent * static_cast<Scalar>(x) /
                                                static_cast<Scalar>(cells);
                const Scalar fz = -extent + 2.0 * extent * static_cast<Scalar>(z) /
                                                static_cast<Scalar>(cells);
                grid.vertices.push_back(Vector3{fx, 0.0, fz});
            }
        for (std::uint32_t z = 0; z < cells; ++z)
            for (std::uint32_t x = 0; x < cells; ++x)
            {
                const std::uint32_t v00 = z * stride + x;
                const std::uint32_t v10 = v00 + 1;
                const std::uint32_t v01 = v00 + stride;
                const std::uint32_t v11 = v01 + 1;
                // Wound so the face normal is +Y (right-handed).
                grid.indices.push_back(v00);
                grid.indices.push_back(v01);
                grid.indices.push_back(v10);
                grid.indices.push_back(v10);
                grid.indices.push_back(v01);
                grid.indices.push_back(v11);
            }
        grid.triangle_count = static_cast<std::uint32_t>(grid.indices.size() / 3);
        return grid;
    }

    /** @brief Brute-force answer: every triangle whose bounds meet the box. */
    std::vector<std::uint32_t> brute_force_query(const Grid& grid, const AABB<Scalar>& box)
    {
        std::vector<std::uint32_t> hits;
        for (std::uint32_t t = 0; t < grid.triangle_count; ++t)
            if (aabb_overlap(local_triangle_bounds(grid.vertices.data(), grid.indices.data(), t),
                             box))
                hits.push_back(t);
        return hits;
    }
} // namespace

// The hierarchy must reproduce the brute-force answer exactly. Anything it adds
// is wasted narrowphase work; anything it drops is a body falling through the
// floor.
TEST(Unit_MeshCollision, HierarchyQueryMatchesBruteForce)
{
    const Grid grid = make_grid(16, 8.0);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);
    const TriangleMeshView<Scalar> mesh = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    const AABB<Scalar> boxes[] = {
        AABB<Scalar>{Vector3{-0.4, -1.0, -0.4}, Vector3{0.4, 1.0, 0.4}},
        AABB<Scalar>{Vector3{-8.5, -1.0, -8.5}, Vector3{8.5, 1.0, 8.5}},
        AABB<Scalar>{Vector3{3.1, -0.1, -2.2}, Vector3{3.9, 0.1, -1.4}},
        AABB<Scalar>{Vector3{20.0, -1.0, 20.0}, Vector3{21.0, 1.0, 21.0}}};

    for (const AABB<Scalar>& box : boxes)
    {
        std::vector<std::uint32_t> found;
        query_mesh_bvh<Scalar>(mesh, box, [&](std::uint32_t t) { found.push_back(t); });
        std::sort(found.begin(), found.end());

        const std::vector<std::uint32_t> expected = brute_force_query(grid, box);
        EXPECT_EQ(found, expected);
    }
}

// A hierarchy that visits everything is a loop with extra steps. A small query
// against a 512-triangle grid must touch a small number of them.
TEST(Unit_MeshCollision, HierarchyPrunesMostOfTheMesh)
{
    const Grid grid = make_grid(16, 8.0);
    ASSERT_EQ(grid.triangle_count, 512u);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);
    const TriangleMeshView<Scalar> mesh = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    std::size_t visited = 0;
    query_mesh_bvh<Scalar>(mesh, AABB<Scalar>{Vector3{-0.2, -0.5, -0.2}, Vector3{0.2, 0.5, 0.2}},
                           [&](std::uint32_t) { ++visited; });
    EXPECT_LT(visited, 32u);
    EXPECT_GT(visited, 0u);
}

// Adjacency is what the internal-edge correction reads, so it has to be right:
// in a closed grid the interior edges are all paired and the perimeter is not.
TEST(Unit_MeshCollision, AdjacencyPairsInteriorEdgesAndLeavesTheBoundaryOpen)
{
    const Grid grid = make_grid(4, 2.0);
    const std::vector<std::uint32_t> adjacency =
        build_triangle_adjacency(grid.indices.data(), grid.triangle_count);
    ASSERT_EQ(adjacency.size(), 3u * grid.triangle_count);

    std::size_t paired = 0;
    std::size_t open = 0;
    for (std::uint32_t t = 0; t < grid.triangle_count; ++t)
        for (std::uint32_t e = 0; e < 3; ++e)
        {
            const std::uint32_t neighbour = adjacency[3u * t + e];
            if (neighbour == no_adjacent_triangle)
            {
                ++open;
                continue;
            }
            ++paired;
            EXPECT_LT(neighbour, grid.triangle_count);
            EXPECT_NE(neighbour, t);
            // The pairing is symmetric: the neighbour names this triangle back.
            bool mutual = false;
            for (std::uint32_t k = 0; k < 3; ++k)
                if (adjacency[3u * neighbour + k] == t)
                    mutual = true;
            EXPECT_TRUE(mutual) << "triangle " << t << " edge " << e;
        }

    // A 4x4 grid of split quads: 32 triangles, 96 half-edges. The open ones are
    // exactly the 16 perimeter segments.
    EXPECT_EQ(open, 16u);
    EXPECT_EQ(paired, 96u - 16u);
}

// A box resting on the mesh floor is the baseline: it finds the surface, the
// normal points out of it, and the depth is what the placement says.
TEST(Unit_MeshCollision, BoxRestingOnTheMeshFindsTheSurface)
{
    const Grid grid = make_grid(8, 4.0);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);
    const TriangleMeshView<Scalar> mesh = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    const OrientedBox<Scalar> box{Vector3{0.13, 0.49, 0.07}, Vector3{0.5, 0.5, 0.5}, IDENTITY};

    std::size_t manifolds = 0;
    std::size_t points = 0;
    generate_convex_mesh_manifolds<Scalar>(
        box, mesh, box.center, box.orientation, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            ++manifolds;
            points += manifold.point_count;
            // The normal runs box -> surface, so it points down; resolving lifts
            // the box back out along +Y.
            EXPECT_NEAR(manifold.normal.y, -1.0, 1e-6);
            for (std::size_t i = 0; i < manifold.point_count; ++i)
                EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-6);
        });

    EXPECT_GT(manifolds, 0u);
    EXPECT_GT(points, 0u);
}

// The test this file exists for. A crate dragged across the floor crosses many
// interior edges; at no point may any contact normal be anything but the floor's.
// Without the adjacency correction the seams produce sideways normals and the
// crate trips on lines that are not there.
TEST(Unit_MeshCollision, SlidingAcrossSeamsNeverProducesAnEdgeNormal)
{
    const Grid grid = make_grid(8, 4.0);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);
    const TriangleMeshView<Scalar> mesh = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    std::size_t contacts = 0;
    Scalar worst = 1.0;
    // Walk right across the grid in small steps, so every seam is crossed and each
    // one is sampled at several offsets rather than only dead centre.
    for (int step = 0; step < 240; ++step)
    {
        const Scalar x = -3.0 + 6.0 * static_cast<Scalar>(step) / 239.0;
        const OrientedBox<Scalar> box{Vector3{x, 0.495, 0.31}, Vector3{0.4, 0.5, 0.4}, IDENTITY};
        generate_convex_mesh_manifolds<Scalar>(
            box, mesh, box.center, box.orientation, 0.0, 1e-3,
            [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
            {
                ++contacts;
                worst = std::min(worst, static_cast<Scalar>(-manifold.normal.y));
            });
    }

    EXPECT_GT(contacts, 200u);
    // Every single contact was the floor's normal, to within floating point.
    EXPECT_GT(worst, 1.0 - 1e-6);
}

// The same for a sphere, which touches at one point and so is the case most
// likely to land exactly on a seam.
TEST(Unit_MeshCollision, SphereRollingAcrossSeamsKeepsTheSurfaceNormal)
{
    const Grid grid = make_grid(8, 4.0);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);
    const TriangleMeshView<Scalar> mesh = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    std::size_t contacts = 0;
    Scalar worst = 1.0;
    for (int step = 0; step < 300; ++step)
    {
        const Scalar x = -3.0 + 6.0 * static_cast<Scalar>(step) / 299.0;
        const SphereCollider<Scalar> ball{Vector3{x, 0.245, x * 0.37}, 0.25};
        generate_convex_mesh_manifolds<Scalar>(
            ball, mesh, ball.center, IDENTITY, 0.0, 1e-3,
            [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
            {
                ++contacts;
                worst = std::min(worst, static_cast<Scalar>(-manifold.normal.y));
            });
    }

    EXPECT_GT(contacts, 250u);
    EXPECT_GT(worst, 1.0 - 1e-6);
}

// The correction must not swallow real geometry. A shape caught on the mesh's
// open boundary — where there is no neighbouring triangle — is genuinely resting
// on an edge, and that edge's normal is the truth.
TEST(Unit_MeshCollision, BoundaryEdgesKeepTheirRealNormal)
{
    // A single triangle: every edge is a boundary.
    const std::vector<Vector3> vertices = {Vector3{-1.0, 0.0, -1.0}, Vector3{-1.0, 0.0, 1.0},
                                           Vector3{1.0, 0.0, -1.0}};
    const std::vector<std::uint32_t> indices = {0, 1, 2};
    const CookedTriangleMesh<Scalar> cooked =
        build_mesh_bvh<Scalar>(vertices.data(), indices.data(), 1);
    const TriangleMeshView<Scalar> mesh =
        make_mesh_view<Scalar>(cooked, vertices.data(), indices.data(), 1);

    // A sphere beside the hypotenuse (the line x + z = 0), in the triangle's own
    // plane, so it touches that edge and nothing else. The edge is a boundary —
    // there is no second triangle — so the correction must leave it alone.
    const SphereCollider<Scalar> ball{Vector3{0.3, 0.0, 0.3}, 0.5};
    std::size_t contacts = 0;
    generate_convex_mesh_manifolds<Scalar>(
        ball, mesh, ball.center, IDENTITY, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            ++contacts;
            // Not the face normal: the sphere is beside the triangle, not above it,
            // so the contact is genuinely on the edge and points sideways.
            EXPECT_LT(std::abs(manifold.normal.y), 0.9);
        });
    EXPECT_EQ(contacts, 1u);
}

// A shape lying across a ridge touches two surfaces with two different normals,
// and it must get two manifolds rather than one averaged one — averaging them is
// how a body sinks into a valley.
TEST(Unit_MeshCollision, RidgeProducesTwoManifoldsWithTwoNormals)
{
    // Two quads meeting at x = 0 in a shallow roof.
    const std::vector<Vector3> vertices = {
        Vector3{-1.0, 0.0, -1.0}, Vector3{-1.0, 0.0, 1.0}, Vector3{0.0, 0.5, -1.0},
        Vector3{0.0, 0.5, 1.0},   Vector3{1.0, 0.0, -1.0}, Vector3{1.0, 0.0, 1.0}};
    const std::vector<std::uint32_t> indices = {0, 1, 2, 2, 1, 3, 2, 3, 4, 4, 3, 5};
    const CookedTriangleMesh<Scalar> cooked =
        build_mesh_bvh<Scalar>(vertices.data(), indices.data(), 4);
    const TriangleMeshView<Scalar> mesh =
        make_mesh_view<Scalar>(cooked, vertices.data(), indices.data(), 4);

    // A wide flat box straddling the ridge, low enough to touch both slopes.
    const OrientedBox<Scalar> plank{Vector3{0.0, 0.56, 0.0}, Vector3{0.9, 0.1, 0.5}, IDENTITY};

    std::vector<Vector3> normals;
    generate_convex_mesh_manifolds<Scalar>(
        plank, mesh, plank.center, plank.orientation, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        { normals.push_back(manifold.normal); });

    ASSERT_GE(normals.size(), 2u);
    // At least two genuinely different normals came back — one per slope.
    Scalar most_different = 1.0;
    for (std::size_t i = 0; i < normals.size(); ++i)
        for (std::size_t j = i + 1; j < normals.size(); ++j)
            most_different = std::min(most_different,
                                      static_cast<Scalar>(dot(normals[i], normals[j])));
    EXPECT_LT(most_different, 0.99);
}

// A placed mesh — rotated and translated — must collide where it actually is.
// The query box is transformed into mesh space rather than the mesh into world
// space, so this is where that transform gets checked.
TEST(Unit_MeshCollision, PlacedMeshCollidesWhereItIs)
{
    const Grid grid = make_grid(4, 2.0);
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(
        grid.vertices.data(), grid.indices.data(), grid.triangle_count);

    // Stand the floor on edge and move it: a wall at x = 3, facing -X.
    const Quaternion upright = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI_HALF);
    const TriangleMeshView<Scalar> wall = make_mesh_view<Scalar>(
        cooked, grid.vertices.data(), grid.indices.data(), grid.triangle_count,
        Vector3{3.0, 0.0, 0.0}, upright);

    // A sphere pressed into the wall from the -X side.
    const SphereCollider<Scalar> ball{Vector3{2.71, 0.0, 0.0}, 0.3};
    std::size_t contacts = 0;
    generate_convex_mesh_manifolds<Scalar>(
        ball, wall, ball.center, IDENTITY, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            ++contacts;
            EXPECT_NEAR(std::abs(manifold.normal.x), 1.0, 1e-5);
            EXPECT_NEAR(manifold.points[0].separation, -0.01, 1e-5);
        });
    EXPECT_GT(contacts, 0u);

    // And nothing at all when it is nowhere near.
    std::size_t misses = 0;
    const SphereCollider<Scalar> distant{Vector3{-5.0, 0.0, 0.0}, 0.3};
    generate_convex_mesh_manifolds<Scalar>(
        distant, wall, distant.center, IDENTITY, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>&, std::uint32_t) { ++misses; });
    EXPECT_EQ(misses, 0u);
}

// An empty mesh is a thing an importer can produce, and it must be inert rather
// than a crash.
TEST(Unit_MeshCollision, EmptyMeshIsInert)
{
    const CookedTriangleMesh<Scalar> cooked = build_mesh_bvh<Scalar>(nullptr, nullptr, 0);
    EXPECT_TRUE(cooked.nodes.empty());
    TriangleMeshView<Scalar> mesh;
    mesh.nodes = cooked.nodes.data();
    mesh.triangle_count = 0;
    mesh.node_count = 0;

    std::size_t visits = 0;
    query_mesh_bvh<Scalar>(mesh, AABB<Scalar>{Vector3{-1.0, -1.0, -1.0}, Vector3{1.0, 1.0, 1.0}},
                           [&](std::uint32_t) { ++visits; });
    EXPECT_EQ(visits, 0u);
}
