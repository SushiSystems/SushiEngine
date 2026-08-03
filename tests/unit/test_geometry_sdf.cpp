/**************************************************************************/
/* test_geometry_sdf.cpp                                                  */
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

// The neutral geometry module, tested without a renderer. That it can be tested at
// all is the point of keeping it neutral: a baker sitting behind Vulkan can only be
// observed through a GI trace.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/signed_distance_field.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Geometry;

namespace
{
    /** @brief A unit cube centred on the origin, as twelve triangles. */
    TriangleMesh unit_cube()
    {
        TriangleMesh mesh;
        const float h = 0.5f;
        const float corners[8][3] = {
            {-h, -h, -h}, {h, -h, -h}, {h, h, -h}, {-h, h, -h},
            {-h, -h, h},  {h, -h, h},  {h, h, h},  {-h, h, h}};
        for (const auto& corner : corners)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        // Wound outward, though the bake only needs consistency: the sign comes from
        // the nearest triangle's geometric normal, so a flipped face would read
        // inside-out exactly there.
        const std::uint32_t faces[12][3] = {
            {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4},
            {3, 7, 6}, {3, 6, 2}, {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};
        for (const auto& face : faces)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /** @brief The brick's distance at the voxel nearest @p point. */
    float sample(const SignedDistanceFieldBrick& brick, float x, float y, float z)
    {
        const int n = brick.resolution;
        const float point[3] = {x, y, z};
        int voxel[3];
        for (int axis = 0; axis < 3; ++axis)
        {
            const float span = brick.aabb_max[axis] - brick.aabb_min[axis];
            const float t = (point[axis] - brick.aabb_min[axis]) / span;
            int index = int(t * float(n));
            if (index < 0)
                index = 0;
            if (index >= n)
                index = n - 1;
            voxel[axis] = index;
        }
        return brick.distances[std::size_t(voxel[0]) +
                               std::size_t(n) * (std::size_t(voxel[1]) +
                                                 std::size_t(n) * std::size_t(voxel[2]))];
    }
}

TEST(Unit_GeometrySDF, ADegenerateMeshBakesNothing)
{
    // Reported as an empty brick rather than as a crash or a field of zeros: zero is
    // a legal distance (it means "on the surface"), so a caller has to be able to
    // tell "no field" from "everything is on the surface".
    TriangleMesh empty;
    const SignedDistanceFieldBrick brick = bake_signed_distance_field(empty.view(), 8);
    EXPECT_EQ(brick.resolution, 0);
    EXPECT_TRUE(brick.distances.empty());
}

TEST(Unit_GeometrySDF, ACubeReadsNegativeInsideAndPositiveOutside)
{
    const TriangleMesh cube = unit_cube();
    const SignedDistanceFieldBrick brick = bake_signed_distance_field(cube.view(), 24);

    ASSERT_EQ(brick.resolution, 24);
    ASSERT_EQ(brick.distances.size(), 24u * 24u * 24u);

    EXPECT_LT(sample(brick, 0.0f, 0.0f, 0.0f), 0.0f) << "the centre is inside";
    EXPECT_GT(sample(brick, 0.9f, 0.0f, 0.0f), 0.0f) << "well past the +x face is outside";
    EXPECT_GT(sample(brick, 0.0f, 0.9f, 0.0f), 0.0f);
    EXPECT_GT(sample(brick, 0.0f, 0.0f, 0.9f), 0.0f);

    // The centre of a unit cube is half an edge from the nearest face, and the voxel
    // sample is within one voxel of that.
    const float voxel = (brick.aabb_max[0] - brick.aabb_min[0]) / 24.0f;
    EXPECT_NEAR(sample(brick, 0.0f, 0.0f, 0.0f), -0.5f, voxel * 2.0f);
}

TEST(Unit_GeometrySDF, TheBoundsArePaddedSoTheSurfaceHasClearance)
{
    // Rays entering from outside must read positive distances before they reach the
    // zero isosurface; a brick tight to the mesh would start them on it.
    const TriangleMesh cube = unit_cube();
    const SignedDistanceFieldBrick brick = bake_signed_distance_field(cube.view(), 16);

    ASSERT_GT(brick.resolution, 0);
    EXPECT_LT(brick.aabb_min[0], -0.5f);
    EXPECT_GT(brick.aabb_max[0], 0.5f);
}

TEST(Unit_GeometrySDF, AStridedViewReadsTheSamePositions)
{
    // The reason TriangleMeshView carries a stride at all: the renderer's vertices
    // are 60 bytes with normals, tangents and two UV sets, and the baker walks them
    // in place rather than being handed a copy.
    struct FatVertex
    {
        float position[3];
        float padding[12];
    };

    const TriangleMesh cube = unit_cube();
    std::vector<FatVertex> fat(cube.vertex_count());
    for (std::size_t i = 0; i < fat.size(); ++i)
    {
        fat[i] = FatVertex{};
        fat[i].position[0] = cube.positions[i * 3 + 0];
        fat[i].position[1] = cube.positions[i * 3 + 1];
        fat[i].position[2] = cube.positions[i * 3 + 2];
        fat[i].padding[0] = 12345.0f; // must never be mistaken for a coordinate
    }

    TriangleMeshView strided;
    strided.positions = fat[0].position;
    strided.position_stride = sizeof(FatVertex);
    strided.vertex_count = fat.size();
    strided.indices = cube.indices.data();
    strided.index_count = cube.indices.size();

    const SignedDistanceFieldBrick packed = bake_signed_distance_field(cube.view(), 12);
    const SignedDistanceFieldBrick spread = bake_signed_distance_field(strided, 12);

    ASSERT_EQ(packed.distances.size(), spread.distances.size());
    for (std::size_t i = 0; i < packed.distances.size(); ++i)
        ASSERT_FLOAT_EQ(packed.distances[i], spread.distances[i]) << "voxel " << i;
}

TEST(Unit_GeometrySDF, BoundsCoverEveryVertex)
{
    const TriangleMesh cube = unit_cube();
    float minimum[3];
    float maximum[3];
    ASSERT_TRUE(compute_bounds(cube.view(), minimum, maximum));
    for (int axis = 0; axis < 3; ++axis)
    {
        EXPECT_FLOAT_EQ(minimum[axis], -0.5f);
        EXPECT_FLOAT_EQ(maximum[axis], 0.5f);
    }

    TriangleMesh empty;
    EXPECT_FALSE(compute_bounds(empty.view(), minimum, maximum));
}
