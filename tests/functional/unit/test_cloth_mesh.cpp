/**************************************************************************/
/* test_cloth_mesh.cpp                                                   */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

// Unit_ClothMesh: the cloth grid triangulation (render/cloth_mesh.hpp
// triangulate_cloth_grid) in isolation — a pure function of a row-major point
// grid, so no runtime or world is needed. Covers vertex/index counts for a
// regular grid, that a flat grid's normals all point the same way, and that
// degenerate (rows < 2 or cols < 2) input triangulates to nothing.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/render/cloth_mesh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Render;

namespace
{
    std::vector<Vector3> flat_grid(std::uint32_t rows, std::uint32_t cols, Scalar spacing)
    {
        std::vector<Vector3> points;
        points.reserve(static_cast<std::size_t>(rows) * cols);
        for (std::uint32_t r = 0; r < rows; ++r)
            for (std::uint32_t c = 0; c < cols; ++c)
                points.push_back(Vector3{Scalar(c) * spacing, Scalar(0), Scalar(r) * spacing});
        return points;
    }
} // namespace

TEST(Unit_ClothMesh, ThreeByThreeGridCounts)
{
    const std::vector<Vector3> points = flat_grid(3, 3, Scalar(0.5));
    std::vector<ClothVertex> vertices;
    std::vector<std::uint32_t> indices;
    triangulate_cloth_grid(points.data(), 3, 3, vertices, indices);

    EXPECT_EQ(vertices.size(), 9u);
    // 2x2 quads, 2 triangles per quad, 3 indices per triangle.
    EXPECT_EQ(indices.size(), 2u * 2u * 2u * 3u);
}

TEST(Unit_ClothMesh, FlatGridNormalsAgree)
{
    const std::vector<Vector3> points = flat_grid(4, 5, Scalar(0.5));
    std::vector<ClothVertex> vertices;
    std::vector<std::uint32_t> indices;
    triangulate_cloth_grid(points.data(), 4, 5, vertices, indices);

    ASSERT_FALSE(vertices.empty());
    const Scalar reference_y = vertices.front().normal.y;
    ASSERT_GT(std::abs(double(reference_y)), 0.5);
    for (const ClothVertex& vertex : vertices)
    {
        EXPECT_NEAR(double(vertex.normal.x), 0.0, 1e-4);
        EXPECT_NEAR(double(vertex.normal.z), 0.0, 1e-4);
        EXPECT_GT(double(vertex.normal.y) * double(reference_y), 0.0);
        EXPECT_NEAR(double(vertex.normal.y), double(reference_y), 1e-4);
    }
}

TEST(Unit_ClothMesh, DegenerateGridProducesNothing)
{
    const std::vector<Vector3> single_row = flat_grid(1, 4, Scalar(0.5));
    std::vector<ClothVertex> vertices;
    std::vector<std::uint32_t> indices;
    triangulate_cloth_grid(single_row.data(), 1, 4, vertices, indices);
    EXPECT_TRUE(vertices.empty());
    EXPECT_TRUE(indices.empty());

    const std::vector<Vector3> single_col = flat_grid(4, 1, Scalar(0.5));
    triangulate_cloth_grid(single_col.data(), 4, 1, vertices, indices);
    EXPECT_TRUE(vertices.empty());
    EXPECT_TRUE(indices.empty());
}
