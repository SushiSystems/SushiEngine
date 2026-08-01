/**************************************************************************/
/* test_deformable_mesh.cpp                                               */
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

// The host half of P6-G2: the seam that replaced the rows-by-cols cloth grid with a
// vertex array and a triangle list, and the inverse map that lets a GPU shade one
// without atomics.
//
// The last case here is the one that carries weight. Production shading happens in
// deformable.comp, which no unit test can run — but the shader's *algorithm* is a
// gather driven entirely by the adjacency table, and a gather can be written on the
// host in a dozen lines. So the test walks the table the way the shader does and
// checks it lands on the same normals as the straightforward scatter. What that
// pins is the part that could silently be wrong: whether the table actually names
// every triangle touching a vertex and nothing else.

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/render/deformable_mesh.hpp>

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

    /** @brief A closed tetrahedron: four vertices, four outward-wound faces. */
    struct Tetrahedron
    {
        std::vector<Vector3> vertices{Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0},
                                      Vector3{0, 0, 1}};
        std::vector<std::uint32_t> indices{0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3};
    };

    DeformableMeshView view_of(const std::vector<Vector3>& vertices,
                               const std::vector<std::uint32_t>& indices)
    {
        DeformableMeshView view;
        view.vertices = vertices.data();
        view.vertex_count = static_cast<std::uint32_t>(vertices.size());
        view.indices = indices.data();
        view.index_count = static_cast<std::uint32_t>(indices.size());
        return view;
    }

    /**
     * @brief Shades a mesh the way deformable.comp does: one gather per vertex.
     *
     * Deliberately a transcription rather than a call into the header — the point is
     * to run the shader's control flow, not the reference's.
     */
    std::vector<Vector3> gather_normals(const DeformableMeshView& view,
                                        const VertexTriangleAdjacency& adjacency)
    {
        std::vector<Vector3> normals(view.vertex_count);
        for (std::uint32_t v = 0; v < view.vertex_count; ++v)
        {
            const std::uint32_t first = adjacency.range[std::size_t(v) * 2];
            const std::uint32_t count = adjacency.range[std::size_t(v) * 2 + 1];
            Vector3 sum{0, 0, 0};
            for (std::uint32_t k = 0; k < count; ++k)
            {
                const std::uint32_t t = adjacency.triangle[first + k];
                const Vector3& a = view.vertices[view.indices[t * 3 + 0]];
                const Vector3& b = view.vertices[view.indices[t * 3 + 1]];
                const Vector3& c = view.vertices[view.indices[t * 3 + 2]];
                sum = sum + cross(b - a, c - a);
            }
            const Scalar len = length(sum);
            if (len > Scalar(1e-12))
                sum = sum * (Scalar(1) / len);
            normals[v] = sum;
        }
        return normals;
    }
} // namespace

TEST(Unit_DeformableMesh, GridTriangulationKeepsItsOldCounts)
{
    std::vector<std::uint32_t> indices;
    build_grid_indices(3, 3, indices);
    // 2x2 quads, 2 triangles per quad, 3 indices per triangle — unchanged from the
    // grid path this replaced, because the diagonal and winding are unchanged.
    EXPECT_EQ(indices.size(), 2u * 2u * 2u * 3u);
    for (const std::uint32_t index : indices)
        EXPECT_LT(index, 9u);
}

TEST(Unit_DeformableMesh, DegenerateGridProducesNoTriangles)
{
    std::vector<std::uint32_t> indices;
    build_grid_indices(1, 4, indices);
    EXPECT_TRUE(indices.empty());
    build_grid_indices(4, 1, indices);
    EXPECT_TRUE(indices.empty());
}

TEST(Unit_DeformableMesh, FlatGridNormalsAgree)
{
    const std::vector<Vector3> points = flat_grid(4, 5, Scalar(0.5));
    std::vector<std::uint32_t> indices;
    build_grid_indices(4, 5, indices);

    std::vector<DeformableVertex> vertices;
    shade_deformable_mesh(view_of(points, indices), vertices);

    ASSERT_EQ(vertices.size(), points.size());
    const Scalar reference_y = vertices.front().normal.y;
    ASSERT_GT(std::abs(double(reference_y)), 0.5);
    for (const DeformableVertex& vertex : vertices)
    {
        EXPECT_NEAR(double(vertex.normal.x), 0.0, 1e-4);
        EXPECT_NEAR(double(vertex.normal.z), 0.0, 1e-4);
        EXPECT_NEAR(double(vertex.normal.y), double(reference_y), 1e-4);
    }
}

TEST(Unit_VertexTriangleAdjacency, NamesEveryTriangleTouchingAVertexAndNoOther)
{
    const Tetrahedron tet;
    VertexTriangleAdjacency adjacency;
    build_vertex_triangle_adjacency(tet.indices.data(),
                                    static_cast<std::uint32_t>(tet.indices.size()), 4, adjacency);

    ASSERT_EQ(adjacency.range.size(), 8u);
    // Every vertex of a tetrahedron is a corner of exactly three of its four faces,
    // and every face contributes three entries, so the table holds twelve.
    EXPECT_EQ(adjacency.triangle.size(), 12u);

    for (std::uint32_t v = 0; v < 4; ++v)
    {
        const std::uint32_t first = adjacency.range[std::size_t(v) * 2];
        const std::uint32_t count = adjacency.range[std::size_t(v) * 2 + 1];
        EXPECT_EQ(count, 3u) << "vertex " << v;
        for (std::uint32_t k = 0; k < count; ++k)
        {
            const std::uint32_t t = adjacency.triangle[first + k];
            const bool uses = tet.indices[t * 3 + 0] == v || tet.indices[t * 3 + 1] == v ||
                              tet.indices[t * 3 + 2] == v;
            EXPECT_TRUE(uses) << "vertex " << v << " was told about triangle " << t;
        }
    }
}

TEST(Unit_VertexTriangleAdjacency, RangesTileTheTableWithoutOverlapOrGap)
{
    const std::vector<Vector3> points = flat_grid(5, 6, Scalar(0.25));
    std::vector<std::uint32_t> indices;
    build_grid_indices(5, 6, indices);

    VertexTriangleAdjacency adjacency;
    build_vertex_triangle_adjacency(indices.data(), static_cast<std::uint32_t>(indices.size()),
                                    static_cast<std::uint32_t>(points.size()), adjacency);

    // Each entry of the triangle array must be claimed by exactly one vertex —
    // an overlap would double-count a face normal, a gap would drop one, and both
    // failures are invisible in a picture until the lighting is subtly wrong.
    std::vector<int> claims(adjacency.triangle.size(), 0);
    std::uint32_t expected_next = 0;
    for (std::size_t v = 0; v < points.size(); ++v)
    {
        const std::uint32_t first = adjacency.range[v * 2];
        const std::uint32_t count = adjacency.range[v * 2 + 1];
        EXPECT_EQ(first, expected_next) << "vertex " << v << " does not start where the last ended";
        expected_next = first + count;
        for (std::uint32_t k = 0; k < count; ++k)
            ++claims[first + k];
    }
    EXPECT_EQ(expected_next, adjacency.triangle.size());
    for (const int claimed : claims)
        EXPECT_EQ(claimed, 1);
}

TEST(Unit_VertexTriangleAdjacency, SurvivesAnIndexPastTheEndOfTheVertexArray)
{
    // Not defensive decoration: the scatter writes at a cursor derived from the index,
    // so an out-of-range index that got counted would write outside the table. The
    // header drops such indices, and this is where that is pinned.
    const std::vector<Vector3> points{Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0}};
    const std::vector<std::uint32_t> indices{0, 1, 2, 0, 1, 99};

    VertexTriangleAdjacency adjacency;
    build_vertex_triangle_adjacency(indices.data(), 6, 3, adjacency);

    ASSERT_EQ(adjacency.range.size(), 6u);
    EXPECT_EQ(adjacency.range[1], 2u) << "vertex 0 is in both triangles";
    EXPECT_EQ(adjacency.range[3], 2u) << "vertex 1 is in both triangles";
    EXPECT_EQ(adjacency.range[5], 1u) << "vertex 2 is only in the well-formed one";
    EXPECT_EQ(adjacency.triangle.size(), 5u) << "the out-of-range corner contributed nothing";
}

TEST(Unit_DeformableMesh, TheAdjacencyGatherLandsOnTheSameNormalsAsTheScatter)
{
    // What the GPU pass does, checked against what the reference does, on a surface
    // with no grid structure at all — the case the old grid formula could not express
    // and the reason the table exists.
    Tetrahedron tet;
    // Skewed so no two faces share an area and a mis-weighted sum cannot pass by
    // symmetry: with a regular tetrahedron every face normal has the same length, so
    // dropping or double-counting one still points roughly the right way.
    tet.vertices[1] = Vector3{Scalar(2.5), Scalar(0.1), Scalar(-0.3)};
    tet.vertices[3] = Vector3{Scalar(-0.2), Scalar(0.4), Scalar(0.9)};

    const DeformableMeshView view = view_of(tet.vertices, tet.indices);
    VertexTriangleAdjacency adjacency;
    build_vertex_triangle_adjacency(view.indices, view.index_count, view.vertex_count, adjacency);

    std::vector<DeformableVertex> reference;
    shade_deformable_mesh(view, reference);
    const std::vector<Vector3> gathered = gather_normals(view, adjacency);

    ASSERT_EQ(gathered.size(), reference.size());
    for (std::size_t v = 0; v < gathered.size(); ++v)
    {
        // Summed in a different order, so equal only to rounding — but the same terms,
        // so the tolerance is float-epsilon territory rather than a fudge factor.
        EXPECT_LT(double(length(gathered[v] - reference[v].normal)), 1e-12)
            << "vertex " << v << " shaded differently under the gather";
        EXPECT_NEAR(double(length(gathered[v])), 1.0, 1e-12);
    }
}

TEST(Unit_DeformableMesh, AVertexNoTriangleUsesKeepsAZeroNormal)
{
    // A fractured body leaves particles in the array that no surface triangle
    // references any more. They must not invent a direction, and they must not make
    // the pass reach past the end of the table looking for one.
    const std::vector<Vector3> points{Vector3{0, 0, 0}, Vector3{1, 0, 0}, Vector3{0, 1, 0},
                                      Vector3{5, 5, 5}};
    const std::vector<std::uint32_t> indices{0, 1, 2};

    std::vector<DeformableVertex> vertices;
    shade_deformable_mesh(view_of(points, indices), vertices);

    ASSERT_EQ(vertices.size(), 4u);
    EXPECT_GT(double(length(vertices[0].normal)), 0.5);
    EXPECT_EQ(double(length(vertices[3].normal)), 0.0);

    VertexTriangleAdjacency adjacency;
    build_vertex_triangle_adjacency(indices.data(), 3, 4, adjacency);
    EXPECT_EQ(adjacency.range[3 * 2 + 1], 0u);
    EXPECT_EQ(adjacency.triangle.size(), 3u);
}
