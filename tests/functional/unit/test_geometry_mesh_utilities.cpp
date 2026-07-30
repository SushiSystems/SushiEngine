/**************************************************************************/
/* test_geometry_mesh_utilities.cpp                                       */
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

// The cooking pipeline's first stage, against the corpus it exists for. Every mesh
// below is a specific way a real asset arrives broken — exploded by an exporter,
// wound both ways, doubled, holed — and the assertions are about what the report
// says as much as about what the repair does, because §8.3 stage 1 is explicit that
// a non-manifold input is a fact the artist gets told rather than a fault silently
// papered over.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Geometry;

namespace
{
    /** @brief The eight corners of a unit cube centred on the origin. */
    const float CUBE_CORNERS[8][3] = {{-0.5f, -0.5f, -0.5f}, {0.5f, -0.5f, -0.5f},
                                      {0.5f, 0.5f, -0.5f},   {-0.5f, 0.5f, -0.5f},
                                      {-0.5f, -0.5f, 0.5f},  {0.5f, -0.5f, 0.5f},
                                      {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f}};

    /** @brief Twelve triangles, wound so the normals point out of the cube. */
    const std::uint32_t CUBE_FACES[12][3] = {{0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
                                             {0, 1, 5}, {0, 5, 4}, {3, 7, 6}, {3, 6, 2},
                                             {0, 4, 7}, {0, 7, 3}, {1, 2, 6}, {1, 6, 5}};

    /** @brief A clean, welded, outward-wound unit cube: eight vertices, twelve faces. */
    TriangleMesh unit_cube()
    {
        TriangleMesh mesh;
        for (const auto& corner : CUBE_CORNERS)
        {
            mesh.positions.push_back(corner[0]);
            mesh.positions.push_back(corner[1]);
            mesh.positions.push_back(corner[2]);
        }
        for (const auto& face : CUBE_FACES)
        {
            mesh.indices.push_back(face[0]);
            mesh.indices.push_back(face[1]);
            mesh.indices.push_back(face[2]);
        }
        return mesh;
    }

    /**
     * @brief The same cube with every triangle carrying its own three vertices.
     *
     * What an exporter that writes one vertex per corner per face produces, which is
     * most of them: thirty-six vertices, not one shared edge, and therefore no
     * adjacency at all until something welds it.
     */
    TriangleMesh exploded_cube()
    {
        TriangleMesh mesh;
        std::uint32_t next = 0;
        for (const auto& face : CUBE_FACES)
        {
            for (int i = 0; i < 3; ++i)
            {
                const float* corner = CUBE_CORNERS[face[i]];
                mesh.positions.push_back(corner[0]);
                mesh.positions.push_back(corner[1]);
                mesh.positions.push_back(corner[2]);
                mesh.indices.push_back(next++);
            }
        }
        return mesh;
    }
} // namespace

TEST(Unit_GeometryMeshUtilities,MeasuresACleanCube)
{
    const TriangleMesh cube = unit_cube();
    const MeshTopologyReport report = analyze_mesh_topology(cube.view());

    EXPECT_EQ(report.vertex_count, 8u);
    EXPECT_EQ(report.triangle_count, 12u);
    EXPECT_EQ(report.out_of_range_triangles, 0u);
    EXPECT_EQ(report.degenerate_triangles, 0u);
    EXPECT_EQ(report.duplicate_triangles, 0u);
    EXPECT_EQ(report.unreferenced_vertices, 0u);
    EXPECT_EQ(report.boundary_edges, 0u);
    EXPECT_EQ(report.non_manifold_edges, 0u);
    EXPECT_EQ(report.inconsistent_edges, 0u);
    EXPECT_EQ(report.connected_components, 1u);
    EXPECT_TRUE(report.watertight());
    EXPECT_TRUE(report.manifold());
    EXPECT_TRUE(report.consistently_oriented());

    // Six unit faces, unit volume, and the sign says the normals point outward.
    EXPECT_NEAR(report.surface_area, 6.0f, 1e-5f);
    EXPECT_NEAR(report.signed_volume, 1.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,ReportsAHoleAsBoundaryEdges)
{
    TriangleMesh holed = unit_cube();
    // Drop the two triangles of one face: the four edges around the hole now have one
    // incident triangle each, which is exactly what "not watertight" means.
    holed.indices.resize(holed.indices.size() - 6);

    const MeshTopologyReport report = analyze_mesh_topology(holed.view());
    EXPECT_EQ(report.boundary_edges, 4u);
    EXPECT_EQ(report.non_manifold_edges, 0u);
    EXPECT_FALSE(report.watertight());
    EXPECT_TRUE(report.manifold());
    EXPECT_EQ(report.connected_components, 1u);
}

TEST(Unit_GeometryMeshUtilities,ReportsADoubledTriangleAsNonManifoldRatherThanHidingIt)
{
    TriangleMesh doubled = unit_cube();
    for (int i = 0; i < 3; ++i)
        doubled.indices.push_back(CUBE_FACES[0][i]);

    const MeshTopologyReport report = analyze_mesh_topology(doubled.view());
    EXPECT_EQ(report.duplicate_triangles, 1u);
    // The three edges of the doubled face carry three triangles each. An analysis that
    // dropped the second copy before counting would call this mesh manifold, which is
    // the report the artist must not be given.
    EXPECT_EQ(report.non_manifold_edges, 3u);
    EXPECT_FALSE(report.manifold());
    EXPECT_FALSE(report.watertight());
}

TEST(Unit_GeometryMeshUtilities,ReportsAFlippedFaceAsInconsistentEdges)
{
    TriangleMesh flipped = unit_cube();
    std::swap(flipped.indices[1], flipped.indices[2]);

    const MeshTopologyReport report = analyze_mesh_topology(flipped.view());
    // All three of the flipped triangle's edges are now walked the same way round as
    // the neighbour that shares them.
    EXPECT_EQ(report.inconsistent_edges, 3u);
    EXPECT_FALSE(report.consistently_oriented());
    // Still closed and still manifold: winding is a separate property from topology,
    // and conflating the two is how a mesh gets "repaired" by having a hole punched.
    EXPECT_TRUE(report.watertight());
}

TEST(Unit_GeometryMeshUtilities,CountsDegenerateAndOutOfRangeTrianglesSeparately)
{
    TriangleMesh dirty = unit_cube();
    // A repeated corner, a zero-area sliver of three collinear points, and a triangle
    // naming a vertex that does not exist.
    dirty.indices.push_back(0);
    dirty.indices.push_back(1);
    dirty.indices.push_back(1);
    dirty.indices.push_back(0);
    dirty.indices.push_back(1);
    dirty.indices.push_back(99);

    const MeshTopologyReport report = analyze_mesh_topology(dirty.view());
    EXPECT_EQ(report.degenerate_triangles, 1u);
    EXPECT_EQ(report.out_of_range_triangles, 1u);
    EXPECT_EQ(report.triangle_count, 14u);
    // Neither contributed an edge, so the clean cube underneath still reads clean.
    EXPECT_EQ(report.boundary_edges, 0u);
    EXPECT_EQ(report.non_manifold_edges, 0u);
    EXPECT_NEAR(report.surface_area, 6.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,ReportsSeparateShellsAsSeparateComponents)
{
    TriangleMesh two = unit_cube();
    const std::uint32_t offset = std::uint32_t(two.vertex_count());
    for (const auto& corner : CUBE_CORNERS)
    {
        two.positions.push_back(corner[0] + 10.0f);
        two.positions.push_back(corner[1]);
        two.positions.push_back(corner[2]);
    }
    for (const auto& face : CUBE_FACES)
    {
        two.indices.push_back(face[0] + offset);
        two.indices.push_back(face[1] + offset);
        two.indices.push_back(face[2] + offset);
    }

    const MeshTopologyReport report = analyze_mesh_topology(two.view());
    EXPECT_EQ(report.connected_components, 2u);
    EXPECT_TRUE(report.watertight());
    EXPECT_NEAR(report.signed_volume, 2.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,WeldingAnExplodedMeshRestoresItsAdjacency)
{
    const TriangleMesh exploded = exploded_cube();
    ASSERT_EQ(exploded.vertex_count(), 36u);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(exploded.view(), MeshRepairOptions{}, repaired);

    // Before: thirty-six distinct edges, every one of them a boundary, and each
    // triangle its own component. This is what "no adjacency at all" measures as.
    EXPECT_EQ(report.before.boundary_edges, 36u);
    EXPECT_EQ(report.before.connected_components, 12u);
    EXPECT_FALSE(report.before.watertight());

    EXPECT_EQ(report.welded_vertices, 28u);
    EXPECT_EQ(repaired.vertex_count(), 8u);
    EXPECT_EQ(repaired.triangle_count(), 12u);
    EXPECT_TRUE(report.after.watertight());
    EXPECT_EQ(report.after.connected_components, 1u);
    EXPECT_NEAR(report.after.signed_volume, 1.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,WeldingRespectsTheToleranceRatherThanACellBoundary)
{
    // Two vertices a quarter of the tolerance apart, deliberately straddling the
    // rounding of the hash grid's cell size: a grid that only looked in its own cell
    // would find no neighbour and weld nothing.
    TriangleMesh mesh;
    const float tolerance = 1.0e-3f;
    const float epsilon = tolerance * 0.25f;
    mesh.positions = {0.0f,   0.0f, 0.0f, 1.0f, 0.0f,    0.0f, 0.0f, 1.0f, 0.0f,
                      -epsilon, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f};
    mesh.indices = {0, 1, 2, 3, 4, 5};

    MeshRepairOptions options;
    options.weld_tolerance = tolerance;
    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(mesh.view(), options, repaired);

    EXPECT_EQ(report.welded_vertices, 1u);
    EXPECT_EQ(repaired.vertex_count(), 5u);
    EXPECT_EQ(repaired.triangle_count(), 2u);
}

TEST(Unit_GeometryMeshUtilities,WeldingASliverIntoALineDropsIt)
{
    // The reason the drop has to run *after* the weld: this triangle has area before
    // welding and none after, so a pipeline that dropped degenerates first would keep
    // a triangle that is a line segment.
    TriangleMesh mesh;
    mesh.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0e-7f, 0.0f};
    mesh.indices = {0, 1, 2};
    ASSERT_GT(analyze_mesh_topology(mesh.view()).surface_area, 0.0f);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(mesh.view(), MeshRepairOptions{}, repaired);
    EXPECT_EQ(report.welded_vertices, 1u);
    EXPECT_EQ(report.removed_triangles, 1u);
    EXPECT_EQ(repaired.triangle_count(), 0u);
}

TEST(Unit_GeometryMeshUtilities,RepairDropsDuplicatesAndDegenerates)
{
    TriangleMesh dirty = unit_cube();
    for (int i = 0; i < 3; ++i)
        dirty.indices.push_back(CUBE_FACES[3][i]);
    dirty.indices.push_back(2);
    dirty.indices.push_back(2);
    dirty.indices.push_back(5);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(dirty.view(), MeshRepairOptions{}, repaired);

    EXPECT_EQ(report.before.duplicate_triangles, 1u);
    EXPECT_EQ(report.before.degenerate_triangles, 1u);
    EXPECT_EQ(report.removed_triangles, 2u);
    EXPECT_EQ(repaired.triangle_count(), 12u);
    EXPECT_TRUE(report.after.watertight());
    EXPECT_EQ(report.after.duplicate_triangles, 0u);
}

TEST(Unit_GeometryMeshUtilities,RepairMakesAMixedWindingAgree)
{
    TriangleMesh mixed = unit_cube();
    // Flip three faces, scattered so the propagation has to cross the whole shell.
    for (const std::size_t face : {std::size_t(0), std::size_t(5), std::size_t(9)})
        std::swap(mixed.indices[face * 3 + 1], mixed.indices[face * 3 + 2]);
    ASSERT_FALSE(analyze_mesh_topology(mixed.view()).consistently_oriented());

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(mixed.view(), MeshRepairOptions{}, repaired);

    EXPECT_TRUE(report.after.consistently_oriented());
    EXPECT_GT(report.reoriented_triangles, 0u);
    EXPECT_TRUE(report.after.watertight());
    // Consistent *and* outward: the volume's sign is the whole test, since a shell
    // agreeing with itself inside-out passes the consistency check and still bakes a
    // distance field with its signs inverted.
    EXPECT_NEAR(report.after.signed_volume, 1.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,RepairTurnsAnInsideOutShellOutward)
{
    TriangleMesh inverted = unit_cube();
    for (std::size_t face = 0; face < inverted.triangle_count(); ++face)
        std::swap(inverted.indices[face * 3 + 1], inverted.indices[face * 3 + 2]);

    const MeshTopologyReport before = analyze_mesh_topology(inverted.view());
    // Uniformly reversed is perfectly *consistent*; only the sign gives it away.
    EXPECT_TRUE(before.consistently_oriented());
    EXPECT_NEAR(before.signed_volume, -1.0f, 1e-5f);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(inverted.view(), MeshRepairOptions{}, repaired);
    EXPECT_EQ(report.reversed_components, 1u);
    EXPECT_EQ(report.reoriented_triangles, 0u);
    EXPECT_NEAR(report.after.signed_volume, 1.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,RepairLeavesAnOpenShellsWindingAlone)
{
    // An open shell encloses nothing, so its signed volume is not a statement about
    // which way is out and reversing on its sign would be a coin flip.
    TriangleMesh open = unit_cube();
    open.indices.resize(open.indices.size() - 6);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(open.view(), MeshRepairOptions{}, repaired);
    EXPECT_EQ(report.reversed_components, 0u);
    EXPECT_EQ(repaired.triangle_count(), 10u);
    EXPECT_EQ(report.after.boundary_edges, 4u);
}

TEST(Unit_GeometryMeshUtilities,RepairCompactsVerticesNothingReferences)
{
    TriangleMesh padded = unit_cube();
    padded.positions.push_back(7.0f);
    padded.positions.push_back(7.0f);
    padded.positions.push_back(7.0f);
    ASSERT_EQ(analyze_mesh_topology(padded.view()).unreferenced_vertices, 1u);

    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(padded.view(), MeshRepairOptions{}, repaired);
    EXPECT_EQ(report.removed_vertices, 1u);
    EXPECT_EQ(repaired.vertex_count(), 8u);
    EXPECT_EQ(report.after.unreferenced_vertices, 0u);
}

TEST(Unit_GeometryMeshUtilities,RepairIsAFunctionOfTheInputAndNotOfTheContainer)
{
    // The same input must repair to the byte-identical mesh every run. The hash grid
    // is the reason this is worth asserting: welding decides against whatever the
    // bucket held, so a candidate order that came out of the container rather than out
    // of the vertex numbering would make two runs disagree.
    const TriangleMesh exploded = exploded_cube();
    TriangleMesh first;
    TriangleMesh second;
    repair_mesh(exploded.view(), MeshRepairOptions{}, first);
    repair_mesh(exploded.view(), MeshRepairOptions{}, second);

    EXPECT_EQ(first.positions, second.positions);
    EXPECT_EQ(first.indices, second.indices);
}

TEST(Unit_GeometryMeshUtilities,RepairRecoversTheSameSurfaceFromAPermutedInput)
{
    // Renumbering the vertices legitimately renumbers the output — welding keeps the
    // lowest-numbered representative — so the assertion is on the properties of the
    // *surface*, which is what the cook consumes and what the report describes.
    const TriangleMesh exploded = exploded_cube();
    TriangleMesh permuted;
    const std::size_t vertices = exploded.vertex_count();
    for (std::size_t i = 0; i < vertices; ++i)
    {
        const std::size_t source = vertices - 1 - i;
        permuted.positions.push_back(exploded.positions[source * 3 + 0]);
        permuted.positions.push_back(exploded.positions[source * 3 + 1]);
        permuted.positions.push_back(exploded.positions[source * 3 + 2]);
    }
    for (const std::uint32_t index : exploded.indices)
        permuted.indices.push_back(std::uint32_t(vertices - 1 - index));

    TriangleMesh from_original;
    TriangleMesh from_permuted;
    const MeshRepairReport original_report =
        repair_mesh(exploded.view(), MeshRepairOptions{}, from_original);
    const MeshRepairReport permuted_report =
        repair_mesh(permuted.view(), MeshRepairOptions{}, from_permuted);

    EXPECT_EQ(from_original.vertex_count(), from_permuted.vertex_count());
    EXPECT_EQ(from_original.triangle_count(), from_permuted.triangle_count());
    EXPECT_EQ(original_report.welded_vertices, permuted_report.welded_vertices);
    EXPECT_TRUE(permuted_report.after.watertight());
    EXPECT_NEAR(original_report.after.signed_volume, permuted_report.after.signed_volume, 1e-5f);
    EXPECT_NEAR(permuted_report.after.surface_area, 6.0f, 1e-5f);
}

TEST(Unit_GeometryMeshUtilities,RepairHandlesAnEmptyMeshWithoutInventingOne)
{
    TriangleMesh empty;
    TriangleMesh repaired;
    const MeshRepairReport report = repair_mesh(empty.view(), MeshRepairOptions{}, repaired);
    EXPECT_EQ(repaired.vertex_count(), 0u);
    EXPECT_EQ(repaired.triangle_count(), 0u);
    EXPECT_EQ(report.before.triangle_count, 0u);
    EXPECT_FALSE(report.before.watertight());
}

TEST(Unit_GeometryMeshUtilities,ClosestPointAnswersEachVoronoiRegionOfATriangle)
{
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {1.0f, 0.0f, 0.0f};
    const float c[3] = {0.0f, 1.0f, 0.0f};
    float out[3];

    // Face region: straight down onto the interior.
    const float above[3] = {0.25f, 0.25f, 1.0f};
    closest_point_on_triangle(above, a, b, c, out);
    EXPECT_NEAR(out[0], 0.25f, 1e-6f);
    EXPECT_NEAR(out[1], 0.25f, 1e-6f);
    EXPECT_NEAR(out[2], 0.0f, 1e-6f);

    // Vertex region: past the corner at A, where the plane projection would land
    // outside the triangle and a naive routine would return it anyway.
    const float beyond_a[3] = {-1.0f, -1.0f, 0.0f};
    closest_point_on_triangle(beyond_a, a, b, c, out);
    EXPECT_NEAR(out[0], 0.0f, 1e-6f);
    EXPECT_NEAR(out[1], 0.0f, 1e-6f);

    // Edge region: outside the hypotenuse, nearest its midpoint.
    const float beyond_bc[3] = {1.0f, 1.0f, 0.0f};
    closest_point_on_triangle(beyond_bc, a, b, c, out);
    EXPECT_NEAR(out[0], 0.5f, 1e-6f);
    EXPECT_NEAR(out[1], 0.5f, 1e-6f);

    // A degenerate triangle must return a point on it rather than a not-a-number.
    closest_point_on_triangle(above, a, a, a, out);
    EXPECT_TRUE(std::isfinite(out[0]));
    EXPECT_TRUE(std::isfinite(out[1]));
    EXPECT_TRUE(std::isfinite(out[2]));
}

TEST(Unit_GeometryMeshUtilities,TetrahedronBarycentricRoundTripsAndExtrapolates)
{
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {1.0f, 0.0f, 0.0f};
    const float c[3] = {0.0f, 1.0f, 0.0f};
    const float d[3] = {0.0f, 0.0f, 1.0f};
    float weights[4];

    // A point inside: four coordinates in range, summing to one, and the weighted sum
    // of the corners reproducing the point — which is §8.6 invariant 2 in miniature.
    const float inside[3] = {0.2f, 0.3f, 0.1f};
    ASSERT_TRUE(tetrahedron_barycentric(inside, a, b, c, d, weights));
    float sum = 0.0f;
    for (int i = 0; i < 4; ++i)
    {
        EXPECT_GE(weights[i], 0.0f);
        EXPECT_LE(weights[i], 1.0f);
        sum += weights[i];
    }
    EXPECT_NEAR(sum, 1.0f, 1e-6f);
    for (int axis = 0; axis < 3; ++axis)
    {
        const float rebuilt = a[axis] * weights[0] + b[axis] * weights[1] +
                              c[axis] * weights[2] + d[axis] * weights[3];
        EXPECT_NEAR(rebuilt, inside[axis], 1e-6f);
    }

    // A point outside still round-trips, through a negative coordinate. This is the
    // mechanism §8.3 stage 6 binds a thin feature with, so it must not be clamped.
    const float outside[3] = {-0.5f, 0.2f, 0.2f};
    ASSERT_TRUE(tetrahedron_barycentric(outside, a, b, c, d, weights));
    bool any_negative = false;
    for (int i = 0; i < 4; ++i)
        any_negative = any_negative || weights[i] < 0.0f;
    EXPECT_TRUE(any_negative);
    for (int axis = 0; axis < 3; ++axis)
    {
        const float rebuilt = a[axis] * weights[0] + b[axis] * weights[1] +
                              c[axis] * weights[2] + d[axis] * weights[3];
        EXPECT_NEAR(rebuilt, outside[axis], 1e-6f);
    }

    // A flat tetrahedron has no coordinates; the caller is told so and gets an
    // average rather than a division by zero.
    const float flat[3] = {1.0f, 1.0f, 0.0f};
    EXPECT_FALSE(tetrahedron_barycentric(inside, a, b, c, flat, weights));
    for (int i = 0; i < 4; ++i)
        EXPECT_NEAR(weights[i], 0.25f, 1e-6f);
}

TEST(Unit_GeometryMeshUtilities,TetrahedronVolumeSignSaysWhichSideTheFourthVertexIsOn)
{
    const float a[3] = {0.0f, 0.0f, 0.0f};
    const float b[3] = {1.0f, 0.0f, 0.0f};
    const float c[3] = {0.0f, 1.0f, 0.0f};
    const float above[3] = {0.0f, 0.0f, 1.0f};
    const float below[3] = {0.0f, 0.0f, -1.0f};

    EXPECT_GT(tetrahedron_signed_volume_times_six(a, b, c, above), 0.0f);
    EXPECT_LT(tetrahedron_signed_volume_times_six(a, b, c, below), 0.0f);
    // Six times the volume: the unit corner tetrahedron has volume one sixth.
    EXPECT_NEAR(tetrahedron_signed_volume_times_six(a, b, c, above), 1.0f, 1e-6f);
}
