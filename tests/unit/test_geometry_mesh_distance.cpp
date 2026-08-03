/**************************************************************************/
/* test_geometry_mesh_distance.cpp                                        */
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

// The distance hierarchy, checked against the thing it replaced. A pruned descent
// through a tree is exactly the kind of code that is fast and subtly wrong — a
// mis-signed box test loses a whole subtree and the answer stays plausible — so the
// central test here is not a hand-computed distance but agreement with the brute-force
// sweep over every triangle, at hundreds of query points, inside and out.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/geometry/mesh_distance_query.hpp>
#include <SushiEngine/geometry/mesh_utilities.hpp>
#include <SushiEngine/geometry/signed_distance_field.hpp>
#include <SushiEngine/geometry/triangle_mesh.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Geometry;

namespace
{
    /** @brief An axis-aligned box as twelve outward-wound triangles. */
    TriangleMesh box_mesh(float half_x, float half_y, float half_z)
    {
        TriangleMesh mesh;
        const float corners[8][3] = {
            {-half_x, -half_y, -half_z}, {half_x, -half_y, -half_z},
            {half_x, half_y, -half_z},   {-half_x, half_y, -half_z},
            {-half_x, -half_y, half_z},  {half_x, -half_y, half_z},
            {half_x, half_y, half_z},    {-half_x, half_y, half_z}};
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

    /**
     * @brief A latitude-longitude sphere: enough triangles that the tree has to branch.
     *
     * A box has twelve triangles and fits in three leaves, which tests nothing about
     * pruning. This is a thousand or so, deep enough that a lost subtree shows up.
     */
    TriangleMesh sphere_mesh(float radius, std::uint32_t stacks, std::uint32_t slices)
    {
        TriangleMesh mesh;
        const float pi = 3.14159265358979323846f;
        for (std::uint32_t stack = 0; stack <= stacks; ++stack)
        {
            const float phi = pi * float(stack) / float(stacks);
            for (std::uint32_t slice = 0; slice < slices; ++slice)
            {
                const float theta = 2.0f * pi * float(slice) / float(slices);
                mesh.positions.push_back(radius * std::sin(phi) * std::cos(theta));
                mesh.positions.push_back(radius * std::cos(phi));
                mesh.positions.push_back(radius * std::sin(phi) * std::sin(theta));
            }
        }
        for (std::uint32_t stack = 0; stack < stacks; ++stack)
        {
            for (std::uint32_t slice = 0; slice < slices; ++slice)
            {
                const std::uint32_t next_slice = (slice + 1) % slices;
                const std::uint32_t a = stack * slices + slice;
                const std::uint32_t b = stack * slices + next_slice;
                const std::uint32_t c = (stack + 1) * slices + slice;
                const std::uint32_t d = (stack + 1) * slices + next_slice;
                // Wound outward: with y = r cos(phi) the next stack is *lower*, so this
                // is the order whose cross product points away from the centre. Getting
                // it backwards inverts every signed distance and nothing else, which is
                // why the analytic test below asks for the sign and not just the size.
                mesh.indices.push_back(a);
                mesh.indices.push_back(b);
                mesh.indices.push_back(c);
                mesh.indices.push_back(b);
                mesh.indices.push_back(d);
                mesh.indices.push_back(c);
            }
        }
        return mesh;
    }

    /** @brief The closest distance by sweeping every triangle — the oracle. */
    float brute_force_distance(const TriangleMesh& mesh, const float point[3])
    {
        float best = -1.0f;
        for (std::size_t t = 0; t < mesh.triangle_count(); ++t)
        {
            const std::uint32_t i0 = mesh.indices[t * 3 + 0];
            const std::uint32_t i1 = mesh.indices[t * 3 + 1];
            const std::uint32_t i2 = mesh.indices[t * 3 + 2];
            const float* a = mesh.positions.data() + i0 * 3;
            const float* b = mesh.positions.data() + i1 * 3;
            const float* c = mesh.positions.data() + i2 * 3;
            // The oracle skips what the hierarchy skips, or the two are not being asked
            // the same question. Zero *area* is the test and not a repeated index: a
            // sphere's pole is a ring of distinct vertices at one position, so the
            // triangles there collapse without any index appearing twice.
            const float edge_one[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const float edge_two[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            const float normal[3] = {edge_one[1] * edge_two[2] - edge_one[2] * edge_two[1],
                                     edge_one[2] * edge_two[0] - edge_one[0] * edge_two[2],
                                     edge_one[0] * edge_two[1] - edge_one[1] * edge_two[0]};
            if (!(std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] +
                            normal[2] * normal[2]) > 0.0f))
                continue;
            float candidate[3];
            closest_point_on_triangle(point, a, b, c, candidate);
            const float dx = point[0] - candidate[0];
            const float dy = point[1] - candidate[1];
            const float dz = point[2] - candidate[2];
            const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (best < 0.0f || distance < best)
                best = distance;
        }
        return best;
    }

    /** @brief A deterministic integer sequence, so a failure reproduces exactly. */
    class Sequence
    {
    public:
        float next(float low, float high) noexcept
        {
            state_ = state_ * 1664525u + 1013904223u;
            const float unit = float(state_ >> 8) / float(1u << 24);
            return low + unit * (high - low);
        }

    private:
        std::uint32_t state_ = 12345u;
    };
} // namespace

TEST(Unit_GeometryMeshDistance,AgreesWithTheBruteForceSweepEverywhere)
{
    const TriangleMesh sphere = sphere_mesh(1.0f, 16, 32);
    ASSERT_GT(sphere.triangle_count(), 500u);

    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(sphere.view()));
    // The poles collapse a quad to a sliver, so the hierarchy holds fewer triangles
    // than the source: skipping them is deliberate, not a lost subtree.
    EXPECT_LE(query.triangle_count(), sphere.triangle_count());
    EXPECT_GT(query.node_count(), 1u);

    Sequence sequence;
    for (int sample = 0; sample < 400; ++sample)
    {
        // Deliberately spanning well inside, just off the surface, and far outside.
        const float point[3] = {sequence.next(-2.0f, 2.0f), sequence.next(-2.0f, 2.0f),
                                sequence.next(-2.0f, 2.0f)};
        const MeshClosestPoint found = query.closest_point(point);
        ASSERT_TRUE(found.valid);
        EXPECT_NEAR(found.distance, brute_force_distance(sphere, point), 1e-4f);

        // The reported point must be the one the reported distance measures to, or a
        // caller that uses the point rather than the number silently disagrees.
        const float dx = point[0] - found.point[0];
        const float dy = point[1] - found.point[1];
        const float dz = point[2] - found.point[2];
        EXPECT_NEAR(std::sqrt(dx * dx + dy * dy + dz * dz), found.distance, 1e-4f);
    }
}

TEST(Unit_GeometryMeshDistance,MeasuresASphereAgainstItsAnalyticDistance)
{
    // Against theory rather than against the oracle, so a bug shared by both routines
    // has somewhere to show. The tessellation chords inward, so a query outside reads
    // slightly long and one inside slightly short; the tolerance is that sagitta.
    const TriangleMesh sphere = sphere_mesh(1.0f, 32, 64);
    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(sphere.view()));

    const float outside[3] = {3.0f, 0.0f, 0.0f};
    EXPECT_NEAR(query.signed_distance(outside), 2.0f, 0.01f);

    const float centre[3] = {0.0f, 0.0f, 0.0f};
    EXPECT_NEAR(query.signed_distance(centre), -1.0f, 0.01f);

    const float inside[3] = {0.5f, 0.0f, 0.0f};
    EXPECT_NEAR(query.signed_distance(inside), -0.5f, 0.01f);
}

TEST(Unit_GeometryMeshDistance,SignsTheInsideOfABoxNegative)
{
    const TriangleMesh box = box_mesh(1.0f, 2.0f, 3.0f);
    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(box.view()));

    const float centre[3] = {0.0f, 0.0f, 0.0f};
    // Nearest face is the x one, a unit away.
    EXPECT_NEAR(query.signed_distance(centre), -1.0f, 1e-5f);

    const float outside[3] = {2.5f, 0.0f, 0.0f};
    EXPECT_NEAR(query.signed_distance(outside), 1.5f, 1e-5f);

    const float on_face[3] = {1.0f, 0.0f, 0.0f};
    EXPECT_NEAR(std::fabs(query.signed_distance(on_face)), 0.0f, 1e-5f);
}

TEST(Unit_GeometryMeshDistance,RefusesAMeshWithNothingUsableInIt)
{
    TriangleMesh degenerate;
    degenerate.positions = {0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f};
    degenerate.indices = {0, 1, 2};

    MeshDistanceQuery query;
    // Three collinear points are not a surface. Reporting that is the point: a query
    // that answered zero everywhere would let a cook proceed against nothing.
    EXPECT_FALSE(query.build(degenerate.view()));
    EXPECT_FALSE(query.ready());

    const float point[3] = {1.0f, 1.0f, 1.0f};
    EXPECT_FALSE(query.closest_point(point).valid);
    EXPECT_EQ(query.signed_distance(point), 0.0f);
}

TEST(Unit_GeometryMeshDistance,SamplesASurfaceDeterministicallyAndOnIt)
{
    const TriangleMesh box = box_mesh(1.0f, 1.0f, 1.0f);
    std::vector<float> first;
    std::vector<float> second;
    const std::size_t count = sample_surface_points(box.view(), 3, first);
    EXPECT_EQ(sample_surface_points(box.view(), 3, second), count);
    EXPECT_EQ(first, second);

    // A lattice of order three is ten points per triangle, corners included.
    EXPECT_EQ(count, box.triangle_count() * 10u);

    // Every sample lies on the surface it was taken from, which is the property the
    // accuracy report leans on.
    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(box.view()));
    for (std::size_t i = 0; i < count; ++i)
        EXPECT_NEAR(query.closest_point(first.data() + i * 3).distance, 0.0f, 1e-5f);

    // Order zero is meaningless and is clamped rather than dividing by it.
    std::vector<float> clamped;
    EXPECT_EQ(sample_surface_points(box.view(), 0, clamped), box.triangle_count() * 3u);
}

TEST(Unit_GeometryMeshDistance,HausdorffOfASurfaceAgainstItselfIsZero)
{
    const TriangleMesh sphere = sphere_mesh(1.0f, 12, 24);
    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(sphere.view()));
    EXPECT_NEAR(one_sided_hausdorff_distance(sphere.view(), query, 3), 0.0f, 1e-5f);
    EXPECT_NEAR(max_protrusion_distance(sphere.view(), query, 3), 0.0f, 1e-5f);
}

TEST(Unit_GeometryMeshDistance,ProtrusionMeasuresHowMuchFatterAColliderIs)
{
    // §7.6's question, asked the way a player experiences it: not how the two surfaces
    // differ but how far the collision geometry sticks out past the visible one.
    const TriangleMesh visible = box_mesh(1.0f, 1.0f, 1.0f);
    const TriangleMesh collider = box_mesh(1.1f, 1.1f, 1.1f);

    MeshDistanceQuery visible_query;
    ASSERT_TRUE(visible_query.build(visible.view()));

    // A face centre of the inflated box is a tenth of a unit outside the visible one;
    // its corners are further, by the diagonal, and the maximum is what is reported.
    const float protrusion = max_protrusion_distance(collider.view(), visible_query, 4);
    EXPECT_GT(protrusion, 0.09f);
    EXPECT_NEAR(protrusion, 0.1f * std::sqrt(3.0f), 1e-4f);

    // The other direction reports nothing sticking out, because the visible mesh is
    // entirely inside the collider — which is exactly why the number is one-sided.
    MeshDistanceQuery collider_query;
    ASSERT_TRUE(collider_query.build(collider.view()));
    EXPECT_NEAR(max_protrusion_distance(visible.view(), collider_query, 4), 0.0f, 1e-5f);

    // The unsigned Hausdorff distance sees the same departure from either side.
    EXPECT_GT(one_sided_hausdorff_distance(visible.view(), collider_query, 4), 0.09f);
}

TEST(Unit_GeometryMeshDistance,HausdorffTightensAsTheLatticeRefines)
{
    // The number is a sampled lower bound, and this is that statement as a test: a
    // coarser lattice can only under-report, so refining must never lower it.
    const TriangleMesh visible = box_mesh(1.0f, 1.0f, 1.0f);
    const TriangleMesh collider = box_mesh(1.05f, 1.0f, 1.0f);
    MeshDistanceQuery visible_query;
    ASSERT_TRUE(visible_query.build(visible.view()));

    const float coarse = max_protrusion_distance(collider.view(), visible_query, 1);
    const float fine = max_protrusion_distance(collider.view(), visible_query, 8);
    EXPECT_GE(fine, coarse - 1e-6f);
}

TEST(Unit_GeometryMeshDistance,TheDistanceFieldBakeMatchesTheQueryItIsBuiltOn)
{
    // The bake reads one query per voxel, so its brick must agree with the query at
    // every voxel centre. This is what keeps the hierarchy's arrival behaviour-neutral
    // for the renderer's global illumination, which consumes the same brick.
    const TriangleMesh box = box_mesh(0.5f, 0.5f, 0.5f);
    const SignedDistanceFieldBrick brick = bake_signed_distance_field(box.view(), 12);
    ASSERT_EQ(brick.resolution, 12);
    ASSERT_EQ(brick.distances.size(), 12u * 12u * 12u);

    MeshDistanceQuery query;
    ASSERT_TRUE(query.build(box.view()));

    const std::int32_t resolution = brick.resolution;
    for (std::int32_t z = 0; z < resolution; ++z)
    {
        for (std::int32_t y = 0; y < resolution; ++y)
        {
            for (std::int32_t x = 0; x < resolution; ++x)
            {
                float centre[3];
                for (int axis = 0; axis < 3; ++axis)
                {
                    const std::int32_t voxel = axis == 0 ? x : (axis == 1 ? y : z);
                    const float span = brick.aabb_max[axis] - brick.aabb_min[axis];
                    centre[axis] = brick.aabb_min[axis] +
                                   (float(voxel) + 0.5f) * span / float(resolution);
                }
                const std::size_t index =
                    std::size_t(x) + std::size_t(resolution) *
                                         (std::size_t(y) + std::size_t(resolution) *
                                                               std::size_t(z));
                ASSERT_NEAR(brick.distances[index], query.signed_distance(centre), 1e-5f);
            }
        }
    }
}
