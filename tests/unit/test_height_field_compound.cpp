/**************************************************************************/
/* test_height_field_compound.cpp                                         */
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

// Unit_HeightField / Unit_Compound: the last two shape kinds of P2.
//
// A height field is terrain, and the property worth testing is the one that
// makes it terrain rather than a mesh: the cells under a body are found by
// arithmetic, so the answer must be exactly the cells the body is over — no
// hierarchy, no search, and no dependence on where the body happens to sit
// relative to a seam. A vehicle crosses a seam every metre, so the internal-edge
// correction gets the same treatment it got for meshes.
//
// A compound is one body with several colliders. What has to hold is that the
// parts are contacts of the *body*: the anchors must be in the body's frame, not
// the part's, or the solver applies a lever arm from the wrong origin and the
// body spins about a point that is not its centre of mass.

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/compound_manifold.hpp>
#include <SushiEngine/physics/collision/height_field_manifold.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    const Quaternion IDENTITY{0.0, 0.0, 0.0, 1.0};

    /** @brief A flat field of the given size, every height zero. */
    std::vector<Scalar> flat_heights(std::uint32_t columns, std::uint32_t rows)
    {
        return std::vector<Scalar>(static_cast<std::size_t>(columns) * rows, 0.0);
    }

    HeightFieldView<Scalar> field_over(const std::vector<Scalar>& heights, std::uint32_t columns,
                                       std::uint32_t rows, Scalar cell = 1.0,
                                       Vector3 center = Vector3{0.0, 0.0, 0.0})
    {
        HeightFieldView<Scalar> field;
        field.heights = heights.data();
        field.columns = columns;
        field.rows = rows;
        field.cell_size_x = cell;
        field.cell_size_z = cell;
        field.center = center;
        return field;
    }
} // namespace

// A box on flat terrain finds the surface, at the depth its placement says.
TEST(Unit_HeightField, BoxOnFlatTerrainRestsOnTheSurface)
{
    const std::vector<Scalar> heights = flat_heights(9, 9);
    const HeightFieldView<Scalar> field = field_over(heights, 9, 9);

    // Cells run from local (0,0) to (8,8), so put the box in the middle of one.
    const OrientedBox<Scalar> box{Vector3{4.5, 0.49, 4.5}, Vector3{0.5, 0.5, 0.5}, IDENTITY};

    std::size_t manifolds = 0;
    generate_convex_height_field_manifolds<Scalar>(
        box, field, box.center, box.orientation, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            ++manifolds;
            EXPECT_NEAR(manifold.normal.y, -1.0, 1e-6);
            for (std::size_t i = 0; i < manifold.point_count; ++i)
                EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-6);
        });
    EXPECT_GT(manifolds, 0u);
}

// The seams again, on the surface where they matter most: a vehicle crossing
// terrain crosses one every metre, and none of them may produce an edge normal.
TEST(Unit_HeightField, SlidingAcrossTerrainSeamsKeepsTheSurfaceNormal)
{
    const std::vector<Scalar> heights = flat_heights(9, 9);
    const HeightFieldView<Scalar> field = field_over(heights, 9, 9);

    std::size_t contacts = 0;
    Scalar worst = 1.0;
    for (int step = 0; step < 300; ++step)
    {
        const Scalar x = 1.0 + 6.0 * static_cast<Scalar>(step) / 299.0;
        const OrientedBox<Scalar> box{Vector3{x, 0.495, 4.3}, Vector3{0.4, 0.5, 0.4}, IDENTITY};
        generate_convex_height_field_manifolds<Scalar>(
            box, field, box.center, box.orientation, 0.0, 1e-3,
            [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
            {
                ++contacts;
                worst = std::min(worst, static_cast<Scalar>(-manifold.normal.y));
            });
    }
    EXPECT_GT(contacts, 250u);
    EXPECT_GT(worst, 1.0 - 1e-6);
}

// Sloped terrain must actually slope: the normal follows the surface rather than
// pointing straight up because the field is a grid.
TEST(Unit_HeightField, SlopedTerrainReportsTheSlopeNormal)
{
    // A ramp rising one unit per cell along x: a 45-degree slope.
    std::vector<Scalar> heights(9 * 9, 0.0);
    for (std::uint32_t row = 0; row < 9; ++row)
        for (std::uint32_t column = 0; column < 9; ++column)
            heights[row * 9 + column] = static_cast<Scalar>(column);
    const HeightFieldView<Scalar> field = field_over(heights, 9, 9);

    // A small sphere sitting just above the slope at x = 4.
    const SphereCollider<Scalar> ball{Vector3{4.0, 4.24, 4.5}, 0.25};

    std::size_t contacts = 0;
    generate_convex_height_field_manifolds<Scalar>(
        ball, field, ball.center, IDENTITY, 0.05, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            ++contacts;
            // The surface normal is (-1, 1, 0)/sqrt(2); the contact normal runs
            // from the ball toward the surface, so it is the negation.
            EXPECT_NEAR(manifold.normal.x, 1.0 / std::sqrt(2.0), 1e-3);
            EXPECT_NEAR(manifold.normal.y, -1.0 / std::sqrt(2.0), 1e-3);
        });
    EXPECT_GT(contacts, 0u);
}

// Only the cells under the body, and only cells that exist. A body off the edge
// of the field must produce nothing rather than reading past the array.
TEST(Unit_HeightField, CellRangeIsClampedToTheField)
{
    const std::vector<Scalar> heights = flat_heights(5, 5);
    const HeightFieldView<Scalar> field = field_over(heights, 5, 5);

    // Sitting on the corner cell: the range must clamp rather than wrap.
    std::vector<std::uint32_t> cells;
    const SphereCollider<Scalar> corner{Vector3{0.0, -0.05, 0.0}, 0.3};
    generate_convex_height_field_manifolds<Scalar>(
        corner, field, corner.center, IDENTITY, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>&, std::uint32_t cell) { cells.push_back(cell); });
    EXPECT_GT(cells.size(), 0u);

    // Well clear of the field in every direction: nothing at all.
    for (const Vector3 away : {Vector3{-10.0, 0.0, 2.0}, Vector3{20.0, 0.0, 2.0},
                               Vector3{2.0, 0.0, -10.0}, Vector3{2.0, 0.0, 20.0}})
    {
        std::size_t hits = 0;
        const SphereCollider<Scalar> distant{away, 0.3};
        generate_convex_height_field_manifolds<Scalar>(
            distant, field, distant.center, IDENTITY, 0.0, 1e-3,
            [&](const ContactManifold<Scalar>&, std::uint32_t) { ++hits; });
        EXPECT_EQ(hits, 0u) << "at (" << away.x << ", " << away.z << ")";
    }
}

// A malformed field is inert rather than a crash: an importer can produce one.
TEST(Unit_HeightField, DegenerateFieldsAreInert)
{
    const SphereCollider<Scalar> ball{Vector3{0.0, 0.0, 0.0}, 1.0};
    std::size_t hits = 0;
    const auto count = [&](const ContactManifold<Scalar>&, std::uint32_t) { ++hits; };

    HeightFieldView<Scalar> empty;
    generate_convex_height_field_manifolds<Scalar>(ball, empty, ball.center, IDENTITY, 0.0, 1e-3,
                                                   count);

    const std::vector<Scalar> single = flat_heights(1, 1);
    HeightFieldView<Scalar> thin = field_over(single, 1, 1);
    generate_convex_height_field_manifolds<Scalar>(ball, thin, ball.center, IDENTITY, 0.0, 1e-3,
                                                   count);

    const std::vector<Scalar> heights = flat_heights(4, 4);
    HeightFieldView<Scalar> zero_cell = field_over(heights, 4, 4, 0.0);
    generate_convex_height_field_manifolds<Scalar>(ball, zero_cell, ball.center, IDENTITY, 0.0,
                                                   1e-3, count);

    EXPECT_EQ(hits, 0u);
}

// A compound's parts are placed by the body's pose, so authoring a part once and
// instancing the body anywhere has to work.
TEST(Unit_Compound, PartsFollowTheBodyPose)
{
    CompoundPart<Scalar> part;
    part.shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    part.local_position = Vector3{1.0, 0.0, 0.0};

    const Quaternion turned = quaternion_axis_angle(Vector3{0.0, 1.0, 0.0}, 1.5707963267948966);
    const CollisionShape<Scalar> placed =
        place_compound_part(part, Vector3{0.0, 3.0, 0.0}, turned);

    // Rotating 90 degrees about Y carries +X to -Z.
    EXPECT_NEAR(placed.center.x, 0.0, 1e-9);
    EXPECT_NEAR(placed.center.y, 3.0, 1e-9);
    EXPECT_NEAR(placed.center.z, -1.0, 1e-9);

    // A half-space part has no centre to move.
    CompoundPart<Scalar> plane_part;
    plane_part.shape = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);
    plane_part.local_position = Vector3{5.0, 5.0, 5.0};
    const CollisionShape<Scalar> plane_placed =
        place_compound_part(plane_part, Vector3{9.0, 9.0, 9.0}, turned);
    EXPECT_NEAR(plane_placed.plane_offset, 0.0, 1e-12);
    EXPECT_NEAR(plane_placed.plane_normal.y, 1.0, 1e-12);
}

// The property a compound exists for: two parts touching the ground at two
// places are two contacts, and both are contacts of the *body*. If the anchors
// stayed in the parts' frames the solver would lever them from the wrong origin.
TEST(Unit_Compound, PartsProduceSeparateBodyAnchoredContacts)
{
    // A dumbbell: two boxes a metre either side of the body's centre.
    std::vector<CompoundPart<Scalar>> parts(2);
    parts[0].shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.3, 0.3, 0.3});
    parts[0].local_position = Vector3{-1.0, 0.0, 0.0};
    parts[1].shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.3, 0.3, 0.3});
    parts[1].local_position = Vector3{1.0, 0.0, 0.0};

    const Vector3 body_center{0.0, 0.29, 0.0};
    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);

    std::vector<std::uint32_t> touched;
    std::vector<Vector3> anchors;
    generate_compound_manifolds<Scalar>(
        parts.data(), 2, body_center, IDENTITY, ground, Vector3{0.0, 0.0, 0.0}, IDENTITY, 0.0,
        1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t part)
        {
            touched.push_back(part);
            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                anchors.push_back(manifold.points[i].anchor_a_local);
                EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-6);
            }
        });

    // Both parts touched, separately.
    ASSERT_EQ(touched.size(), 2u);
    EXPECT_EQ(touched[0], 0u);
    EXPECT_EQ(touched[1], 1u);

    // The anchors are body-local: they straddle the body's centre in x, roughly a
    // metre either side. In the parts' own frames they would all be within 0.3.
    Scalar lowest = 1e30;
    Scalar highest = -1e30;
    for (const Vector3& anchor : anchors)
    {
        lowest = std::min(lowest, anchor.x);
        highest = std::max(highest, anchor.x);
        // Down from the body's centre to the parts' bottoms: the body sits at
        // y = 0.29, a part's underside is a centimetre below the ground at
        // y = -0.01, so in the body's frame that is -0.30.
        EXPECT_NEAR(anchor.y, -0.30, 1e-6);
    }
    EXPECT_LT(lowest, -0.6);
    EXPECT_GT(highest, 0.6);
}

// Two parts must not share a warm-start identity, or one part's accumulated
// impulse is handed to the other the moment they produce the same local feature.
TEST(Unit_Compound, PartsDoNotShareFeatureIdentities)
{
    std::vector<CompoundPart<Scalar>> parts(2);
    parts[0].shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.3, 0.3, 0.3});
    parts[0].local_position = Vector3{-1.0, 0.0, 0.0};
    parts[1].shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.3, 0.3, 0.3});
    parts[1].local_position = Vector3{1.0, 0.0, 0.0};

    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);
    std::vector<std::uint32_t> ids;
    generate_compound_manifolds<Scalar>(
        parts.data(), 2, Vector3{0.0, 0.29, 0.0}, IDENTITY, ground, Vector3{0.0, 0.0, 0.0},
        IDENTITY, 0.0, 1e-3,
        [&](const ContactManifold<Scalar>& manifold, std::uint32_t)
        {
            for (std::size_t i = 0; i < manifold.point_count; ++i)
                ids.push_back(manifold.points[i].feature_id);
        });

    ASSERT_GE(ids.size(), 4u);
    std::sort(ids.begin(), ids.end());
    EXPECT_EQ(std::adjacent_find(ids.begin(), ids.end()), ids.end())
        << "two contact points share an identity";
}

// The compound's bounds enclose every part, which is what the broadphase will
// key on — an undersized box is a missed contact.
TEST(Unit_Compound, BoundsEncloseEveryPart)
{
    std::vector<CompoundPart<Scalar>> parts(3);
    parts[0].shape = make_sphere_shape(Vector3{0.0, 0.0, 0.0}, 0.5);
    parts[0].local_position = Vector3{-2.0, 0.0, 0.0};
    parts[1].shape = make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{0.25, 1.0, 0.25});
    parts[2].shape = make_capsule_shape(Vector3{0.0, 0.0, 0.0}, 0.75, 0.2);
    parts[2].local_position = Vector3{2.0, 0.0, 0.0};

    const AABB<Scalar> bounds =
        compound_bounds<Scalar>(parts.data(), 3, Vector3{0.0, 0.0, 0.0}, IDENTITY);

    EXPECT_NEAR(bounds.min.x, -2.5, 1e-9);
    EXPECT_NEAR(bounds.max.x, 2.2, 1e-9);
    EXPECT_NEAR(bounds.min.y, -1.0, 1e-9);
    EXPECT_NEAR(bounds.max.y, 1.0, 1e-9);
}
