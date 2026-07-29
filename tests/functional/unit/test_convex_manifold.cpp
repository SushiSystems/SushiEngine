/**************************************************************************/
/* test_convex_manifold.cpp                                               */
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

// Unit_ConvexManifold: contact patches for arbitrary convex shapes
// (physics/collision/convex_manifold.hpp).
//
// GJK answers with one point. A body held by one point rocks — the same defect
// manifolds fixed for boxes — so the patch has to be recovered from the normal.
// The tests measure the *shape* of what comes back, because that is the whole
// deliverable: a hull resting flat must produce a spread patch, a capsule on its
// side must produce its two ends, a sphere must produce exactly one point because
// one point is the truth, and a shape caught on a corner must not invent a face.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/convex_manifold.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;
    const Quaternion IDENTITY{0.0, 0.0, 0.0, 1.0};

    std::vector<Vector3> box_vertices(Vector3 half_extents)
    {
        std::vector<Vector3> vertices;
        for (int i = 0; i < 8; ++i)
            vertices.push_back(Vector3{(i & 1) ? half_extents.x : -half_extents.x,
                                       (i & 2) ? half_extents.y : -half_extents.y,
                                       (i & 4) ? half_extents.z : -half_extents.z});
        return vertices;
    }

    ConvexHullView<Scalar> hull_of(const std::vector<Vector3>& vertices, Vector3 center,
                                   Quaternion orientation = Quaternion{0.0, 0.0, 0.0, 1.0})
    {
        ConvexHullView<Scalar> hull;
        hull.vertices = vertices.data();
        hull.vertex_count = static_cast<std::uint32_t>(vertices.size());
        hull.center = center;
        hull.orientation = orientation;
        return hull;
    }

    /** @brief The largest distance between any two of the manifold's points on a. */
    Scalar spread(const ContactManifold<Scalar>& manifold, Vector3 center, Quaternion orientation)
    {
        Scalar largest = 0.0;
        for (std::size_t i = 0; i < manifold.point_count; ++i)
            for (std::size_t j = i + 1; j < manifold.point_count; ++j)
            {
                const Vector3 pi =
                    to_world_anchor(center, orientation, manifold.points[i].anchor_a_local);
                const Vector3 pj =
                    to_world_anchor(center, orientation, manifold.points[j].anchor_a_local);
                largest = std::max(largest, static_cast<Scalar>(length(pi - pj)));
            }
        return largest;
    }
} // namespace

// Extraction on its own: what a shape reports as touching along a direction is
// the input everything else is built from.
TEST(Unit_ConvexManifold, ContactFaceExtractionReportsTheTouchingFeature)
{
    // A sphere: one point, always.
    const SphereCollider<Scalar> sphere{Vector3{0.0, 0.0, 0.0}, 1.0};
    EXPECT_EQ(contact_face(sphere, Vector3{0.0, 1.0, 0.0}, 1e-3).count, 1u);

    // A box square-on to the direction: four corners of a face. Caught on a
    // corner instead: one point, because there is no face there to report.
    const OrientedBox<Scalar> box{Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5}, IDENTITY};
    const ContactFace<Scalar> box_face = contact_face(box, Vector3{0.0, 1.0, 0.0}, 1e-3);
    EXPECT_EQ(box_face.count, 4u);
    EXPECT_NEAR(box_face.normal.y, 1.0, 1e-12);
    EXPECT_EQ(contact_face(box, normalize(Vector3{1.0, 1.0, 1.0}), 1e-3).count, 1u);

    // A capsule standing on its cap: one point. Lying on its side: its two ends.
    CapsuleCollider<Scalar> capsule;
    capsule.center = Vector3{0.0, 0.0, 0.0};
    capsule.half_height = 1.0;
    capsule.radius = 0.25;
    EXPECT_EQ(contact_face(capsule, Vector3{0.0, -1.0, 0.0}, 1e-3).count, 1u);
    capsule.orientation = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 2.0);
    const ContactFace<Scalar> lying = contact_face(capsule, Vector3{0.0, -1.0, 0.0}, 1e-3);
    EXPECT_EQ(lying.count, 2u);
    EXPECT_NEAR(length(lying.points[0].position - lying.points[1].position), 2.0, 1e-9);

    // A hull that is a box behaves like the box, with no face data to consult.
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const ConvexHullView<Scalar> hull = hull_of(corners, Vector3{0.0, 0.0, 0.0});
    const ContactFace<Scalar> hull_face = contact_face(hull, Vector3{0.0, 1.0, 0.0}, 1e-3);
    EXPECT_EQ(hull_face.count, 4u);
    EXPECT_NEAR(std::abs(hull_face.normal.y), 1.0, 1e-9);
    EXPECT_EQ(contact_face(hull, normalize(Vector3{1.0, 1.0, 1.0}), 1e-3).count, 1u);
}

// The headline: a hull resting on a hull produces the square they share, not one
// point of it. This is what lets a cooked convex piece rest the way a primitive
// box already does.
TEST(Unit_ConvexManifold, HullOnHullYieldsTheOverlapPatch)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const ConvexHullView<Scalar> lower = hull_of(corners, Vector3{0.0, 0.0, 0.0});
    const ConvexHullView<Scalar> upper = hull_of(corners, Vector3{0.2, 0.99, 0.0});

    const ContactManifold<Scalar> manifold = generate_convex_manifold<Scalar>(
        lower, upper, lower.center, lower.orientation, upper.center, upper.orientation, 0.0);

    ASSERT_EQ(manifold.point_count, 4);
    EXPECT_NEAR(std::abs(manifold.normal.y), 1.0, 1e-6);
    for (std::size_t i = 0; i < manifold.point_count; ++i)
    {
        EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-5);
        const Vector3 point =
            to_world_anchor(lower.center, lower.orientation, manifold.points[i].anchor_a_local);
        // The patch is the intersection of the two faces in x: [-0.3, 0.5].
        EXPECT_GE(point.x, -0.3 - 1e-4);
        EXPECT_LE(point.x, 0.5 + 1e-4);
        EXPECT_NEAR(std::abs(point.z), 0.5, 1e-4);
    }
    // And it spans that square rather than clustering.
    EXPECT_GT(spread(manifold, lower.center, lower.orientation), 1.0);
}

// A capsule lying on a box touches along a line, and the manifold must be that
// line: two points, at the segment's ends. One point and the capsule rolls.
TEST(Unit_ConvexManifold, CapsuleLyingOnABoxYieldsItsTwoEnds)
{
    const OrientedBox<Scalar> ground{Vector3{0.0, -0.5, 0.0}, Vector3{5.0, 0.5, 5.0}, IDENTITY};
    CapsuleCollider<Scalar> capsule;
    capsule.center = Vector3{0.0, 0.29, 0.0};
    capsule.orientation = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 2.0);
    capsule.half_height = 1.0;
    capsule.radius = 0.3;

    const ContactManifold<Scalar> manifold = generate_convex_manifold<Scalar>(
        ground, capsule, ground.center, ground.orientation, capsule.center, capsule.orientation,
        0.0);

    ASSERT_EQ(manifold.point_count, 2);
    EXPECT_NEAR(manifold.normal.y, 1.0, 1e-5);
    for (std::size_t i = 0; i < manifold.point_count; ++i)
        EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-4);
    EXPECT_NEAR(spread(manifold, ground.center, ground.orientation), 2.0, 1e-3);
}

// A sphere touches at one place however deep it is, so the manifold must be one
// point. Inventing a patch here would be inventing geometry.
TEST(Unit_ConvexManifold, SphereOnABoxYieldsOnePoint)
{
    const OrientedBox<Scalar> ground{Vector3{0.0, -0.5, 0.0}, Vector3{5.0, 0.5, 5.0}, IDENTITY};
    const SphereCollider<Scalar> ball{Vector3{0.0, 0.45, 0.0}, 0.5};

    const ContactManifold<Scalar> manifold = generate_convex_manifold<Scalar>(
        ground, ball, ground.center, ground.orientation, ball.center, IDENTITY, 0.0);

    ASSERT_EQ(manifold.point_count, 1);
    EXPECT_NEAR(manifold.normal.y, 1.0, 1e-5);
    EXPECT_NEAR(manifold.points[0].separation, -0.05, 1e-4);
}

// A hull caught on one corner is a one-point contact, and the extraction's
// alignment test is what keeps it from reporting a face that is not touching.
TEST(Unit_ConvexManifold, CornerContactStaysOnePoint)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> ground{Vector3{0.0, -0.5, 0.0}, Vector3{5.0, 0.5, 5.0}, IDENTITY};
    // Tipped about two axes, so no face and no edge is flush with the ground.
    const Quaternion tilt = mul(quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, 0.6),
                                quaternion_axis_angle(Vector3{1.0, 0.0, 0.0}, 0.5));
    const ConvexHullView<Scalar> crate = hull_of(corners, Vector3{0.0, 0.0, 0.0}, tilt);

    // Lower it until its lowest corner is just below the ground.
    Scalar lowest = 1e30;
    for (const Vector3& corner : corners)
        lowest = std::min(lowest, static_cast<Scalar>(rotate(tilt, corner).y));
    const ConvexHullView<Scalar> placed = hull_of(corners, Vector3{0.0, -lowest - 0.01, 0.0}, tilt);

    const ContactManifold<Scalar> manifold = generate_convex_manifold<Scalar>(
        ground, placed, ground.center, ground.orientation, placed.center, placed.orientation, 0.0);

    ASSERT_EQ(manifold.point_count, 1);
    EXPECT_NEAR(manifold.points[0].separation, -0.01, 1e-4);
}

// Shapes further apart than the contact offset produce nothing; shapes within it
// produce a patch with a positive separation, which is the speculative-contact
// half of §7.5 arriving for free.
TEST(Unit_ConvexManifold, ContactOffsetGovernsGeneration)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const ConvexHullView<Scalar> lower = hull_of(corners, Vector3{0.0, 0.0, 0.0});
    const ConvexHullView<Scalar> upper = hull_of(corners, Vector3{0.0, 1.02, 0.0});

    EXPECT_EQ(generate_convex_manifold<Scalar>(lower, upper, lower.center, lower.orientation,
                                               upper.center, upper.orientation, 0.0)
                  .point_count,
              0);

    const ContactManifold<Scalar> speculative = generate_convex_manifold<Scalar>(
        lower, upper, lower.center, lower.orientation, upper.center, upper.orientation, 0.05);
    ASSERT_EQ(speculative.point_count, 4);
    for (std::size_t i = 0; i < speculative.point_count; ++i)
        EXPECT_NEAR(speculative.points[i].separation, 0.02, 1e-4);
}

// The patch must agree with the primitive path it generalizes: a hull that is a
// box, against a box, has to produce the same manifold the box-box clipper does.
// Two derivations of one answer, so they have to meet.
TEST(Unit_ConvexManifold, HullPatchAgreesWithThePrimitiveBoxClipper)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> lower_box{Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5}, IDENTITY};
    const OrientedBox<Scalar> upper_box{Vector3{0.15, 0.98, -0.1}, Vector3{0.5, 0.5, 0.5},
                                        IDENTITY};
    const ConvexHullView<Scalar> lower_hull = hull_of(corners, lower_box.center);
    const ConvexHullView<Scalar> upper_hull = hull_of(corners, upper_box.center);

    const ContactManifold<Scalar> primitive = generate_obb_obb_manifold(lower_box, upper_box);
    const ContactManifold<Scalar> general = generate_convex_manifold<Scalar>(
        lower_hull, upper_hull, lower_box.center, IDENTITY, upper_box.center, IDENTITY, 0.0);

    ASSERT_EQ(primitive.point_count, 4);
    ASSERT_EQ(general.point_count, 4);
    EXPECT_NEAR(length(primitive.normal - general.normal), 0.0, 1e-5);

    // Same four points, in whatever order each derivation happened to produce.
    for (std::size_t i = 0; i < 4; ++i)
    {
        const Vector3 target =
            to_world_anchor(lower_box.center, IDENTITY, primitive.points[i].anchor_a_local);
        Scalar nearest = 1e30;
        for (std::size_t j = 0; j < 4; ++j)
        {
            const Vector3 candidate =
                to_world_anchor(lower_box.center, IDENTITY, general.points[j].anchor_a_local);
            nearest = std::min(nearest, static_cast<Scalar>(length(candidate - target)));
        }
        EXPECT_LT(nearest, 1e-4) << "primitive point " << i << " has no counterpart";
    }
}

// Feature ids have to be stable across a small settle for warm starting to carry
// the impulses over, and they are the hull's own vertex indices — which do not
// move when the body does.
TEST(Unit_ConvexManifold, FeatureIdsAreStableAcrossASmallSettle)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> ground{Vector3{0.0, -0.5, 0.0}, Vector3{5.0, 0.5, 5.0}, IDENTITY};
    const ConvexHullView<Scalar> before = hull_of(corners, Vector3{0.0, 0.499, 0.0});
    const ConvexHullView<Scalar> after = hull_of(corners, Vector3{0.0005, 0.4985, 0.0});

    ContactManifold<Scalar> previous = generate_convex_manifold<Scalar>(
        ground, before, ground.center, IDENTITY, before.center, IDENTITY, 0.0);
    ContactManifold<Scalar> current = generate_convex_manifold<Scalar>(
        ground, after, ground.center, IDENTITY, after.center, IDENTITY, 0.0);
    ASSERT_EQ(previous.point_count, 4);
    ASSERT_EQ(current.point_count, 4);

    for (std::size_t i = 0; i < 4; ++i)
        previous.points[i].normal_lambda = 3.0 + static_cast<Scalar>(i);
    warm_start_manifold(current, previous);

    Scalar inherited = 0.0;
    for (std::size_t i = 0; i < 4; ++i)
        inherited += current.points[i].normal_lambda;
    // All four matched, so the whole of the previous tick's impulse carried over.
    EXPECT_NEAR(inherited, 3.0 + 4.0 + 5.0 + 6.0, 1e-9);
}
