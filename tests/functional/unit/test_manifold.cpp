/**************************************************************************/
/* test_manifold.cpp                                                      */
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

// Unit_Manifold: contact manifold generation (physics/collision/manifold.hpp).
//
// The claim this file has to check is not "a contact was found" — the old
// single-point narrowphase already did that. It is that a *resting* contact
// produces the whole patch it rests on: four points at the corners of the
// overlap, not one point that a box can rock about. So the tests measure the
// shape of the manifold, not just its existence:
//
//   - a box on a plane and a box on a box yield four points spanning the overlap;
//   - the anchors are stored per body, so the separation re-derived from them
//     after either body moves is right without regenerating anything;
//   - the feature ids are stable across a small motion, which is the entire
//     precondition for warm starting carrying the impulses over;
//   - clipping and reduction behave on the edges: an edge-edge crossing is one
//     point (correctly), a reduced set keeps the deepest point and the widest
//     spread, and the choice does not depend on input order beyond a documented
//     tie-break.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/manifold.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief A unit-cube-ish box at @p center with no rotation. */
    OrientedBox<Scalar> upright_box(Vector3 center, Vector3 half_extents)
    {
        return OrientedBox<Scalar>{center, half_extents, Quaternion{0.0, 0.0, 0.0, 1.0}};
    }

    /** @brief The world-space point of a manifold point on body a. */
    Vector3 world_a(const ContactPoint<Scalar>& point, const OrientedBox<Scalar>& box)
    {
        return to_world_anchor(box.center, box.orientation, point.anchor_a_local);
    }

    /** @brief The largest pairwise distance between the manifold's points on body a. */
    Scalar spread(const ContactManifold<Scalar>& manifold, const OrientedBox<Scalar>& box)
    {
        Scalar largest = 0.0;
        for (std::size_t i = 0; i < manifold.point_count; ++i)
            for (std::size_t j = i + 1; j < manifold.point_count; ++j)
            {
                const Vector3 delta = world_a(manifold.points[i], box) -
                                      world_a(manifold.points[j], box);
                largest = std::max(largest, static_cast<Scalar>(length(delta)));
            }
        return largest;
    }
} // namespace

// A box resting flat on the ground is the case the old single-point narrowphase
// got wrong: it reported one corner, so the box rocked. The manifold must be the
// whole face — four points, one per corner, each just touching.
TEST(Unit_Manifold, BoxOnPlaneYieldsFourCornerPoints)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    // Half a centimetre into the ground, so every corner is genuinely in contact.
    const OrientedBox<Scalar> box = upright_box(Vector3{0.0, 0.495, 0.0}, Vector3{0.5, 0.5, 0.5});

    const ContactManifold<Scalar> manifold = generate_obb_plane_manifold(box, ground);

    ASSERT_EQ(manifold.point_count, 4);
    // The normal runs from the box (a) toward the plane (b), so resolving pushes
    // the box back up along +Y.
    EXPECT_NEAR(manifold.normal.y, -1.0, 1e-12);

    for (std::size_t i = 0; i < manifold.point_count; ++i)
    {
        EXPECT_NEAR(manifold.points[i].separation, -0.005, 1e-12);
        // Each point is a corner of the bottom face: |x| = |z| = 0.5.
        const Vector3 point = world_a(manifold.points[i], box);
        EXPECT_NEAR(std::abs(point.x), 0.5, 1e-12);
        EXPECT_NEAR(std::abs(point.z), 0.5, 1e-12);
        EXPECT_NEAR(point.y, -0.005, 1e-12);
    }
    // The four points span the face's diagonal, not a corner's neighbourhood.
    EXPECT_NEAR(spread(manifold, box), std::sqrt(2.0), 1e-9);
}

// A box tipped up on one edge genuinely touches along a line, and the manifold
// must say so: two points, not four, and both on the low edge.
TEST(Unit_Manifold, TiltedBoxOnPlaneYieldsTheTouchingEdge)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const Scalar angle = PI / 6.0;
    const Quaternion tilt = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, angle);
    // Lower the box until only its lowest edge crosses the ground.
    const Scalar half_height = 0.5 * (std::sin(angle) + std::cos(angle));
    const OrientedBox<Scalar> box{Vector3{0.0, half_height - 0.01, 0.0},
                                  Vector3{0.5, 0.5, 0.5}, tilt};

    const ContactManifold<Scalar> manifold = generate_obb_plane_manifold(box, ground);

    ASSERT_EQ(manifold.point_count, 2);
    for (std::size_t i = 0; i < manifold.point_count; ++i)
        EXPECT_LT(manifold.points[i].separation, 0.0);
    // The two points are the ends of the touching edge, a unit apart along Z.
    EXPECT_NEAR(spread(manifold, box), 1.0, 1e-9);
}

// Two boxes stacked face to face: the manifold is the square they share, and it
// is four points however the top box is offset within the bottom one's face.
TEST(Unit_Manifold, StackedBoxesYieldTheOverlapSquare)
{
    const OrientedBox<Scalar> lower = upright_box(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> upper = upright_box(Vector3{0.2, 0.99, 0.0}, Vector3{0.5, 0.5, 0.5});

    const ContactManifold<Scalar> manifold = generate_obb_obb_manifold(lower, upper);

    ASSERT_EQ(manifold.point_count, 4);
    EXPECT_NEAR(manifold.normal.y, 1.0, 1e-9);
    for (std::size_t i = 0; i < manifold.point_count; ++i)
    {
        EXPECT_NEAR(manifold.points[i].separation, -0.01, 1e-9);
        const Vector3 point = world_a(manifold.points[i], lower);
        // The clipped patch is the intersection of the two faces in x: the top box
        // is shifted by 0.2, so the overlap runs x in [-0.3, 0.5].
        EXPECT_GE(point.x, -0.3 - 1e-9);
        EXPECT_LE(point.x, 0.5 + 1e-9);
        EXPECT_NEAR(std::abs(point.z), 0.5, 1e-9);
    }
}

// Two boxes rotated 45 degrees about different axes cross edge to edge, which is
// genuinely a single point of contact. A manifold that invented more would be
// inventing geometry.
TEST(Unit_Manifold, CrossedEdgesYieldOnePoint)
{
    const Quaternion roll = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 4.0);
    const Quaternion pitch = quaternion_axis_angle(Vector3{1.0, 0.0, 0.0}, PI / 4.0);
    const OrientedBox<Scalar> a{Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5}, roll};
    const Scalar reach = 0.5 * std::sqrt(2.0);
    const OrientedBox<Scalar> b{Vector3{0.0, 2.0 * reach - 0.02, 0.0}, Vector3{0.5, 0.5, 0.5},
                                pitch};

    const ContactManifold<Scalar> manifold = generate_obb_obb_manifold(a, b);

    ASSERT_EQ(manifold.point_count, 1);
    EXPECT_NEAR(manifold.normal.y, 1.0, 1e-9);
    EXPECT_NEAR(manifold.points[0].separation, -0.02, 1e-9);
}

// The point of storing two anchors instead of one world point: the separation is
// still right after a body moves, with no narrowphase call in between. This is
// what lets collision run once per tick and the solve run 32 times.
TEST(Unit_Manifold, RefreshDerivesSeparationFromAnchorsAfterMotion)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    OrientedBox<Scalar> box = upright_box(Vector3{0.0, 0.49, 0.0}, Vector3{0.5, 0.5, 0.5});

    ContactManifold<Scalar> manifold = generate_obb_plane_manifold(box, ground);
    ASSERT_EQ(manifold.point_count, 4);

    // Lift the box by two centimetres; the contact should now report a gap of one.
    box.center.y += 0.02;
    refresh_manifold(manifold, box.center, box.orientation, Vector3{0.0, 0.0, 0.0},
                     Quaternion{0.0, 0.0, 0.0, 1.0});
    for (std::size_t i = 0; i < manifold.point_count; ++i)
        EXPECT_NEAR(manifold.points[i].separation, 0.01, 1e-12);

    // And push it two centimetres down from where it started: deeper by the same.
    box.center.y -= 0.04;
    refresh_manifold(manifold, box.center, box.orientation, Vector3{0.0, 0.0, 0.0},
                     Quaternion{0.0, 0.0, 0.0, 1.0});
    for (std::size_t i = 0; i < manifold.point_count; ++i)
        EXPECT_NEAR(manifold.points[i].separation, -0.03, 1e-12);
}

// Warm starting is only as good as the matching, and the matching is the feature
// id. A box that settles a little between two ticks must produce the same four
// ids, so the impulses that were holding it up carry over.
TEST(Unit_Manifold, FeatureIdsAreStableAcrossSmallMotion)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const OrientedBox<Scalar> before = upright_box(Vector3{0.0, 0.499, 0.0}, Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> after = upright_box(Vector3{0.001, 0.4985, 0.0}, Vector3{0.5, 0.5, 0.5});

    ContactManifold<Scalar> previous = generate_obb_plane_manifold(before, ground);
    ContactManifold<Scalar> current = generate_obb_plane_manifold(after, ground);
    ASSERT_EQ(previous.point_count, 4);
    ASSERT_EQ(current.point_count, 4);

    for (std::size_t i = 0; i < 4; ++i)
    {
        previous.points[i].normal_lambda = 10.0 + static_cast<Scalar>(i);
        previous.points[i].tangent_lambda[0] = 1.0;
        EXPECT_EQ(current.points[i].feature_id, previous.points[i].feature_id);
    }

    warm_start_manifold(current, previous);
    for (std::size_t i = 0; i < 4; ++i)
    {
        EXPECT_NEAR(current.points[i].normal_lambda, 10.0 + static_cast<Scalar>(i), 1e-12);
        EXPECT_NEAR(current.points[i].tangent_lambda[0], 1.0, 1e-12);
    }
}

// A point with no counterpart last tick has no history to inherit. Starting it
// from someone else's impulse is how a stack kicks on the tick a crate lands.
TEST(Unit_Manifold, WarmStartLeavesUnmatchedPointsAtZero)
{
    ContactManifold<Scalar> previous;
    previous.point_count = 1;
    previous.points[0].feature_id = make_feature_id(0, 1, 2, false);
    previous.points[0].normal_lambda = 7.0;

    ContactManifold<Scalar> current;
    current.point_count = 2;
    current.points[0].feature_id = make_feature_id(0, 1, 2, false);
    current.points[1].feature_id = make_feature_id(0, 1, 3, false);

    warm_start_manifold(current, previous);

    EXPECT_NEAR(current.points[0].normal_lambda, 7.0, 1e-12);
    EXPECT_NEAR(current.points[1].normal_lambda, 0.0, 1e-12);
}

// Feature ids must separate the things they name; two contacts that differ only
// in which shape owned the reference face are not the same contact.
TEST(Unit_Manifold, FeatureIdsDistinguishTheirFields)
{
    EXPECT_NE(make_feature_id(0, 0, 0, false), make_feature_id(1, 0, 0, false));
    EXPECT_NE(make_feature_id(0, 0, 0, false), make_feature_id(0, 1, 0, false));
    EXPECT_NE(make_feature_id(0, 0, 0, false), make_feature_id(0, 0, 1, false));
    EXPECT_NE(make_feature_id(0, 0, 0, false), make_feature_id(0, 0, 0, true));
    EXPECT_EQ(make_feature_id(3, 5, 7, true), make_feature_id(3, 5, 7, true));
}

// Sutherland–Hodgman, on its own: a unit square clipped by a plane through its
// middle keeps half its area and gains two boundary vertices, and those two carry
// the crossing id rather than a corner's.
TEST(Unit_Manifold, ClipPolygonHalvesASquare)
{
    ClippedPoint<Scalar> square[max_clipped_points];
    square[0] = {Vector3{-1.0, 0.0, -1.0}, 0};
    square[1] = {Vector3{1.0, 0.0, -1.0}, 1};
    square[2] = {Vector3{1.0, 0.0, 1.0}, 2};
    square[3] = {Vector3{-1.0, 0.0, 1.0}, 3};

    ClippedPoint<Scalar> clipped[max_clipped_points];
    const std::size_t count = clip_polygon_against_plane(
        square, 4, Vector3{1.0, 0.0, 0.0}, 0.0, 9u, clipped);

    ASSERT_EQ(count, 4u);
    std::size_t crossings = 0;
    for (std::size_t i = 0; i < count; ++i)
    {
        EXPECT_LE(clipped[i].position.x, 1e-12);
        if (clipped[i].vertex_id == 9u)
        {
            ++crossings;
            EXPECT_NEAR(clipped[i].position.x, 0.0, 1e-12);
        }
    }
    EXPECT_EQ(crossings, 2u);
}

// A polygon entirely outside the half-space clips to nothing, which is how the
// generator learns two boxes miss along a side plane.
TEST(Unit_Manifold, ClipPolygonRejectsAnOutsidePolygon)
{
    ClippedPoint<Scalar> square[max_clipped_points];
    square[0] = {Vector3{2.0, 0.0, -1.0}, 0};
    square[1] = {Vector3{3.0, 0.0, -1.0}, 1};
    square[2] = {Vector3{3.0, 0.0, 1.0}, 2};
    square[3] = {Vector3{2.0, 0.0, 1.0}, 3};

    ClippedPoint<Scalar> clipped[max_clipped_points];
    EXPECT_EQ(clip_polygon_against_plane(square, 4, Vector3{1.0, 0.0, 0.0}, 0.0, 9u, clipped),
              0u);
}

// The reduction has one job: out of many candidates, keep the four that hold the
// body most rigidly — the deepest, and then the widest spread — never four points
// bunched along one edge.
TEST(Unit_Manifold, ReductionKeepsTheDeepestPointAndTheWidestSpread)
{
    ClippedPoint<Scalar> candidates[max_clipped_points];
    Scalar separations[max_clipped_points];
    // A square's four corners, plus four points clustered near one of them. The
    // deepest is deliberately one of the cluster, so "keep the deepest" and "keep
    // the corners" are in genuine tension.
    candidates[0] = {Vector3{-1.0, 0.0, -1.0}, 0};
    candidates[1] = {Vector3{1.0, 0.0, -1.0}, 1};
    candidates[2] = {Vector3{1.0, 0.0, 1.0}, 2};
    candidates[3] = {Vector3{-1.0, 0.0, 1.0}, 3};
    candidates[4] = {Vector3{-0.95, 0.0, -0.95}, 4};
    candidates[5] = {Vector3{-0.9, 0.0, -0.95}, 5};
    candidates[6] = {Vector3{-0.95, 0.0, -0.9}, 6};
    candidates[7] = {Vector3{-0.9, 0.0, -0.9}, 7};
    for (std::size_t i = 0; i < max_clipped_points; ++i)
        separations[i] = -0.01;
    separations[5] = -0.5; // the deepest

    std::size_t kept[max_manifold_points];
    const std::size_t count = reduce_manifold_points(candidates, separations, 8, kept);

    ASSERT_EQ(count, 4u);
    EXPECT_EQ(kept[0], 5u); // the deepest survives by construction
    // The other three are the far corners, so the kept set spans the square rather
    // than the cluster.
    Scalar largest = 0.0;
    for (std::size_t i = 0; i < count; ++i)
        for (std::size_t j = i + 1; j < count; ++j)
        {
            const Vector3 delta = candidates[kept[i]].position - candidates[kept[j]].position;
            largest = std::max(largest, static_cast<Scalar>(length(delta)));
        }
    EXPECT_GT(largest, 2.5);
}

// Four or fewer candidates are all kept, in order: there is nothing to choose
// between and reordering them would churn the feature ids for no reason.
TEST(Unit_Manifold, ReductionKeepsEverythingWhenItFits)
{
    ClippedPoint<Scalar> candidates[max_clipped_points];
    Scalar separations[max_clipped_points];
    for (std::size_t i = 0; i < 3; ++i)
    {
        candidates[i] = {Vector3{static_cast<Scalar>(i), 0.0, 0.0}, static_cast<std::uint32_t>(i)};
        separations[i] = -0.01;
    }

    std::size_t kept[max_manifold_points];
    ASSERT_EQ(reduce_manifold_points(candidates, separations, 3, kept), 3u);
    EXPECT_EQ(kept[0], 0u);
    EXPECT_EQ(kept[1], 1u);
    EXPECT_EQ(kept[2], 2u);
}

// Speculative contacts (§7.5): a pair that is close but not touching still
// produces a manifold, with a positive separation. The solver reads that as "do
// not cross this surface", which is what stops a fast body passing through it.
TEST(Unit_Manifold, ContactOffsetGeneratesSeparatedContacts)
{
    const OrientedBox<Scalar> lower = upright_box(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> upper = upright_box(Vector3{0.0, 1.02, 0.0}, Vector3{0.5, 0.5, 0.5});

    EXPECT_EQ(generate_obb_obb_manifold(lower, upper).point_count, 0);

    const ContactManifold<Scalar> speculative = generate_obb_obb_manifold(lower, upper, 0.05);
    ASSERT_EQ(speculative.point_count, 4);
    for (std::size_t i = 0; i < speculative.point_count; ++i)
        EXPECT_NEAR(speculative.points[i].separation, 0.02, 1e-9);
}

// The sphere pairings keep their single point — two spheres touch at one place
// however deep they are — but they now report it in the manifold's anchor form,
// so the solver has one code path rather than two.
TEST(Unit_Manifold, SpherePairingsYieldOneAnchoredPoint)
{
    const SphereCollider<Scalar> a{Vector3{0.0, 0.0, 0.0}, 1.0};
    const SphereCollider<Scalar> b{Vector3{1.9, 0.0, 0.0}, 1.0};

    const ContactManifold<Scalar> spheres = generate_sphere_sphere_manifold(a, b);
    ASSERT_EQ(spheres.point_count, 1);
    EXPECT_NEAR(spheres.normal.x, 1.0, 1e-12);
    EXPECT_NEAR(spheres.points[0].separation, -0.1, 1e-12);
    EXPECT_NEAR(spheres.points[0].anchor_a_local.x, 1.0, 1e-12);
    EXPECT_NEAR(spheres.points[0].anchor_b_local.x, -1.0, 1e-12);

    const OrientedBox<Scalar> box = upright_box(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    const SphereCollider<Scalar> resting{Vector3{0.0, 0.9, 0.0}, 0.5};
    const ContactManifold<Scalar> mixed = generate_obb_sphere_manifold(box, resting);
    ASSERT_EQ(mixed.point_count, 1);
    EXPECT_NEAR(mixed.normal.y, 1.0, 1e-12);
    EXPECT_NEAR(mixed.points[0].separation, -0.1, 1e-12);
}

// Boxes that genuinely miss report nothing at all, offset or no offset — the
// generator must not manufacture a contact out of a separating axis it failed to
// prefer.
TEST(Unit_Manifold, SeparatedBoxesYieldNoManifold)
{
    const OrientedBox<Scalar> a = upright_box(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    const OrientedBox<Scalar> b = upright_box(Vector3{3.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});

    EXPECT_EQ(generate_obb_obb_manifold(a, b).point_count, 0);
    EXPECT_EQ(generate_obb_obb_manifold(a, b, 0.1).point_count, 0);

    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const OrientedBox<Scalar> aloft = upright_box(Vector3{0.0, 5.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    EXPECT_EQ(generate_obb_plane_manifold(aloft, ground).point_count, 0);
}
