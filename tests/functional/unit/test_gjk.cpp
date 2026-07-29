/**************************************************************************/
/* test_gjk.cpp                                                           */
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

// Unit_Gjk: the general convex narrowphase (physics/geometry/gjk.hpp).
//
// One routine collides every convex shape with every other, so the tests are
// built the same way: each case has a closed-form answer that does not depend on
// GJK at all, and the routine is asked to reproduce it. Sphere-sphere separation
// is a subtraction. A box against a sphere on an axis is a subtraction. A hull
// that *is* a box must agree with the box. Deep overlap has a known depth.
//
// Two properties get their own cases because they are the ones a convex solver
// silently gets wrong: the witness points must lie on the two surfaces (not merely
// be a plausible pair), and the answer must not depend on which shape was passed
// first.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/geometry/gjk.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief The eight corners of a box of the given half-extents, hull-local. */
    std::vector<Vector3> box_vertices(Vector3 half_extents)
    {
        std::vector<Vector3> vertices;
        for (int i = 0; i < 8; ++i)
            vertices.push_back(Vector3{(i & 1) ? half_extents.x : -half_extents.x,
                                       (i & 2) ? half_extents.y : -half_extents.y,
                                       (i & 4) ? half_extents.z : -half_extents.z});
        return vertices;
    }

    /** @brief A hull view over @p vertices, posed in the world. */
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

    /** @brief An upright capsule of the given dimensions. */
    CapsuleCollider<Scalar> capsule_at(Vector3 center, Scalar half_height, Scalar radius,
                                       Quaternion orientation = Quaternion{0.0, 0.0, 0.0, 1.0})
    {
        CapsuleCollider<Scalar> capsule;
        capsule.center = center;
        capsule.orientation = orientation;
        capsule.half_height = half_height;
        capsule.radius = radius;
        return capsule;
    }
} // namespace

// The support function is the only thing GJK asks of a shape, so it is worth
// pinning down on its own: each shape's furthest point along a direction, checked
// against the answer geometry gives directly.
TEST(Unit_Gjk, SupportFunctionsReturnTheFurthestPoint)
{
    const SphereCollider<Scalar> sphere{Vector3{1.0, 2.0, 3.0}, 0.5};
    const Vector3 sphere_support = support(sphere, Vector3{0.0, 3.0, 0.0});
    EXPECT_NEAR(sphere_support.x, 1.0, 1e-12);
    EXPECT_NEAR(sphere_support.y, 2.5, 1e-12);
    EXPECT_NEAR(sphere_support.z, 3.0, 1e-12);

    // A capsule's segment runs along its local Y, so an upright one supports the
    // top cap's pole; a capsule rolled onto its side supports along X instead.
    const CapsuleCollider<Scalar> upright = capsule_at(Vector3{0.0, 0.0, 0.0}, 1.0, 0.25);
    const Vector3 top = support(upright, Vector3{0.0, 1.0, 0.0});
    EXPECT_NEAR(top.y, 1.25, 1e-12);
    const CapsuleCollider<Scalar> lying =
        capsule_at(Vector3{0.0, 0.0, 0.0}, 1.0, 0.25,
                   quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 2.0));
    const Vector3 end = support(lying, Vector3{-1.0, 0.0, 0.0});
    EXPECT_NEAR(end.x, -1.25, 1e-9);
    EXPECT_NEAR(end.y, 0.0, 1e-9);

    // A hull's support is its extreme vertex, plus whatever convex radius it
    // carries — the inflation is part of the shape, not a fudge in the solver.
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    ConvexHullView<Scalar> hull = hull_of(corners, Vector3{2.0, 0.0, 0.0});
    hull.convex_radius = 0.01;
    const Vector3 hull_support = support(hull, Vector3{1.0, 0.0, 0.0});
    EXPECT_NEAR(hull_support.x, 2.51, 1e-12);

    // An empty hull supports its own centre rather than reading past nothing.
    ConvexHullView<Scalar> empty;
    empty.center = Vector3{4.0, 5.0, 6.0};
    const Vector3 empty_support = support(empty, Vector3{1.0, 1.0, 1.0});
    EXPECT_NEAR(empty_support.x, 4.0, 1e-12);
    EXPECT_NEAR(empty_support.y, 5.0, 1e-12);
}

// The simplest closed form there is: two spheres are apart by the distance
// between their centres less their radii, whichever way round they are asked.
TEST(Unit_Gjk, SeparatedSpheresReportTheAnalyticDistance)
{
    const SphereCollider<Scalar> a{Vector3{0.0, 0.0, 0.0}, 1.0};
    const SphereCollider<Scalar> b{Vector3{5.0, 0.0, 0.0}, 1.5};

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(a, b);

    ASSERT_TRUE(contact.valid);
    EXPECT_NEAR(contact.separation, 2.5, 1e-8);
    EXPECT_NEAR(contact.normal.x, 1.0, 1e-8);
    // The witness points are on the two surfaces, facing each other.
    EXPECT_NEAR(contact.point_a.x, 1.0, 1e-7);
    EXPECT_NEAR(contact.point_b.x, 3.5, 1e-7);
}

// And overlapping spheres report the overlap as a negative separation, which is
// the manifold convention so the caller needs no case split.
TEST(Unit_Gjk, OverlappingSpheresReportTheAnalyticDepth)
{
    const SphereCollider<Scalar> a{Vector3{0.0, 0.0, 0.0}, 1.0};
    const SphereCollider<Scalar> b{Vector3{1.6, 0.0, 0.0}, 1.0};

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(a, b);

    ASSERT_TRUE(contact.valid);
    // EPA approximates a curved surface with a polytope, so the depth converges
    // from below rather than exactly: the tolerance here is the faceting, not
    // slack in the test.
    EXPECT_NEAR(contact.separation, -0.4, 1e-4);
    EXPECT_NEAR(contact.normal.x, 1.0, 1e-2);
}

// A hull that happens to be a box must agree with the box shape it duplicates —
// the general routine and the shape's own geometry are two derivations of the
// same answer, and they have to meet.
TEST(Unit_Gjk, ConvexHullAgreesWithTheEquivalentBox)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});

    // Axis aligned first, where the closest feature is a face and the answer is a
    // subtraction: the sphere's surface at x = d - 1, the box's at x = 0.5.
    for (const Scalar gap : {0.75, 0.1, -0.05})
    {
        const SphereCollider<Scalar> probe{Vector3{1.5 + gap, 0.0, 0.0}, 1.0};
        const OrientedBox<Scalar> box{Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5},
                                      Quaternion{0.0, 0.0, 0.0, 1.0}};
        const ConvexHullView<Scalar> hull = hull_of(corners, Vector3{0.0, 0.0, 0.0});

        const ConvexContact<Scalar> from_box = collide_convex<Scalar>(box, probe);
        const ConvexContact<Scalar> from_hull = collide_convex<Scalar>(hull, probe);

        ASSERT_TRUE(from_box.valid);
        ASSERT_TRUE(from_hull.valid);
        EXPECT_NEAR(from_box.separation, gap, 1e-5) << "gap " << gap;
        EXPECT_NEAR(from_hull.separation, gap, 1e-5) << "gap " << gap;
    }

    // Then tilted, where there is no simple closed form for the gap but the two
    // shapes are still the same shape — which is the claim being tested.
    const Quaternion tilt = quaternion_axis_angle(normalize(Vector3{0.3, 1.0, 0.2}), 0.7);
    for (const Scalar x : {2.0, 1.4, 1.15})
    {
        const SphereCollider<Scalar> probe{Vector3{x, 0.2, -0.1}, 1.0};
        const OrientedBox<Scalar> box{Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5}, tilt};
        const ConvexHullView<Scalar> hull = hull_of(corners, Vector3{0.0, 0.0, 0.0}, tilt);

        const ConvexContact<Scalar> from_box = collide_convex<Scalar>(box, probe);
        const ConvexContact<Scalar> from_hull = collide_convex<Scalar>(hull, probe);

        ASSERT_TRUE(from_box.valid);
        ASSERT_TRUE(from_hull.valid);
        EXPECT_NEAR(from_hull.separation, from_box.separation, 1e-4) << "x " << x;
        EXPECT_NEAR(length(from_hull.normal - from_box.normal), 0.0, 1e-3) << "x " << x;
    }
}

// A capsule against a plane-like box is the case a character controller lives in,
// and its answer is a subtraction: the segment's low end, less the radius.
TEST(Unit_Gjk, CapsuleAgainstABoxReportsTheSweptSphereDistance)
{
    const OrientedBox<Scalar> ground{Vector3{0.0, -0.5, 0.0}, Vector3{5.0, 0.5, 5.0},
                                     Quaternion{0.0, 0.0, 0.0, 1.0}};

    // Segment from y = 0.7 to y = 2.7, radius 0.3, so the capsule's lowest point is
    // at y = 0.4 and the ground's top is at y = 0.
    const CapsuleCollider<Scalar> standing = capsule_at(Vector3{0.0, 1.7, 0.0}, 1.0, 0.3);
    const ConvexContact<Scalar> apart = collide_convex<Scalar>(ground, standing);
    ASSERT_TRUE(apart.valid);
    EXPECT_NEAR(apart.separation, 0.4, 1e-7);
    EXPECT_NEAR(apart.normal.y, 1.0, 1e-6);

    // Lowered until it overlaps by five centimetres.
    const CapsuleCollider<Scalar> sunk = capsule_at(Vector3{0.0, 1.25, 0.0}, 1.0, 0.3);
    const ConvexContact<Scalar> overlapping = collide_convex<Scalar>(ground, sunk);
    ASSERT_TRUE(overlapping.valid);
    EXPECT_NEAR(overlapping.separation, -0.05, 1e-5);
    EXPECT_NEAR(overlapping.normal.y, 1.0, 1e-5);
}

// Two capsules crossing at right angles reduce to the distance between two skew
// segments, less the two radii — a closed form GJK has to reproduce.
TEST(Unit_Gjk, CrossedCapsulesReduceToTheSegmentDistance)
{
    const CapsuleCollider<Scalar> vertical = capsule_at(Vector3{0.0, 0.0, 0.0}, 1.0, 0.2);
    // Rotating about Z carries the local Y axis into the XY plane, so this one's
    // segment runs along X at z = 1 — skew to the vertical one, not crossing it.
    const CapsuleCollider<Scalar> horizontal =
        capsule_at(Vector3{0.0, 0.0, 1.0}, 1.0, 0.25,
                   quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 2.0));

    // The segments are 1.0 apart along Z; subtract the radii.
    const ConvexContact<Scalar> contact = collide_convex<Scalar>(vertical, horizontal);
    ASSERT_TRUE(contact.valid);
    EXPECT_NEAR(contact.separation, 1.0 - 0.45, 1e-6);
    EXPECT_NEAR(contact.normal.z, 1.0, 1e-5);
}

// The witness points are the part a convex solver can quietly get wrong: a right
// depth with a pair of points that are not on the surfaces is useless as a
// manifold anchor. Check them against the surfaces directly.
TEST(Unit_Gjk, WitnessPointsLieOnBothSurfaces)
{
    const SphereCollider<Scalar> a{Vector3{-1.2, 0.4, 0.0}, 0.6};
    const CapsuleCollider<Scalar> b =
        capsule_at(Vector3{1.0, 0.0, 0.0}, 0.8, 0.35,
                   quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, 0.6));

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(a, b);
    ASSERT_TRUE(contact.valid);

    // On the sphere: exactly one radius from the centre.
    EXPECT_NEAR(length(contact.point_a - a.center), a.radius, 1e-6);

    // On the capsule: exactly one radius from its segment.
    Vector3 start;
    Vector3 end;
    capsule_segment(b, start, end);
    const Vector3 axis = end - start;
    Scalar t = dot(contact.point_b - start, axis) / dot(axis, axis);
    t = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
    EXPECT_NEAR(length(contact.point_b - (start + axis * t)), b.radius, 1e-6);

    // And the pair is separated by exactly the reported distance, along the normal.
    EXPECT_NEAR(dot(contact.point_b - contact.point_a, contact.normal), contact.separation, 1e-6);
    EXPECT_NEAR(length(contact.point_b - contact.point_a), contact.separation, 1e-6);
}

// Swapping the arguments must swap the normal and nothing else. A convex routine
// that disagrees with itself by argument order gives a pair of bodies two
// different contacts depending on which index came first.
TEST(Unit_Gjk, ArgumentOrderOnlyFlipsTheNormal)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.4, 0.9, 0.3});
    const ConvexHullView<Scalar> hull =
        hull_of(corners, Vector3{0.6, 0.2, -0.1},
                quaternion_axis_angle(normalize(Vector3{1.0, 0.4, -0.2}), 1.1));
    const CapsuleCollider<Scalar> capsule =
        capsule_at(Vector3{-0.5, 0.0, 0.0}, 0.7, 0.3,
                   quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, 0.4));

    const ConvexContact<Scalar> forward = collide_convex<Scalar>(hull, capsule);
    const ConvexContact<Scalar> reversed = collide_convex<Scalar>(capsule, hull);

    ASSERT_TRUE(forward.valid);
    ASSERT_TRUE(reversed.valid);
    EXPECT_NEAR(forward.separation, reversed.separation, 1e-6);
    EXPECT_NEAR(length(forward.normal + reversed.normal), 0.0, 1e-5);
    EXPECT_NEAR(length(forward.point_a - reversed.point_b), 0.0, 1e-5);
    EXPECT_NEAR(length(forward.point_b - reversed.point_a), 1e-30, 1e-5);
}

// Deep overlap is EPA's case rather than GJK's, and it has a closed form too: a
// box sunk halfway into another along one axis.
TEST(Unit_Gjk, DeepOverlapReportsTheShortestWayOut)
{
    const OrientedBox<Scalar> a{Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.2, 1.0},
                                Quaternion{0.0, 0.0, 0.0, 1.0}};
    // Overlaps by 0.15 along Y, and by much more along X and Z, so the shortest
    // way out is +Y.
    const OrientedBox<Scalar> b{Vector3{0.1, 0.45, -0.1}, Vector3{1.0, 0.4, 1.0},
                                Quaternion{0.0, 0.0, 0.0, 1.0}};

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(a, b);

    ASSERT_TRUE(contact.valid);
    EXPECT_NEAR(contact.normal.y, 1.0, 1e-5);
    EXPECT_NEAR(contact.separation, -0.15, 1e-5);
}

// Concentric shapes are the degenerate case that makes a naive EPA divide by
// zero: the origin is deep inside the Minkowski difference and every direction is
// as good as every other. It must still answer with a unit normal and a depth in
// the right place.
//
// The tolerance here is the honest one for EPA against curved shapes, and it is
// worth stating rather than hiding. EPA measures the polytope it has built, and a
// polytope inscribed in a sphere is always short of it; the error is the sagitta
// of a face, so it grows with the depth and shrinks only as the vertex budget
// rises. At a 1.25 m depth on a 64-vertex budget that is a few percent — fine for
// a contact that is going to be resolved over many substeps anyway, and the reason
// deep-penetration recovery in §7.5 belongs to a signed-distance field rather than
// to a hull method.
TEST(Unit_Gjk, ConcentricShapesStillProduceAUsableAnswer)
{
    const SphereCollider<Scalar> a{Vector3{0.0, 0.0, 0.0}, 1.0};
    const SphereCollider<Scalar> b{Vector3{0.0, 0.0, 0.0}, 0.25};

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(a, b);

    ASSERT_TRUE(contact.valid);
    EXPECT_NEAR(length(contact.normal), 1.0, 1e-9);
    EXPECT_LT(contact.separation, -1.1);
    EXPECT_GE(contact.separation, -1.2501);
}

// A hull with a genuinely non-box shape, so the tests are not all secretly about
// boxes: a regular tetrahedron against a sphere on its axis.
TEST(Unit_Gjk, TetrahedronAgainstASphereOnItsVertexAxis)
{
    // A tetrahedron whose apex is at +Y and whose base sits at y = 0.
    std::vector<Vector3> tetrahedron;
    tetrahedron.push_back(Vector3{0.0, 1.0, 0.0});
    tetrahedron.push_back(Vector3{1.0, 0.0, -0.5});
    tetrahedron.push_back(Vector3{-1.0, 0.0, -0.5});
    tetrahedron.push_back(Vector3{0.0, 0.0, 1.0});

    const ConvexHullView<Scalar> hull = hull_of(tetrahedron, Vector3{0.0, 0.0, 0.0});
    // The sphere's lowest point is at y = 2.0 - 0.5 = 1.5; the apex is at 1.0.
    const SphereCollider<Scalar> above{Vector3{0.0, 2.0, 0.0}, 0.5};

    const ConvexContact<Scalar> contact = collide_convex<Scalar>(hull, above);
    ASSERT_TRUE(contact.valid);
    EXPECT_NEAR(contact.separation, 0.5, 1e-7);
    EXPECT_NEAR(contact.normal.y, 1.0, 1e-6);
    EXPECT_NEAR(contact.point_a.y, 1.0, 1e-6);
}

// The reduction is GJK's inner half, and its job is to name the *part* of the
// simplex nearest the origin — a vertex, an edge, a face, or the whole volume.
// Getting that wrong makes GJK converge to the wrong feature.
TEST(Unit_Gjk, SimplexReductionFindsTheNearestFeature)
{
    MinkowskiVertex<Scalar> simplex[4];

    // A segment whose nearest point to the origin is its interior.
    simplex[0].point = Vector3{-1.0, 1.0, 0.0};
    simplex[1].point = Vector3{1.0, 1.0, 0.0};
    SimplexReduction<Scalar> reduction = reduce_simplex(simplex, 2);
    EXPECT_EQ(reduction.count, 2);
    EXPECT_FALSE(reduction.contains_origin);
    EXPECT_NEAR(reduction.closest.x, 0.0, 1e-12);
    EXPECT_NEAR(reduction.closest.y, 1.0, 1e-12);

    // A segment whose nearest point is an endpoint: the reduction still reports
    // two vertices but weights the far one at zero.
    simplex[0].point = Vector3{1.0, 1.0, 0.0};
    simplex[1].point = Vector3{3.0, 1.0, 0.0};
    reduction = reduce_simplex(simplex, 2);
    EXPECT_NEAR(reduction.closest.x, 1.0, 1e-12);
    EXPECT_NEAR(reduction.weights[0], 1.0, 1e-12);

    // A triangle above the origin: the nearest point is inside its face.
    simplex[0].point = Vector3{-1.0, 1.0, -1.0};
    simplex[1].point = Vector3{1.0, 1.0, -1.0};
    simplex[2].point = Vector3{0.0, 1.0, 1.0};
    reduction = reduce_simplex(simplex, 3);
    EXPECT_EQ(reduction.count, 3);
    EXPECT_NEAR(reduction.closest.y, 1.0, 1e-12);
    EXPECT_NEAR(reduction.closest.x, 0.0, 1e-12);

    // A tetrahedron enclosing the origin.
    simplex[0].point = Vector3{1.0, 1.0, 1.0};
    simplex[1].point = Vector3{-1.0, -1.0, 1.0};
    simplex[2].point = Vector3{-1.0, 1.0, -1.0};
    simplex[3].point = Vector3{1.0, -1.0, -1.0};
    reduction = reduce_simplex(simplex, 4);
    EXPECT_TRUE(reduction.contains_origin);

    // The same tetrahedron pushed clear of the origin no longer encloses it, and
    // reduces to the face facing back toward it.
    for (int i = 0; i < 4; ++i)
        simplex[i].point = simplex[i].point + Vector3{0.0, 5.0, 0.0};
    reduction = reduce_simplex(simplex, 4);
    EXPECT_FALSE(reduction.contains_origin);
    // The surviving sub-simplex is whichever feature genuinely carries the closest
    // point — here the bottom edge, not a whole face — because vertices the answer
    // does not rest on are dropped. Keeping them is what stalls GJK's search.
    EXPECT_GE(reduction.count, 1);
    EXPECT_LE(reduction.count, 3);
    EXPECT_GT(reduction.closest.y, 0.0);
    // Whatever it kept, the weights are a partition of unity over it and they
    // reproduce the reported closest point.
    Scalar total = 0.0;
    Vector3 rebuilt{0.0, 0.0, 0.0};
    for (int i = 0; i < reduction.count; ++i)
    {
        total += reduction.weights[i];
        rebuilt = rebuilt + simplex[reduction.indices[i]].point * reduction.weights[i];
    }
    EXPECT_NEAR(total, 1.0, 1e-12);
    EXPECT_NEAR(length(rebuilt - reduction.closest), 0.0, 1e-12);
}
