/**************************************************************************/
/* test_narrowphase_dispatch.cpp                                          */
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

// Unit_NarrowphaseDispatch: the shape-pair table (§4.2).
//
// The table's job is not to compute anything — the routines behind it are tested
// on their own — but to make sure every pair reaches the right one. So the checks
// are structural: every convex pairing is registered and every one of them is
// registered in *both* orders; an unregistered pair reports nothing rather than
// being quietly approximated by whatever compiles; and the two orders of a pair
// differ only in which way the normal points.
//
// That last one is the property the old code broke without anyone noticing:
// `gather_rigid_descs` collapsed anything that was not a box into a sphere, so a
// cylinder simulated as a ball and nothing said so (§1.2 item 4).

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    std::vector<Vector3> box_vertices(Vector3 half_extents)
    {
        std::vector<Vector3> vertices;
        for (int i = 0; i < 8; ++i)
            vertices.push_back(Vector3{(i & 1) ? half_extents.x : -half_extents.x,
                                       (i & 2) ? half_extents.y : -half_extents.y,
                                       (i & 4) ? half_extents.z : -half_extents.z});
        return vertices;
    }

    /** @brief The world point of a manifold point on body a. */
    Vector3 world_a(const ContactManifold<Scalar>& manifold, std::size_t i,
                    const CollisionShape<Scalar>& shape)
    {
        return to_world_anchor(shape.center, shape.orientation, manifold.points[i].anchor_a_local);
    }
} // namespace

// Every convex shape must reach every other convex shape and the ground, in both
// orders. A hole here is a pair of bodies that silently pass through each other.
TEST(Unit_NarrowphaseDispatch, EveryConvexPairingIsRegisteredInBothOrders)
{
    const NarrowphaseTable<Scalar>& table = narrowphase_table<Scalar>();
    const ShapeType convex[] = {ShapeType::sphere, ShapeType::oriented_box, ShapeType::capsule,
                                ShapeType::convex_hull};

    for (const ShapeType a : convex)
    {
        for (const ShapeType b : convex)
            EXPECT_NE(table.get(a, b), nullptr)
                << "missing " << static_cast<int>(a) << " vs " << static_cast<int>(b);
        EXPECT_NE(table.get(a, ShapeType::plane), nullptr);
        EXPECT_NE(table.get(ShapeType::plane, a), nullptr);
    }
}

// Two half-spaces have no contact to report, and an axis-aligned box is an
// oriented box with the identity orientation rather than a second code path — so
// both are deliberately absent, and absence must mean "no contact" rather than
// "something plausible".
TEST(Unit_NarrowphaseDispatch, UnregisteredPairsReportNothing)
{
    const NarrowphaseTable<Scalar>& table = narrowphase_table<Scalar>();
    EXPECT_EQ(table.get(ShapeType::plane, ShapeType::plane), nullptr);
    EXPECT_EQ(table.get(ShapeType::box, ShapeType::sphere), nullptr);

    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);
    const CollisionShape<Scalar> ceiling = make_plane_shape(Vector3{0.0, -1.0, 0.0}, -5.0);
    EXPECT_EQ(generate_shape_manifold<Scalar>(ground, ceiling).point_count, 0);
}

// Each pairing, through the table, against the answer geometry gives directly.
TEST(Unit_NarrowphaseDispatch, EachPairingProducesTheExpectedContact)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);

    // A box resting flat on the ground: four points, five millimetres in.
    const CollisionShape<Scalar> box =
        make_box_shape(Vector3{0.0, 0.495, 0.0}, Vector3{0.5, 0.5, 0.5});
    const ContactManifold<Scalar> box_ground = generate_shape_manifold<Scalar>(box, ground);
    ASSERT_EQ(box_ground.point_count, 4);
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(box_ground.points[i].separation, -0.005, 1e-9);

    // A sphere on the ground: one point, whatever the depth.
    const CollisionShape<Scalar> ball = make_sphere_shape(Vector3{0.0, 0.45, 0.0}, 0.5);
    const ContactManifold<Scalar> ball_ground = generate_shape_manifold<Scalar>(ball, ground);
    ASSERT_EQ(ball_ground.point_count, 1);
    EXPECT_NEAR(ball_ground.points[0].separation, -0.05, 1e-9);

    // A capsule lying on the ground: its two ends.
    const CollisionShape<Scalar> capsule =
        make_capsule_shape(Vector3{0.0, 0.29, 0.0}, 1.0, 0.3,
                           quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, PI / 2.0));
    const ContactManifold<Scalar> capsule_ground =
        generate_shape_manifold<Scalar>(capsule, ground);
    ASSERT_EQ(capsule_ground.point_count, 2);
    for (std::size_t i = 0; i < 2; ++i)
        EXPECT_NEAR(capsule_ground.points[i].separation, -0.01, 1e-9);

    // A hull on the ground: the face it rests on.
    const CollisionShape<Scalar> hull =
        make_hull_shape(Vector3{0.0, 0.49, 0.0}, corners.data(), 8);
    const ContactManifold<Scalar> hull_ground = generate_shape_manifold<Scalar>(hull, ground);
    ASSERT_EQ(hull_ground.point_count, 4);
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(hull_ground.points[i].separation, -0.01, 1e-9);

    // A sphere resting on a box: one point, and the normal runs box -> sphere.
    const CollisionShape<Scalar> pedestal =
        make_box_shape(Vector3{0.0, -0.5, 0.0}, Vector3{1.0, 0.5, 1.0});
    const CollisionShape<Scalar> perched = make_sphere_shape(Vector3{0.0, 0.45, 0.0}, 0.5);
    const ContactManifold<Scalar> mixed = generate_shape_manifold<Scalar>(pedestal, perched);
    ASSERT_EQ(mixed.point_count, 1);
    EXPECT_NEAR(mixed.normal.y, 1.0, 1e-5);
    EXPECT_NEAR(mixed.points[0].separation, -0.05, 1e-5);

    // A capsule standing on a hull, which exercises the pairing with no primitive
    // routine behind it at all.
    const CollisionShape<Scalar> standing = make_capsule_shape(Vector3{0.0, 1.79, 0.0}, 1.0, 0.3);
    const CollisionShape<Scalar> base = make_hull_shape(Vector3{0.0, 0.0, 0.0}, corners.data(), 8);
    const ContactManifold<Scalar> capsule_hull = generate_shape_manifold<Scalar>(base, standing);
    ASSERT_GE(capsule_hull.point_count, 1);
    EXPECT_NEAR(capsule_hull.normal.y, 1.0, 1e-4);
    EXPECT_NEAR(capsule_hull.points[0].separation, -0.01, 1e-4);
}

// The ordered pair is the table's key, so both orders must resolve and they must
// describe the same contact — a caller must never have to sort its two bodies
// before asking, and two bodies must not get different physics depending on which
// index came first.
TEST(Unit_NarrowphaseDispatch, SwappingTheArgumentsOnlyFlipsTheContact)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);
    const CollisionShape<Scalar> box =
        make_box_shape(Vector3{0.0, 0.49, 0.0}, Vector3{0.5, 0.5, 0.5});
    const CollisionShape<Scalar> hull =
        make_hull_shape(Vector3{0.2, 1.48, 0.0}, corners.data(), 8);

    // Convex against a plane, both ways.
    const ContactManifold<Scalar> forward = generate_shape_manifold<Scalar>(box, ground);
    const ContactManifold<Scalar> reversed = generate_shape_manifold<Scalar>(ground, box);
    ASSERT_EQ(forward.point_count, reversed.point_count);
    EXPECT_NEAR(length(forward.normal + reversed.normal), 0.0, 1e-12);
    for (std::size_t i = 0; i < forward.point_count; ++i)
    {
        EXPECT_NEAR(forward.points[i].separation, reversed.points[i].separation, 1e-12);
        // The anchors swapped sides with the bodies.
        EXPECT_NEAR(length(forward.points[i].anchor_a_local - reversed.points[i].anchor_b_local),
                    0.0, 1e-12);
    }

    // Convex against convex, both ways.
    const ContactManifold<Scalar> up = generate_shape_manifold<Scalar>(box, hull);
    const ContactManifold<Scalar> down = generate_shape_manifold<Scalar>(hull, box);
    ASSERT_EQ(up.point_count, down.point_count);
    ASSERT_GT(up.point_count, 0);
    EXPECT_NEAR(length(up.normal + down.normal), 0.0, 1e-5);
    for (std::size_t i = 0; i < up.point_count; ++i)
    {
        const Vector3 point = world_a(up, i, box);
        Scalar nearest = 1e30;
        for (std::size_t j = 0; j < down.point_count; ++j)
        {
            const Vector3 candidate =
                to_world_anchor(box.center, box.orientation, down.points[j].anchor_b_local);
            nearest = std::min(nearest, static_cast<Scalar>(length(candidate - point)));
        }
        EXPECT_LT(nearest, 1e-4);
    }
}

// The table is the contract the extract will consume, so the factories have to
// produce shapes the table can actually key on — including the decision that an
// axis-aligned box is an oriented box.
TEST(Unit_NarrowphaseDispatch, FactoriesProduceTheKindsTheTableKnows)
{
    const std::vector<Vector3> corners = box_vertices(Vector3{0.5, 0.5, 0.5});
    EXPECT_EQ(make_sphere_shape(Vector3{0.0, 0.0, 0.0}, 1.0).type, ShapeType::sphere);
    EXPECT_EQ(make_box_shape(Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 1.0, 1.0}).type,
              ShapeType::oriented_box);
    EXPECT_EQ(make_capsule_shape(Vector3{0.0, 0.0, 0.0}, 1.0, 0.5).type, ShapeType::capsule);
    EXPECT_EQ(make_hull_shape(Vector3{0.0, 0.0, 0.0}, corners.data(), 8).type,
              ShapeType::convex_hull);
    EXPECT_EQ(make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0).type, ShapeType::plane);
}

// Shapes that miss report nothing, and shapes within the contact offset report a
// positive separation — the speculative-contact contract, holding through the
// table as well as through each routine.
TEST(Unit_NarrowphaseDispatch, ContactOffsetPassesThroughTheTable)
{
    const CollisionShape<Scalar> ground = make_plane_shape(Vector3{0.0, 1.0, 0.0}, 0.0);
    const CollisionShape<Scalar> aloft =
        make_box_shape(Vector3{0.0, 0.52, 0.0}, Vector3{0.5, 0.5, 0.5});

    EXPECT_EQ(generate_shape_manifold<Scalar>(aloft, ground).point_count, 0);

    const ContactManifold<Scalar> speculative =
        generate_shape_manifold<Scalar>(aloft, ground, 0.05);
    ASSERT_EQ(speculative.point_count, 4);
    for (std::size_t i = 0; i < 4; ++i)
        EXPECT_NEAR(speculative.points[i].separation, 0.02, 1e-9);
}
