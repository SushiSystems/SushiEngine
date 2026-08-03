/**************************************************************************/
/* test_collision.cpp                                                    */
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

// Unit_Collision: the narrowphase (physics/collision.hpp) and the positional contact
// resolution (physics/contact_solver.hpp) in isolation. Both are pure host code over
// RigidBody arrays and plain shapes, so no runtime is needed — the contact test even
// runs a full predict/resolve/derive-velocity sub-step loop on the CPU and asserts a
// dropped particle comes to rest exactly on the ground.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/narrowphase.hpp>
#include <SushiEngine/physics/collision/contact_solver.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

TEST(Unit_Collision, SpherePlanePenetration)
{
    SphereCollider<Scalar> sphere{Vector3{0, Scalar(0.3), 0}, Scalar(0.5)};
    PlaneCollider<Scalar> plane; // y-up at origin
    const Contact<Scalar> contact = collide_sphere_plane(sphere, plane);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 0.2, 1e-4);
    EXPECT_NEAR(double(contact.normal.y), 1.0, 1e-4);
}

TEST(Unit_Collision, SpherePlaneClear)
{
    SphereCollider<Scalar> sphere{Vector3{0, Scalar(2.0), 0}, Scalar(0.5)};
    PlaneCollider<Scalar> plane;
    EXPECT_FALSE(collide_sphere_plane(sphere, plane).hit);
}

TEST(Unit_Collision, SphereSphereOverlap)
{
    SphereCollider<Scalar> a{Vector3{0, 0, 0}, Scalar(1)};
    SphereCollider<Scalar> b{Vector3{Scalar(1.5), 0, 0}, Scalar(1)};
    const Contact<Scalar> contact = collide_sphere_sphere(a, b);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 0.5, 1e-4);
    EXPECT_NEAR(double(contact.normal.x), 1.0, 1e-4);
}

TEST(Unit_Collision, BoxPlanePenetration)
{
    BoxCollider<Scalar> box{Vector3{0, Scalar(0.4), 0}, Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)}};
    PlaneCollider<Scalar> plane;
    const Contact<Scalar> contact = collide_box_plane(box, plane);
    ASSERT_TRUE(contact.hit);
    // Box half-height 0.5, centre at 0.4 -> penetration 0.5 - 0.4 = 0.1.
    EXPECT_NEAR(double(contact.depth), 0.1, 1e-4);
}

TEST(Unit_Collision, OBBPlaneAxisAlignedRestsOnItsFace)
{
    OrientedBox<Scalar> box;
    box.center = Vector3{0, Scalar(0.4), 0};
    box.half_extents = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
    const PlaneCollider<Scalar> plane;

    const Contact<Scalar> contact = collide_obb_plane(box, plane);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 0.1, 1e-6);
    EXPECT_NEAR(double(contact.normal.y), 1.0, 1e-6);
}

TEST(Unit_Collision, OBBPlaneRotatedRestsOnItsEdge)
{
    // A cube tipped 45 degrees about z reaches sqrt(2)/2 below its centre rather than
    // its half-extent, because two axes now project onto the plane normal. This is
    // exactly the height a sphere fallback gets wrong.
    OrientedBox<Scalar> box;
    box.half_extents = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
    box.orientation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(3.14159265358979323846 / 4.0));
    const Scalar reach = Scalar(0.5) * std::sqrt(Scalar(2));
    box.center = Vector3{0, reach - Scalar(0.05), 0};

    const Contact<Scalar> contact = collide_obb_plane(box, PlaneCollider<Scalar>{});
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 0.05, 1e-6);

    // Lift it clear of its true reach and the contact must vanish.
    box.center = Vector3{0, reach + Scalar(0.05), 0};
    EXPECT_FALSE(collide_obb_plane(box, PlaneCollider<Scalar>{}).hit);
}

TEST(Unit_Collision, OBBPlaneClear)
{
    OrientedBox<Scalar> box;
    box.center = Vector3{0, Scalar(3), 0};
    EXPECT_FALSE(collide_obb_plane(box, PlaneCollider<Scalar>{}).hit);
}

TEST(Unit_Collision, OBBSphereAgainstFace)
{
    OrientedBox<Scalar> box;
    box.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};
    const SphereCollider<Scalar> sphere{Vector3{Scalar(1.3), 0, 0}, Scalar(0.5)};

    const Contact<Scalar> contact = collide_obb_sphere(box, sphere);
    ASSERT_TRUE(contact.hit);
    // Closest point sits on the +x face at x = 1: gap 0.3, radius 0.5 -> depth 0.2.
    EXPECT_NEAR(double(contact.depth), 0.2, 1e-6);
    EXPECT_NEAR(double(contact.normal.x), 1.0, 1e-6);
    EXPECT_NEAR(double(contact.point.x), 1.0, 1e-6);
}

TEST(Unit_Collision, OBBSphereAgainstEdgeAndCorner)
{
    OrientedBox<Scalar> box;
    box.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};

    // Edge: closest point (1, 1, 0), so the normal runs diagonally in xy.
    const Contact<Scalar> edge =
        collide_obb_sphere(box, SphereCollider<Scalar>{Vector3{Scalar(1.2), Scalar(1.2), 0}, Scalar(0.5)});
    ASSERT_TRUE(edge.hit);
    EXPECT_NEAR(double(edge.depth), 0.5 - std::sqrt(0.08), 1e-6);
    EXPECT_NEAR(double(edge.normal.x), double(edge.normal.y), 1e-6);
    EXPECT_NEAR(double(edge.normal.z), 0.0, 1e-6);

    // Corner: closest point (1, 1, 1), the normal is the body diagonal.
    const Contact<Scalar> corner = collide_obb_sphere(
        box, SphereCollider<Scalar>{Vector3{Scalar(1.2), Scalar(1.2), Scalar(1.2)}, Scalar(0.5)});
    ASSERT_TRUE(corner.hit);
    EXPECT_NEAR(double(corner.depth), 0.5 - std::sqrt(0.12), 1e-6);
    EXPECT_NEAR(double(corner.normal.x), 1.0 / std::sqrt(3.0), 1e-6);
}

TEST(Unit_Collision, OBBSphereClear)
{
    OrientedBox<Scalar> box;
    box.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};
    // Gap along +x is 0.7, wider than the radius.
    EXPECT_FALSE(
        collide_obb_sphere(box, SphereCollider<Scalar>{Vector3{Scalar(1.7), 0, 0}, Scalar(0.5)}).hit);
}

TEST(Unit_Collision, OBBSphereRespectsBoxRotation)
{
    // A box rotated 45 degrees about z reaches sqrt(2) along the world x axis at its
    // corner, so a sphere that clears the unrotated box now touches it.
    OrientedBox<Scalar> box;
    box.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};
    box.orientation = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(3.14159265358979323846 / 4.0));
    const SphereCollider<Scalar> sphere{Vector3{Scalar(1.6), 0, 0}, Scalar(0.5)};

    EXPECT_FALSE(collide_obb_sphere(OrientedBox<Scalar>{box.center, box.half_extents, {}}, sphere).hit);
    const Contact<Scalar> contact = collide_obb_sphere(box, sphere);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 0.5 - (1.6 - std::sqrt(2.0)), 1e-6);
}

TEST(Unit_Collision, OBBObbFaceContactAndSeparation)
{
    OrientedBox<Scalar> a;
    a.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};

    OrientedBox<Scalar> b = a;
    b.center = Vector3{Scalar(1.8), 0, 0};
    const Contact<Scalar> hit = collide_obb_obb(a, b);
    ASSERT_TRUE(hit.hit);
    EXPECT_NEAR(double(hit.depth), 0.2, 1e-6);
    EXPECT_NEAR(double(hit.normal.x), 1.0, 1e-6); // oriented a -> b
    EXPECT_NEAR(double(hit.normal.y), 0.0, 1e-6);

    b.center = Vector3{Scalar(2.2), 0, 0};
    EXPECT_FALSE(collide_obb_obb(a, b).hit);
}

TEST(Unit_Collision, OBBObbNormalFlipsWithArgumentOrder)
{
    OrientedBox<Scalar> a;
    a.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};
    OrientedBox<Scalar> b = a;
    b.center = Vector3{0, Scalar(1.5), 0};

    const Contact<Scalar> ab = collide_obb_obb(a, b);
    const Contact<Scalar> ba = collide_obb_obb(b, a);
    ASSERT_TRUE(ab.hit);
    ASSERT_TRUE(ba.hit);
    EXPECT_NEAR(double(ab.normal.y), 1.0, 1e-6);
    EXPECT_NEAR(double(ba.normal.y), -1.0, 1e-6);
    EXPECT_NEAR(double(ab.depth), double(ba.depth), 1e-9);
}

TEST(Unit_Collision, OBBObbEdgeEdgeSeparatingAxis)
{
    // Two boxes whose faces all overlap in projection but which are separated by an
    // edge-edge cross axis: only the 9 cross products in the SAT can find this.
    const Scalar quarter_turn = Scalar(3.14159265358979323846 / 4.0);
    OrientedBox<Scalar> a;
    a.half_extents = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
    a.orientation = quaternion_axis_angle(Vector3{1, 0, 0}, quarter_turn);

    OrientedBox<Scalar> b;
    b.half_extents = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
    b.orientation = quaternion_axis_angle(Vector3{0, 1, 0}, quarter_turn);
    b.center = Vector3{Scalar(1.28), 0, 0};

    EXPECT_FALSE(collide_obb_obb(a, b).hit);

    // Nudged together they must overlap again, so the miss above is a real separation
    // rather than the axis loop bailing out early.
    b.center = Vector3{Scalar(1.1), 0, 0};
    EXPECT_TRUE(collide_obb_obb(a, b).hit);
}

TEST(Unit_Collision, OBBObbParallelEdgesSkipDegenerateAxes)
{
    // Identical orientations make all nine cross products degenerate; the SAT must
    // skip them rather than treat a zero-length axis as separating.
    OrientedBox<Scalar> a;
    a.half_extents = Vector3{Scalar(1), Scalar(1), Scalar(1)};
    OrientedBox<Scalar> b = a;
    b.center = Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)};

    const Contact<Scalar> contact = collide_obb_obb(a, b);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.depth), 1.5, 1e-6);
}

TEST(Unit_Collision, DroppedParticleRestsOnGround)
{
    RigidBody body;
    body.position = Vector3{0, Scalar(5), 0};
    body.inv_mass = Scalar(1);

    const Scalar radius = Scalar(0.5);
    const PlaneCollider<Scalar> ground; // y = 0
    const Scalar h = Scalar(1.0 / 60.0);
    const Vector3 gravity{0, Scalar(-9.8), 0};

    for (int step = 0; step < 400; ++step)
    {
        predict(body, gravity, h);
        resolve_contacts(&body, &radius, 1, ground, 4);
        update_velocity(body, h);
    }

    // It should be sitting exactly on the ground (centre one radius up) and at rest.
    EXPECT_NEAR(double(body.position.y), 0.5, 1e-2);
    EXPECT_LT(std::fabs(double(body.velocity.y)), 0.2);
}

TEST(Unit_Collision, SphereInsideBoxPushesOutOfTheNearestFace)
{
    // Deep penetration: the sphere centre is inside the box, so clamping to the surface
    // returns the centre itself and there is no closest-point direction. The old code
    // picked +Y unconditionally, which pushed a body sideways through the box it was
    // inside whenever another face was nearer.
    BoxCollider<Scalar> box{Vector3{0, 0, 0}, Vector3{Scalar(2), Scalar(2), Scalar(0.5)}};

    // Nearest face is +Z: the centre is 0.4 from it and 2.0 from the others.
    SphereCollider<Scalar> sphere{Vector3{0, 0, Scalar(0.1)}, Scalar(0.25)};
    const Contact<Scalar> contact = collide_box_sphere(box, sphere);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.normal.z), 1.0, 1e-9);
    EXPECT_NEAR(double(contact.normal.y), 0.0, 1e-9);
    // Radius plus how far in the centre is: 0.25 + (0.5 - 0.1).
    EXPECT_NEAR(double(contact.depth), 0.65, 1e-9);

    // The mirrored case must pick the opposite face, not the same one.
    SphereCollider<Scalar> below{Vector3{0, 0, Scalar(-0.1)}, Scalar(0.25)};
    const Contact<Scalar> mirrored = collide_box_sphere(box, below);
    ASSERT_TRUE(mirrored.hit);
    EXPECT_NEAR(double(mirrored.normal.z), -1.0, 1e-9);
}

TEST(Unit_Collision, SphereInsideOrientedBoxPushesOutOfTheNearestFace)
{
    // The oriented counterpart, and the reason it is tested separately: the normal is
    // chosen in the box's local frame and rotated back out, so a wrong frame would
    // still produce a unit normal and only be visible as a wrong direction.
    const double half_pi = 3.14159265358979323846 * 0.5;
    const Scalar s = Scalar(std::sin(half_pi * 0.5));
    const Scalar c = Scalar(std::cos(half_pi * 0.5));
    // A quarter turn about X maps the box's local +Z onto world +Y.
    OrientedBox<Scalar> box{Vector3{0, 0, 0}, Vector3{Scalar(2), Scalar(2), Scalar(0.5)},
                            Quaternion{s, 0, 0, c}};

    SphereCollider<Scalar> sphere{Vector3{0, Scalar(0.1), 0}, Scalar(0.25)};
    const Contact<Scalar> contact = collide_obb_sphere(box, sphere);
    ASSERT_TRUE(contact.hit);
    EXPECT_NEAR(double(contact.normal.y), 1.0, 1e-6);
    EXPECT_NEAR(double(contact.depth), 0.65, 1e-6);
}
