/**************************************************************************/
/* test_contact_body.cpp                                                  */
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

// Unit_ContactBody: the unified contact view (physics/contact_solver.hpp) that lets
// the rigid and cloth worlds resolve against each other through one pass. Three
// things have to hold for that to be sound: the AABB must enclose the *oriented*
// shape (it feeds the broadphase, so an undersized box means missed contacts), the
// narrowphase dispatch must orient its normal a->b for all four shape pairings
// including the flipped box-second case, and the resolve must split the correction
// by inverse mass so coupling is genuinely two-way.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/narrowphase.hpp>
#include <SushiEngine/physics/collision/contact_solver.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr double PI = 3.14159265358979323846;

    /** @brief A sphere contact body whose position lives in @p position. */
    ContactBody<Scalar> sphere_body(Vector3& position, Scalar radius, Scalar inv_mass)
    {
        ContactBody<Scalar> body;
        body.position = &position;
        body.radius = radius;
        body.inv_mass = inv_mass;
        return body;
    }

    /** @brief Identity, as its own addressable storage for the box helpers below. */
    Quaternion upright_orientation = Quaternion{};

    /**
     * @brief A box contact body whose pose lives in @p position and @p orientation.
     *
     * The pose is held by pointer so a correction writes the caller's storage, which is
     * exactly how the live simulation aims these at its solver buffers.
     */
    ContactBody<Scalar> box_body(Vector3& position, Vector3 half_extents, Scalar inv_mass,
                                 Quaternion& orientation = upright_orientation)
    {
        ContactBody<Scalar> body;
        body.position = &position;
        body.orientation = &orientation;
        body.is_box = true;
        body.half_extents = half_extents;
        body.inv_mass = inv_mass;
        return body;
    }
}

TEST(Unit_ContactBody, SphereAABBIsTheRadiusBox)
{
    Vector3 position{Scalar(1), Scalar(2), Scalar(3)};
    const AABB<Scalar> box = contact_body_aabb(sphere_body(position, Scalar(0.5), Scalar(1)));
    EXPECT_NEAR(double(box.min.x), 0.5, 1e-9);
    EXPECT_NEAR(double(box.max.z), 3.5, 1e-9);
}

TEST(Unit_ContactBody, RotatedBoxAABBGrowsToEncloseTheOrientedShape)
{
    Vector3 position{0, 0, 0};
    const Vector3 half{Scalar(1), Scalar(0.25), Scalar(0.25)};

    const AABB<Scalar> upright = contact_body_aabb(box_body(position, half, Scalar(1)));
    EXPECT_NEAR(double(upright.max.x), 1.0, 1e-9);
    EXPECT_NEAR(double(upright.max.y), 0.25, 1e-9);

    // Rotated 45 degrees about z the long axis leans into y, so the enclosing box must
    // grow on both axes; an AABB that stayed at the unrotated extents would let the
    // broadphase cull a pair the narrowphase would have caught.
    Quaternion tilt = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(PI / 4.0));
    const AABB<Scalar> rotated = contact_body_aabb(box_body(position, half, Scalar(1), tilt));
    const double expected = (1.0 + 0.25) * std::sqrt(0.5);
    EXPECT_NEAR(double(rotated.max.x), expected, 1e-9);
    EXPECT_NEAR(double(rotated.max.y), expected, 1e-9);
    EXPECT_NEAR(double(rotated.max.z), 0.25, 1e-9); // the rotation axis is untouched
}

TEST(Unit_ContactBody, NarrowphaseOrientsTheNormalFromAToBForEveryShapePair)
{
    Vector3 left{0, 0, 0};
    Vector3 right{Scalar(1.5), 0, 0};
    const Vector3 half{Scalar(1), Scalar(1), Scalar(1)};

    // sphere -> sphere
    {
        const Contact<Scalar> c = contact_body_narrowphase(sphere_body(left, Scalar(1), Scalar(1)),
                                                           sphere_body(right, Scalar(1), Scalar(1)));
        ASSERT_TRUE(c.hit);
        EXPECT_NEAR(double(c.normal.x), 1.0, 1e-9);
    }
    // box -> sphere
    {
        const Contact<Scalar> c = contact_body_narrowphase(box_body(left, half, Scalar(1)),
                                                           sphere_body(right, Scalar(1), Scalar(1)));
        ASSERT_TRUE(c.hit);
        EXPECT_NEAR(double(c.normal.x), 1.0, 1e-9);
    }
    // sphere -> box: the implementation tests box-first and flips, so this is the case
    // a sign slip would hide in.
    {
        const Contact<Scalar> c = contact_body_narrowphase(sphere_body(left, Scalar(1), Scalar(1)),
                                                           box_body(right, half, Scalar(1)));
        ASSERT_TRUE(c.hit);
        EXPECT_NEAR(double(c.normal.x), 1.0, 1e-9);
    }
    // box -> box
    {
        const Contact<Scalar> c = contact_body_narrowphase(box_body(left, half, Scalar(1)),
                                                           box_body(right, half, Scalar(1)));
        ASSERT_TRUE(c.hit);
        EXPECT_NEAR(double(c.normal.x), 1.0, 1e-9);
    }
}

TEST(Unit_ContactBody, EqualMassesSplitTheCorrection)
{
    Vector3 left{0, 0, 0};
    Vector3 right{Scalar(1.5), 0, 0};
    ContactBody<Scalar> a = sphere_body(left, Scalar(1), Scalar(1));
    ContactBody<Scalar> b = sphere_body(right, Scalar(1), Scalar(1));

    resolve_contact_bodies(a, b); // overlap 0.5
    EXPECT_NEAR(double(left.x), -0.25, 1e-9);
    EXPECT_NEAR(double(right.x), 1.75, 1e-9);
}

TEST(Unit_ContactBody, InfiniteMassAbsorbsNothing)
{
    Vector3 anchor{0, 0, 0};
    Vector3 free{Scalar(1.5), 0, 0};
    ContactBody<Scalar> a = sphere_body(anchor, Scalar(1), Scalar(0)); // pinned
    ContactBody<Scalar> b = sphere_body(free, Scalar(1), Scalar(1));

    resolve_contact_bodies(a, b);
    EXPECT_NEAR(double(anchor.x), 0.0, 1e-9);
    EXPECT_NEAR(double(free.x), 2.0, 1e-9); // takes the whole 0.5
}

TEST(Unit_ContactBody, TwoPinnedBodiesDoNotMove)
{
    Vector3 left{0, 0, 0};
    Vector3 right{Scalar(1.5), 0, 0};
    ContactBody<Scalar> a = sphere_body(left, Scalar(1), Scalar(0));
    ContactBody<Scalar> b = sphere_body(right, Scalar(1), Scalar(0));

    resolve_contact_bodies(a, b);
    EXPECT_NEAR(double(left.x), 0.0, 1e-9);
    EXPECT_NEAR(double(right.x), 1.5, 1e-9);
}

TEST(Unit_ContactBody, ClothParticlesDoNotSelfCollide)
{
    Vector3 left{0, 0, 0};
    Vector3 right{Scalar(0.5), 0, 0};
    ContactBody<Scalar> a = sphere_body(left, Scalar(1), Scalar(1));
    ContactBody<Scalar> b = sphere_body(right, Scalar(1), Scalar(1));
    // Said as data now, not as a type tag: both particles are on the cloth layer,
    // and the cloth layer does not accept itself (§4.4, §1.2 item 15).
    a.filter = self_excluding_filter(CollisionLayers::cloth);
    b.filter = a.filter;

    resolve_contact_bodies(a, b);
    EXPECT_NEAR(double(left.x), 0.0, 1e-9);
    EXPECT_NEAR(double(right.x), 0.5, 1e-9);

    // A cloth particle against a rigid body still resolves — only self-collision is off.
    b.filter = CollisionFilter{};
    resolve_contact_bodies(a, b);
    EXPECT_LT(double(left.x), -1e-6);
}

TEST(Unit_ContactBody, ClothPushesBackOnARigidBody)
{
    // The two-way coupling claim, at its smallest: a light cloth particle against a
    // heavy rigid body must move the rigid body by a small but non-zero amount.
    Vector3 cloth_position{0, 0, 0};
    Vector3 rigid_position{Scalar(1.5), 0, 0};
    ContactBody<Scalar> cloth = sphere_body(cloth_position, Scalar(1), Scalar(10));
    ContactBody<Scalar> rigid = sphere_body(rigid_position, Scalar(1), Scalar(1));
    cloth.filter = self_excluding_filter(CollisionLayers::cloth);

    resolve_contact_bodies(cloth, rigid);
    EXPECT_LT(double(cloth_position.x), -1e-6);  // the light one moves most
    EXPECT_GT(double(rigid_position.x), 1.5 + 1e-9); // but the heavy one does move
    EXPECT_NEAR(double(rigid_position.x) - 1.5, 0.5 / 11.0, 1e-9);
}

TEST(Unit_ContactBody, PlaneResolveLiftsABodyClearAndLeavesPinnedBodiesAlone)
{
    Vector3 position{0, Scalar(0.2), 0};
    ContactBody<Scalar> body = sphere_body(position, Scalar(0.5), Scalar(1));
    resolve_contact_body_plane(body, PlaneCollider<Scalar>{});
    EXPECT_NEAR(double(position.y), 0.5, 1e-9);

    Vector3 pinned_position{0, Scalar(0.2), 0};
    ContactBody<Scalar> pinned = sphere_body(pinned_position, Scalar(0.5), Scalar(0));
    resolve_contact_body_plane(pinned, PlaneCollider<Scalar>{});
    EXPECT_NEAR(double(pinned_position.y), 0.2, 1e-9);
}

TEST(Unit_ContactBody, PlaneResolveUsesTheOrientedBoxReach)
{
    // Tipped 45 degrees about z a cube rests on an edge sqrt(2)/2 below its centre,
    // so the lift must take it there rather than to its half-extent. Rotation is pinned
    // here (zero inverse inertia) to isolate the reach from the toppling the next test
    // covers.
    Vector3 position{0, 0, 0};
    Quaternion tilt = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(PI / 4.0));
    ContactBody<Scalar> body =
        box_body(position, Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)}, Scalar(1), tilt);

    resolve_contact_body_plane(body, PlaneCollider<Scalar>{});
    EXPECT_NEAR(double(position.y), 0.5 * std::sqrt(2.0), 1e-9);
}

TEST(Unit_ContactBody, ContactAwayFromTheCentreOfMassProducesATurn)
{
    // The whole point of carrying a lever arm: a box caught on one corner by the ground
    // must rotate toward lying flat, not just rise. With zero inverse inertia the same
    // contact may only translate it.
    const Vector3 half{Scalar(0.5), Scalar(0.5), Scalar(0.5)};
    const Quaternion tilt = quaternion_axis_angle(Vector3{0, 0, 1}, Scalar(PI / 6.0));

    Vector3 free_position{0, Scalar(0.3), 0};
    Quaternion free_orientation = tilt;
    ContactBody<Scalar> spinner = box_body(free_position, half, Scalar(1), free_orientation);
    spinner.inv_inertia = Vector3{Scalar(6), Scalar(6), Scalar(6)};
    resolve_contact_body_plane(spinner, PlaneCollider<Scalar>{});
    EXPECT_GT(std::fabs(double(free_orientation.z) - double(tilt.z)), 1e-9)
        << "an off-centre contact must rotate the body";

    Vector3 pinned_position{0, Scalar(0.3), 0};
    Quaternion pinned_orientation = tilt;
    ContactBody<Scalar> slider = box_body(pinned_position, half, Scalar(1), pinned_orientation);
    resolve_contact_body_plane(slider, PlaneCollider<Scalar>{}); // inv_inertia stays zero
    EXPECT_NEAR(double(pinned_orientation.z), double(tilt.z), 1e-12);
    EXPECT_GT(double(pinned_position.y), 0.3);
}

TEST(Unit_ContactBody, ZeroInverseInertiaReproducesThePurelyPositionalSplit)
{
    // The angular path must be a strict extension: with no rotational freedom the pair
    // has to land exactly where the older inverse-mass-only projection put it.
    Vector3 left{0, 0, 0};
    Vector3 right{Scalar(1.5), 0, 0};
    ContactBody<Scalar> a = sphere_body(left, Scalar(1), Scalar(1));
    ContactBody<Scalar> b = sphere_body(right, Scalar(1), Scalar(3));

    resolve_contact_bodies(a, b); // overlap 0.5, weights 1 and 3
    EXPECT_NEAR(double(left.x), -0.125, 1e-9);
    EXPECT_NEAR(double(right.x), 1.875, 1e-9);
}

TEST(Unit_ContactBody, PlaneContactClearsThePenetrationWhateverTheMass)
{
    // The plane path used to carry an extra inv_mass / w factor, which reproduced the
    // older purely-positional behaviour only for a body of unit inverse mass. A heavier
    // body cleared a fraction of its penetration per sweep and a lighter one overshot.
    // One sweep, no rotational freedom: the body must land exactly on the surface for
    // any inverse mass, because a plane is immovable and absorbs none of the correction.
    const double masses[3] = {0.25, 1.0, 4.0};
    for (double inv_mass : masses)
    {
        Vector3 position{0, Scalar(0.3), 0};
        ContactBody<Scalar> body = sphere_body(position, Scalar(0.5), Scalar(inv_mass));
        resolve_contact_body_plane(body, PlaneCollider<Scalar>{});
        EXPECT_NEAR(double(position.y), 0.5, 1e-12)
            << "inverse mass " << inv_mass << " must not change how far a plane pushes";
    }
}

TEST(Unit_ContactBody, PlaneAndPairProjectionsAgreeOnTheSameGeometry)
{
    // A plane is the limit of a pair whose second body has no mobility at all, so the
    // two paths must move the first body identically. They did not: the plane path used
    // a different formula, so the same overlap resolved differently depending on whether
    // the ground was modelled as a half-space or as an immovable body.
    Vector3 plane_position{0, Scalar(0.3), 0};
    ContactBody<Scalar> against_plane = sphere_body(plane_position, Scalar(0.5), Scalar(0.25));
    resolve_contact_body_plane(against_plane, PlaneCollider<Scalar>{});

    // The same sphere against an enormous immovable sphere approximating the ground.
    const Scalar ground_radius = Scalar(1e6);
    Vector3 falling_position{0, Scalar(0.3), 0};
    Vector3 ground_position{0, -ground_radius, 0};
    ContactBody<Scalar> falling = sphere_body(falling_position, Scalar(0.5), Scalar(0.25));
    ContactBody<Scalar> ground = sphere_body(ground_position, ground_radius, Scalar(0));
    resolve_contact_bodies(ground, falling);

    EXPECT_NEAR(double(falling_position.y), double(plane_position.y), 1e-6);
}
