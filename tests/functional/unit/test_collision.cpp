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

#include <SushiEngine/physics/collision.hpp>
#include <SushiEngine/physics/contact_solver.hpp>
#include <SushiEngine/physics/rigid_body.hpp>

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
