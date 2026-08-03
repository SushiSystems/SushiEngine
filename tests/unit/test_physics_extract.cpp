/**************************************************************************/
/* test_physics_extract.cpp                                               */
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

// The translation from authored components to physics descriptors, tested on its
// own. It used to live three private methods deep in RuntimeSimulation, where
// reaching it meant standing up a whole live world -- so the cases that actually
// bite (a Cylinder collapsing to a sphere, a plane that is also a rigid body, a
// collider overriding a visual shape) were never checked directly.

#include <gtest/gtest.h>

#include <SushiEngine/simulation/physics_extract.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief A live entity at the origin with nothing attached. */
    PhysicsSourceEntity bare_entity(EntityId id)
    {
        PhysicsSourceEntity entity;
        entity.id = id;
        return entity;
    }

    /** @brief A dynamic entity with a primitive collider of @p params. */
    PhysicsSourceEntity body_with_collider(EntityId id, PrimitiveKind kind,
                                           const Vector3& params)
    {
        PhysicsSourceEntity entity = bare_entity(id);
        entity.has_physics_body = true;
        entity.has_collider = true;
        entity.collider_params = ColliderParams{kind, params};
        return entity;
    }
}

TEST(Unit_PhysicsExtract, OnlyEntitiesWithABodyBecomeBodies)
{
    std::vector<PhysicsSourceEntity> entities;
    entities.push_back(bare_entity(1));
    entities.push_back(body_with_collider(2, PrimitiveKind::Box, Vector3{1, 1, 1}));
    entities.push_back(bare_entity(3));

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies(entities);
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_EQ(bodies[0].id, 2u);
}

TEST(Unit_PhysicsExtract, TheDescriptorOrderFollowsTheEntityOrder)
{
    // Load-bearing: the body's slot index is its position in this list, and a
    // constraint names bodies by slot. A reordering here silently rewires the scene.
    std::vector<PhysicsSourceEntity> entities;
    for (EntityId id : {EntityId(7), EntityId(3), EntityId(9)})
        entities.push_back(body_with_collider(id, PrimitiveKind::Sphere, Vector3{1, 1, 1}));

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies(entities);
    ASSERT_EQ(bodies.size(), 3u);
    EXPECT_EQ(bodies[0].id, 7u);
    EXPECT_EQ(bodies[1].id, 3u);
    EXPECT_EQ(bodies[2].id, 9u);
}

TEST(Unit_PhysicsExtract, OnlyABoxCollidesAsABox)
{
    const std::vector<RigidBodyDesc> boxes =
        extract_rigid_bodies({body_with_collider(1, PrimitiveKind::Box, Vector3{2, 3, 4})});
    ASSERT_EQ(boxes.size(), 1u);
    EXPECT_EQ(boxes[0].collider.shape, ColliderShape::Box);
    EXPECT_DOUBLE_EQ(double(boxes[0].collider.half_extents.y), 3.0);

    // A cylinder is a *capsule*, not a sphere (§1.2 item 4). Still an approximation
    // — a capsule has round ends — but one that stands on the ground and rolls about
    // the right axis, which the bounding sphere did neither of. There is no longer a
    // second, flattened description of the shape beside the collider: the live tick
    // generates manifolds from the collider itself, so a capsule collides as one.
    const std::vector<RigidBodyDesc> cylinders = extract_rigid_bodies(
        {body_with_collider(1, PrimitiveKind::Cylinder, Vector3{2, 3, 4})});
    ASSERT_EQ(cylinders.size(), 1u);
    EXPECT_EQ(cylinders[0].collider.shape, ColliderShape::Capsule);
    EXPECT_DOUBLE_EQ(double(cylinders[0].collider.radius), 2.0);
    EXPECT_DOUBLE_EQ(double(cylinders[0].collider.half_height), 1.0) << "3 tall minus a 2 cap";
}

TEST(Unit_PhysicsExtract, ABoxCollidesAtItsSmallestHalfExtent)
{
    // Deliberately an under-approximation: a box that reported its bounding radius
    // would hover above the ground by the difference.
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Box, Vector3{5, 2, 9});
    EXPECT_DOUBLE_EQ(double(collision_radius(entity)), 2.0);
}

TEST(Unit_PhysicsExtract, AColliderOverridesTheVisualShape)
{
    PhysicsSourceEntity entity = bare_entity(1);
    entity.has_physics_body = true;
    entity.has_shape = true;
    entity.shape_params = ShapeParams{PrimitiveKind::Box, Vector3{5, 5, 5}};
    entity.has_collider = true;
    entity.collider_params = ColliderParams{PrimitiveKind::Sphere, Vector3{1, 1, 1}};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_EQ(bodies[0].collider.shape, ColliderShape::Sphere)
        << "the collider decides, not the visual";
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.radius), 1.0);
}

TEST(Unit_PhysicsExtract, TheVisualShapeIsUsedWhenThereIsNoCollider)
{
    PhysicsSourceEntity entity = bare_entity(1);
    entity.has_physics_body = true;
    entity.has_shape = true;
    entity.shape_params = ShapeParams{PrimitiveKind::Box, Vector3{4, 4, 4}};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_EQ(bodies[0].collider.shape, ColliderShape::Box);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.x), 4.0);
}

TEST(Unit_PhysicsExtract, APlaneColliderBecomesAStaticHalfSpaceAtItsWorldTransform)
{
    PhysicsSourceEntity ground = bare_entity(1);
    ground.has_collider = true;
    ground.collider_params = ColliderParams{PrimitiveKind::Plane, Vector3{0, 1, 0}};
    ground.world_position = Vector3{0, Scalar(3), 0};
    ground.local_position = Vector3{0, Scalar(-99), 0}; // the world transform wins

    const std::vector<PlaneDesc> planes = extract_static_planes({ground});
    ASSERT_EQ(planes.size(), 1u);
    EXPECT_DOUBLE_EQ(double(planes[0].point.y), 3.0);
    EXPECT_DOUBLE_EQ(double(planes[0].normal.y), 1.0);
}

TEST(Unit_PhysicsExtract, AMovingPlaneIsNotAStaticSurface)
{
    // An entity with both a plane collider and a rigid body would give the physics a
    // static surface and a moving body describing the same thing, which can only
    // disagree.
    PhysicsSourceEntity moving = bare_entity(1);
    moving.has_collider = true;
    moving.collider_params = ColliderParams{PrimitiveKind::Plane, Vector3{0, 1, 0}};
    moving.has_physics_body = true;

    EXPECT_TRUE(extract_static_planes({moving}).empty());
}

TEST(Unit_PhysicsExtract, ARigidBodySeedsFromItsLocalTransform)
{
    // Preserved from the code this replaced, not chosen here: a body seeds from its
    // own transform while a plane is placed by its world transform. Pinned so that
    // changing it is a decision someone takes, not a side effect of a later edit.
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Sphere, Vector3{1, 1, 1});
    entity.local_position = Vector3{Scalar(1), Scalar(2), Scalar(3)};
    entity.world_position = Vector3{Scalar(90), Scalar(90), Scalar(90)};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_DOUBLE_EQ(double(bodies[0].position.x), 1.0);
    EXPECT_DOUBLE_EQ(double(bodies[0].position.z), 3.0);
}

// ---------------------------------------------------------------------------
// Scale, and the mass derived from it (§1.2 item 5, P0 carry-over 2)
// ---------------------------------------------------------------------------

TEST(Unit_PhysicsExtract, ScalingAnEntityScalesWhatItCollidesAs)
{
    // The bug in one assertion: this crate is drawn twice as wide and used to
    // collide as if it were not.
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Box, Vector3{1, 1, 1});
    entity.local_scale = Vector3{Scalar(2), Scalar(3), Scalar(0.5)};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.x), 2.0);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.y), 3.0);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.z), 0.5);
    EXPECT_DOUBLE_EQ(double(collider_sphere_radius(bodies[0].collider)), 0.5)
        << "still the smallest half-extent";
}

TEST(Unit_PhysicsExtract, ANonUniformlyScaledSphereTakesItsLargestAxis)
{
    // A sphere under a non-uniform scale is an ellipsoid, which the narrowphase
    // does not have. Of the two available answers, the larger keeps the drawing
    // inside the collider rather than poking out of it.
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Sphere, Vector3{1, 1, 1});
    entity.local_scale = Vector3{Scalar(2), Scalar(5), Scalar(3)};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.radius), 5.0);
}

TEST(Unit_PhysicsExtract, AMirroredEntityCollidesAsItsMirrorImage)
{
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Box, Vector3{1, 2, 3});
    entity.local_scale = Vector3{Scalar(-1), Scalar(-1), Scalar(-1)};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.x), 1.0);
    EXPECT_DOUBLE_EQ(double(bodies[0].collider.half_extents.z), 3.0);
}

TEST(Unit_PhysicsExtract, WithoutADensityTheAuthoredMassIsKeptExactly)
{
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Box, Vector3{1, 1, 1});
    entity.physics_params.inv_mass = Scalar(0.25);
    entity.physics_params.inv_inertia = Vector3{Scalar(7), Scalar(8), Scalar(9)};

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    EXPECT_DOUBLE_EQ(double(bodies[0].inv_mass), 0.25);
    EXPECT_DOUBLE_EQ(double(bodies[0].inv_inertia.y), 8.0);
}

TEST(Unit_PhysicsExtract, ADensityDerivesMassFromTheScaledShape)
{
    // Two metres on a side at 1000 kg/m^3 is eight tonnes, and the inertia is the
    // box formula about its own centre. Deriving from the *unscaled* shape would
    // give one tonne — a failure that looks derived, which is worse than the typed
    // number it replaced.
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Box, Vector3{1, 1, 1});
    entity.local_scale = Vector3{Scalar(2), Scalar(2), Scalar(2)};
    entity.physics_params.density = Scalar(1000);
    entity.physics_params.inv_mass = Scalar(1); // ignored

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    const double mass = 4.0 * 4.0 * 4.0 * 1000.0;
    EXPECT_NEAR(double(bodies[0].inv_mass), 1.0 / mass, 1e-12);
    const double inertia = (mass / 3.0) * (2.0 * 2.0 + 2.0 * 2.0);
    EXPECT_NEAR(double(bodies[0].inv_inertia.x), 1.0 / inertia, 1e-12);
}

TEST(Unit_PhysicsExtract, ADensityOnASphereMatchesTheClosedForm)
{
    PhysicsSourceEntity entity = body_with_collider(1, PrimitiveKind::Sphere, Vector3{2, 2, 2});
    entity.physics_params.density = Scalar(500);

    const std::vector<RigidBodyDesc> bodies = extract_rigid_bodies({entity});
    ASSERT_EQ(bodies.size(), 1u);
    const double pi = 3.14159265358979323846;
    const double mass = (4.0 / 3.0) * pi * 8.0 * 500.0;
    EXPECT_NEAR(double(bodies[0].inv_mass), 1.0 / mass, 1e-12);
    EXPECT_NEAR(double(bodies[0].inv_inertia.z), 1.0 / (0.4 * mass * 4.0), 1e-12);
}
