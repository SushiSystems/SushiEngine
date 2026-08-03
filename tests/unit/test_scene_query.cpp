/**************************************************************************/
/* test_scene_query.cpp                                                   */
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

// Unit_SceneQuery: raycasts, sweeps, overlaps and closest points (§7.7).
//
// Two kinds of assertion here, and the second is the one that earns its keep.
// The closed-form shapes are checked against arithmetic done by hand — a sphere
// of radius one, two metres away, is hit at one metre and nowhere else. The
// shapes with no closed form are checked against the closed-form ones: a capsule
// long enough to look like a sphere at its cap, a hull whose vertices are a box,
// each cast through the general conservative-advancement routine and required to
// agree with the analytic answer. That is what makes the generic path testable at
// all, since its own arithmetic is exactly what is under suspicion.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/bvh_broadphase.hpp>
#include <SushiEngine/physics/collision/scene_query.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;

    Vector3T<Real> vec(Real x, Real y, Real z) { return Vector3T<Real>{x, y, z}; }

    /** @brief A scene of type-erased shapes with a broadphase over them. */
    struct Scene
    {
        BVHBroadphase<Real> broadphase;
        std::vector<CollisionShape<Real>> shapes;

        ProxyId add(const CollisionShape<Real>& shape, std::uint32_t flags = 0,
                    const CollisionFilter& filter = CollisionFilter{})
        {
            const std::uint32_t payload = static_cast<std::uint32_t>(shapes.size());
            shapes.push_back(shape);
            return broadphase.create_proxy(shape_world_bounds(shape), filter, flags, payload);
        }

        /** @brief The shape lookup every query is parameterized on. */
        CollisionShape<Real> shape_of(ProxyId id) const
        {
            return shapes[broadphase.proxy(id).payload];
        }
    };

    const Real tolerance = 1e-5;
} // namespace

// ---------------------------------------------------------------------------
// One ray, one shape
// ---------------------------------------------------------------------------

TEST(Unit_SceneQuery, SphereIsHitAtTheArithmeticDistance)
{
    RayHit<Real> hit;
    ASSERT_TRUE(ray_cast(SphereCollider<Real>{vec(2, 0, 0), 1.0}, vec(0, 0, 0), vec(1, 0, 0), 10.0,
                         hit));
    EXPECT_NEAR(hit.distance, 1.0, tolerance);
    EXPECT_NEAR(hit.point.x, 1.0, tolerance);
    EXPECT_NEAR(hit.normal.x, -1.0, tolerance);

    // Behind the ray, and beyond its reach: both misses.
    EXPECT_FALSE(ray_cast(SphereCollider<Real>{vec(-2, 0, 0), 1.0}, vec(0, 0, 0), vec(1, 0, 0),
                          10.0, hit));
    EXPECT_FALSE(ray_cast(SphereCollider<Real>{vec(2, 0, 0), 1.0}, vec(0, 0, 0), vec(1, 0, 0), 0.5,
                          hit));
}

TEST(Unit_SceneQuery, ARayStartingInsideASphereReportsZeroRatherThanAMiss)
{
    RayHit<Real> hit;
    ASSERT_TRUE(ray_cast(SphereCollider<Real>{vec(0, 0, 0), 1.0}, vec(0.2, 0, 0), vec(1, 0, 0),
                         10.0, hit));
    EXPECT_NEAR(hit.distance, 0.0, tolerance);
}

TEST(Unit_SceneQuery, PlaneIsHitFromAboveAndNotFromBelow)
{
    RayHit<Real> hit;
    ASSERT_TRUE(ray_cast(PlaneCollider<Real>{vec(0, 1, 0), 0.0}, vec(0, 3, 0), vec(0, -1, 0), 10.0,
                         hit));
    EXPECT_NEAR(hit.distance, 3.0, tolerance);
    EXPECT_NEAR(hit.normal.y, 1.0, tolerance);

    EXPECT_FALSE(ray_cast(PlaneCollider<Real>{vec(0, 1, 0), 0.0}, vec(0, -3, 0), vec(0, 1, 0), 10.0,
                          hit));
}

TEST(Unit_SceneQuery, BoxReportsTheFaceItWasEnteredThrough)
{
    const OrientedBox<Real> box{vec(0, 0, 0), vec(1, 2, 3),
                                QuaternionT<Real>{0.0, 0.0, 0.0, 1.0}};
    RayHit<Real> hit;
    ASSERT_TRUE(ray_cast(box, vec(-5, 0, 0), vec(1, 0, 0), 10.0, hit));
    EXPECT_NEAR(hit.distance, 4.0, tolerance);
    EXPECT_NEAR(hit.normal.x, -1.0, tolerance);

    ASSERT_TRUE(ray_cast(box, vec(0, 9, 0), vec(0, -1, 0), 20.0, hit));
    EXPECT_NEAR(hit.distance, 7.0, tolerance);
    EXPECT_NEAR(hit.normal.y, 1.0, tolerance);

    // A rotated box: a quarter turn about Z swaps which half-extent faces X.
    const Real half = std::sqrt(0.5);
    const OrientedBox<Real> turned{vec(0, 0, 0), vec(1, 2, 3),
                                   QuaternionT<Real>{0.0, 0.0, half, half}};
    ASSERT_TRUE(ray_cast(turned, vec(-5, 0, 0), vec(1, 0, 0), 10.0, hit));
    EXPECT_NEAR(hit.distance, 3.0, tolerance);
}

TEST(Unit_SceneQuery, TriangleIsHitInsideAndMissedOutside)
{
    const TriangleCollider<Real> triangle{vec(0, 0, 0), vec(4, 0, 0), vec(0, 4, 0)};
    RayHit<Real> hit;
    ASSERT_TRUE(ray_cast(triangle, vec(1, 1, 5), vec(0, 0, -1), 10.0, hit));
    EXPECT_NEAR(hit.distance, 5.0, tolerance);
    EXPECT_NEAR(hit.normal.z, 1.0, tolerance);

    EXPECT_FALSE(ray_cast(triangle, vec(3, 3, 5), vec(0, 0, -1), 10.0, hit));
}

TEST(Unit_SceneQuery, TheGeneralRoutineAgreesWithTheClosedFormItReplaces)
{
    // A capsule with a zero-length segment *is* a sphere, so the conservative
    // advancement and the quadratic must land on the same number. If they do not,
    // the generic routine is wrong and every hull cast is wrong with it.
    CapsuleCollider<Real> capsule;
    capsule.center = vec(2, 0, 0);
    capsule.half_height = 0.0;
    capsule.radius = 1.0;

    RayHit<Real> generic;
    ASSERT_TRUE(ray_cast(capsule, vec(0, 0, 0), vec(1, 0, 0), 10.0, generic));
    RayHit<Real> analytic;
    ASSERT_TRUE(ray_cast(SphereCollider<Real>{vec(2, 0, 0), 1.0}, vec(0, 0, 0), vec(1, 0, 0), 10.0,
                         analytic));
    EXPECT_NEAR(generic.distance, analytic.distance, 1e-4);
    EXPECT_NEAR(generic.normal.x, analytic.normal.x, 1e-3);

    // And a hull whose vertices are a unit box must answer like the box.
    const Vector3T<Real> corners[8] = {vec(-1, -1, -1), vec(1, -1, -1), vec(-1, 1, -1),
                                       vec(1, 1, -1),   vec(-1, -1, 1), vec(1, -1, 1),
                                       vec(-1, 1, 1),   vec(1, 1, 1)};
    ConvexHullView<Real> hull;
    hull.vertices = corners;
    hull.vertex_count = 8;
    hull.center = vec(3, 0, 0);

    RayHit<Real> hull_hit;
    ASSERT_TRUE(ray_cast(hull, vec(-2, 0.3, 0.4), vec(1, 0, 0), 20.0, hull_hit));
    EXPECT_NEAR(hull_hit.distance, 4.0, 1e-4);
    EXPECT_NEAR(hull_hit.normal.x, -1.0, 1e-3);
}

TEST(Unit_SceneQuery, TheTableReachesEveryRegisteredShapeAndNothingElse)
{
    RayHit<Real> hit;
    EXPECT_TRUE(ray_cast_shape<Real>(make_sphere_shape<Real>(vec(2, 0, 0), 1.0), vec(0, 0, 0),
                                     vec(1, 0, 0), 10.0, hit));
    EXPECT_TRUE(ray_cast_shape<Real>(
        make_box_shape<Real>(vec(2, 0, 0), vec(1, 1, 1), QuaternionT<Real>{0, 0, 0, 1}),
        vec(0, 0, 0), vec(1, 0, 0), 10.0, hit));
    EXPECT_TRUE(ray_cast_shape<Real>(
        make_capsule_shape<Real>(vec(2, 0, 0), 0.5, 0.5, QuaternionT<Real>{0, 0, 0, 1}),
        vec(0, 0, 0), vec(1, 0, 0), 10.0, hit));
    EXPECT_TRUE(ray_cast_shape<Real>(make_plane_shape<Real>(vec(0, 1, 0), 0.0), vec(0, 3, 0),
                                     vec(0, -1, 0), 10.0, hit));

    // `ShapeType::box` has no entries anywhere, deliberately: an axis-aligned box
    // is an oriented box with the identity rotation, and a second spelling of the
    // same shape is how a table grows a hole nobody notices.
    CollisionShape<Real> unregistered;
    unregistered.type = ShapeType::box;
    EXPECT_FALSE(
        ray_cast_shape<Real>(unregistered, vec(0, 0, 0), vec(1, 0, 0), 10.0, hit));
}

// ---------------------------------------------------------------------------
// Over a broadphase
// ---------------------------------------------------------------------------

TEST(Unit_SceneQuery, ClosestHitIsTheNearestOfSeveralAlongTheRay)
{
    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(10, 0, 0), 1.0));
    scene.add(make_sphere_shape<Real>(vec(4, 0, 0), 1.0));
    scene.add(make_sphere_shape<Real>(vec(7, 0, 0), 1.0));
    scene.add(make_sphere_shape<Real>(vec(4, 8, 0), 1.0)); // nowhere near the ray

    const RayHit<Real> hit = raycast_closest<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); }, vec(0, 0, 0),
        vec(1, 0, 0), 50.0);
    ASSERT_TRUE(hit.hit);
    EXPECT_EQ(hit.payload, 1u);
    EXPECT_NEAR(hit.distance, 3.0, tolerance);

    const std::vector<RayHit<Real>> all = raycast_all<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); }, vec(0, 0, 0),
        vec(1, 0, 0), 50.0);
    ASSERT_EQ(all.size(), 3u);
    EXPECT_EQ(all[0].payload, 1u);
    EXPECT_EQ(all[1].payload, 2u);
    EXPECT_EQ(all[2].payload, 0u);
    EXPECT_LT(all[0].distance, all[1].distance);
}

TEST(Unit_SceneQuery, TheFilterIsAppliedBeforeTheGeometry)
{
    const std::uint32_t scenery = 1u << 3;
    CollisionFilter scenery_filter;
    scenery_filter.layer = scenery;

    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(4, 0, 0), 1.0), 0u, scenery_filter);
    scene.add(make_sphere_shape<Real>(vec(8, 0, 0), 1.0));

    QueryFilter<Real> filter;
    filter.layer_mask = ~scenery;
    const RayHit<Real> hit = raycast_closest<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); }, vec(0, 0, 0),
        vec(1, 0, 0), 50.0, filter);
    ASSERT_TRUE(hit.hit);
    EXPECT_EQ(hit.payload, 1u) << "the nearer sphere is on a layer the query was told to ignore";

    // And the predicate: "anything but the body that fired this shot".
    QueryFilter<Real> exclude;
    exclude.predicate = [](ProxyId, std::uint32_t payload) { return payload != 0u; };
    const RayHit<Real> second = raycast_closest<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); }, vec(0, 0, 0),
        vec(1, 0, 0), 50.0, exclude);
    ASSERT_TRUE(second.hit);
    EXPECT_EQ(second.payload, 1u);
}

TEST(Unit_SceneQuery, TriggersAreQueryableAndRejectableByFlag)
{
    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(4, 0, 0), 1.0), BodyFlags::trigger);
    scene.add(make_sphere_shape<Real>(vec(8, 0, 0), 1.0));

    const auto shape_of = [&](ProxyId id) { return scene.shape_of(id); };

    // A trigger is a shape that reports and never pushes, so a query sees it by
    // default — a volume the player can walk into is a volume a probe can find.
    EXPECT_EQ(
        raycast_closest<Real>(scene.broadphase, shape_of, vec(0, 0, 0), vec(1, 0, 0), 50.0).payload,
        0u);

    QueryFilter<Real> solid_only;
    solid_only.reject_flags = BodyFlags::trigger;
    EXPECT_EQ(raycast_closest<Real>(scene.broadphase, shape_of, vec(0, 0, 0), vec(1, 0, 0), 50.0,
                                    solid_only)
                  .payload,
              1u);
}

TEST(Unit_SceneQuery, OverlapRejectsWhatOnlyTheBoundingBoxesShare)
{
    // Two spheres whose boxes overlap at the corner but whose surfaces do not.
    // A trigger built on the broadphase's answer would fire here, which is the
    // bug this test exists to keep out.
    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(1.5, 1.5, 0), 1.0));
    scene.add(make_sphere_shape<Real>(vec(0.5, 0, 0), 1.0));

    const auto shape_of = [&](ProxyId id) { return scene.shape_of(id); };
    const CollisionShape<Real> query = make_sphere_shape<Real>(vec(0, 0, 0), 1.0);

    EXPECT_TRUE(aabb_overlap(shape_world_bounds(query), shape_world_bounds(scene.shapes[0])));
    const std::vector<ProxyId> found =
        overlap_shape<Real>(scene.broadphase, shape_of, query);
    ASSERT_EQ(found.size(), 1u);
    EXPECT_EQ(scene.broadphase.proxy(found[0]).payload, 1u);
}

TEST(Unit_SceneQuery, ClosestPointReportsTheGapAndBothWitnesses)
{
    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(5, 0, 0), 1.0));
    scene.add(make_sphere_shape<Real>(vec(-9, 0, 0), 1.0));

    const ClosestResult<Real> result = closest_point<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); },
        make_sphere_shape<Real>(vec(0, 0, 0), 1.0), 10.0);
    ASSERT_TRUE(result.found);
    EXPECT_EQ(result.payload, 0u);
    EXPECT_NEAR(result.distance, 3.0, 1e-4);
    EXPECT_NEAR(result.point_on_query.x, 1.0, 1e-4);
    EXPECT_NEAR(result.point_on_shape.x, 4.0, 1e-4);
    EXPECT_NEAR(result.normal.x, 1.0, 1e-4);
}

TEST(Unit_SceneQuery, ASweepStopsWhereTheShapesTouchRatherThanWhereTheirCentresMeet)
{
    Scene scene;
    scene.add(make_box_shape<Real>(vec(10, 0, 0), vec(1, 4, 4), QuaternionT<Real>{0, 0, 0, 1}),
              BodyFlags::static_body);

    const CollisionShape<Real> mover = make_sphere_shape<Real>(vec(0, 0, 0), 0.5);
    const RayHit<Real> hit = sweep_shape<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); }, mover, vec(1, 0, 0),
        20.0);
    ASSERT_TRUE(hit.hit);
    // Wall face at x = 9, sphere radius 0.5: the centre travels 8.5 metres.
    EXPECT_NEAR(hit.distance, 8.5, 1e-4);
    EXPECT_NEAR(hit.normal.x, -1.0, 1e-3);
}

TEST(Unit_SceneQuery, ASweepThatReachesNothingReportsNoHit)
{
    Scene scene;
    scene.add(make_sphere_shape<Real>(vec(50, 0, 0), 1.0), BodyFlags::static_body);
    const RayHit<Real> hit = sweep_shape<Real>(
        scene.broadphase, [&](ProxyId id) { return scene.shape_of(id); },
        make_sphere_shape<Real>(vec(0, 0, 0), 0.5), vec(1, 0, 0), 10.0);
    EXPECT_FALSE(hit.hit);
}

TEST(Unit_SceneQuery, QueriesAgreeWhicheverBroadphaseIsBehindThem)
{
    // The query layer names `IBroadphase`, so the §4.4 substitutability claim has
    // to hold for queries too: the same ray, the same answer, either structure.
    SweepAndPruneBroadphase<Real> sweep;
    BVHBroadphase<Real> hierarchy;
    std::vector<CollisionShape<Real>> shapes;
    for (int i = 0; i < 40; ++i)
    {
        const Real x = Real(i) * 1.7 - 30.0;
        const CollisionShape<Real> shape =
            make_sphere_shape<Real>(vec(x, std::sin(Real(i)) * 0.4, 0.0), 0.6);
        shapes.push_back(shape);
        const std::uint32_t payload = static_cast<std::uint32_t>(i);
        sweep.create_proxy(shape_world_bounds(shape), CollisionFilter{}, 0u, payload);
        hierarchy.create_proxy(shape_world_bounds(shape), CollisionFilter{}, 0u, payload);
    }
    const auto shape_of = [&](ProxyId id) { return shapes[id]; };

    const RayHit<Real> a = raycast_closest<Real>(sweep, shape_of, vec(-60, 0, 0), vec(1, 0, 0),
                                                 200.0);
    const RayHit<Real> b = raycast_closest<Real>(hierarchy, shape_of, vec(-60, 0, 0), vec(1, 0, 0),
                                                 200.0);
    ASSERT_TRUE(a.hit);
    EXPECT_EQ(a.payload, b.payload);
    EXPECT_DOUBLE_EQ(a.distance, b.distance);
}
