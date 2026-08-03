/**************************************************************************/
/* test_sdf_manifold.cpp                                                  */
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

// Unit_SDFManifold: signed-distance-field collision, §7.5's "first-class
// narrowphase path" (physics/geometry/shapes.hpp's SDFCollider, and
// physics/collision/sdf_manifold.hpp).
//
// A real cooked field comes from the collision cooker (P4) and is exercised
// there; what this file owns is the narrowphase arithmetic in isolation, and
// the oracle it is checked against is a field with a *known* closed form: a
// flat plane's signed distance is just its coordinate along the plane's own
// normal, so every voxel can be filled analytically and the routine's answer
// compared against that formula rather than against itself.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/collision/sdf_manifold.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /**
     * @brief A synthetic field, over `[-1, 1]^3`, whose value is exactly the
     *        local Y coordinate — the signed distance to the plane `y = 0`.
     *
     * Filled per the same voxel-centre convention `SignedDistanceFieldBrick`
     * documents (`bake_signed_distance_field`'s own file), so the discretization
     * error this test allows for is exactly the one a cooked field would have.
     */
    struct PlaneField
    {
        std::vector<float> distances;
        std::int32_t resolution = 0;
        Vector3 field_min{-1.0, -1.0, -1.0};
        Vector3 field_max{1.0, 1.0, 1.0};

        SDFCollider<Scalar> shape(Vector3 center = Vector3{0.0, 0.0, 0.0},
                                  Quaternion orientation = Quaternion{0.0, 0.0, 0.0, 1.0}) const
        {
            SDFCollider<Scalar> field;
            field.distances = distances.data();
            field.resolution = resolution;
            field.field_min = field_min;
            field.field_max = field_max;
            field.center = center;
            field.orientation = orientation;
            return field;
        }
    };

    PlaneField build_plane_field(std::int32_t resolution)
    {
        PlaneField field;
        field.resolution = resolution;
        field.distances.resize(std::size_t(resolution) * std::size_t(resolution) *
                               std::size_t(resolution));
        const double span = field.field_max.y - field.field_min.y;
        for (std::int32_t z = 0; z < resolution; ++z)
            for (std::int32_t y = 0; y < resolution; ++y)
            {
                const double local_y =
                    field.field_min.y + (double(y) + 0.5) * span / double(resolution);
                for (std::int32_t x = 0; x < resolution; ++x)
                {
                    const std::size_t index = std::size_t(x) +
                                              std::size_t(resolution) *
                                                  (std::size_t(y) +
                                                   std::size_t(resolution) * std::size_t(z));
                    field.distances[index] = float(local_y);
                }
            }
        return field;
    }
} // namespace

// The gradient of a flat plane's field is the plane's own normal everywhere —
// the simplest possible non-degenerate case, and the one every other
// assertion in this file leans on.
TEST(Unit_SDFManifold, GradientOfAPlaneFieldIsThePlaneNormal)
{
    const PlaneField built = build_plane_field(64);
    const SDFCollider<Scalar> field = built.shape();

    const Vector3 gradient = sdf_gradient_world(field, Vector3{0.1, 0.3, -0.2});
    EXPECT_NEAR(gradient.x, 0.0, 1e-6);
    EXPECT_NEAR(gradient.y, 1.0, 1e-6);
    EXPECT_NEAR(gradient.z, 0.0, 1e-6);
}

// A sphere well clear of the surface produces no manifold at all — the same
// "further apart than contact_offset reports nothing" contract every other
// narrowphase routine in the engine holds to.
TEST(Unit_SDFManifold, SphereFarFromTheFieldProducesNoContact)
{
    const PlaneField built = build_plane_field(64);
    const SDFCollider<Scalar> field = built.shape();
    const SphereCollider<Scalar> sphere{Vector3{0.0, 0.5, 0.0}, 0.1};

    const ContactManifold<Scalar> manifold = generate_convex_sdf_manifold<Scalar>(
        sphere, field, sphere.center, Quaternion{0.0, 0.0, 0.0, 1.0}, 0.05);
    EXPECT_EQ(manifold.point_count, 0);
}

// A sphere overlapping the plane produces exactly one point, pushing the
// sphere out along the plane's normal — up, away from the solid the field
// encodes as `y < 0`.
TEST(Unit_SDFManifold, OverlappingSphereProducesAnUpwardPushingContact)
{
    const PlaneField built = build_plane_field(64);
    const SDFCollider<Scalar> field = built.shape();
    // Centre just above the plane, radius large enough that the sphere's
    // underside is well inside it.
    const SphereCollider<Scalar> sphere{Vector3{0.0, 0.02, 0.0}, 0.1};

    const ContactManifold<Scalar> manifold = generate_convex_sdf_manifold<Scalar>(
        sphere, field, sphere.center, Quaternion{0.0, 0.0, 0.0, 1.0}, 0.05);
    ASSERT_EQ(manifold.point_count, 1);

    // The normal runs from the shape (a) toward the field's solid (b), i.e.
    // downward here — resolving therefore moves the sphere along `-normal`,
    // upward, out of the ground.
    EXPECT_NEAR(manifold.normal.x, 0.0, 1e-3);
    EXPECT_NEAR(manifold.normal.y, -1.0, 1e-3);
    EXPECT_NEAR(manifold.normal.z, 0.0, 1e-3);

    // The sphere's underside sits 8 cm below the plane; the reported
    // separation should be negative and in that neighbourhood, within the
    // half-voxel quantization a 64-voxel field over a 2 m span carries.
    EXPECT_LT(manifold.points[0].separation, 0.0);
    EXPECT_NEAR(manifold.points[0].separation, -0.08, 0.05);
}

// The dispatch table (§4.2) must resolve both orders of an SDF pair, and the
// two answers must be the same contact seen from either side — the property
// `gjk.hpp`'s own tests hold every convex pair to.
TEST(Unit_SDFManifold, DispatchTableResolvesBothOrders)
{
    const PlaneField built = build_plane_field(64);
    const CollisionShape<Scalar> field_shape =
        make_sdf_shape<Scalar>(built.distances.data(), built.resolution, built.field_min,
                               built.field_max, Vector3{0.0, 0.0, 0.0});
    const CollisionShape<Scalar> sphere_shape =
        make_sphere_shape<Scalar>(Vector3{0.0, 0.02, 0.0}, 0.1);

    const ContactManifold<Scalar> forward =
        generate_shape_manifold<Scalar>(sphere_shape, field_shape, 0.05);
    const ContactManifold<Scalar> reversed =
        generate_shape_manifold<Scalar>(field_shape, sphere_shape, 0.05);

    ASSERT_EQ(forward.point_count, 1);
    ASSERT_EQ(reversed.point_count, 1);
    EXPECT_NEAR(forward.normal.x, -reversed.normal.x, 1e-6);
    EXPECT_NEAR(forward.normal.y, -reversed.normal.y, 1e-6);
    EXPECT_NEAR(forward.normal.z, -reversed.normal.z, 1e-6);
    EXPECT_NEAR(forward.points[0].separation, reversed.points[0].separation, 1e-6);
}

// world_bounds must report the field's padded local bounds carried into the
// world by its placement, the same contract every other shape's bounds hold.
TEST(Unit_SDFManifold, WorldBoundsFollowsPlacement)
{
    const PlaneField built = build_plane_field(8);
    const SDFCollider<Scalar> field = built.shape(Vector3{5.0, 0.0, 0.0});

    const AABB<Scalar> bounds = world_bounds(field);
    EXPECT_NEAR(bounds.min.x, 4.0, 1e-9);
    EXPECT_NEAR(bounds.max.x, 6.0, 1e-9);
    EXPECT_NEAR(bounds.min.y, -1.0, 1e-9);
    EXPECT_NEAR(bounds.max.y, 1.0, 1e-9);
}

// An empty field (no baked distances — an asset that carries none) must
// report no contact rather than dereference a null pointer.
TEST(Unit_SDFManifold, EmptyFieldProducesNoContact)
{
    SDFCollider<Scalar> field; // default: distances == nullptr
    const SphereCollider<Scalar> sphere{Vector3{0.0, 0.0, 0.0}, 0.5};

    const ContactManifold<Scalar> manifold = generate_convex_sdf_manifold<Scalar>(
        sphere, field, sphere.center, Quaternion{0.0, 0.0, 0.0, 1.0}, 0.05);
    EXPECT_EQ(manifold.point_count, 0);
}
