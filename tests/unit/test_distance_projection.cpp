/**************************************************************************/
/* test_distance_projection.cpp                                           */
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

// Unit_DistanceProjection: the XPBD rigid distance constraint, measured against a
// conservation law rather than against itself.
//
// The defect this file exists to see is a frame error: an angular correction applied
// as a body-local vector where apply_angular_correction expects a world-frame one,
// which turns every rotated body about the wrong axis. The obvious tests are blind to
// it. Cloth and particles have zero inverse inertia, so there is no angular term at
// all, and the solver conformance suite compares a host and a device implementation
// that call the same functor, so a wrong formula is shared rather than exposed.
//
// What catches it is a physical invariant no implementation gets a vote on. A
// constraint impulse is internal: it acts equal and opposite along the line joining
// the two anchors, so it can change neither the total linear momentum nor the total
// angular momentum of the pair. In XPBD's positional space that reads
//
//     sum over bodies of [ I_world * dtheta + x cross (m * dx) ]  ==  0
//
// and it is identically zero only when dtheta is the *world*-frame rotation the
// impulse implies. A body-frame dtheta leaves it zero for an unrotated body and nine
// orders of magnitude larger for a rotated one.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/constraints/distance_projection.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief The world-frame rotation vector taking @p before to @p after. */
    Vector3 delta_rotation(const Quaternion& before, const Quaternion& after)
    {
        // Left multiplication, matching apply_angular_correction's convention: the
        // delta quaternion is `after * conjugate(before)`, and twice its vector part
        // is the rotation vector for a small rotation. The sign correction takes the
        // shorter path, as update_velocity does.
        const Quaternion delta = mul(after, conjugate(before));
        const Scalar sign = delta.w < 0.0 ? -1.0 : 1.0;
        return Vector3{delta.x, delta.y, delta.z} * (2.0 * sign);
    }

    /** @brief `I_world * v`, from the body-local diagonal inverse inertia. */
    Vector3 world_inertia_times(const RigidBody& body, const Vector3& v)
    {
        const Vector3 local = rotate(conjugate(body.orientation), v);
        const Vector3 scaled{local.x / body.inv_inertia.x, local.y / body.inv_inertia.y,
                             local.z / body.inv_inertia.z};
        return rotate(body.orientation, scaled);
    }

    /** @brief A two-body scene joined by one distance constraint. */
    struct Scene
    {
        RigidBody bodies[2];
        XPBDDistanceConstraint constraint;
    };

    /**
     * @brief Two anisotropic bodies at the given orientations, joined off-centre.
     *
     * The rest length is set a tenth of a millimetre under the current separation,
     * so one projection is a small correction: the invariant below is exact only to
     * first order, and a large correction would drown the signal in the quaternion
     * renormalization's second-order terms.
     */
    Scene make_scene(const Quaternion& qa, const Quaternion& qb)
    {
        Scene scene;
        scene.bodies[0].position = Vector3{-0.6, 0.1, 0.0};
        scene.bodies[0].orientation = qa;
        scene.bodies[0].inv_mass = 1.0;
        scene.bodies[0].inv_inertia = Vector3{2.0, 9.0, 4.0};
        scene.bodies[1].position = Vector3{0.7, 0.0, 0.2};
        scene.bodies[1].orientation = qb;
        scene.bodies[1].inv_mass = 0.5;
        scene.bodies[1].inv_inertia = Vector3{5.0, 3.0, 7.0};

        scene.constraint.a = 0;
        scene.constraint.b = 1;
        scene.constraint.local_anchor_a = Vector3{0.4, 0.3, -0.2};
        scene.constraint.local_anchor_b = Vector3{-0.1, 0.5, 0.25};
        scene.constraint.compliance = 0.0;

        const Vector3 pa = scene.bodies[0].position +
                           rotate(scene.bodies[0].orientation, scene.constraint.local_anchor_a);
        const Vector3 pb = scene.bodies[1].position +
                           rotate(scene.bodies[1].orientation, scene.constraint.local_anchor_b);
        scene.constraint.rest_length = length(pb - pa) - 1e-4;
        return scene;
    }

    /** @brief The pair's total change in linear and angular momentum over one projection. */
    void momentum_change(const Quaternion& qa, const Quaternion& qb, Vector3& linear,
                         Vector3& angular)
    {
        Scene scene = make_scene(qa, qb);
        const RigidBody before[2] = {scene.bodies[0], scene.bodies[1]};

        Scalar lambda = 0.0;
        XPBDDistanceProjection{}(scene.constraint, scene.bodies, lambda, 1.0 / 60.0);

        linear = Vector3{0.0, 0.0, 0.0};
        angular = Vector3{0.0, 0.0, 0.0};
        for (int i = 0; i < 2; ++i)
        {
            const Vector3 dx = scene.bodies[i].position - before[i].position;
            const Vector3 dtheta =
                delta_rotation(before[i].orientation, scene.bodies[i].orientation);
            const Scalar mass = 1.0 / before[i].inv_mass;
            linear = linear + dx * mass;
            angular = angular + world_inertia_times(before[i], dtheta) +
                      cross(before[i].position, dx * mass);
        }
    }
} // namespace

// The control: with both bodies unrotated the body frame and the world frame
// coincide, so a frame error cannot show here even when there is one.
TEST(Unit_DistanceProjection, ConservesMomentumForUnrotatedBodies)
{
    Vector3 linear;
    Vector3 angular;
    momentum_change(Quaternion{0.0, 0.0, 0.0, 1.0}, Quaternion{0.0, 0.0, 0.0, 1.0}, linear,
                    angular);

    EXPECT_LT(length(linear), 1e-12);
    EXPECT_LT(length(angular), 1e-12);
}

// The case that exposes a frame error. A constraint impulse is internal, so it cannot
// change the pair's angular momentum however the two bodies happen to be oriented. A
// body-frame correction leaves 7e-6 here — small enough to look like noise in a scene
// and large enough to be a body turning about the wrong axis.
TEST(Unit_DistanceProjection, ConservesMomentumForRotatedBodies)
{
    const Quaternion qa = quaternion_axis_angle(normalize(Vector3{0.3, 1.0, -0.5}), 0.9);
    const Quaternion qb = quaternion_axis_angle(normalize(Vector3{-0.7, 0.2, 0.4}), 1.3);

    Vector3 linear;
    Vector3 angular;
    momentum_change(qa, qb, linear, angular);

    EXPECT_LT(length(linear), 1e-12);
    EXPECT_LT(length(angular), 1e-12);
}

// And it holds for orientations in general, not for the one that happened to be
// picked: a sweep about several axes, each of which puts the body frame somewhere
// different relative to the world.
TEST(Unit_DistanceProjection, ConservesMomentumAcrossASweepOfOrientations)
{
    const Vector3 axes[] = {Vector3{1.0, 0.0, 0.0}, Vector3{0.0, 1.0, 0.0},
                            Vector3{0.0, 0.0, 1.0}, normalize(Vector3{1.0, 1.0, 1.0}),
                            normalize(Vector3{-0.2, 0.7, 0.6})};
    for (const Vector3& axis : axes)
        for (const Scalar angle : {0.3, 1.1, 2.4, 3.0})
        {
            Vector3 linear;
            Vector3 angular;
            momentum_change(quaternion_axis_angle(axis, angle),
                            quaternion_axis_angle(axis, -angle * 0.5), linear, angular);
            EXPECT_LT(length(linear), 1e-12);
            EXPECT_LT(length(angular), 1e-12) << "axis (" << axis.x << ", " << axis.y << ", "
                                              << axis.z << ") angle " << angle;
        }
}

// The constraint does its job: a hard (zero-compliance) constraint on a rotated pair
// converges to its rest length. Convergence on its own is a weak signal — an iterative
// solver corrects its own errors given enough sweeps, so a frame error can hide behind
// it — but a failure here would be unambiguous.
TEST(Unit_DistanceProjection, HardConstraintReachesItsRestLength)
{
    const Quaternion qa = quaternion_axis_angle(normalize(Vector3{0.3, 1.0, -0.5}), 0.9);
    const Quaternion qb = quaternion_axis_angle(normalize(Vector3{-0.7, 0.2, 0.4}), 1.3);
    Scene scene = make_scene(qa, qb);
    scene.constraint.rest_length = 0.75;

    for (int iteration = 0; iteration < 24; ++iteration)
    {
        Scalar lambda = 0.0;
        XPBDDistanceProjection{}(scene.constraint, scene.bodies, lambda, 1.0 / 60.0);
    }

    const Vector3 pa = scene.bodies[0].position +
                       rotate(scene.bodies[0].orientation, scene.constraint.local_anchor_a);
    const Vector3 pb = scene.bodies[1].position +
                       rotate(scene.bodies[1].orientation, scene.constraint.local_anchor_b);
    EXPECT_NEAR(length(pb - pa), 0.75, 1e-9);
}

// A body with no rotational freedom must behave exactly as the purely linear
// projection does — this is the path cloth and particles take, and the one no
// angular frame error can reach, so it is worth pinning separately.
TEST(Unit_DistanceProjection, BodiesWithoutRotationSplitByInverseMassAlone)
{
    Scene scene = make_scene(quaternion_axis_angle(Vector3{0.0, 1.0, 0.0}, 0.7),
                             quaternion_axis_angle(Vector3{1.0, 0.0, 0.0}, 1.2));
    scene.bodies[0].inv_inertia = Vector3{0.0, 0.0, 0.0};
    scene.bodies[1].inv_inertia = Vector3{0.0, 0.0, 0.0};
    scene.constraint.rest_length = 0.75;

    const Quaternion qa = scene.bodies[0].orientation;
    const Quaternion qb = scene.bodies[1].orientation;
    const Vector3 xa = scene.bodies[0].position;
    const Vector3 xb = scene.bodies[1].position;

    Scalar lambda = 0.0;
    XPBDDistanceProjection{}(scene.constraint, scene.bodies, lambda, 1.0 / 60.0);

    // Neither body turned at all.
    EXPECT_LT(length(delta_rotation(qa, scene.bodies[0].orientation)), 1e-15);
    EXPECT_LT(length(delta_rotation(qb, scene.bodies[1].orientation)), 1e-15);
    // And the two displacements are opposed and in inverse-mass proportion.
    const Vector3 dx_a = scene.bodies[0].position - xa;
    const Vector3 dx_b = scene.bodies[1].position - xb;
    EXPECT_NEAR(length(dx_a) * scene.bodies[1].inv_mass,
                length(dx_b) * scene.bodies[0].inv_mass, 1e-12);
    EXPECT_LT(length(dx_a * (1.0 / scene.bodies[0].inv_mass) +
                     dx_b * (1.0 / scene.bodies[1].inv_mass)),
              1e-12);
}
