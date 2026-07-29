/**************************************************************************/
/* test_contact_projection.cpp                                            */
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

// Unit_ContactProjection: resolving a manifold (physics/solver/contact_projection.hpp).
//
// The three behaviours this phase exists to deliver, each of which the engine
// simply could not express before — there was no velocity pass and no friction
// anywhere (§1.2 item 2):
//
//   - a dropped body bounces to the height its restitution says, and a resting
//     one does not buzz;
//   - a box on a ramp below the friction angle stays put and above it slides,
//     which is the angle-of-repose acceptance criterion of §16's P1 row;
//   - a stack settles flat instead of rocking, because the manifold gives the
//     solver four points to hold it by.
//
// These run the whole substep schedule the header documents, on the host, over
// plain RigidBodyT — no runtime, no scene, no ECS. What is being measured is the
// physics, not the plumbing.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/solver/contact_projection.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar GRAVITY = 9.81;

    /** @brief A unit-mass cube of the given half-extent, at rest at @p position. */
    RigidBody unit_cube(Vector3 position, Scalar half_extent = 0.5, Scalar mass = 1.0)
    {
        RigidBody body;
        body.position = position;
        body.prev_position = position;
        body.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
        body.prev_orientation = body.orientation;
        body.inv_mass = 1.0 / mass;
        // Solid cube about its centre: I = m a^2 / 6 for full edge a.
        const Scalar edge = 2.0 * half_extent;
        const Scalar inertia = mass * edge * edge / 6.0;
        body.inv_inertia = Vector3{1.0 / inertia, 1.0 / inertia, 1.0 / inertia};
        return body;
    }

    /** @brief A box collider tracking @p body's current pose. */
    OrientedBox<Scalar> box_of(const RigidBody& body, Vector3 half_extents)
    {
        return OrientedBox<Scalar>{body.position, half_extents, body.orientation};
    }

    /**
     * @brief Runs one full tick of the documented schedule against a ground plane.
     *
     * Generates the manifold once, warm-starts it from @p previous, then runs
     * @p substeps of capture / clear / predict / solve / derive / velocity-solve.
     * This is the loop `PhysicsScene::step` will be, reduced to one body and one
     * plane so a test can watch it.
     */
    void tick_against_plane(RigidBody& body, Vector3 half_extents,
                            const PlaneCollider<Scalar>& ground, ContactManifold<Scalar>& manifold,
                            const ContactSolveParams<Scalar>& params, Scalar dt,
                            std::size_t substeps)
    {
        // Contacts are *generated* further out than they are *resolved* to: the
        // offset has to cover a tick's motion, or a corner that comes down within
        // the tick has no constraint until the next tick and arrives already deep.
        const Scalar contact_offset = params.rest_offset + 0.03;
        const ContactManifold<Scalar> previous = manifold;
        manifold = generate_obb_plane_manifold(box_of(body, half_extents), ground, contact_offset);
        warm_start_manifold(manifold, previous);

        RigidBody ground_body = immovable_body<Scalar>();
        const Scalar h = dt / static_cast<Scalar>(substeps);
        for (std::size_t s = 0; s < substeps; ++s)
        {
            capture_contact_velocities(manifold, body, ground_body);
            if (s > 0)
                clear_manifold_impulses(manifold);
            predict(body, Vector3{0.0, -GRAVITY, 0.0}, h);
            solve_manifold_positions(manifold, body, ground_body, params);
            update_velocity(body, h);
            solve_manifold_velocities(manifold, body, ground_body, params, h);
        }
    }

    /** @brief Frictionless, bounceless contact parameters. */
    ContactSolveParams<Scalar> plain_params()
    {
        ContactSolveParams<Scalar> params;
        params.static_friction = 0.0;
        params.dynamic_friction = 0.0;
        params.restitution = 0.0;
        return params;
    }
} // namespace

// The baseline the rest builds on: a box dropped on the ground stops on the
// ground, at the rest offset, and stays there.
TEST(Unit_ContactProjection, DroppedBoxComesToRestOnTheSurface)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    RigidBody body = unit_cube(Vector3{0.0, 2.0, 0.0});
    ContactManifold<Scalar> manifold;
    ContactSolveParams<Scalar> params = plain_params();
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 8.0);

    for (int tick = 0; tick < 180; ++tick)
        tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0, 8);

    EXPECT_NEAR(body.position.y, 0.5, 1e-3);
    EXPECT_LT(std::abs(body.velocity.y), 1e-2);
    // It landed flat: no accumulated tumble. The residual is what a Gauss-Seidel
    // sweep over four points leaves behind — each point is projected against the
    // poses the previous one produced, so the four are not perfectly symmetric.
    EXPECT_LT(length(body.angular_velocity), 5e-3);
}

// rest_offset is the visible-penetration contract made a number (§7.6): a
// positive value keeps a sliver of air, a negative one lets the surface sink in.
TEST(Unit_ContactProjection, RestOffsetSetsWhereTheSurfaceComesToRest)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};

    for (const Scalar offset : {-0.01, 0.0, 0.02})
    {
        RigidBody body = unit_cube(Vector3{0.0, 1.5, 0.0});
        ContactManifold<Scalar> manifold;
        ContactSolveParams<Scalar> params = plain_params();
        params.rest_offset = offset;
        params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 8.0);

        for (int tick = 0; tick < 180; ++tick)
            tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0,
                               8);

        // The manifold's normal runs box -> plane, so a positive rest offset holds
        // the box that much *above* the surface.
        EXPECT_NEAR(body.position.y, 0.5 + offset, 2e-3) << "rest_offset " << offset;
    }
}

// Restitution, the thing a purely positional solver cannot express: a body
// dropped from a height returns to a fraction of it, and the fraction is e^2.
TEST(Unit_ContactProjection, RestitutionReturnsTheExpectedFractionOfTheDrop)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const Scalar drop_height = 1.0; // above the resting height
    const Scalar restitution = 0.6;

    RigidBody body = unit_cube(Vector3{0.0, 0.5 + drop_height, 0.0});
    ContactManifold<Scalar> manifold;
    ContactSolveParams<Scalar> params = plain_params();
    params.restitution = restitution;
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 16.0);

    Scalar apex = 0.0;
    bool bounced = false;
    for (int tick = 0; tick < 240; ++tick)
    {
        tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0, 16);
        if (!bounced && body.velocity.y > 0.0)
            bounced = true;
        if (bounced)
            apex = std::max(apex, body.position.y - 0.5);
        if (bounced && body.velocity.y < 0.0 && apex > 0.0)
            break;
    }

    ASSERT_TRUE(bounced);
    // Energy scales with e^2, so the rebound height does too. The tolerance is
    // wide because a discrete substep loses a little to the tick it lands on.
    EXPECT_NEAR(apex, drop_height * restitution * restitution, 0.12);
}

// The anti-jitter threshold, and the reason it exists: a resting body's contacts
// carry a closing speed of about g*h every substep purely because gravity had a
// substep to act. Bouncing that back is how a settled stack buzzes for ever.
TEST(Unit_ContactProjection, RestingBodyDoesNotBuzzUnderRestitution)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    RigidBody body = unit_cube(Vector3{0.0, 0.5, 0.0});
    ContactManifold<Scalar> manifold;
    ContactSolveParams<Scalar> params = plain_params();
    params.restitution = 0.9;
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 8.0);

    for (int tick = 0; tick < 240; ++tick)
        tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0, 8);

    EXPECT_NEAR(body.position.y, 0.5, 2e-3);
    EXPECT_LT(std::abs(body.velocity.y), 5e-2);
}

// The angle of repose: below atan(mu) a box on a ramp stays where it is, above
// it the box slides. Static friction has to be *positional* for the first half
// of that to hold — a velocity-level friction leaves a residual creep every step
// that no amount of damping removes.
TEST(Unit_ContactProjection, BoxHoldsOnARampBelowTheFrictionAngleAndSlidesAbove)
{
    const Scalar mu = 0.6;
    const Scalar friction_angle = std::atan(mu);

    const auto slide_distance = [&](Scalar angle) -> Scalar
    {
        // A ramp is a plane whose normal is tilted about Z.
        const Vector3 normal{-std::sin(angle), std::cos(angle), 0.0};
        const PlaneCollider<Scalar> ramp{normal, 0.0};

        RigidBody body = unit_cube(Vector3{0.0, 0.0, 0.0});
        // Seat the box on the ramp: its centre sits half a height along the normal.
        body.position = normal * 0.5;
        body.prev_position = body.position;
        body.orientation = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, angle);
        body.prev_orientation = body.orientation;

        ContactManifold<Scalar> manifold;
        ContactSolveParams<Scalar> params;
        params.static_friction = mu;
        params.dynamic_friction = mu;
        params.restitution = 0.0;
        params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 16.0);

        const Vector3 start = body.position;
        for (int tick = 0; tick < 120; ++tick)
            tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ramp, manifold, params, 1.0 / 60.0,
                               16);
        return length(body.position - start);
    };

    // Comfortably below the friction angle: it must not creep, over two seconds.
    EXPECT_LT(slide_distance(friction_angle * 0.6), 0.01);
    // Comfortably above it: it must genuinely slide.
    EXPECT_GT(slide_distance(friction_angle * 1.6), 0.25);
}

// Dynamic friction is a bounded force, not a weld: a box launched across the
// ground decelerates at about mu*g and stops, rather than stopping instantly or
// sliding for ever.
TEST(Unit_ContactProjection, SlidingBoxDeceleratesAtTheFrictionRate)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const Scalar mu = 0.3;
    const Scalar launch_speed = 5.0;

    RigidBody body = unit_cube(Vector3{0.0, 0.5, 0.0});
    body.velocity = Vector3{launch_speed, 0.0, 0.0};
    ContactManifold<Scalar> manifold;
    ContactSolveParams<Scalar> params;
    params.static_friction = mu;
    params.dynamic_friction = mu;
    params.restitution = 0.0;
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 16.0);

    // Classical answer: v = v0 - mu*g*t, so it stops after v0/(mu*g) seconds and
    // travels v0^2/(2*mu*g).
    const Scalar expected_time = launch_speed / (mu * GRAVITY);
    const int ticks = static_cast<int>(expected_time * 60.0 * 0.5);
    for (int tick = 0; tick < ticks; ++tick)
        tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0, 16);

    const Scalar elapsed = static_cast<Scalar>(ticks) / 60.0;
    EXPECT_NEAR(body.velocity.x, launch_speed - mu * GRAVITY * elapsed, 0.6);
    EXPECT_GT(body.velocity.x, 0.0);
}

// The point of a manifold rather than a point: a box landing slightly tilted
// settles flat and stops, instead of being held by one corner at a time and
// rocking. This is §1.2 item 3, measured.
TEST(Unit_ContactProjection, TiltedLandingSettlesFlatInsteadOfRocking)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    RigidBody body = unit_cube(Vector3{0.0, 0.9, 0.0});
    body.orientation = quaternion_axis_angle(Vector3{0.0, 0.0, 1.0}, 0.08);
    body.prev_orientation = body.orientation;

    ContactManifold<Scalar> manifold;
    ContactSolveParams<Scalar> params;
    params.static_friction = 0.6;
    params.dynamic_friction = 0.5;
    params.restitution = 0.0;
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 16.0);

    for (int tick = 0; tick < 240; ++tick)
        tick_against_plane(body, Vector3{0.5, 0.5, 0.5}, ground, manifold, params, 1.0 / 60.0, 16);

    // Flat: the box's local Y axis is back to world up.
    const Vector3 up = rotate(body.orientation, Vector3{0.0, 1.0, 0.0});
    EXPECT_GT(up.y, 0.999);
    EXPECT_LT(length(body.angular_velocity), 2e-2);
    EXPECT_NEAR(body.position.y, 0.5, 5e-3);
}

// Two boxes resting on each other on the ground: the stack must hold its shape,
// which is the manifold and the warm start together doing their job.
TEST(Unit_ContactProjection, TwoBoxStackHoldsItsShape)
{
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    RigidBody lower = unit_cube(Vector3{0.0, 0.55, 0.0});
    RigidBody upper = unit_cube(Vector3{0.0, 1.6, 0.0});
    RigidBody ground_body = immovable_body<Scalar>();

    ContactSolveParams<Scalar> params;
    params.static_friction = 0.6;
    params.dynamic_friction = 0.5;
    params.restitution = 0.0;
    params.restitution_threshold = 2.0 * GRAVITY * (1.0 / 60.0 / 16.0);

    ContactManifold<Scalar> lower_ground;
    ContactManifold<Scalar> pair;
    const Scalar dt = 1.0 / 60.0;
    const std::size_t substeps = 16;
    const Scalar h = dt / static_cast<Scalar>(substeps);

    for (int tick = 0; tick < 300; ++tick)
    {
        const ContactManifold<Scalar> previous_ground = lower_ground;
        const ContactManifold<Scalar> previous_pair = pair;
        lower_ground =
            generate_obb_plane_manifold(box_of(lower, Vector3{0.5, 0.5, 0.5}), ground, 0.02);
        pair = generate_obb_obb_manifold(box_of(lower, Vector3{0.5, 0.5, 0.5}),
                                         box_of(upper, Vector3{0.5, 0.5, 0.5}), 0.02);
        warm_start_manifold(lower_ground, previous_ground);
        warm_start_manifold(pair, previous_pair);

        for (std::size_t s = 0; s < substeps; ++s)
        {
            capture_contact_velocities(lower_ground, lower, ground_body);
            capture_contact_velocities(pair, lower, upper);
            if (s > 0)
            {
                clear_manifold_impulses(lower_ground);
                clear_manifold_impulses(pair);
            }
            predict(lower, Vector3{0.0, -GRAVITY, 0.0}, h);
            predict(upper, Vector3{0.0, -GRAVITY, 0.0}, h);
            solve_manifold_positions(lower_ground, lower, ground_body, params);
            solve_manifold_positions(pair, lower, upper, params);
            update_velocity(lower, h);
            update_velocity(upper, h);
            solve_manifold_velocities(lower_ground, lower, ground_body, params, h);
            solve_manifold_velocities(pair, lower, upper, params, h);
        }
    }

    EXPECT_NEAR(lower.position.y, 0.5, 5e-3);
    EXPECT_NEAR(upper.position.y, 1.5, 1e-2);
    EXPECT_GT(rotate(upper.orientation, Vector3{0.0, 1.0, 0.0}).y, 0.999);
    // A stack drifts sideways, slowly, and it is worth being precise about why
    // rather than hiding it behind a loose bound. Sweeping the manifold's points in
    // sequence leaves each body with a residual angular velocity of order 1e-3
    // rad/s — the points are projected against the poses the previous one produced,
    // so the four are not symmetric — and friction faithfully converts that residual
    // spin into a lateral crawl. It is a discretization error, not a defect in the
    // model: it halves for every doubling of the substep count (measured at 8, 16,
    // 32 and 64), which is what a first-order residual does and what a genuine
    // asymmetry would not. At 16 substeps it is about two millimetres per second.
    EXPECT_LT(std::abs(upper.position.x), 0.03);
    EXPECT_LT(std::abs(upper.position.x - lower.position.x), 2e-3);
}

// An immovable body takes none of a correction, which is what makes the same
// projection usable for a pair and for static geometry — one code path, so the
// plane case cannot disagree with the pair case the way it used to (§1.3).
TEST(Unit_ContactProjection, ImmovableBodyAbsorbsNothing)
{
    RigidBody ground_body = immovable_body<Scalar>();
    const Vector3 before = ground_body.position;

    apply_positional_impulse(ground_body, Vector3{0.0, 100.0, 0.0}, Vector3{0.5, 0.0, 0.5}, 1.0);
    apply_velocity_impulse(ground_body, Vector3{0.0, 100.0, 0.0}, Vector3{0.5, 0.0, 0.5}, 1.0);

    EXPECT_NEAR(ground_body.position.y, before.y, 1e-15);
    EXPECT_NEAR(length(ground_body.velocity), 0.0, 1e-15);
    EXPECT_NEAR(length(ground_body.angular_velocity), 0.0, 1e-15);
    EXPECT_NEAR(generalized_inverse_mass(ground_body, Vector3{0.5, 0.0, 0.5},
                                         Vector3{0.0, 1.0, 0.0}),
                0.0, 1e-15);
}

// The tangent basis has to be built the same way every time for the friction
// accumulator to mean anything across substeps, and it has to be orthonormal for
// the clamp to be a circle rather than an ellipse.
TEST(Unit_ContactProjection, TangentBasisIsOrthonormalAndDeterministic)
{
    const Vector3 normals[] = {Vector3{0.0, 1.0, 0.0}, Vector3{1.0, 0.0, 0.0},
                               Vector3{0.0, 0.0, 1.0},
                               normalize(Vector3{0.3, -0.8, 0.5})};
    for (const Vector3& normal : normals)
    {
        Vector3 t0;
        Vector3 t1;
        contact_tangent_basis(normal, t0, t1);
        EXPECT_NEAR(length(t0), 1.0, 1e-12);
        EXPECT_NEAR(length(t1), 1.0, 1e-12);
        EXPECT_NEAR(dot(t0, normal), 0.0, 1e-12);
        EXPECT_NEAR(dot(t1, normal), 0.0, 1e-12);
        EXPECT_NEAR(dot(t0, t1), 0.0, 1e-12);

        Vector3 again_0;
        Vector3 again_1;
        contact_tangent_basis(normal, again_0, again_1);
        EXPECT_NEAR(length(again_0 - t0), 0.0, 1e-15);
        EXPECT_NEAR(length(again_1 - t1), 0.0, 1e-15);
    }
}

// Materials decide the coefficients, and a contact between two of them resolves
// the same way whichever body is named first.
TEST(Unit_ContactProjection, ParametersComeFromTheCombinedMaterials)
{
    PhysicsMaterial rubber;
    rubber.static_friction = 1.2;
    rubber.dynamic_friction = 1.0;
    rubber.restitution = 0.8;
    PhysicsMaterial ice;
    ice.static_friction = 0.1;
    ice.dynamic_friction = 0.05;
    ice.restitution = 0.1;

    const ContactSolveParams<Scalar> forward = make_contact_params(rubber, ice, 0.001, 0.5);
    const ContactSolveParams<Scalar> reversed = make_contact_params(ice, rubber, 0.001, 0.5);

    EXPECT_NEAR(forward.static_friction, reversed.static_friction, 1e-15);
    EXPECT_NEAR(forward.dynamic_friction, reversed.dynamic_friction, 1e-15);
    EXPECT_NEAR(forward.restitution, reversed.restitution, 1e-15);
    // Friction averages by default; restitution takes the grippier mode (maximum).
    EXPECT_NEAR(forward.static_friction, 0.65, 1e-12);
    EXPECT_NEAR(forward.restitution, 0.8, 1e-12);
    EXPECT_NEAR(forward.rest_offset, 0.001, 1e-15);
}
