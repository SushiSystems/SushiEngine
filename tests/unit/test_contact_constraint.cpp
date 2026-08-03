/**************************************************************************/
/* test_contact_constraint.cpp                                            */
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

// Unit_ContactConstraint: a contact as a constraint kind the solver holds (§6.3).
//
// `test_contact_projection.cpp` already proves the *arithmetic* — a box bounces to
// the height its restitution says, a ramp below the friction angle holds, a stack
// settles flat. It proves it by driving the projection functions directly, which is
// the right way to measure physics and says nothing about whether the solver can
// actually carry contacts.
//
// This suite is about exactly that gap. Two claims, and they are separable:
//
//   - **The layout is right.** A contact takes a colour from the union of every
//     constraint kind, so a contact and a distance constraint sharing a body never
//     share a colour — which is what stops two graph nodes writing one body at once.
//     Static geometry holds no colour, because it takes no correction.
//   - **The schedule survives the seam.** Submitting manifolds to `IConstraintSolver`
//     and stepping gives the same resting height, the same warm-started impulses and
//     the same settled stack as driving the projections by hand.
//
// The host solver is the subject; the runtime-backed one is held to the same
// outcomes by `test_solver_conformance.cpp`, which is where a device is stood up.

#include <cmath>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/solver/contact_store.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    constexpr Scalar GRAVITY = 9.81;

    /** @brief The budget every scene here runs under. */
    PhysicsConfiguration contact_scene()
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = 64;
        configuration.capacities.constraints = 256;
        configuration.capacities.contacts = 256;
        configuration.capacities.colors = 8;
        configuration.substeps.minimum = 8;
        configuration.substeps.maximum = 8;
        return configuration;
    }

    /** @brief A unit-mass cube of the given half-extent, at rest at @p position. */
    RigidBody unit_cube(Vector3 position, Scalar half_extent = 0.5, Scalar mass = 1.0)
    {
        RigidBody body;
        body.position = position;
        body.prev_position = position;
        body.orientation = Quaternion{0.0, 0.0, 0.0, 1.0};
        body.prev_orientation = body.orientation;
        body.inv_mass = 1.0 / mass;
        const Scalar edge = 2.0 * half_extent;
        const Scalar inertia = mass * edge * edge / 6.0;
        body.inv_inertia = Vector3{1.0 / inertia, 1.0 / inertia, 1.0 / inertia};
        return body;
    }

    /** @brief Frictionless, bounceless contact parameters with the anti-jitter floor set. */
    ContactSolveParams<Scalar> plain_params(Scalar substep)
    {
        ContactSolveParams<Scalar> params;
        params.static_friction = 0.0;
        params.dynamic_friction = 0.0;
        params.restitution = 0.0;
        params.restitution_threshold = 2.0 * GRAVITY * substep;
        return params;
    }

    /**
     * @brief A single-point contact holding a unit cube at @p position on the ground.
     *
     * For the tests that measure bookkeeping — order, counts, refusals — rather than
     * physics. It is still a *valid* contact, because a contact the solve discards as
     * degenerate would let a bookkeeping test pass for the wrong reason.
     */
    ContactManifold<Scalar> ground_point_manifold(const Vector3& position)
    {
        const Quaternion identity{0.0, 0.0, 0.0, 1.0};
        return make_point_manifold(Vector3{0.0, -1.0, 0.0},
                                   Vector3{position.x, position.y - 0.5, position.z},
                                   Vector3{position.x, 0.0, position.z}, position.y - 0.5,
                                   position, identity, Vector3{0.0, 0.0, 0.0}, identity, 0u);
    }

    /**
     * @brief One tick of the loop a scene actually runs: regenerate, warm start, submit, step.
     *
     * The shape is the point. Manifolds are generated from the poses the solver
     * currently holds, inherited from last tick's so the accumulators carry, handed
     * over as contacts, and read back afterwards so the next tick can inherit them
     * again. Nothing here reaches into the solve; the seam is the only contact.
     */
    void tick_on_ground(IConstraintSolver<Scalar>& solver, const std::vector<BodyHandle>& handles,
                        std::vector<ContactManifold<Scalar>>& manifolds, Vector3 half_extents,
                        const ContactSolveParams<Scalar>& params, Scalar dt)
    {
        const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
        // Generated further out than they are resolved to: an offset that did not
        // cover a tick's motion would leave a corner arriving mid-tick unconstrained
        // until the next one, by which time it is already deep (§7.6).
        const Scalar contact_offset = params.rest_offset + 0.03;

        solver.begin_contacts();
        for (std::size_t i = 0; i < handles.size(); ++i)
        {
            RigidBody body;
            ASSERT_TRUE(solver.read_body(handles[i], body));

            const ContactManifold<Scalar> previous = manifolds[i];
            manifolds[i] = generate_obb_plane_manifold(
                OrientedBox<Scalar>{body.position, half_extents, body.orientation}, ground,
                contact_offset);
            warm_start_manifold(manifolds[i], previous);
            if (manifolds[i].point_count == 0)
                continue;

            ContactConstraint contact;
            contact.a = std::uint32_t(solver.body_slot(handles[i]));
            contact.b = null_contact_body;
            contact.key = i;
            contact.manifold = manifolds[i];
            contact.params = params;
            solver.add_contact(contact);
        }

        StepParameters<Scalar> parameters;
        parameters.delta_time = dt;
        parameters.gravity = Vector3{0.0, -GRAVITY, 0.0};
        solver.step(parameters);

        // The accumulators the solve settled on come back out, or warm starting has
        // nothing to inherit and a resting box rebuilds its normal impulse from zero
        // every tick.
        for (std::size_t i = 0; i < solver.contact_count(); ++i)
        {
            ContactConstraint solved;
            ASSERT_TRUE(solver.read_contact(i, solved));
            manifolds[solved.key] = solved.manifold;
        }
    }
} // namespace

// ---------------------------------------------------------------------------
// The layout: colouring over the union of the kinds.
// ---------------------------------------------------------------------------

TEST(Unit_ContactConstraint, ContactsSharingABodyTakeDifferentColours)
{
    // The property the whole colouring exists for. Two contacts on body 0 projected
    // in one node would have two lanes correcting one body's position concurrently.
    IncrementalColoring persistent(16, 8);
    ContactStore store(16, 64, 8);
    store.begin();

    const ContactPlacement first = store.place(0, 1, persistent);
    const ContactPlacement second = store.place(0, 2, persistent);

    ASSERT_TRUE(first.placed);
    ASSERT_TRUE(second.placed);
    EXPECT_NE(first.color, second.color);

    // And two that share nothing may share one, or colouring would be pointless.
    const ContactPlacement third = store.place(3, 4, persistent);
    ASSERT_TRUE(third.placed);
    EXPECT_EQ(third.color, first.color);
}

TEST(Unit_ContactConstraint, StaticGeometryHoldsNoColour)
{
    // A hundred crates on one floor is the common case, and if the floor held a
    // colour it would serialize every one of them into its own node.
    IncrementalColoring persistent(16, 8);
    ContactStore store(16, 64, 8);
    store.begin();

    const ContactPlacement first = store.place(0, null_contact_body, persistent);
    const ContactPlacement second = store.place(1, null_contact_body, persistent);

    ASSERT_TRUE(first.placed);
    ASSERT_TRUE(second.placed);
    EXPECT_EQ(first.color, second.color);
    EXPECT_EQ(store.band_size(first.color), 2u);
}

TEST(Unit_ContactConstraint, AContactAvoidsTheColourAPersistentConstraintHolds)
{
    // §6.3's union requirement, stated as a test. The distance constraint took
    // colour 0 on body 0; a contact on body 0 that also took colour 0 would be
    // projected by a node running beside the one projecting the constraint.
    IncrementalColoring persistent(16, 8);
    const std::uint32_t taken = persistent.assign(0, 1);
    ASSERT_EQ(taken, 0u);

    ContactStore store(16, 64, 8);
    store.begin();
    const ContactPlacement placement = store.place(0, 5, persistent);

    ASSERT_TRUE(placement.placed);
    EXPECT_EQ(placement.color, 1u);
}

TEST(Unit_ContactConstraint, ATickForgetsTheLastOne)
{
    // Contacts have no lifetime past a tick, so the colouring must not either: a
    // store that remembered would run out of colours after eight ticks of a crate
    // resting on a floor.
    IncrementalColoring persistent(16, 8);
    ContactStore store(16, 64, 8);

    for (int tick = 0; tick < 32; ++tick)
    {
        store.begin();
        const ContactPlacement placement = store.place(0, null_contact_body, persistent);
        ASSERT_TRUE(placement.placed);
        EXPECT_EQ(placement.color, 0u);
        EXPECT_EQ(store.live_count(), 1u);
    }
}

TEST(Unit_ContactConstraint, AFullBandRefusesRatherThanOverwriting)
{
    // Capacity is a hard ceiling, and exceeding it is a budget being exceeded, not
    // an error: what a refused contact costs is a little penetration for one tick.
    IncrementalColoring persistent(16, 8);
    ContactStore store(64, 8, 1); // one colour, so the band is the whole budget
    store.begin();

    for (int i = 0; i < 8; ++i)
        EXPECT_TRUE(store.place(std::uint32_t(i), null_contact_body, persistent).placed);
    EXPECT_FALSE(store.place(9, null_contact_body, persistent).placed);
    EXPECT_EQ(store.live_count(), 8u);
}

// ---------------------------------------------------------------------------
// The schedule, through the solver seam.
// ---------------------------------------------------------------------------

TEST(Unit_ContactConstraint, ADroppedBoxComesToRestThroughTheSolver)
{
    // The same outcome `test_contact_projection.cpp` measures by driving the
    // projections directly. Reaching it through `add_contact` and `step` is what
    // says the contact kind is wired into the substep loop in the right places —
    // a preparation after predict, or a velocity pass before it, lands elsewhere.
    const PhysicsConfiguration configuration = contact_scene();
    HostXPBDSolver<Scalar> solver(configuration);

    const BodyHandle handle = solver.add_body(unit_cube(Vector3{0.0, 2.0, 0.0}));
    ASSERT_TRUE(handle.valid());

    std::vector<BodyHandle> handles{handle};
    std::vector<ContactManifold<Scalar>> manifolds(1);
    const Scalar dt = 1.0 / 60.0;
    const ContactSolveParams<Scalar> params = plain_params(dt / 8.0);

    for (int tick = 0; tick < 180; ++tick)
        tick_on_ground(solver, handles, manifolds, Vector3{0.5, 0.5, 0.5}, params, dt);

    RigidBody body;
    ASSERT_TRUE(solver.read_body(handle, body));
    EXPECT_NEAR(body.position.y, 0.5, 1e-3);
    EXPECT_LT(std::abs(body.velocity.y), 1e-2);
    EXPECT_LT(length(body.angular_velocity), 5e-3);
}

TEST(Unit_ContactConstraint, TheSolvedImpulsesComeBackOut)
{
    // Warm starting depends entirely on this. A resting box's normal impulse is
    // roughly `m * g * h` spread over its four corners, and if the readback gave
    // zeros the friction cone would be empty at the top of every tick.
    const PhysicsConfiguration configuration = contact_scene();
    HostXPBDSolver<Scalar> solver(configuration);

    const BodyHandle handle = solver.add_body(unit_cube(Vector3{0.0, 0.55, 0.0}));
    std::vector<BodyHandle> handles{handle};
    std::vector<ContactManifold<Scalar>> manifolds(1);
    const Scalar dt = 1.0 / 60.0;
    const ContactSolveParams<Scalar> params = plain_params(dt / 8.0);

    for (int tick = 0; tick < 60; ++tick)
        tick_on_ground(solver, handles, manifolds, Vector3{0.5, 0.5, 0.5}, params, dt);

    ASSERT_EQ(solver.contact_count(), 1u);
    ContactConstraint solved;
    ASSERT_TRUE(solver.read_contact(0, solved));
    ASSERT_EQ(solved.manifold.point_count, 4u);

    Scalar total = 0.0;
    for (std::size_t i = 0; i < solved.manifold.point_count; ++i)
    {
        EXPECT_GT(solved.manifold.points[i].normal_lambda, 0.0);
        total += solved.manifold.points[i].normal_lambda;
    }
    // One substep of gravity, in impulse units: the four corners together hold the
    // box up, so the sum is what one substep of weight would have moved it by.
    EXPECT_NEAR(total, GRAVITY * (dt / 8.0) * (dt / 8.0), 1e-4);
}

TEST(Unit_ContactConstraint, SubmissionOrderIsWhatComesBack)
{
    // Storage order is a colouring — an implementation detail no caller can predict.
    // A caller that matched solved contacts to its own manifolds by storage order
    // would silently pair the wrong impulses with the wrong pairs.
    const PhysicsConfiguration configuration = contact_scene();
    HostXPBDSolver<Scalar> solver(configuration);

    std::vector<BodyHandle> handles;
    for (int i = 0; i < 4; ++i)
        handles.push_back(solver.add_body(unit_cube(Vector3{Scalar(i) * 4.0, 0.4, 0.0})));

    solver.begin_contacts();
    for (std::size_t i = 0; i < handles.size(); ++i)
    {
        ContactConstraint contact;
        contact.a = std::uint32_t(solver.body_slot(handles[i]));
        contact.b = null_contact_body;
        contact.key = 100 + i;
        contact.manifold = ground_point_manifold(Vector3{Scalar(i) * 4.0, 0.4, 0.0});
        contact.params = plain_params(1.0 / 480.0);
        ASSERT_TRUE(solver.add_contact(contact));
    }

    ASSERT_EQ(solver.contact_count(), handles.size());
    for (std::size_t i = 0; i < solver.contact_count(); ++i)
    {
        ContactConstraint solved;
        ASSERT_TRUE(solver.read_contact(i, solved));
        EXPECT_EQ(solved.key, 100u + i);
    }

    // And the next submission starts empty rather than appending to this one.
    solver.begin_contacts();
    EXPECT_EQ(solver.contact_count(), 0u);
}

TEST(Unit_ContactConstraint, AStackOfBoxesSettlesThroughTheSolver)
{
    // Body-to-body contacts, which is the case the static sentinel does not cover:
    // both sides take a correction, and the pair must colour against each other.
    const PhysicsConfiguration configuration = contact_scene();
    HostXPBDSolver<Scalar> solver(configuration);

    constexpr std::size_t COUNT = 4;
    std::vector<BodyHandle> handles;
    for (std::size_t i = 0; i < COUNT; ++i)
        handles.push_back(solver.add_body(unit_cube(Vector3{0.0, 0.5 + Scalar(i) * 1.02, 0.0})));

    const Vector3 half{0.5, 0.5, 0.5};
    const PlaneCollider<Scalar> ground{Vector3{0.0, 1.0, 0.0}, 0.0};
    const Scalar dt = 1.0 / 60.0;
    const ContactSolveParams<Scalar> params = plain_params(dt / 8.0);
    const Scalar contact_offset = 0.03;

    std::vector<ContactManifold<Scalar>> ground_manifolds(COUNT);
    std::vector<ContactManifold<Scalar>> pair_manifolds(COUNT);

    for (int tick = 0; tick < 240; ++tick)
    {
        std::vector<RigidBody> bodies(COUNT);
        for (std::size_t i = 0; i < COUNT; ++i)
            ASSERT_TRUE(solver.read_body(handles[i], bodies[i]));

        solver.begin_contacts();
        for (std::size_t i = 0; i < COUNT; ++i)
        {
            const OrientedBox<Scalar> box{bodies[i].position, half, bodies[i].orientation};

            const ContactManifold<Scalar> previous_ground = ground_manifolds[i];
            ground_manifolds[i] = generate_obb_plane_manifold(box, ground, contact_offset);
            warm_start_manifold(ground_manifolds[i], previous_ground);
            if (ground_manifolds[i].point_count > 0)
            {
                ContactConstraint contact;
                contact.a = std::uint32_t(solver.body_slot(handles[i]));
                contact.b = null_contact_body;
                contact.key = i;
                contact.manifold = ground_manifolds[i];
                contact.params = params;
                solver.add_contact(contact);
            }

            if (i + 1 >= COUNT)
                continue;
            const OrientedBox<Scalar> above{bodies[i + 1].position, half,
                                            bodies[i + 1].orientation};
            const ContactManifold<Scalar> previous_pair = pair_manifolds[i];
            pair_manifolds[i] = generate_obb_obb_manifold(box, above, contact_offset);
            warm_start_manifold(pair_manifolds[i], previous_pair);
            if (pair_manifolds[i].point_count == 0)
                continue;
            ContactConstraint contact;
            contact.a = std::uint32_t(solver.body_slot(handles[i]));
            contact.b = std::uint32_t(solver.body_slot(handles[i + 1]));
            contact.key = COUNT + i;
            contact.manifold = pair_manifolds[i];
            contact.params = params;
            solver.add_contact(contact);
        }

        StepParameters<Scalar> parameters;
        parameters.delta_time = dt;
        parameters.gravity = Vector3{0.0, -GRAVITY, 0.0};
        solver.step(parameters);

        for (std::size_t k = 0; k < solver.contact_count(); ++k)
        {
            ContactConstraint solved;
            ASSERT_TRUE(solver.read_contact(k, solved));
            if (solved.key < COUNT)
                ground_manifolds[solved.key] = solved.manifold;
            else
                pair_manifolds[solved.key - COUNT] = solved.manifold;
        }
    }

    // Each box rests on the one below with its faces touching, to within what a
    // Gauss-Seidel sweep leaves as residual penetration in a four-high stack.
    for (std::size_t i = 0; i < COUNT; ++i)
    {
        RigidBody body;
        ASSERT_TRUE(solver.read_body(handles[i], body));
        EXPECT_NEAR(body.position.y, 0.5 + Scalar(i), 2e-2) << "box " << i;
        EXPECT_LT(length(body.velocity), 5e-2) << "box " << i;
    }
}

TEST(Unit_ContactConstraint, TheStatisticsCountWhatWasSubmitted)
{
    // The Physics panel reads these, and a zero where a number belongs is worse
    // than no field at all — it reads as "no contacts" rather than "not measured".
    const PhysicsConfiguration configuration = contact_scene();
    HostXPBDSolver<Scalar> solver(configuration);

    const BodyHandle handle = solver.add_body(unit_cube(Vector3{0.0, 0.4, 0.0}));
    std::vector<BodyHandle> handles{handle};
    std::vector<ContactManifold<Scalar>> manifolds(1);
    const Scalar dt = 1.0 / 60.0;

    tick_on_ground(solver, handles, manifolds, Vector3{0.5, 0.5, 0.5}, plain_params(dt / 8.0), dt);

    EXPECT_EQ(solver.statistics().manifolds, 1u);
    EXPECT_EQ(solver.statistics().contact_points, 4u);
}

TEST(Unit_ContactConstraint, ARefusedContactIsCountedNotSwallowed)
{
    PhysicsConfiguration configuration = contact_scene();
    configuration.capacities.contacts = 2;
    configuration.capacities.colors = 1;
    HostXPBDSolver<Scalar> solver(configuration);

    std::vector<BodyHandle> handles;
    for (int i = 0; i < 3; ++i)
        handles.push_back(solver.add_body(unit_cube(Vector3{Scalar(i) * 4.0, 0.4, 0.0})));

    solver.begin_contacts();
    int placed = 0;
    for (std::size_t i = 0; i < handles.size(); ++i)
    {
        ContactConstraint contact;
        contact.a = std::uint32_t(solver.body_slot(handles[i]));
        contact.b = null_contact_body;
        contact.manifold = ground_point_manifold(Vector3{Scalar(i) * 4.0, 0.4, 0.0});
        if (solver.add_contact(contact))
            ++placed;
    }

    EXPECT_EQ(placed, 2);
    EXPECT_EQ(solver.statistics().capacity_overflows, 1u);
}
