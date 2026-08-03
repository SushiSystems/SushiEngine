/**************************************************************************/
/* test_conservative_advancement.cpp                                      */
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

// Unit_ConservativeAdvancement: §7.5 tier 2 (physics/collision/conservative_advancement.hpp).
//
// Tier 1 (the widened, per-tick manifold in sim/physics_simulation.hpp) already
// covers straight-line fast motion, so these cases target what only tier 2 can
// answer: an exact time of impact against a closed-form analytic case (two
// spheres closing at a known speed), the two ways a search must terminate
// honestly (separating pairs, and a pair already inside the target separation),
// and the property that motivates the whole file — a body that is not
// translating at all can still tunnel through something if it is spinning fast
// enough, which a linear-only bound can never see and the angular allowance
// here is what catches.

#include <cmath>

#include <gtest/gtest.h>

#include <SushiEngine/physics/collision/conservative_advancement.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    CollisionShape<Scalar> sphere_at(Vector3 center, Scalar radius)
    {
        return make_sphere_shape<Scalar>(center, radius);
    }

    CollisionShape<Scalar> box_at(Vector3 center, Vector3 half_extents)
    {
        return make_box_shape<Scalar>(center, half_extents);
    }

    constexpr Vector3 zero{0.0, 0.0, 0.0};
} // namespace

// Two spheres closing head-on at a known relative speed have a closed-form time
// of impact: the gap between their surfaces divided by the closing speed. This
// is the oracle every other case in this file is checked against indirectly —
// if the simplest possible case does not match arithmetic, nothing else can be
// trusted either.
TEST(Unit_ConservativeAdvancement, SphereSphereMatchesClosedFormTimeOfImpact)
{
    const CollisionShape<Scalar> a = sphere_at(Vector3{0.0, 0.0, 0.0}, 0.5);
    const CollisionShape<Scalar> b = sphere_at(Vector3{10.0, 0.0, 0.0}, 0.5);
    const Vector3 velocity_b{-5.0, 0.0, 0.0};

    const auto result =
        conservative_advance<Scalar>(a, zero, zero, b, velocity_b, zero, 0.0, 3.0);

    ASSERT_TRUE(result.impact);
    // Surfaces are 9 m apart (10 m of centre separation minus the two radii);
    // closing at 5 m/s reaches zero separation at 1.8 s.
    EXPECT_NEAR(result.time_of_impact, 1.8, 1e-3);
    EXPECT_TRUE(result.contact.valid);
    EXPECT_NEAR(result.contact.separation, 0.0, 1e-3);
}

// The same pair, but generated out to a positive target separation exactly the
// way tier 1's contact_offset is: the impact lands earlier by exactly that much
// distance over the same closing speed.
TEST(Unit_ConservativeAdvancement, RespectsAPositiveTargetSeparation)
{
    const CollisionShape<Scalar> a = sphere_at(Vector3{0.0, 0.0, 0.0}, 0.5);
    const CollisionShape<Scalar> b = sphere_at(Vector3{10.0, 0.0, 0.0}, 0.5);
    const Vector3 velocity_b{-5.0, 0.0, 0.0};

    const auto result =
        conservative_advance<Scalar>(a, zero, zero, b, velocity_b, zero, 0.03, 3.0);

    ASSERT_TRUE(result.impact);
    EXPECT_NEAR(result.time_of_impact, (9.0 - 0.03) / 5.0, 1e-3);
}

// Two spheres moving apart never reach the target separation, and the search
// must say so rather than reporting a stale or default time.
TEST(Unit_ConservativeAdvancement, SeparatingPairReportsNoImpact)
{
    const CollisionShape<Scalar> a = sphere_at(Vector3{0.0, 0.0, 0.0}, 0.5);
    const CollisionShape<Scalar> b = sphere_at(Vector3{10.0, 0.0, 0.0}, 0.5);
    const Vector3 velocity_b{5.0, 0.0, 0.0}; // moving further away

    const auto result =
        conservative_advance<Scalar>(a, zero, zero, b, velocity_b, zero, 0.0, 3.0);

    EXPECT_FALSE(result.impact);
}

// A pair already inside the target separation at t = 0 is an immediate impact —
// the search must not require any advance to notice what is already true.
TEST(Unit_ConservativeAdvancement, AlreadyTouchingPairImpactsAtTimeZero)
{
    const CollisionShape<Scalar> a = sphere_at(Vector3{0.0, 0.0, 0.0}, 0.5);
    const CollisionShape<Scalar> b = sphere_at(Vector3{0.9, 0.0, 0.0}, 0.5);

    const auto result =
        conservative_advance<Scalar>(a, zero, zero, b, zero, zero, 0.0, 1.0);

    ASSERT_TRUE(result.impact);
    EXPECT_NEAR(result.time_of_impact, 0.0, 1e-9);
}

// A shape that is only rotating — no translation at all — can still sweep a
// far point into something it started nowhere near. This is the case tier 1
// cannot see (its speculative margin is built from linear velocity alone) and
// the reason this file exists: with the rotation, the search must find the
// impact; with the identical geometry and zero angular velocity, it must not,
// because nothing is moving at all.
TEST(Unit_ConservativeAdvancement, PureRotationCanStillProduceAnImpact)
{
    // A thin rod along local X, centred at the origin, lying flat at t = 0.
    const CollisionShape<Scalar> rod = box_at(Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 0.02, 0.02});
    // Just above the rod's tip — untouched while the rod is flat, but close
    // enough that a small rotation about Z sweeps the tip's neighbourhood
    // upward into it.
    const CollisionShape<Scalar> target = sphere_at(Vector3{1.0, 0.15, 0.0}, 0.05);

    const Vector3 spin{0.0, 0.0, 1.0}; // rad/s about Z

    const auto rotating =
        conservative_advance<Scalar>(rod, zero, spin, target, zero, zero, 0.0, 2.0);
    EXPECT_TRUE(rotating.impact);
    if (rotating.impact)
    {
        EXPECT_GT(rotating.time_of_impact, 0.0);
        EXPECT_LT(rotating.time_of_impact, 2.0);
    }

    const auto stationary =
        conservative_advance<Scalar>(rod, zero, zero, target, zero, zero, 0.0, 2.0);
    EXPECT_FALSE(stationary.impact);
}

// needs_conservative_advancement is the trigger sim/ reads to decide whether a
// pair is worth this tier at all: a body whose motion this tick is large next
// to its own thinnest dimension needs it, and one that barely moves does not.
TEST(Unit_ConservativeAdvancement, NeedsAdvancementTracksMotionAgainstThinness)
{
    // A plate 4 cm thick, moving at 200 m/s: several multiples of its own
    // thickness in a 1/60 s tick.
    const CollisionShape<Scalar> fast_plate =
        box_at(Vector3{0.0, 0.0, 0.0}, Vector3{1.0, 1.0, 0.02});
    const Scalar tick = 1.0 / 60.0;
    EXPECT_TRUE(needs_conservative_advancement<Scalar>(fast_plate, Vector3{0.0, 0.0, 200.0},
                                                        zero, tick));

    // The same plate, barely drifting: nowhere near its own thickness this tick.
    EXPECT_FALSE(needs_conservative_advancement<Scalar>(fast_plate, Vector3{0.0, 0.0, 0.01},
                                                         zero, tick));

    // A thick, slow crate needs nothing either.
    const CollisionShape<Scalar> crate = box_at(Vector3{0.0, 0.0, 0.0}, Vector3{0.5, 0.5, 0.5});
    EXPECT_FALSE(
        needs_conservative_advancement<Scalar>(crate, Vector3{0.0, 0.0, 1.0}, zero, tick));
}

// A half-space plane is not a bounded convex set GJK can be asked about, and it
// never needs this tier (its own analytic separation is exact at any range) —
// the distance dispatch must say "unregistered" rather than guess, and the
// thinnest-extent query must never trigger the tier for one.
TEST(Unit_ConservativeAdvancement, PlanesAreExcludedFromThisTier)
{
    const CollisionShape<Scalar> plane = make_plane_shape<Scalar>(Vector3{0.0, 1.0, 0.0}, 0.0);
    const CollisionShape<Scalar> sphere = sphere_at(Vector3{0.0, 5.0, 0.0}, 0.5);

    const auto result = conservative_advance<Scalar>(sphere, Vector3{0.0, -50.0, 0.0}, zero,
                                                       plane, zero, zero, 0.0, 1.0);
    EXPECT_FALSE(result.impact);

    EXPECT_FALSE(needs_conservative_advancement<Scalar>(plane, zero, zero, 1.0 / 60.0));
}
