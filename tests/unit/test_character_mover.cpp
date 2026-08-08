/**************************************************************************/
/* test_character_mover.cpp                                               */
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

// Unit_CharacterMover: collide-and-slide, stepping, and the slope rule, against a world
// made of half-spaces rather than a scene (P9-B, §16.47).
//
// The point of testing it this way is that it can be: the mover takes its sweep as a
// callable, so a "world" here is a list of planes and a hit is arithmetic. No broadphase,
// no solver, no device. What that buys is the ability to state a geometric claim exactly
// — a 44-degree ramp is walkable and a 46-degree one is not — where an integration test
// would have to settle for "roughly stopped".
//
// The last test is the one worth keeping when the others look redundant: it runs the whole
// suite's geometry about a non-Y up vector, and fails the moment any slope-related line in
// the mover reaches for a world axis instead of the one it was handed.

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/character/character_mover.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    using Real = double;
    using Vec = Vector3T<Real>;

    /**
     * @brief One solid half-space: everything below @ref normal through @ref point.
     *
     * A plane rather than a box because every case here is about a *face* — a floor, a
     * wall, a ramp, the top of a step — and a face is what a capsule sweep resolves
     * against. Building boxes would add a second thing that could be wrong.
     */
    struct Plane
    {
        Vec point;
        Vec normal;
    };

    /**
     * @brief The signed distance from a capsule's surface to a plane.
     *
     * A capsule is a segment swept by a sphere, so its distance to a plane is the
     * nearer endpoint's distance minus the radius — which is the whole reason the
     * capsule is cheap and the reason this stub is four lines rather than a narrowphase.
     */
    Real capsule_distance(const CapsuleCollider<Real>& capsule, const Plane& plane)
    {
        const Vec axis = rotate(capsule.orientation, Vec{0, 1, 0}) * capsule.half_height;
        const Real low = dot(capsule.center - axis - plane.point, plane.normal);
        const Real high = dot(capsule.center + axis - plane.point, plane.normal);
        return (low < high ? low : high) - capsule.radius;
    }

    /**
     * @brief A world of half-spaces, answering the one question the mover asks.
     *
     * Advances in small steps rather than solving for the impact analytically. The mover
     * is what is under test, and a stub that is itself a clever root-finder is a stub
     * that can be wrong in a way that looks like the mover being wrong.
     */
    class PlaneWorld
    {
        public:
            std::vector<Plane> planes;

            RayHit<Real> operator()(const CapsuleCollider<Real>& capsule,
                                    const Vec& direction, Real distance) const
            {
                constexpr Real STEP = 1e-3;
                constexpr Real TOUCH = 1e-4;
                RayHit<Real> hit;
                for (Real travelled = 0; travelled <= distance; travelled += STEP)
                {
                    CapsuleCollider<Real> moved = capsule;
                    moved.center = capsule.center + direction * travelled;
                    for (const Plane& plane : planes)
                    {
                        if (capsule_distance(moved, plane) > TOUCH)
                            continue;
                        // Touching is not blocking, and the real `sweep_shape` says so the
                        // same way: a capsule standing on the floor is at zero separation
                        // from it, and a step along that floor is not a collision with it.
                        // Without this the stub stops every walk at distance zero against
                        // the ground the character is standing on.
                        if (dot(direction, plane.normal) >= -1e-9)
                            continue;
                        hit.hit = true;
                        hit.distance = travelled;
                        hit.normal = plane.normal;
                        hit.point = moved.center;
                        return hit;
                    }
                }
                return hit;
            }
    };

    /** @brief A standing capsule: 0.4 m radius, 1.8 m tall overall. */
    CapsuleCollider<Real> standing_capsule(const Vec& center)
    {
        CapsuleCollider<Real> capsule;
        capsule.center = center;
        capsule.radius = 0.4;
        capsule.half_height = 0.5;
        return capsule;
    }

    /** @brief The default dials, with stepping on and a 45-degree slope limit. */
    CharacterMoveSettings<Real> settings()
    {
        return CharacterMoveSettings<Real>{};
    }

    /** @brief A ground plane at y = 0. */
    Plane floor_plane()
    {
        return Plane{Vec{0, 0, 0}, Vec{0, 1, 0}};
    }

    /** @brief A wall facing -x, standing at x = @p x. */
    Plane wall_at(Real x)
    {
        return Plane{Vec{x, 0, 0}, Vec{-1, 0, 0}};
    }

    /** @brief A ramp rising along +x at @p degrees, through the origin. */
    Plane ramp(Real degrees)
    {
        const Real radians = degrees * 3.14159265358979323846 / 180.0;
        return Plane{Vec{0, 0, 0}, Vec{-std::sin(radians), std::cos(radians), 0}};
    }

    /** @brief The capsule's resting centre height above a flat floor. */
    constexpr Real REST_Y = 0.9;
}

TEST(Unit_CharacterMover, UnobstructedMotionIsSpentInFull)
{
    // The baseline. If this is wrong nothing after it means anything, and it is also the
    // only test that pins down what `remaining` reads when nothing was in the way.
    PlaneWorld world{{floor_plane()}};
    const CharacterMoveResult<Real> result = move_character(
        world, standing_capsule(Vec{0, REST_Y, 0}), Vec{2, 0, 0}, Vec{0, 1, 0}, settings());

    EXPECT_NEAR(result.position.x, 2.0, 1e-2);
    EXPECT_NEAR(length(result.remaining), 0.0, 1e-6);
    EXPECT_TRUE(result.grounded) << "a capsule resting on a floor is not airborne";
}

TEST(Unit_CharacterMover, AWallIsSlidAlongRatherThanStoppedDead)
{
    // Walking diagonally into a wall must keep the component parallel to it. A controller
    // that stopped at the first hit would make every wall a glue trap, which is what
    // "collide and slide" exists to avoid.
    PlaneWorld world{{floor_plane(), wall_at(1.0)}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, REST_Y, 0}), Vec{1, 0, 1}, Vec{0, 1, 0},
                       settings());

    EXPECT_LT(result.position.x, 0.7) << "the capsule went through the wall";
    EXPECT_GT(result.position.z, 0.5) << "the along-wall component was thrown away";
}

TEST(Unit_CharacterMover, AWallIsNeverClimbedByPressingIntoIt)
{
    // The bug this algorithm is most often shipped with. Projecting motion onto a
    // vertical face leaves an upward component whenever the input has any, so a
    // character walking into a wall while falling can be lifted by its own fall. The
    // mover removes the up-component of a slide along an unwalkable face; without that
    // line this test rises.
    PlaneWorld world{{floor_plane(), wall_at(1.0)}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, REST_Y, 0}), Vec{2, 0, 0}, Vec{0, 1, 0},
                       settings());

    EXPECT_NEAR(result.position.y, REST_Y, 1e-2) << "walking into a wall lifted the capsule";
    EXPECT_GT(length(result.remaining), 0.5)
        << "a blocked walk must report what it could not spend";
}

TEST(Unit_CharacterMover, AStepBelowTheStepHeightIsClimbed)
{
    // A stair is a wall the controller is allowed to go over. Two planes make one: a
    // riser at x = 1 and a tread at y = 0.3, which is inside the 0.4 default.
    PlaneWorld world{{floor_plane(), wall_at(1.0), Plane{Vec{0, 0.3, 0}, Vec{0, 1, 0}}}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, REST_Y, 0}), Vec{1.2, 0, 0}, Vec{0, 1, 0},
                       settings());

    EXPECT_TRUE(result.stepped) << "the step was treated as a wall";
    EXPECT_GT(result.position.x, 1.0) << "the capsule did not cross the riser";
    EXPECT_NEAR(result.position.y, REST_Y + 0.3, 5e-2) << "it did not end up on the tread";
}

TEST(Unit_CharacterMover, AStepAboveTheStepHeightIsNotClimbed)
{
    // The same scene with the tread raised past the limit. What makes this worth its own
    // test is that the failure mode is not "does not climb" — it is climbing anyway,
    // because the down-sweep of the step attempt finds *some* floor and accepts it.
    CharacterMoveSettings<Real> dials = settings();
    dials.step_height = 0.2;
    PlaneWorld world{{floor_plane(), wall_at(1.0), Plane{Vec{0, 0.5, 0}, Vec{0, 1, 0}}}};
    const CharacterMoveResult<Real> result = move_character(
        world, standing_capsule(Vec{0, REST_Y, 0}), Vec{1.2, 0, 0}, Vec{0, 1, 0}, dials);

    EXPECT_FALSE(result.stepped) << "a step taller than the limit was climbed";
    EXPECT_NEAR(result.position.y, REST_Y, 1e-2) << "the capsule rose without stepping";
}

TEST(Unit_CharacterMover, ARampBelowTheSlopeLimitIsWalkable)
{
    PlaneWorld world{{ramp(30.0)}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, 0.5, 0}), Vec{0, -0.2, 0}, Vec{0, 1, 0},
                       settings());

    EXPECT_TRUE(result.grounded) << "a 30-degree ramp is a floor";
    EXPECT_GT(dot(result.ground_normal, Vec{0, 1, 0}), 0.8)
        << "the ground normal did not come from the ramp";
}

TEST(Unit_CharacterMover, ARampAboveTheSlopeLimitIsAWall)
{
    // 60 degrees against the 45-degree default. Standing on it must read as airborne,
    // because a character that counts a cliff face as ground can jump off one.
    PlaneWorld world{{ramp(60.0)}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, 0.5, 0}), Vec{0, -0.2, 0}, Vec{0, 1, 0},
                       settings());

    EXPECT_FALSE(result.grounded) << "a 60-degree face was reported as standable";
}

TEST(Unit_CharacterMover, ACeilingRejectsTheStepAttempt)
{
    // The first of the step's three sweeps, and the one whose absence is invisible in a
    // flat scene: without the up-sweep a character in a low tunnel steps *through* the
    // ceiling to get onto a crate.
    PlaneWorld world{{floor_plane(), wall_at(1.0), Plane{Vec{0, 0.3, 0}, Vec{0, 1, 0}},
                      Plane{Vec{0, REST_Y + 0.55, 0}, Vec{0, -1, 0}}}};
    const CharacterMoveResult<Real> result =
        move_character(world, standing_capsule(Vec{0, REST_Y, 0}), Vec{1.2, 0, 0}, Vec{0, 1, 0},
                       settings());

    EXPECT_FALSE(result.stepped) << "the capsule stepped up through a ceiling";
}

TEST(Unit_CharacterMover, TheGeometryHoldsAboutANonVerticalUp)
{
    // The test this file exists for. Everything above runs about world +Y, which is
    // exactly the axis a controller is most likely to have hardcoded — so all of it
    // would pass on a mover that ignores its `up` argument entirely.
    //
    // Here the whole scene is rotated so up is +X: a floor facing +X, a wall facing +Y,
    // and a walk along +Y into it. A mover that reaches for world +Y anywhere reads this
    // floor as a 90-degree cliff and this wall as level ground, and every assertion
    // below inverts.
    const Vec up{1, 0, 0};
    PlaneWorld world{{Plane{Vec{0, 0, 0}, Vec{1, 0, 0}}, Plane{Vec{0, 1, 0}, Vec{0, -1, 0}}}};

    CapsuleCollider<Real> capsule = standing_capsule(Vec{REST_Y, 0, 0});
    // Lying along +X so the capsule stands in this world's up, the same way the ones
    // above stand in theirs.
    capsule.orientation = quaternion_axis_angle(Vec{0, 0, 1}, 3.14159265358979323846 / 2.0);

    const CharacterMoveResult<Real> result =
        move_character(world, capsule, Vec{0, 2, 0}, up, settings());

    EXPECT_TRUE(result.grounded) << "the floor was not recognised about a non-Y up";
    EXPECT_NEAR(dot(result.ground_normal, up), 1.0, 1e-6)
        << "the ground normal came from the wall, not the floor";
    EXPECT_NEAR(dot(result.position, up), REST_Y, 5e-2)
        << "sliding along the wall moved the capsule along the up axis";
    EXPECT_LT(result.position.y, 0.7) << "the capsule went through the wall";
}
