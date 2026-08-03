/**************************************************************************/
/* test_physics_authoring.cpp                                             */
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

/**
 * @file test_physics_authoring.cpp
 * @brief What an author can now reach, proved by building it and stepping it.
 *
 * Two things were built and unreachable before this: §5.3's surface materials, which
 * existed as a type nothing read — every contact in the world solved at one hard-coded
 * friction whatever a body's material said — and §7.7's collision filter, which the
 * physics honoured but nothing could author. Both are now fields on the Collider, and
 * these are the tests that they are fields with *consequences*.
 *
 * Built through the sample scene rather than by hand wherever possible, because that
 * scene is the exposure work's own claim — "everything P0 to P7 built is reachable
 * without writing C++" — and a claim nothing tests is a claim.
 */

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <SushiEngine/simulation/simulation.hpp>

#include <SushiEngine/authoring/command_history.hpp>
#include "physics_sample_scene.hpp"
#include "scene_serializer.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    /** @brief Empties the world, demo seeds included, so a test builds from zero. */
    void clear_world(IWorldEditor& world)
    {
        for (const EntityId id : world.entities())
            world.destroy(id);
    }

    /** @brief The first entity carrying @p name, or `NULL_ENTITY`. */
    EntityId find_by_name(IWorldEditor& world, const std::string& name)
    {
        for (const EntityId id : world.entities())
            if (world.name(id) == name)
                return id;
        return NULL_ENTITY;
    }

    /** @brief Runs @p count fixed steps. */
    void step(ISimulation& simulation, int count)
    {
        for (int i = 0; i < count; ++i)
            simulation.tick(simulation.fixed_dt_seconds());
    }

    /**
     * @brief A box of @p half_size at @p position with a body and an authored surface.
     *
     * Everything a surface test needs and nothing it does not: the two boxes it builds
     * must differ in *only* the material, so every other argument is shared.
     */
    EntityId make_block(IWorldEditor& world, const std::string& name, const Vector3& position,
                        Scalar static_friction, Scalar dynamic_friction, Scalar restitution)
    {
        const EntityId id = world.create_box(name);
        EntityTransform transform = world.transform(id);
        transform.position = position;
        transform.scale = Vector3{0.5, 0.5, 0.5};
        world.set_transform(id, transform);

        PhysicsBodyParameters body;
        body.density = Scalar(1000);
        world.set_has_physics_body(id, true);
        world.set_physics_body_params(id, body);

        ColliderParameters collider = world.collider_params(id);
        collider.static_friction = static_friction;
        collider.dynamic_friction = dynamic_friction;
        collider.restitution = restitution;
        world.set_collider_params(id, collider);
        return id;
    }

    /** @brief A large static box acting as a floor, with its own authored surface. */
    EntityId make_floor(IWorldEditor& world, const Vector3& position)
    {
        const EntityId id = world.create_box("Floor");
        EntityTransform transform = world.transform(id);
        transform.position = position;
        transform.scale = Vector3{20, 1, 20};
        world.set_transform(id, transform);

        // Static: inv_mass = 0 pins it in place. Without a physics body at all the
        // entity never becomes a RigidBodyDescription (extract_rigid_bodies requires
        // has_physics_body) and, being a box rather than a Plane primitive, it is
        // not picked up by extract_static_planes either -- it would simply not
        // exist for anything to fall onto.
        PhysicsBodyParameters body;
        body.inv_mass = 0;
        world.set_has_physics_body(id, true);
        world.set_physics_body_params(id, body);
        return id;
    }
}

TEST(Integration_PhysicsAuthoring, RestitutionDecidesWhetherABodyBounces)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    make_floor(world, Vector3{0, 0, 0});
    // Same height, same density, same shape. The only difference is one number.
    const EntityId dead = make_block(world, "Dead", Vector3{-3, 6, 0}, Scalar(0.6), Scalar(0.5),
                                     Scalar(0));
    const EntityId bouncy = make_block(world, "Bouncy", Vector3{3, 6, 0}, Scalar(0.6),
                                       Scalar(0.5), Scalar(0.9));

    // Long enough for both to land, and for the bouncy one to be on its way back up.
    Scalar dead_high = -1000;
    Scalar bouncy_high = -1000;
    for (int i = 0; i < 200; ++i)
    {
        simulation->tick(simulation->fixed_dt_seconds());
        // The peak *after* the first landing, which is the quantity restitution names.
        if (i < 60)
            continue;
        dead_high = std::max(dead_high, world.transform(dead).position.y);
        bouncy_high = std::max(bouncy_high, world.transform(bouncy).position.y);
    }

    // Not an absolute height: gravity here is the celestial sum sampled per body, not a
    // constant, so what is asserted is the *comparison* the material is responsible for.
    EXPECT_GT(double(bouncy_high), double(dead_high) + 0.2)
        << "a restitution of 0.9 must return visibly more height than one of 0";
}

TEST(Integration_PhysicsAuthoring, FrictionDecidesWhetherABodySlidesDownARamp)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    // A ramp at about 17 degrees: past ice's own sliding angle, which is `atan(0.02)` or
    // roughly one degree, and well below rubber's, which is `atan(1.4)` or about 54. That
    // is the whole test — the angle is chosen so the two materials must disagree, and the
    // two numbers that decide it are the only difference between the blocks.
    const EntityId ramp = make_floor(world, Vector3{0, 0, 0});
    EntityTransform ramp_transform = world.transform(ramp);
    ramp_transform.rotation = Quaternion{0, 0, Scalar(0.1494), Scalar(0.9888)};
    world.set_transform(ramp, ramp_transform);
    ColliderParameters ramp_surface = world.collider_params(ramp);
    ramp_surface.static_friction = Scalar(0.6);
    ramp_surface.dynamic_friction = Scalar(0.5);
    world.set_collider_params(ramp, ramp_surface);

    const EntityId slippery =
        make_block(world, "Ice", Vector3{0, 2.2, -4}, Scalar(0.02), Scalar(0.02), Scalar(0));
    const EntityId grippy =
        make_block(world, "Rubber", Vector3{0, 2.2, 4}, Scalar(1.4), Scalar(1.2), Scalar(0));

    // Let both land and settle onto the slope before anything is measured, so what is
    // compared is sliding rather than the difference in where they happened to bounce.
    step(*simulation, 90);
    const Scalar ice_start = world.transform(slippery).position.x;
    const Scalar rubber_start = world.transform(grippy).position.x;

    step(*simulation, 300);

    const double ice_moved =
        std::abs(double(world.transform(slippery).position.x - ice_start));
    const double rubber_moved =
        std::abs(double(world.transform(grippy).position.x - rubber_start));

    // Absolute on the grippy side, because "does not slide" is a statement with a number:
    // a block that creeps a metre down a slope it should hold on is a friction bug, not a
    // tolerance. Relative on the other side, because how *far* ice travels depends on the
    // ramp's length, which is not what this test is about.
    EXPECT_LT(rubber_moved, 0.3) << "static friction of 1.4 holds on a 17 degree slope";
    EXPECT_GT(ice_moved, rubber_moved + 0.5)
        << "static friction of 0.02 does not, so it must end up visibly further down";
}

TEST(Integration_PhysicsAuthoring, ExcludedLayersPassThroughEachOther)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    make_floor(world, Vector3{0, 0, 0});

    const auto set_layer = [&](EntityId id, std::uint32_t layer, std::uint32_t mask)
    {
        ColliderParameters collider = world.collider_params(id);
        collider.layer = layer;
        collider.collides_with = mask;
        world.set_collider_params(id, collider);
    };

    // The pair that must pass through each other: each excludes the other's layer, both
    // directions, because the filter test requires both and a one-sided exclusion does
    // nothing at all.
    const EntityId ghost_low = make_block(world, "GhostLow", Vector3{-4, 2, 0}, Scalar(0.6),
                                          Scalar(0.5), Scalar(0));
    const EntityId ghost_high = make_block(world, "GhostHigh", Vector3{-4, 5, 0}, Scalar(0.6),
                                           Scalar(0.5), Scalar(0));
    set_layer(ghost_low, 2, ~(std::uint32_t(1) << 3));
    set_layer(ghost_high, 3, ~(std::uint32_t(1) << 2));

    // The control: identical geometry, default layers, so it stacks. Without it a filter
    // bug that dropped *every* contact would produce the same reading as a working one.
    const EntityId solid_low =
        make_block(world, "SolidLow", Vector3{4, 2, 0}, Scalar(0.6), Scalar(0.5), Scalar(0));
    const EntityId solid_high =
        make_block(world, "SolidHigh", Vector3{4, 5, 0}, Scalar(0.6), Scalar(0.5), Scalar(0));

    step(*simulation, 240);

    const double ghost_gap = std::abs(double(world.transform(ghost_high).position.y -
                                             world.transform(ghost_low).position.y));
    const double solid_gap = std::abs(double(world.transform(solid_high).position.y -
                                             world.transform(solid_low).position.y));

    // The ghosts end up at the same height, both resting on the floor, having passed
    // through one another. The control pair cannot, because one is standing on the other.
    //
    // 0.5 is the geometric ideal (two 0.5-tall blocks stacked with zero overlap), not what
    // an iterative XPBD solver actually settles to: rest_offset defaults to 0 (the solver
    // aims for zero penetration) but never fully closes it in a finite number of substeps,
    // so a resting stack keeps a few millimetres of overlap per contact -- the same slop
    // every iterative contact solver (Bullet, PhysX, Jolt) carries. 0.49 leaves room for
    // that without accepting "nothing collided at all" (which would read near zero).
    EXPECT_GT(solid_gap, 0.49) << "the control pair must stack, or nothing here is colliding";
    EXPECT_LT(ghost_gap, solid_gap * 0.5)
        << "bodies on mutually excluded layers must end up side by side, not stacked";
}

TEST(Integration_PhysicsAuthoring, TheSampleSceneBuildsAndSettles)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();

    Editor::build_physics_sample_scene(world);

    // Everything the scene claims to demonstrate, by name, so a rename or a dropped
    // section fails here rather than being noticed as a demo that quietly got smaller.
    for (const char* name : {"Ground", "Stack 0", "Stack 4", "Ramp", "Ice Block",
                             "Rubber Block", "Bouncing Ball", "Ghost A", "Ghost B", "Chassis",
                             "Door", "Pendulum 0", "Pendulum 3", "Cloth", "Key Light"})
        EXPECT_NE(find_by_name(world, name), NULL_ENTITY) << name << " is missing";

    const EntityId door = find_by_name(world, "Door");
    ASSERT_NE(door, NULL_ENTITY);
    ASSERT_TRUE(world.has_joint(door));

    const Scalar door_start = world.transform(door).position.y;
    step(*simulation, 180);

    // P3's acceptance clause, in a scene an author can open: the door hangs from its
    // hinge instead of falling, and the hinge reports what it is carrying.
    JointState load;
    EXPECT_FALSE(world.joint_broken(door)) << "its own weight is far below the threshold";
    EXPECT_TRUE(world.joint_load(door, load));
    EXPECT_LT(double(door_start - world.transform(door).position.y), 0.5)
        << "the door hangs; it does not fall";

    // And the stack settles rather than drifting or exploding, which is the one thing a
    // scene of contacts can do wrong that no single assertion above would catch.
    const EntityId top = find_by_name(world, "Stack 4");
    ASSERT_NE(top, NULL_ENTITY);
    const Scalar settled = world.transform(top).position.y;
    step(*simulation, 120);
    EXPECT_LT(std::abs(double(world.transform(top).position.y - settled)), 0.05)
        << "a settled stack must stay settled";
}

TEST(Integration_PhysicsAuthoring, TheSurfaceAndTheFilterSurviveTheSceneFile)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId id = world.create_box("Surface");
    ColliderParameters authored = world.collider_params(id);
    authored.static_friction = Scalar(0.125);
    authored.dynamic_friction = Scalar(0.0625);
    authored.restitution = Scalar(0.875);
    authored.friction_combine = 1;
    authored.restitution_combine = 2;
    authored.layer = 7;
    authored.collides_with = 0x0F0F0F0Fu;
    authored.trigger = true;
    authored.continuous_collision = true;
    world.set_collider_params(id, authored);

    const nlohmann::json snapshot = Scene::capture_scene(world);
    world.set_collider_params(id, ColliderParameters{});
    Scene::apply_scene(world, snapshot);

    const ColliderParameters restored = world.collider_params(find_by_name(world, "Surface"));
    EXPECT_DOUBLE_EQ(double(restored.static_friction), double(authored.static_friction));
    EXPECT_DOUBLE_EQ(double(restored.dynamic_friction), double(authored.dynamic_friction));
    EXPECT_DOUBLE_EQ(double(restored.restitution), double(authored.restitution));
    EXPECT_EQ(restored.friction_combine, authored.friction_combine);
    EXPECT_EQ(restored.restitution_combine, authored.restitution_combine);
    EXPECT_EQ(restored.layer, authored.layer);
    EXPECT_EQ(restored.collides_with, authored.collides_with);
    EXPECT_EQ(restored.trigger, authored.trigger);
    EXPECT_EQ(restored.continuous_collision, authored.continuous_collision);
}

TEST(Integration_PhysicsAuthoring, ATriggerVolumeReportsOverlapButNeverStopsTheBody)
{
    // §16.45.1: `Collider::flags` (trigger, continuous collision) has been read by the
    // solver since the collision system was built — `record.trigger` skips resolving the
    // contact (`physics_simulation.hpp`'s "reported, never resolved") and still reports it
    // via `ContactEvent::trigger` — but nothing on `ColliderParameters` ever set the bit, so a
    // trigger volume was solvable and not placeable. This is that field, proved both ways:
    // the body passes straight through, and the overlap is still seen.
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    const EntityId floor = make_floor(world, Vector3{0, 0, 0});
    ColliderParameters floor_collider = world.collider_params(floor);
    floor_collider.trigger = true;
    world.set_collider_params(floor, floor_collider);

    const EntityId box =
        make_block(world, "Falling", Vector3{0, 3, 0}, Scalar(0.6), Scalar(0.5), Scalar(0));

    bool saw_trigger_event = false;
    for (int i = 0; i < 120; ++i)
    {
        simulation->tick(simulation->fixed_dt_seconds());
        for (const ContactEvent& event : world.physics_contacts())
        {
            if (event.trigger &&
                ((event.a == floor && event.b == box) || (event.a == box && event.b == floor)))
                saw_trigger_event = true;
        }
    }

    EXPECT_TRUE(saw_trigger_event) << "a trigger-flagged floor must still report the overlap";
    EXPECT_LT(double(world.transform(box).position.y), -1.0)
        << "a trigger never stops the body; it must fall straight through";
}

TEST(Integration_PhysicsAuthoring, TheDebugReadoutReportsWhatTheSolverKnows)
{
    const auto simulation = create_simulation();
    ASSERT_NE(simulation, nullptr);
    IWorldEditor& world = simulation->world();
    clear_world(world);

    make_floor(world, Vector3{0, 0, 0});
    const EntityId box = make_block(world, "Box", Vector3{0, 4, 0}, Scalar(0.6), Scalar(0.5),
                                    Scalar(0));
    const EntityId bare = world.create("NoBody");

    step(*simulation, 4);

    RigidDebugState state;
    EXPECT_FALSE(world.physics_body_debug(bare, state)) << "an entity with no body has none";
    ASSERT_TRUE(world.physics_body_debug(box, state));

    // The bound has to contain the body it belongs to, or the debug view is drawing boxes
    // that are not the ones the broadphase tests — which is the one way this readout can
    // be worse than nothing.
    const Vector3 position = world.transform(box).position;
    EXPECT_LE(double(state.bounds_min.x), double(position.x));
    EXPECT_GE(double(state.bounds_max.x), double(position.x));
    EXPECT_LE(double(state.bounds_min.y), double(position.y));
    EXPECT_GE(double(state.bounds_max.y), double(position.y));
    EXPECT_FALSE(state.sleeping) << "a body still falling is not asleep";

    // Contacts are the same stream gameplay is told about, not a second one, so once the
    // box has landed there is something in it.
    step(*simulation, 200);
    EXPECT_FALSE(world.physics_contacts().empty())
        << "a landed box against a floor is a contact somebody can draw";
}
