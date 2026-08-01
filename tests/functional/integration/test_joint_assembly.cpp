/**************************************************************************/
/* test_joint_assembly.cpp                                                */
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

// P3's acceptance criterion, as a test rather than an opinion: *"the chassis-plus-
// hinged-door scene works end to end -- the door swings within its limits, carries
// load, reports its hinge force, and tears off above its break threshold."*
//
// Driven through `IJointService` on the live `PhysicsSimulation`, not through the
// solver directly, because the claim is about the whole path: the boundary vocabulary
// converts, the joint is coloured against the tick's contacts, the solve runs on the
// device inside the one composition, the load comes back off it, and the break
// decision is taken on the host at the step boundary because a topology change may
// not happen against a running graph.
//
// The scene is the one sec. 10.2 writes out: a 900 kg chassis, a 35 kg door on a hinge
// about the door's local +Y with limits of [0 deg, 68 deg], hinge friction so it does
// not swing free, and a break force above which the door comes off. The door and the
// chassis are put on layers that exclude each other, which is how an assembly stops
// its own parts fighting the joint that holds them.

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/animation/skeleton_blob.hpp>
#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/physics_assembly.hpp>
#include <SushiEngine/sim/physics_simulation.hpp>
#include <SushiEngine/sim/ragdoll.hpp>

#include "test_helpers.hpp"

using namespace SushiEngine;
using namespace SushiEngine::Simulation;

namespace
{
    constexpr std::size_t ITERATIONS = 8;
    constexpr std::size_t SUBSTEPS = 8;
    constexpr Scalar SUBSTEP_DT = Scalar(1.0 / 480.0);
    constexpr double PI = 3.14159265358979323846;

    constexpr EntityId CHASSIS = 1;
    constexpr EntityId DOOR = 2;

    /** @brief The layer the chassis and the door share, and which they exclude. */
    constexpr std::uint32_t CAR_LAYER = 1u << 4;

    /**
     * @brief Still air, which is what every scene without weather installed passes.
     *
     * An *empty* sampler rather than one returning zero: the physics skips it entirely,
     * so a scene with no wind is bit-for-bit the scene that had no wind seam at all
     * (§11.6, `physics/aero/wind.hpp`).
     */
    WindSampler still_air()
    {
        return WindSampler{};
    }

    GravitySampler earth_gravity()
    {
        return [](const Vector3&) { return Vector3{0, Scalar(-9.81), 0}; };
    }

    GravitySampler no_gravity()
    {
        return [](const Vector3&) { return Vector3{0, 0, 0}; };
    }

    /**
     * @brief A box collider on the car layer, which does not collide with itself.
     *
     * Sec. 10.2's "part 0 and part 1 do not collide with each other", expressed the way
     * §4.4 requires it to be: as filter data, not as a type tag the contact pass has to
     * branch on. Without it the door's own collider pushes against the chassis it is
     * hinged to and the joint spends its whole budget fighting a contact.
     */
    Collider car_part(const Vector3& half_extents)
    {
        Collider collider;
        collider.shape = ColliderShape::Box;
        collider.half_extents = half_extents;
        collider.filter.layer = CAR_LAYER;
        collider.filter.collides_with = ~CAR_LAYER;
        return collider;
    }

    /** @brief The chassis: heavy, pinned, and the thing the door hangs off. */
    RigidBodyDesc chassis_desc()
    {
        RigidBodyDesc desc;
        desc.id = CHASSIS;
        desc.position = Vector3{0, Scalar(2), 0};
        // Pinned rather than merely heavy: the acceptance is about the door and the
        // hinge, and a chassis free to recoil would make every measurement below a
        // measurement of two things.
        desc.inv_mass = 0;
        desc.inv_inertia = Vector3{0, 0, 0};
        desc.collider = car_part(Vector3{Scalar(1), Scalar(0.6), Scalar(0.5)});
        return desc;
    }

    /** @brief The door: 35 kg, hung off the chassis's right edge. */
    RigidBodyDesc door_desc()
    {
        RigidBodyDesc desc;
        desc.id = DOOR;
        // Its centre of mass sits half a metre out from the hinge line, which is what
        // gives the hinge a moment to carry.
        desc.position = Vector3{Scalar(1.5), Scalar(2), 0};
        desc.inv_mass = Scalar(1) / Scalar(35);
        const Scalar inertia = Scalar(3);
        desc.inv_inertia = Vector3{Scalar(1) / inertia, Scalar(1) / inertia,
                                   Scalar(1) / inertia};
        desc.collider = car_part(Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.05)});
        return desc;
    }

    /** @brief The hinge of sec. 10.2: axis local +Y, limits [0, 68 deg], 4 N.m friction. */
    JointDesc door_hinge(Scalar break_force)
    {
        JointDesc joint;
        joint.body_a = CHASSIS;
        joint.body_b = DOOR;
        joint.params.type = JointType::Hinge;
        // The hinge line: the chassis's right edge, and the door's inboard edge.
        joint.params.anchor_a = Vector3{Scalar(1), 0, 0};
        joint.params.anchor_b = Vector3{Scalar(-0.5), 0, 0};
        joint.params.axis_a = Vector3{0, 1, 0};
        joint.params.axis_b = Vector3{0, 1, 0};

        joint.params.twist_limit.enabled = true;
        joint.params.twist_limit.lower = 0;
        joint.params.twist_limit.upper = Scalar(68.0 * PI / 180.0);

        // It does not swing free: a rate drive toward standstill, saturated at the
        // hinge's friction torque.
        joint.params.motor.type = JointMotorType::Velocity;
        joint.params.motor.target = 0;
        joint.params.motor.max_force = Scalar(4);

        joint.params.break_force = break_force;
        return joint;
    }

    /** @brief The hinge angle the door currently holds, derived from the two poses. */
    double door_angle(const IPhysicsScene& physics)
    {
        SolvedPose chassis;
        SolvedPose door;
        if (!physics.rigid_pose(CHASSIS, chassis) || !physics.rigid_pose(DOOR, door))
            return 0.0;
        // The hinge axis is world +Y and both frames were built from local +Y, so the
        // hinge angle is the relative rotation's twist about it. Derived here from the
        // boundary poses rather than read out of the solver, so this measures what a
        // game would measure.
        const Quaternion relative = mul(conjugate(chassis.orientation), door.orientation);
        const Vector3 rotation = Physics::joint_rotation_vector(relative);
        return double(rotation.y);
    }
}

TEST(Integration_JointAssembly, TheHingedDoorHangsFromTheChassisAndCarriesItsWeight)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);

    const JointId hinge = physics->create_joint(door_hinge(0));
    ASSERT_NE(hinge, NULL_JOINT);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    // The door has not fallen: the hinge holds it where it was hung, to within the
    // sag a compliant-by-nothing positional solve leaves behind.
    SolvedPose door;
    ASSERT_TRUE(physics->rigid_pose(DOOR, door));
    EXPECT_NEAR(double(door.position.x), 1.5, 1e-2);
    EXPECT_NEAR(double(door.position.y), 2.0, 1e-2);
    EXPECT_NEAR(double(door.position.z), 0.0, 1e-3);

    // And it reports what it is carrying. Sec. 10.4's recovery is exact, so this is
    // statics rather than a tolerance: the only external force on a resting door is
    // its weight, and the hinge's angular row carries pure torque, so the reaction
    // force must be mg and the reaction torque mg times the lever arm.
    JointState state;
    ASSERT_TRUE(physics->joint_state(hinge, state));
    EXPECT_NEAR(double(length(state.force)), 35.0 * 9.81, 2.0);
    EXPECT_NEAR(double(length(state.torque)), 35.0 * 9.81 * 0.5, 2.0);

    EXPECT_TRUE(physics->joint_broken_events().empty());
}

TEST(Integration_JointAssembly, TheDoorSwingsOnlyWithinItsLimits)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);

    const JointId hinge = physics->create_joint(door_hinge(0));
    ASSERT_NE(hinge, NULL_JOINT);

    // Drive the door open with the motor rather than by kicking it, so the test is
    // about the *limit* rather than about how hard something was thrown.
    JointMotorDesc opening;
    opening.type = JointMotorType::Velocity;
    opening.target = Scalar(2);
    opening.max_force = Scalar(200);
    ASSERT_TRUE(physics->set_joint_motor(hinge, opening));

    const double upper = 68.0 * PI / 180.0;
    double widest = 0.0;
    double narrowest = 0.0;
    for (int tick = 0; tick < 480; ++tick)
    {
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
        const double angle = door_angle(*physics);
        widest = angle > widest ? angle : widest;
        narrowest = angle < narrowest ? angle : narrowest;
    }

    // Open, and stopped: the motor pushes indefinitely and the limit is the only thing
    // that can hold it. A limit that did nothing would leave the door spinning.
    EXPECT_GT(widest, 0.5 * upper) << "the door never opened, so the limit proves nothing";
    EXPECT_LT(widest, upper + 2e-2) << "the door swung past its upper limit";
    EXPECT_GT(narrowest, -2e-2) << "the door swung past its lower limit";
    EXPECT_NEAR(door_angle(*physics), upper, 2e-2) << "the door did not rest against its stop";
}

TEST(Integration_JointAssembly, HingeFrictionStopsTheDoorSwingingFree)
{
    // The same scene twice, differing only in the friction. Zero gravity, so the only
    // thing that can slow the door is the hinge itself.
    double free_angle_travel = 0.0;
    double damped_angle_travel = 0.0;

    for (int with_friction = 0; with_friction < 2; ++with_friction)
    {
        auto physics = create_physics_simulation(Harness::shared_context());
        physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);

        JointDesc joint = door_hinge(0);
        // No stop, so the measurement is of the swing and not of a collision with a limit.
        joint.params.twist_limit.enabled = false;
        if (with_friction == 0)
            joint.params.motor.type = JointMotorType::Disabled;
        const JointId hinge = physics->create_joint(joint);
        ASSERT_NE(hinge, NULL_JOINT);

        // One shove, then let go.
        JointMotorDesc shove;
        shove.type = JointMotorType::Velocity;
        shove.target = Scalar(2);
        shove.max_force = Scalar(200);
        ASSERT_TRUE(physics->set_joint_motor(hinge, shove));
        for (int tick = 0; tick < 20; ++tick)
            physics->step(no_gravity(), still_air(), SUBSTEPS);

        JointMotorDesc released;
        if (with_friction != 0)
        {
            released.type = JointMotorType::Velocity;
            released.target = 0;
            released.max_force = Scalar(4);
        }
        ASSERT_TRUE(physics->set_joint_motor(hinge, released));

        const double released_at = door_angle(*physics);
        for (int tick = 0; tick < 240; ++tick)
            physics->step(no_gravity(), still_air(), SUBSTEPS);
        const double travel = door_angle(*physics) - released_at;

        if (with_friction != 0)
            damped_angle_travel = travel;
        else
            free_angle_travel = travel;
    }

    EXPECT_GT(free_angle_travel, 0.5) << "a frictionless hinge should coast";
    EXPECT_LT(damped_angle_travel, 0.25 * free_angle_travel)
        << "friction did not slow the swing";
}

TEST(Integration_JointAssembly, AHardEnoughImpactTearsTheDoorOff)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);

    // The door's own weight puts about 340 N through the hinge, so a threshold well
    // above that survives ordinary hanging and gives only to something violent.
    const JointId hinge = physics->create_joint(door_hinge(Scalar(12000)));
    ASSERT_NE(hinge, NULL_JOINT);

    for (int tick = 0; tick < 60; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    ASSERT_TRUE(physics->joint_broken_events().empty())
        << "a door broke off under its own weight";

    // The impact: the door is teleported out to where the hinge is grossly violated,
    // which is what a hard hit looks like to a positional solver -- a separation the
    // constraint has to close in one tick, and a load that scales with it.
    physics->set_rigid_pose(DOOR, Vector3{Scalar(3.5), Scalar(2), 0}, Quaternion{});

    bool broke = false;
    JointBrokenEvent event;
    for (int tick = 0; tick < 30 && !broke; ++tick)
    {
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
        if (!physics->joint_broken_events().empty())
        {
            broke = true;
            event = physics->joint_broken_events().front();
        }
    }

    ASSERT_TRUE(broke) << "the hinge carried a load it should not have survived";
    EXPECT_EQ(event.joint, hinge);
    EXPECT_EQ(event.a, CHASSIS);
    EXPECT_EQ(event.b, DOOR);
    EXPECT_GT(double(event.force), 12000.0);

    // Reported once, and then gone: the joint no longer exists, so it can neither be
    // read nor destroyed a second time, and it fires no further events.
    JointState gone;
    EXPECT_FALSE(physics->joint_state(hinge, gone));
    EXPECT_FALSE(physics->destroy_joint(hinge));

    physics->step(earth_gravity(), still_air(), SUBSTEPS);
    EXPECT_TRUE(physics->joint_broken_events().empty())
        << "a broken joint reported itself twice";

    // And the door is a free rigid body now: nothing holds it, so it falls.
    SolvedPose before;
    ASSERT_TRUE(physics->rigid_pose(DOOR, before));
    for (int tick = 0; tick < 120; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);
    SolvedPose after;
    ASSERT_TRUE(physics->rigid_pose(DOOR, after));
    EXPECT_LT(double(after.position.y), double(before.position.y) - 0.1)
        << "the door did not come off";
}

TEST(Integration_JointAssembly, DestroyingAPartTakesItsJointsWithIt)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);

    const JointId hinge = physics->create_joint(door_hinge(0));
    ASSERT_NE(hinge, NULL_JOINT);
    physics->step(earth_gravity(), still_air(), SUBSTEPS);
    EXPECT_EQ(physics->statistics().joints, std::size_t(1));

    // The door is destroyed by the ECS: the solver drops every joint naming its slot,
    // and the boundary record goes with it, so a stale identity is not readable.
    physics->set_rigid_bodies({chassis_desc()}, ITERATIONS, SUBSTEP_DT);
    physics->step(earth_gravity(), still_air(), SUBSTEPS);

    EXPECT_EQ(physics->statistics().joints, std::size_t(0));
    JointState state;
    EXPECT_FALSE(physics->joint_state(hinge, state));
    EXPECT_FALSE(physics->destroy_joint(hinge));
}

TEST(Integration_JointAssembly, AJointNeedsBothOfItsBodiesToExist)
{
    auto physics = create_physics_simulation(Harness::shared_context());
    physics->set_rigid_bodies({chassis_desc()}, ITERATIONS, SUBSTEP_DT);

    // An immovable endpoint is a body with zero inverse mass, not a missing one, so a
    // joint naming an entity with no body is refused rather than half-created. A
    // half-created joint would be one that projected against slot zero.
    EXPECT_EQ(physics->create_joint(door_hinge(0)), NULL_JOINT);

    physics->set_rigid_bodies({chassis_desc(), door_desc()}, ITERATIONS, SUBSTEP_DT);
    EXPECT_NE(physics->create_joint(door_hinge(0)), NULL_JOINT);
}

// -- The assembly, instanced and simulated ---------------------------------
//
// The unit tests hold the assembly asset and the ragdoll rig to their translation:
// blob round trips, world placement, mass distribution, and the bind offset that
// recovers a joint's pose from its part. What only appears once the whole path runs is
// whether the thing they describe actually holds together when the solver gets it — so
// these two scenes instance an assembly the ordinary way (the caller owns the entities,
// `instantiate_assembly` says what to create, `IJointService` creates the joints) and
// then step it.

namespace
{
    /** @brief The sec. 10.2 car, as an asset rather than as two hand-written descs. */
    PhysicsAssembly car_assembly()
    {
        PhysicsAssembly assembly;

        AssemblyPart chassis;
        chassis.collider = car_part(Vector3{Scalar(1), Scalar(0.6), Scalar(0.5)});
        chassis.inv_mass = 0;
        chassis.inv_inertia = Vector3{0, 0, 0};
        assembly.parts.push_back(chassis);
        assembly.part_names.push_back("chassis");

        AssemblyPart door;
        door.collider = car_part(Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.05)});
        door.local_position = Vector3{Scalar(1.5), 0, 0};
        door.inv_mass = Scalar(1) / Scalar(35);
        const Scalar inertia = Scalar(3);
        door.inv_inertia =
            Vector3{Scalar(1) / inertia, Scalar(1) / inertia, Scalar(1) / inertia};
        assembly.parts.push_back(door);
        assembly.part_names.push_back("door");

        AssemblyJoint hinge;
        hinge.part_a = 0;
        hinge.part_b = 1;
        hinge.params = door_hinge(0).params;
        assembly.joints.push_back(hinge);

        // One group for the whole car, excluding itself: the parts overlap and must not
        // push each other, or the hinge spends its budget fighting a contact.
        assembly.group_masks.assign(1, assembly_group_excluding_self(0));
        return assembly;
    }

    /** @brief A five-joint chain, enough to be a limb and short enough to reason about. */
    Animation::SkeletonDesc chain_desc()
    {
        const auto joint = [](const char* name, int parent, Scalar y)
        {
            Animation::JointDesc desc;
            desc.name = name;
            desc.parent = parent;
            desc.bind_translation = Animation::Vector3f{0.0f, float(y), 0.0f};
            return desc;
        };

        Animation::SkeletonDesc desc;
        desc.joints.push_back(joint("root", -1, Scalar(3)));
        desc.joints.push_back(joint("a", 0, Scalar(-0.4)));
        desc.joints.push_back(joint("b", 1, Scalar(-0.4)));
        desc.joints.push_back(joint("c", 2, Scalar(-0.4)));
        desc.joints.push_back(joint("tip", 3, Scalar(-0.4)));
        return desc;
    }

    /** @brief Instances @p assembly into @p physics, returning the part entities. */
    std::vector<EntityId> instance_into(IPhysicsScene& physics, const PhysicsAssembly& assembly,
                                        const Vector3& root, const Quaternion& orientation,
                                        std::vector<JointId>& joints_out)
    {
        std::vector<EntityId> entities;
        for (std::size_t i = 0; i < assembly.parts.size(); ++i)
            entities.push_back(EntityId(100 + i));

        const AssemblyInstantiation plan = instantiate_assembly(
            to_view(assembly), entities.data(), entities.size(), root, orientation);
        // Every part in one call, then the joints: an assembly is instanced atomically in
        // the sense that matters, which is that its joints are created against a body set
        // that already holds all of them.
        physics.set_rigid_bodies(plan.bodies, ITERATIONS, SUBSTEP_DT);
        for (const JointDesc& joint : plan.joints)
            joints_out.push_back(physics.create_joint(joint));
        return entities;
    }
}

TEST(Integration_JointAssembly, AnInstancedAssemblyIsHeldTogetherByItsOwnJoints)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    std::vector<JointId> joints;
    const std::vector<EntityId> parts = instance_into(
        *physics, car_assembly(), Vector3{0, Scalar(2), 0}, Quaternion{}, joints);
    ASSERT_EQ(parts.size(), 2u);
    ASSERT_EQ(joints.size(), 1u);
    ASSERT_NE(joints[0], NULL_JOINT);

    for (int tick = 0; tick < 240; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    // The door is still on its hinge, at the offset the asset placed it, and the hinge
    // reports the weight it carries — the same statics the hand-built scene checks,
    // reached through the asset instead of through two hand-written descriptions.
    SolvedPose chassis;
    SolvedPose door;
    ASSERT_TRUE(physics->rigid_pose(parts[0], chassis));
    ASSERT_TRUE(physics->rigid_pose(parts[1], door));
    EXPECT_NEAR(double(door.position.x - chassis.position.x), 1.5, 2e-2);
    EXPECT_NEAR(double(door.position.y - chassis.position.y), 0.0, 2e-2);

    JointState state;
    ASSERT_TRUE(physics->joint_state(joints[0], state));
    EXPECT_NEAR(double(length(state.force)), 35.0 * 9.81, 5.0);

    // And the parts never pushed each other: the filter matrix the asset carries put them
    // on a group that excludes itself, so no contact was generated between the door's
    // collider and the chassis it overlaps.
    EXPECT_TRUE(physics->contact_events().empty());
}

TEST(Integration_JointAssembly, ARagdollBuiltFromASkeletonHangsTogetherAndFalls)
{
    auto physics = create_physics_simulation(Harness::shared_context());

    std::vector<std::byte> blob;
    ASSERT_TRUE(Animation::build_skeleton_blob(chain_desc(), blob));
    const Animation::SkeletonView skeleton =
        Animation::load_skeleton_blob(blob.data(), blob.size());
    ASSERT_TRUE(skeleton.valid());

    RagdollProfile profile;
    profile.total_mass = Scalar(40);
    const RagdollRig rig = build_ragdoll_rig(skeleton, profile);
    // root, a, b and c have children; tip is a leaf and gets no part.
    ASSERT_EQ(rig.assembly.parts.size(), 4u);
    ASSERT_EQ(rig.assembly.joints.size(), 3u);

    std::vector<JointId> joints;
    const std::vector<EntityId> parts =
        instance_into(*physics, rig.assembly, Vector3{0, 0, 0}, Quaternion{}, joints);
    for (const JointId joint : joints)
        ASSERT_NE(joint, NULL_JOINT);

    // The separations the cone-twists are holding, measured before anything moves.
    std::vector<double> rest;
    for (const AssemblyJoint& joint : rig.assembly.joints)
    {
        SolvedPose a;
        SolvedPose b;
        ASSERT_TRUE(physics->rigid_pose(parts[joint.part_a], a));
        ASSERT_TRUE(physics->rigid_pose(parts[joint.part_b], b));
        rest.push_back(double(length(b.position - a.position)));
    }

    for (int tick = 0; tick < 300; ++tick)
        physics->step(earth_gravity(), still_air(), SUBSTEPS);

    // It fell: nothing is pinned, so a ragdoll under gravity goes down.
    SolvedPose root_pose;
    ASSERT_TRUE(physics->rigid_pose(parts[0], root_pose));
    EXPECT_LT(double(root_pose.position.y), -1.0);

    // And it is still one body rather than four — every cone-twist is holding its pair at
    // the separation it started with. This is the check that fails if the twist range is
    // not centred on the twist the bind pose already holds: a limit fighting the rest pose
    // pumps the chain apart inside a second.
    for (std::size_t i = 0; i < rig.assembly.joints.size(); ++i)
    {
        const AssemblyJoint& joint = rig.assembly.joints[i];
        SolvedPose a;
        SolvedPose b;
        ASSERT_TRUE(physics->rigid_pose(parts[joint.part_a], a));
        ASSERT_TRUE(physics->rigid_pose(parts[joint.part_b], b));
        EXPECT_NEAR(double(length(b.position - a.position)), rest[i], 2e-2)
            << "joint " << i << " came apart";
    }

    // The targets a pose modifier consumes: one per part, in the character's own object
    // space, and every one of them somewhere the character actually is.
    std::vector<Animation::RagdollJointTarget> targets;
    ASSERT_EQ(resolve_ragdoll_targets(rig, *physics, parts.data(), parts.size(), Mat4{},
                                      Scalar(1), targets),
              rig.assembly.parts.size());
    for (const Animation::RagdollJointTarget& target : targets)
    {
        EXPECT_LT(target.joint, skeleton.joint_count);
        EXPECT_FLOAT_EQ(target.weight, 1.0f);
        EXPECT_LT(double(target.object_space_transform.m[13]), 0.0)
            << "a target stayed at the bind pose while the ragdoll fell";
    }
}
