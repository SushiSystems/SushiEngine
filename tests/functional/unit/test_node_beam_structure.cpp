/**************************************************************************/
/* test_node_beam_structure.cpp                                           */
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
 * @file test_node_beam_structure.cpp
 * @brief §11.2's hybrid, instanced: what a cooked vehicle becomes in a solver, and
 *        what the tick boundary does to it.
 *
 * The assets here are hand-written rather than cooked. That is the point of the
 * format being data (§11.3): a structure test that had to run a cooker first would
 * be measuring the cooker's node placement every time it wanted to ask what happens
 * when one beam breaks, and it could not ask about a two-part vehicle at all until
 * the cooker learned to split parts.
 */

#include <cmath>
#include <cstdint>
#include <memory>
#include <vector>

#include <gtest/gtest.h>

#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/solver/host_solver.hpp>
#include <SushiEngine/physics/vehicle/node_beam_structure.hpp>

using namespace SushiEngine;
using namespace SushiEngine::Physics;

namespace
{
    /** @brief A budget wide enough for every vehicle below. */
    PhysicsConfiguration structure_scene(std::size_t bodies = 64)
    {
        PhysicsConfiguration configuration;
        configuration.capacities.bodies = bodies;
        configuration.capacities.constraints = 128;
        configuration.capacities.contacts = 128;
        configuration.capacities.joints = 64;
        configuration.capacities.beams = 128;
        configuration.capacities.colors = 8;
        configuration.substeps.minimum = 4;
        configuration.substeps.maximum = 8;
        return configuration;
    }

    /** @brief One node record, spelled out so a test reads as the vehicle it builds. */
    Cooking::NodeBeamNodeRecord make_node(const Vector3& position, Scalar mass,
                                          std::uint32_t part = 0,
                                          std::uint32_t flags = Cooking::NodeBeamNodeFlags::none)
    {
        Cooking::NodeBeamNodeRecord node{};
        node.position = position;
        node.mass = mass;
        node.radius = Scalar(0.05);
        node.drag_area = Scalar(0);
        node.part = part;
        node.flags = flags;
        return node;
    }

    /** @brief One beam record between two nodes, at a compliance and two thresholds. */
    Cooking::NodeBeamBeamRecord make_beam(std::uint32_t a, std::uint32_t b, Scalar rest_length,
                                          Scalar compliance, Scalar deform_force,
                                          Scalar break_force, std::uint32_t part = 0)
    {
        Cooking::NodeBeamBeamRecord beam{};
        beam.a = a;
        beam.b = b;
        beam.part = part;
        beam.kind = Cooking::NodeBeamBeamKind::structural;
        beam.rest_length = rest_length;
        beam.compliance = compliance;
        beam.damping = Scalar(0);
        beam.deform_force = deform_force;
        beam.break_force = break_force;
        beam.plastic_creep = Scalar(0.5);
        beam.maximum_plastic_strain = Scalar(0.5);
        return beam;
    }

    /** @brief One attachment holding a node to the core. */
    Cooking::NodeBeamAttachmentRecord make_attachment(std::uint32_t node, const Vector3& anchor,
                                                      Scalar break_force,
                                                      std::uint32_t part = 0)
    {
        Cooking::NodeBeamAttachmentRecord attachment{};
        attachment.node = node;
        attachment.part = part;
        attachment.core_anchor = anchor;
        attachment.compliance = Scalar(0);
        attachment.break_force = break_force;
        return attachment;
    }

    /** @brief Fills the summary fields the blob writer checks against the arrays. */
    void finish(Cooking::NodeBeamAsset& asset)
    {
        Scalar node_mass = 0;
        std::uint32_t parts = 0;
        for (const Cooking::NodeBeamNodeRecord& node : asset.nodes)
        {
            node_mass += node.mass;
            if (node.part + 1 > parts)
                parts = node.part + 1;
        }
        for (const Cooking::NodeBeamBeamRecord& beam : asset.beams)
        {
            if (beam.part + 1 > parts)
                parts = beam.part + 1;
        }
        for (const Cooking::NodeBeamAttachmentRecord& attachment : asset.attachments)
        {
            if (attachment.part + 1 > parts)
                parts = attachment.part + 1;
        }
        asset.summary.node_mass = node_mass;
        asset.summary.total_mass = node_mass + asset.core.mass;
        asset.summary.part_count = parts;
    }

    /** @brief Turns an authored asset into the bytes a structure is created from. */
    struct Blob
    {
        std::vector<std::byte> bytes;
        Cooking::NodeBeamAssetView view;

        explicit Blob(const Cooking::NodeBeamAsset& asset)
        {
            if (Cooking::build_node_beam_blob(asset, bytes))
                view = Cooking::load_node_beam_blob(bytes.data(), bytes.size());
        }
    };

    /**
     * @brief A pinned node with one hanging under it, on a single beam.
     *
     * The smallest structure whose beam is guaranteed to carry a load an assertion
     * can predict: gravity times the hanging node's mass, and nothing else.
     */
    Cooking::NodeBeamAsset hanging_pair(Scalar deform_force, Scalar break_force)
    {
        Cooking::NodeBeamAsset asset;
        asset.nodes.push_back(
            make_node(Vector3{0, 1, 0}, Scalar(1), 0, Cooking::NodeBeamNodeFlags::fixed));
        asset.nodes.push_back(make_node(Vector3{0, 0, 0}, Scalar(100)));
        asset.beams.push_back(
            make_beam(0, 1, Scalar(1), Scalar(1e-4), deform_force, break_force));
        finish(asset);
        return asset;
    }

    /**
     * @brief A two-part vehicle: a chassis shell on a core, and a door on the chassis.
     *
     * Part 1's only ties are one cross-part beam and one attachment, so a test can
     * take both away and watch the part come off without disturbing part 0.
     */
    Cooking::NodeBeamAsset two_part_vehicle(Scalar door_break_force)
    {
        Cooking::NodeBeamAsset asset;
        asset.nodes.push_back(make_node(Vector3{-0.5, 0, 0}, Scalar(20)));
        asset.nodes.push_back(make_node(Vector3{0.5, 0, 0}, Scalar(20)));
        asset.nodes.push_back(make_node(Vector3{0.5, 0.6, 0}, Scalar(5), 1));
        asset.nodes.push_back(make_node(Vector3{1.1, 0.6, 0}, Scalar(5), 1));

        // Part 0's own beam, then the one tie between the two parts.
        asset.beams.push_back(make_beam(0, 1, Scalar(1), Scalar(1e-7), 0, 0));
        asset.beams.push_back(
            make_beam(1, 2, Scalar(0.6), Scalar(1e-7), 0, door_break_force, 1));
        asset.beams.push_back(make_beam(2, 3, Scalar(0.6), Scalar(1e-7), 0, 0, 1));

        asset.core.mass = Scalar(800);
        asset.core.center_of_mass = Vector3{0, 0, 0};
        asset.core.principal_inertia = Vector3{200, 400, 300};
        asset.core.principal_rotation = Quaternion{0, 0, 0, 1};

        asset.attachments.push_back(make_attachment(0, Vector3{-0.5, 0, 0}, Scalar(0)));
        asset.attachments.push_back(make_attachment(1, Vector3{0.5, 0, 0}, Scalar(0)));
        asset.attachments.push_back(
            make_attachment(2, Vector3{0.5, 0.6, 0}, door_break_force, 1));

        finish(asset);
        return asset;
    }

    /** @brief Advances the solver one tick and runs the structure's boundary pass. */
    NodeBeamTickReport tick(HostXpbdSolver<Scalar>& solver, NodeBeamStructure& structure,
                            const Vector3& gravity = Vector3{0, -9.81, 0})
    {
        StepParameters<Scalar> parameters;
        parameters.delta_time = Scalar(1) / Scalar(60);
        parameters.gravity = gravity;
        solver.step(parameters);
        return structure.end_tick(solver);
    }

    /** @brief The distance between two points, for assertions that are about a gap. */
    Scalar separation(const Vector3& a, const Vector3& b)
    {
        return length(b - a);
    }

    /**
     * @brief Throws one body across the scene, which is how a test loads a vehicle.
     *
     * Gravity cannot do it: it is uniform, so a vehicle with no ground under it falls
     * as one piece and every beam and mount in it carries exactly nothing. An impact
     * is a relative velocity, and this is the smallest way to make one.
     */
    void yank(HostXpbdSolver<Scalar>& solver, BodyHandle handle, const Vector3& velocity)
    {
        RigidBody body;
        ASSERT_TRUE(solver.read_body(handle, body));
        body.velocity = velocity;
        EXPECT_TRUE(solver.write_body(handle, body));
    }
} // namespace

/** @brief Every record in the asset becomes something in the solver. */
TEST(Unit_NodeBeamStructure, InstancesEveryRecord)
{
    const Blob blob(two_part_vehicle(Scalar(0)));
    ASSERT_TRUE(blob.view.valid);

    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    EXPECT_EQ(structure.node_count(), blob.view.node_count);
    EXPECT_EQ(structure.beam_count(), blob.view.beam_count);
    EXPECT_EQ(structure.live_beam_count(), blob.view.beam_count);
    EXPECT_EQ(structure.live_attachment_count(), blob.view.attachment_count);
    EXPECT_EQ(structure.part_count(), 2u);
    EXPECT_TRUE(structure.has_core());
}

/** @brief The spawn pose moves the whole vehicle, nodes and core alike. */
TEST(Unit_NodeBeamStructure, PlacesTheAssetAtTheSpawnPose)
{
    const Cooking::NodeBeamAsset asset = two_part_vehicle(Scalar(0));
    const Blob blob(asset);
    ASSERT_TRUE(blob.view.valid);

    NodeBeamStructureSettings<Scalar> settings;
    settings.position = Vector3{10, 4, -7};
    settings.orientation = quaternion_axis_angle(Vector3{0, 1, 0}, Scalar(1.1));

    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, settings));

    for (std::uint32_t i = 0; i < blob.view.node_count; ++i)
    {
        const Vector3 expected =
            settings.position + rotate(settings.orientation, asset.nodes[i].position);
        EXPECT_LT(separation(expected, structure.node_positions()[i]), Scalar(1e-12));
    }

    // The core is stored at its centre of mass in its principal frame; the origin the
    // asset was authored about has to come back out of it unchanged.
    RigidBody core;
    ASSERT_TRUE(solver.read_body(structure.core(), core));
    EXPECT_LT(separation(settings.position, body_origin(core)), Scalar(1e-12));
}

/** @brief The core carries the asset's mass and the inverse of its principal moments. */
TEST(Unit_NodeBeamStructure, CoreCarriesTheAssetMassAndInertia)
{
    const Cooking::NodeBeamAsset asset = two_part_vehicle(Scalar(0));
    const Blob blob(asset);
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    RigidBody core;
    ASSERT_TRUE(solver.read_body(structure.core(), core));
    EXPECT_NEAR(core.inv_mass, Scalar(1) / asset.core.mass, Scalar(1e-15));
    EXPECT_NEAR(core.inv_inertia.x, Scalar(1) / asset.core.principal_inertia.x, Scalar(1e-15));
    EXPECT_NEAR(core.inv_inertia.y, Scalar(1) / asset.core.principal_inertia.y, Scalar(1e-15));
    EXPECT_NEAR(core.inv_inertia.z, Scalar(1) / asset.core.principal_inertia.z, Scalar(1e-15));
}

/** @brief §11.1's node is a particle: mass, no inertia, and a pinned one has no mass either. */
TEST(Unit_NodeBeamStructure, NodesAreParticles)
{
    const Blob blob(hanging_pair(Scalar(0), Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    RigidBody pinned;
    RigidBody hanging;
    ASSERT_TRUE(solver.read_body(structure.node(0), pinned));
    ASSERT_TRUE(solver.read_body(structure.node(1), hanging));

    EXPECT_EQ(pinned.inv_mass, Scalar(0));
    EXPECT_NEAR(hanging.inv_mass, Scalar(1) / Scalar(100), Scalar(1e-15));
    for (const RigidBody& body : {pinned, hanging})
    {
        EXPECT_EQ(body.inv_inertia.x, Scalar(0));
        EXPECT_EQ(body.inv_inertia.y, Scalar(0));
        EXPECT_EQ(body.inv_inertia.z, Scalar(0));
    }
}

/** @brief A vehicle spawned in motion starts with that motion everywhere. */
TEST(Unit_NodeBeamStructure, SpawnVelocityReachesEveryBody)
{
    const Blob blob(two_part_vehicle(Scalar(0)));
    NodeBeamStructureSettings<Scalar> settings;
    settings.velocity = Vector3{12, 0, 0};

    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, settings));

    RigidBody body;
    for (std::size_t i = 0; i < structure.node_count(); ++i)
    {
        ASSERT_TRUE(solver.read_body(structure.node(i), body));
        EXPECT_EQ(body.velocity.x, Scalar(12));
    }
    ASSERT_TRUE(solver.read_body(structure.core(), body));
    EXPECT_EQ(body.velocity.x, Scalar(12));
}

/** @brief §11.2's other end of the dial: an empty core is a pure node-beam vehicle. */
TEST(Unit_NodeBeamStructure, PureNodeBeamAssetHasNoCore)
{
    const Blob blob(hanging_pair(Scalar(0), Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    EXPECT_FALSE(structure.has_core());
    EXPECT_FALSE(structure.core().valid());
    EXPECT_EQ(structure.live_attachment_count(), 0u);
}

/** @brief An asset that never loaded builds nothing. */
TEST(Unit_NodeBeamStructure, RefusesAnInvalidAsset)
{
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    EXPECT_FALSE(
        structure.create(solver, Cooking::NodeBeamAssetView{}, NodeBeamStructureSettings<Scalar>{}));
    EXPECT_EQ(structure.node_count(), 0u);
}

/**
 * @brief A budget that runs out part way through gives every slot back.
 *
 * Measured by what fits *afterwards*: a solver sized for four bodies that refused a
 * five-body vehicle must still have four free, and the only way to see that is to
 * put four in.
 */
TEST(Unit_NodeBeamStructure, RefusalLeavesTheSolverEmpty)
{
    const Blob wide(two_part_vehicle(Scalar(0)));  // four nodes and a core: five bodies.
    const Blob narrow(hanging_pair(Scalar(0), Scalar(0)));

    HostXpbdSolver<Scalar> solver(structure_scene(4));
    NodeBeamStructure refused;
    ASSERT_FALSE(refused.create(solver, wide.view, NodeBeamStructureSettings<Scalar>{}));
    EXPECT_EQ(refused.node_count(), 0u);
    EXPECT_FALSE(refused.has_core());

    NodeBeamStructure accepted;
    EXPECT_TRUE(accepted.create(solver, narrow.view, NodeBeamStructureSettings<Scalar>{}));
}

/** @brief Destroying a structure returns everything it took. */
TEST(Unit_NodeBeamStructure, DestroyGivesEverySlotBack)
{
    const Blob blob(two_part_vehicle(Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene(5));

    NodeBeamStructure first;
    ASSERT_TRUE(first.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));
    first.destroy(solver);
    EXPECT_EQ(first.node_count(), 0u);

    NodeBeamStructure second;
    EXPECT_TRUE(second.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));
}

/** @brief A beam worked past its yield load keeps the length it was pulled to. */
TEST(Unit_NodeBeamStructure, DeformsABeamThatPassesItsThreshold)
{
    // 100 kg under gravity is 981 N, an order of magnitude past this threshold.
    const Blob blob(hanging_pair(Scalar(100), Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    // Not on the first tick: the beam starts at its rest length, and the load that
    // yields it is the one that builds as the hanging node pulls away. Measured, it
    // passes 100 N on the third.
    std::uint32_t deformed = 0;
    std::uint32_t broken = 0;
    for (int i = 0; i < 6; ++i)
    {
        const NodeBeamTickReport report = tick(solver, structure);
        deformed += report.beams_deformed;
        broken += report.beams_broken;
    }
    EXPECT_GT(deformed, 0u);
    EXPECT_EQ(broken, 0u);
    EXPECT_EQ(structure.live_beam_count(), 1u);

    // The dent is permanent: the beam rests longer than it was cooked at, and the
    // strain accumulator says by how much.
    BeamConstraint beam;
    ASSERT_TRUE(solver.read_beam(structure.beam(0), beam));
    EXPECT_GT(beam.rest_length, beam.initial_rest_length);
    EXPECT_GT(beam_plastic_strain(beam), Scalar(0));
    EXPECT_GT(beam.accumulated_plastic_strain, Scalar(0));

    // And it hardens: the accumulator is bounded by the authored maximum however
    // long the load stays on.
    for (int i = 0; i < 120; ++i)
        tick(solver, structure);
    ASSERT_TRUE(solver.read_beam(structure.beam(0), beam));
    EXPECT_LE(beam.accumulated_plastic_strain, beam.maximum_plastic_strain);
}

/** @brief A beam worked past its break load leaves the world, and the node falls free. */
TEST(Unit_NodeBeamStructure, BreaksABeamThatPassesItsBreakThreshold)
{
    const Blob blob(hanging_pair(Scalar(0), Scalar(100)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    std::uint32_t broken = 0;
    for (int i = 0; i < 30; ++i)
        broken += tick(solver, structure).beams_broken;
    EXPECT_EQ(broken, 1u);
    EXPECT_EQ(structure.live_beam_count(), 0u);
    EXPECT_FALSE(structure.beam(0).valid());

    // Nothing holds the hanging node now, so the gap to the pinned one only grows.
    const Scalar before = separation(structure.node_positions()[0], structure.node_positions()[1]);
    for (int i = 0; i < 10; ++i)
        tick(solver, structure);
    const Scalar after = separation(structure.node_positions()[0], structure.node_positions()[1]);
    EXPECT_GT(after, before + Scalar(0.01));
}

/** @brief A part that loses its last tie is reported, once. */
TEST(Unit_NodeBeamStructure, LosingEveryTieDetachesThePart)
{
    // A newton is nothing next to the load of arresting a node thrown at fifty metres
    // a second, so the door's two ties fail and part 0's — authored unbreakable — do not.
    const Blob blob(two_part_vehicle(Scalar(1)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));
    yank(solver, structure.node(2), Vector3{0, -50, 0});

    std::uint32_t detached = 0;
    std::uint32_t attachments_broken = 0;
    std::uint32_t beams_broken = 0;
    for (int i = 0; i < 8; ++i)
    {
        const NodeBeamTickReport report = tick(solver, structure);
        detached += report.parts_detached;
        attachments_broken += report.attachments_broken;
        beams_broken += report.beams_broken;
    }

    EXPECT_EQ(attachments_broken, 1u);
    EXPECT_EQ(beams_broken, 1u);
    EXPECT_EQ(detached, 1u);
    EXPECT_TRUE(structure.part_detached(1));
    EXPECT_FALSE(structure.part_detached(0));

    // Reported once and not again, however long the part keeps falling.
    for (int i = 0; i < 4; ++i)
        EXPECT_EQ(tick(solver, structure).parts_detached, 0u);
}

/** @brief A part nothing ever held is never reported as having come off. */
TEST(Unit_NodeBeamStructure, PartWithNoTiesIsNeverDetached)
{
    const Blob blob(hanging_pair(Scalar(0), Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    for (int i = 0; i < 8; ++i)
        EXPECT_EQ(tick(solver, structure).parts_detached, 0u);
    EXPECT_FALSE(structure.part_detached(0));
}

/** @brief An unbreakable attachment keeps the shell on the core through a fall. */
TEST(Unit_NodeBeamStructure, AttachmentHoldsTheShellToTheCore)
{
    const Blob blob(two_part_vehicle(Scalar(0)));
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));

    RigidBody core;
    ASSERT_TRUE(solver.read_body(structure.core(), core));
    // In the core's frame, because the assembly picks up a spin when only the core is
    // pushed — and a mount that held perfectly through a rotation would read as a
    // large drift if the offset were compared in world space.
    const Vector3 authored =
        rotate(conjugate(core.orientation), structure.node_positions()[0] - core.position);
    const Vector3 started = structure.node_positions()[0];

    yank(solver, structure.core(), Vector3{20, 0, 0});
    for (int i = 0; i < 60; ++i)
        tick(solver, structure);

    ASSERT_TRUE(solver.read_body(structure.core(), core));
    const Vector3 held =
        rotate(conjugate(core.orientation), structure.node_positions()[0] - core.position);
    // Half a metre of travel, so the test cannot pass by nothing having happened.
    EXPECT_GT(separation(started, structure.node_positions()[0]), Scalar(0.5));
    EXPECT_LT(separation(authored, held), Scalar(1e-4));
    EXPECT_EQ(structure.live_attachment_count(), blob.view.attachment_count);
}

/** @brief The render binding reads the live node cloud, not the cooked one (§0.4). */
TEST(Unit_NodeBeamStructure, SkinFollowsTheInstancedStructure)
{
    Cooking::NodeBeamAsset asset = two_part_vehicle(Scalar(0));
    const Vector3 vertex{0, 0.2, 0.1};
    Cooking::NodeBeamSkinRecord record{};
    for (std::uint32_t i = 0; i < Cooking::NODE_BEAM_SKIN_INFLUENCES; ++i)
    {
        record.nodes[i] = i;
        record.weights[i] = 0.25f;
    }
    float offset[3] = {0.0f, 0.0f, 0.0f};
    Cooking::build_node_beam_skin_offset(asset.nodes.data(), record, vertex, offset);
    for (int i = 0; i < 3; ++i)
        record.offset[i] = offset[i];
    asset.skin.push_back(record);
    finish(asset);

    const Blob blob(asset);
    ASSERT_TRUE(blob.view.valid);

    NodeBeamStructureSettings<Scalar> settings;
    settings.position = Vector3{3, 5, -2};

    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    ASSERT_TRUE(structure.create(solver, blob.view, settings));

    const Vector3 skinned = Cooking::evaluate_node_beam_skin(blob.view, blob.view.skin[0],
                                                             structure.node_positions().data());
    // A millionth of a metre: the offset is stored as a `float`, and that is the
    // whole of the error the reconstruction can carry at this distance from the
    // origin.
    EXPECT_LT(separation(settings.position + vertex, skinned), Scalar(1e-6));
}

/**
 * @brief Two runs of the same vehicle break the same things in the same order (§0.5).
 *
 * The assertion that matters is the *positions* after the breakages, not the counts:
 * a boundary pass that removed constraints in an order that varied would leave the
 * two solvers with different slot layouts, and a replay that reproduces the counts
 * and not the poses is not a replay.
 */
TEST(Unit_NodeBeamStructure, ReplayIsIdentical)
{
    const Blob blob(two_part_vehicle(Scalar(1)));
    std::vector<Vector3> reference;
    std::vector<std::uint32_t> reference_reports;

    for (int run = 0; run < 2; ++run)
    {
        HostXpbdSolver<Scalar> solver(structure_scene());
        NodeBeamStructure structure;
        ASSERT_TRUE(structure.create(solver, blob.view, NodeBeamStructureSettings<Scalar>{}));
        yank(solver, structure.node(2), Vector3{0, -50, 0});

        std::vector<std::uint32_t> reports;
        for (int i = 0; i < 12; ++i)
        {
            const NodeBeamTickReport report = tick(solver, structure);
            reports.push_back(report.beams_broken);
            reports.push_back(report.attachments_broken);
            reports.push_back(report.parts_detached);
        }

        if (run == 0)
        {
            reference = structure.node_positions();
            reference_reports = reports;
            continue;
        }

        ASSERT_EQ(reports, reference_reports);
        ASSERT_EQ(structure.node_positions().size(), reference.size());
        for (std::size_t i = 0; i < reference.size(); ++i)
        {
            EXPECT_EQ(structure.node_positions()[i].x, reference[i].x);
            EXPECT_EQ(structure.node_positions()[i].y, reference[i].y);
            EXPECT_EQ(structure.node_positions()[i].z, reference[i].z);
        }
    }
}

/** @brief A structure that was never created does nothing at the tick boundary. */
TEST(Unit_NodeBeamStructure, EmptyStructureReportsNothing)
{
    HostXpbdSolver<Scalar> solver(structure_scene());
    NodeBeamStructure structure;
    const NodeBeamTickReport report = structure.end_tick(solver);
    EXPECT_EQ(report.beams_deformed, 0u);
    EXPECT_EQ(report.beams_broken, 0u);
    EXPECT_EQ(report.attachments_broken, 0u);
    EXPECT_EQ(report.parts_detached, 0u);
}
