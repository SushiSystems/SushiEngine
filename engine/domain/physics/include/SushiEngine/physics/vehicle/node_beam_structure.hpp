/**************************************************************************/
/* node_beam_structure.hpp                                                */
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

#pragma once

/**
 * @file node_beam_structure.hpp
 * @brief §11.2's hybrid, alive: a cooked vehicle turned into bodies and constraints,
 *        and the tick boundary where it dents, breaks, and loses parts.
 *
 * `.sushinodebeam` is a description. This is what makes it a thing in a world:
 * every node record becomes a particle in the solver, every beam record becomes the
 * fifth constraint kind, and — when the asset carries one — the rigid core becomes a
 * body the shell hangs off. Nothing here is new physics. It is the *assembly* §11
 * opens by saying it is.
 *
 * ### The three decisions this file is
 *
 * **The attachment is a ball joint, not a new kind.** §10.3 describes an attachment
 * that averages its correction across a small vertex neighbourhood, so a mount does
 * not tear one vertex out of a mesh. That averaging is a *soft-body* problem, where
 * the endpoint is a patch of a continuum. A node-beam shell has no such ambiguity:
 * the cooker already chose which node the mount acts on, and a node is a whole body.
 * So the attachment is `JointKind::Ball` with the lever on the core and none on the
 * node — which is exactly the constraint that is wanted, and arrives with §10.4's
 * force recovery and `JointFlags::broken` already built. A new kind would have
 * re-derived both.
 *
 * **The tick boundary belongs here, not in the solver.** A solver projects; it does
 * not decide policy. The dent and the failure are read out of what the solve
 * recovered and applied by whoever owns the structure, which is the asymmetry
 * `IConstraintSolver::write_beam` exists to serve and `write_element` deliberately
 * does not. @ref NodeBeamStructureT::end_tick is that owner's one call.
 *
 * **A part comes off by losing its last tie, and then nothing happens to it.** When
 * every beam joining a part to the rest of the vehicle and every attachment holding
 * it to the core have broken, the part is already free: its nodes are still bodies,
 * still beamed to each other, still colliding, and they now fly away as the loose
 * node cloud a torn-off door is. There is nothing to remove and nothing to respawn.
 * What this class adds is the *report* — a caller that wants to play a sound or spawn
 * debris needs to be told, and reconstructing "is this part still held" from the
 * outside would mean walking the whole beam list every tick.
 *
 * The structure **borrows** the solver, on `SoftBodyScene`'s reasoning: several things
 * reach the same solver and giving one of them the lifetime is what makes the others
 * hold pointers they cannot validate. Every call that touches the world takes the
 * solver as an argument, so a structure can never act on a solver it was not given.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/beam_constraint.hpp>
#include <SushiEngine/physics/constraints/beam_projection.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/aero/wind.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/solver/solver_interface.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Where a cooked vehicle is put, and what its bodies are made of.
         *
         * @tparam T The scalar element type the solver runs in.
         */
        template <typename T>
        struct NodeBeamStructureSettings
        {
            /** @brief World position the asset's local origin lands on. */
            Vector3T<T> position{};

            /** @brief World orientation applied about that origin. */
            QuaternionT<T> orientation{};

            /**
             * @brief Linear velocity every body starts with, in metres per second.
             *
             * Here rather than left to the caller because a vehicle spawned in motion
             * is the ordinary case and doing it afterwards means writing every node
             * back individually — and getting the core's rigid velocity field right
             * as well, which is the half a caller forgets.
             */
            Vector3T<T> velocity{};

            /** @brief Scene material table index the nodes are given. */
            std::uint32_t node_material_index = 0;

            /**
             * @brief Scene material table index the rigid core is given.
             *
             * Its own, because the core and the shell are not the same surface: the
             * shell is the panel that touches the world and the core is a mass and an
             * inertia tensor whose collision shape is authored separately (§11.2).
             */
            std::uint32_t core_material_index = 0;

            /**
             * @brief Air density the shell's drag is computed at, in kg/m³.
             *
             * A setting rather than a constant because a vehicle on a mountain pass and
             * one at sea level are the same asset in different air, and because zero is
             * how a test says "no aerodynamics" without touching the asset.
             */
            T air_density = sea_level_air_density<T>;

            /**
             * @brief Drag coefficient of one node's tributary area, dimensionless.
             *
             * A node stands for a patch of panel presented to the airflow, and a flat
             * plate is about 1.2. It is one number for the whole shell because the
             * alternative is a per-node coefficient the cooker has no way to derive — the
             * mesh knows a node's area, not which way its panel faces the wind.
             */
            T node_drag_coefficient = T(1.2);
        };

        /** @brief What one tick's boundary pass changed; every count is for that tick alone. */
        struct NodeBeamTickReport
        {
            /** @brief Beams whose rest length crept — the permanent dents taken this tick. */
            std::uint32_t beams_deformed = 0;

            /** @brief Beams that passed their break threshold and were removed. */
            std::uint32_t beams_broken = 0;

            /** @brief Attachments that passed their break threshold and were removed. */
            std::uint32_t attachments_broken = 0;

            /**
             * @brief Parts that lost their last tie to the rest of the vehicle.
             *
             * Reported once, on the tick it happens. A part that was never tied to
             * anything is never counted; see @ref NodeBeamStructureT::part_detached.
             */
            std::uint32_t parts_detached = 0;
        };

        /**
         * @brief One instanced vehicle: nodes, beams, a core, and what holds them together.
         *
         * Non-copyable, because two copies would name the same solver slots and the
         * second to be destroyed would remove bodies the first had already removed —
         * or, worse, bodies that had since taken the slots.
         *
         * @tparam T The scalar element type the solver runs in.
         */
        template <typename T>
        class NodeBeamStructureT
        {
            public:
                /** @brief The solver seam this structure instances into. */
                using Solver = IConstraintSolver<T>;

                NodeBeamStructureT() = default;
                NodeBeamStructureT(const NodeBeamStructureT&) = delete;
                NodeBeamStructureT& operator=(const NodeBeamStructureT&) = delete;
                NodeBeamStructureT(NodeBeamStructureT&&) = default;
                NodeBeamStructureT& operator=(NodeBeamStructureT&&) = default;

                /**
                 * @brief Instances a cooked vehicle into @p solver.
                 *
                 * All or nothing: a budget exhausted part way through removes
                 * everything already added and reports failure, because a vehicle
                 * missing the beams that would not fit is a structure that folds the
                 * first time it is touched, and it would fold in a way that reads as a
                 * physics bug rather than as a capacity one.
                 *
                 * @param solver   The world; borrowed only for the duration of the call.
                 * @param view     A validated asset; an invalid one is refused.
                 * @param settings Where to put it and what to build it from.
                 * @return False when the asset was unusable or the solver had no room.
                 */
                bool create(Solver& solver, const Cooking::NodeBeamAssetView& view,
                            const NodeBeamStructureSettings<T>& settings)
                {
                    destroy(solver);
                    if (!view.valid || view.node_count == 0)
                        return false;

                    parts_.assign(view.summary.part_count == 0 ? 1 : view.summary.part_count,
                                  PartState{});
                    if (!create_core(solver, view, settings) ||
                        !create_nodes(solver, view, settings) || !create_beams(solver, view) ||
                        !create_attachments(solver, view))
                    {
                        destroy(solver);
                        return false;
                    }

                    refresh_node_positions(solver);
                    return true;
                }

                /**
                 * @brief Removes every body this structure added, and with them its constraints.
                 *
                 * The beams and attachments are not removed one by one: `remove_body`
                 * already takes every constraint naming the body with it, and removing
                 * a constraint whose body is gone would be the second half of a job the
                 * first half already finished.
                 *
                 * @param solver The world the structure was created in.
                 */
                void destroy(Solver& solver)
                {
                    for (const BodyHandle& node : nodes_)
                    {
                        if (node.valid())
                            solver.remove_body(node);
                    }
                    if (core_.valid())
                        solver.remove_body(core_);

                    nodes_.clear();
                    node_parts_.clear();
                    node_positions_.clear();
                    beams_.clear();
                    attachments_.clear();
                    parts_.clear();
                    core_ = BodyHandle{};
                    core_frame_ = QuaternionT<T>{};
                    core_center_ = Vector3T<T>{};
                }

                /**
                 * @brief Applies §11.1's tick-boundary rules, once, after the solve.
                 *
                 * In index order throughout — beams before attachments, each in the
                 * order the asset lists them — because the pass removes constraints and
                 * a removal order that varied would give a device solver a different
                 * slot layout on a replay, which is §0.5's whole failure mode.
                 *
                 * @param solver The world the structure was created in.
                 * @return What changed this tick.
                 */
                NodeBeamTickReport end_tick(Solver& solver)
                {
                    NodeBeamTickReport report;
                    refresh_node_positions(solver);
                    step_beams(solver, report);
                    step_attachments(solver, report);
                    collect_detached_parts(report);
                    return report;
                }

                /**
                 * @brief Re-reads every node's position out of the solver.
                 *
                 * The one transfer the render binding and the plasticity pass share.
                 * Cached in the boundary `Scalar` whatever column the solve runs in,
                 * because that is the type everything reading it speaks — and because
                 * `evaluate_node_beam_skin` takes exactly this array as its override.
                 *
                 * @param solver The world the structure was created in.
                 */
                void refresh_node_positions(const Solver& solver)
                {
                    RigidBodyT<T> body;
                    for (std::size_t i = 0; i < nodes_.size(); ++i)
                    {
                        if (!nodes_[i].valid() || !solver.read_body(nodes_[i], body))
                            continue;
                        node_positions_[i] = Vector3{Scalar(body.position.x),
                                                     Scalar(body.position.y),
                                                     Scalar(body.position.z)};
                    }
                }

                /**
                 * @brief Where every node is, in asset order, in world space.
                 *
                 * As of the last @ref refresh_node_positions or @ref end_tick. Hand it
                 * to `Cooking::evaluate_node_beam_skin` as the position override and the
                 * render mesh follows the deformed structure (§0.4, §8.6).
                 */
                const std::vector<Vector3>& node_positions() const noexcept
                {
                    return node_positions_;
                }

                /** @brief How many nodes were instanced. */
                std::size_t node_count() const noexcept { return nodes_.size(); }

                /**
                 * @brief One node's body handle, for naming it in something else.
                 * @param index The node, in asset order; out of range gives an invalid handle.
                 */
                BodyHandle node(std::size_t index) const noexcept
                {
                    return index < nodes_.size() ? nodes_[index] : BodyHandle{};
                }

                /** @brief The chassis core's body, or an invalid handle for a pure node-beam vehicle. */
                BodyHandle core() const noexcept { return core_; }

                /** @brief Whether the asset carried a rigid core at all (§11.2). */
                bool has_core() const noexcept { return core_.valid(); }

                /**
                 * @brief The rotation taking asset space into the core's body frame.
                 *
                 * The conjugate of the cooked principal rotation, because the core is
                 * instanced *rotated into* its principal frame — the only frame in which
                 * an inertia tensor is the diagonal `RigidBodyT` stores. Anything
                 * mounting to the core has to express its anchors and axes there, and
                 * deriving this a second time at every mounting site is how two places
                 * end up disagreeing about which way the chassis faces.
                 */
                const QuaternionT<T>& core_frame() const noexcept { return core_frame_; }

                /**
                 * @brief The core's centre of mass, in asset space.
                 *
                 * The other half of what a mount needs: a joint's local anchor is
                 * measured from the centre of mass, and the asset states positions
                 * from the origin.
                 */
                const Vector3T<T>& core_center() const noexcept { return core_center_; }

                /** @brief How many beams were instanced, including the ones that have since broken. */
                std::size_t beam_count() const noexcept { return beams_.size(); }

                /**
                 * @brief One beam's handle, for reading the load it is carrying.
                 *
                 * §9.3's *mukavemet* readout for a structural member: an inspector
                 * resolves this against the solver and gets `beam_force` and
                 * `beam_plastic_strain` for it. Invalid once the beam has broken,
                 * which is the honest answer — there is no member left to read.
                 *
                 * @param index The beam, in asset order; out of range gives an invalid handle.
                 */
                ConstraintHandle beam(std::size_t index) const noexcept
                {
                    return index < beams_.size() ? beams_[index].handle : ConstraintHandle{};
                }

                /**
                 * @brief One attachment's handle, for reading what the mount is carrying.
                 * @param index The attachment, in asset order; out of range gives an invalid handle.
                 */
                JointHandle attachment(std::size_t index) const noexcept
                {
                    return index < attachments_.size() ? attachments_[index].handle : JointHandle{};
                }

                /** @brief How many beams are still in the world. */
                std::size_t live_beam_count() const noexcept
                {
                    std::size_t live = 0;
                    for (const BeamLink& beam : beams_)
                    {
                        if (beam.handle.valid())
                            ++live;
                    }
                    return live;
                }

                /** @brief How many attachments are still holding the shell to the core. */
                std::size_t live_attachment_count() const noexcept
                {
                    std::size_t live = 0;
                    for (const AttachmentLink& attachment : attachments_)
                    {
                        if (attachment.handle.valid())
                            ++live;
                    }
                    return live;
                }

                /** @brief How many parts the asset declared. */
                std::size_t part_count() const noexcept { return parts_.size(); }

                /**
                 * @brief Whether a part has lost every tie to the rest of the vehicle.
                 *
                 * False for a part that had no ties to begin with, and that is deliberate
                 * rather than an oversight: a single-part asset with no core is held
                 * together by nothing *by design*, and answering "detached" for the whole
                 * vehicle on its first tick would make the readout useless for the case
                 * it exists to report.
                 *
                 * @param part The part index; out of range answers false.
                 */
                bool part_detached(std::size_t part) const noexcept
                {
                    return part < parts_.size() && parts_[part].detached;
                }

            private:
                /** @brief A beam, and the two nodes it needs for the plasticity pass. */
                struct BeamLink
                {
                    ConstraintHandle handle;
                    std::uint32_t node_a = 0;
                    std::uint32_t node_b = 0;
                };

                /** @brief An attachment, and the part it holds down. */
                struct AttachmentLink
                {
                    JointHandle handle;
                    std::uint32_t part = 0;
                };

                /** @brief How many ties still hold one part on, and whether it has let go. */
                struct PartState
                {
                    /** @brief Live attachments plus live beams crossing out of this part. */
                    std::uint32_t ties = 0;

                    /** @brief How many it started with; a part that began at zero is never reported. */
                    std::uint32_t initial_ties = 0;

                    /** @brief Whether @ref ties has reached zero from a non-zero start. */
                    bool detached = false;

                    /** @brief Whether the detachment has already been reported once. */
                    bool reported = false;
                };

                /** @brief Builds the chassis body, when the asset carries one. */
                bool create_core(Solver& solver, const Cooking::NodeBeamAssetView& view,
                                 const NodeBeamStructureSettings<T>& settings)
                {
                    if (!Cooking::node_beam_has_core(view.core))
                        return true;

                    RigidBodyT<T> body;
                    // The body frame is the *principal* frame, because that is the only
                    // frame in which an inertia tensor is the diagonal `inv_inertia`
                    // stores. Rotating the body into it and expressing the centre of
                    // mass there is what keeps `body_origin` landing back on the
                    // asset's origin.
                    const QuaternionT<T> principal = column_quaternion(view.core.principal_rotation);
                    const Vector3T<T> center = column_vector(view.core.center_of_mass);
                    core_frame_ = conjugate(principal);
                    core_center_ = center;
                    body.orientation = mul(settings.orientation, principal);
                    body.previous_orientation = body.orientation;
                    body.center_of_mass_local = rotate(conjugate(principal), center);
                    body.position = settings.position + rotate(settings.orientation, center);
                    body.previous_position = body.position;
                    body.velocity = settings.velocity;
                    body.inv_mass = T(1) / T(view.core.mass);
                    body.inv_inertia = inverse_inertia(view.core.principal_inertia);
                    body.material_index = settings.core_material_index;
                    body.flags = BodyFlags::dynamic_body;

                    core_ = solver.add_body(body);
                    return core_.valid();
                }

                /** @brief Builds one particle per node record. */
                bool create_nodes(Solver& solver, const Cooking::NodeBeamAssetView& view,
                                  const NodeBeamStructureSettings<T>& settings)
                {
                    nodes_.reserve(view.node_count);
                    node_parts_.reserve(view.node_count);
                    node_positions_.assign(view.node_count, Vector3{0, 0, 0});

                    for (std::uint32_t i = 0; i < view.node_count; ++i)
                    {
                        const Cooking::NodeBeamNodeRecord& record = view.nodes[i];
                        RigidBodyT<T> body;
                        body.position = settings.position +
                                        rotate(settings.orientation, column_vector(record.position));
                        body.previous_position = body.position;
                        body.velocity = settings.velocity;
                        // A node is a particle: §11.1 defines it as a body with zero
                        // inverse inertia, which is what makes a beam's lack of anchors
                        // correct and what leaves its orientation meaningless.
                        body.inv_inertia = Vector3T<T>{T(0), T(0), T(0)};
                        const bool pinned =
                            (record.flags & Cooking::NodeBeamNodeFlags::fixed) != 0;
                        body.inv_mass =
                            (pinned || !(record.mass > Scalar(0))) ? T(0) : T(1) / T(record.mass);
                        body.material_index = settings.node_material_index;
                        // §11.6, at the shell's end: the cooker measures each node's
                        // tributary area, and this is what reads it. A node is a flat
                        // patch of panel, so its drag coefficient is a plate's; the
                        // constant is derived here rather than authored because what an
                        // asset has is an area and what `predict` spends is an
                        // acceleration.
                        body.drag_coefficient =
                            pinned ? T(0)
                                   : quadratic_drag_constant(settings.node_drag_coefficient,
                                                             T(record.drag_area), T(record.mass),
                                                             settings.air_density);
                        body.flags = BodyFlags::dynamic_body;

                        const BodyHandle handle = solver.add_body(body);
                        if (!handle.valid())
                            return false;
                        nodes_.push_back(handle);
                        node_parts_.push_back(record.part);
                        node_positions_[i] = Vector3{Scalar(body.position.x),
                                                     Scalar(body.position.y),
                                                     Scalar(body.position.z)};
                    }
                    return true;
                }

                /** @brief Builds one beam per beam record, and counts the ties they make. */
                bool create_beams(Solver& solver, const Cooking::NodeBeamAssetView& view)
                {
                    beams_.reserve(view.beam_count);
                    for (std::uint32_t i = 0; i < view.beam_count; ++i)
                    {
                        const Cooking::NodeBeamBeamRecord& record = view.beams[i];
                        BeamConstraintT<T> beam;
                        beam.a = std::uint32_t(solver.body_slot(nodes_[record.a]));
                        beam.b = std::uint32_t(solver.body_slot(nodes_[record.b]));
                        beam.flags = BeamFlags::enabled;
                        beam.rest_length = T(record.rest_length);
                        beam.initial_rest_length = T(record.rest_length);
                        beam.compliance = T(record.compliance);
                        beam.damping = T(record.damping);
                        beam.deform_force = T(record.deform_force);
                        beam.break_force = T(record.break_force);
                        beam.plastic_creep = T(record.plastic_creep);
                        beam.maximum_plastic_strain = T(record.maximum_plastic_strain);

                        const ConstraintHandle handle = solver.add_beam(beam);
                        if (!handle.valid())
                            return false;

                        BeamLink link;
                        link.handle = handle;
                        link.node_a = record.a;
                        link.node_b = record.b;
                        beams_.push_back(link);
                        add_beam_ties(link);
                    }
                    return true;
                }

                /** @brief Builds one ball joint per attachment record. */
                bool create_attachments(Solver& solver, const Cooking::NodeBeamAssetView& view)
                {
                    if (!core_.valid())
                        return true;

                    const QuaternionT<T> principal = column_quaternion(view.core.principal_rotation);
                    const std::uint32_t core_slot = std::uint32_t(solver.body_slot(core_));
                    attachments_.reserve(view.attachment_count);

                    for (std::uint32_t i = 0; i < view.attachment_count; ++i)
                    {
                        const Cooking::NodeBeamAttachmentRecord& record = view.attachments[i];
                        JointConstraintT<T> joint;
                        joint.kind = JointKind::Ball;
                        joint.flags = JointFlags::enabled;
                        joint.a = core_slot;
                        joint.b = std::uint32_t(solver.body_slot(nodes_[record.node]));
                        // The record's anchor is already measured from the core's centre
                        // of mass, which is what a joint's local anchor means — but in
                        // asset space, so it turns into the core's principal frame.
                        joint.local_anchor_a =
                            rotate(conjugate(principal), column_vector(record.core_anchor));
                        // None on the node: a particle's position *is* its centre of mass.
                        joint.local_anchor_b = Vector3T<T>{T(0), T(0), T(0)};
                        joint.compliance = T(record.compliance);
                        joint.break_force = T(record.break_force);

                        const JointHandle handle = solver.add_joint(joint);
                        if (!handle.valid())
                            return false;

                        AttachmentLink link;
                        link.handle = handle;
                        link.part = record.part;
                        attachments_.push_back(link);
                        add_tie(record.part);
                    }
                    return true;
                }

                /** @brief Reads every beam back, dents it, and removes the ones that failed. */
                void step_beams(Solver& solver, NodeBeamTickReport& report)
                {
                    BeamConstraintT<T> beam;
                    for (BeamLink& link : beams_)
                    {
                        if (!link.handle.valid())
                            continue;
                        if (!solver.read_beam(link.handle, beam))
                        {
                            // The solver no longer knows this beam, which means a body it
                            // named was removed from outside. The tie it stood for is
                            // gone either way, so it is accounted for as broken rather
                            // than left holding a part on that nothing holds.
                            link.handle = ConstraintHandle{};
                            drop_beam_ties(link);
                            continue;
                        }

                        if (beam_should_break(beam))
                        {
                            solver.remove_beam(link.handle);
                            link.handle = ConstraintHandle{};
                            drop_beam_ties(link);
                            ++report.beams_broken;
                            continue;
                        }

                        const T before = beam.rest_length;
                        apply_beam_plasticity(column_vector(node_positions_[link.node_a]),
                                              column_vector(node_positions_[link.node_b]), beam);
                        if (beam.rest_length != before)
                        {
                            solver.write_beam(link.handle, beam);
                            ++report.beams_deformed;
                        }
                    }
                }

                /** @brief Reads every attachment back and removes the ones that tore out. */
                void step_attachments(Solver& solver, NodeBeamTickReport& report)
                {
                    JointConstraintT<T> joint;
                    for (AttachmentLink& link : attachments_)
                    {
                        if (!link.handle.valid())
                            continue;
                        if (!solver.read_joint(link.handle, joint))
                        {
                            link.handle = JointHandle{};
                            drop_tie(link.part);
                            continue;
                        }
                        if (!joint_should_break(joint))
                            continue;

                        solver.remove_joint(link.handle);
                        link.handle = JointHandle{};
                        drop_tie(link.part);
                        ++report.attachments_broken;
                    }
                }

                /** @brief Turns this tick's tie losses into one report line each. */
                void collect_detached_parts(NodeBeamTickReport& report) noexcept
                {
                    for (PartState& part : parts_)
                    {
                        if (!part.detached || part.reported)
                            continue;
                        part.reported = true;
                        ++report.parts_detached;
                    }
                }

                /** @brief Counts a beam as a tie for each part it crosses out of. */
                void add_beam_ties(const BeamLink& link) noexcept
                {
                    const std::uint32_t part_a = node_parts_[link.node_a];
                    const std::uint32_t part_b = node_parts_[link.node_b];
                    if (part_a == part_b)
                        return;
                    add_tie(part_a);
                    add_tie(part_b);
                }

                /** @brief Gives back what @ref add_beam_ties took. */
                void drop_beam_ties(const BeamLink& link) noexcept
                {
                    const std::uint32_t part_a = node_parts_[link.node_a];
                    const std::uint32_t part_b = node_parts_[link.node_b];
                    if (part_a == part_b)
                        return;
                    drop_tie(part_a);
                    drop_tie(part_b);
                }

                /** @brief Records one more thing holding @p part on. */
                void add_tie(std::uint32_t part) noexcept
                {
                    if (part >= parts_.size())
                        return;
                    ++parts_[part].ties;
                    ++parts_[part].initial_ties;
                }

                /** @brief Records one fewer, and notices the last one going. */
                void drop_tie(std::uint32_t part) noexcept
                {
                    if (part >= parts_.size() || parts_[part].ties == 0)
                        return;
                    --parts_[part].ties;
                    if (parts_[part].ties == 0 && parts_[part].initial_ties > 0)
                        parts_[part].detached = true;
                }

                /** @brief A boundary-precision vector in the solver's column. */
                static Vector3T<T> column_vector(const Vector3& v) noexcept
                {
                    return Vector3T<T>{T(v.x), T(v.y), T(v.z)};
                }

                /** @brief A boundary-precision rotation in the solver's column. */
                static QuaternionT<T> column_quaternion(const Quaternion& q) noexcept
                {
                    return QuaternionT<T>{T(q.x), T(q.y), T(q.z), T(q.w)};
                }

                /**
                 * @brief The reciprocal of each principal moment, with zero for zero.
                 *
                 * A zero moment is a degenerate cook — a core with no extent about an
                 * axis — and zero inverse inertia is the reading that makes the body
                 * refuse to spin about it rather than the infinity that would make it
                 * spin without bound.
                 */
                static Vector3T<T> inverse_inertia(const Vector3& inertia) noexcept
                {
                    return Vector3T<T>{inertia.x > Scalar(0) ? T(1) / T(inertia.x) : T(0),
                                       inertia.y > Scalar(0) ? T(1) / T(inertia.y) : T(0),
                                       inertia.z > Scalar(0) ? T(1) / T(inertia.z) : T(0)};
                }

                std::vector<BodyHandle> nodes_;
                std::vector<std::uint32_t> node_parts_;
                std::vector<Vector3> node_positions_;
                std::vector<BeamLink> beams_;
                std::vector<AttachmentLink> attachments_;
                std::vector<PartState> parts_;
                BodyHandle core_;
                QuaternionT<T> core_frame_{};
                Vector3T<T> core_center_{};
        };

        /** @brief The boundary structure: @ref NodeBeamStructureT fixed to `Scalar`. */
        using NodeBeamStructure = NodeBeamStructureT<Scalar>;
    } // namespace Physics
} // namespace SushiEngine
