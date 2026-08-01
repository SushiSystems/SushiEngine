/**************************************************************************/
/* physics_simulation.hpp                                                */
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
 * @file physics_simulation.hpp
 * @brief The physics seam: the live scene, driven through one constraint solver.
 *
 * `IPhysicsScene` is the abstraction the live world drives its rigid bodies and
 * cloth through, in the fixed boundary `Scalar` precision. `PhysicsSimulation` is
 * the implementation, computing the XPBD solve in `double` and converting at this
 * boundary, so the ECS and renderer never see the solver's internals.
 *
 * ### One solver, not a family of solvers
 *
 * This class used to hold two `PhysicsWorld`s — one for rigid bodies, one for cloth
 * — driven in lockstep so that contacts spanning them could be resolved between
 * their substeps, plus a host contact pass that pushed single points apart. It now
 * holds **one `IConstraintSolver`**, and that is §0.1's decision finally taking
 * effect rather than a refactor for its own sake:
 *
 * - **A cloth particle is a body.** It has zero inverse inertia and links to its
 *   neighbours through the same `XpbdDistanceConstraint` a rope uses. It was never
 *   a different kind of thing; it lived in a different world only because the world
 *   was immutable and a cloth had to be built all at once.
 * - **A contact is a constraint.** Manifolds are generated here, on the host, and
 *   submitted to the solver as the per-tick constraint kind of §6.3. The solve —
 *   non-penetration, static friction positionally, dynamic friction and restitution
 *   in the velocity pass — happens inside the one composition, on the device, in the
 *   right place in the substep schedule.
 * - So **rigid-to-cloth contact is an ordinary contact**, not a coupling. There is
 *   no lockstep to maintain because there is nothing to keep in step.
 *
 * ### Three consequences worth stating rather than discovering
 *
 * 1. **The substep count is derived, not passed.** §6.2: it is a function of
 *    simulation state, and a caller passing it would make the simulation depend on
 *    something outside itself. What the caller's `substeps` argument now means is a
 *    *floor* — the quality it is willing to pay for regardless of how slowly things
 *    happen to be moving. State can raise it; the caller can raise the floor;
 *    neither lowers what the other asked for.
 * 2. **The field is sampled per body per tick, not per substep.** `predict` runs on
 *    the device inside one composition (§6.6), so there is no point inside the
 *    substep loop at which a host sampler could be called at all. The sampled value
 *    lands in `RigidBodyT::external_acceleration`, which is where `StepParameters`
 *    always said a non-uniform field belonged.
 * 3. **The `iterations` argument is ignored.** §0.2: small steps, not many
 *    iterations. The substep schedule subsumes it, and honouring both would be two
 *    dials controlling one quantity.
 *
 * ### What is still on the host
 *
 * Broadphase, manifold generation and the warm-start cache. §6.6 puts all three in
 * the graph eventually; they are not there yet, and this file is where they will
 * move from. The bodies are read out in one bulk transfer per tick for exactly this
 * reason — the host needs the poses to find contacts — and written back the same
 * way. That single pair of transfers is the honest remaining cost of host-side
 * collision detection, and it is one pair, not one per body.
 */

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>


#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/bvh_broadphase.hpp>
#include <SushiEngine/physics/collision/conservative_advancement.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/collision/scene_query.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/aero/wind.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/scene/islands.hpp>
#include <SushiEngine/physics/soft/cloth.hpp>
#include <SushiEngine/physics/soft/soft_body_instance.hpp>
#include <SushiEngine/physics/soft/soft_body_scene.hpp>
#include <SushiEngine/physics/soft/soft_rigid_shape_collision.hpp>
#include <SushiEngine/physics/soft/soft_self_collision.hpp>
#include <SushiEngine/physics/soft/soft_soft_collision.hpp>
#include <SushiEngine/physics/cooking/node_beam_asset.hpp>
#include <SushiEngine/physics/solver/contact_constraint.hpp>
#include <SushiEngine/execution/context.hpp>
#include <SushiEngine/physics/solver/runtime_graph_builder.hpp>
#include <SushiEngine/physics/vehicle/vehicle_instance.hpp>
#include <SushiEngine/sim/collider.hpp>
#include <SushiEngine/sim/physics_services.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief The `IPhysicsScene` implementation; the XPBD solve runs in `double`.
         *
         * It implements every service because it genuinely does every one of those jobs.
         * Consumers should not: the split exists so a caller depends on the one
         * service it uses, and naming this class instead is how that gets undone.
         */
        class PhysicsSimulation final : public IPhysicsScene
        {
            public:
                /**
                 * @brief Creates an empty physics simulation backed by @p context.
                 * @param context The execution context backing the body buffers and the
                 *                solve graph.
                 */
                explicit PhysicsSimulation(Execution::Context& context) noexcept
                    : context_(context)
                {
                }

                // -- IRigidBodyService ---------------------------------------------

                /**
                 * @copydoc IRigidBodyService::set_rigid_bodies
                 *
                 * A *diff*, not a rebuild. The world is mutable now (§6.4), so an
                 * entity that was here last frame keeps its body, its handle and its
                 * velocity; one that has gone is removed with its constraints; one
                 * that is new is admitted. The previous implementation rebuilt the
                 * whole world and carried velocities across by hand, which is the
                 * same outcome reached by re-deriving what the handles already knew.
                 *
                 * @param iterations Ignored; the substep schedule subsumes it (§0.2).
                 */
                void set_rigid_bodies(const std::vector<RigidBodyDesc>& bodies,
                                      std::size_t iterations, Scalar substep_dt) override
                {
                    (void)iterations;
                    substep_dt_ = T(substep_dt);
                    if (bodies.empty() && rigid_.empty())
                        return;
                    ensure_solver();

                    refresh_bodies();
                    seen_.clear();
                    for (const RigidBodyDesc& desc : bodies)
                        seen_.insert(desc.id);

                    // Removals first, so a scene that swaps one body for another does
                    // not transiently need capacity for both.
                    for (std::size_t i = rigid_.size(); i-- > 0;)
                    {
                        if (seen_.count(rigid_[i].entity) != 0)
                            continue;
                        solver_->remove_body(rigid_[i].handle);
                        rigid_.erase(rigid_.begin() + std::ptrdiff_t(i));
                    }
                    reindex_rigid();
                    // `remove_body` already took every joint naming the freed slot; this
                    // drops the host records that named it, so a stale identity cannot
                    // be read back or destroyed twice.
                    prune_joints();

                    for (const RigidBodyDesc& desc : bodies)
                    {
                        const auto it = rigid_index_.find(desc.id);
                        if (it != rigid_index_.end())
                        {
                            update_rigid_entry(rigid_[it->second], desc);
                            continue;
                        }
                        RigidEntry entry;
                        entry.entity = desc.id;
                        Body body;
                        body.position = to_vector(desc.position);
                        body.prev_position = body.position;
                        body.orientation = to_quaternion(desc.orientation);
                        body.prev_orientation = body.orientation;
                        body.inv_mass = T(desc.inv_mass);
                        body.inv_inertia = to_vector(desc.inv_inertia);
                        body.drag_coefficient = T(desc.drag_coefficient);
                        entry.handle = solver_->add_body(body);
                        if (!entry.handle.valid())
                            continue;
                        entry.collider = desc.collider;
                        entry.filter = desc.collider.filter;
                        entry.material = to_material(desc.material);
                        rigid_.push_back(entry);
                        rigid_index_.emplace(desc.id, rigid_.size() - 1);
                    }

                    note_membership_changed();
                }

                void update_rigid_body_params(EntityId id, Scalar inv_mass,
                                              const Vector3& inv_inertia,
                                              Scalar drag_coefficient) override
                {
                    if (!solver_)
                        return;
                    const auto it = rigid_index_.find(id);
                    if (it == rigid_index_.end())
                        return;
                    wake_body(rigid_[it->second].handle);
                    Body body;
                    if (!solver_->read_body(rigid_[it->second].handle, body))
                        return;
                    body.inv_mass = T(inv_mass);
                    body.inv_inertia = to_vector(inv_inertia);
                    body.drag_coefficient = T(drag_coefficient);
                    solver_->write_body(rigid_[it->second].handle, body);
                    bodies_dirty_ = true;
                }

                bool rigid_pose(EntityId id, SolvedPose& out) const override
                {
                    if (!solver_)
                        return false;
                    const auto it = rigid_index_.find(id);
                    if (it == rigid_index_.end())
                        return false;
                    // From the host mirror the tick already refreshed. Served from
                    // the device instead, this would be one queue round trip per
                    // entity per frame, on the one call every entity makes.
                    refresh_bodies();
                    const std::size_t slot = solver_->body_slot(rigid_[it->second].handle);
                    if (slot >= bodies_.size())
                        return false;
                    out.position = from_vector(bodies_[slot].position);
                    out.orientation = from_quaternion(bodies_[slot].orientation);
                    return true;
                }

                /** @copydoc IRigidBodyService::rigid_debug_state */
                bool rigid_debug_state(EntityId id, RigidDebugState& out) const override
                {
                    if (!solver_)
                        return false;
                    const auto it = rigid_index_.find(id);
                    if (it == rigid_index_.end())
                        return false;
                    refresh_bodies();
                    const std::size_t slot = solver_->body_slot(rigid_[it->second].handle);
                    if (slot >= bodies_.size())
                        return false;

                    // The bound comes from the same routine the broadphase uses on the same
                    // shape, rather than from a stored proxy: a proxy is only as fresh as the
                    // last index update, and a debug view whose boxes lag the bodies they
                    // belong to is worse than none — it looks like a broadphase bug.
                    const Physics::Aabb<T> bounds =
                        Physics::shape_world_bounds(shape_for_slot(std::uint32_t(slot)));
                    out.bounds_min = from_vector(bounds.min);
                    out.bounds_max = from_vector(bounds.max);
                    out.island = bodies_[slot].island_index;
                    out.sleeping =
                        Physics::has_any_flag(bodies_[slot].flags, Physics::BodyFlags::sleeping);
                    out.is_static =
                        Physics::has_any_flag(bodies_[slot].flags, Physics::BodyFlags::static_body);
                    return true;
                }

                void set_rigid_pose(EntityId id, const Vector3& position,
                                    const Quaternion& orientation) override
                {
                    if (!solver_)
                        return;
                    const auto it = rigid_index_.find(id);
                    if (it == rigid_index_.end())
                        return;
                    // Before the read, so the body this call is about comes back awake
                    // and the write below is not undone by a stale flag word.
                    wake_body(rigid_[it->second].handle);
                    Body body;
                    if (!solver_->read_body(rigid_[it->second].handle, body))
                        return;
                    body.position = to_vector(position);
                    body.orientation = to_quaternion(orientation);
                    // Clear velocity and align the previous pose so the next velocity
                    // derivation sees no jump — the body is placed, not thrown.
                    body.prev_position = body.position;
                    body.prev_orientation = body.orientation;
                    body.velocity = Vector3T<T>{T(0), T(0), T(0)};
                    body.angular_velocity = Vector3T<T>{T(0), T(0), T(0)};
                    solver_->write_body(rigid_[it->second].handle, body);
                    bodies_dirty_ = true;
                    query_dirty_ = true;
                }

                // -- IClothService -------------------------------------------------

                /**
                 * @copydoc IClothService::set_cloth_grids
                 *
                 * Rebuilt wholesale rather than diffed, unlike the rigid bodies, and
                 * the asymmetry is deliberate: a rigid body's identity survives a
                 * change to its description, while a cloth whose row count changed is
                 * a different cloth with a different constraint topology. There is
                 * nothing to carry across. Grids whose shape has not changed are left
                 * alone, so the common case — nothing changed — costs a comparison.
                 *
                 * @param iterations Ignored; the substep schedule subsumes it (§0.2).
                 */
                void set_cloth_grids(const std::vector<ClothDesc>& grids,
                                     std::size_t iterations, Scalar substep_dt) override
                {
                    (void)iterations;
                    substep_dt_ = T(substep_dt);
                    if (grids.empty() && cloth_.empty())
                        return;
                    if (cloth_matches(grids))
                        return;
                    ensure_solver();

                    for (const ClothEntry& entry : cloth_)
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                            solver_->remove_body(handle);
                    cloth_.clear();
                    cloth_index_.clear();

                    for (const ClothDesc& desc : grids)
                    {
                        if (desc.rows == 0 || desc.cols == 0)
                            continue;
                        ClothEntry entry;
                        entry.entity = desc.id;
                        entry.rows = desc.rows;
                        entry.cols = desc.cols;
                        entry.spacing = T(desc.spacing);
                        entry.thickness = T(desc.thickness);
                        entry.grid = Physics::build_cloth_grid<T>(
                            *solver_, desc.rows, desc.cols, T(desc.spacing),
                            to_vector(desc.origin), T(desc.compliance));
                        cloth_index_.emplace(desc.id, cloth_.size());
                        cloth_.push_back(std::move(entry));
                    }

                    note_membership_changed();
                }

                std::vector<Vector3> cloth_positions(EntityId id) const override
                {
                    std::vector<Vector3> positions;
                    if (!solver_)
                        return positions;
                    const auto it = cloth_index_.find(id);
                    if (it == cloth_index_.end())
                        return positions;
                    refresh_bodies();
                    const ClothEntry& entry = cloth_[it->second];
                    positions.reserve(entry.grid.bodies.size());
                    for (const Physics::BodyHandle handle : entry.grid.bodies)
                    {
                        const std::size_t slot = solver_->body_slot(handle);
                        positions.push_back(slot < bodies_.size()
                                                ? from_vector(bodies_[slot].position)
                                                : Vector3{});
                    }
                    return positions;
                }

                bool cloth_dimensions(EntityId id, std::uint32_t& rows,
                                      std::uint32_t& cols) const override
                {
                    const auto it = cloth_index_.find(id);
                    if (it == cloth_index_.end())
                        return false;
                    rows = static_cast<std::uint32_t>(cloth_[it->second].rows);
                    cols = static_cast<std::uint32_t>(cloth_[it->second].cols);
                    return true;
                }

                // -- ISoftBodyService (§9) -----------------------------------------

                /** @copydoc ISoftBodyService::set_soft_bodies */
                void set_soft_bodies(const std::vector<SoftBodyDesc>& bodies) override
                {
                    if (bodies.empty() && soft_.empty())
                        return;
                    if (soft_matches(bodies))
                        return;

                    soft_.clear();
                    soft_index_.clear();
                    soft_scene_.clear();

                    // Reserved before anything is built, because the scene borrows
                    // pointers into this vector: a reallocation partway through would
                    // leave the scene holding addresses of moved-from instances.
                    soft_.reserve(bodies.size());

                    for (const SoftBodyDesc& desc : bodies)
                    {
                        const Physics::Cooking::SoftBodyAssetView view =
                            Physics::Cooking::load_soft_body_blob(desc.asset, desc.asset_size);
                        if (!view.valid)
                            continue;

                        SoftEntry entry;
                        entry.entity = desc.id;
                        entry.key = soft_key(desc);
                        Physics::SoftBodyPrecisionRequest request;
                        request.cosmetic = desc.cosmetic;
                        request.participates_in_rollback = desc.participates_in_rollback;
                        if (!entry.instance.create(
                                view, desc.level, desc.material, desc.origin,
                                Physics::resolve_soft_body_precision(view, request)))
                            continue;

                        Physics::SoftBodyCollisionSettings<T> settings;
                        settings.thickness = T(desc.thickness);
                        settings.self_collision = desc.self_collision;
                        entry.instance.set_collision(settings);

                        soft_index_.emplace(desc.id, soft_.size());
                        soft_.push_back(std::move(entry));
                    }

                    rebuild_soft_scene();
                }

                /** @copydoc ISoftBodyService::soft_body_surface */
                bool soft_body_surface(EntityId id, std::vector<Vector3>& positions,
                                       std::vector<std::uint32_t>& indices) const override
                {
                    positions.clear();
                    indices.clear();
                    const auto it = soft_index_.find(id);
                    if (it == soft_index_.end())
                        return false;

                    const Physics::SoftBodyInstance& instance = soft_[it->second].instance;
                    const std::size_t count = instance.particle_count();
                    positions.reserve(count);
                    for (std::size_t i = 0; i < count; ++i)
                        positions.push_back(instance.particle_position(i));

                    const std::uint32_t* source = instance.surface_indices();
                    if (source != nullptr)
                        indices.assign(source, source + instance.surface_index_count());
                    return true;
                }

                /** @copydoc ISoftBodyService::soft_body_maximum_stress */
                Scalar soft_body_maximum_stress(EntityId id) const override
                {
                    const auto it = soft_index_.find(id);
                    return it == soft_index_.end() ? Scalar(0)
                                                   : soft_[it->second].instance.maximum_stress();
                }

                /** @copydoc ISoftBodyService::soft_body_elements */
                bool soft_body_elements(
                    EntityId id, std::vector<SoftBodyElementSample>& elements) const override
                {
                    elements.clear();
                    const auto it = soft_index_.find(id);
                    if (it == soft_index_.end())
                        return false;

                    const Physics::SoftBodyInstance& instance = soft_[it->second].instance;
                    const std::size_t count = instance.element_count();
                    elements.reserve(count);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        SoftBodyElementSample sample;
                        if (instance.element_sample(i, sample.vertex, sample.von_mises_stress,
                                                    sample.plastic_strain))
                            elements.push_back(sample);
                    }
                    return true;
                }

                // -- IStaticGeometryService ----------------------------------------

                void set_static_planes(const std::vector<PlaneDesc>& planes) override
                {
                    planes_.clear();
                    planes_.reserve(planes.size());
                    for (const PlaneDesc& desc : planes)
                    {
                        const Vector3T<T> normal = normalize(to_vector(desc.normal));
                        Physics::PlaneCollider<T> plane;
                        plane.normal = normal;
                        plane.offset = dot(normal, to_vector(desc.point));
                        planes_.push_back(plane);
                    }
                    note_membership_changed();
                }

                // -- IJointService (§10) -------------------------------------------

                /**
                 * @copydoc IJointService::create_joint
                 *
                 * Both endpoints must already own bodies, and the joint names their
                 * *slots* rather than their handles: the solver colours and projects by
                 * slot, and the handle is the host's lifetime bookkeeping. A body
                 * removed later takes its joints with it inside the solver, and the
                 * record here is pruned in the same pass.
                 */
                JointId create_joint(const JointDesc& desc) override
                {
                    ensure_solver();
                    const auto a = rigid_index_.find(desc.body_a);
                    const auto b = rigid_index_.find(desc.body_b);
                    if (a == rigid_index_.end() || b == rigid_index_.end())
                        return NULL_JOINT;

                    Physics::JointConstraintT<T> joint;
                    joint.a = std::uint32_t(solver_->body_slot(rigid_[a->second].handle));
                    joint.b = std::uint32_t(solver_->body_slot(rigid_[b->second].handle));
                    if (joint.a >= solver_->body_capacity() || joint.b >= solver_->body_capacity())
                        return NULL_JOINT;

                    const JointParams& params = desc.params;
                    joint.kind = to_joint_kind(params.type);
                    joint.flags = Physics::JointFlags::enabled;
                    joint.local_anchor_a = to_vector(params.anchor_a);
                    joint.local_anchor_b = to_vector(params.anchor_b);
                    joint.local_basis_a =
                        Physics::joint_frame_from_axis(to_vector(params.axis_a));
                    joint.local_basis_b =
                        Physics::joint_frame_from_axis(to_vector(params.axis_b));
                    joint.compliance = T(params.compliance);
                    joint.linear_limit = to_joint_limit(params.linear_limit);
                    joint.twist_limit = to_joint_limit(params.twist_limit);
                    joint.swing_limit = to_joint_limit(params.swing_limit);
                    joint.motor = to_joint_motor(params.motor);
                    joint.break_force = T(params.break_force);
                    joint.break_torque = T(params.break_torque);

                    const Physics::JointHandle handle = solver_->add_joint(joint);
                    if (!handle.valid())
                        return NULL_JOINT;

                    JointEntry entry;
                    entry.id = next_joint_id_++;
                    entry.handle = handle;
                    entry.a = desc.body_a;
                    entry.b = desc.body_b;
                    joints_.push_back(entry);
                    // A new joint is a disturbance: it can be holding two settled bodies
                    // apart at a distance neither of them currently is, and a joint whose
                    // bodies are asleep would resolve nothing until something else woke
                    // them. Both ends, because either may be the sleeping one.
                    wake_body(rigid_[a->second].handle);
                    wake_body(rigid_[b->second].handle);
                    return entry.id;
                }

                /** @copydoc IJointService::destroy_joint */
                bool destroy_joint(JointId joint) override
                {
                    for (std::size_t i = 0; i < joints_.size(); ++i)
                    {
                        if (joints_[i].id != joint)
                            continue;
                        if (solver_)
                            solver_->remove_joint(joints_[i].handle);
                        joints_.erase(joints_.begin() + std::ptrdiff_t(i));
                        return true;
                    }
                    return false;
                }

                /** @copydoc IJointService::joint_state */
                bool joint_state(JointId joint, JointState& out) const override
                {
                    const JointEntry* entry = find_joint(joint);
                    if (entry == nullptr || !solver_)
                        return false;
                    Physics::JointConstraintT<T> solved;
                    if (!solver_->read_joint(entry->handle, solved))
                        return false;
                    out.force = from_vector(Physics::joint_force(solved));
                    out.torque = from_vector(Physics::joint_torque(solved));
                    out.peak_force = Scalar(solved.peak_force);
                    out.peak_torque = Scalar(solved.peak_torque);
                    return true;
                }

                /** @copydoc IJointService::set_joint_motor */
                bool set_joint_motor(JointId joint, const JointMotorDesc& motor) override
                {
                    const JointEntry* entry = find_joint(joint);
                    if (entry == nullptr || !solver_)
                        return false;
                    Physics::JointConstraintT<T> stored;
                    if (!solver_->read_joint(entry->handle, stored))
                        return false;
                    stored.motor = to_joint_motor(motor);
                    return solver_->write_joint(entry->handle, stored);
                }

                /** @copydoc IJointService::set_joint_limits */
                bool set_joint_limits(JointId joint, const JointLimitDesc& linear,
                                      const JointLimitDesc& twist,
                                      const JointLimitDesc& swing) override
                {
                    const JointEntry* entry = find_joint(joint);
                    if (entry == nullptr || !solver_)
                        return false;
                    Physics::JointConstraintT<T> stored;
                    if (!solver_->read_joint(entry->handle, stored))
                        return false;
                    stored.linear_limit = to_joint_limit(linear);
                    stored.twist_limit = to_joint_limit(twist);
                    stored.swing_limit = to_joint_limit(swing);
                    return solver_->write_joint(entry->handle, stored);
                }

                /** @copydoc IJointService::joint_broken_events */
                const std::vector<JointBrokenEvent>& joint_broken_events() const noexcept override
                {
                    return joint_events_;
                }

                // -- IVehicleService -----------------------------------------------

                /** @copydoc IVehicleService::set_vehicles */
                void set_vehicles(const std::vector<VehicleDesc>& vehicles) override
                {
                    if (vehicles.empty() && vehicles_.empty())
                        return;
                    if (vehicles_match(vehicles))
                        return;
                    ensure_solver();

                    for (VehicleEntry& entry : vehicles_)
                        if (entry.instance)
                            entry.instance->destroy(*solver_);
                    vehicles_.clear();
                    vehicle_index_.clear();

                    for (const VehicleDesc& desc : vehicles)
                    {
                        const Physics::Cooking::NodeBeamAssetView view =
                            Physics::Cooking::load_node_beam_blob(desc.asset, desc.asset_size);
                        if (!view.valid)
                            continue;

                        VehicleEntry entry;
                        entry.entity = desc.id;
                        entry.asset = desc.asset;
                        entry.asset_size = desc.asset_size;
                        entry.position = desc.position;
                        entry.orientation = desc.orientation;
                        entry.instance = std::make_unique<Physics::VehicleInstanceT<T>>();

                        Physics::NodeBeamStructureSettings<T> settings;
                        settings.position = to_vector(desc.position);
                        settings.orientation = to_quaternion(desc.orientation);
                        settings.velocity = to_vector(desc.velocity);

                        if (!entry.instance->create(*solver_, view,
                                                    to_vehicle_setup(desc.setup), settings))
                            continue;

                        entry.surface_indices.assign(
                            view.surface_indices,
                            view.surface_indices + view.surface_index_count);
                        vehicle_index_.emplace(desc.id, vehicles_.size());
                        vehicles_.push_back(std::move(entry));
                    }
                    // A vehicle is hundreds of bodies, so every proxy index the broadphase
                    // holds is now stale - the same rebuild a rigid-body change forces.
                    proxies_dirty_ = true;
                    bodies_dirty_ = true;
                }

                /** @copydoc IVehicleService::set_vehicle_input */
                bool set_vehicle_input(EntityId id, const VehicleInput& input) override
                {
                    const auto it = vehicle_index_.find(id);
                    if (it == vehicle_index_.end())
                        return false;
                    vehicles_[it->second].input = input;
                    return true;
                }

                /** @copydoc IVehicleService::vehicle_report */
                bool vehicle_report(EntityId id, VehicleReport& out) const override
                {
                    const auto it = vehicle_index_.find(id);
                    if (it == vehicle_index_.end())
                        return false;
                    out = vehicles_[it->second].report;
                    return true;
                }

                /** @copydoc IVehicleService::vehicle_core_pose */
                bool vehicle_core_pose(EntityId id, SolvedPose& out) const override
                {
                    const auto it = vehicle_index_.find(id);
                    if (it == vehicle_index_.end() || !solver_)
                        return false;
                    const VehicleEntry& entry = vehicles_[it->second];
                    if (!entry.instance || !entry.instance->structure().has_core())
                        return false;
                    Body core;
                    if (!solver_->read_body(entry.instance->structure().core(), core))
                        return false;
                    out.position = from_vector(core.position);
                    out.orientation = from_quaternion(core.orientation);
                    return true;
                }

                /** @copydoc IVehicleService::vehicle_node_positions */
                bool vehicle_node_positions(EntityId id, std::vector<Vector3>& out) const override
                {
                    const auto it = vehicle_index_.find(id);
                    if (it == vehicle_index_.end() || !solver_)
                        return false;
                    const VehicleEntry& entry = vehicles_[it->second];
                    if (!entry.instance)
                        return false;
                    const Physics::NodeBeamStructureT<T>& structure = entry.instance->structure();
                    out.clear();
                    out.reserve(structure.node_count());
                    Body node;
                    for (std::size_t i = 0; i < structure.node_count(); ++i)
                    {
                        if (!solver_->read_body(structure.node(i), node))
                            continue;
                        out.push_back(from_vector(node.position));
                    }
                    return true;
                }

                /** @copydoc IVehicleService::vehicle_surface */
                bool vehicle_surface(EntityId id, std::vector<Vector3>& positions,
                                     std::vector<std::uint32_t>& indices) const override
                {
                    const auto it = vehicle_index_.find(id);
                    if (it == vehicle_index_.end())
                        return false;
                    const VehicleEntry& entry = vehicles_[it->second];
                    if (entry.surface_indices.empty() || !vehicle_node_positions(id, positions))
                        return false;
                    indices = entry.surface_indices;
                    return true;
                }

                // -- IPhysicsStepper -----------------------------------------------

                /**
                 * @copydoc IPhysicsStepper::step
                 *
                 * The whole tick, in the order the design requires:
                 *
                 * 1. Bring the poses to the host — one bulk transfer, because the
                 *    collision detection that finds this tick's contacts is still a
                 *    host stage (§6.6 moves it, later).
                 * 2. Sample the gravity field per body and fold it into each body's
                 *    own external acceleration.
                 * 3. Refresh the broadphase from those poses, generate a manifold per
                 *    surviving pair, warm-start each from last tick's, and submit them
                 *    as this tick's contact constraints.
                 * 4. One `step`, which is one `run()` of one composition: predict,
                 *    every constraint kind including the contacts, derive velocity,
                 *    and the velocity pass — all of it, every substep, on the device.
                 * 5. Take the solved manifolds back for the next tick to inherit.
                 *
                 * @param substeps A floor under the derived substep count, not the
                 *                 count itself; see this file's header.
                 */
                void step(const GravitySampler& gravity, const WindSampler& wind,
                          std::size_t substeps) override
                {
                    const std::size_t floor = substeps > 0 ? substeps : 1;
                    const T delta_time = T(floor) * substep_dt_;

                    // Measuring is off unless something asked, so a tick nobody is
                    // profiling reads no clock at all (§13.3). The host stages are timed
                    // here because this is where they run; the device half is the
                    // solver's own measurement and arrives with its statistics.
                    StageTimer timer(profiling_requested_);

                    // Before the rigid early-out, not after it. Soft bodies live in their
                    // own world with their own substep schedule and never enter the rigid
                    // solver, so a scene made only of them has an empty `rigid_` and an
                    // empty `cloth_` and would otherwise return without stepping anything.
                    step_soft_bodies(gravity, delta_time, floor);
                    timer.lap(stage_timings_.soft_body_ms);

                    if (!solver_ || (rigid_.empty() && cloth_.empty()))
                    {
                        const T soft_ms = stage_timings_.soft_body_ms;
                        statistics_ = Physics::PhysicsStatistics{};
                        // Reinstated after the wipe rather than left out of it: a scene of
                        // nothing but soft bodies still did work this tick, and reporting
                        // zero for it would make the one case where the soft solve is the
                        // whole cost the one case where it is invisible.
                        statistics_.timings.soft_body_ms = soft_ms;
                        statistics_.timings.total_ms = timer.total();
                        events_.clear();
                        joint_events_.clear();
                        return;
                    }

                    bodies_dirty_ = true;
                    refresh_bodies();
                    apply_gravity_field(gravity, wind);
                    timer.lap(stage_timings_.write_back_ms);
                    refresh_contact_index(delta_time);
                    timer.lap(stage_timings_.broadphase_ms);
                    submit_contacts(delta_time, floor);
                    timer.lap(stage_timings_.narrowphase_ms);

                    Physics::StepParameters<T> parameters;
                    parameters.delta_time = delta_time;
                    // Zero, because the field has already been folded into each
                    // body. Passing it here as well would apply it twice.
                    parameters.gravity = Vector3T<T>{T(0), T(0), T(0)};
                    parameters.substep_floor = floor;
                    // The vehicles' own half of the tick, spent *after* the contacts are
                    // submitted and before the solve: the tyre model reads the normal load
                    // off this tick's manifolds (§11.5), so it cannot run before they
                    // exist, and it puts velocity impulses on the wheels, so it cannot run
                    // after they have been solved.
                    step_vehicles_begin(delta_time);
                    solver_->step(parameters);
                    // Discarded rather than recorded: the solve's cost is the solver's
                    // own measurement, and a host clock around a `run()` would also be
                    // timing the dispatch this side of it.
                    timer.lap();

                    collect_contacts();
                    // Break thresholds are evaluated here, on the host, between steps.
                    // Not a compromise: removing a joint is a topology change, and a
                    // topology change never happens against a running graph (§6.6).
                    // The load it is tested against came off the device with the joint.
                    break_overloaded_joints();
                    // Beams dent, mounts tear out, and a part that has lost its last tie is
                    // reported as having come off - all of it at the tick boundary, for the
                    // same reason breaking a joint is: a topology change never happens
                    // against a running graph (§6.6).
                    step_vehicles_end();

                    bodies_dirty_ = true;
                    refresh_bodies();
                    // The second half of the tick's transfer cost, added to the first:
                    // §16.6's honest remaining price of host-side collision detection is
                    // one number, not two.
                    timer.lap(stage_timings_.write_back_ms, StageTimer::Accumulate);
                    // The partition and the sleep decision, on this tick's evidence.
                    // Its own write-back is counted here rather than above, because it
                    // is a cost the island pass causes and not one the tick would pay
                    // without it.
                    update_islands(delta_time);
                    timer.lap(stage_timings_.island_build_ms);
                    // After the solve and after the poses came back, so an event
                    // reports where the contact ended up and what impulse it took —
                    // not where it was predicted to be before anything was resolved.
                    build_contact_events();
                    stage_timings_.total_ms = timer.total();
                    refresh_statistics();
                    // Everything moved; the next query rebuilds before it answers.
                    query_dirty_ = true;
                }

                /** @copydoc IPhysicsStepper::statistics */
                const Physics::PhysicsStatistics& statistics() const noexcept override
                {
                    return statistics_;
                }

                /** @copydoc IPhysicsStepper::set_profiling_requested */
                void set_profiling_requested(bool enabled) override
                {
                    profiling_requested_ = enabled;
                }

                // -- IContactEventService ------------------------------------------

                /** @copydoc IContactEventService::contact_events */
                const std::vector<ContactEvent>& contact_events() const noexcept override
                {
                    return events_;
                }

                // -- ICollisionQueryService (§7.7) ---------------------------------
                //
                // Queries run against their own hierarchy rather than against the
                // contact pass's, and that is deliberate: a query asks about geometry
                // that is not moving toward anything, so it has no use for the swept
                // bounds the contact pass needs. The index is rebuilt when the world
                // has moved and reused when it has not, so a system firing twenty
                // rays in one frame pays for one build.

                SceneRayHit raycast_closest(const Vector3& origin, const Vector3& direction,
                                            Scalar max_distance,
                                            const SceneQueryFilter& filter) const override
                {
                    refresh_query_index();
                    const Physics::RayHit<T> hit = Physics::raycast_closest<T>(
                        query_index_, [this](Physics::ProxyId id) { return query_shape(id); },
                        to_vector(origin), normalize(to_vector(direction)), T(max_distance),
                        to_query_filter(filter));
                    return from_hit(hit);
                }

                std::vector<SceneRayHit> raycast_all(
                    const Vector3& origin, const Vector3& direction, Scalar max_distance,
                    const SceneQueryFilter& filter) const override
                {
                    refresh_query_index();
                    const std::vector<Physics::RayHit<T>> hits = Physics::raycast_all<T>(
                        query_index_, [this](Physics::ProxyId id) { return query_shape(id); },
                        to_vector(origin), normalize(to_vector(direction)), T(max_distance),
                        to_query_filter(filter));
                    std::vector<SceneRayHit> converted;
                    converted.reserve(hits.size());
                    for (const Physics::RayHit<T>& hit : hits)
                        converted.push_back(from_hit(hit));
                    return converted;
                }

                std::vector<EntityId> overlap_sphere(
                    const Vector3& center, Scalar radius,
                    const SceneQueryFilter& filter) const override
                {
                    refresh_query_index();
                    const std::vector<Physics::ProxyId> found = Physics::overlap_shape<T>(
                        query_index_, [this](Physics::ProxyId id) { return query_shape(id); },
                        Physics::make_sphere_shape<T>(to_vector(center), T(radius)),
                        to_query_filter(filter));
                    std::vector<EntityId> entities;
                    entities.reserve(found.size());
                    for (const Physics::ProxyId id : found)
                        entities.push_back(query_entities_[query_index_.proxy(id).payload]);
                    return entities;
                }

                SceneRayHit sweep_sphere(const Vector3& center, Scalar radius,
                                         const Vector3& direction, Scalar max_distance,
                                         const SceneQueryFilter& filter) const override
                {
                    refresh_query_index();
                    const Physics::RayHit<T> hit = Physics::sweep_shape<T>(
                        query_index_, [this](Physics::ProxyId id) { return query_shape(id); },
                        Physics::make_sphere_shape<T>(to_vector(center), T(radius)),
                        normalize(to_vector(direction)), T(max_distance),
                        to_query_filter(filter));
                    return from_hit(hit);
                }

                SceneRayHit closest_point(const Vector3& point, Scalar max_distance,
                                          const SceneQueryFilter& filter) const override
                {
                    refresh_query_index();
                    const Physics::ClosestResult<T> result = Physics::closest_point<T>(
                        query_index_, [this](Physics::ProxyId id) { return query_shape(id); },
                        Physics::make_sphere_shape<T>(to_vector(point), T(0)), T(max_distance),
                        to_query_filter(filter));
                    SceneRayHit hit;
                    if (!result.found)
                        return hit;
                    hit.hit = true;
                    hit.entity = query_entities_[result.payload];
                    hit.point = from_vector(result.point_on_shape);
                    hit.normal = from_vector(result.normal * T(-1));
                    hit.distance = Scalar(result.distance);
                    return hit;
                }

            private:
                using T = double;
                using Solver = Physics::RuntimeGraphBuilder<T>;
                using Body = Physics::RigidBodyT<T>;
                using Contact = Physics::ContactConstraintT<T>;

                /**
                 * @brief Splits one tick into stages with a host clock, or does nothing.
                 *
                 * Off is the default and off costs nothing — no clock is read, and every
                 * `lap` is a branch that returns. That is the shape §13.3 asks for: a
                 * tick nobody is watching must not pay to be watchable.
                 *
                 * `lap` closes the current stage and opens the next, so the stages tile
                 * the tick exactly and `total` is not the sum of a set of measurements
                 * that quietly overlap.
                 */
                class StageTimer
                {
                    public:
                        /** @brief Whether a lap accumulates into its field or replaces it. */
                        enum Mode { Replace, Accumulate };

                        /** @brief Starts the tick when @p enabled; otherwise inert. */
                        explicit StageTimer(bool enabled) noexcept : enabled_(enabled)
                        {
                            if (enabled_)
                                began_ = mark_ = std::chrono::steady_clock::now();
                        }

                        /** @brief Closes a stage and discards its cost. */
                        void lap() noexcept
                        {
                            if (enabled_)
                                mark_ = std::chrono::steady_clock::now();
                        }

                        /**
                         * @brief Closes a stage into @p field.
                         * @param field Destination, in milliseconds.
                         * @param mode  Replace (the default) or add to what is there.
                         */
                        void lap(T& field, Mode mode = Replace) noexcept
                        {
                            if (!enabled_)
                                return;
                            const std::chrono::steady_clock::time_point now =
                                std::chrono::steady_clock::now();
                            const T elapsed =
                                T(std::chrono::duration<double, std::milli>(now - mark_)
                                      .count());
                            field = mode == Accumulate ? field + elapsed : elapsed;
                            mark_ = now;
                        }

                        /** @brief The whole tick so far, in milliseconds; zero when off. */
                        T total() const noexcept
                        {
                            if (!enabled_)
                                return T(0);
                            return T(std::chrono::duration<double, std::milli>(
                                         std::chrono::steady_clock::now() - began_)
                                         .count());
                        }

                    private:
                        bool enabled_ = false;
                        std::chrono::steady_clock::time_point began_{};
                        std::chrono::steady_clock::time_point mark_{};
                };

                /** @brief One rigid body: who owns it, where it lives, what it collides as. */
                struct RigidEntry
                {
                    EntityId entity = NULL_ENTITY;
                    Physics::BodyHandle handle;
                    Collider collider{};
                    Physics::CollisionFilter filter{};

                    /**
                     * @brief The surface this body contacts as.
                     *
                     * Held per body rather than referenced by `RigidBodyT::material_index`
                     * into a scene table, because there is no scene table: the index exists
                     * so a *device* kernel can reach a material without a pointer, and the
                     * manifold pass that needs one here runs on the host, where the body's
                     * own record is already in hand. A table would be a second place a
                     * material could live and a second thing to keep in step.
                     */
                    Physics::PhysicsMaterialT<T> material{};
                };

                /**
                 * @brief One joint: its boundary identity and the solver handle behind it.
                 *
                 * The two entities are kept so a broken-joint event can name them
                 * *after* the joint is gone — the solver's descriptor holds body slots,
                 * and a slot is not an identity a listener can act on.
                 */
                struct JointEntry
                {
                    JointId id = NULL_JOINT;
                    Physics::JointHandle handle;
                    EntityId a = NULL_ENTITY;
                    EntityId b = NULL_ENTITY;
                };

                /** @brief What a soft body was built from, for the rebuild comparison. */
                struct SoftKey
                {
                    std::size_t asset_size = 0;
                    std::uint32_t level = 0;
                    Vector3 origin;
                    Scalar thickness = Scalar(0.01);
                    bool self_collision = false;
                    bool cosmetic = false;
                    bool rollback = false;
                    Physics::SoftBodyMaterialT<Scalar> material{};

                    bool operator==(const SoftKey& other) const noexcept
                    {
                        return asset_size == other.asset_size && level == other.level &&
                               origin.x == other.origin.x && origin.y == other.origin.y &&
                               origin.z == other.origin.z && thickness == other.thickness &&
                               self_collision == other.self_collision &&
                               cosmetic == other.cosmetic && rollback == other.rollback &&
                               material.young_modulus == other.material.young_modulus &&
                               material.poisson_ratio == other.material.poisson_ratio &&
                               material.density == other.material.density &&
                               material.damping == other.material.damping &&
                               material.yield_stress == other.material.yield_stress &&
                               material.plastic_creep == other.material.plastic_creep &&
                               material.maximum_plastic_strain ==
                                   other.material.maximum_plastic_strain &&
                               material.fracture_stress == other.material.fracture_stress;
                    }
                };

                /** @brief One tetrahedral soft body and what it was built from. */
                struct SoftEntry
                {
                    EntityId entity = NULL_ENTITY;
                    SoftKey key;
                    Physics::SoftBodyInstance instance;
                };

                /**
                 * @brief One hybrid vehicle, its controls, and what its drivetrain did.
                 *
                 * The instance is held behind a pointer because `VehicleInstanceT` deletes
                 * its copy constructor and therefore has no implicit move either — a value
                 * of it cannot live in a vector at all. That is the right property for the
                 * type (it owns solver handles; copying one would double-free a world) and
                 * the pointer is what lets a set of them be reconciled.
                 */
                struct VehicleEntry
                {
                    EntityId entity = NULL_ENTITY;
                    std::unique_ptr<Physics::VehicleInstanceT<T>> instance;
                    VehicleInput input;
                    VehicleReport report;

                    /**
                     * @brief The shell's surface triangles, copied out of the blob at create.
                     *
                     * Copied rather than borrowed because the view is not retained past the
                     * call that built the vehicle — the caller owns those bytes and is free
                     * to free them — and the render extract asks for these every frame.
                     */
                    std::vector<std::uint32_t> surface_indices;

                    /** @brief What the last `set_vehicles` built this from, to diff against. */
                    const std::byte* asset = nullptr;
                    std::size_t asset_size = 0;
                    Vector3 position;
                    Quaternion orientation;
                };

                /** @brief One cloth grid and the shape of it. */
                struct ClothEntry
                {
                    EntityId entity = NULL_ENTITY;
                    Physics::ClothGridHandles grid;
                    std::size_t rows = 0;
                    std::size_t cols = 0;
                    T spacing = T(0.5);
                    T thickness = T(0.1);
                };

                /**
                 * @brief One thing the contact pass can collide, and what it belongs to.
                 *
                 * A proxy's index in @ref contact_proxies_ is its payload, and that
                 * index is what a warm-start key is built from — so the list is
                 * rebuilt only when the *membership* changes, never merely because
                 * something moved. A list rebuilt every tick would renumber the
                 * proxies, the keys would not match, and every contact in the scene
                 * would silently start from zero impulse every frame.
                 */
                struct ContactProxy
                {
                    Physics::CollisionShape<T> shape;
                    /** @brief The body slot, or `null_contact_body` for static geometry. */
                    std::uint32_t slot = Physics::null_contact_body;
                    Physics::ProxyId id = Physics::null_proxy;
                    /** @brief Who to report this as; `NULL_ENTITY` for anything unowned. */
                    EntityId entity = NULL_ENTITY;
                    Physics::CollisionFilter filter{};
                    std::uint32_t flags = 0;
                    /** @brief Detected and reported, never resolved. */
                    bool trigger = false;

                    /**
                     * @brief How far this proxy can travel this tick, in metres (§7.5 tier 1).
                     *
                     * The speculative margin's half. Manifold generation is widened by the
                     * pair's two, so a body that will cross a surface *during* the tick still
                     * gets a manifold at the start of it — without one, the per-substep
                     * positional pass has nothing to catch the crossing with, and the body
                     * arrives on the far side.
                     *
                     * Zero for static geometry and for anything asleep, both of which are
                     * exactly right: neither is going anywhere.
                     */
                    T speculative_margin = 0;

                    /**
                     * @brief Whether this proxy's motion needs §7.5 tier 2 this tick.
                     *
                     * Derived once per proxy in `refresh_contact_index` rather than
                     * recomputed per pair — a fast body can appear in several pairs a
                     * tick, and the thinnest-dimension check is the same answer every
                     * time. False for static geometry and for anything asleep.
                     */
                    bool needs_conservative_advancement = false;
                };

                /**
                 * @brief One pair that touched, kept so the next tick can compare.
                 *
                 * Held in a vector sorted by @ref key rather than in a hash map, and
                 * the reason is the events: begin, persist and end are the three
                 * outcomes of a merge between last tick's list and this one's, and a
                 * merge needs an order. A hash map would give the same *set* and a
                 * different *sequence* on a different machine, which is exactly what
                 * §12.1 forbids for anything gameplay can observe — and a listener
                 * that spawns an effect observes it.
                 */
                struct ContactRecord
                {
                    std::uint64_t key = 0;
                    std::uint32_t a_slot = Physics::null_contact_body;
                    std::uint32_t b_slot = Physics::null_contact_body;
                    EntityId a_entity = NULL_ENTITY;
                    EntityId b_entity = NULL_ENTITY;
                    bool trigger = false;
                    Physics::ContactManifold<T> manifold;

                    bool operator<(const ContactRecord& other) const noexcept
                    {
                        return key < other.key;
                    }
                };

                static Vector3T<T> to_vector(const Vector3& v) noexcept
                {
                    return Vector3T<T>{T(v.x), T(v.y), T(v.z)};
                }
                static QuaternionT<T> to_quaternion(const Quaternion& q) noexcept
                {
                    return QuaternionT<T>{T(q.x), T(q.y), T(q.z), T(q.w)};
                }
                static Vector3 from_vector(const Vector3T<T>& v) noexcept
                {
                    return Vector3{Scalar(v.x), Scalar(v.y), Scalar(v.z)};
                }
                static Quaternion from_quaternion(const QuaternionT<T>& q) noexcept
                {
                    return Quaternion{Scalar(q.x), Scalar(q.y), Scalar(q.z), Scalar(q.w)};
                }

                /**
                 * @brief Contacts are generated out to here and resolved to @ref REST_OFFSET.
                 *
                 * The two differ for the reason §7.6 gives: a contact found only when
                 * the surfaces already touch is a contact the solver first sees after
                 * a tick's worth of approach has been spent, and the body arrives
                 * already interpenetrating. Generating further out costs a few
                 * manifolds that resolve to nothing.
                 */
                static constexpr T CONTACT_OFFSET = T(0.03);

                /** @brief The separation contacts come to rest at; zero is touching. */
                static constexpr T REST_OFFSET = T(0);

                /**
                 * @brief The fastest a contact may push two bodies apart, in metres a second.
                 *
                 * §7.6's depenetration budget. Without it, a body spawned a metre inside
                 * another has a metre of separation error and the positional pass projects
                 * all of it in one substep — the pair leaves at a speed nothing in the scene
                 * put there and takes whatever it hits with it. With it, the same overlap
                 * resolves over as many substeps as it needs and looks like being pushed out
                 * rather than fired out.
                 *
                 * Three metres a second is about walking pace: fast enough that a normal
                 * overlap of a few centimetres clears within a tick, slow enough that a gross
                 * one cannot become a projectile. It bounds only *recovery*; nothing here can
                 * let a body sink further than it otherwise would.
                 */
                static constexpr T MAX_DEPENETRATION_VELOCITY = T(3);

                /**
                 * @brief Creates the solver on first use, sized once and never resized.
                 *
                 * The capacities are a budget, not a guess that grows: a `Buffer`
                 * cannot grow in place, and the compiled graph captured raw pointers
                 * into every one of them. Exceeding one is counted in the statistics
                 * as a capacity overflow rather than silently reallocating something
                 * the device is reading.
                 *
                 * The colour ceiling is 16 rather than the configuration default of
                 * 32 because the solve graph holds a node per colour per substep per
                 * *kind*, and with contacts and joints that is now six kinds of node;
                 * 32 colours would double a compile that happens while a scene is
                 * loading. Sixteen clears a cloth grid's busiest vertex, which links
                 * to eight neighbours.
                 *
                 * The joint budget is small beside the constraint budget on purpose.
                 * Joints are authored one at a time — a mechanism, a ragdoll, a
                 * vehicle's suspension — while distance constraints are spent by the
                 * ten thousand on cloth and lattices, and a joint descriptor is several
                 * times the size of a distance constraint.
                 */
                void ensure_solver()
                {
                    if (solver_)
                        return;
                    Physics::PhysicsConfigurationT<T> configuration;
                    configuration.capacities.bodies = 16384;
                    configuration.capacities.constraints = 65536;
                    configuration.capacities.joints = 2048;
                    configuration.capacities.contacts = 16384;
                    configuration.capacities.colors = 16;
                    configuration.substeps.minimum = 4;
                    configuration.substeps.maximum = 16;
                    // The profiler panel's request, consumed here because profiling is a
                    // construction-time property of the solve graph (off = no timestamps
                    // on the hot path at all — configuration.hpp).
                    configuration.profiling = profiling_requested_;
                    solver_.reset(new Solver(context_, configuration));
                    bodies_.assign(configuration.capacities.bodies, Body{});
                    // The sleep thresholds are kept rather than re-derived: the island
                    // pass runs on the host, outside the solver, and a second copy of
                    // these two numbers is how the tick and the solver come to disagree
                    // about what "settled" means.
                    sleep_motion_threshold_ = configuration.sleep_motion_threshold;
                    sleep_delay_ = configuration.sleep_delay;
                    bodies_dirty_ = true;
                }

                /** @brief The boundary joint type, as the solver's kind. */
                static Physics::JointKind to_joint_kind(JointType type) noexcept
                {
                    // A cast rather than a switch, and the two enumerations are kept in
                    // the same order for it. A switch here would be a function every
                    // new joint kind has to edit, which is the §4.2 violation the
                    // projection dispatch exists to avoid; a boundary that reintroduced
                    // it would undo that one file further out.
                    return static_cast<Physics::JointKind>(static_cast<std::uint32_t>(type));
                }

                /** @brief The boundary limit, in the solver's precision. */
                static Physics::JointLimitT<T> to_joint_limit(const JointLimitDesc& limit) noexcept
                {
                    Physics::JointLimitT<T> out;
                    out.lower = T(limit.lower);
                    out.upper = T(limit.upper);
                    out.compliance = T(limit.compliance);
                    out.enabled = limit.enabled;
                    return out;
                }

                /** @brief The boundary drive, in the solver's precision. */
                static Physics::JointMotorT<T> to_joint_motor(const JointMotorDesc& motor) noexcept
                {
                    Physics::JointMotorT<T> out;
                    out.target = T(motor.target);
                    out.max_force = T(motor.max_force);
                    out.compliance = T(motor.compliance);
                    out.damping = T(motor.damping);
                    out.mode = static_cast<Physics::JointMotorMode>(
                        static_cast<std::uint32_t>(motor.type));
                    return out;
                }

                /** @brief The record for @p joint, or null. */
                const JointEntry* find_joint(JointId joint) const noexcept
                {
                    for (const JointEntry& entry : joints_)
                        if (entry.id == joint)
                            return &entry;
                    return nullptr;
                }

                /** @brief Drops joint records whose bodies are gone; the solver already has. */
                void prune_joints()
                {
                    for (std::size_t i = joints_.size(); i-- > 0;)
                    {
                        if (rigid_index_.count(joints_[i].a) != 0 &&
                            rigid_index_.count(joints_[i].b) != 0)
                            continue;
                        // No `remove_joint` here: `remove_body` already took every
                        // joint naming the freed slot with it, and asking the solver
                        // again would be asking it about a handle it has retired.
                        joints_.erase(joints_.begin() + std::ptrdiff_t(i));
                    }
                }

                /**
                 * @brief Destroys the joints whose load passed a break threshold.
                 *
                 * Ordered by joint identity because a listener that spawns an effect
                 * observes the sequence, and identities are handed out in creation
                 * order — so the sequence is a function of the scene rather than of
                 * whatever order the records happen to sit in (§12.1). The walk is
                 * downward so an erase does not skip the next candidate.
                 */
                void break_overloaded_joints()
                {
                    joint_events_.clear();
                    if (!solver_)
                        return;

                    for (std::size_t i = joints_.size(); i-- > 0;)
                    {
                        Physics::JointConstraintT<T> solved;
                        if (!solver_->read_joint(joints_[i].handle, solved))
                            continue;
                        if (!Physics::joint_should_break(solved))
                            continue;

                        JointBrokenEvent event;
                        event.joint = joints_[i].id;
                        event.a = joints_[i].a;
                        event.b = joints_[i].b;
                        event.force = Scalar(solved.peak_force);
                        event.torque = Scalar(solved.peak_torque);
                        joint_events_.push_back(event);

                        solver_->remove_joint(joints_[i].handle);
                        joints_.erase(joints_.begin() + std::ptrdiff_t(i));
                    }

                    std::sort(joint_events_.begin(), joint_events_.end(),
                              [](const JointBrokenEvent& lhs, const JointBrokenEvent& rhs)
                              { return lhs.joint < rhs.joint; });
                }

                /** @brief Rebuilds the entity-to-index map after the vector moved. */
                void reindex_rigid()
                {
                    rigid_index_.clear();
                    for (std::size_t i = 0; i < rigid_.size(); ++i)
                        rigid_index_.emplace(rigid_[i].entity, i);
                }

                /** @brief Applies a description to a body that already exists. */
                void update_rigid_entry(RigidEntry& entry, const RigidBodyDesc& desc)
                {
                    entry.collider = desc.collider;
                    entry.filter = desc.collider.filter;
                    entry.material = to_material(desc.material);
                    Body body;
                    if (!solver_->read_body(entry.handle, body))
                        return;
                    body.inv_mass = T(desc.inv_mass);
                    body.inv_inertia = to_vector(desc.inv_inertia);
                    body.drag_coefficient = T(desc.drag_coefficient);
                    solver_->write_body(entry.handle, body);
                    bodies_dirty_ = true;
                }

                /**
                 * @brief Whether @p vehicles is the set already built, in the same order.
                 *
                 * A vehicle is four hundred bodies placed relative to a cooked structure, so
                 * there is no patching one: the question this answers is only "may the
                 * rebuild be skipped entirely". Compared by asset bytes and root pose,
                 * because those are what instancing reads — a setup change *does* need the
                 * rebuild, and is caught by the caller reissuing the set.
                 *
                 * @param vehicles The requested set.
                 */
                bool vehicles_match(const std::vector<VehicleDesc>& vehicles) const noexcept
                {
                    if (vehicles.size() != vehicles_.size())
                        return false;
                    for (std::size_t i = 0; i < vehicles.size(); ++i)
                    {
                        const VehicleEntry& entry = vehicles_[i];
                        const VehicleDesc& desc = vehicles[i];
                        if (entry.entity != desc.id || entry.asset != desc.asset ||
                            entry.asset_size != desc.asset_size)
                            return false;
                        if (entry.position.x != desc.position.x ||
                            entry.position.y != desc.position.y ||
                            entry.position.z != desc.position.z)
                            return false;
                    }
                    return true;
                }

                /**
                 * @brief The authored vehicle setup at this solver's precision.
                 *
                 * A field-by-field conversion for the same reason `to_material` is one: `T`
                 * is not always the boundary's `Scalar`, and a `VehicleAssetT<double>` is
                 * not a `VehicleAssetT<float>` whatever their layouts happen to be.
                 *
                 * @param setup The authored vehicle.
                 */
                static const Physics::VehicleAssetT<T>& to_vehicle_setup(
                    const Physics::VehicleAssetT<Scalar>& setup) noexcept
                {
                    static_assert(std::is_same_v<T, Scalar>,
                                  "the vehicle setup crosses the boundary as a value rather "
                                  "than as a blob, so this pass-through is only correct while "
                                  "the solve runs at the boundary's own precision; a narrower "
                                  "solve needs a real field-by-field conversion here, and "
                                  "silently reinterpreting one would be a wrong car");
                    return setup;
                }

                /**
                 * @brief Spends every vehicle's input and runs its drivetrain, tyres and drag.
                 *
                 * The input is *spent*, not consumed: throttle is a state an input device
                 * holds down, so a caller that stops calling `set_vehicle_input` keeps the
                 * pedal where it left it, which is what a pedal does.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 */
                void step_vehicles_begin(T delta_time)
                {
                    for (VehicleEntry& entry : vehicles_)
                    {
                        if (!entry.instance)
                            continue;
                        Physics::VehicleInstanceT<T>& vehicle = *entry.instance;
                        vehicle.set_throttle(T(entry.input.throttle));
                        vehicle.set_clutch(T(entry.input.clutch));
                        vehicle.select_gear(entry.input.gear);
                        vehicle.set_steer_angle(*solver_, T(entry.input.steer));
                        vehicle.set_brake_torque(*solver_, T(entry.input.brake));

                        const Physics::PowertrainReportT<T> report =
                            vehicle.begin_tick(*solver_, delta_time);
                        entry.report.engine_rate = Scalar(report.engine_rate);
                        entry.report.engine_torque = Scalar(report.engine_torque);
                        entry.report.clutch_torque = Scalar(report.clutch_torque);
                        entry.report.clutch_slip = Scalar(report.clutch_slip);
                        entry.report.clutch_slipping = report.clutch_slipping;
                    }
                }

                /** @brief Runs every vehicle's tick boundary: plasticity, breakage, detachment. */
                void step_vehicles_end()
                {
                    for (VehicleEntry& entry : vehicles_)
                    {
                        if (!entry.instance)
                            continue;
                        const Physics::NodeBeamTickReport report =
                            entry.instance->end_tick(*solver_);
                        entry.report.beams_broken += report.beams_broken;
                        entry.report.parts_detached += report.parts_detached;
                    }
                }

                /**
                 * @brief The boundary's surface material at this solver's precision.
                 *
                 * A field-by-field conversion rather than a cast of the whole struct, because
                 * `T` is not always the boundary's `Scalar` — the cosmetic column runs at
                 * `float` (§6.5) — and a `PhysicsMaterialT<double>` is not a
                 * `PhysicsMaterialT<float>` whatever their layouts happen to be.
                 *
                 * @param material The authored surface.
                 */
                static Physics::PhysicsMaterialT<T> to_material(
                    const Physics::PhysicsMaterialT<Scalar>& material) noexcept
                {
                    Physics::PhysicsMaterialT<T> out;
                    out.static_friction = T(material.static_friction);
                    out.dynamic_friction = T(material.dynamic_friction);
                    out.restitution = T(material.restitution);
                    out.density = T(material.density);
                    out.rolling_friction = T(material.rolling_friction);
                    out.spinning_friction = T(material.spinning_friction);
                    out.friction_combine = material.friction_combine;
                    out.restitution_combine = material.restitution_combine;
                    return out;
                }

                /**
                 * @brief Everything about a `SoftBodyDesc` that a rebuild would change.
                 *
                 * Deliberately not the asset *pointer*: the caller owns those bytes and is
                 * free to move them between frames, so comparing addresses would rebuild
                 * every body every tick for a scene that did not change. The size stands in
                 * for the asset's identity, which is weaker than a hash and stronger than
                 * nothing — an edit that changed a cook without changing its byte count
                 * would be missed, and that is a cook-time change the editor already
                 * re-issues the whole set for.
                 */
                static SoftKey soft_key(const SoftBodyDesc& desc) noexcept
                {
                    SoftKey key;
                    key.asset_size = desc.asset_size;
                    key.level = desc.level;
                    key.origin = desc.origin;
                    key.thickness = desc.thickness;
                    key.self_collision = desc.self_collision;
                    key.cosmetic = desc.cosmetic;
                    key.rollback = desc.participates_in_rollback;
                    key.material = desc.material;
                    return key;
                }

                /** @brief Whether @p bodies describes exactly the soft bodies already built. */
                bool soft_matches(const std::vector<SoftBodyDesc>& bodies) const
                {
                    std::size_t wanted = 0;
                    for (const SoftBodyDesc& desc : bodies)
                        if (desc.asset != nullptr && desc.asset_size != 0)
                            ++wanted;
                    if (wanted != soft_.size())
                        return false;
                    for (const SoftBodyDesc& desc : bodies)
                    {
                        if (desc.asset == nullptr || desc.asset_size == 0)
                            continue;
                        const auto it = soft_index_.find(desc.id);
                        if (it == soft_index_.end())
                            return false;
                        if (!(soft_[it->second].key == soft_key(desc)))
                            return false;
                    }
                    return true;
                }

                /** @brief Which closed-form shape a rigid collider becomes for a soft-rigid contact (§9.6.1). */
                static Physics::SoftRigidPrimitiveKind to_primitive_kind(ColliderShape shape) noexcept
                {
                    switch (shape)
                    {
                        case ColliderShape::Sphere:
                            return Physics::SoftRigidPrimitiveKind::Sphere;
                        case ColliderShape::Capsule:
                            return Physics::SoftRigidPrimitiveKind::Capsule;
                        case ColliderShape::Plane:
                            return Physics::SoftRigidPrimitiveKind::Plane;
                        case ColliderShape::Box:
                        case ColliderShape::CookedAsset:
                            break;
                    }
                    // A cooked asset has no baked field reachable from this seam yet, so
                    // it takes the same bounding-box placeholder `collider_shape<T>`
                    // already uses for the rigid narrowphase — a stated approximation,
                    // not a silently invented one.
                    return Physics::SoftRigidPrimitiveKind::Box;
                }

                /**
                 * @brief Re-admits the gameplay-column bodies to the interleaved scene,
                 *        and rebuilds every soft-body collider alongside them (§9.6).
                 *
                 * Only the gameplay column joins `soft_scene_` or gets a collider, and
                 * the omission is the design rather than a gap. `SoftBodyScene<T>`
                 * interleaves substeps so that two bodies in contact see each other's
                 * mid-substep state — without which a stack of soft bodies cannot be
                 * correct however good the contact code is (§9.6). But it interleaves
                 * bodies of *one* width, and a cosmetic body is stored at another. A
                 * body that opted out of precision has, by §6.5's own definition,
                 * opted out of being something another body's correctness depends on,
                 * so it steps alone and free of collision, for now.
                 *
                 * Every non-trigger rigid body in the scene is a candidate soft-rigid
                 * partner; there is no broadphase pairing here yet; a soft body is
                 * already the expensive half of a tick (§16.21), and a handful of
                 * closed-form point tests per rigid body is not what that cost is
                 * dominated by. Two-way coupling is deliberately not attempted: pushing
                 * a rigid body from the host, mid-tick, would fight whatever pose the
                 * device solver is about to write back over it. A soft body rests on
                 * and slides against rigid geometry today; it does not yet move it.
                 */
                void rebuild_soft_scene()
                {
                    soft_scene_.clear();
                    soft_rigid_colliders_.clear();
                    soft_rigid_targets_.clear();
                    soft_self_colliders_.clear();
                    soft_soft_colliders_.clear();
                    soft_collider_sets_.assign(soft_.size(), Physics::SoftBodyColliderSet<T>{});

                    const T restitution_threshold = T(2) * T(9.81) * substep_dt_;

                    for (std::size_t i = 0; i < soft_.size(); ++i)
                    {
                        Physics::FiniteElementModel<T>* model = soft_[i].instance.gameplay_model();
                        if (model == nullptr)
                            continue;
                        soft_scene_.add_body(model);

                        if (soft_[i].key.self_collision)
                        {
                            soft_self_colliders_.push_back(Physics::SoftSelfCollider<T>{});
                            Physics::SoftSelfCollider<T>& self = soft_self_colliders_.back();
                            self.surface.surface_indices = model->surface_indices.data();
                            self.surface.index_count = model->surface_indices.size();
                            self.surface.collision = model->collision;
                            self.restitution_threshold = restitution_threshold;
                            soft_collider_sets_[i].add(&self);
                        }

                        for (std::size_t j = 0; j < rigid_.size(); ++j)
                        {
                            if (Physics::has_any_flag(rigid_[j].collider.flags,
                                                      Physics::BodyFlags::trigger))
                                continue;

                            soft_rigid_colliders_.push_back(Physics::SoftRigidPrimitiveCollider<T>{});
                            Physics::SoftRigidPrimitiveCollider<T>& partner =
                                soft_rigid_colliders_.back();
                            partner.kind = to_primitive_kind(rigid_[j].collider.shape);
                            partner.surface_vertices = model->surface_vertices.data();
                            partner.surface_vertex_count = model->surface_vertices.size();
                            partner.contact_offset = model->collision.thickness;
                            partner.params = Physics::make_contact_params(
                                model->collision.surface, Physics::PhysicsMaterialT<T>{},
                                model->collision.thickness, restitution_threshold);
                            // `configured` stays false — set by the first per-tick
                            // refresh, never by this shape's default placement.
                            soft_rigid_targets_.push_back(j);
                            soft_collider_sets_[i].add(&partner);
                        }

                        model->attach_collider(&soft_collider_sets_[i]);
                    }

                    // §9.6.2: every pair of gameplay-column bodies, unconditionally —
                    // unlike self-collision there is no opt-out in `SoftBodyCollisionSettings`,
                    // since two distinct bodies touching is the ordinary case a scene needs
                    // rather than the expensive extra self-collision is.
                    for (std::size_t i = 0; i < soft_.size(); ++i)
                    {
                        Physics::FiniteElementModel<T>* model_i = soft_[i].instance.gameplay_model();
                        if (model_i == nullptr)
                            continue;
                        for (std::size_t k = i + 1; k < soft_.size(); ++k)
                        {
                            Physics::FiniteElementModel<T>* model_k =
                                soft_[k].instance.gameplay_model();
                            if (model_k == nullptr)
                                continue;

                            soft_soft_colliders_.push_back(Physics::SoftSoftCollider<T>{});
                            Physics::SoftSoftCollider<T>& pair = soft_soft_colliders_.back();
                            pair.first = model_i->surface();
                            pair.second = model_k->surface();
                            pair.restitution_threshold = restitution_threshold;
                            pair.build();
                            soft_scene_.add_pair_collider(&pair);
                        }
                    }

                    refresh_soft_colliders(substep_dt_);
                }

                /**
                 * @brief Places every soft-rigid primitive collider at its rigid
                 *        partner's current pose, and refreshes every soft collider's
                 *        anti-jitter floor, once per tick.
                 *
                 * Reads `bodies_` as it stands at the top of the tick — last tick's
                 * solved poses, refreshed at the end of `step` (§6.1's "read once per
                 * tick" contract, the same one the SDF collider's own docs name). A
                 * rigid slot not yet in `bodies_` (the very first tick a body exists)
                 * is left `configured == false` rather than read at its default pose,
                 * so nothing collides against the world origin for one stray tick.
                 *
                 * @param substep This tick's actual substep duration — the quantity
                 *                both the restitution floor and the depenetration
                 *                budget (§7.6, §16.19) are derived from, freshly
                 *                each tick rather than from the configured target.
                 */
                void refresh_soft_colliders(T substep)
                {
                    const T restitution_threshold = T(2) * T(9.81) * substep;
                    const T max_depenetration = MAX_DEPENETRATION_VELOCITY * substep;

                    for (std::size_t n = 0; n < soft_rigid_colliders_.size(); ++n)
                    {
                        const std::size_t j = soft_rigid_targets_[n];
                        Physics::SoftRigidPrimitiveCollider<T>& shape = soft_rigid_colliders_[n];
                        shape.params.restitution_threshold = restitution_threshold;
                        shape.params.max_depenetration = max_depenetration;
                        if (!solver_ || j >= rigid_.size())
                            continue;

                        const std::size_t slot = solver_->body_slot(rigid_[j].handle);
                        if (slot >= bodies_.size())
                            continue;

                        const Body& body = bodies_[slot];
                        const Collider& collider = rigid_[j].collider;
                        const Vector3T<T> offset =
                            rotate(body.orientation, to_vector(collider.local_offset));

                        switch (shape.kind)
                        {
                            case Physics::SoftRigidPrimitiveKind::Sphere:
                                shape.sphere.center = body.position + offset;
                                shape.sphere.radius = T(collider.radius);
                                break;
                            case Physics::SoftRigidPrimitiveKind::Capsule:
                                shape.capsule.center = body.position + offset;
                                shape.capsule.orientation = body.orientation;
                                shape.capsule.half_height = T(collider.half_height);
                                shape.capsule.radius = T(collider.radius);
                                break;
                            case Physics::SoftRigidPrimitiveKind::Plane:
                                // A plane is authored directly in world space (§8.6) and
                                // never reads the body's pose — see
                                // `soft_rigid_shape_collision.hpp`'s file header.
                                shape.plane.normal = normalize(to_vector(collider.half_extents));
                                shape.plane.offset = T(collider.radius);
                                break;
                            case Physics::SoftRigidPrimitiveKind::Box:
                                shape.box.center = body.position + offset;
                                shape.box.orientation = body.orientation;
                                shape.box.half_extents = to_vector(collider.half_extents);
                                break;
                        }
                        shape.configured = true;
                    }

                    // The self- and soft-soft colliders carry the same anti-jitter
                    // floor; refreshed alongside the rigid partners' poses so all
                    // three read this tick's actual substep length rather than the
                    // configured target `rebuild_soft_scene` seeded them with.
                    for (Physics::SoftSelfCollider<T>& self : soft_self_colliders_)
                        self.restitution_threshold = restitution_threshold;
                    for (Physics::SoftSoftCollider<T>& pair : soft_soft_colliders_)
                        pair.restitution_threshold = restitution_threshold;
                }

                /** @brief Advances every soft body by one tick, gameplay column together. */
                void step_soft_bodies(const GravitySampler& gravity, T delta_time,
                                      std::size_t substeps)
                {
                    if (!soft_.empty())
                        refresh_soft_colliders(delta_time / T(substeps > 0 ? substeps : 1));

                    for (SoftEntry& entry : soft_)
                    {
                        // Sampled at the body's own position, like a rigid body's, so a
                        // soft body on a planet feels the same field the crate beside it
                        // does rather than a scene-wide constant.
                        const Vector3 origin = entry.instance.particle_count() > 0
                                                   ? entry.instance.particle_position(0)
                                                   : Vector3{0, 0, 0};
                        entry.instance.set_external_acceleration(gravity(origin));
                    }

                    if (soft_scene_.body_count() != 0)
                        soft_scene_.step(delta_time, substeps);
                    for (SoftEntry& entry : soft_)
                        if (entry.instance.gameplay_model() == nullptr)
                            entry.instance.step(Scalar(delta_time), substeps);

                    // §9.5, after every body has stepped: fracture reads the stress
                    // `end_tick` just measured. `rebuild_soft_body_surface` (called
                    // inside a fracture pass that actually removed something) replaces
                    // `surface_indices`/`surface_vertices`, which is exactly what every
                    // §9.6 collider above holds a raw pointer into — so a real fracture
                    // this tick invalidates those pointers, and the only safe answer is
                    // the same rebuild a membership change already triggers, not a
                    // narrower patch-up that has to be right about which pointers moved.
                    bool topology_changed = false;
                    for (SoftEntry& entry : soft_)
                        topology_changed |= entry.instance.step_fracture();
                    if (topology_changed)
                        rebuild_soft_scene();
                }

                /** @brief Whether @p grids describes exactly the cloths already built. */
                bool cloth_matches(const std::vector<ClothDesc>& grids) const
                {
                    std::size_t wanted = 0;
                    for (const ClothDesc& desc : grids)
                        if (desc.rows != 0 && desc.cols != 0)
                            ++wanted;
                    if (wanted != cloth_.size())
                        return false;
                    for (const ClothDesc& desc : grids)
                    {
                        if (desc.rows == 0 || desc.cols == 0)
                            continue;
                        const auto it = cloth_index_.find(desc.id);
                        if (it == cloth_index_.end())
                            return false;
                        const ClothEntry& entry = cloth_[it->second];
                        if (entry.rows != desc.rows || entry.cols != desc.cols)
                            return false;
                    }
                    return true;
                }

                /** @brief Records that the set of collidable things changed. */
                void note_membership_changed()
                {
                    proxies_dirty_ = true;
                    query_dirty_ = true;
                    bodies_dirty_ = true;
                    // The warm-start keys are built from proxy indices, and those are
                    // about to be renumbered. Impulses inherited across a renumbering
                    // would be attached to the wrong pairs, which is worse than
                    // inheriting nothing for one tick. The contact events go with
                    // them: an `End` derived from a stale numbering would name a pair
                    // that never touched.
                    current_.clear();
                    previous_.clear();

                    // §9.6's soft-rigid pairing depends on `rigid_` as much as on
                    // `soft_`, and this is the one hook every setter that can change
                    // `rigid_` (bodies, cloth) already calls — `set_soft_bodies` calls
                    // `rebuild_soft_scene` directly instead of through here, so this is
                    // not a double rebuild on that path, only coverage for the paths
                    // that used to leave the pairing silently stale.
                    rebuild_soft_scene();
                }

                /** @brief Brings the solver's bodies to the host mirror, if they moved. */
                void refresh_bodies() const
                {
                    if (!solver_ || !bodies_dirty_)
                        return;
                    bodies_dirty_ = false;
                    if (bodies_.size() < solver_->body_capacity())
                        bodies_.assign(solver_->body_capacity(), Body{});
                    const std::size_t count = live_slot_count();
                    if (count > 0)
                        solver_->read_bodies(0, count, bodies_.data());
                }

                /** @brief One past the highest body slot this scene has ever used. */
                std::size_t live_slot_count() const
                {
                    std::size_t high = 0;
                    for (const RigidEntry& entry : rigid_)
                        high = std::max(high, solver_->body_slot(entry.handle) + 1);
                    for (const ClothEntry& entry : cloth_)
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                            high = std::max(high, solver_->body_slot(handle) + 1);
                    return std::min(high, bodies_.size());
                }

                /**
                 * @brief Samples both fields per body and folds them into each one.
                 *
                 * Per body, once per tick. The alternative — the uniform vector
                 * `StepParameters` carries — cannot express a planetary field, and
                 * sampling inside the substep loop is not available at all now that
                 * `predict` runs on the device (§6.6).
                 *
                 * Gravity and wind land in the same field and in the same pass because
                 * they are the same kind of thing: a per-body acceleration the live world
                 * knows and the solver must not (§4.5). Wind arrives as a *difference*
                 * against the still-air drag `predict` will apply anyway, so a scene with
                 * no wind installed is bit-for-bit the scene that had no wind seam at all
                 * — see `physics/aero/wind.hpp`.
                 */
                void apply_gravity_field(const GravitySampler& gravity, const WindSampler& wind)
                {
                    const std::size_t count = live_slot_count();
                    for (std::size_t slot = 0; slot < count; ++slot)
                    {
                        const Vector3 position = from_vector(bodies_[slot].position);
                        Vector3T<T> sampled = to_vector(gravity(position));
                        if (wind)
                        {
                            sampled = sampled + Physics::wind_drag_acceleration(
                                                    bodies_[slot], to_vector(wind(position)));
                        }
                        bodies_[slot].external_acceleration = sampled;
                    }
                    write_every_body();
                    bodies_dirty_ = false;
                }

                /** @brief Sends the host mirror's copy of one body back to the solver. */
                void write_field(Physics::BodyHandle handle)
                {
                    const std::size_t slot = solver_->body_slot(handle);
                    if (slot < bodies_.size())
                        solver_->write_body(handle, bodies_[slot]);
                }

                /**
                 * @brief Wakes a body and everything its island is resting against.
                 *
                 * Called by every entry point that disturbs a body from outside the
                 * tick. The asymmetry is deliberate and is what makes sleeping safe:
                 * falling asleep is a decision a whole island has to earn, and waking is
                 * immediate and needs no decision at all. A crate teleported into a
                 * settled stack must not leave that stack hanging in the air until
                 * something re-derives the partition.
                 *
                 * The whole scene is written back rather than the island's members
                 * alone: a wake is a rare, caller-driven event, and picking out the
                 * members would mean trusting an island index that the wake itself is
                 * evidence has gone stale.
                 *
                 * @param handle The body to wake.
                 */
                void wake_body(Physics::BodyHandle handle)
                {
                    refresh_bodies();
                    const std::size_t slot = solver_->body_slot(handle);
                    if (slot >= bodies_.size())
                        return;
                    Physics::wake_island(bodies_.data(), live_slot_count(),
                                         std::uint32_t(slot), islands_);
                    write_every_body();
                }

                /**
                 * @brief How many of this scene's body slots are awake.
                 *
                 * Counted from the host mirror rather than from the partition's
                 * `awake_count`, which counts *islands*. A static body is in no island
                 * and is neither awake nor asleep in the sense this reports — it is not
                 * simulated at all — so it is excluded from both halves.
                 */
                std::size_t awake_body_count() const
                {
                    const std::size_t count = live_slot_count();
                    std::size_t awake = 0;
                    for (std::size_t slot = 0; slot < count; ++slot)
                        if (Physics::is_simulated(bodies_[slot].flags))
                            ++awake;
                    return awake;
                }

                /** @brief How many of this scene's body slots are asleep. */
                std::size_t sleeping_body_count() const
                {
                    const std::size_t count = live_slot_count();
                    std::size_t sleeping = 0;
                    for (std::size_t slot = 0; slot < count; ++slot)
                        if (Physics::has_any_flag(bodies_[slot].flags,
                                                  Physics::BodyFlags::sleeping))
                            ++sleeping;
                    return sleeping;
                }

                /** @brief Sends every body this scene owns back to the solver. */
                void write_every_body()
                {
                    // Through the handles rather than by slot, because a slot this scene
                    // does not own is not this scene's to write.
                    for (const RigidEntry& entry : rigid_)
                        write_field(entry.handle);
                    for (const ClothEntry& entry : cloth_)
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                            write_field(handle);
                }

                /**
                 * @brief Partitions the scene into islands and decides which of them sleep.
                 *
                 * Run at the end of the tick rather than the start, and that is the
                 * whole design: the eligibility test reads a body's smoothed motion,
                 * which is a statement about the tick that just happened. Deciding
                 * before the solve would put a body to sleep on last tick's evidence
                 * and then solve it anyway.
                 *
                 * The edges are the things that actually transmit a disturbance, and
                 * every one of them is something this scene owns rather than something
                 * the solver is asked for: this tick's resolved contacts, the joints,
                 * and each cloth lattice. A trigger is not an edge — it is detected and
                 * never resolved, so it moves nothing and must not keep a body awake or
                 * drag two islands together.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 */
                void update_islands(T delta_time)
                {
                    const std::size_t count = live_slot_count();
                    if (count == 0)
                    {
                        islands_ = Physics::IslandSet{};
                        return;
                    }

                    // The smoothed measure the sleep test reads. Host-side because the
                    // island pass is, and because a body's own history is what is being
                    // smoothed — the device's motion column is this tick's speed alone,
                    // which is what the substep schedule wants and not what sleeping does.
                    for (std::size_t slot = 0; slot < count; ++slot)
                        Physics::update_motion_measure(bodies_[slot], delta_time);

                    island_builder_.begin(count);
                    for (const ContactRecord& record : current_)
                    {
                        if (record.trigger)
                            continue;
                        if (record.b_slot == Physics::null_contact_body)
                            continue; // static geometry conducts nothing (§16.4)
                        island_builder_.connect(std::uint32_t(record.a_slot),
                                                std::uint32_t(record.b_slot),
                                                bodies_.data());
                    }
                    for (const JointEntry& joint : joints_)
                    {
                        Physics::JointConstraintT<T> descriptor;
                        if (!solver_->read_joint(joint.handle, descriptor))
                            continue;
                        island_builder_.connect(descriptor.a, descriptor.b, bodies_.data());
                    }
                    for (const ClothEntry& entry : cloth_)
                    {
                        // Every particle joined to the first rather than to its lattice
                        // neighbours: a lattice is connected by construction, so the
                        // component this produces is the same one the edges would, at a
                        // pass instead of four per particle.
                        if (entry.grid.bodies.empty())
                            continue;
                        const std::uint32_t first =
                            std::uint32_t(solver_->body_slot(entry.grid.bodies.front()));
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                            island_builder_.connect(
                                first, std::uint32_t(solver_->body_slot(handle)),
                                bodies_.data());
                    }

                    island_builder_.finish(bodies_.data(), count, delta_time,
                                           sleep_motion_threshold_, sleep_delay_,
                                           islands_);
                    // The decision only exists once the solver has it: `finish` wrote it
                    // onto the host mirror's flags, timers and island indices, and the
                    // solve that must skip a sleeping body runs from the device's copy.
                    write_every_body();
                }

                /**
                 * @brief Rebuilds or refreshes the contact broadphase from the poses.
                 *
                 * Rebuilt only when the membership changed; otherwise every proxy's
                 * shape is recomputed and its bounds updated in place, which is what
                 * the hierarchy was built incremental for. The displacement handed to
                 * `update_proxy` is a tick's travel, so a pair that will only start
                 * overlapping partway through the tick is already a candidate.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 */
                void refresh_contact_index(T delta_time)
                {
                    if (proxies_dirty_)
                        rebuild_contact_index();

                    for (ContactProxy& proxy : contact_proxies_)
                    {
                        if (proxy.slot == Physics::null_contact_body)
                            continue;
                        // A sleeping body has not moved, so its proxy is already right
                        // and refreshing it costs a hierarchy descent to learn nothing.
                        // This is where §16.4's ten-thousand-body measurement came from
                        // — the island decision reaching the broadphase — and it is why
                        // the proxy is left in the tree rather than removed: something
                        // awake that comes near it still has to find it.
                        if (Physics::has_any_flag(bodies_[proxy.slot].flags,
                                                  Physics::BodyFlags::sleeping))
                        {
                            // Cleared rather than left: a body that was moving fast and then
                            // settled would otherwise keep last tick's margin for as long as
                            // it slept, widening every manifold around a stack that is not
                            // going anywhere.
                            proxy.speculative_margin = 0;
                            proxy.needs_conservative_advancement = false;
                            continue;
                        }
                        proxy.shape = shape_for_slot(proxy.slot);
                        const Vector3T<T> travel =
                            bodies_[proxy.slot].velocity * delta_time;
                        // The same displacement the broadphase sweeps its bounds by, which is
                        // the point: the narrowphase must look as far as the broadphase did,
                        // or the pair is found and then discarded for being far apart.
                        proxy.speculative_margin = length(travel);
                        // §7.5 tier 2's own trigger: state-derived, so the decision does
                        // not depend on anything but this tick's velocities and shape —
                        // widened by the author's own opt-in, for a body (a bullet) whose
                        // *shape* is not thin but whose consequence of tunnelling is severe
                        // enough that the motion-derived fraction should not be the only say.
                        proxy.needs_conservative_advancement =
                            Physics::has_any_flag(bodies_[proxy.slot].flags,
                                                  Physics::BodyFlags::continuous_collision) ||
                            Physics::needs_conservative_advancement<T>(
                                proxy.shape, bodies_[proxy.slot].velocity,
                                bodies_[proxy.slot].angular_velocity, delta_time);
                        // The broadphase must be told, not just the narrowphase. Its own
                        // enlargement is a fixed multiple of one tick's travel, refreshed
                        // only once the body has actually left its last stored box (§7.1) —
                        // which for an ordinary body lags a tick or two behind, and a tick
                        // or two of travel is exactly what a thin, fast-moving pair cannot
                        // spare: `submit_contacts` only ever looks at pairs the broadphase
                        // already proposed, so a pair the broadphase never proposes is a
                        // pair neither tier of §7.5 gets a chance to fix, however wide the
                        // narrowphase's own speculative margin is. Recomputed every tick
                        // from `proxy.flags` (the author's own opt-in, if any) rather than
                        // accumulated onto whatever the broadphase currently holds, so a
                        // body that slows back down stops paying for a swept box the tick
                        // after it no longer needs one.
                        const std::uint32_t broadphase_flags =
                            proxy.needs_conservative_advancement
                                ? (proxy.flags | Physics::BodyFlags::continuous_collision)
                                : proxy.flags;
                        contact_index_.set_proxy_state(proxy.id, proxy.filter, broadphase_flags);
                        contact_index_.update_proxy(proxy.id, Physics::shape_world_bounds(proxy.shape),
                                                    travel);
                    }
                    contact_index_.update();
                }

                /** @brief Recreates every contact proxy, renumbering them from zero. */
                void rebuild_contact_index()
                {
                    proxies_dirty_ = false;
                    contact_index_ = Physics::BvhBroadphase<T>{};
                    contact_proxies_.clear();

                    // What each slot collides as, resolved before any proxy is built,
                    // because building one asks the question.
                    collider_of_slot_.clear();
                    material_of_slot_.clear();
                    cloth_radius_.assign(bodies_.size(), T(0));
                    for (const RigidEntry& entry : rigid_)
                    {
                        const std::size_t slot = solver_->body_slot(entry.handle);
                        if (slot < bodies_.size())
                        {
                            collider_of_slot_.emplace(std::uint32_t(slot), entry.collider);
                            material_of_slot_.emplace(std::uint32_t(slot), entry.material);
                        }
                    }
                    for (const ClothEntry& entry : cloth_)
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                        {
                            const std::size_t slot = solver_->body_slot(handle);
                            if (slot < cloth_radius_.size())
                                cloth_radius_[slot] = entry.thickness;
                        }

                    for (const RigidEntry& entry : rigid_)
                    {
                        const std::size_t slot = solver_->body_slot(entry.handle);
                        if (slot >= bodies_.size())
                            continue;
                        ContactProxy proxy;
                        proxy.slot = std::uint32_t(slot);
                        proxy.shape = shape_for_slot(proxy.slot);
                        proxy.filter = entry.filter;
                        proxy.entity = entry.entity;
                        proxy.trigger = (entry.collider.flags &
                                         Physics::BodyFlags::trigger) != 0u;
                        proxy.flags = bodies_[slot].inv_mass > T(0)
                                          ? 0u
                                          : Physics::BodyFlags::static_body;
                        if (proxy.trigger)
                            proxy.flags |= Physics::BodyFlags::trigger;
                        add_contact_proxy(proxy);
                    }
                    for (const ClothEntry& entry : cloth_)
                    {
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                        {
                            const std::size_t slot = solver_->body_slot(handle);
                            if (slot >= bodies_.size())
                                continue;
                            ContactProxy proxy;
                            proxy.slot = std::uint32_t(slot);
                            proxy.shape = shape_for_slot(proxy.slot);
                            // Cloth has no self-collision yet, and that is said by its
                            // filter rather than by a flag every routine has to know.
                            proxy.filter = Physics::self_excluding_filter(
                                Physics::CollisionLayers::cloth);
                            proxy.entity = entry.entity;
                            proxy.flags = bodies_[slot].inv_mass > T(0)
                                              ? 0u
                                              : Physics::BodyFlags::static_body;
                            add_contact_proxy(proxy);
                        }
                    }
                    for (const Physics::PlaneCollider<T>& plane : planes_)
                    {
                        ContactProxy proxy;
                        proxy.slot = Physics::null_contact_body;
                        proxy.shape = Physics::make_plane_shape<T>(plane.normal, plane.offset);
                        proxy.flags = Physics::BodyFlags::static_body;
                        add_contact_proxy(proxy);
                    }
                }

                /** @brief Registers one proxy, its index becoming its payload. */
                void add_contact_proxy(ContactProxy& proxy)
                {
                    const std::uint32_t payload = std::uint32_t(contact_proxies_.size());
                    proxy.id = contact_index_.create_proxy(
                        Physics::shape_world_bounds(proxy.shape), proxy.filter, proxy.flags,
                        payload);
                    contact_proxies_.push_back(proxy);
                }

                /** @brief The collision shape body slot @p slot presents this tick. */
                Physics::CollisionShape<T> shape_for_slot(std::uint32_t slot) const
                {
                    if (slot < cloth_radius_.size() && cloth_radius_[slot] > T(0))
                        return Physics::make_sphere_shape<T>(bodies_[slot].position,
                                                             cloth_radius_[slot]);
                    const auto it = collider_of_slot_.find(slot);
                    if (it == collider_of_slot_.end())
                        return Physics::make_sphere_shape<T>(bodies_[slot].position, T(0.5));
                    return collider_shape<T>(it->second, bodies_[slot].position,
                                             bodies_[slot].orientation);
                }

                /**
                 * @brief The surface @p slot contacts as, or the default for anything else.
                 *
                 * The default rather than a refusal, because the slots without an entry are
                 * cloth particles, soft-body vertices and the standing plane body — none of
                 * which is a rigid body an author gave a material to, and all of which have
                 * to contact *something*. `PhysicsMaterialT`'s own defaults describe an
                 * ordinary solid, so an unauthored surface behaves plausibly rather than
                 * like frictionless glass.
                 *
                 * @param slot The body slot to resolve.
                 */
                const Physics::PhysicsMaterialT<T>& material_of(std::uint32_t slot) const noexcept
                {
                    static const Physics::PhysicsMaterialT<T> DEFAULT_MATERIAL{};
                    const auto it = material_of_slot_.find(slot);
                    return it == material_of_slot_.end() ? DEFAULT_MATERIAL : it->second;
                }

                /**
                 * @brief Generates this tick's manifolds and submits them as contacts.
                 *
                 * @param delta_time The tick's duration, in seconds.
                 * @param floor      The substep floor, for the restitution threshold.
                 */
                void submit_contacts(T delta_time, std::size_t floor)
                {
                    solver_->begin_contacts();
                    previous_.swap(current_);
                    current_.clear();

                    const T substep = delta_time / T(floor > 0 ? floor : 1);
                    // The two numbers that are properties of the *step* rather than of the
                    // surfaces, resolved once here and folded into every pair's params below.
                    //
                    // A resting body's contacts carry a closing speed of about `g * h` every
                    // substep purely because gravity had a substep to act; returning that is
                    // how a settled stack buzzes for ever.
                    const T restitution_threshold = T(2) * T(9.81) * substep;
                    // A velocity in the contract, a distance in the projection: the
                    // multiplication happens once, here, where the substep length is already
                    // known, so the projection stays a pure function of its parameters.
                    const T max_depenetration = MAX_DEPENETRATION_VELOCITY * substep;

                    for (const Physics::BroadphasePair& pair : contact_index_.pairs())
                    {
                        const std::uint32_t first =
                            contact_index_.proxy(pair.a).payload;
                        const std::uint32_t second =
                            contact_index_.proxy(pair.b).payload;
                        if (first >= contact_proxies_.size() ||
                            second >= contact_proxies_.size())
                            continue;

                        // The manifold's normal points from `a` to `b` and `a` must be
                        // a body, so static geometry is always the second of the two.
                        const bool flip =
                            contact_proxies_[first].slot == Physics::null_contact_body;
                        const ContactProxy& lhs =
                            flip ? contact_proxies_[second] : contact_proxies_[first];
                        const ContactProxy& rhs =
                            flip ? contact_proxies_[first] : contact_proxies_[second];
                        if (lhs.slot == Physics::null_contact_body)
                            continue; // two pieces of static geometry; nothing to move

                        ContactRecord record;
                        record.key = pair_key(flip ? second : first, flip ? first : second);
                        record.a_slot = lhs.slot;
                        record.b_slot = rhs.slot;
                        record.a_entity = lhs.entity;
                        record.b_entity = rhs.entity;
                        record.trigger = lhs.trigger || rhs.trigger;
                        // §7.5 tier 1. Generated out to the contact offset *plus how far the
                        // pair can close this tick*, so a body that will cross the surface
                        // mid-tick has a constraint waiting for it. The positional pass then
                        // catches the crossing on whichever substep the predict makes the
                        // separation negative — which is why the vast majority of fast motion
                        // needs no sweep at all, and why this costs a few inert manifolds
                        // rather than a closest-point query per body.
                        //
                        // The pair's summed travel and not its closing speed along the normal:
                        // the normal is what generation is about to compute, so the magnitude
                        // is the only bound available before it exists. It over-generates for
                        // a pair moving *apart*, which costs a manifold that resolves to
                        // nothing — the safe direction.
                        const T speculative =
                            CONTACT_OFFSET + lhs.speculative_margin + rhs.speculative_margin;
                        record.manifold = Physics::generate_shape_manifold<T>(
                            lhs.shape, rhs.shape, speculative);

                        // §7.5 tier 2. Tier 1 above already covers the overwhelming
                        // majority of fast motion; this runs whenever at least one
                        // side's motion this tick is large enough, relative to its own
                        // thinnest dimension, that the start-pose manifold cannot be
                        // trusted — and deliberately *not* gated on tier 1 having found
                        // nothing. It can find something and still be wrong: tier 1
                        // samples the pair's geometry only at the tick's start pose, so
                        // a pair whose relative motion this tick is many multiples of
                        // the thinner shape's own size can have *already crossed* by
                        // that sample, and a deep-penetration manifold generated from
                        // the wrong side of the crossing resolves the body out through
                        // the nearest face of *that* configuration — which is the far
                        // face, not the one it approached through. Conservative
                        // advancement finds the exact time of impact by construction,
                        // so when it reports one it is trusted over tier 1's answer
                        // rather than only filling in for it.
                        if (lhs.needs_conservative_advancement || rhs.needs_conservative_advancement)
                        {
                            const Vector3T<T> lhs_velocity = bodies_[lhs.slot].velocity;
                            const Vector3T<T> lhs_angular = bodies_[lhs.slot].angular_velocity;
                            const Vector3T<T> rhs_velocity =
                                rhs.slot == Physics::null_contact_body
                                    ? Vector3T<T>{}
                                    : bodies_[rhs.slot].velocity;
                            const Vector3T<T> rhs_angular =
                                rhs.slot == Physics::null_contact_body
                                    ? Vector3T<T>{}
                                    : bodies_[rhs.slot].angular_velocity;
                            const auto advancement = Physics::conservative_advance<T>(
                                lhs.shape, lhs_velocity, lhs_angular, rhs.shape, rhs_velocity,
                                rhs_angular, CONTACT_OFFSET, delta_time);
                            if (advancement.impact)
                            {
                                const Physics::CollisionShape<T> moved_lhs =
                                    Physics::advance_collision_shape<T>(
                                        lhs.shape, lhs_velocity, lhs_angular,
                                        advancement.time_of_impact);
                                const Physics::CollisionShape<T> moved_rhs =
                                    Physics::advance_collision_shape<T>(
                                        rhs.shape, rhs_velocity, rhs_angular,
                                        advancement.time_of_impact);
                                record.manifold = Physics::make_point_manifold<T>(
                                    advancement.contact.normal, advancement.contact.point_a,
                                    advancement.contact.point_b, advancement.contact.separation,
                                    moved_lhs.center, moved_lhs.orientation, moved_rhs.center,
                                    moved_rhs.orientation, Physics::make_feature_id(0, 0, 0, false));
                                // The anchors are material points, valid at any pose; what
                                // they mean *right now* — the separation the touching test
                                // and the first substep actually see — is what the tick's
                                // start pose says, not the advanced one they were sampled at.
                                Physics::refresh_manifold(record.manifold, lhs.shape.center,
                                                          lhs.shape.orientation, rhs.shape.center,
                                                          rhs.shape.orientation);
                            }
                        }

                        if (record.manifold.point_count == 0)
                            continue;

                        // Last tick's impulses, matched point by point on the feature
                        // ids the clipper stamped. Without this a box on a ramp has no
                        // friction cone until one builds, and creeps for the substep
                        // it takes to build it.
                        const ContactRecord* was = find_previous(record.key);
                        if (was != nullptr)
                            Physics::warm_start_manifold(record.manifold, was->manifold);

                        // A speculative manifold is a *constraint*, not a *touching pair*, and
                        // conflating the two would make a body report a contact — with an
                        // audio impact and a VFX burst behind it — while it is still metres
                        // away and merely heading this way. So the touching test stays what it
                        // was before the margin widened: within the contact offset.
                        //
                        // Only the deepest point is asked, because that is the point the event
                        // reports at (§16.6) and a manifold whose nearest corner is inside the
                        // offset is touching however far its others are.
                        T nearest = record.manifold.points[0].separation;
                        for (std::size_t p = 1; p < record.manifold.point_count; ++p)
                            nearest = std::min(nearest, record.manifold.points[p].separation);
                        const bool touching = nearest <= CONTACT_OFFSET;

                        if (touching)
                            current_.push_back(record);
                        if (record.trigger)
                            continue; // reported, never resolved

                        // A pair with nothing simulated on either side is still a
                        // *touching* pair and stays in the list above, because a settled
                        // stack that fell asleep has not stopped touching itself — if it
                        // left the list, every contact in it would report End and then
                        // Begin again on waking, and §16.6's "begins once, then persists"
                        // would be false for every stack that ever settles. What it does
                        // not need is to be solved: both projections early-out on a body
                        // that is not simulated, so submitting it would spend a contact
                        // slot and a colour band to compute nothing.
                        if (!Physics::is_simulated(bodies_[lhs.slot].flags) &&
                            (rhs.slot == Physics::null_contact_body ||
                             !Physics::is_simulated(bodies_[rhs.slot].flags)))
                            continue;

                        Contact contact;
                        contact.a = lhs.slot;
                        contact.b = rhs.slot;
                        contact.key = record.key;
                        // §5.3's materials, combined per pair rather than one constant for
                        // the whole scene. Until now every contact in the world solved at
                        // 0.6/0.5/0 whatever its surfaces said, which made `PhysicsMaterial`
                        // a type nothing read. A side with no rigid body — the standing
                        // plane, a cloth particle — resolves to the default surface, which
                        // is deliberately a fixed ground rather than a mirror of whatever is
                        // standing on it: an ice cube should be slippery *against the floor*,
                        // and a floor that copied the cube's friction would cancel exactly
                        // the difference the author authored.
                        contact.params = Physics::make_contact_params(
                            material_of(lhs.slot), material_of(rhs.slot), REST_OFFSET,
                            restitution_threshold);
                        contact.params.max_depenetration = max_depenetration;
                        contact.manifold = record.manifold;
                        solver_->add_contact(contact);
                    }

                    // The broadphase emits its pairs in proxy order, which is a
                    // function of its insertion history rather than of the scene. The
                    // event merge below needs an order that is a function of the
                    // scene alone, so it is imposed here and not inherited.
                    std::sort(current_.begin(), current_.end());
                }

                /** @brief Last tick's record for @p key, or null; @ref previous_ is sorted. */
                const ContactRecord* find_previous(std::uint64_t key) const
                {
                    ContactRecord probe;
                    probe.key = key;
                    const auto it =
                        std::lower_bound(previous_.begin(), previous_.end(), probe);
                    if (it == previous_.end() || it->key != key)
                        return nullptr;
                    return &*it;
                }

                /** @brief Takes the solved manifolds back, for the next tick to inherit. */
                void collect_contacts()
                {
                    const std::size_t count = solver_->contact_count();
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        Contact contact;
                        if (!solver_->read_contact(i, contact))
                            continue;
                        ContactRecord probe;
                        probe.key = contact.key;
                        const auto it =
                            std::lower_bound(current_.begin(), current_.end(), probe);
                        if (it != current_.end() && it->key == contact.key)
                            it->manifold = contact.manifold;
                    }
                }

                /**
                 * @brief Diffs last tick's touching pairs against this tick's.
                 *
                 * A linear merge of two sorted lists, which is the same shape the
                 * broadphase's pair cache uses for the same reason: the three
                 * outcomes — a pair only in the new list, in both, or only in the old
                 * — *are* begin, persist and end, and deriving them any other way
                 * means deciding twice what "still touching" means.
                 */
                void build_contact_events()
                {
                    events_.clear();
                    std::size_t i = 0;
                    std::size_t j = 0;
                    while (i < previous_.size() || j < current_.size())
                    {
                        if (j >= current_.size() ||
                            (i < previous_.size() && previous_[i].key < current_[j].key))
                        {
                            events_.push_back(
                                make_event(previous_[i], ContactPhase::End));
                            ++i;
                            continue;
                        }
                        if (i >= previous_.size() || current_[j].key < previous_[i].key)
                        {
                            events_.push_back(
                                make_event(current_[j], ContactPhase::Begin));
                            ++j;
                            continue;
                        }
                        events_.push_back(make_event(current_[j], ContactPhase::Persist));
                        ++i;
                        ++j;
                    }
                }

                /** @brief One record, as the event a gameplay system reads. */
                ContactEvent make_event(const ContactRecord& record, ContactPhase phase) const
                {
                    ContactEvent event;
                    event.a = record.a_entity;
                    event.b = record.b_entity;
                    event.phase = phase;
                    event.trigger = record.trigger;
                    event.normal = from_vector(record.manifold.normal);

                    // The deepest point, because that is the one a sound, a decal or a
                    // damage number is about. Averaging the four would put the point
                    // in the middle of a face the body only touched at one corner.
                    std::size_t deepest = 0;
                    for (std::size_t p = 1; p < record.manifold.point_count; ++p)
                        if (record.manifold.points[p].separation <
                            record.manifold.points[deepest].separation)
                            deepest = p;

                    T impulse = T(0);
                    for (std::size_t p = 0; p < record.manifold.point_count; ++p)
                        impulse += record.manifold.points[p].normal_lambda;
                    event.impulse = Scalar(record.trigger ? T(0) : impulse);

                    if (record.a_slot < bodies_.size() && record.manifold.point_count > 0)
                    {
                        const Body& body = bodies_[record.a_slot];
                        event.point = from_vector(
                            body.position +
                            rotate(body.orientation,
                                   record.manifold.points[deepest].anchor_a_local));
                    }
                    return event;
                }

                /** @brief An order-stable key for the pair of proxies @p a and @p b. */
                static std::uint64_t pair_key(std::uint32_t a, std::uint32_t b) noexcept
                {
                    return (std::uint64_t(a) << 32) | std::uint64_t(b);
                }

                /** @brief Refreshes the per-tick counters after a step. */
                void refresh_statistics()
                {
                    statistics_ = solver_->statistics();
                    statistics_.broadphase_pairs_produced = contact_index_.pairs().size();
                    // The solver measured the device half and left the host stages zero;
                    // fill them in without touching what it reported. Assigned field by
                    // field rather than by struct copy for exactly that reason.
                    statistics_.timings.broadphase_ms = stage_timings_.broadphase_ms;
                    statistics_.timings.narrowphase_ms = stage_timings_.narrowphase_ms;
                    statistics_.timings.island_build_ms = stage_timings_.island_build_ms;
                    statistics_.timings.write_back_ms = stage_timings_.write_back_ms;
                    statistics_.timings.soft_body_ms = stage_timings_.soft_body_ms;
                    statistics_.timings.total_ms = stage_timings_.total_ms;
                    // The partition is the host's, so its four counters are too. The
                    // solver reports every live slot as awake because from inside the
                    // solve that is all a slot can be; the scene is what knows better.
                    statistics_.islands = islands_.islands.size();
                    statistics_.largest_island = islands_.largest;
                    statistics_.awake_bodies = awake_body_count();
                    statistics_.sleeping_bodies = sleeping_body_count();
                    // Per tick, as the field's name says — so it is this tick's broken
                    // joints and not a running total. Assigned after the solver's
                    // statistics are copied in, because that copy would overwrite it.
                    statistics_.fracture_events = joint_events_.size();
                }

                // -- The query side ------------------------------------------------

                /** @brief The shape a query proxy stands for. */
                Physics::CollisionShape<T> query_shape(Physics::ProxyId id) const
                {
                    return query_shapes_[query_index_.proxy(id).payload];
                }

                /** @brief The boundary filter, in the collision layer's own vocabulary. */
                Physics::QueryFilter<T> to_query_filter(const SceneQueryFilter& filter) const
                {
                    Physics::QueryFilter<T> converted;
                    converted.layer_mask = filter.layer_mask;
                    if (!filter.include_triggers)
                        converted.reject_flags = Physics::BodyFlags::trigger;
                    if (filter.exclude != NULL_ENTITY)
                    {
                        const EntityId excluded = filter.exclude;
                        const std::vector<EntityId>* entities = &query_entities_;
                        converted.predicate = [excluded, entities](Physics::ProxyId,
                                                                   std::uint32_t payload)
                        { return (*entities)[payload] != excluded; };
                    }
                    return converted;
                }

                /** @brief A physics-layer hit, at the boundary precision. */
                SceneRayHit from_hit(const Physics::RayHit<T>& hit) const
                {
                    SceneRayHit converted;
                    if (!hit.hit)
                        return converted;
                    converted.hit = true;
                    converted.entity = query_entities_[hit.payload];
                    converted.point = from_vector(hit.point);
                    converted.normal = from_vector(hit.normal);
                    converted.distance = Scalar(hit.distance);
                    return converted;
                }

                /** @brief Rebuilds the query hierarchy if the world has moved. */
                void refresh_query_index() const
                {
                    if (!query_dirty_ || !solver_)
                        return;
                    refresh_bodies();
                    query_dirty_ = false;
                    query_index_ = Physics::BvhBroadphase<T>{};
                    query_shapes_.clear();
                    query_entities_.clear();

                    for (const RigidEntry& entry : rigid_)
                    {
                        const std::size_t slot = solver_->body_slot(entry.handle);
                        if (slot >= bodies_.size())
                            continue;
                        add_query_proxy(collider_shape<T>(entry.collider,
                                                          bodies_[slot].position,
                                                          bodies_[slot].orientation),
                                        entry.entity,
                                        bodies_[slot].inv_mass > T(0)
                                            ? 0u
                                            : Physics::BodyFlags::static_body);
                    }
                    for (const Physics::PlaneCollider<T>& plane : planes_)
                        add_query_proxy(Physics::make_plane_shape<T>(plane.normal, plane.offset),
                                        NULL_ENTITY, Physics::BodyFlags::static_body);
                }

                /** @brief Records one shape and gives the hierarchy a proxy for it. */
                void add_query_proxy(const Physics::CollisionShape<T>& shape, EntityId entity,
                                     std::uint32_t flags) const
                {
                    const std::uint32_t payload =
                        static_cast<std::uint32_t>(query_shapes_.size());
                    query_shapes_.push_back(shape);
                    query_entities_.push_back(entity);
                    query_index_.create_proxy(Physics::shape_world_bounds(shape),
                                              Physics::CollisionFilter{}, flags, payload);
                }

                Execution::Context& context_;
                std::unique_ptr<Solver> solver_;

                std::vector<RigidEntry> rigid_;
                std::unordered_map<EntityId, std::size_t> rigid_index_;
                std::vector<ClothEntry> cloth_;
                std::unordered_map<EntityId, std::size_t> cloth_index_;
                std::vector<SoftEntry> soft_;
                std::unordered_map<EntityId, std::size_t> soft_index_;
                /** @brief Borrows into @ref soft_; rebuilt whenever that vector is. */
                Physics::SoftBodyScene<T> soft_scene_;

                // §9.6's colliders, one generation at a time. `std::deque` rather than
                // `std::vector`: `soft_collider_sets_` and `soft_scene_` hold raw
                // pointers into these three, taken while `rebuild_soft_scene` is still
                // appending, and a deque never invalidates a reference to an element
                // already in it — a vector would need every member counted before the
                // first pointer could safely be handed out.
                std::deque<Physics::SoftRigidPrimitiveCollider<T>> soft_rigid_colliders_;
                /** @brief Parallel to @ref soft_rigid_colliders_: which @ref rigid_ entry each one tests against. */
                std::vector<std::size_t> soft_rigid_targets_;
                std::deque<Physics::SoftSelfCollider<T>> soft_self_colliders_;
                std::deque<Physics::SoftSoftCollider<T>> soft_soft_colliders_;
                /** @brief One set per @ref soft_ entry, combining its self- and rigid colliders. */
                std::vector<Physics::SoftBodyColliderSet<T>> soft_collider_sets_;
                std::vector<Physics::PlaneCollider<T>> planes_;
                std::unordered_set<EntityId> seen_;

                T substep_dt_ = T(1) / T(240);

                // Requested by the profiler panel, consumed when the solver is built.
                // Profiling is a construction-time property of the solve graph — with
                // it off the hot path carries no timestamping at all — so a request
                // that arrives after the solver exists applies to the next one, not
                // to this one. Recreating a live solver to honour a checkbox would
                // discard every body's velocity to answer a question about timing.
                bool profiling_requested_ = false;
                // The host stages of the last tick. Kept here rather than assigned
                // straight onto `statistics_`, because the solver's statistics are
                // copied wholesale over that struct after the tick and would erase
                // anything written to it earlier.
                Physics::PhysicsStageTimings<T> stage_timings_{};

                // The island partition, and the union-find scratch it is built with.
                // Kept across ticks because the builder owns that scratch and a
                // per-tick allocation of it is the cost the feature exists to avoid,
                // and because `wake_island` needs the partition a later tick was
                // decided against.
                Physics::IslandBuilder<T> island_builder_;
                Physics::IslandSet islands_{};
                T sleep_motion_threshold_ = T(0.01);
                T sleep_delay_ = T(0.5);

                // The contact side: proxies numbered once per membership change, the
                // hierarchy refreshed in place every tick, and the manifolds keyed by
                // the pair of proxy numbers so warm starting has something stable to
                // match against.
                Physics::BvhBroadphase<T> contact_index_;
                std::vector<ContactProxy> contact_proxies_;
                std::vector<T> cloth_radius_;
                // The scene's vehicles, and the index that finds one by entity. Held behind
                // pointers because `VehicleInstanceT` is neither copyable nor movable, which
                // is correct for a type that owns solver handles.
                std::vector<VehicleEntry> vehicles_;
                std::unordered_map<EntityId, std::size_t> vehicle_index_;

                std::unordered_map<std::uint32_t, Collider> collider_of_slot_;
                // What each rigid slot contacts as, rebuilt beside the colliders above and
                // for the same reason: the manifold pass asks the question per pair, and
                // walking `rigid_` for an answer would make it quadratic.
                std::unordered_map<std::uint32_t, Physics::PhysicsMaterialT<T>> material_of_slot_;
                std::vector<ContactRecord> current_;
                std::vector<ContactRecord> previous_;
                std::vector<ContactEvent> events_;
                bool proxies_dirty_ = true;

                std::vector<JointEntry> joints_;
                std::vector<JointBrokenEvent> joint_events_;
                // Starts at one so that zero is never handed out: `NULL_JOINT` is zero,
                // and an identity that could equal it would be an identity a caller
                // could not test.
                JointId next_joint_id_ = 1;

                Physics::PhysicsStatistics statistics_{};

                // `mutable` because a query is a const operation on the world that may
                // still have to notice the world moved — the alternative is a
                // non-const raycast, which would make every gameplay system that reads
                // the scene hold a mutable reference to it. The body mirror is the
                // same argument: reading a pose must not require a mutable scene.
                mutable std::vector<Body> bodies_;
                mutable bool bodies_dirty_ = true;
                mutable Physics::BvhBroadphase<T> query_index_;
                mutable std::vector<Physics::CollisionShape<T>> query_shapes_;
                mutable std::vector<EntityId> query_entities_;
                mutable bool query_dirty_ = true;
        };

        /**
         * @brief Creates the physics simulation.
         * @param context The execution context backing the physics buffers and graphs.
         * @return An owned physics simulation; never null.
         */
        inline std::unique_ptr<IPhysicsScene> create_physics_simulation(
            Execution::Context& context)
        {
            return std::unique_ptr<IPhysicsScene>(new PhysicsSimulation(context));
        }
    } // namespace Simulation
} // namespace SushiEngine
