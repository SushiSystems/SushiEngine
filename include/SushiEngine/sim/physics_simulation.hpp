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
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/bvh_broadphase.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/collision/narrowphase_dispatch.hpp>
#include <SushiEngine/physics/collision/scene_query.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/soft/cloth.hpp>
#include <SushiEngine/physics/solver/contact_constraint.hpp>
#include <SushiEngine/physics/solver/runtime_graph_builder.hpp>
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
         * It implements all four services because it genuinely does all four jobs.
         * Consumers should not: the split exists so a caller depends on the one
         * service it uses, and naming this class instead is how that gets undone.
         */
        class PhysicsSimulation final : public IPhysicsScene
        {
            public:
                /**
                 * @brief Creates an empty physics simulation backed by @p runtime.
                 * @param runtime The runtime backing the body buffers and the solve graph.
                 */
                explicit PhysicsSimulation(SushiRuntime::API::Runtime& runtime) noexcept
                    : runtime_(runtime)
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

                void set_rigid_pose(EntityId id, const Vector3& position,
                                    const Quaternion& orientation) override
                {
                    if (!solver_)
                        return;
                    const auto it = rigid_index_.find(id);
                    if (it == rigid_index_.end())
                        return;
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
                void step(const GravitySampler& gravity, std::size_t substeps) override
                {
                    if (!solver_ || (rigid_.empty() && cloth_.empty()))
                    {
                        statistics_ = Physics::PhysicsStatistics{};
                        events_.clear();
                        return;
                    }

                    const std::size_t floor = substeps > 0 ? substeps : 1;
                    const T delta_time = T(floor) * substep_dt_;

                    bodies_dirty_ = true;
                    refresh_bodies();
                    apply_gravity_field(gravity);
                    refresh_contact_index(delta_time);
                    submit_contacts(delta_time, floor);

                    Physics::StepParameters<T> parameters;
                    parameters.delta_time = delta_time;
                    // Zero, because the field has already been folded into each
                    // body. Passing it here as well would apply it twice.
                    parameters.gravity = Vector3T<T>{T(0), T(0), T(0)};
                    parameters.substep_floor = floor;
                    solver_->step(parameters);

                    collect_contacts();

                    bodies_dirty_ = true;
                    refresh_bodies();
                    // After the solve and after the poses came back, so an event
                    // reports where the contact ended up and what impulse it took —
                    // not where it was predicted to be before anything was resolved.
                    build_contact_events();
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

                /** @brief One rigid body: who owns it, where it lives, what it collides as. */
                struct RigidEntry
                {
                    EntityId entity = NULL_ENTITY;
                    Physics::BodyHandle handle;
                    Collider collider{};
                    Physics::CollisionFilter filter{};
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
                 * *kind*, and with contacts that is now four kinds of node; 32
                 * colours would quadruple a compile that happens while a scene is
                 * loading. Sixteen clears a cloth grid's busiest vertex, which links
                 * to eight neighbours.
                 */
                void ensure_solver()
                {
                    if (solver_)
                        return;
                    Physics::PhysicsConfigurationT<T> configuration;
                    configuration.capacities.bodies = 16384;
                    configuration.capacities.constraints = 65536;
                    configuration.capacities.contacts = 16384;
                    configuration.capacities.colors = 16;
                    configuration.substeps.minimum = 4;
                    configuration.substeps.maximum = 16;
                    // The profiler panel's request, consumed here because profiling is a
                    // construction-time property of the solve graph (off = no timestamps
                    // on the hot path at all — configuration.hpp).
                    configuration.profiling = profiling_requested_;
                    solver_.reset(new Solver(runtime_, configuration));
                    bodies_.assign(configuration.capacities.bodies, Body{});
                    bodies_dirty_ = true;
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
                    Body body;
                    if (!solver_->read_body(entry.handle, body))
                        return;
                    body.inv_mass = T(desc.inv_mass);
                    body.inv_inertia = to_vector(desc.inv_inertia);
                    body.drag_coefficient = T(desc.drag_coefficient);
                    solver_->write_body(entry.handle, body);
                    bodies_dirty_ = true;
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
                 * @brief Samples the gravity field per body and folds it into each one.
                 *
                 * Per body, once per tick. The alternative — the uniform vector
                 * `StepParameters` carries — cannot express a planetary field, and
                 * sampling inside the substep loop is not available at all now that
                 * `predict` runs on the device (§6.6).
                 */
                void apply_gravity_field(const GravitySampler& gravity)
                {
                    const std::size_t count = live_slot_count();
                    for (std::size_t slot = 0; slot < count; ++slot)
                    {
                        const Vector3T<T> sampled =
                            to_vector(gravity(from_vector(bodies_[slot].position)));
                        bodies_[slot].external_acceleration = sampled;
                    }
                    // Written back through the handles rather than by slot, because a
                    // slot the scene does not own is not this scene's to write.
                    for (const RigidEntry& entry : rigid_)
                        write_field(entry.handle);
                    for (const ClothEntry& entry : cloth_)
                        for (const Physics::BodyHandle handle : entry.grid.bodies)
                            write_field(handle);
                    bodies_dirty_ = false;
                }

                /** @brief Sends one body's sampled acceleration back to the solver. */
                void write_field(Physics::BodyHandle handle)
                {
                    const std::size_t slot = solver_->body_slot(handle);
                    if (slot < bodies_.size())
                        solver_->write_body(handle, bodies_[slot]);
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
                        proxy.shape = shape_for_slot(proxy.slot);
                        const Vector3T<T> travel =
                            bodies_[proxy.slot].velocity * delta_time;
                        contact_index_.update_proxy(
                            proxy.id, Physics::shape_world_bounds(proxy.shape), travel);
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
                    cloth_radius_.assign(bodies_.size(), T(0));
                    for (const RigidEntry& entry : rigid_)
                    {
                        const std::size_t slot = solver_->body_slot(entry.handle);
                        if (slot < bodies_.size())
                            collider_of_slot_.emplace(std::uint32_t(slot), entry.collider);
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
                    Physics::ContactSolveParams<T> params;
                    params.rest_offset = REST_OFFSET;
                    params.static_friction = T(0.6);
                    params.dynamic_friction = T(0.5);
                    params.restitution = T(0);
                    // A resting body's contacts carry a closing speed of about
                    // `g * h` every substep purely because gravity had a substep to
                    // act; returning that is how a settled stack buzzes for ever.
                    params.restitution_threshold = T(2) * T(9.81) * substep;

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
                        record.manifold = Physics::generate_shape_manifold<T>(
                            lhs.shape, rhs.shape, CONTACT_OFFSET);
                        if (record.manifold.point_count == 0)
                            continue;

                        // Last tick's impulses, matched point by point on the feature
                        // ids the clipper stamped. Without this a box on a ramp has no
                        // friction cone until one builds, and creeps for the substep
                        // it takes to build it.
                        const ContactRecord* was = find_previous(record.key);
                        if (was != nullptr)
                            Physics::warm_start_manifold(record.manifold, was->manifold);

                        current_.push_back(record);
                        if (record.trigger)
                            continue; // reported, never resolved

                        Contact contact;
                        contact.a = lhs.slot;
                        contact.b = rhs.slot;
                        contact.key = record.key;
                        contact.params = params;
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

                SushiRuntime::API::Runtime& runtime_;
                std::unique_ptr<Solver> solver_;

                std::vector<RigidEntry> rigid_;
                std::unordered_map<EntityId, std::size_t> rigid_index_;
                std::vector<ClothEntry> cloth_;
                std::unordered_map<EntityId, std::size_t> cloth_index_;
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

                // The contact side: proxies numbered once per membership change, the
                // hierarchy refreshed in place every tick, and the manifolds keyed by
                // the pair of proxy numbers so warm starting has something stable to
                // match against.
                Physics::BvhBroadphase<T> contact_index_;
                std::vector<ContactProxy> contact_proxies_;
                std::vector<T> cloth_radius_;
                std::unordered_map<std::uint32_t, Collider> collider_of_slot_;
                std::vector<ContactRecord> current_;
                std::vector<ContactRecord> previous_;
                std::vector<ContactEvent> events_;
                bool proxies_dirty_ = true;

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
         * @param runtime The runtime backing the physics buffers and graphs.
         * @return An owned physics simulation; never null.
         */
        inline std::unique_ptr<IPhysicsScene> create_physics_simulation(
            SushiRuntime::API::Runtime& runtime)
        {
            return std::unique_ptr<IPhysicsScene>(new PhysicsSimulation(runtime));
        }
    } // namespace Simulation
} // namespace SushiEngine
