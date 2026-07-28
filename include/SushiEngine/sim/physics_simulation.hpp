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
 * @brief The physics seam, split out of `RuntimeSimulation`.
 *
 * `IPhysicsSimulation` is the abstraction the live world drives its rigid bodies and
 * cloth through, in the fixed boundary `Scalar` precision. `PhysicsSimulation` is the
 * implementation, computing the XPBD solve in `double` and converting at this
 * boundary, so the ECS and renderer never see the solver's internals. Extracting this
 * also gives `RuntimeSimulation` one fewer responsibility: it marshals entity poses to
 * and from descriptors here and no longer owns a `PhysicsWorld` (single
 * responsibility).
 */

#include <cstddef>
#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/cloth.hpp>
#include <SushiEngine/physics/collision/narrowphase.hpp>
#include <SushiEngine/physics/collision/contact_solver.hpp>
#include <SushiEngine/physics/scene/physics_world.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/solver/xpbd_solver.hpp>
#include <SushiEngine/sim/physics_services.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief The `IPhysicsScene` implementation; the XPBD solve runs in `double`.
         *
         * Owns a `Physics::PhysicsWorld` for rigid bodies and another for cloth (kept
         * separate so a rigid-body rebuild's velocity carry-over never has to
         * special-case a pinned grid), and converts every boundary value between
         * `Scalar` and `T` at this class's edge.
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
                 * @param runtime The runtime backing the body buffers and solve graphs.
                 */
                explicit PhysicsSimulation(SushiRuntime::API::Runtime& runtime) noexcept
                    : runtime_(runtime) {}

                void set_rigid_bodies(const std::vector<RigidBodyDesc>& bodies,
                                      std::size_t iterations, Scalar substep_dt) override
                {
                    std::unordered_map<EntityId, Body> previous;
                    if (rigid_)
                        for (const auto& entry : body_ids_)
                            previous.emplace(entry.first, rigid_->body(entry.second));

                    rigid_.reset();
                    body_ids_.clear();
                    rigid_radii_.clear();
                    rigid_is_box_.clear();
                    rigid_half_extents_.clear();
                    if (bodies.empty())
                        return;

                    auto world = std::make_unique<World>(runtime_);
                    rigid_radii_.reserve(bodies.size());
                    rigid_is_box_.reserve(bodies.size());
                    rigid_half_extents_.reserve(bodies.size());
                    for (const RigidBodyDesc& desc : bodies)
                    {
                        Body body;
                        const auto it = previous.find(desc.id);
                        if (it != previous.end())
                        {
                            body = it->second;
                        }
                        else
                        {
                            body.position = to_vector(desc.position);
                            body.orientation = to_quaternion(desc.orientation);
                        }
                        body.inv_mass = T(desc.inv_mass);
                        body.inv_inertia = to_vector(desc.inv_inertia);
                        body.drag_coefficient = T(desc.drag_coefficient);
                        // Shape data parallels body id (add order), so rigid_*_[BodyId]
                        // describes the body world->add_body returns.
                        rigid_radii_.push_back(T(desc.radius));
                        rigid_is_box_.push_back(desc.box);
                        rigid_half_extents_.push_back(to_vector(desc.half_extents));
                        body_ids_.emplace(desc.id, world->add_body(body));
                    }
                    world->finalize(iterations, T(substep_dt), Projection{});
                    rigid_ = std::move(world);
                    substep_dt_ = T(substep_dt);
                }

                void update_rigid_body_params(EntityId id, Scalar inv_mass,
                                              const Vector3& inv_inertia,
                                              Scalar drag_coefficient) override
                {
                    if (!rigid_)
                        return;
                    const auto it = body_ids_.find(id);
                    if (it == body_ids_.end())
                        return;
                    Body& body = rigid_->body(it->second);
                    body.inv_mass = T(inv_mass);
                    body.inv_inertia = to_vector(inv_inertia);
                    body.drag_coefficient = T(drag_coefficient);
                }

                bool rigid_pose(EntityId id, SolvedPose& out) const override
                {
                    if (!rigid_)
                        return false;
                    const auto it = body_ids_.find(id);
                    if (it == body_ids_.end())
                        return false;
                    const Body& body = rigid_->body(it->second);
                    out.position = from_vector(body.position);
                    out.orientation = from_quaternion(body.orientation);
                    return true;
                }

                void set_rigid_pose(EntityId id, const Vector3& position,
                                    const Quaternion& orientation) override
                {
                    if (!rigid_)
                        return;
                    const auto it = body_ids_.find(id);
                    if (it == body_ids_.end())
                        return;
                    Body& body = rigid_->body(it->second);
                    body.position = to_vector(position);
                    body.orientation = to_quaternion(orientation);
                    // Clear velocity and align the previous pose so the next velocity
                    // derivation sees no jump — the body is placed, not thrown.
                    body.prev_position = body.position;
                    body.prev_orientation = body.orientation;
                    body.velocity = Vector3T<T>{T(0), T(0), T(0)};
                    body.angular_velocity = Vector3T<T>{T(0), T(0), T(0)};
                }

                void set_cloth_grids(const std::vector<ClothDesc>& grids,
                                     std::size_t iterations, Scalar substep_dt) override
                {
                    cloth_.reset();
                    cloth_grids_.clear();
                    cloth_radii_.clear();
                    if (grids.empty())
                        return;

                    auto world = std::make_unique<World>(runtime_);
                    bool any_bodies = false;
                    for (const ClothDesc& desc : grids)
                    {
                        if (desc.rows == 0 || desc.cols == 0)
                            continue;
                        Physics::ClothGrid grid = Physics::build_cloth_grid<Constraint>(
                            *world, desc.rows, desc.cols, T(desc.spacing),
                            to_vector(desc.origin), T(desc.compliance));
                        // One radius per grid point, appended in the same order
                        // build_cloth_grid registered the bodies, so cloth_radii_ stays
                        // parallel to the cloth world's body buffer across all grids.
                        cloth_radii_.insert(cloth_radii_.end(), grid.bodies.size(),
                                            T(desc.thickness));
                        if (!grid.bodies.empty())
                            any_bodies = true;
                        cloth_grids_.emplace(desc.id, std::move(grid));
                    }

                    if (!any_bodies)
                    {
                        cloth_grids_.clear();
                        cloth_radii_.clear();
                        return;
                    }
                    world->finalize(iterations, T(substep_dt), Projection{});
                    cloth_ = std::move(world);
                    substep_dt_ = T(substep_dt);
                }

                std::vector<Vector3> cloth_positions(EntityId id) const override
                {
                    std::vector<Vector3> positions;
                    if (!cloth_)
                        return positions;
                    const auto it = cloth_grids_.find(id);
                    if (it == cloth_grids_.end())
                        return positions;
                    positions.reserve(it->second.bodies.size());
                    for (const Physics::BodyId body_id : it->second.bodies)
                        positions.push_back(from_vector(cloth_->body(body_id).position));
                    return positions;
                }

                bool cloth_dimensions(EntityId id, std::uint32_t& rows,
                                      std::uint32_t& cols) const override
                {
                    if (!cloth_)
                        return false;
                    const auto it = cloth_grids_.find(id);
                    if (it == cloth_grids_.end())
                        return false;
                    rows = static_cast<std::uint32_t>(it->second.rows);
                    cols = static_cast<std::uint32_t>(it->second.cols);
                    return true;
                }

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
                }

                void step(const GravitySampler& gravity, std::size_t substeps) override
                {
                    // Convert at the boundary: the solver holds T-precision positions, the
                    // sampler speaks boundary Scalar. Evaluated per body, per sub-step, so a
                    // body that moves through the field feels the field change.
                    const auto acceleration_at = [&gravity](const Vector3T<T>& position) noexcept
                    {
                        return to_vector(gravity(from_vector(position)));
                    };
                    // The candidate pairs are found once for the whole tick, against
                    // bounds swept by how far a body can travel in it. Rebuilding them
                    // every sub-step — twice every sub-step, as this did — sorts the
                    // whole scene tens of times per tick to learn something that barely
                    // changes, and it is the single reason a large sub-step count was
                    // unaffordable. The per-sub-step work that remains is the
                    // resolution, which is what actually has to be repeated.
                    refresh_contact_pairs(T(substeps) * substep_dt_);

                    // Drive both worlds in lockstep so contacts spanning them (rigid↔cloth)
                    // are resolved before either derives its velocity — two-way coupling.
                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        if (rigid_)
                        {
                            rigid_->predict_substep_field(acceleration_at);
                            rigid_->solve_constraints();
                        }
                        if (cloth_)
                        {
                            cloth_->predict_substep_field(acceleration_at);
                            cloth_->solve_constraints();
                        }
                        resolve_contacts();
                        if (rigid_)
                            rigid_->derive_velocity();
                        if (cloth_)
                            cloth_->derive_velocity();
                    }

                    refresh_statistics(substeps);
                }

                /** @copydoc IPhysicsStepper::statistics */
                const Physics::PhysicsStatistics& statistics() const noexcept override
                {
                    return statistics_;
                }

            private:
                using T = double;
                using Constraint = Physics::XpbdDistanceConstraintT<T>;
                using Projection = Physics::XpbdDistanceProjectionT<T>;
                using World = Physics::PhysicsWorld<Constraint>;
                using Body = Physics::RigidBodyT<T>;

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

                /** @brief Contact sweeps per sub-step, enough to settle modest stacks. */
                static constexpr std::size_t CONTACT_ITERATIONS = 2;

                /**
                 * @brief Rebuilds the unified body view (rigid + cloth) for this sub-step.
                 *
                 * Each entry points straight into its owning buffer's body, so a contact
                 * correction moves the real body. Reused across sub-steps (cleared and
                 * refilled) to avoid per-sub-step allocation.
                 */
                void build_contact_bodies()
                {
                    contact_bodies_.clear();
                    contact_speeds_.clear();
                    if (rigid_)
                    {
                        const std::size_t count = rigid_->body_count();
                        for (std::size_t i = 0; i < count && i < rigid_radii_.size(); ++i)
                        {
                            Body& body = rigid_->body(static_cast<Physics::BodyId>(i));
                            Physics::ContactBody<T> entry;
                            entry.position = &body.position;
                            entry.orientation = &body.orientation;
                            entry.inv_mass = body.inv_mass;
                            entry.inv_inertia = body.inv_inertia;
                            entry.is_box = rigid_is_box_[i];
                            entry.half_extents = rigid_half_extents_[i];
                            entry.radius = rigid_radii_[i];
                            entry.is_cloth = false;
                            contact_bodies_.push_back(entry);
                            // Recorded alongside the view because the broadphase runs
                            // once per tick now and has to know how far each entry can
                            // travel before the next one.
                            contact_speeds_.push_back(length(body.velocity));
                        }
                    }
                    if (cloth_)
                    {
                        const std::size_t count = cloth_->body_count();
                        for (std::size_t i = 0; i < count && i < cloth_radii_.size(); ++i)
                        {
                            Body& body = cloth_->body(static_cast<Physics::BodyId>(i));
                            Physics::ContactBody<T> entry;
                            entry.position = &body.position;
                            entry.inv_mass = body.inv_mass;
                            entry.radius = cloth_radii_[i];
                            entry.is_cloth = true;
                            contact_bodies_.push_back(entry);
                            contact_speeds_.push_back(length(body.velocity));
                        }
                    }
                }

                /**
                 * @brief Finds this tick's candidate pairs, once, against swept bounds.
                 *
                 * Sweeping the bounds by how far a body can travel over the whole tick
                 * is what makes once-per-tick sound: a pair that will only start
                 * overlapping three sub-steps from now is already a candidate, so the
                 * resolution pass never needs a broadphase it does not have.
                 *
                 * The pair list is then sorted by body index. Sweep-and-prune emits
                 * pairs in whatever order its axis sort produced, and the contact
                 * resolution is Gauss-Seidel — order-dependent by construction — so
                 * an unsorted list makes the result depend on something that is not
                 * simulation state. Sorting costs a fraction of the sweep and buys the
                 * determinism rule outright.
                 *
                 * @param tick_duration The whole step's duration, in seconds.
                 */
                void refresh_contact_pairs(T tick_duration)
                {
                    build_contact_bodies();
                    const std::size_t count = contact_bodies_.size();
                    pairs_.clear();
                    if (count == 0)
                        return;

                    aabbs_.clear();
                    aabbs_.reserve(count);
                    for (std::size_t i = 0; i < count; ++i)
                    {
                        Physics::Aabb<T> bounds =
                            Physics::contact_body_aabb(contact_bodies_[i]);
                        const T margin = contact_speeds_[i] * tick_duration;
                        bounds.min = bounds.min - Vector3T<T>{margin, margin, margin};
                        bounds.max = bounds.max + Vector3T<T>{margin, margin, margin};
                        aabbs_.push_back(bounds);
                    }
                    Physics::sweep_and_prune(aabbs_, pairs_);
                    std::sort(pairs_.begin(), pairs_.end());
                }

                /**
                 * @brief Resolves this sub-step's contacts over the tick's candidate pairs.
                 *
                 * Two-way by inverse mass, so rigid↔cloth pushes both ways, plus every
                 * body against the static planes. Several sweeps, because one pass of a
                 * Gauss-Seidel resolution leaves a stack leaning.
                 */
                void resolve_contacts()
                {
                    if (contact_bodies_.empty())
                        return;

                    for (std::size_t iteration = 0; iteration < CONTACT_ITERATIONS; ++iteration)
                    {
                        for (const auto& pair : pairs_)
                            Physics::resolve_contact_bodies(contact_bodies_[pair.first],
                                                            contact_bodies_[pair.second]);
                        for (Physics::ContactBody<T>& body : contact_bodies_)
                            for (const Physics::PlaneCollider<T>& plane : planes_)
                                Physics::resolve_contact_body_plane(body, plane);
                    }
                }

                /**
                 * @brief Refreshes the per-tick counters after a step.
                 *
                 * What can honestly be reported at this stage and no more. The contact
                 * pass is still a host pass outside the solve graph, so there are no
                 * manifolds to count and no per-stage device timings to read; those
                 * appear when the contacts move into the graph, and reporting a zero
                 * is the truthful placeholder until they do.
                 *
                 * @param substeps The sub-step count this tick ran.
                 */
                void refresh_statistics(std::size_t substeps)
                {
                    statistics_ = Physics::PhysicsStatistics{};
                    statistics_.awake_bodies = contact_bodies_.size();
                    statistics_.broadphase_pairs_produced = pairs_.size();
                    statistics_.contact_points = pairs_.size();
                    statistics_.substeps = substeps;
                    if (rigid_)
                    {
                        statistics_.colors = rigid_->color_count();
                        statistics_.compile_count += rigid_->compile_count();
                    }
                    if (cloth_)
                        statistics_.compile_count += cloth_->compile_count();
                }

                SushiRuntime::API::Runtime& runtime_;
                std::unique_ptr<World> rigid_;
                std::unordered_map<EntityId, Physics::BodyId> body_ids_;
                std::vector<T> rigid_radii_;
                std::vector<char> rigid_is_box_;
                std::vector<Vector3T<T>> rigid_half_extents_;
                std::unique_ptr<World> cloth_;
                std::unordered_map<EntityId, Physics::ClothGrid> cloth_grids_;
                std::vector<T> cloth_radii_;
                std::vector<Physics::PlaneCollider<T>> planes_;
                T substep_dt_ = T(1) / T(240);
                // Reused scratch for the contact pass. The pair list is found once per
                // tick and read every sub-step, so it outlives a single sub-step.
                std::vector<Physics::ContactBody<T>> contact_bodies_;
                std::vector<T> contact_speeds_;
                std::vector<Physics::Aabb<T>> aabbs_;
                std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs_;
                Physics::PhysicsStatistics statistics_{};
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
