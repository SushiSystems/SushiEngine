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
 * @brief The runtime-precision physics seam, split out of `RuntimeSimulation`.
 *
 * `IPhysicsSimulation` is the abstraction the live world drives its rigid bodies and
 * cloth through, in the fixed boundary `Scalar` precision. `PhysicsSimulation<T>` is
 * the implementation, computing the XPBD solve in element type `T` (`float` or
 * `double`) and converting at this boundary, so the ECS and renderer never see the
 * solver's precision. Both `PhysicsSimulation<float>` and `PhysicsSimulation<double>`
 * are compiled into the simulation library, and `create_physics_simulation` picks one
 * at runtime — which is what makes the engine's float/double choice a live decision
 * rather than a build flag. Extracting this also gives `RuntimeSimulation` one fewer
 * responsibility: it marshals entity poses to and from descriptors here and no longer
 * owns a `PhysicsWorld` (single responsibility).
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/cloth.hpp>
#include <SushiEngine/physics/collision.hpp>
#include <SushiEngine/physics/contact_solver.hpp>
#include <SushiEngine/physics/physics_world.hpp>
#include <SushiEngine/physics/rigid_body.hpp>
#include <SushiEngine/physics/xpbd_constraint.hpp>
#include <SushiEngine/physics/xpbd_solver.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief A rigid body to (re)build, addressed by its owning entity. */
        struct RigidBodyDesc
        {
            EntityId id = NULL_ENTITY;   /**< The entity that owns this body. */
            Vector3 position;            /**< Seed position (used only for a newly added body). */
            Quaternion orientation;      /**< Seed orientation (used only for a newly added body). */
            Scalar inv_mass = Scalar(1); /**< Inverse mass; 0 pins the body. */
            Vector3 inv_inertia;         /**< Diagonal body-local inverse inertia. */
            Scalar radius = Scalar(0.5); /**< Collision radius when the body collides as a sphere. */
            bool box = false;            /**< Collide as an oriented box (else a sphere). */
            Vector3 half_extents{Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)}}; /**< Box half-extents. */
        };

        /** @brief A cloth grid to (re)build, addressed by its owning entity. */
        struct ClothDesc
        {
            EntityId id = NULL_ENTITY;     /**< The entity that owns this grid. */
            std::size_t rows = 0;          /**< Grid rows (row 0 is pinned). */
            std::size_t cols = 0;          /**< Grid columns. */
            Scalar spacing = Scalar(0.5);  /**< Distance between adjacent grid points. */
            Vector3 origin;                /**< World position of grid point (0, 0). */
            Scalar compliance = Scalar(0); /**< XPBD compliance of every constraint. */
            Scalar thickness = Scalar(0.1);/**< Per-particle collision radius against obstacles. */
        };

        /** @brief A static half-space plane the physics collides bodies against. */
        struct PlaneDesc
        {
            Vector3 point;                            /**< A point on the plane, in world space. */
            Vector3 normal{Vector3{0, 1, 0}};         /**< Unit plane normal (solid side is below it). */
        };

        /** @brief A solved rigid-body pose, in boundary precision. */
        struct SolvedPose
        {
            Vector3 position;
            Quaternion orientation;
        };

        /**
         * @brief The precision-agnostic physics surface the live world drives.
         *
         * Every value crossing this interface is boundary `Scalar`; the implementation
         * converts to and from its own solve precision internally. The world rebuilds
         * the body/grid set (only when it changes), steps once per tick, and reads the
         * solved poses back — it never sees a `PhysicsWorld`, a `RigidBody`, or the
         * solve precision.
         */
        class IPhysicsSimulation
        {
            public:
                virtual ~IPhysicsSimulation() = default;

                /**
                 * @brief Rebuilds the rigid-body world from @p bodies.
                 *
                 * A body-count change needs a fresh solve graph, so this rebuilds
                 * wholesale, but preserves each persisting entity's live velocity and
                 * pose (matched by `id`) so toggling one body does not reset the others
                 * already in motion; a newly added body seeds from its descriptor pose,
                 * at rest. Call only when the set of bodies actually changes.
                 *
                 * @param bodies     The full set of rigid bodies after the change.
                 * @param iterations Gauss-Seidel sweeps per sub-step.
                 * @param substep_dt The fixed sub-step duration, in seconds.
                 */
                virtual void set_rigid_bodies(const std::vector<RigidBodyDesc>& bodies,
                                              std::size_t iterations, Scalar substep_dt) = 0;

                /**
                 * @brief Updates a live body's mass/inertia without a rebuild; a no-op if absent.
                 * @param id          The entity whose body to update.
                 * @param inv_mass    New inverse mass.
                 * @param inv_inertia New diagonal body-local inverse inertia.
                 */
                virtual void update_rigid_body_params(EntityId id, Scalar inv_mass,
                                                      const Vector3& inv_inertia) = 0;

                /**
                 * @brief Reads a body's solved pose.
                 * @param id  The entity whose body to read.
                 * @param out Receives the solved pose when @p id has a body.
                 * @return Whether @p id owns a rigid body (and @p out was written).
                 */
                virtual bool rigid_pose(EntityId id, SolvedPose& out) const = 0;

                /**
                 * @brief Teleports a live body to @p position/@p orientation, zeroing its velocity.
                 *
                 * The write path for a manual transform edit while the world is playing:
                 * the body jumps to the authored pose and its velocity is cleared (so it
                 * does not fly off with stale momentum), then the solve continues from
                 * there — dragging a rigid body in the viewport moves the physics, it is
                 * not immediately overwritten. A no-op if @p id has no body.
                 *
                 * @param id          The entity whose body to move.
                 * @param position    The new world position.
                 * @param orientation The new world orientation.
                 */
                virtual void set_rigid_pose(EntityId id, const Vector3& position,
                                            const Quaternion& orientation) = 0;

                /**
                 * @brief Rebuilds the cloth world from @p grids.
                 *
                 * Unlike rigid bodies, no live state is carried over — a rows/cols/
                 * spacing change replaces the grid topology outright, so there is
                 * nothing meaningful to preserve.
                 *
                 * @param grids      The full set of cloth grids after the change.
                 * @param iterations Gauss-Seidel sweeps per sub-step.
                 * @param substep_dt The fixed sub-step duration, in seconds.
                 */
                virtual void set_cloth_grids(const std::vector<ClothDesc>& grids,
                                             std::size_t iterations, Scalar substep_dt) = 0;

                /**
                 * @brief A cloth grid's current world-space particle positions.
                 * @param id The entity whose grid to read.
                 * @return Row-major positions (`row * cols + col`); empty if @p id has none.
                 */
                virtual std::vector<Vector3> cloth_positions(EntityId id) const = 0;

                /**
                 * @brief A cloth grid's row/column dimensions.
                 * @param id   The entity whose grid to read.
                 * @param rows Receives the row count.
                 * @param cols Receives the column count.
                 * @return Whether @p id owns a cloth grid (and the outputs were written).
                 */
                virtual bool cloth_dimensions(EntityId id, std::uint32_t& rows,
                                              std::uint32_t& cols) const = 0;

                /**
                 * @brief Sets the static planes bodies and cloth collide against.
                 *
                 * Cheap to call every tick (it just replaces a small list), so a host
                 * re-supplies the scene's ground/ramp planes each step rather than
                 * tracking when they move. An empty list disables plane contacts.
                 *
                 * @param planes The static collision planes, in world space.
                 */
                virtual void set_static_planes(const std::vector<PlaneDesc>& planes) = 0;

                /**
                 * @brief Advances rigid and cloth by one outer step, resolving contacts.
                 *
                 * Each sub-step, after the constraint solve, rigid bodies are separated
                 * from each other and pushed out of the static planes, and cloth
                 * particles are pushed out of the planes and out of the rigid bodies
                 * (snapshotted as sphere obstacles) — so rigid bodies rest and stack and
                 * cloth drapes over them.
                 *
                 * @param gravity  External acceleration applied every sub-step.
                 * @param substeps Number of sub-steps this step.
                 */
                virtual void step(const Vector3& gravity, std::size_t substeps) = 0;
        };

        /**
         * @brief An `IPhysicsSimulation` whose solve runs in element type @p T.
         *
         * Owns a `Physics::PhysicsWorld` of the matching precision for rigid bodies and
         * another for cloth (kept separate so a rigid-body rebuild's velocity carry-over
         * never has to special-case a pinned grid), and converts every boundary value
         * between `Scalar` and `T` at this class's edge.
         *
         * @tparam T The solve precision (`float` or `double`).
         */
        template <typename T>
        class PhysicsSimulation final : public IPhysicsSimulation
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
                        // Shape data parallels body id (add order), so rigid_*_[BodyId]
                        // describes the body world->add_body returns.
                        rigid_radii_.push_back(T(desc.radius));
                        rigid_is_box_.push_back(desc.box);
                        rigid_half_extents_.push_back(to_vector(desc.half_extents));
                        body_ids_.emplace(desc.id, world->add_body(body));
                    }
                    world->finalize(iterations, T(substep_dt), Projection{});
                    rigid_ = std::move(world);
                }

                void update_rigid_body_params(EntityId id, Scalar inv_mass,
                                              const Vector3& inv_inertia) override
                {
                    if (!rigid_)
                        return;
                    const auto it = body_ids_.find(id);
                    if (it == body_ids_.end())
                        return;
                    Body& body = rigid_->body(it->second);
                    body.inv_mass = T(inv_mass);
                    body.inv_inertia = to_vector(inv_inertia);
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

                void step(const Vector3& gravity, std::size_t substeps) override
                {
                    const Vector3T<T> g = to_vector(gravity);
                    // Drive both worlds in lockstep so contacts spanning them (rigid↔cloth)
                    // are resolved before either derives its velocity — two-way coupling.
                    for (std::size_t s = 0; s < substeps; ++s)
                    {
                        if (rigid_)
                        {
                            rigid_->predict_substep(g);
                            rigid_->solve_constraints();
                        }
                        if (cloth_)
                        {
                            cloth_->predict_substep(g);
                            cloth_->solve_constraints();
                        }
                        resolve_contacts();
                        if (rigid_)
                            rigid_->derive_velocity();
                        if (cloth_)
                            cloth_->derive_velocity();
                    }
                }

            private:
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
                    if (rigid_)
                    {
                        const std::size_t count = rigid_->body_count();
                        for (std::size_t i = 0; i < count && i < rigid_radii_.size(); ++i)
                        {
                            Body& body = rigid_->body(static_cast<Physics::BodyId>(i));
                            Physics::ContactBody<T> entry;
                            entry.position = &body.position;
                            entry.inv_mass = body.inv_mass;
                            entry.is_box = rigid_is_box_[i];
                            entry.half_extents = rigid_half_extents_[i];
                            entry.orientation = body.orientation;
                            entry.radius = rigid_radii_[i];
                            entry.is_cloth = false;
                            contact_bodies_.push_back(entry);
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
                        }
                    }
                }

                /**
                 * @brief The unified contact pass: broadphase, then two-way pair + plane resolve.
                 *
                 * Builds one body view over both worlds, culls candidate pairs with
                 * sweep-and-prune, and resolves them by inverse mass (so rigid↔cloth pushes
                 * both ways), plus every body against the static planes. Runs a few sweeps
                 * so stacks and drapes settle.
                 */
                void resolve_contacts()
                {
                    build_contact_bodies();
                    const std::size_t count = contact_bodies_.size();
                    if (count == 0)
                        return;

                    for (std::size_t iteration = 0; iteration < CONTACT_ITERATIONS; ++iteration)
                    {
                        aabbs_.clear();
                        aabbs_.reserve(count);
                        for (const Physics::ContactBody<T>& body : contact_bodies_)
                            aabbs_.push_back(Physics::contact_body_aabb(body));
                        Physics::sweep_and_prune(aabbs_, pairs_);
                        for (const auto& pair : pairs_)
                            Physics::resolve_contact_bodies(contact_bodies_[pair.first],
                                                            contact_bodies_[pair.second]);
                        for (Physics::ContactBody<T>& body : contact_bodies_)
                            for (const Physics::PlaneCollider<T>& plane : planes_)
                                Physics::resolve_contact_body_plane(body, plane);
                    }
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
                // Reused scratch for the per-sub-step contact pass.
                std::vector<Physics::ContactBody<T>> contact_bodies_;
                std::vector<Physics::Aabb<T>> aabbs_;
                std::vector<std::pair<std::uint32_t, std::uint32_t>> pairs_;
        };

        /**
         * @brief Creates the physics simulation of the requested precision.
         *
         * Instantiates `PhysicsSimulation<float>` or `PhysicsSimulation<double>`,
         * compiling both into the library so the choice is made here, at runtime.
         *
         * @param precision The solve precision to run in.
         * @param runtime   The runtime backing the physics buffers and graphs.
         * @return An owned physics simulation; never null.
         */
        inline std::unique_ptr<IPhysicsSimulation> create_physics_simulation(
            Precision precision, SushiRuntime::API::Runtime& runtime)
        {
            if (precision == Precision::Double)
                return std::unique_ptr<IPhysicsSimulation>(new PhysicsSimulation<double>(runtime));
            return std::unique_ptr<IPhysicsSimulation>(new PhysicsSimulation<float>(runtime));
        }
    } // namespace Simulation
} // namespace SushiEngine
