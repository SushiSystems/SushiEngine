/**************************************************************************/
/* physics_services.hpp                                                   */
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
 * @file physics_services.hpp
 * @brief The physics boundary, split by what a consumer actually needs.
 *
 * There used to be one `IPhysicsSimulation` mixing rigid bodies, cloth, static
 * geometry and stepping, so every consumer depended on all of it. That is the
 * interface-segregation violation §4.3 names, and the cost was not only
 * conceptual: the single interface lived in a header that pulls in SushiRuntime and
 * most of `physics/`, so a gameplay system that only wanted to read a solved pose
 * paid for the entire solver in compile time.
 *
 * The services below are the split. This header names no runtime type, no solver,
 * and no shape — only the boundary vocabulary — so depending on one service costs
 * what that service is worth. `IPhysicsScene` composes them and exists for the one
 * object that owns the whole thing; the concrete implementation lives in
 * `physics_simulation.hpp`, which is where the heavy includes stay.
 */

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
#include <SushiEngine/simulation/collider.hpp>
#include <SushiEngine/simulation/joint_params.hpp>
#include <SushiEngine/simulation/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief A rigid body to (re)build, addressed by its owning entity. */
        struct RigidBodyDescription
        {
            EntityId id = NULL_ENTITY;   /**< The entity that owns this body. */
            Vector3 position;            /**< Seed position (used only for a newly added body). */
            Quaternion orientation;      /**< Seed orientation (used only for a newly added body). */
            Scalar inv_mass = Scalar(1); /**< Inverse mass; 0 pins the body. */
            Vector3 inv_inertia;         /**< Diagonal body-local inverse inertia. */
            Scalar drag_coefficient = Scalar(0); /**< Quadratic drag: acceleration -k|v|v, m⁻¹; 0 disables. */

            /**
             * @brief What this body collides as: the full collider, scaled (§5.5).
             *
             * There used to be a `radius`, a `box` flag and a `half_extents` beside
             * this, derived from it, because the single-point contact path could not
             * read a collider. That path is gone — the live tick generates manifolds
             * and submits them to the solver — and so are they. Keeping a derived
             * copy of a record that is already here is how a body ends up colliding
             * as a box of one size while reporting a radius from another.
             */
            Collider collider{};

            /**
             * @brief The surface this body contacts as (§5.3).
             *
             * Carried by value, exactly as `SoftBodyDescription` carries its constitutive material,
             * rather than as an index into a scene material table. There is no such table,
             * and the reason there is none is worth stating: `RigidBodyT::material_index`
             * exists so a *device* kernel can reach a material without following a pointer,
             * and the manifold pass that resolves a contact's surfaces runs on the host,
             * where the body's own record is already in hand. A table would be a second
             * place a material could live and a second thing to keep in step with the first.
             */
            Physics::PhysicsMaterialT<Scalar> material{};
        };

        /** @brief A cloth grid to (re)build, addressed by its owning entity. */
        struct ClothDescription
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
        struct PlaneDescription
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
         * @brief Gravitational acceleration sampled at a body's world position.
         *
         * The seam through which the live world hands the physics a *field* rather than one
         * scene-wide vector (dependency inversion — the solver names this abstraction, never
         * the astronomy behind it). Boundary `Scalar` in and out; the host samples the true
         * per-body field (falling off with altitude, curving toward the attractor) into each
         * body's own external acceleration. A uniform field is just a sampler that ignores
         * its argument.
         *
         * Sampled **once per body per tick**, not per sub-step. That is forced rather than
         * chosen: the prediction step runs on the device inside one composition, so there is
         * no point inside the sub-step loop at which a host sampler could be called at all.
         * A body travels at most the sub-step schedule's motion budget in a tick, over which
         * any field smooth enough to be worth sampling has not meaningfully changed.
         */
        using GravitySampler = std::function<Vector3(const Vector3& position)>;

        /**
         * @brief The air's own velocity sampled at a body's world position, in m/s.
         *
         * §11.6's cross-system tie-in, and §4.5's second example of the same principle
         * @ref GravitySampler is the first of: *"vehicle drag and downforce, and cloth and
         * rope wind response, sample it through a `WindSampler` seam that mirrors the
         * existing `GravitySampler` exactly — the physics names the abstraction, never the
         * meteorology behind it. A flag on a pole in a storm and a car's high-speed lift
         * come from the same field."*
         *
         * It is deliberately the same shape as @ref GravitySampler down to the signature,
         * because the two are the same *kind* of thing: a field the live world knows about
         * and the solver must not. Still air is a sampler that returns zero, which is what
         * every scene without weather installed gets and costs it nothing.
         *
         * Sampled once per body per tick, for @ref GravitySampler's reason — `predict` runs
         * on the device and there is no point inside the substep loop at which a host
         * sampler could be called.
         */
        using WindSampler = std::function<Vector3(const Vector3& position)>;

        /**
         * @brief Rigid bodies: what exists, what it weighs, and where it ended up.
         *
         * The service a gameplay system that moves or reads an object depends on, and
         * the only one it should have to.
         */
        class IRigidBodyService
        {
            public:
                virtual ~IRigidBodyService() = default;

                /**
                 * @brief Reconciles the rigid-body world with @p bodies.
                 *
                 * A diff, not a rebuild: an entity that was already here keeps its
                 * body and its live velocity, one that has gone is removed with its
                 * constraints, and a new one seeds from its descriptor pose, at rest.
                 * The solve graph does not change shape for any of it — that is what
                 * the mutable world is for. Call whenever the set of bodies changes.
                 *
                 * @param bodies     The full set of rigid bodies after the change.
                 * @param iterations Ignored. The sub-step schedule subsumes it: this
                 *                   solver takes small steps rather than many
                 *                   iterations, and honouring both would be two dials
                 *                   controlling one quantity.
                 * @param substep_dt The fixed sub-step duration, in seconds.
                 */
                virtual void set_rigid_bodies(const std::vector<RigidBodyDescription>& bodies,
                                              std::size_t iterations, Scalar substep_dt) = 0;

                /**
                 * @brief Updates a live body's mass/inertia/drag without a rebuild; a no-op if absent.
                 * @param id               The entity whose body to update.
                 * @param inv_mass         New inverse mass.
                 * @param inv_inertia      New diagonal body-local inverse inertia.
                 * @param drag_coefficient New quadratic drag coefficient.
                 */
                virtual void update_rigid_body_params(EntityId id, Scalar inv_mass,
                                                      const Vector3& inv_inertia,
                                                      Scalar drag_coefficient) = 0;

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
                 * there.
                 *
                 * @param id          The entity whose body to move.
                 * @param position    The new world position.
                 * @param orientation The new world orientation.
                 */
                virtual void set_rigid_pose(EntityId id, const Vector3& position,
                                            const Quaternion& orientation) = 0;

                /**
                 * @brief Reads what a body is doing, for a debug view.
                 * @param id  The entity whose body to read.
                 * @param out Receives the state when @p id has a body.
                 * @return Whether @p id owns a rigid body (and @p out was written).
                 */
                virtual bool rigid_debug_state(EntityId id, RigidDebugState& out) const = 0;
        };

        /** @brief Cloth grids: topology in, particle positions out. */
        class IClothService
        {
            public:
                virtual ~IClothService() = default;

                /**
                 * @brief Rebuilds the cloth world from @p grids.
                 *
                 * Unlike a rigid-body rebuild no live state is carried over: a rows,
                 * columns or spacing change replaces the grid topology outright, so
                 * there is nothing meaningful to preserve.
                 *
                 * @param grids      The full set of cloth grids after the change.
                 * @param iterations Gauss-Seidel sweeps per sub-step.
                 * @param substep_dt The fixed sub-step duration, in seconds.
                 */
                virtual void set_cloth_grids(const std::vector<ClothDescription>& grids,
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
        };

        /**
         * @brief A tetrahedral soft body to (re)build, addressed by its owning entity.
         *
         * The asset is passed as bytes rather than as a path or a handle because this
         * header names no file system and no cache: where a `.sushisoft` blob came from
         * is the caller's business, and by the time it gets here it is either a valid
         * blob or nothing. The pointer is read during the `set_soft_bodies` call and
         * not retained — instantiation copies out everything the solve needs, so the
         * caller may free the bytes the moment the call returns.
         */
        struct SoftBodyDescription
        {
            EntityId id = NULL_ENTITY;        /**< The entity that owns this body. */
            const std::byte* asset = nullptr; /**< A `.sushisoft` blob; not retained past the call. */
            std::size_t asset_size = 0;       /**< Length of @ref asset in bytes. */
            std::uint32_t level = 0;          /**< Which cooked simulation level to build (0 is finest). */
            Vector3 origin;                   /**< World position of the asset's local origin. */
            Physics::SoftBodyMaterialT<Scalar> material{}; /**< Constitutive parameters (§9.2). */
            Scalar thickness = Scalar(0.01);  /**< The half-width the surface presents to a contact. */
            bool self_collision = false;      /**< Whether the surface is tested against itself (§9.6.3). */
            bool cosmetic = false;            /**< Asks for the narrow column; only a request (§6.5). */
            bool participates_in_rollback = false; /**< Overrides @ref cosmetic outright. */
        };

        /**
         * @brief Tetrahedral soft bodies: a cooked asset in, a deformed surface out.
         *
         * Separate from @ref IClothService rather than folded into it, because the two
         * have almost nothing in common on this side of the seam. A cloth grid is
         * described by three numbers and lives in the rigid solver as pinned particles;
         * a soft body is described by a cooked asset it cannot be reconstructed without,
         * and lives in its own world with its own contact machinery. Merging them would
         * produce an interface where half the methods return nothing for half the
         * callers — the shape §4.3 exists to prevent.
         */
        class ISoftBodyService
        {
            public:
                virtual ~ISoftBodyService() = default;

                /**
                 * @brief Rebuilds the soft-body world from @p bodies.
                 *
                 * Wholesale, like cloth and unlike rigid bodies, and for the same reason:
                 * a body whose asset or level changed has a different particle count and
                 * a different element list, so there is no state to carry across. A set
                 * that has not changed is left alone, so the common case costs a
                 * comparison.
                 *
                 * @param bodies The full set of soft bodies after the change.
                 */
                virtual void set_soft_bodies(const std::vector<SoftBodyDescription>& bodies) = 0;

                /**
                 * @brief A soft body's deformed surface, as of the last completed tick.
                 *
                 * Positions are every particle, and the indices address them directly, so
                 * the pair is a self-contained triangle mesh — which is what §8.6's third
                 * invariant needs: the render mesh is *derived from* the simulated state
                 * rather than synchronised with it, so it cannot lag it.
                 *
                 * @param id           The entity whose body to read.
                 * @param positions    Receives the world-space particle positions; cleared first.
                 * @param indices      Receives the surface triangle list; cleared first.
                 * @return Whether @p id owns a soft body (and the outputs were written).
                 */
                virtual bool soft_body_surface(EntityId id, std::vector<Vector3>& positions,
                                               std::vector<std::uint32_t>& indices) const = 0;

                /**
                 * @brief The largest von Mises stress in a body, from its last tick (§9.3).
                 * @param id The entity whose body to read.
                 * @return Zero when @p id owns no soft body.
                 */
                virtual Scalar soft_body_maximum_stress(EntityId id) const = 0;

                /**
                 * @brief Every element of a body, with its last tick's two readouts.
                 *
                 * The interior, which @ref soft_body_surface deliberately does not show: a
                 * body's surface can look untouched while the elements behind it are past
                 * yield, and that gap is exactly what a debug view exists to close.
                 *
                 * @param id       The entity whose body to read.
                 * @param elements Receives one entry per tetrahedron; cleared first.
                 * @return Whether @p id owns a soft body.
                 */
                virtual bool soft_body_elements(
                    EntityId id, std::vector<SoftBodyElementSample>& elements) const = 0;
        };

        /** @brief The immovable surfaces everything else is pushed out of. */
        class IStaticGeometryService
        {
            public:
                virtual ~IStaticGeometryService() = default;

                /**
                 * @brief Sets the static planes bodies and cloth collide against.
                 *
                 * Cheap to call every tick (it just replaces a small list), so a host
                 * re-supplies the scene's ground/ramp planes each step rather than
                 * tracking when they move. An empty list disables plane contacts.
                 *
                 * @param planes The static collision planes, in world space.
                 */
                virtual void set_static_planes(const std::vector<PlaneDescription>& planes) = 0;
        };

        /**
         * @brief What a scene query hit, in boundary precision.
         *
         * `entity` rather than a body handle, because the caller of a query is a
         * gameplay system and what it wants back is the thing it already names.
         */
        struct SceneRayHit
        {
            bool hit = false;
            EntityId entity = NULL_ENTITY;
            Vector3 point;                    /**< The world point of impact. */
            Vector3 normal{Vector3{0, 1, 0}}; /**< The outward surface normal there. */
            Scalar distance = 0;              /**< Along the ray, from its origin. */
        };

        /**
         * @brief Which entities a query is willing to see.
         *
         * The layer mask is the same word bodies carry, so "the camera ignores the
         * player" is authored once and honoured by contacts and queries alike. The
         * exclusion is the common special case of §7.7's optional predicate, kept
         * as data so this header stays free of a callback type in a struct that
         * crosses a precision boundary.
         */
        struct SceneQueryFilter
        {
            std::uint32_t layer_mask = 0xFFFFFFFFu;
            /** @brief Skipped whatever its layer — usually the body that fired the query. */
            EntityId exclude = NULL_ENTITY;
            /** @brief Whether shapes flagged as triggers answer this query. */
            bool include_triggers = true;
        };

        /**
         * @brief What touched what this tick.
         *
         * Its own service for §4.3's reason: a door that opens when something stands
         * on a pressure plate, an impact sound, a damage volume and a checkpoint
         * trigger are all this interface and none of them has any business depending
         * on rigid-body lifecycle or on the stepper to get an answer.
         *
         * The list is rebuilt every tick and is ordered deterministically — by the
         * colliders involved, never by the order the broadphase happened to find
         * them — because a listener that spawns an effect must not spawn it in a
         * different order on a different machine (§12.1).
         */
        class IContactEventService
        {
            public:
                virtual ~IContactEventService() = default;

                /**
                 * @brief This tick's contact events, valid until the next @ref
                 *        IPhysicsStepper::step.
                 */
                virtual const std::vector<ContactEvent>& contact_events() const noexcept = 0;
        };

        /**
         * @brief A joint to create between two entities that already own rigid bodies.
         *
         * Both endpoints are bodies. An immovable endpoint is a body with zero inverse
         * mass, not a missing one — which keeps every joint two-sided and stops a
         * one-sided projection existing to disagree with the two-sided one.
         */
        struct JointDescription
        {
            EntityId body_a = NULL_ENTITY;
            EntityId body_b = NULL_ENTITY;

            /** @brief What is held between them. */
            JointParameters params;
        };

        /** @brief An opaque identity for a live joint; zero names none. */
        using JointId = std::uint32_t;

        /** @brief The joint identity that never names a joint. */
        inline constexpr JointId NULL_JOINT = 0;

        /**
         * @brief A joint that exceeded a break threshold and is gone.
         *
         * Reported once, on the tick it broke, after which the joint no longer exists
         * and its bodies are free. The load it was carrying is included because that is
         * what a listener wants: how hard the impact was, not merely that there was one.
         */
        struct JointBrokenEvent
        {
            JointId joint = NULL_JOINT;
            EntityId a = NULL_ENTITY;
            EntityId b = NULL_ENTITY;
            /** @brief The peak force magnitude that broke it, in newtons. */
            Scalar force = 0;
            /** @brief The peak torque magnitude that broke it, in newton-metres. */
            Scalar torque = 0;
        };

        /**
         * @brief A hybrid vehicle to (re)build, addressed by its owning entity.
         *
         * The asset is bytes for the same reason `SoftBodyDescription`'s is: this header names no
         * file system and no cache, and where a `.sushinodebeam` blob came from is the
         * caller's business. Read during the `set_vehicles` call and not retained —
         * instancing copies out everything the solve needs.
         *
         * The *vehicle* asset beside it is not bytes, because it is not a cooked blob: the
         * corners, the tyres, the drivetrain and the aerodynamics are authored numbers, and
         * §11's whole split is that the structure is cooked and the setup is not.
         */
        struct VehicleDescription
        {
            EntityId id = NULL_ENTITY;        /**< The entity that owns this vehicle. */
            const std::byte* asset = nullptr; /**< A `.sushinodebeam` blob; not retained. */
            std::size_t asset_size = 0;       /**< Length of @ref asset in bytes. */
            Vector3 position;                 /**< World position of the structure's origin. */
            Quaternion orientation;           /**< World orientation about that origin. */
            Vector3 velocity;                 /**< Speed every body starts with, in m/s. */

            /** @brief The authored setup: corners, tyres, drivetrain, aerodynamics (§11). */
            Physics::VehicleAssetT<Scalar> setup{};
        };

        /**
         * @brief Vehicles: a cooked structure in, a drivable body out.
         *
         * Its own service for §4.3's reason, and a sharp one: a vehicle needs the node-beam
         * structure, the suspension, the drivetrain and the tyres, and *nothing else in a
         * game does*. Folding this into the rigid-body service would make every consumer
         * that wants a solved pose depend on §11 as well.
         */
        class IVehicleService
        {
            public:
                virtual ~IVehicleService() = default;

                /**
                 * @brief Reconciles the vehicle world with @p vehicles.
                 *
                 * A rebuild rather than a diff, unlike `set_rigid_bodies`, and the reason is
                 * worth stating: a vehicle is four hundred bodies, two thousand beams and a
                 * drivetrain whose every part is placed relative to a cooked structure, so
                 * "the same vehicle with one number changed" is not a thing that can be
                 * patched in place. Called only when the *set* changes, which for vehicles
                 * is rare — an author placing one, not a frame passing.
                 *
                 * @param vehicles The full set of vehicles after the change.
                 */
                virtual void set_vehicles(const std::vector<VehicleDescription>& vehicles) = 0;

                /**
                 * @brief Records what the driver is asking for; spent by the next step.
                 * @param id    The entity whose vehicle to drive.
                 * @param input The controls.
                 * @return Whether @p id owns a vehicle.
                 */
                virtual bool set_vehicle_input(EntityId id, const VehicleInput& input) = 0;

                /**
                 * @brief Reads what the drivetrain did last step.
                 * @param id  The entity whose vehicle to read.
                 * @param out Receives the report when @p id owns a vehicle.
                 * @return Whether @p out was written.
                 */
                virtual bool vehicle_report(EntityId id, VehicleReport& out) const = 0;

                /**
                 * @brief Reads the vehicle's rigid core pose, which is where the vehicle *is*.
                 *
                 * The core rather than a node, because the core is the one body whose pose
                 * is the vehicle's own: §11.2's hybrid puts the mass and the inertia there
                 * and hangs a deformable shell off it, so a node's position is a panel's
                 * position and only the core's is the car's.
                 *
                 * @param id  The entity whose vehicle to read.
                 * @param out Receives the pose when @p id owns a vehicle with a core.
                 * @return Whether @p out was written.
                 */
                virtual bool vehicle_core_pose(EntityId id, SolvedPose& out) const = 0;

                /**
                 * @brief Reads the world positions of the vehicle's shell nodes.
                 *
                 * For the editor's node/beam view (§14) and for anything that draws a
                 * deformed body. Filled rather than returned, so a caller drawing every
                 * frame reuses its buffer instead of allocating one per frame per vehicle.
                 *
                 * @param id  The entity whose vehicle to read.
                 * @param out Receives one position per node.
                 * @return Whether @p id owns a vehicle.
                 */
                virtual bool vehicle_node_positions(EntityId id,
                                                    std::vector<Vector3>& out) const = 0;

                /**
                 * @brief The shell's collision surface, live, as triangles.
                 *
                 * The same shape `ISoftBodyService::soft_body_surface` produces and for the
                 * same consumer: a deformable mesh the renderer draws. What is drawn is the
                 * surface the vehicle *collides* as, read straight off the live node bodies
                 * with no cache in between — so a dented panel is dented on screen in the
                 * tick it was dented, and the drawing cannot disagree with the collision,
                 * because they are the same triangles.
                 *
                 * The cooked asset also carries a per-vertex skinning for a separate visual
                 * mesh (`NodeBeamSkinRecord`), which is the prettier answer and needs that
                 * mesh's own index buffer — which lives in the visual asset, not here. This
                 * is the surface the physics owns end to end.
                 *
                 * @param id        The entity whose vehicle to read.
                 * @param positions Receives one position per node.
                 * @param indices   Receives the surface triangles, indexing @p positions.
                 * @return Whether @p id owns a live vehicle with a surface.
                 */
                virtual bool vehicle_surface(EntityId id, std::vector<Vector3>& positions,
                                             std::vector<std::uint32_t>& indices) const = 0;
        };

        /**
         * @brief Joints: what is attached to what, how hard it is being pulled, and when it gives.
         *
         * Its own service because a mechanism, a vehicle, a ragdoll and a destructible
         * mount are all this interface and none of them needs the stepper, the
         * broadphase, or cloth to ask it anything. A door that reports its hinge load
         * and tears off above a threshold depends on this and on nothing else.
         */
        class IJointService
        {
            public:
                virtual ~IJointService() = default;

                /**
                 * @brief Creates a joint between two entities that already own bodies.
                 *
                 * @param desc What to create.
                 * @return Its identity, or @ref NULL_JOINT when either entity has no
                 *         body or the joint budget is exhausted — a budget being
                 *         exceeded, counted in the statistics, not an error.
                 */
                virtual JointId create_joint(const JointDescription& desc) = 0;

                /**
                 * @brief Destroys a joint. What breaking one actually does.
                 * @param joint The joint to destroy.
                 * @return True when a live joint was destroyed by this call.
                 */
                virtual bool destroy_joint(JointId joint) = 0;

                /**
                 * @brief The load the last step left on a joint.
                 * @param joint The joint to read.
                 * @param out   Receives the load when @p joint is live.
                 * @return True when @p joint named a live joint.
                 */
                virtual bool joint_state(JointId joint, JointState& out) const = 0;

                /**
                 * @brief Replaces a joint's drive, live.
                 * @param joint The joint to change.
                 * @param motor The drive to install.
                 * @return True when @p joint named a live joint.
                 */
                virtual bool set_joint_motor(JointId joint, const JointMotorDescription& motor) = 0;

                /**
                 * @brief Replaces a joint's three limits, live.
                 *
                 * All three at once rather than one call per limit: they are one
                 * statement about what the joint allows, and a seam with three setters
                 * is a seam where two of them can be forgotten.
                 *
                 * @param joint  The joint to change.
                 * @param linear The travel or range limit.
                 * @param twist  The rotation limit about the primary axis.
                 * @param swing  The cone limit off the primary axis.
                 * @return True when @p joint named a live joint.
                 */
                virtual bool set_joint_limits(JointId joint, const JointLimitDescription& linear,
                                              const JointLimitDescription& twist,
                                              const JointLimitDescription& swing) = 0;

                /**
                 * @brief The joints that broke during the last step.
                 *
                 * Valid until the next @ref IPhysicsStepper::step, and ordered by
                 * joint identity so a listener that spawns an effect observes the same
                 * sequence on every machine (§12.1).
                 */
                virtual const std::vector<JointBrokenEvent>& joint_broken_events()
                    const noexcept = 0;
        };

        /**
         * @brief Asking the collision world a question that is not "what is touching".
         *
         * §7.7's service. A weapon, a camera, a footstep probe, a placement tool and
         * a line of sight are all this interface, and none of them has any business
         * depending on the stepper or on rigid-body lifecycle to get an answer —
         * which is the whole argument of §4.3 restated for the one service that most
         * of a game actually calls.
         */
        class ICollisionQueryService
        {
            public:
                virtual ~ICollisionQueryService() = default;

                /**
                 * @brief The nearest thing a ray hits.
                 * @param origin       Where the ray starts, in world space.
                 * @param direction    Its direction; normalized by the implementation.
                 * @param max_distance How far to look, in metres.
                 * @param filter       Which entities to consider.
                 * @return The nearest hit, or one with `hit == false`.
                 */
                virtual SceneRayHit raycast_closest(const Vector3& origin,
                                                    const Vector3& direction, Scalar max_distance,
                                                    const SceneQueryFilter& filter) const = 0;

                /** @brief Everything a ray hits, nearest first. */
                virtual std::vector<SceneRayHit> raycast_all(
                    const Vector3& origin, const Vector3& direction, Scalar max_distance,
                    const SceneQueryFilter& filter) const = 0;

                /**
                 * @brief Every entity whose shape a sphere actually overlaps.
                 *
                 * "Actually" is the difference between this and a bounding-box test,
                 * and it is what a trigger volume needs: one that fired on boxes is
                 * one that fires in the doorway next to it.
                 */
                virtual std::vector<EntityId> overlap_sphere(
                    const Vector3& center, Scalar radius,
                    const SceneQueryFilter& filter) const = 0;

                /**
                 * @brief Moves a sphere until something stops it.
                 * @param center       Where the sphere starts.
                 * @param radius       Its radius.
                 * @param direction    Direction of travel; normalized by the implementation.
                 * @param distance     How far to try to move.
                 * @param filter       Which entities to consider.
                 * @return The first blocking hit; `distance` is the travel before contact.
                 */
                virtual SceneRayHit sweep_sphere(const Vector3& center, Scalar radius,
                                                 const Vector3& direction, Scalar distance,
                                                 const SceneQueryFilter& filter) const = 0;

                /**
                 * @brief The nearest surface point to @p point, within @p max_distance.
                 * @param point        The world point to measure from.
                 * @param max_distance How far to search.
                 * @param filter       Which entities to consider.
                 * @return The hit, whose `point` and `normal` are on the found surface
                 *         and whose `distance` is the gap; `hit` is false if nothing is
                 *         in range.
                 */
                virtual SceneRayHit closest_point(const Vector3& point, Scalar max_distance,
                                                  const SceneQueryFilter& filter) const = 0;
        };

        /** @brief Advancing time, and reporting what that cost. */
        class IPhysicsStepper
        {
            public:
                virtual ~IPhysicsStepper() = default;

                /**
                 * @brief Advances everything by one tick.
                 *
                 * Rigid bodies and cloth are one body set in one solver, so there is
                 * no ordering between them to get right: contacts — body to body,
                 * body to cloth, and against static geometry — are constraints
                 * resolved inside the sub-step loop alongside every other kind.
                 *
                 * @param gravity  The per-body gravitational field, sampled at each
                 *                 body's position once this tick.
                 * @param wind     The per-body wind field (§11.6), sampled the same way.
                 *                 Still air is a sampler returning zero; an empty one is
                 *                 the same thing and is what a scene with no weather
                 *                 installed passes.
                 * @param substeps A **floor** under the sub-step count, not the count
                 *                 itself. The count is derived from simulation state,
                 *                 because a caller setting it outright would make the
                 *                 simulation depend on something outside its own
                 *                 state; what a caller legitimately knows is the
                 *                 quality it is willing to pay for whatever the state
                 *                 says. State may raise the count, the caller may
                 *                 raise the floor, and neither lowers what the other
                 *                 asked for.
                 */
                virtual void step(const GravitySampler& gravity, const WindSampler& wind,
                                  std::size_t substeps) = 0;

                /**
                 * @brief What the last @ref step contained and what it cost.
                 *
                 * Reported through the stepper rather than fetched from the solver,
                 * because a consumer holding only this service is exactly the one that
                 * wants the number — a profiler panel steps nothing and reads
                 * everything.
                 *
                 * @return The statistics for the most recent step.
                 */
                virtual const Physics::PhysicsStatistics& statistics() const noexcept = 0;

                /**
                 * @brief Requests per-stage timing collection (the profiler panel's seam).
                 *
                 * Profiling is part of the solver's *construction*
                 * (`PhysicsConfiguration::profiling` — off, the dispatch hot path reads
                 * no timestamps at all), so this is a request consumed when the solver
                 * is next built, not a live toggle on a running one: open the panel
                 * before the scene's physics first steps and the timings flow. Default
                 * implementation ignores the request, for steppers with nothing to time.
                 *
                 * @param enabled Whether solvers built from now on should collect timings.
                 */
                virtual void set_profiling_requested(bool enabled) { (void)enabled; }

                /**
                 * @brief Requests that a joint whose island is asleep be dropped from
                 * the solve graph rather than dispatched for its projection to early
                 * out on the same sleeping check (§16.44).
                 *
                 * Unlike @ref set_profiling_requested this is a live toggle: parking
                 * is simulation-tick state, not solver-construction state, so it takes
                 * effect from the next `step()` rather than the next solver rebuild.
                 * Off by default. Default implementation ignores the request, for
                 * steppers with nothing to park.
                 *
                 * @param enabled Whether a sleeping joint should be parked from now on.
                 */
                virtual void set_park_sleeping_joints_requested(bool enabled)
                {
                    (void)enabled;
                }
        };

        /**
         * @brief Every physics service at once, for the object that owns the simulation.
         *
         * The live world holds one of these because it genuinely does every one of these
         * jobs. Nothing else should: a system that raycasts wants a query service, a
         * panel that draws a graph wants a stepper, a mechanism wants the joint
         * service, and depending on this instead is how the split gets undone one call
         * site at a time.
         */
        class IPhysicsScene : public IRigidBodyService,
                              public IClothService,
                              public ISoftBodyService,
                              public IJointService,
                              public IVehicleService,
                              public IStaticGeometryService,
                              public ICollisionQueryService,
                              public IContactEventService,
                              public IPhysicsStepper
        {
            public:
                ~IPhysicsScene() override = default;
        };
    } // namespace Simulation
} // namespace SushiEngine
