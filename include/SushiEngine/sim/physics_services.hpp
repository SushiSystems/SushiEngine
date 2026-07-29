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
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/sim/collider.hpp>
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
         * @brief Gravitational acceleration sampled at a body's world position.
         *
         * The seam through which the live world hands the physics a *field* rather than one
         * scene-wide vector (dependency inversion — the solver names this abstraction, never
         * the astronomy behind it). Boundary `Scalar` in and out; the host samples the true
         * per-body field (falling off with altitude, curving toward the attractor) and the
         * solver evaluates it once per body per sub-step. A uniform field is just a sampler
         * that ignores its argument.
         */
        using GravitySampler = std::function<Vector3(const Vector3& position)>;

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
                virtual void set_static_planes(const std::vector<PlaneDesc>& planes) = 0;
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

        /** @brief Where in a contact's life an event was reported. */
        enum class ContactPhase : std::uint8_t
        {
            /** @brief The two surfaces met this tick and were not touching last tick. */
            Begin,
            /** @brief They were touching last tick and still are. */
            Persist,
            /** @brief They were touching last tick and are not now. */
            End
        };

        /**
         * @brief One thing touching another, reported to gameplay.
         *
         * Reported per *pair of colliders*, not per contact point: a crate landing
         * flat on the ground produces four points and one event, because "the crate
         * landed" is one thing that happened. The point and normal are the manifold's
         * deepest point, which is the one a sound, a decal or a damage number wants.
         *
         * A pair against static geometry has `b == NULL_ENTITY`. That is not a
         * missing value — static geometry is not an entity, and inventing one so the
         * field is never null would mean every listener filtering out a fiction.
         */
        struct ContactEvent
        {
            EntityId a = NULL_ENTITY;
            EntityId b = NULL_ENTITY;
            ContactPhase phase = ContactPhase::Begin;

            /** @brief The manifold's deepest point, in world space. */
            Vector3 point;

            /** @brief Unit normal, pointing from @ref a toward @ref b. */
            Vector3 normal{Vector3{0, 1, 0}};

            /**
             * @brief Total normal impulse the contact carried, in newton-seconds.
             *
             * What separates a scrape from a crash, and the reason the solved
             * manifolds are read back off the device at all. Zero on a trigger
             * overlap, which is detected and never resolved, and on an `End`, which
             * reports the impulse the contact carried on its last live tick.
             */
            Scalar impulse = 0;

            /**
             * @brief Whether one side was a trigger, so the pair was reported and not resolved.
             *
             * A separate flag rather than a separate event stream: a listener that
             * wants both — a damage volume that also pushes — should not have to
             * subscribe twice and correlate, and one that wants only triggers has a
             * one-line filter.
             */
            bool trigger = false;
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
                 * @brief Advances rigid and cloth by one outer step, resolving contacts.
                 *
                 * Each sub-step, after the constraint solve, rigid bodies are separated
                 * from each other and pushed out of the static planes, and cloth
                 * particles are pushed out of the planes and out of the rigid bodies
                 * (snapshotted as sphere obstacles) — so rigid bodies rest and stack and
                 * cloth drapes over them.
                 *
                 * @param gravity  The per-body gravitational field, sampled at each body's
                 *                 position every sub-step.
                 * @param substeps Number of sub-steps this step.
                 */
                virtual void step(const GravitySampler& gravity, std::size_t substeps) = 0;

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
        };

        /**
         * @brief Every physics service at once, for the object that owns the simulation.
         *
         * The live world holds one of these because it genuinely does all four jobs.
         * Nothing else should: a system that raycasts wants a query service, a panel
         * that draws a graph wants a stepper, and depending on this instead is how the
         * split gets undone one call site at a time.
         */
        class IPhysicsScene : public IRigidBodyService,
                              public IClothService,
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
