/**************************************************************************/
/* simulation.hpp                                                         */
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
 * @file simulation.hpp
 * @brief The plain-C++ seam between a host (the editor) and a live ECS world.
 *
 * The world is ticked on SushiRuntime with SYCL kernels, but none of that leaks
 * across this interface: `ISimulation` names no runtime, SYCL, or ECS types, only
 * the value types from the BLAS seam. The implementation lives in one compiled
 * library (`sushi_sim`) so device code stays contained in a single translation
 * unit, and a host depends only on this abstraction (dependency inversion) — a
 * different world backend, or a headless stub, can replace it without the host
 * changing. Each frame the host `tick()`s the world and reads the extracted
 * `RenderScene` snapshot to draw it.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace sim
    {
        /**
         * @brief A stable, editor-facing handle to one world entity.
         *
         * Assigned by the simulation and constant for the entity's lifetime, unlike
         * the ECS's internal generation-checked handle. Zero is the null id (no
         * entity), so a host can use it as an unselected sentinel.
         */
        using EntityId = std::uint64_t;

        /** @brief The null entity id; no entity carries it. */
        constexpr EntityId NULL_ENTITY = 0;

        /** @brief An entity's authorable transform, as the inspector edits it. */
        struct EntityTransform
        {
            Vec3 position;               /**< World position. */
            Quat rotation;               /**< Orientation. */
            Vec3 scale{Vec3{1, 1, 1}};   /**< Per-axis scale. */
        };

        /** @brief One drawable object extracted from the world: identity, transform, colour. */
        struct RenderInstance
        {
            EntityId id = NULL_ENTITY; /**< The entity this instance draws, for picking. */
            Mat4 model;                /**< Object-to-world transform composed from the entity's state. */
            Vec3 color;                /**< Base colour. */
        };

        /**
         * @brief A camera defined in world terms, resolved to matrices by the viewer.
         *
         * The aspect ratio belongs to the panel the camera is drawn into, not the
         * world, so the snapshot carries the field of view and clip planes and the
         * eye/target/up frame; the viewer builds view and projection at its own size.
         */
        struct CameraState
        {
            Vec3 position;                  /**< Eye position in world space. */
            Vec3 target;                    /**< Point the camera looks at. */
            Vec3 up;                        /**< World up direction. */
            Scalar vertical_fov_radians = 1;/**< Vertical field of view in radians. */
            Scalar near_plane = Scalar(0.1);/**< Near clip distance (> 0). */
            Scalar far_plane = Scalar(500); /**< Far clip distance (> near). */
        };

        /**
         * @brief The authorable parameters of a camera entity.
         *
         * The lens (fov, clip planes) plus how the camera is routed: which display it
         * drives, its priority among cameras on that display, and whether it is active.
         * The eye/target/up frame is not here — it comes from the entity's transform.
         */
        struct CameraParams
        {
            Scalar vertical_fov_radians = Scalar(1.0471976); /**< Vertical FOV in radians. */
            Scalar near_plane = Scalar(0.1);                 /**< Near clip distance (> 0). */
            Scalar far_plane = Scalar(500);                  /**< Far clip distance (> near). */
            std::uint32_t display_index = 0;                 /**< Which display this camera drives. */
            std::int32_t priority = 0;                       /**< Higher wins on a shared display. */
            bool active = true;                              /**< Whether it contributes at all. */
        };

        /**
         * @brief The authorable parameters of a "Rigid Body" entity.
         *
         * Mirrors `Physics::RigidBody`'s mass/inertia, in editor-facing form: no
         * position/orientation (the entity's `Transform`/`Orientation` already carry
         * those) and no simulated velocity (that lives in the physics world, not in
         * anything the Inspector authors).
         */
        struct PhysicsBodyParams
        {
            Scalar inv_mass = Scalar(1);  /**< Inverse mass; 0 pins the body in place. */
            Vec3 inv_inertia{0, 0, 0};    /**< Diagonal body-local inverse inertia; 0 = no rotation response. */
        };

        /** @brief The resolved camera for one display: the winner among its cameras. */
        struct DisplayCamera
        {
            std::uint32_t display = 0; /**< The display index this camera drives. */
            CameraState state;         /**< The pose and lens to render it with. */
        };

        /**
         * @brief A read-only snapshot of the world for one frame.
         *
         * Rebuilt by the simulation's extract step after every `tick()`; the host
         * reads it but never mutates the world through it. A host copy today — the
         * zero-copy path that shares the world's device columns with the renderer is
         * a later interop milestone.
         */
        struct RenderScene
        {
            std::vector<RenderInstance> instances;       /**< Every drawable object this frame. */
            std::vector<DisplayCamera> display_cameras;  /**< The resolved camera per display. */
            CameraState camera;                          /**< The default game camera (lowest display), only meaningful when @ref has_camera is true. */
            bool has_camera = false;                     /**< Whether any active camera resolved this frame; false means nothing should be drawn as "the game". */
        };

        /**
         * @brief The editor's read/write surface onto the live world.
         *
         * The query and mutation half of the seam, split from `ISimulation` so a panel
         * that only inspects or edits entities depends on this narrow interface, not on
         * the stepping engine (interface segregation). Entities are addressed by their
         * stable `EntityId`; every mutation is applied to the world so the next
         * extracted `RenderScene` reflects it. Names and visibility are editor metadata
         * the simulation keeps host-side; transform and colour are backed by real ECS
         * components.
         */
        class IWorldEditor
        {
            public:
                virtual ~IWorldEditor() = default;

                /** @brief The live entities in display order. */
                virtual std::vector<EntityId> entities() const = 0;

                /** @brief Whether @p id names a live entity. */
                virtual bool exists(EntityId id) const noexcept = 0;

                /** @brief The entity's display name (empty if it does not exist). */
                virtual std::string name(EntityId id) const = 0;

                /** @brief The entity's transform (identity if it does not exist). */
                virtual EntityTransform transform(EntityId id) const = 0;

                /** @brief The entity's base colour (zero if it does not exist). */
                virtual Vec3 color(EntityId id) const = 0;

                /** @brief Whether the entity is drawn. */
                virtual bool visible(EntityId id) const noexcept = 0;

                /**
                 * @brief Creates a static entity at the origin and selects nothing.
                 *
                 * The new entity carries no motion, so it stays where it is placed and
                 * edited even while the world is playing (unlike the seeded demo cubes,
                 * which their systems drive).
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create(const std::string& name) = 0;

                /** @brief Destroys @p id; a no-op if it does not exist. */
                virtual void destroy(EntityId id) = 0;

                /** @brief Sets the entity's display name. */
                virtual void set_name(EntityId id, const std::string& name) = 0;

                /** @brief Writes the entity's transform components. */
                virtual void set_transform(EntityId id, const EntityTransform& transform) = 0;

                /** @brief Writes the entity's base colour. */
                virtual void set_color(EntityId id, const Vec3& color) = 0;

                /** @brief Sets whether the entity is drawn. */
                virtual void set_visible(EntityId id, bool visible) = 0;

                /**
                 * @brief Whether the entity carries a Renderer component.
                 *
                 * Mirrors Unity's MeshRenderer: an entity always has a Transform, but
                 * only draws (and has a `color()`) when a Renderer is attached.
                 */
                virtual bool has_renderer(EntityId id) const noexcept = 0;

                /**
                 * @brief Attaches or detaches the Renderer component.
                 *
                 * Attaching gives the entity a default colour; detaching stops it being
                 * drawn. A no-op on an entity whose component set cannot be changed
                 * (e.g. the seeded, animated demo cubes).
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Renderer after this call.
                 */
                virtual void set_has_renderer(EntityId id, bool value) = 0;

                /** @brief The entity's parent, or `NULL_ENTITY` if it is a root. */
                virtual EntityId parent(EntityId id) const noexcept = 0;

                /**
                 * @brief Reparents @p child under @p new_parent.
                 *
                 * Pass `NULL_ENTITY` to make @p child a root. A no-op if it would create a
                 * cycle (@p new_parent is @p child itself or one of its own descendants) —
                 * the caller does not need to check ancestry itself.
                 *
                 * @param child Entity being reparented.
                 * @param new_parent The new parent, or `NULL_ENTITY` for root.
                 */
                virtual void set_parent(EntityId child, EntityId new_parent) = 0;

                /**
                 * @brief Creates a camera entity: a pose plus a `CameraParams`.
                 *
                 * A camera is a first-class entity (it appears in the hierarchy and has a
                 * transform) but carries no mesh, so it is not drawn as an object; instead
                 * it contributes to the resolved `RenderScene::display_cameras`. Its lens
                 * defaults to a standard perspective on display 0.
                 *
                 * @param name Display name for the new camera.
                 * @return The new camera's stable id.
                 */
                virtual EntityId create_camera(const std::string& name) = 0;

                /** @brief Whether @p id is a camera entity. */
                virtual bool is_camera(EntityId id) const noexcept = 0;

                /** @brief The camera's parameters (defaults if @p id is not a camera). */
                virtual CameraParams camera_params(EntityId id) const = 0;

                /** @brief Writes a camera entity's parameters; a no-op for non-cameras. */
                virtual void set_camera_params(EntityId id, const CameraParams& params) = 0;

                /**
                 * @brief Attaches or detaches the Camera component on an existing entity.
                 *
                 * Unlike `create_camera` (which makes a new camera entity), this toggles
                 * the Camera component on @p id in place, so any entity can become — or
                 * stop being — a camera. Attaching gives it default lens parameters; a
                 * no-op on an entity whose component set cannot be changed.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Camera after this call.
                 */
                virtual void set_is_camera(EntityId id, bool value) = 0;

                /**
                 * @brief Whether @p id is driven by the physics world (a "Rigid Body").
                 *
                 * While attached, the simulation is the source of truth for the
                 * entity's `Transform`/`Orientation` — `set_transform` still writes
                 * through, but the next `tick()` overwrites it with the solved pose,
                 * the same way a Rigidbody in Unity overrides manual transform edits
                 * while simulating.
                 */
                virtual bool has_physics_body(EntityId id) const noexcept = 0;

                /** @brief The entity's authored mass/inertia (defaults if not a rigid body). */
                virtual PhysicsBodyParams physics_body_params(EntityId id) const = 0;

                /**
                 * @brief Writes a rigid body's mass/inertia; a no-op for non-rigid-bodies.
                 *
                 * Applied immediately to the live simulated body when one already
                 * exists, so dragging the Inspector's mass slider does not force a
                 * physics-world rebuild.
                 */
                virtual void set_physics_body_params(EntityId id,
                                                     const PhysicsBodyParams& params) = 0;

                /**
                 * @brief Attaches or detaches the physics simulation on an existing entity.
                 *
                 * Unlike Renderer/Camera, this needs no ECS component migration — the
                 * entity's pose stays owned by `Transform`/`Orientation`; attaching only
                 * starts (and detaching stops) a physics body tracking that pose. A
                 * body count change is a physics-world rebuild (deferred to the next
                 * `tick()`), analogous to how the ECS schedule recompiles only when its
                 * chunk set changes.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should be physics-driven after this call.
                 */
                virtual void set_has_physics_body(EntityId id, bool value) = 0;
        };

        /**
         * @brief A live world a host ticks and draws without seeing the runtime.
         *
         * Owns the ECS world, its schedule, and the SushiRuntime that executes it.
         * `tick()` advances the world one fixed step; `render_scene()` returns the
         * snapshot extracted after the most recent tick; `world()` is the editor's
         * read/write surface onto the same world.
         */
        class ISimulation
        {
            public:
                virtual ~ISimulation() = default;

                /**
                 * @brief Advances the world by one fixed simulation step.
                 *
                 * Runs the schedule on the runtime (the systems execute as SYCL
                 * kernels) and refreshes the render snapshot. The step is fixed and
                 * deterministic, so a host gates motion by choosing whether to call
                 * this (play/pause), not by scaling a delta into device code.
                 */
                virtual void tick() = 0;

                /** @brief The snapshot extracted after the most recent `tick()`. */
                virtual const RenderScene& render_scene() const noexcept = 0;

                /** @brief Number of live entities in the world. */
                virtual std::size_t entity_count() const noexcept = 0;

                /** @brief The editor's read/write surface onto this world. */
                virtual IWorldEditor& world() noexcept = 0;
        };

        /**
         * @brief Creates the runtime-backed live world.
         *
         * Brings up a SushiRuntime and an empty ECS world with no entities and no
         * scene loaded — the editor starts scene-less, matching a fresh project, and
         * populates the world only via `IWorldEditor` (New Entity, GameObject menu) or
         * by loading a `.sushiscene`. The only place the runtime is constructed for
         * the editor.
         *
         * @return An owned simulation; never null (throws on runtime bring-up failure).
         */
        std::unique_ptr<ISimulation> create_simulation();
    } // namespace sim
} // namespace SushiEngine
