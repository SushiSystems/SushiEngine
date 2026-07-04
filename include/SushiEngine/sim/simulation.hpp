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
#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/sim/components.hpp>

namespace SushiEngine
{
    namespace Simulation
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
            Vector3 position;               /**< World position. */
            Quaternion rotation;               /**< Orientation. */
            Vector3 scale{Vector3{1, 1, 1}};   /**< Per-axis scale. */
        };

        /** @brief One drawable object extracted from the world: identity, transform, colour. */
        struct RenderInstance
        {
            EntityId id = NULL_ENTITY; /**< The entity this instance draws, for picking. */
            Mat4 model;                /**< Object-to-world transform composed from the entity's state. */
            Vector3 color;                /**< Base colour; also drives @ref material.albedo. */
            PrimitiveKind shape_kind = PrimitiveKind::Box; /**< Which mesh to draw this instance with. */
            Vector3 shape_params{Vector3{0.5, 0.5, 0.5}};     /**< Per-kind shape parameters, see @ref ShapeParams. */
            Render::Material material{}; /**< PBR metallic-roughness surface (albedo synced from @ref color). */
        };

        /** @brief One simulated cloth grid's world-space points, ready to draw. */
        struct ClothInstance
        {
            EntityId id = NULL_ENTITY;      /**< The entity this cloth grid draws, for picking. */
            std::uint32_t rows = 0;         /**< Grid rows. */
            std::uint32_t cols = 0;         /**< Grid columns. */
            std::uint32_t first_vertex = 0; /**< Offset of this grid's points into @ref RenderScene::cloth_vertices. */
            Vector3 color{Vector3{0.85, 0.85, 0.9}}; /**< Base colour; cloth entities carry no Tint yet, so this is a fixed default. */
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
            Vector3 position;                  /**< Eye position in world space. */
            Vector3 target;                    /**< Point the camera looks at. */
            Vector3 up;                        /**< World up direction. */
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
            Vector3 inv_inertia{0, 0, 0};    /**< Diagonal body-local inverse inertia; 0 = no rotation response. */
        };

        /**
         * @brief The authorable parameters of a "Cloth" entity.
         *
         * Mirrors `Physics::build_cloth_grid`'s arguments, minus `origin` (the
         * entity's `Transform::position` supplies that, the same way a Rigid Body's
         * starting pose comes from `Transform`/`Orientation` rather than its own
         * field) and minus the world the grid is built into (that is
         * `RuntimeSimulation`-internal, not authorable). Row 0 of the grid is always
         * pinned, matching `build_cloth_grid`'s only supported topology today —
         * pinning just the corners is not yet exposed (see ARCHITECTURE.md §4.2).
         */
        struct ClothParams
        {
            std::size_t rows = 4;      /**< Grid rows (>= 1); row 0 is pinned. */
            std::size_t cols = 4;      /**< Grid columns (>= 1). */
            Scalar spacing = Scalar(0.5); /**< Distance between adjacent grid points. */
            Scalar compliance = Scalar(0); /**< XPBD compliance of every constraint; 0 is rigid. */
        };

        /**
         * @brief The authorable parameters of a "Shape" entity: its visual mesh.
         *
         * `kind` is fixed at creation (set by which `create_*` primitive call made
         * the entity). `params` is editable and interpreted per `kind`: Box uses it
         * as half-extents; Sphere uses `params.x` as radius; Cylinder uses
         * `params.x` as radius and `params.y` as half-height. Plane is not a valid
         * Shape kind (Terrain uses a thin, flat Box for its visual instead).
         */
        struct ShapeParams
        {
            PrimitiveKind kind = PrimitiveKind::Box;
            Vector3 params{Vector3{0.5, 0.5, 0.5}};
        };

        /**
         * @brief The authorable parameters of a "Collider" entity: its collision volume.
         *
         * Pure authoring data today: no narrowphase or contact solver reads it yet,
         * so a Collider carries no mass or velocity of its own — whether it moves is
         * entirely determined by whether the same entity also has a Rigid Body.
         * A Collider with no Rigid Body (e.g. Terrain) is implicitly static and
         * gravity-exempt, since nothing integrates its pose. `kind`/`params` follow
         * the same convention as `ShapeParams`, plus Plane, which uses `params` as
         * the collider's local-space normal (default {0, 1, 0}). A Collider's kind
         * can be changed independently of any Shape on the same entity (e.g. a
         * box-shaped visual with a simpler sphere collider).
         */
        struct ColliderParams
        {
            PrimitiveKind kind = PrimitiveKind::Box;
            Vector3 params{Vector3{0.5, 0.5, 0.5}};
        };

        /**
         * @brief Which kind of UI node a UI entity is.
         *
         * A `Canvas` is the full-viewport root every other UI element lays out
         * inside; `Image` and `Panel` draw a filled rectangle; `Text` draws a
         * label; `Button` draws a filled rectangle with a centred label and
         * reacts to the pointer. Modelled the same way as `PrimitiveKind` — a
         * plain host-side tag on the entity's UI record, not an ECS component —
         * since no Schedule system reads it (see components.hpp).
         */
        enum class UIElementKind : std::uint32_t
        {
            Canvas,
            Panel,
            Image,
            Text,
            Button,
        };

        /**
         * @brief The authorable parameters of a UI entity: a UGUI RectTransform plus paint.
         *
         * The rect follows Unity's uGUI model exactly: `anchor_min`/`anchor_max`
         * are normalized [0,1] fractions of the parent rect, `pivot` is the
         * element's own normalized handle, and `anchored_position`/`size_delta`
         * are pixel offsets resolved against the anchored span (see
         * `SushiEngine::UI::resolve_rect`). A `Canvas` ignores the rect and fills
         * its viewport; its `size` doubles as the reference resolution for a future
         * scaler. `text` is an inline fixed buffer (like `UI::UIText`) so the whole
         * struct stays a trivially copyable value across the seam.
         */
        struct UIElementParams
        {
            UIElementKind kind = UIElementKind::Image;
            Scalar anchor_min_x = Scalar(0.5);   /**< Left anchor, fraction of parent width. */
            Scalar anchor_min_y = Scalar(0.5);   /**< Bottom anchor, fraction of parent height. */
            Scalar anchor_max_x = Scalar(0.5);   /**< Right anchor, fraction of parent width. */
            Scalar anchor_max_y = Scalar(0.5);   /**< Top anchor, fraction of parent height. */
            Scalar pivot_x = Scalar(0.5);        /**< Element pivot X, normalized. */
            Scalar pivot_y = Scalar(0.5);        /**< Element pivot Y, normalized. */
            Scalar position_x = Scalar(0);       /**< Anchored position X, pixels. */
            Scalar position_y = Scalar(0);       /**< Anchored position Y, pixels. */
            Scalar size_x = Scalar(160);         /**< Width added to the anchored span, pixels. */
            Scalar size_y = Scalar(40);          /**< Height added to the anchored span, pixels. */
            Vector3 color{Vector3{Scalar(0.9), Scalar(0.9), Scalar(0.9)}}; /**< Fill/text colour. */
            Scalar alpha = Scalar(1);            /**< Opacity in [0,1]. */
            Scalar font_size = Scalar(18);       /**< Text/label point size. */
            char text[64] = {};                  /**< Inline label text (Text/Button). */
        };

        /**
         * @brief One value of a user-defined "script" component field.
         *
         * A tagged union-by-convention: `kind` picks which of the payload members
         * is meaningful. This is the data-driven stand-in for a compiled
         * MonoBehaviour field — the engine has no scripting VM, so a custom
         * component is authoring data (a named, typed set of fields) the editor
         * lets a user attach, edit, and serialize, alongside a generated C++ system
         * stub they fill in. Unlike ECS components this is a boundary DTO, not a
         * device type, so it need not be trivially copyable.
         */
        enum class ScriptFieldKind : std::uint32_t
        {
            Float,
            Int,
            Bool,
            Vector3,
            Color,
            Text,
        };

        /** @brief One named, typed field of a script component instance. */
        struct ScriptField
        {
            std::string name;                       /**< Field name as authored. */
            ScriptFieldKind kind = ScriptFieldKind::Float;
            Scalar number = Scalar(0);              /**< Float/Int payload. */
            bool flag = false;                      /**< Bool payload. */
            Vector3 vector{};                       /**< Vector3/Color payload. */
            std::string text;                       /**< Text payload. */
        };

        /**
         * @brief One user-defined "script" component attached to an entity.
         *
         * Named by `type_name` (the script's class name) and carrying a flat list
         * of authored fields. Instances are stored per entity by the simulation the
         * same way cloth parameters are — plain host bookkeeping keyed on
         * `EntityId` — while the catalog of definitions lives in the editor.
         */
        struct ScriptComponent
        {
            std::string type_name;               /**< The script's type/class name. */
            std::vector<ScriptField> fields;     /**< Its authored fields, in order. */
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
            std::vector<ClothInstance> cloth_instances;  /**< Every simulated cloth grid this frame, as a wireframe topology. */
            std::vector<Vector3> cloth_vertices;         /**< World-space points for every @ref cloth_instances entry, concatenated. */
            Render::Environment environment;             /**< The sun, WGS84 planet, atmosphere, clouds, and stars lighting this frame. */
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

                /** @brief The entity's world transform. */
                virtual EntityTransform world_transform(EntityId id) const = 0;

                /** @brief The entity's base colour (zero if it does not exist). */
                virtual Vector3 color(EntityId id) const = 0;

                /** @brief The entity's PBR material (defaults if it does not exist). */
                virtual Render::Material material(EntityId id) const = 0;

                /** @brief The scene-global lighting environment (sun, planet, atmosphere). */
                virtual Render::Environment environment() const = 0;

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

                /**
                 * @brief Writes the entity's local transform component.
                 *
                 * @param id    Entity to update.
                 * @param local The new local transform.
                 */
                virtual void set_transform(EntityId id, const EntityTransform& local) = 0;

                /**
                 * @brief Sets the entity's world transform by recomputing its local transform.
                 *
                 * @param id    Entity to update.
                 * @param world The new world transform.
                 */
                virtual void set_world_transform(EntityId id, const EntityTransform& world) = 0;

                /** @brief Writes the entity's base colour. */
                virtual void set_color(EntityId id, const Vector3& color) = 0;

                /**
                 * @brief Writes the entity's PBR material.
                 *
                 * The material's @c albedo is kept in sync with the entity's base colour by
                 * the extract step, so authoring albedo here is overridden by @ref set_color;
                 * the metallic, roughness, and emissive fields are what this authors.
                 */
                virtual void set_material(EntityId id, const Render::Material& material) = 0;

                /** @brief Writes the scene-global lighting environment. */
                virtual void set_environment(const Render::Environment& environment) = 0;

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
                 * @brief Changes the display order of an entity relative to a target entity.
                 *
                 * @param id The entity to move.
                 * @param target The target entity to move relative to.
                 * @param insert_after If true, moves @p id after @p target. If false, moves @p id before @p target.
                 */
                virtual void move_entity(EntityId id, EntityId target, bool insert_after) = 0;

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

                /**
                 * @brief Whether @p id owns a simulated cloth grid.
                 *
                 * Like `has_physics_body`, cloth needs no ECS component migration —
                 * the grid is host-side bookkeeping keyed by `EntityId`, not a
                 * per-particle entity (see ARCHITECTURE.md §4.2). The entity's own
                 * `Transform`/`Orientation` are left alone; the grid's world positions
                 * are read separately via `cloth_particle_positions`.
                 */
                virtual bool has_cloth(EntityId id) const noexcept = 0;

                /** @brief The entity's authored cloth grid parameters (defaults if not cloth). */
                virtual ClothParams cloth_params(EntityId id) const = 0;

                /**
                 * @brief Writes a cloth entity's grid parameters; a no-op for non-cloth entities.
                 *
                 * Unlike a Rigid Body's mass/inertia, a parameter change here alters
                 * the grid's body count, so it is treated the same as attaching/
                 * detaching: applied lazily, on the next `tick()`, via a full rebuild.
                 */
                virtual void set_cloth_params(EntityId id, const ClothParams& params) = 0;

                /**
                 * @brief Attaches or detaches a simulated cloth grid on an existing entity.
                 *
                 * Deferred to the next `tick()`, same as `set_has_physics_body` — a
                 * grid (body/constraint count) change is a physics-world rebuild, not
                 * an immediate mutation.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should own a cloth grid after this call.
                 */
                virtual void set_has_cloth(EntityId id, bool value) = 0;

                /**
                 * @brief The cloth grid's current world-space particle positions.
                 *
                 * Row-major (`row * cols + col`, matching `Physics::ClothGrid`), read
                 * directly off the live simulated bodies; empty if @p id is not a
                 * cloth entity or its grid has not been built yet (the tick right
                 * after `set_has_cloth(id, true)`). This is the read-only seam a
                 * future debug draw or a real deforming mesh renderer would consume —
                 * neither exists yet, so today nothing draws the grid (see
                 * ARCHITECTURE.md §4.2).
                 */
                virtual std::vector<Vector3> cloth_particle_positions(EntityId id) const = 0;

                /**
                 * @brief Creates a Box entity: a Shape plus a matching Collider.
                 *
                 * The visual and the collider default to the same half-extents; either
                 * can be edited or removed independently afterward through
                 * `set_shape_params`/`set_has_collider`.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_box(const std::string& name) = 0;

                /** @brief Creates a Sphere entity: a Shape plus a matching Collider. See `create_box`. */
                virtual EntityId create_sphere(const std::string& name) = 0;

                /** @brief Creates a Cylinder entity: a Shape plus a matching Collider. See `create_box`. */
                virtual EntityId create_cylinder(const std::string& name) = 0;

                /**
                 * @brief Creates a flat Terrain entity: a large thin Box Shape plus a
                 * Plane Collider, with no physics body.
                 *
                 * Terrain never carries a Rigid Body, which is what makes it immune to
                 * gravity — nothing integrates its pose — while its `Collider` still
                 * marks it as a participant for a future rigidbody/softbody
                 * narrowphase, since collider data alone (not the absence of motion) is
                 * what that milestone will query.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_terrain(const std::string& name) = 0;

                /**
                 * @brief Creates a Cloth entity: an entity owning a simulated cloth grid.
                 *
                 * The grid hangs from its pinned top row at the entity's
                 * `Transform::position` with default rows/cols/spacing, so a freshly
                 * created cloth is visible as a flat resting sheet in edit mode and
                 * begins to drape as soon as the world is played. Equivalent to
                 * creating an empty entity and `set_has_cloth(id, true)`, bundled so
                 * the Entity menu can offer Cloth as a first-class object.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_cloth(const std::string& name) = 0;

                /** @brief Whether @p id carries a visual Shape (Box/Sphere/Cylinder). */
                virtual bool has_shape(EntityId id) const noexcept = 0;

                /** @brief The entity's shape kind/parameters (defaults if it has no Shape). */
                virtual ShapeParams shape_params(EntityId id) const = 0;

                /** @brief Writes a Shape entity's parameters; a no-op for entities without one. */
                virtual void set_shape_params(EntityId id, const ShapeParams& params) = 0;

                /**
                 * @brief Attaches or detaches a Shape on an existing entity.
                 *
                 * Not offered directly in the Inspector's "Add Component" popup — a
                 * Shape without sane defaults is only meaningful via `create_box`/
                 * `create_sphere`/`create_cylinder`, which call this internally. Exposed
                 * on the interface so scene load can restore a saved primitive's Shape
                 * without re-deriving which `create_*` call originally made it.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Shape after this call.
                 */
                virtual void set_has_shape(EntityId id, bool value) = 0;

                /** @brief Whether @p id carries a Collider. */
                virtual bool has_collider(EntityId id) const noexcept = 0;

                /** @brief The entity's collider kind/parameters (defaults if it has no Collider). */
                virtual ColliderParams collider_params(EntityId id) const = 0;

                /** @brief Writes a Collider's parameters; a no-op for entities without one. */
                virtual void set_collider_params(EntityId id, const ColliderParams& params) = 0;

                /**
                 * @brief Attaches or detaches a Collider on an existing entity.
                 *
                 * Independent of any Shape the entity carries — a Collider can be added
                 * to a bare entity as an invisible volume, or removed from a primitive
                 * to make it visual-only.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Collider after this call.
                 */
                virtual void set_has_collider(EntityId id, bool value) = 0;

                /**
                 * @brief Creates a UI Canvas: the full-viewport root of a UI tree.
                 *
                 * A Canvas carries a UI element record with `UIElementKind::Canvas`;
                 * every other UI element lays out inside the Canvas that is its
                 * ancestor. It draws nothing itself but establishes the pixel rect
                 * children resolve against (see @ref UIElementParams).
                 *
                 * @param name Display name for the new canvas.
                 * @return The new canvas entity's stable id.
                 */
                virtual EntityId create_canvas(const std::string& name) = 0;

                /**
                 * @brief Creates a UI element (Panel/Image/Text/Button) under @p parent.
                 *
                 * The element is parented to @p parent (typically a Canvas) so it
                 * inherits its layout rect; pass `NULL_ENTITY` to anchor it directly
                 * to the viewport. Its rect defaults to a centred box the caller can
                 * re-anchor via `set_ui_params`.
                 *
                 * @param name   Display name for the new element.
                 * @param kind   Which UI element to create.
                 * @param parent The UI ancestor to lay out inside, or `NULL_ENTITY`.
                 * @return The new element entity's stable id.
                 */
                virtual EntityId create_ui_element(const std::string& name, UIElementKind kind,
                                                   EntityId parent) = 0;

                /** @brief Whether @p id carries a UI element record. */
                virtual bool has_ui(EntityId id) const noexcept = 0;

                /** @brief Whether @p id is a UI Canvas (a UI element of kind Canvas). */
                virtual bool is_canvas(EntityId id) const noexcept = 0;

                /** @brief The entity's UI element parameters (defaults if it has no UI). */
                virtual UIElementParams ui_params(EntityId id) const = 0;

                /** @brief Writes a UI entity's element parameters; a no-op for non-UI entities. */
                virtual void set_ui_params(EntityId id, const UIElementParams& params) = 0;

                /**
                 * @brief Attaches or detaches a UI element on an existing entity.
                 *
                 * Attaching defaults the entity to an `Image` element; the kind can
                 * then be changed via `set_ui_params`. Detaching removes it from the
                 * UI overlay. Like cloth, this is host-side bookkeeping needing no ECS
                 * component migration.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should carry a UI element after this call.
                 */
                virtual void set_has_ui(EntityId id, bool value) = 0;

                /** @brief The type names of every script component attached to @p id, in order. */
                virtual std::vector<std::string> script_components(EntityId id) const = 0;

                /** @brief Whether @p id carries a script component named @p type_name. */
                virtual bool has_script_component(EntityId id,
                                                  const std::string& type_name) const = 0;

                /** @brief The named script component's authored fields (empty if absent). */
                virtual ScriptComponent script_component(EntityId id,
                                                         const std::string& type_name) const = 0;

                /**
                 * @brief Attaches a script component instance to @p id.
                 *
                 * A no-op if a component of the same `type_name` is already attached,
                 * so re-adding never duplicates. The instance carries its own copy of
                 * the field values (seeded by the editor from the definition catalog),
                 * so later edits to the definition do not retroactively change it.
                 *
                 * @param id        The entity to update.
                 * @param component The script component instance to attach.
                 */
                virtual void add_script_component(EntityId id,
                                                  const ScriptComponent& component) = 0;

                /** @brief Overwrites the fields of the like-named script component on @p id; a no-op if absent. */
                virtual void set_script_component(EntityId id,
                                                  const ScriptComponent& component) = 0;

                /** @brief Detaches the script component named @p type_name; a no-op if absent. */
                virtual void remove_script_component(EntityId id,
                                                     const std::string& type_name) = 0;

                /**
                 * @brief Tells the world the pixel size the UI is currently being viewed at.
                 *
                 * Every UI entity's layout (see `UIElementParams`/`SushiEngine::UI::resolve_rect`)
                 * resolves against a Canvas's rect, and a full-viewport Canvas's rect is the
                 * screen it fills — so the host (the editor's viewport panel, or the runtime
                 * window) calls this once per frame with its current pixel size before reading
                 * back `ui_params`/the UI overlay, the same way a resize event drives any other
                 * screen-space layout. A host with more than one UI-bearing surface (e.g. Scene
                 * and Game views) calls it with whichever surface's size should currently drive
                 * layout; the most recent call wins.
                 *
                 * @param width  Target width in pixels (clamped to at least 1).
                 * @param height Target height in pixels (clamped to at least 1).
                 */
                virtual void set_ui_target_size(std::uint32_t width, std::uint32_t height) = 0;
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
                 * @brief Advances the world by zero or more fixed simulation steps.
                 *
                 * Feeds @p real_delta_seconds into an internal `Loop::FixedTimestepClock`
                 * and runs the schedule on the runtime (the systems execute as SYCL
                 * kernels) once per whole fixed step the clock reports — zero if the
                 * caller has been ticking faster than the fixed rate, more than one if a
                 * host frame hitched. Each individual step is fixed and deterministic; a
                 * host still gates motion by choosing whether to call this at all
                 * (play/pause), not by scaling @p real_delta_seconds into device code.
                 *
                 * @param real_delta_seconds Wall-clock time since the last call, in
                 * seconds, as measured by the host (never read from inside the sim).
                 */
                virtual void tick(Scalar real_delta_seconds) = 0;

                /**
                 * @brief The duration of one fixed simulation step, in seconds.
                 *
                 * The size of the step `tick()`'s internal `Loop::FixedTimestepClock`
                 * advances by; a host uses this to force exactly one step (e.g. a
                 * "Step" button while paused) by calling `tick(fixed_dt_seconds())`
                 * regardless of how much real time actually elapsed.
                 */
                virtual Scalar fixed_dt_seconds() const noexcept = 0;

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
         * populates the world only via `IWorldEditor` (New Entity, Entity menu) or
         * by loading a `.sushiscene`. The only place the runtime is constructed for
         * the editor.
         *
         * @return An owned simulation; never null (throws on runtime bring-up failure).
         */
        std::unique_ptr<ISimulation> create_simulation();
    } // namespace Simulation
} // namespace SushiEngine
