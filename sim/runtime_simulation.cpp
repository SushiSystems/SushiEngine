/**************************************************************************/
/* runtime_simulation.cpp                                                 */
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

// The one SYCL translation unit behind the ISimulation seam: it owns a
// SushiRuntime, an ECS World, and a Schedule, and drives an editable live world.
// Two systems demonstrate the runtime's dependency tracker — "spin" advances each
// animated cube's orientation, "orbit" advances its position — over disjoint
// components, so they run in parallel exactly as the sandbox proves. Every value a
// kernel touches is precomputed on the host into a component (the per-step rotation
// quaternion, the per-step orbit rotation as a cos/sin pair), so the kernels are
// pure arithmetic and capture no host state, which is what makes them legal device
// code. The editor addresses entities by a stable EntityId this file maps onto the
// ECS handle; names and visibility are host-side editor metadata, while transform
// and colour are real components. Entities the editor creates carry no motion, so
// they stay where they are placed and edited even while the world plays; only the
// seeded demo cubes are driven by the two systems. After each step, and after any
// edit, an extract pass reads the columns back on the host into the RenderScene the
// editor draws.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/physics/cloth.hpp>
#include <SushiEngine/sim/components.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        namespace
        {
            /** @brief a / b, treating a near-zero divisor as 1 so a degenerate parent scale never divides by zero. */
            Scalar safe_div(Scalar a, Scalar b) noexcept
            {
                constexpr Scalar EPSILON = Scalar(1e-8);
                return (b > EPSILON || b < -EPSILON) ? a / b : a;
            }

            constexpr std::size_t CHUNK_CAPACITY = 256;

            // Rigid Body tuning. The outer fixed step is FIXED_TICK_DT_SECONDS, owned
            // by the Loop::FixedTimestepClock every RuntimeSimulation instance keeps;
            // the physics sub-step duration is derived from it (fixed_dt / substeps
            // per tick) rather than hardcoded separately, so there is exactly one
            // source of truth for the tick duration.
            constexpr Scalar FIXED_TICK_DT_SECONDS = Scalar(1.0 / 60.0);
            constexpr Scalar PHYSICS_GRAVITY_Y = Scalar(-9.8);
            constexpr std::size_t PHYSICS_SUBSTEPS_PER_TICK = 4;
            constexpr std::size_t PHYSICS_ITERATIONS = 8;

            /**
             * @brief The runtime-backed live world behind the ISimulation seam.
             *
             * Constructs the runtime, world, and schedule; seeds the animated demo
             * cubes; registers the two systems; and on each tick — and after each edit
             * — extracts a fresh RenderScene. The extract is a host read of the
             * shared-USM columns via `World::get`, composed into model matrices — the
             * simple, correct path before device-shared interop lands.
             */
            class RuntimeSimulation final : public ISimulation, public IWorldEditor
            {
                public:
                    RuntimeSimulation()
                        : runtime_(SushiRuntime::API::Runtime::create()),
                          world_(runtime_, CHUNK_CAPACITY),
                          schedule_(runtime_),
                          clock_(FIXED_TICK_DT_SECONDS)
                    {
                        // Reserve every archetype up front so neither the seed, the
                        // editor's first create, nor a later Add/Remove Component
                        // toggle allocates a chunk mid-run. Transform + Orientation are
                        // mandatory on every entity; Tint (Renderer) and Camera are
                        // independently pluggable, so all four combinations exist.
                        world_.reserve<Transform, Orientation, SpinStep, OrbitState, Tint>(
                            CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation>(CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation, Tint>(CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation, Camera>(CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation, Tint, Camera>(CHUNK_CAPACITY);
                        register_systems();
                        extract(); // a valid (empty) snapshot before the first tick
                    }

                    // --- ISimulation -------------------------------------------------

                    void tick(Scalar real_delta_seconds) override
                    {
                        clock_.accumulate(real_delta_seconds);
                        while (clock_.consume_step())
                            step_once();
                        interpolation_ = clock_.interpolation();
                    }

                    Scalar fixed_dt_seconds() const noexcept override
                    {
                        return clock_.fixed_dt();
                    }

                    const RenderScene& render_scene() const noexcept override
                    {
                        return scene_;
                    }

                    std::size_t entity_count() const noexcept override
                    {
                        return order_.size();
                    }

                    IWorldEditor& world() noexcept override { return *this; }

                    // --- IWorldEditor ------------------------------------------------

                    std::vector<EntityId> entities() const override { return order_; }

                    bool exists(EntityId id) const noexcept override
                    {
                        return records_.find(id) != records_.end();
                    }

                    std::string name(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->name : std::string{};
                    }

                    EntityTransform transform(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !world_.alive(record->entity))
                            return EntityTransform{};
                        const Transform& t = world_.get<Transform>(record->entity);
                        const Orientation& o = world_.get<Orientation>(record->entity);
                        EntityTransform out;
                        out.position = t.position;
                        out.rotation = o.rotation;
                        out.scale = t.scale;
                        return out;
                    }

                    Vector3 color(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_renderer ||
                            !world_.alive(record->entity))
                            return Vector3{};
                        return world_.get<Tint>(record->entity).color;
                    }

                    bool visible(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->visible;
                    }

                    EntityId create(const std::string& display_name) override
                    {
                        const Entity entity = world_.spawn(Transform{}, Orientation{},
                                                           Tint{Vector3{Scalar(0.8), Scalar(0.8),
                                                                     Scalar(0.8)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_renderer = true;
                        records_.emplace(id, record);
                        extract();
                        return id;
                    }

                    void destroy(EntityId id) override
                    {
                        const auto it = records_.find(id);
                        if (it == records_.end())
                            return;
                        if (it->second.has_physics_body)
                            physics_dirty_ = true;
                        physics_body_ids_.erase(id);
                        if (it->second.has_cloth)
                            cloth_dirty_ = true;
                        cloth_grids_.erase(id);
                        CommandBuffer commands;
                        commands.destroy(it->second.entity);
                        commands.apply(world_);
                        records_.erase(it);
                        order_.erase(std::remove(order_.begin(), order_.end(), id),
                                     order_.end());
                        // Destroying a parent leaves its children as roots rather than
                        // cascading the destroy, matching how the Hierarchy shows them.
                        for (auto& entry : records_)
                            if (entry.second.parent == id)
                                entry.second.parent = NULL_ENTITY;
                        extract();
                    }

                    void set_name(EntityId id, const std::string& display_name) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                            record->name = display_name;
                    }

                    void set_transform(EntityId id, const EntityTransform& value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !world_.alive(record->entity))
                            return;
                        Transform& t = world_.get<Transform>(record->entity);
                        Orientation& o = world_.get<Orientation>(record->entity);
                        t.position = value.position;
                        t.scale = value.scale;
                        o.rotation = value.rotation;
                        extract();
                    }

                    void set_color(EntityId id, const Vector3& value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_renderer ||
                            !world_.alive(record->entity))
                            return;
                        world_.get<Tint>(record->entity).color = value;
                        extract();
                    }

                    bool has_renderer(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_renderer;
                    }

                    void set_has_renderer(EntityId id, bool value) override
                    {
                        migrate_components(id, value, /*camera=*/find(id) != nullptr &&
                                                            find(id)->is_camera);
                    }

                    void set_is_camera(EntityId id, bool value) override
                    {
                        migrate_components(
                            id, /*renderer=*/find(id) != nullptr && find(id)->has_renderer,
                            value);
                    }

                    bool has_physics_body(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_physics_body;
                    }

                    PhysicsBodyParams physics_body_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->physics_params : PhysicsBodyParams{};
                    }

                    void set_physics_body_params(EntityId id,
                                                 const PhysicsBodyParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->physics_params = params;
                        // Applied live when a body already exists, so editing mass/inertia
                        // never forces a physics-world rebuild (see set_has_physics_body).
                        const auto it = physics_body_ids_.find(id);
                        if (physics_ && it != physics_body_ids_.end())
                        {
                            Physics::RigidBody& body = physics_->body(it->second);
                            body.inv_mass = params.inv_mass;
                            body.inv_inertia = params.inv_inertia;
                        }
                    }

                    void set_has_physics_body(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_physics_body == value)
                            return;
                        record->has_physics_body = value;
                        physics_dirty_ = true;
                    }

                    bool has_cloth(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_cloth;
                    }

                    ClothParams cloth_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->cloth_params : ClothParams{};
                    }

                    void set_cloth_params(EntityId id, const ClothParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->cloth_params = params;
                        // Rows/cols change the grid's body count, so — unlike a Rigid
                        // Body's mass/inertia — every parameter edit here forces a
                        // rebuild rather than being applied live.
                        if (record->has_cloth)
                            cloth_dirty_ = true;
                    }

                    void set_has_cloth(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_cloth == value)
                            return;
                        record->has_cloth = value;
                        cloth_dirty_ = true;
                    }

                    std::vector<Vector3> cloth_particle_positions(EntityId id) const override
                    {
                        std::vector<Vector3> positions;
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_cloth || !physics_cloth_)
                            return positions;
                        const auto grid_it = cloth_grids_.find(id);
                        if (grid_it == cloth_grids_.end())
                            return positions;
                        const Physics::ClothGrid& grid = grid_it->second;
                        positions.reserve(grid.bodies.size());
                        for (const Physics::BodyId body_id : grid.bodies)
                            positions.push_back(physics_cloth_->body(body_id).position);
                        return positions;
                    }

                    EntityId create_camera(const std::string& display_name) override
                    {
                        // A default camera looking down -Z from a few units back, so the
                        // seeded scene is visible without any rotation authoring yet.
                        Transform transform;
                        transform.position = Vector3{0, Scalar(3), Scalar(12)};
                        const Entity entity =
                            world_.spawn(transform, Orientation{}, Camera{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        records_.emplace(id, Record{entity, display_name, true, false, true});
                        extract();
                        return id;
                    }

                    bool is_camera(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->is_camera;
                    }

                    CameraParams camera_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->is_camera ||
                            !world_.alive(record->entity))
                            return CameraParams{};
                        const Camera& c = world_.get<Camera>(record->entity);
                        CameraParams params;
                        params.vertical_fov_radians = c.vertical_fov_radians;
                        params.near_plane = c.near_plane;
                        params.far_plane = c.far_plane;
                        params.display_index = c.display_index;
                        params.priority = c.priority;
                        params.active = c.active;
                        return params;
                    }

                    void set_camera_params(EntityId id, const CameraParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->is_camera ||
                            !world_.alive(record->entity))
                            return;
                        Camera& c = world_.get<Camera>(record->entity);
                        c.vertical_fov_radians = params.vertical_fov_radians;
                        c.near_plane = params.near_plane;
                        c.far_plane = params.far_plane;
                        c.display_index = params.display_index;
                        c.priority = params.priority;
                        c.active = params.active;
                        extract();
                    }

                    void set_visible(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                        {
                            record->visible = value;
                            extract();
                        }
                    }

                    EntityId parent(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->parent : NULL_ENTITY;
                    }

                    void set_parent(EntityId child, EntityId new_parent) override
                    {
                        Record* child_record = find(child);
                        if (child_record == nullptr || child == new_parent)
                            return;
                        if (new_parent != NULL_ENTITY)
                        {
                            if (find(new_parent) == nullptr || is_descendant(new_parent, child))
                                return;
                        }

                        // Reparenting must not move the object: recompute the child's
                        // local transform so its resolved world pose is unchanged,
                        // rather than leaving it to be reinterpreted in the new
                        // parent's space (which is what jumps it).
                        const EntityTransform child_world = world_transform(child);
                        child_record->parent = new_parent;
                        const EntityTransform parent_world =
                            new_parent != NULL_ENTITY ? world_transform(new_parent)
                                                       : EntityTransform{};

                        EntityTransform new_local;
                        new_local.scale = Vector3{safe_div(child_world.scale.x, parent_world.scale.x),
                                              safe_div(child_world.scale.y, parent_world.scale.y),
                                              safe_div(child_world.scale.z, parent_world.scale.z)};
                        new_local.rotation =
                            normalize(mul(conjugate(parent_world.rotation), child_world.rotation));
                        const Vector3 delta = rotate(conjugate(parent_world.rotation),
                                                  child_world.position - parent_world.position);
                        new_local.position = Vector3{safe_div(delta.x, parent_world.scale.x),
                                                 safe_div(delta.y, parent_world.scale.y),
                                                 safe_div(delta.z, parent_world.scale.z)};

                        set_transform(child, new_local);
                    }

                private:
                    /** @brief The editor metadata paired with each entity's ECS handle. */
                    struct Record
                    {
                        Entity entity;
                        std::string name;
                        bool visible = true;
                        bool animated = false;
                        bool is_camera = false;
                        EntityId parent = NULL_ENTITY;
                        // Whether the Tint (Renderer) component is attached. Editor-created
                        // entities start with one; `set_has_renderer` toggles it. Cameras
                        // default to none, matching Unity's empty-GameObject-with-Camera.
                        bool has_renderer = false;
                        // Whether the entity is tracked by physics_ (see set_has_physics_body).
                        // Unlike has_renderer/is_camera this needs no ECS migration, so it is
                        // plain host bookkeeping rather than a component toggle.
                        bool has_physics_body = false;
                        PhysicsBodyParams physics_params{};
                        // Whether a cloth grid is tracked by physics_cloth_ (see
                        // set_has_cloth). Same plain-host-bookkeeping treatment as
                        // has_physics_body: cloth needs no ECS component migration.
                        bool has_cloth = false;
                        ClothParams cloth_params{};
                    };

                    const Record* find(EntityId id) const noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    /**
                     * @brief Whether @p candidate is @p ancestor or one of its descendants.
                     *
                     * Walks up from @p candidate through its parent chain looking for
                     * @p ancestor, bounded by the live entity count so a corrupt chain can
                     * never loop forever.
                     */
                    bool is_descendant(EntityId candidate, EntityId ancestor) const noexcept
                    {
                        std::size_t guard = records_.size() + 1;
                        for (EntityId current = candidate;
                             current != NULL_ENTITY && guard > 0; --guard)
                        {
                            if (current == ancestor)
                                return true;
                            const Record* record = find(current);
                            current = record != nullptr ? record->parent : NULL_ENTITY;
                        }
                        return false;
                    }

                    /**
                     * @brief Composes an entity's world-space TRS along its parent chain.
                     *
                     * Position/rotation/scale are chained directly (world_scale =
                     * parent_scale * local_scale, world_rotation = parent_rotation *
                     * local_rotation, world_position = parent_position + parent_rotation
                     * applied to parent_scale * local_position) rather than through a
                     * general Mat4 product, so scale never introduces shear and the
                     * result can be inverted — reparenting needs exactly that inverse to
                     * keep an object's world pose fixed across the move. A bounded walk
                     * guards against a corrupt chain rather than recursing unboundedly.
                     */
                    EntityTransform world_transform(EntityId id) const
                    {
                        std::vector<EntityTransform> chain;
                        std::size_t guard = records_.size() + 1;
                        for (EntityId current = id;
                             current != NULL_ENTITY && guard > 0; --guard)
                        {
                            const Record* record = find(current);
                            if (record == nullptr || !world_.alive(record->entity))
                                break;
                            const Transform& t = world_.get<Transform>(record->entity);
                            const Orientation& o = world_.get<Orientation>(record->entity);
                            chain.push_back(EntityTransform{t.position, o.rotation, t.scale});
                            current = record->parent;
                        }
                        EntityTransform result; // identity
                        for (auto it = chain.rbegin(); it != chain.rend(); ++it)
                        {
                            EntityTransform next;
                            next.scale = Vector3{result.scale.x * it->scale.x,
                                             result.scale.y * it->scale.y,
                                             result.scale.z * it->scale.z};
                            next.rotation = normalize(mul(result.rotation, it->rotation));
                            const Vector3 scaled_local = Vector3{result.scale.x * it->position.x,
                                                          result.scale.y * it->position.y,
                                                          result.scale.z * it->position.z};
                            next.position =
                                result.position + rotate(result.rotation, scaled_local);
                            result = next;
                        }
                        return result;
                    }

                    /** @brief The object-to-world matrix for @p id, built from @ref world_transform. */
                    Mat4 world_matrix(EntityId id) const
                    {
                        const EntityTransform world = world_transform(id);
                        return compose_transform(world.position, world.rotation, world.scale);
                    }

                    Record* find(EntityId id) noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    /** @brief The physics sub-step duration: the clock's fixed step split into equal sub-steps. */
                    Scalar substep_dt() const noexcept
                    {
                        return clock_.fixed_dt() / Scalar(PHYSICS_SUBSTEPS_PER_TICK);
                    }

                    /**
                     * @brief Runs exactly one fixed simulation step: physics, then the schedule, then extract.
                     *
                     * Called once per whole fixed step `tick()`'s clock reports, so a
                     * hitched host frame that accumulates more than one step's worth of
                     * real time replays this deterministically once per step rather than
                     * scaling a single step by however long the frame took.
                     */
                    void step_once()
                    {
                        if (physics_dirty_)
                            rebuild_physics();
                        if (physics_)
                        {
                            physics_->step(Vector3{0, PHYSICS_GRAVITY_Y, 0}, PHYSICS_SUBSTEPS_PER_TICK);
                            for (const auto& [id, body_id] : physics_body_ids_)
                            {
                                const Record* record = find(id);
                                if (record == nullptr || !world_.alive(record->entity))
                                    continue;
                                const Physics::RigidBody& solved = physics_->body(body_id);
                                world_.get<Transform>(record->entity).position = solved.position;
                                world_.get<Orientation>(record->entity).rotation = solved.orientation;
                            }
                        }
                        if (cloth_dirty_)
                            rebuild_cloth();
                        if (physics_cloth_)
                            physics_cloth_->step(Vector3{0, PHYSICS_GRAVITY_Y, 0}, PHYSICS_SUBSTEPS_PER_TICK);
                        schedule_.run(world_);
                        extract();
                    }

                    /**
                     * @brief Attaches or detaches the Renderer (Tint) and Camera
                     * components on @p id, moving it between ECS archetypes as needed.
                     *
                     * The ECS has no in-place add/remove: an entity's component set is
                     * fixed by its archetype, so "pluggable" components are implemented
                     * by destroying the old entity and spawning a new one in the target
                     * archetype, carrying over Transform/Orientation and whichever of
                     * Tint/Camera survive the toggle (defaulted if newly attached). Seeded,
                     * animated entities (SpinStep/OrbitState) are not migrated — their
                     * component set is fixed for the demo, matching how they are not
                     * otherwise editable while playing.
                     *
                     * @param id       The entity to update.
                     * @param renderer Whether it should carry a Tint (Renderer) after this call.
                     * @param camera   Whether it should carry a Camera after this call.
                     */
                    void migrate_components(EntityId id, bool renderer, bool camera)
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->animated ||
                            !world_.alive(record->entity))
                            return;
                        if (record->has_renderer == renderer && record->is_camera == camera)
                            return;

                        const Transform t = world_.get<Transform>(record->entity);
                        const Orientation o = world_.get<Orientation>(record->entity);
                        const Tint tint = record->has_renderer
                                              ? world_.get<Tint>(record->entity)
                                              : Tint{Vector3{Scalar(0.8), Scalar(0.8), Scalar(0.8)}};
                        const Camera cam = record->is_camera ? world_.get<Camera>(record->entity)
                                                             : Camera{};

                        world_.destroy(record->entity);

                        Entity new_entity;
                        if (renderer && camera)
                            new_entity = world_.spawn(t, o, tint, cam);
                        else if (renderer)
                            new_entity = world_.spawn(t, o, tint);
                        else if (camera)
                            new_entity = world_.spawn(t, o, cam);
                        else
                            new_entity = world_.spawn(t, o);

                        record->entity = new_entity;
                        record->has_renderer = renderer;
                        record->is_camera = camera;
                        extract();
                    }

                    /**
                     * @brief Rebuilds physics_ from the current set of Rigid Body entities.
                     *
                     * A body-count change needs a fresh `PhysicsWorld` (it compiles a solve
                     * graph sized to its bodies at `finalize()`, the same one-shot-build
                     * contract as `XpbdSolver`/`ConstraintSolver`), so this snapshots every
                     * currently-simulated body's live state first — position, orientation,
                     * velocity — and carries it over, so toggling physics on one entity does
                     * not reset every other rigid body already falling in the scene. A
                     * brand-new rigid body instead seeds from its current `Transform`/
                     * `Orientation`, at rest. Called lazily from `tick()`, never eagerly from
                     * the toggle itself, so several Inspector edits in one frame cost one
                     * rebuild, not one per edit.
                     */
                    void rebuild_physics()
                    {
                        std::unordered_map<EntityId, Physics::RigidBody> previous;
                        if (physics_)
                            for (const auto& [id, body_id] : physics_body_ids_)
                                previous.emplace(id, physics_->body(body_id));

                        physics_.reset();
                        physics_body_ids_.clear();

                        auto next_world =
                            std::make_unique<Physics::PhysicsWorld<Physics::XpbdDistanceConstraint>>(
                                runtime_);

                        for (EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_physics_body ||
                                !world_.alive(record->entity))
                                continue;

                            Physics::RigidBody body;
                            const auto previous_it = previous.find(id);
                            if (previous_it != previous.end())
                            {
                                body = previous_it->second;
                            }
                            else
                            {
                                body.position = world_.get<Transform>(record->entity).position;
                                body.orientation = world_.get<Orientation>(record->entity).rotation;
                            }
                            body.inv_mass = record->physics_params.inv_mass;
                            body.inv_inertia = record->physics_params.inv_inertia;

                            physics_body_ids_.emplace(id, next_world->add_body(body));
                        }

                        if (!physics_body_ids_.empty())
                        {
                            next_world->finalize(PHYSICS_ITERATIONS, substep_dt(),
                                                 Physics::XpbdDistanceProjection{});
                            physics_ = std::move(next_world);
                        }
                        physics_dirty_ = false;
                    }

                    /**
                     * @brief Rebuilds physics_cloth_ from the current set of Cloth entities.
                     *
                     * Unlike `rebuild_physics`, no live state is carried over: a rows/
                     * cols/spacing change replaces the grid's topology outright, so
                     * there is nothing meaningful to preserve across the rebuild
                     * (compare a Rigid Body, whose identity and body count are stable
                     * across a param edit). Each Cloth entity's grid originates at its
                     * own `Transform::position`, mirroring how a Rigid Body seeds from
                     * its `Transform`/`Orientation` rather than authoring its own pose.
                     * Called lazily from `step_once()`, same "rebuild only when the
                     * input set changes" discipline as `rebuild_physics`.
                     */
                    void rebuild_cloth()
                    {
                        physics_cloth_.reset();
                        cloth_grids_.clear();

                        auto next_world =
                            std::make_unique<Physics::PhysicsWorld<Physics::XpbdDistanceConstraint>>(
                                runtime_);

                        for (EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_cloth ||
                                !world_.alive(record->entity) ||
                                record->cloth_params.rows == 0 || record->cloth_params.cols == 0)
                                continue;

                            const Vector3 origin = world_.get<Transform>(record->entity).position;
                            Physics::ClothGrid grid = Physics::build_cloth_grid(
                                *next_world, record->cloth_params.rows, record->cloth_params.cols,
                                record->cloth_params.spacing, origin, record->cloth_params.compliance);
                            cloth_grids_.emplace(id, std::move(grid));
                        }

                        bool has_bodies = false;
                        for (const auto& entry : cloth_grids_)
                            if (!entry.second.bodies.empty())
                                has_bodies = true;

                        if (has_bodies)
                        {
                            next_world->finalize(PHYSICS_ITERATIONS, substep_dt(),
                                                 Physics::XpbdDistanceProjection{});
                            physics_cloth_ = std::move(next_world);
                        }
                        else
                        {
                            cloth_grids_.clear();
                        }
                        cloth_dirty_ = false;
                    }

                    /** @brief The camera used when no active camera exists, so the Game view is never black. */
                    static CameraState default_camera() noexcept
                    {
                        CameraState state;
                        state.position = Vector3{0, Scalar(7), Scalar(12)};
                        state.target = Vector3{0, Scalar(0.75), 0};
                        state.up = Vector3{0, 1, 0};
                        state.vertical_fov_radians = Scalar(1.0471976);
                        state.near_plane = Scalar(0.1);
                        state.far_plane = Scalar(500);
                        return state;
                    }

                    /** @brief Derives eye/target/up from a camera entity's pose and lens. */
                    static CameraState camera_state_of(const Transform& transform,
                                                       const Orientation& orientation,
                                                       const Camera& camera) noexcept
                    {
                        const Mat4 rotation = mat4_from_quaternion(orientation.rotation);
                        // Column-major basis: right = col0, up = col1, +Z = col2. A camera
                        // looks down its local -Z, so forward is the negated third column.
                        const Vector3 forward{-rotation.m[8], -rotation.m[9], -rotation.m[10]};
                        const Vector3 up{rotation.m[4], rotation.m[5], rotation.m[6]};
                        CameraState state;
                        state.position = transform.position;
                        state.target = transform.position + forward;
                        state.up = up;
                        state.vertical_fov_radians = camera.vertical_fov_radians;
                        state.near_plane = camera.near_plane;
                        state.far_plane = camera.far_plane;
                        return state;
                    }

                    /**
                     * @brief Registers the two per-cube systems.
                     *
                     * "spin" writes Orientation from the precomputed SpinStep; "orbit"
                     * writes Transform and advances OrbitState. Their write sets are
                     * disjoint, so the dependency tracker runs them concurrently. Only
                     * the seeded archetype carries SpinStep/OrbitState, so editor-created
                     * entities (which lack them) are never matched and stay still.
                     */
                    void register_systems()
                    {
                        schedule_.each<Write<Orientation>, Read<SpinStep>>("spin",
                            [](std::size_t i, Orientation* orientation, const SpinStep* step)
                            {
                                orientation[i].rotation =
                                    normalize(mul(step[i].delta, orientation[i].rotation));
                            });

                        schedule_.each<Write<Transform>, Write<OrbitState>>("orbit",
                            [](std::size_t i, Transform* transform, OrbitState* orbit)
                            {
                                const Scalar c = orbit[i].cos_angle;
                                const Scalar s = orbit[i].sin_angle;
                                const Scalar nc = c * orbit[i].step_cos - s * orbit[i].step_sin;
                                const Scalar ns = s * orbit[i].step_cos + c * orbit[i].step_sin;
                                orbit[i].cos_angle = nc;
                                orbit[i].sin_angle = ns;
                                transform[i].position.x = orbit[i].center.x + orbit[i].radius * nc;
                                transform[i].position.z = orbit[i].center.z + orbit[i].radius * ns;
                            });
                    }

                    /**
                     * @brief Reads the world's columns and rebuilds the render snapshot.
                     *
                     * A host read of the shared-USM component columns (via `World::get`)
                     * composed into per-instance model matrices; invisible entities are
                     * skipped. Run after every tick and after every edit so the view
                     * always matches the world.
                     */
                    void extract()
                    {
                        scene_.instances.clear();
                        scene_.instances.reserve(order_.size());

                        // Per display, keep the active camera with the highest priority.
                        struct Winner { std::int32_t priority; CameraState state; };
                        std::unordered_map<std::uint32_t, Winner> winners;

                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !world_.alive(record->entity))
                                continue;
                            const Transform& transform = world_.get<Transform>(record->entity);
                            const Orientation& orientation =
                                world_.get<Orientation>(record->entity);

                            if (record->is_camera)
                            {
                                const Camera& camera = world_.get<Camera>(record->entity);
                                if (!camera.active)
                                    continue;
                                const CameraState state =
                                    camera_state_of(transform, orientation, camera);
                                const auto it = winners.find(camera.display_index);
                                if (it == winners.end() || camera.priority > it->second.priority)
                                    winners[camera.display_index] = Winner{camera.priority, state};
                                continue;
                            }

                            if (!record->visible || !record->has_renderer)
                                continue;
                            const Tint& tint = world_.get<Tint>(record->entity);
                            RenderInstance instance;
                            instance.id = id;
                            instance.model = world_matrix(id);
                            instance.color = tint.color;
                            scene_.instances.push_back(instance);
                        }

                        // Publish the resolved cameras sorted by display, and pick the
                        // lowest-display one as the default so a single-viewport host works.
                        scene_.display_cameras.clear();
                        scene_.display_cameras.reserve(winners.size());
                        for (const auto& entry : winners)
                            scene_.display_cameras.push_back(DisplayCamera{entry.first, entry.second.state});
                        std::sort(scene_.display_cameras.begin(), scene_.display_cameras.end(),
                                  [](const DisplayCamera& a, const DisplayCamera& b)
                                  { return a.display < b.display; });
                        scene_.has_camera = !scene_.display_cameras.empty();
                        scene_.camera = scene_.has_camera ? scene_.display_cameras.front().state
                                                          : default_camera();
                    }

                    SushiRuntime::API::Runtime runtime_;
                    World world_;
                    Schedule schedule_;
                    Loop::FixedTimestepClock clock_;
                    // The clock's leftover fraction after the most recent tick(), for a
                    // future render-interpolation consumer; not read anywhere yet.
                    Scalar interpolation_ = 0;
                    std::vector<EntityId> order_;
                    std::unordered_map<EntityId, Record> records_;
                    EntityId next_id_ = 1;
                    RenderScene scene_;
                    std::unique_ptr<Physics::PhysicsWorld<Physics::XpbdDistanceConstraint>> physics_;
                    std::unordered_map<EntityId, Physics::BodyId> physics_body_ids_;
                    bool physics_dirty_ = false;
                    // A separate PhysicsWorld from physics_ rather than sharing one:
                    // rebuild_physics()'s snapshot-and-carry-over discipline (see its
                    // Doxygen) is keyed on individual free bodies and would need to
                    // special-case an entire pinned grid to avoid corrupting it on
                    // every unrelated Rigid Body toggle, so keeping cloth in its own
                    // PhysicsWorld<XpbdDistanceConstraint> (same constraint type, a
                    // different instance) leaves rebuild_physics() untouched.
                    std::unique_ptr<Physics::PhysicsWorld<Physics::XpbdDistanceConstraint>> physics_cloth_;
                    std::unordered_map<EntityId, Physics::ClothGrid> cloth_grids_;
                    bool cloth_dirty_ = false;
            };
        } // namespace

        std::unique_ptr<ISimulation> create_simulation()
        {
            return std::unique_ptr<ISimulation>(new RuntimeSimulation());
        }
    } // namespace Simulation
} // namespace SushiEngine
