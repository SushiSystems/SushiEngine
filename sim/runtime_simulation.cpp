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
#include <SushiEngine/sim/physics_simulation.hpp>
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
                    explicit RuntimeSimulation()
                        : runtime_(SushiRuntime::API::Runtime::create()),
                          world_(runtime_, CHUNK_CAPACITY),
                          schedule_(runtime_),
                          clock_(FIXED_TICK_DT_SECONDS),
                          physics_(create_physics_simulation(runtime_))
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

                    Render::Material material(EntityId id) const override
                    {
                        const Record* record = find(id);
                        Render::Material material = record != nullptr ? record->material
                                                                      : Render::Material{};
                        material.albedo = color(id);
                        return material;
                    }

                    Render::Environment environment() const override
                    {
                        return scene_.environment;
                    }

                    bool visible(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->visible;
                    }

                    EntityId create(const std::string& display_name) override
                    {
                        // A truly empty entity: just the mandatory Transform/Orientation,
                        // no Renderer and no mesh, matching Unity's empty GameObject. A
                        // Renderer (and, bound to it, a mesh Shape) is added on demand
                        // through Add Component, so a bare Create Entity never draws.
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        records_.emplace(id, record);
                        extract();
                        return id;
                    }

                    void destroy(EntityId id) override
                    {
                        const auto it = records_.find(id);
                        if (it == records_.end())
                            return;
                        // The physics simulation regenerates its body/grid set from the
                        // surviving entities on the next rebuild, so a destroy only needs
                        // to flag that rebuild — it holds no per-entity map to prune here.
                        if (it->second.has_physics_body)
                            physics_dirty_ = true;
                        if (it->second.has_cloth)
                            cloth_dirty_ = true;
                        if (world_.alive(it->second.ui_mirror))
                            world_.destroy(it->second.ui_mirror);
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
                        // While playing, a manual transform edit (e.g. dragging the gizmo)
                        // must move the physics body too, otherwise the next tick overwrites
                        // it with the solved pose. A no-op when the body does not exist yet
                        // (before the first physics rebuild), which is when the rebuild
                        // seeds from this same transform instead.
                        if (record->has_physics_body)
                            physics_->set_rigid_pose(id, value.position, value.rotation);
                        extract();
                    }

                    EntityTransform world_transform(EntityId id) const override
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

                    void set_world_transform(EntityId id, const EntityTransform& world) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !world_.alive(record->entity))
                            return;

                        if (record->parent == NULL_ENTITY)
                        {
                            set_transform(id, world);
                            return;
                        }

                        const EntityTransform parent_world = world_transform(record->parent);

                        EntityTransform new_local;
                        new_local.scale = Vector3{safe_div(world.scale.x, parent_world.scale.x),
                                              safe_div(world.scale.y, parent_world.scale.y),
                                              safe_div(world.scale.z, parent_world.scale.z)};
                        new_local.rotation =
                            normalize(mul(conjugate(parent_world.rotation), world.rotation));
                        const Vector3 delta = rotate(conjugate(parent_world.rotation),
                                                  world.position - parent_world.position);
                        new_local.position = Vector3{safe_div(delta.x, parent_world.scale.x),
                                                 safe_div(delta.y, parent_world.scale.y),
                                                 safe_div(delta.z, parent_world.scale.z)};

                        set_transform(id, new_local);
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

                    void set_material(EntityId id, const Render::Material& value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->material = value;
                        extract();
                    }

                    void set_environment(const Render::Environment& value) override
                    {
                        scene_.environment = value;
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
                        // never forces a physics-world rebuild (see set_has_physics_body);
                        // a no-op inside the physics simulation when the entity has none.
                        physics_->update_rigid_body_params(id, params.inv_mass, params.inv_inertia);
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
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_cloth)
                            return {};
                        return physics_->cloth_positions(id);
                    }

                    EntityId create_box(const std::string& display_name) override
                    {
                        return create_primitive(display_name, PrimitiveKind::Box,
                                                Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)});
                    }

                    EntityId create_sphere(const std::string& display_name) override
                    {
                        return create_primitive(display_name, PrimitiveKind::Sphere,
                                                Vector3{Scalar(0.5), Scalar(0.5), Scalar(0.5)});
                    }

                    EntityId create_cylinder(const std::string& display_name) override
                    {
                        return create_primitive(display_name, PrimitiveKind::Cylinder,
                                                Vector3{Scalar(0.5), Scalar(1.0), Scalar(0.5)});
                    }

                    EntityId create_terrain(const std::string& display_name) override
                    {
                        // A large, thin flat Box stands in for the terrain's visual;
                        // its Collider is Plane, and it never gets a physics body, so
                        // nothing ever integrates its pose (see create_terrain's Doxygen).
                        const Vector3 half_extents{Scalar(50), Scalar(0.1), Scalar(50)};
                        const ColliderParams terrain_collider{PrimitiveKind::Plane, Vector3{0, 1, 0}};
                        const EntityId id = create_primitive(display_name, PrimitiveKind::Box,
                                                             half_extents, &terrain_collider);
                        Record* record = find(id);
                        if (record != nullptr)
                            world_.get<Tint>(record->entity).color =
                                Vector3{Scalar(0.35), Scalar(0.55), Scalar(0.3)};
                        return id;
                    }

                    EntityId create_cloth(const std::string& display_name) override
                    {
                        // A bare entity that owns a cloth grid: no Renderer/Shape (the
                        // cloth draws as a wireframe strand set, not a solid mesh). The
                        // grid seeds from the entity's Transform::position, so moving
                        // the entity moves the pinned top edge.
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_cloth = true;
                        records_.emplace(id, record);
                        cloth_dirty_ = true;
                        extract();
                        return id;
                    }

                    bool has_shape(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_shape;
                    }

                    ShapeParams shape_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->shape_params : ShapeParams{};
                    }

                    void set_shape_params(EntityId id, const ShapeParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_shape)
                            return;
                        record->shape_params = params;
                        extract();
                    }

                    void set_has_shape(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_shape == value)
                            return;
                        record->has_shape = value;
                        extract();
                    }

                    bool has_collider(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_collider;
                    }

                    ColliderParams collider_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->collider_params : ColliderParams{};
                    }

                    void set_collider_params(EntityId id, const ColliderParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_collider)
                            return;
                        record->collider_params = params;
                    }

                    void set_has_collider(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_collider == value)
                            return;
                        record->has_collider = value;
                        // Attaching a Collider defaults it to match the entity's own
                        // visual Shape when it has one, so a newly-added Collider on a
                        // primitive is collidable out of the box; a bare entity falls
                        // back to a unit Box volume.
                        if (value)
                            record->collider_params = record->has_shape
                                                          ? ColliderParams{record->shape_params.kind,
                                                                          record->shape_params.params}
                                                          : ColliderParams{};
                    }

                    EntityId create_canvas(const std::string& display_name) override
                    {
                        const EntityId id = create(display_name);
                        Record* record = find(id);
                        if (record != nullptr)
                        {
                            record->has_ui = true;
                            record->ui_params = UIElementParams{};
                            record->ui_params.kind = UIElementKind::Canvas;
                            record->ui_params.size_x = static_cast<Scalar>(ui_target_size_.x);
                            record->ui_params.size_y = static_cast<Scalar>(ui_target_size_.y);
                            sync_ui_mirror(*record);
                        }
                        extract();
                        return id;
                    }

                    EntityId create_ui_element(const std::string& display_name, UIElementKind kind,
                                               EntityId parent) override
                    {
                        const EntityId id = create(display_name);
                        Record* record = find(id);
                        if (record != nullptr)
                        {
                            record->has_ui = true;
                            record->ui_params = default_ui_params(kind);
                            sync_ui_mirror(*record);
                        }
                        if (parent != NULL_ENTITY && find(parent) != nullptr)
                            set_parent(id, parent);
                        extract();
                        return id;
                    }

                    bool has_ui(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_ui;
                    }

                    bool is_canvas(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_ui &&
                               record->ui_params.kind == UIElementKind::Canvas;
                    }

                    UIElementParams ui_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->ui_params : UIElementParams{};
                    }

                    void set_ui_params(EntityId id, const UIElementParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_ui)
                            return;
                        record->ui_params = params;
                        sync_ui_mirror(*record);
                        extract();
                    }

                    void set_has_ui(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_ui == value)
                            return;
                        record->has_ui = value;
                        if (value)
                            record->ui_params = default_ui_params(UIElementKind::Image);
                        sync_ui_mirror(*record);
                        extract();
                    }

                    void set_ui_target_size(std::uint32_t width, std::uint32_t height) override
                    {
                        ui_target_size_.x = width > 0 ? width : 1;
                        ui_target_size_.y = height > 0 ? height : 1;
                        for (auto& entry : records_)
                        {
                            if (!entry.second.has_ui ||
                                entry.second.ui_params.kind != UIElementKind::Canvas)
                                continue;
                            // In the default ConstantPixelSize mode a Canvas's rect always
                            // fills the actual target regardless of this size, but keeping the
                            // authored value in step with the viewport keeps the inspector's
                            // display honest and gives ScaleWithScreenSize the same tracking.
                            entry.second.ui_params.size_x = static_cast<Scalar>(ui_target_size_.x);
                            entry.second.ui_params.size_y = static_cast<Scalar>(ui_target_size_.y);
                            sync_ui_mirror(entry.second);
                        }
                    }

                    std::vector<std::string> script_components(EntityId id) const override
                    {
                        std::vector<std::string> names;
                        const Record* record = find(id);
                        if (record != nullptr)
                            for (const ScriptComponent& script : record->scripts)
                                names.push_back(script.type_name);
                        return names;
                    }

                    bool has_script_component(EntityId id,
                                              const std::string& type_name) const override
                    {
                        return find_script(find(id), type_name) != nullptr;
                    }

                    ScriptComponent script_component(EntityId id,
                                                     const std::string& type_name) const override
                    {
                        const ScriptComponent* script = find_script(find(id), type_name);
                        return script != nullptr ? *script : ScriptComponent{};
                    }

                    void add_script_component(EntityId id, const ScriptComponent& component) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || find_script(record, component.type_name) != nullptr)
                            return;
                        record->scripts.push_back(component);
                    }

                    void set_script_component(EntityId id, const ScriptComponent& component) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        for (ScriptComponent& script : record->scripts)
                            if (script.type_name == component.type_name)
                            {
                                script.fields = component.fields;
                                return;
                            }
                    }

                    void remove_script_component(EntityId id,
                                                 const std::string& type_name) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->scripts.erase(
                            std::remove_if(record->scripts.begin(), record->scripts.end(),
                                           [&](const ScriptComponent& script)
                                           { return script.type_name == type_name; }),
                            record->scripts.end());
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

                    void move_entity(EntityId id, EntityId target, bool insert_after) override
                    {
                        if (id == target || id == NULL_ENTITY || target == NULL_ENTITY)
                            return;

                        auto it_id = std::find(order_.begin(), order_.end(), id);
                        auto it_target = std::find(order_.begin(), order_.end(), target);

                        if (it_id == order_.end() || it_target == order_.end())
                            return;

                        order_.erase(it_id);

                        // Iterator might have been invalidated, so find again
                        it_target = std::find(order_.begin(), order_.end(), target);

                        if (insert_after)
                            order_.insert(it_target + 1, id);
                        else
                            order_.insert(it_target, id);
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
                        // The PBR material's metallic/roughness/emissive (albedo comes from
                        // the Tint each extract). Host bookkeeping keyed on EntityId, like
                        // the shape/collider params below — no ECS component.
                        Render::Material material{};
                        // Whether the entity is tracked by physics_ (see set_has_physics_body).
                        // Unlike has_renderer/is_camera this needs no ECS migration, so it is
                        // plain host bookkeeping rather than a component toggle.
                        bool has_physics_body = false;
                        PhysicsBodyParams physics_params{};
                        // Whether a cloth grid is tracked by the physics simulation (see
                        // set_has_cloth). Same plain-host-bookkeeping treatment as
                        // has_physics_body: cloth needs no ECS component migration.
                        bool has_cloth = false;
                        ClothParams cloth_params{};
                        // Neither read nor written by any Schedule system, so — like
                        // has_physics_body/has_cloth — these are plain host bookkeeping
                        // rather than ECS components; no archetype migration needed.
                        bool has_shape = false;
                        ShapeParams shape_params{};
                        bool has_collider = false;
                        ColliderParams collider_params{};
                        // A UI element (Canvas/Panel/Image/Text/Button). Like cloth,
                        // this is host bookkeeping keyed on EntityId — no ECS
                        // migration — since the UI overlay is drawn host-side, not by
                        // any Schedule system.
                        bool has_ui = false;
                        UIElementParams ui_params{};
                        // The real ECS entity mirroring `ui_params` into `UI::` components
                        // (RectTransform/Canvas/UIImage/UIText/UIButton per `ui_mirror_kind`),
                        // so `SushiEngine::UI::resolve_rect` is the one and only layout
                        // formula anything in the engine or editor reads — see sync_ui_mirror.
                        Entity ui_mirror{};
                        UIElementKind ui_mirror_kind = UIElementKind::Image;
                        // User-defined "script" components: authoring data only (the
                        // engine has no scripting VM), attached and edited per entity
                        // and serialized with the scene.
                        std::vector<ScriptComponent> scripts{};
                    };

                    /**
                     * @brief Shared implementation of `create_box`/`create_sphere`/
                     * `create_cylinder`/`create_terrain`: a Renderer entity with a
                     * Shape and a matching Collider.
                     *
                     * Factored out because every primitive entity is spawned the same
                     * way — only the shape kind, default params, and (for Terrain)
                     * collider kind/params differ between callers.
                     *
                     * @param display_name       Display name for the new entity.
                     * @param kind               The visual Shape kind.
                     * @param default_params     The Shape's initial params.
                     * @param collider_override  When set, the Collider's kind/params, overriding
                     *                           the default of matching the Shape exactly.
                     * @return The new entity's stable id.
                     */
                    EntityId create_primitive(const std::string& display_name, PrimitiveKind kind,
                                              const Vector3& default_params,
                                              const ColliderParams* collider_override = nullptr)
                    {
                        const Entity entity = world_.spawn(
                            Transform{}, Orientation{},
                            Tint{Vector3{Scalar(0.8), Scalar(0.8), Scalar(0.8)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_renderer = true;
                        record.has_shape = true;
                        record.shape_params = ShapeParams{kind, default_params};
                        record.has_collider = true;
                        record.collider_params = collider_override != nullptr
                                                     ? *collider_override
                                                     : ColliderParams{kind, default_params};
                        records_.emplace(id, record);
                        extract();
                        return id;
                    }

                    const Record* find(EntityId id) const noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    /** @brief The script component named @p name on @p record, or null. */
                    static const ScriptComponent* find_script(const Record* record,
                                                              const std::string& name) noexcept
                    {
                        if (record == nullptr)
                            return nullptr;
                        for (const ScriptComponent& script : record->scripts)
                            if (script.type_name == name)
                                return &script;
                        return nullptr;
                    }

                    /** @brief The default rect/paint for a freshly created UI element of @p kind. */
                    static UIElementParams default_ui_params(UIElementKind kind)
                    {
                        UIElementParams params;
                        params.kind = kind;
                        switch (kind)
                        {
                            case UIElementKind::Canvas:
                                params.anchor_min_x = 0;
                                params.anchor_min_y = 0;
                                params.anchor_max_x = 1;
                                params.anchor_max_y = 1;
                                params.position_x = 0;
                                params.position_y = 0;
                                params.size_x = Scalar(1280);
                                params.size_y = Scalar(720);
                                break;
                            case UIElementKind::Text:
                                params.size_x = Scalar(200);
                                params.size_y = Scalar(40);
                                params.color = Vector3{1, 1, 1};
                                std::snprintf(params.text, sizeof(params.text), "%s", "Text");
                                break;
                            case UIElementKind::Button:
                                params.size_x = Scalar(160);
                                params.size_y = Scalar(48);
                                params.color = Vector3{Scalar(0.26), Scalar(0.5), Scalar(0.85)};
                                std::snprintf(params.text, sizeof(params.text), "%s", "Button");
                                break;
                            case UIElementKind::Image:
                            case UIElementKind::Panel:
                                params.size_x = Scalar(200);
                                params.size_y = Scalar(120);
                                params.color = Vector3{Scalar(0.85), Scalar(0.85), Scalar(0.9)};
                                break;
                        }
                        return params;
                    }

                    /** @brief Converts an authored `UIElementParams` rect into a `UI::RectTransform`. */
                    static UI::RectTransform to_rect_transform(const UIElementParams& params) noexcept
                    {
                        UI::RectTransform transform;
                        transform.anchor_min = UI::Vector2{params.anchor_min_x, params.anchor_min_y};
                        transform.anchor_max = UI::Vector2{params.anchor_max_x, params.anchor_max_y};
                        transform.pivot = UI::Vector2{params.pivot_x, params.pivot_y};
                        transform.anchored_position = UI::Vector2{params.position_x, params.position_y};
                        transform.size_delta = UI::Vector2{params.size_x, params.size_y};
                        return transform;
                    }

                    /** @brief Converts an authored fill/text colour into a `UI::Color` at full alpha-scaled opacity. */
                    static UI::Color to_ui_color(const Vector3& color, Scalar alpha) noexcept
                    {
                        return UI::Color{color.x, color.y, color.z, alpha};
                    }

                    /**
                     * @brief Mirrors @p record's `ui_params` into a real `UI::`-component ECS entity.
                     *
                     * `World` fixes an entity's component set at spawn time (no add/remove after
                     * the fact), so a UI record's mirror entity is destroyed and respawned
                     * whenever the required `UI::` component combination changes (i.e. when its
                     * `UIElementKind` changes, or the UI is first attached); otherwise the
                     * existing mirror's components are updated in place. This is the single
                     * point where host-side `UIElementParams` bookkeeping is reconciled with the
                     * real `SushiEngine::UI::` components that `SushiEngine::UI::resolve_rect`
                     * actually lays out — the editor and any runtime UI overlay both read the
                     * mirror's `UI::ComputedRect`/`UI::RectTransform`, so there is exactly one
                     * UI layout mechanism in the engine.
                     *
                     * @param record The UI record to mirror; a no-op if it carries no UI.
                     */
                    void sync_ui_mirror(Record& record)
                    {
                        if (!record.has_ui)
                        {
                            if (world_.alive(record.ui_mirror))
                                world_.destroy(record.ui_mirror);
                            record.ui_mirror = Entity{};
                            return;
                        }

                        const UIElementKind kind = record.ui_params.kind;
                        const bool needs_respawn =
                            !world_.alive(record.ui_mirror) || record.ui_mirror_kind != kind;
                        const UI::RectTransform transform = to_rect_transform(record.ui_params);

                        if (needs_respawn)
                        {
                            if (world_.alive(record.ui_mirror))
                                world_.destroy(record.ui_mirror);

                            switch (kind)
                            {
                                case UIElementKind::Canvas:
                                    record.ui_mirror = world_.spawn(
                                        UI::Canvas{UI::Vector2{record.ui_params.size_x,
                                                               record.ui_params.size_y}},
                                        transform, UI::ComputedRect{});
                                    break;
                                case UIElementKind::Text:
                                {
                                    UI::UIText text{};
                                    UI::set_text(text, record.ui_params.text);
                                    text.font_size = record.ui_params.font_size;
                                    text.color = to_ui_color(record.ui_params.color,
                                                             record.ui_params.alpha);
                                    record.ui_mirror =
                                        world_.spawn(transform, UI::ComputedRect{}, text);
                                    break;
                                }
                                case UIElementKind::Button:
                                {
                                    UI::UIButton button{};
                                    record.ui_mirror = world_.spawn(
                                        transform, UI::ComputedRect{},
                                        UI::UIImage{to_ui_color(record.ui_params.color,
                                                                record.ui_params.alpha)},
                                        button);
                                    world_.get<UI::UIButton>(record.ui_mirror).target_graphic =
                                        record.ui_mirror;
                                    break;
                                }
                                case UIElementKind::Image:
                                case UIElementKind::Panel:
                                default:
                                    record.ui_mirror = world_.spawn(
                                        transform, UI::ComputedRect{},
                                        UI::UIImage{to_ui_color(record.ui_params.color,
                                                                record.ui_params.alpha)});
                                    break;
                            }
                            record.ui_mirror_kind = kind;
                            return;
                        }

                        world_.get<UI::RectTransform>(record.ui_mirror) = transform;
                        switch (kind)
                        {
                            case UIElementKind::Canvas:
                                world_.get<UI::Canvas>(record.ui_mirror).reference_size =
                                    UI::Vector2{record.ui_params.size_x, record.ui_params.size_y};
                                break;
                            case UIElementKind::Text:
                            {
                                UI::UIText& text = world_.get<UI::UIText>(record.ui_mirror);
                                UI::set_text(text, record.ui_params.text);
                                text.font_size = record.ui_params.font_size;
                                text.color =
                                    to_ui_color(record.ui_params.color, record.ui_params.alpha);
                                break;
                            }
                            case UIElementKind::Button:
                                world_.get<UI::UIImage>(record.ui_mirror).color =
                                    to_ui_color(record.ui_params.color, record.ui_params.alpha);
                                break;
                            case UIElementKind::Image:
                            case UIElementKind::Panel:
                            default:
                                world_.get<UI::UIImage>(record.ui_mirror).color =
                                    to_ui_color(record.ui_params.color, record.ui_params.alpha);
                                break;
                        }
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
                        {
                            physics_->set_rigid_bodies(gather_rigid_descs(), PHYSICS_ITERATIONS,
                                                       substep_dt());
                            physics_dirty_ = false;
                        }
                        if (cloth_dirty_)
                        {
                            physics_->set_cloth_grids(gather_cloth_descs(), PHYSICS_ITERATIONS,
                                                      substep_dt());
                            cloth_dirty_ = false;
                        }

                        // Refresh the static collision planes every step — cheap, and it
                        // tracks a moved terrain without extra dirty bookkeeping — then
                        // step, which resolves rigid/rigid, rigid/plane, and cloth/rigid
                        // contacts inside the solve.
                        physics_->set_static_planes(gather_static_planes());
                        physics_->step(Vector3{0, PHYSICS_GRAVITY_Y, 0}, PHYSICS_SUBSTEPS_PER_TICK);

                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_physics_body ||
                                !world_.alive(record->entity))
                                continue;
                            SolvedPose pose;
                            if (physics_->rigid_pose(id, pose))
                            {
                                world_.get<Transform>(record->entity).position = pose.position;
                                world_.get<Orientation>(record->entity).rotation = pose.orientation;
                            }
                        }

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
                     * @brief Collects a descriptor per Rigid Body entity for a rebuild.
                     *
                     * The pose seeds a newly added body; the physics simulation ignores it
                     * for a body it already tracks, carrying that body's live state over
                     * instead (see `IPhysicsSimulation::set_rigid_bodies`). Built fresh each
                     * rebuild from the current entity set, so a destroyed entity simply drops
                     * out of the list.
                     *
                     * @return One descriptor per live physics-driven entity, in display order.
                     */
                    std::vector<RigidBodyDesc> gather_rigid_descs() const
                    {
                        std::vector<RigidBodyDesc> descs;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_physics_body ||
                                !world_.alive(record->entity))
                                continue;
                            RigidBodyDesc desc;
                            desc.id = id;
                            desc.position = world_.get<Transform>(record->entity).position;
                            desc.orientation = world_.get<Orientation>(record->entity).rotation;
                            desc.inv_mass = record->physics_params.inv_mass;
                            desc.inv_inertia = record->physics_params.inv_inertia;
                            desc.radius = collision_radius(*record);
                            // A Box collider (or, absent one, a Box visual) collides as an
                            // oriented box; anything else falls back to a sphere of radius.
                            if (record->has_collider)
                            {
                                desc.box = record->collider_params.kind == PrimitiveKind::Box;
                                desc.half_extents = record->collider_params.params;
                            }
                            else if (record->has_shape)
                            {
                                desc.box = record->shape_params.kind == PrimitiveKind::Box;
                                desc.half_extents = record->shape_params.params;
                            }
                            descs.push_back(desc);
                        }
                        return descs;
                    }

                    /**
                     * @brief The collision radius a body collides as (contacts treat bodies as spheres).
                     *
                     * Taken from the entity's Collider if it has one, else its visual
                     * Shape, else a unit default. A Box/Cylinder uses its smallest
                     * half-extent so it rests on the ground at the right height rather
                     * than hovering by its bounding radius.
                     */
                    static Scalar collision_radius(const Record& record) noexcept
                    {
                        const auto radius_of = [](PrimitiveKind kind, const Vector3& p) -> Scalar
                        {
                            switch (kind)
                            {
                                case PrimitiveKind::Sphere:
                                    return p.x;
                                case PrimitiveKind::Cylinder:
                                    return p.x;
                                case PrimitiveKind::Box:
                                {
                                    const Scalar xy = p.x < p.y ? p.x : p.y;
                                    return xy < p.z ? xy : p.z;
                                }
                                case PrimitiveKind::Plane:
                                    return Scalar(0.25);
                            }
                            return Scalar(0.5);
                        };
                        if (record.has_collider)
                            return radius_of(record.collider_params.kind, record.collider_params.params);
                        if (record.has_shape)
                            return radius_of(record.shape_params.kind, record.shape_params.params);
                        return Scalar(0.5);
                    }

                    /**
                     * @brief Collects the scene's static collision planes from Plane colliders.
                     *
                     * Every entity carrying a `Plane` Collider (e.g. Terrain) becomes one
                     * static half-space: its collider's local normal rotated into world
                     * space at the entity's world position. Bodies and cloth are pushed
                     * out of these each sub-step. An entity with a Plane collider and a
                     * Rigid Body is skipped — a moving plane is not a static surface.
                     *
                     * @return One plane per static Plane collider in the scene.
                     */
                    std::vector<PlaneDesc> gather_static_planes() const
                    {
                        std::vector<PlaneDesc> planes;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_collider ||
                                record->has_physics_body || !world_.alive(record->entity) ||
                                record->collider_params.kind != PrimitiveKind::Plane)
                                continue;
                            const EntityTransform world = world_transform(id);
                            PlaneDesc plane;
                            plane.point = world.position;
                            plane.normal = rotate(world.rotation, record->collider_params.params);
                            planes.push_back(plane);
                        }
                        return planes;
                    }

                    /**
                     * @brief Collects a descriptor per Cloth entity for a rebuild.
                     *
                     * Each grid originates at its entity's `Transform::position`, mirroring
                     * how a Rigid Body seeds from its pose. Built fresh each rebuild, so a
                     * destroyed or detached cloth simply drops out of the list; unlike a
                     * Rigid Body no live state is carried over, since a rows/cols/spacing
                     * change replaces the grid topology outright.
                     *
                     * @return One descriptor per live cloth entity with a non-degenerate grid.
                     */
                    std::vector<ClothDesc> gather_cloth_descs() const
                    {
                        std::vector<ClothDesc> descs;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_cloth ||
                                !world_.alive(record->entity) ||
                                record->cloth_params.rows == 0 || record->cloth_params.cols == 0)
                                continue;
                            ClothDesc desc;
                            desc.id = id;
                            desc.rows = record->cloth_params.rows;
                            desc.cols = record->cloth_params.cols;
                            desc.spacing = record->cloth_params.spacing;
                            desc.origin = world_.get<Transform>(record->entity).position;
                            desc.compliance = record->cloth_params.compliance;
                            desc.thickness = record->cloth_params.spacing * Scalar(0.25);
                            descs.push_back(desc);
                        }
                        return descs;
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
                     * skipped. Drawing gates on `has_shape` rather than `has_renderer` —
                     * a bare entity with a Renderer but no authored Shape has nothing to
                     * draw a mesh from, matching how a bare "Create Entity" is a plain
                     * Transform, not a disguised cube. Also rebuilds the cloth wireframe
                     * list from every live grid's current particle positions. Run after
                     * every tick and after every edit so the view always matches the world.
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

                            if (!record->visible || !record->has_shape || !record->has_renderer)
                                continue;
                            const Tint& tint = world_.get<Tint>(record->entity);
                            RenderInstance instance;
                            instance.id = id;
                            instance.model = world_matrix(id);
                            instance.color = tint.color;
                            instance.shape_kind = record->shape_params.kind;
                            instance.shape_params = record->shape_params.params;
                            // Albedo tracks the entity's Tint; the rest of the PBR material is
                            // the authored per-entity record.
                            instance.material = record->material;
                            instance.material.albedo = tint.color;
                            scene_.instances.push_back(instance);
                        }

                        scene_.cloth_instances.clear();
                        scene_.cloth_vertices.clear();
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_cloth || !record->visible)
                                continue;
                            std::uint32_t rows = 0;
                            std::uint32_t cols = 0;
                            std::vector<Vector3> positions;
                            if (physics_->cloth_dimensions(id, rows, cols))
                                positions = physics_->cloth_positions(id);
                            if (positions.empty() && world_.alive(record->entity))
                            {
                                // No simulated grid yet (edit mode, before the first
                                // tick): synthesize a flat resting sheet matching
                                // build_cloth_grid's layout (origin + (col, 0, row) *
                                // spacing) so a newly created Cloth is visible at once.
                                // Once the world is played the simulated positions above
                                // take over.
                                rows = static_cast<std::uint32_t>(record->cloth_params.rows);
                                cols = static_cast<std::uint32_t>(record->cloth_params.cols);
                                if (rows == 0 || cols == 0)
                                    continue;
                                const Vector3 origin =
                                    world_.get<Transform>(record->entity).position;
                                const Scalar spacing = record->cloth_params.spacing;
                                positions.reserve(static_cast<std::size_t>(rows) * cols);
                                for (std::uint32_t r = 0; r < rows; ++r)
                                    for (std::uint32_t c = 0; c < cols; ++c)
                                        positions.push_back(
                                            Vector3{origin.x + Scalar(c) * spacing, origin.y,
                                                    origin.z + Scalar(r) * spacing});
                            }
                            if (positions.empty())
                                continue;
                            ClothInstance cloth_instance;
                            cloth_instance.id = id;
                            cloth_instance.rows = rows;
                            cloth_instance.cols = cols;
                            cloth_instance.first_vertex =
                                static_cast<std::uint32_t>(scene_.cloth_vertices.size());
                            scene_.cloth_vertices.insert(scene_.cloth_vertices.end(),
                                                         positions.begin(), positions.end());
                            scene_.cloth_instances.push_back(cloth_instance);
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
                    // The most recent size a host reported via set_ui_target_size(), used to
                    // keep a Canvas's authored size tracking the actual viewport.
                    struct
                    {
                        std::uint32_t x = 1280;
                        std::uint32_t y = 720;
                    } ui_target_size_;
                    RenderScene scene_;
                    // The physics solve, behind a seam. It owns the rigid and cloth
                    // PhysicsWorlds; this class only marshals entity poses to and from it.
                    std::unique_ptr<IPhysicsSimulation> physics_;
                    bool physics_dirty_ = false;
                    bool cloth_dirty_ = false;
            };
        } // namespace

        std::unique_ptr<ISimulation> create_simulation()
        {
            return std::unique_ptr<ISimulation>(new RuntimeSimulation());
        }
    } // namespace Simulation
} // namespace SushiEngine
