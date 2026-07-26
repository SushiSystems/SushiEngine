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
#include <SushiEngine/animation/device_batch_evaluator.hpp>
#include <SushiEngine/animation/gltf_skeleton_import.hpp>
#include <SushiEngine/astro/astro_dynamics.hpp>
#include <SushiEngine/astro/scene_frame.hpp>
#include <SushiEngine/astro/surface_frame.hpp>
#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/physics/cloth.hpp>
#include <SushiEngine/sim/components.hpp>
#include <SushiEngine/sim/physics_simulation.hpp>
#include <SushiEngine/sim/simulation.hpp>
#include <SushiEngine/sim/weather_cloudscape_compiler.hpp>
#include <SushiEngine/sim/weather_field_buffer.hpp>
#include <SushiEngine/sim/weather_wind.hpp>
#include <SushiEngine/sim/weather_world_coupling.hpp>
#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/deterministic_backend.hpp>
#include <SushiEngine/vfx/effect_database.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

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
                          physics_(create_physics_simulation(runtime_)),
                          crowd_evaluator_(runtime_)
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

                    double julian_date() const noexcept override { return julian_date_; }

                    void set_julian_date(double julian_date) override
                    {
                        julian_date_ = julian_date;
                        scene_.environment.observer.julian_date = julian_date;
                        extract();
                    }

                    void set_time_scale_days_per_second(Scalar days_per_second) override
                    {
                        time_scale_days_per_second_ = double(days_per_second);
                    }

                    void set_sky_observer(const Render::SkyObserver& observer) noexcept override
                    {
                        // Adopt the observer's geometry and anchor body for astro placement,
                        // but keep the epoch — the sim owns the master clock, and extract
                        // re-stamps observer.julian_date from it.
                        const double epoch = scene_.environment.observer.julian_date;
                        scene_.environment.observer = observer;
                        scene_.environment.observer.julian_date = epoch;
                    }

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
                        // Capture whether this edit moves the entity, before overwriting — a
                        // pure rotation (no move) is the only case that may re-derive the
                        // ground-local orientation below, so translation can never tilt it.
                        const bool moved =
                            length(value.position - t.position) > Scalar(1e-4);
                        t.position = value.position;
                        t.scale = value.scale;
                        o.rotation = value.rotation;
                        // A surface-anchored entity's scene rotation is re-derived from its
                        // ground-local orientation each step, so a gizmo *rotation* (position
                        // unchanged) must update that stored orientation or it would be lost.
                        // Only when unmoved: at the same position the tangent frame is
                        // identical, so the strip round-trips exactly; a translation leaves the
                        // ground-local pose untouched, keeping "upright" upright at any latitude.
                        if (record->surface_anchored && !moved)
                            record->surface_local_orientation =
                                ground_local_orientation(value.position, value.rotation);
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

                    EntityFrame entity_frame(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr)
                            return EntityFrame{};
                        return EntityFrame{record->reference_body, record->frame_mode};
                    }

                    void set_entity_frame(EntityId id, const EntityFrame& frame) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->reference_body = frame.reference_body;
                        record->frame_mode = frame.mode;
                        // Surface is the ground-anchored orientation path; keep the existing
                        // surface-anchoring flag in step so apply_surface_constraints drives it,
                        // seeding the ground-local orientation from the current pose on enable.
                        const bool surface = resolved_frame_mode(*record) == FrameMode::Surface;
                        if (record->surface_anchored != surface)
                        {
                            record->surface_anchored = surface;
                            if (surface && world_.alive(record->entity))
                                record->surface_local_orientation =
                                    world_.get<Orientation>(record->entity).rotation;
                        }
                        extract();
                    }

                    EntityTransform frame_local_transform(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !world_.alive(record->entity))
                            return EntityTransform{};
                        const Transform& t = world_.get<Transform>(record->entity);
                        const Orientation& o = world_.get<Orientation>(record->entity);
                        EntityTransform out;
                        out.scale = t.scale;
                        if (resolved_frame_mode(*record) == FrameMode::Surface)
                        {
                            // Geodetic: the place-on-a-planet coordinate. x=lat°, y=lon°,
                            // z=altitude m — how an author thinks about a spawn, not a raw
                            // millions-of-metres Cartesian offset. Rotation is ground-local.
                            const Astro::GeodeticCoordinate geo = Astro::body_fixed_to_geodetic(
                                static_cast<Astro::BodyId>(record->reference_body),
                                scene_to_body_fixed(record->reference_body, t.position));
                            out.position =
                                Vector3{Scalar(geo.latitude_radians * Astro::RADIANS_TO_DEGREES),
                                        Scalar(geo.longitude_radians * Astro::RADIANS_TO_DEGREES),
                                        Scalar(geo.altitude_metres)};
                            out.rotation = record->surface_local_orientation;
                        }
                        else
                        {
                            out.position =
                                t.position - reference_center_scene(record->reference_body);
                            out.rotation = o.rotation;
                        }
                        return out;
                    }

                    void set_frame_local_transform(EntityId id,
                                                   const EntityTransform& local) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !world_.alive(record->entity))
                            return;
                        Transform& t = world_.get<Transform>(record->entity);
                        Orientation& o = world_.get<Orientation>(record->entity);
                        t.scale = local.scale;
                        if (resolved_frame_mode(*record) == FrameMode::Surface)
                        {
                            // Position is geodetic (lat°, lon°, altitude m); place it on the
                            // reference ellipsoid and store the ground-local orientation, from
                            // which apply_surface_constraints derives the scene rotation.
                            const Astro::GeodeticCoordinate geo{
                                double(local.position.x) * Astro::DEGREES_TO_RADIANS,
                                double(local.position.y) * Astro::DEGREES_TO_RADIANS,
                                double(local.position.z)};
                            const Vector3 body_fixed = Astro::geodetic_to_body_fixed(
                                static_cast<Astro::BodyId>(record->reference_body), geo);
                            t.position = body_fixed_to_scene(record->reference_body, body_fixed);
                            record->surface_anchored = true;
                            record->surface_local_orientation = normalize(local.rotation);
                            o.rotation = record->surface_local_orientation;
                        }
                        else
                        {
                            t.position =
                                reference_center_scene(record->reference_body) + local.position;
                            o.rotation = local.rotation;
                        }
                        if (record->has_physics_body)
                            physics_->set_rigid_pose(id, t.position, o.rotation);
                        extract();
                    }

                    bool is_surface_frame(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->reference_body >= 0 &&
                               resolved_frame_mode(*record) == FrameMode::Surface;
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
                        // The observer epoch the environment carries seeds the master
                        // clock, so authoring the sky date or loading a scene sets the
                        // simulation's "now"; thereafter the sim owns and advances it.
                        julian_date_ = value.observer.julian_date;
                        extract();
                    }

                    /**
                     * @brief Installs (or clears) the weather provider and caches its capability.
                     *
                     * The one place a concrete provider type is named. Everything after this
                     * point — the tick, the column sample, the field publish, the editor's
                     * authoring surface — goes through an interface, which is what makes any
                     * implementation of the seam installable rather than just the one the host
                     * happened to be written against.
                     *
                     * @param provider The provider to take ownership of, or null to clear.
                     */
                    void install_weather_provider(std::unique_ptr<IWeatherProvider> provider)
                    {
                        weather_provider_ = std::move(provider);
                        weather_authoring_ =
                            dynamic_cast<IWeatherAuthoring*>(weather_provider_.get());
                    }

                    /**
                     * @brief The geodetic position of a scene-space XZ point.
                     *
                     * The scene is a flat tangent patch anchored at the sky observer's geodetic
                     * position, with **+X east and +Z north** — the same convention
                     * `weather_field_buffer.hpp` states for the published field and the rain
                     * emitter's wind drift already assumes. Sharing one mapping is what keeps
                     * the column sampled here and the field sampled in the shader describing the
                     * same point.
                     *
                     * @param x Scene X, metres.
                     * @param z Scene Z, metres.
                     * @return The geodetic position of that point.
                     */
                    GeodeticPosition geodetic_at_scene(double x, double z) const
                    {
                        constexpr double MIN_COS_LATITUDE = 0.05;
                        const double radius = std::max(double(scene_.environment.planet.mean_radius()), 1.0);
                        const double latitude = scene_.environment.observer.latitude_radians;
                        const double cos_latitude = std::max(std::cos(latitude), MIN_COS_LATITUDE);
                        return GeodeticPosition{latitude + z / radius,
                                                scene_.environment.observer.longitude_radians +
                                                    x / (radius * cos_latitude)};
                    }

                    bool procedural_weather_enabled() const noexcept override
                    {
                        return static_cast<bool>(weather_provider_);
                    }

                    void set_procedural_weather_enabled(bool value) override
                    {
                        if (value == static_cast<bool>(weather_provider_))
                            return;
                        if (value)
                        {
                            // A fixed default seed for this phase's wiring: the editor's Weather
                            // panel v2 owns exposing a reseed control on top of this seam later.
                            // The scene planet's mean radius anchors T1/T2's tangent-plane math to
                            // whichever body is dominant, matching Environment::planet already.
                            constexpr std::uint64_t DEFAULT_WEATHER_SEED = 1;
                            install_weather_provider(std::make_unique<ProceduralWeather>(
                                DEFAULT_WEATHER_SEED, scene_.environment.planet.mean_radius()));
                            // W5's acceptance bar wants reduced visibility to actually show up
                            // under rain; froxel fog stays author-off by default (FogParams::enabled
                            // = false) since most scenes never touch weather. A one-shot default
                            // nudge here, not a continuous override: it only fires the instant
                            // procedural weather turns on, and the author's own choice afterward
                            // (including turning it back off) is never re-applied on top of it --
                            // the same "fixed default, not a running override" shape as this
                            // function's seed above.
                            if (!scene_.environment.fog.enabled)
                                scene_.environment.fog.enabled = true;
                        }
                        else
                        {
                            install_weather_provider(nullptr);
                        }
                        extract();
                    }

                    IWeatherAuthoring* weather_authoring() noexcept override
                    {
                        return weather_authoring_;
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
                        physics_->update_rigid_body_params(id, params.inv_mass, params.inv_inertia,
                                                           params.drag_coefficient);
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

                    bool has_crowd(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_crowd;
                    }

                    CrowdParams crowd_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->crowd_params : CrowdParams{};
                    }

                    void set_crowd_params(EntityId id, const CrowdParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->crowd_params = params;
                    }

                    void set_has_crowd(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->has_crowd = value;
                    }

                    std::uint32_t register_crowd_skeleton(const std::string& gltf_path) override
                    {
                        const auto cached = crowd_skeleton_cache_.find(gltf_path);
                        if (cached != crowd_skeleton_cache_.end())
                            return cached->second;

                        std::vector<std::byte> blob;
                        if (!Animation::import_gltf_skeleton(gltf_path.c_str(), blob))
                            return 0;
                        const Animation::AssetId asset_id = crowd_database_.add_skeleton(std::move(blob));
                        if (asset_id == Animation::INVALID_ASSET)
                            return 0;

                        crowd_skeletons_.push_back(asset_id);
                        const std::uint32_t handle = static_cast<std::uint32_t>(crowd_skeletons_.size());
                        crowd_skeleton_cache_.emplace(gltf_path, handle);
                        return handle;
                    }

                    std::uint32_t register_crowd_clip(const std::string& gltf_path,
                                                      std::uint32_t skeleton_handle) override
                    {
                        if (skeleton_handle == 0 || skeleton_handle > crowd_skeletons_.size())
                            return 0;
                        const std::string cache_key =
                            gltf_path + "|" + std::to_string(skeleton_handle);
                        const auto cached = crowd_clip_cache_.find(cache_key);
                        if (cached != crowd_clip_cache_.end())
                            return cached->second;

                        // The joint order this produces must match the registered skeleton's —
                        // true when both come from the same file (the common case), a real,
                        // undetected mismatch if not (no cross-check performed here; a caller
                        // mixing skeleton and clip files takes that on faith, same as
                        // Animation::retarget_clip's own documented assumptions elsewhere).
                        Animation::GltfAnimationImport import;
                        if (!Animation::import_gltf_animated(gltf_path.c_str(), import) ||
                            import.clips.empty())
                            return 0;
                        // The first animation in the file — selecting one by name is real,
                        // unbuilt follow-up scope (see register_crowd_clip's own Doxygen).
                        const Animation::AssetId asset_id =
                            crowd_database_.add_clip(std::move(import.clips.front().blob));
                        if (asset_id == Animation::INVALID_ASSET)
                            return 0;

                        crowd_clips_.push_back(asset_id);
                        const std::uint32_t handle = static_cast<std::uint32_t>(crowd_clips_.size());
                        crowd_clip_cache_.emplace(cache_key, handle);
                        return handle;
                    }

                    bool has_light(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_light;
                    }

                    LightParams light_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->light_params : LightParams{};
                    }

                    void set_light_params(EntityId id, const LightParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_light)
                            return;
                        record->light_params = params;
                        extract();
                    }

                    void set_has_light(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_light == value)
                            return;
                        record->has_light = value;
                        extract();
                    }

                    // Audio authoring: plain host bookkeeping read live by the editor's audio
                    // system, so the setters do not re-extract the render snapshot (audio is
                    // not part of RenderScene).
                    bool has_audio_emitter(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_audio_emitter;
                    }

                    AudioEmitterParams audio_emitter_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->audio_emitter_params : AudioEmitterParams{};
                    }

                    void set_audio_emitter_params(EntityId id, const AudioEmitterParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_audio_emitter)
                            return;
                        record->audio_emitter_params = params;
                    }

                    void set_has_audio_emitter(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_audio_emitter == value)
                            return;
                        record->has_audio_emitter = value;
                    }

                    bool has_reverb_zone(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_reverb_zone;
                    }

                    ReverbZoneParams reverb_zone_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->reverb_zone_params : ReverbZoneParams{};
                    }

                    void set_reverb_zone_params(EntityId id, const ReverbZoneParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_reverb_zone)
                            return;
                        record->reverb_zone_params = params;
                    }

                    void set_has_reverb_zone(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_reverb_zone == value)
                            return;
                        record->has_reverb_zone = value;
                    }

                    bool has_audio_listener(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_audio_listener;
                    }

                    AudioListenerParams audio_listener_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->audio_listener_params : AudioListenerParams{};
                    }

                    void set_audio_listener_params(EntityId id, const AudioListenerParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_audio_listener)
                            return;
                        record->audio_listener_params = params;
                    }

                    void set_has_audio_listener(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_audio_listener == value)
                            return;
                        record->has_audio_listener = value;
                    }

                    EntityId create_light(const std::string& display_name) override
                    {
                        // A bare entity carrying a light record: no Renderer/Shape, since a
                        // light is not drawn. Its Transform places and (for a spot) aims it,
                        // so moving the entity moves the light — the mesh-instance pattern.
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_light = true;
                        records_.emplace(id, record);
                        extract();
                        return id;
                    }

                    bool has_decal(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_decal;
                    }

                    DecalParams decal_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->decal_params : DecalParams{};
                    }

                    void set_decal_params(EntityId id, const DecalParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_decal)
                            return;
                        record->decal_params = params;
                        extract();
                    }

                    void set_has_decal(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_decal == value)
                            return;
                        record->has_decal = value;
                        extract();
                    }

                    EntityId create_decal(const std::string& display_name) override
                    {
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_decal = true;
                        records_.emplace(id, record);
                        extract();
                        return id;
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
                        {
                            world_.get<Tint>(record->entity).color =
                                Vector3{Scalar(0.35), Scalar(0.55), Scalar(0.3)};
                            // The ground is the acceptance bar's own "wet ground" symptom (design
                            // doc §7, W5); opt it in by default so the demo scenario shows it
                            // without extra authoring, matching the material-flag-gated pattern
                            // (MaterialFlags::MATERIAL_WEATHER_WET) every other material still
                            // opts into explicitly.
                            record->material.weather_wettable = true;
                        }
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

                    EntityId create_crowd(const std::string& display_name) override
                    {
                        // A bare entity that owns crowd-batched animation: no Renderer/Shape
                        // (a crowd character draws through RenderScene::skinned_instances, not
                        // a solid-mesh RenderInstance).
                        const Entity entity = world_.spawn(Transform{}, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_crowd = true;
                        records_.emplace(id, record);
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

                    bool surface_anchored(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->surface_anchored;
                    }

                    Quaternion surface_local_orientation(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->surface_local_orientation
                                                 : Quaternion{};
                    }

                    void set_surface_local_orientation(EntityId id,
                                                       const Quaternion& rotation) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->surface_anchored)
                            return;
                        record->surface_local_orientation = normalize(rotation);
                    }

                    void set_surface_anchored(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->surface_anchored == value)
                            return;
                        record->surface_anchored = value;
                        // Seed the ground-local orientation from the entity's current pose so
                        // anchoring does not snap it: the stored scene-frame rotation becomes
                        // the local one, and the next resolve composes the tangent frame in.
                        if (value && world_.alive(record->entity))
                            record->surface_local_orientation =
                                world_.get<Orientation>(record->entity).rotation;
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

                    EntityId create_particle_emitter(const std::string& display_name) override
                    {
                        Transform transform;
                        transform.position = Vector3{0, Scalar(1), 0};
                        const Entity entity = world_.spawn(transform, Orientation{});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false, false};
                        record.has_particle_emitter = true;
                        Vfx::CpuDeterministicBackend::reset(record.particle_pool,
                                                            record.emitter_params.seed);
                        seed_emitter_effect(record);
                        records_.emplace(id, std::move(record));
                        extract();
                        return id;
                    }

                    bool has_particle_emitter(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_particle_emitter;
                    }

                    ParticleEmitterParams particle_emitter_params(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return (record != nullptr && record->has_particle_emitter)
                                   ? record->emitter_params
                                   : ParticleEmitterParams{};
                    }

                    void set_particle_emitter_params(EntityId id,
                                                     const ParticleEmitterParams& params) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_particle_emitter)
                            return;
                        // A seed change restarts the deterministic stream, so the emitter reflects
                        // the new choice from a clean pool.
                        const bool restart = record->emitter_params.seed != params.seed;
                        record->emitter_params = params;
                        if (restart)
                            Vfx::CpuDeterministicBackend::reset(record->particle_pool, params.seed);
                        extract();
                    }

                    void set_has_particle_emitter(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_particle_emitter == value)
                            return;
                        record->has_particle_emitter = value;
                        if (value)
                        {
                            Vfx::CpuDeterministicBackend::reset(record->particle_pool,
                                                                record->emitter_params.seed);
                            seed_emitter_effect(*record);
                        }
                        extract();
                    }

                    const Vfx::ParticleEffect& particle_effect_source(EntityId id) const override
                    {
                        static const Vfx::ParticleEffect EMPTY{};
                        const Record* record = find(id);
                        return (record != nullptr && record->has_particle_emitter)
                                   ? record->effect_source
                                   : EMPTY;
                    }

                    void set_particle_effect_source(EntityId id,
                                                    const Vfx::ParticleEffect& effect) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_particle_emitter)
                            return;
                        record->effect_source = effect;
                        if (record->effect_asset == Vfx::INVALID_EFFECT)
                            record->effect_asset = effect_db_.add(effect);
                        else
                            effect_db_.replace(record->effect_asset, effect);
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
                        // A crowd-batched skinned character (design §12.3/§12.4): same plain
                        // host bookkeeping as cloth (no ECS migration) — playback time is
                        // advanced on the fixed tick and sampled through crowd_evaluator_ at
                        // extract, keyed by EntityId, not a per-instance component.
                        bool has_crowd = false;
                        CrowdParams crowd_params{};
                        // A punctual light on this entity: same plain host bookkeeping as
                        // cloth/shape, extracted into RenderScene::lights each frame with
                        // the entity's transform supplying the light's position and aim.
                        bool has_light = false;
                        LightParams light_params{};
                        // Audio authoring, same plain host bookkeeping as light/cloth (no ECS
                        // migration): an emitter plays a sound at the entity's transform, a
                        // reverb zone imposes its I3DL2 reverb on a listener inside its box, and
                        // the listener marks the ears. Read live by the editor's audio system
                        // through the IWorldEditor accessors each wall-clock frame.
                        bool has_audio_emitter = false;
                        AudioEmitterParams audio_emitter_params{};
                        bool has_reverb_zone = false;
                        ReverbZoneParams reverb_zone_params{};
                        bool has_audio_listener = false;
                        AudioListenerParams audio_listener_params{};
                        // A projected decal on this entity, same host bookkeeping as the
                        // light, extracted into RenderScene::decals each frame.
                        bool has_decal = false;
                        DecalParams decal_params{};
                        // A deterministic particle emitter: same plain host bookkeeping as
                        // cloth (no ECS migration). The ~80 KB fixed pool lives here, off the
                        // ECS chunk, and is advanced on the fixed tick and extracted to
                        // RenderScene::particle_billboards each frame.
                        bool has_particle_emitter = false;
                        ParticleEmitterParams emitter_params{};
                        Vfx::DeterministicEmitterState particle_pool{};
                        /**
                         * @brief Runtime emitter state, kept off @ref emitter_params.
                         *
                         * Those are the *authored* parameters the editor round-trips through the
                         * scene file; a play head and a fractional-spawn carry are neither authored
                         * nor persisted, so they live here with the pool they belong to.
                         */
                        float emitter_time = 0.0f;
                        float emitter_spawn_carry = 0.0f;
                        /**
                         * @brief The effect this emitter owns, and its slot in the database.
                         *
                         * Per entity, not a reference into a shared library: the component is what
                         * makes the entity a particle system, so the effect is its own data. The
                         * asset id is allocated on the first assignment and then reused, so
                         * repeated edits replace in place instead of growing the database.
                         */
                        Vfx::ParticleEffect effect_source{};
                        Vfx::AssetId effect_asset = Vfx::INVALID_EFFECT;
                        // Neither read nor written by any Schedule system, so — like
                        // has_physics_body/has_cloth — these are plain host bookkeeping
                        // rather than ECS components; no archetype migration needed.
                        bool has_shape = false;
                        ShapeParams shape_params{};
                        bool has_collider = false;
                        ColliderParams collider_params{};
                        // Whether the entity's orientation is planet-surface anchored (see
                        // set_surface_anchored): its stored orientation is ground-local
                        // (relative to the local East-North-Up frame on the dominant body),
                        // composed onto the tangent frame each step so "upright" is identity
                        // anywhere on the body. Plain host bookkeeping like the toggles above.
                        bool surface_anchored = false;
                        Quaternion surface_local_orientation{};
                        // The reference frame the inspector authors this entity's transform in
                        // (a celestial body + Free/Surface/Auto mode). Purely an authoring-
                        // boundary projection: the scene-frame Transform stays the working
                        // truth for physics and render — see frame_local_transform. -1 is the
                        // scene root (no offset), so an entity that never picks a body is
                        // unchanged. Surface mode drives surface_anchored above.
                        int reference_body = -1;
                        FrameMode frame_mode = FrameMode::Auto;
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
                     * @brief Builds the per-body gravity field the physics predict samples.
                     *
                     * When the scene has a dominant celestial body, gravity is the summed
                     * on-rails field (the injected @ref Astro::IGravityField, the one gravity
                     * source shared with the orbital integrator) sampled at each body's *own*
                     * heliocentric position — so it carries the real magnitude for Earth,
                     * Mars, the Moon, or any catalogued body, falls off with altitude, and
                     * curves toward the attractor rather than pointing at a fixed world
                     * "down". Unlike the earlier single dominant term evaluated once at the
                     * scene origin, this is a true field: a body high above the surface and
                     * one at the ground feel their correct, different pulls, and third-body
                     * tidal terms are included. The scene frame is constant across the step,
                     * so it is built once and captured; the sampler does one position
                     * bijection and one direction rotation per evaluation. With no dominant
                     * body (a non-astronomical scene, or deep space) it returns the uniform
                     * local demo gravity, keeping the plain physics sandboxes unchanged.
                     *
                     * @return A sampler mapping a scene-frame position to its gravity, m/s².
                     */
                    GravitySampler make_gravity_sampler() const
                    {
                        const int dominant = scene_.environment.dominant_body_id;
                        if (dominant < 0)
                        {
                            // A non-astronomical scene (or deep space): the local demo
                            // gravity, uniform everywhere, keeps the plain sandboxes unchanged.
                            const Vector3 demo{0, PHYSICS_GRAVITY_Y, 0};
                            return [demo](const Vector3&) noexcept { return demo; };
                        }

                        // The one gravity source: the injected summed field, sampled per body
                        // at its true heliocentric position. The scene frame is constant across
                        // the step (it depends only on the epoch and observer), so it is built
                        // once here and captured — the sampler then does one bijection and one
                        // rotation per body: scene position -> heliocentric -> field ->
                        // acceleration rotated back into scene axes.
                        const Render::SkyObserver& observer = scene_.environment.observer;
                        const Astro::SceneFrame frame = Astro::scene_frame_for(
                            julian_date_, observer.latitude_radians, observer.longitude_radians,
                            static_cast<Astro::BodyId>(observer.observer_body));
                        const double julian_date = julian_date_;
                        const Astro::IGravityField* field = &gravity_field_;
                        return [frame, julian_date, field](const Vector3& scene_position) noexcept -> Vector3
                        {
                            const WorldVector3 heliocentric = frame.heliocentric_from_scene(
                                WorldVector3{scene_position.x, scene_position.y, scene_position.z});
                            const WorldVector3 pull = field->acceleration(heliocentric, julian_date);
                            return frame.direction_to_scene(
                                Vector3{Scalar(pull.x), Scalar(pull.y), Scalar(pull.z)});
                        };
                    }

                    /**
                     * @brief The dominant celestial body this frame, if any.
                     * @param body_out Receives the body when the scene has one.
                     * @return True when a dominant body is set (a planet is the ground).
                     */
                    bool dominant_body(Astro::BodyId& body_out) const noexcept
                    {
                        const int dominant = scene_.environment.dominant_body_id;
                        if (dominant < 0)
                            return false;
                        body_out = static_cast<Astro::BodyId>(dominant);
                        return true;
                    }

                    /** @brief The dominant body's centre in the scene frame, metres. */
                    Vector3 planet_center_scene() const noexcept
                    {
                        const WorldVector3 center = scene_.environment.planet_center;
                        return Vector3{Scalar(center.x), Scalar(center.y), Scalar(center.z)};
                    }

                    /**
                     * @brief A reference body's centre in the scene frame, metres.
                     *
                     * The origin an entity's frame-local transform is offset from. Placed
                     * through the scene-frame bijection at the master epoch (the same map the
                     * ephemeris uses), so it is exact for any catalogued body, not only the
                     * dominant one. A body index of -1 is the scene root, whose centre is the
                     * origin — so a frame-local transform then equals the scene transform.
                     *
                     * @param body Celestial body index, or -1 for the scene root.
                     * @return The body centre in scene metres (origin when @p body is -1).
                     */
                    Vector3 reference_center_scene(int body) const
                    {
                        if (body < 0)
                            return Vector3{Scalar(0), Scalar(0), Scalar(0)};
                        const Vector3 helio_au = Astro::planet_heliocentric_au(
                            static_cast<Astro::BodyId>(body), julian_date_);
                        const WorldVector3 helio{
                            double(helio_au.x) * Astro::METRES_PER_ASTRONOMICAL_UNIT,
                            double(helio_au.y) * Astro::METRES_PER_ASTRONOMICAL_UNIT,
                            double(helio_au.z) * Astro::METRES_PER_ASTRONOMICAL_UNIT};
                        const WorldVector3 scene =
                            current_scene_frame().scene_from_heliocentric(helio);
                        return Vector3{Scalar(scene.x), Scalar(scene.y), Scalar(scene.z)};
                    }

                    /** @brief The scene-frame bijection for the current epoch and observer. */
                    Astro::SceneFrame current_scene_frame() const
                    {
                        const Render::SkyObserver& observer = scene_.environment.observer;
                        return Astro::scene_frame_for(
                            julian_date_, observer.latitude_radians, observer.longitude_radians,
                            static_cast<Astro::BodyId>(observer.observer_body));
                    }

                    /** @brief Rotates a vector about the +Z body pole by @p angle radians. */
                    static Vector3 rotate_about_pole(const Vector3& v, Scalar angle) noexcept
                    {
                        const Scalar c = std::cos(angle);
                        const Scalar s = std::sin(angle);
                        return Vector3{v.x * c - v.y * s, v.x * s + v.y * c, v.z};
                    }

                    /**
                     * @brief Body-fixed Cartesian metres of a scene position, centred on @p body.
                     *
                     * Scene offset → ecliptic (the scene-frame rotation) → the body's equatorial
                     * frame → body-fixed by unwinding the prime-meridian spin W(t). The inverse
                     * of @ref body_fixed_to_scene; together they carry a surface pose between the
                     * scene and the geodetic coordinate the inspector authors.
                     *
                     * @param body           Reference body index.
                     * @param scene_position A position in scene metres.
                     * @return Body-fixed Cartesian metres (origin at the body centre).
                     */
                    Vector3 scene_to_body_fixed(int body, const Vector3& scene_position) const
                    {
                        const Astro::BodyId body_id = static_cast<Astro::BodyId>(body);
                        const Astro::SceneFrame frame = current_scene_frame();
                        const Vector3 offset = scene_position - reference_center_scene(body);
                        const Vector3 ecliptic = frame.direction_to_heliocentric(offset);
                        const Vector3 equatorial =
                            Astro::ecliptic_to_body_equatorial(body_id, ecliptic);
                        const Scalar spin =
                            Scalar(Astro::body_rotation_angle(body_id, julian_date_));
                        return rotate_about_pole(equatorial, -spin);
                    }

                    /** @brief The scene position of a body-fixed point; inverse of @ref scene_to_body_fixed. */
                    Vector3 body_fixed_to_scene(int body, const Vector3& body_fixed) const
                    {
                        const Astro::BodyId body_id = static_cast<Astro::BodyId>(body);
                        const Astro::SceneFrame frame = current_scene_frame();
                        const Scalar spin =
                            Scalar(Astro::body_rotation_angle(body_id, julian_date_));
                        const Vector3 equatorial = rotate_about_pole(body_fixed, spin);
                        const Vector3 ecliptic =
                            Astro::body_equatorial_to_ecliptic(body_id, equatorial);
                        return reference_center_scene(body) + frame.direction_to_scene(ecliptic);
                    }

                    /**
                     * @brief Resolves a record's authoring mode, deciding Auto from altitude.
                     *
                     * Free and Surface pass through; Auto becomes Surface when the entity sits
                     * within a small margin of the reference body's equatorial radius (standing
                     * on the ground) and Free otherwise (in flight or orbit). With no reference
                     * body Auto is Free — there is no surface to anchor to.
                     *
                     * @param record The entity whose mode to resolve.
                     * @return The concrete Free or Surface mode.
                     */
                    FrameMode resolved_frame_mode(const Record& record) const
                    {
                        if (record.frame_mode != FrameMode::Auto)
                            return record.frame_mode;
                        if (record.reference_body < 0 || !world_.alive(record.entity))
                            return FrameMode::Free;
                        const Vector3 center = reference_center_scene(record.reference_body);
                        const Vector3 position = world_.get<Transform>(record.entity).position;
                        const Scalar distance = length(position - center);
                        const Scalar radius = Scalar(Astro::surface_preset(
                            static_cast<Astro::BodyId>(record.reference_body)).semi_major_metres);
                        return distance <= radius * Scalar(1.05) ? FrameMode::Surface
                                                                 : FrameMode::Free;
                    }

                    /**
                     * @brief Distance from the dominant body's centre to its surface along a
                     *        scene-frame outward direction.
                     *
                     * The reference ellipsoid's radius at the geocentric latitude the
                     * direction implies (from its projection on the pole), so the surface is
                     * the true flattened ellipsoid rather than a mean sphere — the boundary
                     * the clamp keeps everything outside of.
                     *
                     * @param outward_unit Unit outward direction from the body centre, scene frame.
                     * @return Surface radius along that direction, metres.
                     */
                    Scalar planet_surface_radius(const Vector3& outward_unit) const noexcept
                    {
                        const Scalar a = Scalar(scene_.environment.planet.semi_major);
                        const Scalar b = Scalar(scene_.environment.planet.semi_minor());
                        const Scalar sin_lat = dot(outward_unit, scene_.environment.planet_pole);
                        const Scalar cos_lat =
                            std::sqrt(std::fmax(Scalar(0), Scalar(1) - sin_lat * sin_lat));
                        const Scalar bc = b * cos_lat;
                        const Scalar as = a * sin_lat;
                        const Scalar denom = std::sqrt(bc * bc + as * as);
                        return denom > Scalar(0) ? a * b / denom : a;
                    }

                    /**
                     * @brief The outward geodetic normal at a scene-frame offset from the body.
                     *
                     * The scene-frame analogue of @ref Astro::geodetic_normal: the ellipsoid
                     * gradient, which on a flattened body departs from the geocentric radial
                     * @c normalize(offset). The axial component (along the pole) is scaled by
                     * @c 1/(1-e^2) before renormalising, so "up" is the true local vertical the
                     * ground-local orientation is composed onto — identical on every body via
                     * its own flattening, never a per-hemisphere tilt.
                     *
                     * @param offset Vector from the body centre to the point, scene frame, metres.
                     * @return Unit outward geodetic normal, scene frame.
                     */
                    Vector3 surface_normal_scene(const Vector3& offset) const noexcept
                    {
                        const Scalar a = Scalar(scene_.environment.planet.semi_major);
                        const Scalar b = Scalar(scene_.environment.planet.semi_minor());
                        const Scalar one_minus_e2 = a > Scalar(0) ? (b * b) / (a * a) : Scalar(1);
                        const Vector3& pole = scene_.environment.planet_pole;
                        const Scalar axial = dot(offset, pole);
                        const Vector3 equatorial = offset - pole * axial;
                        const Vector3 gradient =
                            equatorial + pole * (one_minus_e2 > Scalar(0) ? axial / one_minus_e2 : axial);
                        const Scalar gradient_length = length(gradient);
                        return gradient_length > Scalar(0) ? gradient * (Scalar(1) / gradient_length)
                                                           : pole;
                    }

                    /**
                     * @brief The ground-local orientation a world rotation maps to at a surface point.
                     *
                     * The inverse of the compose in @ref apply_surface_constraints: strips the
                     * local tangent frame off a world-space orientation, leaving the
                     * ground-relative part. Must be evaluated at the *same* position the tangent
                     * frame was built from, or the strip does not cancel and the pose tilts — so
                     * the one caller only uses it for a pure rotation (position unchanged). A
                     * no-op with no dominant body.
                     *
                     * @param scene_position The entity's scene position (to build the tangent frame).
                     * @param world_rotation The world-space orientation to decompose.
                     * @return The ground-local orientation.
                     */
                    Quaternion ground_local_orientation(const Vector3& scene_position,
                                                        const Quaternion& world_rotation) const
                    {
                        Astro::BodyId body;
                        if (!dominant_body(body))
                            return world_rotation;
                        const Vector3 offset = scene_position - planet_center_scene();
                        const Vector3 up = surface_normal_scene(offset);
                        const Quaternion tangent =
                            tangent_frame_quaternion(up, scene_.environment.planet_pole);
                        return normalize(mul(conjugate(tangent), world_rotation));
                    }

                    /**
                     * @brief The quaternion of the local East-North-Up tangent frame.
                     *
                     * Builds the scene-frame basis (x=east, y=up, z=south, matching the
                     * scene's local axes) at a surface point and converts it to a rotation.
                     * Composing an entity's ground-local orientation onto this is what makes
                     * "upright" identity everywhere on the body.
                     *
                     * @param up   Unit outward normal at the point, scene frame.
                     * @param pole Unit body north pole, scene frame.
                     * @return The tangent frame as a unit quaternion.
                     */
                    Quaternion tangent_frame_quaternion(const Vector3& up,
                                                        const Vector3& pole) const noexcept
                    {
                        Vector3 east = cross(pole, up);
                        const Scalar east_length = length(east);
                        east = east_length > Scalar(1e-6) ? east * (Scalar(1) / east_length)
                                                          : Vector3{1, 0, 0};
                        const Vector3 south = cross(east, up);

                        // Columns of the rotation are the images of the local axes:
                        // x -> east, y -> up, z -> south. Standard trace conversion.
                        const Scalar trace = east.x + up.y + south.z;
                        Quaternion q;
                        if (trace > Scalar(0))
                        {
                            const Scalar s = std::sqrt(trace + Scalar(1)) * Scalar(2);
                            q.w = Scalar(0.25) * s;
                            q.x = (up.z - south.y) / s;
                            q.y = (south.x - east.z) / s;
                            q.z = (east.y - up.x) / s;
                        }
                        else if (east.x > up.y && east.x > south.z)
                        {
                            const Scalar s =
                                std::sqrt(Scalar(1) + east.x - up.y - south.z) * Scalar(2);
                            q.w = (up.z - south.y) / s;
                            q.x = Scalar(0.25) * s;
                            q.y = (up.x + east.y) / s;
                            q.z = (south.x + east.z) / s;
                        }
                        else if (up.y > south.z)
                        {
                            const Scalar s =
                                std::sqrt(Scalar(1) + up.y - east.x - south.z) * Scalar(2);
                            q.w = (south.x - east.z) / s;
                            q.x = (up.x + east.y) / s;
                            q.y = Scalar(0.25) * s;
                            q.z = (south.y + up.z) / s;
                        }
                        else
                        {
                            const Scalar s =
                                std::sqrt(Scalar(1) + south.z - east.x - up.y) * Scalar(2);
                            q.w = (east.y - up.x) / s;
                            q.x = (south.x + east.z) / s;
                            q.y = (south.y + up.z) / s;
                            q.z = Scalar(0.25) * s;
                        }
                        return normalize(q);
                    }

                    /**
                     * @brief Anchors orientations to the ground and keeps every entity outside
                     *        the planet — the planet-relative pose and the planet collider.
                     *
                     * Runs each step and after every edit (from @ref extract), gated on there
                     * being a dominant body so plain non-astronomical scenes are untouched.
                     * For each entity — cameras included — it composes the local tangent frame
                     * onto an anchored orientation, then pushes the position out to the
                     * ellipsoid surface if it has sunk below it. A rigid body that penetrates
                     * is re-posed through the physics seam, which zeroes its velocity, so the
                     * surface acts as a hard floor rather than letting it tunnel through.
                     */
                    void apply_surface_constraints()
                    {
                        Astro::BodyId body;
                        if (!dominant_body(body))
                            return;

                        const Vector3 center = planet_center_scene();
                        const Vector3 pole = scene_.environment.planet_pole;
                        constexpr Scalar surface_skin_metres = Scalar(0.05);

                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr || !world_.alive(record->entity))
                                continue;

                            Transform& transform = world_.get<Transform>(record->entity);
                            Orientation& orientation = world_.get<Orientation>(record->entity);

                            const Vector3 offset = transform.position - center;
                            const Scalar distance = length(offset);
                            const Vector3 outward =
                                distance > Scalar(0) ? offset * (Scalar(1) / distance) : pole;

                            if (record->surface_anchored)
                            {
                                // "Up" is the geodetic normal (the true local vertical on a
                                // flattened body), not the geocentric radial used for the
                                // radial push-out below — the correct ENU basis to compose the
                                // ground-local orientation onto.
                                const Vector3 up = surface_normal_scene(offset);
                                const Quaternion tangent = tangent_frame_quaternion(up, pole);
                                orientation.rotation =
                                    normalize(mul(tangent, record->surface_local_orientation));
                            }

                            const Scalar surface =
                                planet_surface_radius(outward) + surface_skin_metres;
                            if (distance < surface)
                            {
                                transform.position = center + outward * surface;
                                if (record->has_physics_body)
                                    physics_->set_rigid_pose(id, transform.position,
                                                             orientation.rotation);
                            }
                        }
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
                        physics_->step(make_gravity_sampler(), PHYSICS_SUBSTEPS_PER_TICK);

                        // Advance the master epoch for this fixed step, so the sky and the
                        // on-rails bodies the gravity field sums track the physics solve. The
                        // sky is frozen when the time scale is zero.
                        const double step_days =
                            double(clock_.fixed_dt()) * time_scale_days_per_second_;
                        julian_date_ += step_days;

                        if (weather_provider_)
                            weather_provider_->tick(double(clock_.fixed_dt()),
                                GeodeticPosition{scene_.environment.observer.latitude_radians,
                                                 scene_.environment.observer.longitude_radians},
                                julian_date_);

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
                        step_particle_emitters();
                        step_crowd_playback();
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
                            desc.drag_coefficient = record->physics_params.drag_coefficient;
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
                                Scalar nc = c * orbit[i].step_cos - s * orbit[i].step_sin;
                                Scalar ns = s * orbit[i].step_cos + c * orbit[i].step_sin;
                                // Repeated rotation composition drifts off the unit circle by
                                // rounding error each step; renormalizing keeps that drift from
                                // growing without bound, which matters once it is multiplied by
                                // radius below — at planetary orbit scales an unnormalized drift
                                // of a few ULPs becomes a visible position jitter.
                                const Scalar norm = std::sqrt(nc * nc + ns * ns);
                                if (norm > Scalar(0))
                                {
                                    nc /= norm;
                                    ns /= norm;
                                }
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
                        // Publish the master epoch so the snapshot (and the editor reading
                        // it back) sees the sim's authoritative "now" for the sky.
                        scene_.environment.observer.julian_date = julian_date_;

                        // W4: when procedural weather is active, its compiled column state
                        // replaces the manually authored Cloudscape every extract — the same
                        // Environment::clouds write path manual authoring already uses, so
                        // CloudscapeCompilePass (T3) needs no changes to consume either source.
                        if (weather_provider_)
                        {
                            const GeodeticPosition observer{scene_.environment.observer.latitude_radians,
                                                            scene_.environment.observer.longitude_radians};
                            // Sampled where the camera stands, not at the scene's geodetic anchor.
                            // Those were the same point while the weather was uniform; with a
                            // spatial field they are not, and every symptom below -- the deck
                            // stack, the fog, the wetness, the rain -- describes what is happening
                            // *here*. It is also what makes the field's coverage scale exactly 1
                            // at the camera, so the bake is never stretched where it is seen most
                            // closely.
                            const GeodeticPosition local =
                                scene_.has_camera
                                    ? geodetic_at_scene(double(scene_.camera.position.x),
                                                        double(scene_.camera.position.z))
                                    : observer;
                            // One sampled column drives both the compiled Cloudscape and the
                            // world-coupling signal (fog/turbidity/wetness/precipitation) -- one
                            // cause, every symptom, not two independently sampled systems that
                            // happen to agree.
                            const WeatherColumn column = weather_provider_->sample_column(local);
                            scene_.environment.clouds = weather_compiler_.compile(column);
                            scene_.environment.weather = weather_world_compiler_.compile(column);

                            // The spatial half (docs/slop/atmosphere_system.md §7): the same
                            // provider's horizontal structure, published as a field the cloud
                            // march reads per sample. The column above is what the deck stack was
                            // compiled from, so it is also the field's reference -- record it here,
                            // from the same sample, rather than letting the two drift apart.
                            weather_field_buffer_.set_reference_column(column);
                            weather_provider_->publish_field(observer, weather_field_buffer_);
                            scene_.environment.weather_field = weather_field_buffer_.view();
                        }
                        else
                        {
                            // No dynamic weather: leave the render tier exactly as it behaved
                            // before the field existed (every WeatherCoupling field defaults to
                            // zero/no-op, and an invalid field is ignored outright).
                            scene_.environment.weather = Render::WeatherCoupling{};
                            scene_.environment.weather_field = Render::WeatherField{};
                        }

                        // Anchor ground-relative orientations and keep everything above the
                        // planet surface, before reading poses into the render snapshot — so
                        // both play (post-step) and edit (post-edit) reflect them.
                        apply_surface_constraints();

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

                        extract_crowd();

                        // Deterministic particle emitters: one billboard per live particle in
                        // each emitter's host-side pool, already world-space from the fixed-tick
                        // integration, drawn by the renderer as camera-facing quads.
                        scene_.particle_billboards.clear();
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_particle_emitter ||
                                !record->visible)
                                continue;
                            const Vfx::DeterministicEmitterState& pool = record->particle_pool;
                            for (std::uint32_t i = 0; i < pool.alive_count; ++i)
                            {
                                const Vfx::GpuParticle& particle = pool.particles[i];
                                ParticleBillboard billboard;
                                billboard.position = Vector3{particle.position[0], particle.position[1],
                                                             particle.position[2]};
                                billboard.color = Vector3{particle.color[0], particle.color[1],
                                                          particle.color[2]};
                                billboard.size = particle.size;
                                billboard.alpha = particle.alpha;
                                billboard.rotation = particle.rotation;
                                scene_.particle_billboards.push_back(billboard);
                            }
                        }

                        // Cosmetic emitters: not simulated here at all. The sim places them —
                        // transform, this frame's spawn count, the compiled record and its LUT
                        // atlases — and the renderer emits and integrates them on the GPU.
                        scene_.particle_emitters.clear();
                        const float emitter_dt = static_cast<float>(clock_.fixed_dt());
                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr || !record->has_particle_emitter ||
                                !record->visible || !world_.alive(record->entity))
                                continue;
                            const Vfx::CompiledEffect* compiled = effect_for(*record);
                            if (compiled == nullptr)
                                continue;

                            const Mat4 model = world_matrix(id);
                            const float* curve_luts =
                                compiled->curve_luts.empty() ? nullptr : compiled->curve_luts.data();
                            const float* gradient_luts = compiled->gradient_luts.empty()
                                                             ? nullptr
                                                             : compiled->gradient_luts.data();
                            for (const Vfx::CompiledEmitter& emitter : compiled->emitters)
                            {
                                if (emitter.domain != Vfx::SimulationDomain::Cosmetic)
                                    continue;

                                // The spawn count is the host's to compute — the emit shader is a
                                // pure allocator — so the fractional carry lives on the record and
                                // survives the frame that could not afford a whole particle.
                                std::uint32_t spawn_count = 0;
                                if (record->emitter_params.playing && emitter.spawn_rate > 0.0f)
                                {
                                    record->emitter_spawn_carry +=
                                        emitter.spawn_rate * emitter_dt;
                                    spawn_count = static_cast<std::uint32_t>(
                                        record->emitter_spawn_carry);
                                    record->emitter_spawn_carry -=
                                        static_cast<float>(spawn_count);
                                    if (spawn_count > emitter.capacity)
                                        spawn_count = emitter.capacity;
                                }

                                Render::ParticleEmitterView view;
                                view.model = model;
                                view.compiled = &emitter;
                                view.curve_luts = curve_luts;
                                view.gradient_luts = gradient_luts;
                                view.curve_lut_floats =
                                    static_cast<std::uint32_t>(compiled->curve_luts.size());
                                view.gradient_lut_floats =
                                    static_cast<std::uint32_t>(compiled->gradient_luts.size());
                                view.spawn_count = spawn_count;
                                view.seed = record->emitter_params.seed;
                                view.dt = emitter_dt;
                                view.id = static_cast<std::uint32_t>(id);
                                scene_.particle_emitters.push_back(view);
                            }
                        }

                        // W5 precipitation VFX: a synthetic, sim-owned cosmetic rain emitter that
                        // follows the camera, sourced from the same weather column that darkened
                        // the cloud base and thickened the fog above -- one cause, every symptom
                        // (design doc §7's acceptance bar). Not an authored ECS entity: that would
                        // clutter the Hierarchy/Outliner and the scene file with a system-generated
                        // object, so it is appended straight into the render scene's own
                        // particle-emitter list instead, the same seam the entity loop above
                        // already writes through -- using the GPU cosmetic path per
                        // QualityParams::gpu_particles's own doc ("the deterministic CPU particle
                        // path is unaffected; it is gameplay, not a quality knob"), since ambient
                        // weather rain is squarely cosmetic.
                        //
                        // Rain only: WeatherColumn carries no temperature signal (see
                        // weather_types.hpp), so there is no honest basis to pick snow over rain --
                        // a named scope-down rather than a fabricated phase test.
                        //
                        // The intensity now comes from the column above the *camera* (see
                        // extract()'s sample site), so flying into a shower starts the rain and
                        // flying out of it stops it -- "rain falls from the cell that is raining",
                        // which a single observer-anchored sample could never express.
                        const float precipitation = scene_.environment.weather.precipitation_intensity;
                        constexpr float PRECIPITATION_VFX_THRESHOLD = 0.05f;
                        if (weather_provider_ && scene_.has_camera &&
                            precipitation > PRECIPITATION_VFX_THRESHOLD)
                        {
                            const GeodeticPosition local =
                                geodetic_at_scene(double(scene_.camera.position.x),
                                                  double(scene_.camera.position.z));
                            // weather_wind(): the synoptic field plus a local perturbation, near
                            // the surface; lateral drift only, scaled down below. Only an
                            // authorable provider carries a synoptic layer to sample -- an
                            // ingested one has no such field, and its rain simply falls straight.
                            WindSample wind{};
                            if (weather_authoring_ != nullptr)
                                wind = weather_wind(weather_authoring_->synoptic(), local,
                                                    /*altitude_meters=*/50.0,
                                                    julian_date_ * 86400.0);

                            Vfx::CompiledEmitter& rain = weather_rain_emitter_;
                            rain = Vfx::CompiledEmitter{};
                            rain.capacity = 4096;
                            rain.domain = Vfx::SimulationDomain::Cosmetic;
                            rain.duration = 5.0f;
                            rain.flags = Vfx::EMITTER_LOOPING;
                            rain.spawn_rate = 900.0f * precipitation;
                            rain.shape = Vfx::EmitterShape::Box;
                            rain.shape_box_half_extents[0] = 35.0f;
                            rain.shape_box_half_extents[1] = 15.0f;
                            rain.shape_box_half_extents[2] = 35.0f;
                            rain.lifetime_min = 1.6f;
                            rain.lifetime_max = 2.2f;
                            // Box shape emits along local +Y (particle_common.glsl's sample_shape);
                            // a negative speed range is what turns that into a downward fall.
                            rain.speed_min = -22.0f;
                            rain.speed_max = -15.0f;
                            rain.size_min = 0.03f;
                            rain.size_max = 0.06f;
                            rain.color[0] = 0.55f;
                            rain.color[1] = 0.62f;
                            rain.color[2] = 0.70f;
                            rain.update_flags = Vfx::UPDATE_GRAVITY;
                            // weather_wind()'s lateral drift, scaled well down: rain falls in ~2 s,
                            // so even a brisk wind should nudge the streak, not fling it sideways.
                            constexpr float WIND_DRIFT_SCALE = 0.2f;
                            rain.gravity[0] = static_cast<float>(wind.eastward_mps) * WIND_DRIFT_SCALE;
                            rain.gravity[1] = -3.0f;
                            rain.gravity[2] = static_cast<float>(wind.northward_mps) * WIND_DRIFT_SCALE;
                            rain.blend = Vfx::BlendMode::Alpha;
                            rain.alignment = Vfx::RenderAlignment::VelocityStretched;
                            rain.velocity_stretch = 0.04f;
                            rain.render_flags = Vfx::RENDER_SOFT;
                            rain.soft_fade_distance = 0.3f;

                            weather_rain_spawn_carry_ +=
                                rain.spawn_rate * static_cast<float>(clock_.fixed_dt());
                            std::uint32_t spawn_count =
                                static_cast<std::uint32_t>(weather_rain_spawn_carry_);
                            weather_rain_spawn_carry_ -= static_cast<float>(spawn_count);
                            if (spawn_count > rain.capacity)
                                spawn_count = rain.capacity;

                            const Vector3 rain_center =
                                scene_.camera.position + Vector3{0, Scalar(25.0), 0};
                            Render::ParticleEmitterView view;
                            view.model = compose_transform(rain_center, Quaternion{}, Vector3{1, 1, 1});
                            view.compiled = &weather_rain_emitter_;
                            view.spawn_count = spawn_count;
                            view.seed = 0x57454154u; // 'WEAT' -- a fixed synthetic seed, not an entity.
                            view.dt = static_cast<float>(clock_.fixed_dt());
                            view.id = Render::NO_PICK;
                            scene_.particle_emitters.push_back(view);
                        }
                        else
                        {
                            weather_rain_spawn_carry_ = 0.0f;
                        }

                        // W6 canopy wisp VFX (design doc §7): small wispy cosmetic particles
                        // hugging a cloud deck's own base or top boundary, for close flythroughs.
                        // Gated by camera altitude alone -- not weather-specific like the rain
                        // emitter above -- since a hand-authored Manual-mode sky's deck boundary
                        // is exactly as real a thing to fly through as a procedurally compiled
                        // one, and scene_.environment.clouds already holds whichever is live.
                        // Reuses cloud_genus_profile's own base_altitude/top_altitude per enabled
                        // deck (the same étage data WeatherCloudscapeCompiler and
                        // StaticWeather::decompose already read), so no new authoring surface is
                        // needed to know where a deck's edge sits.
                        constexpr float WISP_BAND_METERS = 60.0f;
                        constexpr float WISP_COVERAGE_THRESHOLD = 0.25f;
                        bool wisp_active = false;
                        float wisp_boundary_altitude = 0.0f;
                        if (scene_.has_camera && scene_.environment.clouds.enabled)
                        {
                            const Vector3 camera_offset =
                                scene_.camera.position - planet_center_scene();
                            const float camera_altitude = float(
                                length(camera_offset) -
                                Scalar(scene_.environment.planet_surface_reference_metres));

                            float best_distance = WISP_BAND_METERS;
                            for (int i = 0; i < Render::CLOUD_MAX_DECKS; ++i)
                            {
                                const Render::CloudDeck& deck = scene_.environment.clouds.decks[i];
                                if (!deck.enabled)
                                    continue;
                                const Render::CloudGenusProfile profile =
                                    Render::cloud_genus_profile(deck.genus);
                                const float coverage =
                                    std::clamp(profile.coverage + deck.coverage_bias, 0.0f, 1.0f);
                                if (coverage < WISP_COVERAGE_THRESHOLD)
                                    continue;
                                const float base_distance =
                                    std::fabs(camera_altitude - profile.base_altitude);
                                if (base_distance < best_distance)
                                {
                                    best_distance = base_distance;
                                    wisp_boundary_altitude = profile.base_altitude;
                                    wisp_active = true;
                                }
                                const float top_distance =
                                    std::fabs(camera_altitude - profile.top_altitude);
                                if (top_distance < best_distance)
                                {
                                    best_distance = top_distance;
                                    wisp_boundary_altitude = profile.top_altitude;
                                    wisp_active = true;
                                }
                            }
                        }

                        if (wisp_active)
                        {
                            Vfx::CompiledEmitter& wisp = weather_wisp_emitter_;
                            wisp = Vfx::CompiledEmitter{};
                            wisp.capacity = 1024;
                            wisp.domain = Vfx::SimulationDomain::Cosmetic;
                            wisp.duration = 6.0f;
                            wisp.flags = Vfx::EMITTER_LOOPING;
                            wisp.spawn_rate = 40.0f; // sparse and wispy, not a dense fog sheet.
                            wisp.shape = Vfx::EmitterShape::Box;
                            wisp.shape_box_half_extents[0] = 40.0f;
                            wisp.shape_box_half_extents[1] = 8.0f; // thin slab hugging the boundary.
                            wisp.shape_box_half_extents[2] = 40.0f;
                            wisp.lifetime_min = 4.0f;
                            wisp.lifetime_max = 7.0f;
                            wisp.speed_min = 0.1f;
                            wisp.speed_max = 0.6f; // barely drifting; wisps loiter, they don't fall.
                            wisp.size_min = 2.5f;
                            wisp.size_max = 5.5f;
                            wisp.color[0] = 0.92f;
                            wisp.color[1] = 0.94f;
                            wisp.color[2] = 0.97f;
                            wisp.update_flags = Vfx::UPDATE_TURBULENCE;
                            wisp.turbulence_frequency = 0.05f;
                            wisp.turbulence_amplitude = 1.2f;
                            wisp.blend = Vfx::BlendMode::Alpha;
                            wisp.alignment = Vfx::RenderAlignment::FaceCamera;
                            wisp.render_flags = Vfx::RENDER_SOFT;
                            wisp.soft_fade_distance = 1.5f;

                            weather_wisp_spawn_carry_ +=
                                wisp.spawn_rate * static_cast<float>(clock_.fixed_dt());
                            std::uint32_t spawn_count =
                                static_cast<std::uint32_t>(weather_wisp_spawn_carry_);
                            weather_wisp_spawn_carry_ -= static_cast<float>(spawn_count);
                            if (spawn_count > wisp.capacity)
                                spawn_count = wisp.capacity;

                            const Vector3 up = normalize(scene_.camera.position - planet_center_scene());
                            const Vector3 wisp_center =
                                planet_center_scene() +
                                up * Scalar(double(scene_.environment.planet_surface_reference_metres) +
                                           double(wisp_boundary_altitude));

                            Render::ParticleEmitterView view;
                            view.model = compose_transform(wisp_center, Quaternion{}, Vector3{1, 1, 1});
                            view.compiled = &weather_wisp_emitter_;
                            view.spawn_count = spawn_count;
                            view.seed = 0x57495350u; // 'WISP' -- a fixed synthetic seed, not an entity.
                            view.dt = static_cast<float>(clock_.fixed_dt());
                            view.id = Render::NO_PICK;
                            scene_.particle_emitters.push_back(view);
                        }
                        else
                        {
                            weather_wisp_spawn_carry_ = 0.0f;
                        }

                        // Punctual lights: a light is a record on the entity, so its world
                        // position and (spot) aim come straight from the entity's transform,
                        // exactly as a mesh instance's model does. The renderer culls the
                        // list into the froxel grid; nothing here knows about clusters.
                        scene_.lights.clear();
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_light || !record->visible)
                                continue;
                            const Mat4 model = world_matrix(id);
                            constexpr float degrees_to_radians = 0.017453292519943295f;
                            Render::PunctualLight light;
                            light.position = WorldVector3{model.m[12], model.m[13], model.m[14]};
                            // The spot aims down the entity's local -Z, like a camera's gaze;
                            // scale is divided out by normalising.
                            light.direction =
                                normalize(Vector3{-model.m[8], -model.m[9], -model.m[10]});
                            light.color = record->light_params.color;
                            light.intensity = record->light_params.intensity;
                            light.range = record->light_params.range;
                            light.type = record->light_params.is_spot ? Render::LightType::Spot
                                                                      : Render::LightType::Point;
                            light.casts_shadows = record->light_params.casts_shadows;
                            light.inner_cone =
                                record->light_params.inner_degrees * degrees_to_radians;
                            light.outer_cone =
                                record->light_params.outer_degrees * degrees_to_radians;
                            light.id = static_cast<std::uint32_t>(id);
                            scene_.lights.push_back(light);
                        }

                        // Projected decals: box centre, orientation, and size from the
                        // entity's world matrix (unit axes are its rotation columns), tint
                        // and opacity from the record. The renderer culls them into the
                        // froxel grid alongside the lights.
                        scene_.decals.clear();
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_decal || !record->visible)
                                continue;
                            const Mat4 model = world_matrix(id);
                            Render::Decal decal;
                            decal.position = WorldVector3{model.m[12], model.m[13], model.m[14]};
                            decal.right = normalize(Vector3{model.m[0], model.m[1], model.m[2]});
                            decal.up = normalize(Vector3{model.m[4], model.m[5], model.m[6]});
                            decal.forward = normalize(Vector3{model.m[8], model.m[9], model.m[10]});
                            decal.half_extents = record->decal_params.half_extents;
                            decal.color = record->decal_params.color;
                            decal.opacity = record->decal_params.opacity;
                            decal.albedo_map = record->decal_params.albedo_map;
                            decal.orm_map = record->decal_params.orm_map;
                            decal.id = static_cast<std::uint32_t>(id);
                            scene_.decals.push_back(decal);
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

                    /**
                     * @brief The compiled effect a record's emitter references, or null.
                     *
                     * Out-of-range indices fall back to the first effect rather than dropping the
                     * emitter: an index can go stale when the library shrinks, and an emitter that
                     * silently stops is harder to diagnose than one playing the wrong effect.
                     */
                    const Vfx::CompiledEffect* effect_for(const Record& record)
                    {
                        if (record.effect_asset == Vfx::INVALID_EFFECT)
                            return nullptr;
                        const Vfx::CompiledEffect& compiled =
                            effect_db_.compiled(record.effect_asset);
                        return compiled.emitters.empty() ? nullptr : &compiled;
                    }

                    /** @brief Gives a freshly added emitter the effect it starts from. */
                    void seed_emitter_effect(Record& record)
                    {
                        if (record.effect_asset != Vfx::INVALID_EFFECT)
                            return;
                        // A component that shows nothing reads as broken rather than as an
                        // invitation to author, so a new emitter starts somewhere visible.
                        record.effect_source = make_fire_effect();
                        record.effect_asset = effect_db_.add(record.effect_source);
                    }

                    /**
                     * @brief The effect's first deterministic emitter, or null when it has none.
                     *
                     * One host pool per entity, so a multi-emitter effect's deterministic half is
                     * represented by its first emitter; the rest is a per-emitter-pool refactor.
                     */
                    static const Vfx::CompiledEmitter* first_deterministic(
                        const Vfx::CompiledEffect& compiled) noexcept
                    {
                        for (const Vfx::CompiledEmitter& emitter : compiled.emitters)
                        {
                            if (emitter.domain == Vfx::SimulationDomain::Deterministic)
                                return &emitter;
                        }
                        return nullptr;
                    }

                    /** @brief Advances every playing emitter's deterministic pool by one tick. */
                    void step_particle_emitters()
                    {
                        const float dt = static_cast<float>(clock_.fixed_dt());
                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr || !record->has_particle_emitter ||
                                !record->emitter_params.playing || !world_.alive(record->entity))
                                continue;
                            const Vfx::CompiledEffect* compiled = effect_for(*record);
                            if (compiled == nullptr)
                                continue;
                            record->emitter_time += dt;

                            // Only the deterministic emitters are stepped here; a cosmetic one is
                            // simulated by the renderer, and the extract merely places it.
                            const Vfx::CompiledEmitter* emitter = first_deterministic(*compiled);
                            if (emitter == nullptr)
                                continue;
                            const Vector3 position = world_.get<Transform>(record->entity).position;
                            const Quaternion rotation =
                                world_.get<Orientation>(record->entity).rotation;
                            Vfx::CpuDeterministicBackend::step(record->particle_pool, *emitter,
                                                               *compiled, dt, position, rotation);
                        }
                    }

                    /** @brief Advances every playing crowd entity's clip playback time. */
                    void step_crowd_playback()
                    {
                        const float dt = static_cast<float>(clock_.fixed_dt());
                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr || !record->has_crowd ||
                                !record->crowd_params.playing)
                                continue;
                            record->crowd_params.time_seconds += dt;
                        }
                    }

                    /**
                     * @brief Samples every crowd entity sharing this frame's bound skeleton
                     * through the SYCL device evaluator and fills @ref scene_'s skinned
                     * instances (design §12.3/§12.4).
                     *
                     * The frame's bound skeleton is whichever crowd entity's `crowd_params.skeleton`
                     * is seen first while walking @ref order_; every later entity naming a
                     * different (nonzero) skeleton handle is skipped this frame — a real,
                     * documented limitation of `Animation::DeviceBatchEvaluator`'s
                     * one-shared-skeleton-per-batch scoping (see `RenderScene::skinned_instances`'
                     * own comment), not a silent drop.
                     */
                    void extract_crowd()
                    {
                        scene_.skinned_instances.clear();

                        // Find the frame's bound skeleton and every crowd entity that shares it.
                        std::uint32_t batch_skeleton_handle = 0;
                        std::vector<EntityId> batch_entities;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_crowd || !record->visible ||
                                record->crowd_params.skeleton == 0 ||
                                record->crowd_params.mesh == Render::INVALID_MESH)
                                continue;
                            if (batch_skeleton_handle == 0)
                                batch_skeleton_handle = record->crowd_params.skeleton;
                            if (record->crowd_params.skeleton != batch_skeleton_handle)
                                continue; // A different rig than this frame's batch — skipped, not drawn wrong.
                            batch_entities.push_back(id);
                        }
                        if (batch_entities.empty())
                            return;

                        if (batch_skeleton_handle != crowd_bound_skeleton_handle_)
                        {
                            const Animation::SkeletonView skeleton =
                                crowd_database_.skeleton(crowd_skeletons_[batch_skeleton_handle - 1]);
                            if (!crowd_evaluator_.bind_skeleton(skeleton))
                                return;
                            crowd_bound_skeleton_handle_ = batch_skeleton_handle;
                            // bind_skeleton invalidates every previously bound device clip.
                            crowd_device_clips_.clear();
                        }

                        std::vector<Animation::DeviceInstanceDesc> instances;
                        instances.reserve(batch_entities.size());
                        std::vector<EntityId> included_entities;
                        included_entities.reserve(batch_entities.size());
                        for (const EntityId id : batch_entities)
                        {
                            const Record* record = find(id);
                            const std::uint32_t clip_handle = record->crowd_params.clip;
                            if (clip_handle == 0 || clip_handle > crowd_clips_.size())
                                continue;

                            auto bound = crowd_device_clips_.find(clip_handle);
                            if (bound == crowd_device_clips_.end())
                            {
                                const Animation::ClipView clip =
                                    crowd_database_.clip(crowd_clips_[clip_handle - 1]);
                                const std::uint32_t device_handle = crowd_evaluator_.bind_clip(clip);
                                if (device_handle == Animation::INVALID_CLIP_HANDLE)
                                    continue;
                                bound = crowd_device_clips_.emplace(clip_handle, device_handle).first;
                            }

                            Animation::DeviceInstanceDesc desc;
                            desc.clip_handle = bound->second;
                            desc.time_seconds = record->crowd_params.time_seconds;
                            desc.loop = record->crowd_params.loop ? 1u : 0u;
                            instances.push_back(desc);
                            included_entities.push_back(id);
                        }
                        if (instances.empty())
                            return;

                        crowd_evaluator_.set_instances(instances);
                        crowd_evaluator_.evaluate();

                        const std::uint32_t joint_count = crowd_evaluator_.joint_count();
                        const Animation::JointMatrix* palettes = crowd_evaluator_.palettes().data();
                        scene_.skinned_instances.reserve(included_entities.size());
                        for (std::size_t i = 0; i < included_entities.size(); ++i)
                        {
                            const EntityId id = included_entities[i];
                            const Record* record = find(id);
                            Render::SkinnedInstance instance;
                            instance.model = world_matrix(id);
                            instance.palette = palettes + i * joint_count;
                            instance.previous_palette = nullptr; // see this method's Doxygen
                            instance.joint_count = joint_count;
                            instance.id = id;
                            instance.mesh = record->crowd_params.mesh;
                            instance.material = record->crowd_params.material;
                            scene_.skinned_instances.push_back(instance);
                        }
                    }

                    /** @brief A deterministic fire plume: buoyant cone, warm colour ramp. */
                    static Vfx::ParticleEffect make_fire_effect()
                    {
                        Vfx::EmitterDescriptor e;
                        e.name = "Fire";
                        e.domain = Vfx::SimulationDomain::Deterministic;
                        e.capacity = 512;
                        e.spawn.rate_per_second = 140.0f;
                        e.shape.shape = Vfx::EmitterShape::Cone;
                        e.shape.radius = 0.2f;
                        e.shape.cone_angle_radians = 0.35f;
                        e.init.lifetime_min = 0.7f;
                        e.init.lifetime_max = 1.4f;
                        e.init.speed_min = 1.2f;
                        e.init.speed_max = 3.0f;
                        e.init.size_min = 0.05f;
                        e.init.size_max = 0.13f;
                        e.gravity.enabled = true;
                        e.gravity.acceleration = Vector3{0, 3.2, 0};
                        e.drag.enabled = true;
                        e.drag.coefficient = 0.5f;
                        e.turbulence.enabled = true;
                        e.turbulence.frequency = 1.0f;
                        e.turbulence.amplitude = 1.4f;
                        e.size_over_life.enabled = true;
                        e.size_over_life.curve.add_key(Vfx::CurveKey{0.0f, 0.3f, 0.0f, 1.5f});
                        e.size_over_life.curve.add_key(Vfx::CurveKey{0.3f, 1.0f, 0.0f, 0.0f});
                        e.size_over_life.curve.add_key(Vfx::CurveKey{1.0f, 0.0f, -1.0f, 0.0f});
                        e.color_over_life.enabled = true;
                        e.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.0f, Vector3{1.0, 0.9, 0.45}});
                        e.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.5f, Vector3{1.0, 0.35, 0.08}});
                        e.color_over_life.gradient.add_color_key(Vfx::ColorKey{1.0f, Vector3{0.15, 0.03, 0.02}});
                        e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.0f, 0.0f});
                        e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.1f, 1.0f});
                        e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{1.0f, 0.0f});
                        Vfx::ParticleEffect effect;
                        effect.name = "Fire";
                        effect.emitters.push_back(e);
                        return effect;
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

                    // The weather seam: ticked from step_once() and compiled into
                    // scene_.environment.clouds from extract() whenever set. Null (the default)
                    // leaves clouds exactly as manual authoring already sets them. Held as the
                    // *interface*, not a concrete provider — see install_weather_provider.
                    std::unique_ptr<IWeatherProvider> weather_provider_;
                    // The installed provider's authoring capability, resolved once at install
                    // rather than re-queried per call; null when it has none.
                    IWeatherAuthoring* weather_authoring_ = nullptr;
                    // Storage behind Environment::weather_field, which borrows it (see
                    // Render::WeatherField). Owned here because this object outlives every frame
                    // whose environment can still be read.
                    WeatherFieldBuffer weather_field_buffer_;
                    WeatherCloudscapeCompiler weather_compiler_;
                    WeatherWorldCoupling weather_world_compiler_;

                    // W5 precipitation VFX (see extract()'s particle-emitter section): a single,
                    // sim-owned cosmetic rain emitter reused frame to frame rather than an
                    // authored ECS entity. ParticleEmitterView::compiled is a non-owning pointer
                    // that must outlive the frame's render, so this has to be a persistent member,
                    // not a stack local built inside extract().
                    Vfx::CompiledEmitter weather_rain_emitter_;
                    float weather_rain_spawn_carry_ = 0.0f;

                    // W6 canopy wisp VFX (see extract()'s particle-emitter section): the same
                    // persistent-member reasoning as weather_rain_emitter_ above.
                    Vfx::CompiledEmitter weather_wisp_emitter_;
                    float weather_wisp_spawn_carry_ = 0.0f;

                    // The deterministic particle path: a small library of built-in effects
                    // (Deterministic domain) an emitter entity references by index, and their
                    // display names for the inspector's picker. Compiled lazily on first step.
                    Vfx::EffectDatabase effect_db_;

                    // The master simulation epoch: the single "now" that both the orbital
                    // dynamics and the scene-frame placement of astro bodies read, so a
                    // free body and the planet it orbits are always evaluated at the same
                    // instant. Seeded from the authored observer epoch (set_environment /
                    // set_julian_date) and advanced by the fixed step each tick, scaled by
                    // time_scale_days_per_second_ (0 freezes the sky). The sim owns this
                    // clock; the editor reads it back through render_scene()'s environment.
                    double julian_date_ = Astro::J2000_JULIAN_DATE;
                    double time_scale_days_per_second_ = 0.0;
                    // The gravitational field the orbital integrator pulls from, behind the
                    // IGravityField seam so the model (summed rails today) is swappable.
                    Astro::SummedRailsGravityField gravity_field_;

                    // Crowd device-batch skinning (design §12.3/§12.4): register_crowd_skeleton/
                    // register_crowd_clip cook glTF content into crowd_database_ once, at load
                    // time; crowd_skeletons_/crowd_clips_ are that database's asset ids, indexed
                    // by the 1-based handle CrowdParams::skeleton/clip name (handle 0 is always
                    // invalid, matching every other "0 = none" id in this codebase). The path
                    // caches make re-registering the same file a no-op lookup, not a re-import.
                    Animation::AnimationDatabase crowd_database_;
                    std::vector<Animation::AssetId> crowd_skeletons_;
                    std::vector<Animation::AssetId> crowd_clips_;
                    std::unordered_map<std::string, std::uint32_t> crowd_skeleton_cache_;
                    std::unordered_map<std::string, std::uint32_t> crowd_clip_cache_;
                    // The SYCL device evaluator every crowd entity is sampled through at
                    // extract (see extract_crowd()). Bound to whichever skeleton handle the
                    // frame's crowd entities actually use — DeviceBatchEvaluator batches one
                    // shared skeleton per call, so a skeleton change rebinds it and every clip
                    // registered against it (crowd_device_clips_ tracks which of our clip
                    // handles are currently bound into *this* skeleton binding).
                    Animation::DeviceBatchEvaluator crowd_evaluator_;
                    std::uint32_t crowd_bound_skeleton_handle_ = 0;
                    std::unordered_map<std::uint32_t, std::uint32_t> crowd_device_clips_;
            };
        } // namespace

        std::unique_ptr<ISimulation> create_simulation()
        {
            return std::unique_ptr<ISimulation>(new RuntimeSimulation());
        }
    } // namespace Simulation
} // namespace SushiEngine
