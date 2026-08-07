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
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/animation/device_batch_evaluator.hpp>
#include <SushiEngine/astro/astro_dynamics.hpp>
#include <SushiEngine/astro/scene_frame.hpp>
#include <SushiEngine/astro/surface_frame.hpp>
#include <SushiEngine/gltf/skeleton_import.hpp>
#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>
#include <SushiEngine/physics/soft/cloth.hpp>
#include <SushiEngine/render/deformable_mesh.hpp>
#include <SushiEngine/simulation/physics_extract.hpp>
#include <SushiEngine/simulation/climatology_asset.hpp>
#include <SushiEngine/simulation/components.hpp>
#include <SushiEngine/simulation/impact_response.hpp>
#include <SushiEngine/simulation/physics_simulation.hpp>
#include <SushiEngine/simulation/seeded_weather.hpp>
#include <SushiEngine/simulation/simulation.hpp>
#include <SushiEngine/simulation/weather_cloudscape_compiler.hpp>
#include <SushiEngine/simulation/atmosphere_forcing_buffer.hpp>
#include <SushiEngine/simulation/weather_field_buffer.hpp>
#include <SushiEngine/simulation/weather_wind.hpp>
#include <SushiEngine/simulation/weather_world_coupling.hpp>
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
                          execution_(runtime_),
                          world_(execution_, CHUNK_CAPACITY),
                          schedule_(execution_),
                          clock_(FIXED_TICK_DT_SECONDS),
                          physics_(create_physics_simulation(execution_)),
                          impacts_(*this),
                          crowd_evaluator_(execution_)
                    {
                        // Registered for the life of the simulation. The listener holds a
                        // reference to this object and this object owns the physics that
                        // calls it, so the two cannot outlive each other — which is what
                        // makes a lifetime this simple correct rather than lucky.
                        if (physics_ != nullptr)
                        {
                            physics_->add_event_sink(&impacts_,
                                                     ImpactResponseListener::filter());
                        }
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
                        // Manual mode has a provider from the first frame. Defining "manual" as
                        // the *absence* of one would leave an authored deck stack applied
                        // uniformly to an entire planet, with no way for the sky to differ from
                        // one place to another.
                        install_mode_provider();
                        extract(); // a valid (empty) snapshot before the first tick
                    }

                    // ISimulation.

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

                    const Physics::PhysicsStatistics& physics_statistics() const noexcept override
                    {
                        // A world whose physics has not been created yet reports a
                        // zeroed value rather than failing, so a panel can draw it
                        // from the first frame instead of guarding every field.
                        static const Physics::PhysicsStatistics NONE{};
                        return physics_ != nullptr ? physics_->statistics() : NONE;
                    }

                    void set_physics_profiling(bool enabled) override
                    {
                        // The stepper latches the request and consumes it when its solve
                        // graph is next built (profiling is construction-time state).
                        if (physics_ != nullptr)
                            physics_->set_profiling_requested(enabled);
                    }

                    void set_park_sleeping_joints(bool enabled) override
                    {
                        if (physics_ != nullptr)
                            physics_->set_park_sleeping_joints_requested(enabled);
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

                    // IWorldEditor.

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

                    std::string prefab_entity_id(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->prefab_entity_id : std::string{};
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

                    MaterialTexturePaths material_texture_paths(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->material_texture_paths
                                                 : MaterialTexturePaths{};
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

                    bool enabled(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->enabled;
                    }

                    bool enabled_in_hierarchy(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && enabled_in_hierarchy(record);
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
                        // Destroying a parent takes its whole subtree with it, matching
                        // how the Hierarchy shows and selects them as one unit. Collected
                        // up front rather than walked live, since each recursive destroy()
                        // below erases entries out of the very map being scanned.
                        std::vector<EntityId> children;
                        for (auto& entry : records_)
                            if (entry.second.parent == id)
                                children.push_back(entry.first);
                        for (const EntityId child : children)
                            destroy(child);
                        // The physics simulation regenerates its body/grid set from the
                        // surviving entities on the next rebuild, so a destroy only needs
                        // to flag that rebuild — it holds no per-entity map to prune here.
                        if (it->second.has_physics_body)
                            physics_dirty_ = true;
                        if (it->second.has_cloth)
                            cloth_dirty_ = true;
                        if (it->second.has_vehicle)
                            vehicles_dirty_ = true;
                        // Its own joint goes with it, and so does any joint that named it
                        // as a partner — those live on *other* records, which is why this
                        // is unconditional rather than a test of this entity's own joint.
                        joints_dirty_ = true;
                        if (world_.alive(it->second.ui_mirror))
                            world_.destroy(it->second.ui_mirror);
                        CommandBuffer commands;
                        commands.destroy(it->second.entity);
                        commands.apply(world_);
                        records_.erase(it);
                        order_.erase(std::remove(order_.begin(), order_.end(), id),
                                     order_.end());
                        extract();
                    }

                    void set_name(EntityId id, const std::string& display_name) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                            record->name = display_name;
                    }

                    void set_prefab_entity_id(EntityId id, const std::string& value) override
                    {
                        Record* record = find(id);
                        if (record != nullptr)
                            record->prefab_entity_id = value;
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
                        // it with the solved pose.
                        place_physics_body(*record, id, value.position, value.rotation);
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
                        place_physics_body(*record, id, t.position, o.rotation);
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

                    void set_material_texture_paths(EntityId id,
                                                    const MaterialTexturePaths& value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->material_texture_paths = value;
                    }

                    void set_environment(const Render::Environment& value) override
                    {
                        const double previous_radius = scene_.environment.planet.mean_radius();
                        scene_.environment = value;
                        // The observer epoch the environment carries seeds the master
                        // clock, so authoring the sky date or loading a scene sets the
                        // simulation's "now"; thereafter the sim owns and advances it.
                        julian_date_ = value.observer.julian_date;
                        // A provider's tangent-plane maths is anchored to the body it was built
                        // for, so a scene that swapped the dominant planet needs a new one.
                        // Deliberately guarded rather than unconditional: this method runs on
                        // every environment edit the editor makes, and rebuilding
                        // `ProceduralWeather` on a colour-picker drag would throw away a
                        // simulated atmosphere the author had been waiting on.
                        if (scene_.environment.planet.mean_radius() != previous_radius)
                            install_mode_provider();
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
                     * @brief Builds and installs the provider the current mode calls for.
                     *
                     * The only place either concrete provider is constructed, so the two modes
                     * cannot drift apart in what they are handed: both get the dominant body's
                     * own radius (which anchors their tangent-plane maths to whatever the scene
                     * is actually orbiting) and the scene's own epoch.
                     */
                    void install_mode_provider()
                    {
                        const double radius = scene_.environment.planet.mean_radius();
                        if (weather_mode_ == WeatherMode::Manual)
                        {
                            install_weather_provider(std::make_unique<SeededWeather>(
                                weather_seed_, radius, julian_date_));
                            return;
                        }

                        // The mean state comes off disk if it is there. When it is not, the core
                        // runs on analytic latitude bands, which is the mean state every
                        // measurement before T0 existed was taken against -- a working
                        // atmosphere, not a degraded one, and the same one a non-Earth body gets.
                        // Nothing here fails over a missing asset. The epoch goes in too, so the
                        // initial state is seeded for the season the scene actually opens in
                        // rather than migrating into it over its first simulated weeks.
                        auto provider = std::make_unique<ProceduralWeather>(
                            weather_seed_, radius, load_climatology(), julian_date_);
                        // Bound now rather than at the host's convenience: the mirror may have
                        // been installed long before this provider existed, and a provider that
                        // never learns about it would answer from the base state forever.
                        provider->set_atmosphere_mirror(atmosphere_mirror_);
                        install_weather_provider(std::move(provider));
                        // Deliberately *not* switching the author's fog on. Such a nudge would
                        // let rain be seen through something, at the cost of enabling weather
                        // silently adding the full authored fog density to a scene that left
                        // fog off on purpose. VolumetricFogPass runs on the weather's own bias
                        // alone, so reduced visibility under rain still shows up and an author
                        // who wanted no fog still has none.
                    }

                    /**
                     * @brief Carries the installed provider's planetary placement to the renderer.
                     *
                     * The one frame conversion in this feature, and it lives here because this is
                     * the only object that holds both sides of it: the provider answers in
                     * geographic coordinates, the cloud march has nothing but a sample's radial
                     * in scene space, and `Environment::planet_body_axes` — which the ephemeris
                     * fills — is the rotation between them. Doing it once for twelve systems on
                     * the host is also what keeps it out of the shader, where it would cost two
                     * inverse trigonometric functions per system per march sample.
                     *
                     * `planet_body_axes` are the body-fixed basis vectors expressed in scene
                     * space (the third *is* the pole), so a body-fixed direction reaches the
                     * scene by the plain linear combination below. It is the exact inverse of the
                     * projection `test_body_frame.cpp` reads an observer's own coordinates back
                     * through, which is what pins the convention.
                     */
                    void publish_synoptic_field()
                    {
                        Render::SynopticFieldView view{};
                        // Valid means "this body has an atmosphere with a latitudinal structure",
                        // which is true wherever weather is running at all -- and is a different
                        // statement from `count`, which is about whether anything is *placed*.
                        view.valid = true;

                        const Atmosphere::SynopticField* field =
                            weather_provider_->synoptic_field();
                        if (field == nullptr)
                        {
                            scene_.environment.synoptic = view;
                            return;
                        }

                        const Vector3* axes = scene_.environment.planet_body_axes;
                        view.itcz_latitude = static_cast<float>(field->itcz_latitude());
                        view.count = std::min(field->count(), Render::SYNOPTIC_FIELD_MAX_CENTRES);
                        for (int i = 0; i < view.count; ++i)
                        {
                            const Atmosphere::SynopticCentre& source = field->centres()[i];
                            const double cos_latitude = std::cos(source.latitude_radians);
                            const double body_x = cos_latitude * std::cos(source.longitude_radians);
                            const double body_y = cos_latitude * std::sin(source.longitude_radians);
                            const double body_z = std::sin(source.latitude_radians);
                            const Vector3 direction{axes[0].x * body_x + axes[1].x * body_y +
                                                        axes[2].x * body_z,
                                                    axes[0].y * body_x + axes[1].y * body_y +
                                                        axes[2].y * body_z,
                                                    axes[0].z * body_x + axes[1].z * body_y +
                                                        axes[2].z * body_z};

                            Render::SynopticFieldCentre& out = view.centres[i];
                            out.direction[0] = static_cast<float>(direction.x);
                            out.direction[1] = static_cast<float>(direction.y);
                            out.direction[2] = static_cast<float>(direction.z);
                            out.falloff = static_cast<float>(source.falloff);
                            out.amplitude = static_cast<float>(source.amplitude);
                            out.convective = static_cast<float>(source.convective);
                            out.precipitation = static_cast<float>(source.precipitation);
                        }
                        scene_.environment.synoptic = view;
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

                    void set_atmosphere_mirror(
                        const Render::IAtmosphereMirror* mirror) noexcept override
                    {
                        atmosphere_mirror_ = mirror;
                        if (weather_provider_)
                            weather_provider_->set_atmosphere_mirror(mirror);
                    }

                    WeatherMode weather_mode() const noexcept override { return weather_mode_; }

                    void set_weather_mode(WeatherMode mode) override
                    {
                        if (mode == weather_mode_ && weather_provider_)
                            return;
                        weather_mode_ = mode;
                        install_mode_provider();
                        extract();
                    }

                    std::uint64_t weather_seed() const noexcept override { return weather_seed_; }

                    void set_weather_seed(std::uint64_t seed) override
                    {
                        if (seed == weather_seed_)
                            return;
                        weather_seed_ = seed;
                        // Only Manual places its sky from this. Procedural remembers it — see
                        // `ISimulation::weather_seed` for why losing it on a mode switch would
                        // read as randomness rather than as a bug.
                        if (weather_mode_ != WeatherMode::Manual)
                            return;
                        install_mode_provider();
                        extract();
                    }

                    IWeatherAuthoring* weather_authoring() noexcept override
                    {
                        return weather_authoring_;
                    }

                    const IWeatherProvider* weather_provider() const noexcept override
                    {
                        return weather_provider_.get();
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

                    PhysicsBodyParameters physics_body_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->physics_parameters
                                                 : PhysicsBodyParameters{};
                    }

                    void set_physics_body_parameters(
                        EntityId id, const PhysicsBodyParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->physics_parameters = parameters;
                        // Applied live when a body already exists, so editing mass/inertia
                        // never forces a physics-world rebuild (see set_has_physics_body);
                        // a no-op inside the physics simulation when the entity has none.
                        physics_->update_rigid_body_parameters(id, parameters.inv_mass,
                                                               parameters.inv_inertia,
                                                               parameters.drag_coefficient);
                    }

                    void set_has_physics_body(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_physics_body == value)
                            return;
                        record->has_physics_body = value;
                        physics_dirty_ = true;
                    }

                    bool has_character(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_character;
                    }

                    CharacterParameters character_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->character_parameters
                                                 : CharacterParameters{};
                    }

                    void set_character_parameters(EntityId id,
                                                  const CharacterParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        // No dirty flag, and that is the whole difference from cloth: a
                        // grid's row count decides how many bodies exist, while a
                        // capsule's radius decides nothing until somebody asks for a
                        // move. There is no built thing here for an edit to invalidate.
                        record->character_parameters = parameters;
                    }

                    void set_has_character(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->has_character = value;
                    }

                    bool has_impact_response(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_impact_response;
                    }

                    ImpactResponse impact_response(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->impact_response : ImpactResponse{};
                    }

                    void set_impact_response(EntityId id, const ImpactResponse& response) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->impact_response = response;
                    }

                    void set_has_impact_response(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->has_impact_response = value;
                    }

                    bool has_cloth(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_cloth;
                    }

                    ClothParameters cloth_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->cloth_parameters : ClothParameters{};
                    }

                    void set_cloth_parameters(EntityId id,
                                              const ClothParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->cloth_parameters = parameters;
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

                    bool has_soft_body(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_soft_body;
                    }

                    SoftBodyParameters soft_body_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->soft_body_parameters
                                                 : SoftBodyParameters{};
                    }

                    void set_soft_body_parameters(EntityId id,
                                                  const SoftBodyParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->soft_body_parameters = parameters;
                        // Every field here is topology or precision, and both are things a
                        // body is built with rather than things it can be told. So an edit
                        // is a rebuild, unlike a Rigid Body's mass.
                        if (record->has_soft_body)
                            soft_dirty_ = true;
                    }

                    void set_has_soft_body(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_soft_body == value)
                            return;
                        record->has_soft_body = value;
                        soft_dirty_ = true;
                    }

                    bool soft_body_surface(EntityId id, std::vector<Vector3>& positions,
                                           std::vector<std::uint32_t>& indices) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_soft_body)
                        {
                            positions.clear();
                            indices.clear();
                            return false;
                        }
                        return physics_->soft_body_surface(id, positions, indices);
                    }

                    Scalar soft_body_maximum_stress(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_soft_body)
                            return Scalar(0);
                        return physics_->soft_body_maximum_stress(id);
                    }

                    bool soft_body_elements(
                        EntityId id,
                        std::vector<SoftBodyElementSample>& elements) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_soft_body)
                        {
                            elements.clear();
                            return false;
                        }
                        return physics_->soft_body_elements(id, elements);
                    }

                    bool has_crowd(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_crowd;
                    }

                    CrowdParameters crowd_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->crowd_parameters : CrowdParameters{};
                    }

                    void set_crowd_parameters(EntityId id,
                                              const CrowdParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->crowd_parameters = parameters;
                        // The paths are the component's persistent half and the handles are
                        // only what this session's registry answered for them, so a write
                        // re-derives rather than trusting a number another session may have
                        // written. Registration is cached by path: the file is read once and
                        // every later write is a map lookup.
                        CrowdParameters& stored = record->crowd_parameters;
                        if (!stored.skeleton_path.empty())
                            stored.skeleton = register_crowd_skeleton(stored.skeleton_path);
                        if (!stored.clip_path.empty())
                            stored.clip = register_crowd_clip(stored.clip_path, stored.skeleton);
                    }

                    void set_has_crowd(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->has_crowd = value;
                    }

                    bool has_prefab_instance(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        // The path is the flag: an instance with no prefab named is not an
                        // instance, which is also how set_prefab_instance clears one.
                        return record != nullptr && !record->prefab_instance.path.empty();
                    }

                    PrefabInstanceParameters prefab_instance(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->prefab_instance
                                                 : PrefabInstanceParameters{};
                    }

                    void set_prefab_instance(EntityId id,
                                             const PrefabInstanceParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr)
                            return;
                        record->prefab_instance = parameters;
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
                        Animation::GLTFAnimationImport import;
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

                    LightParameters light_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->light_parameters : LightParameters{};
                    }

                    void set_light_parameters(EntityId id,
                                              const LightParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_light)
                            return;
                        record->light_parameters = parameters;
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

                    AudioEmitterParameters audio_emitter_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->audio_emitter_parameters
                                                 : AudioEmitterParameters{};
                    }

                    void set_audio_emitter_parameters(
                        EntityId id, const AudioEmitterParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_audio_emitter)
                            return;
                        record->audio_emitter_parameters = parameters;
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

                    ReverbZoneParameters reverb_zone_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->reverb_zone_parameters
                                                 : ReverbZoneParameters{};
                    }

                    void set_reverb_zone_parameters(
                        EntityId id, const ReverbZoneParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_reverb_zone)
                            return;
                        record->reverb_zone_parameters = parameters;
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

                    AudioListenerParameters audio_listener_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->audio_listener_parameters
                                                 : AudioListenerParameters{};
                    }

                    void set_audio_listener_parameters(
                        EntityId id, const AudioListenerParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_audio_listener)
                            return;
                        record->audio_listener_parameters = parameters;
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

                    DecalParameters decal_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->decal_parameters : DecalParameters{};
                    }

                    void set_decal_parameters(EntityId id,
                                              const DecalParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_decal)
                            return;
                        record->decal_parameters = parameters;
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
                        const ColliderParameters terrain_collider{PrimitiveKind::Plane, Vector3{0, 1, 0}};
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
                        // An entity that owns a cloth grid and its own Renderer (so it has an
                        // independent Color and Material in the Inspector, without a solid Shape mesh).
                        // The grid seeds from the entity's Transform::position, so moving the entity
                        // moves the pinned top edge.
                        const Entity entity = world_.spawn(
                            Transform{}, Orientation{},
                            Tint{Vector3{Scalar(0.85), Scalar(0.85), Scalar(0.9)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_renderer = true;
                        record.has_cloth = true;
                        records_.emplace(id, record);
                        cloth_dirty_ = true;
                        extract();
                        return id;
                    }

                    EntityId create_soft_body(const std::string& display_name,
                                              const std::vector<std::byte>& asset) override
                    {
                        // Refused rather than created empty: an entity whose blob does not
                        // load is a Soft Body that can never become one, and leaving it in
                        // the scene would turn a cook failure into a mystery at play time.
                        if (!Physics::Cooking::validate_soft_body_blob(asset.data(),
                                                                       asset.size()))
                            return NULL_ENTITY;

                        const Entity entity = world_.spawn(
                            Transform{}, Orientation{},
                            Tint{Vector3{Scalar(0.85), Scalar(0.85), Scalar(0.9)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_renderer = true;
                        record.has_soft_body = true;
                        record.soft_body_parameters.asset = asset;
                        records_.emplace(id, record);
                        soft_dirty_ = true;
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

                    ShapeParameters shape_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->shape_parameters : ShapeParameters{};
                    }

                    void set_shape_parameters(EntityId id,
                                              const ShapeParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_shape)
                            return;
                        record->shape_parameters = parameters;
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

                    ColliderParameters collider_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->collider_parameters
                                                 : ColliderParameters{};
                    }

                    void set_collider_parameters(EntityId id,
                                                 const ColliderParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_collider)
                            return;
                        record->collider_parameters = parameters;
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
                            record->collider_parameters =
                                record->has_shape
                                    ? ColliderParameters{record->shape_parameters.kind,
                                                         record->shape_parameters.parameters}
                                    : ColliderParameters{};
                    }

                    bool has_joint(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_joint;
                    }

                    PhysicsJointParameters joint_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->joint_parameters
                                                 : PhysicsJointParameters{};
                    }

                    void set_joint_parameters(EntityId id,
                                              const PhysicsJointParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_joint)
                            return;
                        record->joint_parameters = parameters;
                        // Any edit is a new joint: the solver's is rebuilt on the next
                        // reconcile rather than patched, because its multipliers were
                        // accumulated under the limits it is being taken out of. Editing
                        // also un-breaks it, which is what an author who has just changed
                        // the break threshold means by changing it.
                        touch_joint(*record);
                    }

                    void set_has_joint(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_joint == value)
                            return;
                        record->has_joint = value;
                        if (value)
                            record->joint_parameters = PhysicsJointParameters{};
                        touch_joint(*record);
                    }

                    bool joint_load(EntityId id, JointState& out) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || record->live_joint == NULL_JOINT ||
                            physics_ == nullptr)
                            return false;
                        return physics_->joint_state(record->live_joint, out);
                    }

                    bool joint_broken(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->joint_broken;
                    }

                    bool has_vehicle(EntityId id) const noexcept override
                    {
                        const Record* record = find(id);
                        return record != nullptr && record->has_vehicle;
                    }

                    VehicleInstanceParameters vehicle_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->vehicle_parameters
                                                 : VehicleInstanceParameters{};
                    }

                    void set_vehicle_parameters(
                        EntityId id, const VehicleInstanceParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_vehicle)
                            return;
                        record->vehicle_parameters = parameters;
                        vehicles_dirty_ = true;
                    }

                    void set_has_vehicle(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->has_vehicle == value)
                            return;
                        record->has_vehicle = value;
                        if (value)
                            record->vehicle_parameters = VehicleInstanceParameters{};
                        vehicles_dirty_ = true;
                    }

                    bool set_vehicle_input(EntityId id, const VehicleInput& input) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_vehicle)
                            return false;
                        // Recorded here as well as pushed through, so a panel can show back
                        // what it asked for even on a tick where the vehicle is not live —
                        // an author dragging a throttle slider at a car whose asset failed
                        // to load should see the slider move and be told why nothing else
                        // does.
                        record->vehicle_input = input;
                        return physics_ != nullptr && physics_->set_vehicle_input(id, input);
                    }

                    VehicleInput vehicle_input(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->vehicle_input : VehicleInput{};
                    }

                    bool vehicle_report(EntityId id, VehicleReport& out) const override
                    {
                        if (physics_ == nullptr)
                            return false;
                        return physics_->vehicle_report(id, out);
                    }

                    bool vehicle_node_positions(EntityId id,
                                                std::vector<Vector3>& out) const override
                    {
                        if (physics_ == nullptr)
                            return false;
                        return physics_->vehicle_node_positions(id, out);
                    }

                    bool vehicle_surface(EntityId id, std::vector<Vector3>& positions,
                                         std::vector<std::uint32_t>& indices) const override
                    {
                        if (physics_ == nullptr)
                            return false;
                        return physics_->vehicle_surface(id, positions, indices);
                    }

                    bool physics_body_debug(EntityId id, RigidDebugState& out) const override
                    {
                        if (physics_ == nullptr)
                            return false;
                        const Record* record = find(id);
                        if (record == nullptr || !record->has_physics_body)
                            return false;
                        return physics_->rigid_debug_state(id, out);
                    }

                    const std::vector<ContactEvent>& physics_contacts() const noexcept override
                    {
                        // A function-local empty rather than a member: this is the answer
                        // *only* before the physics exists, which is a state the editor is in
                        // for one construction, and a member would be a permanently-live
                        // vector kept for it.
                        static const std::vector<ContactEvent> NONE;
                        return physics_ != nullptr ? physics_->contact_events() : NONE;
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
                            record->ui_parameters = UIElementParameters{};
                            record->ui_parameters.kind = UIElementKind::Canvas;
                            record->ui_parameters.size_x = static_cast<Scalar>(ui_target_size_.x);
                            record->ui_parameters.size_y = static_cast<Scalar>(ui_target_size_.y);
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
                            record->ui_parameters = default_ui_parameters(kind);
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
                               record->ui_parameters.kind == UIElementKind::Canvas;
                    }

                    UIElementParameters ui_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        return record != nullptr ? record->ui_parameters : UIElementParameters{};
                    }

                    void set_ui_parameters(EntityId id,
                                           const UIElementParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_ui)
                            return;
                        record->ui_parameters = parameters;
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
                            record->ui_parameters = default_ui_parameters(UIElementKind::Image);
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
                                entry.second.ui_parameters.kind != UIElementKind::Canvas)
                                continue;
                            // In the default ConstantPixelSize mode a Canvas's rect always
                            // fills the actual target regardless of this size, but keeping the
                            // authored value in step with the viewport keeps the inspector's
                            // display honest and gives ScaleWithScreenSize the same tracking.
                            entry.second.ui_parameters.size_x =
                                static_cast<Scalar>(ui_target_size_.x);
                            entry.second.ui_parameters.size_y =
                                static_cast<Scalar>(ui_target_size_.y);
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

                    CameraParameters camera_parameters(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || !record->is_camera ||
                            !world_.alive(record->entity))
                            return CameraParameters{};
                        const Camera& c = world_.get<Camera>(record->entity);
                        CameraParameters parameters;
                        parameters.vertical_fov_radians = c.vertical_fov_radians;
                        parameters.near_plane = c.near_plane;
                        parameters.far_plane = c.far_plane;
                        parameters.display_index = c.display_index;
                        parameters.priority = c.priority;
                        parameters.active = c.active;
                        return parameters;
                    }

                    void set_camera_parameters(EntityId id,
                                               const CameraParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->is_camera ||
                            !world_.alive(record->entity))
                            return;
                        Camera& c = world_.get<Camera>(record->entity);
                        c.vertical_fov_radians = parameters.vertical_fov_radians;
                        c.near_plane = parameters.near_plane;
                        c.far_plane = parameters.far_plane;
                        c.display_index = parameters.display_index;
                        c.priority = parameters.priority;
                        c.active = parameters.active;
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
                        VFX::CPUDeterministicBackend::reset(record.particle_pool,
                                                            record.emitter_parameters.seed);
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

                    ParticleEmitterParameters particle_emitter_parameters(
                        EntityId id) const override
                    {
                        const Record* record = find(id);
                        return (record != nullptr && record->has_particle_emitter)
                                   ? record->emitter_parameters
                                   : ParticleEmitterParameters{};
                    }

                    void set_particle_emitter_parameters(
                        EntityId id, const ParticleEmitterParameters& parameters) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_particle_emitter)
                            return;
                        // A seed change restarts the deterministic stream, so the emitter reflects
                        // the new choice from a clean pool.
                        const bool restart = record->emitter_parameters.seed != parameters.seed;
                        record->emitter_parameters = parameters;
                        if (restart)
                            VFX::CPUDeterministicBackend::reset(record->particle_pool,
                                                                parameters.seed);
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
                            VFX::CPUDeterministicBackend::reset(record->particle_pool,
                                                                record->emitter_parameters.seed);
                            seed_emitter_effect(*record);
                        }
                        extract();
                    }

                    const VFX::ParticleEffect& particle_effect_source(EntityId id) const override
                    {
                        static const VFX::ParticleEffect EMPTY{};
                        const Record* record = find(id);
                        return (record != nullptr && record->has_particle_emitter)
                                   ? record->effect_source
                                   : EMPTY;
                    }

                    void set_particle_effect_source(EntityId id,
                                                    const VFX::ParticleEffect& effect) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || !record->has_particle_emitter)
                            return;
                        record->effect_source = effect;
                        if (record->effect_asset == VFX::INVALID_EFFECT)
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

                    void set_enabled(EntityId id, bool value) override
                    {
                        Record* record = find(id);
                        if (record != nullptr && record->enabled != value)
                        {
                            record->enabled = value;
                            // enabled_in_hierarchy() cascades: toggling this record can
                            // change the effective enabled state of any descendant, not
                            // just this entity's own components. Unlike destroy() — which
                            // only ever affects the one entity's own physics/soft/cloth/
                            // vehicle flags — we cannot cheaply know which descendants
                            // carry which physics-adjacent components without walking the
                            // subtree, so all four gathers are marked dirty unconditionally.
                            // The O(1) flag-set is dwarfed by the O(n) gather it triggers.
                            physics_dirty_ = true;
                            soft_dirty_ = true;
                            cloth_dirty_ = true;
                            vehicles_dirty_ = true;
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
                        // Which entity of a prefab this one was instantiated from, or
                        // empty. The key an override is matched by, and the reason a
                        // rebuild can compute local edits rather than being told about
                        // them at the moment they happen (prefab_system.md §10, P2).
                        std::string prefab_entity_id;
                        bool visible = true;
                        bool animated = false;
                        bool is_camera = false;
                        // Unity's `activeSelf`: this entity's own on/off switch. Distinct from
                        // `visible` (a render-only local flag, see below) — disabling an entity
                        // stops physics, audio and render for it and its whole subtree
                        // (`enabled_in_hierarchy`), where `visible=false` only stops its own
                        // render contribution.
                        bool enabled = true;
                        EntityId parent = NULL_ENTITY;
                        // Whether the Tint (Renderer) component is attached. Editor-created
                        // entities start with one; `set_has_renderer` toggles it. Cameras
                        // default to none, matching Unity's empty-GameObject-with-Camera.
                        bool has_renderer = false;
                        // The PBR material's metallic/roughness/emissive (albedo comes from
                        // the Tint each extract). Host bookkeeping keyed on EntityId, like
                        // the shape/collider parameters below — no ECS component.
                        Render::Material material{};
                        // The files the material's texture ids were loaded from — the
                        // persistence side of the handles, never touched by the extract.
                        MaterialTexturePaths material_texture_paths{};
                        // Whether the entity is tracked by physics_ (see set_has_physics_body).
                        // Unlike has_renderer/is_camera this needs no ECS migration, so it is
                        // plain host bookkeeping rather than a component toggle.
                        bool has_physics_body = false;
                        PhysicsBodyParameters physics_parameters{};
                        // How this entity walks, if it walks. Nothing is built from these
                        // numbers — they are read when a move is resolved and never
                        // otherwise — so unlike cloth an edit here dirties nothing.
                        bool has_character = false;
                        CharacterParameters character_parameters{};
                        // What this entity does when it is hit. Read by the engine's own
                        // physics listener and by nothing else, so like the character's
                        // numbers an edit here builds nothing and dirties nothing.
                        bool has_impact_response = false;
                        ImpactResponse impact_response{};
                        // Whether a cloth grid is tracked by the physics simulation (see
                        // set_has_cloth). Same plain-host-bookkeeping treatment as
                        // has_physics_body: cloth needs no ECS component migration.
                        bool has_cloth = false;
                        ClothParameters cloth_parameters{};
                        // A tetrahedral soft body (§9). Same plain-host-bookkeeping shape as
                        // cloth, but it carries its own cooked asset by value: a soft body
                        // cannot be rebuilt from numbers the way a grid can, so the blob is
                        // part of the record rather than a reference into something that
                        // might be reloaded out from under it.
                        bool has_soft_body = false;
                        SoftBodyParameters soft_body_parameters{};
                        // A crowd-batched skinned character (design §12.3/§12.4): same plain
                        // host bookkeeping as cloth (no ECS migration) — playback time is
                        // advanced on the fixed tick and sampled through crowd_evaluator_ at
                        // extract, keyed by EntityId, not a per-instance component.
                        bool has_crowd = false;
                        CrowdParameters crowd_parameters{};
                        // Which prefab this entity's subtree was built from, on the root of an
                        // instance and nowhere else. No `has_` flag beside it: a non-empty path
                        // is the flag, so there is one way to be an instance rather than two
                        // that can disagree.
                        PrefabInstanceParameters prefab_instance{};
                        // A punctual light on this entity: same plain host bookkeeping as
                        // cloth/shape, extracted into RenderScene::lights each frame with
                        // the entity's transform supplying the light's position and aim.
                        bool has_light = false;
                        LightParameters light_parameters{};
                        // Audio authoring, same plain host bookkeeping as light/cloth (no ECS
                        // migration): an emitter plays a sound at the entity's transform, a
                        // reverb zone imposes its I3DL2 reverb on a listener inside its box, and
                        // the listener marks the ears. Read live by the editor's audio system
                        // through the IWorldEditor accessors each wall-clock frame.
                        bool has_audio_emitter = false;
                        AudioEmitterParameters audio_emitter_parameters{};
                        bool has_reverb_zone = false;
                        ReverbZoneParameters reverb_zone_parameters{};
                        bool has_audio_listener = false;
                        AudioListenerParameters audio_listener_parameters{};
                        // A projected decal on this entity, same host bookkeeping as the
                        // light, extracted into RenderScene::decals each frame.
                        bool has_decal = false;
                        DecalParameters decal_parameters{};
                        // A deterministic particle emitter: same plain host bookkeeping as
                        // cloth (no ECS migration). The ~80 KB fixed pool lives here, off the
                        // ECS chunk, and is advanced on the fixed tick and extracted to
                        // RenderScene::particle_billboards each frame.
                        bool has_particle_emitter = false;
                        ParticleEmitterParameters emitter_parameters{};
                        VFX::DeterministicEmitterState particle_pool{};
                        /**
                         * @brief Runtime emitter state, kept off @ref emitter_parameters.
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
                        VFX::ParticleEffect effect_source{};
                        VFX::AssetId effect_asset = VFX::INVALID_EFFECT;
                        // Neither read nor written by any Schedule system, so — like
                        // has_physics_body/has_cloth — these are plain host bookkeeping
                        // rather than ECS components; no archetype migration needed.
                        bool has_shape = false;
                        ShapeParameters shape_parameters{};
                        bool has_collider = false;
                        ColliderParameters collider_parameters{};
                        // §5.5's PhysicsJoint: what this entity is attached to. Same plain
                        // host bookkeeping as the collider above — no Schedule system reads
                        // a joint, because the joint that matters is the solver's and this
                        // is only the authoring it is reconciled from (see sync_joints).
                        bool has_joint = false;
                        PhysicsJointParameters joint_parameters{};
                        /**
                         * @brief The live joint this authoring currently owns, if any.
                         *
                         * Runtime state kept off @ref joint_parameters for the reason
                         * `emitter_time` is kept off `emitter_parameters`: those are the
                         * authored numbers the scene file round-trips, and an identity
                         * handed out by the solver is neither authored nor persisted.
                         */
                        JointId live_joint = NULL_JOINT;
                        /**
                         * @brief Which revision of the authoring @ref live_joint was built from.
                         *
                         * A counter rather than a comparison of the parameters themselves.
                         * `PhysicsJointParameters` is trivially copyable but not free of padding,
                         * so a byte comparison could report a difference that is not one, and
                         * a field-by-field comparison is a second place every new joint
                         * parameter has to be remembered. A counter cannot be forgotten
                         * because the one function that bumps it is the one that writes.
                         */
                        std::uint32_t joint_revision = 0;
                        std::uint32_t live_joint_revision = 0;
                        /** @brief Whether the joint broke; see `IWorldEditor::joint_broken`. */
                        bool joint_broken = false;
                        // §5.5's VehicleInstance: which cooked vehicle this entity is, and
                        // what the driver is asking of it. Same plain host bookkeeping as
                        // the joint above.
                        bool has_vehicle = false;
                        VehicleInstanceParameters vehicle_parameters{};
                        VehicleInput vehicle_input{};
                        /**
                         * @brief The `.sushinodebeam` bytes, read once and held.
                         *
                         * Held rather than re-read per reconcile because a vehicle is a
                         * megabyte-scale blob and a reconcile happens whenever *any* vehicle
                         * in the scene changes. Keyed by the path it was read from, so an
                         * author repointing the component reloads and one who did not does
                         * not.
                         */
                        std::vector<std::byte> vehicle_asset;
                        std::string vehicle_asset_source;
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
                        UIElementParameters ui_parameters{};
                        // The real ECS entity mirroring `ui_parameters` into `UI::` components
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
                     * way — only the shape kind, default parameters, and (for Terrain)
                     * collider kind/parameters differ between callers.
                     *
                     * @param display_name       Display name for the new entity.
                     * @param kind               The visual Shape kind.
                     * @param default_parameters The Shape's initial parameters.
                     * @param collider_override  When set, the Collider's kind/parameters,
                     *                           overriding the default of matching the Shape
                     *                           exactly.
                     * @return The new entity's stable id.
                     */
                    EntityId create_primitive(const std::string& display_name, PrimitiveKind kind,
                                              const Vector3& default_parameters,
                                              const ColliderParameters* collider_override = nullptr)
                    {
                        const Entity entity = world_.spawn(
                            Transform{}, Orientation{},
                            Tint{Vector3{Scalar(0.8), Scalar(0.8), Scalar(0.8)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        Record record{entity, display_name, true, false};
                        record.has_renderer = true;
                        record.has_shape = true;
                        record.shape_parameters = ShapeParameters{kind, default_parameters};
                        record.has_collider = true;
                        record.collider_parameters = collider_override != nullptr
                                                     ? *collider_override
                                                     : ColliderParameters{kind, default_parameters};
                        records_.emplace(id, record);
                        extract();
                        return id;
                    }

                    /**
                     * @brief Reads @p record's `.sushinodebeam` if it is not the one already held.
                     *
                     * Keyed on the path rather than reloaded per reconcile, because a
                     * vehicle blob is megabyte-scale and a reconcile happens whenever *any*
                     * vehicle in the scene changes. A read that fails leaves the bytes empty
                     * and the source recorded, so a missing file is asked for once rather
                     * than once per tick — a path typo should not become a stat() storm.
                     *
                     * @param record The record whose asset to make current.
                     * @return Whether usable bytes are now held.
                     */
                    bool refresh_vehicle_asset(Record& record)
                    {
                        const std::string& path = record.vehicle_parameters.asset_path;
                        if (path.empty())
                        {
                            record.vehicle_asset.clear();
                            record.vehicle_asset_source.clear();
                            return false;
                        }
                        if (record.vehicle_asset_source == path)
                            return !record.vehicle_asset.empty();

                        record.vehicle_asset_source = path;
                        record.vehicle_asset.clear();
                        std::ifstream file(path, std::ios::binary | std::ios::ate);
                        if (!file)
                            return false;
                        const std::streamoff size = file.tellg();
                        if (size <= 0)
                            return false;
                        record.vehicle_asset.resize(std::size_t(size));
                        file.seekg(0);
                        file.read(reinterpret_cast<char*>(record.vehicle_asset.data()), size);
                        if (!file)
                        {
                            record.vehicle_asset.clear();
                            return false;
                        }
                        return true;
                    }

                    /**
                     * @brief One descriptor per entity carrying a loadable vehicle.
                     *
                     * The bytes are borrowed from the records, which outlive the call — the
                     * same arrangement `gather_soft_body_descriptions` uses, and safe for
                     * exactly as long as the call: instancing copies out everything the solve
                     * needs.
                     *
                     * Walked in authoring order rather than over the record map, for §12.1's
                     * first rule: a vehicle's four hundred bodies are added in this order, so
                     * a hash-order walk would be a body numbering that varied between runs.
                     */
                    std::vector<VehicleDescription> gather_vehicle_descriptions()
                    {
                        std::vector<VehicleDescription> descriptions;
                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr || !record->has_vehicle ||
                                !world_.alive(record->entity) || !enabled_in_hierarchy(record))
                                continue;
                            if (!refresh_vehicle_asset(*record))
                                continue;

                            VehicleDescription description;
                            description.id = id;
                            description.asset = record->vehicle_asset.data();
                            description.asset_size = record->vehicle_asset.size();
                            description.position = world_.get<Transform>(record->entity).position;
                            description.orientation =
                                world_.get<Orientation>(record->entity).rotation;
                            description.setup = record->vehicle_parameters.setup;
                            descriptions.push_back(description);
                        }
                        return descriptions;
                    }

                    /**
                     * @brief Writes each vehicle's solved core pose back onto its entity.
                     *
                     * The *core*, not a node: §11.2's hybrid puts the mass and the inertia in
                     * one rigid body and hangs a deformable shell off it, so a node's
                     * position is a panel's position and only the core's is the car's.
                     */
                    void read_back_vehicles()
                    {
                        if (physics_ == nullptr)
                            return;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_vehicle ||
                                !world_.alive(record->entity))
                                continue;
                            SolvedPose pose;
                            if (!physics_->vehicle_core_pose(id, pose))
                                continue;
                            world_.get<Transform>(record->entity).position = pose.position;
                            world_.get<Orientation>(record->entity).rotation = pose.orientation;
                        }
                    }

                    /**
                     * @brief Marks a record's joint authoring as changed since it was built.
                     *
                     * One function, so "the authoring moved" and "the live joint is stale"
                     * cannot drift apart: every write to @ref Record::joint_parameters goes
                     * through here, and the reconcile compares nothing but this counter.
                     * It also clears the broken flag, because an author editing a joint that
                     * tore off is asking for it back.
                     *
                     * @param record The record whose joint was written.
                     */
                    void touch_joint(Record& record) noexcept
                    {
                        ++record.joint_revision;
                        record.joint_broken = false;
                        joints_dirty_ = true;
                    }

                    /**
                     * @brief Whether @p record's joint has two endpoints the solver can hold.
                     *
                     * Both endpoints must be live entities that own rigid bodies, and they
                     * must be different entities. A joint from a body to itself is not a
                     * stiff joint but a degenerate one — both its rows reference the same
                     * slot, so the projection cancels and the colourer is handed a
                     * constraint that conflicts with itself — so it is refused here rather
                     * than discovered as a body that will not settle.
                     *
                     * @param record The record to test.
                     * @param owner  The entity that owns @p record, the joint's first body.
                     */
                    bool joint_endpoints_ready(const Record& record, EntityId owner) const noexcept
                    {
                        const EntityId partner = record.joint_parameters.connected_body;
                        if (partner == NULL_ENTITY || partner == owner)
                            return false;
                        if (!record.has_physics_body || !world_.alive(record.entity))
                            return false;
                        const Record* other = find(partner);
                        return other != nullptr && other->has_physics_body &&
                               world_.alive(other->entity);
                    }

                    /**
                     * @brief Reconciles the solver's joints with what the records author.
                     *
                     * A diff in the same shape as `IRigidBodyService::set_rigid_bodies`, and
                     * for the same reason: a joint that has not changed keeps its solver
                     * handle and therefore its warm start, so a scene that is merely being
                     * stepped rebuilds nothing.
                     *
                     * Walked in @ref order_ — the authoring order — rather than over
                     * @ref records_, which is a hash map whose iteration order is not a
                     * property of the scene. Joint identities are handed out in call order,
                     * so an unstable walk here would be an unstable joint numbering, and
                     * §12.1's first rule exists precisely because that kind of order leaks
                     * into results.
                     */
                    void sync_joints()
                    {
                        if (physics_ == nullptr)
                            return;
                        for (const EntityId id : order_)
                        {
                            Record* record = find(id);
                            if (record == nullptr)
                                continue;

                            const bool wanted = record->has_joint && !record->joint_broken &&
                                                joint_endpoints_ready(*record, id);
                            const bool stale =
                                record->live_joint_revision != record->joint_revision;
                            if (record->live_joint != NULL_JOINT && (!wanted || stale))
                            {
                                physics_->destroy_joint(record->live_joint);
                                record->live_joint = NULL_JOINT;
                            }
                            if (!wanted || record->live_joint != NULL_JOINT)
                                continue;

                            JointDescription description;
                            description.body_a = id;
                            description.body_b = record->joint_parameters.connected_body;
                            description.parameters = record->joint_parameters.joint;
                            record->live_joint = physics_->create_joint(description);
                            record->live_joint_revision = record->joint_revision;
                        }
                    }

                    /**
                     * @brief Records the joints that broke during the step just taken.
                     *
                     * The solver has already destroyed them, so this only catches the
                     * authoring up: the identity is dropped and the record is marked broken
                     * so the next reconcile does not immediately build the mount back. That
                     * flag is the whole reason this pass exists — without it a breakable
                     * joint would tear off and reappear on the following tick forever.
                     */
                    void collect_broken_joints()
                    {
                        if (physics_ == nullptr)
                            return;
                        for (const JointBrokenEvent& event : physics_->joint_broken_events())
                        {
                            Record* record = find(event.a);
                            if (record == nullptr || record->live_joint != event.joint)
                                continue;
                            record->live_joint = NULL_JOINT;
                            record->joint_broken = true;
                        }
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
                    static UIElementParameters default_ui_parameters(UIElementKind kind)
                    {
                        UIElementParameters parameters;
                        parameters.kind = kind;
                        switch (kind)
                        {
                            case UIElementKind::Canvas:
                                parameters.anchor_min_x = 0;
                                parameters.anchor_min_y = 0;
                                parameters.anchor_max_x = 1;
                                parameters.anchor_max_y = 1;
                                parameters.position_x = 0;
                                parameters.position_y = 0;
                                parameters.size_x = Scalar(1280);
                                parameters.size_y = Scalar(720);
                                break;
                            case UIElementKind::Text:
                                parameters.size_x = Scalar(200);
                                parameters.size_y = Scalar(40);
                                parameters.color = Vector3{1, 1, 1};
                                std::snprintf(parameters.text, sizeof(parameters.text), "%s",
                                              "Text");
                                break;
                            case UIElementKind::Button:
                                parameters.size_x = Scalar(160);
                                parameters.size_y = Scalar(48);
                                parameters.color = Vector3{Scalar(0.26), Scalar(0.5), Scalar(0.85)};
                                std::snprintf(parameters.text, sizeof(parameters.text), "%s",
                                              "Button");
                                break;
                            case UIElementKind::Image:
                            case UIElementKind::Panel:
                                parameters.size_x = Scalar(200);
                                parameters.size_y = Scalar(120);
                                parameters.color = Vector3{Scalar(0.85), Scalar(0.85), Scalar(0.9)};
                                break;
                        }
                        return parameters;
                    }

                    /** @brief Converts an authored `UIElementParameters` rect into a `UI::RectTransform`. */
                    static UI::RectTransform to_rect_transform(
                        const UIElementParameters& parameters) noexcept
                    {
                        UI::RectTransform transform;
                        transform.anchor_min =
                            UI::Vector2{parameters.anchor_min_x, parameters.anchor_min_y};
                        transform.anchor_max =
                            UI::Vector2{parameters.anchor_max_x, parameters.anchor_max_y};
                        transform.pivot = UI::Vector2{parameters.pivot_x, parameters.pivot_y};
                        transform.anchored_position =
                            UI::Vector2{parameters.position_x, parameters.position_y};
                        transform.size_delta = UI::Vector2{parameters.size_x, parameters.size_y};
                        return transform;
                    }

                    /** @brief Converts an authored fill/text colour into a `UI::Color` at full alpha-scaled opacity. */
                    static UI::Color to_ui_color(const Vector3& color, Scalar alpha) noexcept
                    {
                        return UI::Color{color.x, color.y, color.z, alpha};
                    }

                    /**
                     * @brief Mirrors @p record's `ui_parameters` into a real `UI::`-component
                     * ECS entity.
                     *
                     * `World` fixes an entity's component set at spawn time (no add/remove after
                     * the fact), so a UI record's mirror entity is destroyed and respawned
                     * whenever the required `UI::` component combination changes (i.e. when its
                     * `UIElementKind` changes, or the UI is first attached); otherwise the
                     * existing mirror's components are updated in place. This is the single
                     * point where host-side `UIElementParameters` bookkeeping is reconciled with
                     * the real `SushiEngine::UI::` components that
                     * `SushiEngine::UI::resolve_rect` actually lays out — the editor and any
                     * runtime UI overlay both read the mirror's
                     * `UI::ComputedRect`/`UI::RectTransform`, so there is exactly one UI layout
                     * mechanism in the engine.
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

                        const UIElementKind kind = record.ui_parameters.kind;
                        const bool needs_respawn =
                            !world_.alive(record.ui_mirror) || record.ui_mirror_kind != kind;
                        const UI::RectTransform transform = to_rect_transform(record.ui_parameters);

                        if (needs_respawn)
                        {
                            if (world_.alive(record.ui_mirror))
                                world_.destroy(record.ui_mirror);

                            switch (kind)
                            {
                                case UIElementKind::Canvas:
                                    record.ui_mirror = world_.spawn(
                                        UI::Canvas{UI::Vector2{record.ui_parameters.size_x,
                                                               record.ui_parameters.size_y}},
                                        transform, UI::ComputedRect{});
                                    break;
                                case UIElementKind::Text:
                                {
                                    UI::UIText text{};
                                    UI::set_text(text, record.ui_parameters.text);
                                    text.font_size = record.ui_parameters.font_size;
                                    text.color = to_ui_color(record.ui_parameters.color,
                                                             record.ui_parameters.alpha);
                                    record.ui_mirror =
                                        world_.spawn(transform, UI::ComputedRect{}, text);
                                    break;
                                }
                                case UIElementKind::Button:
                                {
                                    UI::UIButton button{};
                                    record.ui_mirror = world_.spawn(
                                        transform, UI::ComputedRect{},
                                        UI::UIImage{to_ui_color(record.ui_parameters.color,
                                                                record.ui_parameters.alpha)},
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
                                        UI::UIImage{to_ui_color(record.ui_parameters.color,
                                                                record.ui_parameters.alpha)});
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
                                    UI::Vector2{record.ui_parameters.size_x,
                                                record.ui_parameters.size_y};
                                break;
                            case UIElementKind::Text:
                            {
                                UI::UIText& text = world_.get<UI::UIText>(record.ui_mirror);
                                UI::set_text(text, record.ui_parameters.text);
                                text.font_size = record.ui_parameters.font_size;
                                text.color = to_ui_color(record.ui_parameters.color,
                                                         record.ui_parameters.alpha);
                                break;
                            }
                            case UIElementKind::Button:
                                world_.get<UI::UIImage>(record.ui_mirror).color = to_ui_color(
                                    record.ui_parameters.color, record.ui_parameters.alpha);
                                break;
                            case UIElementKind::Image:
                            case UIElementKind::Panel:
                            default:
                                world_.get<UI::UIImage>(record.ui_mirror).color = to_ui_color(
                                    record.ui_parameters.color, record.ui_parameters.alpha);
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

                    /**
                     * @brief Whether @p record and every ancestor above it are enabled.
                     *
                     * Unity's `activeInHierarchy`: physics, audio and render all gate existence
                     * on this, not on @ref Record::enabled alone, which is local to one entity.
                     */
                    bool enabled_in_hierarchy(const Record* record) const noexcept
                    {
                        std::size_t guard = records_.size() + 1;
                        for (const Record* current = record; current != nullptr && guard > 0;
                             --guard)
                        {
                            if (!current->enabled)
                                return false;
                            current = current->parent == NULL_ENTITY ? nullptr
                                                                     : find(current->parent);
                        }
                        return true;
                    }

                    /** @brief The object-to-world matrix for @p id, built from @ref world_transform. */
                    Matrix4 world_matrix(EntityId id) const
                    {
                        const EntityTransform world = world_transform(id);
                        return compose_transform(world.position, world.rotation, world.scale);
                    }

                    /**
                     * @brief Sends a transform edit through to the body that owns @p id.
                     *
                     * One authoring gesture, two meanings, and the body's own
                     * `kinematic` field is which. A dynamic body is *placed*: it jumps
                     * to the pose with its velocity cleared, because a crate dragged
                     * across a room was not thrown across it. A kinematic body is
                     * *moved*: the pose becomes a target the next tick derives a
                     * velocity from, so a platform dragged into a stack pushes the
                     * stack instead of teleporting through it.
                     *
                     * This is also the whole of how a game drives a platform — write
                     * the entity's transform each tick, from a script or an animation
                     * clip, and the physics follows. There is no second API to learn.
                     *
                     * A no-op when the body does not exist yet (before the first
                     * physics rebuild), which is when the rebuild seeds from this same
                     * transform instead.
                     *
                     * @param record   The entity's record, for the flag and the guard.
                     * @param id       The entity being edited.
                     * @param position The new world position.
                     * @param rotation The new world orientation.
                     */
                    void place_physics_body(const Record& record, EntityId id,
                                            const Vector3& position, const Quaternion& rotation)
                    {
                        if (!record.has_physics_body)
                            return;
                        if (record.physics_parameters.kinematic)
                            physics_->move_rigid_body(id, position, rotation);
                        else
                            physics_->set_rigid_pose(id, position, rotation);
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
                     * @brief The wind field, as the physics is allowed to see it (§11.6).
                     *
                     * The other half of §4.5's pair: the physics is handed a field and never
                     * the meteorology behind it, exactly as it is for gravity. Asked through
                     * the provider seam, so every provider answers — an ingested sky blows a
                     * car about with the wind it was handed, and a static one answers zero,
                     * which is the truth about a static sky rather than a capability it lacks.
                     *
                     * With no provider installed there is no sampler at all rather than one
                     * that returns zero: an empty `std::function` is the cheapest possible
                     * "still air", and the physics already skips it.
                     *
                     * Scene axes are east-x and north-z — the same mapping
                     * @ref geodetic_at_scene inverts — so a `WindSample`'s two components go
                     * straight in with no vertical term. Vertical wind is a real thing and
                     * the provider seam does not carry it; saying so here is cheaper than a
                     * zero that reads like a measurement.
                     *
                     * @return A sampler mapping a scene-frame position to wind, m/s.
                     */
                    WindSampler make_wind_sampler() const
                    {
                        if (!weather_provider_)
                            return WindSampler{};

                        const IWeatherProvider* weather = weather_provider_.get();
                        const double seconds = julian_date_ * 86400.0;
                        const double radius =
                            std::max(double(scene_.environment.planet.mean_radius()), 1.0);
                        const double latitude = scene_.environment.observer.latitude_radians;
                        const double longitude = scene_.environment.observer.longitude_radians;
                        constexpr double MIN_COS_LATITUDE = 0.05;
                        const double cos_latitude =
                            std::max(std::cos(latitude), MIN_COS_LATITUDE);

                        return [weather, seconds, radius, latitude, longitude,
                                cos_latitude](const Vector3& position) -> Vector3
                        {
                            const GeodeticPosition local{
                                latitude + double(position.z) / radius,
                                longitude + double(position.x) / (radius * cos_latitude)};
                            const double altitude = std::max(double(position.y), 0.0);
                            const WindSample wind =
                                weather_wind(*weather, local, altitude, seconds);
                            return Vector3{Scalar(wind.eastward_mps), 0,
                                           Scalar(wind.northward_mps)};
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
                     * frame → body-fixed by unwinding the prime meridian. The inverse of
                     * @ref body_fixed_to_scene; together they carry a surface pose between the
                     * scene and the geodetic coordinate the inspector authors.
                     *
                     * The unwind uses @ref Astro::prime_meridian_angle rather than the raw IAU
                     * W, because W is an angle in a different frame from the one the line above
                     * lands in; the two are 52.7 degrees apart for the Moon. Using W here put
                     * the reported longitude that far from the longitude the scene was built
                     * at, which nothing noticed while nothing else spoke body-fixed.
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
                            Scalar(Astro::prime_meridian_angle(body_id, julian_date_));
                        return rotate_about_pole(equatorial, -spin);
                    }

                    /** @brief The scene position of a body-fixed point; inverse of @ref scene_to_body_fixed. */
                    Vector3 body_fixed_to_scene(int body, const Vector3& body_fixed) const
                    {
                        const Astro::BodyId body_id = static_cast<Astro::BodyId>(body);
                        const Astro::SceneFrame frame = current_scene_frame();
                        const Scalar spin =
                            Scalar(Astro::prime_meridian_angle(body_id, julian_date_));
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
                        // A joint names two bodies, so a change to the body set is a change
                        // to which joints can exist: an entity that has just gained a body
                        // makes every joint pointing at it buildable, and one that has lost
                        // it takes its joints down with it inside the solver. Remembered
                        // before the reconcile below clears the flag.
                        const bool bodies_changed = physics_dirty_;
                        if (physics_dirty_)
                        {
                            physics_->set_rigid_bodies(gather_rigid_descriptions(),
                                                       PHYSICS_ITERATIONS, substep_dt());
                            physics_dirty_ = false;
                        }
                        if (soft_dirty_)
                        {
                            physics_->set_soft_bodies(gather_soft_body_descriptions());
                            soft_dirty_ = false;
                        }
                        if (cloth_dirty_)
                        {
                            physics_->set_cloth_grids(gather_cloth_descriptions(),
                                                      PHYSICS_ITERATIONS, substep_dt());
                            cloth_dirty_ = false;
                        }

                        // Refresh the static collision planes every step — cheap, and it
                        // tracks a moved terrain without extra dirty bookkeeping — then
                        // step, which resolves rigid/rigid, rigid/plane, and cloth/rigid
                        // contacts inside the solve.
                        if (vehicles_dirty_)
                        {
                            physics_->set_vehicles(gather_vehicle_descriptions());
                            vehicles_dirty_ = false;
                        }
                        if (joints_dirty_ || bodies_changed)
                        {
                            sync_joints();
                            joints_dirty_ = false;
                        }

                        physics_->set_static_planes(gather_static_planes());
                        physics_->step(make_gravity_sampler(), make_wind_sampler(),
                                       PHYSICS_SUBSTEPS_PER_TICK);
                        // After the step, because that is when its sinks were called:
                        // this advances the cooldowns those calls set and ends any burst
                        // whose time is up — which has to happen on a tick where nothing
                        // was hit, exactly the tick a sink is not called on.
                        impacts_.update(Scalar(clock_.fixed_dt()));
                        collect_broken_joints();
                        // The vehicle's pose follows the solve, like every rigid body's — its
                        // core is one, it simply is not an entity's own body.
                        read_back_vehicles();

                        // Advance the master epoch for this fixed step, so the sky and the
                        // on-rails bodies the gravity field sums track the physics solve. The
                        // sky is frozen when the time scale is zero.
                        const double step_days =
                            double(clock_.fixed_dt()) * time_scale_days_per_second_;
                        julian_date_ += step_days;
                        // The nest is stepped in game time, and this is that clock: the same
                        // scaled seconds the sky advances by, accumulated monotonically so the
                        // renderer can take its own difference against it.
                        atmosphere_seconds_ += step_days * 86400.0;

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
                     * instead (see `IRigidBodyService::set_rigid_bodies`). Built fresh each
                     * rebuild from the current entity set, so a destroyed entity simply drops
                     * out of the list.
                     *
                     * @return One descriptor per live physics-driven entity, in display order.
                     */
                    std::vector<RigidBodyDescription> gather_rigid_descriptions() const
                    {
                        return extract_rigid_bodies(physics_source_entities());
                    }

                    /**
                     * @brief The live entity set, flattened into what the extract reads.
                     *
                     * This is the whole of what `RuntimeSimulation` still owes the
                     * physics translation: which entities are alive, in what order,
                     * and where the hierarchy puts them. Deciding what a collider
                     * *means* is `physics_extract.hpp`'s job, and it can be tested
                     * without a world precisely because this function is the only
                     * part that needs one.
                     *
                     * @return One entry per live entity, in display order.
                     */
                    std::vector<PhysicsSourceEntity> physics_source_entities() const
                    {
                        std::vector<PhysicsSourceEntity> entities;
                        entities.reserve(order_.size());
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !world_.alive(record->entity) ||
                                !enabled_in_hierarchy(record))
                                continue;

                            PhysicsSourceEntity entity;
                            entity.id = id;
                            entity.local_position =
                                world_.get<Transform>(record->entity).position;
                            entity.local_orientation =
                                world_.get<Orientation>(record->entity).rotation;
                            entity.local_scale = world_.get<Transform>(record->entity).scale;

                            const EntityTransform world = world_transform(id);
                            entity.world_position = world.position;
                            entity.world_orientation = world.rotation;
                            entity.world_scale = world.scale;

                            entity.has_physics_body = record->has_physics_body;
                            entity.physics_parameters = record->physics_parameters;
                            entity.has_collider = record->has_collider;
                            entity.collider_parameters = record->collider_parameters;
                            entity.has_shape = record->has_shape;
                            // Only the kind and dimensions travel here: `mesh`/`mesh_path`
                            // are a rendering concern resolve_collider never reads, and this
                            // is rebuilt for every live entity every fixed tick (see
                            // ShapeColliderInput).
                            entity.shape_parameters =
                                ShapeColliderInput{record->shape_parameters.kind,
                                                   record->shape_parameters.parameters};
                            entities.push_back(entity);
                        }
                        return entities;
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
                    std::vector<PlaneDescription> gather_static_planes() const
                    {
                        return extract_static_planes(physics_source_entities());
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
                    /**
                     * @brief Collects a descriptor per live soft-body entity for a rebuild.
                     *
                     * The descriptors point *into* each record's blob rather than copying
                     * it, which is safe for exactly as long as the call: `set_soft_bodies`
                     * reads the bytes to instantiate and retains nothing. Copying here
                     * instead would duplicate every asset in the scene once per rebuild.
                     *
                     * @return One descriptor per live soft-body entity with a non-empty asset.
                     */
                    std::vector<SoftBodyDescription> gather_soft_body_descriptions() const
                    {
                        std::vector<SoftBodyDescription> descriptions;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_soft_body ||
                                record->soft_body_parameters.asset.empty() ||
                                !enabled_in_hierarchy(record))
                                continue;
                            const SoftBodyParameters& parameters = record->soft_body_parameters;
                            SoftBodyDescription description;
                            description.id = id;
                            description.asset = parameters.asset.data();
                            description.asset_size = parameters.asset.size();
                            description.level = parameters.level;
                            description.material = parameters.material;
                            description.thickness = parameters.thickness;
                            description.self_collision = parameters.self_collision;
                            description.cosmetic = parameters.cosmetic;
                            if (world_.alive(record->entity))
                                description.origin = world_.get<Transform>(record->entity).position;
                            descriptions.push_back(description);
                        }
                        return descriptions;
                    }

                    std::vector<ClothDescription> gather_cloth_descriptions() const
                    {
                        std::vector<ClothDescription> descriptions;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_cloth ||
                                !world_.alive(record->entity) ||
                                record->cloth_parameters.rows == 0 ||
                                record->cloth_parameters.cols == 0 ||
                                !enabled_in_hierarchy(record))
                                continue;
                            ClothDescription description;
                            description.id = id;
                            description.rows = record->cloth_parameters.rows;
                            description.cols = record->cloth_parameters.cols;
                            description.spacing = record->cloth_parameters.spacing;
                            description.origin = world_.get<Transform>(record->entity).position;
                            description.compliance = record->cloth_parameters.compliance;
                            description.thickness = record->cloth_parameters.spacing * Scalar(0.25);
                            descriptions.push_back(description);
                        }
                        return descriptions;
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
                        const Matrix4 rotation = matrix4_from_quaternion(orientation.rotation);
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
                            scene_.environment.clouds =
                                weather_compiler_.compile(column, scene_.environment.clouds);
                            scene_.environment.weather = weather_world_compiler_.compile(column);

                            // The spatial half (docs/design/atmosphere_system.md §7): the same
                            // provider's horizontal structure, published as the field the
                            // cloudscape bake resolves a genus and a coverage from, per baked
                            // column. The compiled `Cloudscape` above no longer decides what is
                            // in the sky when that field classifies (§7.4) -- it still carries
                            // the medium's scattering knobs, the weather/erosion scales and the
                            // editor's readout, which are properties of the whole sky rather
                            // than of one column.
                            weather_provider_->publish_field(observer, weather_field_buffer_);
                            scene_.environment.weather_field = weather_field_buffer_.view();

                            // The planetary half. The field above is a lattice over a few
                            // hundred kilometres, which is everything a baked window can see and
                            // nothing a camera in orbit can; this is the same weather stated as
                            // a closed form over the whole body, for the part of the cloud march
                            // that runs past every baked window. See `Render::SynopticFieldView`.
                            publish_synoptic_field();

                            // The parent solution the GPU nest's lateral boundary relaxes toward,
                            // plus the clock it steps on. This is the only channel the
                            // simulation's own time reaches the render tier through, and
                            // publishing it is what tells the renderer there is an atmosphere to
                            // build at all.
                            weather_provider_->publish_forcing(observer, atmosphere_seconds_,
                                                               atmosphere_forcing_buffer_);
                            // f = 2 Omega sin(latitude). Earth's sidereal rotation rate: the
                            // nest is regional, so f's own variation across its few hundred
                            // kilometres is below what the grid resolves and one value at the
                            // centre is the standard f-plane approximation.
                            constexpr double SIDEREAL_ROTATION_RAD_PER_S = 7.2921159e-5;
                            const double coriolis = 2.0 * SIDEREAL_ROTATION_RAD_PER_S *
                                                    std::sin(observer.latitude_radians);
                            double scene_observer_x = 0.0;
                            double scene_observer_z = 0.0;
                            if (scene_.has_camera)
                            {
                                scene_observer_x = double(scene_.camera.position.x);
                                scene_observer_z = double(scene_.camera.position.z);
                            }
                            // The sine of the *rendered* sun's elevation: the dot of the
                            // direction to the sun with local up. §1.6 records the shipped
                            // system reimplementing its own solar-position model so that "the
                            // sun that heats the ground and the sun that is rendered are two
                            // different suns" -- this is the same sun, read straight off the
                            // environment the ephemeris already filled. It carries the time of
                            // day and the season at once, because the declination that lifts the
                            // summer sun is already in it.
                            const Vector3& to_sun = scene_.environment.sun.direction;
                            const Vector3& local_up = scene_.environment.planet_pole;
                            const double up_length =
                                std::sqrt(local_up.x * local_up.x + local_up.y * local_up.y +
                                          local_up.z * local_up.z);
                            const double sun_length =
                                std::sqrt(to_sun.x * to_sun.x + to_sun.y * to_sun.y +
                                          to_sun.z * to_sun.z);
                            const double solar_elevation_sine =
                                (up_length > 0.0 && sun_length > 0.0)
                                    ? (to_sun.x * local_up.x + to_sun.y * local_up.y +
                                       to_sun.z * local_up.z) / (up_length * sun_length)
                                    : 0.0;
                            scene_.environment.atmosphere_forcing =
                                atmosphere_forcing_buffer_.view(scene_observer_x, scene_observer_z,
                                                                atmosphere_seconds_,
                                                                static_cast<float>(coriolis),
                                                                static_cast<float>(solar_elevation_sine));
                        }
                        else
                        {
                            // No dynamic weather: leave the render tier exactly as it behaved
                            // before the field existed (every WeatherCoupling field defaults to
                            // zero/no-op, and an invalid field is ignored outright).
                            scene_.environment.weather = Render::WeatherCoupling{};
                            scene_.environment.weather_field = Render::WeatherField{};
                            // No provider, no parent solution — so the renderer never builds a
                            // nest and a scene without weather costs nothing for one.
                            scene_.environment.atmosphere_forcing = Render::AtmosphereForcing{};
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

                            if (!record->has_shape || !record->has_renderer ||
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            const Tint& tint = world_.get<Tint>(record->entity);
                            RenderInstance instance;
                            instance.id = id;
                            instance.model = world_matrix(id);
                            instance.color = tint.color;
                            instance.shape_kind = record->shape_parameters.kind;
                            instance.shape_parameters = record->shape_parameters.parameters;
                            instance.mesh = record->shape_parameters.mesh;
                            // Albedo tracks the entity's Tint; the rest of the PBR material is
                            // the authored per-entity record.
                            instance.material = record->material;
                            instance.material.albedo = tint.color;
                            scene_.instances.push_back(instance);
                        }

                        scene_.deformable_instances.clear();
                        scene_.deformable_vertices.clear();
                        scene_.deformable_indices.clear();
                        // Reused across entities so a scene full of cloth allocates once.
                        std::vector<std::uint32_t> grid_indices;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_cloth ||
                                !enabled_in_hierarchy(record) || !record->visible)
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
                                rows = static_cast<std::uint32_t>(record->cloth_parameters.rows);
                                cols = static_cast<std::uint32_t>(record->cloth_parameters.cols);
                                if (rows == 0 || cols == 0)
                                    continue;
                                const Vector3 origin =
                                    world_.get<Transform>(record->entity).position;
                                const Scalar spacing = record->cloth_parameters.spacing;
                                positions.reserve(static_cast<std::size_t>(rows) * cols);
                                for (std::uint32_t r = 0; r < rows; ++r)
                                    for (std::uint32_t c = 0; c < cols; ++c)
                                        positions.push_back(
                                            Vector3{origin.x + Scalar(c) * spacing, origin.y,
                                                    origin.z + Scalar(r) * spacing});
                            }
                            if (positions.empty())
                                continue;
                            // The grid's triangulation, emitted once here rather than derived
                            // downstream: the renderer's seam takes a triangle list, and a grid
                            // is now just one of the shapes that can produce one.
                            Render::build_grid_indices(rows, cols, grid_indices);
                            if (grid_indices.empty())
                                continue;

                            DeformableInstance surface;
                            surface.id = id;
                            surface.first_vertex =
                                static_cast<std::uint32_t>(scene_.deformable_vertices.size());
                            surface.vertex_count = static_cast<std::uint32_t>(positions.size());
                            surface.first_index =
                                static_cast<std::uint32_t>(scene_.deformable_indices.size());
                            surface.index_count =
                                static_cast<std::uint32_t>(grid_indices.size());
                            if (record->has_renderer && world_.alive(record->entity))
                                surface.color = world_.get<Tint>(record->entity).color;
                            else
                                surface.color = record->material.albedo;
                            scene_.deformable_vertices.insert(scene_.deformable_vertices.end(),
                                                              positions.begin(), positions.end());
                            scene_.deformable_indices.insert(scene_.deformable_indices.end(),
                                                             grid_indices.begin(),
                                                             grid_indices.end());
                            scene_.deformable_instances.push_back(surface);
                        }

                        // Tetrahedral soft bodies, into the same channel the cloth above
                        // just filled. That they share it is the point of P6-G2: a sheet
                        // and a tetrahedral surface differ in how their triangles were
                        // produced and in nothing the renderer can see.
                        //
                        // Read straight off the live bodies, with no cache between. That is
                        // §8.6's third invariant made structural rather than promised: the
                        // render mesh cannot lag the simulation by a tick because there is
                        // no second copy of it to be a tick behind.
                        std::vector<Vector3> soft_positions;
                        std::vector<std::uint32_t> soft_indices;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_soft_body ||
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            if (!physics_->soft_body_surface(id, soft_positions, soft_indices))
                                continue;
                            if (soft_positions.empty() || soft_indices.size() < 3)
                                continue;

                            DeformableInstance surface;
                            surface.id = id;
                            surface.first_vertex =
                                static_cast<std::uint32_t>(scene_.deformable_vertices.size());
                            surface.vertex_count =
                                static_cast<std::uint32_t>(soft_positions.size());
                            surface.first_index =
                                static_cast<std::uint32_t>(scene_.deformable_indices.size());
                            surface.index_count =
                                static_cast<std::uint32_t>(soft_indices.size());
                            if (record->has_renderer && world_.alive(record->entity))
                                surface.color = world_.get<Tint>(record->entity).color;
                            else
                                surface.color = record->material.albedo;
                            scene_.deformable_vertices.insert(scene_.deformable_vertices.end(),
                                                              soft_positions.begin(),
                                                              soft_positions.end());
                            scene_.deformable_indices.insert(scene_.deformable_indices.end(),
                                                             soft_indices.begin(),
                                                             soft_indices.end());
                            scene_.deformable_instances.push_back(surface);
                        }

                        // §11.2's shell, drawn as the surface it collides as. Same channel as
                        // the cloth and the soft body above and for the same reason P6-G2
                        // gives: how a surface's triangles were produced is not something the
                        // renderer can see. A vehicle was invisible until this loop existed —
                        // its entity followed the rigid core and nothing drew the body.
                        std::vector<Vector3> shell_positions;
                        std::vector<std::uint32_t> shell_indices;
                        for (const EntityId id : order_)
                        {
                            const Record* record = find(id);
                            if (record == nullptr || !record->has_vehicle ||
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            if (!physics_->vehicle_surface(id, shell_positions, shell_indices))
                                continue;
                            if (shell_positions.empty() || shell_indices.size() < 3)
                                continue;

                            DeformableInstance surface;
                            surface.id = id;
                            surface.first_vertex =
                                static_cast<std::uint32_t>(scene_.deformable_vertices.size());
                            surface.vertex_count =
                                static_cast<std::uint32_t>(shell_positions.size());
                            surface.first_index =
                                static_cast<std::uint32_t>(scene_.deformable_indices.size());
                            surface.index_count =
                                static_cast<std::uint32_t>(shell_indices.size());
                            if (record->has_renderer && world_.alive(record->entity))
                                surface.color = world_.get<Tint>(record->entity).color;
                            else
                                surface.color = record->material.albedo;
                            scene_.deformable_vertices.insert(scene_.deformable_vertices.end(),
                                                              shell_positions.begin(),
                                                              shell_positions.end());
                            scene_.deformable_indices.insert(scene_.deformable_indices.end(),
                                                             shell_indices.begin(),
                                                             shell_indices.end());
                            scene_.deformable_instances.push_back(surface);
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
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            const VFX::DeterministicEmitterState& pool = record->particle_pool;
                            for (std::uint32_t i = 0; i < pool.alive_count; ++i)
                            {
                                const VFX::GPUParticle& particle = pool.particles[i];
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
                                !enabled_in_hierarchy(record) || !record->visible ||
                                !world_.alive(record->entity))
                                continue;
                            const VFX::CompiledEffect* compiled = effect_for(*record);
                            if (compiled == nullptr)
                                continue;

                            const Matrix4 model = world_matrix(id);
                            const float* curve_luts =
                                compiled->curve_luts.empty() ? nullptr : compiled->curve_luts.data();
                            const float* gradient_luts = compiled->gradient_luts.empty()
                                                             ? nullptr
                                                             : compiled->gradient_luts.data();
                            for (const VFX::CompiledEmitter& emitter : compiled->emitters)
                            {
                                if (emitter.domain != VFX::SimulationDomain::Cosmetic)
                                    continue;

                                // The spawn count is the host's to compute — the emit shader is a
                                // pure allocator — so the fractional carry lives on the record and
                                // survives the frame that could not afford a whole particle.
                                std::uint32_t spawn_count = 0;
                                if (record->emitter_parameters.playing && emitter.spawn_rate > 0.0f)
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
                                view.seed = record->emitter_parameters.seed;
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
                        // QualityParameters::gpu_particles's own doc ("the deterministic CPU
                        // particle path is unaffected; it is gameplay, not a quality knob"),
                        // since ambient weather rain is squarely cosmetic.
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
                            // weather_wind(): the provider's own wind plus a local perturbation,
                            // near the surface; lateral drift only, scaled down below. Asked
                            // through the provider seam, so every provider answers -- an ingested
                            // sky drifts its rain with the wind it was handed, and a static one
                            // answers zero, which is the truth about a static sky rather than a
                            // capability it lacks.
                            const WindSample wind =
                                weather_wind(*weather_provider_, local, /*altitude_meters=*/50.0,
                                             julian_date_ * 86400.0);

                            VFX::CompiledEmitter& rain = weather_rain_emitter_;
                            rain = VFX::CompiledEmitter{};
                            rain.capacity = 4096;
                            rain.domain = VFX::SimulationDomain::Cosmetic;
                            rain.duration = 5.0f;
                            rain.flags = VFX::EMITTER_LOOPING;
                            rain.spawn_rate = 900.0f * precipitation;
                            rain.shape = VFX::EmitterShape::Box;
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
                            rain.update_flags = VFX::UPDATE_GRAVITY;
                            // weather_wind()'s lateral drift, scaled well down: rain falls in ~2 s,
                            // so even a brisk wind should nudge the streak, not fling it sideways.
                            constexpr float WIND_DRIFT_SCALE = 0.2f;
                            rain.gravity[0] = static_cast<float>(wind.eastward_mps) * WIND_DRIFT_SCALE;
                            rain.gravity[1] = -3.0f;
                            rain.gravity[2] = static_cast<float>(wind.northward_mps) * WIND_DRIFT_SCALE;
                            rain.blend = VFX::BlendMode::Alpha;
                            rain.alignment = VFX::RenderAlignment::VelocityStretched;
                            rain.velocity_stretch = 0.04f;
                            rain.render_flags = VFX::RENDER_SOFT;
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
                            VFX::CompiledEmitter& wisp = weather_wisp_emitter_;
                            wisp = VFX::CompiledEmitter{};
                            wisp.capacity = 1024;
                            wisp.domain = VFX::SimulationDomain::Cosmetic;
                            wisp.duration = 6.0f;
                            wisp.flags = VFX::EMITTER_LOOPING;
                            wisp.spawn_rate = 40.0f; // sparse and wispy, not a dense fog sheet.
                            wisp.shape = VFX::EmitterShape::Box;
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
                            wisp.update_flags = VFX::UPDATE_TURBULENCE;
                            wisp.turbulence_frequency = 0.05f;
                            wisp.turbulence_amplitude = 1.2f;
                            wisp.blend = VFX::BlendMode::Alpha;
                            wisp.alignment = VFX::RenderAlignment::FaceCamera;
                            wisp.render_flags = VFX::RENDER_SOFT;
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
                            if (record == nullptr || !record->has_light ||
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            const Matrix4 model = world_matrix(id);
                            constexpr float degrees_to_radians = 0.017453292519943295f;
                            Render::PunctualLight light;
                            light.position = WorldVector3{model.m[12], model.m[13], model.m[14]};
                            // The spot aims down the entity's local -Z, like a camera's gaze;
                            // scale is divided out by normalising.
                            light.direction =
                                normalize(Vector3{-model.m[8], -model.m[9], -model.m[10]});
                            light.color = record->light_parameters.color;
                            light.intensity = record->light_parameters.intensity;
                            light.range = record->light_parameters.range;
                            light.type = record->light_parameters.is_spot ? Render::LightType::Spot
                                                                      : Render::LightType::Point;
                            light.casts_shadows = record->light_parameters.casts_shadows;
                            light.inner_cone =
                                record->light_parameters.inner_degrees * degrees_to_radians;
                            light.outer_cone =
                                record->light_parameters.outer_degrees * degrees_to_radians;
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
                            if (record == nullptr || !record->has_decal ||
                                !enabled_in_hierarchy(record) || !record->visible)
                                continue;
                            const Matrix4 model = world_matrix(id);
                            Render::Decal decal;
                            decal.position = WorldVector3{model.m[12], model.m[13], model.m[14]};
                            decal.right = normalize(Vector3{model.m[0], model.m[1], model.m[2]});
                            decal.up = normalize(Vector3{model.m[4], model.m[5], model.m[6]});
                            decal.forward = normalize(Vector3{model.m[8], model.m[9], model.m[10]});
                            decal.half_extents = record->decal_parameters.half_extents;
                            decal.color = record->decal_parameters.color;
                            decal.opacity = record->decal_parameters.opacity;
                            decal.albedo_map = record->decal_parameters.albedo_map;
                            decal.orm_map = record->decal_parameters.orm_map;
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
                    const VFX::CompiledEffect* effect_for(const Record& record)
                    {
                        if (record.effect_asset == VFX::INVALID_EFFECT)
                            return nullptr;
                        const VFX::CompiledEffect& compiled =
                            effect_db_.compiled(record.effect_asset);
                        return compiled.emitters.empty() ? nullptr : &compiled;
                    }

                    /** @brief Gives a freshly added emitter the effect it starts from. */
                    void seed_emitter_effect(Record& record)
                    {
                        if (record.effect_asset != VFX::INVALID_EFFECT)
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
                    static const VFX::CompiledEmitter* first_deterministic(
                        const VFX::CompiledEffect& compiled) noexcept
                    {
                        for (const VFX::CompiledEmitter& emitter : compiled.emitters)
                        {
                            if (emitter.domain == VFX::SimulationDomain::Deterministic)
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
                                !record->emitter_parameters.playing ||
                                !world_.alive(record->entity))
                                continue;
                            const VFX::CompiledEffect* compiled = effect_for(*record);
                            if (compiled == nullptr)
                                continue;
                            record->emitter_time += dt;

                            // Only the deterministic emitters are stepped here; a cosmetic one is
                            // simulated by the renderer, and the extract merely places it.
                            const VFX::CompiledEmitter* emitter = first_deterministic(*compiled);
                            if (emitter == nullptr)
                                continue;
                            const Vector3 position = world_.get<Transform>(record->entity).position;
                            const Quaternion rotation =
                                world_.get<Orientation>(record->entity).rotation;
                            VFX::CPUDeterministicBackend::step(record->particle_pool, *emitter,
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
                                !record->crowd_parameters.playing)
                                continue;
                            record->crowd_parameters.time_seconds += dt;
                        }
                    }

                    /**
                     * @brief Samples every crowd entity sharing this frame's bound skeleton
                     * through the SYCL device evaluator and fills @ref scene_'s skinned
                     * instances (design §12.3/§12.4).
                     *
                     * The frame's bound skeleton is whichever crowd entity's
                     * `crowd_parameters.skeleton` is seen first while walking @ref order_; every
                     * later entity naming a
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
                            if (record == nullptr || !record->has_crowd ||
                                !enabled_in_hierarchy(record) || !record->visible ||
                                record->crowd_parameters.skeleton == 0 ||
                                record->crowd_parameters.mesh == Render::INVALID_MESH)
                                continue;
                            if (batch_skeleton_handle == 0)
                                batch_skeleton_handle = record->crowd_parameters.skeleton;
                            if (record->crowd_parameters.skeleton != batch_skeleton_handle)
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

                        std::vector<Animation::DeviceInstanceDescription> instances;
                        instances.reserve(batch_entities.size());
                        std::vector<EntityId> included_entities;
                        included_entities.reserve(batch_entities.size());
                        for (const EntityId id : batch_entities)
                        {
                            const Record* record = find(id);
                            const std::uint32_t clip_handle = record->crowd_parameters.clip;
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

                            Animation::DeviceInstanceDescription description;
                            description.clip_handle = bound->second;
                            description.time_seconds = record->crowd_parameters.time_seconds;
                            description.loop = record->crowd_parameters.loop ? 1u : 0u;
                            instances.push_back(description);
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
                            instance.mesh = record->crowd_parameters.mesh;
                            instance.material = record->crowd_parameters.material;
                            scene_.skinned_instances.push_back(instance);
                        }
                    }

                    /** @brief A deterministic fire plume: buoyant cone, warm colour ramp. */
                    static VFX::ParticleEffect make_fire_effect()
                    {
                        VFX::EmitterDescriptor e;
                        e.name = "Fire";
                        e.domain = VFX::SimulationDomain::Deterministic;
                        e.capacity = 512;
                        e.spawn.rate_per_second = 140.0f;
                        e.shape.shape = VFX::EmitterShape::Cone;
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
                        e.size_over_life.curve.add_key(VFX::CurveKey{0.0f, 0.3f, 0.0f, 1.5f});
                        e.size_over_life.curve.add_key(VFX::CurveKey{0.3f, 1.0f, 0.0f, 0.0f});
                        e.size_over_life.curve.add_key(VFX::CurveKey{1.0f, 0.0f, -1.0f, 0.0f});
                        e.color_over_life.enabled = true;
                        e.color_over_life.gradient.add_color_key(VFX::ColorKey{0.0f, Vector3{1.0, 0.9, 0.45}});
                        e.color_over_life.gradient.add_color_key(VFX::ColorKey{0.5f, Vector3{1.0, 0.35, 0.08}});
                        e.color_over_life.gradient.add_color_key(VFX::ColorKey{1.0f, Vector3{0.15, 0.03, 0.02}});
                        e.color_over_life.gradient.add_alpha_key(VFX::AlphaKey{0.0f, 0.0f});
                        e.color_over_life.gradient.add_alpha_key(VFX::AlphaKey{0.1f, 1.0f});
                        e.color_over_life.gradient.add_alpha_key(VFX::AlphaKey{1.0f, 0.0f});
                        VFX::ParticleEffect effect;
                        effect.name = "Fire";
                        effect.emitters.push_back(e);
                        return effect;
                    }


                    SushiRuntime::API::Runtime runtime_;
                    Execution::Context execution_;
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
                    std::unique_ptr<IPhysicsScene> physics_;
                    // Declared after `physics_` so it is destroyed before it, which is
                    // the order that matters: the scene holds a raw pointer to this and
                    // calls it inside the tick.
                    ImpactResponseListener impacts_;
                    bool physics_dirty_ = false;
                    bool cloth_dirty_ = false;
                    bool soft_dirty_ = false;
                    // Whether any record's joint authoring has moved since the solver's
                    // joints were last reconciled with it. A change to the *body* set
                    // implies one of these too, and step_once() folds the two together.
                    bool joints_dirty_ = false;
                    // Whether any record's vehicle authoring has moved since the physics was
                    // handed its set. Unlike bodies and joints this is a *rebuild* flag: a
                    // vehicle is four hundred bodies placed relative to a cooked structure,
                    // and there is no patching one in place.
                    bool vehicles_dirty_ = false;

                    // The weather seam: ticked from step_once() and compiled into
                    // scene_.environment.clouds from extract() whenever set. Null (the default)
                    // leaves clouds exactly as manual authoring already sets them. Held as the
                    // *interface*, not a concrete provider — see install_weather_provider.
                    std::unique_ptr<IWeatherProvider> weather_provider_;
                    // The installed provider's authoring capability, resolved once at install
                    // rather than re-queried per call; null when it has none.
                    IWeatherAuthoring* weather_authoring_ = nullptr;
                    // Where the sky comes from, and the seed Manual places it from. The mode
                    // decides which provider is installed; the seed is kept across a mode switch
                    // so returning to Manual returns the same sky (ISimulation::weather_seed).
                    WeatherMode weather_mode_ = WeatherMode::Manual;
                    std::uint64_t weather_seed_ = 1;
                    // Storage behind Environment::weather_field, which borrows it (see
                    // Render::WeatherField). Owned here because this object outlives every frame
                    // whose environment can still be read.
                    WeatherFieldBuffer weather_field_buffer_;

                    // Storage behind Environment::atmosphere_forcing, borrowed the same way the
                    // weather field's is, plus the monotonic game clock the nest steps on.
                    AtmosphereForcingBuffer atmosphere_forcing_buffer_;
                    double atmosphere_seconds_ = 0.0;
                    // The renderer's readback of the nest, bound once by the host. Null until
                    // then, which every consumer reads as "answer from the base state".
                    const Render::IAtmosphereMirror* atmosphere_mirror_ = nullptr;
                    WeatherCloudscapeCompiler weather_compiler_;
                    WeatherWorldCoupling weather_world_compiler_;

                    // W5 precipitation VFX (see extract()'s particle-emitter section): a single,
                    // sim-owned cosmetic rain emitter reused frame to frame rather than an
                    // authored ECS entity. ParticleEmitterView::compiled is a non-owning pointer
                    // that must outlive the frame's render, so this has to be a persistent member,
                    // not a stack local built inside extract().
                    VFX::CompiledEmitter weather_rain_emitter_;
                    float weather_rain_spawn_carry_ = 0.0f;

                    // W6 canopy wisp VFX (see extract()'s particle-emitter section): the same
                    // persistent-member reasoning as weather_rain_emitter_ above.
                    VFX::CompiledEmitter weather_wisp_emitter_;
                    float weather_wisp_spawn_carry_ = 0.0f;

                    // The deterministic particle path: a small library of built-in effects
                    // (Deterministic domain) an emitter entity references by index, and their
                    // display names for the inspector's picker. Compiled lazily on first step.
                    VFX::EffectDatabase effect_db_;

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
                    // by the 1-based handle CrowdParameters::skeleton/clip name (handle 0 is always
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
