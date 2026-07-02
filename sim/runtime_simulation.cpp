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
#include <string>
#include <unordered_map>
#include <vector>

#include <SushiEngine/SushiEngine.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace sim
    {
        namespace
        {
            // Components. Rotation is split from translation into its own column so the
            // spin and orbit systems write disjoint components and the dependency
            // tracker runs them in parallel. All are trivially copyable (enforced by
            // component_id), and kept in this single TU so their component ids agree.
            struct Transform
            {
                Vec3 position;
                Vec3 scale{Vec3{1, 1, 1}};
            };
            struct Orientation
            {
                Quat rotation;
            };
            struct SpinStep
            {
                Quat delta; // per-step rotation, precomputed on the host
            };
            struct OrbitState
            {
                Vec3 center;
                Scalar radius = 0;
                Scalar cos_angle = 1; // current orbit angle as a unit (cos, sin) pair
                Scalar sin_angle = 0;
                Scalar step_cos = 1;  // per-step rotation of that pair, precomputed
                Scalar step_sin = 0;
            };
            struct Tint
            {
                Vec3 color;
            };
            // A camera entity's lens and routing. Its pose comes from Transform +
            // Orientation; this adds the projection and which display it drives. Its own
            // archetype (no Tint), so camera entities are never drawn as cubes and never
            // match the spin/orbit systems. Trivially copyable like every component.
            struct Camera
            {
                Scalar vertical_fov_radians = Scalar(1.0471976);
                Scalar near_plane = Scalar(0.1);
                Scalar far_plane = Scalar(500);
                std::uint32_t display_index = 0;
                std::int32_t priority = 0;
                bool active = true;
            };

            constexpr Scalar DT = Scalar(1.0 / 60.0);
            constexpr std::size_t SEED_CUBE_COUNT = 24;
            constexpr std::size_t CHUNK_CAPACITY = 256;

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
                          schedule_(runtime_)
                    {
                        // Reserve both archetypes up front so neither the seed nor the
                        // editor's first create allocates a chunk mid-run.
                        world_.reserve<Transform, Orientation, SpinStep, OrbitState, Tint>(
                            CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation, Tint>(CHUNK_CAPACITY);
                        world_.reserve<Transform, Orientation, Camera>(CHUNK_CAPACITY);
                        seed_world();
                        register_systems();
                        extract(); // a valid snapshot before the first tick
                    }

                    // --- ISimulation -------------------------------------------------

                    void tick() override
                    {
                        schedule_.run(world_);
                        extract();
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

                    Vec3 color(EntityId id) const override
                    {
                        const Record* record = find(id);
                        if (record == nullptr || record->is_camera ||
                            !world_.alive(record->entity))
                            return Vec3{};
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
                                                           Tint{Vec3{Scalar(0.8), Scalar(0.8),
                                                                     Scalar(0.8)}});
                        const EntityId id = next_id_++;
                        order_.push_back(id);
                        records_.emplace(id, Record{entity, display_name, true, false});
                        extract();
                        return id;
                    }

                    void destroy(EntityId id) override
                    {
                        const auto it = records_.find(id);
                        if (it == records_.end())
                            return;
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

                    void set_color(EntityId id, const Vec3& value) override
                    {
                        Record* record = find(id);
                        if (record == nullptr || record->is_camera ||
                            !world_.alive(record->entity))
                            return;
                        world_.get<Tint>(record->entity).color = value;
                        extract();
                    }

                    EntityId create_camera(const std::string& display_name) override
                    {
                        // A default camera looking down -Z from a few units back, so the
                        // seeded scene is visible without any rotation authoring yet.
                        Transform transform;
                        transform.position = Vec3{0, Scalar(3), Scalar(12)};
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

                private:
                    /** @brief The editor metadata paired with each entity's ECS handle. */
                    struct Record
                    {
                        Entity entity;
                        std::string name;
                        bool visible = true;
                        bool animated = false;
                        bool is_camera = false;
                    };

                    const Record* find(EntityId id) const noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    Record* find(EntityId id) noexcept
                    {
                        const auto it = records_.find(id);
                        return it != records_.end() ? &it->second : nullptr;
                    }

                    /**
                     * @brief Places the demo cubes on a ring and precomputes their motion.
                     *
                     * Each cube gets a per-step spin quaternion and a per-step orbit
                     * rotation (as a cos/sin pair), both computed here on the host so
                     * the kernels never call a transcendental or capture host state.
                     */
                    void seed_world()
                    {
                        const Vec3 spin_axis = normalize(Vec3{0, 1, 0});
                        for (std::size_t i = 0; i < SEED_CUBE_COUNT; ++i)
                        {
                            const Scalar t = Scalar(i) / Scalar(SEED_CUBE_COUNT);
                            const Scalar ring_angle = t * Scalar(6.2831853);
                            const Scalar radius = Scalar(4.5);

                            OrbitState orbit;
                            orbit.center = Vec3{0, Scalar(0.75), 0};
                            orbit.radius = radius;
                            orbit.cos_angle = std::cos(ring_angle);
                            orbit.sin_angle = std::sin(ring_angle);
                            const Scalar orbit_speed = Scalar(0.4) + t * Scalar(0.6);
                            orbit.step_cos = std::cos(orbit_speed * DT);
                            orbit.step_sin = std::sin(orbit_speed * DT);

                            Transform transform;
                            transform.position = Vec3{orbit.center.x + radius * orbit.cos_angle,
                                                      orbit.center.y,
                                                      orbit.center.z + radius * orbit.sin_angle};
                            transform.scale = Vec3{Scalar(0.6), Scalar(0.6), Scalar(0.6)};

                            const Scalar spin_speed = Scalar(1.5) + t * Scalar(2.5);
                            SpinStep spin{quat_axis_angle(spin_axis, spin_speed * DT)};

                            Tint tint{hue(t)};

                            const Entity entity =
                                world_.spawn(transform, Orientation{}, spin, orbit, tint);
                            const EntityId id = next_id_++;
                            order_.push_back(id);
                            char label[16];
                            std::snprintf(label, sizeof(label), "Cube %02zu", i);
                            records_.emplace(id, Record{entity, label, true, true});
                        }

                        // One camera entity drives the Game view. It is a real entity in
                        // the hierarchy (posed by its transform), not a hidden field.
                        create_camera("Main Camera");
                    }

                    /** @brief The camera used when no active camera exists, so the Game view is never black. */
                    static CameraState default_camera() noexcept
                    {
                        CameraState state;
                        state.position = Vec3{0, Scalar(7), Scalar(12)};
                        state.target = Vec3{0, Scalar(0.75), 0};
                        state.up = Vec3{0, 1, 0};
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
                        const Mat4 rotation = mat4_from_quat(orientation.rotation);
                        // Column-major basis: right = col0, up = col1, +Z = col2. A camera
                        // looks down its local -Z, so forward is the negated third column.
                        const Vec3 forward{-rotation.m[8], -rotation.m[9], -rotation.m[10]};
                        const Vec3 up{rotation.m[4], rotation.m[5], rotation.m[6]};
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

                            if (!record->visible)
                                continue;
                            const Tint& tint = world_.get<Tint>(record->entity);
                            RenderInstance instance;
                            instance.id = id;
                            instance.model = compose_transform(transform.position,
                                                               orientation.rotation, transform.scale);
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
                        scene_.camera = scene_.display_cameras.empty()
                                            ? default_camera()
                                            : scene_.display_cameras.front().state;
                    }

                    /** @brief A pleasant colour ramp over t in [0, 1); avoids muddy greys. */
                    static Vec3 hue(Scalar t)
                    {
                        const Scalar a = t * Scalar(6.2831853);
                        return Vec3{Scalar(0.5) + Scalar(0.45) * std::cos(a),
                                    Scalar(0.5) + Scalar(0.45) * std::cos(a + Scalar(2.094)),
                                    Scalar(0.5) + Scalar(0.45) * std::cos(a + Scalar(4.188))};
                    }

                    SushiRuntime::API::Runtime runtime_;
                    World world_;
                    Schedule schedule_;
                    std::vector<EntityId> order_;
                    std::unordered_map<EntityId, Record> records_;
                    EntityId next_id_ = 1;
                    RenderScene scene_;
            };
        } // namespace

        std::unique_ptr<ISimulation> create_simulation()
        {
            return std::unique_ptr<ISimulation>(new RuntimeSimulation());
        }
    } // namespace sim
} // namespace SushiEngine
