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
// SushiRuntime, an ECS World, and a Schedule, and drives a live world of spinning,
// orbiting cubes. Two systems demonstrate the runtime's dependency tracker — "spin"
// advances each cube's orientation, "orbit" advances its position — over disjoint
// components, so they run in parallel exactly as the sandbox proves. Every value a
// kernel touches is precomputed on the host into a component (the per-step rotation
// quaternion, the per-step orbit rotation as a cos/sin pair), so the kernels are
// pure arithmetic and capture no host state, which is what makes them legal device
// code. After each step an extract pass reads the columns back on the host and
// composes model matrices into the RenderScene the editor draws.

#include <cmath>
#include <cstddef>
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

            constexpr Scalar DT = Scalar(1.0 / 60.0);
            constexpr std::size_t CUBE_COUNT = 24;
            constexpr std::size_t CHUNK_CAPACITY = 256;

            /**
             * @brief The runtime-backed live world behind the ISimulation seam.
             *
             * Constructs the runtime, world, and schedule; seeds the cubes; registers
             * the two systems; and on each tick runs the schedule and extracts a fresh
             * RenderScene. The extract is a host read of the shared-USM columns via
             * `World::get`, composed into model matrices — the simple, correct path
             * before device-shared interop lands.
             */
            class RuntimeSimulation final : public ISimulation
            {
                public:
                    RuntimeSimulation()
                        : runtime_(SushiRuntime::API::Runtime::create()),
                          world_(runtime_, CHUNK_CAPACITY),
                          schedule_(runtime_)
                    {
                        world_.reserve<Transform, Orientation, SpinStep, OrbitState, Tint>(
                            CHUNK_CAPACITY);
                        seed_world();
                        register_systems();
                        extract(); // a valid snapshot before the first tick
                    }

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
                        return entities_.size();
                    }

                private:
                    /**
                     * @brief Places the cubes on a ring and precomputes their motion.
                     *
                     * Each cube gets a per-step spin quaternion and a per-step orbit
                     * rotation (as a cos/sin pair), both computed here on the host so
                     * the kernels never call a transcendental or capture host state.
                     */
                    void seed_world()
                    {
                        const Vec3 spin_axis = normalize(Vec3{0, 1, 0});
                        for (std::size_t i = 0; i < CUBE_COUNT; ++i)
                        {
                            const Scalar t = Scalar(i) / Scalar(CUBE_COUNT);
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

                            const Entity e = world_.spawn(transform, Orientation{}, spin, orbit, tint);
                            entities_.push_back(e);
                        }

                        scene_.camera.position = Vec3{0, Scalar(7), Scalar(12)};
                        scene_.camera.target = Vec3{0, Scalar(0.75), 0};
                        scene_.camera.up = Vec3{0, 1, 0};
                        scene_.camera.vertical_fov_radians = Scalar(1.0471976);
                        scene_.camera.near_plane = Scalar(0.1);
                        scene_.camera.far_plane = Scalar(500);
                    }

                    /**
                     * @brief Registers the two per-cube systems.
                     *
                     * "spin" writes Orientation from the precomputed SpinStep; "orbit"
                     * writes Transform and advances OrbitState. Their write sets are
                     * disjoint, so the dependency tracker runs them concurrently.
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
                     * composed into per-instance model matrices; the camera is static.
                     */
                    void extract()
                    {
                        scene_.instances.clear();
                        scene_.instances.reserve(entities_.size());
                        for (const Entity e : entities_)
                        {
                            if (!world_.alive(e))
                                continue;
                            const Transform& transform = world_.get<Transform>(e);
                            const Orientation& orientation = world_.get<Orientation>(e);
                            const Tint& tint = world_.get<Tint>(e);
                            RenderInstance instance;
                            instance.model = compose_transform(transform.position,
                                                               orientation.rotation, transform.scale);
                            instance.color = tint.color;
                            scene_.instances.push_back(instance);
                        }
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
                    std::vector<Entity> entities_;
                    RenderScene scene_;
            };
        } // namespace

        std::unique_ptr<ISimulation> create_simulation()
        {
            return std::unique_ptr<ISimulation>(new RuntimeSimulation());
        }
    } // namespace sim
} // namespace SushiEngine
