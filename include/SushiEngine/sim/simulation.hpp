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
#include <memory>
#include <vector>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace sim
    {
        /** @brief One drawable object extracted from the world: transform and colour. */
        struct RenderInstance
        {
            Mat4 model; /**< Object-to-world transform composed from the entity's state. */
            Vec3 color; /**< Base colour. */
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
         * @brief A read-only snapshot of the world for one frame.
         *
         * Rebuilt by the simulation's extract step after every `tick()`; the host
         * reads it but never mutates the world through it. A host copy today — the
         * zero-copy path that shares the world's device columns with the renderer is
         * a later interop milestone.
         */
        struct RenderScene
        {
            std::vector<RenderInstance> instances; /**< Every drawable object this frame. */
            CameraState camera;                    /**< The world's game camera. */
        };

        /**
         * @brief A live world a host ticks and draws without seeing the runtime.
         *
         * Owns the ECS world, its schedule, and the SushiRuntime that executes it.
         * `tick()` advances the world one fixed step; `render_scene()` returns the
         * snapshot extracted after the most recent tick.
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
        };

        /**
         * @brief Creates the runtime-backed live world.
         *
         * Brings up a SushiRuntime, seeds a small world of spinning, orbiting cubes
         * and a game camera, and returns it behind the abstraction. The only place
         * the runtime is constructed for the editor.
         *
         * @return An owned simulation; never null (throws on runtime bring-up failure).
         */
        std::unique_ptr<ISimulation> create_simulation();
    } // namespace sim
} // namespace SushiEngine
