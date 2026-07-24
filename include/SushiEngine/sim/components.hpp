/**************************************************************************/
/* components.hpp                                                        */
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
 * @file components.hpp
 * @brief SushiEngine's built-in ECS component set, in one place.
 *
 * The single home for every component the editor's world knows about, so
 * `RuntimeSimulation` and any future consumer register the same types in the
 * same order (component registration order across translation units must
 * agree — see `component_id<T>()`). Every type here is trivially copyable, as
 * `component_id<T>()` enforces. Transform + Orientation together are the
 * mandatory pose every entity carries; the rest (Tint, Camera, SpinStep,
 * OrbitState) are optional and attached or detached per entity, which is what
 * makes an entity's capabilities — "has a renderer", "is a camera" —
 * pluggable rather than fixed at creation.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/vfx/asset_id.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief World position and scale. Mandatory on every entity, with Orientation. */
        struct Transform
        {
            Vector3 position;
            Vector3 scale{Vector3{1, 1, 1}};
        };

        /**
         * @brief World orientation, split from Transform.
         *
         * Kept as its own column so systems that only spin (write Orientation) and
         * systems that only orbit (write Transform) touch disjoint components and
         * the dependency tracker can run them in parallel.
         */
        struct Orientation
        {
            Quaternion rotation;
        };

        /** @brief A precomputed per-step spin delta; present only on animated entities. */
        struct SpinStep
        {
            Quaternion delta;
        };

        /** @brief Precomputed per-step orbit motion; present only on animated entities. */
        struct OrbitState
        {
            Vector3 center;
            Scalar radius = 0;
            Scalar cos_angle = 1;
            Scalar sin_angle = 0;
            Scalar step_cos = 1;
            Scalar step_sin = 0;
        };

        /**
         * @brief The "Renderer" component: a base colour, drawn as a solid cube.
         *
         * Present only on entities with a renderer attached (`IWorldEditor::has_renderer`
         * / `set_has_renderer`) — the Unity-equivalent of a MeshRenderer, minus the mesh
         * (the sandbox draws every renderer as a cube today).
         */
        struct Tint
        {
            Vector3 color;
        };

        /**
         * @brief The "Camera" component: lens and display routing.
         *
         * Present only on entities with a camera attached (`IWorldEditor::is_camera` /
         * `set_is_camera`). Its pose comes from Transform + Orientation like any other
         * entity; this adds the projection and which display it drives.
         */
        struct Camera
        {
            Scalar vertical_fov_radians = Scalar(1.0471976);
            Scalar near_plane = Scalar(0.1);
            Scalar far_plane = Scalar(500);
            std::uint32_t display_index = 0;
            std::int32_t priority = 0;
            bool active = true;
        };

        /** @brief @ref ParticleEmitter::flags bit: the emitter is actively simulating. */
        constexpr std::uint32_t PARTICLE_EMITTER_PLAYING = 1u << 0;

        /**
         * @brief The "Particle Emitter" component: an entity that plays a VFX effect.
         *
         * Present only on entities with an emitter attached. It stores just a handle into a
         * `Vfx::EffectDatabase` plus the small runtime state a play head needs — the heavy
         * authored data (modules, curves, gradients) lives in the effect asset, and the live
         * particles live in a backend pool (GPU-side for cosmetic emitters, a deterministic
         * CPU pool for gameplay ones). Every field is an id, a scalar, or a flag, so the
         * component is trivially copyable and byte-snapshottable like the rest. The emitter's
         * pose comes from Transform + Orientation, the same columns the renderer reads; each
         * emitter within the referenced effect chooses its own simulation domain.
         */
        struct ParticleEmitter
        {
            Vfx::AssetId effect = Vfx::INVALID_EFFECT; /**< The effect asset to play. */
            std::uint32_t seed = 0;                    /**< Per-instance RNG seed (deterministic path). */
            float time = 0.0f;                         /**< Seconds since the emitter started playing. */
            float spawn_accumulator = 0.0f;            /**< Fractional continuous-spawn carry. */
            std::uint32_t flags = PARTICLE_EMITTER_PLAYING; /**< @ref PARTICLE_EMITTER_PLAYING and future bits. */
        };

        /**
         * @brief Which primitive mesh a Shape or Collider is expressed as.
         *
         * Shared by the editor-facing `ShapeParams`/`ColliderParams` (see
         * simulation.hpp) rather than by an ECS component: neither a visual shape
         * nor a collider is read or written by any Schedule system today, so both
         * are plain host-side bookkeeping on `RuntimeSimulation::Record`, exactly
         * like `PhysicsBodyParams`/`ClothParams` — no ECS archetype migration needed.
         */
        enum class PrimitiveKind : std::uint32_t
        {
            Box,
            Sphere,
            Cylinder,
            Plane,
        };
    } // namespace Simulation
} // namespace SushiEngine
