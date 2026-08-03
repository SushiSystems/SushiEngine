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

#include <SushiEngine/audio/reverb_params.hpp>
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
         * `VFX::EffectDatabase` plus the small runtime state a play head needs — the heavy
         * authored data (modules, curves, gradients) lives in the effect asset, and the live
         * particles live in a backend pool (GPU-side for cosmetic emitters, a deterministic
         * CPU pool for gameplay ones). Every field is an id, a scalar, or a flag, so the
         * component is trivially copyable and byte-snapshottable like the rest. The emitter's
         * pose comes from Transform + Orientation, the same columns the renderer reads; each
         * emitter within the referenced effect chooses its own simulation domain.
         */
        struct ParticleEmitter
        {
            VFX::AssetId effect = VFX::INVALID_EFFECT; /**< The effect asset to play. */
            std::uint32_t seed = 0;                    /**< Per-instance RNG seed (deterministic path). */
            float time = 0.0f;                         /**< Seconds since the emitter started playing. */
            float spawn_accumulator = 0.0f;            /**< Fractional continuous-spawn carry. */
            std::uint32_t flags = PARTICLE_EMITTER_PLAYING; /**< @ref PARTICLE_EMITTER_PLAYING and future bits. */
        };

        // --- Audio (Phase S6) ------------------------------------------------------
        //
        // Consumed by the **wall-clock audio snapshot extract** (`sim/audio_extract.hpp`
        // → `Audio::AudioScene` → the voice manager), like the render extract reads
        // Transform/Tint. No fixed-step Schedule system reads or writes them, and the
        // extract only *reads* the world, so a deterministic run is byte-identical with
        // audio on or off (`docs/slop/audio_system.md` §0, §9). Pose comes from
        // Transform (+ Orientation for the listener's facing), the same columns the
        // renderer reads — an emitter is just an entity that also makes sound.

        /**
         * @brief The "Audio Listener" component: this entity's pose is the ears.
         *
         * Present on the entity the mix is heard from (typically the active camera).
         * Its position and facing come from Transform + Orientation; this adds only the
         * master gain and an active flag. If several exist, the extract takes the first
         * active one.
         */
        struct AudioListener
        {
            float gain = 1.0f;  /**< Master linear gain for the whole mix at this listener. */
            bool active = true; /**< Only an active listener is chosen as the ears. */
        };

        /** @brief @ref AudioEmitter::flags bit: the emitter is actively sounding. */
        constexpr std::uint32_t AUDIO_EMITTER_PLAYING = 1u << 0;
        /** @brief @ref AudioEmitter::flags bit: distance/Doppler apply (else a 2D sound). */
        constexpr std::uint32_t AUDIO_EMITTER_SPATIAL = 1u << 1;
        /** @brief @ref AudioEmitter::flags bit: the emitter is occluded by acoustic geometry. */
        constexpr std::uint32_t AUDIO_EMITTER_OCCLUDED = 1u << 2;

        /**
         * @brief The "Audio Emitter" component: an entity that plays a sound.
         *
         * Stores the routing and attenuation a designer authors plus a @ref sound id the
         * host resolves to a voice source (a tone/sample today; a bank **event** id at
         * S8). Its world position comes from Transform, the same column the renderer
         * reads; the extract feeds that position to the voice manager each wall-clock
         * frame, and the frame-to-frame change is what drives Doppler. Every field is an
         * id, a scalar, or a flag, so the component is trivially copyable like the rest.
         * The fields mirror @ref Audio::VoiceDescriptor so the extract maps them 1:1.
         */
        struct AudioEmitter
        {
            std::uint32_t sound = 0;      /**< Sound/event id the host factory resolves to a source. */
            float gain = 1.0f;           /**< Linear base gain before attenuation. */
            float priority = 0.0f;       /**< Voice-manager real-slot priority (higher wins). */
            std::uint32_t bus = 0;       /**< Target mixer bus id. */
            float min_distance = 1.0f;   /**< Full gain within this radius (metres). */
            float max_distance = 100.0f; /**< Silent and cullable beyond this radius (metres). */
            std::uint32_t distance_model = 0; /**< 0 Linear, 1 Inverse, 2 Exponent (@ref Audio::DistanceModel). */
            float rolloff = 1.0f;        /**< Rolloff factor for the inverse/exponent models. */
            float doppler_scale = 1.0f;  /**< Doppler exaggeration (0 off, 1 physical, >1 more). */
            std::int32_t reverb_bus = -1; /**< Reverb aux-send target bus (−1 = no send). */
            float reverb_send = 0.0f;    /**< Reverb aux-send level in [0, 1]. */
            float source_radius = 0.5f;  /**< Sphere radius for soft-occlusion ray sampling (metres). */
            std::uint32_t flags = AUDIO_EMITTER_PLAYING | AUDIO_EMITTER_SPATIAL;
        };

        /**
         * @brief The "Room" component: an acoustic room volume for portal propagation.
         *
         * A box centred on the entity's Transform position with @ref half_extents that the
         * audio extract reads (with any @ref Portal entities) to build the room/portal graph
         * (`audio/portals.hpp`). A source in a different room from the listener is heard
         * through the doorways joining them, not through the wall. The @ref id is the handle
         * @ref Portal entities reference. Trivially copyable, like every component here.
         */
        struct Room
        {
            std::uint32_t id = 0;                     /**< Room handle referenced by @ref Portal. */
            Vector3 half_extents{Vector3{10, 10, 10}}; /**< Box half-size around the Transform. */
        };

        /**
         * @brief The "Portal" component: a doorway joining two @ref Room volumes.
         *
         * Centred on the entity's Transform position (the opening the sound passes through),
         * joining rooms @ref room_a and @ref room_b. When the listener and a source are in
         * the two rooms, this opening becomes a secondary virtual source at the doorway.
         * Trivially copyable.
         */
        struct Portal
        {
            std::uint32_t room_a = 0;                  /**< One joined room's @ref Room::id. */
            std::uint32_t room_b = 0;                  /**< The other joined room's @ref Room::id. */
            Vector3 half_extents{Vector3{1, 1, 1}};    /**< Opening half-size around the Transform. */
        };

        /**
         * @brief The "Reverb Zone" component: a world box that imposes its reverb.
         *
         * A box centred on the entity's Transform position with @ref half_extents; when
         * the listener is inside, its @ref reverb (the I3DL2 set) drives the reverb aux
         * bus. Overlapping zones are resolved by @ref priority (higher wins) — the
         * distance-weighted blending across zone boundaries (§7) is a later refinement.
         * The I3DL2 parameters live inline (a trivially-copyable POD, `reverb_params.hpp`)
         * so the component needs no reverb *engine*.
         */
        struct ReverbZone
        {
            Vector3 half_extents{Vector3{10, 10, 10}}; /**< Box half-size around the Transform. */
            Audio::I3DL2Reverb reverb;                 /**< The environment's I3DL2 reverb. */
            float send = 1.0f;                         /**< Aux-send scale for emitters in the zone. */
            std::int32_t priority = 0;                 /**< Overlapping zones: higher wins. */
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
