/**************************************************************************/
/* audio_extract.hpp                                                     */
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
 * @file audio_extract.hpp
 * @brief The wall-clock audio snapshot extract — the ECS half of the audio bridge.
 *
 * This is the sim-layer glue that turns the ECS world into the plain-float
 * @ref Audio::SceneSnapshot the @ref Audio::AudioScene reconciles (§9 of
 * `docs/slop/audio_system.md`). It is the audio sibling of the render `extract()`:
 * a **read-only** host walk of the component columns, run once per **wall-clock**
 * frame (outside the fixed-step schedule, like the VFX preview), so it can never
 * perturb a deterministic run — audio on or off is byte-identical.
 *
 * It does the two things the audio module cannot (it lives above the SushiRuntime, so
 * it may): it converts absolute double `WorldVector3` positions into **listener-local
 * float** via the renderer's eye-subtracted-in-double-then-cast idiom (so planet-scale
 * coordinates keep their precision), and it reads the listener's Orientation quaternion
 * into a facing frame. The world uses the glTF convention (local −Z forward, +Y up); the
 * result is handed to the voice manager in that world frame, which is all
 * `head_relative_direction` needs — it builds its head basis from whatever forward/up it
 * is given.
 */

#include <climits>
#include <cstdint>

#include <SushiEngine/audio/audio_scene.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/sim/components.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        namespace Detail
        {
            /** @brief Packs an entity handle into a stable 64-bit emitter key. */
            inline std::uint64_t emitter_key(Entity e) noexcept
            {
                return (static_cast<std::uint64_t>(e.generation) << 32) |
                       static_cast<std::uint64_t>(e.index);
            }

            /** @brief Maps the component's integer distance-model code to the audio enum. */
            inline Audio::DistanceModel distance_model(std::uint32_t code) noexcept
            {
                switch (code)
                {
                    case 1: return Audio::DistanceModel::Inverse;
                    case 2: return Audio::DistanceModel::Exponent;
                    default: return Audio::DistanceModel::Linear;
                }
            }

            /** @brief Listener-local float position: subtract the eye in double, then cast. */
            inline Audio::AudioVec3 to_local(const Vector3& world, const WorldVector3& eye) noexcept
            {
                return Audio::AudioVec3{static_cast<float>(world.x - eye.x),
                                       static_cast<float>(world.y - eye.y),
                                       static_cast<float>(world.z - eye.z)};
            }
        } // namespace Detail

        /**
         * @brief Builds the audio snapshot from the world's audio components.
         *
         * Reads the first active @ref AudioListener as the ears, every
         * @ref AudioEmitter as an audible source (positioned relative to the listener),
         * and the highest-priority @ref ReverbZone that contains the listener as the
         * active environment. The world is only read, never written.
         *
         * @param world The ECS world to read.
         * @param out   The snapshot to fill (cleared first).
         */
        inline void build_audio_snapshot(World& world, Audio::SceneSnapshot& out)
        {
            out.emitters.clear();
            out.has_reverb = false;

            // --- The listener: pose from Transform + Orientation (mandatory pose). ---
            WorldVector3 eye{0.0, 0.0, 0.0};
            bool have_listener = false;
            const Signature listener_sig = make_signature<Transform, Orientation, AudioListener>();
            for (Archetype* a : world.query(listener_sig))
            {
                for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                {
                    for (std::size_t row = 0; row < chunk->count(); ++row)
                    {
                        const Entity e = chunk->entity_at(row);
                        if (!world.get<AudioListener>(e).active)
                            continue;
                        const Transform& t = world.get<Transform>(e);
                        const Quaternion q = world.get<Orientation>(e).rotation;
                        eye = WorldVector3{t.position.x, t.position.y, t.position.z};
                        // glTF frame: local −Z is forward, +Y is up.
                        const Vector3 fwd = rotate(q, Vector3{0, 0, -1});
                        const Vector3 up = rotate(q, Vector3{0, 1, 0});
                        out.listener_forward =
                            Audio::AudioVec3{static_cast<float>(fwd.x), static_cast<float>(fwd.y),
                                             static_cast<float>(fwd.z)};
                        out.listener_up =
                            Audio::AudioVec3{static_cast<float>(up.x), static_cast<float>(up.y),
                                             static_cast<float>(up.z)};
                        have_listener = true;
                        break;
                    }
                    if (have_listener)
                        break;
                }
                if (have_listener)
                    break;
            }

            // --- The emitters: pose from Transform, relative to the listener. ---
            const Signature emitter_sig = make_signature<Transform, AudioEmitter>();
            for (Archetype* a : world.query(emitter_sig))
            {
                for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                {
                    for (std::size_t row = 0; row < chunk->count(); ++row)
                    {
                        const Entity e = chunk->entity_at(row);
                        const AudioEmitter& em = world.get<AudioEmitter>(e);
                        const Transform& t = world.get<Transform>(e);

                        Audio::EmitterSnapshot es;
                        es.key = Detail::emitter_key(e);
                        es.sound = em.sound;
                        es.position = Detail::to_local(t.position, eye);
                        es.gain = em.gain;
                        es.priority = em.priority;
                        es.bus = static_cast<int>(em.bus);
                        es.spatial = (em.flags & AUDIO_EMITTER_SPATIAL) != 0;
                        es.playing = (em.flags & AUDIO_EMITTER_PLAYING) != 0;
                        es.min_distance = em.min_distance;
                        es.max_distance = em.max_distance;
                        es.model = Detail::distance_model(em.distance_model);
                        es.rolloff = em.rolloff;
                        es.doppler_scale = em.doppler_scale;
                        out.emitters.push_back(es);
                    }
                }
            }

            // --- The reverb zone: highest-priority box containing the listener. ---
            std::int32_t best_priority = INT_MIN;
            const Signature zone_sig = make_signature<Transform, ReverbZone>();
            for (Archetype* a : world.query(zone_sig))
            {
                for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                {
                    for (std::size_t row = 0; row < chunk->count(); ++row)
                    {
                        const Entity e = chunk->entity_at(row);
                        const ReverbZone& z = world.get<ReverbZone>(e);
                        const Transform& t = world.get<Transform>(e);
                        const double dx = eye.x - t.position.x;
                        const double dy = eye.y - t.position.y;
                        const double dz = eye.z - t.position.z;
                        const bool inside = (dx < 0 ? -dx : dx) <= z.half_extents.x &&
                                            (dy < 0 ? -dy : dy) <= z.half_extents.y &&
                                            (dz < 0 ? -dz : dz) <= z.half_extents.z;
                        if (inside && (!out.has_reverb || z.priority > best_priority))
                        {
                            out.reverb = z.reverb;
                            out.has_reverb = true;
                            best_priority = z.priority;
                        }
                    }
                }
            }
        }

        /**
         * @brief One wall-clock audio step: build the snapshot and reconcile the voices.
         *
         * The single call the host loop makes each frame (alongside the VFX preview),
         * after the world's transforms are up to date. A convenience over
         * @ref build_audio_snapshot + @ref Audio::AudioScene::apply that reuses a
         * caller-owned snapshot to avoid per-frame allocation.
         *
         * @param world    The ECS world to read.
         * @param scene    The audio scene bridge to drive.
         * @param scratch  A caller-owned snapshot reused across frames.
         */
        inline void extract_audio_scene(World& world, Audio::AudioScene& scene,
                                        Audio::SceneSnapshot& scratch)
        {
            build_audio_snapshot(world, scratch);
            scene.apply(scratch);
        }
    } // namespace Simulation
} // namespace SushiEngine
