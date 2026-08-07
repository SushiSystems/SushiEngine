/**************************************************************************/
/* audio_extract.hpp                                                      */
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
 * `docs/design/audio_system.md`). It is the audio sibling of the render `extract()`:
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

#include <SushiEngine/audio/acoustic_geometry.hpp>
#include <SushiEngine/audio/audio_scene.hpp>
#include <SushiEngine/audio/portals.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/simulation/components.hpp>

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

            /** @brief The listener-local key marker bit for an injected doorway virtual emitter. */
            constexpr std::uint64_t DOORWAY_KEY_BIT = 1ull << 63;
        } // namespace Detail

        /** @brief Tuning for the optional acoustic occlusion/portal pass in the extract. */
        struct AcousticQueryConfiguration
        {
            int ray_count = 8;             /**< Soft-occlusion rays per emitter (sphere sampling). */
            int max_surfaces = 4;          /**< Transmission surfaces folded per blocked ray. */
            int max_doorways = 2;          /**< Doorway virtual sources spawned per cross-room emitter. */
            float reference_distance = 3.0f; /**< Distance at which a doorway is at unit gain. */
        };

        /**
         * @brief Builds a room/portal graph from the world's @ref Room and @ref Portal
         *        components (listener-local, so it matches the snapshot frame).
         *
         * A read-only host walk, run when the topology changes (rooms are static). Room
         * boxes are placed relative to @p eye so the graph shares the snapshot's frame.
         *
         * @param world The ECS world to read.
         * @param eye   The listener world position the local frame is centred on.
         * @param graph The graph to fill (cleared first).
         */
        inline void build_portal_graph(World& world, const WorldVector3& eye, Audio::PortalGraph& graph)
        {
            graph.clear();
            const Signature room_sig = make_signature<Transform, Room>();
            for (Archetype* a : world.query(room_sig))
            {
                for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                {
                    for (std::size_t row = 0; row < chunk->count(); ++row)
                    {
                        const Entity e = chunk->entity_at(row);
                        const Room& r = world.get<Room>(e);
                        const Audio::AudioVec3 c = Detail::to_local(world.get<Transform>(e).position, eye);
                        Audio::AcousticAABB box;
                        box.min = Audio::AudioVec3{c.x - static_cast<float>(r.half_extents.x),
                                                   c.y - static_cast<float>(r.half_extents.y),
                                                   c.z - static_cast<float>(r.half_extents.z)};
                        box.max = Audio::AudioVec3{c.x + static_cast<float>(r.half_extents.x),
                                                   c.y + static_cast<float>(r.half_extents.y),
                                                   c.z + static_cast<float>(r.half_extents.z)};
                        graph.add_room(r.id, box);
                    }
                }
            }
            const Signature portal_sig = make_signature<Transform, Portal>();
            for (Archetype* a : world.query(portal_sig))
            {
                for (const std::unique_ptr<Chunk>& chunk : a->chunks())
                {
                    for (std::size_t row = 0; row < chunk->count(); ++row)
                    {
                        const Entity e = chunk->entity_at(row);
                        const Portal& p = world.get<Portal>(e);
                        const Audio::AudioVec3 c = Detail::to_local(world.get<Transform>(e).position, eye);
                        graph.add_portal(p.room_a, p.room_b, c,
                                         Audio::AudioVec3{static_cast<float>(p.half_extents.x),
                                                          static_cast<float>(p.half_extents.y),
                                                          static_cast<float>(p.half_extents.z)});
                    }
                }
            }
            graph.build();
        }

        /**
         * @brief Builds the audio snapshot from the world's audio components.
         *
         * Reads the first active @ref AudioListener as the ears, every
         * @ref AudioEmitter as an audible source (positioned relative to the listener),
         * and the highest-priority @ref ReverbZone that contains the listener as the
         * active environment. The world is only read, never written.
         *
         * When an @p acoustics scene (and optionally a @p portals graph) is supplied, every
         * @ref AudioEmitter flagged @ref AUDIO_EMITTER_OCCLUDED is soft-occlusion tested
         * against the geometry (in the listener-local frame the snapshot uses), and a
         * cross-room source is muted and replaced by doorway virtual emitters through the
         * portal graph. Passing null for both leaves the run exactly as before (and the
         * whole walk is read-only either way, so a deterministic run stays byte-identical).
         *
         * @param world         The ECS world to read.
         * @param out           The snapshot to fill (cleared first).
         * @param acoustics     Optional acoustic BVH (listener-local frame) for occlusion.
         * @param portals       Optional room/portal graph (listener-local) for doorway sources.
         * @param configuration Occlusion/portal query tuning.
         */
        inline void build_audio_snapshot(World& world, Audio::SceneSnapshot& out,
                                         const Audio::AcousticScene* acoustics = nullptr,
                                         const Audio::PortalGraph* portals = nullptr,
                                         const AcousticQueryConfiguration& configuration =
                                             AcousticQueryConfiguration{})
        {
            out.emitters.clear();
            out.has_reverb = false;

            // The listener: pose from Transform + Orientation (mandatory pose).
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

            // The emitters: pose from Transform, relative to the listener.
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
                        es.trigger = em.trigger;
                        es.min_distance = em.min_distance;
                        es.max_distance = em.max_distance;
                        es.model = Detail::distance_model(em.distance_model);
                        es.rolloff = em.rolloff;
                        es.doppler_scale = em.doppler_scale;
                        es.reverb_bus = em.reverb_bus;
                        es.reverb_send = em.reverb_send;

                        // Occlusion / portal propagation (listener at the local origin).
                        const bool occluded_flag = (em.flags & AUDIO_EMITTER_OCCLUDED) != 0;
                        Audio::OcclusionResult occ;
                        if (acoustics != nullptr && occluded_flag && es.spatial)
                        {
                            occ = acoustics->soft_occlusion(
                                es.position, Audio::AudioVec3{0, 0, 0}, em.source_radius,
                                configuration.ray_count, configuration.max_surfaces);
                            for (int b = 0; b < 3; ++b)
                                es.transmission[b] = occ.transmission[b];
                        }

                        if (portals != nullptr && es.spatial)
                        {
                            const Audio::PortalResolution pr = portals->resolve(
                                Audio::AudioVec3{0, 0, 0}, es.position,
                                configuration.reference_distance, configuration.max_doorways);
                            if (pr.same_room)
                            {
                                es.obstruction = occ.fraction;
                                es.occlusion = 0.0f;
                                out.emitters.push_back(es);
                            }
                            else if (pr.source_reachable && !pr.doorways.empty())
                            {
                                // The wall occludes the direct path; the sound arrives through
                                // the doorways as secondary virtual sources.
                                std::uint64_t door = 0;
                                for (const Audio::PortalSource& ps : pr.doorways)
                                {
                                    Audio::EmitterSnapshot d = es;
                                    d.key = Detail::DOORWAY_KEY_BIT | (door << 48) |
                                            (es.key & 0xffffffffffffull);
                                    d.position = ps.position;
                                    d.gain = es.gain * ps.gain;
                                    d.priority = es.priority - 0.001f;
                                    d.obstruction = 0.0f;
                                    d.occlusion = 0.0f;
                                    d.transmission[0] = d.transmission[1] = d.transmission[2] = 1.0f;
                                    out.emitters.push_back(d);
                                    ++door;
                                }
                            }
                            else
                            {
                                // Isolated room, no open path: only the through-wall leak.
                                es.obstruction = 0.0f;
                                es.occlusion = occ.fraction;
                                out.emitters.push_back(es);
                            }
                        }
                        else
                        {
                            es.obstruction = 0.0f;
                            es.occlusion = occ.fraction;
                            out.emitters.push_back(es);
                        }
                    }
                }
            }

            // The reverb zone: highest-priority box containing the listener.
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
         * @param world         The ECS world to read.
         * @param scene         The audio scene bridge to drive.
         * @param scratch       A caller-owned snapshot reused across frames.
         * @param acoustics     Optional acoustic BVH (listener-local) for occlusion.
         * @param portals       Optional room/portal graph (listener-local) for doorway sources.
         * @param configuration Occlusion/portal query tuning.
         */
        inline void extract_audio_scene(World& world, Audio::AudioScene& scene,
                                        Audio::SceneSnapshot& scratch,
                                        const Audio::AcousticScene* acoustics = nullptr,
                                        const Audio::PortalGraph* portals = nullptr,
                                        const AcousticQueryConfiguration& configuration =
                                            AcousticQueryConfiguration{})
        {
            build_audio_snapshot(world, scratch, acoustics, portals, configuration);
            scene.apply(scratch);
        }
    } // namespace Simulation
} // namespace SushiEngine
