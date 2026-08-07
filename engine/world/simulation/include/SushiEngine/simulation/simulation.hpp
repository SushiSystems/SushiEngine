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
 * library (`sushiengine_simulation`) so device code stays contained in a single translation
 * unit, and a host depends only on this abstraction (dependency inversion) — a
 * different world backend, or a headless stub, can replace it without the host
 * changing. Each frame the host `tick()`s the world and reads the extracted
 * `RenderScene` snapshot to draw it.
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/environment/environment.hpp>
#include <SushiEngine/environment/light.hpp>
// The statistics value type only, not the physics boundary: physics_services.hpp
// includes this header, so naming it here would close a cycle.
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
// The authored vehicle setup only. `vehicle_asset.hpp` names the corner, the tyre
// and the drivetrain and nothing that solves them, so this costs the vocabulary and
// not the solver - the same trade the soft-body material above already makes.
#include <SushiEngine/physics/vehicle/vehicle_asset.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/simulation/components.hpp>
// The joint vocabulary only, for the same reason and by the same route as the
// statistics above: physics_services.hpp includes this header, so the types a
// joint is authored from live in their own header that both of them include.
#include <SushiEngine/simulation/joint_params.hpp>
#include <SushiEngine/simulation/weather_provider.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /**
         * @brief A stable, editor-facing handle to one world entity.
         *
         * Assigned by the simulation and constant for the entity's lifetime, unlike
         * the ECS's internal generation-checked handle. Zero is the null id (no
         * entity), so a host can use it as an unselected sentinel.
         */
        using EntityId = std::uint64_t;

        /** @brief The null entity id; no entity carries it. */
        constexpr EntityId NULL_ENTITY = 0;

        /** @brief An entity's authorable transform, as the inspector edits it. */
        struct EntityTransform
        {
            Vector3 position;               /**< World position. */
            Quaternion rotation;               /**< Orientation. */
            Vector3 scale{Vector3{1, 1, 1}};   /**< Per-axis scale. */
        };

        /**
         * @brief How an entity's authored transform is interpreted relative to its frame.
         *
         * The mode selected in the inspector's Reference row (see @ref EntityFrame). It
         * governs only the authoring boundary — how @ref IWorldEditor::frame_local_transform
         * expresses the pose and how @ref IWorldEditor::set_frame_local_transform stores it —
         * not the scene-frame `Transform` the physics and renderer work in.
         */
        enum class FrameMode
        {
            Auto,    /**< Resolve Surface vs Free from the entity's altitude over the body. */
            Free,    /**< Inertial: position is a scene-axis offset from the body centre. */
            Surface, /**< Ground-fixed: rotation is ground-local (upright is identity anywhere). */
        };

        /**
         * @brief The frame an entity's transform is authored relative to.
         *
         * The Unity-parent analogue with a celestial body as the parent: the inspector edits
         * a *frame-local* transform (small metres from the body, well inside double's clean
         * range) instead of a raw heliocentric number, and rotation in @ref FrameMode::Surface
         * is ground-local so "upright" is identity at any pole, equator, or hemisphere. A
         * @ref reference_body of -1 means the scene root (the transform is the scene transform,
         * unchanged) — the default, so entities that never pick a body behave exactly as before.
         */
        struct EntityFrame
        {
            int reference_body = -1;          /**< Celestial body index (as `Environment`), or -1 for the scene root. */
            FrameMode mode = FrameMode::Auto; /**< How the frame-local transform is interpreted. */
        };

        /** @brief One drawable object extracted from the world: identity, transform, colour. */
        struct RenderInstance
        {
            EntityId id = NULL_ENTITY; /**< The entity this instance draws, for picking. */
            Matrix4 model;                /**< Object-to-world transform composed from the entity's state. */
            Vector3 color;                /**< Base colour; also drives @ref material.albedo. */
            PrimitiveKind shape_kind = PrimitiveKind::Box; /**< Which mesh to draw this instance with. */
            /** @brief Per-kind shape parameters, see @ref ShapeParameters. */
            Vector3 shape_parameters{Vector3{0.5, 0.5, 0.5}};
            Render::Material material{}; /**< PBR metallic-roughness surface (albedo synced from @ref color). */
            Render::MeshId mesh = Render::INVALID_MESH; /**< Mirrors ShapeParameters::mesh. */
        };

        /**
         * @brief One simulated surface's world-space geometry this frame, ready to draw.
         *
         * The extract channel for anything the host deforms per frame. A grid's rows
         * and columns describe only a cloth sheet; a tetrahedral body's surface is a
         * closed triangle mesh with no grid structure, and a fractured one is not even
         * connected. So this carries a vertex range and a triangle range into the
         * scene's concatenated arrays, and says nothing about how either was arranged.
         */
        struct DeformableInstance
        {
            EntityId id = NULL_ENTITY;      /**< The entity this surface draws, for picking. */
            std::uint32_t first_vertex = 0; /**< Offset into @ref RenderScene::deformable_vertices. */
            std::uint32_t vertex_count = 0; /**< Length of this surface's vertex range. */
            std::uint32_t first_index = 0;  /**< Offset into @ref RenderScene::deformable_indices. */
            std::uint32_t index_count = 0;  /**< Length of this surface's triangle range. */
            /**
             * @brief Bumped when the triangle list changes at unchanged counts.
             *
             * Passed straight through to `Render::DeformableMeshView`, which keys its
             * cached vertex-to-triangle table on it. Fracture, the one thing that
             * changes topology here, always changes the counts too, so this stays zero
             * for every producer the engine has today.
             */
            std::uint64_t topology_revision = 0;
            Vector3 color{Vector3{0.85, 0.85, 0.9}}; /**< Base colour, from the entity's Tint or material. */
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
            Vector3 position;                  /**< Eye position in world space. */
            Vector3 target;                    /**< Point the camera looks at. */
            Vector3 up;                        /**< World up direction. */
            Scalar vertical_fov_radians = 1;/**< Vertical field of view in radians. */
            Scalar near_plane = Scalar(0.1);/**< Near clip distance (> 0). */
            Scalar far_plane = Scalar(500); /**< Far clip distance (> near). */
        };

        /**
         * @brief The authorable parameters of a camera entity.
         *
         * The lens (fov, clip planes) plus how the camera is routed: which display it
         * drives, its priority among cameras on that display, and whether it is active.
         * The eye/target/up frame is not here — it comes from the entity's transform.
         */
        struct CameraParameters
        {
            Scalar vertical_fov_radians = Scalar(1.0471976); /**< Vertical FOV in radians. */
            Scalar near_plane = Scalar(0.1);                 /**< Near clip distance (> 0). */
            Scalar far_plane = Scalar(500);                  /**< Far clip distance (> near). */
            std::uint32_t display_index = 0;                 /**< Which display this camera drives. */
            std::int32_t priority = 0;                       /**< Higher wins on a shared display. */
            bool active = true;                              /**< Whether it contributes at all. */
        };

        /**
         * @brief The authorable parameters of a "Particle Emitter" entity (deterministic path).
         *
         * An emitter entity plays a VFX effect on the CPU-deterministic backend so its particles
         * are gameplay-authoritative and rollback-safe. It stores only the effect handle and the
         * play head; the ~80 KB fixed pool lives host-side on the sim, not in the ECS column, and
         * the effect asset lives in the sim's effect database. The emitter's pose comes from the
         * entity's transform.
         */
        struct ParticleEmitterParameters
        {
            // No effect handle: the effect is the component's own data, not something it points
            // at. See IWorldEditor::particle_effect_source.
            std::uint32_t seed = 1;    /**< Per-instance RNG seed (drives the deterministic stream). */
            bool playing = true;       /**< Whether the emitter is currently emitting. */
        };

        /**
         * @brief The authorable parameters of an "Audio Emitter" entity.
         *
         * An emitter entity plays a sound, positioned by the entity's transform. These
         * mirror the fields the audio engine's `VoiceDescriptor`/`AudioEmitter` carry, in
         * editor-facing form (plain scalars + a distance-model code), so the editor's audio
         * system can spawn and steer a live voice from them. Host bookkeeping on the sim,
         * like light/cloth — no ECS migration.
         */
        struct AudioEmitterParameters
        {
            std::uint32_t sound = 0;      /**< Sound/event id the host factory resolves to a source. */
            float gain = 1.0f;            /**< Linear base gain before attenuation. */
            float priority = 0.0f;        /**< Voice-manager real-slot priority (higher wins). */
            std::uint32_t bus = 0;        /**< Target mixer bus id. */
            float min_distance = 1.0f;    /**< Full gain within this radius (metres). */
            float max_distance = 25.0f;   /**< Silent and cullable beyond this radius (metres). */
            std::uint32_t distance_model = 0; /**< 0 Linear, 1 Inverse, 2 Exponent. */
            float rolloff = 1.0f;         /**< Rolloff factor for inverse/exponent models. */
            float doppler_scale = 1.0f;   /**< Doppler exaggeration (0 off, 1 physical, >1 more). */
            float reverb_send = 0.0f;     /**< Reverb aux-send level in [0, 1]. */
            bool spatial = true;          /**< Whether distance/Doppler apply (else a 2D sound). */
            bool playing = true;          /**< Whether the emitter is currently sounding. */
            bool looping = true;          /**< Whether the source loops (music/ambience vs one-shot). */

            /**
             * @brief Bumped to restart the sound from its beginning.
             *
             * The one-shot pulse. A counter and not a boolean, because the audio scene
             * sees one value per frame: a flag raised and lowered between two of them
             * is an edge nothing observes, while a number that differs from the one
             * last seen is observable however long it takes to be noticed.
             *
             * Deliberately **not** serialized. It is live state, not authoring — a
             * scene reloaded with a saved count would either restart every one-shot in
             * it or silence one, depending on what the count happened to be.
             */
            std::uint32_t trigger = 0;
        };

        /**
         * @brief The authorable parameters of a "Reverb Zone" entity.
         *
         * A box around the entity's transform that imposes an I3DL2 reverb on the listener
         * inside it. The fields mirror the I3DL2 parameter set (levels in dB, times in
         * seconds, diffusion/density/wet in percent) plus the box half-extents and an
         * overlap priority — editor-facing plain scalars, mapped to the audio engine's
         * `I3DL2Reverb` by the editor's audio system.
         */
        struct ReverbZoneParameters
        {
            Vector3 half_extents{Vector3{10, 10, 10}}; /**< Box half-size around the transform. */
            float room = -6.0f;           /**< Overall reverb level (dB, <= 0). */
            float room_hf = -3.0f;        /**< HF reverb level relative to @ref room (dB, <= 0). */
            float decay_time = 1.5f;      /**< Broadband RT60 in seconds (0.1 .. 20). */
            float decay_hf_ratio = 0.5f;  /**< RT60_hf / RT60_dc (0.1 .. 2); < 1 -> darker tail. */
            float reflections = -12.0f;   /**< Early-reflection level (dB). */
            float reverb = 0.0f;          /**< Late-reverb level (dB, <= 0). */
            float diffusion = 100.0f;     /**< Echo-buildup density in percent [0, 100]. */
            float density = 100.0f;       /**< Modal density in percent [0, 100]. */
            float wet_dry_mix = 100.0f;   /**< Wet percent [0, 100] (aux-bus use: 100). */
            float send = 1.0f;            /**< Aux-send scale for emitters in the zone. */
            std::int32_t priority = 0;    /**< Overlapping zones: higher wins. */
        };

        /**
         * @brief The authorable parameters of an "Audio Listener" entity (the ears).
         *
         * Marks an entity as the point the mix is heard from; its pose comes from the
         * transform. Only the master gain and an active flag are authored.
         */
        struct AudioListenerParameters
        {
            float gain = 1.0f;  /**< Master linear gain for the whole mix at this listener. */
            bool active = true; /**< Only an active listener is chosen as the ears. */
        };

        /**
         * @brief One already-simulated particle handed to the renderer as a billboard.
         *
         * The extract channel for the CPU-deterministic path (analogous to @ref ClothInstance for
         * cloth): the sim advances each emitter's pool on the fixed tick and emits one of these per
         * live particle, world-space, which the renderer draws as a camera-facing quad. Distinct
         * from the renderer's GPU emitter path (`Render::ParticleEmitterView`), which simulates on
         * the GPU; these positions are already final.
         */
        struct ParticleBillboard
        {
            Vector3 position;                    /**< World-space centre. */
            Vector3 color{Vector3{1, 1, 1}};     /**< Linear-RGB tint. */
            float size = 0.1f;                   /**< World-space size. */
            float alpha = 1.0f;                  /**< Opacity. */
            float rotation = 0.0f;               /**< Roll, radians. */
        };

        /**
         * @brief The authorable parameters of a "Rigid Body" entity.
         *
         * Mirrors `Physics::RigidBody`'s mass/inertia, in editor-facing form: no
         * position/orientation (the entity's `Transform`/`Orientation` already carry
         * those) and no simulated velocity (that lives in the physics world, not in
         * anything the Inspector authors).
         */
        struct PhysicsBodyParameters
        {
            Scalar inv_mass = Scalar(1);  /**< Inverse mass; 0 pins the body in place. */
            Vector3 inv_inertia{0, 0, 0};    /**< Diagonal body-local inverse inertia; 0 = no rotation response. */
            Scalar drag_coefficient = Scalar(0); /**< Quadratic drag: acceleration -k|v|v, m⁻¹; 0 disables. */
            /**
             * @brief Mass per unit volume; zero keeps the hand-authored mass above.
             *
             * The opt-in P0 carry-over 2 asks for. Almost nobody can write a
             * correct inverse inertia tensor for anything but a sphere, and getting
             * it wrong produces a body that tumbles plausibly enough that the error
             * is never traced back — so with a density set, both numbers above are
             * derived from the *scaled* collider instead (see `sim/collider.hpp`).
             * Steel is about 7800, oak about 700, water 1000.
             */
            Scalar density = Scalar(0);

            /**
             * @brief Moved by the game rather than by forces; pushes, is never pushed.
             *
             * A lift, a moving platform, a door driven by an animation clip. The body
             * takes no gravity and no constraint ever moves it, but everything it
             * meets is moved by it — a crate on a rising platform rides up.
             *
             * A boolean rather than an inverse mass of zero, and the difference is
             * the whole point: zero inverse mass already means *pinned*, and a pinned
             * body that never moves cannot push anything. The two states are not the
             * same and cannot share a field.
             *
             * Setting this overrides @ref inv_mass, @ref inv_inertia and @ref density
             * — a kinematic body's inverse mass is zero by definition, and the
             * extract writes it rather than trusting whatever was authored.
             *
             * Drive one by writing the entity's transform each tick, the same call a
             * gizmo drag makes: a kinematic body follows that pose with a velocity
             * derived from it, so the crates it meets are pushed, while a dynamic
             * body is still teleported to it. One authoring gesture, and which of the
             * two it means is this field.
             */
            bool kinematic = false;
        };

        /**
         * @brief The authorable parameters of a character controller.
         *
         * The shape a character *moves* as, which is deliberately not the collider it is
         * *hit* as: a collider is authored to match the art, and a controller capsule is
         * authored to fit through doorways and onto stair treads. Tying them together
         * would mean every art change is a movement change.
         *
         * Every field is geometry the world imposes. Walk speed, jump height and fall
         * acceleration are absent because the controller is a geometry solver and not a
         * character — the game passes the displacement it already decided, and a
         * controller that owned falling would also have to own a gravity *direction*,
         * which on a planet is a function of position (see
         * `docs/design/solar_system_overhaul.md`).
         */
        struct CharacterParameters
        {
            /** @brief The capsule's radius: how wide a gap the character needs. */
            Scalar radius = Scalar(0.4);

            /** @brief Total standing height including both caps. */
            Scalar height = Scalar(1.8);

            /** @brief The tallest lip it climbs without jumping; zero disables stepping. */
            Scalar step_height = Scalar(0.4);

            /** @brief Faces steeper than this are walls rather than floors. */
            Scalar max_slope_degrees = Scalar(45);

            /**
             * @brief Clearance kept from every surface.
             *
             * Small and rarely worth changing, but exposed rather than hidden because
             * it is the first dial to reach for when a character catches on geometry:
             * too small and it sticks to walls, too large and it floats off ledges.
             */
            Scalar skin_width = Scalar(0.02);

            /** @brief How far below to look for ground; what keeps it on a downward ramp. */
            Scalar ground_snap = Scalar(0.1);
        };

        /**
         * @brief Half the capsule's straight segment, excluding its caps.
         *
         * The one conversion between an authored *total height* — which is what a person
         * measures a character in — and the half-height a capsule is defined by. Written
         * once because getting it wrong by a radius produces a character very slightly
         * the wrong size, which is the kind of error that is noticed only after a level
         * has been built around it.
         *
         * Clamped at zero: a character shorter than its own diameter is a sphere, not an
         * error, and refusing to represent one would make the radius field unusable at
         * small heights.
         *
         * @param character The authored parameters.
         * @return The half-height to build the capsule with.
         */
        inline Scalar character_half_height(const CharacterParameters& character) noexcept
        {
            const Scalar half = character.height * Scalar(0.5) - character.radius;
            return half > Scalar(0) ? half : Scalar(0);
        }

        /**
         * @brief What an entity does when something hits it hard enough.
         *
         * The data half of the engine's own physics listener: an author gets an impact
         * sound and a burst of particles by filling this in, with no C++ anywhere. The
         * contact stream underneath it has existed since P1 and had no reader outside
         * the test suite until this.
         *
         * The two thresholds are a ramp rather than a curve. A curve needs an editor of
         * its own, and the difference an author actually hears is between a tap and a
         * crash — which two numbers and a straight line already give.
         */
        struct ImpactResponse
        {
            /** @brief Newton-seconds below which nothing happens at all. */
            Scalar minimum_impulse = Scalar(0.5);

            /** @brief At or above this, the response plays at full strength. */
            Scalar full_impulse = Scalar(20);

            /**
             * @brief Seconds before this entity may respond again.
             *
             * Per entity, and separate from the filter the listener registers with,
             * because that one is per *pair*: a crate bouncing between two walls would
             * pass a pair cooldown twice over and sound like two crates.
             */
            Scalar cooldown_seconds = Scalar(0.15);

            /** @brief Retrigger this entity's audio emitter, scaling its gain by the ramp. */
            bool plays_audio = true;

            /** @brief Run this entity's particle emitter for @ref particle_seconds. */
            bool emits_particles = false;

            /**
             * @brief How long the particle emitter runs after an impact.
             *
             * A burst expressed as a timed stop, because `ParticleEmitterParameters`
             * carries only `seed` and `playing` — there is no burst in the emitter
             * model to ask for. That is the right place for one, and this is the
             * interim shape until it exists; see `docs/design/remaining_work.md`.
             */
            Scalar particle_seconds = Scalar(0.15);
        };

        /**
         * @brief The authorable parameters of a "Cloth" entity.
         *
         * Mirrors `Physics::build_cloth_grid`'s arguments, minus `origin` (the
         * entity's `Transform::position` supplies that, the same way a Rigid Body's
         * starting pose comes from `Transform`/`Orientation` rather than its own
         * field) and minus the world the grid is built into (that is
         * `RuntimeSimulation`-internal, not authorable). Row 0 of the grid is always
         * pinned, matching `build_cloth_grid`'s only supported topology today —
         * pinning just the corners is not yet exposed (see `docs/architecture/domain-physics.md` §1.2).
         */
        struct ClothParameters
        {
            std::size_t rows = 4;      /**< Grid rows (>= 1); row 0 is pinned. */
            std::size_t cols = 4;      /**< Grid columns (>= 1). */
            Scalar spacing = Scalar(0.5); /**< Distance between adjacent grid points. */
            Scalar compliance = Scalar(0); /**< XPBD compliance of every constraint; 0 is rigid. */
        };

        /**
         * @brief The authorable parameters of a "Crowd" entity: one skinned character
         * batched with every other crowd entity for SYCL device-side sampling (design
         * §12.3/§12.4).
         *
         * `skeleton`/`clip` are opaque handles from `ISimulation::register_crowd_skeleton`/
         * `register_crowd_clip`; `mesh`/`material` are `Render::` handles a host mesh/material
         * system already resolved, exactly like every other entity kind's visual
         * (`RenderInstance::material` already crosses this boundary the same way; only
         * *loading* stays outside the sim). Every one of those ids — and the texture ids
         * inside `material` — means something only in the session that produced it, so each
         * of the three assets is paired with the file it came from: the same id-plus-path
         * split `DecalParameters` makes, and the half of the component a scene file
         * round-trips. The maps get no paths of their own because a character file is where
         * they are named, so they come back with the mesh.
         *
         * The split is what makes the two halves resolvable by whoever can resolve them:
         * `set_crowd_parameters` re-registers `skeleton`/`clip` from their paths on every
         * write, so a crowd restored from a file or pasted from another session is bound to
         * this session's handles without its caller registering anything, while `mesh` needs
         * the render asset library and stays the host's to re-import from @ref mesh_path.
         *
         * Every crowd entity extracted in the same frame must share one `skeleton` handle
         * — `Animation::DeviceBatchEvaluator` batches one shared skeleton per call
         * (§12.3's own scoping). The frame's first-seen crowd entity's skeleton wins; any
         * entity naming a different one is skipped that frame, not silently drawn wrong —
         * see `RenderScene::skinned_instances`' own comment.
         */
        struct CrowdParameters
        {
            std::uint32_t skeleton = 0;     /**< A handle from register_crowd_skeleton (0 = none). */
            std::uint32_t clip = 0;         /**< A handle from register_crowd_clip (0 = none). */
            Render::MeshId mesh = Render::INVALID_MESH; /**< The skinned mesh to draw with. */
            Render::Material material{};    /**< Surface to shade with. */
            std::string skeleton_path;      /**< glTF @ref skeleton is registered from. */
            std::string clip_path;          /**< glTF @ref clip is registered from. */
            std::string mesh_path;          /**< glTF @ref mesh is imported from. */
            float time_seconds = 0.0f;      /**< Playback time; `tick()` advances it while playing. */
            bool loop = true;               /**< Whether playback wraps at the clip's end. */
            bool playing = true;            /**< Whether `tick()` advances @ref time_seconds. */
        };

        /**
         * @brief One tetrahedron's particles and what the last tick measured in it.
         *
         * The unit a debug view draws (§9.3's stress heat map, §9.4's plastic-strain
         * heat map) and the unit a gameplay rule reads when "is this part broken" is a
         * question about a *place* rather than about the body as a whole.
         */
        struct SoftBodyElementSample
        {
            std::uint32_t vertex[4] = {0, 0, 0, 0}; /**< Particle indices into the surface positions. */
            Scalar von_mises_stress = 0;           /**< Pascals, from the tick's final pose (§9.3). */
            Scalar plastic_strain = 0;             /**< Accumulated permanent strain, dimensionless (§9.4). */
        };

        /**
         * @brief The authorable parameters of a tetrahedral soft body (§9).
         *
         * Unlike @ref ClothParameters, which describes a shape the physics can build from
         * four numbers, a soft body cannot exist without its cook: the tetrahedral
         * lattice, the rest-state inverses, the surface hierarchy and the embedding
         * table are all things a cooker produced and nothing can re-derive at runtime.
         * So the asset is *part of the parameters* rather than a reference resolved
         * elsewhere — an entity that has lost its blob has lost its body, and that is
         * the honest thing for the type to say.
         *
         * The blob is held by value. It is the largest thing an entity record carries,
         * and holding it anywhere else would mean the world could hand the physics a
         * dangling asset across a scene reload.
         */
        struct SoftBodyParameters
        {
            /** @brief A validated `.sushisoft` blob; empty means the entity has no body. */
            std::vector<std::byte> asset;
            std::uint32_t level = 0;             /**< Which cooked simulation level to build (0 is finest). */
            Physics::SoftBodyMaterialT<Scalar> material{}; /**< Constitutive parameters (§9.2). */
            Scalar thickness = Scalar(0.01);     /**< Contact half-width of the surface (§9.6). */
            bool self_collision = false;         /**< Whether the surface is tested against itself. */
            /**
             * @brief Asks for §6.5's narrow column.
             *
             * A request rather than a setting: a body the deterministic island replays
             * is simulated in `double` however loudly this is set.
             */
            bool cosmetic = false;
        };

        /**
         * @brief The authorable parameters of a "Shape" entity: its visual mesh.
         *
         * `kind` is fixed at creation (set by which `create_*` primitive call made
         * the entity). `parameters` is editable and interpreted per `kind`: Box uses it
         * as half-extents; Sphere uses `parameters.x` as radius; Cylinder uses
         * `parameters.x` as radius and `parameters.y` as half-height. Plane is not a valid
         * Shape kind (Terrain uses a thin, flat Box for its visual instead).
         */
        struct ShapeParameters
        {
            PrimitiveKind kind = PrimitiveKind::Box;
            Vector3 parameters{Vector3{0.5, 0.5, 0.5}};
            std::string mesh_path;                    /**< glTF path `mesh` was imported from; empty = none. */

            /**
             * @brief The `mesh_path` node this Shape draws, by the file's own node index.
             *
             * A path addresses the file, not the geometry in it, so a model whose nodes each
             * carry a different mesh needs this too. The file's index rather than a position
             * in whatever order an importer walked: two parsers agreeing on a walk order today
             * is not a property either of them promises, and `IAssetLibrary::load_gltf_scene`
             * reports this key precisely so nothing has to rely on that.
             */
            std::uint32_t source_node = 0;

            /** @brief Which primitive of that node's mesh; a mesh may hold several. */
            std::uint32_t primitive = 0;

            Render::MeshId mesh = Render::INVALID_MESH; /**< When set, `kind`/`parameters` are ignored downstream. */
        };

        /**
         * @brief The authorable parameters of a punctual light on an entity.
         *
         * The light's *placement* is the entity's Transform — position for a point
         * light, position plus the local -Z aim for a spot — so only its radiometric and
         * cone properties live here. @c intensity is a raw radiance scale on the same
         * footing as the sun's, not a photometric unit (physical camera units arrive with
         * auto-exposure). Cone angles are authored in degrees; the renderer converts.
         */
        struct LightParameters
        {
            Vector3 color{Vector3{1.0, 1.0, 1.0}}; /**< Linear light colour. */
            float intensity = 20.0f;               /**< Radiance scale (HDR; tonemapped later). */
            float range = 10.0f;                   /**< Metres the windowed falloff reaches zero at. */
            bool is_spot = false;                  /**< Spot cone (true) or omnidirectional point (false). */
            float inner_degrees = 20.0f;           /**< Spot inner half-angle, degrees (full brightness). */
            float outer_degrees = 35.0f;           /**< Spot outer half-angle, degrees (dark beyond). */
            bool casts_shadows = false;            /**< Write a shadow map (spot lights only for now). */
        };

        /**
         * @brief The authorable parameters of a projected box decal on an entity.
         *
         * The decal's placement and orientation are the entity's Transform (position and
         * rotation), the same way a light's are; only its box size, tint, opacity, and
         * optional maps live here. The box projects along the entity's local forward axis.
         * With no maps set it is a flat tint; an albedo/ORM map projects real imagery and
         * surface response along the same box (loaded through the asset library like a
         * material's maps).
         */
        struct DecalParameters
        {
            Vector3 color{Vector3{0.5, 0.1, 0.1}};        /**< Linear tint blended onto the surface. */
            Vector3 half_extents{Vector3{1.0, 1.0, 0.5}}; /**< Box half-size along right/up/forward, metres. */
            float opacity = 1.0f;                         /**< Blend weight of the tint, [0, 1]. */
            Render::TextureId albedo_map = Render::INVALID_TEXTURE; /**< Projected albedo imagery (optional). */
            Render::TextureId orm_map = Render::INVALID_TEXTURE;    /**< Projected occlusion-roughness-metallic (optional). */
            std::string albedo_map_path; /**< Source of @ref albedo_map, for persistence. */
            std::string orm_map_path;    /**< Source of @ref orm_map, for persistence. */
        };

        /**
         * @brief The file each of a material's texture slots was loaded from.
         *
         * A @ref Render::Material stores opaque @ref Render::TextureId handles, which
         * are meaningless outside the session that loaded them; the paths are the
         * persistent truth a scene file round-trips, the same split VFX makes between
         * an emitter's `texture` handle and its `texture_path`. Kept beside — not
         * inside — the material because the extract copies the material into every
         * render instance each frame, and nine strings do not belong on that path.
         * An empty string means the slot carries no file-backed texture.
         */
        struct MaterialTexturePaths
        {
            std::string albedo_map;             /**< Source of Material::albedo_map. */
            std::string metallic_roughness_map; /**< Source of Material::metallic_roughness_map. */
            std::string normal_map;             /**< Source of Material::normal_map. */
            std::string height_map;             /**< Source of Material::height_map. */
            std::string occlusion_map;          /**< Source of Material::occlusion_map. */
            std::string emissive_map;           /**< Source of Material::emissive_map. */
            std::string detail_albedo_map;      /**< Source of Material::detail_albedo_map. */
            std::string detail_normal_map;      /**< Source of Material::detail_normal_map. */
            std::string detail_mask_map;        /**< Source of Material::detail_mask_map. */
        };

        /**
         * @brief The authorable parameters of a "Collider" entity: its collision volume.
         *
         * Pure authoring data today: no narrowphase or contact solver reads it yet,
         * so a Collider carries no mass or velocity of its own — whether it moves is
         * entirely determined by whether the same entity also has a Rigid Body.
         * A Collider with no Rigid Body (e.g. Terrain) is implicitly static and
         * gravity-exempt, since nothing integrates its pose. `kind`/`parameters` follow
         * the same convention as `ShapeParameters`, plus Plane, which uses `parameters` as
         * the collider's local-space normal (default {0, 1, 0}). A Collider's kind
         * can be changed independently of any Shape on the same entity (e.g. a
         * box-shaped visual with a simpler sphere collider).
         */
        struct ColliderParameters
        {
            PrimitiveKind kind = PrimitiveKind::Box;
            Vector3 parameters{Vector3{0.5, 0.5, 0.5}};

            /**
             * @brief Which collision layer this body is in, as an index rather than a mask.
             *
             * An *index*, 0 to 31, because a body is in exactly one layer — that is what
             * `Physics::CollisionFilter::layer` means by "the single layer this body belongs
             * to, as a one-bit mask", and offering an author a 32-bit field for a value with
             * one bit set is offering them a way to author something the filter cannot mean.
             * The shift happens once, in `collider_from_parameters`.
             */
            std::uint32_t layer = 0;

            /**
             * @brief The layers this body collides with, as a mask; all by default.
             *
             * A mask here and not an index, because "what do I collide with" genuinely is a
             * set. Two bodies interact only when *each* one's layer is in the other's mask,
             * so a one-sided exclusion silently does nothing — clearing a bit here is a
             * request that has to be honoured on both sides, and the Collider inspector says
             * so where an author can read it.
             */
            std::uint32_t collides_with = 0xFFFFFFFFu;

            /**
             * @brief Coulomb friction of this surface; combined with the other body's.
             *
             * §5.3's material, authored on the collider rather than as a shared asset
             * because that is where the surface is: two crates of the same mesh can be ice
             * and rubber, and nothing about them is shared but the shape. A project-wide
             * material library is a strictly later step and does not change this field —
             * it would name it.
             */
            Scalar static_friction = Scalar(0.6);

            /** @brief Resistance once the contact is already sliding. */
            Scalar dynamic_friction = Scalar(0.5);

            /** @brief How much of the closing speed is returned; 0 is dead, 1 is lossless. */
            Scalar restitution = Scalar(0);

            /**
             * @brief Reports overlaps instead of resolving them.
             *
             * `Physics::BodyFlags::trigger`, authored. The solver carries the bit and events
             * it through `ContactEvent::trigger`; this field is what lets a scene place one,
             * so a trigger volume is placeable and not merely solvable.
             */
            bool trigger = false;

            /**
             * @brief Opts this body in to conservative-advancement continuous collision.
             *
             * `Physics::BodyFlags::continuous_collision`, authored. For anything fast and
             * thin enough to tunnel through a thin static collider in one substep — the
             * budget for how many advancement sweeps this may cost is
             * `PhysicsConfiguration::continuous_advancement_budget` (§16.45.1), a scene-wide
             * setting, not per-body.
             */
            bool continuous_collision = false;

            /**
             * @brief How this surface's friction combines with the one it touches.
             *
             * `Physics::MaterialCombineMode`'s underlying value: Average, Minimum, Multiply,
             * Maximum. Both bodies have an opinion and the pair needs one number, so the
             * *stricter* of the two modes wins — a surface that insists on Minimum cannot be
             * overruled into slipperiness by whatever it happens to touch.
             */
            std::uint32_t friction_combine = 0;

            /**
             * @brief The same, for restitution; `maximum` by default, and deliberately.
             *
             * Averaging bounce is the answer nobody wants: a superball dropped on concrete
             * should bounce, and the mean of a superball and a floor is neither. The
             * grippier-wins rule for restitution means the *bouncier* surface decides, which
             * is what an author who authors one bouncy object expects of it everywhere.
             * Mirrors `Physics::PhysicsMaterialT`'s own default rather than restating it.
             */
            std::uint32_t restitution_combine = 3;
        };

        /**
         * @brief What a body is doing, for a debug view rather than for gameplay.
         *
         * §14's debug-draw bullet asks for "island colouring, sleeping state, broadphase
         * bounds", and all three are properties the solver already tracks and nothing
         * outside it could see. Its own value rather than fields on @ref SolvedPose,
         * because a pose is read every tick by everything that draws and this is read by
         * one panel when it is open — putting them together would make every frame pay for
         * a view that is usually off.
         */
        struct RigidDebugState
        {
            /** @brief The world-space bound the broadphase tests, in metres. */
            Vector3 bounds_min;
            Vector3 bounds_max;

            /**
             * @brief Which island the body was placed in this tick.
             *
             * Drawn as a colour, which is the only useful presentation: the *number* means
             * nothing across ticks — islands are renumbered whenever the partition changes
             * — but two bodies sharing one is the fact worth seeing, and equal colours say
             * that without implying the number is stable.
             */
            std::uint32_t island = 0;

            /** @brief Whether the body is asleep: not integrated, not projected, until woken. */
            bool sleeping = false;

            /** @brief Whether it is a static body, which never moves and never sleeps. */
            bool is_static = false;
        };

        /** @brief Where in a contact's life an event was reported. */
        enum class ContactPhase : std::uint8_t
        {
            /** @brief The two surfaces met this tick and were not touching last tick. */
            Begin,
            /** @brief They were touching last tick and still are. */
            Persist,
            /** @brief They were touching last tick and are not now. */
            End
        };

        /**
         * @brief One thing touching another, reported to gameplay.
         *
         * Reported per *pair of colliders*, not per contact point: a crate landing
         * flat on the ground produces four points and one event, because "the crate
         * landed" is one thing that happened. The point and normal are the manifold's
         * deepest point, which is the one a sound, a decal or a damage number wants.
         *
         * A pair against static geometry has `b == NULL_ENTITY`. That is not a
         * missing value — static geometry is not an entity, and inventing one so the
         * field is never null would mean every listener filtering out a fiction.
         */
        struct ContactEvent
        {
            EntityId a = NULL_ENTITY;
            EntityId b = NULL_ENTITY;
            ContactPhase phase = ContactPhase::Begin;

            /** @brief The manifold's deepest point, in world space. */
            Vector3 point;

            /** @brief Unit normal, pointing from @ref a toward @ref b. */
            Vector3 normal{Vector3{0, 1, 0}};

            /**
             * @brief Total normal impulse the contact carried, in newton-seconds.
             *
             * What separates a scrape from a crash, and the reason the solved
             * manifolds are read back off the device at all. Zero on a trigger
             * overlap, which is detected and never resolved, and on an `End`, which
             * reports the impulse the contact carried on its last live tick.
             */
            Scalar impulse = 0;

            /**
             * @brief Whether one side was a trigger, so the pair was reported and not resolved.
             *
             * A separate flag rather than a separate event stream: a listener that
             * wants both — a damage volume that also pushes — should not have to
             * subscribe twice and correlate, and one that wants only triggers has a
             * one-line filter.
             */
            bool trigger = false;
        };


        /**
         * @brief §5.5's `PhysicsJoint`: what this entity is attached to, and how.
         *
         * The joint lives on **one** of its two bodies and names the other, rather than
         * on a third entity naming both. That is the authoring convention because it is
         * the ownership one: a door's hinge belongs to the door, and deleting the door
         * should take its hinge with it — which it does, for free, when the hinge is the
         * door's own record. The alternative leaves a joint entity behind pointing at
         * something that is gone.
         *
         * The owning entity is the joint's **first** body and @ref connected_body its
         * second, so @ref JointParameters::anchor_a and @ref JointParameters::axis_a are read in
         * the owner's local space. Both endpoints must own rigid bodies: an immovable
         * endpoint is a body of zero inverse mass, not a missing one, which keeps every
         * joint two-sided (see `JointDescription`).
         */
        /**
         * @brief §5.5's `VehicleInstance`: which cooked vehicle this entity is.
         *
         * A path rather than bytes or a handle, unlike every other asset reference at this
         * boundary, and the difference is deliberate. A soft body's `.sushisoft` is loaded
         * by whoever cooked it and handed across as bytes the physics copies out of; a
         * vehicle is placed by an *author*, in a scene file, that has to survive being
         * reopened on another machine — and the only thing that survives that is a path
         * relative to the project.
         *
         * The setup beside it is authored numbers, not a cooked blob: §11's whole split is
         * that the structure is cooked and the corners, tyres, drivetrain and aerodynamics
         * are not. It is carried inline rather than in a second file because that is what
         * the Vehicle window edits, and a separate `.sushivehicle` would be a second thing
         * to keep in step with the scene that names it.
         */
        struct VehicleInstanceParameters
        {
            /** @brief Project-relative path to the `.sushinodebeam` this vehicle is built from. */
            std::string asset_path;

            /** @brief The authored setup: corners, tyres, drivetrain, aerodynamics (§11). */
            Physics::VehicleAssetT<Scalar> setup{};
        };

        /**
         * @brief What a driver is asking the vehicle to do, this tick.
         *
         * Held rather than applied: `set_vehicle_input` records it and the step spends it,
         * because throttle is a *state* an input device holds down and not an event. A
         * caller that stops calling therefore keeps the pedal where it left it, which is
         * what a pedal does.
         */
        struct VehicleInput
        {
            /** @brief 0 to 1. Below the idle band the governor holds the engine up anyway. */
            Scalar throttle = 0;

            /**
             * @brief Brake torque at every wheel, in N·m.
             *
             * A torque and not a fraction, because a brake is a torque: expressing it as
             * "50% braking" would hide that the number an author has to get right is how
             * much torque the discs can make, which is a property of the car.
             */
            Scalar brake = 0;

            /** @brief Steering angle in radians, applied to whichever corners steer. */
            Scalar steer = 0;

            /** @brief Clutch engagement, 0 to 1; zero disconnects the engine entirely. */
            Scalar clutch = 1;

            /**
             * @brief Which gear ratio is selected, as an index into the gearbox.
             *
             * An index rather than a signed gear number, because reverse is a *ratio* the
             * gearbox holds like any other and a separate sign would be a second way to say
             * the same thing. Out of range is refused rather than clamped — silently
             * selecting a gear nobody asked for is worse than refusing.
             */
            std::size_t gear = 0;
        };

        /**
         * @brief What the drivetrain did with the last tick's input.
         *
         * The readout §11.4's chain produces anyway, so a dashboard, a gear-shift sound and
         * a traction-control system are all reading a measurement rather than re-deriving
         * one from wheel speeds.
         */
        struct VehicleReport
        {
            Scalar engine_rate = 0;    /**< Crankshaft speed, in radians per second. */
            Scalar engine_torque = 0;  /**< What the curve gave at that speed, in N·m. */
            Scalar clutch_torque = 0;  /**< What the plate actually carried, in N·m. */
            Scalar clutch_slip = 0;    /**< Speed difference across it, in rad/s. */
            bool clutch_slipping = false; /**< Whether it was at its capacity and slipping. */

            /** @brief How many parts have come off since the vehicle was built. */
            std::size_t parts_detached = 0;

            /** @brief How many beams have broken since it was built. */
            std::size_t beams_broken = 0;
        };

        struct PhysicsJointParameters
        {
            /**
             * @brief The entity this one is attached to; @ref NULL_ENTITY attaches nothing.
             *
             * A joint naming no partner, or naming an entity with no rigid body, is
             * *authoring in progress* rather than an error — an author picks the kind
             * before they pick the partner. It simply does not become a live joint, and
             * the inspector says why rather than the scene silently omitting it.
             */
            EntityId connected_body = NULL_ENTITY;

            /** @brief What is held between the two bodies. */
            JointParameters joint;
        };

        /**
         * @brief Which kind of UI node a UI entity is.
         *
         * A `Canvas` is the full-viewport root every other UI element lays out
         * inside; `Image` and `Panel` draw a filled rectangle; `Text` draws a
         * label; `Button` draws a filled rectangle with a centred label and
         * reacts to the pointer. Modelled the same way as `PrimitiveKind` — a
         * plain host-side tag on the entity's UI record, not an ECS component —
         * since no Schedule system reads it (see components.hpp).
         */
        enum class UIElementKind : std::uint32_t
        {
            Canvas,
            Panel,
            Image,
            Text,
            Button,
        };

        /**
         * @brief The authorable parameters of a UI entity: a UGUI RectTransform plus paint.
         *
         * The rect follows Unity's uGUI model exactly: `anchor_min`/`anchor_max`
         * are normalized [0,1] fractions of the parent rect, `pivot` is the
         * element's own normalized handle, and `anchored_position`/`size_delta`
         * are pixel offsets resolved against the anchored span (see
         * `SushiEngine::UI::resolve_rect`). A `Canvas` ignores the rect and fills
         * its viewport; its `size` doubles as the reference resolution for a future
         * scaler. `text` is an inline fixed buffer (like `UI::UIText`) so the whole
         * struct stays a trivially copyable value across the seam.
         */
        struct UIElementParameters
        {
            UIElementKind kind = UIElementKind::Image;
            Scalar anchor_min_x = Scalar(0.5);   /**< Left anchor, fraction of parent width. */
            Scalar anchor_min_y = Scalar(0.5);   /**< Bottom anchor, fraction of parent height. */
            Scalar anchor_max_x = Scalar(0.5);   /**< Right anchor, fraction of parent width. */
            Scalar anchor_max_y = Scalar(0.5);   /**< Top anchor, fraction of parent height. */
            Scalar pivot_x = Scalar(0.5);        /**< Element pivot X, normalized. */
            Scalar pivot_y = Scalar(0.5);        /**< Element pivot Y, normalized. */
            Scalar position_x = Scalar(0);       /**< Anchored position X, pixels. */
            Scalar position_y = Scalar(0);       /**< Anchored position Y, pixels. */
            Scalar size_x = Scalar(160);         /**< Width added to the anchored span, pixels. */
            Scalar size_y = Scalar(40);          /**< Height added to the anchored span, pixels. */
            Vector3 color{Vector3{Scalar(0.9), Scalar(0.9), Scalar(0.9)}}; /**< Fill/text colour. */
            Scalar alpha = Scalar(1);            /**< Opacity in [0,1]. */
            Scalar font_size = Scalar(18);       /**< Text/label point size. */
            char text[64] = {};                  /**< Inline label text (Text/Button). */
        };

        /**
         * @brief One value of a user-defined "script" component field.
         *
         * A tagged union-by-convention: `kind` picks which of the payload members
         * is meaningful. This is the data-driven stand-in for a compiled
         * MonoBehaviour field — the engine has no scripting VM, so a custom
         * component is authoring data (a named, typed set of fields) the editor
         * lets a user attach, edit, and serialize, alongside a generated C++ system
         * stub they fill in. Unlike ECS components this is a boundary DTO, not a
         * device type, so it need not be trivially copyable.
         */
        enum class ScriptFieldKind : std::uint32_t
        {
            Float,
            Int,
            Bool,
            Vector3,
            Color,
            Text,
        };

        /** @brief One named, typed field of a script component instance. */
        struct ScriptField
        {
            std::string name;                       /**< Field name as authored. */
            ScriptFieldKind kind = ScriptFieldKind::Float;
            Scalar number = Scalar(0);              /**< Float/Int payload. */
            bool flag = false;                      /**< Bool payload. */
            Vector3 vector{};                       /**< Vector3/Color payload. */
            std::string text;                       /**< Text payload. */
        };

        /**
         * @brief One user-defined "script" component attached to an entity.
         *
         * Named by `type_name` (the script's class name) and carrying a flat list
         * of authored fields. Instances are stored per entity by the simulation the
         * same way cloth parameters are — plain host bookkeeping keyed on
         * `EntityId` — while the catalog of definitions lives in the editor.
         */
        struct ScriptComponent
        {
            std::string type_name;               /**< The script's type/class name. */
            std::vector<ScriptField> fields;     /**< Its authored fields, in order. */
        };

        /**
         * @brief The prefab an entity's subtree was built from, and the revision it was built at.
         *
         * Carried by an instance's root entity and by no other. Two fields are the whole
         * linkage: the refresh pass that runs when a scene opens compares @ref revision against
         * the prefab's current one and rebuilds what does not match. Edits made inside the
         * subtree do not survive that rebuild; preserving them is override resolution, which is
         * a later phase and has no affordance in this one.
         */
        struct PrefabInstanceParameters
        {
            std::string path;     /**< The `.sushiprefab` this subtree was built from. */
            std::string revision; /**< The prefab's revision at the time it was built. */
        };

        /** @brief The resolved camera for one display: the winner among its cameras. */
        struct DisplayCamera
        {
            std::uint32_t display = 0; /**< The display index this camera drives. */
            CameraState state;         /**< The pose and lens to render it with. */
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
            std::vector<RenderInstance> instances;       /**< Every drawable object this frame. */
            std::vector<DisplayCamera> display_cameras;  /**< The resolved camera per display. */
            CameraState camera;                          /**< The default game camera (lowest display), only meaningful when @ref has_camera is true. */
            bool has_camera = false;                     /**< Whether any active camera resolved this frame; false means nothing should be drawn as "the game". */
            std::vector<DeformableInstance> deformable_instances; /**< Every host-simulated surface this frame. */
            std::vector<Vector3> deformable_vertices;    /**< World-space points for every @ref deformable_instances entry, concatenated. */
            /**
             * @brief Triangle lists for every @ref deformable_instances entry, concatenated.
             *
             * Each instance's slice is numbered relative to its own @ref
             * DeformableInstance::first_vertex, not into the concatenated vertex array. That
             * keeps a surface's topology independent of what else happened to be extracted
             * this frame — the same list can be reused across frames without rewriting, and
             * the renderer applies the offset once per draw rather than once per index.
             */
            std::vector<std::uint32_t> deformable_indices;
            std::vector<Render::PunctualLight> lights;   /**< Every punctual light this frame, placed by its entity's transform. */
            std::vector<Render::Decal> decals;           /**< Every projected decal this frame, placed by its entity's transform. */
            std::vector<ParticleBillboard> particle_billboards; /**< Every live deterministic-emitter particle this frame. */
            /**
             * @brief The frame's cosmetic emitters, for the GPU simulation path.
             *
             * The other half of what a scene emitter can be. An effect whose emitters declare the
             * Cosmetic domain is not stepped on the CPU at all: the sim only places it (transform,
             * this frame's spawn count, its compiled record and LUT atlases) and the renderer emits
             * and integrates it on the GPU, which is what buys ribbons, mesh particles, depth
             * collision, and counts a host pool could not hold.
             *
             * The `compiled` and LUT pointers are into the sim's own effect database, which
             * outlives the frame; they are re-emitted every frame rather than cached.
             */
            std::vector<Render::ParticleEmitterView> particle_emitters;
            Render::Environment environment;             /**< The sun, WGS84 planet, atmosphere, clouds, and stars lighting this frame. */
            /**
             * @brief Every crowd character this frame, device-batch-sampled (design §12.3/§12.4).
             *
             * `RuntimeSimulation` gathers every crowd entity sharing the frame's bound skeleton,
             * samples them all in one `Animation::DeviceBatchEvaluator::evaluate()` call, and
             * fills one `Render::SkinnedInstance` per entity here — `palette`/`previous_palette`
             * point into the evaluator's own retained buffers (valid until the next `tick()`
             * calls `evaluate()` again, which is after this snapshot's consumer has read it).
             * A crowd entity naming a skeleton other than the frame's bound one is skipped
             * (not drawn) rather than sampled wrong; `previous_palette` is always null (the
             * device evaluator does not yet retain a prior-frame palette, so crowd motion
             * vectors currently read as zero motion — a documented, not silent, limitation).
             */
            std::vector<Render::SkinnedInstance> skinned_instances;
        };

        /**
         * @brief The editor's read/write surface onto the live world.
         *
         * The query and mutation half of the seam, split from `ISimulation` so a panel
         * that only inspects or edits entities depends on this narrow interface, not on
         * the stepping engine (interface segregation). Entities are addressed by their
         * stable `EntityId`; every mutation is applied to the world so the next
         * extracted `RenderScene` reflects it. Names and visibility are editor metadata
         * the simulation keeps host-side; transform and colour are backed by real ECS
         * components.
         */
        class IWorldEditor
        {
            public:
                virtual ~IWorldEditor() = default;

                /** @brief The live entities in display order. */
                virtual std::vector<EntityId> entities() const = 0;

                /** @brief Whether @p id names a live entity. */
                virtual bool exists(EntityId id) const noexcept = 0;

                /** @brief The entity's display name (empty if it does not exist). */
                virtual std::string name(EntityId id) const = 0;

                /**
                 * @brief Which entity of a prefab this one was instantiated from.
                 *
                 * Empty for an entity that came from no prefab — including one an author
                 * added to an instance by hand, which is a state the refresh reads and
                 * not a missing value.
                 *
                 * This is what an override is keyed by. A rebuild serializes each live
                 * member, matches it to the prefab's record by this id, and treats the
                 * components that differ as the instance's local edits — so nothing has
                 * to be recorded at the moment of an edit, and no component setter has
                 * to know that prefabs exist (`docs/design/prefab_system.md` §10, P2).
                 */
                virtual std::string prefab_entity_id(EntityId id) const = 0;

                /** @brief The entity's transform (identity if it does not exist). */
                virtual EntityTransform transform(EntityId id) const = 0;

                /** @brief The entity's world transform. */
                virtual EntityTransform world_transform(EntityId id) const = 0;

                /** @brief The entity's base colour (zero if it does not exist). */
                virtual Vector3 color(EntityId id) const = 0;

                /** @brief The entity's PBR material (defaults if it does not exist). */
                virtual Render::Material material(EntityId id) const = 0;

                /**
                 * @brief The source paths of the entity's material textures.
                 *
                 * The persistence side of @ref material: the material's texture ids are
                 * session-local handles, and these are the files they were loaded from
                 * (empty for slots with no file-backed texture).
                 *
                 * @param id The entity to read.
                 * @return The per-slot source paths (all empty if @p id does not exist).
                 */
                virtual MaterialTexturePaths material_texture_paths(EntityId id) const = 0;

                /** @brief The scene-global lighting environment (sun, planet, atmosphere). */
                virtual Render::Environment environment() const = 0;

                /** @brief Whether the entity is drawn. */
                virtual bool visible(EntityId id) const noexcept = 0;

                /** @brief Whether the entity's own flag is set (Unity's `activeSelf`). */
                virtual bool enabled(EntityId id) const noexcept = 0;

                /**
                 * @brief Whether the entity and every ancestor above it are enabled (Unity's
                 * `activeInHierarchy`). What physics, audio and render actually gate on — not
                 * @ref enabled alone, which is local to this one entity.
                 */
                virtual bool enabled_in_hierarchy(EntityId id) const noexcept = 0;

                /**
                 * @brief Creates a static entity at the origin and selects nothing.
                 *
                 * The new entity carries no motion, so it stays where it is placed and
                 * edited even while the world is playing (unlike the seeded demo cubes,
                 * which their systems drive).
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create(const std::string& name) = 0;

                /** @brief Destroys @p id; a no-op if it does not exist. */
                virtual void destroy(EntityId id) = 0;

                /** @brief Sets the entity's display name. */
                virtual void set_name(EntityId id, const std::string& name) = 0;

                /**
                 * @brief Records which prefab entity this one came from.
                 *
                 * Written by `apply_prefab` as it creates each entity, and preserved by
                 * `save_prefab` rather than reminted — which is what makes an id survive
                 * a re-author. An artist who reorders a prefab's contents and saves
                 * again keeps every id, so every instance's overrides stay attached to
                 * the entity they were made against.
                 *
                 * An empty string clears it, detaching the entity from its prefab.
                 */
                virtual void set_prefab_entity_id(EntityId id, const std::string& value) = 0;

                /**
                 * @brief Writes the entity's local transform component.
                 *
                 * @param id    Entity to update.
                 * @param local The new local transform.
                 */
                virtual void set_transform(EntityId id, const EntityTransform& local) = 0;

                /**
                 * @brief Sets the entity's world transform by recomputing its local transform.
                 *
                 * @param id    Entity to update.
                 * @param world The new world transform.
                 */
                virtual void set_world_transform(EntityId id, const EntityTransform& world) = 0;

                /** @brief The entity's reference frame (scene root if it never picked one). */
                virtual EntityFrame entity_frame(EntityId id) const = 0;

                /**
                 * @brief Sets the entity's reference frame (body + mode).
                 *
                 * Re-expresses nothing in the world by itself — the scene `Transform` is
                 * untouched — so picking a body is a pure change of how the inspector reads and
                 * writes the pose. @ref FrameMode::Surface additionally marks the entity
                 * surface-anchored so its orientation is kept ground-local each step.
                 *
                 * @param id    The entity to update.
                 * @param frame The reference body and interpretation mode.
                 */
                virtual void set_entity_frame(EntityId id, const EntityFrame& frame) = 0;

                /**
                 * @brief The entity's transform expressed in its reference frame.
                 *
                 * Position is the offset from the reference body's scene-frame centre (so it is
                 * small metres, not a heliocentric number); rotation is ground-local in
                 * @ref FrameMode::Surface, else the scene rotation; scale is unchanged. Equals
                 * @ref transform when the reference body is -1 (the scene root).
                 *
                 * @param id The entity to read.
                 * @return The frame-local transform (identity if @p id does not exist).
                 */
                virtual EntityTransform frame_local_transform(EntityId id) const = 0;

                /**
                 * @brief Writes the entity's scene transform from a frame-local one.
                 *
                 * The inverse of @ref frame_local_transform: composes the frame-local pose back
                 * onto the reference body's scene centre (and, in Surface mode, stores the
                 * ground-local rotation), then writes the resulting scene `Transform`. Moves the
                 * live physics body too, like @ref set_transform.
                 *
                 * @param id    The entity to update.
                 * @param local The transform expressed in the entity's reference frame.
                 */
                virtual void set_frame_local_transform(EntityId id, const EntityTransform& local) = 0;

                /**
                 * @brief Whether the entity's frame resolves to Surface (ground-fixed) mode.
                 *
                 * True when the reference frame is Surface, or Auto resolved to Surface by
                 * altitude. In that case @ref frame_local_transform's position is a **geodetic**
                 * coordinate — `x` latitude degrees, `y` longitude degrees, `z` altitude metres
                 * above the reference ellipsoid — rather than a Cartesian offset, so the
                 * inspector can label it and the author places a spawn the way a map does.
                 *
                 * @param id The entity to query.
                 * @return Whether the frame-local position is geodetic (Surface).
                 */
                virtual bool is_surface_frame(EntityId id) const = 0;

                /** @brief Writes the entity's base colour. */
                virtual void set_color(EntityId id, const Vector3& color) = 0;

                /**
                 * @brief Writes the entity's PBR material.
                 *
                 * The material's @c albedo is kept in sync with the entity's base colour by
                 * the extract step, so authoring albedo here is overridden by @ref set_color;
                 * the metallic, roughness, and emissive fields are what this authors.
                 */
                virtual void set_material(EntityId id, const Render::Material& material) = 0;

                /**
                 * @brief Writes the source paths of the entity's material textures.
                 *
                 * Bookkeeping only — loading the files and writing the resulting ids
                 * into the material stays the caller's job (the editor's inspector and
                 * the scene loader's resolve pass both do exactly that).
                 *
                 * @param id    The entity to update.
                 * @param paths The per-slot source paths to remember.
                 */
                virtual void set_material_texture_paths(EntityId id,
                                                        const MaterialTexturePaths& paths) = 0;

                /** @brief Writes the scene-global lighting environment. */
                virtual void set_environment(const Render::Environment& environment) = 0;

                /**
                 * @brief Where this scene's weather comes from.
                 *
                 * The W4 seam (`docs/design/weather_and_clouds.md` §3): whichever
                 * `IWeatherProvider` the mode selects compiles into `Environment::clouds` every
                 * tick, exactly where manual deck authoring writes it, so
                 * `CloudscapeCompilePass` (T3) sees no difference between them.
                 *
                 * @see Simulation::WeatherMode, which carries the argument for why this is a
                 *      mode rather than a boolean.
                 */
                virtual WeatherMode weather_mode() const noexcept = 0;

                /**
                 * @brief Installs the provider the given mode calls for.
                 *
                 * Switching modes replaces the provider outright rather than converting one into
                 * the other: a placed sky and a grown one have no state in common, and pretending
                 * otherwise would mean a seed that silently stopped meaning anything.
                 *
                 * @param mode Where the weather should come from after this call.
                 */
                virtual void set_weather_mode(WeatherMode mode) = 0;

                /**
                 * @brief The seed Manual mode places its weather from.
                 *
                 * Kept by the host across a mode switch, so leaving Manual and coming back
                 * returns the same sky rather than a new one — a seed an author chose is a
                 * decision, and losing it on a radio button would be a bug that reads as
                 * randomness.
                 */
                virtual std::uint64_t weather_seed() const noexcept = 0;

                /**
                 * @brief Chooses the sky Manual mode places.
                 *
                 * Takes effect immediately in Manual and is remembered in Procedural.
                 *
                 * @param seed Any 64-bit value; identical seeds reproduce identical weather.
                 */
                virtual void set_weather_seed(std::uint64_t seed) = 0;

                /**
                 * @brief The installed weather provider's authoring surface, or null.
                 *
                 * Null whenever no provider is installed, or when the installed one cannot be
                 * authored (an ingested provider fed by real observations has no meaningful
                 * response to "place a low here"). The Weather panel reads and edits through
                 * this rather than through a long run of `IWorldEditor` pass-through methods —
                 * the same reasoning that already keeps cloth grids and audio zones as host-side
                 * objects the editor reaches into.
                 *
                 * Returning the capability rather than the concrete provider is deliberate:
                 * naming `ProceduralWeather` here would force the host to store that exact
                 * type, so no other implementation of the provider seam could be installed
                 * at all.
                 */
                virtual IWeatherAuthoring* weather_authoring() noexcept = 0;

                /**
                 * @brief The installed weather provider, read-only, or null if none is installed.
                 *
                 * The counterpart to @ref weather_authoring, and separate from it for the reason
                 * the two interfaces are separate at all (ISP, `docs/design/atmosphere_system.md`
                 * §3.5): the Weather panel's map *draws* the pressure and thermal fields, which
                 * is a read, and *injects* an anomaly when clicked, which is a write. Handing the
                 * panel one object that does both would put the authoring surface in front of
                 * every consumer that only wants to look at the weather.
                 *
                 * Const on purpose: nothing reached through here can change the simulation, so a
                 * panel, a debug overlay or a test can sample the field without being able to
                 * disturb it.
                 */
                virtual const IWeatherProvider* weather_provider() const noexcept = 0;

                /** @brief Sets whether the entity is drawn. */
                virtual void set_visible(EntityId id, bool visible) = 0;

                /** @brief Sets the entity's own flag (Unity's `activeSelf`). */
                virtual void set_enabled(EntityId id, bool enabled) = 0;

                /**
                 * @brief Whether the entity carries a Renderer component.
                 *
                 * Mirrors Unity's MeshRenderer: an entity always has a Transform, but
                 * only draws (and has a `color()`) when a Renderer is attached.
                 */
                virtual bool has_renderer(EntityId id) const noexcept = 0;

                /**
                 * @brief Attaches or detaches the Renderer component.
                 *
                 * Attaching gives the entity a default colour; detaching stops it being
                 * drawn. A no-op on an entity whose component set cannot be changed
                 * (e.g. the seeded, animated demo cubes).
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Renderer after this call.
                 */
                virtual void set_has_renderer(EntityId id, bool value) = 0;

                /** @brief The entity's parent, or `NULL_ENTITY` if it is a root. */
                virtual EntityId parent(EntityId id) const noexcept = 0;

                /**
                 * @brief Reparents @p child under @p new_parent.
                 *
                 * Pass `NULL_ENTITY` to make @p child a root. A no-op if it would create a
                 * cycle (@p new_parent is @p child itself or one of its own descendants) —
                 * the caller does not need to check ancestry itself.
                 *
                 * @param child Entity being reparented.
                 * @param new_parent The new parent, or `NULL_ENTITY` for root.
                 */
                virtual void set_parent(EntityId child, EntityId new_parent) = 0;

                /**
                 * @brief Changes the display order of an entity relative to a target entity.
                 *
                 * @param id The entity to move.
                 * @param target The target entity to move relative to.
                 * @param insert_after If true, moves @p id after @p target. If false, moves @p id before @p target.
                 */
                virtual void move_entity(EntityId id, EntityId target, bool insert_after) = 0;

                /**
                 * @brief Creates a camera entity: a pose plus a `CameraParameters`.
                 *
                 * A camera is a first-class entity (it appears in the hierarchy and has a
                 * transform) but carries no mesh, so it is not drawn as an object; instead
                 * it contributes to the resolved `RenderScene::display_cameras`. Its lens
                 * defaults to a standard perspective on display 0.
                 *
                 * @param name Display name for the new camera.
                 * @return The new camera's stable id.
                 */
                virtual EntityId create_camera(const std::string& name) = 0;

                /** @brief Whether @p id is a camera entity. */
                virtual bool is_camera(EntityId id) const noexcept = 0;

                /** @brief The camera's parameters (defaults if @p id is not a camera). */
                virtual CameraParameters camera_parameters(EntityId id) const = 0;

                /** @brief Writes a camera entity's parameters; a no-op for non-cameras. */
                virtual void set_camera_parameters(EntityId id,
                                                   const CameraParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches the Camera component on an existing entity.
                 *
                 * Unlike `create_camera` (which makes a new camera entity), this toggles
                 * the Camera component on @p id in place, so any entity can become — or
                 * stop being — a camera. Attaching gives it default lens parameters; a
                 * no-op on an entity whose component set cannot be changed.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Camera after this call.
                 */
                virtual void set_is_camera(EntityId id, bool value) = 0;

                /**
                 * @brief Creates a new entity that plays a VFX effect (deterministic path).
                 *
                 * A particle emitter is a first-class entity with a transform but no mesh; its
                 * particles are simulated on the CPU-deterministic backend each fixed tick and
                 * drawn as billboards. Its pool lives host-side on the sim, so attaching one needs
                 * no ECS migration — like cloth.
                 *
                 * @param name Display name for the new emitter.
                 * @return The new emitter's stable id.
                 */
                virtual EntityId create_particle_emitter(const std::string& name) = 0;

                /** @brief Whether @p id is a deterministic particle emitter. */
                virtual bool has_particle_emitter(EntityId id) const noexcept = 0;

                /** @brief The emitter's parameters (defaults if @p id is not an emitter). */
                virtual ParticleEmitterParameters particle_emitter_parameters(
                    EntityId id) const = 0;

                /** @brief Writes an emitter's parameters; a no-op for non-emitters. */
                virtual void set_particle_emitter_parameters(
                    EntityId id, const ParticleEmitterParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches the particle emitter on an existing entity.
                 *
                 * Host-side like cloth: no ECS migration; attaching starts a deterministic pool
                 * tracking the entity's transform, detaching stops it.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should be a particle emitter after this call.
                 */
                virtual void set_has_particle_emitter(EntityId id, bool value) = 0;

                /**
                 * @brief The effect an emitter entity owns.
                 *
                 * A particle system is a *component*: putting one on an entity is what makes that
                 * entity emit, and the effect it plays is that component's own data, not a shared
                 * asset it points at. So it lives here with the rest of the entity's state, which
                 * is also what lets the scene file round-trip it.
                 *
                 * @param id An emitter entity.
                 * @return Its authored effect; a default-constructed one when @p id has no emitter.
                 */
                virtual const VFX::ParticleEffect& particle_effect_source(EntityId id) const = 0;

                /**
                 * @brief Replaces the effect an emitter entity owns.
                 *
                 * Recompiles in place, so the emitter picks the change up on its next tick without
                 * restarting — an author dragging a slider wants the running effect to change.
                 *
                 * @param id     An emitter entity; ignored when it has no emitter.
                 * @param effect The effect it should play.
                 */
                virtual void set_particle_effect_source(EntityId id,
                                                        const VFX::ParticleEffect& effect) = 0;

                /**
                 * @brief Whether @p id is driven by the physics world (a "Rigid Body").
                 *
                 * While attached, the simulation is the source of truth for the
                 * entity's `Transform`/`Orientation` — `set_transform` still writes
                 * through, but the next `tick()` overwrites it with the solved pose,
                 * the same way a Rigidbody in Unity overrides manual transform edits
                 * while simulating.
                 */
                virtual bool has_physics_body(EntityId id) const noexcept = 0;

                /** @brief The entity's authored mass/inertia (defaults if not a rigid body). */
                virtual PhysicsBodyParameters physics_body_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a rigid body's mass/inertia; a no-op for non-rigid-bodies.
                 *
                 * Applied immediately to the live simulated body when one already
                 * exists, so dragging the Inspector's mass slider does not force a
                 * physics-world rebuild.
                 */
                virtual void set_physics_body_parameters(
                    EntityId id, const PhysicsBodyParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches the physics simulation on an existing entity.
                 *
                 * Unlike Renderer/Camera, this needs no ECS component migration — the
                 * entity's pose stays owned by `Transform`/`Orientation`; attaching only
                 * starts (and detaching stops) a physics body tracking that pose. A
                 * body count change is a physics-world rebuild (deferred to the next
                 * `tick()`), analogous to how the ECS schedule recompiles only when its
                 * chunk set changes.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should be physics-driven after this call.
                 */
                virtual void set_has_physics_body(EntityId id, bool value) = 0;

                /**
                 * @brief Whether @p id is authored as a character controller.
                 *
                 * A character is an entity with a **kinematic** rigid body plus these
                 * parameters. The flag does not create the body and does not imply one:
                 * the two are authored separately because they answer different
                 * questions — "what shape is this in the world" and "how does this walk
                 * through it" — and a character whose body was authored dynamic is a
                 * mistake `ICharacterService::move_character` reports rather than one
                 * this flag can prevent.
                 */
                virtual bool has_character(EntityId id) const noexcept = 0;

                /** @brief The entity's authored character parameters, or defaults. */
                virtual CharacterParameters character_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a character's capsule and movement limits.
                 *
                 * Applied immediately and never deferred, unlike cloth: nothing is built
                 * from these numbers. They are read at the moment a move is resolved and
                 * at no other time, so there is no world for a change to invalidate.
                 */
                virtual void set_character_parameters(EntityId id,
                                                      const CharacterParameters& parameters) = 0;

                /**
                 * @brief Marks an existing entity as a character controller, or stops.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should be a character after this call.
                 */
                virtual void set_has_character(EntityId id, bool value) = 0;

                /** @brief Whether @p id responds to being hit. */
                virtual bool has_impact_response(EntityId id) const noexcept = 0;

                /** @brief The entity's authored impact response, or defaults. */
                virtual ImpactResponse impact_response(EntityId id) const = 0;

                /** @brief Writes an entity's impact response; applied immediately. */
                virtual void set_impact_response(EntityId id,
                                                 const ImpactResponse& response) = 0;

                /** @brief Attaches or detaches an impact response on an existing entity. */
                virtual void set_has_impact_response(EntityId id, bool value) = 0;

                /**
                 * @brief Whether @p id owns a simulated cloth grid.
                 *
                 * Like `has_physics_body`, cloth needs no ECS component migration —
                 * the grid is host-side bookkeeping keyed by `EntityId`, not a
                 * per-particle entity (see `docs/architecture/domain-physics.md` §1.2). The entity's own
                 * `Transform`/`Orientation` are left alone; the grid's world positions
                 * are read separately via `cloth_particle_positions`.
                 */
                virtual bool has_cloth(EntityId id) const noexcept = 0;

                /** @brief The entity's authored cloth grid parameters (defaults if not cloth). */
                virtual ClothParameters cloth_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a cloth entity's grid parameters; a no-op for non-cloth entities.
                 *
                 * Unlike a Rigid Body's mass/inertia, a parameter change here alters
                 * the grid's body count, so it is treated the same as attaching/
                 * detaching: applied lazily, on the next `tick()`, via a full rebuild.
                 */
                virtual void set_cloth_parameters(EntityId id,
                                                  const ClothParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a simulated cloth grid on an existing entity.
                 *
                 * Deferred to the next `tick()`, same as `set_has_physics_body` — a
                 * grid (body/constraint count) change is a physics-world rebuild, not
                 * an immediate mutation.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should own a cloth grid after this call.
                 */
                virtual void set_has_cloth(EntityId id, bool value) = 0;

                /**
                 * @brief The cloth grid's current world-space particle positions.
                 *
                 * Row-major (`row * cols + col`, matching `Physics::ClothGrid`), read
                 * directly off the live simulated bodies; empty if @p id is not a
                 * cloth entity or its grid has not been built yet (the tick right
                 * after `set_has_cloth(id, true)`). This is the read-only seam a
                 * future debug draw or a real deforming mesh renderer would consume —
                 * neither exists yet, so today nothing draws the grid (see
                 * `docs/architecture/domain-physics.md` §1.2).
                 */
                virtual std::vector<Vector3> cloth_particle_positions(EntityId id) const = 0;

                /** @brief Whether @p id owns a tetrahedral soft body (§9). */
                virtual bool has_soft_body(EntityId id) const noexcept = 0;

                /**
                 * @brief The soft body's authored parameters, including its cooked asset.
                 * @param id The entity to read; a non-soft-body one reads as defaults.
                 */
                virtual SoftBodyParameters soft_body_parameters(EntityId id) const = 0;

                /**
                 * @brief Replaces the soft body's parameters and rebuilds it.
                 *
                 * Everything here is topology: a different asset, a different level, even
                 * a different precision column is a body with a different particle count
                 * and a different element list. So this rebuilds rather than edits, and
                 * whatever deformation the old body had accumulated is gone — which is
                 * what "the cook is part of the parameters" means in practice.
                 */
                virtual void set_soft_body_parameters(EntityId id,
                                                      const SoftBodyParameters& parameters) = 0;

                /** @brief Attaches or detaches a soft body on @p id. */
                virtual void set_has_soft_body(EntityId id, bool value) = 0;

                /**
                 * @brief The soft body's deformed surface as of the last completed tick.
                 *
                 * The same pair the renderer draws, offered here so a debug view or a
                 * gameplay query reads the *simulated* surface rather than a copy of it
                 * — §8.6's third invariant is that there is nothing to fall out of step
                 * with, and a second source of truth is exactly what would create one.
                 *
                 * @param id       The entity to read.
                 * @param positions Receives world-space particle positions; cleared first.
                 * @param indices   Receives the surface triangle list; cleared first.
                 * @return False when @p id owns no soft body.
                 */
                virtual bool soft_body_surface(EntityId id, std::vector<Vector3>& positions,
                                               std::vector<std::uint32_t>& indices) const = 0;

                /** @brief The largest von Mises stress in @p id's body, from its last tick (§9.3). */
                virtual Scalar soft_body_maximum_stress(EntityId id) const = 0;

                /**
                 * @brief Every tetrahedron of @p id's body, with its last tick's readouts.
                 *
                 * What the editor's debug views draw. The interior is the point: a body's
                 * surface can look untouched while the elements behind it are past yield,
                 * and a heat map over the surface alone would show none of it.
                 *
                 * @param id       The entity to read.
                 * @param elements Receives one entry per tetrahedron; cleared first.
                 * @return False when @p id owns no soft body.
                 */
                virtual bool soft_body_elements(
                    EntityId id, std::vector<SoftBodyElementSample>& elements) const = 0;

                /**
                 * @brief Whether @p id is a Crowd entity (design §12.3/§12.4).
                 *
                 * Like cloth, a crowd entity needs no ECS component migration for its
                 * animation state — playback time is host-side bookkeeping keyed by
                 * `EntityId`, sampled through `Animation::DeviceBatchEvaluator` at extract.
                 * The entity's own `Transform`/`Orientation` place it, same as every other
                 * visual entity kind.
                 */
                virtual bool has_crowd(EntityId id) const noexcept = 0;

                /** @brief The entity's authored crowd parameters (defaults if not a crowd). */
                virtual CrowdParameters crowd_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a crowd entity's parameters, re-binding its animation assets.
                 *
                 * `skeleton`/`clip` are re-registered from `skeleton_path`/`clip_path` on the
                 * way in whenever those are set, so a caller restoring a component another
                 * session wrote — a scene load, a paste — hands over the paths and reads this
                 * session's handles back from @ref crowd_parameters. Registration is cached by
                 * path, so a write that changes nothing else costs a lookup; a path naming no
                 * loadable file leaves its handle at 0, which the extract skips.
                 *
                 * @param id         The entity to update; a no-op if it does not exist.
                 * @param parameters The new skeleton/clip/mesh/material/path/playback state.
                 */
                virtual void set_crowd_parameters(EntityId id,
                                                  const CrowdParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches crowd-batched animation on an existing entity.
                 * @param id    The entity to update.
                 * @param value Whether it should be a crowd entity after this call.
                 */
                virtual void set_has_crowd(EntityId id, bool value) = 0;

                /**
                 * @brief Whether @p id is the root of a prefab instance.
                 *
                 * True exactly when the entity's @ref PrefabInstanceParameters names a path.
                 * There is no `set_has_prefab_instance` to go with this: a non-empty path is
                 * what makes an entity an instance, so the component has no
                 * present-but-empty state for a flag to distinguish. A reader looking for the
                 * fourth accessor its neighbours have is told here that it is absent on
                 * purpose.
                 */
                virtual bool has_prefab_instance(EntityId id) const noexcept = 0;

                /** @brief The entity's prefab linkage (defaults if it is not an instance). */
                virtual PrefabInstanceParameters prefab_instance(EntityId id) const = 0;

                /**
                 * @brief Writes an entity's prefab linkage.
                 *
                 * Pass a value whose `path` is empty to unlink the subtree: the entities stay
                 * exactly as they are and stop being rebuilt from the asset.
                 *
                 * @param id         The entity to update.
                 * @param parameters The prefab and the revision it was built at.
                 */
                virtual void set_prefab_instance(EntityId id,
                                                 const PrefabInstanceParameters& parameters) = 0;

                /** @brief Whether @p id carries a punctual light. */
                virtual bool has_light(EntityId id) const noexcept = 0;

                /** @brief The light parameters of @p id, or defaults if it carries none. */
                virtual LightParameters light_parameters(EntityId id) const = 0;

                /**
                 * @brief Updates the light parameters of a light-bearing entity.
                 * @param id         The entity to update; a no-op if it carries no light.
                 * @param parameters The new radiometric and cone parameters.
                 */
                virtual void set_light_parameters(EntityId id,
                                                  const LightParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a punctual light on an existing entity.
                 *
                 * Applied immediately (a light is host bookkeeping, not a physics-world
                 * rebuild): the next extract carries it into @ref RenderScene::lights.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should carry a light after this call.
                 */
                virtual void set_has_light(EntityId id, bool value) = 0;

                /** @brief Whether @p id carries an audio emitter. */
                virtual bool has_audio_emitter(EntityId id) const noexcept = 0;

                /** @brief The audio-emitter parameters of @p id, or defaults if it carries none. */
                virtual AudioEmitterParameters audio_emitter_parameters(EntityId id) const = 0;

                /** @brief Writes an emitter's parameters; a no-op for non-emitters. */
                virtual void set_audio_emitter_parameters(
                    EntityId id, const AudioEmitterParameters& parameters) = 0;

                /** @brief Attaches or detaches an audio emitter on an existing entity (host bookkeeping). */
                virtual void set_has_audio_emitter(EntityId id, bool value) = 0;

                /** @brief Whether @p id carries a reverb zone. */
                virtual bool has_reverb_zone(EntityId id) const noexcept = 0;

                /** @brief The reverb-zone parameters of @p id, or defaults if it carries none. */
                virtual ReverbZoneParameters reverb_zone_parameters(EntityId id) const = 0;

                /** @brief Writes a reverb zone's parameters; a no-op for non-zones. */
                virtual void set_reverb_zone_parameters(
                    EntityId id, const ReverbZoneParameters& parameters) = 0;

                /** @brief Attaches or detaches a reverb zone on an existing entity (host bookkeeping). */
                virtual void set_has_reverb_zone(EntityId id, bool value) = 0;

                /** @brief Whether @p id is an audio listener (the ears). */
                virtual bool has_audio_listener(EntityId id) const noexcept = 0;

                /** @brief The audio-listener parameters of @p id, or defaults if it carries none. */
                virtual AudioListenerParameters audio_listener_parameters(EntityId id) const = 0;

                /** @brief Writes a listener's parameters; a no-op for non-listeners. */
                virtual void set_audio_listener_parameters(
                    EntityId id, const AudioListenerParameters& parameters) = 0;

                /** @brief Attaches or detaches the audio listener on an existing entity (host bookkeeping). */
                virtual void set_has_audio_listener(EntityId id, bool value) = 0;

                /**
                 * @brief Creates a bare entity carrying a punctual light.
                 *
                 * No Shape or Renderer — a light is not drawn — just a Transform that
                 * places and (for a spot) aims it, the same way a mesh instance is placed.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_light(const std::string& name) = 0;

                /** @brief Whether @p id carries a projected decal. */
                virtual bool has_decal(EntityId id) const noexcept = 0;

                /** @brief The decal parameters of @p id, or defaults if it carries none. */
                virtual DecalParameters decal_parameters(EntityId id) const = 0;

                /**
                 * @brief Updates the decal parameters of a decal-bearing entity.
                 * @param id         The entity to update; a no-op if it carries no decal.
                 * @param parameters The new tint, box size, and opacity.
                 */
                virtual void set_decal_parameters(EntityId id,
                                                  const DecalParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a projected decal on an existing entity.
                 * @param id    The entity to update.
                 * @param value Whether it should carry a decal after this call.
                 */
                virtual void set_has_decal(EntityId id, bool value) = 0;

                /**
                 * @brief Creates a bare entity carrying a projected decal.
                 *
                 * No Shape or Renderer — a decal is not drawn as geometry — just a Transform
                 * that places and aims the projection box.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_decal(const std::string& name) = 0;

                /**
                 * @brief Creates a Box entity: a Shape plus a matching Collider.
                 *
                 * The visual and the collider default to the same half-extents; either
                 * can be edited or removed independently afterward through
                 * `set_shape_parameters`/`set_has_collider`.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_box(const std::string& name) = 0;

                /** @brief Creates a Sphere entity: a Shape plus a matching Collider. See `create_box`. */
                virtual EntityId create_sphere(const std::string& name) = 0;

                /** @brief Creates a Cylinder entity: a Shape plus a matching Collider. See `create_box`. */
                virtual EntityId create_cylinder(const std::string& name) = 0;

                /**
                 * @brief Creates a flat Terrain entity: a large thin Box Shape plus a
                 * Plane Collider, with no physics body.
                 *
                 * Terrain never carries a Rigid Body, which is what makes it immune to
                 * gravity — nothing integrates its pose — while its `Collider` still
                 * marks it as a participant for a future rigidbody/softbody
                 * narrowphase, since collider data alone (not the absence of motion) is
                 * what that milestone will query.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_terrain(const std::string& name) = 0;

                /**
                 * @brief Creates a Cloth entity: an entity owning a simulated cloth grid.
                 *
                 * The grid hangs from its pinned top row at the entity's
                 * `Transform::position` with default rows/cols/spacing, so a freshly
                 * created cloth is visible as a flat resting sheet in edit mode and
                 * begins to drape as soon as the world is played. Equivalent to
                 * creating an empty entity and `set_has_cloth(id, true)`, bundled so
                 * the Entity menu can offer Cloth as a first-class object.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_cloth(const std::string& name) = 0;

                /**
                 * @brief Creates a Soft Body entity from a cooked asset.
                 *
                 * The asset is required rather than optional, and that asymmetry with
                 * @ref create_cloth is the point: a cloth with default parameters is a
                 * sheet, while a soft body with no cook is nothing at all. An entity
                 * created with an unusable blob would be a Soft Body that can never
                 * become one, so the call refuses instead.
                 *
                 * @param name  Display name for the new entity.
                 * @param asset A validated `.sushisoft` blob, copied into the entity.
                 * @return The new entity's stable id, or `NULL_ENTITY` when @p asset is
                 *         not a blob this build can load.
                 */
                virtual EntityId create_soft_body(const std::string& name,
                                                  const std::vector<std::byte>& asset) = 0;

                /**
                 * @brief Creates a Crowd entity: a device-batch-sampled skinned character.
                 *
                 * Equivalent to creating an empty entity and `set_has_crowd(id, true)`,
                 * bundled so the Entity menu can offer Crowd as a first-class object. The
                 * new entity's `crowd_parameters` are all defaults (no skeleton/clip/mesh bound
                 * yet) until `set_crowd_parameters` names the glTF files to register them from,
                 * or the ids `register_crowd_skeleton`/`register_crowd_clip` already handed out.
                 *
                 * @param name Display name for the new entity.
                 * @return The new entity's stable id.
                 */
                virtual EntityId create_crowd(const std::string& name) = 0;

                /** @brief Whether @p id carries a visual Shape (Box/Sphere/Cylinder). */
                virtual bool has_shape(EntityId id) const noexcept = 0;

                /** @brief The entity's shape kind/parameters (defaults if it has no Shape). */
                virtual ShapeParameters shape_parameters(EntityId id) const = 0;

                /** @brief Writes a Shape entity's parameters; a no-op for entities without one. */
                virtual void set_shape_parameters(EntityId id,
                                                  const ShapeParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a Shape on an existing entity.
                 *
                 * Not offered directly in the Inspector's "Add Component" popup — a
                 * Shape without sane defaults is only meaningful via `create_box`/
                 * `create_sphere`/`create_cylinder`, which call this internally. Exposed
                 * on the interface so scene load can restore a saved primitive's Shape
                 * without re-deriving which `create_*` call originally made it.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Shape after this call.
                 */
                virtual void set_has_shape(EntityId id, bool value) = 0;

                /** @brief Whether @p id carries a Collider. */
                virtual bool has_collider(EntityId id) const noexcept = 0;

                /** @brief The entity's collider kind/parameters (defaults if it has no Collider). */
                virtual ColliderParameters collider_parameters(EntityId id) const = 0;

                /** @brief Writes a Collider's parameters; a no-op for entities without one. */
                virtual void set_collider_parameters(EntityId id,
                                                     const ColliderParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a Collider on an existing entity.
                 *
                 * Independent of any Shape the entity carries — a Collider can be added
                 * to a bare entity as an invisible volume, or removed from a primitive
                 * to make it visual-only.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Collider after this call.
                 */
                virtual void set_has_collider(EntityId id, bool value) = 0;

                /** @brief Whether @p id carries a Physics Joint. */
                virtual bool has_joint(EntityId id) const noexcept = 0;

                /** @brief The entity's joint authoring (defaults if it has no Physics Joint). */
                virtual PhysicsJointParameters joint_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a joint's authoring; a no-op for entities without one.
                 *
                 * Takes effect on the next step: the live joint the previous authoring
                 * produced is destroyed and a new one created, because the solver's joint
                 * carries accumulated multipliers and warm-start state that belong to the
                 * limits they were solved under. Editing a limit while playing therefore
                 * costs the joint its warm start, which is a settling tick — the honest
                 * alternative being a joint whose stored load was measured against a range
                 * it no longer has.
                 */
                virtual void set_joint_parameters(EntityId id,
                                                  const PhysicsJointParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a Physics Joint on an existing entity.
                 *
                 * Independent of whether either endpoint has a rigid body yet. A joint
                 * with nothing to hold is authoring in progress, not an error — it
                 * becomes live the moment both endpoints are bodies.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should have a Physics Joint after this call.
                 */
                virtual void set_has_joint(EntityId id, bool value) = 0;

                /**
                 * @brief The load the last step left on @p id's joint.
                 *
                 * §10.4's force and torque recovery, at the authoring boundary: the mean
                 * the mount is being pulled by and the worst single substep it survived.
                 * This is the readout §14's assembly editor calls "a live joint-load
                 * readout while playing", and it is a measurement rather than an estimate
                 * — XPBD's multipliers convert to force exactly.
                 *
                 * @param id  The entity whose joint to read.
                 * @param out Receives the load when @p id owns a *live* joint.
                 * @return False when the entity has no joint, when its joint is not live
                 *         (an endpoint has no body), or when it has broken — three states
                 *         a caller distinguishes with @ref joint_broken and
                 *         @ref PhysicsJointParameters::connected_body rather than by a load of
                 *         zero, which is also what a joint at rest reads.
                 */
                virtual bool joint_load(EntityId id, JointState& out) const = 0;

                /**
                 * @brief Whether @p id's joint has broken and is gone from the solve.
                 *
                 * A joint past its break threshold is destroyed by the physics and
                 * reported once (`IJointService::joint_broken_events`). The *authoring*
                 * survives — an author who set a threshold has not thereby deleted their
                 * hinge — so this is the flag that keeps the broken joint from being
                 * immediately recreated by the next reconcile. It clears when the joint's
                 * authoring is edited or the scene is reloaded, both of which are the
                 * author saying "put it back".
                 */
                virtual bool joint_broken(EntityId id) const noexcept = 0;

                /**
                 * @brief Where a body is, how big its broadphase bound is, and whether it sleeps.
                 *
                 * §14's debug-draw bullet, at the authoring boundary. Everything it reports is
                 * a property the solver already tracks and nothing outside the physics could
                 * see, which is why it needs a call of its own rather than being derivable
                 * from the collider and the transform: a *bound* is not the collider, an
                 * island is not a component, and "asleep" is the difference between a settled
                 * stack and a broken one.
                 *
                 * @param id  The entity to read.
                 * @param out Receives the state when @p id owns a rigid body.
                 * @return Whether @p out was written.
                 */
                /** @brief Whether @p id carries a Vehicle. */
                virtual bool has_vehicle(EntityId id) const noexcept = 0;

                /** @brief The entity's vehicle authoring (defaults if it has none). */
                virtual VehicleInstanceParameters vehicle_parameters(EntityId id) const = 0;

                /**
                 * @brief Writes a vehicle's authoring; a no-op for entities without one.
                 *
                 * Takes effect on the next step, and rebuilds the vehicle outright rather
                 * than patching it. That is not a shortcut: a vehicle is four hundred bodies
                 * and two thousand beams placed relative to a cooked structure, so "the same
                 * vehicle with one number changed" is not a thing that can be edited in
                 * place — and a rebuild that pretended otherwise would leave half the car
                 * built to one setup and half to another.
                 */
                virtual void set_vehicle_parameters(
                    EntityId id, const VehicleInstanceParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a Vehicle on an existing entity.
                 *
                 * A vehicle with no asset path is authoring in progress rather than an
                 * error, exactly like a joint with no partner: an author picks the drivetrain
                 * before they pick the body it goes in.
                 */
                virtual void set_has_vehicle(EntityId id, bool value) = 0;

                /**
                 * @brief Records what the driver is asking of @p id's vehicle.
                 *
                 * Held, not applied: throttle is a *state* an input device holds down, so a
                 * caller that stops calling leaves the pedal where it was — which is what a
                 * pedal does, and what makes this callable from a UI slider and from an
                 * input action without the two disagreeing about whose turn it is.
                 *
                 * @param id    The entity whose vehicle to drive.
                 * @param input The controls.
                 * @return Whether @p id owns a live vehicle.
                 */
                virtual bool set_vehicle_input(EntityId id, const VehicleInput& input) = 0;

                /** @brief The controls last recorded for @p id, so a UI can show them back. */
                virtual VehicleInput vehicle_input(EntityId id) const = 0;

                /**
                 * @brief What @p id's drivetrain did on the last step.
                 * @param id  The entity whose vehicle to read.
                 * @param out Receives the report when @p id owns a *live* vehicle.
                 * @return False when the entity has no vehicle or its asset failed to load —
                 *         two states the panel distinguishes, because both read as a
                 *         stationary car.
                 */
                virtual bool vehicle_report(EntityId id, VehicleReport& out) const = 0;

                /**
                 * @brief The world positions of @p id's shell nodes, for §14's node/beam view.
                 *
                 * Filled rather than returned, so a panel drawing every frame reuses its
                 * buffer instead of allocating four hundred vectors per frame per vehicle.
                 *
                 * @param id  The entity whose vehicle to read.
                 * @param out Receives one position per node.
                 * @return Whether @p id owns a live vehicle.
                 */
                virtual bool vehicle_node_positions(EntityId id,
                                                    std::vector<Vector3>& out) const = 0;

                /**
                 * @brief The shell's collision surface, live, as triangles.
                 *
                 * What the renderer draws a vehicle as, and what the physics collides it as
                 * — the same triangles, read straight off the live nodes with no cache
                 * between, so a dented panel is dented on screen in the tick it was dented
                 * and the drawing cannot disagree with the collision.
                 *
                 * @param id        The entity whose vehicle to read.
                 * @param positions Receives one position per node.
                 * @param indices   Receives the surface triangles, indexing @p positions.
                 * @return Whether @p id owns a live vehicle with a surface.
                 */
                virtual bool vehicle_surface(EntityId id, std::vector<Vector3>& positions,
                                             std::vector<std::uint32_t>& indices) const = 0;

                virtual bool physics_body_debug(EntityId id, RigidDebugState& out) const = 0;

                /**
                 * @brief Everything touching, as of the last step.
                 *
                 * The contact stream the physics already produces for gameplay, exposed for
                 * the debug view — the *same* stream, not a second one, so what is drawn is
                 * exactly what a listener would be told about. Valid until the next step.
                 *
                 * `Begin` and `Persist` are the live contacts; an `End` names a pair that has
                 * just stopped touching and is included rather than filtered, because a
                 * contact that vanishes for one tick and returns is a symptom worth seeing
                 * and a filtered stream would hide it.
                 */
                virtual const std::vector<ContactEvent>& physics_contacts() const noexcept = 0;

                /**
                 * @brief Whether @p id's orientation is anchored to the planet surface.
                 *
                 * When set, the entity's authored orientation is treated as *local to the
                 * ground* — relative to the East-North-Up tangent frame at its position on
                 * the dominant body — rather than to the fixed scene axes. The simulation
                 * composes the tangent frame onto it each step, so "upright" is identity
                 * everywhere on the body: an anchored entity stands straight in the
                 * southern hemisphere and at the poles, not tilted by its position. A
                 * no-op in a scene with no dominant body.
                 */
                virtual bool surface_anchored(EntityId id) const noexcept = 0;

                /**
                 * @brief The entity's ground-local orientation (identity if not anchored).
                 *
                 * The orientation *relative to the local tangent frame* — what the entity
                 * faces on the ground, independent of where on the planet it stands. The
                 * scene-frame orientation the renderer sees is this composed with the
                 * tangent frame; see @ref surface_anchored.
                 */
                virtual Quaternion surface_local_orientation(EntityId id) const = 0;

                /**
                 * @brief Sets the entity's ground-local orientation; a no-op if unanchored.
                 * @param id       The entity to update.
                 * @param rotation The orientation relative to the local tangent frame.
                 */
                virtual void set_surface_local_orientation(EntityId id,
                                                           const Quaternion& rotation) = 0;

                /**
                 * @brief Anchors or unanchors @p id's orientation to the planet surface.
                 * @param id    The entity to update.
                 * @param value Whether its orientation should be ground-relative.
                 */
                virtual void set_surface_anchored(EntityId id, bool value) = 0;

                /**
                 * @brief Creates a UI Canvas: the full-viewport root of a UI tree.
                 *
                 * A Canvas carries a UI element record with `UIElementKind::Canvas`;
                 * every other UI element lays out inside the Canvas that is its
                 * ancestor. It draws nothing itself but establishes the pixel rect
                 * children resolve against (see @ref UIElementParameters).
                 *
                 * @param name Display name for the new canvas.
                 * @return The new canvas entity's stable id.
                 */
                virtual EntityId create_canvas(const std::string& name) = 0;

                /**
                 * @brief Creates a UI element (Panel/Image/Text/Button) under @p parent.
                 *
                 * The element is parented to @p parent (typically a Canvas) so it
                 * inherits its layout rect; pass `NULL_ENTITY` to anchor it directly
                 * to the viewport. Its rect defaults to a centred box the caller can
                 * re-anchor via `set_ui_parameters`.
                 *
                 * @param name   Display name for the new element.
                 * @param kind   Which UI element to create.
                 * @param parent The UI ancestor to lay out inside, or `NULL_ENTITY`.
                 * @return The new element entity's stable id.
                 */
                virtual EntityId create_ui_element(const std::string& name, UIElementKind kind,
                                                   EntityId parent) = 0;

                /** @brief Whether @p id carries a UI element record. */
                virtual bool has_ui(EntityId id) const noexcept = 0;

                /** @brief Whether @p id is a UI Canvas (a UI element of kind Canvas). */
                virtual bool is_canvas(EntityId id) const noexcept = 0;

                /** @brief The entity's UI element parameters (defaults if it has no UI). */
                virtual UIElementParameters ui_parameters(EntityId id) const = 0;

                /** @brief Writes a UI entity's element parameters; a no-op for non-UI entities. */
                virtual void set_ui_parameters(EntityId id,
                                               const UIElementParameters& parameters) = 0;

                /**
                 * @brief Attaches or detaches a UI element on an existing entity.
                 *
                 * Attaching defaults the entity to an `Image` element; the kind can
                 * then be changed via `set_ui_parameters`. Detaching removes it from the
                 * UI overlay. Like cloth, this is host-side bookkeeping needing no ECS
                 * component migration.
                 *
                 * @param id    The entity to update.
                 * @param value Whether it should carry a UI element after this call.
                 */
                virtual void set_has_ui(EntityId id, bool value) = 0;

                /** @brief The type names of every script component attached to @p id, in order. */
                virtual std::vector<std::string> script_components(EntityId id) const = 0;

                /** @brief Whether @p id carries a script component named @p type_name. */
                virtual bool has_script_component(EntityId id,
                                                  const std::string& type_name) const = 0;

                /** @brief The named script component's authored fields (empty if absent). */
                virtual ScriptComponent script_component(EntityId id,
                                                         const std::string& type_name) const = 0;

                /**
                 * @brief Attaches a script component instance to @p id.
                 *
                 * A no-op if a component of the same `type_name` is already attached,
                 * so re-adding never duplicates. The instance carries its own copy of
                 * the field values (seeded by the editor from the definition catalog),
                 * so later edits to the definition do not retroactively change it.
                 *
                 * @param id        The entity to update.
                 * @param component The script component instance to attach.
                 */
                virtual void add_script_component(EntityId id,
                                                  const ScriptComponent& component) = 0;

                /** @brief Overwrites the fields of the like-named script component on @p id; a no-op if absent. */
                virtual void set_script_component(EntityId id,
                                                  const ScriptComponent& component) = 0;

                /** @brief Detaches the script component named @p type_name; a no-op if absent. */
                virtual void remove_script_component(EntityId id,
                                                     const std::string& type_name) = 0;

                /**
                 * @brief Tells the world the pixel size the UI is currently being viewed at.
                 *
                 * Every UI entity's layout (see
                 * `UIElementParameters`/`SushiEngine::UI::resolve_rect`) resolves against a
                 * Canvas's rect, and a full-viewport Canvas's rect is the screen it fills — so
                 * the host (the editor's viewport panel, or the runtime window) calls this once
                 * per frame with its current pixel size before reading back `ui_parameters`/the UI
                 * overlay, the same way a resize event drives any other screen-space layout. A
                 * host with more than one UI-bearing surface (e.g. Scene and Game views) calls
                 * it with whichever surface's size should currently drive layout; the most
                 * recent call wins.
                 *
                 * @param width  Target width in pixels (clamped to at least 1).
                 * @param height Target height in pixels (clamped to at least 1).
                 */
                virtual void set_ui_target_size(std::uint32_t width, std::uint32_t height) = 0;
        };

        /**
         * @brief A live world a host ticks and draws without seeing the runtime.
         *
         * Owns the ECS world, its schedule, and the SushiRuntime that executes it.
         * `tick()` advances the world one fixed step; `render_scene()` returns the
         * snapshot extracted after the most recent tick; `world()` is the editor's
         * read/write surface onto the same world.
         */
        class ISimulation
        {
            public:
                virtual ~ISimulation() = default;

                /**
                 * @brief Advances the world by zero or more fixed simulation steps.
                 *
                 * Feeds @p real_delta_seconds into an internal `Loop::FixedTimestepClock`
                 * and runs the schedule on the runtime (the systems execute as SYCL
                 * kernels) once per whole fixed step the clock reports — zero if the
                 * caller has been ticking faster than the fixed rate, more than one if a
                 * host frame hitched. Each individual step is fixed and deterministic; a
                 * host still gates motion by choosing whether to call this at all
                 * (play/pause), not by scaling @p real_delta_seconds into device code.
                 *
                 * @param real_delta_seconds Wall-clock time since the last call, in
                 * seconds, as measured by the host (never read from inside the sim).
                 */
                virtual void tick(Scalar real_delta_seconds) = 0;

                /**
                 * @brief The duration of one fixed simulation step, in seconds.
                 *
                 * The size of the step `tick()`'s internal `Loop::FixedTimestepClock`
                 * advances by; a host uses this to force exactly one step (e.g. a
                 * "Step" button while paused) by calling `tick(fixed_dt_seconds())`
                 * regardless of how much real time actually elapsed.
                 */
                virtual Scalar fixed_dt_seconds() const noexcept = 0;

                /** @brief The snapshot extracted after the most recent `tick()`. */
                virtual const RenderScene& render_scene() const noexcept = 0;

                /** @brief Number of live entities in the world. */
                virtual std::size_t entity_count() const noexcept = 0;

                /**
                 * @brief What the most recent physics step contained and what it cost.
                 *
                 * Exposed here rather than by handing out the physics scene, because a
                 * caller that wants the numbers is not a caller that should be able to
                 * add bodies. A world with no physics reports a zeroed value rather
                 * than failing, so a panel can draw it unconditionally.
                 */
                virtual const Physics::PhysicsStatistics& physics_statistics() const noexcept = 0;

                /**
                 * @brief Requests per-stage physics timings (the profiler panel's seam).
                 *
                 * Forwards to the physics stepper's request: profiling is a
                 * construction-time property of the solve graph (off, the dispatch hot
                 * path reads no timestamps), so the request takes effect when the
                 * solver is next built — open the panel before physics first steps and
                 * the timings flow. Virtual with a no-op default so a headless or
                 * physics-less implementation ignores it harmlessly.
                 *
                 * @param enabled Whether solvers built from now on collect timings.
                 */
                virtual void set_physics_profiling(bool enabled) { (void)enabled; }

                /**
                 * @brief Requests that a joint whose island is asleep be dropped from
                 * the physics solve graph rather than dispatched and left to its
                 * projection's own early return (§16.44).
                 *
                 * Forwards to the physics stepper's request. Unlike @ref
                 * set_physics_profiling this is a live toggle, not construction-time
                 * state: it takes effect from the next tick. Off by default. Virtual
                 * with a no-op default so a headless or physics-less implementation
                 * ignores it harmlessly.
                 *
                 * @param enabled Whether a sleeping joint should be parked from now on.
                 */
                virtual void set_park_sleeping_joints(bool enabled) { (void)enabled; }

                /** @brief The editor's read/write surface onto this world. */
                virtual IWorldEditor& world() noexcept = 0;

                /**
                 * @brief Binds the renderer's readback of the GPU atmosphere.
                 *
                 * `docs/design/atmosphere_system.md` §3.2: the atmosphere's state lives on the GPU
                 * and is written by exactly one path, and the coarse, stale summary the mirror
                 * carries is the only thing that flows back the other way.
                 *
                 * On `ISimulation` rather than on `IWorldEditor` deliberately: connecting a
                 * renderer to a simulation is *host wiring*, not world editing, and a shipped
                 * runtime with no editor in it needs to do exactly this. Bound once by whoever
                 * owns both objects; a host that never binds one leaves every weather query
                 * answered from the base state — a clear sky with the synoptic wind — rather than
                 * from a sky that has silently stopped evolving.
                 *
                 * @param mirror The renderer's mirror, or null to unbind.
                 */
                virtual void set_atmosphere_mirror(
                    const Render::IAtmosphereMirror* mirror) noexcept = 0;

                /**
                 * @brief The master simulation epoch as a Julian Date.
                 *
                 * The single "now" the orbital dynamics and the sky are evaluated at. The
                 * simulation owns and advances it; a host reads it back (also carried on
                 * `render_scene().environment.observer`) to drive the sky from the same
                 * clock rather than a separate one.
                 */
                virtual double julian_date() const noexcept = 0;

                /**
                 * @brief Sets the master epoch, seeking the sky and the dynamics to it.
                 * @param julian_date The Julian Date to set as the current epoch.
                 */
                virtual void set_julian_date(double julian_date) = 0;

                /**
                 * @brief Updates the observer the astro placement uses, without re-extracting.
                 *
                 * The orbital dynamics derive a free body's scene pose through the scene
                 * frame built from this observer (its latitude, longitude, and anchor body),
                 * so it must match the one the host renders the sky with for a body and the
                 * planet it orbits to line up. A host that drives the sky each frame pushes
                 * the observer here cheaply; the epoch it carries is ignored (the sim owns
                 * that — see @ref julian_date). Applied to the next extracted snapshot.
                 *
                 * @param observer The current sky observer (latitude/longitude/anchor body).
                 */
                virtual void set_sky_observer(const Render::SkyObserver& observer) noexcept = 0;

                /**
                 * @brief Sets how fast the epoch advances while the world plays.
                 *
                 * The sky-days that pass per real second of simulation time; zero freezes
                 * the sky (planets and free bodies hold their positions). A host's
                 * "animate sky" toggle and rate map onto this — the sim advances the clock
                 * per fixed step, so time flow stays deterministic.
                 *
                 * @param days_per_second Sky-days advanced per real second (0 freezes it).
                 */
                virtual void set_time_scale_days_per_second(Scalar days_per_second) = 0;

                /**
                 * @brief Loads a glTF skeleton for crowd device-batch sampling (design §12.3).
                 *
                 * Pure animation-data import (`Animation::AnimationDatabase`, no Vulkan/render
                 * dependency) — unlike a mesh, a skeleton needs no render asset library, so the
                 * sim loads it directly rather than requiring a host to resolve it first.
                 * Loading the same path twice returns the same handle (cached), not a second
                 * copy. Rebinding `Animation::DeviceBatchEvaluator` to a different skeleton
                 * (a crowd entity naming a handle other than the currently-bound one)
                 * invalidates every clip registered so far for the *device* path — see
                 * `register_crowd_clip`.
                 *
                 * @param gltf_path Filesystem path to a `.gltf`/`.glb` file naming a skeleton.
                 * @return A handle for `CrowdParameters::skeleton`, or 0 if the file is missing or
                 *         carries no skeleton.
                 */
                virtual std::uint32_t register_crowd_skeleton(const std::string& gltf_path) = 0;

                /**
                 * @brief Loads a glTF animation clip for crowd device-batch sampling (design §12.3).
                 *
                 * @param gltf_path        Filesystem path to a `.gltf`/`.glb` file naming a clip.
                 * @param skeleton_handle  The skeleton (from `register_crowd_skeleton`) the clip's
                 *                        joint tracks are authored against.
                 * @return A handle for `CrowdParameters::clip`, or 0 if the file is missing,
                 *         carries no clip, or `skeleton_handle` is invalid.
                 */
                virtual std::uint32_t register_crowd_clip(const std::string& gltf_path,
                                                          std::uint32_t skeleton_handle) = 0;
        };

        /**
         * @brief Creates the runtime-backed live world.
         *
         * Brings up a SushiRuntime and an empty ECS world with no entities and no
         * scene loaded — the editor starts scene-less, matching a fresh project, and
         * populates the world only via `IWorldEditor` (New Entity, Entity menu) or
         * by loading a `.sushiscene`. The only place the runtime is constructed for
         * the editor.
         *
         * @return An owned simulation; never null (throws on runtime bring-up failure).
         */
        std::unique_ptr<ISimulation> create_simulation();
    } // namespace Simulation
} // namespace SushiEngine
