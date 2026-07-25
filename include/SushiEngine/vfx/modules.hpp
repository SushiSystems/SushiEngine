/**************************************************************************/
/* modules.hpp                                                            */
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
 * @file modules.hpp
 * @brief The particle module taxonomy — one descriptor struct per authorable behaviour.
 *
 * An emitter is a stack of modules across four stages (spawn, shape, init, update) plus a
 * render module. Rather than a type-erased polymorphic list (which cannot be trivially
 * copied to the GPU and would force a vtable dispatch per particle), each module is its own
 * plain descriptor struct carrying an @c enabled flag; @ref EmitterDescriptor aggregates the
 * set. This is the Open/Closed seam: a new behaviour adds a struct here, a bake handler in
 * @ref EmitterCompiler, and a branch in the sim shader / CPU integrator — no existing module,
 * pass, or backend is touched. Update modules that vary a property over life own an
 * @ref AnimationCurve or @ref ColorGradient directly (authoring types; only their baked LUTs
 * reach the GPU).
 */

#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/vfx/curve.hpp>
#include <SushiEngine/vfx/gradient.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /**
         * @brief Which simulator advances an emitter's particles.
         *
         * @c Cosmetic runs on the GPU (millions, non-deterministic, outside the fixed-step
         * sim island); @c Deterministic runs on the CPU inside the ECS tick (bounded,
         * byte-reproducible, gameplay-authoritative). Both consume the same CompiledEffect,
         * so this only chooses the backend, never the look. Shared by the ECS emitter
         * component and the authoring model.
         */
        enum class SimulationDomain : std::uint32_t
        {
            Cosmetic = 0,
            Deterministic = 1,
        };

        /** @brief The volume new particles are born in and the direction they emit along. */
        enum class EmitterShape : std::uint32_t
        {
            Point = 0,      /**< All particles born at the emitter origin. */
            Sphere = 1,     /**< Born in/on a sphere; emit radially outward. */
            Hemisphere = 2, /**< Upper half-sphere; emit radially outward. */
            Cone = 3,       /**< Born in a cone; emit along the cone axis, spread by angle. */
            Box = 4,        /**< Born in a box; emit along +Y. */
            Circle = 5,     /**< Born on a ring in the XZ plane; emit radially. */
        };

        /** @brief How a particle's colour composites against the scene. */
        enum class BlendMode : std::uint32_t
        {
            Additive = 0,      /**< src + dst; fire, sparks, magic. */
            Alpha = 1,         /**< src.a lerp; smoke, dust. */
            Premultiplied = 2, /**< src + dst*(1-a); pre-multiplied textures. */
        };

        /** @brief Whether and how alive particles are ordered before drawing. */
        enum class SortMode : std::uint32_t
        {
            None = 0,         /**< No sort; correct for additive/premultiplied blending. */
            ViewDistance = 1, /**< Back-to-front by camera distance; needed for alpha. */
        };

        /** @brief How a billboard is oriented in view space. */
        enum class RenderAlignment : std::uint32_t
        {
            FaceCamera = 0,        /**< Camera-facing quad. */
            VelocityStretched = 1, /**< Stretched along the velocity vector. */
            Ribbon = 2,            /**< A trail strip through the particle's recent positions. */
            Mesh = 3,              /**< A solid mesh instance per particle (debris, shells). */
        };

        /** @brief One scheduled burst of particles at a point in the emitter's life. */
        struct ParticleBurst
        {
            float time = 0.0f;          /**< Seconds after emitter start (looped by duration). */
            std::uint32_t count = 0;    /**< Particles emitted at @ref time. */
        };

        /** @brief Spawn stage: how many particles are born and when. */
        struct SpawnModule
        {
            bool enabled = true;                  /**< Whether continuous emission runs. */
            float rate_per_second = 32.0f;        /**< Continuous emission rate. */
            std::vector<ParticleBurst> bursts;    /**< Discrete bursts, additive to the rate. */
        };

        /** @brief Shape stage: the birth volume and initial emit direction. */
        struct ShapeModule
        {
            EmitterShape shape = EmitterShape::Cone; /**< The birth volume. */
            float radius = 0.5f;                     /**< Sphere/hemisphere/circle/cone base radius. */
            float cone_angle_radians = 0.436332f;    /**< Cone half-angle (~25 degrees). */
            float arc_radians = 6.2831853f;          /**< Emission arc (2*pi = full). */
            Vector3 box_half_extents{Vector3{0.5, 0.5, 0.5}}; /**< Box half-size. */
            bool emit_from_shell = false;            /**< Born on the surface rather than the volume. */
        };

        /**
         * @brief Init stage: per-particle attributes sampled at birth from [min, max] ranges.
         *
         * Every field is a min/max pair the RNG draws uniformly, which is what gives a fountain
         * its spread without per-particle authoring. Colour is a base tint the render/update
         * stage may override with a colour-over-life gradient.
         */
        struct InitModule
        {
            float lifetime_min = 1.0f;            /**< Shortest particle lifetime, seconds. */
            float lifetime_max = 2.0f;            /**< Longest particle lifetime, seconds. */
            float speed_min = 1.0f;               /**< Slowest initial speed along the emit direction. */
            float speed_max = 3.0f;               /**< Fastest initial speed along the emit direction. */
            float size_min = 0.1f;                /**< Smallest initial world-space size. */
            float size_max = 0.25f;               /**< Largest initial world-space size. */
            float rotation_min = 0.0f;            /**< Smallest initial roll, radians. */
            float rotation_max = 0.0f;            /**< Largest initial roll, radians. */
            float angular_velocity_min = 0.0f;    /**< Slowest spin, radians/second. */
            float angular_velocity_max = 0.0f;    /**< Fastest spin, radians/second. */
            Vector3 color{Vector3{1, 1, 1}};      /**< Base linear-RGB tint at birth. */
        };

        /** @brief Update stage: constant acceleration applied every tick. */
        struct GravityModule
        {
            bool enabled = false;                 /**< Whether gravity is applied. */
            Vector3 acceleration{Vector3{0, -9.81, 0}}; /**< World-space acceleration, m/s^2. */
        };

        /** @brief Update stage: velocity-proportional damping. */
        struct DragModule
        {
            bool enabled = false;   /**< Whether drag is applied. */
            float coefficient = 0.5f; /**< Fraction of velocity shed per second. */
        };

        /**
         * @brief Update stage: curl-noise turbulence that swirls particles without divergence.
         *
         * A divergence-free curl of a gradient-noise field, so particles are pushed around
         * as if by wind eddies but never converge to or spray from a point. Frequency sets the
         * eddy scale; amplitude the push strength.
         */
        struct TurbulenceModule
        {
            bool enabled = false;    /**< Whether turbulence is applied. */
            float frequency = 0.5f;  /**< Spatial frequency of the noise field (1/metre). */
            float amplitude = 1.0f;  /**< Push strength, m/s^2. */
        };

        /** @brief What a force field does to the particles inside it. */
        enum class ForceFieldKind : std::uint32_t
        {
            Point = 0,  /**< Pulls toward (or, negative, pushes from) a point. */
            Vortex = 1, /**< Swirls around an axis through a point. */
            Drag = 2,   /**< Damps velocity inside the radius, a still-air pocket. */
        };

        /**
         * @brief Update stage: one placed field that bends particles as they pass through it.
         *
         * Where gravity and turbulence act everywhere alike, a force field has a *place*: a
         * position, a radius it reaches, and a falloff over that reach. That is what makes it the
         * tool for authored motion — an updraft over a fire, a vortex in a portal, a pocket of
         * still air behind cover — rather than for a global condition.
         *
         * The position and axis are in the emitter's local frame, so the field travels with its
         * emitter; the compiler bakes them to world space per frame.
         */
        struct ForceFieldModule
        {
            bool enabled = false;                     /**< Whether this field is applied. */
            ForceFieldKind kind = ForceFieldKind::Point; /**< What the field does. */
            Vector3 position{};                       /**< Field centre, emitter-local. */
            Vector3 axis{Vector3{0, 1, 0}};           /**< Swirl axis, Vortex only, emitter-local. */
            float strength = 10.0f;                   /**< m/s^2, or velocity shed per second for Drag. */
            float radius = 5.0f;                      /**< Reach in metres; nothing outside it is touched. */
            float falloff = 1.0f;                     /**< Strength exponent over the normalised distance. */
        };

        /**
         * @brief Update stage: bounce the cosmetic particles off whatever is on screen.
         *
         * A screen-space test against the depth the renderer already has, so it costs no collision
         * geometry and no broadphase, and it is honest about what that buys: it can only know about
         * surfaces the camera can see. Particles behind the viewer, off the sides, or hidden behind
         * something nearer pass straight through. That is the right trade for sparks skittering off
         * a floor in view; it is the wrong tool for gameplay collision, which is why the
         * deterministic backend ignores this module entirely.
         */
        struct CollisionModule
        {
            bool enabled = false;      /**< Whether depth collision is applied. */
            float restitution = 0.35f; /**< Fraction of the normal velocity kept on bounce. */
            float friction = 0.2f;     /**< Fraction of the tangential velocity shed on bounce. */
            /**
             * @brief How far behind a surface still counts as touching it, metres.
             *
             * The depth buffer records a surface, not a solid: past this depth a particle is taken
             * to be behind the object rather than resting on it, and is let through. Too small and
             * fast particles tunnel; too large and they bounce off the far side of thin geometry.
             */
            float thickness = 0.5f;
        };

        /** @brief Update stage: multiplies each particle's size by a curve of its age. */
        struct SizeOverLifeModule
        {
            bool enabled = false;    /**< Whether the size curve is applied. */
            AnimationCurve curve;    /**< Multiplier vs normalized age. */
        };

        /** @brief Update stage: replaces each particle's colour/alpha from a gradient of its age. */
        struct ColorOverLifeModule
        {
            bool enabled = false;    /**< Whether the colour gradient is applied. */
            ColorGradient gradient;  /**< Colour and alpha vs normalized age. */
        };

        /** @brief The @ref RenderModule::texture value meaning "no sprite texture". */
        constexpr std::uint32_t NO_PARTICLE_TEXTURE = 0xFFFFFFFFu;

        /**
         * @brief Render stage: how alive particles are drawn — the particle *material*.
         *
         * Blend mode, sort mode, billboard alignment, the soft-particle depth fade, the sprite
         * texture and its optional flipbook (sub-UV) atlas, and whether the particle receives
         * scene lighting. Deliberately not a PBR material: a puff has no roughness or normal
         * map, and a particle that needs a real surface is a @c Mesh-aligned one drawn by the
         * mesh path. Untextured particles draw as the built-in soft dot.
         *
         * The texture appears twice because the two forms serve different lifetimes:
         * @ref texture is the renderer's runtime handle, valid only for the session that
         * loaded it, while @ref texture_path is what an effect asset persists. Whatever
         * deserialises an effect resolves the path into the handle.
         */
        struct RenderModule
        {
            BlendMode blend = BlendMode::Additive;          /**< Compositing mode. */
            SortMode sort = SortMode::None;                 /**< Draw ordering. */
            RenderAlignment alignment = RenderAlignment::FaceCamera; /**< Billboard orientation. */
            float velocity_stretch = 0.05f;                 /**< Streak metres per m/s, VelocityStretched only. */
            bool soft_particles = true;                     /**< Fade where the billboard meets geometry. */
            float soft_fade_distance = 0.5f;                /**< World distance over which the soft fade ramps. */
            std::uint32_t texture = NO_PARTICLE_TEXTURE;    /**< Texture-library handle, or @ref NO_PARTICLE_TEXTURE. */
            std::string texture_path;                       /**< Source of @ref texture, for persistence. */
            std::uint32_t mesh = 0;                         /**< Mesh-registry handle, Mesh alignment only. */
            std::uint32_t flipbook_rows = 1;                /**< Sub-UV atlas rows (1 = no flipbook). */
            std::uint32_t flipbook_columns = 1;             /**< Sub-UV atlas columns (1 = no flipbook). */
            bool lit = false;                               /**< Receive the sun, clustered lights, and SH ambient. */
        };
    } // namespace Vfx
} // namespace SushiEngine
