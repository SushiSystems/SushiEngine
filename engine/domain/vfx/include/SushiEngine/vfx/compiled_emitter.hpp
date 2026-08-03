/**************************************************************************/
/* compiled_emitter.hpp                                                   */
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
 * @file compiled_emitter.hpp
 * @brief The POD boundary between authoring and simulation — one emitter, flattened.
 *
 * @ref EmitterCompiler turns a heap-backed @ref EmitterDescriptor (curves, gradients, vectors,
 * strings) into a trivially-copyable @ref CompiledEmitter: packed scalars, a bitfield of
 * enabled update modules, and **row offsets** into two shared LUT atlases holding the baked
 * curves and gradients. This is the only artifact both simulation backends consume — the GPU
 * uploads the CompiledEmitter array plus the atlas bytes verbatim, and the deterministic CPU
 * integrator reads the identical layout — so an effect looks the same in either domain. It is
 * the particle system's equivalent of resolving authored @c RenderSettings into a POD
 * @c QualityParameters: rich intent in, flat data out.
 *
 * The per-particle working layout (@ref GPUParticle) is defined here too so the CPU backend,
 * the GPU system, and the compute shader share one 64-byte record.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/vfx/modules.hpp>

namespace SushiEngine
{
    namespace VFX
    {
        /** @brief Samples per baked scalar-curve LUT row. */
        constexpr std::uint32_t CURVE_LUT_WIDTH = 64;

        /** @brief RGBA samples per baked gradient LUT row. */
        constexpr std::uint32_t GRADIENT_LUT_WIDTH = 64;

        /** @brief Largest number of discrete bursts a compiled emitter carries. */
        constexpr std::uint32_t MAX_COMPILED_BURSTS = 8;

        /** @brief Sentinel LUT offset meaning "this emitter has no such curve/gradient". */
        constexpr std::int32_t NO_LUT = -1;

        /** @brief @ref CompiledEmitter::flags bits. */
        enum EmitterFlags : std::uint32_t
        {
            EMITTER_LOOPING = 1u << 0,
            EMITTER_PREWARM = 1u << 1,
        };

        /** @brief @ref CompiledEmitter::shape_flags bits. */
        enum ShapeFlags : std::uint32_t
        {
            SHAPE_EMIT_FROM_SHELL = 1u << 0,
        };

        /** @brief @ref CompiledEmitter::update_flags bits selecting the active update modules. */
        enum UpdateFlags : std::uint32_t
        {
            UPDATE_GRAVITY = 1u << 0,
            UPDATE_DRAG = 1u << 1,
            UPDATE_TURBULENCE = 1u << 2,
            UPDATE_SIZE_OVER_LIFE = 1u << 3,
            UPDATE_COLOR_OVER_LIFE = 1u << 4,
            UPDATE_FORCE_FIELDS = 1u << 5,
            UPDATE_COLLISION = 1u << 6,
        };

        /**
         * @brief Force fields one emitter can carry.
         *
         * Fixed because the compiled record is a POD both backends and the GPU read as bytes.
         * Mirrored by `MAX_FORCE_FIELDS` in particle_common.glsl.
         */
        constexpr std::uint32_t MAX_FORCE_FIELDS = 4;

        /**
         * @brief One compiled force field: where it is, how far it reaches, what it does.
         *
         * @c position and @c axis are emitter-local as authored; the render system bakes them to
         * world space when it flattens the emitter, so the shader evaluates in the same absolute
         * space the particles live in.
         */
        struct CompiledForceField
        {
            float position[3] = {0.0f, 0.0f, 0.0f}; /**< Field centre. */
            float strength = 0.0f;                  /**< m/s^2, or per-second damping for Drag. */
            float axis[3] = {0.0f, 1.0f, 0.0f};     /**< Swirl axis, Vortex only. */
            float radius = 0.0f;                    /**< Reach in metres. */
            ForceFieldKind kind = ForceFieldKind::Point; /**< What the field does. */
            float falloff = 1.0f;                   /**< Exponent over the normalised distance. */
            float pad[2] = {0.0f, 0.0f};            /**< Keeps the record three vec4s wide. */
        };

        /** @brief @ref CompiledEmitter::render_flags bits. */
        enum RenderFlags : std::uint32_t
        {
            RENDER_SOFT = 1u << 0,
            RENDER_LIT = 1u << 1,
            /**
             * @brief The emitter carries a sprite texture, so @ref CompiledEmitter::texture is real.
             *
             * A flag rather than a sentinel test in the shader because the two cases differ in
             * more than the sample: an untextured particle is the built-in radial dot, whose
             * falloff a texture's own alpha replaces. It is also the bit the renderer clears when
             * a texture cannot be resolved, which is what keeps the fragment stage from indexing
             * the bindless heap with a slot that was never allocated.
             */
            RENDER_TEXTURED = 1u << 2,
            /**
             * @brief Collide against the SDF clipmap rather than the depth pyramid.
             *
             * Rides here rather than taking its own @ref UpdateFlags bit because it selects
             * *how* @ref UPDATE_COLLISION collides, not *whether* it does: a shader that sees
             * `UPDATE_COLLISION` still knows exactly one branch to take. It is also the bit the
             * renderer clears when the distance field is unavailable, which drops the emitter
             * back to the depth test rather than to no collision at all.
             */
            RENDER_DISTANCE_FIELD_COLLISION = 1u << 3,
        };

        /**
         * @brief One emitter flattened to a device-uploadable record.
         *
         * Every field is a scalar, an enum (fixed to a @c uint32 base), or a small fixed
         * array, so the whole struct is trivially copyable and byte-comparable. Baked curves
         * and gradients are referenced by @ref size_curve_lut / @ref color_gradient_lut row
         * offsets into the owning @ref CompiledEffect's atlases (@ref NO_LUT when absent).
         */
        struct CompiledEmitter
        {
            std::uint32_t capacity = 0;                       /**< Max simultaneously-live particles. */
            SimulationDomain domain = SimulationDomain::Cosmetic; /**< Selecting backend. */
            float duration = 5.0f;                            /**< Emitter cycle length, seconds. */
            std::uint32_t flags = EMITTER_LOOPING;            /**< @ref EmitterFlags bits. */

            float spawn_rate = 0.0f;                          /**< Continuous emission, particles/second. */
            std::uint32_t burst_count = 0;                    /**< Active entries in @ref burst_time. */
            float burst_time[MAX_COMPILED_BURSTS] = {};       /**< Burst trigger times, seconds. */
            std::uint32_t burst_amount[MAX_COMPILED_BURSTS] = {}; /**< Particles per burst. */

            EmitterShape shape = EmitterShape::Point;         /**< Birth volume. */
            std::uint32_t shape_flags = 0;                    /**< @ref ShapeFlags bits. */
            float shape_radius = 0.5f;                        /**< Radius parameter. */
            float shape_cone_angle = 0.0f;                    /**< Cone half-angle, radians. */
            float shape_arc = 6.2831853f;                     /**< Emission arc, radians. */
            float shape_box_half_extents[3] = {0.5f, 0.5f, 0.5f}; /**< Box half-size. */

            float lifetime_min = 1.0f;                        /**< Init lifetime range, seconds. */
            float lifetime_max = 2.0f;
            float speed_min = 1.0f;                           /**< Init speed range along emit dir. */
            float speed_max = 3.0f;
            float size_min = 0.1f;                            /**< Init size range, world units. */
            float size_max = 0.25f;
            float rotation_min = 0.0f;                        /**< Init roll range, radians. */
            float rotation_max = 0.0f;
            float angular_velocity_min = 0.0f;                /**< Init spin range, radians/second. */
            float angular_velocity_max = 0.0f;
            float color[3] = {1.0f, 1.0f, 1.0f};              /**< Base linear-RGB tint. */

            std::uint32_t update_flags = 0;                   /**< @ref UpdateFlags bits. */
            float gravity[3] = {0.0f, 0.0f, 0.0f};            /**< Constant acceleration, m/s^2. */
            float drag_coefficient = 0.0f;                    /**< Velocity shed per second. */
            float turbulence_frequency = 0.0f;                /**< Curl-noise spatial frequency. */
            float turbulence_amplitude = 0.0f;                /**< Curl-noise push strength, m/s^2. */
            float collision_restitution = 0.0f;               /**< Normal velocity kept on bounce. */
            float collision_friction = 0.0f;                  /**< Tangential velocity shed on bounce. */
            float collision_thickness = 0.5f;                 /**< Depth behind a surface still counted. */
            std::uint32_t force_field_count = 0;              /**< Active entries in @ref force_fields. */
            CompiledForceField force_fields[MAX_FORCE_FIELDS] = {}; /**< Placed fields, emitter-local. */
            std::int32_t size_curve_lut = NO_LUT;             /**< Curve-atlas row, or NO_LUT. */
            std::int32_t color_gradient_lut = NO_LUT;         /**< Gradient-atlas row, or NO_LUT. */

            BlendMode blend = BlendMode::Additive;            /**< Compositing mode. */
            SortMode sort = SortMode::ViewDistance;           /**< Draw ordering. */
            RenderAlignment alignment = RenderAlignment::FaceCamera; /**< Billboard orientation. */
            float velocity_stretch = 0.05f;                   /**< Streak metres per m/s. */
            std::uint32_t render_flags = RENDER_SOFT;         /**< @ref RenderFlags bits. */
            float soft_fade_distance = 0.5f;                  /**< Soft-particle ramp distance. */
            std::uint32_t texture = NO_PARTICLE_TEXTURE;      /**< Texture-library handle, or none. */
            std::uint32_t mesh = 0;                           /**< Mesh-registry handle, Mesh alignment only. */
            std::uint32_t flipbook_rows = 1;                  /**< Sub-UV atlas rows. */
            std::uint32_t flipbook_columns = 1;               /**< Sub-UV atlas columns. */

            float beam_start[3] = {0.0f, 0.0f, 0.0f};         /**< Beam origin, emitter-local. */
            float beam_width = 0.25f;                         /**< Beam strip width, metres. */
            float beam_end[3] = {0.0f, 0.0f, 5.0f};           /**< Beam terminus, emitter-local. */
            float beam_sag = 0.0f;                            /**< Midpoint droop, metres. */
            float beam_noise_amplitude = 0.0f;                /**< Lateral jitter, metres. */
            float beam_noise_frequency = 4.0f;                /**< Jitter cycles along the span. */
            float beam_pad[2] = {0.0f, 0.0f};                 /**< Keeps the beam block three vec4s wide. */
        };

        /**
         * @brief The per-particle working record, shared by the CPU backend, the GPU system,
         *        and the compute shader.
         *
         * An 80-byte, 16-byte-aligned AoS record laid out as five vec4s so it matches a std430
         * storage-buffer element with no padding surprises. @c seed carries the particle's RNG
         * sub-stream; @c emitter_index keys it to its @ref CompiledEmitter.
         */
        struct GPUParticle
        {
            float position[3] = {0.0f, 0.0f, 0.0f}; /**< World position (camera-relative on GPU). */
            float life = 0.0f;                       /**< Remaining life normalized to [0, 1]. */
            float velocity[3] = {0.0f, 0.0f, 0.0f};  /**< World velocity, m/s. */
            float age = 0.0f;                        /**< Seconds since birth. */
            float color[3] = {1.0f, 1.0f, 1.0f};     /**< Current linear-RGB tint. */
            float alpha = 1.0f;                      /**< Current opacity. */
            float size = 0.1f;                       /**< Current world-space size. */
            float rotation = 0.0f;                   /**< Current roll, radians. */
            float lifetime = 1.0f;                   /**< Total lifetime, seconds. */
            float angular_velocity = 0.0f;           /**< Spin, radians/second. */
            std::uint32_t seed = 0;                  /**< RNG sub-stream selector. */
            std::uint32_t emitter_index = 0;         /**< Owning CompiledEmitter index. */
            std::uint32_t flipbook_frame = 0;        /**< Current sub-UV cell. */
            float birth_size = 0.1f;                 /**< Size at birth; size-over-life multiplies it. */
        };

        static_assert(sizeof(GPUParticle) == 80, "GPUParticle must stay an 80-byte std430 record.");

        /**
         * @brief A compiled effect: the emitter records plus the two baked LUT atlases.
         *
         * The owning container the @ref EffectDatabase caches and the GPU system uploads.
         * @ref emitters is the POD array; @ref curve_luts holds @c curve_lut_count rows of
         * @ref CURVE_LUT_WIDTH floats; @ref gradient_luts holds @c gradient_lut_count rows of
         * @ref GRADIENT_LUT_WIDTH RGBA quadruplets. Emitters reference rows by offset.
         */
        struct CompiledEffect
        {
            std::vector<CompiledEmitter> emitters;  /**< One record per emitter. */
            std::vector<float> curve_luts;          /**< Concatenated scalar-curve rows. */
            std::vector<float> gradient_luts;       /**< Concatenated RGBA gradient rows. */

            /**
             * @brief Pointer to a baked scalar-curve row.
             * @param offset Row index, or @ref NO_LUT.
             * @return The @ref CURVE_LUT_WIDTH-float row, or nullptr for @ref NO_LUT.
             */
            const float* curve_row(std::int32_t offset) const noexcept
            {
                if (offset < 0)
                    return nullptr;
                const std::size_t base = static_cast<std::size_t>(offset) * CURVE_LUT_WIDTH;
                return base < curve_luts.size() ? &curve_luts[base] : nullptr;
            }

            /**
             * @brief Pointer to a baked RGBA gradient row.
             * @param offset Row index, or @ref NO_LUT.
             * @return The @ref GRADIENT_LUT_WIDTH-RGBA row, or nullptr for @ref NO_LUT.
             */
            const float* gradient_row(std::int32_t offset) const noexcept
            {
                if (offset < 0)
                    return nullptr;
                const std::size_t base =
                    static_cast<std::size_t>(offset) * GRADIENT_LUT_WIDTH * 4;
                return base < gradient_luts.size() ? &gradient_luts[base] : nullptr;
            }

            /** @brief Whether the effect compiled to at least one emitter. */
            bool empty() const noexcept
            {
                return emitters.empty();
            }
        };

        /**
         * @brief Linearly samples a baked scalar-curve row at a normalized time.
         *
         * The CPU mirror of the sim shader's LUT fetch, so the two domains read the same value.
         *
         * @param row           The @ref CURVE_LUT_WIDTH-float row, or nullptr.
         * @param time          Normalized age in [0, 1] (clamped).
         * @param default_value Returned when @p row is nullptr.
         * @return The sampled curve value.
         */
        inline float sample_curve_lut(const float* row, float time, float default_value) noexcept
        {
            if (row == nullptr)
                return default_value;
            const float clamped = time < 0.0f ? 0.0f : (time > 1.0f ? 1.0f : time);
            const float x = clamped * static_cast<float>(CURVE_LUT_WIDTH - 1);
            const std::uint32_t i = static_cast<std::uint32_t>(x);
            const std::uint32_t j = i + 1 < CURVE_LUT_WIDTH ? i + 1 : i;
            const float f = x - static_cast<float>(i);
            return row[i] + (row[j] - row[i]) * f;
        }

        /**
         * @brief Linearly samples a baked RGBA gradient row at a normalized time.
         * @param row  The @ref GRADIENT_LUT_WIDTH-RGBA row, or nullptr.
         * @param time Normalized age in [0, 1] (clamped).
         * @param out  Receives the 4 interpolated RGBA components (untouched when @p row is nullptr).
         */
        inline void sample_gradient_lut(const float* row, float time, float out[4]) noexcept
        {
            if (row == nullptr)
                return;
            const float clamped = time < 0.0f ? 0.0f : (time > 1.0f ? 1.0f : time);
            const float x = clamped * static_cast<float>(GRADIENT_LUT_WIDTH - 1);
            const std::uint32_t i = static_cast<std::uint32_t>(x);
            const std::uint32_t j = i + 1 < GRADIENT_LUT_WIDTH ? i + 1 : i;
            const float f = x - static_cast<float>(i);
            for (std::uint32_t c = 0; c < 4; ++c)
                out[c] = row[i * 4 + c] + (row[j * 4 + c] - row[i * 4 + c]) * f;
        }
    } // namespace VFX
} // namespace SushiEngine
