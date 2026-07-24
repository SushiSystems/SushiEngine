/**************************************************************************/
/* emitter_compiler.hpp                                                   */
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
 * @file emitter_compiler.hpp
 * @brief Bakes an authored @ref ParticleEffect into a POD @ref CompiledEffect.
 *
 * The single translation from artist intent to runtime data: it clamps each emitter's capacity
 * to its domain budget, packs the scalar module parameters, folds the enabled-module set into
 * bitfields, and bakes each curve/gradient into a LUT atlas row whose offset is stored on the
 * @ref CompiledEmitter. Every module kind is handled by one small, independent step — adding a
 * module means adding a step here, nothing else changes (the Open/Closed seam). It allocates
 * only into the returned @ref CompiledEffect; it holds no state and mutates nothing.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/emitter_descriptor.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Vfx
    {
        /**
         * @brief Compiles authored effects into device-uploadable @ref CompiledEffect data.
         *
         * Stateless: the only entry point is @ref compile, which returns a fresh
         * @ref CompiledEffect owning its emitter records and LUT atlases.
         */
        class EmitterCompiler
        {
            public:
                /**
                 * @brief Compiles one effect asset.
                 * @param effect The authored effect (its emitters and modules).
                 * @return A compiled effect: POD emitter records plus baked LUT atlases.
                 */
                static CompiledEffect compile(const ParticleEffect& effect)
                {
                    CompiledEffect out;
                    out.emitters.reserve(effect.emitters.size());
                    for (const EmitterDescriptor& descriptor : effect.emitters)
                        out.emitters.push_back(compile_emitter(descriptor, out));
                    return out;
                }

            private:
                /**
                 * @brief Compiles one emitter, appending any baked LUT rows to @p effect.
                 * @param descriptor The authored emitter.
                 * @param effect     The compiled effect whose atlases receive baked rows.
                 * @return The flattened POD emitter record.
                 */
                static CompiledEmitter compile_emitter(const EmitterDescriptor& descriptor,
                                                       CompiledEffect& effect)
                {
                    CompiledEmitter compiled;

                    const std::uint32_t budget =
                        descriptor.domain == SimulationDomain::Deterministic
                            ? MAX_DETERMINISTIC_PARTICLES
                            : MAX_EMITTER_CAPACITY;
                    compiled.capacity = std::min(descriptor.capacity, budget);
                    if (compiled.capacity == 0)
                        compiled.capacity = 1;
                    compiled.domain = descriptor.domain;
                    compiled.duration = descriptor.duration > 0.0f ? descriptor.duration : 1.0f;
                    compiled.flags = 0;
                    if (descriptor.looping)
                        compiled.flags |= EMITTER_LOOPING;
                    if (descriptor.prewarm)
                        compiled.flags |= EMITTER_PREWARM;

                    compile_spawn(descriptor.spawn, compiled);
                    compile_shape(descriptor.shape, compiled);
                    compile_init(descriptor.init, compiled);
                    compile_update(descriptor, compiled, effect);
                    compile_render(descriptor.render, compiled);
                    return compiled;
                }

                /** @brief Packs the spawn rate and up to @ref MAX_COMPILED_BURSTS bursts. */
                static void compile_spawn(const SpawnModule& spawn, CompiledEmitter& compiled)
                {
                    compiled.spawn_rate = spawn.enabled ? std::max(spawn.rate_per_second, 0.0f) : 0.0f;
                    compiled.burst_count =
                        std::min(static_cast<std::uint32_t>(spawn.bursts.size()), MAX_COMPILED_BURSTS);
                    for (std::uint32_t i = 0; i < compiled.burst_count; ++i)
                    {
                        compiled.burst_time[i] = spawn.bursts[i].time;
                        compiled.burst_amount[i] = spawn.bursts[i].count;
                    }
                }

                /** @brief Packs the birth-volume parameters. */
                static void compile_shape(const ShapeModule& shape, CompiledEmitter& compiled)
                {
                    compiled.shape = shape.shape;
                    compiled.shape_radius = shape.radius;
                    compiled.shape_cone_angle = shape.cone_angle_radians;
                    compiled.shape_arc = shape.arc_radians;
                    compiled.shape_box_half_extents[0] = static_cast<float>(shape.box_half_extents.x);
                    compiled.shape_box_half_extents[1] = static_cast<float>(shape.box_half_extents.y);
                    compiled.shape_box_half_extents[2] = static_cast<float>(shape.box_half_extents.z);
                    compiled.shape_flags = 0;
                    if (shape.emit_from_shell)
                        compiled.shape_flags |= SHAPE_EMIT_FROM_SHELL;
                }

                /** @brief Packs the per-particle birth ranges. */
                static void compile_init(const InitModule& init, CompiledEmitter& compiled)
                {
                    compiled.lifetime_min = init.lifetime_min;
                    compiled.lifetime_max = std::max(init.lifetime_max, init.lifetime_min);
                    compiled.speed_min = init.speed_min;
                    compiled.speed_max = std::max(init.speed_max, init.speed_min);
                    compiled.size_min = init.size_min;
                    compiled.size_max = std::max(init.size_max, init.size_min);
                    compiled.rotation_min = init.rotation_min;
                    compiled.rotation_max = std::max(init.rotation_max, init.rotation_min);
                    compiled.angular_velocity_min = init.angular_velocity_min;
                    compiled.angular_velocity_max =
                        std::max(init.angular_velocity_max, init.angular_velocity_min);
                    compiled.color[0] = static_cast<float>(init.color.x);
                    compiled.color[1] = static_cast<float>(init.color.y);
                    compiled.color[2] = static_cast<float>(init.color.z);
                }

                /** @brief Sets the update-module bitfield, scalar forces, and baked-LUT offsets. */
                static void compile_update(const EmitterDescriptor& descriptor,
                                           CompiledEmitter& compiled, CompiledEffect& effect)
                {
                    compiled.update_flags = 0;
                    if (descriptor.gravity.enabled)
                    {
                        compiled.update_flags |= UPDATE_GRAVITY;
                        compiled.gravity[0] = static_cast<float>(descriptor.gravity.acceleration.x);
                        compiled.gravity[1] = static_cast<float>(descriptor.gravity.acceleration.y);
                        compiled.gravity[2] = static_cast<float>(descriptor.gravity.acceleration.z);
                    }
                    if (descriptor.drag.enabled)
                    {
                        compiled.update_flags |= UPDATE_DRAG;
                        compiled.drag_coefficient = descriptor.drag.coefficient;
                    }
                    if (descriptor.turbulence.enabled)
                    {
                        compiled.update_flags |= UPDATE_TURBULENCE;
                        compiled.turbulence_frequency = descriptor.turbulence.frequency;
                        compiled.turbulence_amplitude = descriptor.turbulence.amplitude;
                    }
                    if (descriptor.size_over_life.enabled && !descriptor.size_over_life.curve.empty())
                    {
                        compiled.update_flags |= UPDATE_SIZE_OVER_LIFE;
                        compiled.size_curve_lut = bake_curve(descriptor.size_over_life.curve, effect);
                    }
                    if (descriptor.color_over_life.enabled)
                    {
                        compiled.update_flags |= UPDATE_COLOR_OVER_LIFE;
                        compiled.color_gradient_lut =
                            bake_gradient(descriptor.color_over_life.gradient, effect);
                    }
                }

                /** @brief Packs the render settings. */
                static void compile_render(const RenderModule& render, CompiledEmitter& compiled)
                {
                    compiled.blend = render.blend;
                    compiled.sort = render.sort;
                    compiled.alignment = render.alignment;
                    compiled.render_flags = 0;
                    if (render.soft_particles)
                        compiled.render_flags |= RENDER_SOFT;
                    if (render.lit)
                        compiled.render_flags |= RENDER_LIT;
                    compiled.soft_fade_distance = render.soft_fade_distance;
                    compiled.texture = render.texture;
                    compiled.flipbook_rows = std::max(render.flipbook_rows, 1u);
                    compiled.flipbook_columns = std::max(render.flipbook_columns, 1u);
                }

                /**
                 * @brief Bakes a curve to a new atlas row and returns its offset.
                 * @param curve  The curve to bake.
                 * @param effect The compiled effect whose curve atlas grows by one row.
                 * @return The new row's offset.
                 */
                static std::int32_t bake_curve(const AnimationCurve& curve, CompiledEffect& effect)
                {
                    const std::int32_t offset =
                        static_cast<std::int32_t>(effect.curve_luts.size() / CURVE_LUT_WIDTH);
                    const std::size_t base = effect.curve_luts.size();
                    effect.curve_luts.resize(base + CURVE_LUT_WIDTH);
                    curve.bake(&effect.curve_luts[base], CURVE_LUT_WIDTH, 1.0f);
                    return offset;
                }

                /**
                 * @brief Bakes a gradient to a new atlas row and returns its offset.
                 * @param gradient The gradient to bake.
                 * @param effect   The compiled effect whose gradient atlas grows by one row.
                 * @return The new row's offset.
                 */
                static std::int32_t bake_gradient(const ColorGradient& gradient, CompiledEffect& effect)
                {
                    const std::int32_t offset = static_cast<std::int32_t>(
                        effect.gradient_luts.size() / (GRADIENT_LUT_WIDTH * 4));
                    const std::size_t base = effect.gradient_luts.size();
                    effect.gradient_luts.resize(base + GRADIENT_LUT_WIDTH * 4);
                    gradient.bake(&effect.gradient_luts[base], GRADIENT_LUT_WIDTH);
                    return offset;
                }
        };
    } // namespace Vfx
} // namespace SushiEngine
