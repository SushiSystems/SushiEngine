/**************************************************************************/
/* render_settings.hpp                                                    */
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
 * @file render_settings.hpp
 * @brief How the renderer is asked to trade fidelity against frame time.
 *
 * Deliberately separate from Environment: that describes the world being drawn and is
 * authored by the simulation, while this describes the machinery drawing it and is
 * authored by the host. A scene view reads these once per frame; nothing here changes
 * what the scene *is*, only how many samples and how much resolution it is resolved
 * with. Plain trivially-copyable data, no Vulkan.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief The tier gating the expensive half of every pass.
         *
         * The red line between realism and performance moves by dropping a tier, never
         * by disabling a feature ad hoc — so a pass reads the tier and scales its own
         * work rather than the host reaching in and switching individual effects off.
         */
        enum class RenderQuality : std::uint32_t
        {
            Low,
            Medium,
            High,
            Ultra,
        };

        /**
         * @brief Which anti-aliasing resolve the frame ends with.
         *
         * @c Temporal is the default: at these resolutions it costs less than MSAA,
         * removes shimmer MSAA cannot (specular, alpha-tested, and ray-marched sky
         * detail), and is what makes rendering below the output resolution viable.
         * @c Fxaa is the cheap fallback for the low tier and for a host that cannot
         * tolerate any history at all.
         */
        enum class AntiAliasingMode : std::uint32_t
        {
            None,
            Fxaa,
            Temporal,
        };

        /**
         * @brief How the internal render resolution reaches the output resolution.
         *
         * @c Temporal reuses the anti-aliasing history: because every frame is jittered
         * to a different sub-pixel position, accumulating render-resolution samples into
         * the larger output grid recovers detail a spatial filter cannot. A vendor
         * upscaler lands here as a new value and a new pass, never as a change to the
         * existing ones.
         */
        enum class UpscaleMode : std::uint32_t
        {
            None,
            Temporal,
        };

        /** @brief How much history the temporal resolve keeps, and how it is filtered. */
        struct TemporalSettings
        {
            /** @brief History weight where nothing moves; the still-image quality knob. */
            float feedback_still = 0.97f;

            /** @brief History weight under full-screen motion; lower kills ghosting. */
            float feedback_moving = 0.88f;

            /** @brief Strength of the post-resolve sharpen that offsets temporal blur, [0, 1]. */
            float sharpness = 0.5f;

            /** @brief Scales the sub-pixel jitter; below 1 trades aliasing for stability. */
            float jitter_scale = 1.0f;

            /**
             * @brief Clamp the history to the neighbourhood of the current pixel.
             *
             * The one mechanism that keeps a reprojected sample from a surface that is
             * no longer there out of the result. Off only for debugging.
             */
            bool clamp_history = true;
        };

        /**
         * @brief How the sun's shadows are resolved.
         *
         * Three mechanisms at three scales, because no single one covers the range a
         * planet-scale scene spans: cascades for the metres-to-kilometres body of the
         * shadow, a screen-space march for the centimetres of contact the cascades
         * cannot resolve, and a traced ray on the top tier for the exact answer.
         */
        struct ShadowSettings
        {
            bool enabled = true;

            /**
             * @brief Cascades the view frustum is split across, 1 to 4.
             *
             * More cascades put more texels where the eye is without enlarging any one
             * map; the cost is one extra pass over the shadow-casting geometry each.
             */
            std::uint32_t cascade_count = 4;

            /** @brief Side of one cascade's shadow map in texels. */
            std::uint32_t resolution = 2048;

            /** @brief Distance from the eye the cascades cover, metres. */
            float distance = 400.0f;

            /**
             * @brief Blend between logarithmic and uniform cascade splits, [0, 1].
             *
             * A logarithmic split is optimal for perspective but starves the far
             * cascades; a uniform one wastes the near ones. The practical split is the
             * mix, and this is where it sits.
             */
            float split_blend = 0.85f;

            /** @brief Depth bias along the surface normal, in shadow texels. */
            float normal_bias = 1.6f;

            /** @brief Constant depth bias, in shadow-map depth units. */
            float depth_bias = 0.0008f;

            /**
             * @brief Smallest filter radius, in shadow texels.
             *
             * The penumbra is derived from how far the blocker stands above the receiver,
             * so a surface touching its caster would filter over nothing at all; this is
             * the floor that keeps even a hard contact from reading as a stair-step of
             * shadow-map texels.
             */
            float filter_radius = 2.0f;

            /** @brief Largest filter radius the penumbra estimate may reach, in texels. */
            float max_filter_radius = 24.0f;

            /**
             * @brief Multiplies the sun's real angular size when sizing the penumbra.
             *
             * One is physically correct: the sun subtends about half a degree, so its
             * shadows are genuinely sharp near contact and soften slowly with height.
             * Above one exaggerates that softening, which is what most rendered scenes
             * actually want — a real sun shadow at these distances lands within a couple
             * of shadow-map texels and reads as aliasing rather than as softness.
             */
            float softness = 2.5f;

            /** @brief Fraction of a cascade's far edge blended into the next, [0, 0.5]. */
            float cascade_blend = 0.1f;

            /** @brief Recover small-scale contact the cascade resolution cannot resolve. */
            bool contact_shadows = true;

            /** @brief How far the screen-space march reaches from the surface, metres. */
            float contact_distance = 0.35f;

            /** @brief Steps the screen-space march takes; more is smoother and dearer. */
            std::uint32_t contact_steps = 12;

            /**
             * @brief Trace the sun ray instead of sampling a cascade, on the Ultra tier.
             *
             * Ignored unless the device offers ray queries and an acceleration structure
             * was built, in which case the cascades remain the fallback for whatever the
             * structure does not contain.
             */
            bool ray_traced = false;
        };

        /**
         * @brief The automatic governor holding the frame budget under load.
         *
         * Drives the internal render scale from the measured GPU frame time, so a heavy
         * view loses resolution instead of frame rate; the temporal upscale is what
         * makes the change hard to see.
         */
        struct DynamicResolutionSettings
        {
            bool enabled = false;

            /** @brief GPU milliseconds the controller steers the frame toward. */
            float target_milliseconds = 8.0f;

            /** @brief Lowest internal scale the controller may fall to, per axis. */
            float minimum_scale = 0.6f;

            /** @brief Highest internal scale the controller may rise to, per axis. */
            float maximum_scale = 1.0f;
        };

        /**
         * @brief Shading at a reduced rate where the reduction is invisible.
         *
         * A per-tile rate image is derived from the previous frame's luminance contrast
         * and this frame's velocity: flat interiors and fast-moving regions shade
         * coarsely, edges and still detail shade per pixel. Ignored on a device without
         * attachment-based fragment shading rate.
         */
        struct VariableRateShadingSettings
        {
            bool enabled = false;

            /** @brief Local luminance contrast below which a tile may shade coarsely. */
            float luminance_threshold = 0.08f;

            /** @brief Screen motion, in UV per frame, above which a tile may shade coarsely. */
            float velocity_threshold = 0.02f;
        };

        /**
         * @brief Everything the host asks of the renderer's performance/fidelity trade.
         *
         * Defaults describe the project's target: High tier, temporal anti-aliasing at
         * full resolution, no automatic resolution scaling until a host opts in.
         */
        struct RenderSettings
        {
            RenderQuality quality = RenderQuality::High;
            AntiAliasingMode anti_aliasing = AntiAliasingMode::Temporal;
            UpscaleMode upscale = UpscaleMode::Temporal;

            /**
             * @brief Manual internal render scale per axis, [0.5, 1].
             *
             * The floor the dynamic controller starts from and the value used verbatim
             * when the controller is off.
             */
            float render_scale = 1.0f;

            ShadowSettings shadows;
            TemporalSettings temporal;
            DynamicResolutionSettings dynamic_resolution;
            VariableRateShadingSettings variable_rate_shading;
        };
    } // namespace Render
} // namespace SushiEngine
