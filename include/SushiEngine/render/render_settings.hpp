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
         * the larger output grid recovers detail a spatial filter cannot. The vendor
         * values name an upscaler backend behind the one @c IUpscaler interface; a
         * backend the build does not carry resolves back to @c Temporal rather than
         * failing, so a project may request one unconditionally.
         */
        enum class UpscaleMode : std::uint32_t
        {
            None,
            Temporal,
            Fsr,  /**< AMD FidelityFX Super Resolution 3.1, vendor-agnostic. */
            Dlss, /**< NVIDIA DLSS through Streamline. */
            Xess, /**< Intel XeSS, DP4a fallback on non-Intel. */
        };

        /**
         * @brief How finished frames are handed to the display.
         *
         * @c Vsync is the tear-free default (FIFO, always available). @c Mailbox keeps
         * the latest frame and drops the rest — tear-free but with the latency of an
         * uncapped renderer, at the cost of drawing frames that are never shown.
         * @c Immediate presents as soon as the frame is ready and may tear; it is the
         * lowest-latency option and the honest one to measure raw frame cost against.
         */
        enum class PresentMode : std::uint32_t
        {
            Vsync,
            Mailbox,
            Immediate,
        };

        /**
         * @brief How the display-transform pass derives its exposure multiplier.
         *
         * @c Manual takes the scene-authored exposure (@c Environment::exposure) scaled by
         * a user EV compensation — the value the renderer has always used, so it is the
         * default and reproduces the prior look exactly. @c Automatic replaces it with an
         * eye-adaptation value the auto-exposure pass derives from a luminance histogram of
         * the resolved frame, so sun, sky, and emissive intensities read correctly across a
         * wide range without hand-tuning.
         */
        enum class ExposureMode : std::uint32_t
        {
            Manual,
            Automatic,
        };

        /**
         * @brief Which tone curve maps the linear HDR scene onto the display.
         *
         * @c AgX is the default: it desaturates toward white as values climb, preserving
         * hue through highlights where a filmic curve skews it, and is the current industry
         * consensus. @c ACES is the prior Narkowicz filmic curve, kept for parity. @c Neutral
         * is the Khronos PBR Neutral transform, which holds material colour with minimal
         * shift for look-dev and product rendering.
         */
        enum class TonemapOperator : std::uint32_t
        {
            AgX,
            ACES,
            Neutral,
        };

        /**
         * @brief Physically-based auto-exposure (eye adaptation) parameters.
         *
         * The auto-exposure pass builds a log-luminance histogram of the resolved frame,
         * takes the average of its central mass (extremes discarded), and adapts a stored
         * exposure toward the value that maps that average onto @c key middle grey. Only
         * consulted when @c PostProcessSettings::exposure_mode is @c Automatic.
         */
        struct AutoExposureSettings
        {
            float min_ev = -6.0f;      /**< Darkest scene EV100 the adaptation will resolve to. */
            float max_ev = 16.0f;      /**< Brightest scene EV100 the adaptation will resolve to. */
            float compensation = 0.0f; /**< Exposure compensation added on top, in stops (EV). */
            float speed_up = 3.0f;     /**< Adaptation rate toward a brighter scene, stops/second. */
            float speed_down = 1.0f;   /**< Adaptation rate toward a darker scene, stops/second. */

            /**
             * @brief The metered luminance the exposure maps to middle grey.
             *
             * 0.18 is the photographic key; lowering it exposes darker, raising it brighter.
             */
            float key = 0.18f;

            /** @brief Fraction of the darkest histogram mass ignored as shadow noise, [0,1). */
            float low_percent = 0.5f;

            /** @brief Fraction of the brightest histogram mass ignored as highlights, [0,1). */
            float high_percent = 0.9f;
        };

        /**
         * @brief Energy-conserving bloom (light bleed) parameters.
         *
         * A progressive mip pyramid down- then up-sampled so a bright pixel scatters into a
         * wide, stable halo. Threshold-free by default (the whole HDR image scatters, which
         * is what conserves energy); @c threshold lifts a soft knee for a stylised, contrasty
         * bloom when wanted. @c intensity is the linear blend of the scattered pyramid back
         * into the scene at composite.
         */
        struct BloomSettings
        {
            bool enabled = true;          /**< Whether the pyramid is built and composited. */
            float intensity = 0.05f;      /**< Blend of the bloom pyramid into the scene, [0,1]. */
            float threshold = 0.0f;       /**< Soft-knee luminance floor; 0 = threshold-free. */
            float threshold_knee = 0.5f;  /**< Width of the soft knee above the threshold. */
        };

        /**
         * @brief Colour grade applied inside the display-transform pass, before the tone map.
         *
         * One fullscreen pass carries the whole grade → tone map → encode chain, so grading
         * costs no extra pass. White balance shifts the neutral point; lift/gamma/gain are the
         * classic three-way shadows/mids/highlights control; contrast and saturation are the
         * final global shape. A 3D-LUT slot is reserved (@c lut_enabled) for a look-up-table
         * grade; the texture binding lands with the LUT asset path.
         */
        struct ColorGradeSettings
        {
            float temperature = 0.0f; /**< White-balance temperature shift, [-1,1] (warm positive). */
            float tint = 0.0f;        /**< White-balance green/magenta tint, [-1,1]. */
            float contrast = 1.0f;    /**< Global contrast about middle grey; 1 = neutral. */
            float saturation = 1.0f;  /**< Global saturation; 0 = greyscale, 1 = neutral. */
            float lift[3] = {0.0f, 0.0f, 0.0f};  /**< Additive shadow offset per channel. */
            float gamma[3] = {1.0f, 1.0f, 1.0f}; /**< Midtone power per channel. */
            float gain[3] = {1.0f, 1.0f, 1.0f};  /**< Highlight multiplier per channel. */
            bool lut_enabled = false;            /**< Reserved 3D-LUT grade slot; off today. */
        };

        /**
         * @brief Gather-based depth-of-field (bokeh) parameters.
         *
         * A physical thin-lens circle of confusion from the focus distance and aperture, its
         * radius bounded so the gather cost stays fixed. Tier-gated off on the low tiers and
         * off by default: it is a look choice, not a fidelity default.
         */
        struct DepthOfFieldSettings
        {
            bool enabled = false;         /**< Whether the DoF gather runs. */
            float focus_distance = 10.0f; /**< Distance in metres held in perfect focus. */
            float focus_range = 2.0f;     /**< Metres around the focus plane still sharp. */
            float aperture = 2.8f;        /**< f-number; smaller opens the aperture and blurs more. */
            float max_radius = 6.0f;      /**< Circle-of-confusion ceiling in pixels. */
        };

        /**
         * @brief Per-pixel velocity motion blur parameters.
         *
         * Gathers along the shipped velocity target so both camera and object motion smear.
         * Tier-gated and off by default.
         */
        struct MotionBlurSettings
        {
            bool enabled = false;      /**< Whether the velocity gather runs. */
            float intensity = 0.5f;    /**< Scales the sampled velocity; 0 = no blur. */
            std::uint32_t samples = 8; /**< Taps gathered along the velocity vector. */
        };

        /**
         * @brief The whole post-processing and display-transform stack's parameters.
         *
         * Every field is data the passes read; no pass names the editor and the editor names
         * no pass (the Post-Process window writes this block, the passes consume it). Defaults
         * describe the project target: manual exposure reproducing the prior look, AgX tone
         * mapping, a gentle energy-conserving bloom, a neutral grade, and the optional
         * cinematic effects (DoF, motion blur) off until authored.
         */
        struct PostProcessSettings
        {
            ExposureMode exposure_mode = ExposureMode::Manual;
            float exposure_compensation = 0.0f; /**< Manual-mode EV compensation, in stops. */
            TonemapOperator tonemap = TonemapOperator::AgX;

            AutoExposureSettings auto_exposure;
            BloomSettings bloom;
            ColorGradeSettings grade;
            DepthOfFieldSettings depth_of_field;
            MotionBlurSettings motion_blur;

            float vignette = 0.25f;             /**< Corner darkening strength, 0 = off. */
            float chromatic_aberration = 0.0f;  /**< Edge colour-fringe width in pixels, 0 = off. */
            float film_grain = 0.0f;            /**< Linear-space grain intensity, 0 = off. */
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
         * @brief The clustered Forward+ punctual-light engine's budget.
         *
         * The froxel grid's dimensions are fixed in the renderer (@c cluster_config.hpp);
         * what the host authors — and the tier scales — is how many lights the scene may
         * carry and how far the grid reaches. @c max_lights is the ceiling the light
         * buffer is packed to (a scene with more drops its surplus), and
         * @c cluster_far_distance is the view distance the depth slices span: lights and
         * fragments beyond it fold into the last slice, so it trades cull precision at
         * distance for a shorter, denser near field where lights actually cluster.
         */
        struct LightEngineSettings
        {
            std::uint32_t max_lights = 256;         /**< Punctual lights packed per frame (High baseline). */
            float cluster_far_distance = 2000.0f;   /**< View metres the froxel grid's depth spans. */

            /**
             * @brief Side of the shared punctual shadow atlas, in texels (High baseline).
             *
             * The atlas is a 4×4 grid of equal tiles, so a shadow-casting spot light gets
             * a tile of @c shadow_atlas_size / 4 texels. Zero disables punctual shadows.
             */
            std::uint32_t shadow_atlas_size = 2048;

            /** @brief Shadow-casting spot lights that may claim an atlas tile per frame. */
            std::uint32_t max_shadow_casters = 16;

            /** @brief Projected decals culled into the froxel grid per frame (High baseline). */
            std::uint32_t max_decals = 64;

            /**
             * @brief Shadow the lights the atlas had no tile for, by tracing the GI field.
             *
             * The atlas can only hold so many casters; beyond that a light used to be shaded
             * fully unshadowed. With this on, each pixel instead samples a few of those
             * lights in proportion to what they are worth to it and marches the global
             * illumination distance field toward each for visibility, letting the temporal
             * resolve average the rest. Needs GI to be on (the field is built there) and the
             * tier to permit it; without either, the prior behaviour stands.
             */
            bool stochastic_shadows = true;

            /** @brief How far one stochastic shadow ray marches before giving up, metres. */
            float stochastic_distance = 40.0f;

            /**
             * @brief Penumbra width of a stochastic shadow, as an apparent light size.
             *
             * Larger blurs the shadow edge wider — the distance field's own soft-shadow
             * term, which costs nothing extra because the march already knows how close it
             * passed to a surface.
             */
            float stochastic_softness = 8.0f;
        };

        /**
         * @brief Ground-truth ambient occlusion's budget.
         *
         * GTAO darkens the creases and contacts the flat ambient/IBL term cannot see. It is
         * a per-pixel screen-space cost, so the tier scales how many horizon slices and steps
         * it marches; @c radius is the world reach of the occlusion and @c intensity /
         * @c power shape how strongly it darkens. Zero @c enabled drops the pass to a cleared
         * unoccluded target that costs nothing to sample.
         */
        struct GtaoSettings
        {
            bool enabled = true;         /**< Whether the horizon march runs at all. */
            float radius = 1.0f;         /**< World metres the occlusion reaches. */
            float intensity = 1.0f;      /**< Blend of the AO into full effect, [0,1]. */
            float power = 1.5f;          /**< Contrast applied to the visibility. */
            std::uint32_t slices = 3;    /**< Rotated horizon slices (High baseline). */
            std::uint32_t steps = 6;     /**< Horizon steps per slice side (High baseline). */
        };

        /**
         * @brief Screen-space reflections' budget.
         *
         * SSR traces the scene's own colour into glossy surfaces through the hierarchical-Z
         * pyramid, falling back to the IBL/probe environment where a ray leaves the screen.
         * @c enabled also gates the hi-Z pyramid build the trace depends on — the same
         * pyramid Phase 10's occlusion culling will reuse. The reflection trace itself lands
         * in a later increment; the pyramid is the shared foundation.
         */
        struct SsrSettings
        {
            bool enabled = true;           /**< Build the hi-Z pyramid (and, later, trace SSR). */
            std::uint32_t max_steps = 64;  /**< Ray-march steps before giving up (High baseline). */
            float thickness = 0.5f;        /**< View-space depth a hit is accepted within, metres. */
            float roughness_cutoff = 0.6f; /**< Above this roughness the surface uses IBL only. */
            float intensity = 1.0f;        /**< Blend of the traced reflection into the surface. */
        };

        /**
         * @brief The GPU-driven geometry path's controls.
         *
         * When enabled and the tier permits it (Medium and up), the renderer stops issuing
         * one draw per instance: a compute pass frustum-, coverage-, and occlusion-culls
         * every instance on the GPU and writes one multi-draw-indirect command per mesh, so
         * the CPU cost is flat in the number of distinct meshes rather than instances.
         * @c occlusion tests against the previous frame's depth pyramid; @c min_screen_diameter
         * is the LOD gate that drops instances too small on screen to matter. The debug fields
         * surface the cull to the editor without changing what ships.
         */
        struct GpuCullingSettings
        {
            bool enabled = true;             /**< Take the GPU-driven path when the tier permits. */
            bool occlusion = true;           /**< Cull instances behind last frame's depth. */
            bool frustum = true;             /**< Cull instances outside the view frustum. */
            /** @brief Drop an instance whose projected diameter is below this many pixels. */
            float min_screen_diameter = 2.0f;
            /**
             * @brief Freeze the cull frustum at its current pose for debugging.
             *
             * Reserved for the editor's cull debug view: holds the test frustum so the effect
             * of culling is visible as the camera moves past it. Off in every shipping frame.
             */
            bool freeze = false;
            /** @brief Whether the editor should read back and show the per-frame cull counts. */
            bool show_statistics = false;
        };

        /**
         * @brief How the frame reaches the GPU and the display.
         *
         * Delivery, not fidelity: nothing here changes a pixel, only how much of the
         * device is kept busy and how long a finished frame waits. @c async_compute lets
         * the render graph place its flagged compute passes on a second queue so they
         * overlap the graphics work they do not depend on; @c frames_in_flight is how
         * many frames the CPU may run ahead (deeper hides a hitch, shallower cuts input
         * latency and memory); @c present_mode is the swapchain's pacing.
         */
        struct FrameDeliverySettings
        {
            /**
             * @brief Submit graph-flagged compute passes on a dedicated compute queue.
             *
             * Ignored on a device with no compute queue family distinct from graphics,
             * and gated by the tier — so this only *permits* the second queue.
             */
            bool async_compute = true;

            /** @brief Frames the CPU may record ahead of the GPU, 2 or 3. */
            std::uint32_t frames_in_flight = 2;

            PresentMode present_mode = PresentMode::Vsync;
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
            LightEngineSettings lights;
            GtaoSettings gtao;
            SsrSettings ssr;
            PostProcessSettings post;
            GpuCullingSettings gpu_culling;
            FrameDeliverySettings delivery;
        };
    } // namespace Render
} // namespace SushiEngine
