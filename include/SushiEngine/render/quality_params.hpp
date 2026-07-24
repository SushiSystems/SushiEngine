/**************************************************************************/
/* quality_params.hpp                                                     */
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
 * @file quality_params.hpp
 * @brief The one place the quality tier turns into concrete per-pass parameters.
 *
 * @c RenderQuality names a tier; it does not say what a tier *does*. This resolver is
 * where the red line between realism and performance is actually drawn: it maps the
 * tier to the tap counts, march budgets, and feature toggles every pass reads, so a
 * pass consumes *resolved parameters* and never branches on the raw enum. Keeping the
 * policy in one function is what stops the tier meaning one thing in the shadow pass
 * and something incompatible in the cloud pass.
 *
 * The contract with the host is deliberate: the authored @c RenderSettings *are* the
 * High-tier baseline. High resolves to exactly what the host asked for; a lower tier
 * scales that request down for performance; Ultra pushes the expensive half up. So the
 * editor's per-field sliders still mean something — they set the target the tier scales
 * from — and dropping a tier moves the whole red line at once rather than disabling
 * features one at a time. Plain trivially-copyable data, no Vulkan.
 */

#include <cstdint>

#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        /**
         * @brief The tier-resolved knobs that have no home in @c RenderSettings.
         *
         * Everything here is a value a pass reads instead of a hard-coded shader
         * literal or the raw tier. Values that *do* live in @c RenderSettings (shadow
         * resolution, contact march, VRS thresholds) are scaled in the effective
         * settings instead — see @c ResolvedQuality — so there is exactly one source of
         * truth per quantity.
         */
        struct QualityParams
        {
            /**
             * @brief Percentage-closer soft-shadow filter taps, per shaded pixel.
             *
             * The penumbra filter's cost is linear in this; the radius alone does not
             * cheapen it. Bounded by the sixteen-entry Poisson disc the sampler walks.
             */
            std::uint32_t shadow_filter_taps = 16;

            /** @brief Blocker-search taps that size the penumbra before the filter runs. */
            std::uint32_t shadow_blocker_taps = 8;

            /**
             * @brief The volumetric cloud march's step envelope near the ground.
             *
             * The altitude term interpolates the real step count between @c near and
             * @c far, so this is the ceiling the tier caps ground-level cloud cost at.
             */
            std::uint32_t cloud_primary_steps_near = 96;

            /** @brief The cloud march's step floor at the top of the shell / from space. */
            std::uint32_t cloud_primary_steps_far = 32;

            /** @brief Steps the cloud march takes toward the sun to self-shadow the deck. */
            std::uint32_t cloud_light_steps = 5;

            /**
             * @brief The coarsest per-axis rate the variable-rate mask may emit.
             *
             * One means full rate — no coarsening, the quality-first answer; two permits
             * up to a two-wide tile; four permits the aggressive four-wide tile. Capped
             * against the device's own maximum where the two disagree.
             */
            std::uint32_t vrs_max_coarse_axis = 2;

            /** @brief Whether the anisotropic specular lobe is evaluated at this tier. */
            bool lobe_anisotropy = true;

            /** @brief Whether the clearcoat lobe is evaluated at this tier. */
            bool lobe_clearcoat = true;

            /** @brief Whether the sheen lobe is evaluated at this tier. */
            bool lobe_sheen = true;

            /** @brief Whether the transmission lobe is evaluated at this tier. */
            bool lobe_transmission = true;

            /**
             * @brief Whether volumetric fog is built at this tier.
             *
             * The fog froxel volume is cheap, but its in-scatter is the least essential
             * atmosphere term, so the lowest tier drops it; every other tier keeps the
             * author's @c FogParams::enabled decision.
             */
            bool volumetric_fog = true;

            /**
             * @brief Whether probe-volume GI is relit at this tier.
             *
             * Global illumination is a High/Ultra feature: the lower tiers keep the flat
             * environment ambient. Gated with the author's @c GiParams::enabled, so this
             * only permits GI — it never forces it on.
             */
            bool probe_gi = false;

            /**
             * @brief Whether bloom is built and composited at this tier.
             *
             * The mip-pyramid scatter is cheap and near-universal, so only the lowest tier
             * drops it; every other tier keeps the author's @c BloomSettings::enabled choice.
             */
            bool bloom = true;

            /**
             * @brief Whether depth of field runs at this tier.
             *
             * A gather-based bokeh is a High/Ultra cinematic effect; the lower tiers force it
             * off. Gated with the author's @c DepthOfFieldSettings::enabled, so this only
             * permits DoF — it never forces it on.
             */
            bool depth_of_field = false;

            /**
             * @brief Whether motion blur runs at this tier.
             *
             * Like DoF, permitted only on the upper tiers and never forced on; the author's
             * @c MotionBlurSettings::enabled still decides.
             */
            bool motion_blur = false;

            /**
             * @brief Whether the GPU-driven geometry path is taken at this tier.
             *
             * GPU instancing, per-mesh multi-draw-indirect, and GPU frustum/occlusion/LOD
             * culling remove the per-instance CPU cost of dense scenes. The lowest tier keeps
             * the classic one-draw-per-instance path — its scenes are the simplest and its
             * devices least likely to want extra compute passes — while every other tier takes
             * the GPU-driven path. Gated further by the author's @c GpuCullingSettings::enabled
             * and by the bindless heap being present, so this only *permits* it.
             */
            bool gpu_driven = true;

            /**
             * @brief Whether the mesh-shader (meshlet) draw path is taken at this tier.
             *
             * The Ultra crown only: a task shader culls each mesh's clusters and a mesh shader
             * emits the survivors, finer than whole-object culling. Gated further by the device
             * offering VK_EXT_mesh_shader; where it does not, the same geometry draws through
             * the GPU-driven or classic path, so no hardware is left unable to render — the
             * meshlet path is purely additive.
             */
            bool meshlets = false;

            /**
             * @brief Whether flagged compute passes may run on the async compute queue.
             *
             * Overlapping the LUT, cluster-build, and occlusion work with the graphics
             * queue's shadow and depth passes recovers frame time that neither queue was
             * using; the cost is a second command buffer and a cross-queue semaphore per
             * frame. The lowest tier keeps everything on one queue — its devices are the
             * least likely to have a queue family to spare and the most likely to lose
             * more to the extra submission than the overlap returns. Gated further by the
             * author's @c FrameDeliverySettings::async_compute and by the device actually
             * offering a compute family distinct from graphics, so this only permits it.
             */
            bool async_compute = true;

            /**
             * @brief The most skinned characters evaluated and skinned at full rate.
             *
             * The animation system's headline budget knob (design §6.6): instances beyond
             * this either drop to a throttled update rate (A2) or are not skinned. The floor
             * tier keeps a small crowd; Ultra allows a full one. A1 reads it as a hard cap on
             * skinned instances; the LOD/throttle ladder that softens it lands in A2.
             */
            std::uint32_t max_skinned_instances = 128;

            /**
             * @brief Levels the skinned bone-LOD ladder is biased coarser by at this tier.
             *
             * Added to each instance's distance-derived LOD level, so a lower tier poses
             * fewer joints per character at the same distance. Zero on High/Ultra (the
             * authored ladder is used as-is); positive on the cheaper tiers.
             */
            std::uint32_t bone_lod_bias = 0;

            /**
             * @brief Bone influences blended per skinned vertex (4 or 8).
             *
             * Four covers all but the most deforming rigs; Ultra spends the second set of
             * four on shoulders and hips. The skin vertex stream and the skinning dispatch
             * both read this so the two never disagree.
             */
            std::uint32_t animation_influences = 4;

            /**
             * @brief Lights each pixel samples and shadow-marches beyond the atlas budget.
             *
             * The cost of stochastic light visibility is set by this and not by how many
             * lights the scene holds, which is the whole point: the shadowed-light ceiling
             * stops being a memory budget and becomes a sample budget. Zero switches the
             * path off, restoring "unshadowed beyond the atlas" — the floor tiers, which
             * have no GI field to march anyway.
             */
            std::uint32_t stochastic_light_samples = 0;

            /**
             * @brief Whether the GPU cosmetic particle path runs at this tier.
             *
             * The compute emit/simulate passes and the billboard draw are permitted on every
             * tier but the lowest, which drops cosmetic VFX entirely — its devices are the least
             * able to spare the compute and its scenes the least likely to demand spectacle. The
             * deterministic CPU particle path is unaffected; it is gameplay, not a quality knob.
             */
            bool gpu_particles = true;

            /**
             * @brief The shared particle pool's ceiling at this tier.
             *
             * The cosmetic pool is sized once, but this caps how many particles the emitters may
             * keep alive at a tier: Ultra allows a dense storm, the lower tiers a modest haze.
             * A budget the host scales spawn rates against, mirroring @c max_skinned_instances.
             */
            std::uint32_t max_particles = 1u << 18;

            /**
             * @brief Fixed sub-steps the particle integrator takes per frame at this tier.
             *
             * More sub-steps keep fast, turbulent particles stable at a cost linear in this;
             * one is the floor, the upper tiers spend more for smoother motion.
             */
            std::uint32_t particle_sim_substeps = 1;
        };

        /**
         * @brief The full result of resolving a tier: effective settings plus knobs.
         *
         * @c settings is the authored request with its tier-scaled fields overwritten,
         * and is what every pass and every uniform fill reads downstream; the authored
         * copy the host owns is left untouched so the next frame scales from the same
         * baseline. @c params is the rest.
         */
        struct ResolvedQuality
        {
            RenderSettings settings;
            QualityParams params;
        };

        /**
         * @brief Resolves the tier in @p authored into effective settings and knobs.
         *
         * Pure and side-effect free: same input, same output, every frame. High returns
         * @p authored unchanged (it is the baseline); Low and Medium scale the expensive
         * half down; Ultra pushes it up. Call once per frame before any pass reads the
         * settings, and once from the editor to display what a tier resolves to.
         *
         * @param authored The host's authored settings, taken as the High-tier baseline.
         * @return The effective settings every pass should read, and the extra knobs.
         */
        ResolvedQuality resolve_quality(const RenderSettings& authored) noexcept;
    } // namespace Render
} // namespace SushiEngine
