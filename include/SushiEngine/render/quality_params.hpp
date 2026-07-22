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
