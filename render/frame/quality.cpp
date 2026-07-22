/**************************************************************************/
/* quality.cpp                                                            */
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

#include <SushiEngine/render/quality_params.hpp>

#include <algorithm>

namespace SushiEngine
{
    namespace Render
    {
        namespace
        {
            /**
             * @brief Scales an authored shadow-map side and snaps it to a usable size.
             *
             * A shadow map wants an even, cache-friendly side; an arbitrary fraction of
             * the authored value would give neither. The scaled value is rounded to a
             * multiple of 256 and clamped to a sane range so a tier can never ask the
             * atlas for a two-texel map or one no device will allocate.
             */
            std::uint32_t scale_shadow_resolution(std::uint32_t authored, float factor) noexcept
            {
                const float scaled = static_cast<float>(authored) * factor;
                std::uint32_t rounded =
                    (static_cast<std::uint32_t>(scaled + 128.0f) / 256u) * 256u;
                if (rounded < 512u)
                    rounded = 512u;
                if (rounded > 4096u)
                    rounded = 4096u;
                return rounded;
            }

            /** @brief Clamps a tap count to the sixteen-entry Poisson disc the sampler walks. */
            std::uint32_t clamp_taps(std::uint32_t taps) noexcept
            {
                return std::min(std::max(taps, 1u), 16u);
            }
        } // namespace

        ResolvedQuality resolve_quality(const RenderSettings& authored) noexcept
        {
            ResolvedQuality out;
            out.settings = authored;
            RenderSettings& s = out.settings;
            QualityParams& q = out.params;

            switch (authored.quality)
            {
                case RenderQuality::Low:
                    // The cheap floor: half-resolution cascades, at most two of them, a
                    // short contact march, the coarsest VRS, and base PBR only.
                    s.shadows.cascade_count = std::min(authored.shadows.cascade_count, 2u);
                    s.shadows.resolution = scale_shadow_resolution(authored.shadows.resolution, 0.5f);
                    s.shadows.contact_steps = std::max(authored.shadows.contact_steps / 3u, 4u);
                    s.lights.max_lights = std::min(authored.lights.max_lights, 64u);
                    s.lights.shadow_atlas_size =
                        std::min(authored.lights.shadow_atlas_size, 1024u);
                    s.lights.max_shadow_casters = std::min(authored.lights.max_shadow_casters, 4u);
                    s.lights.max_decals = std::min(authored.lights.max_decals, 16u);
                    s.gtao.slices = std::min(authored.gtao.slices, 2u);
                    s.gtao.steps = std::min(authored.gtao.steps, 4u);
                    q.shadow_filter_taps = 6;
                    q.shadow_blocker_taps = 3;
                    q.cloud_primary_steps_near = 48;
                    q.cloud_primary_steps_far = 16;
                    q.cloud_light_steps = 3;
                    q.vrs_max_coarse_axis = 4;
                    q.lobe_anisotropy = false;
                    q.lobe_clearcoat = false;
                    q.lobe_sheen = false;
                    q.lobe_transmission = false;
                    break;

                case RenderQuality::Medium:
                    s.shadows.cascade_count = std::min(authored.shadows.cascade_count, 3u);
                    s.shadows.resolution =
                        scale_shadow_resolution(authored.shadows.resolution, 0.75f);
                    s.shadows.contact_steps = std::max(authored.shadows.contact_steps * 2u / 3u, 6u);
                    s.lights.max_lights = std::min(authored.lights.max_lights, 128u);
                    s.lights.shadow_atlas_size =
                        std::min(authored.lights.shadow_atlas_size, 2048u);
                    s.lights.max_shadow_casters = std::min(authored.lights.max_shadow_casters, 8u);
                    s.lights.max_decals = std::min(authored.lights.max_decals, 32u);
                    s.gtao.slices = std::min(authored.gtao.slices, 3u);
                    s.gtao.steps = std::min(authored.gtao.steps, 5u);
                    q.shadow_filter_taps = 10;
                    q.shadow_blocker_taps = 5;
                    q.cloud_primary_steps_near = 72;
                    q.cloud_primary_steps_far = 24;
                    q.cloud_light_steps = 4;
                    q.vrs_max_coarse_axis = 2;
                    q.lobe_anisotropy = false;
                    q.lobe_clearcoat = true;
                    q.lobe_sheen = false;
                    q.lobe_transmission = false;
                    break;

                case RenderQuality::High:
                    // The baseline: the authored settings are used verbatim, so the
                    // default tier renders exactly what the host asked for. Only the
                    // knobs with no home in RenderSettings are filled here.
                    q.shadow_filter_taps = 16;
                    q.shadow_blocker_taps = 8;
                    q.cloud_primary_steps_near = 96;
                    q.cloud_primary_steps_far = 32;
                    q.cloud_light_steps = 5;
                    q.vrs_max_coarse_axis = 2;
                    q.lobe_anisotropy = true;
                    q.lobe_clearcoat = true;
                    q.lobe_sheen = true;
                    q.lobe_transmission = true;
                    break;

                case RenderQuality::Ultra:
                    // The expensive half pushed up: full-resolution cascades (never below
                    // 2k), a long contact march, no VRS coarsening, every lobe. The
                    // traced-shadow branch keys off Ultra separately in the shadow pass.
                    s.shadows.resolution = std::max(authored.shadows.resolution, 2048u);
                    // A floor, not a ceiling: Ultra must never resolve a shorter march than
                    // whatever the authored (High-baseline) value already asks for.
                    s.shadows.contact_steps = std::max(authored.shadows.contact_steps, 24u);
                    // A floor, like the shadow fields: Ultra never resolves fewer lights
                    // than the authored baseline already asks for.
                    s.lights.max_lights = std::max(authored.lights.max_lights, 1024u);
                    s.lights.shadow_atlas_size =
                        std::max(authored.lights.shadow_atlas_size, 4096u);
                    s.lights.max_shadow_casters = std::max(authored.lights.max_shadow_casters, 16u);
                    s.lights.max_decals = std::max(authored.lights.max_decals, 128u);
                    s.gtao.slices = std::max(authored.gtao.slices, 4u);
                    s.gtao.steps = std::max(authored.gtao.steps, 8u);
                    q.shadow_filter_taps = 16;
                    q.shadow_blocker_taps = 8;
                    q.cloud_primary_steps_near = 128;
                    q.cloud_primary_steps_far = 48;
                    q.cloud_light_steps = 6;
                    q.vrs_max_coarse_axis = 1;
                    q.lobe_anisotropy = true;
                    q.lobe_clearcoat = true;
                    q.lobe_sheen = true;
                    q.lobe_transmission = true;
                    break;
            }

            q.shadow_filter_taps = clamp_taps(q.shadow_filter_taps);
            q.shadow_blocker_taps = clamp_taps(q.shadow_blocker_taps);
            return out;
        }
    } // namespace Render
} // namespace SushiEngine
