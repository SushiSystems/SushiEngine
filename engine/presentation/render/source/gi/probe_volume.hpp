/**************************************************************************/
/* probe_volume.hpp                                                       */
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
 * @file probe_volume.hpp
 * @brief The camera-relative irradiance probe clipmap: its dimensions and GPU config.
 *
 * A nested cascade of diffuse irradiance probe grids around the camera. Each cascade is a
 * regular lattice of the same probe count but twice the spacing of the one inside it, so
 * the finest gives dense near-field GI and the coarser ones reach far out — a shading point
 * reads the finest cascade that contains it and falls back outward. Each probe stores the
 * same nine second-order SH coefficients the IBL build already produces for the whole
 * environment, so the shading pass blends probes exactly the way it evaluates the global
 * set — one polynomial in the surface normal — and the only new work per pixel is a
 * trilinear gather of eight neighbours in the chosen cascade. Every grid snaps to its own
 * world lattice so the probes sit at fixed world points and never swim as the camera moves;
 * only their camera-relative origin shifts, which is what @ref configure_probe_volume
 * computes. @ref ProbeVolumeConfig is the std140 block the shader reads to locate the
 * cascades, kept as flat arrays so the C++ and GLSL packings can never disagree.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            /** @brief Probe count along the horizontal axes of each cascade. */
            constexpr std::int32_t PROBE_COUNT_HORIZONTAL = 32;

            /** @brief Probe count along the vertical axis (thin: outdoor GI is mostly lateral). */
            constexpr std::int32_t PROBE_COUNT_VERTICAL = 8;

            /** @brief Probes in one cascade. */
            constexpr std::int32_t PROBE_COUNT_TOTAL =
                PROBE_COUNT_HORIZONTAL * PROBE_COUNT_VERTICAL * PROBE_COUNT_HORIZONTAL;

            /** @brief Number of nested cascades: finest, and coarser ones for distant coverage. */
            constexpr std::int32_t GI_NUM_CASCADES = 3;

            /** @brief Probes across every cascade — the size the SH grid and relight span. */
            constexpr std::int32_t PROBE_COUNT_ALL_CASCADES = PROBE_COUNT_TOTAL * GI_NUM_CASCADES;

            /** @brief Metres between adjacent probes in the finest cascade; each coarser doubles it. */
            constexpr float PROBE_SPACING_METRES = 4.0f;

            /** @brief Second-order SH coefficients stored per probe (RGB in @c vec4.rgb). */
            constexpr std::int32_t PROBE_SH_COEFFICIENTS = 9;

            /**
             * @brief The std140 block locating the probe cascades, mirroring @c GiProbeVolume.
             *
             * Flat 16-byte-aligned arrays so the GLSL side reads the identical layout. All
             * positions are camera-relative, like every other scene quantity, so the block
             * rebases with the camera at planetary distances. The per-cascade origin and
             * spacing are packed one @c vec4 each: xyz origin, w spacing.
             */
            struct ProbeVolumeConfig
            {
                float params[4];          /**< x enabled; y indirect intensity; z normal bias metres; w cascade count. */
                std::int32_t counts[4];   /**< xyz probe counts per axis; w probes per cascade. */
                float cascade_origin[GI_NUM_CASCADES][4]; /**< Per cascade: xyz camera-relative origin, w spacing metres. */
            };

            static_assert(sizeof(ProbeVolumeConfig) == 32 + GI_NUM_CASCADES * 16,
                          "ProbeVolumeConfig must match its std140 GLSL mirror");

            /** @brief Spacing of cascade @p cascade, doubling outward from the finest. */
            constexpr float probe_cascade_spacing(std::int32_t cascade) noexcept
            {
                return PROBE_SPACING_METRES * static_cast<float>(1 << cascade);
            }

            /** @brief Bytes the probe SH storage buffer occupies (nine vec4 per probe, all cascades). */
            constexpr std::uint32_t probe_sh_buffer_bytes() noexcept
            {
                return static_cast<std::uint32_t>(PROBE_COUNT_ALL_CASCADES) * PROBE_SH_COEFFICIENTS *
                       4u * sizeof(float);
            }

            /**
             * @brief Fills a probe config for this frame, snapping each cascade to the world.
             *
             * Every cascade is anchored to the nearest multiple of its own spacing to the
             * camera, so probes occupy fixed world positions frame to frame; only their
             * camera-relative origin moves. When disabled the block is still filled (enable
             * flag cleared) so the shading pass has a valid binding to read and fall back on.
             *
             * @param eye         The camera world position in metres.
             * @param enabled     Whether probe GI is active this frame (tier and author gate).
             * @param intensity   Multiplier applied to the gathered indirect diffuse.
             * @param normal_bias Metres the sample point is pushed along the normal, leak guard.
             * @param out         Receives the filled block.
             */
            void configure_probe_volume(const double eye[3], bool enabled, float intensity,
                                        float normal_bias, ProbeVolumeConfig& out) noexcept;
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
