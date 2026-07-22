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
 * A single cascade of diffuse irradiance probes on a regular lattice around the camera.
 * Each probe stores the same nine second-order SH coefficients the IBL build already
 * produces for the whole environment, so the shading pass blends probes exactly the way
 * it evaluates the global set — one polynomial in the surface normal — and the only new
 * work per pixel is a trilinear gather of eight neighbours. The grid snaps to a world
 * lattice so the probes sit at fixed world points and never swim as the camera moves;
 * only their camera-relative origin shifts, which is what @ref configure_probe_volume
 * computes. @ref ProbeVolumeConfig is the std140 block the shader reads to locate the
 * lattice, kept as flat arrays so the C++ and GLSL packings can never disagree.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            /** @brief Probe count along the horizontal axes of the single cascade. */
            constexpr std::int32_t PROBE_COUNT_HORIZONTAL = 32;

            /** @brief Probe count along the vertical axis (thin: outdoor GI is mostly lateral). */
            constexpr std::int32_t PROBE_COUNT_VERTICAL = 8;

            /** @brief Total probes in the cascade. */
            constexpr std::int32_t PROBE_COUNT_TOTAL =
                PROBE_COUNT_HORIZONTAL * PROBE_COUNT_VERTICAL * PROBE_COUNT_HORIZONTAL;

            /** @brief Metres between adjacent probes on every axis. */
            constexpr float PROBE_SPACING_METRES = 4.0f;

            /** @brief Second-order SH coefficients stored per probe (RGB in @c vec4.rgb). */
            constexpr std::int32_t PROBE_SH_COEFFICIENTS = 9;

            /**
             * @brief The std140 block locating the probe lattice, mirroring @c GiProbeVolume.
             *
             * Flat 16-byte-aligned arrays so the GLSL side reads the identical layout. All
             * positions are camera-relative, like every other scene quantity, so the block
             * rebases with the camera at planetary distances.
             */
            struct ProbeVolumeConfig
            {
                float origin_enabled[4];  /**< xyz camera-relative origin of probe (0,0,0); w = enabled. */
                float spacing_bias[4];    /**< xyz spacing in metres; w = normal bias in metres. */
                float intensity[4];       /**< x = indirect intensity; yzw spare. */
                std::int32_t counts[4];   /**< xyz probe counts per axis; w = total probe count. */
            };

            static_assert(sizeof(ProbeVolumeConfig) == 64,
                          "ProbeVolumeConfig must match its std140 GLSL mirror");

            /** @brief Bytes the probe SH storage buffer occupies (nine vec4 per probe). */
            constexpr std::uint32_t probe_sh_buffer_bytes() noexcept
            {
                return static_cast<std::uint32_t>(PROBE_COUNT_TOTAL) * PROBE_SH_COEFFICIENTS *
                       4u * sizeof(float);
            }

            /**
             * @brief Fills a probe config for this frame, snapping the lattice to the world.
             *
             * The lattice is anchored to the nearest spacing-multiple world point to the
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
