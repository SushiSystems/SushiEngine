/**************************************************************************/
/* cluster_config.hpp                                                     */
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
 * @file cluster_config.hpp
 * @brief The froxel cluster grid's fixed dimensions and the packed GPU light layout.
 *
 * The grid is the shared infrastructure the whole light engine (and later the aerial-
 * perspective froxels and volumetric fog) addresses through: a view-frustum-aligned
 * @c CLUSTER_X × @c CLUSTER_Y × @c CLUSTER_Z froxel volume, logarithmically sliced in
 * depth so slice thickness tracks the eye's depth precision. The build pass writes,
 * for every cluster, how many lights touch it and which ones; the shading pass reads
 * that back per pixel.
 *
 * These constants are duplicated verbatim in the shaders (`cluster_build.comp`,
 * `clustered_lighting.glsl`) because GLSL cannot include this header — the grid shape
 * is compile-time on both sides. Any change here must change both shader copies; the
 * per-frame quantities that *do* vary (the depth range, the light count, the screen
 * tile size) travel through @ref LightClusterUniforms instead.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        namespace Lighting
        {
            /** @brief Froxel columns across the screen width. */
            constexpr std::uint32_t CLUSTER_X = 16;

            /** @brief Froxel rows down the screen height. */
            constexpr std::uint32_t CLUSTER_Y = 9;

            /** @brief Logarithmic depth slices from near to the cull far plane. */
            constexpr std::uint32_t CLUSTER_Z = 24;

            /** @brief Total froxels in the grid. */
            constexpr std::uint32_t CLUSTER_COUNT = CLUSTER_X * CLUSTER_Y * CLUSTER_Z;

            /**
             * @brief Lights a single cluster may reference.
             *
             * The index list gives each cluster a fixed slot of this many entries
             * (cluster @c i owns indices @c [i*CAP, i*CAP+count)), so the build pass
             * needs no global atomic and no clear — it writes an authoritative count and
             * up to this many indices, race-free. A cluster that overflows drops the
             * surplus (bounded, documented cost), which a later compacted-atomic build
             * can remove when froxel occupancy demands it.
             */
            constexpr std::uint32_t MAX_LIGHTS_PER_CLUSTER = 64;

            /** @brief Entries in the per-frame cluster→light index list. */
            constexpr std::uint32_t LIGHT_INDEX_COUNT = CLUSTER_COUNT * MAX_LIGHTS_PER_CLUSTER;

            /** @brief 32-bit words one @c GpuLight occupies (four vec4 lanes). */
            constexpr std::uint32_t GPU_LIGHT_FLOATS = 16;

            /** @brief Bytes one packed light occupies in the storage buffer (std430). */
            constexpr std::uint32_t GPU_LIGHT_BYTES = GPU_LIGHT_FLOATS * sizeof(float);

            /**
             * @brief Decals a single cluster may reference.
             *
             * Fewer than lights: decals are large authored volumes, so a cluster rarely
             * overlaps many. Same fixed-slot scheme as the light index list (no atomic, no
             * clear); a cluster that overflows drops the surplus.
             */
            constexpr std::uint32_t MAX_DECALS_PER_CLUSTER = 16;

            /** @brief Entries in the per-frame cluster→decal index list. */
            constexpr std::uint32_t DECAL_INDEX_COUNT = CLUSTER_COUNT * MAX_DECALS_PER_CLUSTER;

            /** @brief 32-bit words one @c GpuDecal occupies (six vec4 lanes). */
            constexpr std::uint32_t GPU_DECAL_FLOATS = 24;

            /** @brief Bytes one packed decal occupies in the storage buffer (std430). */
            constexpr std::uint32_t GPU_DECAL_BYTES = GPU_DECAL_FLOATS * sizeof(float);
        } // namespace Lighting
    } // namespace Render
} // namespace SushiEngine
