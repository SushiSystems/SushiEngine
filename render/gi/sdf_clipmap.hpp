/**************************************************************************/
/* sdf_clipmap.hpp                                                        */
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
 * @file sdf_clipmap.hpp
 * @brief The camera-centred scene distance field the SDF probe tracer marches.
 *
 * A single coarse clipmap: a cube 3D texture around the camera holding, per voxel, the
 * distance to the nearest scene surface and that surface's albedo. It is rebuilt each
 * frame from the scene's analytic primitives (box, sphere, cylinder) — the same shapes
 * the geometry pass draws — so the probe tracer can sphere-trace it for occlusion and a
 * single coloured bounce without touching per-triangle geometry. Imported meshes fold in
 * later as baked distance bricks sampled into the same clipmap. Everything is
 * camera-relative, so the field rebases with the camera at planetary distances. The two
 * std140 blocks below are the single source of truth their GLSL mirrors follow.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            struct SceneDrawList;
        }

        namespace Gi
        {
            /** @brief Per-axis voxel resolution of the scene distance clipmap. */
            constexpr std::int32_t SDF_CLIPMAP_RESOLUTION = 64;

            /** @brief World extent the clipmap cube spans, metres. Sized to the probe cascade. */
            constexpr float SDF_CLIPMAP_EXTENT_METRES = 128.0f;

            /** @brief Maximum analytic primitives folded into the clipmap per frame. */
            constexpr std::int32_t MAX_SDF_PRIMITIVES = 256;

            /** @brief Per-axis voxel resolution of an imported mesh's baked SDF brick. */
            constexpr std::int32_t SDF_BRICK_RESOLUTION = 32;

            /** @brief Voxels in one brick. */
            constexpr std::int32_t SDF_BRICK_VOXELS =
                SDF_BRICK_RESOLUTION * SDF_BRICK_RESOLUTION * SDF_BRICK_RESOLUTION;

            /** @brief Maximum mesh bricks the shared atlas holds. */
            constexpr std::int32_t MAX_SDF_BRICKS = 64;

            /** @brief Maximum imported-mesh instances folded into the clipmap per frame. */
            constexpr std::int32_t MAX_SDF_MESH_INSTANCES = 128;

            /** @brief The primitive a @ref SdfPrimitive evaluates, matching MeshKind. */
            enum class SdfPrimitiveKind : std::int32_t
            {
                Box = 0,
                Sphere = 1,
                Cylinder = 2,
            };

            /**
             * @brief One analytic scene primitive, camera-relative, as the populate pass reads it.
             *
             * Axis-aligned: rotation is dropped, which a low-frequency distance field for GI
             * tolerates and which imported-mesh bricks later cover for arbitrary geometry.
             */
            struct SdfPrimitive
            {
                float center_kind[4]; /**< xyz camera-relative centre; w = SdfPrimitiveKind. */
                float extent[4];      /**< xyz half-extents (box) or radius (x) + half-height (y). */
                float albedo[4];      /**< rgb surface albedo for the bounce; w spare. */
            };

            static_assert(sizeof(SdfPrimitive) == 48,
                          "SdfPrimitive must match its std140 GLSL mirror");

            /**
             * @brief One imported-mesh instance's placement into the clipmap, camera-relative.
             *
             * The populate pass transforms a voxel's camera-relative world point into the
             * mesh's local frame with @c inv_model, tests it against the local AABB, samples
             * the mesh's baked brick in the shared atlas at @c slot, and scales the local
             * distance back to world by @c world_scale. Rotation is honoured here (unlike the
             * analytic primitives) because the brick lives in the mesh's own frame.
             */
            struct SdfMeshInstance
            {
                float inv_model[16];  /**< Camera-relative world -> mesh local. */
                float aabb_min[4];    /**< xyz local AABB min; w = brick slot. */
                float aabb_max[4];    /**< xyz local AABB max; w = local-to-world distance scale. */
                float albedo[4];      /**< rgb bounce albedo; w spare. */
            };

            static_assert(sizeof(SdfMeshInstance) == 112,
                          "SdfMeshInstance must match its std140 GLSL mirror");

            /**
             * @brief The block locating the clipmap in space, mirroring @c SdfClipmapConfig.
             */
            struct SdfClipmapConfig
            {
                float origin_voxel[4];      /**< xyz camera-relative min corner; w = voxel size, metres. */
                std::int32_t resolution[4]; /**< xyz voxel counts; w = live primitive count. */
                std::int32_t extra[4];      /**< x = mesh-instance count; yzw spare. */
            };

            static_assert(sizeof(SdfClipmapConfig) == 48,
                          "SdfClipmapConfig must match its std140 GLSL mirror");

            /**
             * @brief Fills the clipmap config, snapping the cube to a voxel lattice.
             *
             * Anchoring the min corner to a voxel multiple keeps the field from swimming as
             * the camera moves — a voxel holds the same world region until the camera crosses
             * a voxel, exactly like the probe lattice.
             *
             * @param eye             The camera world position in metres.
             * @param primitive_count Primitives populated this frame.
             * @param out             Receives the filled block.
             */
            void configure_sdf_clipmap(const double eye[3], std::int32_t primitive_count,
                                       SdfClipmapConfig& out) noexcept;

            /**
             * @brief Extracts the frame's axis-aligned analytic primitives, camera-relative.
             *
             * Reads each primitive instance's transform and shape parameters, drops rotation,
             * and writes a camera-relative box/sphere/cylinder with its albedo. Imported-mesh
             * instances are skipped here (they fold in as baked bricks later).
             *
             * @param draws The frame's draw list.
             * @param eye   The camera world position the primitives are rebased against.
             * @param out   Receives up to @ref MAX_SDF_PRIMITIVES primitives.
             * @param max   Capacity of @p out.
             * @return The number of primitives written.
             */
            std::int32_t build_sdf_primitives(const Frame::SceneDrawList& draws, const double eye[3],
                                              SdfPrimitive* out, std::int32_t max) noexcept;

            /**
             * @brief Fills one mesh instance from its transform and its cached brick placement.
             *
             * Builds the camera-relative world-to-local matrix (the inverse of the model with
             * its translation rebased against the eye) and records the atlas slot, the local
             * AABB the brick spans, the albedo, and the local-to-world distance scale.
             *
             * @param model    The instance's object-to-world transform.
             * @param eye       The camera world position.
             * @param aabb_min  The brick's local AABB minimum (three floats).
             * @param aabb_max  The brick's local AABB maximum (three floats).
             * @param slot      The instance's brick slot in the shared atlas.
             * @param albedo    The bounce albedo (three floats).
             * @param out       Receives the filled instance.
             */
            void fill_sdf_mesh_instance(const Mat4& model, const double eye[3],
                                        const float aabb_min[3], const float aabb_max[3],
                                        std::int32_t slot, const float albedo[3],
                                        SdfMeshInstance& out) noexcept;
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
