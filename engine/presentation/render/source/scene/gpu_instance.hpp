/**************************************************************************/
/* gpu_instance.hpp                                                       */
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
 * @file gpu_instance.hpp
 * @brief The per-object records the GPU-driven geometry path culls and draws from.
 *
 * When GPU-driven rendering is on (Phase 10), the CPU stops issuing one draw per
 * instance. Instead it packs every drawable into a @ref GpuInstance array, groups them
 * by mesh into @ref GpuDrawBucket ranges, and hands the whole set to the GPU: a compute
 * pass tests each instance against the frustum, its screen coverage, and last frame's
 * occlusion pyramid, and writes a compacted survivor list plus one
 * `VkDrawIndexedIndirectCommand` per bucket whose instance count it decides. The draw
 * passes then bind each bucket's geometry once and issue a single indirect draw. So the
 * CPU cost is flat in the number of distinct meshes, not the number of instances.
 *
 * Everything a draw used to carry in the 128-byte push constant that varies per instance
 * — the transform, the material and motion indices, the picking id — moves into
 * @ref GpuInstance, because an indirect draw carries no per-draw push constant. The
 * layouts here are the C++ mirror of the `std430` blocks in `cull.comp` and
 * `mesh_gpu.vert`; keep the two in lockstep. GPU data is 32-bit regardless of the
 * engine's Scalar precision, so transforms and bounds are explicit float arrays that a
 * double build narrows to float exactly here, at the upload boundary.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            /**
             * @brief One drawable's GPU record: transform, bounds, and SoA indices.
             *
             * 96 bytes, `std430`. @c model is camera-relative (the eye subtracted in
             * double before the float cast, exactly as the CPU push path did) with the
             * primitive scale already baked in, so the cull and vertex shaders read the
             * same geometry the classic path drew. @c bounding_sphere is camera-relative
             * against the same current eye: xyz the centre, w the radius, the shape the
             * frustum and occlusion tests need.
             */
            struct GpuInstance
            {
                float model[16];          /**< Camera-relative object-to-world, scale baked. */
                float bounding_sphere[4]; /**< xyz = camera-relative centre, w = radius. */
                std::uint32_t material_index; /**< Index into the frame's material array. */
                std::uint32_t motion_index;   /**< Index into the frame's motion array. */
                std::uint32_t entity_id;      /**< Picking id written to the id target. */
                std::uint32_t bucket_index;   /**< Which draw bucket (mesh) this instance draws with. */
            };

            static_assert(sizeof(GpuInstance) == 96,
                          "GpuInstance must match the std430 block in cull.comp / mesh_gpu.vert");

            /**
             * @brief CPU-side metadata for one draw bucket: a unique mesh drawn this frame.
             *
             * Not a GPU structure — it stays on the host, where the draw passes read the
             * mesh's own vertex and index buffers to bind and the indirect-command slot to
             * dispatch. All instances sharing a mesh land in one bucket and one indirect
             * draw; @c candidate_base is where this bucket's slice of the instance and
             * compacted arrays begins (a prefix sum of the earlier buckets' counts).
             */
            struct GpuDrawBucket
            {
                VkBuffer vertices = VK_NULL_HANDLE; /**< The mesh's vertex buffer to bind. */
                VkBuffer indices = VK_NULL_HANDLE;  /**< The mesh's index buffer to bind. */
                std::uint32_t index_count = 0;      /**< Indices the indirect draw submits. */
                std::uint32_t candidate_base = 0;   /**< Offset into the instance/compacted arrays. */
                std::uint32_t candidate_count = 0;  /**< Instances this bucket may draw (pre-cull). */
            };

            /**
             * @brief Per-bucket metadata the cull shader reads, one `uvec4` each (`std430`).
             *
             * x = the mesh's index count (written into the indirect command), y = the
             * bucket's @c candidate_base (where its survivors compact to). z, w spare.
             */
            struct GpuBucketMeta
            {
                std::uint32_t index_count;
                std::uint32_t candidate_base;
                std::uint32_t reserved0;
                std::uint32_t reserved1;
            };

            static_assert(sizeof(GpuBucketMeta) == 16,
                          "GpuBucketMeta must match the std430 uvec4 in cull.comp");

            /**
             * @brief The push constant a GPU-driven draw hands its vertex shader.
             *
             * The indirect command carries no push data, so the one value that still
             * differs per bucket — where the bucket's compacted survivors start — is pushed
             * on the CPU before that bucket's single indirect draw. @c gl_InstanceIndex then
             * indexes the compacted list at @c candidate_base + instance.
             */
            struct GpuDrawPush
            {
                std::uint32_t candidate_base;
                std::uint32_t reserved;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
