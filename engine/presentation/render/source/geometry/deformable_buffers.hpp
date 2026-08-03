/**************************************************************************/
/* deformable_buffers.hpp                                                 */
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
 * @file deformable_buffers.hpp
 * @brief The per-view, per-frame buffers host-simulated surfaces are shaded into and drawn from.
 *
 * Deformable geometry is simulated on the host and changes every frame, so unlike the mesh
 * registry's assets it cannot be shared between views or between frames in flight. Each view
 * owns one of these, with a set of buffers per frame slot, and the buffers only ever grow.
 *
 * Four buffers, and which of them is host-visible follows from how often its contents change:
 *
 * - **positions** change every frame, because that is what simulating means. Host-visible,
 *   repacked per frame, in a mesh-local frame with the camera-relative origin recorded per
 *   mesh so planet-scale precision survives.
 * - **indices** change only when the topology does. Host-visible, and doubling as the index
 *   buffer the draw binds — the indices are mesh-local and the draw supplies the vertex
 *   offset, so nothing has to rewrite them into a shared numbering.
 * - **adjacency** is a pure function of the indices, so it changes exactly as often. Its
 *   *contents* are cached (see @ref prepare) because building it is the one part of this that
 *   is more than a copy.
 * - **vertices** is the only device-local buffer, because it is the only one the GPU writes:
 *   the compute pass shades into it and the draw reads it back as a vertex stream.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/deformable_mesh.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "geometry/mesh_registry.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Geometry
        {
            /**
             * @brief One deformable mesh's slice of the shared buffers.
             *
             * Everything the compute pass needs to shade the mesh and the opaque pass needs to
             * draw it: where each of its four arrays begins, how long two of them are, its
             * camera-relative origin (added back to the local positions on the GPU), and which
             * entry of the frame's mesh list it came from (for its material and pick id).
             */
            struct DeformableMeshRange
            {
                std::uint32_t base_vertex = 0;  /**< First vertex in the shared position/vertex buffers. */
                std::uint32_t base_index = 0;   /**< First index in the shared index buffer. */
                std::uint32_t vertex_count = 0;
                std::uint32_t index_count = 0;
                std::uint32_t adjacency_range_base = 0;    /**< First uint of this mesh's range pairs. */
                std::uint32_t adjacency_triangle_base = 0; /**< First uint of this mesh's triangle list. */
                std::uint32_t view_index = 0;   /**< Index into the frame's deformable mesh list. */
                float origin[3] = {0.0f, 0.0f, 0.0f}; /**< Camera-relative mesh origin. */
            };

            /**
             * @brief Growable per-frame-slot buffers for host-simulated surfaces.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class DeformableBuffers
            {
                public:
                    /**
                     * @brief Allocates the per-slot buffer sets.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     */
                    DeformableBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~DeformableBuffers();

                    DeformableBuffers(const DeformableBuffers&) = delete;
                    DeformableBuffers& operator=(const DeformableBuffers&) = delete;

                    /**
                     * @brief Packs the frame's geometry and lays out the shared buffers.
                     *
                     * Computes each mesh's slices, packs positions (mesh-local, camera-relative
                     * origin recorded per mesh), copies the triangle lists, and packs the
                     * vertex-to-triangle adjacency the compute pass gathers normals through.
                     * No per-vertex float work happens here beyond the position narrowing that
                     * the precision split requires — the shading is the compute pass's job.
                     *
                     * The adjacency is cached per mesh across frames and rebuilt only when the
                     * cache key — the mesh's pick id, its topology revision, and its two counts
                     * — stops matching. A miss costs a rebuild and nothing else: the cache can
                     * make a frame slower, never wrong.
                     *
                     * @param slot       The frame slot being recorded.
                     * @param meshes     The frame's deformable surfaces.
                     * @param mesh_count Number of entries in @p meshes.
                     * @param eye        Camera world position, for the camera-relative origin.
                     */
                    void prepare(std::uint32_t slot, const DeformableMeshView* meshes,
                                 std::size_t mesh_count, const double eye[3]);

                    /** @brief Whether the frame packed any drawable deformable geometry. */
                    bool empty() const noexcept { return ranges_.empty(); }

                    /** @brief The mesh slices this frame, one per drawable surface. */
                    const std::vector<DeformableMeshRange>& ranges() const noexcept
                    {
                        return ranges_;
                    }

                    /** @brief The drawable mesh: the compute-written vertex buffer and the index buffer. */
                    const Mesh& mesh(std::uint32_t slot) const noexcept { return meshes_[slot]; }

                    /** @brief This slot's host-visible packed-position buffer. */
                    VkBuffer positions(std::uint32_t slot) const noexcept;

                    /** @brief This slot's host-visible index buffer, also read by the compute pass. */
                    VkBuffer indices(std::uint32_t slot) const noexcept;

                    /** @brief This slot's host-visible adjacency range-pair buffer. */
                    VkBuffer adjacency_ranges(std::uint32_t slot) const noexcept;

                    /** @brief This slot's host-visible adjacency triangle buffer. */
                    VkBuffer adjacency_triangles(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of the position buffer the descriptor must expose. */
                    VkDeviceSize positions_range() const noexcept;

                    /** @brief Bytes of the index buffer the descriptor must expose. */
                    VkDeviceSize indices_range() const noexcept;

                    /** @brief Bytes of the adjacency range buffer the descriptor must expose. */
                    VkDeviceSize adjacency_ranges_range() const noexcept;

                    /** @brief Bytes of the adjacency triangle buffer the descriptor must expose. */
                    VkDeviceSize adjacency_triangles_range() const noexcept;

                private:
                    /** @brief A VMA-backed buffer and the capacity it was allocated at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    /**
                     * @brief One mesh's cached adjacency and the key it was built for.
                     *
                     * Held in frame order rather than in a map: the frame's mesh list is
                     * near-always stable, so slot `i` near-always still holds mesh `i`'s
                     * table, and a reorder costs a rebuild instead of a hash per mesh.
                     */
                    struct TopologyCacheEntry
                    {
                        std::uint32_t id = 0;
                        std::uint64_t revision = 0;
                        std::uint32_t vertex_count = 0;
                        std::uint32_t index_count = 0;
                        bool valid = false;
                        VertexTriangleAdjacency adjacency;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage,
                              bool host_visible);
                    void destroy(Allocation& target);
                    void upload(Allocation& target, const void* source, std::size_t bytes,
                                VkBufferUsageFlags usage);

                    /** @brief Returns the adjacency for @p mesh, rebuilding it only on a key miss. */
                    const VertexTriangleAdjacency& adjacency_for(std::size_t cache_slot,
                                                                 const DeformableMeshView& mesh);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Allocation> positions_;  /**< Host-visible packed positions, per slot. */
                    std::vector<Allocation> indices_;    /**< Host-visible mesh-local triangle lists, per slot. */
                    std::vector<Allocation> adjacency_ranges_;    /**< Host-visible range pairs, per slot. */
                    std::vector<Allocation> adjacency_triangles_; /**< Host-visible triangle lists, per slot. */
                    std::vector<Allocation> vertices_;   /**< Device-local, compute-written, per slot. */
                    std::vector<Mesh> meshes_;
                    std::vector<DeformableMeshRange> ranges_;
                    std::vector<TopologyCacheEntry> topology_cache_;

                    std::vector<float> packed_positions_;
                    std::vector<std::uint32_t> packed_indices_;
                    std::vector<std::uint32_t> packed_adjacency_ranges_;
                    std::vector<std::uint32_t> packed_adjacency_triangles_;
                    std::uint32_t total_vertices_ = 0;
            };
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
