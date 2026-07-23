/**************************************************************************/
/* cloth_buffers.hpp                                                      */
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
 * @file cloth_buffers.hpp
 * @brief The per-view, per-frame buffers soft bodies are triangulated into and drawn from.
 *
 * Soft-body geometry is simulated on the host and changes every frame, so unlike the mesh
 * registry's assets it cannot be shared between views or between frames in flight. Each
 * view owns one of these, with a set of buffers per frame slot, and the buffers only ever
 * grow.
 *
 * The triangulation is GPU-driven (Phase 10 item 6): the host uploads only the particle
 * positions (in a strand-local frame, with the camera-relative origin recorded per strand
 * so planet-scale precision survives), and a compute pass writes the drawable vertex and
 * index buffers from them. So the host does no per-vertex float work — @ref prepare packs
 * positions and computes the layout, @ref ClothPass fills the geometry, and the opaque pass
 * draws it.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>
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
             * @brief One soft-body grid's slice of the shared vertex and index buffers.
             *
             * Everything the compute pass needs to triangulate the strand and the opaque pass
             * needs to draw it: where its vertices and indices begin, how big its grid is, its
             * camera-relative origin (added back to the local positions on the GPU), and which
             * entry of the frame's strand list it came from (for its material and pick id).
             */
            struct ClothStrandRange
            {
                std::uint32_t rows = 0;
                std::uint32_t cols = 0;
                std::uint32_t base_vertex = 0;
                std::uint32_t base_index = 0;
                std::uint32_t vertex_count = 0;
                std::uint32_t index_count = 0;
                std::uint32_t strand_index = 0; /**< Index into the frame's strand list. */
                float origin[3] = {0.0f, 0.0f, 0.0f}; /**< Camera-relative strand origin. */
            };

            /**
             * @brief Growable soft-body position, vertex, and index buffers, per frame slot.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class ClothBuffers
            {
                public:
                    /**
                     * @brief Allocates the per-slot buffer sets.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     */
                    ClothBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~ClothBuffers();

                    ClothBuffers(const ClothBuffers&) = delete;
                    ClothBuffers& operator=(const ClothBuffers&) = delete;

                    /**
                     * @brief Packs the frame's particle positions and lays out the geometry.
                     *
                     * Computes each strand's vertex/index slice, packs the particle positions
                     * into this slot's host-visible position buffer in a strand-local frame
                     * (camera-relative origin recorded per strand), and grows the device-local
                     * vertex and index buffers the compute pass will fill. No per-vertex float
                     * work happens here — that is the compute pass's job.
                     *
                     * @param slot         The frame slot being recorded.
                     * @param strands      The frame's soft-body grids.
                     * @param strand_count Number of entries in @p strands.
                     * @param eye          Camera world position, for the camera-relative origin.
                     */
                    void prepare(std::uint32_t slot, const ClothStrandView* strands,
                                 std::size_t strand_count, const double eye[3]);

                    /** @brief Whether the frame packed any drawable soft-body geometry. */
                    bool empty() const noexcept { return ranges_.empty(); }

                    /** @brief The strand slices this frame, one per drawable grid. */
                    const std::vector<ClothStrandRange>& ranges() const noexcept { return ranges_; }

                    /** @brief The drawable mesh: the device-local vertex and index buffers. */
                    const Mesh& mesh(std::uint32_t slot) const noexcept { return meshes_[slot]; }

                    /** @brief This slot's host-visible packed-position buffer. */
                    VkBuffer positions(std::uint32_t slot) const noexcept;

                    /** @brief Bytes of the position buffer the descriptor must expose. */
                    VkDeviceSize positions_range() const noexcept;

                private:
                    /** @brief A VMA-backed buffer and the capacity it was allocated at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage,
                              bool host_visible);
                    void destroy(Allocation& target);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Allocation> positions_; /**< Host-visible packed positions, per slot. */
                    std::vector<Allocation> vertices_;  /**< Device-local, compute-written, per slot. */
                    std::vector<Allocation> indices_;   /**< Device-local, compute-written, per slot. */
                    std::vector<Mesh> meshes_;
                    std::vector<ClothStrandRange> ranges_;
                    std::vector<float> packed_positions_; /**< Scratch for the current frame's pack. */
                    std::uint32_t total_vertices_ = 0;
            };
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
