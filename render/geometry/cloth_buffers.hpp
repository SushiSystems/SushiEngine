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
 * @brief The per-view, per-frame vertex buffers soft bodies are drawn from.
 *
 * Soft-body geometry is simulated on the host and changes every frame, so unlike the
 * mesh registry's assets it cannot be shared between views or between frames in
 * flight. Each view owns one of these, with a buffer pair per frame slot, and the
 * buffers only ever grow.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

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
             * @brief Growable soft-body vertex and index buffers, one pair per frame slot.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class ClothBuffers
            {
                public:
                    /**
                     * @brief Allocates the per-slot buffer pairs.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     */
                    ClothBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~ClothBuffers();

                    ClothBuffers(const ClothBuffers&) = delete;
                    ClothBuffers& operator=(const ClothBuffers&) = delete;

                    /**
                     * @brief Rewrites one slot's buffers, growing them if needed.
                     * @param slot         The frame slot being recorded.
                     * @param vertices     Triangulated soft-body vertices.
                     * @param vertex_count Number of entries in @p vertices.
                     * @param indices      Triangle indices into @p vertices.
                     * @param index_count  Number of entries in @p indices.
                     * @return The updated mesh, or an empty mesh if nothing was given.
                     */
                    const Mesh& upload(std::uint32_t slot, const MeshVertex* vertices,
                                       std::size_t vertex_count, const std::uint32_t* indices,
                                       std::size_t index_count);

                private:
                    /** @brief A VMA-backed buffer and the capacity it was allocated at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage);
                    void destroy(Allocation& target);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Allocation> vertices_;
                    std::vector<Allocation> indices_;
                    std::vector<Mesh> meshes_;
            };
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
