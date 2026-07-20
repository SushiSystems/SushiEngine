/**************************************************************************/
/* cloth_buffers.cpp                                                      */
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

#include "geometry/cloth_buffers.hpp"

#include <cstring>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            ClothBuffers::ClothBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                vertices_.resize(frame_slots);
                indices_.resize(frame_slots);
                meshes_.resize(frame_slots);
            }

            ClothBuffers::~ClothBuffers()
            {
                for (Allocation& allocation : vertices_)
                    destroy(allocation);
                for (Allocation& allocation : indices_)
                    destroy(allocation);
            }

            void ClothBuffers::grow(Allocation& target, VkDeviceSize bytes,
                                    VkBufferUsageFlags usage)
            {
                if (bytes == 0 || bytes <= target.capacity)
                    return;
                destroy(target);

                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = bytes;
                buffer_info.usage = usage;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(cloth)");
                target.mapped = info.pMappedData;
                target.capacity = bytes;
            }

            void ClothBuffers::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

            const Mesh& ClothBuffers::upload(std::uint32_t slot, const MeshVertex* vertices,
                                             std::size_t vertex_count,
                                             const std::uint32_t* indices,
                                             std::size_t index_count)
            {
                Mesh& mesh = meshes_[slot];
                mesh = Mesh{};
                if (vertices == nullptr || indices == nullptr || vertex_count == 0 ||
                    index_count == 0)
                    return mesh;

                Allocation& vertex_buffer = vertices_[slot];
                Allocation& index_buffer = indices_[slot];
                grow(vertex_buffer, vertex_count * sizeof(MeshVertex),
                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                grow(index_buffer, index_count * sizeof(std::uint32_t),
                     VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                std::memcpy(vertex_buffer.mapped, vertices, vertex_count * sizeof(MeshVertex));
                std::memcpy(index_buffer.mapped, indices, index_count * sizeof(std::uint32_t));

                mesh = Mesh{vertex_buffer.buffer, index_buffer.buffer,
                            static_cast<std::uint32_t>(vertex_count),
                            static_cast<std::uint32_t>(index_count)};
                return mesh;
            }
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
