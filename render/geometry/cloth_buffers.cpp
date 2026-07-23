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
                positions_.resize(frame_slots);
                vertices_.resize(frame_slots);
                indices_.resize(frame_slots);
                meshes_.resize(frame_slots);
            }

            ClothBuffers::~ClothBuffers()
            {
                for (Allocation& allocation : positions_)
                    destroy(allocation);
                for (Allocation& allocation : vertices_)
                    destroy(allocation);
                for (Allocation& allocation : indices_)
                    destroy(allocation);
            }

            void ClothBuffers::grow(Allocation& target, VkDeviceSize bytes,
                                    VkBufferUsageFlags usage, bool host_visible)
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
                if (host_visible)
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(cloth)");
                target.mapped = host_visible ? info.pMappedData : nullptr;
                target.capacity = bytes;
            }

            void ClothBuffers::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

            void ClothBuffers::prepare(std::uint32_t slot, const ClothStrandView* strands,
                                       std::size_t strand_count, const double eye[3])
            {
                ranges_.clear();
                packed_positions_.clear();
                total_vertices_ = 0;
                meshes_[slot] = Mesh{};
                if (strands == nullptr || strand_count == 0)
                    return;

                std::uint32_t vertex_total = 0;
                std::uint32_t index_total = 0;
                for (std::size_t s = 0; s < strand_count; ++s)
                {
                    const ClothStrandView& strand = strands[s];
                    if (strand.rows < 2 || strand.cols < 2 || strand.vertices == nullptr)
                        continue;

                    const std::uint32_t vertices = strand.rows * strand.cols;
                    const std::uint32_t indices = (strand.rows - 1) * (strand.cols - 1) * 6;

                    ClothStrandRange range;
                    range.rows = strand.rows;
                    range.cols = strand.cols;
                    range.base_vertex = vertex_total;
                    range.base_index = index_total;
                    range.vertex_count = vertices;
                    range.index_count = indices;
                    range.strand_index = static_cast<std::uint32_t>(s);

                    // The strand origin is its first particle; the local positions packed
                    // below are each particle minus that origin, computed in double so a
                    // planet-scale absolute position never touches single precision. The
                    // origin itself is made camera-relative the same way, and the GPU adds
                    // the two back — exactly reproducing the per-vertex eye subtraction the
                    // classic CPU triangulation did, but only once per strand here.
                    const Vector3& origin = strand.vertices[0];
                    range.origin[0] = static_cast<float>(origin.x - eye[0]);
                    range.origin[1] = static_cast<float>(origin.y - eye[1]);
                    range.origin[2] = static_cast<float>(origin.z - eye[2]);
                    ranges_.push_back(range);

                    for (std::uint32_t i = 0; i < vertices; ++i)
                    {
                        const Vector3& p = strand.vertices[i];
                        packed_positions_.push_back(static_cast<float>(p.x - origin.x));
                        packed_positions_.push_back(static_cast<float>(p.y - origin.y));
                        packed_positions_.push_back(static_cast<float>(p.z - origin.z));
                        packed_positions_.push_back(0.0f); // pad to a vec4 for the std430 stride
                    }

                    vertex_total += vertices;
                    index_total += indices;
                }

                if (vertex_total == 0)
                    return;
                total_vertices_ = vertex_total;

                grow(positions_[slot], packed_positions_.size() * sizeof(float),
                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, true);
                std::memcpy(positions_[slot].mapped, packed_positions_.data(),
                            packed_positions_.size() * sizeof(float));

                // The vertex and index buffers are device-local: the GPU writes them (storage)
                // and the draw reads them (vertex/index). No host mapping — nothing but the
                // positions above ever leaves the host.
                grow(vertices_[slot], static_cast<VkDeviceSize>(vertex_total) * sizeof(MeshVertex),
                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);
                grow(indices_[slot], static_cast<VkDeviceSize>(index_total) * sizeof(std::uint32_t),
                     VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, false);

                meshes_[slot] = Mesh{vertices_[slot].buffer, indices_[slot].buffer, vertex_total,
                                     index_total, 0.0f};
            }

            VkBuffer ClothBuffers::positions(std::uint32_t slot) const noexcept
            {
                return positions_[slot].buffer;
            }

            VkDeviceSize ClothBuffers::positions_range() const noexcept
            {
                return static_cast<VkDeviceSize>(total_vertices_) * 4 * sizeof(float);
            }
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
