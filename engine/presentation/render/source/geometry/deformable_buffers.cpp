/**************************************************************************/
/* deformable_buffers.cpp                                                 */
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

#include "geometry/deformable_buffers.hpp"

#include <cstring>

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            DeformableBuffers::DeformableBuffers(Vulkan::VulkanDevice& device,
                                                 std::uint32_t frame_slots)
                : device_(device)
            {
                positions_.resize(frame_slots);
                indices_.resize(frame_slots);
                adjacency_ranges_.resize(frame_slots);
                adjacency_triangles_.resize(frame_slots);
                vertices_.resize(frame_slots);
                meshes_.resize(frame_slots);
            }

            DeformableBuffers::~DeformableBuffers()
            {
                for (std::vector<Allocation>* set :
                     {&positions_, &indices_, &adjacency_ranges_, &adjacency_triangles_,
                      &vertices_})
                    for (Allocation& allocation : *set)
                        destroy(allocation);
            }

            void DeformableBuffers::grow(Allocation& target, VkDeviceSize bytes,
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
                              "vmaCreateBuffer(deformable)");
                target.mapped = host_visible ? info.pMappedData : nullptr;
                target.capacity = bytes;
            }

            void DeformableBuffers::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

            void DeformableBuffers::upload(Allocation& target, const void* source,
                                           std::size_t bytes, VkBufferUsageFlags usage)
            {
                grow(target, bytes, usage, true);
                if (target.mapped != nullptr && bytes != 0)
                    std::memcpy(target.mapped, source, bytes);
            }

            const VertexTriangleAdjacency& DeformableBuffers::adjacency_for(
                std::size_t cache_slot, const DeformableMeshView& mesh)
            {
                if (topology_cache_.size() <= cache_slot)
                    topology_cache_.resize(cache_slot + 1);

                TopologyCacheEntry& entry = topology_cache_[cache_slot];
                const bool hit = entry.valid && entry.id == mesh.id &&
                                 entry.revision == mesh.topology_revision &&
                                 entry.vertex_count == mesh.vertex_count &&
                                 entry.index_count == mesh.index_count;
                if (!hit)
                {
                    build_vertex_triangle_adjacency(mesh.indices, mesh.index_count,
                                                    mesh.vertex_count, entry.adjacency);
                    entry.id = mesh.id;
                    entry.revision = mesh.topology_revision;
                    entry.vertex_count = mesh.vertex_count;
                    entry.index_count = mesh.index_count;
                    entry.valid = true;
                }
                return entry.adjacency;
            }

            void DeformableBuffers::prepare(std::uint32_t slot, const DeformableMeshView* meshes,
                                            std::size_t mesh_count, const double eye[3])
            {
                ranges_.clear();
                packed_positions_.clear();
                packed_indices_.clear();
                packed_adjacency_ranges_.clear();
                packed_adjacency_triangles_.clear();
                total_vertices_ = 0;
                meshes_[slot] = Mesh{};
                if (meshes == nullptr || mesh_count == 0)
                    return;

                std::uint32_t vertex_total = 0;
                std::uint32_t index_total = 0;
                for (std::size_t m = 0; m < mesh_count; ++m)
                {
                    const DeformableMeshView& view = meshes[m];
                    if (view.vertices == nullptr || view.vertex_count == 0 ||
                        view.indices == nullptr || view.index_count < 3)
                        continue;

                    const VertexTriangleAdjacency& adjacency = adjacency_for(m, view);

                    DeformableMeshRange range;
                    range.base_vertex = vertex_total;
                    range.base_index = index_total;
                    range.vertex_count = view.vertex_count;
                    range.index_count = view.index_count;
                    range.adjacency_range_base =
                        static_cast<std::uint32_t>(packed_adjacency_ranges_.size());
                    range.adjacency_triangle_base =
                        static_cast<std::uint32_t>(packed_adjacency_triangles_.size());
                    range.view_index = static_cast<std::uint32_t>(m);

                    // The mesh origin is its first vertex; the local positions packed below are
                    // each vertex minus that origin, computed in double so a planet-scale
                    // absolute position never touches single precision. The origin itself is
                    // made camera-relative the same way, and the GPU adds the two back — one
                    // subtraction per mesh instead of one per vertex.
                    const Vector3& origin = view.vertices[0];
                    range.origin[0] = static_cast<float>(origin.x - eye[0]);
                    range.origin[1] = static_cast<float>(origin.y - eye[1]);
                    range.origin[2] = static_cast<float>(origin.z - eye[2]);
                    ranges_.push_back(range);

                    for (std::uint32_t i = 0; i < view.vertex_count; ++i)
                    {
                        const Vector3& p = view.vertices[i];
                        packed_positions_.push_back(static_cast<float>(p.x - origin.x));
                        packed_positions_.push_back(static_cast<float>(p.y - origin.y));
                        packed_positions_.push_back(static_cast<float>(p.z - origin.z));
                        packed_positions_.push_back(0.0f); // pad to a vec4 for the std430 stride
                    }

                    // Copied verbatim, still mesh-local: the draw supplies base_vertex as the
                    // index-buffer vertex offset, so nothing has to rewrite these into the
                    // shared numbering — which is also what lets this buffer be uploaded once
                    // per topology change rather than rebuilt per frame.
                    packed_indices_.insert(packed_indices_.end(), view.indices,
                                           view.indices + view.index_count);
                    packed_adjacency_ranges_.insert(packed_adjacency_ranges_.end(),
                                                    adjacency.range.begin(),
                                                    adjacency.range.end());
                    packed_adjacency_triangles_.insert(packed_adjacency_triangles_.end(),
                                                       adjacency.triangle.begin(),
                                                       adjacency.triangle.end());

                    vertex_total += view.vertex_count;
                    index_total += view.index_count;
                }

                if (vertex_total == 0)
                    return;
                total_vertices_ = vertex_total;

                // A mesh whose every triangle was malformed contributes no adjacency entries,
                // and a descriptor may not expose a zero-byte range; one padding entry keeps
                // the binding legal without the shader ever reading it (a vertex with no
                // triangles has a count of zero, so the gather loop does not run).
                if (packed_adjacency_triangles_.empty())
                    packed_adjacency_triangles_.push_back(0);

                upload(positions_[slot], packed_positions_.data(),
                       packed_positions_.size() * sizeof(float),
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
                upload(indices_[slot], packed_indices_.data(),
                       packed_indices_.size() * sizeof(std::uint32_t),
                       VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
                upload(adjacency_ranges_[slot], packed_adjacency_ranges_.data(),
                       packed_adjacency_ranges_.size() * sizeof(std::uint32_t),
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);
                upload(adjacency_triangles_[slot], packed_adjacency_triangles_.data(),
                       packed_adjacency_triangles_.size() * sizeof(std::uint32_t),
                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT);

                // The only device-local buffer, because it is the only one the GPU writes: the
                // compute pass shades into it (storage) and the draw reads it (vertex).
                grow(vertices_[slot], static_cast<VkDeviceSize>(vertex_total) * sizeof(MeshVertex),
                     VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                     false);

                meshes_[slot] = Mesh{vertices_[slot].buffer, indices_[slot].buffer, vertex_total,
                                     index_total, 0.0f};
            }

            VkBuffer DeformableBuffers::positions(std::uint32_t slot) const noexcept
            {
                return positions_[slot].buffer;
            }

            VkBuffer DeformableBuffers::indices(std::uint32_t slot) const noexcept
            {
                return indices_[slot].buffer;
            }

            VkBuffer DeformableBuffers::adjacency_ranges(std::uint32_t slot) const noexcept
            {
                return adjacency_ranges_[slot].buffer;
            }

            VkBuffer DeformableBuffers::adjacency_triangles(std::uint32_t slot) const noexcept
            {
                return adjacency_triangles_[slot].buffer;
            }

            VkDeviceSize DeformableBuffers::positions_range() const noexcept
            {
                return static_cast<VkDeviceSize>(packed_positions_.size()) * sizeof(float);
            }

            VkDeviceSize DeformableBuffers::indices_range() const noexcept
            {
                return static_cast<VkDeviceSize>(packed_indices_.size()) * sizeof(std::uint32_t);
            }

            VkDeviceSize DeformableBuffers::adjacency_ranges_range() const noexcept
            {
                return static_cast<VkDeviceSize>(packed_adjacency_ranges_.size()) *
                       sizeof(std::uint32_t);
            }

            VkDeviceSize DeformableBuffers::adjacency_triangles_range() const noexcept
            {
                return static_cast<VkDeviceSize>(packed_adjacency_triangles_.size()) *
                       sizeof(std::uint32_t);
            }
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
