/**************************************************************************/
/* instance_system.cpp                                                    */
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

#include "scene/instance_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "geometry/mesh_registry.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            namespace
            {
                /** @brief Length of the model matrix's largest 3x3 column: its peak scale. */
                float largest_column(const float model[16]) noexcept
                {
                    const float c0 = std::sqrt(model[0] * model[0] + model[1] * model[1] +
                                               model[2] * model[2]);
                    const float c1 = std::sqrt(model[4] * model[4] + model[5] * model[5] +
                                               model[6] * model[6]);
                    const float c2 = std::sqrt(model[8] * model[8] + model[9] * model[9] +
                                               model[10] * model[10]);
                    return std::max(c0, std::max(c1, c2));
                }
            } // namespace

            InstanceSystem::InstanceSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device), slots_(frame_slots)
            {
            }

            InstanceSystem::~InstanceSystem()
            {
                for (Slot& slot : slots_)
                    destroy(slot);
            }

            void InstanceSystem::build(std::uint32_t slot, const MeshInstance* instances,
                                       std::size_t count, const double eye[3],
                                       const std::uint32_t* materials,
                                       const std::uint32_t* motions,
                                       Geometry::MeshRegistry& meshes)
            {
                current_slot_ = slot;
                instances_.clear();
                bucket_meta_.clear();
                buckets_.clear();
                bucket_lookup_.clear();
                if (instances == nullptr || count == 0)
                    return;
                instances_.reserve(count);

                for (std::size_t i = 0; i < count; ++i)
                {
                    const MeshInstance& instance = instances[i];
                    const bool imported = instance.mesh != INVALID_MESH;
                    const Geometry::Mesh& mesh =
                        imported ? meshes.mesh(instance.mesh) : meshes.primitive(instance.kind);
                    if (mesh.index_count == 0)
                        continue;

                    // One bucket per distinct mesh, keyed by its vertex buffer — the same
                    // grouping the classic draw loop makes so a mesh's buffers bind once.
                    std::uint32_t bucket;
                    auto found = bucket_lookup_.find(mesh.vertices);
                    if (found == bucket_lookup_.end())
                    {
                        bucket = static_cast<std::uint32_t>(buckets_.size());
                        bucket_lookup_.emplace(mesh.vertices, bucket);
                        GpuDrawBucket entry;
                        entry.vertices = mesh.vertices;
                        entry.indices = mesh.indices;
                        entry.index_count = mesh.index_count;
                        entry.candidate_base = 0;
                        entry.candidate_count = 0;
                        buckets_.push_back(entry);
                    }
                    else
                    {
                        bucket = found->second;
                    }
                    ++buckets_[bucket].candidate_count;

                    // The transform the draw actually uses — primitive scaling baked in, then
                    // made camera-relative in double before the float cast, exactly as the
                    // classic push path did — so the cull and vertex shaders see one geometry.
                    const Mat4 model =
                        imported ? instance.model
                                 : mul(instance.model, Geometry::shape_scale(
                                                           instance.kind, instance.shape_params));

                    GpuInstance packed{};
                    for (int m = 0; m < 16; ++m)
                        packed.model[m] = static_cast<float>(model.m[m]);
                    packed.model[12] = static_cast<float>(model.m[12] - eye[0]);
                    packed.model[13] = static_cast<float>(model.m[13] - eye[1]);
                    packed.model[14] = static_cast<float>(model.m[14] - eye[2]);

                    // A local-origin bounding sphere scaled by the transform's largest column
                    // stays a valid world bound under any non-uniform scale, so the cull can
                    // only ever be conservative — it never rejects a visible object.
                    const float radius = mesh.radius * largest_column(packed.model);
                    packed.bounding_sphere[0] = packed.model[12];
                    packed.bounding_sphere[1] = packed.model[13];
                    packed.bounding_sphere[2] = packed.model[14];
                    packed.bounding_sphere[3] = radius;

                    packed.material_index = materials != nullptr ? materials[i] : 0u;
                    packed.motion_index = motions != nullptr ? motions[i] : 0u;
                    packed.entity_id = instance.id;
                    packed.bucket_index = bucket;
                    instances_.push_back(packed);
                }

                // The prefix sum that gives each bucket a contiguous slice of the instance and
                // compacted arrays; the cull shader compacts a bucket's survivors into it.
                std::uint32_t base = 0;
                bucket_meta_.reserve(buckets_.size());
                for (GpuDrawBucket& entry : buckets_)
                {
                    entry.candidate_base = base;
                    GpuBucketMeta meta;
                    meta.index_count = entry.index_count;
                    meta.candidate_base = base;
                    meta.reserved0 = 0;
                    meta.reserved1 = 0;
                    bucket_meta_.push_back(meta);
                    base += entry.candidate_count;
                }
            }

            void InstanceSystem::upload()
            {
                if (slots_.empty())
                    return;
                Slot& slot = slots_[current_slot_];

                if (!instances_.empty())
                {
                    const VkDeviceSize bytes = instances_.size() * sizeof(GpuInstance);
                    grow(slot.instances, slot.instances_allocation, slot.instances_mapped,
                         slot.instances_capacity, bytes);
                    if (slot.instances_mapped != nullptr)
                        std::memcpy(slot.instances_mapped, instances_.data(),
                                    static_cast<std::size_t>(bytes));
                }
                if (!bucket_meta_.empty())
                {
                    const VkDeviceSize bytes = bucket_meta_.size() * sizeof(GpuBucketMeta);
                    grow(slot.buckets, slot.buckets_allocation, slot.buckets_mapped,
                         slot.buckets_capacity, bytes);
                    if (slot.buckets_mapped != nullptr)
                        std::memcpy(slot.buckets_mapped, bucket_meta_.data(),
                                    static_cast<std::size_t>(bytes));
                }
            }

            VkBuffer InstanceSystem::instance_buffer() const noexcept
            {
                return slots_.empty() ? VK_NULL_HANDLE : slots_[current_slot_].instances;
            }

            VkDeviceSize InstanceSystem::instance_buffer_range() const noexcept
            {
                if (instances_.empty())
                    return 0;
                return instances_.size() * sizeof(GpuInstance);
            }

            VkBuffer InstanceSystem::bucket_buffer() const noexcept
            {
                return slots_.empty() ? VK_NULL_HANDLE : slots_[current_slot_].buckets;
            }

            VkDeviceSize InstanceSystem::bucket_buffer_range() const noexcept
            {
                if (bucket_meta_.empty())
                    return 0;
                return bucket_meta_.size() * sizeof(GpuBucketMeta);
            }

            void InstanceSystem::grow(VkBuffer& buffer, VmaAllocation& allocation, void*& mapped,
                                      VkDeviceSize& capacity, VkDeviceSize bytes)
            {
                if (bytes <= capacity && buffer != VK_NULL_HANDLE)
                    return;
                if (buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), buffer, allocation);

                // Grow with headroom so a scene that adds a few instances a frame does not
                // reallocate every frame; the extra bytes are host-visible staging, cheap.
                const VkDeviceSize allocated = bytes + bytes / 2 + 256;
                VkBufferCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                info.size = allocated;
                info.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo mapped_info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &info, &alloc, &buffer,
                                              &allocation, &mapped_info),
                              "vmaCreateBuffer(instance system)");
                mapped = mapped_info.pMappedData;
                capacity = allocated;
            }

            void InstanceSystem::destroy(Slot& slot)
            {
                if (slot.instances != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.instances, slot.instances_allocation);
                if (slot.buckets != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), slot.buckets, slot.buckets_allocation);
                slot = Slot{};
            }
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
