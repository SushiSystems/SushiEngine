/**************************************************************************/
/* instance_system.hpp                                                    */
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
 * @file instance_system.hpp
 * @brief Packs the frame's drawables into the GPU-driven instance and bucket buffers.
 *
 * The GPU-driven geometry path replaces the CPU's one-draw-per-instance loop with two
 * device buffers and a cull dispatch. This system builds those buffers each frame: it
 * groups every mesh instance by the geometry it draws with into @ref GpuDrawBucket
 * ranges, packs a @ref GpuInstance record for each (camera-relative transform, bounding
 * sphere, and the material/motion/pick indices the draw used to push), and lays out the
 * per-bucket metadata the cull shader reads. It is the exact analogue of MotionSystem
 * and MaterialSystem — one host-mapped buffer per frame slot, grown on demand — and the
 * cull pass and the GPU vertex shader consume what it produces.
 *
 * Only opaque mesh instances travel this path. Soft bodies and the selection outline
 * stay on the classic CPU draw path, because they are few, special, or
 * rebuilt per frame; the win here is removing the per-instance CPU cost of dense scenes,
 * not the handful of one-off draws.
 */

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "scene/gpu_instance.hpp"

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
            class MeshRegistry;
        }

        namespace Scene
        {
            /**
             * @brief Builds and owns the per-frame GPU-driven instance and bucket buffers.
             *
             * Per frame slot rather than shared, because the buffers are host-written each
             * frame and a slot still in flight may be reading the one before it.
             * Non-copyable: it owns Vulkan buffers.
             */
            class InstanceSystem
            {
                public:
                    /**
                     * @brief Allocates one instance and one bucket buffer per frame slot.
                     * @param device      The live Vulkan device.
                     * @param frame_slots How many frames may be in flight.
                     */
                    InstanceSystem(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~InstanceSystem();

                    InstanceSystem(const InstanceSystem&) = delete;
                    InstanceSystem& operator=(const InstanceSystem&) = delete;

                    /**
                     * @brief Groups the frame's instances into buckets and packs their records.
                     *
                     * One bucket per distinct mesh; every instance of that mesh shares it and
                     * lands in one indirect draw. Each bucket reserves a contiguous slice of
                     * the instance and compacted arrays (@c candidate_base), so the cull
                     * shader can compact survivors per bucket without a global counter.
                     *
                     * @param slot      The frame slot whose buffers this frame writes.
                     * @param instances The frame's mesh instances.
                     * @param count     Number of entries in @p instances.
                     * @param eye       Camera world position, subtracted to stay camera-relative.
                     * @param materials Per-instance index into the frame's material array.
                     * @param motions   Per-instance index into the frame's motion array.
                     * @param meshes    The registry the bounds and buffers come from.
                     *
                     * @p materials and @p motions are packed by the caller (the same
                     * MaterialSystem/MotionSystem pushes the classic path would make), so the
                     * GPU draw carries them in the instance record instead of a push constant.
                     * Both are @p count long and indexed by the original instance order.
                     */
                    void build(std::uint32_t slot, const MeshInstance* instances, std::size_t count,
                               const double eye[3], const std::uint32_t* materials,
                               const std::uint32_t* motions, Geometry::MeshRegistry& meshes);

                    /** @brief Copies the packed arrays into this frame slot's buffers. */
                    void upload();

                    /** @brief The buckets this frame draws, one per distinct mesh. */
                    const std::vector<GpuDrawBucket>& buckets() const noexcept { return buckets_; }

                    /** @brief Total instances packed this frame, across every bucket. */
                    std::uint32_t instance_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(instances_.size());
                    }

                    /** @brief Number of buckets, i.e. distinct meshes, drawn this frame. */
                    std::uint32_t bucket_count() const noexcept
                    {
                        return static_cast<std::uint32_t>(buckets_.size());
                    }

                    /** @brief Whether the frame packed anything worth a GPU-driven pass. */
                    bool empty() const noexcept { return instances_.empty(); }

                    /** @brief The current slot's packed-instance buffer. */
                    VkBuffer instance_buffer() const noexcept;

                    /** @brief Bytes of the instance buffer the descriptor must expose. */
                    VkDeviceSize instance_buffer_range() const noexcept;

                    /** @brief The current slot's packed per-bucket metadata buffer. */
                    VkBuffer bucket_buffer() const noexcept;

                    /** @brief Bytes of the bucket buffer the descriptor must expose. */
                    VkDeviceSize bucket_buffer_range() const noexcept;

                private:
                    /** @brief One frame slot's two packing buffers. */
                    struct Slot
                    {
                        VkBuffer instances = VK_NULL_HANDLE;
                        VmaAllocation instances_allocation = VK_NULL_HANDLE;
                        void* instances_mapped = nullptr;
                        VkDeviceSize instances_capacity = 0;
                        VkBuffer buckets = VK_NULL_HANDLE;
                        VmaAllocation buckets_allocation = VK_NULL_HANDLE;
                        void* buckets_mapped = nullptr;
                        VkDeviceSize buckets_capacity = 0;
                    };

                    void grow(VkBuffer& buffer, VmaAllocation& allocation, void*& mapped,
                              VkDeviceSize& capacity, VkDeviceSize bytes);
                    void destroy(Slot& slot);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Slot> slots_;
                    std::vector<GpuInstance> instances_;
                    std::vector<GpuBucketMeta> bucket_meta_;
                    std::vector<GpuDrawBucket> buckets_;
                    std::unordered_map<VkBuffer, std::uint32_t> bucket_lookup_;
                    std::uint32_t current_slot_ = 0;
            };
        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
