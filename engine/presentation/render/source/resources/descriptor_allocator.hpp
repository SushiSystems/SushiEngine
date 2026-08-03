/**************************************************************************/
/* descriptor_allocator.hpp                                               */
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
 * @file descriptor_allocator.hpp
 * @brief Throw-away descriptor sets, recycled per frame slot.
 *
 * A graph pass binds resources the graph chose for it this frame, so its
 * descriptor set is only valid for this frame — writing one per frame from a pool
 * that is reset wholesale is both cheaper and simpler than caching sets and
 * rebuilding them whenever a target is reallocated. Each frame slot owns its own
 * chain of pools, reset when that slot comes round again and the fence guarding it
 * has been waited on.
 */

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            /**
             * @brief A per-frame-slot linear allocator for descriptor sets.
             *
             * Grows by chaining new pools when a pool runs out, so a frame that suddenly
             * needs more sets does not fail. Non-copyable: it owns Vulkan pools.
             */
            class DescriptorAllocator
            {
                public:
                    /**
                     * @brief Creates the per-slot pool chains.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames the caller cycles through.
                     */
                    DescriptorAllocator(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~DescriptorAllocator();

                    DescriptorAllocator(const DescriptorAllocator&) = delete;
                    DescriptorAllocator& operator=(const DescriptorAllocator&) = delete;

                    /**
                     * @brief Recycles a slot's sets and directs allocation at it.
                     *
                     * Every set previously allocated from @p slot becomes invalid, so the
                     * caller must have waited on that slot's fence first.
                     *
                     * @param slot The frame slot now being recorded.
                     */
                    void begin_frame(std::uint32_t slot);

                    /**
                     * @brief Allocates one set for the frame currently being recorded.
                     * @param layout The set layout to allocate against.
                     * @return The set, valid until this slot's next begin_frame().
                     */
                    VkDescriptorSet allocate(VkDescriptorSetLayout layout);

                private:
                    /** @brief One frame slot's chain of pools and how far into it we are. */
                    struct Slot
                    {
                        std::vector<VkDescriptorPool> pools;
                        std::size_t current = 0;
                    };

                    VkDescriptorPool create_pool();

                    Vulkan::VulkanDevice& device_;
                    std::vector<Slot> slots_;
                    std::uint32_t recording_slot_ = 0;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
