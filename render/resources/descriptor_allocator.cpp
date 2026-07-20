/**************************************************************************/
/* descriptor_allocator.cpp                                               */
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

#include "resources/descriptor_allocator.hpp"

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            namespace
            {
                // Sized for a frame's worth of pass sets with room to spare; a frame that
                // outgrows one pool simply chains another rather than failing.
                constexpr std::uint32_t SETS_PER_POOL = 128;
            } // namespace

            DescriptorAllocator::DescriptorAllocator(Vulkan::VulkanDevice& device,
                                                     std::uint32_t frame_slots)
                : device_(device)
            {
                slots_.resize(frame_slots);
            }

            DescriptorAllocator::~DescriptorAllocator()
            {
                for (Slot& slot : slots_)
                    for (VkDescriptorPool pool : slot.pools)
                        vkDestroyDescriptorPool(device_.device(), pool, nullptr);
            }

            VkDescriptorPool DescriptorAllocator::create_pool()
            {
                const VkDescriptorPoolSize sizes[] = {
                    {VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, SETS_PER_POOL * 4},
                    {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, SETS_PER_POOL * 12},
                    {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, SETS_PER_POOL * 4},
                    {VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, SETS_PER_POOL * 2},
                };

                VkDescriptorPoolCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                info.maxSets = SETS_PER_POOL;
                info.poolSizeCount = static_cast<std::uint32_t>(sizeof(sizes) / sizeof(sizes[0]));
                info.pPoolSizes = sizes;

                VkDescriptorPool pool = VK_NULL_HANDLE;
                Vulkan::check(vkCreateDescriptorPool(device_.device(), &info, nullptr, &pool),
                              "vkCreateDescriptorPool(frame)");
                return pool;
            }

            void DescriptorAllocator::begin_frame(std::uint32_t slot_index)
            {
                if (slot_index >= slots_.size())
                    return;
                recording_slot_ = slot_index;
                Slot& slot = slots_[slot_index];
                for (VkDescriptorPool pool : slot.pools)
                    Vulkan::check(vkResetDescriptorPool(device_.device(), pool, 0),
                                  "vkResetDescriptorPool(frame)");
                slot.current = 0;
            }

            VkDescriptorSet DescriptorAllocator::allocate(VkDescriptorSetLayout layout)
            {
                Slot& slot = slots_[recording_slot_];
                if (slot.pools.empty())
                    slot.pools.push_back(create_pool());

                for (;;)
                {
                    VkDescriptorSetAllocateInfo info{};
                    info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                    info.descriptorPool = slot.pools[slot.current];
                    info.descriptorSetCount = 1;
                    info.pSetLayouts = &layout;

                    VkDescriptorSet set = VK_NULL_HANDLE;
                    const VkResult result =
                        vkAllocateDescriptorSets(device_.device(), &info, &set);
                    if (result == VK_SUCCESS)
                        return set;
                    if (result != VK_ERROR_OUT_OF_POOL_MEMORY &&
                        result != VK_ERROR_FRAGMENTED_POOL)
                        Vulkan::check(result, "vkAllocateDescriptorSets(frame)");

                    ++slot.current;
                    if (slot.current == slot.pools.size())
                        slot.pools.push_back(create_pool());
                }
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
