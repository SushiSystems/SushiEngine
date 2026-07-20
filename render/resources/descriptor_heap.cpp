/**************************************************************************/
/* descriptor_heap.cpp                                                    */
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

#include "resources/descriptor_heap.hpp"

#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            DescriptorHeap::DescriptorHeap(Vulkan::VulkanDevice& device,
                                           std::uint32_t texture_capacity,
                                           std::uint32_t buffer_capacity)
                : device_(device), texture_capacity_(texture_capacity),
                  buffer_capacity_(buffer_capacity)
            {
                if (!device_.supports_descriptor_indexing())
                    return;

                // The capacities are also clamped to what the device will accept in one
                // update-after-bind set, so a heap request larger than the hardware allows
                // shrinks rather than failing device-side.
                VkPhysicalDeviceDescriptorIndexingProperties indexing{};
                indexing.sType =
                    VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_DESCRIPTOR_INDEXING_PROPERTIES;
                VkPhysicalDeviceProperties2 properties{};
                properties.sType = VK_STRUCTURE_TYPE_PHYSICAL_DEVICE_PROPERTIES_2;
                properties.pNext = &indexing;
                vkGetPhysicalDeviceProperties2(device_.physical_device(), &properties);
                if (indexing.maxDescriptorSetUpdateAfterBindSampledImages > 0 &&
                    texture_capacity_ > indexing.maxDescriptorSetUpdateAfterBindSampledImages)
                    texture_capacity_ = indexing.maxDescriptorSetUpdateAfterBindSampledImages;
                if (indexing.maxDescriptorSetUpdateAfterBindStorageBuffers > 0 &&
                    buffer_capacity_ > indexing.maxDescriptorSetUpdateAfterBindStorageBuffers)
                    buffer_capacity_ = indexing.maxDescriptorSetUpdateAfterBindStorageBuffers;

                VkDescriptorSetLayoutBinding bindings[2]{};
                bindings[0].binding = TEXTURE_BINDING;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[0].descriptorCount = texture_capacity_;
                bindings[0].stageFlags = VK_SHADER_STAGE_ALL;
                bindings[1].binding = BUFFER_BINDING;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[1].descriptorCount = buffer_capacity_;
                bindings[1].stageFlags = VK_SHADER_STAGE_ALL;

                // Partially bound: a slot no shader reads may stay unwritten. Update after
                // bind: registering a texture does not have to wait for the set to be idle.
                const VkDescriptorBindingFlags binding_flags =
                    VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT |
                    VK_DESCRIPTOR_BINDING_UPDATE_AFTER_BIND_BIT |
                    VK_DESCRIPTOR_BINDING_UPDATE_UNUSED_WHILE_PENDING_BIT;
                const VkDescriptorBindingFlags flags[2] = {binding_flags, binding_flags};

                VkDescriptorSetLayoutBindingFlagsCreateInfo flags_info{};
                flags_info.sType =
                    VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO;
                flags_info.bindingCount = 2;
                flags_info.pBindingFlags = flags;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.pNext = &flags_info;
                layout_info.flags =
                    VK_DESCRIPTOR_SET_LAYOUT_CREATE_UPDATE_AFTER_BIND_POOL_BIT;
                layout_info.bindingCount = 2;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &layout_),
                              "vkCreateDescriptorSetLayout(heap)");

                const VkDescriptorPoolSize sizes[2] = {
                    {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, texture_capacity_},
                    {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, buffer_capacity_}};

                VkDescriptorPoolCreateInfo pool_info{};
                pool_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
                pool_info.flags = VK_DESCRIPTOR_POOL_CREATE_UPDATE_AFTER_BIND_BIT;
                pool_info.maxSets = 1;
                pool_info.poolSizeCount = 2;
                pool_info.pPoolSizes = sizes;
                Vulkan::check(
                    vkCreateDescriptorPool(device_.device(), &pool_info, nullptr, &pool_),
                    "vkCreateDescriptorPool(heap)");

                VkDescriptorSetAllocateInfo set_info{};
                set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
                set_info.descriptorPool = pool_;
                set_info.descriptorSetCount = 1;
                set_info.pSetLayouts = &layout_;
                Vulkan::check(vkAllocateDescriptorSets(device_.device(), &set_info, &set_),
                              "vkAllocateDescriptorSets(heap)");
            }

            DescriptorHeap::~DescriptorHeap()
            {
                if (pool_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorPool(device_.device(), pool_, nullptr);
                if (layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), layout_, nullptr);
            }

            std::uint32_t DescriptorHeap::take_slot(std::vector<std::uint32_t>& free_list,
                                                    std::uint32_t& next, std::uint32_t capacity)
            {
                if (!free_list.empty())
                {
                    const std::uint32_t index = free_list.back();
                    free_list.pop_back();
                    return index;
                }
                if (next >= capacity)
                    return INVALID_HEAP_INDEX;
                return next++;
            }

            std::uint32_t DescriptorHeap::allocate_texture(VkImageView view, VkSampler sampler,
                                                           VkImageLayout layout)
            {
                if (!available())
                    return INVALID_HEAP_INDEX;
                const std::uint32_t index =
                    take_slot(free_textures_, next_texture_, texture_capacity_);
                if (index == INVALID_HEAP_INDEX)
                    return index;

                VkDescriptorImageInfo image{};
                image.sampler = sampler;
                image.imageView = view;
                image.imageLayout = layout;

                VkWriteDescriptorSet write{};
                write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                write.dstSet = set_;
                write.dstBinding = TEXTURE_BINDING;
                write.dstArrayElement = index;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                write.pImageInfo = &image;
                vkUpdateDescriptorSets(device_.device(), 1, &write, 0, nullptr);
                return index;
            }

            void DescriptorHeap::free_texture(std::uint32_t index)
            {
                if (index != INVALID_HEAP_INDEX && index < texture_capacity_)
                    free_textures_.push_back(index);
            }

            std::uint32_t DescriptorHeap::allocate_buffer(VkBuffer buffer, VkDeviceSize offset,
                                                          VkDeviceSize range)
            {
                if (!available())
                    return INVALID_HEAP_INDEX;
                const std::uint32_t index =
                    take_slot(free_buffers_, next_buffer_, buffer_capacity_);
                if (index == INVALID_HEAP_INDEX)
                    return index;

                VkDescriptorBufferInfo info{};
                info.buffer = buffer;
                info.offset = offset;
                info.range = range;

                VkWriteDescriptorSet write{};
                write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                write.dstSet = set_;
                write.dstBinding = BUFFER_BINDING;
                write.dstArrayElement = index;
                write.descriptorCount = 1;
                write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                write.pBufferInfo = &info;
                vkUpdateDescriptorSets(device_.device(), 1, &write, 0, nullptr);
                return index;
            }

            void DescriptorHeap::free_buffer(std::uint32_t index)
            {
                if (index != INVALID_HEAP_INDEX && index < buffer_capacity_)
                    free_buffers_.push_back(index);
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
