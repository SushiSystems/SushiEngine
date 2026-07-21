/**************************************************************************/
/* descriptor_writer.hpp                                                  */
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
 * @file descriptor_writer.hpp
 * @brief The one place a descriptor set is written and a set is bound.
 *
 * VK_EXT_descriptor_heap (Vulkan Roadmap 2026) replaces pool-allocated sets, the
 * vkUpdateDescriptorSets write, and the vkCmdBindDescriptorSets bind with a heap
 * model. Keeping every write behind DescriptorWriter and every bind behind
 * bind_descriptor_set() means that swap is a change to these two seams, not a sweep
 * through every pass. Allocation already routes through DescriptorAllocator and the
 * bindless heap through DescriptorHeap; this closes the write/bind half.
 *
 * The writer holds the buffer/image/acceleration-structure infos its writes point at
 * — the part callers get wrong when they build the arrays on the stack and let them
 * die before the commit — so it must outlive its update()/push() call, which a local
 * in the recording scope does. Commit in one of two modes: update() into a set that
 * DescriptorAllocator handed out, or push() straight into the command buffer for a
 * push-descriptor set.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            /**
             * @brief Accumulates one set's descriptor writes, then commits them.
             *
             * Non-owning of the resources it references; owns only the small info structs
             * the Vulkan writes point at. Chainable: each add returns the writer.
             */
            class DescriptorWriter
            {
                public:
                    DescriptorWriter() noexcept = default;

                    /**
                     * @brief Queues a uniform buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind; a null buffer is skipped.
                     * @param range   Bytes visible from offset zero.
                     * @return The writer, for chaining.
                     */
                    DescriptorWriter& uniform_buffer(std::uint32_t binding, VkBuffer buffer,
                                                     VkDeviceSize range)
                    {
                        return buffer_write(binding, buffer, range,
                                            VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
                    }

                    /**
                     * @brief Queues a storage buffer write.
                     * @param binding Binding number in the set.
                     * @param buffer  The buffer to bind; a null buffer is skipped.
                     * @param range   Bytes visible from offset zero.
                     * @return The writer, for chaining.
                     */
                    DescriptorWriter& storage_buffer(std::uint32_t binding, VkBuffer buffer,
                                                     VkDeviceSize range)
                    {
                        return buffer_write(binding, buffer, range,
                                            VK_DESCRIPTOR_TYPE_STORAGE_BUFFER);
                    }

                    /**
                     * @brief Queues a combined image sampler write.
                     * @param binding Binding number in the set.
                     * @param view    The image view to sample; a null view is skipped.
                     * @param sampler The sampler to pair with it.
                     * @return The writer, for chaining.
                     */
                    DescriptorWriter& sampled_image(std::uint32_t binding, VkImageView view,
                                                    VkSampler sampler)
                    {
                        if (count_ >= CAPACITY || view == VK_NULL_HANDLE)
                            return *this;
                        images_[count_].sampler = sampler;
                        images_[count_].imageView = view;
                        images_[count_].imageLayout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                        image_write(binding, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
                        return *this;
                    }

                    /**
                     * @brief Queues a storage image write.
                     * @param binding Binding number in the set.
                     * @param view    The image view to store into; a null view is skipped.
                     * @param layout  The layout the image is in (defaults to GENERAL).
                     * @return The writer, for chaining.
                     */
                    DescriptorWriter& storage_image(std::uint32_t binding, VkImageView view,
                                                    VkImageLayout layout = VK_IMAGE_LAYOUT_GENERAL)
                    {
                        if (count_ >= CAPACITY || view == VK_NULL_HANDLE)
                            return *this;
                        images_[count_].imageView = view;
                        images_[count_].imageLayout = layout;
                        image_write(binding, VK_DESCRIPTOR_TYPE_STORAGE_IMAGE);
                        return *this;
                    }

                    /**
                     * @brief Queues an acceleration structure write for a ray query.
                     * @param binding   Binding number in the set.
                     * @param structure The top-level structure; a null handle is skipped.
                     * @return The writer, for chaining.
                     */
                    DescriptorWriter& acceleration_structure(std::uint32_t binding,
                                                             VkAccelerationStructureKHR structure)
                    {
                        if (count_ >= CAPACITY || structure == VK_NULL_HANDLE)
                            return *this;
                        structure_handles_[count_] = structure;
                        structures_[count_].sType =
                            VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
                        structures_[count_].accelerationStructureCount = 1;
                        structures_[count_].pAccelerationStructures = &structure_handles_[count_];

                        VkWriteDescriptorSet& write = writes_[count_];
                        write = {};
                        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        write.pNext = &structures_[count_];
                        write.dstBinding = binding;
                        write.descriptorCount = 1;
                        write.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
                        ++count_;
                        return *this;
                    }

                    /**
                     * @brief Applies every queued write to an allocated set.
                     * @param device The device owning the set.
                     * @param set    The destination set from DescriptorAllocator.
                     */
                    void update(VkDevice device, VkDescriptorSet set)
                    {
                        if (count_ == 0)
                            return;
                        for (std::uint32_t i = 0; i < count_; ++i)
                            writes_[i].dstSet = set;
                        vkUpdateDescriptorSets(device, count_, writes_, 0, nullptr);
                        count_ = 0;
                    }

                    /**
                     * @brief Pushes every queued write into the command buffer.
                     * @param cmd        The recording command buffer.
                     * @param bind_point Graphics or compute.
                     * @param layout     The pipeline layout the set belongs to.
                     * @param set_index  The set number being pushed.
                     */
                    void push(VkCommandBuffer cmd, VkPipelineBindPoint bind_point,
                              VkPipelineLayout layout, std::uint32_t set_index)
                    {
                        if (count_ == 0)
                            return;
                        vkCmdPushDescriptorSet(cmd, bind_point, layout, set_index, count_, writes_);
                        count_ = 0;
                    }

                private:
                    static constexpr std::uint32_t CAPACITY = 16;

                    DescriptorWriter& buffer_write(std::uint32_t binding, VkBuffer buffer,
                                                   VkDeviceSize range, VkDescriptorType type)
                    {
                        if (count_ >= CAPACITY || buffer == VK_NULL_HANDLE)
                            return *this;
                        buffers_[count_].buffer = buffer;
                        buffers_[count_].offset = 0;
                        buffers_[count_].range = range;

                        VkWriteDescriptorSet& write = writes_[count_];
                        write = {};
                        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        write.dstBinding = binding;
                        write.descriptorCount = 1;
                        write.descriptorType = type;
                        write.pBufferInfo = &buffers_[count_];
                        ++count_;
                        return *this;
                    }

                    void image_write(std::uint32_t binding, VkDescriptorType type)
                    {
                        VkWriteDescriptorSet& write = writes_[count_];
                        write = {};
                        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
                        write.dstBinding = binding;
                        write.descriptorCount = 1;
                        write.descriptorType = type;
                        write.pImageInfo = &images_[count_];
                        ++count_;
                    }

                    VkWriteDescriptorSet writes_[CAPACITY]{};
                    VkDescriptorBufferInfo buffers_[CAPACITY]{};
                    VkDescriptorImageInfo images_[CAPACITY]{};
                    VkWriteDescriptorSetAccelerationStructureKHR structures_[CAPACITY]{};
                    VkAccelerationStructureKHR structure_handles_[CAPACITY]{};
                    std::uint32_t count_ = 0;
            };

            /**
             * @brief Binds one descriptor set, the one place a set is bound.
             *
             * The bind counterpart to DescriptorWriter: routing every set bind through
             * here keeps VK_EXT_descriptor_heap's changed binding model a one-seam swap.
             *
             * @param cmd        The recording command buffer.
             * @param bind_point Graphics or compute.
             * @param layout     The pipeline layout the set belongs to.
             * @param first_set  The set number to bind at.
             * @param set        The descriptor set to bind.
             */
            inline void bind_descriptor_set(VkCommandBuffer cmd, VkPipelineBindPoint bind_point,
                                            VkPipelineLayout layout, std::uint32_t first_set,
                                            VkDescriptorSet set)
            {
                vkCmdBindDescriptorSets(cmd, bind_point, layout, first_set, 1, &set, 0, nullptr);
            }
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
