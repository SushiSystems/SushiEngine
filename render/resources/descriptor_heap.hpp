/**************************************************************************/
/* descriptor_heap.hpp                                                    */
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
 * @file descriptor_heap.hpp
 * @brief One global, update-after-bind descriptor array the whole renderer indexes into.
 *
 * The bindless heap replaces per-material and per-target descriptor sets with a
 * single set bound once: a texture is registered and thereafter identified by a
 * plain @c uint32_t index that a shader reads out of a material or push constant.
 * Registration is free of frame timing because the bindings are declared
 * update-after-bind and partially bound — writing a slot no pass reads this frame
 * is legal while the set is in use.
 *
 * Requires VK_EXT_descriptor_indexing (core in Vulkan 1.2). When the device does
 * not offer it, available() is false and the heap allocates nothing; callers keep
 * their explicit sets, which is why the renderer still runs on such a device.
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
            /** @brief The index a heap allocation returns when the heap is full or absent. */
            constexpr std::uint32_t INVALID_HEAP_INDEX = 0xFFFFFFFFu;

            /**
             * @brief The renderer's global texture and storage-buffer descriptor heap.
             *
             * Non-copyable: it owns the set layout, pool, and set.
             */
            class DescriptorHeap
            {
                public:
                    /** @brief Binding number of the combined-image-sampler array. */
                    static constexpr std::uint32_t TEXTURE_BINDING = 0;

                    /** @brief Binding number of the storage-buffer array. */
                    static constexpr std::uint32_t BUFFER_BINDING = 1;

                    /**
                     * @brief Binding number of the 3D-texture (volume) array.
                     *
                     * The per-frame push set is full at its guaranteed 32 entries, so a
                     * device-global volume every shading pass may need — the GI distance
                     * clipmap a stochastic shadow ray marches — reaches them through the heap
                     * instead. Kept small: these are engine-owned fields, not content.
                     */
                    static constexpr std::uint32_t VOLUME_BINDING = 2;

                    /** @brief Volume slots reserved; engine fields only, so a handful suffices. */
                    static constexpr std::uint32_t VOLUME_CAPACITY = 4;

                    /**
                     * @brief Creates the heap, or nothing if the device lacks the feature.
                     * @param device           The live Vulkan device.
                     * @param texture_capacity Slots reserved for combined image samplers.
                     * @param buffer_capacity  Slots reserved for storage buffers.
                     */
                    DescriptorHeap(Vulkan::VulkanDevice& device, std::uint32_t texture_capacity,
                                   std::uint32_t buffer_capacity);
                    ~DescriptorHeap();

                    DescriptorHeap(const DescriptorHeap&) = delete;
                    DescriptorHeap& operator=(const DescriptorHeap&) = delete;

                    /** @brief Whether the device supported descriptor indexing and the heap exists. */
                    bool available() const noexcept { return set_ != VK_NULL_HANDLE; }

                    /** @brief The layout to include in a pipeline layout that indexes the heap. */
                    VkDescriptorSetLayout layout() const noexcept { return layout_; }

                    /** @brief The set to bind once per frame at the heap's set index. */
                    VkDescriptorSet set() const noexcept { return set_; }

                    /**
                     * @brief Registers a texture and returns the index shaders address it by.
                     * @param view    The image view to sample.
                     * @param sampler The sampler to pair it with.
                     * @param layout  The layout the image will be in when sampled.
                     * @return The heap index, or INVALID_HEAP_INDEX if unavailable or full.
                     */
                    std::uint32_t allocate_texture(VkImageView view, VkSampler sampler,
                                                   VkImageLayout layout);

                    /**
                     * @brief Registers a 3D texture and returns the slot shaders address it by.
                     *
                     * Volume slots are never released: the fields that occupy them are created
                     * once with the device and live as long as it does, so there is no free
                     * list to keep.
                     *
                     * @param view    The 3D image view to sample.
                     * @param sampler The sampler to pair it with.
                     * @return The slot index, or INVALID_HEAP_INDEX if unavailable or full.
                     */
                    std::uint32_t allocate_volume(VkImageView view, VkSampler sampler);

                    /**
                     * @brief Releases a texture slot for reuse.
                     * @param index An index previously returned by allocate_texture().
                     */
                    void free_texture(std::uint32_t index);

                    /**
                     * @brief Registers a storage buffer and returns its heap index.
                     * @param buffer The buffer to expose.
                     * @param offset Offset into @p buffer, in bytes.
                     * @param range  Bytes visible from @p offset, or VK_WHOLE_SIZE.
                     * @return The heap index, or INVALID_HEAP_INDEX if unavailable or full.
                     */
                    std::uint32_t allocate_buffer(VkBuffer buffer, VkDeviceSize offset,
                                                  VkDeviceSize range);

                    /**
                     * @brief Releases a storage-buffer slot for reuse.
                     * @param index An index previously returned by allocate_buffer().
                     */
                    void free_buffer(std::uint32_t index);

                private:
                    std::uint32_t take_slot(std::vector<std::uint32_t>& free_list,
                                            std::uint32_t& next, std::uint32_t capacity);

                    Vulkan::VulkanDevice& device_;
                    VkDescriptorSetLayout layout_ = VK_NULL_HANDLE;
                    VkDescriptorPool pool_ = VK_NULL_HANDLE;
                    VkDescriptorSet set_ = VK_NULL_HANDLE;
                    std::uint32_t texture_capacity_ = 0;
                    std::uint32_t buffer_capacity_ = 0;
                    std::uint32_t next_texture_ = 0;
                    std::uint32_t next_buffer_ = 0;
                    std::uint32_t next_volume_ = 0;
                    std::vector<std::uint32_t> free_textures_;
                    std::vector<std::uint32_t> free_buffers_;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
