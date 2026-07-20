/**************************************************************************/
/* transient_pool.hpp                                                     */
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
 * @file transient_pool.hpp
 * @brief Physical backing for the render graph's transient resources.
 *
 * The graph works in virtual resources; these pools own the real VkImages and
 * VkBuffers behind them. A physical entry is handed out by acquire() and returned
 * by release() the moment its last reading pass has been scheduled, so two
 * transients whose lifetimes do not overlap share one allocation — that reuse is
 * the graph's memory aliasing. Because the layout of a VkImage survives the
 * hand-off, the synchronisation state is tracked on the physical entry, not on the
 * virtual resource that borrowed it.
 */

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include "graph/resource_handle.hpp"
#include "graph/resource_state.hpp"

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
            /** @brief The index an entry lookup returns when nothing matches. */
            constexpr std::uint32_t INVALID_ENTRY = 0xFFFFFFFFu;

            /**
             * @brief A pool of reusable images backing the graph's transient textures.
             *
             * Entries live across frames: an image whose description is requested again
             * next frame is handed back rather than reallocated, so a steady-state frame
             * performs no image allocation at all. Entries unused for RETIRE_FRAMES
             * consecutive frames are destroyed, which is how a resize releases the old
             * size's images without an explicit invalidation call.
             */
            class TexturePool
            {
                public:
                    /**
                     * @brief One physical image and the views covering all of it.
                     *
                     * A depth/stencil image carries a second, depth-aspect-only view:
                     * Vulkan forbids sampling through a view that spans both aspects, so
                     * the combined view serves the attachment and @c sample_view serves
                     * any pass that reads the depth back.
                     */
                    struct Entry
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        VkImageView sample_view = VK_NULL_HANDLE;
                        Graph::TextureDesc desc{};
                        Graph::TextureState state{};
                        bool in_use = false;
                        std::uint32_t frames_unused = 0;
                    };

                    /**
                     * @brief Binds the pool to the device it allocates from.
                     * @param device The live Vulkan device.
                     */
                    explicit TexturePool(Vulkan::VulkanDevice& device);
                    ~TexturePool();

                    TexturePool(const TexturePool&) = delete;
                    TexturePool& operator=(const TexturePool&) = delete;

                    /**
                     * @brief Frees every entry for a new frame and retires stale ones.
                     *
                     * Must be called only when no in-flight frame still reads the pool's
                     * images, because retiring destroys them.
                     */
                    void begin_frame();

                    /**
                     * @brief Hands out an image matching a description, creating one if needed.
                     * @param desc The image to back a transient with.
                     * @return The entry index, valid until the next begin_frame().
                     */
                    std::uint32_t acquire(const Graph::TextureDesc& desc);

                    /**
                     * @brief Returns an entry to the free list so a later transient may reuse it.
                     * @param entry Index previously returned by acquire().
                     */
                    void release(std::uint32_t entry);

                    /**
                     * @brief The image, view, and tracked state behind an entry.
                     * @param entry_index Index previously returned by acquire().
                     * @return The mutable entry, so the graph can update its tracked state.
                     */
                    Entry& entry(std::uint32_t entry_index) { return entries_[entry_index]; }

                    /** @brief Destroys every entry; the caller must have idled the device. */
                    void clear();

                private:
                    static constexpr std::uint32_t RETIRE_FRAMES = 4;

                    void destroy(Entry& entry);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Entry> entries_;
            };

            /**
             * @brief A pool of reusable buffers backing the graph's transient buffers.
             *
             * Mirrors TexturePool; host-visible entries stay permanently mapped so a
             * readback or an upload needs no map/unmap on the frame path.
             */
            class BufferPool
            {
                public:
                    /** @brief One physical buffer and its mapping, if host-visible. */
                    struct Entry
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        Graph::BufferDesc desc{};
                        Graph::BufferState state{};
                        bool in_use = false;
                        std::uint32_t frames_unused = 0;
                    };

                    /**
                     * @brief Binds the pool to the device it allocates from.
                     * @param device The live Vulkan device.
                     */
                    explicit BufferPool(Vulkan::VulkanDevice& device);
                    ~BufferPool();

                    BufferPool(const BufferPool&) = delete;
                    BufferPool& operator=(const BufferPool&) = delete;

                    /** @brief Frees every entry for a new frame and retires stale ones. */
                    void begin_frame();

                    /**
                     * @brief Hands out a buffer matching a description, creating one if needed.
                     * @param desc The buffer to back a transient with.
                     * @return The entry index, valid until the next begin_frame().
                     */
                    std::uint32_t acquire(const Graph::BufferDesc& desc);

                    /**
                     * @brief Returns an entry to the free list so a later transient may reuse it.
                     * @param entry Index previously returned by acquire().
                     */
                    void release(std::uint32_t entry);

                    /**
                     * @brief The buffer, mapping, and tracked state behind an entry.
                     * @param entry_index Index previously returned by acquire().
                     * @return The mutable entry, so the graph can update its tracked state.
                     */
                    Entry& entry(std::uint32_t entry_index) { return entries_[entry_index]; }

                    /** @brief Destroys every entry; the caller must have idled the device. */
                    void clear();

                private:
                    static constexpr std::uint32_t RETIRE_FRAMES = 4;

                    void destroy(Entry& entry);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Entry> entries_;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
