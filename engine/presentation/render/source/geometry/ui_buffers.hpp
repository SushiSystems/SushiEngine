/**************************************************************************/
/* ui_buffers.hpp                                                         */
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
 * @file ui_buffers.hpp
 * @brief The UI overlay's per-frame geometry: draw list in, one vertex/index buffer out.
 *
 * The overlay's geometry is rebuilt from scratch every frame on the host — a UI is a few
 * hundred quads, far below the point where keeping it resident and diffing it would pay.
 * The buffers themselves are per frame slot and only ever grow, the same arrangement
 * `DeformableBuffers` uses for soft-body vertices and for the same reason: a frame in flight
 * must not have its geometry rewritten underneath it.
 *
 * Rectangles and glyphs land in the *same* buffer with the same vertex format. A plain
 * rectangle carries the atlas coordinate of a fully-opaque texel and a glyph carries its
 * own, so the fragment shader has one path and the whole overlay is one indexed draw
 * rather than one per element.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vk_mem_alloc.h>
#include <vulkan/vulkan.h>

#include <SushiEngine/render/scene_view.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Assets
        {
            class FontAtlas;
        }

        namespace Geometry
        {
            /**
             * @brief One overlay vertex: screen position, atlas coordinate, premultiplied colour.
             *
             * 20 bytes, tightly packed. The colour is premultiplied here rather than in the
             * shader so the blend is a plain "over" and a fully transparent element
             * contributes nothing at all rather than a black fringe.
             */
            struct UIVertex
            {
                float x = 0.0f;
                float y = 0.0f;
                float u = 0.0f;
                float v = 0.0f;
                std::uint8_t color[4] = {255, 255, 255, 255};
            };

            /**
             * @brief Growable per-slot overlay vertex and index buffers.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class UIBuffers
            {
                public:
                    /**
                     * @brief Allocates the per-slot buffer set.
                     * @param device      The live Vulkan device.
                     * @param frame_slots Number of frames in flight.
                     */
                    UIBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots);
                    ~UIBuffers();

                    UIBuffers(const UIBuffers&) = delete;
                    UIBuffers& operator=(const UIBuffers&) = delete;

                    /**
                     * @brief Tessellates the frame's draw list into this slot's buffers.
                     *
                     * Rectangles become one quad each; every printable character of every text
                     * run becomes one quad placed along its run's baseline. Text is skipped
                     * (rectangles still drawn) when @p font is invalid, because a missing font
                     * should cost the labels, not the whole overlay.
                     *
                     * @param slot The frame slot being recorded.
                     * @param ui   The frame's resolved UI geometry.
                     * @param font The baked glyph atlas, or an invalid one for no text.
                     */
                    void prepare(std::uint32_t slot, const UIView& ui,
                                 const Assets::FontAtlas& font);

                    /** @brief Whether the frame produced any drawable geometry. */
                    bool empty() const noexcept { return index_count_ == 0; }

                    /** @brief Indices to draw this frame. */
                    std::uint32_t index_count() const noexcept { return index_count_; }

                    /** @brief This slot's vertex buffer. */
                    VkBuffer vertices(std::uint32_t slot) const noexcept;

                    /** @brief This slot's index buffer. */
                    VkBuffer indices(std::uint32_t slot) const noexcept;

                private:
                    /** @brief A VMA-backed host-visible buffer and the capacity it was made at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage);
                    void destroy(Allocation& target);

                    /** @brief Appends one axis-aligned quad, in the winding the pipeline expects. */
                    void push_quad(float x0, float y0, float x1, float y1, float u0, float v0,
                                   float u1, float v1, const std::uint8_t color[4]);

                    Vulkan::VulkanDevice& device_;
                    std::vector<Allocation> vertices_;
                    std::vector<Allocation> indices_;
                    std::vector<UIVertex> vertex_scratch_;
                    std::vector<std::uint32_t> index_scratch_;
                    std::uint32_t index_count_ = 0;
            };
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
