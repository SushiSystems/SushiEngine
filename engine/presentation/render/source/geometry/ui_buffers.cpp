/**************************************************************************/
/* ui_buffers.cpp                                                         */
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

#include "geometry/ui_buffers.hpp"

#include <algorithm>
#include <cstring>

#include "material/font_atlas.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            namespace
            {
                /** @brief Clamps a [0, 1] channel and premultiplies it by @p alpha. */
                std::uint8_t premultiplied_channel(Scalar value, Scalar alpha) noexcept
                {
                    const double scaled =
                        std::max(0.0, std::min(1.0, double(value))) * std::max(0.0, std::min(1.0, double(alpha)));
                    return static_cast<std::uint8_t>(scaled * 255.0 + 0.5);
                }

                /** @brief Packs a UI colour to premultiplied RGBA8. */
                void pack_color(const UI::Color& color, std::uint8_t out[4]) noexcept
                {
                    out[0] = premultiplied_channel(color.r, color.a);
                    out[1] = premultiplied_channel(color.g, color.a);
                    out[2] = premultiplied_channel(color.b, color.a);
                    out[3] = static_cast<std::uint8_t>(
                        std::max(0.0, std::min(1.0, double(color.a))) * 255.0 + 0.5);
                }
            } // namespace

            UIBuffers::UIBuffers(Vulkan::VulkanDevice& device, std::uint32_t frame_slots)
                : device_(device)
            {
                vertices_.resize(frame_slots);
                indices_.resize(frame_slots);
            }

            UIBuffers::~UIBuffers()
            {
                for (Allocation& allocation : vertices_)
                    destroy(allocation);
                for (Allocation& allocation : indices_)
                    destroy(allocation);
            }

            void UIBuffers::grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage)
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
                alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                              VMA_ALLOCATION_CREATE_MAPPED_BIT;

                VmaAllocationInfo info{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                              &target.buffer, &target.allocation, &info),
                              "vmaCreateBuffer(ui overlay)");
                target.mapped = info.pMappedData;
                target.capacity = bytes;
            }

            void UIBuffers::destroy(Allocation& target)
            {
                if (target.buffer != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), target.buffer, target.allocation);
                target = Allocation{};
            }

            void UIBuffers::push_quad(float x0, float y0, float x1, float y1, float u0, float v0,
                                      float u1, float v1, const std::uint8_t color[4])
            {
                const std::uint32_t base = static_cast<std::uint32_t>(vertex_scratch_.size());

                UIVertex vertex;
                std::memcpy(vertex.color, color, 4);

                vertex.x = x0; vertex.y = y0; vertex.u = u0; vertex.v = v0;
                vertex_scratch_.push_back(vertex);
                vertex.x = x1; vertex.y = y0; vertex.u = u1; vertex.v = v0;
                vertex_scratch_.push_back(vertex);
                vertex.x = x1; vertex.y = y1; vertex.u = u1; vertex.v = v1;
                vertex_scratch_.push_back(vertex);
                vertex.x = x0; vertex.y = y1; vertex.u = u0; vertex.v = v1;
                vertex_scratch_.push_back(vertex);

                index_scratch_.push_back(base + 0);
                index_scratch_.push_back(base + 1);
                index_scratch_.push_back(base + 2);
                index_scratch_.push_back(base + 0);
                index_scratch_.push_back(base + 2);
                index_scratch_.push_back(base + 3);
            }

            void UIBuffers::prepare(std::uint32_t slot, const UIView& ui,
                                    const Assets::FontAtlas& font)
            {
                vertex_scratch_.clear();
                index_scratch_.clear();
                index_count_ = 0;
                if (slot >= vertices_.size() || ui.empty())
                    return;

                const float white_u = font.white_u();
                const float white_v = font.white_v();

                // Rectangles first, then text: the draw list keeps each in paint order, and a
                // label always belongs on top of the panel it labels.
                for (std::size_t i = 0; i < ui.rect_count; ++i)
                {
                    const UI::UIDrawRect& rect = ui.rects[i];
                    if (rect.color.a <= 0)
                        continue;
                    std::uint8_t color[4];
                    pack_color(rect.color, color);
                    const float x0 = static_cast<float>(rect.rect.min.x);
                    const float y0 = static_cast<float>(rect.rect.min.y);
                    push_quad(x0, y0, x0 + static_cast<float>(rect.rect.size.x),
                              y0 + static_cast<float>(rect.rect.size.y), white_u, white_v, white_u,
                              white_v, color);
                }

                if (font.valid() && font.bake_height() > 0.0f)
                {
                    for (std::size_t i = 0; i < ui.text_count; ++i)
                    {
                        const UI::UITextRun& run = ui.texts[i];
                        if (run.length == 0 || run.color.a <= 0)
                            continue;
                        std::uint8_t color[4];
                        pack_color(run.color, color);

                        const float font_size = static_cast<float>(run.font_size);
                        const float scale = font_size / font.bake_height();
                        const std::uint32_t length =
                            std::min<std::uint32_t>(run.length, UI::UI_TEXT_CAPACITY);

                        // Vertically centred in the run's rectangle, which is what a label in a
                        // laid-out box reads as; horizontally the run's own alignment decides,
                        // which is why a button's label lands in the middle of its fill.
                        const float box_height = static_cast<float>(run.rect.size.y);
                        const float box_width = static_cast<float>(run.rect.size.x);
                        float pen_x = static_cast<float>(run.rect.min.x);
                        if (run.align != UI::TextAlign::Left)
                        {
                            const float text_width = font.measure(run.text, length, font_size);
                            const float slack = box_width - text_width;
                            pen_x += run.align == UI::TextAlign::Center ? slack * 0.5f : slack;
                        }
                        const float pen_y = static_cast<float>(run.rect.min.y) +
                                            (box_height - font.line_height() * scale) * 0.5f +
                                            font.ascent() * scale;

                        for (std::uint32_t c = 0; c < length; ++c)
                        {
                            const char codepoint = run.text[c];
                            if (codepoint == '\0')
                                break;
                            const Assets::FontGlyph& glyph = font.glyph(codepoint);
                            if (glyph.width > 0.0f && glyph.height > 0.0f)
                            {
                                const float x0 = pen_x + glyph.offset_x * scale;
                                const float y0 = pen_y + glyph.offset_y * scale;
                                push_quad(x0, y0, x0 + glyph.width * scale,
                                          y0 + glyph.height * scale, glyph.u0, glyph.v0, glyph.u1,
                                          glyph.v1, color);
                            }
                            pen_x += glyph.advance * scale;
                        }
                    }
                }

                if (index_scratch_.empty())
                    return;

                const VkDeviceSize vertex_bytes = vertex_scratch_.size() * sizeof(UIVertex);
                const VkDeviceSize index_bytes = index_scratch_.size() * sizeof(std::uint32_t);
                grow(vertices_[slot], vertex_bytes, VK_BUFFER_USAGE_VERTEX_BUFFER_BIT);
                grow(indices_[slot], index_bytes, VK_BUFFER_USAGE_INDEX_BUFFER_BIT);
                if (vertices_[slot].mapped == nullptr || indices_[slot].mapped == nullptr)
                    return;

                std::memcpy(vertices_[slot].mapped, vertex_scratch_.data(),
                            static_cast<std::size_t>(vertex_bytes));
                std::memcpy(indices_[slot].mapped, index_scratch_.data(),
                            static_cast<std::size_t>(index_bytes));
                index_count_ = static_cast<std::uint32_t>(index_scratch_.size());
            }

            VkBuffer UIBuffers::vertices(std::uint32_t slot) const noexcept
            {
                return slot < vertices_.size() ? vertices_[slot].buffer : VK_NULL_HANDLE;
            }

            VkBuffer UIBuffers::indices(std::uint32_t slot) const noexcept
            {
                return slot < indices_.size() ? indices_[slot].buffer : VK_NULL_HANDLE;
            }
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
