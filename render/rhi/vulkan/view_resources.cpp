/**************************************************************************/
/* view_resources.cpp                                                     */
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

#include "view_resources.hpp"

#include <cstring>

#include "vulkan_check.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /**
                 * @brief Describes one of the graph's colour targets.
                 * @param width  Target width in pixels.
                 * @param height Target height in pixels.
                 * @param format The target's format.
                 * @param name   Debug name reported by the graph.
                 * @return The description to create the transient with.
                 */
                Graph::TextureDesc color_target(std::uint32_t width, std::uint32_t height,
                                                VkFormat format, const char* name)
                {
                    Graph::TextureDesc desc;
                    desc.width = width;
                    desc.height = height;
                    desc.format = format;
                    desc.name = name;
                    return desc;
                }

                /**
                 * @brief Rounds a pixel count up to a whole number of tiles.
                 * @param extent Pixels to cover.
                 * @param tile   Pixels one tile covers.
                 * @return The tile count, at least one.
                 */
                std::uint32_t tile_count(std::uint32_t extent, std::uint32_t tile) noexcept
                {
                    return tile == 0 ? 1u : (extent + tile - 1) / tile;
                }
            } // namespace

            ViewResources::ViewResources(VulkanDevice& device, VkSampler ui_sampler,
                                         std::uint32_t width, std::uint32_t height)
                : device_(device), ui_sampler_(ui_sampler), width_(width), height_(height)
            {
                create_command();
                create_targets();
            }

            ViewResources::~ViewResources()
            {
                // The view idles the device before it is destroyed, so its resources are
                // free to release here without a further wait.
                destroy_targets();
                for (Slot& slot : slots_)
                {
                    slot.textures.reset();
                    slot.buffers.reset();
                    if (slot.fence != VK_NULL_HANDLE)
                        vkDestroyFence(device_.device(), slot.fence, nullptr);
                    if (slot.pool != VK_NULL_HANDLE)
                        vkDestroyCommandPool(device_.device(), slot.pool, nullptr);
                }
            }

            void ViewResources::create_command()
            {
                for (Slot& slot : slots_)
                {
                    slot.textures.reset(new Resources::TexturePool(device_));
                    slot.buffers.reset(new Resources::BufferPool(device_));

                    VkCommandPoolCreateInfo pool_info{};
                    pool_info.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
                    pool_info.flags = VK_COMMAND_POOL_CREATE_TRANSIENT_BIT;
                    pool_info.queueFamilyIndex = device_.graphics_queue_family();
                    check(vkCreateCommandPool(device_.device(), &pool_info, nullptr, &slot.pool),
                          "vkCreateCommandPool");

                    VkCommandBufferAllocateInfo cmd_info{};
                    cmd_info.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
                    cmd_info.commandPool = slot.pool;
                    cmd_info.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
                    cmd_info.commandBufferCount = 1;
                    check(vkAllocateCommandBuffers(device_.device(), &cmd_info, &slot.cmd),
                          "vkAllocateCommandBuffers");

                    VkFenceCreateInfo fence_info{};
                    fence_info.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
                    check(vkCreateFence(device_.device(), &fence_info, nullptr, &slot.fence),
                          "vkCreateFence");
                }
            }

            void ViewResources::create_targets()
            {
                for (Slot& slot : slots_)
                {
                    // The resolve image and the picking readback outlive the frame — the
                    // editor samples one and pick() reads the other — so they are owned
                    // here and imported into the graph rather than allocated by it.
                    VkImageCreateInfo image_info{};
                    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    image_info.imageType = VK_IMAGE_TYPE_2D;
                    image_info.format = Frame::RESOLVE_FORMAT;
                    image_info.extent = {width_, height_, 1};
                    image_info.mipLevels = 1;
                    image_info.arrayLayers = 1;
                    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                    image_info.usage =
                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                    VmaAllocationCreateInfo image_alloc{};
                    image_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    check(vmaCreateImage(device_.allocator(), &image_info, &image_alloc,
                                         &slot.resolve, &slot.resolve_allocation, nullptr),
                          "vmaCreateImage(resolve)");

                    VkImageViewCreateInfo view_info{};
                    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                    view_info.image = slot.resolve;
                    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                    view_info.format = Frame::RESOLVE_FORMAT;
                    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    view_info.subresourceRange.levelCount = 1;
                    view_info.subresourceRange.layerCount = 1;
                    check(vkCreateImageView(device_.device(), &view_info, nullptr,
                                            &slot.resolve_view),
                          "vkCreateImageView(resolve)");
                    slot.resolve_state = Graph::TextureState{};

                    // Sized for the full output extent even though the id target may be
                    // rendered smaller: the render scale moves every frame, and a buffer
                    // that had to be reallocated with it would stall the frame it changed.
                    VkBufferCreateInfo readback_info{};
                    readback_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    readback_info.size =
                        VkDeviceSize(width_) * height_ * sizeof(std::uint32_t);
                    readback_info.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;

                    VmaAllocationCreateInfo readback_alloc{};
                    readback_alloc.usage = VMA_MEMORY_USAGE_AUTO_PREFER_HOST;
                    readback_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT |
                                           VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo readback_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &readback_info, &readback_alloc,
                                          &slot.readback, &slot.readback_allocation,
                                          &readback_mapped),
                          "vmaCreateBuffer(readback)");
                    slot.readback_mapped = readback_mapped.pMappedData;
                    slot.readback_state = Graph::BufferState{};
                    slot.readback_width = 0;
                    slot.readback_height = 0;

                    VkBufferCreateInfo uniform_info{};
                    uniform_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    uniform_info.size = sizeof(Scene::SceneUniforms);
                    uniform_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

                    VmaAllocationCreateInfo uniform_alloc{};
                    uniform_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    uniform_alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                          VMA_ALLOCATION_CREATE_MAPPED_BIT;
                    VmaAllocationInfo uniform_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &uniform_info, &uniform_alloc,
                                          &slot.uniforms, &slot.uniforms_allocation,
                                          &uniform_mapped),
                          "vmaCreateBuffer(scene uniforms)");
                    slot.uniforms_mapped = uniform_mapped.pMappedData;
                    slot.uniforms_state = Graph::BufferState{};

                    VkBufferCreateInfo temporal_info = uniform_info;
                    temporal_info.size = sizeof(Scene::TemporalUniforms);
                    VmaAllocationInfo temporal_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &temporal_info, &uniform_alloc,
                                          &slot.temporal, &slot.temporal_allocation,
                                          &temporal_mapped),
                          "vmaCreateBuffer(temporal uniforms)");
                    slot.temporal_mapped = temporal_mapped.pMappedData;
                    slot.temporal_state = Graph::BufferState{};

                    VkBufferCreateInfo shadow_info = uniform_info;
                    shadow_info.size = sizeof(Scene::ShadowUniforms);
                    VmaAllocationInfo shadow_mapped{};
                    check(vmaCreateBuffer(device_.allocator(), &shadow_info, &uniform_alloc,
                                          &slot.shadow, &slot.shadow_allocation, &shadow_mapped),
                          "vmaCreateBuffer(shadow uniforms)");
                    slot.shadow_mapped = shadow_mapped.pMappedData;
                    slot.shadow_state = Graph::BufferState{};

                    slot.ever_rendered = false;
                }

                // The accumulated history lives at the output extent, which is what makes
                // rendering below it and resolving into it an upscale rather than a blur.
                for (History& history : history_)
                {
                    VkImageCreateInfo image_info{};
                    image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                    image_info.imageType = VK_IMAGE_TYPE_2D;
                    image_info.format = Frame::HDR_FORMAT;
                    image_info.extent = {width_, height_, 1};
                    image_info.mipLevels = 1;
                    image_info.arrayLayers = 1;
                    image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                    image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                    image_info.usage =
                        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                    VmaAllocationCreateInfo image_alloc{};
                    image_alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    check(vmaCreateImage(device_.allocator(), &image_info, &image_alloc,
                                         &history.image, &history.allocation, nullptr),
                          "vmaCreateImage(history)");

                    VkImageViewCreateInfo view_info{};
                    view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                    view_info.image = history.image;
                    view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                    view_info.format = Frame::HDR_FORMAT;
                    view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    view_info.subresourceRange.levelCount = 1;
                    view_info.subresourceRange.layerCount = 1;
                    check(vkCreateImageView(device_.device(), &view_info, nullptr, &history.view),
                          "vkCreateImageView(history)");
                    history.state = Graph::TextureState{};
                }

                // Nothing accumulated into the new images yet.
                history_valid_ = false;
            }

            void ViewResources::destroy_targets()
            {
                for (History& history : history_)
                {
                    if (history.view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), history.view, nullptr);
                    if (history.image != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), history.image, history.allocation);
                    history.view = VK_NULL_HANDLE;
                    history.image = VK_NULL_HANDLE;
                }
                history_valid_ = false;

                for (Slot& slot : slots_)
                {
                    if (slot.shadow != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.shadow,
                                         slot.shadow_allocation);
                    if (slot.temporal != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.temporal,
                                         slot.temporal_allocation);
                    if (slot.uniforms != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.uniforms,
                                         slot.uniforms_allocation);
                    if (slot.readback != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), slot.readback,
                                         slot.readback_allocation);
                    if (slot.resolve_view != VK_NULL_HANDLE)
                        vkDestroyImageView(device_.device(), slot.resolve_view, nullptr);
                    if (slot.resolve != VK_NULL_HANDLE)
                        vmaDestroyImage(device_.allocator(), slot.resolve,
                                        slot.resolve_allocation);
                    slot.shadow = VK_NULL_HANDLE;
                    slot.shadow_mapped = nullptr;
                    slot.temporal = VK_NULL_HANDLE;
                    slot.temporal_mapped = nullptr;
                    slot.uniforms = VK_NULL_HANDLE;
                    slot.uniforms_mapped = nullptr;
                    slot.readback = VK_NULL_HANDLE;
                    slot.readback_mapped = nullptr;
                    slot.resolve_view = VK_NULL_HANDLE;
                    slot.resolve = VK_NULL_HANDLE;
                    if (slot.textures)
                        slot.textures->clear();
                    if (slot.buffers)
                        slot.buffers->clear();
                }
            }

            void ViewResources::resize(std::uint32_t width, std::uint32_t height)
            {
                vkDeviceWaitIdle(device_.device());
                destroy_targets();
                width_ = width;
                height_ = height;
                create_targets();
            }

            VkCommandPool ViewResources::command_pool(std::uint32_t slot) const noexcept
            {
                return slots_[slot].pool;
            }

            VkCommandBuffer ViewResources::command_buffer(std::uint32_t slot) const noexcept
            {
                return slots_[slot].cmd;
            }

            VkFence ViewResources::fence(std::uint32_t slot) const noexcept
            {
                return slots_[slot].fence;
            }

            bool ViewResources::ever_rendered(std::uint32_t slot) const noexcept
            {
                return slots_[slot].ever_rendered;
            }

            void ViewResources::mark_rendered(std::uint32_t slot) noexcept
            {
                slots_[slot].ever_rendered = true;
            }

            void ViewResources::begin_slot(std::uint32_t slot)
            {
                slots_[slot].textures->begin_frame();
                slots_[slot].buffers->begin_frame();
            }

            Resources::TexturePool& ViewResources::textures(std::uint32_t slot)
            {
                return *slots_[slot].textures;
            }

            Resources::BufferPool& ViewResources::buffers(std::uint32_t slot)
            {
                return *slots_[slot].buffers;
            }

            void ViewResources::upload_scene(std::uint32_t slot,
                                             const Scene::SceneUniforms& uniforms)
            {
                std::memcpy(slots_[slot].uniforms_mapped, &uniforms, sizeof(uniforms));
            }

            void ViewResources::upload_temporal(std::uint32_t slot,
                                                const Scene::TemporalUniforms& uniforms)
            {
                std::memcpy(slots_[slot].temporal_mapped, &uniforms, sizeof(uniforms));
            }

            void ViewResources::upload_shadow(std::uint32_t slot,
                                              const Scene::ShadowUniforms& uniforms)
            {
                std::memcpy(slots_[slot].shadow_mapped, &uniforms, sizeof(uniforms));
            }

            void ViewResources::note_render_extent(std::uint32_t slot, std::uint32_t render_width,
                                                   std::uint32_t render_height) noexcept
            {
                slots_[slot].readback_width = render_width;
                slots_[slot].readback_height = render_height;
            }

            Frame::FrameTargets ViewResources::declare_targets(Graph::RenderGraph& graph,
                                                               const Frame::FrameContext& frame,
                                                               bool shading_rate_enabled,
                                                               std::uint32_t rate_texel_width,
                                                               std::uint32_t rate_texel_height)
            {
                Frame::FrameTargets targets;

                Graph::TextureDesc depth_desc =
                    color_target(frame.width, frame.height, Frame::DEPTH_FORMAT, "depth");
                depth_desc.aspect = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;

                targets.hdr = graph.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "hdr"));
                targets.composite = graph.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "composite"));
                targets.scene = graph.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "scene"));
                targets.id = graph.create_texture(
                    color_target(frame.width, frame.height, Frame::ID_FORMAT, "picking ids"));
                targets.velocity = graph.create_texture(color_target(
                    frame.width, frame.height, Frame::VELOCITY_FORMAT, "velocity"));
                targets.depth = graph.create_texture(depth_desc);

                // The cascades share one image as a two-by-two grid of tiles. It is
                // created even when shadows are off, at one texel a tile: the shading
                // pass reads the descriptor unconditionally, and a valid tiny image is
                // cheaper than a shader permutation that avoids it.
                Graph::TextureDesc shadow_desc =
                    color_target(frame.shadow_resolution * 2, frame.shadow_resolution * 2,
                                 Frame::SHADOW_FORMAT, "shadow atlas");
                shadow_desc.aspect = VK_IMAGE_ASPECT_DEPTH_BIT;
                targets.shadow_atlas = graph.create_texture(shadow_desc);
                targets.contact_shadow = graph.create_texture(color_target(
                    frame.width, frame.height, Frame::CONTACT_SHADOW_FORMAT, "contact shadows"));
                targets.ray_shadow = graph.create_texture(color_target(
                    frame.width, frame.height, Frame::CONTACT_SHADOW_FORMAT, "ray shadows"));
                // The cloud march runs at half resolution; the graph derives the reduced
                // viewport from this extent, so no pass carries a resolution of its own.
                targets.cloud = graph.create_texture(color_target(
                    (frame.width + 1) / 2, (frame.height + 1) / 2, Frame::HDR_FORMAT, "clouds"));

                if (shading_rate_enabled)
                {
                    Graph::TextureDesc rate_desc = color_target(
                        tile_count(frame.width, rate_texel_width),
                        tile_count(frame.height, rate_texel_height),
                        Frame::SHADING_RATE_FORMAT, "shading rate");
                    targets.shading_rate = graph.create_texture(rate_desc);
                }

                // The two accumulated frames, ping-ponged by parity: this frame resolves
                // into one and reads the other, and they swap roles next frame.
                const std::uint32_t parity = frame.index % 2;
                Graph::ImportedTexture resolved;
                resolved.image = history_[parity].image;
                resolved.view = history_[parity].view;
                resolved.desc = color_target(width_, height_, Frame::HDR_FORMAT, "resolved");
                resolved.desc.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                resolved.state = &history_[parity].state;
                targets.resolved = graph.import_texture(resolved);

                Graph::ImportedTexture history = resolved;
                history.image = history_[1 - parity].image;
                history.view = history_[1 - parity].view;
                history.desc.name = "history";
                history.state = &history_[1 - parity].state;
                targets.history = graph.import_texture(history);

                Graph::ImportedTexture resolve;
                resolve.image = slots_[frame.slot].resolve;
                resolve.view = slots_[frame.slot].resolve_view;
                resolve.desc = color_target(width_, height_, Frame::RESOLVE_FORMAT, "resolve");
                resolve.desc.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                resolve.state = &slots_[frame.slot].resolve_state;
                targets.resolve = graph.import_texture(resolve);

                // The display transform writes straight into the image the host samples
                // unless a spatial filter still has to run over it, in which case it needs
                // its own intermediate to read from.
                targets.display =
                    frame.settings.anti_aliasing == AntiAliasingMode::Fxaa
                        ? graph.create_texture(
                              color_target(width_, height_, Frame::RESOLVE_FORMAT, "tonemapped"))
                        : targets.resolve;

                Graph::ImportedBuffer readback;
                readback.buffer = slots_[frame.slot].readback;
                readback.mapped = slots_[frame.slot].readback_mapped;
                readback.desc.size = VkDeviceSize(width_) * height_ * sizeof(std::uint32_t);
                readback.desc.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                readback.desc.host_visible = true;
                readback.desc.name = "picking readback";
                readback.state = &slots_[frame.slot].readback_state;
                targets.readback = graph.import_buffer(readback);

                Graph::ImportedBuffer uniforms;
                uniforms.buffer = slots_[frame.slot].uniforms;
                uniforms.mapped = slots_[frame.slot].uniforms_mapped;
                uniforms.desc.size = sizeof(Scene::SceneUniforms);
                uniforms.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                uniforms.desc.host_visible = true;
                uniforms.desc.name = "scene uniforms";
                uniforms.state = &slots_[frame.slot].uniforms_state;
                targets.uniforms = graph.import_buffer(uniforms);

                Graph::ImportedBuffer temporal;
                temporal.buffer = slots_[frame.slot].temporal;
                temporal.mapped = slots_[frame.slot].temporal_mapped;
                temporal.desc.size = sizeof(Scene::TemporalUniforms);
                temporal.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                temporal.desc.host_visible = true;
                temporal.desc.name = "temporal uniforms";
                temporal.state = &slots_[frame.slot].temporal_state;
                targets.temporal = graph.import_buffer(temporal);

                Graph::ImportedBuffer shadow;
                shadow.buffer = slots_[frame.slot].shadow;
                shadow.mapped = slots_[frame.slot].shadow_mapped;
                shadow.desc.size = sizeof(Scene::ShadowUniforms);
                shadow.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                shadow.desc.host_visible = true;
                shadow.desc.name = "shadow uniforms";
                shadow.state = &slots_[frame.slot].shadow_state;
                targets.shadow = graph.import_buffer(shadow);

                return targets;
            }

            SceneViewTexture ViewResources::texture(std::uint32_t slot) const noexcept
            {
                SceneViewTexture texture;
                texture.sampler = ui_sampler_;
                texture.image_view = slots_[slot].resolve_view;
                return texture;
            }

            std::uint32_t ViewResources::pick(std::uint32_t slot, std::uint32_t x, std::uint32_t y,
                                              std::uint32_t output_width,
                                              std::uint32_t output_height) const
            {
                const Slot& s = slots_[slot];
                if (!s.ever_rendered || s.readback_mapped == nullptr || x >= output_width ||
                    y >= output_height || s.readback_width == 0 || s.readback_height == 0)
                    return NO_PICK;

                // Ensure the copy that filled the readback buffer has completed before
                // reading it; a click is rare, so the wait is not a hot path.
                check(vkWaitForFences(device_.device(), 1, &s.fence, VK_TRUE, UINT64_MAX),
                      "vkWaitForFences(pick)");

                // The id target was written at the internal render extent, which the
                // resolution governor may have put below the extent the click came in at.
                const std::uint32_t sample_x =
                    x * s.readback_width / (output_width < 1 ? 1 : output_width);
                const std::uint32_t sample_y =
                    y * s.readback_height / (output_height < 1 ? 1 : output_height);
                if (sample_x >= s.readback_width || sample_y >= s.readback_height)
                    return NO_PICK;

                const std::uint32_t* pixels =
                    static_cast<const std::uint32_t*>(s.readback_mapped);
                return pixels[static_cast<std::size_t>(sample_y) * s.readback_width + sample_x];
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
