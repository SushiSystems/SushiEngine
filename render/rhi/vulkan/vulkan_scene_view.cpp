/**************************************************************************/
/* vulkan_scene_view.cpp                                                  */
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

#include "vulkan_scene_view.hpp"

#include <cstring>

#include "frame/temporal_jitter.hpp"
#include "material/asset_library.hpp"
#include "resources/sampler_cache.hpp"
#include "scene/scene_uniforms.hpp"
#include "scene/shadow_uniforms.hpp"
#include "scene/temporal_uniforms.hpp"
#include "vulkan_check.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            namespace
            {
                /** @brief Upper bound on timed passes in one frame. */
                constexpr std::uint32_t MAX_TIMED_PASSES = 16;

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

            VulkanSceneView::VulkanSceneView(VulkanDevice& device,
                                             Assets::AssetLibrary& assets)
                : device_(device), assets_(assets), descriptors_(device, SLOTS),
                  cloth_(device, SLOTS), materials_(device, assets.textures(), SLOTS),
                  motion_(device, SLOTS), accelerator_(device, assets.meshes(), SLOTS),
                  profiler_(device, SLOTS, MAX_TIMED_PASSES),
                  graph_(device, &profiler_),
                  ibl_pass_(device, assets.shaders(), assets.pipelines(), assets.samplers(),
                            assets.layout(), assets.cloud_noise()),
                  depth_prepass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                                 assets.meshes(), motion_),
                  shadow_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes()),
                  contact_shadow_pass_(device, assets.shaders(), assets.pipelines(),
                                       assets.layout()),
                  ray_shadow_pass_(device, assets.shaders(), assets.pipelines(), accelerator_),
                  opaque_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                               assets.meshes(), cloth_, materials_, motion_,
                               assets.cloud_noise(), ibl_pass_),
                  shading_rate_pass_(device, assets.shaders(), assets.pipelines()),
                  sky_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                            assets.cloud_noise()),
                  cloud_pass_(device, assets.shaders(), assets.pipelines(), assets.layout(),
                              assets.cloud_noise()),
                  cloud_composite_pass_(device, assets.shaders(), assets.pipelines(),
                                        assets.layout()),
                  taa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  tonemap_pass_(device, assets.shaders(), assets.pipelines(), assets.layout()),
                  fxaa_pass_(device, assets.shaders(), assets.pipelines(), assets.layout())
            {
                // Registration order is execution order. The IBL chain is captured before
                // anything samples it; the depth prepass runs first among the geometry so
                // the contact-shadow march has a complete depth buffer to walk, and the
                // cascades are drawn before the surfaces that sample them; the shading
                // rate mask sits after the geometry pass
                // because it reads that pass's motion vectors and before the sky because
                // the sky is what it steers; the temporal resolve needs a complete scene,
                // so it follows the cloud composite; and the display transform is last
                // except for the spatial filter and the picking readback.
                passes_ = {&ibl_pass_,
                           &depth_prepass_,
                           &shadow_pass_,
                           &contact_shadow_pass_,
                           &ray_shadow_pass_,
                           &opaque_pass_,
                           &shading_rate_pass_,
                           &sky_pass_,
                           &cloud_pass_,
                           &cloud_composite_pass_,
                           &taa_pass_,
                           &tonemap_pass_,
                           &fxaa_pass_,
                           &picking_pass_};
                ui_sampler_ = assets.samplers().get(Resources::SamplerDesc{});
                create_slots();
                create_targets();
            }

            VulkanSceneView::~VulkanSceneView()
            {
                vkDeviceWaitIdle(device_.device());
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

            void VulkanSceneView::create_slots()
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

            void VulkanSceneView::create_targets()
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
                render_width_ = width_;
                render_height_ = height_;
            }

            void VulkanSceneView::destroy_targets()
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

            void VulkanSceneView::resize(std::uint32_t width, std::uint32_t height)
            {
                const std::uint32_t new_width = width < 1 ? 1 : width;
                const std::uint32_t new_height = height < 1 ? 1 : height;
                if (new_width == width_ && new_height == height_)
                    return;
                vkDeviceWaitIdle(device_.device());
                destroy_targets();
                width_ = new_width;
                height_ = new_height;
                create_targets();
            }

            void VulkanSceneView::set_settings(const RenderSettings& settings)
            {
                // Switching the anti-aliasing mode invalidates what has been accumulated:
                // the history was produced by a different resolve, and blending the two
                // would show as a lingering ghost of the old mode.
                if (settings.anti_aliasing != settings_.anti_aliasing)
                    history_valid_ = false;
                if (!settings.dynamic_resolution.enabled &&
                    settings_.dynamic_resolution.enabled)
                    resolution_.reset();
                settings_ = settings;
            }

            void VulkanSceneView::render_resolution(std::uint32_t& width,
                                                    std::uint32_t& height) const noexcept
            {
                width = render_width_;
                height = render_height_;
            }

            float VulkanSceneView::measured_frame_milliseconds() const noexcept
            {
                float total = 0.0f;
                for (const Graph::PassTiming& timing : profiler_.timings())
                    total += timing.milliseconds;
                return total;
            }

            void VulkanSceneView::update_render_extent()
            {
                const float governed =
                    resolution_.update(settings_.dynamic_resolution,
                                       measured_frame_milliseconds());
                // The manual scale is the ceiling the governor works under, so a host that
                // has already chosen to render small is never scaled back up past it.
                const float scale = governed * (settings_.render_scale < 1.0f
                                                    ? settings_.render_scale
                                                    : 1.0f);
                const std::uint32_t width = Frame::scaled_extent(width_, scale);
                const std::uint32_t height = Frame::scaled_extent(height_, scale);
                if (width == render_width_ && height == render_height_)
                    return;
                render_width_ = width;
                render_height_ = height;
            }

            SceneViewTexture VulkanSceneView::texture(std::uint32_t slot) const noexcept
            {
                SceneViewTexture texture;
                texture.sampler = ui_sampler_;
                texture.image_view = slots_[slot].resolve_view;
                return texture;
            }

            std::size_t VulkanSceneView::pass_timing_count() const noexcept
            {
                return profiler_.timings().size();
            }

            ScenePassTiming VulkanSceneView::pass_timing(std::size_t index) const noexcept
            {
                const std::vector<Graph::PassTiming>& timings = profiler_.timings();
                if (index >= timings.size())
                    return ScenePassTiming{};
                ScenePassTiming timing;
                timing.name = timings[index].name.c_str();
                timing.milliseconds = timings[index].milliseconds;
                return timing;
            }

            Frame::FrameTargets VulkanSceneView::declare_targets(Slot& slot,
                                                                 const Frame::FrameContext& frame)
            {
                Frame::FrameTargets targets;

                Graph::TextureDesc depth_desc =
                    color_target(frame.width, frame.height, Frame::DEPTH_FORMAT, "depth");
                depth_desc.aspect = VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;

                targets.hdr = graph_.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "hdr"));
                targets.composite = graph_.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "composite"));
                targets.scene = graph_.create_texture(
                    color_target(frame.width, frame.height, Frame::HDR_FORMAT, "scene"));
                targets.id = graph_.create_texture(
                    color_target(frame.width, frame.height, Frame::ID_FORMAT, "picking ids"));
                targets.velocity = graph_.create_texture(color_target(
                    frame.width, frame.height, Frame::VELOCITY_FORMAT, "velocity"));
                targets.depth = graph_.create_texture(depth_desc);

                // The cascades share one image as a two-by-two grid of tiles. It is
                // created even when shadows are off, at one texel a tile: the shading
                // pass reads the descriptor unconditionally, and a valid tiny image is
                // cheaper than a shader permutation that avoids it.
                Graph::TextureDesc shadow_desc =
                    color_target(frame.shadow_resolution * 2, frame.shadow_resolution * 2,
                                 Frame::SHADOW_FORMAT, "shadow atlas");
                shadow_desc.aspect = VK_IMAGE_ASPECT_DEPTH_BIT;
                targets.shadow_atlas = graph_.create_texture(shadow_desc);
                targets.contact_shadow = graph_.create_texture(color_target(
                    frame.width, frame.height, Frame::CONTACT_SHADOW_FORMAT, "contact shadows"));
                targets.ray_shadow = graph_.create_texture(color_target(
                    frame.width, frame.height, Frame::CONTACT_SHADOW_FORMAT, "ray shadows"));
                // The cloud march runs at half resolution; the graph derives the reduced
                // viewport from this extent, so no pass carries a resolution of its own.
                targets.cloud = graph_.create_texture(color_target(
                    (frame.width + 1) / 2, (frame.height + 1) / 2, Frame::HDR_FORMAT, "clouds"));

                if (shading_rate_pass_.enabled(frame))
                {
                    Graph::TextureDesc rate_desc = color_target(
                        tile_count(frame.width, shading_rate_pass_.texel_width()),
                        tile_count(frame.height, shading_rate_pass_.texel_height()),
                        Frame::SHADING_RATE_FORMAT, "shading rate");
                    targets.shading_rate = graph_.create_texture(rate_desc);
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
                targets.resolved = graph_.import_texture(resolved);

                Graph::ImportedTexture history = resolved;
                history.image = history_[1 - parity].image;
                history.view = history_[1 - parity].view;
                history.desc.name = "history";
                history.state = &history_[1 - parity].state;
                targets.history = graph_.import_texture(history);

                Graph::ImportedTexture resolve;
                resolve.image = slot.resolve;
                resolve.view = slot.resolve_view;
                resolve.desc = color_target(width_, height_, Frame::RESOLVE_FORMAT, "resolve");
                resolve.desc.usage =
                    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
                resolve.state = &slot.resolve_state;
                targets.resolve = graph_.import_texture(resolve);

                // The display transform writes straight into the image the host samples
                // unless a spatial filter still has to run over it, in which case it needs
                // its own intermediate to read from.
                targets.display =
                    frame.settings.anti_aliasing == AntiAliasingMode::Fxaa
                        ? graph_.create_texture(
                              color_target(width_, height_, Frame::RESOLVE_FORMAT, "tonemapped"))
                        : targets.resolve;

                Graph::ImportedBuffer readback;
                readback.buffer = slot.readback;
                readback.mapped = slot.readback_mapped;
                readback.desc.size = VkDeviceSize(width_) * height_ * sizeof(std::uint32_t);
                readback.desc.usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                readback.desc.host_visible = true;
                readback.desc.name = "picking readback";
                readback.state = &slot.readback_state;
                targets.readback = graph_.import_buffer(readback);

                Graph::ImportedBuffer uniforms;
                uniforms.buffer = slot.uniforms;
                uniforms.mapped = slot.uniforms_mapped;
                uniforms.desc.size = sizeof(Scene::SceneUniforms);
                uniforms.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                uniforms.desc.host_visible = true;
                uniforms.desc.name = "scene uniforms";
                uniforms.state = &slot.uniforms_state;
                targets.uniforms = graph_.import_buffer(uniforms);

                Graph::ImportedBuffer temporal;
                temporal.buffer = slot.temporal;
                temporal.mapped = slot.temporal_mapped;
                temporal.desc.size = sizeof(Scene::TemporalUniforms);
                temporal.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                temporal.desc.host_visible = true;
                temporal.desc.name = "temporal uniforms";
                temporal.state = &slot.temporal_state;
                targets.temporal = graph_.import_buffer(temporal);

                Graph::ImportedBuffer shadow;
                shadow.buffer = slot.shadow;
                shadow.mapped = slot.shadow_mapped;
                shadow.desc.size = sizeof(Scene::ShadowUniforms);
                shadow.desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                shadow.desc.host_visible = true;
                shadow.desc.name = "shadow uniforms";
                shadow.state = &slot.shadow_state;
                targets.shadow = graph_.import_buffer(shadow);

                return targets;
            }

            void VulkanSceneView::render(const CameraView& camera, const Environment& environment,
                                         const MeshInstance* instances, std::size_t count,
                                         std::uint32_t selected_id,
                                         const ClothStrandView* strands, std::size_t strand_count)
            {
                const std::uint32_t index = frame_counter_ % SLOTS;
                Slot& slot = slots_[index];

                if (slot.ever_rendered)
                    check(vkWaitForFences(device_.device(), 1, &slot.fence, VK_TRUE, UINT64_MAX),
                          "vkWaitForFences");
                // The fence has been waited on, so this slot's previous submit is complete:
                // its timestamps are readable and its transient resources are free.
                profiler_.resolve(index);
                // Texture streaming and shader reload are device-level, so the asset
                // library owns them; it reports back when the pipelines this view built
                // have been invalidated.
                if (assets_.update())
                    for (Passes::IRenderPass* pass : passes_)
                        pass->rebuild_pipelines();

                // The governor reads the timings that were just resolved, so the extent it
                // picks is a response to a completed frame rather than a guess.
                update_render_extent();

                check(vkResetFences(device_.device(), 1, &slot.fence), "vkResetFences");
                check(vkResetCommandPool(device_.device(), slot.pool, 0), "vkResetCommandPool");
                descriptors_.begin_frame(index);
                slot.textures->begin_frame();
                slot.buffers->begin_frame();

                Frame::FrameContext frame;
                frame.index = frame_counter_;
                frame.slot = index;
                frame.width = render_width_;
                frame.height = render_height_;
                frame.output_width = width_;
                frame.output_height = height_;
                frame.settings = settings_;
                frame.camera = &camera;
                frame.environment = &environment;
                Scene::camera_eye(camera.view, frame.eye);
                frame.draws.instances = instances;
                frame.draws.instance_count = count;
                frame.draws.strands = strands;
                frame.draws.strand_count = strand_count;
                frame.draws.selected_id = selected_id;
                frame.descriptors = &descriptors_;
                frame.samplers = &assets_.samplers();
                frame.layout = &assets_.layout();
                frame.history_valid = history_valid_ && frame.temporal_enabled();

                // The jitter cycle is longer when resolving into a larger grid, because
                // more distinct sub-pixel positions are needed to fill it.
                const bool upscaling = render_width_ < width_ || render_height_ < height_;
                const std::uint32_t phases =
                    Frame::JITTER_PHASE_COUNT * (upscaling ? 2u : 1u);
                if (frame.temporal_enabled())
                    Frame::frame_jitter(frame_counter_, render_width_, render_height_,
                                        settings_.temporal.jitter_scale, phases, frame.jitter);

                Scene::SceneUniforms uniforms;
                Scene::fill_scene_uniforms(camera, environment, frame.eye,
                                           static_cast<float>(frame_counter_) * 0.016f, uniforms);
                // The image-based lighting chain is this view's, so its parameters are
                // stamped in here rather than by the environment fill, which knows
                // nothing about the renderer's internals.
                uniforms.ibl_params[0] = environment.ibl_intensity;
                uniforms.ibl_params[1] = static_cast<float>(ibl_pass_.specular_mip_count());
                uniforms.ibl_params[2] = environment.image_based_lighting ? 1.0f : 0.0f;
                // The jitter enters the projection here and nowhere else, so every pass
                // that transforms by it inherits the offset and no world-space value
                // moves. It is subtracted, not added: the third column is scaled by view
                // z and the perspective divide is by -z, so a positive entry shifts the
                // result negative. The sign matters — it is what the sky's ray offset and
                // the motion vector's jitter removal are both matched against.
                uniforms.proj[8] -= frame.jitter[0];
                uniforms.proj[9] -= frame.jitter[1];
                std::memcpy(slot.uniforms_mapped, &uniforms, sizeof(uniforms));

                // The cascades are fitted before the targets are declared, because the
                // atlas the graph allocates has to be sized to the fit.
                Scene::ShadowUniforms shadow_uniforms;
                frame.cascade_count = Scene::fit_shadow_cascades(
                    camera, frame.eye, environment.sun.direction, settings_.shadows,
                    shadow_uniforms);
                frame.shadow_resolution =
                    frame.cascade_count > 0 ? (settings_.shadows.resolution < 64u
                                                   ? 64u
                                                   : settings_.shadows.resolution)
                                            : 1u;
                // Stamped after the fit rather than inside it: whether the frame traces
                // depends on the device and on this view's passes, which the fit knows
                // nothing about, and the flag the material shader reads has to describe
                // what actually ran.
                shadow_uniforms.flags[2] = ray_shadow_pass_.enabled(frame) ? 1.0f : 0.0f;
                std::memcpy(slot.shadow_mapped, &shadow_uniforms, sizeof(shadow_uniforms));

                Scene::TemporalUniforms temporal;
                Scene::fill_temporal_uniforms(settings_, previous_camera_.view,
                                              previous_camera_.projection, frame.jitter,
                                              previous_jitter_, render_width_, render_height_,
                                              width_, height_, frame.history_valid, temporal);
                std::memcpy(slot.temporal_mapped, &temporal, sizeof(temporal));

                materials_.begin_frame(index);
                motion_.begin_frame(index, frame.eye);
                graph_.begin_frame(*slot.textures, *slot.buffers);
                frame.targets = declare_targets(slot, frame);
                for (Passes::IRenderPass* pass : passes_)
                    pass->register_pass(graph_, frame);

                // The editor samples the resolve image between frames, so the frame must
                // end with it in a shader-readable layout. Declaring that as a read is all
                // it takes — the graph derives the transition like any other.
                const Graph::TextureHandle resolve = frame.targets.resolve;
                graph_.add_pass(
                    "present",
                    [resolve](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(resolve, Graph::TextureAccess::SampledFragment);
                        builder.set_side_effect();
                    },
                    Graph::RenderGraph::ExecuteFunction{});
                graph_.compile();
                // Every pass has registered, so the frame's per-object arrays are complete
                // and can be uploaded before any of them records a descriptor write.
                materials_.upload();
                motion_.upload();

                VkCommandBufferBeginInfo begin{};
                begin.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
                begin.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
                check(vkBeginCommandBuffer(slot.cmd, &begin), "vkBeginCommandBuffer");
                profiler_.begin_frame(index, slot.cmd);
                graph_.execute(slot.cmd);
                check(vkEndCommandBuffer(slot.cmd), "vkEndCommandBuffer");

                VkCommandBufferSubmitInfo cmd_submit{};
                cmd_submit.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_SUBMIT_INFO;
                cmd_submit.commandBuffer = slot.cmd;
                VkSubmitInfo2 submit{};
                submit.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO_2;
                submit.commandBufferInfoCount = 1;
                submit.pCommandBufferInfos = &cmd_submit;
                check(vkQueueSubmit2(device_.graphics_queue(), 1, &submit, slot.fence),
                      "vkQueueSubmit2");

                motion_.end_frame();
                previous_camera_ = camera;
                previous_jitter_[0] = frame.jitter[0];
                previous_jitter_[1] = frame.jitter[1];
                // Only a temporal frame leaves something behind worth blending with; when
                // the resolve did not run, the image at this parity is stale.
                history_valid_ = frame.temporal_enabled();
                slot.readback_width = render_width_;
                slot.readback_height = render_height_;
                slot.ever_rendered = true;
                current_slot_ = index;
                ++frame_counter_;
            }

            std::uint32_t VulkanSceneView::pick(std::uint32_t x, std::uint32_t y)
            {
                Slot& slot = slots_[current_slot_];
                if (!slot.ever_rendered || slot.readback_mapped == nullptr || x >= width_ ||
                    y >= height_ || slot.readback_width == 0 || slot.readback_height == 0)
                    return NO_PICK;

                // Ensure the copy that filled the readback buffer has completed before
                // reading it; a click is rare, so the wait is not a hot path.
                check(vkWaitForFences(device_.device(), 1, &slot.fence, VK_TRUE, UINT64_MAX),
                      "vkWaitForFences(pick)");

                // The id target was written at the internal render extent, which the
                // resolution governor may have put below the extent the click came in at.
                const std::uint32_t sample_x =
                    x * slot.readback_width / (width_ < 1 ? 1 : width_);
                const std::uint32_t sample_y =
                    y * slot.readback_height / (height_ < 1 ? 1 : height_);
                if (sample_x >= slot.readback_width || sample_y >= slot.readback_height)
                    return NO_PICK;

                const std::uint32_t* pixels =
                    static_cast<const std::uint32_t*>(slot.readback_mapped);
                return pixels[static_cast<std::size_t>(sample_y) * slot.readback_width +
                              sample_x];
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
